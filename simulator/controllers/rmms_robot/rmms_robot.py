"""rmms_robot — "bộ não" dùng chung cho mọi robot RMMS trong Webots.

Robot làm đúng 3 việc:
  1. NGHE   kênh rmms/v1/{robotId}/order   → nhận lệnh đi (dãy điểm)
  2. CHẠY   lần lượt qua từng điểm, tới điểm dừng thì dừng chờ, xong thì về trạm sạc
  3. BÁO    kênh rmms/v1/{robotId}/state   mỗi giây (vị trí, pin, đang làm gì)
            kênh rmms/v1/{robotId}/connection   ONLINE khi bật; mất điện thì Mosquitto tự báo OFFLINE

Lệnh nhận được 2 kiểu:
  - có "path"  (mỗi điểm có nodeId + x + y, điểm dừng có stopSeq) → đi đúng theo path   ← cách đã chốt (backend tính đường)
  - chỉ có "stops" (như code backend hiện tại) → robot TẠM tự tìm đường trong map/<tầng>.json

Robot tự biết mình là ai qua robot.getName() (MED-01, SUP-01...), nên mọi robot dùng chung file này.
"""
import heapq
import json
import math
import os
import queue
import sys
from datetime import datetime, timezone

from controller import Robot

try:
    import paho.mqtt.client as mqtt
except ImportError:
    mqtt = None

if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")

ROOT = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", ".."))


class _Tee:
    """RMMS_LOG=1 → in ra console Webots VÀ ghi thêm vào simulator/logs/<robot>.log (để xem lại sau)."""
    def __init__(self, name):
        os.makedirs(os.path.join(ROOT, "logs"), exist_ok=True)
        self.f = open(os.path.join(ROOT, "logs", f"{name}.log"), "a", encoding="utf-8", buffering=1)
        self.out = sys.stdout

    def write(self, s):
        self.out.write(s)
        self.f.write(s)

    def flush(self):
        self.out.flush()
MQTT_HOST = os.environ.get("MQTT_HOST", "127.0.0.1")
MQTT_PORT = int(os.environ.get("MQTT_PORT", "1883"))

WHEEL_R, TRACK, MAX_WHEEL = 0.08, 0.42, 20.0
STOP_TOL, PASS_TOL = 0.25, 0.45       # sai số khi tới điểm dừng / điểm đi ngang (m)
DWELL_S = 5.0                         # lệnh KHÔNG có compartments (vd gửi tay) → đứng cứng 5 s như cũ
SLOT_FULL_M = 0.10                    # cảm biến ngăn đo < 0.10 m = có hộp hàng; ngăn trống đo 0.30 m
CLOSE_S = 1.5                         # cảm biến xác nhận xong → chờ 1.5 s (đóng ngăn) rồi mới đi
TIMEOUT_BY_ITEM = {"TH": 180, "VT": 300, "MAU": 120}   # hết giờ chờ (giây) theo loại hàng — Prototype v8.2
GRACE_S = 120                         # hết giờ → báo Operator; im lặng thêm 2 phút thì robot tự đi tiếp
BATTERY_PER_M = 0.15                  # % pin mỗi mét
BATTERY_LOW, BATTERY_CRITICAL = 15, 10   # % — khớp cấu hình backend (Battery.WarningPercent / CriticalPercent)
RESCUE_BATTERY = 60                   # người tới thay/sạc pin tại chỗ (lệnh RECOVER) → pin lên mức này
CHARGE_PER_S = 1.0                    # % pin mỗi giây khi sạc
SENSORS = ("ds_front", "ds_front_left", "ds_front_right", "ds_diag_left", "ds_diag_right", "ds_left", "ds_right")
LED = {"IDLE": 1, "MOVING": 2, "AT_STOP": 2, "RETURNING": 2, "CHARGING": 3, "ERROR": 4}


def arg(name, default=""):
    for a in sys.argv[1:]:
        if a.startswith(f"--{name}="):
            return a.split("=", 1)[1]
    return default


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def now_iso():
    return datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")


# ============================================================ bản đồ (chỉ dùng khi lệnh không có "path")
class FloorMap:
    def __init__(self, floor):
        data = json.load(open(os.path.join(ROOT, "map", f"{floor}.json"), encoding="utf-8"))
        self.xy = {n["id"]: (n["x"], n["y"]) for n in data["nodes"]}
        self.adj = {n: [] for n in self.xy}
        for a, b in data["edges"]:
            d = math.dist(self.xy[a], self.xy[b])
            self.adj[a].append((b, d))
            self.adj[b].append((a, d))

    def nearest(self, x, y):
        return min(self.xy, key=lambda n: math.dist((x, y), self.xy[n]))

    def route(self, start, goal):
        best, prev, heap = {start: 0.0}, {}, [(0.0, start)]
        while heap:
            c, n = heapq.heappop(heap)
            if n == goal:
                break
            for m, d in self.adj[n]:
                if c + d < best.get(m, math.inf):
                    best[m], prev[m] = c + d, n
                    heapq.heappush(heap, (c + d, m))
        if goal not in best:
            raise ValueError(f"không có đường {start} → {goal}")
        path = [goal]
        while path[-1] != start:
            path.append(prev[path[-1]])
        return path[::-1]


# ============================================================ MQTT
class Radio:
    """Nối Mosquitto. Không có broker thì robot vẫn chạy, cứ vài giây thử lại."""

    def __init__(self, rid):
        self.rid = rid
        self.inbox = queue.Queue()
        self.client = None
        if mqtt is None:
            print(f"[{rid}] THIẾU paho-mqtt → chạy: pip install paho-mqtt", flush=True)
            return
        c = (mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id=f"webots-{rid}")
             if hasattr(mqtt, "CallbackAPIVersion") else mqtt.Client(client_id=f"webots-{rid}"))
        c.will_set(f"rmms/v1/{rid}/connection", json.dumps({"robotId": rid, "connection": "OFFLINE"}), qos=1, retain=True)
        c.on_connect = self._on_connect
        c.on_message = lambda cl, u, msg: self.inbox.put(msg.payload)
        c.reconnect_delay_set(1, 5)
        c.connect_async(MQTT_HOST, MQTT_PORT, keepalive=10)
        c.loop_start()
        self.client = c
        self.dark_until = None             # giả lập MẤT KẾT NỐI: tới mốc giờ này mới nối lại

    def go_dark(self, seconds, now):
        """Giả lập mất mạng: ngừng vòng mạng → không gửi state, không ping broker.
        Quá 15 s (1.5 × keepalive) broker tự phát last will OFFLINE — y như robot thật rớt Wi-Fi."""
        if self.client is None:
            return
        self.dark_until = now + seconds
        self.client.loop_stop()

    def check_dark(self, now):
        """Hết thời gian mất mạng → bật lại, paho tự nối lại và _on_connect báo ONLINE + nghe lệnh lại."""
        if self.dark_until is not None and now >= self.dark_until:
            self.dark_until = None
            self.client.loop_start()
            if not self.client.is_connected():
                self.client.reconnect_delay_set(1, 5)
            return True
        return False

    def _on_connect(self, client, userdata, flags, rc, props=None):
        client.subscribe(f"rmms/v1/{self.rid}/order", qos=1)
        client.subscribe(f"rmms/v1/{self.rid}/command", qos=1)   # STOP/RESUME khẩn của backend (Hải, 25/11)
        client.publish(f"rmms/v1/{self.rid}/connection",
                       json.dumps({"robotId": self.rid, "connection": "ONLINE", "ts": now_iso()}), qos=1, retain=True)
        print(f"[{self.rid}] MQTT đã kết nối {MQTT_HOST}:{MQTT_PORT}, đang nghe rmms/v1/{self.rid}/order", flush=True)

    @property
    def online(self):
        return self.client is not None and self.dark_until is None and self.client.is_connected()

    def send_state(self, payload):
        if self.online:
            self.client.publish(f"rmms/v1/{self.rid}/state", json.dumps(payload), qos=0)

    def send_event(self, payload):
        """Sự kiện một lần (cảm biến ngăn đổi, hết giờ chờ...) — QoS 1 để backend không bỏ sót."""
        if self.online:
            self.client.publish(f"rmms/v1/{self.rid}/event",
                                json.dumps({"robotId": self.rid, "ts": now_iso(), **payload}), qos=1)


# ============================================================ chạy theo điểm + né vật cản
class Driver:
    """Đi lần lượt qua các điểm. Gặp vật cản thì chậm lại, dừng, xoay, vòng qua."""

    def __init__(self, robot, step):
        self.left, self.right = robot.getDevice("left_wheel_motor"), robot.getDevice("right_wheel_motor")
        for m in (self.left, self.right):
            m.setPosition(float("inf"))
            m.setVelocity(0.0)
        self.ds = {n: robot.getDevice(n) for n in SENSORS}
        for s in self.ds.values():
            s.enable(step)
        self.dt = step / 1000.0
        self.v = self.w = 0.0
        self.rotating = False
        self.avoid, self.turn, self.avoid_t, self.bypass = "CLEAR", -1, 0.0, 0.0

    def ranges(self):
        return {n: (2.0 if math.isnan(s.getValue()) else s.getValue()) for n, s in self.ds.items()}

    def stop(self):
        self.v = self.w = 0.0
        self.left.setVelocity(0.0)
        self.right.setVelocity(0.0)

    def toward(self, x, y, heading, tx, ty, max_v, final, moved):
        """Một bước điều khiển tới (tx, ty). Trả về khoảng cách còn lại."""
        d = math.hypot(tx - x, ty - y)
        err = wrap(math.atan2(ty - y, tx - x) - heading)
        r = self.ranges()
        front = min(r["ds_front"], r["ds_front_left"], r["ds_front_right"])
        diag = min(r["ds_diag_left"], r["ds_diag_right"])
        self.avoid_t += self.dt
        if self.avoid == "TURN":                      # đang xoay tìm hướng trống
            v, w = 0.0, self.turn * 1.0
            if front > 0.8 and diag > 0.4:
                self.avoid, self.bypass = "BYPASS", 0.9
            elif self.avoid_t > 7:
                self.turn, self.avoid_t = -self.turn, 0.0
        elif self.avoid == "BYPASS":                  # đi vòng qua vật cản một đoạn
            v, w = 0.3, 0.0
            self.bypass -= moved
            if front < 0.32 or diag < 0.24:
                self.avoid, self.avoid_t = "TURN", 0.0
            elif self.bypass <= 0:
                self.avoid = "CLEAR"
        else:
            if self.rotating and abs(err) < 0.15:
                self.rotating = False
            elif not self.rotating and abs(err) > 0.55:
                self.rotating = True
            if self.rotating:                         # quay tại chỗ cho đúng hướng
                v, w = 0.0, clamp(2.0 * err, -1.6, 1.6)
                if abs(w) < 0.35:
                    w = math.copysign(0.35, err)
            else:
                v = max_v * max(0.0, math.cos(err))
                v = min(v, max(0.08, 0.8 * d)) if final else min(v, max(0.4, 1.2 * d))
                w = clamp(2.0 * err, -1.6, 1.6)
                if front < 0.32 or diag < 0.24:       # sắp đụng: dừng, chọn bên trống hơn để xoay
                    left = r["ds_diag_left"] + r["ds_left"] + r["ds_front_left"]
                    right = r["ds_diag_right"] + r["ds_right"] + r["ds_front_right"]
                    self.turn = 1 if left > right + 0.3 else -1
                    self.avoid, self.avoid_t, v, w = "TURN", 0.0, 0.0, 0.0
                elif front < 0.9:                     # có vật phía trước: chạy chậm lại
                    v *= clamp((front - 0.32) / 0.58, 0.2, 1.0)
        self._apply(v, w)
        return d

    def _apply(self, v, w):
        dv, dw = 1.0 * self.dt, 5.0 * self.dt
        self.v = v if abs(v) < abs(self.v) else clamp(v, self.v - dv, self.v + dv)
        self.w = clamp(w, self.w - dw, self.w + dw)
        left = (self.v - self.w * TRACK / 2) / WHEEL_R
        right = (self.v + self.w * TRACK / 2) / WHEEL_R
        k = max(1.0, abs(left) / MAX_WHEEL, abs(right) / MAX_WHEEL)
        self.left.setVelocity(left / k)
        self.right.setVelocity(right / k)


# ============================================================ robot
class RmmsRobot:
    def __init__(self):
        self.robot = Robot()
        self.step_ms = int(self.robot.getBasicTimeStep())
        self.dt = self.step_ms / 1000.0
        self.rid = self.robot.getName()
        if os.environ.get("RMMS_LOG"):
            sys.stdout = _Tee(self.rid)
        self.floor, self.home_node = arg("floor", "F2"), arg("home", "")
        self.speed = float(arg("speed", "1.0"))
        self.gps, self.compass = self.robot.getDevice("gps"), self.robot.getDevice("compass")
        self.gps.enable(self.step_ms)
        self.compass.enable(self.step_ms)
        self.led = self.robot.getDevice("status_led")
        self.drive = Driver(self.robot, self.step_ms)
        self.map = FloorMap(self.floor)
        self.radio = Radio(self.rid)

        self.x = self.y = self.th = 0.0
        self.prev = None
        self.speed_now = 0.0
        self.battery = 100.0
        self.status = "IDLE"
        self.order_id = None
        self.path, self.i = [], 0          # path = [(nodeId, x, y, stopSeq|None)]
        self.dwell_left = 0.0
        self.at_stop = None
        self.home_xy = None
        self.next_state = 0.0

        # cảm biến ngăn hàng: thiết bị tên slot_1, slot_2... (build_world.py gắn trên khay)
        self.slots = {}
        for k in range(self.robot.getNumberOfDevices()):
            dev = self.robot.getDeviceByIndex(k)
            if dev.getName().startswith("slot_"):
                dev.enable(self.step_ms)
                self.slots[int(dev.getName()[5:])] = dev
        self.slot_full = {i: False for i in self.slots}
        self.slot_dist = {i: 0.3 for i in self.slots}
        self.slot_votes = {i: 0 for i in self.slots}
        self.task_slot, self.task_item, self.stop_info = {}, {}, {}
        self.wait = None                   # đang chờ cảm biến ở điểm dừng (xem start_wait)
        self.error = None                  # sự cố đang có, vd "BATTERY_CRITICAL" (status = ERROR)
        self.before_error = None           # trạng thái trước sự cố, để RECOVER chạy tiếp
        self.low_warned = False            # đã báo pin yếu chưa (báo 1 lần)
        self.done_cmds = set()             # commandId đã làm (backend có thể gửi trùng)

    # ---- cảm biến ngăn hàng ------------------------------------------------
    def read_slots(self):
        """Đọc cảm biến từng ngăn. Ngăn nào đổi trạng thái thì in ra + báo backend."""
        for i, dev in self.slots.items():
            v = dev.getValue()
            v = 0.3 if math.isnan(v) else v
            raw = v < SLOT_FULL_M
            self.slot_votes[i] = self.slot_votes[i] + 1 if raw != self.slot_full[i] else 0
            full = raw if self.slot_votes[i] >= 3 else self.slot_full[i]   # chống nhiễu: 3 lần liền mới tin
            if full != self.slot_full[i]:
                self.slot_votes[i] = 0
                print(f"[{self.rid}] CẢM BIẾN ngăn {i}: {'TRỐNG → CÓ HÀNG' if full else 'CÓ HÀNG → TRỐNG'}"
                      f"  (khoảng cách đo được {self.slot_dist[i]:.2f} m → {v:.2f} m)", flush=True)
                self.radio.send_event({"type": "SLOT", "slot": i, "occupied": full, "distanceM": round(v, 3),
                                       "taskId": self._slot_task(i)})
                self.slot_dist[i] = v                  # số đo lúc trạng thái ổn định gần nhất
            self.slot_full[i] = full

    def _slot_task(self, slot):
        return next((t for t, s in self.task_slot.items() if s == slot), None)

    def start_wait(self, seq):
        """Tới điểm dừng: nếu biết việc nào nằm ngăn nào thì CHỜ CẢM BIẾN, không thì đứng cứng 5 s."""
        stop = self.stop_info.get(seq) or {}
        tasks = [t for t in stop.get("taskIds", []) if self.task_slot.get(t) in self.slots]
        if not tasks:
            self.wait, self.dwell_left = None, DWELL_S
            return f"chờ {DWELL_S:.0f} s (lệnh không có compartments)"
        load = str(stop.get("action", "")).upper().startswith("PICK")
        timeout = stop.get("timeoutSec") or min(
            TIMEOUT_BY_ITEM.get(str(self.task_item.get(t) or "").split("-")[0], 180) for t in tasks)
        self.wait = {"seq": seq, "load": load, "tasks": tasks, "slots": [self.task_slot[t] for t in tasks],
                     "phase": "WAITING", "left": float(timeout), "confirmedBy": None,
                     "grace": float(stop.get("graceSec") or GRACE_S)}
        return (f"chờ y tá {'ĐẶT hàng vào' if load else 'LẤY hàng ra khỏi'} ngăn "
                f"{', '.join(map(str, self.wait['slots']))} (hết giờ sau {timeout:.0f} s)")

    def update_wait(self):
        """Gọi mỗi bước khi đang AT_STOP có chờ cảm biến. Trả True khi được phép đi tiếp."""
        w = self.wait
        missing = [t for t in w["tasks"] if self.slot_full[self.task_slot[t]] != w["load"]]
        if w["phase"] == "CLOSING":
            w["left"] -= self.dt
            return w["left"] <= 0
        if not missing or w["confirmedBy"]:
            by = w["confirmedBy"] or "SENSOR"
            print(f"[{self.rid}] {'CẢM BIẾN xác nhận' if by == 'SENSOR' else 'OPERATOR bấm xác nhận'}: "
                  f"đã {'lấy đủ hàng' if w['load'] else 'giao xong, ngăn trống'} → đóng ngăn, đi tiếp", flush=True)
            self.radio.send_event({"type": "STOP_DONE", "stopSeq": w["seq"], "by": by, "taskIds": w["tasks"],
                                   "action": "PICKUP" if w["load"] else "DELIVER"})
            w["phase"], w["left"] = "CLOSING", CLOSE_S
            return False
        w["left"] -= self.dt
        if w["left"] > 0:
            return False
        if w["phase"] == "WAITING":                    # hết giờ lần 1 → báo Operator, chờ thêm 2 phút
            kind = "PICKUP_SHORT" if w["load"] else "NOT_RECEIVED"
            print(f"[{self.rid}] HẾT GIỜ CHỜ ở điểm {w['seq']}: {kind} {missing} → báo Operator "
                  f"(xác nhận / chờ thêm), im lặng {w['grace']:.0f} s thì tự đi", flush=True)
            self.radio.send_event({"type": kind, "stopSeq": w["seq"], "taskIds": missing})
            w["phase"], w["left"] = "TIMEOUT", w["grace"]
            return False
        kind = "PICKUP_FAILED" if w["load"] else "DELIVERY_FAILED"   # hết cả 2 phút → tự đi, hàng mang về
        print(f"[{self.rid}] Không ai xử lý: {kind} {missing} → tự đi tiếp", flush=True)
        self.radio.send_event({"type": kind, "stopSeq": w["seq"], "taskIds": missing})
        return True

    # ---- vị trí ----------------------------------------------------------
    def sense(self):
        gx, gy, _ = self.gps.getValues()
        n = self.compass.getValues()
        if not math.isnan(gx):
            self.x, self.y = gx, gy
            self.th = math.atan2(n[0], n[1])          # la bàn → hướng: 0 = đông, π/2 = bắc
        moved = math.dist(self.prev, (self.x, self.y)) if self.prev else 0.0
        self.prev = (self.x, self.y)
        self.speed_now = 0.8 * self.speed_now + 0.2 * moved / self.dt
        return moved if moved < 1.0 else 0.0

    # ---- lệnh ------------------------------------------------------------
    def path_from_order(self, o):
        if o.get("path"):                             # backend đã tính đường → đi đúng như vậy
            return [(p.get("nodeId", "?"), float(p["x"]), float(p["y"]), p.get("stopSeq")) for p in o["path"]]
        cur = self.map.nearest(self.x, self.y)        # chỉ có điểm dừng → tạm tự tìm đường trong bản đồ
        out = []
        for s in sorted(o.get("stops", []), key=lambda s: s.get("seq", 0)):
            nodes = self.map.route(cur, s["nodeId"])
            for k, nid in enumerate(nodes):
                if out and out[-1][0] == nid:
                    continue
                if not out and math.dist((self.x, self.y), self.map.xy[nid]) < 0.3:
                    continue
                x, y = self.map.xy[nid]
                out.append((nid, x, y, s.get("seq") if k == len(nodes) - 1 else None))
            cur = s["nodeId"]
        return out

    def handle(self, raw):
        try:
            o = json.loads(raw)
        except ValueError:
            print(f"[{self.rid}] Bỏ qua lệnh không phải JSON", flush=True)
            return
        cmd = str(o.get("command", "")).upper()
        cid = o.get("commandId")
        if cid:                                         # backend gửi "ít nhất 1 lần" → có thể trùng, bỏ lệnh đã làm
            if cid in self.done_cmds:
                return
            self.done_cmds.add(cid)
        if cmd == "STOP":                               # DỪNG KHẨN (nút đỏ của Operator): đứng yên, GIỮ lệnh đang dở
            if self.error != "OPERATOR_STOP":
                self.before_error = self.status if self.status != "ERROR" else self.before_error
                self.status, self.error = "ERROR", "OPERATOR_STOP"
                self.drive.stop()
                print(f"[{self.rid}] DỪNG KHẨN theo lệnh Operator", flush=True)
            self.radio.send_event({"type": "COMMAND_DONE", "command": "STOP", "commandId": cid})
            return
        if cmd == "RESUME":                             # bỏ dừng khẩn → chạy tiếp lệnh đang dở (không tự nhận lệnh mới)
            if self.error == "OPERATOR_STOP":
                self.status, self.error = self.before_error or "IDLE", None
                print(f"[{self.rid}] Hết dừng khẩn → {self.status}", flush=True)
            self.radio.send_event({"type": "COMMAND_DONE", "command": "RESUME", "commandId": cid})
            return
        if cmd == "CANCEL":
            self.path, self.i, self.order_id, self.wait = [], 0, None, None
            self.status = "IDLE"
            self.drive.stop()
            print(f"[{self.rid}] Huỷ lệnh, đứng yên", flush=True)
            return
        if cmd == "CONFIRM":                            # Operator bấm "Đã lấy hàng" / "Cho đi tiếp"
            if self.wait:
                self.wait["confirmedBy"] = "OPERATOR"
            return
        if cmd == "FAULT":                              # GIẢ LẬP SỰ CỐ để thử WF-04 (chỉ dùng khi test/demo)
            fault = str(o.get("fault", "")).upper()
            if fault == "BATTERY":                      # pin tụt về mức X — chạy tiếp vài mét là chạm ngưỡng nguy hiểm
                self.battery = float(o.get("value", BATTERY_CRITICAL + 2))
                self.low_warned = self.battery <= BATTERY_LOW
                print(f"[{self.rid}] GIẢ LẬP: pin tụt còn {self.battery:.0f}%", flush=True)
            elif fault == "OFFLINE":                    # rớt mạng N giây, robot VẪN chạy tiếp lệnh đang có
                sec = float(o.get("seconds", 20))
                print(f"[{self.rid}] GIẢ LẬP: MẤT KẾT NỐI {sec:.0f} s — vẫn chạy lệnh đang có, không báo được về", flush=True)
                self.radio.go_dark(sec, self.robot.getTime())
            return
        if cmd == "RECOVER":                            # người tới xử lý: thay/sạc pin tại chỗ rồi cho chạy tiếp
            if self.error and self.error != "OPERATOR_STOP":   # dừng khẩn thì phải RESUME, không phải RECOVER
                print(f"[{self.rid}] ĐÃ XỬ LÝ sự cố {self.error} → pin {max(self.battery, RESCUE_BATTERY):.0f}%, "
                      f"chạy tiếp", flush=True)
                self.radio.send_event({"type": "RECOVERED", "error": self.error})
                self.battery = max(self.battery, RESCUE_BATTERY)
                self.status, self.error, self.low_warned = self.before_error or "IDLE", None, False
            return
        if cmd == "EXTEND":                             # Operator bấm "Chờ thêm"
            if self.wait and self.wait["phase"] == "TIMEOUT":
                self.wait["phase"], self.wait["left"] = "WAITING", float(o.get("seconds", 120))
                print(f"[{self.rid}] Operator cho chờ thêm {self.wait['left']:.0f} s", flush=True)
            return
        try:
            path = self.path_from_order(o)
        except (KeyError, ValueError) as e:
            print(f"[{self.rid}] Lệnh lỗi: {e}", flush=True)
            self.status = "ERROR"
            return
        if not path:
            print(f"[{self.rid}] Lệnh không có điểm nào", flush=True)
            return
        self.order_id, self.path, self.i, self.at_stop, self.wait = o.get("orderId"), path, 0, None, None
        self.task_slot = {c["taskId"]: int(c["slot"]) for c in o.get("compartments", []) if c.get("taskId")}
        self.task_item = {c["taskId"]: c.get("item") for c in o.get("compartments", []) if c.get("taskId")}
        self.stop_info = {s.get("seq"): s for s in o.get("stops", [])}
        self.status = "MOVING"
        kind = "path (backend tính đường)" if o.get("path") else "stops (robot tự tìm đường)"
        print(f"[{self.rid}] NHẬN LỆNH {self.order_id} kiểu {kind}: "
              + " → ".join(p[0] + (f"[dừng {p[3]}]" if p[3] is not None else "") for p in path), flush=True)

    def go_home(self):
        node = self.home_node or self.map.nearest(*self.home_xy)
        nodes = self.map.route(self.map.nearest(self.x, self.y), node)
        self.path = [(n, *self.map.xy[n], None) for n in nodes] + [("HOME", *self.home_xy, None)]
        self.i, self.status = 0, "RETURNING"
        print(f"[{self.rid}] Xong lệnh {self.order_id} → về trạm {node}", flush=True)

    # ---- mỗi bước ------------------------------------------------------------
    def update(self, moved):
        if self.status == "CHARGING":
            self.battery = min(100.0, self.battery + CHARGE_PER_S * self.dt)
            if self.battery >= 95:
                self.status = "IDLE"
            self.drive.stop()
            return
        self.battery = max(0.0, self.battery - BATTERY_PER_M * moved)
        if self.status == "ERROR":                       # đang sự cố → đứng yên chờ người (lệnh RECOVER)
            self.drive.stop()
            return
        if self.check_battery():
            return
        if self.status == "AT_STOP":
            self.drive.stop()
            if self.wait:                                # chờ cảm biến ngăn hàng
                go = self.update_wait()
            else:                                        # không có thông tin ngăn → đứng cứng 5 s
                self.dwell_left -= self.dt
                go = self.dwell_left <= 0
            if go:
                self.status, self.at_stop, self.wait = "MOVING", None, None
                self.i += 1
                self._after_point()
            return
        if self.status not in ("MOVING", "RETURNING") or self.i >= len(self.path):
            self.drive.stop()
            return
        nid, tx, ty, stop = self.path[self.i]
        last = self.i == len(self.path) - 1
        tol = STOP_TOL if (stop is not None or last) else PASS_TOL
        d = self.drive.toward(self.x, self.y, self.th, tx, ty, self.speed, stop is not None or last, moved)
        if d < tol:
            if stop is not None and self.status == "MOVING":
                self.status, self.at_stop = "AT_STOP", stop
                self.drive.stop()
                print(f"[{self.rid}] TỚI điểm dừng {stop}: {nid} — {self.start_wait(stop)}", flush=True)
                self.send_state()
                return
            self.i += 1
            self._after_point()

    def check_battery(self):
        """Pin ≤ 15%: báo BATTERY_LOW một lần. Pin ≤ 10% khi đang làm việc: DỪNG TẠI CHỖ, báo BATTERY_CRITICAL
        (hàng vẫn trong ngăn) → backend/Operator quyết: cứu tại chỗ (RECOVER) hay giao việc cho robot khác.
        Trả True nếu vừa dừng vì pin."""
        working = self.status in ("MOVING", "AT_STOP", "RETURNING")
        if working and self.battery <= BATTERY_LOW and not self.low_warned:
            self.low_warned = True
            print(f"[{self.rid}] PIN YẾU {self.battery:.0f}% (≤ {BATTERY_LOW}%)", flush=True)
            self.radio.send_event({"type": "BATTERY_LOW", "battery": int(self.battery), "orderId": self.order_id})
        if working and self.battery <= BATTERY_CRITICAL:
            self.before_error, self.status, self.error = self.status, "ERROR", "BATTERY_CRITICAL"
            self.drive.stop()
            cargo = [{"slot": i, "taskId": self._slot_task(i)} for i in sorted(self.slots) if self.slot_full[i]]
            print(f"[{self.rid}] HẾT PIN GIỮA ĐƯỜNG {self.battery:.0f}% tại ({self.x:.1f}, {self.y:.1f}) → dừng, "
                  f"chờ người. Hàng đang chở: {cargo or 'không'}", flush=True)
            self.radio.send_event({"type": "BATTERY_CRITICAL", "battery": int(self.battery), "orderId": self.order_id,
                                   "x": round(self.x, 2), "y": round(self.y, 2), "cargo": cargo})
            self.send_state()
            return True
        return False

    def _after_point(self):
        if self.i < len(self.path):
            return
        self.drive.stop()
        if self.status == "RETURNING":
            self.status = "CHARGING" if self.battery < 95 else "IDLE"
            self.order_id = None
            print(f"[{self.rid}] Đã về trạm, {'sạc pin' if self.status == 'CHARGING' else 'sẵn sàng'}", flush=True)
        else:
            self.go_home()

    def send_state(self):
        nxt = self.path[self.i][0] if self.status in ("MOVING", "RETURNING") and self.i < len(self.path) else None
        self.radio.send_state({
            "robotId": self.rid, "ts": now_iso(), "floor": self.floor,
            "x": round(self.x, 2), "y": round(self.y, 2), "theta": round(self.th, 3),
            "battery": int(round(self.battery)), "speed": round(self.speed_now, 2),
            "status": self.status, "orderId": self.order_id, "nextNodeId": nxt, "atStopSeq": self.at_stop,
            "error": self.error,                     # THÊM (24/11): mã sự cố khi status = ERROR, vd BATTERY_CRITICAL
            # THÊM (22/11): trạng thái từng ngăn theo cảm biến + đang chờ gì ở điểm dừng
            "compartments": [{"slot": i, "occupied": self.slot_full[i], "taskId": self._slot_task(i)}
                             for i in sorted(self.slots)],
            "wait": None if not self.wait else {
                "stopSeq": self.wait["seq"], "expect": "LOAD" if self.wait["load"] else "UNLOAD",
                "slots": self.wait["slots"], "phase": self.wait["phase"], "leftSec": round(self.wait["left"])},
        })

    def run(self):
        self.robot.step(self.step_ms)
        self.robot.step(self.step_ms)
        self.sense()
        self.home_xy = (self.x, self.y)
        print(f"[{self.rid}] SẴN SÀNG tại {self.home_node} ({self.x:.2f}, {self.y:.2f}) tầng {self.floor}", flush=True)
        while self.robot.step(self.step_ms) != -1:
            moved = self.sense()
            self.read_slots()
            while not self.radio.inbox.empty():
                self.handle(self.radio.inbox.get())
            if self.radio.check_dark(self.robot.getTime()):
                print(f"[{self.rid}] CÓ MẠNG LẠI → nối lại broker, báo ONLINE", flush=True)
            self.update(moved)
            led = LED.get(self.status, 1)
            if self.wait:                                 # trắng = đang chờ hàng; đỏ = hết giờ chờ
                led = 4 if self.wait["phase"] == "TIMEOUT" else 5
            self.led.set(led)
            t = self.robot.getTime()
            if t >= self.next_state:
                self.next_state = t + 1.0
                self.send_state()


if __name__ == "__main__":
    RmmsRobot().run()
