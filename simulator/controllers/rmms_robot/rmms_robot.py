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
MQTT_HOST = os.environ.get("MQTT_HOST", "127.0.0.1")
MQTT_PORT = int(os.environ.get("MQTT_PORT", "1883"))

WHEEL_R, TRACK, MAX_WHEEL = 0.08, 0.42, 20.0
STOP_TOL, PASS_TOL = 0.25, 0.45       # sai số khi tới điểm dừng / điểm đi ngang (m)
DWELL_S = 5.0                         # đứng ở điểm dừng bao lâu (tạm: sau này chờ xác nhận / sensor ngăn)
BATTERY_PER_M = 0.15                  # % pin mỗi mét
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

    def _on_connect(self, client, userdata, flags, rc, props=None):
        client.subscribe(f"rmms/v1/{self.rid}/order", qos=1)
        client.publish(f"rmms/v1/{self.rid}/connection",
                       json.dumps({"robotId": self.rid, "connection": "ONLINE", "ts": now_iso()}), qos=1, retain=True)
        print(f"[{self.rid}] MQTT đã kết nối {MQTT_HOST}:{MQTT_PORT}, đang nghe rmms/v1/{self.rid}/order", flush=True)

    @property
    def online(self):
        return self.client is not None and self.client.is_connected()

    def send_state(self, payload):
        if self.online:
            self.client.publish(f"rmms/v1/{self.rid}/state", json.dumps(payload), qos=0)


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
        if str(o.get("command", "")).upper() in ("CANCEL", "STOP"):
            self.path, self.i, self.order_id = [], 0, None
            self.status = "IDLE"
            self.drive.stop()
            print(f"[{self.rid}] Huỷ lệnh, đứng yên", flush=True)
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
        self.order_id, self.path, self.i, self.at_stop = o.get("orderId"), path, 0, None
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
        if self.status == "AT_STOP":
            self.drive.stop()
            self.dwell_left -= self.dt
            if self.dwell_left <= 0:
                self.status, self.at_stop = "MOVING", None
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
                self.status, self.at_stop, self.dwell_left = "AT_STOP", stop, DWELL_S
                self.drive.stop()
                print(f"[{self.rid}] TỚI điểm dừng {stop}: {nid} — chờ {DWELL_S:.0f} s", flush=True)
                self.send_state()
                return
            self.i += 1
            self._after_point()

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
        })

    def run(self):
        self.robot.step(self.step_ms)
        self.robot.step(self.step_ms)
        self.sense()
        self.home_xy = (self.x, self.y)
        print(f"[{self.rid}] SẴN SÀNG tại {self.home_node} ({self.x:.2f}, {self.y:.2f}) tầng {self.floor}", flush=True)
        while self.robot.step(self.step_ms) != -1:
            moved = self.sense()
            while not self.radio.inbox.empty():
                self.handle(self.radio.inbox.get())
            self.update(moved)
            self.led.set(LED.get(self.status, 1))
            t = self.robot.getTime()
            if t >= self.next_state:
                self.next_state = t + 1.0
                self.send_state()


if __name__ == "__main__":
    RmmsRobot().run()
