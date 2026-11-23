"""rmms_nurse — "y tá giả" trong Webots: đặt hộp hàng vào ngăn robot ở điểm LẤY, lấy hộp ra ở điểm GIAO.

Đây là phần GIẢ LẬP CON NGƯỜI, không phải phần của hệ thống RMMS:
  - robot KHÔNG được y tá báo gì; robot chỉ biết hàng đã vào/ra nhờ CẢM BIẾN NGĂN (slot_1, slot_2...)
  - y tá chỉ đặt/lấy một hộp vật lý (có khung va chạm) → cảm biến của robot đo được → robot tự quyết đi tiếp

Cách y tá biết phải làm gì: nghe giống backend — lệnh rmms/v1/+/order (việc nào ở ngăn nào)
và state rmms/v1/+/state (robot đang đứng ở điểm dừng số mấy).

Thử tình huống y tá vắng mặt (để thấy robot báo HẾT GIỜ):
    python tools/send_order.py nurse absent NV-05      # y tá bỏ qua việc NV-05
    python tools/send_order.py nurse absent all        # y tá vắng hẳn
    python tools/send_order.py nurse normal            # y tá làm bình thường lại
"""
import json
import os
import queue
import random
import sys

from controller import Supervisor

try:
    import paho.mqtt.client as mqtt
except ImportError:
    mqtt = None

if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")


class _Tee:
    """RMMS_LOG=1 → in ra console Webots VÀ ghi thêm vào simulator/logs/<tên>.log (để xem lại sau)."""
    def __init__(self, name):
        d = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "logs")
        os.makedirs(d, exist_ok=True)
        self.f, self.out = open(os.path.join(d, f"{name}.log"), "a", encoding="utf-8", buffering=1), sys.stdout

    def write(self, s):
        self.out.write(s)
        self.f.write(s)

    def flush(self):
        self.out.flush()


if os.environ.get("RMMS_LOG"):
    sys.stdout = _Tee("NURSE")

HOST, PORT = os.environ.get("MQTT_HOST", "127.0.0.1"), int(os.environ.get("MQTT_PORT", "1883"))
DELAY_S = (2.0, 5.0)              # y tá mất 2–5 giây mới tới đặt/lấy hàng
ITEM_COLOR = {"TH": "0.9 0.25 0.25", "VT": "0.95 0.6 0.15", "MAU": "0.6 0.3 0.85"}


class Nurse:
    def __init__(self):
        self.sv = Supervisor()
        self.step_ms = int(self.sv.getBasicTimeStep())
        self.inbox = queue.Queue()
        self.orders = {}             # robotId → lệnh mới nhất
        self.done = set()            # (robotId, orderId, stopSeq) đã xử lý
        self.todo = []               # [(lúc_làm, robotId, stop, lệnh)]
        self.absent = set()          # taskId y tá bỏ qua; "all" = vắng hẳn
        self.items = {}              # (robotId, ngăn) → (nút hộp hàng, vị trí ngăn trên thân robot)
        self.rng = random.Random(42)
        if mqtt is None:
            print("[Y TÁ] THIẾU paho-mqtt → pip install paho-mqtt", flush=True)
            return
        c = (mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id="webots-nurse")
             if hasattr(mqtt, "CallbackAPIVersion") else mqtt.Client(client_id="webots-nurse"))
        c.on_connect = lambda cl, *a: cl.subscribe([("rmms/v1/+/order", 1), ("rmms/v1/+/state", 0),
                                                     ("rmms/sim/nurse", 1)])
        c.on_message = lambda cl, u, m: self.inbox.put((m.topic, m.payload))
        c.reconnect_delay_set(1, 5)
        c.connect_async(HOST, PORT, 10)
        c.loop_start()

    # ---- nghe ---------------------------------------------------------------
    def on_message(self, topic, raw):
        try:
            msg = json.loads(raw)
        except ValueError:
            return
        if topic == "rmms/sim/nurse":
            if msg.get("mode") == "normal":
                self.absent.clear()
            self.absent |= set(msg.get("absent", []))
            print(f"[Y TÁ] Bỏ qua: {sorted(self.absent) or 'không — làm bình thường'}", flush=True)
            return
        rid, kind = topic.split("/")[2], topic.split("/")[3]
        if kind == "order" and "stops" in msg:
            self.orders[rid] = msg
        elif kind == "state" and msg.get("status") == "AT_STOP" and msg.get("atStopSeq") is not None:
            order = self.orders.get(rid)
            if not order or order.get("orderId") != msg.get("orderId"):
                return
            key = (rid, order["orderId"], msg["atStopSeq"])
            stop = next((s for s in order["stops"] if s.get("seq") == msg["atStopSeq"]), None)
            if key in self.done or stop is None:
                return
            self.done.add(key)
            self.todo.append((self.sv.getTime() + self.rng.uniform(*DELAY_S), rid, stop, order))

    # ---- làm ----------------------------------------------------------------
    def act(self, rid, stop, order):
        slots = {c["taskId"]: c for c in order.get("compartments", [])}
        load = str(stop.get("action", "")).upper().startswith("PICK")
        for tid in stop.get("taskIds", []):
            c = slots.get(tid)
            if not c:
                continue
            if "all" in self.absent or tid in self.absent:
                print(f"[Y TÁ] (vắng) KHÔNG {'đặt' if load else 'lấy'} {tid} ở ngăn {c['slot']} của {rid}", flush=True)
                continue
            if load:
                self.put_item(rid, int(c["slot"]), c)
            else:
                self.take_item(rid, int(c["slot"]), tid)

    def put_item(self, rid, slot, c):
        """Đặt một hộp hàng vào ngăn. Hộp là vật RIÊNG (không gắn vào robot) vì tia cảm biến Webots
        bỏ qua chính thân robot mang nó — gắn vào robot thì cảm biến sẽ không bao giờ thấy hộp."""
        robot_def = rid.replace("-", "_")
        sensor = self.sv.getFromDef(f"{robot_def}.SLOT_{slot}")
        if sensor is None or self.sv.getFromDef(robot_def) is None:
            print(f"[Y TÁ] {rid} không có ngăn {slot}", flush=True)
            return
        if (rid, slot) in self.items:
            return
        x, y, z = sensor.getField("translation").getSFVec3f()       # vị trí ngăn, tính theo thân robot
        n_slots = sum(1 for i in range(1, 9) if self.sv.getFromDef(f"{robot_def}.SLOT_{i}"))
        w = 0.40 / max(1, n_slots) * 0.85
        color = ITEM_COLOR.get(str(c.get("item") or "").split("-")[0], "0.9 0.9 0.9")
        self.sv.getRoot().getField("children").importMFNodeFromString(-1, (
            f'DEF ITEM_{robot_def}_{slot} Solid {{ translation 0 0 -5 '
            f'children [ Shape {{ appearance PBRAppearance {{ baseColor {color} roughness 0.6 metalness 0 }} '
            f'geometry Box {{ size {w:.3f} 0.2 0.05 }} }} ] '
            f'name "item_{robot_def}_{slot}" boundingObject Box {{ size {w:.3f} 0.2 0.05 }} }}'))
        self.items[(rid, slot)] = (self.sv.getFromDef(f"ITEM_{robot_def}_{slot}"), (x, y, z + 0.03))
        self.follow()
        print(f"[Y TÁ] ĐẶT {c['taskId']} ({c.get('item')} ×{c.get('qty')}) vào ngăn {slot} của {rid}", flush=True)

    def take_item(self, rid, slot, tid):
        item = self.items.pop((rid, slot), None)
        if item:
            item[0].remove()
            print(f"[Y TÁ] LẤY {tid} ra khỏi ngăn {slot} của {rid}", flush=True)

    def follow(self):
        """Mỗi bước: dời từng hộp tới đúng ngăn của robot đang chở nó (như hộp nằm yên trong ngăn)."""
        dt = self.step_ms / 1000.0
        for (rid, _), (node, (lx, ly, lz)) in self.items.items():
            robot = self.sv.getFromDef(rid.replace("-", "_"))
            p, r, v = robot.getPosition(), robot.getOrientation(), robot.getVelocity()
            wx = p[0] + r[0] * lx + r[1] * ly + r[2] * lz + v[0] * dt     # + v·dt: bù trễ một bước
            wy = p[1] + r[3] * lx + r[4] * ly + r[5] * lz + v[1] * dt
            wz = p[2] + r[6] * lx + r[7] * ly + r[8] * lz
            node.getField("translation").setSFVec3f([wx, wy, wz])
            node.getField("rotation").setSFRotation(robot.getField("rotation").getSFRotation())

    def run(self):
        print("[Y TÁ] Sẵn sàng: robot tới điểm lấy thì đặt hàng, tới điểm giao thì lấy hàng", flush=True)
        while self.sv.step(self.step_ms) != -1:
            while not self.inbox.empty():
                self.on_message(*self.inbox.get())
            now = self.sv.getTime()
            for job in [j for j in self.todo if j[0] <= now]:
                self.todo.remove(job)
                self.act(*job[1:])
            self.follow()


if __name__ == "__main__":
    Nurse().run()
