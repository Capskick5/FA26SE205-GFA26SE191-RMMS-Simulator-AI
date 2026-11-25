"""Đóng vai backend để thử robot Webots mà KHÔNG cần backend thật.

    python tools/send_order.py watch                                  # xem robot báo cáo (state + connection)
    python tools/send_order.py MED-01 F2-PHARM F2-WARD-A              # gửi lệnh kiểu HIỆN TẠI của Hải: chỉ có stops
    python tools/send_order.py MED-01 F2-PHARM F2-WARD-A --path       # gửi lệnh kiểu ĐỀ XUẤT: có path (x, y) do "backend" tính
    python tools/send_order.py MED-01 cancel                          # huỷ lệnh, robot đứng yên
    python tools/send_order.py MED-01 confirm                         # Operator bấm "Đã lấy hàng / cho đi tiếp"
    python tools/send_order.py MED-01 extend 120                      # Operator bấm "Chờ thêm 120 s"
    python tools/send_order.py MED-01 fault battery 12                # SỰ CỐ: pin tụt còn 12% → chạy thêm ~13 m là hết pin, dừng
    python tools/send_order.py MED-01 fault offline 20                # SỰ CỐ: rớt mạng 20 s (robot vẫn chạy, không báo về)
    python tools/send_order.py MED-01 recover                         # người tới xử lý: thay pin tại chỗ, chạy tiếp
    python tools/send_order.py MED-01 stop                            # DỪNG KHẨN (giống nút của Hải, topic /command)
    python tools/send_order.py MED-01 resume                          # bỏ dừng khẩn, chạy tiếp
    python tools/send_order.py nurse absent NV-TEST                   # y tá giả bỏ qua việc NV-TEST (thử hết giờ)
    python tools/send_order.py nurse normal                           # y tá giả làm bình thường lại

Lệnh gửi tay có việc NV-TEST ở ngăn 1: điểm đầu y tá đặt hộp, điểm thứ hai y tá lấy hộp → xem cảm biến.

Điểm đầu tiên = điểm LẤY hàng, các điểm sau = điểm GIAO. Mã điểm xem trong map/F2.json (vd F2-PHARM, F2-WARD-A, F2-ICU).
Cần Mosquitto đang chạy (task VS Code "1. Mosquitto: bật").
"""
import heapq
import json
import math
import os
import sys
import time

import paho.mqtt.client as mqtt

if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
HOST, PORT = os.environ.get("MQTT_HOST", "127.0.0.1"), int(os.environ.get("MQTT_PORT", "1883"))


def client(name):
    c = (mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id=name)
         if hasattr(mqtt, "CallbackAPIVersion") else mqtt.Client(client_id=name))
    try:
        c.connect(HOST, PORT, 30)
    except OSError:
        sys.exit(f"Không nối được Mosquitto {HOST}:{PORT} — bật bằng task '1. Mosquitto: bật' trước.")
    return c


def watch():
    c = client(f"rmms-watch-{os.getpid()}")

    def on_msg(cl, u, msg):
        p = json.loads(msg.payload)
        kind = msg.topic.split("/")[-1]
        if kind == "state":
            stop = f" | ĐANG Ở điểm dừng {p['atStopSeq']}" if p.get("atStopSeq") is not None else ""
            print(f"{time.strftime('%H:%M:%S')} {p['robotId']:<9} {p['status']:<9} ({p['x']:6.2f},{p['y']:6.2f}) "
                  f"pin {p['battery']:3d}% v={p['speed']:.2f} lệnh={p.get('orderId')} tới={p.get('nextNodeId')}{stop}")
        elif kind == "event":
            detail = {k: v for k, v in p.items() if k not in ("robotId", "ts", "type")}
            print(f"{time.strftime('%H:%M:%S')} {p['robotId']:<9} ★ SỰ KIỆN {p['type']}: {detail}")
        else:
            print(f"{time.strftime('%H:%M:%S')} {msg.topic}: {p.get('connection')}")

    c.on_message = on_msg
    c.subscribe([("rmms/v1/+/state", 0), ("rmms/v1/+/connection", 1), ("rmms/v1/+/event", 1)])
    print("Đang xem rmms/v1/+/state và /connection — Ctrl+C để thoát")
    try:
        c.loop_forever()
    except KeyboardInterrupt:
        pass


def plan_path(floor_map, robot_id, stops):
    """Làm giống backend sẽ làm: Dijkstra từ trạm nhà của robot qua các điểm dừng → dãy (nodeId, x, y)."""
    xy = {n["id"]: (n["x"], n["y"]) for n in floor_map["nodes"]}
    adj = {n: [] for n in xy}
    for a, b in floor_map["edges"]:
        d = math.dist(xy[a], xy[b])
        adj[a].append((b, d))
        adj[b].append((a, d))

    def route(s, g):
        best, prev, h = {s: 0}, {}, [(0, s)]
        while h:
            c, n = heapq.heappop(h)
            for m, d in adj[n]:
                if c + d < best.get(m, math.inf):
                    best[m], prev[m] = c + d, n
                    heapq.heappush(h, (c + d, m))
        p = [g]
        while p[-1] != s:
            p.append(prev[p[-1]])
        return p[::-1]

    cur = next(r["home"] for r in floor_map["robots"] if r["id"] == robot_id)
    path = []
    for seq, stop in enumerate(stops, 1):
        nodes = route(cur, stop)
        for k, n in enumerate(nodes):
            if path and path[-1]["nodeId"] == n:
                continue
            item = {"nodeId": n, "x": xy[n][0], "y": xy[n][1]}
            if k == len(nodes) - 1:
                item["stopSeq"] = seq
            path.append(item)
        cur = stop
    return path


def send(robot_id, stops, with_path):
    c = client(f"rmms-test-{os.getpid()}")
    c.loop_start()
    if robot_id == "nurse":                                    # điều khiển "y tá giả" trong Webots
        payload = {"mode": "normal"} if stops[0] == "normal" else {"absent": stops[1:] or ["all"]}
        c.publish("rmms/sim/nurse", json.dumps(payload), qos=1).wait_for_publish(5)
        print(f"Đã gửi rmms/sim/nurse: {payload}")
        return
    topic = f"rmms/v1/{robot_id}/order"
    if stops[0] in ("stop", "resume"):                         # đúng định dạng backend của Hải (25/11)
        import uuid
        topic = f"rmms/v1/{robot_id}/command"
        payload = {"commandId": str(uuid.uuid4()), "command": stops[0].upper(), "issuedAt": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())}
    elif stops == ["cancel"]:
        payload = {"command": "CANCEL", "robotId": robot_id}
    elif stops[0] == "fault":                                  # giả lập sự cố: fault battery 12 | fault offline 20
        kind = stops[1].upper() if len(stops) > 1 else "BATTERY"
        payload = {"command": "FAULT", "robotId": robot_id, "fault": kind}
        if len(stops) > 2:
            payload["value" if kind == "BATTERY" else "seconds"] = float(stops[2])
    elif stops[0] == "recover":                                # người tới xử lý sự cố, cho robot chạy tiếp
        payload = {"command": "RECOVER", "robotId": robot_id}
    elif stops[0] in ("confirm", "extend"):                    # Operator bấm "Đã lấy hàng" / "Chờ thêm"
        payload = {"command": stops[0].upper(), "robotId": robot_id}
        if stops[0] == "extend":
            payload["seconds"] = int(stops[1]) if len(stops) > 1 else 120
    else:
        floor = robot_id and json.load(open(os.path.join(ROOT, "map", "F2.json"), encoding="utf-8"))
        known = {n["id"] for n in floor["nodes"]}
        bad = [s for s in stops if s not in known]
        if bad:
            sys.exit(f"Không có điểm {bad} trong map/F2.json")
        payload = {
            "orderId": f"T-TEST-{time.strftime('%H%M%S')}", "robotId": robot_id, "mapVersionId": None,
            "stops": [{"seq": i, "nodeId": s, "action": "PICKUP" if i == 1 else "DELIVER",
                       "taskIds": ["NV-TEST"] if i in (1, 2) else []}
                      for i, s in enumerate(stops, 1)],
            "compartments": [{"slot": 1, "taskId": "NV-TEST", "item": "TH-TEST", "qty": 1}],
        }
        if with_path:
            payload["path"] = plan_path(floor, robot_id, stops)
    c.publish(topic, json.dumps(payload), qos=1).wait_for_publish(5)
    print(f"Đã gửi {topic}:\n{json.dumps(payload, ensure_ascii=False, indent=1)}")
    c.loop_stop()


def main():
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    if not args or args[0] == "watch":
        watch()
    elif len(args) < 2:
        print(__doc__)
    else:
        send(args[0], args[1:], "--path" in sys.argv)


if __name__ == "__main__":
    main()
