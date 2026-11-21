"""Đóng vai backend để thử robot Webots mà KHÔNG cần backend thật.

    python tools/send_order.py watch                                  # xem robot báo cáo (state + connection)
    python tools/send_order.py MED-01 F2-PHARM F2-WARD-A              # gửi lệnh kiểu HIỆN TẠI của Hải: chỉ có stops
    python tools/send_order.py MED-01 F2-PHARM F2-WARD-A --path       # gửi lệnh kiểu ĐỀ XUẤT: có path (x, y) do "backend" tính
    python tools/send_order.py MED-01 cancel                          # huỷ lệnh, robot đứng yên

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
        else:
            print(f"{time.strftime('%H:%M:%S')} {msg.topic}: {p.get('connection')}")

    c.on_message = on_msg
    c.subscribe([("rmms/v1/+/state", 0), ("rmms/v1/+/connection", 1)])
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
    if stops == ["cancel"]:
        payload = {"command": "CANCEL", "robotId": robot_id}
    else:
        floor = robot_id and json.load(open(os.path.join(ROOT, "map", "F2.json"), encoding="utf-8"))
        known = {n["id"] for n in floor["nodes"]}
        bad = [s for s in stops if s not in known]
        if bad:
            sys.exit(f"Không có điểm {bad} trong map/F2.json")
        payload = {
            "orderId": f"T-TEST-{time.strftime('%H%M%S')}", "robotId": robot_id, "mapVersionId": None,
            "stops": [{"seq": i, "nodeId": s, "action": "PICKUP" if i == 1 else "DELIVER", "taskIds": []}
                      for i, s in enumerate(stops, 1)],
            "compartments": [],
        }
        if with_path:
            payload["path"] = plan_path(floor, robot_id, stops)
    topic = f"rmms/v1/{robot_id}/order"
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
