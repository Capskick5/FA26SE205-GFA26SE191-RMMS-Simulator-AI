"""mini_backend — "backend giả" để thấy AI điều khiển robot Webots, KHÔNG cần backend thật của Hải.

Làm đúng những việc backend thật sẽ làm, theo đúng thứ tự:
  1. Lấy danh sách việc chờ        (thật: bảng Task trong DB   | giả: file demo/scenarios/*.json)
  2. Hỏi robot đang ở đâu, pin bao nhiêu  (nghe MQTT rmms/v1/+/state vài giây)
  3. Tính bảng khoảng cách          (Dijkstra trên bản đồ tầng)
  4. Gọi AI: POST /plan              (AI lỗi/tắt → luật dự phòng BM04: mỗi chuyến 1 việc)
  5. Đổi kế hoạch thành lệnh, gửi   rmms/v1/{robotId}/order   (đúng contracts/mqtt.md, có path)
  6. Xem robot chạy tới khi về hết  (--watch)

    python demo/mini_backend.py                       # chạy kịch bản mặc định, gửi lệnh, xem robot chạy
    python demo/mini_backend.py --dry-run             # chỉ in kế hoạch, không gửi lệnh
    python demo/mini_backend.py --scenario demo/scenarios/f2_gio_cao_diem.json --no-watch

Cần: Mosquitto (task "1. Mosquitto: bật"), AI (python ai-scheduler/server.py), Webots đang chạy world F2.
"""
import argparse
import json
import os
import sys
import time
import urllib.error
import urllib.request
from datetime import datetime, timedelta, timezone
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))
from floor_graph import FloorGraph  # noqa: E402

try:
    import paho.mqtt.client as mqtt
except ImportError:
    mqtt = None

if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")

HERE = Path(__file__).resolve().parent
HOST, PORT = os.environ.get("MQTT_HOST", "127.0.0.1"), int(os.environ.get("MQTT_PORT", "1883"))
FREE = ("IDLE", "CHARGING")      # robot ở trạng thái này mới được giao việc


def say(step, text):
    print(f"\n=== Bước {step}: {text}")


def mqtt_client(name):
    if mqtt is None:
        sys.exit("Thiếu paho-mqtt → pip install paho-mqtt")
    c = (mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id=name)
         if hasattr(mqtt, "CallbackAPIVersion") else mqtt.Client(client_id=name))
    try:
        c.connect(HOST, PORT, 30)
    except OSError:
        sys.exit(f"Không nối được Mosquitto {HOST}:{PORT} — bật bằng task '1. Mosquitto: bật' trước.")
    return c


# ------------------------------------------------------------------ bước 2
def read_robot_states(seconds):
    """Nghe state của mọi robot trong vài giây, lấy bản tin mới nhất của từng con."""
    states = {}
    c = mqtt_client(f"rmms-minibe-read-{os.getpid()}")
    c.on_message = lambda cl, u, m: states.__setitem__(m.topic.split("/")[2], json.loads(m.payload))
    c.subscribe("rmms/v1/+/state", 0)
    c.loop_start()
    time.sleep(seconds)
    c.loop_stop()
    c.disconnect()
    return states


def build_robots(g, states, offline):
    types = g.data["robot_types"]
    robots, notes = [], []
    for r in g.data["robots"]:
        t = types[r["type"]]
        s = states.get(r["id"])
        if s is None:
            if not offline:
                notes.append(f"{r['id']}: không nghe thấy state (Webots chưa chạy?) → giả sử ở trạm {r['home']}, pin 100%")
            node, battery = r["home"], 100
        elif s.get("status") not in FREE:
            notes.append(f"{r['id']}: đang {s.get('status')} → không giao thêm")
            continue
        else:
            node, battery = g.nearest(s["x"], s["y"]), s.get("battery", 100)
        robots.append({"robotId": r["id"], "type": r["type"], "node": node, "battery": battery,
                       "speed": t["speed"], "slots": t["slots"]})
    return robots, notes


# ------------------------------------------------------------------ bước 4
def call_ai(url, req):
    body = json.dumps(req).encode("utf-8")
    rq = urllib.request.Request(url.rstrip("/") + "/plan", body, {"Content-Type": "application/json"})
    with urllib.request.urlopen(rq, timeout=5) as resp:
        return json.loads(resp.read())


def fallback_bm04(req):
    """Luật dự phòng khi AI lỗi: việc gấp trước, mỗi robot rảnh nhận đúng 1 việc cùng loại, mỗi chuyến 1 việc."""
    order = sorted(req["tasks"], key=lambda t: (t["priority"] != "URGENT", t["deadline"]))
    free = {r["robotId"]: r for r in req["robots"] if r["battery"] >= 20}
    plans, left = [], []
    for t in order:
        bot = next((r for r in free.values() if r["type"] == t["robotType"]), None)
        if bot is None:
            left.append({"taskId": t["taskId"], "reason": "chờ lượt sau (luật dự phòng: 1 việc/chuyến)"})
            continue
        del free[bot["robotId"]]
        plans.append({"robotId": bot["robotId"], "taskIds": [t["taskId"]], "stops": [
            {"seq": 1, "nodeId": t["pickup"], "action": "PICKUP", "taskIds": [t["taskId"]]},
            {"seq": 2, "nodeId": t["dropoff"], "action": "DELIVER", "taskIds": [t["taskId"]]}]})
    return {"plans": plans, "unassigned": left, "explain": [], "score": None, "meta": {"algorithm": "BM04 fallback"}}


# ------------------------------------------------------------------ bước 5
def compartments_for(stops, tasks):
    """Chia ngăn: lấy hàng thì vào ngăn trống nhỏ nhất, giao xong thì ngăn trống lại."""
    used, out = {}, []
    for s in stops:
        for tid in s["taskIds"]:
            if s["action"] == "PICKUP":
                slot = next(i for i in range(1, 99) if i not in used.values())
                used[tid] = slot
                t = tasks[tid]
                out.append({"slot": slot, "taskId": tid, "item": t.get("item"), "qty": t.get("qty")})
            else:
                used.pop(tid, None)
    return out


def build_orders(g, plan_out, req, scenario_tasks):
    start = {r["robotId"]: r["node"] for r in req["robots"]}
    stamp = time.strftime("%H%M%S")
    orders = []
    for p in plan_out["plans"]:
        orders.append({
            "orderId": f"T-{stamp}-{p['robotId']}",
            "robotId": p["robotId"],
            "mapVersionId": f"{req.get('floor', 'F2')}-demo",
            "stops": p["stops"],
            "path": g.path_for(start[p["robotId"]], p["stops"]),
            "compartments": compartments_for(p["stops"], scenario_tasks),
        })
    return orders


def publish(orders):
    c = mqtt_client(f"rmms-minibe-send-{os.getpid()}")
    c.loop_start()
    for o in orders:
        c.publish(f"rmms/v1/{o['robotId']}/order", json.dumps(o), qos=1).wait_for_publish(5)
        print(f"  → rmms/v1/{o['robotId']}/order  {o['orderId']}: "
              + " → ".join(f"{s['nodeId']}({s['action'][0]}:{','.join(s['taskIds'])})" for s in o["stops"]))
    c.loop_stop()
    c.disconnect()


# ------------------------------------------------------------------ bước 6
def watch(orders, tasks, t0):
    """In mỗi khi robot đổi trạng thái; đánh dấu từng việc giao kịp/trễ; dừng khi giao đủ."""
    busy = {o["robotId"]: o for o in orders}
    assigned = {tid for o in orders for s in o["stops"] for tid in s["taskIds"]}
    last, done = {}, set()
    deadline_at = {k: t0 + v["deadlineInSec"] for k, v in tasks.items()}
    c = mqtt_client(f"rmms-minibe-watch-{os.getpid()}")

    def on_msg(cl, u, m):
        s = json.loads(m.payload)
        rid = s["robotId"]
        if rid not in busy:
            return
        if m.topic.endswith("/event"):            # cảm biến ngăn đổi, xác nhận xong, hết giờ chờ...
            el = time.time() - t0
            if s["type"] == "SLOT":
                print(f"  [{el:6.1f}s] {rid:<8} ◆ cảm biến ngăn {s['slot']}: "
                      f"{'CÓ HÀNG' if s['occupied'] else 'TRỐNG'} (đo {s['distanceM']} m) — {s.get('taskId')}", flush=True)
            elif s["type"] == "STOP_DONE":
                print(f"  [{el:6.1f}s] {rid:<8} ◆ xác nhận bởi {s['by']}: {s['action']} {', '.join(s['taskIds'])}",
                      flush=True)
            else:
                print(f"  [{el:6.1f}s] {rid:<8} ⚠ {s['type']} {s.get('taskIds')}", flush=True)
            return
        key = (s["status"], s.get("atStopSeq"))
        if last.get(rid) == key:
            return
        last[rid] = key
        el = time.time() - t0
        extra = ""
        if s["status"] == "AT_STOP" and s.get("atStopSeq"):
            stop = next(x for x in busy[rid]["stops"] if x["seq"] == s["atStopSeq"])
            extra = f" tại {stop['nodeId']} — {stop['action']} {', '.join(stop['taskIds'])}"
            if stop["action"] == "DELIVER":
                for tid in stop["taskIds"]:
                    late = time.time() - deadline_at[tid]
                    extra += f"\n{'':>18}✔ {tid} giao lúc {el:5.0f}s, hạn {tasks[tid]['deadlineInSec']}s → " \
                             + ("KỊP" if late <= 0 else f"TRỄ {late:.0f}s")
                    done.add(tid)
        print(f"  [{el:6.1f}s] {rid:<8} {s['status']:<9}{extra}", flush=True)
        if assigned <= done:              # giao đủ mọi việc đã phân → thôi xem (robot tự về trạm)
            cl.disconnect()

    c.on_message = on_msg
    c.subscribe([("rmms/v1/+/state", 0), ("rmms/v1/+/event", 1)])
    print("  (Ctrl+C để thôi xem — robot vẫn chạy tiếp)")
    try:
        c.loop_forever()
    except KeyboardInterrupt:
        pass
    print(f"\nXong: giao {len(done)}/{len(tasks)} việc trong {time.time() - t0:.0f} s.")


# ------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser(description="Backend giả: việc chờ → AI → lệnh MQTT → robot Webots")
    ap.add_argument("--scenario", default=str(HERE / "scenarios" / "f2_gio_cao_diem.json"))
    ap.add_argument("--ai", default=os.environ.get("AI_URL", "http://127.0.0.1:8000"))
    ap.add_argument("--dry-run", action="store_true", help="chỉ in kế hoạch, không gửi lệnh")
    ap.add_argument("--offline", action="store_true", help="không nghe MQTT, coi mọi robot ở trạm, pin 100%%")
    ap.add_argument("--no-watch", action="store_true", help="gửi lệnh xong thì thoát")
    a = ap.parse_args()

    sc = json.loads(Path(a.scenario).read_text(encoding="utf-8"))
    g = FloorGraph(sc.get("floor", "F2"))
    tasks = {t["taskId"]: t for t in sc["tasks"]}
    now = datetime.now(timezone.utc)
    iso = lambda dt: dt.strftime("%Y-%m-%dT%H:%M:%SZ")  # noqa: E731

    say(1, f"Lấy việc chờ — {sc.get('name', a.scenario)}")
    for t in sc["tasks"]:
        print(f"  {t['taskId']} {'GẤP  ' if t['priority'] == 'URGENT' else 'thường'} {t['robotType']:<11} "
              f"{t['pickup']} → {t['dropoff']:<10} hạn {t['deadlineInSec']:>4}s")

    say(2, "Hỏi robot đang ở đâu (nghe MQTT state)")
    states = {} if a.offline else read_robot_states(2.5)
    robots, notes = build_robots(g, states, a.offline)
    for r in robots:
        print(f"  {r['robotId']:<8} {r['type']:<11} ở {r['node']:<10} pin {r['battery']}%")
    for n in notes:
        print(f"  ! {n}")

    say(3, "Tính bảng khoảng cách (Dijkstra)")
    nodes = [r["node"] for r in robots] + [t["pickup"] for t in sc["tasks"]] + [t["dropoff"] for t in sc["tasks"]]
    dist = g.distances(nodes)
    print(f"  {len(dist)} điểm × {len(dist)} điểm, ví dụ F2-PHARM → F2-LAB = {dist.get('F2-PHARM', {}).get('F2-LAB')} m")

    req = {
        "now": iso(now), "floor": sc.get("floor", "F2"), "config": sc.get("config", {}),
        "robots": robots, "distances": dist,
        "tasks": [{"taskId": t["taskId"], "priority": t["priority"], "robotType": t["robotType"],
                   "pickup": t["pickup"], "dropoff": t["dropoff"], "slots": t.get("slots", 1),
                   "createdAt": iso(now), "deadline": iso(now + timedelta(seconds=t["deadlineInSec"]))}
                  for t in sc["tasks"]],
    }

    say(4, f"Gọi AI: POST {a.ai}/plan")
    try:
        out = call_ai(a.ai, req)
        s = out["score"]
        print(f"  AI trả lời sau {out['meta']['elapsedMs']} ms ({out['meta']['algorithm']})")
        print(f"  Điểm phạt: bản nháp {s['draft']} → AI {s['final']} (giảm {s['improvementPct']}%) | "
              f"việc trễ dự kiến: {s['draftLateTasks']} → {s['lateTasks']}")
        for e in out["explain"]:
            print(f"   • {e['text']}")
    except (urllib.error.URLError, OSError, ValueError, KeyError) as e:
        print(f"  ! AI không trả lời ({e}) → dùng LUẬT DỰ PHÒNG BM04")
        out = fallback_bm04(req)
    for u in out["unassigned"]:
        print(f"   ✗ {u['taskId']}: {u['reason']}")

    say(5, "Đổi kế hoạch thành lệnh MQTT")
    orders = build_orders(g, out, req, tasks)
    if a.dry_run:
        print(json.dumps(orders[0], ensure_ascii=False, indent=1) if orders else "  (không có lệnh)")
        print("\n--dry-run: không gửi.")
        return
    t0 = time.time()
    publish(orders)

    if not a.no_watch and orders:
        say(6, "Xem robot chạy")
        watch(orders, tasks, t0)


if __name__ == "__main__":
    main()
