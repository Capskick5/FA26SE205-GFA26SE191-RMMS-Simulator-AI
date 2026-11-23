"""Xem AI nghĩ từng bước — bằng chứng "bộ não AI" chỉ là code Python, chạy trên máy, không gọi đi đâu.

    python ai-scheduler/xem_ai_nghi.py                 # kịch bản demo F2 (7 việc, 3 robot)
    python ai-scheduler/xem_ai_nghi.py <file.json>     # kịch bản khác trong demo/scenarios/

In ra 3 phần:
  A. BẰNG CHỨNG LÀ CODE: file nào, dùng thư viện gì (không có thư viện mạng/AI), chạy 2 lần ra y hệt nhau
  B. NHẬT KÝ SUY NGHĨ: lọc → nháp → sửa dần, từng quyết định kèm điểm phạt
  C. KẾT QUẢ: robot nào chạy những điểm nào; luật so với AI
"""
import ast
import json
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
sys.path[:0] = [str(HERE), str(ROOT / "demo")]
from floor_graph import FloorGraph  # noqa: E402
from planner import plan  # noqa: E402

if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")


def build_request(scenario_path):
    sc = json.loads(Path(scenario_path).read_text(encoding="utf-8"))
    g = FloorGraph(sc.get("floor", "F2"))
    types = g.data["robot_types"]
    robots = [{"robotId": r["id"], "type": r["type"], "node": r["home"], "battery": 100,
               "speed": types[r["type"]]["speed"], "slots": types[r["type"]]["slots"]} for r in g.data["robots"]]
    now = "2026-11-21T08:00:00Z"

    def at(sec):
        return f"2026-11-21T{8 + sec // 3600:02d}:{sec // 60 % 60:02d}:{sec % 60:02d}Z"

    tasks = [{"taskId": t["taskId"], "priority": t["priority"], "robotType": t["robotType"], "pickup": t["pickup"],
              "dropoff": t["dropoff"], "slots": t.get("slots", 1), "createdAt": now, "deadline": at(t["deadlineInSec"])}
             for t in sc["tasks"]]
    nodes = [r["node"] for r in robots] + [t["pickup"] for t in tasks] + [t["dropoff"] for t in tasks]
    return sc, {"now": now, "config": sc.get("config", {}), "robots": robots, "tasks": tasks,
                "distances": g.distances(nodes)}


def imports_of(path):
    tree = ast.parse(Path(path).read_text(encoding="utf-8"))
    mods = set()
    for n in ast.walk(tree):
        if isinstance(n, ast.Import):
            mods |= {a.name.split(".")[0] for a in n.names}
        elif isinstance(n, ast.ImportFrom) and n.module:
            mods.add(n.module.split(".")[0])
    return sorted(mods - {"__future__"})


def main():
    scenario = sys.argv[1] if len(sys.argv) > 1 else ROOT / "demo" / "scenarios" / "f2_gio_cao_diem.json"
    sc, req = build_request(scenario)

    print("=" * 78)
    print("A. BẰNG CHỨNG BỘ NÃO AI LÀ CODE")
    print("=" * 78)
    src = HERE / "planner.py"
    lines = src.read_text(encoding="utf-8").count("\n")
    print(f"  File      : {src}  ({lines} dòng Python)")
    print(f"  Thư viện  : {', '.join(imports_of(src))}")
    print("              → toàn thư viện CÓ SẴN của Python. Không có requests/openai/torch/tensorflow,")
    print("                không gọi Internet, không có mô hình train sẵn.")
    t0 = time.perf_counter()
    out1 = plan(req)
    ms = (time.perf_counter() - t0) * 1000
    out2 = plan(req)
    same = out1["plans"] == out2["plans"]
    print(f"  Chạy 2 lần: {'GIỐNG HỆT nhau' if same else 'KHÁC nhau (!)'} → cùng đầu vào luôn cùng đáp án (không ngẫu nhiên)")
    print(f"  Thời gian : {ms:.1f} ms, đã thử {out1['meta']['tried']} phương án nhét việc")

    print()
    print("=" * 78)
    print(f"B. NHẬT KÝ AI SUY NGHĨ — {sc.get('name')}")
    print("=" * 78)
    for t in req["tasks"]:
        print(f"  việc {t['taskId']}: {t['priority']:<6} {t['robotType']:<11} {t['pickup']} → {t['dropoff']:<10}"
              f" hạn {t['deadline'][11:19]} (bắt đầu 08:00:00)")
    last = None
    for e in out1["trace"]:
        if e["step"] != last:
            print(f"\n  [{e['step']}]")
            last = e["step"]
        print(f"    • {e['text']}")

    print()
    print("=" * 78)
    print("C. KẾT QUẢ")
    print("=" * 78)
    for p in out1["plans"]:
        print(f"  {p['robotId']:<7} {p['distanceM']:>6} m, xong sau {p['finishInSec']:>4} s: "
              + " → ".join(f"{s['nodeId'].replace('F2-', '')}({'lấy' if s['action'] == 'PICKUP' else 'giao'} "
                           f"{','.join(s['taskIds'])})" for s in p["stops"]))
    s = out1["score"]
    print(f"\n  LUẬT (bản nháp): điểm phạt {s['draft']:<6} | việc trễ {s['draftLateTasks']} | "
          f"{s.get('draftDistanceM', '?')} m")
    print(f"  AI (sửa dần)   : điểm phạt {s['final']:<6} | việc trễ {s['lateTasks']} | "
          f"{s.get('finalDistanceM', '?')} m   → giảm {s['improvementPct']}%")


if __name__ == "__main__":
    main()
