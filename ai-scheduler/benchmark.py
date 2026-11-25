"""benchmark — đo AI Scheduler so với luật, trên NHIỀU kịch bản ngẫu nhiên (không chọn kịch bản đẹp).

    python ai-scheduler/benchmark.py            # 200 kịch bản, in bảng tổng hợp
    python ai-scheduler/benchmark.py 500 --csv  # 500 kịch bản, ghi thêm ai-scheduler/benchmark_result.csv

So 3 cách chia việc trên CÙNG một bộ việc + robot + bản đồ F2 thật:
  BM04  luật dự phòng đã nộp: việc gấp trước, mỗi chuyến 1 việc, robot nào rảnh sớm nhất nhận
  LUẬT  bước 2 của AI (chia nháp): gấp trước, nhét vào chỗ tăng điểm phạt ít nhất, có gom việc
  AI    bước 2 + bước 3 (sửa dần: chuyển / đổi chéo)
Thông số giống backend: trọng số gấp 6 / cao 3 / thường 1, 200 m = 1 điểm, tốc độ thực 0.72, đứng 6 s mỗi điểm.
Kịch bản có seed cố định → chạy lại ra đúng số cũ (hội đồng kiểm tra được).
"""
import csv
import random
import statistics
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path[:0] = [str(HERE), str(HERE.parent / "demo")]
from floor_graph import FloorGraph  # noqa: E402
from planner import Plan, Planner  # noqa: E402

if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")

NOW = "2026-11-21T08:00:00Z"
CONFIG = {"priorityWeight": {"URGENT": 6, "HIGH": 3, "NORMAL": 1}, "metersPerPoint": 200,
          "serviceSeconds": 6, "speedFactor": 0.72, "timeLimitSeconds": 1.0,
          "batteryPerMeter": 0.01}   # đo chất lượng CHIA VIỆC nên nới giới hạn pin (giả sử robot đã sạc giữa các chuyến)
DEPTS = ["F2-WARD-A", "F2-WARD-B", "F2-PEDS", "F2-ICU", "F2-CARD", "F2-LAB", "F2-SURG", "F2-RESP"]
ROBOTS = [{"robotId": "MED-01", "type": "MED_CART", "node": "F2-CS-PH", "speed": 1.0, "slots": 4},
          {"robotId": "MED-02", "type": "MED_CART", "node": "F2-CS-PH", "speed": 1.0, "slots": 4},
          {"robotId": "SUP-01", "type": "SUPPLY_CART", "node": "F2-CS-ST", "speed": 0.7, "slots": 3}]


def scenario(seed, graph):
    r = random.Random(seed)
    tasks = []
    for i in range(r.randint(6, 12)):
        sup = r.random() < 0.3
        sec = r.randint(180, 720)
        tasks.append({"taskId": f"T{i:02d}", "robotType": "SUPPLY_CART" if sup else "MED_CART",
                      "priority": r.choices(["URGENT", "HIGH", "NORMAL"], [25, 15, 60])[0],
                      "pickup": "F2-STORE" if sup else "F2-PHARM", "dropoff": r.choice(DEPTS),
                      "createdAt": NOW, "deadline": f"2026-11-21T08:{sec // 60:02d}:{sec % 60:02d}Z"})
    nodes = [x["node"] for x in ROBOTS] + [t["pickup"] for t in tasks] + [t["dropoff"] for t in tasks]
    return {"now": NOW, "config": CONFIG, "robots": ROBOTS, "tasks": tasks, "distances": graph.distances(nodes)}


def bm04(p: Planner) -> Plan:
    """Mỗi chuyến 1 việc: việc gấp trước; giao cho robot cùng loại xong sớm nhất; chuyến nối tiếp nhau."""
    plan = Plan(routes={b: [] for b in p.bots})
    for t in sorted(p.tasks.values(), key=lambda t: (-t.weight, t.deadline_s, t.id)):
        cands = [b for b in plan.routes if p.bots[b].type == t.robot_type]
        finish = lambda b: (p.evaluate(b, plan.routes[b]).finish_s if plan.routes[b] else 0)  # noqa: E731
        best = min(cands, key=lambda b: (finish(b), b))
        plan.routes[best] = plan.routes[best] + [(t.id, "P"), (t.id, "D")]
    return plan


def measure(p: Planner, plan: Plan) -> dict:
    late, late_min, eta, meters, wait = 0, 0.0, [], 0.0, 0.0
    for b, e in p.plan_evals(plan)[1].items():      # tính cả xếp hàng ở điểm lấy hàng
        meters += e.meters
        wait += e.wait_s
        for tid, t in e.deliver_s.items():
            eta.append(t / 60)
            over = t - p.tasks[tid].deadline_s
            if over > 0:
                late += 1
                late_min += over / 60
    return {"late": late, "lateMin": late_min, "avgEta": statistics.mean(eta) if eta else 0, "meters": meters,
            "cost": p.total_cost(plan), "queueSec": wait, "missed": len(p.tasks) - len(eta)}


def main():
    n = int(next((a for a in sys.argv[1:] if a.isdigit()), 200))
    graph = FloorGraph("F2")
    rows = []
    for seed in range(n):
        req = scenario(seed, graph)
        p = Planner(req)
        draft = p.step2_greedy(p.step1_filter())
        final, _ = p.step3_improve(draft.copy())
        row = {"seed": seed, "tasks": len(req["tasks"])}
        for name, plan in (("BM04", bm04(p)), ("LUAT", draft), ("AI", final)):
            row.update({f"{name}_{k}": v for k, v in measure(p, plan).items()})
        rows.append(row)

    def avg(key):
        return statistics.mean(r[key] for r in rows)

    print(f"\n{n} kịch bản ngẫu nhiên tầng F2 · 3 robot · 6–12 việc mỗi kịch bản (seed 0..{n - 1})\n")
    print(f"{'':28}{'BM04 (1 việc/chuyến)':>22}{'LUẬT (chia nháp)':>18}{'AI (sửa dần)':>15}")
    for key, label, fmt in (("late", "Việc trễ / kịch bản", "{:.2f}"), ("lateMin", "Tổng phút trễ / kịch bản", "{:.1f}"),
                            ("avgEta", "Thời gian giao TB (phút)", "{:.2f}"), ("meters", "Tổng mét chạy", "{:.0f}"), ("queueSec", "Giây xếp hàng ở quầy", "{:.0f}"),
                            ("cost", "Điểm phạt", "{:.2f}")):
        print(f"{label:28}" + "".join(f"{fmt.format(avg(f'{m}_{key}')):>{w}}" for m, w in (("BM04", 22), ("LUAT", 18), ("AI", 15))))
    better = sum(r["AI_cost"] < r["LUAT_cost"] - 1e-9 for r in rows)
    worse = sum(r["AI_cost"] > r["LUAT_cost"] + 1e-9 for r in rows)
    print(f"{'Việc không giao được':28}" + "".join(f"{avg(f'{m}_missed'):>{w}.2f}" for m, w in (("BM04", 22), ("LUAT", 18), ("AI", 15))))
    print(f"\nAI tốt hơn LUẬT ở {better}/{n} kịch bản, bằng ở {n - better - worse}, tệ hơn ở {worse}.")
    print(f"AI so với BM04: việc trễ giảm {100 * (1 - avg('AI_late') / max(avg('BM04_late'), 1e-9)):.0f}%, "
          f"thời gian giao TB giảm {100 * (1 - avg('AI_avgEta') / avg('BM04_avgEta')):.0f}%, "
          f"quãng đường giảm {100 * (1 - avg('AI_meters') / avg('BM04_meters')):.0f}%.")
    if "--csv" in sys.argv:
        out = HERE / "benchmark_result.csv"
        with out.open("w", newline="", encoding="utf-8") as f:
            w = csv.DictWriter(f, fieldnames=list(rows[0]))
            w.writeheader()
            w.writerows(rows)
        print(f"Đã ghi {out}")


if __name__ == "__main__":
    main()
