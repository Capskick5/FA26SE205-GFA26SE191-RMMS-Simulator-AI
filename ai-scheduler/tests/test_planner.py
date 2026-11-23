"""Kiểm tra AI Scheduler.   Chạy:  python -m unittest discover -s ai-scheduler/tests -v"""
import random
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from planner import plan  # noqa: E402

NOW = "2026-11-21T08:00:00Z"
# Hành lang thẳng: A —10m— B —10m— C —10m— D   (khoảng cách = |vị trí a − vị trí b|)
POS = {"A": 0, "B": 10, "C": 20, "D": 30, "E": 40}
DIST = {a: {b: abs(pa - pb) for b, pb in POS.items() if b != a} for a, pa in POS.items()}


def bot(rid, node="A", type_="MED_CART", battery=100, slots=4, speed=1.0):
    return {"robotId": rid, "type": type_, "node": node, "battery": battery, "slots": slots, "speed": speed}


def task(tid, pickup, dropoff, sec, prio="NORMAL", type_="MED_CART"):
    return {"taskId": tid, "priority": prio, "robotType": type_, "pickup": pickup, "dropoff": dropoff,
            "createdAt": NOW, "deadline": f"2026-11-21T08:{sec // 60:02d}:{sec % 60:02d}Z"}


def run(robots, tasks, **cfg):
    return plan({"now": NOW, "robots": robots, "tasks": tasks, "distances": DIST,
                 "config": {"serviceSeconds": 0, **cfg}})


def owner(out, tid):
    return next(p["robotId"] for p in out["plans"] if tid in p["taskIds"])


class Rules(unittest.TestCase):
    def test_dung_loai_robot(self):
        out = run([bot("MED-01"), bot("SUP-01", type_="SUPPLY_CART")],
                  [task("T1", "A", "B", 300, type_="SUPPLY_CART")])
        self.assertEqual(owner(out, "T1"), "SUP-01")

    def test_khong_co_robot_loai_do(self):
        out = run([bot("MED-01")], [task("T1", "A", "B", 300, type_="WASTE_CART")])
        self.assertEqual([u["taskId"] for u in out["unassigned"]], ["T1"])

    def test_pin_yeu_bi_loai(self):
        out = run([bot("MED-01", battery=15), bot("MED-02", node="E")], [task("T1", "A", "B", 300)])
        self.assertEqual(owner(out, "T1"), "MED-02")
        self.assertEqual(out["skippedRobots"][0]["robotId"], "MED-01")

    def test_lay_truoc_giao_sau(self):
        out = run([bot("MED-01", node="D")], [task("T1", "B", "A", 300)])
        actions = [s["action"] for s in out["plans"][0]["stops"]]
        self.assertEqual(actions, ["PICKUP", "DELIVER"])

    def test_khong_qua_so_ngan(self):
        # 1 ngăn, 2 việc cùng lấy ở A → không được ôm cả 2 cùng lúc
        out = run([bot("MED-01", slots=1)], [task("T1", "A", "C", 600), task("T2", "A", "D", 600)])
        load, worst = 0, 0
        for s in out["plans"][0]["stops"]:
            load += len(s["taskIds"]) if s["action"] == "PICKUP" else -len(s["taskIds"])
            worst = max(worst, load)
        self.assertLessEqual(worst, 1)

    def test_gom_don_cung_duong(self):
        # 2 việc cùng lấy ở A, giao ở C và D, hạn rộng → 1 robot gom cả 2 (đỡ chạy)
        out = run([bot("MED-01"), bot("MED-02")], [task("T1", "A", "C", 600), task("T2", "A", "D", 600)])
        self.assertEqual(owner(out, "T1"), owner(out, "T2"))
        self.assertEqual(sorted(out["plans"][0]["stops"][0]["taskIds"]), ["T1", "T2"])   # lấy cùng lúc → 1 điểm dừng

    def test_mot_diem_lay_moi_chuyen(self):
        # luật backend: 1 chuyến chỉ lấy ở 1 điểm, lấy hết rồi mới giao
        out = run([bot("MED-01")], [task("T1", "A", "C", 600), task("T2", "B", "D", 600)], onePickupPerTrip=True)
        self.assertEqual(len(out["plans"]), 1)
        self.assertEqual(len(out["unassigned"]), 1)
        stops = out["plans"][0]["stops"]
        self.assertEqual([s["action"] for s in stops], ["PICKUP", "DELIVER"])

    def test_viec_gap_duoc_giao_truoc(self):
        out = run([bot("MED-01", slots=4)],
                  [task("THUONG", "A", "D", 40), task("GAP", "A", "D", 40, prio="URGENT")], serviceSeconds=5)
        eta = {e["taskId"]: e["etaSec"] for e in out["explain"]}
        self.assertLessEqual(eta["GAP"], eta["THUONG"])


class Improvement(unittest.TestCase):
    def test_ai_khong_bao_gio_te_hon_ban_nhap(self):
        rng = random.Random(7)
        for _ in range(150):
            robots = [bot(f"R{i}", node=rng.choice(list(POS)), slots=rng.choice([1, 2, 4]))
                      for i in range(rng.randint(1, 3))]
            tasks = [task(f"T{i}", rng.choice(list(POS)), rng.choice(list(POS)), rng.randint(10, 120),
                          prio=rng.choice(["URGENT", "NORMAL"])) for i in range(rng.randint(1, 7))]
            s = run(robots, tasks, serviceSeconds=3)["score"]
            self.assertLessEqual(s["final"], s["draft"] + 1e-9)

    def test_ket_qua_on_dinh(self):
        robots = [bot("R1"), bot("R2", node="E")]
        tasks = [task(f"T{i}", "A", list(POS)[i % 5], 60 + i * 7) for i in range(6)]
        self.assertEqual(run(robots, tasks)["plans"], run(robots, tasks)["plans"])

    def test_kich_ban_demo_f2_ai_het_tre(self):
        """Kịch bản trong demo/: bản nháp trễ 2 việc, AI sửa còn 0."""
        root = Path(__file__).resolve().parents[2]
        sys.path.insert(0, str(root / "demo"))
        import json
        from floor_graph import FloorGraph
        sc = json.loads((root / "demo" / "scenarios" / "f2_gio_cao_diem.json").read_text(encoding="utf-8"))
        g = FloorGraph("F2")
        robots = [bot("MED-01", "F2-CS-PH"), bot("MED-02", "F2-CS-PH"),
                  bot("SUP-01", "F2-CS-ST", type_="SUPPLY_CART", slots=3, speed=0.7)]
        tasks = [task(t["taskId"], t["pickup"], t["dropoff"], t["deadlineInSec"], t["priority"], t["robotType"])
                 for t in sc["tasks"]]
        nodes = [r["node"] for r in robots] + [t["pickup"] for t in tasks] + [t["dropoff"] for t in tasks]
        out = plan({"now": NOW, "robots": robots, "tasks": tasks, "distances": g.distances(nodes),
                    "config": sc["config"]})
        self.assertGreater(out["score"]["draftLateTasks"], 0)
        self.assertEqual(out["score"]["lateTasks"], 0)
        self.assertEqual(out["unassigned"], [])


if __name__ == "__main__":
    unittest.main()
