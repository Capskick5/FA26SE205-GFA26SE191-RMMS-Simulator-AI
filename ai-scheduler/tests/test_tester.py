"""Kiểm thử độc lập (QA) cho AI Scheduler — black-box qua plan(req) và HTTP server.

Chạy:  PYTHONIOENCODING=utf-8 python -m unittest discover -s ai-scheduler/tests -p "test_tester.py" -v
Test nào fail vì lỗi thật của code thì có ghi chú "BUG:" ngay trên test.
"""
import heapq
import http.client
import json
import math
import random
import socket
import subprocess
import sys
import time
import unittest
from pathlib import Path

HERE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(HERE))
from planner import DEFAULTS, plan  # noqa: E402

NOW = "2026-11-21T08:00:00Z"
COORD = {"A": (0, 0), "B": (10, 0), "C": (20, 0), "D": (20, 15), "E": (0, 25), "F": (35, 30)}


def euclid(coord):
    return {a: {b: round(math.dist(pa, pb), 1) for b, pb in coord.items() if b != a} for a, pa in coord.items()}


DIST = euclid(COORD)


def mk_bot(rid, node="A", type_="MED_CART", battery=100, slots=4, speed=1.0):
    return {"robotId": rid, "type": type_, "node": node, "battery": battery, "slots": slots, "speed": speed}


def mk_task(tid, pickup, dropoff, deadline_sec=600, prio="NORMAL", type_="MED_CART", slots=1, created=NOW):
    t = {"taskId": tid, "priority": prio, "robotType": type_, "pickup": pickup, "dropoff": dropoff,
         "slots": slots, "createdAt": created}
    if deadline_sec is not None:
        m, s = divmod(deadline_sec, 60)
        t["deadline"] = f"2026-11-21T{8 + m // 60:02d}:{m % 60:02d}:{s:02d}Z"
    return t


def mk_req(robots, tasks, dist=DIST, **cfg):
    return {"now": NOW, "robots": robots, "tasks": tasks, "distances": dist, "config": cfg}


def dget(dist, a, b):
    if a == b:
        return 0.0
    v = dist.get(a, {}).get(b)
    if v is None:
        v = dist.get(b, {}).get(a)
    return None if v is None else float(v)


# ------------------------------------------------------------------ kiểm tra bất biến trên đầu ra
def check_invariants(tc, req, out):
    cfg = {**DEFAULTS, **(req.get("config") or {})}
    tasks = {t["taskId"]: t for t in req["tasks"]}
    robots = {r["robotId"]: r for r in req["robots"]}
    dist = req.get("distances") or {}

    # (1) mỗi việc đúng một lần
    picked, delivered = [], []
    for p in out["plans"]:
        for s in p["stops"]:
            (picked if s["action"] == "PICKUP" else delivered).extend(s["taskIds"])
        in_stops = {x for s in p["stops"] for x in s["taskIds"]}
        tc.assertEqual(sorted(p["taskIds"]), sorted(in_stops), "plan.taskIds khác các taskId trong stops")
    unassigned = [u["taskId"] for u in out["unassigned"]]
    tc.assertEqual(len(picked), len(set(picked)), f"có việc bị lấy 2 lần: {picked}")
    tc.assertEqual(len(delivered), len(set(delivered)), f"có việc bị giao 2 lần: {delivered}")
    tc.assertEqual(sorted(picked), sorted(delivered), "việc có PICKUP mà không có DELIVER (hoặc ngược lại)")
    tc.assertFalse(set(picked) & set(unassigned), "việc vừa được giao vừa nằm trong unassigned")
    tc.assertEqual(len(unassigned), len(set(unassigned)))
    tc.assertEqual(sorted(picked + unassigned), sorted(tasks), "có việc bị mất hoặc thừa")

    skipped = {s["robotId"] for s in out["skippedRobots"]}
    for p in out["plans"]:
        rid, r = p["robotId"], robots[p["robotId"]]
        stops = p["stops"]
        # (2) lấy trước giao sau
        pos = {}
        for i, s in enumerate(stops):
            for x in s["taskIds"]:
                pos.setdefault(x, {})[s["action"]] = i
        for x, d in pos.items():
            tc.assertLess(d["PICKUP"], d["DELIVER"], f"{x}: giao trước khi lấy trên {rid}")
            # node đúng
            tc.assertEqual(stops[d["PICKUP"]]["nodeId"], tasks[x]["pickup"])
            tc.assertEqual(stops[d["DELIVER"]]["nodeId"], tasks[x]["dropoff"])
        # (3) không quá số ngăn
        cap, load = max(1, int(r.get("slots", 1))), 0
        for s in stops:
            for x in s["taskIds"]:
                load += int(tasks[x].get("slots", 1)) * (1 if s["action"] == "PICKUP" else -1)
            tc.assertLessEqual(load, cap, f"{rid} chở quá {cap} ngăn")
            tc.assertGreaterEqual(load, 0)
        # (4) đúng loại, đủ pin, không bị loại
        tc.assertNotIn(rid, skipped)
        tc.assertGreaterEqual(float(r.get("battery", 100)), cfg["minBattery"])
        for x in p["taskIds"]:
            tc.assertEqual(tasks[x]["robotType"], r["type"], f"{x} giao nhầm loại robot {rid}")
        # (5) một điểm lấy mỗi chuyến
        if cfg["onePickupPerTrip"]:
            acts = [s["action"] for s in stops]
            first_d = acts.index("DELIVER")
            tc.assertNotIn("PICKUP", acts[first_d:], f"{rid}: lấy hàng sau khi đã giao")
            tc.assertEqual(len({s["nodeId"] for s in stops if s["action"] == "PICKUP"}), 1)
        # seq liên tục, mọi chặng có đường, distanceM khớp, pin đủ
        tc.assertEqual([s["seq"] for s in stops], list(range(1, len(stops) + 1)))
        here, meters = r["node"], 0.0
        for s in stops:
            leg = dget(dist, here, s["nodeId"])
            tc.assertIsNotNone(leg, f"{rid}: không có đường {here} → {s['nodeId']}")
            meters += leg
            here = s["nodeId"]
        tc.assertAlmostEqual(p["distanceM"], meters, delta=0.15)
        tc.assertGreaterEqual(float(r.get("battery", 100)) - meters * cfg["batteryPerMeter"],
                              cfg["reserveBattery"] - 1e-6, f"{rid} không đủ pin cho cả chuyến")

    # robot pin yếu phải nằm trong skippedRobots
    for rid, r in robots.items():
        if float(r.get("battery", 100)) < cfg["minBattery"]:
            tc.assertIn(rid, skipped)


def simulate_queue(req, out):
    """Mô phỏng độc lập hàng chờ ở điểm lấy hàng từ các stops đã trả về. Trả ({robot: wait}, [khoảng lấy hàng])."""
    cfg = {**DEFAULTS, **(req.get("config") or {})}
    svc, sf = float(cfg["serviceSeconds"]), float(cfg["speedFactor"])
    robots = {r["robotId"]: r for r in req["robots"]}
    docks = {t["pickup"] for t in req["tasks"]}
    visits = {}
    for p in out["plans"]:
        r = robots[p["robotId"]]
        here, vs = r["node"], []
        for s in p["stops"]:
            if s["nodeId"] != here or not vs:
                travel = dget(req["distances"], here, s["nodeId"]) / (float(r.get("speed", 1.0)) * sf)
                # quy tắc mới: lần ghé đầu tiên luôn tốn serviceSeconds (kể cả robot đứng sẵn ở đó)
                vs.append({"node": s["nodeId"], "travel": travel, "service": True, "pickup": False})
                here = s["nodeId"]
            if s["action"] == "PICKUP":
                vs[-1]["pickup"] = True
        visits[p["robotId"]] = vs
    free, waits, intervals = {}, {b: 0.0 for b in visits}, []
    heap = [(vs[0]["travel"], b, 0) for b, vs in visits.items() if vs]
    heapq.heapify(heap)
    while heap:
        arrive, b, i = heapq.heappop(heap)
        v = visits[b][i]
        start = arrive
        if cfg["pickupOneAtATime"] and v["pickup"] and v["service"] and v["node"] in docks:
            start = max(arrive, free.get(v["node"], 0.0))
            free[v["node"]] = start + svc
            waits[b] += start - arrive
            intervals.append((v["node"], start, start + svc, b))
        done = start + (svc if v["service"] else 0.0)
        if i + 1 < len(visits[b]):
            heapq.heappush(heap, (done + visits[b][i + 1]["travel"], b, i + 1))
    return waits, intervals


def random_req(rng, n_robots=(1, 4), n_tasks=(0, 8)):
    nodes = list(COORD)
    types = ["MED_CART", "SUPPLY_CART"]
    robots = [mk_bot(f"R{i}", node=rng.choice(nodes), type_=rng.choice(types),
                     battery=rng.choice([15, 25, 40, 60, 100]), slots=rng.choice([1, 2, 3, 4]),
                     speed=rng.choice([0.5, 0.7, 1.0]))
              for i in range(rng.randint(*n_robots))]
    tasks = [mk_task(f"T{i:02d}", rng.choice(nodes), rng.choice(nodes), rng.randint(-60, 400),
                     prio=rng.choice(["URGENT", "NORMAL", "HIGH"]), type_=rng.choice(types),
                     slots=rng.choice([1, 1, 1, 2]))
             for i in range(rng.randint(*n_tasks))]
    cfg = {"serviceSeconds": rng.choice([0, 5, 30]), "onePickupPerTrip": rng.random() < 0.3,
           "pickupOneAtATime": rng.random() < 0.7, "speedFactor": rng.choice([0.72, 1.0])}
    return mk_req(robots, tasks, **cfg)


# ================================================================== 1–5 bất biến
class Invariants(unittest.TestCase):
    def test_bat_bien_tren_300_bo_ngau_nhien(self):
        rng = random.Random(2026)
        for k in range(300):
            req = random_req(rng)
            out = plan(req)
            with self.subTest(case=k):
                check_invariants(self, req, out)

    def test_dung_loai_va_pin_yeu(self):
        req = mk_req([mk_bot("LOW", battery=19.9), mk_bot("SUP", type_="SUPPLY_CART"), mk_bot("MED", node="F")],
                     [mk_task("T1", "A", "B"), mk_task("T2", "A", "C", type_="SUPPLY_CART")])
        out = plan(req)
        check_invariants(self, req, out)
        owner = {x: p["robotId"] for p in out["plans"] for x in p["taskIds"]}
        self.assertEqual(owner, {"T1": "MED", "T2": "SUP"})
        self.assertEqual([s["robotId"] for s in out["skippedRobots"]], ["LOW"])

    def test_pin_bang_dung_nguong_van_duoc_nhan(self):
        out = plan(mk_req([mk_bot("R1", battery=20)], [mk_task("T1", "A", "B")]))
        self.assertEqual(out["skippedRobots"], [])
        self.assertEqual(len(out["plans"]), 1)

    def test_viec_qua_so_ngan_robot(self):
        req = mk_req([mk_bot("R1", slots=2)], [mk_task("BIG", "A", "B", slots=3), mk_task("OK", "A", "B", slots=2)])
        out = plan(req)
        check_invariants(self, req, out)
        self.assertEqual([u["taskId"] for u in out["unassigned"]], ["BIG"])

    def test_mot_diem_lay_moi_chuyen_nhieu_viec(self):
        rng = random.Random(5)
        for k in range(60):
            req = random_req(rng, n_tasks=(2, 8))
            req["config"]["onePickupPerTrip"] = True
            with self.subTest(case=k):
                check_invariants(self, req, plan(req))


# ================================================================== 6 không tệ hơn bản nháp
class NotWorseThanDraft(unittest.TestCase):
    # BUG (nếu fail): bước 3 (a) nhét lại việc chưa giao làm tăng điểm phạt → final > draft.
    def test_final_khong_lon_hon_draft_300_bo(self):
        rng = random.Random(42)
        bad = []
        for k in range(300):
            req = random_req(rng)
            s = plan(req)["score"]
            if s["final"] > s["draft"] + 1e-9:
                bad.append((k, s["draft"], s["final"]))
        self.assertEqual(bad, [], f"{len(bad)} bộ có final > draft, ví dụ {bad[:3]}")

    def test_final_khong_lon_hon_draft_bo_mac_dinh(self):
        rng = random.Random(7)
        for k in range(100):
            req = random_req(rng)
            req["config"] = {}           # đúng cấu hình mặc định
            s = plan(req)["score"]
            with self.subTest(case=k):
                self.assertLessEqual(s["final"], s["draft"] + 1e-9)


# ================================================================== 7 tất định
class Deterministic(unittest.TestCase):
    def test_nho_giong_nhau(self):
        rng = random.Random(11)
        for _ in range(30):
            req = random_req(rng)
            self.assertEqual(plan(json.loads(json.dumps(req)))["plans"], plan(json.loads(json.dumps(req)))["plans"])

    # BUG (nếu fail): bước 3 dừng theo đồng hồ (timeLimitSeconds) → bài lớn chạy 2 lần ra 2 kế hoạch khác nhau.
    def test_bai_lon_giong_nhau(self):
        req = big_req()
        a, b = plan(json.loads(json.dumps(req))), plan(json.loads(json.dumps(req)))
        self.assertEqual(a["plans"], b["plans"],
                         f"timeUp lần 1={a['meta'].get('timeUp')}, lần 2={b['meta'].get('timeUp')}")

    def test_thu_tu_dau_vao_khong_doi_ket_qua(self):
        rng = random.Random(3)
        for _ in range(30):
            req = random_req(rng)
            rev = json.loads(json.dumps(req))
            rev["tasks"].reverse()
            rev["robots"].reverse()
            self.assertEqual(plan(req)["plans"], plan(rev)["plans"])


def big_req(seed=99):
    rng = random.Random(seed)
    nodes = [f"N{i}" for i in range(12)]
    coord = {n: (rng.uniform(0, 80), rng.uniform(0, 40)) for n in nodes}
    dist = euclid(coord)
    robots = [mk_bot(f"R{i}", node=rng.choice(nodes), slots=rng.choice([2, 3, 4])) for i in range(5)]
    tasks = [mk_task(f"T{i:02d}", rng.choice(nodes[:4]), rng.choice(nodes[4:]), rng.randint(60, 900),
                     prio=rng.choice(["URGENT", "NORMAL"])) for i in range(30)]
    return mk_req(robots, tasks, dist=dist, serviceSeconds=10)


# ================================================================== 8 xếp hàng ở điểm lấy hàng
class QueueAtDock(unittest.TestCase):
    def _check(self, req):
        out = plan(req)
        waits, intervals = simulate_queue(req, out)
        # không chồng lấn ở cùng một quầy
        by_node = {}
        for node, s, e, b in intervals:
            by_node.setdefault(node, []).append((s, e, b))
        for node, iv in by_node.items():
            iv.sort()
            for (s1, e1, b1), (s2, e2, b2) in zip(iv, iv[1:]):
                self.assertGreaterEqual(s2, e1 - 1e-6, f"{b1} và {b2} lấy hàng chồng nhau ở {node}")
        # queueWaitSec từng plan và tổng khớp mô phỏng độc lập
        for p in out["plans"]:
            self.assertAlmostEqual(p["queueWaitSec"], waits[p["robotId"]], delta=0.51)
        self.assertAlmostEqual(sum(p["queueWaitSec"] for p in out["plans"]), sum(waits.values()),
                               delta=0.51 * max(1, len(out["plans"])))
        self.assertAlmostEqual(out["score"]["finalQueueWaitSec"], sum(waits.values()), delta=0.51)
        return out

    def test_hang_cho_khop_mo_phong_ngau_nhien(self):
        rng = random.Random(8)
        for k in range(150):
            req = random_req(rng, n_tasks=(2, 8))
            req["config"]["pickupOneAtATime"] = True
            with self.subTest(case=k):
                self._check(req)

    def test_tat_luat_thi_khong_cho(self):
        rng = random.Random(9)
        for k in range(50):
            req = random_req(rng, n_tasks=(2, 8))
            req["config"]["pickupOneAtATime"] = False
            out = plan(req)
            with self.subTest(case=k):
                self.assertTrue(all(p["queueWaitSec"] == 0 for p in out["plans"]))
                self.assertEqual(out["score"]["finalQueueWaitSec"], 0)

    def test_hai_robot_bat_buoc_cung_quay(self):
        # mỗi robot 1 ngăn, hạn gấp → buộc dùng 2 robot cùng lấy ở B; cả 2 xuất phát từ A (cách B 10 m)
        req = mk_req([mk_bot("R1", slots=1), mk_bot("R2", slots=1)],
                     [mk_task("T1", "B", "C", 80), mk_task("T2", "B", "C", 80)], serviceSeconds=30)
        out = self._check(req)
        self.assertEqual(len(out["plans"]), 2)
        self.assertEqual(out["score"]["finalQueueWaitSec"], 30)

    # (từng là BUG, dev đã sửa): robot đứng sẵn ở quầy vẫn phải tốn serviceSeconds và xếp hàng.
    def test_hai_robot_dau_san_o_quay(self):
        req = mk_req([mk_bot("R1", node="B", slots=1), mk_bot("R2", node="B", slots=1)],
                     [mk_task("T1", "B", "C", 60), mk_task("T2", "B", "C", 60)], serviceSeconds=30)
        out = plan(req)
        self.assertEqual(len(out["plans"]), 2)
        eta = sorted(e["etaSec"] for e in out["explain"])
        # quầy phục vụ 1 robot một lúc, lấy hàng mất 30 s → 2 lần lấy phải nối tiếp nhau
        self.assertGreater(out["score"]["finalQueueWaitSec"] + (eta[1] - eta[0]), 0,
                           f"2 robot lấy hàng cùng lúc ở B, eta={eta}")


# ================================================================== 9 trường hợp biên
class EdgeCases(unittest.TestCase):
    def test_khong_co_viec(self):
        out = plan(mk_req([mk_bot("R1")], []))
        self.assertEqual((out["plans"], out["unassigned"]), ([], []))
        self.assertEqual(out["score"]["final"], 0)

    def test_khong_co_robot(self):
        req = mk_req([], [mk_task("T1", "A", "B"), mk_task("T2", "C", "D")])
        out = plan(req)
        check_invariants(self, req, out)
        self.assertEqual(sorted(u["taskId"] for u in out["unassigned"]), ["T1", "T2"])

    def test_req_rong_hoan_toan(self):
        out = plan({})
        self.assertEqual(out["plans"], [])

    def test_lay_va_giao_cung_mot_diem(self):
        req = mk_req([mk_bot("R1", node="C")], [mk_task("T1", "A", "A")], serviceSeconds=10)
        out = plan(req)
        check_invariants(self, req, out)
        self.assertEqual([s["action"] for s in out["plans"][0]["stops"]], ["PICKUP", "DELIVER"])

    def test_han_da_qua(self):
        req = mk_req([mk_bot("R1")], [mk_task("T1", "A", "B", 0, created="2026-11-21T07:00:00Z"),
                                      {**mk_task("T2", "A", "C"), "deadline": "2026-11-21T07:30:00Z"}])
        out = plan(req)
        check_invariants(self, req, out)
        ex = {e["taskId"]: e for e in out["explain"]}
        self.assertEqual(ex["T2"]["deadlineInSec"], -1800)
        self.assertGreater(ex["T2"]["lateSec"], 1800)
        self.assertEqual(out["score"]["lateTasks"], 2)

    def test_khong_ghi_han(self):
        tasks = [mk_task("U", "A", "B", None, prio="URGENT", created="2026-11-21T07:55:00Z"),
                 mk_task("N", "A", "B", None, prio="NORMAL", created="2026-11-21T07:55:00Z"),
                 {k: v for k, v in mk_task("X", "A", "B", None, prio="URGENT").items() if k != "createdAt"}]
        out = plan(mk_req([mk_bot("R1")], tasks))
        dl = {e["taskId"]: e["deadlineInSec"] for e in out["explain"]}
        self.assertEqual(dl, {"U": 600, "N": 3300, "X": 900})

    def test_han_null_va_chuoi_rong(self):
        tasks = [{**mk_task("A1", "A", "B", None, prio="URGENT"), "deadline": None},
                 {**mk_task("A2", "A", "B", None), "deadline": ""}]
        dl = {e["taskId"]: e["deadlineInSec"] for e in plan(mk_req([mk_bot("R1")], tasks))["explain"]}
        self.assertEqual(dl, {"A1": 900, "A2": 3600})

    def test_uu_tien_la(self):
        req = mk_req([mk_bot("R1")], [mk_task("T1", "A", "B", prio="LOW"), mk_task("T2", "A", "C", prio="urgent"),
                                      {k: v for k, v in mk_task("T3", "A", "D").items() if k != "priority"}])
        out = plan(req)
        check_invariants(self, req, out)
        self.assertEqual(out["unassigned"], [])
        self.assertEqual({e["taskId"]: e["priority"] for e in out["explain"]},
                         {"T1": "LOW", "T2": "URGENT", "T3": "NORMAL"})

    def test_uu_tien_high_duoc_giao_truoc_normal(self):
        req = mk_req([mk_bot("R1", slots=1)],
                     [mk_task("NOR", "A", "D", 40), mk_task("HI", "A", "D", 40, prio="HIGH")], serviceSeconds=5)
        out = plan(req)
        check_invariants(self, req, out)
        eta = {e["taskId"]: e["etaSec"] for e in out["explain"]}
        self.assertLess(eta["HI"], eta["NOR"])

    def test_thieu_khoang_cach_toi_diem_giao(self):
        req = mk_req([mk_bot("R1")], [mk_task("T1", "A", "ZZZ"), mk_task("T2", "A", "B")])
        out = plan(req)
        check_invariants(self, req, out)
        self.assertEqual([u["taskId"] for u in out["unassigned"]], ["T1"])

    def test_thieu_khoang_cach_giua_hai_viec(self):
        # A-B và C-D có đường, nhưng B ↔ C và A ↔ C không có: 1 robot ở A không thể làm cả 2
        dist = {"A": {"B": 10}, "C": {"D": 10}}
        req = mk_req([mk_bot("R1"), mk_bot("R2", node="C")], [mk_task("T1", "A", "B"), mk_task("T2", "C", "D")],
                     dist=dist)
        out = plan(req)
        check_invariants(self, req, out)
        self.assertEqual(out["unassigned"], [])
        owner = {x: p["robotId"] for p in out["plans"] for x in p["taskIds"]}
        self.assertEqual(owner, {"T1": "R1", "T2": "R2"})

    def test_khoang_cach_rong(self):
        req = mk_req([mk_bot("R1")], [mk_task("T1", "A", "B")], dist={})
        out = plan(req)
        check_invariants(self, req, out)
        self.assertEqual(len(out["unassigned"]), 1)

    def test_robot_khong_du_pin_cho_ca_chuyen(self):
        # pin 25%, 0.15%/m, dự trữ 10% → chạy tối đa 100 m; A→F→A ≈ 92 m mỗi lượt
        req = mk_req([mk_bot("R1", battery=25)], [mk_task("T1", "F", "A")])
        out = plan(req)
        check_invariants(self, req, out)

    def test_trung_task_id(self):
        # hành vi mới: taskId trùng → ValueError, không gộp âm thầm
        req = mk_req([mk_bot("R1")], [mk_task("T1", "A", "B"), mk_task("T1", "C", "D")])
        with self.assertRaises(ValueError):
            plan(req)

    def test_trung_robot_id(self):
        req = mk_req([mk_bot("R1"), mk_bot("R1", node="F")], [mk_task("T1", "A", "B")])
        with self.assertRaises(ValueError):
            plan(req)

    def test_speed_khong_duong(self):
        for bad in ({"speed": 0}, {"speed": -1}):
            with self.subTest(bad=bad), self.assertRaises(ValueError):
                plan(mk_req([{**mk_bot("R1"), **bad}], [mk_task("T1", "A", "B")]))
        with self.assertRaises(ValueError):
            plan(mk_req([mk_bot("R1")], [mk_task("T1", "A", "B")], speedFactor=0))

    def test_high_khong_ghi_han_mac_dinh_30_phut(self):
        out = plan(mk_req([mk_bot("R1")], [mk_task("H", "A", "B", None, prio="HIGH")]))
        self.assertEqual(out["explain"][0]["deadlineInSec"], 1800)
        self.assertIn("(cao)", out["explain"][0]["text"])


# ================================================================== 10 hiệu năng
class Performance(unittest.TestCase):
    # (từng là BUG, dev đã sửa): timeLimitSeconds trước đây chỉ kiểm tra giữa các vòng → 2.1–4.8 s.
    def test_5_robot_30_viec_duoi_2_giay(self):
        req = big_req()
        t0 = time.perf_counter()
        out = plan(req)
        el = time.perf_counter() - t0
        check_invariants(self, req, out)
        self.assertLess(el, 2.0, f"mất {el:.2f}s, meta={ {k: out['meta'][k] for k in ('rounds', 'timeUp')} }")

    def test_5_robot_30_viec_nhieu_seed(self):
        worst = 0.0
        for seed in range(5):
            t0 = time.perf_counter()
            plan(big_req(seed))
            worst = max(worst, time.perf_counter() - t0)
        self.assertLess(worst, 2.0, f"chậm nhất {worst:.2f}s")


# ================================================================== 11 HTTP server
def free_port():
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


class HttpServer(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.port = free_port()
        cls.proc = subprocess.Popen([sys.executable, str(HERE / "server.py"), str(cls.port)],
                                    stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        deadline = time.time() + 15
        while time.time() < deadline:
            try:
                if cls._req("GET", "/health")[0] == 200:
                    return
            except OSError:
                time.sleep(0.1)
        cls.proc.kill()
        raise RuntimeError("server không lên")

    @classmethod
    def tearDownClass(cls):
        cls.proc.terminate()
        try:
            cls.proc.wait(5)
        except subprocess.TimeoutExpired:
            cls.proc.kill()

    @classmethod
    def _req(cls, method, path, body=None, headers=None):
        c = http.client.HTTPConnection("127.0.0.1", cls.port, timeout=10)
        try:
            c.request(method, path, body=body, headers=headers or {})
            r = c.getresponse()
            data = r.read()
            return r.status, (json.loads(data) if data else None)
        finally:
            c.close()

    def _valid(self):
        return mk_req([mk_bot("R1"), mk_bot("R2", node="F")],
                      [mk_task("T1", "A", "B"), mk_task("T2", "C", "D", prio="URGENT"), mk_task("T3", "E", "F")])

    def test_health(self):
        code, body = self._req("GET", "/health")
        self.assertEqual((code, body["status"]), (200, "ok"))

    def test_plan_hop_le(self):
        req = self._valid()
        code, out = self._req("POST", "/plan", json.dumps(req), {"Content-Type": "application/json"})
        self.assertEqual(code, 200)
        check_invariants(self, req, out)

    def test_json_sai(self):
        code, body = self._req("POST", "/plan", b"{not json", {"Content-Type": "application/json"})
        self.assertEqual(code, 400)
        self.assertIn("error", body)

    def test_thieu_truong_bat_buoc(self):
        for missing in ("pickup", "taskId", "robotType", "dropoff"):
            req = self._valid()
            del req["tasks"][0][missing]
            code, body = self._req("POST", "/plan", json.dumps(req))
            with self.subTest(missing=missing):
                self.assertEqual(code, 400, body)
        req = self._valid()
        del req["robots"][0]["node"]
        self.assertEqual(self._req("POST", "/plan", json.dumps(req))[0], 400)

    # (từng là BUG, dev đã sửa): dữ liệu sai kiểu/định dạng là lỗi của bên gọi → 400, không phải 500.
    def test_du_lieu_sai_dinh_dang_tra_400(self):
        bad = {
            "deadline_sai": (lambda r: r["tasks"][0].__setitem__("deadline", "ngay mai")),
            "body_la_mang": None,
        }
        for name, mutate in bad.items():
            if mutate is None:
                body = "[1, 2]"
            else:
                req = self._valid()
                mutate(req)
                body = json.dumps(req)
            code, _ = self._req("POST", "/plan", body)
            with self.subTest(case=name):
                self.assertEqual(code, 400)

    def test_trung_id_va_speed_0_tra_400(self):
        cases = {}
        r = self._valid(); r["tasks"][1]["taskId"] = "T1"; cases["trung_taskId"] = r
        r = self._valid(); r["robots"][1]["robotId"] = "R1"; cases["trung_robotId"] = r
        r = self._valid(); r["robots"][0]["speed"] = 0; cases["speed_0"] = r
        r = self._valid(); r["config"]["speedFactor"] = 0; cases["speedFactor_0"] = r
        for name, req in cases.items():
            code, body = self._req("POST", "/plan", json.dumps(req))
            with self.subTest(case=name):
                self.assertEqual(code, 400, body)
                self.assertIn("error", body)
        self.assertEqual(self._req("GET", "/health")[0], 200)   # server vẫn sống sau các lỗi

    def test_chunked(self):
        req = self._valid()
        raw = json.dumps(req).encode("utf-8")
        c = http.client.HTTPConnection("127.0.0.1", self.port, timeout=10)
        try:
            c.putrequest("POST", "/plan")
            c.putheader("Content-Type", "application/json")
            c.putheader("Transfer-Encoding", "chunked")
            c.endheaders()
            for i in range(0, len(raw), 37):
                chunk = raw[i:i + 37]
                c.send(f"{len(chunk):x}\r\n".encode() + chunk + b"\r\n")
            c.send(b"0\r\n\r\n")
            r = c.getresponse()
            out = json.loads(r.read())
        finally:
            c.close()
        self.assertEqual(r.status, 200)
        n = sum(len(p["taskIds"]) for p in out["plans"]) + len(out["unassigned"])
        self.assertEqual(n, 3)
        check_invariants(self, req, out)

    def test_duong_dan_la(self):
        self.assertEqual(self._req("GET", "/xyz")[0], 404)
        self.assertEqual(self._req("POST", "/xyz", "{}")[0], 404)


if __name__ == "__main__":
    unittest.main()
