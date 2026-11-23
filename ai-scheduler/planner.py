"""planner — "bộ não" chia việc của RMMS: Chia nháp rồi sửa dần (greedy insertion + local search).

Đầu vào : danh sách nhiệm vụ (lấy ở đâu, giao ở đâu, gấp hay thường, hạn chót)
          + danh sách robot (đang ở đâu, loại gì, pin bao nhiêu)
          + bảng khoảng cách giữa các điểm (backend tính bằng Dijkstra trên bản đồ)
Đầu ra  : mỗi robot chạy những điểm nào, theo thứ tự nào, kèm lý do.

4 bước:
  1. LỌC      robot rảnh, đúng loại hàng, pin đủ
  2. NHÁP     xếp việc gấp trước; lần lượt nhét từng việc vào chỗ làm điểm phạt tăng ít nhất
  3. SỬA DẦN  thử CHUYỂN 1 việc sang robot khác, thử ĐỔI CHÉO 2 việc; điểm phạt giảm thì giữ
  4. TRẢ      kế hoạch + điểm phạt bản nháp so với bản đã sửa

Điểm phạt của một kế hoạch = Σ (phút trễ × hệ số ưu tiên) + (tổng mét chạy ÷ 100)
Chỉ dùng thư viện chuẩn của Python — không mạng, không API ngoài, không train.
"""
from __future__ import annotations

import math
import time
from dataclasses import dataclass, field
from datetime import datetime, timedelta, timezone

# ------------------------------------------------------------------ cấu hình mặc định (backend gửi đè được)
DEFAULTS = {
    "priorityWeight": {"URGENT": 3.0, "HIGH": 2.0, "NORMAL": 1.0},  # 1 phút trễ việc gấp = 3 phút trễ việc thường
    "onePickupPerTrip": False,                          # True = mỗi chuyến chỉ lấy hàng ở 1 điểm, lấy hết rồi mới giao
                                                        #   (đúng mô hình dữ liệu backend hiện tại)
    "metersPerPoint": 100.0,                            # chạy 100 m = 1 điểm phạt
    "minBattery": 20.0,                                 # dưới mức này robot không nhận việc
    "reserveBattery": 10.0,                             # chạy xong chuyến vẫn phải còn ít nhất chừng này
    "batteryPerMeter": 0.15,                            # % pin mỗi mét (khớp robot Webots)
    "serviceSeconds": 30.0,                             # đứng ở mỗi điểm dừng bao lâu (Webots: 5)
    "speedFactor": 1.0,                                 # tốc độ thực tế / tốc độ ghi trên giấy (đo được: Webots ≈ 0.72
                                                        #   vì robot phải tăng tốc, quay đầu, né nhau)
    "defaultDeadlineMin": {"URGENT": 15.0, "NORMAL": 60.0},  # việc không ghi hạn thì lấy hạn này tính từ lúc tạo
    "timeLimitSeconds": 1.0,                            # bước 3 chạy tối đa bao lâu
}
EPS = 1e-9


# ------------------------------------------------------------------ dữ liệu
@dataclass
class Task:
    id: str
    priority: str
    robot_type: str
    pickup: str
    dropoff: str
    slots: int
    deadline_s: float          # còn bao nhiêu giây nữa là tới hạn (âm = đã trễ rồi)
    waited_s: float            # đã chờ bao lâu
    weight: float              # hệ số ưu tiên


@dataclass
class Bot:
    id: str
    type: str
    node: str
    battery: float
    speed: float               # m/s
    capacity: int              # số ngăn


@dataclass
class Plan:
    """Kế hoạch: mỗi robot một dãy sự kiện (taskId, 'P' lấy | 'D' giao)."""
    routes: dict[str, list[tuple[str, str]]] = field(default_factory=dict)
    unassigned: dict[str, str] = field(default_factory=dict)   # taskId → lý do

    def copy(self) -> "Plan":
        return Plan({r: list(ev) for r, ev in self.routes.items()}, dict(self.unassigned))


@dataclass
class RouteEval:
    cost: float
    meters: float
    finish_s: float
    deliver_s: dict[str, float]


class Planner:
    def __init__(self, req: dict):
        self.cfg = {**DEFAULTS, **(req.get("config") or {})}
        self.dist = req.get("distances") or {}
        now = _parse_time(req.get("now")) or datetime.now(timezone.utc)
        self.tasks = {t["taskId"]: self._task(t, now) for t in req.get("tasks", [])}
        self.bots = {r["robotId"]: self._bot(r) for r in req.get("robots", [])}
        self.skipped_bots: dict[str, str] = {}
        self._eval_cache: dict[tuple, RouteEval | None] = {}
        self.trace: list[dict] = []        # nhật ký "AI nghĩ gì" từng bước — chỉ để xem, không ảnh hưởng kết quả
        self.tries = 0                     # đã thử bao nhiêu phương án

    def _log(self, step: str, text: str):
        self.trace.append({"step": step, "text": text})

    def _route_str(self, events) -> str:
        return " → ".join(f"{'lấy' if k == 'P' else 'giao'} {t}@"
                          f"{(self.tasks[t].pickup if k == 'P' else self.tasks[t].dropoff).replace('F2-', '')}"
                          for t, k in events) or "(rảnh)"

    # ---------- đọc đầu vào
    def _task(self, t: dict, now: datetime) -> Task:
        prio = str(t.get("priority", "NORMAL")).upper()
        created = _parse_time(t.get("createdAt")) or now
        deadline = _parse_time(t.get("deadline"))
        if deadline is None:
            minutes = self.cfg["defaultDeadlineMin"].get(prio, 60.0)
            deadline = created + timedelta(minutes=minutes)
        return Task(
            id=t["taskId"], priority=prio, robot_type=t["robotType"],
            pickup=t["pickup"], dropoff=t["dropoff"], slots=int(t.get("slots", 1)),
            deadline_s=(deadline - now).total_seconds(), waited_s=(now - created).total_seconds(),
            weight=float(self.cfg["priorityWeight"].get(prio, 1.0)),
        )

    def _bot(self, r: dict) -> Bot:
        return Bot(id=r["robotId"], type=r["type"], node=r["node"], battery=float(r.get("battery", 100)),
                   speed=float(r.get("speed", 1.0)), capacity=max(1, int(r.get("slots", 1))))

    def d(self, a: str, b: str) -> float:
        if a == b:
            return 0.0
        v = self.dist.get(a, {}).get(b)
        if v is None:
            v = self.dist.get(b, {}).get(a)
        return math.inf if v is None else float(v)

    # ---------- chấm điểm một chuyến của một robot
    def evaluate(self, bot_id: str, events: list[tuple[str, str]]) -> RouteEval | None:
        """Mô phỏng robot chạy dãy sự kiện. Trả None nếu không làm được (quá ngăn, hết pin, không có đường)."""
        key = (bot_id, tuple(events))
        if key in self._eval_cache:
            return self._eval_cache[key]
        bot, cfg = self.bots[bot_id], self.cfg
        t, meters, load, here = 0.0, 0.0, 0, bot.node
        deliver: dict[str, float] = {}
        result: RouteEval | None = None
        one_pickup, first_pickup, delivered_any = cfg["onePickupPerTrip"], None, False
        for task_id, kind in events:
            task = self.tasks[task_id]
            node = task.pickup if kind == "P" else task.dropoff
            if one_pickup:                 # luật backend: lấy hết ở 1 điểm rồi mới đi giao
                if kind == "P" and (delivered_any or (first_pickup and node != first_pickup)):
                    break
                first_pickup = first_pickup or (node if kind == "P" else None)
                delivered_any = delivered_any or kind == "D"
            step = self.d(here, node)
            if math.isinf(step):
                break
            if node != here:               # cùng một chỗ thì không tốn thêm thời gian đứng
                t += step / (bot.speed * cfg["speedFactor"]) + cfg["serviceSeconds"]
            meters += step
            here = node
            load += task.slots if kind == "P" else -task.slots
            if load > bot.capacity:
                break
            if kind == "D":
                deliver[task_id] = t
        else:
            if bot.battery - meters * cfg["batteryPerMeter"] >= cfg["reserveBattery"]:
                late_pts = sum(max(0.0, deliver[k] - self.tasks[k].deadline_s) / 60.0 * self.tasks[k].weight
                               for k in deliver)
                result = RouteEval(late_pts + meters / cfg["metersPerPoint"], meters, t, deliver)
        self._eval_cache[key] = result
        return result

    def route_cost(self, bot_id: str, events: list) -> float:
        if not events:
            return 0.0
        e = self.evaluate(bot_id, events)
        return math.inf if e is None else e.cost

    def total_cost(self, plan: Plan) -> float:
        return sum(self.route_cost(r, ev) for r, ev in plan.routes.items())

    # ---------- tìm chỗ nhét tốt nhất cho một việc
    def best_insertion(self, plan: Plan, task_id: str, only: str | None = None):
        """Thử mọi robot hợp lệ × mọi vị trí lấy × mọi vị trí giao. Trả (tăng_điểm, robot, dãy_mới) tốt nhất."""
        task = self.tasks[task_id]
        best = (math.inf, None, None)
        for bot_id in sorted(plan.routes):
            if only and bot_id != only:
                continue
            if self.bots[bot_id].type != task.robot_type:
                continue
            ev = plan.routes[bot_id]
            base = self.route_cost(bot_id, ev)
            for i in range(len(ev) + 1):
                for j in range(i, len(ev) + 1):
                    cand = ev[:i] + [(task_id, "P")] + ev[i:j] + [(task_id, "D")] + ev[j:]
                    self.tries += 1
                    delta = self.route_cost(bot_id, cand) - base
                    if delta < best[0] - EPS:
                        best = (delta, bot_id, cand)
        return best

    # ---------- bước 1: lọc
    def step1_filter(self) -> Plan:
        plan = Plan()
        for b in sorted(self.bots.values(), key=lambda b: b.id):
            if b.battery < self.cfg["minBattery"]:
                self.skipped_bots[b.id] = f"pin {b.battery:.0f}% < {self.cfg['minBattery']:.0f}%"
                self._log("1-LỌC", f"loại {b.id}: {self.skipped_bots[b.id]}")
            else:
                plan.routes[b.id] = []
                self._log("1-LỌC", f"giữ {b.id} ({b.type}, ở {b.node}, pin {b.battery:.0f}%, {b.capacity} ngăn)")
        return plan

    # ---------- bước 2: chia nháp
    def step2_greedy(self, plan: Plan) -> Plan:
        order = sorted(self.tasks.values(), key=lambda t: (-t.weight, t.deadline_s, -t.waited_s, t.id))
        for task in order:
            before = self.tries
            delta, bot_id, cand = self.best_insertion(plan, task.id)
            if bot_id is None:
                plan.unassigned[task.id] = self._why_unassigned(task, plan)
                self._log("2-NHÁP", f"{task.id}: không nhét được — {plan.unassigned[task.id]}")
            else:
                plan.routes[bot_id] = cand
                self._log("2-NHÁP", f"{task.id} ({task.priority}, hạn {task.deadline_s:.0f}s): thử "
                                    f"{self.tries - before} chỗ → rẻ nhất là {bot_id}, điểm phạt +{delta:.3f}. "
                                    f"{bot_id}: {self._route_str(cand)}")
        self._log("2-NHÁP", f"XONG bản nháp: tổng điểm phạt {self.total_cost(plan):.3f}")
        return plan

    def _why_unassigned(self, task: Task, plan: Plan) -> str:
        same_type = [b for b in self.bots.values() if b.type == task.robot_type]
        if not same_type:
            return f"không có robot loại {task.robot_type} đang rảnh → chờ lượt chia sau"
        if not any(b.id in plan.routes for b in same_type):
            return "robot cùng loại đều không đủ pin"
        return "không nhét được: quá số ngăn, không đủ pin cho cả chuyến, hoặc không có đường"

    # ---------- bước 3: sửa dần
    def step3_improve(self, plan: Plan) -> tuple[Plan, dict]:
        stats = {"relocate": 0, "swap": 0, "rounds": 0, "timeUp": False}
        stop_at = time.perf_counter() + float(self.cfg["timeLimitSeconds"])
        improved = True
        while improved:
            improved = False
            stats["rounds"] += 1
            if time.perf_counter() > stop_at:
                stats["timeUp"] = True
                break
            # (a) việc chưa ai nhận: thử nhét lại (có thể bước trước đã mở ra chỗ trống)
            for task_id in sorted(plan.unassigned):
                delta, bot_id, cand = self.best_insertion(plan, task_id)
                if bot_id is not None:
                    plan.routes[bot_id] = cand
                    del plan.unassigned[task_id]
                    improved = True
            # (b) CHUYỂN: nhấc 1 việc ra, nhét vào chỗ tốt nhất (robot khác hoặc vị trí khác)
            for task_id in self._assigned(plan):
                owner = self._owner(plan, task_id)
                old = plan.routes[owner]
                without = [e for e in old if e[0] != task_id]
                saved = self.route_cost(owner, old) - self.route_cost(owner, without)
                trial = plan.copy()
                trial.routes[owner] = without
                delta, bot_id, cand = self.best_insertion(trial, task_id)
                if bot_id is not None and delta < saved - EPS:
                    trial.routes[bot_id] = cand
                    move = (f"DỜI {task_id} sang vị trí khác trong chuyến {owner}" if bot_id == owner
                            else f"CHUYỂN {task_id}: {owner} → {bot_id}")
                    self._log("3-SỬA", f"{move}. Tổng điểm phạt "
                                       f"{self.total_cost(plan):.3f} → {self.total_cost(trial):.3f} (giảm → GIỮ). "
                                       f"{bot_id}: {self._route_str(cand)}")
                    plan = trial
                    stats["relocate"] += 1
                    improved = True
            # (c) ĐỔI CHÉO: việc x của robot A ↔ việc y của robot B
            assigned = self._assigned(plan)
            for a_idx, x in enumerate(assigned):
                for y in assigned[a_idx + 1:]:
                    ra, rb = self._owner(plan, x), self._owner(plan, y)
                    if ra == rb or self.tasks[x].robot_type != self.tasks[y].robot_type:
                        continue
                    before = self.route_cost(ra, plan.routes[ra]) + self.route_cost(rb, plan.routes[rb])
                    trial = plan.copy()
                    trial.routes[ra] = [e for e in trial.routes[ra] if e[0] != x]
                    trial.routes[rb] = [e for e in trial.routes[rb] if e[0] != y]
                    _, b1, c1 = self.best_insertion(trial, x, only=rb)
                    if b1 is None:
                        continue
                    trial.routes[rb] = c1
                    _, b2, c2 = self.best_insertion(trial, y, only=ra)
                    if b2 is None:
                        continue
                    trial.routes[ra] = c2
                    after = self.route_cost(ra, trial.routes[ra]) + self.route_cost(rb, trial.routes[rb])
                    if after < before - EPS:
                        self._log("3-SỬA", f"ĐỔI CHÉO {x} ({ra}) ↔ {y} ({rb}). Tổng điểm phạt "
                                           f"{self.total_cost(plan):.3f} → {self.total_cost(trial):.3f} (giảm → GIỮ)")
                        plan = trial
                        stats["swap"] += 1
                        improved = True
        self._log("3-SỬA", f"DỪNG sau {stats['rounds']} vòng: vòng cuối thử hết mà không giảm được nữa. "
                           f"Tổng điểm phạt {self.total_cost(plan):.3f}")
        return plan, stats

    def _assigned(self, plan: Plan) -> list[str]:
        return sorted({e[0] for ev in plan.routes.values() for e in ev})

    @staticmethod
    def _owner(plan: Plan, task_id: str) -> str:
        return next(r for r, ev in plan.routes.items() if any(e[0] == task_id for e in ev))

    # ---------- chạy cả 4 bước
    def solve(self, improve: bool = True) -> dict:
        t0 = time.perf_counter()
        draft = self.step2_greedy(self.step1_filter())
        draft_cost = self.total_cost(draft)
        final, stats = (self.step3_improve(draft.copy()) if improve else (draft, {}))
        final_cost = self.total_cost(final)
        return self._output(draft, draft_cost, final, final_cost, stats, time.perf_counter() - t0, improve)

    # ---------- bước 4: đóng gói kết quả
    def _output(self, draft, draft_cost, plan, cost, stats, elapsed, improved) -> dict:
        plans, explain = [], []
        for bot_id in sorted(plan.routes):
            ev = plan.routes[bot_id]
            if not ev:
                continue
            e = self.evaluate(bot_id, ev)
            plans.append({
                "robotId": bot_id,
                "stops": self._stops(ev),
                "taskIds": sorted({x[0] for x in ev}),
                "distanceM": round(e.meters, 1),
                "finishInSec": round(e.finish_s),
                "cost": round(e.cost, 3),
            })
            for task_id in sorted(e.deliver_s, key=e.deliver_s.get):
                task = self.tasks[task_id]
                eta, late = e.deliver_s[task_id], max(0.0, e.deliver_s[task_id] - task.deadline_s)
                mates = sorted({x[0] for x in ev} - {task_id})
                explain.append({
                    "taskId": task_id, "robotId": bot_id, "priority": task.priority,
                    "etaSec": round(eta), "deadlineInSec": round(task.deadline_s), "lateSec": round(late),
                    "text": (f"{task_id} ({'GẤP' if task.priority == 'URGENT' else 'thường'}) → {bot_id}"
                             + (f", gom chung với {', '.join(mates)}" if mates else "")
                             + f"; giao sau ~{eta / 60:.1f} phút, hạn còn {task.deadline_s / 60:.1f} phút"
                             + (f" → TRỄ {late / 60:.1f} phút" if late > 0 else " → kịp")),
                })
        late_count = sum(1 for x in explain if x["lateSec"] > 0)
        gain = 0.0 if draft_cost <= EPS else (draft_cost - cost) / draft_cost * 100
        return {
            "plans": plans,
            "unassigned": [{"taskId": k, "reason": v} for k, v in sorted(plan.unassigned.items())],
            "skippedRobots": [{"robotId": k, "reason": v} for k, v in sorted(self.skipped_bots.items())],
            "explain": explain,
            "score": {
                "draft": round(draft_cost, 3),           # bản nháp = luật "gấp trước, chỗ rẻ nhất"
                "final": round(cost, 3),                 # sau khi AI sửa dần
                "improvementPct": round(gain, 1),
                "lateTasks": late_count,
                "draftLateTasks": self._late_count(draft),
                **self._totals(draft, "draft"),
                **self._totals(plan, "final"),
            },
            "meta": {"algorithm": "greedy-insertion + local-search (relocate, swap)",
                     "improved": improved, "elapsedMs": round(elapsed * 1000, 1), "tried": self.tries, **stats},
            "trace": self.trace,     # từng bước AI nghĩ (để Operator/hội đồng xem; backend có thể bỏ qua)
        }

    def _totals(self, plan: Plan, prefix: str) -> dict:
        """Tổng mét chạy và thời điểm robot cuối cùng xong việc — backend lưu để so AI với luật."""
        evals = [self.evaluate(b, ev) for b, ev in plan.routes.items() if ev]
        return {f"{prefix}DistanceM": round(sum(e.meters for e in evals), 1),
                f"{prefix}FinishSec": round(max((e.finish_s for e in evals), default=0))}

    def _late_count(self, plan: Plan) -> int:
        n = 0
        for bot_id, ev in plan.routes.items():
            if ev:
                e = self.evaluate(bot_id, ev)
                n += sum(1 for k, t in e.deliver_s.items() if t > self.tasks[k].deadline_s)
        return n

    def _stops(self, events: list[tuple[str, str]]) -> list[dict]:
        """Dãy sự kiện → điểm dừng. Hai sự kiện liền nhau cùng điểm, cùng hành động thì gộp làm một."""
        stops: list[dict] = []
        for task_id, kind in events:
            task = self.tasks[task_id]
            node = task.pickup if kind == "P" else task.dropoff
            action = "PICKUP" if kind == "P" else "DELIVER"
            if stops and stops[-1]["nodeId"] == node and stops[-1]["action"] == action:
                stops[-1]["taskIds"].append(task_id)
            else:
                stops.append({"seq": len(stops) + 1, "nodeId": node, "action": action, "taskIds": [task_id]})
        return stops


def _parse_time(s):
    if not s:
        return None
    dt = datetime.fromisoformat(str(s).replace("Z", "+00:00"))
    return dt if dt.tzinfo else dt.replace(tzinfo=timezone.utc)


def plan(req: dict) -> dict:
    """Hàm chính: backend gửi yêu cầu (dict) → trả kế hoạch (dict). `improve: false` = chỉ chạy bản nháp."""
    return Planner(req).solve(improve=req.get("improve", True))
