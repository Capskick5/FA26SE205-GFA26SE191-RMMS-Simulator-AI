# Hợp đồng API — Backend ↔ AI Scheduler (bản nháp v0.1, 21/11/2026)

AI Scheduler là dịch vụ HTTP chạy cạnh backend (demo: `http://127.0.0.1:8000`). Backend gọi khi có việc chờ.
AI **không** đọc DB, **không** biết Webots/MQTT: chỉ nhận dữ liệu → trả kế hoạch. Backend giữ toàn quyền quyết định.

| Gọi | Để làm gì |
|---|---|
| `GET /health` | kiểm tra AI còn sống → `{"status":"ok"}` |
| `POST /plan` | gửi việc chờ + robot rảnh + khoảng cách → nhận kế hoạch |

**AI lỗi, tắt, hoặc quá 5 giây không trả lời → backend dùng luật dự phòng BM04** (việc gấp trước, mỗi chuyến 1 việc).

## 1. Yêu cầu `POST /plan`

```json
{
  "now": "2026-11-21T08:00:00Z",
  "config": { "serviceSeconds": 30, "speedFactor": 0.72 },
  "robots": [
    { "robotId": "MED-01", "type": "MED_CART", "node": "F2-CS-PH", "battery": 95, "speed": 1.0, "slots": 4 }
  ],
  "tasks": [
    { "taskId": "NV-0101", "priority": "URGENT", "robotType": "MED_CART",
      "pickup": "F2-PHARM", "dropoff": "F2-WARD-B", "slots": 1,
      "createdAt": "2026-11-21T07:58:00Z", "deadline": "2026-11-21T08:10:00Z" }
  ],
  "distances": { "F2-CS-PH": { "F2-PHARM": 7.2, "F2-WARD-B": 39.6 }, "F2-PHARM": { "F2-WARD-B": 32.4 } }
}
```

| Trường | Ý nghĩa | Backend lấy từ đâu |
|---|---|---|
| `robots[]` | **chỉ robot đang rảnh** (IDLE/CHARGING) | bảng Robot + state MQTT mới nhất |
| `robots[].node` | điểm gần robot nhất | toạ độ `x,y` trong state → điểm gần nhất trên bản đồ |
| `robots[].slots` | số ngăn | loại robot |
| `tasks[]` | việc đang chờ (chưa ai nhận) | bảng Task |
| `tasks[].priority` | `URGENT` / `NORMAL` | |
| `tasks[].deadline` | hạn chót; bỏ trống → gấp 15 phút, thường 60 phút tính từ `createdAt` | |
| `distances` | mét giữa mọi cặp điểm có trong yêu cầu (một chiều là đủ) | **Dijkstra** trên bản đồ tầng |
| `config` | (tuỳ chọn) đè cấu hình mặc định, xem mục 3 | cấu hình Admin |

## 2. Trả về

```json
{
  "plans": [
    { "robotId": "MED-01", "taskIds": ["NV-0101", "NV-0102"],
      "stops": [
        { "seq": 1, "nodeId": "F2-PHARM",  "action": "PICKUP",  "taskIds": ["NV-0101", "NV-0102"] },
        { "seq": 2, "nodeId": "F2-WARD-B", "action": "DELIVER", "taskIds": ["NV-0101"] },
        { "seq": 3, "nodeId": "F2-PEDS",   "action": "DELIVER", "taskIds": ["NV-0102"] }
      ],
      "distanceM": 71.2, "finishInSec": 160, "cost": 0.712 }
  ],
  "unassigned":    [ { "taskId": "NV-0105", "reason": "không có robot loại SUPPLY_CART đang rảnh → chờ lượt chia sau" } ],
  "skippedRobots": [ { "robotId": "MED-02", "reason": "pin 15% < 20%" } ],
  "explain": [ { "taskId": "NV-0101", "robotId": "MED-01", "etaSec": 70, "deadlineInSec": 600, "lateSec": 0,
                 "text": "NV-0101 (GẤP) → MED-01, gom chung với NV-0102; giao sau ~1.2 phút, hạn còn 10.0 phút → kịp" } ],
  "score": { "draft": 5.755, "final": 2.916, "improvementPct": 49.3, "draftLateTasks": 3, "lateTasks": 0 },
  "meta":  { "algorithm": "greedy-insertion + local-search (relocate, swap)", "elapsedMs": 1.8 }
}
```

- `plans[].stops` có **đúng dạng `stops`** của lệnh MQTT (`contracts/mqtt.md`) → backend tạo Trip, thêm `path` (Dijkstra), `compartments`, rồi gửi.
- `unassigned`: để lại cho lượt chia sau.
- `explain[].text`: hiện cho Operator ("vì sao robot này nhận việc này").
- `score`: `draft` = bản nháp theo luật, `final` = sau khi AI sửa. Lưu lại để làm báo cáo hiệu quả AI.

## 3. Cấu hình (Admin chỉnh được)

| Khoá | Mặc định | Ý nghĩa |
|---|---|---|
| `priorityWeight` | `{"URGENT": 3, "NORMAL": 1}` | 1 phút trễ của việc gấp nặng bằng 3 phút trễ việc thường |
| `metersPerPoint` | 100 | chạy 100 m = 1 điểm phạt |
| `minBattery` | 20 | dưới % này robot không nhận việc |
| `reserveBattery` | 10 | chạy xong chuyến phải còn ≥ % này |
| `batteryPerMeter` | 0.15 | % pin mỗi mét |
| `serviceSeconds` | 30 | đứng ở mỗi điểm dừng (Webots: 6) |
| `speedFactor` | 1.0 | tốc độ thực / tốc độ danh nghĩa (đo Webots: 0.72) |
| `timeLimitSeconds` | 1.0 | thời gian tối đa cho bước "sửa dần" |

**Điểm phạt** = Σ (phút trễ × `priorityWeight`) + tổng mét ÷ `metersPerPoint`. AI chọn kế hoạch có điểm phạt nhỏ nhất tìm được.

## 4. Còn phải chốt với backend
1. Backend gọi AI **khi nào**: mỗi khi có việc mới, hay định kỳ (ví dụ 10 giây một lần)? Đề xuất: cả hai, gộp các yêu cầu sát nhau.
2. Kế hoạch đã gửi cho robot rồi thì **không chia lại** (bản 1). Có muốn chèn việc gấp vào chuyến đang chạy không? (bản 2)
3. Lưu `score` vào bảng nào để làm báo cáo?
