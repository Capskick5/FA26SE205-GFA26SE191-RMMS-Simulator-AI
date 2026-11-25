# Hợp đồng MQTT — Backend ↔ Robot (bản nháp v0.4, 25/11/2026)

Robot (Webots hôm nay, robot thật sau này) và Backend chỉ nói chuyện qua Mosquitto bằng các topic dưới đây.
Hai bên làm đúng file này thì không cần biết code của nhau. **Đổi gì ở đây phải báo cả hai bên.**

**v0.2 (22/11):** thêm cảm biến ngăn hàng — trường `compartments`, `wait` trong `state`; topic `event`;
lệnh `CONFIRM`, `EXTEND`; `stops[].timeoutSec`, `graceSec`. Mọi trường v0.1 giữ nguyên tên và ý nghĩa.

**v0.3 (24/11):** sự cố cho WF-04 — `state.error`; event `BATTERY_LOW`, `BATTERY_CRITICAL`, `RECOVERED`; lệnh `RECOVER`;
lệnh giả lập `FAULT` (chỉ để test/demo). **Lưu ý backend (Hải, 23/11):** `RobotStateSubscriberWorker.Valid` đang từ chối
`status = RETURNING` → robot đang về trạm bị coi là tin hỏng. Cần thêm `RETURNING` vào danh sách hợp lệ.

**v0.4 (25/11):** khớp code Hải (commit a9ac67d) — topic `rmms/v1/{robotId}/command` cho `STOP`/`RESUME` khẩn
(payload `{commandId, command, issuedAt}`). Robot trả event `COMMAND_DONE` có `commandId`; `commandId` trùng thì bỏ qua
(outbox gửi "ít nhất 1 lần"). STOP → `status: ERROR`, `error: OPERATOR_STOP`, giữ lệnh đang dở; RESUME → chạy tiếp lệnh đó.

| Topic | Chiều | Khi nào | QoS / retain |
|---|---|---|---|
| `rmms/v1/{robotId}/order` | Backend → Robot | khi giao chuyến, huỷ, xác nhận, chờ thêm | 1 / không |
| `rmms/v1/{robotId}/command` | Backend → Robot | Operator bấm dừng khẩn / cho chạy lại (`STOP`, `RESUME`) | 1 / không |
| `rmms/v1/{robotId}/state` | Robot → Backend | mỗi 1 giây | 0 / không |
| `rmms/v1/{robotId}/event` | Robot → Backend | khi có chuyện xảy ra một lần (cảm biến ngăn đổi, hết giờ chờ...) | 1 / không |
| `rmms/v1/{robotId}/connection` | Robot → Backend | khi kết nối; mất điện thì Mosquitto tự gửi (last will) | 1 / **có** |

`robotId` = mã robot trong hệ thống, ví dụ `MED-01`, `SUP-01`.
Toạ độ: mét, `x` hướng đông, `y` hướng bắc, gốc theo bản đồ tầng (xem `simulator/map/F2.json`).

## 1. `order` — giao chuyến

```json
{
  "orderId": "T-0012",
  "robotId": "MED-01",
  "mapVersionId": "…",
  "stops": [
    { "seq": 1, "nodeId": "F2-PHARM",  "action": "PICKUP",  "taskIds": ["NV-0101"] },
    { "seq": 2, "nodeId": "F2-WARD-A", "action": "DELIVER", "taskIds": ["NV-0101"] }
  ],
  "path": [
    { "nodeId": "F2-CS-PH",    "x": 11.0, "y": 33.0 },
    { "nodeId": "F2-PHARM",    "x": 11.0, "y": 25.8, "stopSeq": 1 },
    { "nodeId": "J:F2-PHARM",  "x": 11.0, "y": 22.0 },
    { "nodeId": "J:F2-WARD-A", "x": 27.0, "y": 22.0 },
    { "nodeId": "F2-WARD-A",   "x": 27.0, "y": 25.8, "stopSeq": 2 }
  ],
  "compartments": [ { "slot": 1, "taskId": "NV-0101", "item": "TH-PARA", "qty": 20 } ]
}
```

- `stops`, `compartments`: **đã có** trong code backend (Hải, 20/11).
- `path`: **ĐỀ XUẤT THÊM** — mọi điểm robot đi qua theo đường Dijkstra backend tính; điểm dừng có `stopSeq` khớp `stops.seq`.
  Robot Webots hiện đọc được cả lệnh **không có** `path` (khi đó robot tạm tự tìm đường trên bản đồ).

- `stops[].timeoutSec` (tuỳ chọn): chờ cảm biến tối đa bao lâu ở điểm này. Bỏ trống → theo loại hàng
  (mã bắt đầu `TH` thuốc 180 s, `VT` vật tư 300 s, `MAU` mẫu 120 s — Prototype v8.2).
- `stops[].graceSec` (tuỳ chọn): hết giờ rồi, chờ Operator thêm bao lâu trước khi robot tự đi (mặc định 120 s).

**Lệnh điều khiển** (cùng topic `order`, có trường `command`):

| Lệnh | Khi nào (nút trên màn Operator) | Robot làm gì |
|---|---|---|
| `{ "command": "CANCEL", "robotId": "MED-01" }` | Huỷ chuyến | dừng tại chỗ, về `IDLE` |
| `{ "command": "CONFIRM", "robotId": "MED-01" }` | "Đã lấy hàng" / "Cho đi tiếp" | thôi chờ cảm biến, đi tiếp (`STOP_DONE` với `by: OPERATOR`) |
| `{ "command": "EXTEND", "robotId": "MED-01", "seconds": 120 }` | "Chờ thêm" (sau khi hết giờ) | chờ cảm biến thêm `seconds` giây |
| `{ "command": "RECOVER", "robotId": "MED-01" }` | "Đã xử lý sự cố" (người tới thay/sạc pin tại chỗ) | hết `ERROR`, pin ≥ 60%, chạy tiếp lệnh đang dở |
| `{ "command": "FAULT", "fault": "BATTERY", "value": 12 }` | **chỉ để test/demo** | pin tụt còn `value`% |
| `{ "command": "FAULT", "fault": "OFFLINE", "seconds": 25 }` | **chỉ để test/demo** | rớt mạng `seconds` giây, robot vẫn chạy lệnh đang có |

## 1c. Sự cố (WF-04)

| Sự cố | Robot làm gì | Backend thấy gì |
|---|---|---|
| **Pin ≤ 15%** khi đang làm việc | chạy tiếp | event `BATTERY_LOW` (1 lần) |
| **Pin ≤ 10%** khi đang làm việc (hết pin giữa đường) | **dừng tại chỗ**, giữ hàng trong ngăn, chờ người | `status: ERROR`, `error: BATTERY_CRITICAL`; event `BATTERY_CRITICAL` có `x, y, cargo[{slot, taskId}]` |
| **Mất mạng** | vẫn chạy lệnh đang có (robot tự hành), không báo về được | hết `state` → >5 s coi mất kết nối; ~15 s sau Mosquitto gửi `connection: OFFLINE`; có mạng lại → `ONLINE` + `state` như bình thường |

Gợi ý backend: `BATTERY_CRITICAL` → tạo sự cố, Operator chọn **cứu tại chỗ** (gửi `RECOVER`) hoặc **chuyển việc cho robot khác**
(huỷ chuyến này, tạo chuyến mới từ vị trí `x, y` của robot hỏng — hàng trong `cargo`).

## 1b. Robot chờ ở điểm dừng — CẢM BIẾN NGĂN HÀNG

Mỗi ngăn có 1 cảm biến: **có hộp hàng / trống**. Robot biết việc nào ở ngăn nào nhờ `compartments` trong lệnh.

| Điểm dừng | Robot chờ tới khi | Hết `timeoutSec` | Hết thêm `graceSec` mà Operator im lặng |
|---|---|---|---|
| `PICKUP` | các ngăn của việc ở điểm này **CÓ HÀNG** | event `PICKUP_SHORT`, đèn đỏ | event `PICKUP_FAILED`, robot đi tiếp |
| `DELIVER` | các ngăn của việc ở điểm này **TRỐNG** | event `NOT_RECEIVED`, đèn đỏ | event `DELIVERY_FAILED`, robot đi tiếp, **hàng mang về** |

Cảm biến xác nhận đủ → event `STOP_DONE` (`by: SENSOR`) → chờ 1,5 s (đóng ngăn) → robot rời điểm dừng.
Lệnh **không có** `compartments` (vd gửi tay) → robot đứng cứng 5 giây như v0.1.

**Ý nghĩa cho backend:** rời `AT_STOP` ở điểm `PICKUP` = đã lấy đủ hàng; rời `AT_STOP` ở điểm `DELIVER` = ngăn đã
trống — nay là do **cảm biến đo thật**, không còn là đoán theo thời gian. Muốn biết chính xác thì nghe event `STOP_DONE`.

## 2. `state` — robot báo cáo mỗi giây

```json
{
  "robotId": "MED-01", "ts": "2026-11-21T03:15:03Z", "floor": "F2",
  "x": 19.4, "y": 22.0, "theta": 0.0,
  "battery": 82, "speed": 0.95,
  "status": "MOVING", "orderId": "T-0012",
  "nextNodeId": "J:F2-WARD-A", "atStopSeq": null,
  "compartments": [ { "slot": 1, "occupied": true, "taskId": "NV-0101" }, { "slot": 2, "occupied": false, "taskId": null } ],
  "wait": null
}
```
Khi đang chờ ở điểm dừng: `"wait": { "stopSeq": 1, "expect": "LOAD", "slots": [1], "phase": "WAITING", "leftSec": 172 }`
(`expect`: `LOAD` chờ đặt hàng vào / `UNLOAD` chờ lấy hàng ra; `phase`: `WAITING` → `TIMEOUT` → `CLOSING`).

| Trường | Ý nghĩa |
|---|---|
| `status` | `IDLE` rảnh · `MOVING` đang đi · `AT_STOP` đang ở điểm dừng · `RETURNING` đang về trạm · `CHARGING` đang sạc · `ERROR` lỗi |
| `atStopSeq` | đang đứng ở điểm dừng số mấy (khi `AT_STOP`), còn lại `null` |
| `error` | mã sự cố khi `status = ERROR` (vd `BATTERY_CRITICAL`), còn lại `null` |
| `theta` | hướng mặt robot (radian): 0 = đông, π/2 = bắc |
| `battery` | % pin (số nguyên) |

Gợi ý phía backend: lưu `x, y, battery, speed` vào `TelemetrySnapshot`, cập nhật `Robot.CurrentX/CurrentY/Battery/LastHeartbeatAt`.
**Quá 5 giây không nhận được `state` → coi robot mất kết nối** (NFR).

## 2b. `event` — chuyện xảy ra một lần

```json
{ "robotId": "MED-01", "ts": "2026-11-22T02:04:31Z", "type": "SLOT", "slot": 1, "occupied": true, "distanceM": 0.01, "taskId": "NV-0101" }
```

| `type` | Thêm trường | Nghĩa |
|---|---|---|
| `SLOT` | `slot, occupied, distanceM, taskId` | cảm biến ngăn đổi trạng thái (đã chống nhiễu) |
| `STOP_DONE` | `stopSeq, by (SENSOR/OPERATOR), action, taskIds` | xong điểm dừng, robot sắp đi |
| `PICKUP_SHORT` / `NOT_RECEIVED` | `stopSeq, taskIds` | hết giờ chờ → hiện cảnh báo cho Operator |
| `PICKUP_FAILED` / `DELIVERY_FAILED` | `stopSeq, taskIds` | Operator không xử lý → robot tự đi tiếp |
| `BATTERY_LOW` | `battery, orderId` | pin ≤ 15% khi đang làm việc |
| `BATTERY_CRITICAL` | `battery, orderId, x, y, cargo[{slot, taskId}]` | pin ≤ 10% → robot dừng tại chỗ chờ người |
| `RECOVERED` | `error` | người đã xử lý (lệnh `RECOVER`), robot chạy tiếp |
| `COMMAND_DONE` | `command, commandId` | robot đã làm lệnh `STOP`/`RESUME` (backend đánh dấu outbox đã giao) |

## 3. `connection`

```json
{ "robotId": "MED-01", "connection": "ONLINE", "ts": "2026-11-21T03:10:00Z" }
```
Retain = có, nên backend khởi động sau vẫn biết ngay robot nào đang online. Robot mất điện đột ngột → Mosquitto tự gửi `"connection": "OFFLINE"`.

## 4. Còn phải chốt với backend

1. Robot giao xong thì **tự về trạm sạc** (Webots đang làm vậy) hay **chờ backend ra lệnh về**?
2. ~~Ở điểm dừng robot chờ tín hiệu gì?~~ → **ĐỀ XUẤT v0.2:** cảm biến ngăn + nút Operator `CONFIRM`/`EXTEND`,
   hết giờ theo v8.2 (mục 1b). Webots đã làm xong, cần Hải đồng ý + làm phía backend (nghe `event`, nút trên FE).
3. Backend có thêm `path` vào `order` không?
