# Hợp đồng MQTT — Backend ↔ Robot (bản nháp v0.1, 21/11/2026)

Robot (Webots hôm nay, robot thật sau này) và Backend chỉ nói chuyện qua Mosquitto bằng 3 topic dưới đây.
Hai bên làm đúng file này thì không cần biết code của nhau. **Đổi gì ở đây phải báo cả hai bên.**

| Topic | Chiều | Khi nào | QoS / retain |
|---|---|---|---|
| `rmms/v1/{robotId}/order` | Backend → Robot | khi giao chuyến hoặc huỷ | 1 / không |
| `rmms/v1/{robotId}/state` | Robot → Backend | mỗi 1 giây | 0 / không |
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

Huỷ: `{ "command": "CANCEL", "robotId": "MED-01" }` → robot dừng tại chỗ, về `IDLE`.

## 2. `state` — robot báo cáo mỗi giây

```json
{
  "robotId": "MED-01", "ts": "2026-11-21T03:15:03Z", "floor": "F2",
  "x": 19.4, "y": 22.0, "theta": 0.0,
  "battery": 82, "speed": 0.95,
  "status": "MOVING", "orderId": "T-0012",
  "nextNodeId": "J:F2-WARD-A", "atStopSeq": null
}
```

| Trường | Ý nghĩa |
|---|---|
| `status` | `IDLE` rảnh · `MOVING` đang đi · `AT_STOP` đang ở điểm dừng · `RETURNING` đang về trạm · `CHARGING` đang sạc · `ERROR` lỗi |
| `atStopSeq` | đang đứng ở điểm dừng số mấy (khi `AT_STOP`), còn lại `null` |
| `theta` | hướng mặt robot (radian): 0 = đông, π/2 = bắc |
| `battery` | % pin (số nguyên) |

Gợi ý phía backend: lưu `x, y, battery, speed` vào `TelemetrySnapshot`, cập nhật `Robot.CurrentX/CurrentY/Battery/LastHeartbeatAt`.
**Quá 5 giây không nhận được `state` → coi robot mất kết nối** (NFR).

## 3. `connection`

```json
{ "robotId": "MED-01", "connection": "ONLINE", "ts": "2026-11-21T03:10:00Z" }
```
Retain = có, nên backend khởi động sau vẫn biết ngay robot nào đang online. Robot mất điện đột ngột → Mosquitto tự gửi `"connection": "OFFLINE"`.

## 4. Còn phải chốt với backend

1. Robot giao xong thì **tự về trạm sạc** (Webots đang làm vậy) hay **chờ backend ra lệnh về**?
2. Ở điểm dừng, robot chờ **tín hiệu gì để đi tiếp** (người xác nhận / sensor ngăn)? Webots tạm đứng 5 giây.
3. Backend có thêm `path` vào `order` không?
