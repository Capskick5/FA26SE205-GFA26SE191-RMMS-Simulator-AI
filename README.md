# RMMS — Simulator + AI Scheduler (FA26SE205 · GFA26SE191)

Phần của hệ thống RMMS chạy bằng Python:

| Thư mục | Là gì | Nói chuyện với ai |
|---|---|---|
| `contracts/` | Hợp đồng giao tiếp với backend (MQTT, API của AI) | Backend đọc để làm cho khớp |
| `simulator/` | Robot mô phỏng trên Webots, thay robot thật + toà nhà | Backend, **chỉ qua MQTT** |
| `ai-scheduler/` | *(đang làm)* Xếp ưu tiên, chọn robot, xếp thứ tự điểm | Backend, **chỉ qua HTTP** |

Backend (.NET) và Frontend (React) nằm ở repo khác. AI không biết Webots; Webots không biết AI — cả hai chỉ nói với backend.

## Chạy robot mô phỏng
Cần: Webots R2023b+, Python 3 + `pip install paho-mqtt pillow`, Mosquitto (cổng 1883).

```
cd simulator
python tools/send_order.py watch                       # xem robot báo cáo
python tools/send_order.py MED-01 F2-PHARM F2-WARD-A   # gửi thử một chuyến
```
Mở `simulator/worlds/rmms_F2.wbt` trong Webots, bấm ▶. Chi tiết: `simulator/tools/README.md`.
