# RMMS — Simulator + AI Scheduler (FA26SE205 · GFA26SE191)

Phần của hệ thống RMMS chạy bằng Python:

| Thư mục | Là gì | Nói chuyện với ai |
|---|---|---|
| `contracts/` | Hợp đồng giao tiếp: `mqtt.md` (robot), `ai-api.md` (AI) | Backend đọc để làm cho khớp |
| `simulator/` | Robot mô phỏng trên Webots, thay robot thật + toà nhà | Backend, **chỉ qua MQTT** |
| `ai-scheduler/` | Chia việc: ưu tiên, chọn robot, gom việc, xếp thứ tự điểm | Backend, **chỉ qua HTTP** |
| `demo/` | "Backend giả" để chạy AI + Webots khi chưa có backend thật | Gọi AI + gửi MQTT |

Backend (.NET) và Frontend (React) nằm ở repo khác. AI không biết Webots; Webots không biết AI — cả hai chỉ nói với backend.

Cần: Webots R2023b+, Python 3.11+, `pip install paho-mqtt pillow`, Mosquitto (cổng 1883). AI không cần thư viện ngoài.

## Chạy thử cả luồng: việc chờ → AI → robot Webots (4 cửa sổ)

```
1. Mosquitto        task VS Code "1. Mosquitto: bật"
2. Webots           task "4. Webots: mở world F2" → bấm ▶
3. AI               python ai-scheduler/server.py               (task "6. AI: bật")
4. Backend giả      python demo/mini_backend.py                 (task "7. Demo: AI chia việc cho robot")
```
`python demo/mini_backend.py --offline --dry-run` = chỉ xem AI chia thế nào, không cần Webots/Mosquitto.

## Kiểm tra AI
```
python -m unittest discover -s ai-scheduler/tests -v
```

## Gửi lệnh tay cho một robot
```
cd simulator
python tools/send_order.py watch                       # xem robot báo cáo
python tools/send_order.py MED-01 F2-PHARM F2-WARD-A   # gửi thử một chuyến
```
Chi tiết: `simulator/tools/README.md`.
