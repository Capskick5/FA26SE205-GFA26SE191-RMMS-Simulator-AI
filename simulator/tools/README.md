# tools — tạo world Webots từ bản đồ

Hai lệnh, chạy theo thứ tự, **đứng ở thư mục `Simulator-Webots`** (không phải trong `tools`):

```
python tools/fe_to_map.py        # B1: lấy bản đồ tầng F2 từ code FE  → map/F2.json
python tools/build_world.py      # B2: map/F2.json                    → worlds/rmms_F2.wbt
```

Tầng khác: thêm tên tầng vào cuối cả 2 lệnh, ví dụ `python tools/fe_to_map.py F3` rồi `python tools/build_world.py F3`.
Trong VS Code: chạy task **"5. Webots: tạo lại world F2 từ FE"** là làm luôn cả 2 bước.

## Khi nào chạy lại?
Khi có người sửa bản đồ trong FE (`Frontend/src/legacy/engine.js`: danh sách `LOCS` hoặc `addRobot`).
Chạy xong, trong Webots bấm **File → Reload World**.

## Mỗi file làm gì
| File | Vào | Ra |
|---|---|---|
| `fe_to_map.py` | `engine.js` của FE (10 px = 1 m) | `map/F2.json` — phòng, điểm, đồ thị đường đi, robot (đơn vị mét) |
| `build_world.py` | `map/F2.json` | `worlds/rmms_F2.wbt` + biển tên phòng `worlds/textures/` |

## Thử robot không cần backend: `send_order.py`
Đóng vai backend: gửi lệnh vào `rmms/v1/<robot>/order` và xem robot báo cáo. Cần Mosquitto đang chạy.
```
python tools/send_order.py watch                               # cửa sổ 1: xem robot báo cáo mỗi giây
python tools/send_order.py MED-01 F2-PHARM F2-WARD-A           # cửa sổ 2: lệnh kiểu hiện tại (chỉ điểm dừng)
python tools/send_order.py SUP-01 F2-STORE F2-ICU --path       # lệnh kiểu đề xuất (có path x, y)
python tools/send_order.py MED-01 cancel                       # huỷ, robot đứng yên
```
Điểm đầu = điểm lấy hàng, các điểm sau = điểm giao. Robot dừng mỗi điểm 5 giây rồi đi tiếp, xong tự về trạm sạc.

## Lưu ý
- **Không sửa tay** file `.wbt` và `F2.json` — lần chạy sau sẽ ghi đè.
- Sau này bản đồ lấy từ file JSON Admin import vào backend; khi đó chỉ thay B1, B2 giữ nguyên.
- Cần Python 3 + Pillow (vẽ biển tên phòng). Thiếu Pillow thì world vẫn tạo được, chỉ không có biển.
