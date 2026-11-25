"""server — mở AI Scheduler thành dịch vụ HTTP để backend gọi.

    python ai-scheduler/server.py            # chạy ở http://127.0.0.1:8000
    python ai-scheduler/server.py 8001       # đổi cổng

    GET  /health  → {"status": "ok"}
    POST /plan    → gửi JSON yêu cầu (xem contracts/ai-api.md), nhận JSON kế hoạch

Chỉ dùng thư viện chuẩn Python. Không cần Internet.
"""
import json
import sys
import traceback
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))
from planner import plan  # noqa: E402

if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")


class Handler(BaseHTTPRequestHandler):
    def _send(self, code, body):
        data = json.dumps(body, ensure_ascii=False).encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def _body(self):
        """Đọc body: có Content-Length thì đọc đủ độ dài; gửi kiểu chunked (HttpClient .NET hay dùng) thì ghép từng khúc."""
        if "chunked" in self.headers.get("Transfer-Encoding", "").lower():
            data = b""
            while True:
                size = int(self.rfile.readline().strip().split(b";")[0], 16)
                if size == 0:
                    self.rfile.readline()
                    return data
                data += self.rfile.read(size)
                self.rfile.readline()
        return self.rfile.read(int(self.headers.get("Content-Length", 0)))

    def do_GET(self):
        if self.path == "/health":
            self._send(200, {"status": "ok", "service": "rmms-ai-scheduler"})
        else:
            self._send(404, {"error": "không có đường dẫn này"})

    def do_POST(self):
        if self.path != "/plan":
            return self._send(404, {"error": "không có đường dẫn này"})
        try:
            req = json.loads(self._body() or b"{}")
        except ValueError:
            return self._send(400, {"error": "body không phải JSON"})
        try:
            out = plan(req)
        except KeyError as e:
            return self._send(400, {"error": f"thiếu trường {e}"})
        except (ValueError, TypeError, AttributeError) as e:   # dữ liệu sai định dạng là lỗi bên gọi → 400, không phải 500
            return self._send(400, {"error": f"dữ liệu không hợp lệ: {e}"})
        except Exception as e:  # lỗi bất ngờ: báo 500, backend sẽ chuyển sang luật dự phòng
            traceback.print_exc()
            return self._send(500, {"error": str(e)})
        s = out["score"]
        print(f"/plan: {len(req.get('tasks', []))} việc, {len(req.get('robots', []))} robot → "
              f"điểm phạt nháp {s['draft']} → AI {s['final']} (-{s['improvementPct']}%), "
              f"trễ {s['draftLateTasks']} → {s['lateTasks']}, {out['meta']['elapsedMs']} ms", flush=True)
        self._send(200, out)

    def log_message(self, fmt, *args):   # tắt log mặc định cho gọn
        pass


def main():
    port = int(sys.argv[1]) if len(sys.argv) > 1 else 8000
    srv = ThreadingHTTPServer(("127.0.0.1", port), Handler)
    print(f"AI Scheduler đang chạy: http://127.0.0.1:{port}  (POST /plan, GET /health) — Ctrl+C để tắt", flush=True)
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
