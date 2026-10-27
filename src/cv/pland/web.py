"""Веб: страница, MJPEG-поток, /status.

Почему раньше «зависали квадраты»: HTTPServer был однопоточным и раздавал
бесконечный поток в одном потоке. Любой подвисший клиент (обновили страницу,
пропал Wi-Fi, браузер не закрыл соединение) блокировал сервер навсегда —
новые подключения не обслуживались, на экране оставался последний кадр.
Плюс каждый клиент копировал и кодировал кадр заново.

Теперь: ThreadingHTTPServer, лимит клиентов, один энкодер на всех,
таймауты сокетов, обрыв «мёртвого» потока и автопереподключение страницы.
"""

from __future__ import annotations

import json
import socket
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

from .config import Config
from .hub import FrameHub, Status
from .log import log

INDEX_HTML = """<!doctype html>
<html lang="ru"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Precision Landing</title>
<style>
 body{background:#111;color:#ddd;font-family:ui-monospace,monospace;text-align:center;margin:0;padding:12px}
 h1{font-size:15px;font-weight:600;color:#0f8;margin:4px 0 10px}
 img{max-width:100%;border:2px solid #0f8;border-radius:6px;display:block;margin:0 auto}
 #s{margin-top:10px;font-size:13px;color:#8f8;white-space:pre-line}
</style></head><body>
<h1>Precision Landing — ArUco</h1>
<img id="v" alt="stream" src="/stream">
<div id="s">подключение…</div>
<script>
var v=document.getElementById('v'), s=document.getElementById('s');
function reload(){ setTimeout(function(){ v.src='/stream?t='+Date.now(); }, 1000); }
v.onerror=function(){ s.textContent='нет потока, переподключение…'; reload(); };
setInterval(function(){
  fetch('/status',{cache:'no-store'}).then(function(r){return r.json();}).then(function(j){
    s.textContent='ID '+j.marker+'   ang '+j.angle_x.toFixed(3)+' / '+j.angle_y.toFixed(3)+
      '   alt '+j.alt.toFixed(2)+' м\\nlink '+(j.link?'OK':'LOST')+
      '   '+j.fps.toFixed(1)+' fps   клиентов '+j.clients+(j.last_error?('\\n'+j.last_error):'');
  }).catch(function(){ s.textContent='нет /status'; });
},1000);
</script></body></html>"""


class ServerContext:
    def __init__(self, cfg: Config, hub: FrameHub, status: Status):
        self.cfg = cfg
        self.hub = hub
        self.status = status


class StreamHandler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.0"
    server_version = "pland-v2/1.0"
    ctx: ServerContext = None  # подставляется перед созданием сервера

    def log_message(self, fmt, *args):  # тишина в консоли
        pass

    def _send_plain(
        self, code: int, body: bytes, ctype: str = "text/plain; charset=utf-8"
    ) -> None:
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        try:
            self.wfile.write(body)
        except OSError:
            pass

    # -- маршруты ------------------------------------------------------------
    def do_GET(self) -> None:
        path = self.path.split("?", 1)[0].rstrip("/") or "/"
        if path == "/":
            self._send_plain(200, INDEX_HTML.encode("utf-8"), "text/html; charset=utf-8")
        elif path in ("/stream", "/video.mjpg"):
            self._stream()
        elif path == "/status":
            body = json.dumps(self.ctx.status.snapshot(), ensure_ascii=False).encode("utf-8")
            self._send_plain(200, body, "application/json; charset=utf-8")
        elif path == "/healthz":
            self._send_plain(200, b"ok")
        else:
            self._send_plain(404, b"not found")

    # -- MJPEG ---------------------------------------------------------------
    def _stream(self) -> None:
        hub = self.ctx.hub
        if not hub.acquire_client():
            self._send_plain(503, b"too many clients")
            return
        try:
            try:
                self.connection.settimeout(self.ctx.cfg.client_timeout_s)
                self.connection.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                self.connection.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 65536)
            except OSError:
                pass

            self.send_response(200)
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.send_header("Cache-Control", "no-store, no-cache, must-revalidate, max-age=0")
            self.send_header("Pragma", "no-cache")
            self.send_header("Connection", "close")
            self.send_header("X-Accel-Buffering", "no")
            self.end_headers()

            last_seq = -1
            last_progress = time.monotonic()
            while True:
                jpeg, seq = hub.wait_jpeg(last_seq, timeout=1.0)
                if jpeg is None or seq == last_seq:
                    # кадров нет: если главный цикл жив — появятся; если нет,
                    # рвём соединение, чтобы страница переподключилась
                    if time.monotonic() - last_progress > self.ctx.cfg.idle_close_s:
                        break
                    continue

                header = (
                    b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: "
                    + str(len(jpeg)).encode()
                    + b"\r\n\r\n"
                )
                # одна запись = меньше сисколлов и никаких «склеек» по Nagle
                self.wfile.write(header + jpeg + b"\r\n")
                last_seq = seq
                last_progress = time.monotonic()
        except (BrokenPipeError, ConnectionResetError, TimeoutError, OSError):
            pass
        finally:
            self.close_connection = True
            hub.release_client()


class StreamServer(ThreadingHTTPServer):
    daemon_threads = True
    allow_reuse_address = True
    request_queue_size = 32


def start_http_server(cfg: Config, hub: FrameHub, status: Status) -> StreamServer:
    StreamHandler.ctx = ServerContext(cfg, hub, status)
    server = StreamServer(("0.0.0.0", cfg.http_port), StreamHandler)
    threading.Thread(target=server.serve_forever, name="http", daemon=True).start()
    log(f"[HTTP] http://0.0.0.0:{cfg.http_port}  (/ , /stream, /status)")
    return server
