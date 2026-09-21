import io
import os
import time
import cv2
import json
import threading
from datetime import datetime
from http.server import HTTPServer, BaseHTTPRequestHandler
from socketserver import ThreadingMixIn
from picamera2 import Picamera2

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
PHOTO_FOLDER = os.path.join(BASE_DIR, "photos")
os.makedirs(PHOTO_FOLDER, exist_ok=True)


class MarkerDataStore:
    def __init__(self):
        self.detected_ids = []
        self.lock = threading.Lock()

    def update_ids(self, new_ids):
        with self.lock:
            if set(self.detected_ids) != set(new_ids):
                if new_ids:
                    print(f"[INFO SERVER] Метки: {new_ids}")
                else:
                    print("[INFO SERVER] Метки потеряны")
            self.detected_ids = new_ids

    def get_json(self):
        with self.lock:
            return json.dumps(
                {
                    "timestamp": time.time(),
                    "count": len(self.detected_ids),
                    "markers": self.detected_ids,
                }
            )


marker_store = MarkerDataStore()


class ThreadedHTTPServer(ThreadingMixIn, HTTPServer):
    daemon_threads = True


class StreamingOutput:
    def __init__(self):
        self.frame = None
        self.condition = threading.Condition()

    def write(self, frame):
        with self.condition:
            self.frame = frame
            self.condition.notify_all()


output = StreamingOutput()


class StreamingHandler(BaseHTTPRequestHandler):

    def send_html_file(self, filepath: str, status_code: int = 200) -> None:
        """Вспомогательный метод для отправки HTML-файла."""
        if not os.path.exists(filepath):
            self.send_error_response("404: Страница не найдена", status=404)
            return

        try:
            with open(filepath, "rb") as f:
                content = f.read()

            self.send_response(status_code)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(content)))
            self.end_headers()

            self.wfile.write(content)
        except Exception as e:
            self.send_error_response(f"500: Ошибка сервера ({e})", status=500)

    def send_error_response(self, message: str, status: int = 404) -> None:
        """Метод отправки ошибок."""
        self.send_response(status)
        self.send_header("Content-Type", "text/plain; charset=utf-8")
        self.end_headers()
        self.wfile.write(message.encode("utf-8"))

    def do_GET(self):
        if self.path in ("/", "/index.html", "/stream"):
            self.send_html_file("web/index.html")

        elif self.path == "/api/markers":
            json_data = marker_store.get_json().encode("utf-8")
            self.send_response(200)
            self.send_header("Access-Control-Allow-Origin", "*")
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(json_data)))
            self.end_headers()
            self.wfile.write(json_data)

        elif self.path == "/video.mjpg":
            self.send_response(200)
            self.send_header("Access-Control-Allow-Origin", "*")
            self.send_header("Cache-Control", "no-store, no-cache, must-revalidate")
            self.send_header(
                "Content-Type", "multipart/x-mixed-replace; boundary=FRAME"
            )
            self.end_headers()

            try:
                while True:
                    with output.condition:
                        if not output.condition.wait(timeout=1.0):
                            continue
                        frame = output.frame

                    if frame is None:
                        continue

                    # Сжатие с качеством 50 вместо 60 — дает дополнительную экономию CPU
                    ret, jpeg = cv2.imencode(
                        ".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), 50]
                    )
                    if not ret:
                        continue

                    jpeg_bytes = jpeg.tobytes()
                    self.wfile.write(b"--FRAME\r\n")
                    self.send_header("Content-Type", "image/jpeg")
                    self.send_header("Content-Length", str(len(jpeg_bytes)))
                    self.end_headers()
                    self.wfile.write(jpeg_bytes)
                    self.wfile.write(b"\r\n")
            except Exception:
                pass
        else:
            self.send_error(404)
            self.end_headers()

    def do_POST(self):
        if self.path == "/capture":
            try:
                with output.condition:
                    if not output.condition.wait(timeout=1.0):
                        self.send_error_response("500: Timeout", 500)
                        return
                    frame = output.frame

                if frame is None:
                    self.send_error_response("500: Timeout", 500)
                    return

                filename = datetime.now().strftime("photo_%Y%m%d_%H%M%S.jpg")
                filepath = os.path.join(PHOTO_FOLDER, filename)
                cv2.imwrite(filepath, frame)

                response = f'{{"status": "ok", "filename": "{filename}"}}'.encode("utf-8")
                self.send_response(200)
                self.send_header("Content-Type", "application/json")
                self.send_header("Content-Length", str(len(response)))
                self.end_headers()
                self.wfile.write(response)
            except Exception:
                self.send_error_response("500: Saving Error", 500)
        else:
            self.send_error(404)
            self.end_headers()


def camera_worker():
    print("[CAM] Инициализация оптимизированной камеры...")
    picam2 = Picamera2()

    # 640x480 — хорошая картинка для стрима
    config = picam2.create_video_configuration(
        main={"size": (640, 480), "format": "BGR888"}
    )
    picam2.configure(config)
    picam2.start()

    while True:
        try:
            frame = picam2.capture_array()
            output.write(frame)

            # Небольшой микро-отдых для CPU
            time.sleep(0.005)

        except Exception as e:
            print(f"[CAM ERROR] {e}")
            time.sleep(0.05)

    picam2.stop()


def main():
    cam_thread = threading.Thread(target=camera_worker, daemon=True)
    cam_thread.start()

    server_address = ("", 8080)
    httpd = ThreadedHTTPServer(server_address, StreamingHandler)
    print("[HTTP] Оптимизированный сервер запущен на 8080...")

    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        httpd.server_close()


if __name__ == "__main__":
    main()
