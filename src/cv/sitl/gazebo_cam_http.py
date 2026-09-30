"""
MJPEG-стрим с камеры Gazebo через HTTP.
Запуск: python3 gazebo_cam_http.py
Открыть в браузере: http://localhost:8080
"""

import io
import time
import threading
import numpy as np
import cv2

from http.server import HTTPServer, BaseHTTPRequestHandler
from gz.transport13 import Node
from gz.msgs10.image_pb2 import Image


CAMERA_TOPIC = "/iris/camera/image_raw"
HTTP_PORT = 8080

# Хранилище последнего кадра
latest_frame = None
frame_lock = threading.Lock()


def image_callback(msg: Image) -> None:
    global latest_frame

    width = msg.width
    height = msg.height
    step = msg.step
    pixel_format = msg.pixel_format_type

    raw = np.frombuffer(msg.data, dtype=np.uint8)

    channels_map = {
        "rgb_int8": 3, "rgba_int8": 4, "l_int8": 1,
        "bgr_int8": 3, "bgra_int8": 4,
    }
    channels = channels_map.get(pixel_format, 3)

    try:
        img = raw.reshape((height, max(step // channels, 1), channels))
        img = img[:, :width, :]
    except ValueError:
        return

    if pixel_format == "rgb_int8":
        img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
    elif pixel_format == "rgba_int8":
        img = cv2.cvtColor(img, cv2.COLOR_RGBA2BGR)
    elif pixel_format == "bgra_int8":
        img = cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)
    elif pixel_format == "l_int8":
        img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)

    # Кодируем в JPEG для стриминга
    _, jpg = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 80])

    with frame_lock:
        latest_frame = jpg.tobytes()


def gazebo_listener():
    """Поток подписки на топик Gazebo."""
    node = Node()
    if node.subscribe(Image, CAMERA_TOPIC, image_callback):
        print(f"[Gazebo] Подписка на {CAMERA_TOPIC} создана")
    else:
        print(f"[Gazebo] Ошибка подписки на {CAMERA_TOPIC}")
        return

    while True:
        time.sleep(0.001)


class MJPEGHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path == "/" or self.path == "/stream":
            self.send_response(200)
            self.send_header("Age", 0)
            self.send_header("Cache-Control", "no-cache, private")
            self.send_header("Pragma", "no-cache")
            self.send_header(
                "Content-Type",
                "multipart/x-mixed-replace; boundary=frame",
            )
            self.end_headers()

            while True:
                with frame_lock:
                    frame = latest_frame

                if frame is None:
                    time.sleep(0.1)
                    continue

                try:
                    self.send_frame(frame)
                except (BrokenPipeError, ConnectionResetError):
                    break  # клиент отключился

        elif self.path == "/snapshot":
            # Одиночный кадр в виде JPEG
            with frame_lock:
                frame = latest_frame

            if frame is None:
                self.send_error(503, "Кадр ещё не получен")
                return

            self.send_response(200)
            self.send_header("Content-Type", "image/jpeg")
            self.send_header("Content-Length", str(len(frame)))
            self.end_headers()
            self.wfile.write(frame)

        else:
            self.send_error(404)

    def send_frame(self, frame: bytes):
        self.wfile.write(b"--frame\r\n")
        self.wfile.write(b"Content-Type: image/jpeg\r\n")
        self.wfile.write(f"Content-Length: {len(frame)}\r\n".encode())
        self.wfile.write(b"\r\n")
        self.wfile.write(frame)
        self.wfile.write(b"\r\n")

    def log_message(self, format, *args):
        # Тишина в консоли, кроме первого подключения
        if "404" not in (args[1] if len(args) > 1 else ""):
            print(f"[HTTP] {args[0]} — {args[1] if len(args) > 1 else ''}")


def main():
    # Запускаем слушатель Gazebo в отдельном потоке
    listener_thread = threading.Thread(target=gazebo_listener, daemon=True)
    listener_thread.start()

    # Даём немного времени на подключение
    time.sleep(1)

    server = HTTPServer(("0.0.0.0", HTTP_PORT), MJPEGHandler)
    print(f"[HTTP] Сервер запущен на http://localhost:{HTTP_PORT}")
    print(f"[HTTP] Стрим:    http://localhost:{HTTP_PORT}/stream")
    print(f"[HTTP] Снимок:   http://localhost:{HTTP_PORT}/snapshot")
    print("Ctrl+C для выхода")

    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nЗавершение работы")
    finally:
        server.shutdown()


if __name__ == "__main__":
    main()
