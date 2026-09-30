#!/usr/bin/env python3
"""
MJPEG-стрим с камеры Gazebo + детекция ArUco-меток.
Запуск:  python3 gazebo_aruco_http.py
Браузер: http://localhost:8080
"""

import time
import json
import threading
import numpy as np
import cv2

from http.server import HTTPServer, BaseHTTPRequestHandler
from socketserver import ThreadingMixIn
from gz.transport13 import Node
from gz.msgs10.image_pb2 import Image


# ──────────────────────────────────────────────
#  Конфигурация
# ──────────────────────────────────────────────
CAMERA_TOPIC = "/iris/camera/image_raw"
HTTP_PORT = 8080

# Размеры для быстрой детекции (оригинал остаётся для стрима)
DETECT_WIDTH = 320
DETECT_HEIGHT = 240
DETECT_EVERY_N = 3   # детекция каждый N-й кадр

# ──────────────────────────────────────────────
#  Хранилище данных меток
# ──────────────────────────────────────────────
class MarkerDataStore:
    def __init__(self):
        self.detected_ids = []
        self.lock = threading.Lock()

    def update_ids(self, new_ids):
        with self.lock:
            if set(self.detected_ids) != set(new_ids):
                if new_ids:
                    print(f"[ArUco] Метки: {new_ids}")
                else:
                    print("[ArUco] Метки потеряны")
            self.detected_ids = new_ids

    def get_json(self):
        with self.lock:
            return json.dumps({
                "timestamp": time.time(),
                "count": len(self.detected_ids),
                "markers": self.detected_ids
            })


marker_store = MarkerDataStore()

# ──────────────────────────────────────────────
#  Потоковый буфер (condition variable)
# ──────────────────────────────────────────────
class FrameBuffer:
    def __init__(self):
        self.frame = None
        self.condition = threading.Condition()

    def write(self, frame):
        with self.condition:
            self.frame = frame
            self.condition.notify_all()

    def read(self, timeout=1.0):
        with self.condition:
            if not self.condition.wait(timeout=timeout):
                return None
            return self.frame


frame_buffer = FrameBuffer()

# ──────────────────────────────────────────────
#  Многопоточный HTTP-сервер
# ──────────────────────────────────────────────
class ThreadedHTTPServer(ThreadingMixIn, HTTPServer):
    daemon_threads = True


# ──────────────────────────────────────────────
#  HTTP-обработчик
# ──────────────────────────────────────────────
class StreamingHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path in ('/', '/index.html', '/stream'):
            self._serve_html()

        elif self.path == '/api/markers':
            self._serve_markers_json()

        elif self.path == '/video.mjpg':
            self._serve_mjpeg_stream()

        elif self.path == '/snapshot':
            self._serve_snapshot()

        else:
            self.send_error(404)
            self.end_headers()

    def _serve_html(self):
        content = """
        <!DOCTYPE html>
        <html>
        <head>
            <title>Gazebo ArUco Stream</title>
            <style>
                body {
                    background: #121212; color: #fff;
                    text-align: center; font-family: monospace;
                    margin: 0; padding: 20px;
                }
                img {
                    max-width: 100%;
                    border: 2px solid #00ff88;
                    border-radius: 8px;
                }
                #info {
                    margin-top: 15px; font-size: 1.2rem; color: #00ff88;
                }
                a {
                    color: #00ff88; text-decoration: none;
                    border: 1px solid #00ff88; padding: 5px 15px;
                    border-radius: 5px; display: inline-block; margin-top: 10px;
                }
            </style>
        </head>
        <body>
            <h2>Gazebo + ArUco Detector</h2>
            <img src="/video.mjpg" width="640" height="480" />
            <div id="info">Ожидание данных...</div>
            <div>
                <a href="/snapshot">Снимок</a>
                <a href="/api/markers">JSON API</a>
            </div>
            <script>
                setInterval(async () => {
                    try {
                        let res = await fetch('/api/markers');
                        let data = await res.json();
                        let info = document.getElementById('info');
                        if (data.count > 0) {
                            info.innerText =
                                `Найдено меток: ${data.count} — ID: [ ${data.markers.join(', ')} ]`;
                            info.style.color = '#00ff88';
                        } else {
                            info.innerText = 'Меток не обнаружено';
                            info.style.color = '#ff4444';
                        }
                    } catch(e) {}
                }, 500);
            </script>
        </body>
        </html>
        """.encode('utf-8')

        self.send_response(200)
        self.send_header('Content-Type', 'text/html; charset=utf-8')
        self.send_header('Content-Length', str(len(content)))
        self.end_headers()
        self.wfile.write(content)

    def _serve_markers_json(self):
        json_data = marker_store.get_json().encode('utf-8')
        self.send_response(200)
        self.send_header('Access-Control-Allow-Origin', '*')
        self.send_header('Content-Type', 'application/json')
        self.send_header('Content-Length', str(len(json_data)))
        self.end_headers()
        self.wfile.write(json_data)

    def _serve_mjpeg_stream(self):
        self.send_response(200)
        self.send_header('Access-Control-Allow-Origin', '*')
        self.send_header('Cache-Control', 'no-store, no-cache, must-revalidate')
        self.send_header(
            'Content-Type',
            'multipart/x-mixed-replace; boundary=FRAME'
        )
        self.end_headers()

        try:
            while True:
                frame = frame_buffer.read(timeout=1.0)
                if frame is None:
                    continue

                ret, jpeg = cv2.imencode(
                    '.jpg', frame,
                    [int(cv2.IMWRITE_JPEG_QUALITY), 80]
                )
                if not ret:
                    continue

                jpeg_bytes = jpeg.tobytes()
                self.wfile.write(b'--FRAME\r\n')
                self.send_header('Content-Type', 'image/jpeg')
                self.send_header('Content-Length', str(len(jpeg_bytes)))
                self.end_headers()
                self.wfile.write(jpeg_bytes)
                self.wfile.write(b'\r\n')

        except (BrokenPipeError, ConnectionResetError):
            pass  # клиент отключился

    def _serve_snapshot(self):
        frame = frame_buffer.read(timeout=1.0)
        if frame is None:
            self.send_error(503, "Кадр ещё не получен")
            return

        ret, jpeg = cv2.imencode('.jpg', frame)
        if not ret:
            self.send_error(500, "Ошибка кодирования")
            return

        jpeg_bytes = jpeg.tobytes()
        self.send_response(200)
        self.send_header('Content-Type', 'image/jpeg')
        self.send_header('Content-Length', str(len(jpeg_bytes)))
        self.end_headers()
        self.wfile.write(jpeg_bytes)

    def log_message(self, fmt, *args):
        # Подавляем шум, печатаем только ошибки
        if args and "404" in str(args[0]):
            print(f"[HTTP] 404 — {self.path}")


# ──────────────────────────────────────────────
#  Декодирование изображения из Gazebo
# ──────────────────────────────────────────────
def decode_gazebo_image(msg: Image) -> np.ndarray | None:
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
        return None

    if pixel_format == "rgb_int8":
        img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
    elif pixel_format == "rgba_int8":
        img = cv2.cvtColor(img, cv2.COLOR_RGBA2BGR)
    elif pixel_format == "bgra_int8":
        img = cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)
    elif pixel_format == "l_int8":
        img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)

    return img


# ──────────────────────────────────────────────
#  Инициализация ArUco-детектора
# ──────────────────────────────────────────────
def create_aruco_detector():
    """Создаёт детектор с оптимизированными параметрами."""
    try:
        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        aruco_params = cv2.aruco.DetectorParameters()

        # --- ОПТИМИЗАЦИЯ ---
        # Увеличиваем шаг окна адаптивного порога — меньше вычислений
        aruco_params.adaptiveThreshWinSizeStep = 10
        # Отключаем субпиксельное уточнение углов — экономит CPU
        aruco_params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_NONE

        detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)
        return detector, False  # legacy_mode = False

    except AttributeError:
        # Совместимость со старыми версиями OpenCV (<4.7)
        aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
        aruco_params = cv2.aruco.DetectorParameters_create()
        aruco_params.adaptiveThreshWinSizeStep = 10
        aruco_params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_NONE
        return (aruco_dict, aruco_params), True  # legacy_mode = True


# ──────────────────────────────────────────────
#  Поток: Gazebo → декодирование → ArUco → стрим
# ──────────────────────────────────────────────
def gazebo_aruco_worker():
    print("[Gazebo] Подключение к топику:", CAMERA_TOPIC)

    node = Node()
    subscribed = node.subscribe(Image, CAMERA_TOPIC, image_callback)
    if not subscribed:
        print(f"[Gazebo] Ошибка подписки на {CAMERA_TOPIC}")
        print("        Проверьте: gz topic -l | grep -i camera")
        return
    print("[Gazebo] Подписка создана, ожидаем кадры...")

    # Держим узел живым
    while True:
        time.sleep(0.001)


def image_callback(msg: Image) -> None:
    """Вызывается транспортом Gazebo при каждом новом кадре."""
    frame = decode_gazebo_image(msg)
    if frame is None:
        return

    process_frame(frame)


# ──────────────────────────────────────────────
#  Детекция и отрисовка (вызывается на каждый кадр)
# ──────────────────────────────────────────────
# Состояние детектора между кадрами
_detector, _legacy = create_aruco_detector()
_frame_count = 0
_last_corners = []
_last_ids = None

# Коэффициент масштабирования: детекция на 320×240, отрисовка на оригинале
_scale_x = 1.0
_scale_y = 1.0


def process_frame(frame: np.ndarray) -> None:
    global _frame_count, _last_corners, _last_ids
    global _scale_x, _scale_y

    h, w = frame.shape[:2]
    _frame_count += 1

    # ── Детекция каждые N кадров ──
    if _frame_count % DETECT_EVERY_N == 0:
        # 1. Сжимаем кадр для быстрого сканирования
        small = cv2.resize(
            frame, (DETECT_WIDTH, DETECT_HEIGHT),
            interpolation=cv2.INTER_NEAREST
        )
        gray_small = cv2.cvtColor(small, cv2.COLOR_BGR2GRAY)

        # 2. Поиск меток на уменьшенном кадре
        if not _legacy:
            corners, ids, _ = _detector.detectMarkers(gray_small)
        else:
            aruco_dict, aruco_params = _detector
            corners, ids, _ = cv2.aruco.detectMarkers(
                gray_small, aruco_dict, parameters=aruco_params
            )

        # 3. Масштабируем координаты углов обратно к оригиналу
        _scale_x = w / DETECT_WIDTH
        _scale_y = h / DETECT_HEIGHT

        current_ids = []
        if ids is not None:
            current_ids = [int(x) for x in ids.flatten()]
            _last_corners = [c * np.array([_scale_x, _scale_y]) for c in corners]
            _last_ids = ids
        else:
            _last_corners = []
            _last_ids = None

        marker_store.update_ids(current_ids)

    # ── Отрисовка на оригинальном кадре (каждый кадр) ──
    if _last_ids is not None and len(_last_corners) > 0:
        # drawDetectedMarkers рисует на переданном кадре
        cv2.aruco.drawDetectedMarkers(frame, _last_corners, _last_ids)

        # Подписываем ID у каждой метки
        for i, marker_id in enumerate(_last_ids.flatten()):
            c = _last_corners[i][0]
            cx = int((c[0][0] + c[2][0]) / 2)
            cy = int((c[0][1] + c[2][1]) / 2)
            cv2.putText(
                frame, f"ID: {marker_id}",
                (cx - 20, cy - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2
            )

    # ── Отправка кадра в буфер для HTTP-стрима ──
    frame_buffer.write(frame)


# ──────────────────────────────────────────────
#  Точка входа
# ──────────────────────────────────────────────
def main():
    # Поток чтения Gazebo + ArUco
    worker = threading.Thread(target=gazebo_aruco_worker, daemon=True)
    worker.start()

    # Даём время на подключение
    time.sleep(2)

    if frame_buffer.frame is None:
        print("[!] Кадры пока не получены. Проверьте:")
        print("    1. Gazebo запущен, мир загружен")
        print("    2. Топик существует: gz topic -l | grep -i camera")
        print(f"    3. Имя топика: {CAMERA_TOPIC}")

    server = ThreadedHTTPServer(("0.0.0.0", HTTP_PORT), StreamingHandler)
    print(f"\n[HTTP] Сервер: http://localhost:{HTTP_PORT}")
    print(f"[HTTP] Стрим:  http://localhost:{HTTP_PORT}/video.mjpg")
    print(f"[HTTP] Снимок: http://localhost:{HTTP_PORT}/snapshot")
    print(f"[HTTP] API:    http://localhost:{HTTP_PORT}/api/markers")
    print("Ctrl+C для выхода\n")

    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nЗавершение работы")
    finally:
        server.shutdown()


if __name__ == "__main__":
    main()
