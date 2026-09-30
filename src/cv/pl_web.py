import io
import time
import math
import threading
import numpy as np
import cv2 as cv
from http.server import HTTPServer, BaseHTTPRequestHandler
from pymavlink import mavutil
from picamera2 import Picamera2

HTTP_PORT = 8080

# --- User config ---
FRAME_W, FRAME_H = 640, 480
ARUCO_DICT = cv.aruco.DICT_4X4_50

# Конфигурация посадочной доски: ID и физический размер стороны в метрах
TARGET_MARKERS = {
    22: 0.034,  # 34 мм — приоритет 1 (главная цель посадки)
    33: 0.067,  # 67 мм — приоритет 2 (средняя высота)
    44: 0.126   # 126 мм — приоритет 3 (большая высота)
}

# Порядок приоритета: от самой точной к самой крупной
PRIORITY_ORDER = [22, 33, 44]

SERIAL_IP = "tcp:127.0.0.1:5602"  # Port связи с полетником
BAUD = 921600
USE_FULL_POSE = False   # True: расчет xyz + position_valid=1; False: только углы

# --- Load calibration ---
fs = cv.FileStorage('calibration/camera.yaml', cv.FILE_STORAGE_READ)
K = fs.getNode('camera_matrix').mat()
D = fs.getNode('distortion_coefficients').mat()
fs.release()

# --- Global Storage & Locks ---
latest_jpeg = None
frame_lock = threading.Lock()

current_distance = 0.0
distance_lock = threading.Lock()


class MJPEGHandler(BaseHTTPRequestHandler):
    """HTTP-сервер для видеопотока."""
    def do_GET(self):
        if self.path in ("/", "/stream"):
            self.send_response(200)
            self.send_header("Age", 0)
            self.send_header("Cache-Control", "no-cache, private")
            self.send_header("Pragma", "no-cache")
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.end_headers()

            while True:
                with frame_lock:
                    jpg_data = latest_jpeg

                if jpg_data is None:
                    time.sleep(0.05)
                    continue

                try:
                    self.wfile.write(b"--frame\r\n")
                    self.wfile.write(b"Content-Type: image/jpeg\r\n")
                    self.wfile.write(f"Content-Length: {len(jpg_data)}\r\n".encode())
                    self.wfile.write(b"\r\n")
                    self.wfile.write(jpg_data)
                    self.wfile.write(b"\r\n")
                    time.sleep(0.03)
                except (BrokenPipeError, ConnectionResetError):
                    break
        else:
            self.send_error(404)

    def log_message(self, format, *args):
        pass


def run_http_server():
    server = HTTPServer(("0.0.0.0", HTTP_PORT), MJPEGHandler)
    print(f"[HTTP] Сервер запущен на http://localhost:{HTTP_PORT}")
    server.serve_forever()


# --- MAVLink connection ---
m = mavutil.mavlink_connection(SERIAL_IP)
try:
    m.wait_heartbeat(timeout=5)
    print("[MAVLink] Heartbeat получен")
except Exception:
    print("[MAVLink] Heartbeat не получен, продолжаем...")


def mavlink_altitude_listener():
    """Фоновый поток считывания высоты по MAVLink."""
    global current_distance
    while True:
        msg = m.recv_match(
            type=['DISTANCE_SENSOR', 'LOCAL_POSITION_NED', 'GLOBAL_POSITION_INT'],
            blocking=True,
            timeout=0.5
        )
        if msg is None:
            continue

        msg_type = msg.get_type()
        alt = None

        if msg_type == 'DISTANCE_SENSOR':
            alt = msg.current_distance / 100.0
        elif msg_type == 'LOCAL_POSITION_NED':
            alt = -msg.z
        elif msg_type == 'GLOBAL_POSITION_INT':
            alt = msg.relative_alt / 1000.0

        if alt is not None and alt >= 0:
            with distance_lock:
                current_distance = float(alt)


threading.Thread(target=run_http_server, daemon=True).start()
threading.Thread(target=mavlink_altitude_listener, daemon=True).start()

# --- Инициализация камеры ---
print("[CAM] Инициализация Picamera2...")
picam2 = Picamera2()
config = picam2.create_video_configuration(main={"size": (FRAME_W, FRAME_H), "format": "RGB888"})
picam2.configure(config)
picam2.start()

# --- Инициализация ArUco ---
dict_ = cv.aruco.getPredefinedDictionary(ARUCO_DICT)
params = cv.aruco.DetectorParameters()
detector = cv.aruco.ArucoDetector(dict_, params)

def center_from_corners(corners):
    pts = corners.reshape(-1, 2)
    c = pts.mean(axis=0)
    return float(c[0]), float(c[1])

fx, fy = K[0,0], K[1,1]
cx, cy = K[0,2], K[1,2]

send_rate_hz = 20.0
send_period = 1.0 / send_rate_hz
next_send_time = time.time()

log_rate_hz = 2.0
log_period = 1.0 / log_rate_hz
next_log_time = time.time()

while True:
    rgb_frame = picam2.capture_array()
    frame = cv.cvtColor(rgb_frame, cv.COLOR_RGB2BGR)
    draw_frame = frame.copy()

    corners, ids, _ = detector.detectMarkers(frame)

    if ids is not None:
        cv.aruco.drawDetectedMarkers(draw_frame, corners, ids)
        detected_ids = ids.flatten()

        # 1. Поиск наиболее приоритетной метки из доступных на кадре
        active_id = None
        for candidate_id in PRIORITY_ORDER:
            if candidate_id in detected_ids:
                active_id = candidate_id
                break

        # 2. Если найдена хотя бы одна целевая метка из нашего списка
        if active_id is not None:
            tag_size = TARGET_MARKERS[active_id]
            idx = np.where(detected_ids == active_id)[0][0]
            marker_corners = corners[idx]

            # Вычисление углов относительно оптической оси
            u, v = center_from_corners(marker_corners)
            angle_x = math.atan2((u - cx) / fx, 1.0)
            angle_y = math.atan2((v - cy) / fy, 1.0)

            x_b = y_b = z_b = 0.0
            position_valid = 0

            if USE_FULL_POSE:
                obj_pts = np.array([
                    [-tag_size/2,  tag_size/2, 0],
                    [ tag_size/2,  tag_size/2, 0],
                    [ tag_size/2, -tag_size/2, 0],
                    [-tag_size/2, -tag_size/2, 0],
                ], dtype=np.float32)
                img_pts = marker_corners.reshape(-1,2).astype(np.float32)
                okp, rvec, tvec = cv.solvePnP(obj_pts, img_pts, K, D, flags=cv.SOLVEPNP_IPPE_SQUARE)
                if okp:
                    x_b = float(tvec[2])
                    y_b = float(tvec[0])
                    z_b = float(tvec[1])
                    position_valid = 1
                    angle_x = math.atan2(y_b, x_b)
                    angle_y = math.atan2(z_b, x_b)

            tnow = time.time()

            # Отправка MAVLink сообщения (20 Гц)
            if tnow >= next_send_time:
                next_send_time += send_period

                m.mav.landing_target_send(
                    int(tnow * 1e6),
                    0,
                    mavutil.mavlink.MAV_FRAME_BODY_FRD,
                    float(angle_x), float(angle_y),
                    0.0,                # distance (0.0 — работа с бортовым дальномером)
                    tag_size, tag_size, # size_x, size_y текущей метки
                    x_b, y_b, z_b,
                    [1.0, 0.0, 0.0, 0.0],
                    mavutil.mavlink.LANDING_TARGET_TYPE_VISION_FIDUCIAL,
                    position_valid
                )

            # Вывод логов в консоль (2 Гц)
            if tnow >= next_log_time:
                next_log_time = tnow + log_period
                with distance_lock:
                    telemetry_alt = current_distance
                print(f"[TARGET LOCK] ID: {active_id} ({tag_size*1000:.0f}mm) | AngX: {angle_x:.3f} rad | AngY: {angle_y:.3f} rad | MAV Alt: {telemetry_alt:.2f} m")

    # Кодирование кадра для веб-сервера
    _, jpg = cv.imencode(".jpg", draw_frame, [cv.IMWRITE_JPEG_QUALITY, 80])
    with frame_lock:
        latest_jpeg = jpg.tobytes()

    time.sleep(0.005)