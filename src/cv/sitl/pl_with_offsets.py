import io
import time
import math
import threading
import numpy as np
import cv2 as cv
from http.server import HTTPServer, BaseHTTPRequestHandler
from pymavlink import mavutil
from gz.transport13 import Node
from gz.msgs10.image_pb2 import Image

# --- Gazebo & HTTP Config ---
CAMERA_TOPIC = "/iris/camera/image_raw"
HTTP_PORT = 8080

# --- User config ---
FRAME_W, FRAME_H = 640, 480
ARUCO_DICT = cv.aruco.DICT_4X4_50

# Приоритетные метки и их размеры (в метрах)
TARGET_MARKERS = {
    22: 0.034,  # 34 мм — приоритет 1 (главная цель посадки, центр координат)
    33: 0.067,  # 67 мм — приоритет 2 (средняя высота)
    44: 0.126,  # 126 мм — приоритет 3 (большая высота)
}
PRIORITY_ORDER = [22, 33, 44]

# Смещение центра каждой метки относительно центра ID 22 (в метрах): (dx, dy)
# dx - вперед/назад, dy - влево/вправо (зависит от ориентации камеры)
OFFSETS = {
    22: (0.0, 0.0),
    33: (0.0645, 0.0),
    44: (0.036, -0.105)
}

# --- MAVLink Config (SITL) ---
SERIAL_IP = "udpin:127.0.0.1:14551"
USE_FULL_POSE = False  # True: расчет xyz + position_valid=1; False: только углы

# --- Load calibration ---
fs = cv.FileStorage('../calibration/camera.yaml', cv.FILE_STORAGE_READ)
K = fs.getNode('camera_matrix').mat()
D = fs.getNode('distortion_coefficients').mat()
fs.release()

# --- Global Frame & Distance Storage ---
latest_frame = None
latest_jpeg = None
frame_lock = threading.Lock()
current_distance = 0.0
distance_lock = threading.Lock()

def image_callback(msg: Image) -> None:
    """Callback-функция приема кадров из топика Gazebo."""
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
        img = cv.cvtColor(img, cv.COLOR_RGB2BGR)
    elif pixel_format == "rgba_int8":
        img = cv.cvtColor(img, cv.COLOR_RGBA2BGR)
    elif pixel_format == "bgra_int8":
        img = cv.cvtColor(img, cv.COLOR_BGRA2BGR)
    elif pixel_format == "l_int8":
        img = cv.cvtColor(img, cv.COLOR_GRAY2BGR)

    with frame_lock:
        latest_frame = img

def gazebo_listener():
    """Поток подписки на топик Gazebo."""
    node = Node()
    if node.subscribe(Image, CAMERA_TOPIC, image_callback):
        print(f"[Gazebo] Подписка на {CAMERA_TOPIC} успешно создана")
    else:
        print(f"[Gazebo] Ошибка подписки на {CAMERA_TOPIC}")
        return
    while True:
        time.sleep(0.001)

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
        elif self.path == "/snapshot":
            with frame_lock:
                jpg_data = latest_jpeg
            if jpg_data is None:
                self.send_error(503, "Кадр ещё не получен")
                return
            self.send_response(200)
            self.send_header("Content-Type", "image/jpeg")
            self.send_header("Content-Length", str(len(jpg_data)))
            self.end_headers()
            self.wfile.write(jpg_data)
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
    """Отдельный поток для постоянного считывания высоты из MAVLink."""
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

# --- Запуск фоновых потоков ---
threading.Thread(target=gazebo_listener, daemon=True).start()
threading.Thread(target=run_http_server, daemon=True).start()
threading.Thread(target=mavlink_altitude_listener, daemon=True).start()

# --- Инициализация ArUco ---
aruco = cv.aruco
dict_ = aruco.getPredefinedDictionary(ARUCO_DICT)
params = aruco.DetectorParameters()
detector = aruco.ArucoDetector(dict_, params)

def center_from_corners(corners):
    pts = corners.reshape(-1, 2)
    c = pts.mean(axis=0)
    return float(c[0]), float(c[1])

fx, fy = K[0, 0], K[1, 1]
cx, cy = K[0, 2], K[1, 2]

send_rate_hz = 20.0
send_period = 1.0 / send_rate_hz
next_send_time = time.time()

log_rate_hz = 2.0
log_period = 1.0 / log_rate_hz
next_log_time = time.time()

# --- Main Processing Loop ---
while True:
    with frame_lock:
        if latest_frame is None:
            frame = None
        else:
            frame = latest_frame.copy()

    if frame is None:
        time.sleep(0.01)
        continue

    draw_frame = frame.copy()
    corners, ids, _ = detector.detectMarkers(frame)

    if ids is not None:
        aruco.drawDetectedMarkers(draw_frame, corners, ids)
        detected_ids = ids.flatten()

        # Выбор метки по приоритету
        active_id = None
        for candidate_id in PRIORITY_ORDER:
            if candidate_id in detected_ids:
                active_id = candidate_id
                break

        if active_id is not None:
            tag_size = TARGET_MARKERS[active_id]
            idx = np.where(detected_ids == active_id)[0][0]
            marker_corners = corners[idx]

            u, v = center_from_corners(marker_corners)

            # --- Компенсация смещения относительно ID 22 ---
            dx_m, dy_m = OFFSETS.get(active_id, (0.0, 0.0))
            if dx_m != 0.0 or dy_m != 0.0:
                with distance_lock:
                    alt_eff = max(current_distance, 0.3) # Защита от деления на 0
                # Переводим физическое смещение в пиксельное на текущей высоте
                u += (dx_m * fx) / alt_eff
                v += (dy_m * fy) / alt_eff

            angle_x = math.atan2((u - cx) / fx, 1.0)
            angle_y = math.atan2((v - cy) / fy, 1.0)

            x_b = y_b = z_b = 0.0
            position_valid = 0

            if USE_FULL_POSE:
                obj_pts = np.array([
                    [-tag_size / 2, tag_size / 2, 0],
                    [tag_size / 2, tag_size / 2, 0],
                    [tag_size / 2, -tag_size / 2, 0],
                    [-tag_size / 2, -tag_size / 2, 0],
                ], dtype=np.float32)
                img_pts = marker_corners.reshape(-1, 2).astype(np.float32)
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
                with distance_lock:
                    dist = current_distance

                m.mav.landing_target_send(
                    int(tnow * 1e6),
                    0,
                    mavutil.mavlink.MAV_FRAME_BODY_FRD,
                    float(angle_x),
                    float(angle_y),
                    dist,                   # distance (высота до земли в метрах)
                    tag_size,
                    tag_size,
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
                print(
                    f"[TARGET LOCK] ID: {active_id} ({tag_size*1000:.0f}mm) | "
                    f"AngX: {angle_x:.3f} rad | AngY: {angle_y:.3f} rad | "
                    f"MAV Alt: {telemetry_alt:.2f} m"
                )

    # Кодирование кадра с метками в JPEG и обновление глобальной переменной для HTTP
    _, jpg = cv.imencode(".jpg", draw_frame, [cv.IMWRITE_JPEG_QUALITY, 80])
    with frame_lock:
        latest_jpeg = jpg.tobytes()