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

TARGET_MARKERS = {
    22: 0.034,  # 34 мм — приоритет 1 (главная цель посадки)
    33: 0.067,  # 67 мм — приоритет 2 (средняя высота)
    44: 0.126,  # 126 мм — приоритет 3 (большая высота)
}
PRIORITY_ORDER = [22, 33, 44]
# Смещение центра каждой метки относительно центра ID 22 (в метрах): (dx, dy)
OFFSETS = {
    22: (0.0, 0.0),
    33: (0.0645, 0.0),
    44: (0.036, -0.105)
}
# SERIAL_IP = "tcp:127.0.0.1:5601"
SERIAL_IP = "udpout:127.0.0.1:14551"

BAUD = 921600
USE_FULL_POSE = False  # True: расчет xyz + position_valid=1; False: только углы

# Цвет фона для отладочного стрима: (0, 0, 0) - черный, (255, 255, 255) - белый
CANVAS_BG_COLOR = (0, 0, 0)

# --- Load calibration ---
fs = cv.FileStorage("calibration/camera.yaml", cv.FILE_STORAGE_READ)
K = fs.getNode("camera_matrix").mat()
D = fs.getNode("distortion_coefficients").mat()
fs.release()

# --- Global Storage & Locks ---
latest_frame = None
frame_lock = threading.Lock()

current_distance = 0.0
distance_lock = threading.Lock()


class MJPEGHandler(BaseHTTPRequestHandler):
    """HTTP-сервер для легковесного векторного видеопотока."""

    def do_GET(self):
        if self.path in ("/", "/stream"):
            self.send_response(200)
            self.send_header("Age", "0")
            self.send_header("Cache-Control", "no-cache, private")
            self.send_header("Pragma", "no-cache")
            self.send_header(
                "Content-Type", "multipart/x-mixed-replace; boundary=frame"
            )
            self.end_headers()

            while True:
                with frame_lock:
                    frame_to_send = None if latest_frame is None else latest_frame.copy()

                if frame_to_send is None:
                    time.sleep(0.04)
                    continue

                # Кодирование происходит только в потоке клиента при наличии подключения
                ok, jpg = cv.imencode(".jpg", frame_to_send, [cv.IMWRITE_JPEG_QUALITY, 60])
                if not ok:
                    time.sleep(0.02)
                    continue

                jpg_data = jpg.tobytes()

                try:
                    self.wfile.write(b"--frame\r\n")
                    self.wfile.write(b"Content-Type: image/jpeg\r\n")
                    self.wfile.write(f"Content-Length: {len(jpg_data)}\r\n".encode())
                    self.wfile.write(b"\r\n")
                    self.wfile.write(jpg_data)
                    self.wfile.write(b"\r\n")
                    # Ограничение стрима до ~20 FPS для экономии полосы пропускания и CPU
                    time.sleep(0.05)
                except (BrokenPipeError, ConnectionResetError, TimeoutError, OSError):
                    break
        else:
            self.send_error(404)

    def log_message(self, format, *args):
        pass


class QuickTimeoutServer(HTTPServer):
    """Сервер с установкой таймаута на клиентские сокеты во избежание подвисаний."""
    def finish_request(self, request, client_address):
        try:
            request.settimeout(1.5)
        except Exception:
            pass
        super().finish_request(request, client_address)


def run_http_server():
    server = QuickTimeoutServer(("0.0.0.0", HTTP_PORT), MJPEGHandler)
    print(f"[HTTP] Сервер запущен на http://0.0.0.0:{HTTP_PORT}")
    server.serve_forever()


def check_connection(master_instance, timeout=5):
    """Waits for heartbeat and returns True/False if connected/disconnected."""
    start_time = time.time()
    while time.time() - start_time <= timeout:
        master_instance.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
            mavutil.mavlink.MAV_AUTOPILOT_INVALID,
            0, 0, 0
        )

        msg = master_instance.wait_heartbeat(timeout=1.0)
        if msg is not None and master_instance.target_system != 0:
            print(
                f"[Connected] System ID: {master_instance.target_system}, Component ID: {master_instance.target_component}"
            )
            return True
        else:
            print(
                f"[Wrong ID] System ID: {master_instance.target_system} Trying again..."
            )
    print("[Timeout] Connection failed")
    return False


# --- MAVLink connection ---
m = mavutil.mavlink_connection(SERIAL_IP)
while not check_connection(m):
    print("[MAVLink] Reconnecting...")


def mavlink_altitude_listener():
    """Фоновый поток считывания высоты по MAVLink."""
    global current_distance
    while True:
        msg = m.recv_match(
            type=["DISTANCE_SENSOR", "LOCAL_POSITION_NED", "GLOBAL_POSITION_INT"],
            blocking=True,
            timeout=0.5,
        )
        if msg is None:
            continue

        msg_type = msg.get_type()
        alt = None

        if msg_type == "LOCAL_POSITION_NED":
            alt = -msg.z

        if alt is not None and alt >= 0:
            with distance_lock:
                current_distance = float(alt)


threading.Thread(target=run_http_server, daemon=True).start()
threading.Thread(target=mavlink_altitude_listener, daemon=True).start()

# --- Инициализация камеры ---
print("[CAM] Инициализация Picamera2...")
picam2 = Picamera2()

# config = picam2.create_video_configuration(
#     main={"size": (FRAME_W, FRAME_H), "format": "RGB888"}
# )
# picam2.configure(config)

config = picam2.create_configuration()
config["main"] = {"size": (FRAME_W, FRAME_H), "format": "RGB888"}

picam2.configure(config)

picam2.set_controls({
    "AfMode": controls.AfModeEnum.Continuous,
    "AfSpeed": controls.AfSpeedEnum.Fast,
})

picam2.start()

# --- Инициализация ArUco ---
dict_ = cv.aruco.getPredefinedDictionary(ARUCO_DICT)
params = cv.aruco.DetectorParameters()
detector = cv.aruco.ArucoDetector(dict_, params)


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

while True:
    rgb_frame = picam2.capture_array()
    # Детекция может проводиться напрямую по серому/RGB кадру, минимизируя аллокации
    frame_gray = cv.cvtColor(rgb_frame, cv.COLOR_RGB2GRAY)

    corners, ids, _ = detector.detectMarkers(frame_gray)

    # Легковесное однородное полотно для передачи по сети
    canvas = np.full((FRAME_H, FRAME_W, 3), CANVAS_BG_COLOR, dtype=np.uint8)

    # Отрисовка центра оптической оси (прицел)
    cv.drawMarker(canvas, (int(cx), int(cy)), (70, 70, 70), cv.MARKER_CROSS, 20, 1)

    target_point = None

    if ids is not None:
        cv.aruco.drawDetectedMarkers(canvas, corners, ids)
        detected_ids = ids.flatten()

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

            # Компенсация смещения относительно ID 22
            dx_m, dy_m = OFFSETS.get(active_id, (0.0, 0.0))
            if dx_m != 0.0 or dy_m != 0.0:
                with distance_lock:
                    alt_eff = max(current_distance, 0.3)
                u += (dx_m * fx) / alt_eff
                v += (dy_m * fy) / alt_eff

            target_point = (int(u), int(v))

            angle_x = math.atan2((u - cx) / fx, 1.0)
            angle_y = math.atan2((v - cy) / fy, 1.0)

            x_b = y_b = z_b = 0.0
            position_valid = 0

            if USE_FULL_POSE:
                obj_pts = np.array(
                    [
                        [-tag_size / 2, tag_size / 2, 0],
                        [tag_size / 2, tag_size / 2, 0],
                        [tag_size / 2, -tag_size / 2, 0],
                        [-tag_size / 2, -tag_size / 2, 0],
                    ],
                    dtype=np.float32,
                )
                img_pts = marker_corners.reshape(-1, 2).astype(np.float32)
                okp, rvec, tvec = cv.solvePnP(
                    obj_pts, img_pts, K, D, flags=cv.SOLVEPNP_IPPE_SQUARE
                )
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
                    float(angle_x),
                    float(angle_y),
                    0.0,  # distance (0.0 — работа с бортовым дальномером)
                    tag_size,
                    tag_size,  # size_x, size_y текущей метки
                    x_b,
                    y_b,
                    z_b,
                    [1.0, 0.0, 0.0, 0.0],
                    mavutil.mavlink.LANDING_TARGET_TYPE_VISION_FIDUCIAL,
                    position_valid,
                )

            # Вывод логов в консоль (2 Гц)
            if tnow >= next_log_time:
                next_log_time = tnow + log_period
                with distance_lock:
                    telemetry_alt = current_distance
                print(
                    f"[TARGET LOCK] ID: {active_id} ({tag_size*1000:.0f}mm) | AngX: {angle_x:.3f} rad | AngY: {angle_y:.3f} rad | MAV Alt: {telemetry_alt:.2f} m"
                )

    # Визуализация скорректированной точки посадки на холсте
    if target_point is not None:
        cv.circle(canvas, target_point, 5, (0, 0, 255), -1)
        cv.circle(canvas, target_point, 12, (0, 255, 255), 1)

    # Обновление кадра для фонового стрима (без сжатия в главном цикле)
    with frame_lock:
        latest_frame = canvas
