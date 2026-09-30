import io
import time, math
import threading
import numpy as np
import cv2 as cv
from http.server import HTTPServer, BaseHTTPRequestHandler
from pymavlink import mavutil
from gz.transport13 import Node
from gz.msgs10.image_pb2 import Image


def recv_ack(master_instance, command, timeout=3):
    """"""
    cmd_ack_flags = {
        "0": "Accepted",
        "1": "Temporarily Rejected",
        "2": "Denied",
        "3": "Unsupported",
        "4": "Failed",
        "5": "In Progress",
        "6": "Cancelled",
        "7": "CMD_LONG Only",
        "8": "CMD_INT Only",
        "9": "CMD Unsupported MAV_FRAME",
        "10": "Not In Control",
    }

    ack = master_instance.recv_match(type="COMMAND_ACK", blocking=True, timeout=timeout)
    if ack and ack.command == command:
        if ack.result == 0:
            print(
                f"[Accepted] COMMAND_ACK.result: {ack.result} ({cmd_ack_flags[str(ack.result)]})"
            )
        else:
            print(
                f"[Not Accepted] COMMAND_ACK.result: {ack.result} ({cmd_ack_flags[str(ack.result)]})"
            )

CAMERA_TOPIC = "/iris/camera/image_raw"
HTTP_PORT = 8080

# --- User config ---
FRAME_W, FRAME_H = 640, 480
ARUCO_DICT = cv.aruco.DICT_4X4_50
TAG_ID = 33
TAG_SIZE_M = 0.5  # marker side length in meters (used only if you want pose/xyz)
SERIAL_IP = "udpin:127.0.0.1:14551"  # Pixhawk TELEM port wired to Pi UART
BAUD = 921600                # or 57600 depending on your setup
USE_FULL_POSE = False        # True: send xyz + position_valid=1; False: angles-only

# --- Load calibration ---
fs = cv.FileStorage('calibration/camera.yaml', cv.FILE_STORAGE_READ)
K = fs.getNode('camera_matrix').mat()
D = fs.getNode('distortion_coefficients').mat()
fs.release()

# --- Global Frame Storage ---
latest_frame = None
latest_jpeg = None
frame_lock = threading.Lock()


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

    # Конвертация форматов цвета в BGR для OpenCV
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
    """Сервер для стриминга видео на localhost:8080."""
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
                    time.sleep(0.03)  # ~30 FPS для стрима
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
        pass  # Отключаем спам в консоли при запросах


def run_http_server():
    server = HTTPServer(("0.0.0.0", HTTP_PORT), MJPEGHandler)
    print(f"[HTTP] Сервер запущен на http://localhost:{HTTP_PORT}")
    server.serve_forever()


# --- Запуск фоновых потоков ---
threading.Thread(target=gazebo_listener, daemon=True).start()
threading.Thread(target=run_http_server, daemon=True).start()

aruco = cv.aruco
dict_ = aruco.getPredefinedDictionary(ARUCO_DICT)
params = aruco.DetectorParameters()
detector = aruco.ArucoDetector(dict_, params)

# MAVLink connection
m = mavutil.mavlink_connection(SERIAL_IP)
try:
    m.wait_heartbeat(timeout=5)
    print("[MAVLink] Heartbeat получен")
except Exception:
    print("[MAVLink] Heartbeat не получен, продолжаем...")

def center_from_corners(corners):
    pts = corners.reshape(-1, 2)
    c = pts.mean(axis=0)
    return float(c[0]), float(c[1])

fx, fy = K[0,0], K[1,1]
cx, cy = K[0,2], K[1,2]

rate_hz = 20.0
period = 1.0 / rate_hz
next_t = time.time()

while True:
    with frame_lock:
        if latest_frame is None:
            frame = None
        else:
            frame = latest_frame.copy()

    if frame is None:
        time.sleep(0.01)
        continue

    # Копия кадра для отрисовки меток
    draw_frame = frame.copy()

    # Detect ArUco markers
    corners, ids, _ = detector.detectMarkers(frame)
    if ids is not None and TAG_ID in ids.flatten():
        # Отрисовка обнаруженных меток
        aruco.drawDetectedMarkers(draw_frame, corners, ids)

        idx = np.where(ids.flatten() == TAG_ID)[0][0]
        marker_corners = corners[idx]

        # --- Compute angles relative to optical axis ---
        u, v = center_from_corners(marker_corners)
        angle_x = math.atan2((u - cx) / fx, 1.0)
        angle_y = math.atan2((v - cy) / fy, 1.0)

        # Optional: full pose to get body-frame xyz from camera frame
        x_b = y_b = z_b = 0.0
        position_valid = 0
        if USE_FULL_POSE:
            obj_pts = np.array([
                [-TAG_SIZE_M/2,  TAG_SIZE_M/2, 0],
                [ TAG_SIZE_M/2,  TAG_SIZE_M/2, 0],
                [ TAG_SIZE_M/2, -TAG_SIZE_M/2, 0],
                [-TAG_SIZE_M/2, -TAG_SIZE_M/2, 0],
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

        # Throttle to target rate
        tnow = time.time()
        if tnow >= next_t:
            next_t += period

            # LANDING_TARGET send
            m.mav.landing_target_send(
                int(tnow * 1e6),        # time_usec
                0,                      # target_num
                mavutil.mavlink.MAV_FRAME_BODY_NED,
                float(angle_x), float(angle_y),
                0.0,                    # distance
                TAG_SIZE_M, TAG_SIZE_M, # size_x, size_y
                x_b, y_b, z_b,          # position in body frame
                [1.0, 0.0, 0.0, 0.0],   # orientation
                mavutil.mavlink.LANDING_TARGET_TYPE_VISION_FIDUCIAL,
                position_valid
            )
            print("[LANDING_TARGET] message sended.")
    print(corners)
    # Кодирование кадра с метками в JPEG и обновление глобальной переменной для HTTP
    _, jpg = cv.imencode(".jpg", draw_frame, [cv.IMWRITE_JPEG_QUALITY, 80])
    with frame_lock:
        latest_jpeg = jpg.tobytes()