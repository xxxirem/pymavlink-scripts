#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Precision landing по ArUco-меткам — облегчённая исправленная версия.

Запуск:
    python3 pl_web_centered_light_trae.py

Веб:
    http://<ip>:8080/        — страница, которая сама переподключает поток
    http://<ip>:8080/stream  — чистый MJPEG (для VLC/ffmpeg)

Что исправлено относительно pl_web_centered_light.py:
  * Веб-поток. Был однопоточный HTTPServer: один «прилипший» клиент (F5,
    пропавший Wi-Fi) блокировал сервер навсегда, а каждый клиент заново
    копировал кадр и кодировал JPEG. Теперь ThreadingHTTPServer, JPEG
    кодируется один раз на всех, у сокетов есть таймауты, а при отсутствии
    новых кадров соединение закрывается и страница переподключается сама.
  * Координаты. Цель считается в 3D через solvePnP и переводится из системы
    камеры в систему корпуса (FRD) явной матрицей монтажа MOUNT_YAW_DEG.
    angle_x — отклонение вперёд (X-Z), angle_y — вправо (Y-Z), как ждёт
    ArduPilot для MAV_FRAME_BODY_FRD. MOUNT_YAW_DEG=0 => верх кадра = нос.
  * Смещение меток 33/44 к центру ID22 считается как вектор в плоскости листа
    (а не через деление на высоту EKF), с правильным знаком.
  * MAVLink читается и пишется из одного потока (pymavlink не потокобезопасен),
    landing_target шлётся с постоянной частотой и держит последнюю цель.
"""

import socket
import math
import threading
import time
from dataclasses import dataclass
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import numpy as np
import cv2 as cv
from picamera2 import Picamera2
from pymavlink import mavutil

# --- Настройки ---
HTTP_PORT = 8080
FRAME_W, FRAME_H = 640, 480
ARUCO_DICT = cv.aruco.DICT_4X4_50

TARGET_MARKERS = {
    22: 0.034,  # 34 мм  — приоритет 1 (главная цель посадки)
    33: 0.067,  # 67 мм  — приоритет 2
    44: 0.126,  # 126 мм — приоритет 3
}
PRIORITY_ORDER = [22, 33, 44]
# Центр метки относительно центра ID22 в системе листа: +x вправо, +y вверх
OFFSETS = {
    22: (0.0, 0.0),
    33: (0.0645, 0.0),
    44: (0.036, -0.105),
}

SERIAL_IP = "udpout:127.0.0.1:14551"
USE_FULL_POSE = False  # True: xyz + position_valid=1; False: только углы
SEND_HZ = 20.0
TARGET_HOLD_S = 0.3  # сколько держим последнюю цель при пропадании метки

# Камера смотрит вниз. Поворот камеры вокруг оптической оси:
#   0 = верх кадра смотрит в нос, 90 = вправо, 180 = в хвост, 270 = влево
MOUNT_YAW_DEG = 0.0

# Веб
STREAM_FPS = 10.0
JPEG_QUALITY = 60
MAX_CLIENTS = 3
IDLE_CLOSE_S = 10.0

# --- Системы координат ---
# Камера (OpenCV): +x вправо по кадру, +y вниз по кадру, +z вдоль объектива (вниз)
# Корпус (FRD):    +x вперёд (нос),      +y вправо,        +z вниз


def _rot_z(deg: float) -> np.ndarray:
    a = math.radians(deg)
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


# yaw=0: верх кадра -> нос, право кадра -> правый борт
_BASE_CAM2BODY = np.array(
    [
        [0.0, -1.0, 0.0],
        [1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0],
    ]
)
R_BODY_CAM = _rot_z(MOUNT_YAW_DEG) @ _BASE_CAM2BODY  # p_body = R @ p_cam

# --- Калибровка (calibration/ лежит на уровень выше папки trae) ---
CALIB_PATH = Path(__file__).resolve().parent.parent / "calibration" / "camera.yaml"
_fs = cv.FileStorage(str(CALIB_PATH), cv.FILE_STORAGE_READ)
K = _fs.getNode("camera_matrix").mat()
D = _fs.getNode("distortion_coefficients").mat()
_fs.release()
fx, fy = K[0, 0], K[1, 1]
cx, cy = K[0, 2], K[1, 2]
print(f"[CALIB] fx={fx:.1f} fy={fy:.1f} cx={cx:.1f} cy={cy:.1f}")

# --- ArUco ---
_dict = cv.aruco.getPredefinedDictionary(ARUCO_DICT)
_params = cv.aruco.DetectorParameters()
detector = cv.aruco.ArucoDetector(_dict, _params)

_OBJ_CACHE = {}


def marker_object_points(size: float) -> np.ndarray:
    """Углы метки в её системе (x вправо, y вверх, z из плоскости).

    Порядок ЛВ, ПВ, ПН, ЛН — как у corners из detectMarkers и как требует
    SOLVEPNP_IPPE_SQUARE.
    """
    pts = _OBJ_CACHE.get(size)
    if pts is None:
        h = size / 2.0
        pts = np.array(
            [[-h, h, 0.0], [h, h, 0.0], [h, -h, 0.0], [-h, -h, 0.0]], dtype=np.float32
        )
        _OBJ_CACHE[size] = pts
    return pts


@dataclass
class Target:
    marker_id: int
    marker_size: float
    bx: float  # вперёд, м
    by: float  # вправо, м
    bz: float  # вниз, м
    angle_x: float  # rad, X-Z
    angle_y: float  # rad, Y-Z
    distance: float
    stamp: float


_smooth: np.ndarray | None = None


def _smooth_body(p: np.ndarray) -> np.ndarray:
    """Лёгкое сглаживание вектора цели (EMA со сбросом на резком скачке)."""
    global _smooth
    if _smooth is None or np.linalg.norm(p - _smooth) > 0.25:
        _smooth = p.copy()
    else:
        _smooth = _smooth + 0.35 * (p - _smooth)
    return _smooth


def estimate_target(marker_id: int, corners: np.ndarray, size: float) -> Target | None:
    """Поза метки через solvePnP -> центр ID22 -> система корпуса -> углы."""
    obj = marker_object_points(size)
    img = corners.reshape(-1, 2).astype(np.float32)

    ok, rvec, tvec = cv.solvePnP(obj, img, K, D, flags=cv.SOLVEPNP_IPPE_SQUARE)
    if not ok:
        return None

    t = np.asarray(tvec, dtype=np.float64).reshape(3)
    if not (0.05 <= t[2] <= 8.0):
        return None

    # отбраковка «кривых» поз по ошибке репроекции
    proj, _ = cv.projectPoints(obj, rvec, tvec, K, D)
    err = float(np.sqrt(np.mean(np.sum((proj.reshape(4, 2) - img) ** 2, axis=1))))
    if err > 5.0:
        return None

    r_mc = cv.Rodrigues(rvec)[0]
    ox, oy = OFFSETS.get(marker_id, (0.0, 0.0))
    # вектор «от центра метки к центру ID22» в системе метки
    d = np.array([-ox, -oy, 0.0])
    p_cam = r_mc @ d + t
    p_body = _smooth_body(R_BODY_CAM @ p_cam)

    if p_body[2] <= 1e-6:
        return None
    angle_x = math.atan2(p_body[0], p_body[2])
    angle_y = math.atan2(p_body[1], p_body[2])
    if abs(angle_x) > 0.7 or abs(angle_y) > 0.7:
        return None

    return Target(
        marker_id=marker_id,
        marker_size=size,
        bx=float(p_body[0]),
        by=float(p_body[1]),
        bz=float(p_body[2]),
        angle_x=angle_x,
        angle_y=angle_y,
        distance=float(np.linalg.norm(p_body)),
        stamp=time.monotonic(),
    )


# --- MAVLink: один поток и на чтение, и на отправку ---
class MavlinkNode:
    TYPES = ["HEARTBEAT", "DISTANCE_SENSOR", "LOCAL_POSITION_NED"]

    def __init__(self, url: str):
        self.url = url
        self._lock = threading.Lock()
        self._target: Target | None = None
        self._alt = 0.0
        self._link = False
        self._stop = threading.Event()

    def start(self) -> None:
        threading.Thread(target=self._run, name="mavlink", daemon=True).start()

    def publish(self, target: Target) -> None:
        with self._lock:
            self._target = target

    def telemetry(self):
        with self._lock:
            return self._alt, self._link

    def _run(self) -> None:
        while not self._stop.is_set():
            conn = None
            try:
                conn = mavutil.mavlink_connection(self.url)
                hb = conn.wait_heartbeat(timeout=8)
                if hb is None:
                    raise RuntimeError("нет heartbeat")
                conn.target_system = hb.get_srcSystem()
                conn.target_component = hb.get_srcComponent()
                print("[MAV] connected")
                with self._lock:
                    self._link = True
                self._loop(conn)
            except Exception as exc:
                print(f"[MAV] {exc}")
            finally:
                with self._lock:
                    self._link = False
                if conn is not None:
                    try:
                        conn.close()
                    except Exception:
                        pass
            self._stop.wait(1.0)

    def _loop(self, conn) -> None:
        period = 1.0 / SEND_HZ
        next_send = time.monotonic()
        next_hb = 0.0
        last_rx = time.monotonic()
        while not self._stop.is_set():
            now = time.monotonic()

            if now >= next_send:
                self._send(conn, now)
                next_send = max(now, next_send + period)

            if now >= next_hb:
                conn.mav.heartbeat_send(
                    mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
                    mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                    0, 0, 0,
                )
                next_hb = now + 1.0

            msg = conn.recv_match(type=self.TYPES, blocking=True, timeout=0.02)
            if msg is not None:
                last_rx = time.monotonic()
                mtype = msg.get_type()
                if mtype == "DISTANCE_SENSOR":
                    # orientation 25 = MAV_SENSOR_ORIENTATION_DOWN
                    if getattr(msg, "orientation", 25) == 25 and msg.current_distance > 0:
                        with self._lock:
                            self._alt = msg.current_distance / 100.0
                elif mtype == "LOCAL_POSITION_NED":
                    with self._lock:
                        self._alt = float(-msg.z)

            if time.monotonic() - last_rx > 8.0:
                raise RuntimeError("потеря связи")

    def _send(self, conn, now: float) -> None:
        with self._lock:
            t = self._target
        if t is None or now - t.stamp > TARGET_HOLD_S:
            return

        if USE_FULL_POSE:
            x, y, z, valid = t.bx, t.by, t.bz, 1
        else:
            x = y = z = 0.0
            valid = 0

        # size_x/size_y в спецификации MAVLink — угловой размер [rad]
        size_rad = t.marker_size / max(t.distance, 1e-3)
        conn.mav.landing_target_send(
            int(time.time() * 1e6),
            0,
            mavutil.mavlink.MAV_FRAME_BODY_FRD,
            float(t.angle_x),
            float(t.angle_y),
            0.0,  # distance=0 -> используем бортовой дальномер
            size_rad,
            size_rad,
            x, y, z,
            [1.0, 0.0, 0.0, 0.0],
            mavutil.mavlink.LANDING_TARGET_TYPE_VISION_FIDUCIAL,
            valid,
        )


# --- Веб: один энкодер на всех клиентов + ThreadingHTTPServer ---
class StreamState:
    def __init__(self, fps: float, quality: int):
        self.cond = threading.Condition()
        self.jpeg = b""
        self.seq = 0
        self.clients = 0
        self.stop = False
        self.fps = fps
        self.quality = quality
        self._next = 0.0

    def publish_canvas(self, canvas: np.ndarray) -> None:
        """Кодируем кадр только если есть клиенты (экономия CPU на Pi Zero)."""
        with self.cond:
            if self.clients <= 0:
                return
        now = time.monotonic()
        if now < self._next:
            return
        self._next = now + 1.0 / self.fps
        ok, buf = cv.imencode(".jpg", canvas, [cv.IMWRITE_JPEG_QUALITY, self.quality])
        if not ok:
            return
        data = buf.tobytes()
        with self.cond:
            self.jpeg = data
            self.seq += 1
            self.cond.notify_all()

    def wait(self, last_seq: int, timeout: float):
        deadline = time.monotonic() + timeout
        with self.cond:
            while not self.stop and self.seq == last_seq:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    return None, last_seq
                self.cond.wait(remaining)
            if self.stop:
                return None, last_seq
            return self.jpeg, self.seq


INDEX_HTML = """<!doctype html>
<html lang="ru"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Precision Landing</title>
<style>
 body{background:#111;color:#0f8;font-family:ui-monospace,monospace;text-align:center;margin:8px}
 img{max-width:100%;border:2px solid #0f8;border-radius:6px}
</style></head><body>
<h3>Precision Landing &mdash; ArUco</h3>
<img id="v" src="/stream" alt="stream">
<script>
var v=document.getElementById('v');
function reconnect(){setTimeout(function(){v.src='/stream?t='+Date.now();},1000);}
v.onerror=reconnect;
</script></body></html>"""


class StreamHandler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.0"
    state: StreamState = None  # подставляется перед созданием сервера

    def log_message(self, *args):
        pass

    def _plain(self, code, body: bytes, ctype="text/plain; charset=utf-8"):
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        try:
            self.wfile.write(body)
        except OSError:
            pass

    def do_GET(self):
        path = self.path.split("?", 1)[0]
        if path in ("/", "/index.html"):
            self._plain(200, INDEX_HTML.encode("utf-8"), "text/html; charset=utf-8")
        elif path in ("/stream", "/video.mjpg"):
            self._stream()
        else:
            self._plain(404, b"not found")

    def _stream(self):
        st = self.state
        with st.cond:
            if st.clients >= MAX_CLIENTS:
                self._plain(503, b"too many clients")
                return
            st.clients += 1
        try:
            try:
                self.connection.settimeout(5.0)
                self.connection.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            except OSError:
                pass

            self.send_response(200)
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Connection", "close")
            self.end_headers()

            last_seq = -1
            last_progress = time.monotonic()
            while not st.stop:
                jpeg, seq = st.wait(last_seq, timeout=1.0)
                if seq == last_seq:
                    # новых кадров нет: если главный цикл жив — они появятся,
                    # иначе рвём соединение, чтобы страница переподключилась
                    if time.monotonic() - last_progress > IDLE_CLOSE_S:
                        break
                    continue
                last_seq = seq
                last_progress = time.monotonic()
                header = (
                    b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: "
                    + str(len(jpeg)).encode()
                    + b"\r\n\r\n"
                )
                self.wfile.write(header + jpeg + b"\r\n")
        except (BrokenPipeError, ConnectionResetError, TimeoutError, OSError):
            pass
        finally:
            self.close_connection = True
            with st.cond:
                st.clients = max(0, st.clients - 1)
                st.cond.notify_all()


class StreamServer(ThreadingHTTPServer):
    daemon_threads = True
    allow_reuse_address = True


def start_http(state: StreamState) -> StreamServer:
    StreamHandler.state = state
    server = StreamServer(("0.0.0.0", HTTP_PORT), StreamHandler)
    threading.Thread(target=server.serve_forever, name="http", daemon=True).start()
    print(f"[HTTP] http://0.0.0.0:{HTTP_PORT}  (/ — страница, /stream — MJPEG)")
    return server


# --- Главный цикл ---
def main() -> None:
    state = StreamState(STREAM_FPS, JPEG_QUALITY)
    start_http(state)

    mav = MavlinkNode(SERIAL_IP)
    mav.start()

    print("[CAM] инициализация Picamera2...")
    picam2 = Picamera2()
    conf = picam2.create_video_configuration(
        main={"size": (FRAME_W, FRAME_H), "format": "RGB888"}
    )
    picam2.configure(conf)
    picam2.start()
    print("[CAM] запущена")

    global _smooth
    next_log = 0.0

    while True:
        try:
            rgb = picam2.capture_array()
            gray = cv.cvtColor(rgb, cv.COLOR_RGB2GRAY)
            corners, ids, _ = detector.detectMarkers(gray)
        except Exception as exc:
            print(f"[CAM] {exc}")
            time.sleep(0.2)
            continue

        # лёгкий холст: только векторная отрисовка
        canvas = np.zeros((FRAME_H, FRAME_W, 3), dtype=np.uint8)
        cv.drawMarker(canvas, (int(cx), int(cy)), (70, 70, 70), cv.MARKER_CROSS, 20, 1)

        target = None
        try:
            if ids is not None:
                cv.aruco.drawDetectedMarkers(canvas, corners, ids)
                detected = ids.flatten()
                for mid in PRIORITY_ORDER:
                    idx = np.where(detected == mid)[0]
                    if idx.size == 0:
                        continue
                    target = estimate_target(mid, corners[idx[0]], TARGET_MARKERS[mid])
                    if target is not None:
                        break
        except Exception as exc:  # один битый кадр не должен ронять цикл
            print(f"[LOOP] {exc}")

        if target is None:
            _smooth = None  # сброс фильтра, чтобы не тянуть старую цель

        if target is not None:
            mav.publish(target)

            # точка цели в кадре (для глаз): обратно из корпуса в камеру
            p_cam = R_BODY_CAM.T @ np.array([target.bx, target.by, target.bz])
            if p_cam[2] > 1e-6:
                u = int(cx + fx * p_cam[0] / p_cam[2])
                v = int(cy + fy * p_cam[1] / p_cam[2])
                cv.circle(canvas, (u, v), 5, (0, 0, 255), -1)
                cv.circle(canvas, (u, v), 12, (0, 255, 255), 1)

            now = time.monotonic()
            if now >= next_log:
                next_log = now + 0.5
                alt, link = mav.telemetry()
                print(
                    f"[TARGET] ID {target.marker_id} ({target.marker_size*1000:.0f}mm) | "
                    f"ax {target.angle_x:+.3f} ay {target.angle_y:+.3f} rad | "
                    f"fwd {target.bx:+.2f} right {target.by:+.2f} down {target.bz:.2f} m | "
                    f"alt {alt:.2f} m link={'OK' if link else 'LOST'}"
                )

        state.publish_canvas(canvas)


if __name__ == "__main__":
    main()
