#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Precision Landing по ArUco-полю (3 метки на A4) — версия 2.

Запуск:
    python3 pl_web_centered_light_v2.py                # рабочий режим
    python3 pl_web_centered_light_v2.py --diag         # + отладочный вывод 5 Гц
    python3 pl_web_centered_light_v2.py --selftest     # проверка геометрии (без камеры)

Что исправлено относительно pl_web_centered_light.py
=====================================================

1) Смещения OFFSETS. Раньше они переводились в пиксели через текущую высоту:

       u += dx * fx / alt
       v += dy * fy / alt

   Это неверно сразу по четырём причинам:
     * «alt» — это z из LOCAL_POSITION_NED (высота над EKF-origin), а не высота
       над листом. Любая ошибка масштаба даёт ошибку в смещении; при alt = 0
       (первые кадры, потеря EKF, origin не выставлен) включался зажим 0.3 м и
       поправка улетала на сотни пикселей;
     * масштаб 1/alt растёт при снижении, поэтому у земли поправка «взрывается»
       именно тогда, когда точность важнее всего, и цель уходит за кадр;
     * смещение в пикселях верно только если плоскость метки параллельна
       матрице (крен/тангаж дрона уже дают ошибку) и если лист не повёрнут;
     * не учитывалась дисторсия объектива (у этого объектива k1 = -0.42,
       k3 = -2.7), а при больших смещениях ошибка максимальна.

   Теперь смещение живёт в 3D: solvePnP даёт позу метки (R, t), центр цели
   считается как P_cam = R * (dx, dy, 0) + t. Высота в этой формуле вообще не
   участвует, дисторсия учтена в solvePnP, перспектива и поворот листа — тоже.

2) Оси камеры и корпуса. Скрипт считал angle_x из координаты u (право кадра),
   т.е. молча предполагал, что «право кадра = нос дрона». ArduPilot для
   MAV_FRAME_BODY_FRD трактует angle_x как угол в плоскости X-Z (вперёд), а
   angle_y — в плоскости Y-Z (вправо). Если камера стоит иначе (а это самый
   частый случай), дрон летит в повёрнутом на 90° направлении и уходит за лист.
   Теперь поворот камеры задаётся явно: MOUNT_YAW_DEG (0/90/180/270).

3) Поток MAVLink. `landing_target` отправлялся только внутри `if ids is not None`,
   т.е. при потере метки сообщения прекращались совсем. Отправка перенесена в
   отдельный поток с фиксированной частотой и удержанием последней цели
   (TARGET_HOLD_S), а чтение и запись MAVLink больше не идут из разных потоков
   в один объект pymavlink.

4) Веб-поток. Причины «зависания квадратов»:
     * HTTPServer был однопоточным: один «повисший» клиент (обновили страницу,
       пропал Wi-Fi, браузер держит соединение) блокировал сервер навсегда,
       новые подключения не обслуживались — на экране оставался последний кадр;
     * каждый клиент заново копировал кадр (921 КБ) и заново кодировал JPEG;
     * любое исключение в главном цикле (камера/MAVLink) убивало детекцию, но
       daemon-поток HTTP продолжал отдавать последний кадр — «картинка замерла»;
     * time.time() вместо time.monotonic().
   Теперь: ThreadingHTTPServer, лимит клиентов, один общий JPEG-энкодер,
   таймауты сокетов, автопереподключение страницы, watchdog.

5) Производительность на Pi Zero 2 W: кадр берётся как YUV420 и детекция идёт
   по Y-плоскости (без cvtColor и без копии RGB), параметры детектора настроены,
   JPEG кодируется один раз на всех клиентов.

Раскладка листа (вид сверху, как напечатано):
    ┌──────────────────────────────┐
    │  [22]        [33]            │   ID22 35 мм, ID33 70 мм, ID44 130 мм
    │   ▲                          │   (в OFFSETS — реальные размеры печати)
    │   │  dx →                    │
    │   ▼                          │
    │        [   44   ]            │
    └──────────────────────────────┘
    OFFSETS[(id)] = (dx, dy) — центр метки относительно центра ID22
    в системе листа: +x вправо, +y вверх (как на напечатанном листе).

ПРОВЕРКА ОСЕЙ (делается один раз, 2 минуты, без винтов):
    Подвесьте дрон над листом, метка 22 в центре кадра, высота ~1 м.
    Сдвиньте дрон на 20 см в сторону НОСА, не меняя высоту и курс.
    В консоли (режим --diag) должно быть:
        body=(+0.20, ~0.00, ~1.00)  →  «target: FORWARD 0.20 m»
    Если вместо этого появляется RIGHT/LEFT/BACK — поменяйте MOUNT_YAW_DEG
    (0 → нос смотрит вверх кадра, 90 → вправо, 180 → вниз, 270 → влево).
    Если направление верное, но знак обратный — проверьте, что лист лежит
    «лицом вверх» и не перевёрнут.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import signal
import socket
import sys
import threading
import time
from dataclasses import dataclass, field, replace
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

try:  # OpenCV нужен всегда, но при --selftest его отсутствие не мешает
    import cv2 as cv
except Exception:  # pragma: no cover
    cv = None

try:
    from picamera2 import Picamera2
except Exception:  # pragma: no cover - на ПК модуля нет
    Picamera2 = None

try:
    from libcamera import controls as libcam
except Exception:  # pragma: no cover
    libcam = None

try:
    from pymavlink import mavutil
except Exception:  # pragma: no cover
    mavutil = None


BASE_DIR = Path(__file__).resolve().parent
CANVAS_BG = (0, 0, 0)
EXIT_WATCHDOG = 3


def log(msg: str) -> None:
    """Печать с меткой времени и принудительным сбросом буфера (важно для systemd)."""
    print(f"[{time.strftime('%H:%M:%S')}] {msg}", flush=True)


# ----------------------------------------------------------------------------
# Конфигурация
# ----------------------------------------------------------------------------


@dataclass(frozen=True)
class Config:
    # --- камера -------------------------------------------------------------
    frame_size: Tuple[int, int] = (640, 480)
    capture_fps: int = 20
    calib_path: Path = BASE_DIR / "calibration" / "camera.yaml"
    # "off" | "continuous" | "manual"  (автофокус на снижении любит «рыскать»)
    af_mode: str = "off"
    lens_position: float = 1.5  # диоптрии: 1.5 ≈ 0.67 м, 3.0 ≈ 0.33 м

    # --- метки --------------------------------------------------------------
    aruco_dict_name: str = "DICT_4X4_50"
    # Реальные размеры печати (PDF может масштабировать, поэтому мерить линейкой)
    marker_sizes: Dict[int, float] = field(
        default_factory=lambda: {22: 0.034, 33: 0.067, 44: 0.126}
    )
    priority: Tuple[int, ...] = (22, 33, 44)
    # Смещение центра метки относительно центра ID22, в метрах, +y вверх по листу
    offsets: Dict[int, Tuple[float, float]] = field(
        default_factory=lambda: {
            22: (0.000, 0.000),
            33: (0.0645, 0.000),
            44: (0.036, -0.105),
        }
    )
    detect_scale: float = 1.0  # <1.0 ускоряет детекцию, но теряет мелкую метку
    corner_refine: bool = False  # True = точнее, но дороже по CPU
    min_marker_px: float = 10.0  # минимальная сторона метки в пикселях
    max_reproj_px: float = 5.0  # отбраковка «кривой» позы
    min_range_m: float = 0.05
    max_range_m: float = 8.0

    # --- монтаж камеры ------------------------------------------------------
    # 0 = верх кадра смотрит в нос, 90 = вправо, 180 = в хвост, 270 = влево.
    # 270 воспроизводит математику старого скрипта (право кадра = нос).
    mount_yaw_deg: float = 0.0
    mount_pitch_deg: float = 0.0
    mount_roll_deg: float = 0.0

    # --- фильтрация и логика ------------------------------------------------
    smooth_alpha: float = 0.35  # 1.0 = без сглаживания
    jump_reset_m: float = 0.25  # скачок цели больше этого — сброс фильтра
    target_hold_s: float = 0.30  # сколько держим последнюю цель при пропадании
    max_angle_rad: float = 0.70  # ~40°, защита от ложных детекций
    use_full_pose: bool = False  # True: x/y/z + position_valid=1
    send_distance: bool = False  # True: отдавать distance из позы

    # --- MAVLink ------------------------------------------------------------
    mav_url: str = "udpout:127.0.0.1:14551"
    send_hz: float = 20.0
    connect_timeout_s: float = 8.0
    link_timeout_s: float = 8.0
    send_heartbeat: bool = True

    # --- веб ----------------------------------------------------------------
    http_port: int = 8080
    stream_fps: float = 10.0
    jpeg_quality: int = 60
    max_clients: int = 3
    client_timeout_s: float = 5.0
    idle_close_s: float = 10.0

    # --- служебное ----------------------------------------------------------
    watchdog_s: float = 8.0
    log_period_s: float = 2.0
    camera_retry_s: float = 3.0


# ----------------------------------------------------------------------------
# Геометрия
# ----------------------------------------------------------------------------

# Камера (OpenCV): x — вправо по кадру, y — вниз по кадру, z — вдоль оси объектива.
# Корпус (FRD):   x — вперёд,          y — вправо,          z — вниз.
#
# Матрица ниже — «камера смотрит вниз, верх кадра смотрит в нос».
# Столбцы: оси камеры, выраженные в осях корпуса.
BASE_CAM_TO_BODY = np.array(
    [
        [0.0, -1.0, 0.0],
        [1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0],
    ],
    dtype=np.float64,
)


def _rot_x(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


def _rot_y(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def _rot_z(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def mount_matrix(yaw_deg: float, pitch_deg: float = 0.0, roll_deg: float = 0.0) -> np.ndarray:
    """Матрица поворота «камера → корпус» (p_body = R @ p_cam)."""
    r = _rot_z(math.radians(yaw_deg))
    r = r @ _rot_y(math.radians(pitch_deg))
    r = r @ _rot_x(math.radians(roll_deg))
    return r @ BASE_CAM_TO_BODY


def camera_to_body_matrix(cfg: Config) -> np.ndarray:
    return mount_matrix(cfg.mount_yaw_deg, cfg.mount_pitch_deg, cfg.mount_roll_deg)


def marker_object_points(size: float) -> np.ndarray:
    """Углы метки в её собственной системе: x вправо, y вверх, z из плоскости.

    Порядок обязателен для SOLVEPNP_IPPE_SQUARE: ЛВ, ПВ, ПН, ЛН.
    Совпадает с порядком corners из cv.aruco.detectMarkers.
    """
    h = float(size) / 2.0
    return np.array(
        [
            [-h, h, 0.0],
            [h, h, 0.0],
            [h, -h, 0.0],
            [-h, -h, 0.0],
        ],
        dtype=np.float32,
    )


def angles_from_body(p_body: np.ndarray) -> Optional[Tuple[float, float, float]]:
    """Вектор цели в системе корпуса → (angle_x, angle_y, distance).

    ArduPilot (MAV_FRAME_BODY_FRD) ждёт:
        angle_x = atan2(x, z) — отклонение в плоскости «вперёд-вниз»;
        angle_y = atan2(y, z) — отклонение в плоскости «вправо-вниз».
    """
    x, y, z = float(p_body[0]), float(p_body[1]), float(p_body[2])
    if z <= 1e-6:
        return None
    dist = math.sqrt(x * x + y * y + z * z)
    return math.atan2(x, z), math.atan2(y, z), dist


def target_point_camera(
    r_marker_cam: np.ndarray, t_marker_cam: np.ndarray, offset_xy: Tuple[float, float]
) -> np.ndarray:
    """Центр цели (ID22) в системе камеры по позе найденной метки.

    offset_xy — смещение центра ID22 относительно центра метки в системе листа.
    Знак обратный: нам нужен вектор «от метки к ID22».
    """
    d = np.array([-float(offset_xy[0]), -float(offset_xy[1]), 0.0])
    return r_marker_cam @ d + t_marker_cam.reshape(3)


# ----------------------------------------------------------------------------
# Калибровка
# ----------------------------------------------------------------------------


@dataclass
class Calibration:
    K: np.ndarray
    D: np.ndarray
    width: int
    height: int

    @property
    def fx(self) -> float:
        return float(self.K[0, 0])

    @property
    def fy(self) -> float:
        return float(self.K[1, 1])

    @property
    def cx(self) -> float:
        return float(self.K[0, 2])

    @property
    def cy(self) -> float:
        return float(self.K[1, 2])

    @classmethod
    def load(cls, path: Path, frame_size: Optional[Tuple[int, int]] = None) -> "Calibration":
        if cv is None:
            raise RuntimeError("OpenCV не установлен (pip install opencv-python-headless)")
        path = Path(path)
        if not path.exists():
            raise FileNotFoundError(f"нет файла калибровки: {path}")

        fs = cv.FileStorage(str(path), cv.FILE_STORAGE_READ)
        if not fs.isOpened():
            raise ValueError(f"не удалось открыть {path}")
        try:
            k = fs.getNode("camera_matrix").mat()
            d = fs.getNode("distortion_coefficients").mat()
            w = int(fs.getNode("image_width").real())
            h = int(fs.getNode("image_height").real())
        finally:
            fs.release()

        if k is None or d is None:
            raise ValueError(f"в {path} нет camera_matrix / distortion_coefficients")

        k = np.asarray(k, dtype=np.float64).reshape(3, 3)
        d = np.asarray(d, dtype=np.float64).reshape(1, -1)

        if frame_size and w > 0 and h > 0 and (w, h) != tuple(frame_size):
            sx = frame_size[0] / float(w)
            sy = frame_size[1] / float(h)
            k[0, 0] *= sx
            k[0, 2] *= sx
            k[1, 1] *= sy
            k[1, 2] *= sy
            log(f"[CALIB] K отмасштабирована {w}x{h} -> {frame_size[0]}x{frame_size[1]}")
            w, h = int(frame_size[0]), int(frame_size[1])

        calib = cls(K=k, D=d, width=w, height=h)
        # Principal point должен быть примерно в центре кадра. Если он уехал,
        # калибровка битая и дрон будет систематически промахиваться.
        if w and h:
            dx, dy = abs(calib.cx - w / 2.0), abs(calib.cy - h / 2.0)
            if dx > 0.15 * w or dy > 0.15 * h:
                log(
                    "[CALIB] ВНИМАНИЕ: principal point "
                    f"({calib.cx:.1f}, {calib.cy:.1f}) далеко от центра кадра "
                    f"({w / 2:.0f}, {h / 2:.0f}) — перекалибруйте на этом разрешении"
                )
        log(
            f"[CALIB] {path.name}: fx={calib.fx:.1f} fy={calib.fy:.1f} "
            f"cx={calib.cx:.1f} cy={calib.cy:.1f}"
        )
        return calib


# ----------------------------------------------------------------------------
# Камера
# ----------------------------------------------------------------------------


class CameraSource:
    """Picamera2 с захватом только яркости (Y-плоскость YUV420)."""

    def __init__(self, cfg: Config):
        self.cfg = cfg
        self._cam = None
        self._convert = False
        self.last_error = ""

    @property
    def is_open(self) -> bool:
        return self._cam is not None

    def open(self) -> bool:
        if Picamera2 is None:
            log("[CAM] picamera2 недоступна — запуск без камеры")
            return False
        cam = None
        try:
            cam = Picamera2()
            us = int(1e6 / max(1, self.cfg.capture_fps))
            controls = {"FrameDurationLimits": (us, us)}
            try:
                conf = cam.create_video_configuration(
                    main={"size": self.cfg.frame_size, "format": "YUV420"},
                    buffer_count=2,
                    controls=controls,
                )
                cam.configure(conf)
                self._convert = False
            except Exception as exc:
                log(f"[CAM] YUV420 недоступен ({exc}), пробуем RGB888")
                conf = cam.create_video_configuration(
                    main={"size": self.cfg.frame_size, "format": "RGB888"},
                    buffer_count=2,
                )
                cam.configure(conf)
                self._convert = True

            cam.start()
            time.sleep(0.3)  # дать AE/AWB устояться
            self._apply_lens(cam)
            self._cam = cam
            log(f"[CAM] запущена {self.cfg.frame_size[0]}x{self.cfg.frame_size[1]} "
                f"@ {self.cfg.capture_fps} fps, YUV420={'нет' if self._convert else 'да'}")
            return True
        except Exception as exc:
            self.last_error = f"cam open: {exc}"
            log(f"[CAM] ошибка инициализации: {exc}")
            _shutdown_cam(cam)
            self._cam = None
            return False

    def _apply_lens(self, cam) -> None:
        if self.cfg.af_mode == "off" or libcam is None:
            return
        try:
            if self.cfg.af_mode == "continuous":
                cam.set_controls(
                    {
                        "AfMode": libcam.AfModeEnum.Continuous,
                        "AfSpeed": libcam.AfSpeedEnum.Fast,
                    }
                )
            elif self.cfg.af_mode == "manual":
                cam.set_controls(
                    {
                        "AfMode": libcam.AfModeEnum.Manual,
                        "LensPosition": float(self.cfg.lens_position),
                    }
                )
            log(f"[CAM] фокус: {self.cfg.af_mode}")
        except Exception as exc:
            log(f"[CAM] не удалось применить фокус ({self.cfg.af_mode}): {exc}")

    def read(self) -> Optional[np.ndarray]:
        if self._cam is None:
            return None
        try:
            arr = self._cam.capture_array("main")
        except Exception as exc:
            self.last_error = f"cam read: {exc}"
            log(f"[CAM] ошибка захвата: {exc}")
            self.close()
            return None

        w, h = self.cfg.frame_size
        if self._convert or arr.ndim == 3:
            return cv.cvtColor(arr[..., :3], cv.COLOR_RGB2GRAY)
        # Y-плоскость YUV420: первые h строк, уже непрерывна
        return np.ascontiguousarray(arr[:h, :w])

    def close(self) -> None:
        cam, self._cam = self._cam, None
        _shutdown_cam(cam)


def _shutdown_cam(cam) -> None:
    """Аккуратно гасим Picamera2, даже если объект не до конца инициализирован."""
    if cam is None:
        return
    try:
        cam.stop()
    except Exception:
        pass
    try:
        cam.close()
    except Exception:
        pass


# ----------------------------------------------------------------------------
# Детекция и оценка цели
# ----------------------------------------------------------------------------


@dataclass
class Detection:
    marker_id: int
    corners: np.ndarray  # (4, 2) float32, полное разрешение
    side_px: float


@dataclass
class Target:
    marker_id: int
    marker_size: float
    body_x: float
    body_y: float
    body_z: float
    angle_x: float
    angle_y: float
    distance: float
    reproj_px: float
    stamp: float


class ArucoTracker:
    """Детекция, выбор метки по приоритету, поза и пересчёт на цель ID22."""

    def __init__(self, cfg: Config, calib: Calibration, r_body_cam: np.ndarray):
        if cv is None:
            raise RuntimeError("OpenCV не установлен")
        self.cfg = cfg
        self.calib = calib
        self.r_body_cam = r_body_cam
        self.last_detections: List[Detection] = []
        self._filtered: Optional[np.ndarray] = None

        dictionary = cv.aruco.getPredefinedDictionary(
            getattr(cv.aruco, cfg.aruco_dict_name)
        )
        p = cv.aruco.DetectorParameters()
        # Мелкие метки: узкое адаптивное окно + низкий порог периметра
        p.adaptiveThreshWinSizeMin = 5
        p.adaptiveThreshWinSizeMax = 23
        p.adaptiveThreshWinSizeStep = 6
        p.minMarkerPerimeterRate = 0.02
        p.maxMarkerPerimeterRate = 4.0
        p.polygonalApproxAccuracyRate = 0.03
        p.errorCorrectionRate = 0.6
        p.minCornerDistanceRate = 0.02
        p.minDistanceToBorder = 2
        if cfg.corner_refine:
            p.cornerRefinementMethod = cv.aruco.CORNER_REFINE_SUBPIX
            p.cornerRefinementWinSize = 5
            p.cornerRefinementMaxIterations = 30
            p.cornerRefinementMinAccuracy = 0.05
        else:
            p.cornerRefinementMethod = cv.aruco.CORNER_REFINE_NONE
        self.detector = cv.aruco.ArucoDetector(dictionary, p)

    # -- детекция ------------------------------------------------------------
    def _detect(self, gray: np.ndarray) -> List[Detection]:
        scale = self.cfg.detect_scale
        if scale != 1.0:
            small = cv.resize(gray, None, fx=scale, fy=scale, interpolation=cv.INTER_AREA)
        else:
            small = gray

        corners, ids, _ = self.detector.detectMarkers(small)
        out: List[Detection] = []
        if ids is None:
            return out

        for i, raw_id in enumerate(ids.flatten()):
            mid = int(raw_id)
            if mid not in self.cfg.marker_sizes:
                continue
            c = corners[i].reshape(4, 2).astype(np.float64) / scale
            side = 0.5 * (
                np.linalg.norm(c[1] - c[0]) + np.linalg.norm(c[2] - c[1])
            )
            if side < self.cfg.min_marker_px:
                continue
            out.append(Detection(marker_id=mid, corners=c.astype(np.float32), side_px=float(side)))
        return out

    # -- поза ----------------------------------------------------------------
    def _target_from_detection(self, det: Detection) -> Optional[Target]:
        size = self.cfg.marker_sizes[det.marker_id]
        obj = marker_object_points(size)
        img = det.corners.reshape(4, 1, 2).astype(np.float32)

        try:
            ok, rvec, tvec = cv.solvePnP(
                obj, img, self.calib.K, self.calib.D, flags=cv.SOLVEPNP_IPPE_SQUARE
            )
        except cv.error as exc:
            log(f"[POSE] solvePnP id={det.marker_id}: {exc}")
            return None
        if not ok:
            return None

        t = np.asarray(tvec, dtype=np.float64).reshape(3)
        z_cam = float(t[2])
        if not (self.cfg.min_range_m <= z_cam <= self.cfg.max_range_m):
            return None

        try:
            proj, _ = cv.projectPoints(obj, rvec, tvec, self.calib.K, self.calib.D)
        except cv.error:
            return None
        err = float(
            np.sqrt(np.mean(np.sum((proj.reshape(4, 2) - det.corners) ** 2, axis=1)))
        )
        if err > self.cfg.max_reproj_px:
            return None

        r_mc = cv.Rodrigues(rvec)[0]
        p_cam = target_point_camera(r_mc, t, self.cfg.offsets.get(det.marker_id, (0.0, 0.0)))
        p_body = self.r_body_cam @ p_cam

        angles = angles_from_body(p_body)
        if angles is None:
            return None
        ax, ay, dist = angles
        if abs(ax) > self.cfg.max_angle_rad or abs(ay) > self.cfg.max_angle_rad:
            return None

        return Target(
            marker_id=det.marker_id,
            marker_size=size,
            body_x=float(p_body[0]),
            body_y=float(p_body[1]),
            body_z=float(p_body[2]),
            angle_x=ax,
            angle_y=ay,
            distance=dist,
            reproj_px=err,
            stamp=time.monotonic(),
        )

    # -- сглаживание ---------------------------------------------------------
    def _smooth(self, p: np.ndarray) -> np.ndarray:
        alpha = min(1.0, max(0.05, self.cfg.smooth_alpha))
        if self._filtered is None or np.linalg.norm(p - self._filtered) > self.cfg.jump_reset_m:
            self._filtered = p.copy()
        elif alpha < 1.0:
            self._filtered = self._filtered + alpha * (p - self._filtered)
        else:
            self._filtered = p.copy()
        return self._filtered.copy()

    def reset(self) -> None:
        self._filtered = None

    # -- основной вход -------------------------------------------------------
    def process(self, gray: np.ndarray) -> Optional[Target]:
        dets = self._detect(gray)
        self.last_detections = dets

        best: Optional[Target] = None
        for mid in self.cfg.priority:
            det = next((d for d in dets if d.marker_id == mid), None)
            if det is None:
                continue
            candidate = self._target_from_detection(det)
            if candidate is not None:  # поза не сошлась — пробуем следующую метку
                best = candidate
                break

        if best is None:
            return None

        p = self._smooth(np.array([best.body_x, best.body_y, best.body_z], dtype=np.float64))
        angles = angles_from_body(p)
        if angles is None:
            return None
        best.body_x, best.body_y, best.body_z = float(p[0]), float(p[1]), float(p[2])
        best.angle_x, best.angle_y, best.distance = angles
        return best


# ----------------------------------------------------------------------------
# MAVLink
# ----------------------------------------------------------------------------


@dataclass
class Telemetry:
    altitude: float = 0.0
    alt_source: str = "-"
    armed: bool = False
    mode: str = "-"
    link_ok: bool = False
    rx_age: float = 1e9


class MavlinkNode:
    """Один поток на всё: чтение телеметрии + отправка landing_target.

    Объект pymavlink не потокобезопасен, поэтому сокет не трогает никто, кроме
    этого потока. Главный цикл только кладёт последнюю цель в atomic-слот.
    """

    TYPES = [
        "HEARTBEAT",
        "DISTANCE_SENSOR",
        "LOCAL_POSITION_NED",
        "GLOBAL_POSITION_INT",
    ]

    def __init__(self, cfg: Config):
        self.cfg = cfg
        self._lock = threading.Lock()
        self._target: Optional[Target] = None
        self._tele = Telemetry()
        self._tele_lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._conn = None
        self._last_rx = 0.0

    # -- API -----------------------------------------------------------------
    def start(self) -> None:
        if mavutil is None:
            log("[MAV] pymavlink не установлен — MAVLink отключён")
            return
        self._thread = threading.Thread(target=self._run, name="mavlink", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)

    def publish(self, target: Target) -> None:
        with self._lock:
            self._target = target

    def telemetry(self) -> Telemetry:
        with self._tele_lock:
            t = replace(self._tele)
        t.rx_age = time.monotonic() - self._last_rx if self._last_rx else 1e9
        t.link_ok = t.rx_age < self.cfg.link_timeout_s
        return t

    def link_ok(self) -> bool:
        return self.telemetry().link_ok

    # -- поток ---------------------------------------------------------------
    def _run(self) -> None:
        while not self._stop.is_set():
            try:
                self._conn = self._connect()
                log(f"[MAV] соединение установлено: {self.cfg.mav_url}")
                self._loop()
            except Exception as exc:
                log(f"[MAV] {exc}")
            finally:
                if self._conn is not None:
                    try:
                        self._conn.close()
                    except Exception:
                        pass
                    self._conn = None
                with self._tele_lock:
                    self._tele.link_ok = False
            self._stop.wait(1.0)

    def _connect(self):
        conn = mavutil.mavlink_connection(
            self.cfg.mav_url, autoreconnect=self.cfg.mav_url.startswith("tcp")
        )
        hb = conn.wait_heartbeat(timeout=self.cfg.connect_timeout_s)
        if hb is None:
            conn.close()
            raise RuntimeError(f"нет heartbeat за {self.cfg.connect_timeout_s:.0f} c")
        conn.target_system = hb.get_srcSystem()
        conn.target_component = hb.get_srcComponent()
        self._last_rx = time.monotonic()
        return conn

    def _loop(self) -> None:
        period = 1.0 / max(1.0, self.cfg.send_hz)
        next_send = time.monotonic()
        next_hb = time.monotonic()

        while not self._stop.is_set():
            now = time.monotonic()

            if now >= next_send:
                self._send_target(now)
                next_send += period
                if next_send <= now:  # не копим «долг» при подвисаниях
                    next_send = now + period

            if self.cfg.send_heartbeat and now >= next_hb:
                self._conn.mav.heartbeat_send(
                    mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
                    mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                    0,
                    0,
                    0,
                )
                next_hb = now + 1.0

            msg = self._conn.recv_match(type=self.TYPES, blocking=True, timeout=0.02)
            if msg is not None:
                self._last_rx = time.monotonic()
                self._handle(msg)

            if time.monotonic() - self._last_rx > self.cfg.link_timeout_s:
                raise RuntimeError("потеря связи с полётным контроллером")

    def _handle(self, msg) -> None:
        mtype = msg.get_type()
        with self._tele_lock:
            if mtype == "HEARTBEAT":
                self._tele.armed = bool(
                    msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
                )
                self._tele.mode = mavutil.mode_string_v10(msg)
            elif mtype == "DISTANCE_SENSOR":
                # orientation 25 = MAV_SENSOR_ORIENTATION_DOWN
                if getattr(msg, "orientation", 25) == 25 and msg.current_distance > 0:
                    self._tele.altitude = msg.current_distance / 100.0
                    self._tele.alt_source = "rng"
            elif mtype == "LOCAL_POSITION_NED":
                if -msg.z >= 0:
                    self._tele.altitude = float(-msg.z)
                    self._tele.alt_source = "ekf"
            elif mtype == "GLOBAL_POSITION_INT":
                if self._tele.alt_source == "-" and msg.relative_alt > 0:
                    self._tele.altitude = msg.relative_alt / 1000.0
                    self._tele.alt_source = "gps"

    def _send_target(self, now: float) -> None:
        with self._lock:
            tgt = self._target
        if tgt is None:
            return
        if now - tgt.stamp > self.cfg.target_hold_s:
            return

        if self.cfg.use_full_pose:
            x, y, z = tgt.body_x, tgt.body_y, tgt.body_z
            valid = 1
        else:
            x = y = z = 0.0
            valid = 0

        self._conn.mav.landing_target_send(
            int(time.time() * 1e6),
            0,
            mavutil.mavlink.MAV_FRAME_BODY_FRD,
            float(tgt.angle_x),
            float(tgt.angle_y),
            float(tgt.distance) if self.cfg.send_distance else 0.0,
            float(tgt.marker_size),
            float(tgt.marker_size),
            x,
            y,
            z,
            [1.0, 0.0, 0.0, 0.0],
            mavutil.mavlink.LANDING_TARGET_TYPE_VISION_FIDUCIAL,
            valid,
        )


# ----------------------------------------------------------------------------
# Общий статус (для веба, HUD и watchdog)
# ----------------------------------------------------------------------------


class Status:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._data: Dict[str, object] = {
            "fps": 0.0,
            "frames": 0,
            "detected": 0,
            "locked": 0,
            "errors": 0,
            "last_error": "",
            "marker": None,
            "angle_x": 0.0,
            "angle_y": 0.0,
            "distance": 0.0,
            "alt": 0.0,
            "alt_src": "-",
            "mode": "-",
            "armed": False,
            "link": False,
            "clients": 0,
            "camera_ok": False,
        }
        self._last_iter = time.monotonic()

    def update(self, **kwargs) -> None:
        with self._lock:
            self._data.update(kwargs)
            self._last_iter = time.monotonic()

    def bump(self, key: str, delta: int = 1) -> None:
        with self._lock:
            try:
                self._data[key] = int(self._data.get(key, 0)) + delta
            except (TypeError, ValueError):
                self._data[key] = delta

    def beat(self) -> None:
        with self._lock:
            self._last_iter = time.monotonic()

    def stall_age(self) -> float:
        with self._lock:
            return time.monotonic() - self._last_iter

    def snapshot(self) -> Dict[str, object]:
        with self._lock:
            return dict(self._data)


# ----------------------------------------------------------------------------
# Веб: один энкодер на всех клиентов + ThreadingHTTPServer
# ----------------------------------------------------------------------------


class FrameHub:
    """Хранит последний холст, кодирует JPEG один раз и раздаёт всем клиентам."""

    def __init__(self, cfg: Config):
        self.cfg = cfg
        self._cond = threading.Condition()
        self._canvas: Optional[np.ndarray] = None
        self._seq = 0
        self._jpeg = b""
        self._jpeg_seq = -1
        self._clients = 0
        self._stop = False
        self._thread: Optional[threading.Thread] = None
        self.encoded_frames = 0

    def start(self) -> None:
        self._thread = threading.Thread(target=self._encode_loop, name="jpeg", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        with self._cond:
            self._stop = True
            self._cond.notify_all()

    @property
    def clients(self) -> int:
        with self._cond:
            return self._clients

    def publish(self, canvas: np.ndarray) -> None:
        # canvas создаётся заново на каждый кадр и после публикации не меняется,
        # поэтому копия не нужна (иначе 921 КБ memcpy на каждый кадр).
        with self._cond:
            self._canvas = canvas
            self._seq += 1
            self._cond.notify_all()

    def acquire_client(self) -> bool:
        with self._cond:
            if self._clients >= self.cfg.max_clients:
                return False
            self._clients += 1
            self._cond.notify_all()
            return True

    def release_client(self) -> None:
        with self._cond:
            self._clients = max(0, self._clients - 1)
            self._cond.notify_all()

    def wait_jpeg(self, since: int, timeout: float) -> Tuple[Optional[bytes], int]:
        deadline = time.monotonic() + timeout
        with self._cond:
            while not self._stop and self._jpeg_seq == since:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    return None, since
                self._cond.wait(remaining)
            if self._stop:
                return None, since
            return self._jpeg, self._jpeg_seq

    def _encode_loop(self) -> None:
        min_period = 1.0 / max(1.0, self.cfg.stream_fps)
        last_encoded_at = 0.0
        while True:
            with self._cond:
                while not self._stop and (
                    self._clients <= 0
                    or self._canvas is None
                    or self._seq == self._jpeg_seq
                ):
                    self._cond.wait(0.5)
                if self._stop:
                    return
                canvas, seq = self._canvas, self._seq

            # ограничение FPS: не жжём CPU, если кадры приходят чаще
            delay = min_period - (time.monotonic() - last_encoded_at)
            if delay > 0:
                time.sleep(delay)
                continue

            ok, buf = cv.imencode(
                ".jpg", canvas, [cv.IMWRITE_JPEG_QUALITY, int(self.cfg.jpeg_quality)]
            )
            last_encoded_at = time.monotonic()
            if not ok:
                continue
            data = buf.tobytes()
            with self._cond:
                self._jpeg = data
                self._jpeg_seq = seq
                self.encoded_frames += 1
                self._cond.notify_all()


class ServerContext:
    def __init__(self, cfg: Config, hub: FrameHub, status: Status):
        self.cfg = cfg
        self.hub = hub
        self.status = status


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


class StreamHandler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.0"
    server_version = "pl-v2/1.0"
    ctx: ServerContext = None  # подставляется перед созданием сервера

    # -- утилиты -------------------------------------------------------------
    def log_message(self, fmt, *args):  # тишина в консоли
        pass

    def _send_plain(self, code: int, body: bytes, ctype: str = "text/plain; charset=utf-8") -> None:
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
                    # кадров нет: если главный цикл жив, они появятся;
                    # если нет — рвём соединение, страница переподключится
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
        except (BrokenPipeError, ConnectionResetError, TimeoutError, socket.timeout, OSError):
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
    log(f"[HTTP] http://0.0.0.0:{cfg.http_port}  (/stream, /status)")
    return server


# ----------------------------------------------------------------------------
# Отрисовка холста
# ----------------------------------------------------------------------------

ID_COLORS = {22: (80, 220, 80), 33: (80, 220, 220), 44: (220, 160, 80)}


class Renderer:
    """Лёгкий векторный холст: метки, прицел, компас осей корпуса, HUD."""

    def __init__(self, cfg: Config, calib: Calibration, r_body_cam: np.ndarray):
        self.cfg = cfg
        self.calib = calib
        self.r_body_cam = r_body_cam

    def _blank(self) -> np.ndarray:
        w, h = self.cfg.frame_size
        return np.full((h, w, 3), CANVAS_BG, dtype=np.uint8)

    def message(self, text: str) -> np.ndarray:
        img = self._blank()
        cv.putText(img, text, (16, 40), cv.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 1, cv.LINE_AA)
        return img

    def draw(
        self,
        detections: Sequence[Detection],
        target: Optional[Target],
        status: Dict[str, object],
        link_ok: bool,
        clients: int,
    ) -> np.ndarray:
        img = self._blank()
        w, h = self.cfg.frame_size
        cx, cy = int(round(self.calib.cx)), int(round(self.calib.cy))

        # прицел оптической оси
        cv.drawMarker(img, (cx, cy), (70, 70, 70), cv.MARKER_CROSS, 18, 1)

        # метки
        for det in detections:
            color = ID_COLORS.get(det.marker_id, (170, 170, 170))
            thick = 2 if target is not None and det.marker_id == target.marker_id else 1
            pts = det.corners.astype(np.int32).reshape(-1, 1, 2)
            cv.polylines(img, [pts], True, color, thick, cv.LINE_AA)
            cv.putText(
                img,
                str(det.marker_id),
                (int(det.corners[0, 0]), int(det.corners[0, 1]) - 5),
                cv.FONT_HERSHEY_SIMPLEX,
                0.42,
                color,
                1,
                cv.LINE_AA,
            )

        # цель: направление на неё в пикселях кадра (без дисторсии — только для глаз)
        if target is not None:
            p_cam = self.r_body_cam.T @ np.array(
                [target.body_x, target.body_y, target.body_z], dtype=np.float64
            )
            if p_cam[2] > 1e-6:
                u = self.calib.cx + self.calib.fx * p_cam[0] / p_cam[2]
                v = self.calib.cy + self.calib.fy * p_cam[1] / p_cam[2]
                u = int(max(-2000, min(2000, u)))
                v = int(max(-2000, min(2000, v)))
                cv.line(img, (cx, cy), (u, v), (0, 120, 255), 1, cv.LINE_AA)
                cv.circle(img, (u, v), 9, (0, 0, 255), 2, cv.LINE_AA)
                cv.circle(img, (u, v), 3, (0, 0, 255), -1)

        # компас: куда в кадре смотрят нос (F) и правый борт (R)
        origin = (w - 34, h - 30)
        for vec, label, color in (
            (np.array([1.0, 0.0, 0.0]), "F", (0, 200, 255)),
            (np.array([0.0, 1.0, 0.0]), "R", (0, 255, 0)),
        ):
            d = self.r_body_cam.T @ vec
            n = math.hypot(d[0], d[1])
            if n < 1e-6:
                continue
            end = (
                int(origin[0] + 22 * d[0] / n),
                int(origin[1] + 22 * d[1] / n),
            )
            cv.arrowedLine(img, origin, end, color, 1, cv.LINE_AA, tipLength=0.35)
            cv.putText(
                img,
                label,
                (end[0] + 2, end[1] + 4),
                cv.FONT_HERSHEY_SIMPLEX,
                0.38,
                color,
                1,
                cv.LINE_AA,
            )

        # HUD
        if target is not None:
            line1 = (
                f"ID {target.marker_id}  ax {target.angle_x:+.3f}  ay {target.angle_y:+.3f} rad  "
                f"d {target.distance:.2f} m"
            )
        else:
            line1 = "NO TARGET"
        line2 = (
            f"alt {float(status.get('alt', 0.0)):.2f} m ({status.get('alt_src', '-')})  "
            f"{float(status.get('fps', 0.0)):.1f} fps  {status.get('mode', '-')}"
        )
        line3 = f"link {'OK' if link_ok else 'LOST'}  clients {clients}  id {status.get('marker') or '--'}"
        for i, text in enumerate((line1, line2, line3)):
            cv.putText(
                img,
                text,
                (8, 18 + i * 16),
                cv.FONT_HERSHEY_SIMPLEX,
                0.42,
                (230, 230, 230) if target is not None else (120, 120, 120),
                1,
                cv.LINE_AA,
            )
        return img


# ----------------------------------------------------------------------------
# Самопроверка геометрии (python3 ... --selftest)
# ----------------------------------------------------------------------------


def run_selftest() -> int:
    failures: List[str] = []

    def check(name: str, cond: bool, detail: str = "") -> None:
        if cond:
            print(f"  PASS  {name}")
        else:
            print(f"  FAIL  {name}  {detail}")
            failures.append(name)

    print("Проверка матриц монтажа")
    for yaw in (0.0, 90.0, 180.0, 270.0, -90.0, 45.0):
        r = mount_matrix(yaw)
        check(
            f"ортогональность yaw={yaw:g}",
            np.allclose(r @ r.T, np.eye(3), atol=1e-9) and abs(np.linalg.det(r) - 1) < 1e-9,
        )

    r0 = mount_matrix(0.0)
    up = r0.T @ np.array([1.0, 0.0, 0.0])
    right = r0.T @ np.array([0.0, 1.0, 0.0])
    check("yaw=0: нос смотрит вверх кадра", np.allclose(up, [0, -1, 0], atol=1e-9), str(up))
    check("yaw=0: правый борт вправо кадра", np.allclose(right, [1, 0, 0], atol=1e-9), str(right))

    legacy = mount_matrix(270.0)
    check("yaw=270 воспроизводит старую логику (право кадра = нос)",
          np.allclose(legacy, np.eye(3), atol=1e-9), str(legacy))

    yaw90 = mount_matrix(90.0)
    check(
        "yaw=90: верх кадра смотрит вправо",
        np.allclose(yaw90 @ np.array([0.0, -1.0, 0.0]), [0, 1, 0], atol=1e-9),
        str(yaw90 @ np.array([0, -1.0, 0])),
    )

    print("Проверка пересчёта смещения")
    t = np.array([0.0, 0.0, 1.0])
    p = target_point_camera(np.eye(3), t, (0.0, 0.0))
    check("нулевое смещение не двигает цель", np.allclose(p, t), str(p))
    p = target_point_camera(np.eye(3), t, (0.05, -0.03))
    check("смещение идёт в сторону ID22", np.allclose(p, [-0.05, 0.03, 1.0]), str(p))

    # поворот листа на 90° вокруг нормали: поправка должна повернуться вместе с ним
    r90 = _rot_z(math.pi / 2)
    p_rot = target_point_camera(r90, t, (0.05, 0.0))
    check("поворот листа учитывается", np.allclose(p_rot, [0.0, -0.05, 1.0], atol=1e-9), str(p_rot))

    print("Проверка углов")
    ax, ay, dist = angles_from_body(np.array([0.0, 0.0, 1.0]))
    check("цель по центру -> углы 0", abs(ax) < 1e-12 and abs(ay) < 1e-12 and abs(dist - 1) < 1e-12)

    ax, ay, _ = angles_from_body(np.array([1.0, 0.0, 1.0]))
    check("цель впереди -> angle_x > 0", ax > 0 and abs(ay) < 1e-12, f"{ax:.3f}")

    ax, ay, _ = angles_from_body(np.array([0.0, 1.0, 1.0]))
    check("цель справа -> angle_y > 0", ay > 0 and abs(ax) < 1e-12, f"{ay:.3f}")

    ax1, _, _ = angles_from_body(np.array([0.1, 0.0, 1.0]))
    ax2, _, _ = angles_from_body(np.array([0.1, 0.0, 2.0]))
    check("угол падает с высотой", ax1 > ax2 > 0, f"{ax1:.3f} {ax2:.3f}")

    print()
    if failures:
        print(f"FAILED: {len(failures)}")
        return 1
    print("OK: все проверки пройдены")
    return 0


# ----------------------------------------------------------------------------
# Главный цикл
# ----------------------------------------------------------------------------


def _watchdog(stop: threading.Event, status: Status, timeout: float) -> None:
    """Если главный цикл встал (камера/сокет) — аварийный выход.

    Перезапуск процесса оставляем systemd (Restart=always): это надёжнее,
    чем пытаться «оживить» зависший поток из Python.
    """
    while not stop.wait(1.0):
        age = status.stall_age()
        if age > timeout:
            log(f"[WATCHDOG] главный цикл не отвечает {age:.0f} c — аварийный выход")
            os._exit(EXIT_WATCHDOG)


def _install_signals(stop: threading.Event) -> None:
    def handler(signum, _frame):
        log(f"[MAIN] сигнал {signum}, завершение")
        stop.set()

    for sig in (signal.SIGINT, signal.SIGTERM):
        try:
            signal.signal(sig, handler)
        except (ValueError, OSError):
            pass


def _diag_line(target: Optional[Target], tele: Telemetry) -> str:
    if target is None:
        return f"[DIAG] нет цели | link={'OK' if tele.link_ok else 'LOST'} alt={tele.altitude:.2f}"
    hint = []
    if abs(target.body_x) > 0.03:
        hint.append(f"{'FORWARD' if target.body_x > 0 else 'BACK'} {abs(target.body_x):.2f} m")
    if abs(target.body_y) > 0.03:
        hint.append(f"{'RIGHT' if target.body_y > 0 else 'LEFT'} {abs(target.body_y):.2f} m")
    where = ", ".join(hint) if hint else "по центру"
    return (
        f"[DIAG] id={target.marker_id} reproj={target.reproj_px:.1f}px "
        f"body=({target.body_x:+.2f},{target.body_y:+.2f},{target.body_z:.2f}) "
        f"ax={target.angle_x:+.3f} ay={target.angle_y:+.3f} -> target: {where}"
    )


def run(cfg: Config, args: argparse.Namespace) -> int:
    if cv is None:
        log("OpenCV не установлен: pip install opencv-python-headless")
        return 2

    calib = Calibration.load(cfg.calib_path, cfg.frame_size)
    r_body_cam = camera_to_body_matrix(cfg)
    log(
        f"[MOUNT] yaw={cfg.mount_yaw_deg:g} deg pitch={cfg.mount_pitch_deg:g} deg "
        f"roll={cfg.mount_roll_deg:g} deg"
    )

    status = Status()
    hub = FrameHub(cfg)
    hub.start()
    try:
        server = start_http_server(cfg, hub, status)
    except OSError as exc:
        log(f"[HTTP] не удалось занять порт {cfg.http_port}: {exc}")
        hub.stop()
        return 3
    renderer = Renderer(cfg, calib, r_body_cam)
    tracker = ArucoTracker(cfg, calib, r_body_cam)

    link = MavlinkNode(cfg)
    if not args.no_mavlink:
        link.start()

    cam = CameraSource(cfg)
    if not args.no_camera:
        if not cam.open():
            log("[CAM] камера не открылась, продолжаем попытки в фоне")

    stop = threading.Event()
    _install_signals(stop)
    threading.Thread(
        target=_watchdog, args=(stop, status, cfg.watchdog_s), name="wd", daemon=True
    ).start()

    log("[MAIN] старт")
    last_cam_try = 0.0
    last_log = 0.0
    last_nocam = 0.0
    next_diag = 0.0
    fps_n = 0
    fps_t0 = time.monotonic()
    fps = 0.0
    frames = 0

    while not stop.is_set():
        now = time.monotonic()
        frame = cam.read() if not args.no_camera else None

        if frame is None:
            status.bump("errors")
            status.update(
                camera_ok=False,
                clients=hub.clients,
                last_error=cam.last_error or ("камера отключена (--no-camera)" if args.no_camera else ""),
            )
            if now - last_cam_try > cfg.camera_retry_s and not args.no_camera:
                last_cam_try = now
                if not cam.is_open:
                    log("[CAM] попытка переоткрыть камеру")
                    cam.open()
            if now - last_nocam > 1.0:
                last_nocam = now
                hub.publish(renderer.message(cam.last_error or "NO CAMERA"))
            stop.wait(0.2)
            continue

        try:
            target = tracker.process(frame)
        except Exception as exc:  # один битый кадр не должен ронять посадку
            status.bump("errors")
            status.update(last_error=f"tracker: {exc}")
            log(f"[TRACKER] {exc}")
            target = None

        tele = link.telemetry()

        if target is not None:
            link.publish(target)

        frames += 1
        fps_n += 1
        dt = now - fps_t0
        if dt >= 1.0:
            fps = fps_n / dt
            fps_n = 0
            fps_t0 = now

        status.update(
            camera_ok=True,
            frames=frames,
            fps=round(fps, 1),
            detected=len(tracker.last_detections),
            marker=target.marker_id if target else None,
            angle_x=round(target.angle_x, 4) if target else 0.0,
            angle_y=round(target.angle_y, 4) if target else 0.0,
            distance=round(target.distance, 3) if target else 0.0,
            alt=round(tele.altitude, 2),
            alt_src=tele.alt_source,
            mode=tele.mode,
            armed=tele.armed,
            link=tele.link_ok,
            clients=hub.clients,
        )

        canvas = renderer.draw(
            tracker.last_detections, target, status.snapshot(), tele.link_ok, hub.clients
        )
        hub.publish(canvas)

        if target is not None and now - last_log >= cfg.log_period_s:
            last_log = now
            log(
                f"[LOCK] id={target.marker_id} ax={target.angle_x:+.3f} "
                f"ay={target.angle_y:+.3f} d={target.distance:.2f} m alt={tele.altitude:.2f} m"
            )

        if args.diag and now >= next_diag:
            next_diag = now + 0.2
            print(_diag_line(target, tele), flush=True)

    log("[MAIN] остановка")
    hub.stop()
    try:
        server.shutdown()
    except Exception:
        pass
    server.server_close()
    cam.close()
    link.stop()
    log("[MAIN] завершено")
    return 0


def parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Precision landing по ArUco (v2)")
    p.add_argument("--url", default=None, help="MAVLink URL, напр. udpout:127.0.0.1:14551")
    p.add_argument("--port", type=int, default=None, help="порт веб-сервера")
    p.add_argument("--mount-yaw", type=float, default=None, help="поворот камеры, градусы")
    p.add_argument("--no-mavlink", action="store_true", help="не подключаться к полётнику")
    p.add_argument("--no-camera", action="store_true", help="без камеры (проверка веба)")
    p.add_argument("--diag", action="store_true", help="вывод вектора цели 5 Гц")
    p.add_argument("--selftest", action="store_true", help="проверка геометрии и выход")
    return p.parse_args(argv)


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = parse_args(argv)
    if args.selftest:
        return run_selftest()

    cfg = Config()
    overrides = {}
    if args.url:
        overrides["mav_url"] = args.url
    if args.port:
        overrides["http_port"] = args.port
    if args.mount_yaw is not None:
        overrides["mount_yaw_deg"] = args.mount_yaw
    if overrides:
        cfg = replace(cfg, **overrides)

    try:
        return run(cfg, args)
    except FileNotFoundError as exc:
        log(f"[FATAL] {exc}")
        return 2
    except KeyboardInterrupt:
        return 0


if __name__ == "__main__":
    sys.exit(main())
