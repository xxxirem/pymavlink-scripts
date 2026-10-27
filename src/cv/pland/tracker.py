"""Детекция ArUco, оценка позы и пересчёт на центр ID22.

Ключевая идея: смещение OFFSETS применяется не к пикселям, а к 3D-точке.
solvePnP даёт позу найденной метки (R, t) в системе камеры; центр ID22 —
это R * (-dx, -dy, 0) + t. Высота в этой формуле не участвует вообще,
поэтому нет ни деления на «почти ноль», ни взрыва поправки у земли.
"""

from __future__ import annotations

import time
from dataclasses import dataclass
from typing import List, Optional, Sequence

import numpy as np

from .calibration import Calibration
from .config import Config
from .geometry import angles_from_body, marker_object_points, target_point_camera
from .log import log

try:
    import cv2 as cv
except Exception:  # pragma: no cover
    cv = None


@dataclass
class Detection:
    marker_id: int
    corners: np.ndarray  # (4, 2) float32, координаты в полном разрешении
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
        small = (
            cv.resize(gray, None, fx=scale, fy=scale, interpolation=cv.INTER_AREA)
            if scale != 1.0
            else gray
        )

        corners, ids, _ = self.detector.detectMarkers(small)
        out: List[Detection] = []
        if ids is None:
            return out

        for i, raw_id in enumerate(ids.flatten()):
            mid = int(raw_id)
            if mid not in self.cfg.marker_sizes:
                continue
            c = corners[i].reshape(4, 2).astype(np.float64) / scale
            side = 0.5 * (np.linalg.norm(c[1] - c[0]) + np.linalg.norm(c[2] - c[1]))
            if side < self.cfg.min_marker_px:
                continue
            out.append(
                Detection(
                    marker_id=mid,
                    corners=c.astype(np.float32),
                    side_px=float(side),
                )
            )
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

        # Контроль качества позы: если углы метки не объясняют её же пиксели,
        # значит детекция ложная или углы «поплыли».
        try:
            proj, _ = cv.projectPoints(obj, rvec, tvec, self.calib.K, self.calib.D)
        except cv.error:
            return None
        err = float(np.sqrt(np.mean(np.sum((proj.reshape(4, 2) - det.corners) ** 2, axis=1))))
        if err > self.cfg.max_reproj_px:
            return None

        r_mc = cv.Rodrigues(rvec)[0]
        p_cam = target_point_camera(
            r_mc, t, self.cfg.offsets.get(det.marker_id, (0.0, 0.0))
        )
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


__all__: Sequence[str] = ("ArucoTracker", "Detection", "Target")
