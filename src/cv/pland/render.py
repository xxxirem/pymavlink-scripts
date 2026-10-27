"""Отрисовка отладочного холста (без видео, только векторы — дёшево для Zero 2 W)."""

from __future__ import annotations

import math
from typing import Dict, Optional, Sequence

import numpy as np

from .calibration import Calibration
from .config import CANVAS_BG, Config
from .tracker import Detection, Target

try:
    import cv2 as cv
except Exception:  # pragma: no cover
    cv = None

ID_COLORS = {22: (80, 220, 80), 33: (80, 220, 220), 44: (220, 160, 80)}


class Renderer:
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

        # найденные метки
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

        # направление на цель (без дисторсии — только для глаз)
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
            end = (int(origin[0] + 22 * d[0] / n), int(origin[1] + 22 * d[1] / n))
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
        line3 = (
            f"link {'OK' if link_ok else 'LOST'}  clients {clients}  "
            f"id {status.get('marker') or '--'}"
        )
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
