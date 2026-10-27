"""Загрузка внутренних параметров камеры (camera.yaml из OpenCV FileStorage)."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Optional, Tuple

import numpy as np

from .log import log

try:
    import cv2 as cv
except Exception:  # pragma: no cover
    cv = None


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

        # Если разрешение захвата отличается от калибровочного — масштабируем K,
        # иначе все углы будут считаться в «чужих» пикселях.
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

        # Principal point должен быть примерно в центре кадра: если уехал,
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
