"""Камера: Picamera2, захват только яркости (Y-плоскость YUV420).

Зачем YUV420: Pi Zero 2 W не тянет лишние копии. RGB888 — это 921 КБ на кадр
плюс cvtColor в градации серого. Y-плоскость — 307 КБ и уже готовая яркость,
на которой ArUco работает не хуже.
"""

from __future__ import annotations

import time
from typing import Optional

import numpy as np

from .config import Config
from .log import log

try:
    from picamera2 import Picamera2
except Exception:  # pragma: no cover - на ПК модуля нет
    Picamera2 = None

try:
    from libcamera import controls as libcam
except Exception:  # pragma: no cover
    libcam = None


def _shutdown_cam(cam) -> None:
    """Погасить Picamera2, даже если объект не до конца инициализирован."""
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


def _to_gray(rgb: np.ndarray) -> np.ndarray:
    """RGB -> градации серого без OpenCV (быстрый целочисленный вариант)."""
    c = rgb[..., :3].astype(np.uint16)
    return ((c[..., 0] * 77 + c[..., 1] * 150 + c[..., 2] * 29) >> 8).astype(np.uint8)


class CameraSource:
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
            try:
                conf = cam.create_video_configuration(
                    main={"size": self.cfg.frame_size, "format": "YUV420"},
                    buffer_count=2,
                    controls={"FrameDurationLimits": (us, us)},
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
            log(
                f"[CAM] запущена {self.cfg.frame_size[0]}x{self.cfg.frame_size[1]} "
                f"@ {self.cfg.capture_fps} fps, YUV420={'нет' if self._convert else 'да'}"
            )
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
            return _to_gray(arr)
        # Y-плоскость YUV420: первые h строк, срез уже непрерывен
        return np.ascontiguousarray(arr[:h, :w])

    def close(self) -> None:
        cam, self._cam = self._cam, None
        _shutdown_cam(cam)
