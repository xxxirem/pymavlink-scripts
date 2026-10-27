"""Статус для веба/лога и общий JPEG-конвейер.

FrameHub — единственное место, где кадр превращается в JPEG. Раньше каждый
клиент копировал холст (921 КБ) и кодировал его заново: три вкладки браузера
= три кодирования на каждые 50 мс. Здесь кодирование одно, кадры не копятся
(клиент всегда получает последний), поэтому память не растёт.
"""

from __future__ import annotations

import threading
import time
from typing import Dict, Optional, Tuple

import numpy as np

from .config import Config

try:
    import cv2 as cv
except Exception:  # pragma: no cover
    cv = None


class Status:
    """Потокобезопасный словарь состояния + отметка «главный цикл жив»."""

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

    def stall_age(self) -> float:
        with self._lock:
            return time.monotonic() - self._last_iter

    def snapshot(self) -> Dict[str, object]:
        with self._lock:
            return dict(self._data)


class FrameHub:
    """Последний холст -> один JPEG -> все клиенты."""

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
