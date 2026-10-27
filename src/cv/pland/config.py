#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Все настройки системы в одном месте.

Правится только этот файл (или аргументы командной строки для трёх параметров).
Смысл параметров описан в PRECISION_LANDING_V2.md.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, Tuple

# src/cv/pland/config.py -> PACKAGE_DIR = src/cv/pland, BASE_DIR = src/cv
PACKAGE_DIR = Path(__file__).resolve().parent
BASE_DIR = PACKAGE_DIR.parent

CANVAS_BG = (0, 0, 0)  # фон отладочного стрима (BGR)
EXIT_WATCHDOG = 3  # код выхода при зависании главного цикла


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
