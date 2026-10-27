#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Precision landing по ArUco — точка входа.

Вся логика разложена по модулям в пакете `pland` (см. PRECISION_LANDING_V2.md):

    pland/config.py       — настройки (правятся руками)
    pland/geometry.py     — математика осей и смещений
    pland/calibration.py  — camera.yaml
    pland/camera.py       — Picamera2
    pland/tracker.py      — ArUco + поза + цель
    pland/mavlink_node.py — MAVLink
    pland/hub.py          — статус + JPEG-конвейер
    pland/web.py          — HTTP/MJPEG
    pland/render.py       — холст
    pland/app.py          — главный цикл
    pland/cli.py          — аргументы

Запуск (из любого каталога):
    python3 pl_web_centered_light_v2.py                 # рабочий режим
    python3 pl_web_centered_light_v2.py --diag          # + вектор цели 5 Гц
    python3 pl_web_centered_light_v2.py --selftest      # проверка геометрии
"""

import sys
from pathlib import Path

# пакет лежит рядом с этим файлом; добавляем каталог, чтобы `import pland`
# работал независимо от текущего каталога
_here = str(Path(__file__).resolve().parent)
if _here not in sys.path:
    sys.path.insert(0, _here)

from pland.cli import main  # noqa: E402

if __name__ == "__main__":
    sys.exit(main())