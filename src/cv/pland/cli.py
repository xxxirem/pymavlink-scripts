"""Аргументы командной строки и точка входа."""

from __future__ import annotations

import argparse
from dataclasses import replace
from typing import Optional, Sequence

from .config import Config
from .geometry import run_selftest
from .log import log


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

    from .app import run  # поздний импорт: --selftest работает без OpenCV

    try:
        return run(cfg, args)
    except FileNotFoundError as exc:
        log(f"[FATAL] {exc}")
        return 2
    except KeyboardInterrupt:
        return 0
