"""Главный цикл: камера -> детекция -> цель -> холст -> веб."""

from __future__ import annotations

import argparse
import os
import signal
import threading
import time
from typing import Optional

from .calibration import Calibration
from .camera import CameraSource
from .config import EXIT_WATCHDOG, Config
from .geometry import camera_to_body_matrix
from .hub import FrameHub, Status
from .log import log
from .mavlink_node import MavlinkNode, Telemetry
from .render import Renderer
from .tracker import ArucoTracker, Target
from .web import start_http_server

try:
    import cv2 as cv
except Exception:  # pragma: no cover
    cv = None


def _watchdog(stop: threading.Event, status: Status, timeout: float) -> None:
    """Если главный цикл встал (камера/сокет) — аварийный выход.

    Перезапуск оставляет systemd (Restart=always): это надёжнее, чем пытаться
    оживить зависший поток из Python.
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
    if not args.no_camera and not cam.open():
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
                last_error=cam.last_error
                or ("камера отключена (--no-camera)" if args.no_camera else ""),
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
