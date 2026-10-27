"""Геометрия: системы координат, монтаж камеры, смещения меток.

Здесь нет ни OpenCV, ни камеры — только numpy, поэтому модуль можно
проверять на любом компьютере:  python3 -m pland.geometry --selftest

Системы координат
-----------------
Камера (OpenCV): x — вправо по кадру, y — вниз по кадру, z — вдоль объектива.
Корпус (FRD):    x — вперёд,          y — вправо,          z — вниз.

Метка (ArUco/solvePnP): x — вправо по листу, y — вверх по листу, z — из листа
навстречу камере. В этих осях и заданы OFFSETS в config.py.
"""

from __future__ import annotations

import math
from typing import Optional, Tuple

import numpy as np

from .config import Config

# Столбцы матрицы — оси камеры, выраженные в осях корпуса.
# Соответствует монтажу «камера смотрит вниз, верх кадра смотрит в нос».
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
    """Матрица поворота «камера → корпус»: p_body = R @ p_cam."""
    r = _rot_z(math.radians(yaw_deg))
    r = r @ _rot_y(math.radians(pitch_deg))
    r = r @ _rot_x(math.radians(roll_deg))
    return r @ BASE_CAM_TO_BODY


def camera_to_body_matrix(cfg: Config) -> np.ndarray:
    return mount_matrix(cfg.mount_yaw_deg, cfg.mount_pitch_deg, cfg.mount_roll_deg)


def marker_object_points(size: float) -> np.ndarray:
    """Углы метки в её системе координат.

    Порядок обязателен для SOLVEPNP_IPPE_SQUARE: ЛВ, ПВ, ПН, ЛН — он же
    совпадает с порядком corners из cv.aruco.detectMarkers.
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
    """Вектор цели в системе корпуса -> (angle_x, angle_y, distance).

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

    offset_xy — где центр этой метки находится относительно центра ID22
    (в метрах, +y вверх по листу). Нам нужен обратный вектор: от метки к ID22.
    """
    d = np.array([-float(offset_xy[0]), -float(offset_xy[1]), 0.0])
    return r_marker_cam @ d + np.asarray(t_marker_cam, dtype=np.float64).reshape(3)


# ---------------------------------------------------------------------------
# Самопроверка
# ---------------------------------------------------------------------------


def run_selftest() -> int:
    failures = []

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
    check(
        "yaw=270 воспроизводит старую логику (право кадра = нос)",
        np.allclose(legacy, np.eye(3), atol=1e-9),
        str(legacy),
    )

    yaw90 = mount_matrix(90.0)
    check(
        "yaw=90: верх кадра смотрит вправо",
        np.allclose(yaw90 @ np.array([0.0, -1.0, 0.0]), [0, 1, 0], atol=1e-9),
        str(yaw90 @ np.array([0.0, -1.0, 0.0])),
    )

    print("Проверка пересчёта смещения")
    t = np.array([0.0, 0.0, 1.0])
    p = target_point_camera(np.eye(3), t, (0.0, 0.0))
    check("нулевое смещение не двигает цель", np.allclose(p, t), str(p))
    p = target_point_camera(np.eye(3), t, (0.05, -0.03))
    check("смещение идёт в сторону ID22", np.allclose(p, [-0.05, 0.03, 1.0]), str(p))

    # лист повёрнут на 90° вокруг нормали — поправка должна повернуться вместе с ним
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


if __name__ == "__main__":
    raise SystemExit(run_selftest())
