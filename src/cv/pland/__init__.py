"""Precision landing по ArUco: пакет модулей.

    config.py       — все настройки (единственное место, которое правится руками)
    geometry.py     — матрицы монтажа камеры, пересчёт смещений и углов
    calibration.py  — загрузка camera.yaml
    camera.py       — Picamera2, захват яркости
    tracker.py      — детекция ArUco, поза, выбор метки, сглаживание
    mavlink_node.py — поток MAVLink: телеметрия + отправка landing_target
    hub.py          — статус и общий JPEG-конвейер для веба
    web.py          — HTTP-сервер (MJPEG, /status, страница)
    render.py       — отрисовка холста для стрима
    app.py          — главный цикл, watchdog, сигналы
    cli.py          — аргументы командной строки и точка входа
"""

__version__ = "2.0"
