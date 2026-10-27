"""Печать в консоль/syslog."""

from __future__ import annotations

import time


def log(msg: str) -> None:
    """Метка времени + принудительный flush (важно при запуске из systemd)."""
    print(f"[{time.strftime('%H:%M:%S')}] {msg}", flush=True)
