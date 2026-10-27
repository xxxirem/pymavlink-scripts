"""Позволяет запускать пакет как `python3 -m pland`."""

from .cli import main

if __name__ == "__main__":
    raise SystemExit(main())
