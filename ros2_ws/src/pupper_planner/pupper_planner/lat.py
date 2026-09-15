"""Latency instrumentation: one grep-able line per stage, joined later by tools/latency_report.py."""
from __future__ import annotations

import datetime
import time


def lat_line(evt: str, key: str = "") -> str:
    return f"LAT evt={evt} t={time.monotonic_ns()} wall={datetime.datetime.now().isoformat(timespec='milliseconds')} key={key}"
