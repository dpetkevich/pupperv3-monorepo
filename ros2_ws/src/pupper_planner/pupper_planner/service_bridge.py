"""call_service: one rclpy service call from a worker thread without spinning."""
from __future__ import annotations

import threading
import time
from typing import Any, Optional


def call_service(client: Any, request: Any, timeout_s: float = 2.0, server_wait_s: float = 0.5) -> Optional[Any]:
    """Returns the response or None (unavailable / timed out)."""
    if not client.service_is_ready() and not client.wait_for_service(timeout_sec=server_wait_s):
        return None
    fut = client.call_async(request)
    ev = threading.Event()
    fut.add_done_callback(lambda _f: ev.set())
    if not ev.wait(timeout_s):
        return None
    try:
        return fut.result()
    except Exception:
        return None
