"""send_and_wait: run one rclpy action goal from a worker thread without spinning.

Uses ``send_goal_async``/``get_result_async`` futures with ``add_done_callback`` + ``threading.Event``.
Cancels the child goal when ``cancel_event`` is set or the deadline passes. Never calls spin.
"""
from __future__ import annotations

import threading
import time
from typing import Any, Callable, Optional, Tuple

STATUS_SUCCEEDED = 4
STATUS_CANCELED = 5
STATUS_ABORTED = 6

CANCEL_GRACE_S = 3.0


def _wait(future: Any, cancel: Optional[threading.Event], deadline_mono: float, poll_s: float = 0.05) -> str:
    """Wait for a future. Returns "done" | "cancel" | "timeout"."""
    ev = threading.Event()
    future.add_done_callback(lambda _f: ev.set())
    while not ev.is_set():
        if cancel is not None and cancel.is_set():
            return "cancel"
        if time.monotonic() >= deadline_mono:
            return "timeout"
        ev.wait(poll_s)
    return "done"


def status_name(code: int) -> str:
    return {STATUS_SUCCEEDED: "SUCCEEDED", STATUS_CANCELED: "CANCELED", STATUS_ABORTED: "ABORTED"}.get(code, f"STATUS_{code}")


def send_and_wait(
    client: Any,
    goal: Any,
    on_feedback: Optional[Callable[[Any], None]],
    deadline_mono: float,
    cancel_event: Optional[threading.Event],
    server_wait_s: float = 1.0,
) -> Tuple[str, Any]:
    """Returns (status, result). status: SUCCEEDED | CANCELED | ABORTED | REJECTED | TIMEOUT | UNAVAILABLE.

    On TIMEOUT/CANCELED the child goal is cancelled and we wait up to CANCEL_GRACE_S for it to wind down so the
    motion server has published its zero before the next step starts.
    """
    if not client.server_is_ready() and not client.wait_for_server(timeout_sec=server_wait_s):
        return "UNAVAILABLE", None

    def fb_cb(msg: Any) -> None:
        if on_feedback is not None:
            try:
                on_feedback(msg.feedback)
            except Exception:
                pass

    gh_future = client.send_goal_async(goal, feedback_callback=fb_cb)
    st = _wait(gh_future, cancel_event, min(deadline_mono, time.monotonic() + 2.0))
    if st != "done":
        return ("CANCELED" if st == "cancel" else "TIMEOUT"), None
    goal_handle = gh_future.result()
    if goal_handle is None or not goal_handle.accepted:
        return "REJECTED", None

    res_future = goal_handle.get_result_async()
    st = _wait(res_future, cancel_event, deadline_mono)
    if st == "done":
        wrapped = res_future.result()
        return status_name(wrapped.status), wrapped.result

    # cancel or timeout: cancel the child and wait a bounded time for the result
    try:
        cancel_future = goal_handle.cancel_goal_async()
        _wait(cancel_future, None, time.monotonic() + CANCEL_GRACE_S)
    except Exception:
        pass
    st2 = _wait(res_future, None, time.monotonic() + CANCEL_GRACE_S)
    result = res_future.result().result if st2 == "done" else None
    return ("CANCELED" if st == "cancel" else "TIMEOUT"), result
