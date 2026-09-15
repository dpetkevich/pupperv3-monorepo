"""Bridge rclpy futures into asyncio without ever spinning the node from the event loop.

rclpy futures are not ``concurrent.futures``; ``asyncio.wrap_future`` does not work
on them. The pattern here is the only safe one when the node is spun by a
background executor thread: register a done-callback on the ROS thread and hop
back to the loop with ``call_soon_threadsafe``.
"""

from __future__ import annotations

import asyncio
from typing import Any


async def await_ros_future(ros_future: Any, timeout: float) -> Any:
    """Await an rclpy Future (or anything with add_done_callback/result) with a timeout.

    Raises asyncio.TimeoutError if the future has not completed in ``timeout`` seconds;
    the ROS future is cancelled on timeout when it supports it.
    """
    loop = asyncio.get_running_loop()
    afut: asyncio.Future[Any] = loop.create_future()

    def _done(f: Any) -> None:
        def _set() -> None:
            if afut.done():
                return
            try:
                afut.set_result(f.result())
            except Exception as e:  # noqa: BLE001 - propagate whatever ROS raised
                afut.set_exception(e)

        loop.call_soon_threadsafe(_set)

    ros_future.add_done_callback(_done)
    try:
        return await asyncio.wait_for(afut, timeout)
    except asyncio.TimeoutError:
        cancel = getattr(ros_future, "cancel", None)
        if callable(cancel):
            try:
                cancel()
            except Exception:  # noqa: BLE001
                pass
        raise
