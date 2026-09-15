"""ExecutorBridge behaviour with rclpy and the interface packages mocked out."""

from __future__ import annotations

import asyncio
import sys
import time
from types import SimpleNamespace
from unittest.mock import MagicMock

import pytest

for name in (
    "rclpy", "rclpy.action", "rclpy.executors", "rclpy.node", "rclpy.qos",
    "controller_manager_msgs", "controller_manager_msgs.srv",
    "std_msgs", "std_msgs.msg", "std_srvs", "std_srvs.srv",
    "pupper_interfaces", "pupper_interfaces.action", "pupper_interfaces.msg", "pupper_interfaces.srv",
):
    sys.modules[name] = MagicMock()
sys.modules["rclpy"].ok.return_value = True

import executor_bridge  # noqa: E402
from executor_bridge import ExecutorBridge  # noqa: E402
from ros_async import await_ros_future  # noqa: E402


class FakeRosFuture:
    """Minimal stand-in for rclpy.task.Future."""

    def __init__(self) -> None:
        self._cbs: list = []
        self._result = None
        self._exc: Exception | None = None
        self._done = False
        self.cancelled = False

    def add_done_callback(self, cb) -> None:
        if self._done:
            cb(self)
        else:
            self._cbs.append(cb)

    def set_result(self, r) -> None:
        self._result, self._done = r, True
        for cb in self._cbs:
            cb(self)

    def set_exception(self, e: Exception) -> None:
        self._exc, self._done = e, True
        for cb in self._cbs:
            cb(self)

    def result(self):
        if self._exc:
            raise self._exc
        return self._result

    def cancel(self) -> None:
        self.cancelled = True


@pytest.fixture
def bridge() -> ExecutorBridge:
    b = ExecutorBridge()
    for client in (b._executor_stop, b._motion_stop, b._follow_off, b._follow_on, b._look, b._switch, b._executor_prepare):
        client.service_is_ready.return_value = False
        client.srv_name = "fake"
    b._execute_plan.server_is_ready.return_value = True
    return b


# --------------------------------------------------------------------- ros_async
async def test_await_ros_future_resolves_from_other_thread() -> None:
    fut = FakeRosFuture()
    loop = asyncio.get_running_loop()
    loop.call_later(0.02, lambda: __import__("threading").Thread(target=fut.set_result, args=(42,)).start())
    assert await await_ros_future(fut, timeout=1.0) == 42


async def test_await_ros_future_times_out_and_cancels() -> None:
    fut = FakeRosFuture()
    t0 = time.monotonic()
    with pytest.raises(asyncio.TimeoutError):
        await await_ros_future(fut, timeout=0.05)
    assert time.monotonic() - t0 < 0.5
    assert fut.cancelled


async def test_await_ros_future_propagates_exception() -> None:
    fut = FakeRosFuture()
    fut.set_exception(RuntimeError("boom"))
    with pytest.raises(RuntimeError):
        await await_ros_future(fut, timeout=0.5)


# ------------------------------------------------------------------ execute_plan
async def test_execute_plan_rejects_locally_without_ros_round_trip(bridge: ExecutorBridge) -> None:
    out = await bridge.execute_plan('{"v":1,"steps":[{"id":"s1","skill":"spin","args":{}}]}')
    assert out.startswith("Rejected:") and "unknown skill" in out
    bridge._execute_plan.send_goal_async.assert_not_called()


async def test_execute_plan_rejects_when_server_absent(bridge: ExecutorBridge) -> None:
    bridge._execute_plan.server_is_ready.return_value = False
    out = await bridge.execute_plan('{"v":1,"steps":[{"id":"s1","skill":"stop","args":{}}]}')
    assert out == "Rejected: the plan executor is not running."


async def test_execute_plan_accepted_when_result_is_slow(bridge: ExecutorBridge) -> None:
    handle = SimpleNamespace(accepted=True, get_result_async=lambda: FakeRosFuture(), cancel_goal_async=lambda: FakeRosFuture())
    goal_fut = FakeRosFuture()
    goal_fut.set_result(handle)
    bridge._execute_plan.send_goal_async.return_value = goal_fut
    out = await bridge.execute_plan('{"v":1,"steps":[{"id":"s1","skill":"turn","args":{"degrees":90}}]}')
    assert out.startswith("Accepted: turn 90 degrees left")
    assert bridge._goal_handle is handle


async def test_execute_plan_fast_rejection_from_executor(bridge: ExecutorBridge) -> None:
    result_fut = FakeRosFuture()
    result_fut.set_result(SimpleNamespace(result=SimpleNamespace(outcome="REJECTED", reason="estop latched", summary="", failed_step=-1)))
    handle = SimpleNamespace(accepted=True, get_result_async=lambda: result_fut)
    goal_fut = FakeRosFuture()
    goal_fut.set_result(handle)
    bridge._execute_plan.send_goal_async.return_value = goal_fut
    out = await bridge.execute_plan('{"v":1,"steps":[{"id":"s1","skill":"turn","args":{"degrees":90}}]}')
    assert out == "Rejected: estop latched"
    assert bridge._goal_handle is None


# -------------------------------------------------------------------------- stop
async def test_stop_never_raises_when_services_absent(bridge: ExecutorBridge) -> None:
    out = await bridge.stop()
    assert out.startswith("Stopped.")
    assert "not responding" in out


async def test_stop_cancels_goal_and_survives_hanging_cancel(bridge: ExecutorBridge) -> None:
    hanging = FakeRosFuture()
    bridge._goal_handle = SimpleNamespace(cancel_goal_async=lambda: hanging)
    executor_bridge.SERVICE_TIMEOUT_S = 0.05
    try:
        t0 = time.monotonic()
        out = await bridge.stop()
        assert out.startswith("Stopped.")
        assert time.monotonic() - t0 < 2.0
        assert hanging.cancelled
    finally:
        executor_bridge.SERVICE_TIMEOUT_S = 1.0


# ------------------------------------------------------------------------ status
async def test_status_reports_not_responding_when_never_seen(bridge: ExecutorBridge) -> None:
    assert "not responding" in await bridge.status()


async def test_status_reports_not_responding_when_stale(bridge: ExecutorBridge) -> None:
    bridge._on_state(SimpleNamespace(status="idle", pose_summary="", last_result=""))
    bridge._state_mono = time.monotonic() - 5.0
    assert "not responding" in await bridge.status()


async def test_status_renders_fresh_running_state(bridge: ExecutorBridge) -> None:
    bridge._on_state(SimpleNamespace(status="running", step_index=1, step_count=3, current_skill="go_to_object",
                                     current_target="bed", progress=0.5, follow_lock=False, pose_summary="1.2 m from start"))
    out = await bridge.status()
    assert "Step 2 of 3" in out and "bed" in out and "50 percent" in out


# --------------------------------------------------------------------- ask_scene
async def test_ask_scene_renders_objects(bridge: ExecutorBridge) -> None:
    resp = FakeRosFuture()
    resp.set_result(SimpleNamespace(ok=True, text="A bedroom.", objects_json='[{"label":"bed","heading_deg":25,"distance_est_m":2.0}]'))
    bridge._look.service_is_ready.return_value = True
    bridge._look.call_async.return_value = resp
    out = await bridge.ask_scene("describe")
    assert "bed, slightly to the left, about 2.0 metres away" in out


# ------------------------------------------------------------------ animate/power
async def test_animate_rejects_unknown_name(bridge: ExecutorBridge) -> None:
    out = await bridge.animate("moonwalk")
    assert out.startswith("Unknown animation")


async def test_power_rejects_bad_mode(bridge: ExecutorBridge) -> None:
    assert "must be" in await bridge.power("sideways")
