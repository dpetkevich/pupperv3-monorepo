import asyncio
from types import SimpleNamespace
from unittest.mock import AsyncMock, MagicMock

from plan_watcher import PlanWatcher


class Bridge:
    def __init__(self) -> None:
        self.cb = None

    def on_event(self, cb) -> None:
        self.cb = cb


def _session() -> MagicMock:
    s = MagicMock()
    s.current_agent.chat_ctx.copy.return_value = MagicMock()
    s.current_agent.update_chat_ctx = AsyncMock()
    return s


async def test_plan_done_is_spoken_and_reflex_cancel_is_silent() -> None:
    session, bridge = _session(), Bridge()
    w = PlanWatcher(session, bridge)
    w.start()
    bridge.cb(SimpleNamespace(type="plan_done", summary="Done: reached the bed.", detail_json="{}"))
    bridge.cb(SimpleNamespace(type="plan_cancelled", summary="Stopped.", detail_json='{"cause":"reflex"}'))
    await asyncio.sleep(0.05)
    assert session.generate_reply.call_count == 1
    assert "[EXECUTOR REPORT] Done: reached the bed." in session.generate_reply.call_args.kwargs["instructions"]


async def test_step_events_become_throttled_notes() -> None:
    session, bridge = _session(), Bridge()
    w = PlanWatcher(session, bridge, step_note_interval_s=10.0)
    w.start()
    for i in range(5):
        bridge.cb(SimpleNamespace(type="step_started", summary=f"step {i}", detail_json="{}"))
    await asyncio.sleep(0.05)
    session.generate_reply.assert_not_called()
    assert session.current_agent.update_chat_ctx.await_count == 1
