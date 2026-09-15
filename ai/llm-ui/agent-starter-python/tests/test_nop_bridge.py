import asyncio

from nop_bridge import NopBridge


async def test_nop_runs_plan_and_emits_events(monkeypatch) -> None:
    monkeypatch.setattr("nop_bridge.STEP_SECONDS", 0.01)
    b = NopBridge()
    events: list[str] = []
    b.on_event(lambda ev: events.append(ev.type))
    out = await b.execute_plan('{"v":1,"steps":[{"id":"s1","skill":"turn","args":{"degrees":90}},{"id":"s2","skill":"say","args":{"text":"hi"}}]}')
    assert out.startswith("Accepted:")
    await asyncio.sleep(0.2)
    assert events[0] == "plan_accepted" and "say" in events and events[-1] == "plan_done"
    assert "not moving" in await b.status()


async def test_nop_stop_cancels() -> None:
    b = NopBridge()
    events: list[str] = []
    b.on_event(lambda ev: events.append(ev.type))
    await b.execute_plan('{"v":1,"steps":[{"id":"s1","skill":"wait","args":{"seconds":5}}]}')
    await asyncio.sleep(0.05)
    out = await b.stop()
    assert out.startswith("Stopped.")
    assert "plan_cancelled" in events
