"""NopBridge: the ExecutorBridge interface with no robot behind it.

Validates plans against the real schema, "runs" steps on a timer and emits the
same event/state shapes the ROS bridge would, so the prompt and PlanWatcher can be
iterated on a laptop:  python3 src/agent.py --tool-server nop console
"""

from __future__ import annotations

import asyncio
import json
import logging
import time
from dataclasses import dataclass, field
from typing import Any, Callable, Optional

from animations import ANIMATION_NAMES
from plan_schema import PlanError, describe_plan, describe_step, validate_plan
from speech_render import render_scene, render_state

logger = logging.getLogger("nop_bridge")

STEP_SECONDS = 0.8


@dataclass
class FakeState:
    status: str = "idle"
    plan_id: str = ""
    source: str = ""
    step_index: int = 0
    step_count: int = 0
    current_skill: str = ""
    current_target: str = ""
    progress: float = 0.0
    follow_lock: bool = False
    walking_active: bool = False
    estop_latched: bool = False
    active_cmd_source: str = "none"
    pose_summary: str = ""
    last_result: str = ""


@dataclass
class FakeEvent:
    type: str
    plan_id: str = ""
    source: str = "llm"
    step_id: str = ""
    skill: str = ""
    summary: str = ""
    detail_json: str = "{}"
    stamp: float = field(default_factory=time.time)


class NopBridge:
    def __init__(self) -> None:
        self._callbacks: list[Callable[[Any], None]] = []
        self.latest_state = FakeState()
        self._task: Optional[asyncio.Task[None]] = None
        self._plan_counter = 0
        self.speaking = False
        logger.info("NopBridge started (no robot)")

    def shutdown(self) -> None:
        if self._task:
            self._task.cancel()

    def on_event(self, callback: Callable[[Any], None]) -> None:
        self._callbacks.append(callback)

    def set_speaking(self, speaking: bool) -> None:
        self.speaking = speaking

    def _emit(self, ev: FakeEvent) -> None:
        logger.info("fake event %s %s %s", ev.type, ev.skill, ev.summary)
        for cb in list(self._callbacks):
            cb(ev)

    async def execute_plan(self, plan_json: str) -> str:
        try:
            plan = validate_plan(plan_json)
        except PlanError as e:
            return f"Rejected: {e}"
        if self._task and not self._task.done():
            if plan.get("mode", "replace") == "replace":
                self._task.cancel()
                await asyncio.sleep(0)
            else:
                return "Rejected: queue mode is not supported by the nop bridge; use replace."
        self._plan_counter += 1
        plan_id = f"nop-{self._plan_counter}"
        self._task = asyncio.create_task(self._run(plan, plan_id))
        return f"Accepted: {describe_plan(plan)}. I'll report when it's done."

    async def _run(self, plan: dict[str, Any], plan_id: str) -> None:
        steps = plan["steps"]
        st = self.latest_state
        st.status, st.plan_id, st.source = "running", plan_id, plan.get("source", "llm")
        st.step_count, st.walking_active = len(steps), True
        self._emit(FakeEvent("plan_accepted", plan_id, summary=f"Starting: {describe_plan(plan)}"))
        try:
            for i, step in enumerate(steps):
                st.step_index, st.current_skill = i, step["skill"]
                st.current_target = str(step.get("args", {}).get("label") or step.get("args", {}).get("who") or "")
                st.progress = 0.0
                self._emit(FakeEvent("step_started", plan_id, step_id=step["id"], skill=step["skill"], summary=describe_step(step)))
                if step["skill"] == "say":
                    self._emit(FakeEvent("say", plan_id, step_id=step["id"], skill="say", summary=step["args"]["text"]))
                elif step["skill"] == "stop":
                    pass
                elif step["skill"] == "wait":
                    await asyncio.sleep(float(step["args"]["seconds"]))
                else:
                    await asyncio.sleep(STEP_SECONDS)
                st.progress = 1.0
                self._emit(FakeEvent("step_done", plan_id, step_id=step["id"], skill=step["skill"], summary=f"done: {describe_step(step)}"))
            st.status, st.last_result = "idle", f"succeeded: {describe_plan(plan)}"
            st.pose_summary = "about 1 metre from where I started"
            self._emit(FakeEvent("plan_done", plan_id, summary=f"Done: {describe_plan(plan)}", detail_json=json.dumps({"outcome": "SUCCEEDED"})))
        except asyncio.CancelledError:
            st.status, st.last_result = "idle", "cancelled"
            self._emit(FakeEvent("plan_cancelled", plan_id, summary="Stopped.", detail_json=json.dumps({"cause": "tool"})))
            raise
        finally:
            st.current_skill, st.current_target, st.progress = "", "", 0.0

    async def stop(self) -> str:
        if self._task and not self._task.done():
            self._task.cancel()
            try:
                await self._task
            except asyncio.CancelledError:
                pass
        self.latest_state.status = "idle"
        return "Stopped. " + await self.status()

    async def status(self) -> str:
        return render_state(self.latest_state)

    async def ask_scene(self, prompt: str) -> str:
        await asyncio.sleep(0.5)
        objects = [
            {"label": "bed", "heading_deg": 25.0, "elevation_deg": -8.0, "close": False, "distance_est_m": 2.1},
            {"label": "door", "heading_deg": -60.0, "elevation_deg": 2.0, "close": False, "distance_est_m": 3.5},
        ]
        return render_scene(f"(fake scene for: {prompt}) A bedroom with a bed and a door.", json.dumps(objects))

    async def animate(self, animation_name: str) -> str:
        if animation_name not in ANIMATION_NAMES:
            return f"Unknown animation '{animation_name}'. Available: {', '.join(ANIMATION_NAMES)}"
        await asyncio.sleep(1.0)
        return f"Finished the {animation_name} animation."

    async def power(self, mode: str) -> str:
        mode = mode.strip().lower()
        if mode not in ("activate", "deactivate"):
            return "power mode must be 'activate' or 'deactivate'."
        self.latest_state.walking_active = mode == "activate"
        return "Motors on, walking controller active." if mode == "activate" else "Motors off."

    async def prepare(self) -> None:
        self.latest_state.walking_active = True

    async def follow_me(self) -> str:
        self.latest_state.follow_lock = True
        return "Following (fake)."

    async def stop_following(self) -> str:
        self.latest_state.follow_lock = False
        return "Stopped following (fake)."
