"""PlanWatcher: turns executor events into speech and grounded chat context.

Events arrive on the ROS thread; everything that touches the LiveKit session hops
to the asyncio loop first. Only plan-level outcomes and explicit `say` steps are
spoken. Step events become throttled system notes so "what are you doing?" is
answered from executor truth, not the model's memory of what it asked for.
"""

from __future__ import annotations

import asyncio
import json
import logging
import time
from typing import Any, Optional

logger = logging.getLogger("plan_watcher")

SPEAK_TYPES = {"plan_done", "plan_failed", "plan_cancelled"}
SILENT_CANCEL_CAUSES = {"reflex", "teleop", "estop"}
REPORT_INSTRUCTIONS = (
    "[EXECUTOR REPORT] {summary} Relay this in one short sentence, in character. "
    "Do not add claims. Do not call tools."
)
SAY_INSTRUCTIONS = "[EXECUTOR SAY] Say this in character, adding nothing: {summary}"


class PlanWatcher:
    def __init__(self, session: Any, bridge: Any, step_note_interval_s: float = 3.0) -> None:
        self.session = session
        self.bridge = bridge
        self.step_note_interval_s = step_note_interval_s
        self._loop: Optional[asyncio.AbstractEventLoop] = None
        self._last_note_mono = 0.0

    def start(self) -> None:
        """Call from inside the running event loop."""
        self._loop = asyncio.get_running_loop()
        self.bridge.on_event(self._on_event_any_thread)

    def _on_event_any_thread(self, ev: Any) -> None:
        if self._loop is None:
            return
        self._loop.call_soon_threadsafe(self._handle, ev)

    # ---------------------------------------------------------------- loop thread
    def _handle(self, ev: Any) -> None:
        ev_type = str(getattr(ev, "type", ""))
        summary = str(getattr(ev, "summary", "") or "").strip()
        if ev_type == "say":
            if summary:
                self._speak(SAY_INSTRUCTIONS.format(summary=summary))
            return
        if ev_type in SPEAK_TYPES:
            if ev_type == "plan_cancelled" and self._cause(ev) in SILENT_CANCEL_CAUSES:
                logger.info("plan cancelled by %s; earcon already played, staying quiet", self._cause(ev))
                return
            self._speak(REPORT_INSTRUCTIONS.format(summary=summary or ev_type.replace("_", " ")))
            return
        if ev_type.startswith("step_") or ev_type == "plan_accepted":
            now = time.monotonic()
            if now - self._last_note_mono >= self.step_note_interval_s or ev_type == "plan_accepted":
                self._last_note_mono = now
                asyncio.ensure_future(self._note(f"[EXECUTOR NOTE] {ev_type}: {summary}"))

    @staticmethod
    def _cause(ev: Any) -> str:
        try:
            return str(json.loads(getattr(ev, "detail_json", "") or "{}").get("cause", ""))
        except (json.JSONDecodeError, AttributeError):
            return ""

    def _speak(self, instructions: str) -> None:
        try:
            self.session.generate_reply(instructions=instructions)
        except Exception:  # noqa: BLE001
            logger.exception("generate_reply failed")

    async def _note(self, text: str) -> None:
        try:
            agent = self.session.current_agent
            ctx = agent.chat_ctx.copy()
            ctx.add_message(role="system", content=text)
            await agent.update_chat_ctx(ctx)
        except Exception:  # noqa: BLE001
            logger.exception("update_chat_ctx failed")
