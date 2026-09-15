"""PlanRunner: walks a validated Plan calling a ``Skills`` implementation. rclpy-free and unit-testable.

Outcomes: SUCCEEDED | FAILED | CANCELLED. Summaries are built from step results, never from intent.
"""
from __future__ import annotations

import threading
import time
from dataclasses import dataclass, field
from typing import Any, Callable, Dict, List, Optional, Protocol

from .plan_schema import SAFETY_ABORT_REASONS, Plan, Step

# child-action reasons that end a plan as CANCELLED (not FAILED) with a safety cause
SAFETY_CANCEL_CAUSE = {"ESTOP": "estop", "FALLEN": "estop", "TELEOP": "teleop", "TELEOP_TAKEOVER": "teleop"}


@dataclass
class StepResult:
    ok: bool
    reason: str = ""          # "" on success; CANCELLED | TIMEOUT | child reason on failure
    summary: str = ""         # speakable fragment, e.g. "turned 180°"
    detail: Dict[str, Any] = field(default_factory=dict)

    @property
    def cancelled(self) -> bool:
        return self.reason == "CANCELLED"


@dataclass
class PlanOutcome:
    outcome: str              # SUCCEEDED | FAILED | CANCELLED
    failed_step: int = -1
    reason: str = ""
    summary: str = ""
    replan_allowed: bool = False
    step_results: List[StepResult] = field(default_factory=list)


class Skills(Protocol):
    def mark_start(self) -> None: ...
    def stop(self) -> None: ...
    def execute(self, step: Step, deadline_mono: float, cancel: threading.Event, feedback: Callable[[str, float], None]) -> StepResult: ...


EventCb = Callable[[str, Step | None, str, Dict[str, Any]], None]   # (type, step, summary, detail)
ProgressCb = Callable[[int, int, str, str, float], None]            # (step_index, step_count, skill, phase, progress)


class PlanRunner:
    def __init__(self, skills: Skills, on_event: Optional[EventCb] = None, on_progress: Optional[ProgressCb] = None) -> None:
        self.skills = skills
        self.on_event = on_event or (lambda *a: None)
        self.on_progress = on_progress or (lambda *a: None)

    def run(self, plan: Plan, cancel: threading.Event, cancel_cause: Optional[Callable[[], str]] = None) -> PlanOutcome:
        """``cancel_cause`` answers "who set the cancel event": "tool" (agent/stop service) or "reflex"."""
        self._cancel_cause = cancel_cause or (lambda: "tool")
        results: List[StepResult] = []
        n = len(plan.steps)
        if plan.replan_of is None:
            self.skills.mark_start()

        for i, step in enumerate(plan.steps):
            if cancel.is_set():
                return self._finish(plan, "CANCELLED", i, "CANCELLED", results)
            self.on_event("step_started", step, self._describe(step), {"index": i})
            self.on_progress(i, n, step.skill, "running", 0.0)
            timeout = step.timeout()
            deadline = time.monotonic() + timeout if timeout > 0 else float("inf")

            def fb(phase: str, progress: float, _i=i, _s=step) -> None:
                self.on_progress(_i, n, _s.skill, phase, progress)

            try:
                res = self.skills.execute(step, deadline, cancel, fb)
            except Exception as e:  # a skill bug must not take the executor down
                res = StepResult(ok=False, reason=f"EXCEPTION:{type(e).__name__}:{e}")
            results.append(res)
            if res.ok:
                self.on_event("step_done", step, res.summary, res.detail)
                continue
            if res.cancelled or cancel.is_set():
                self.skills.stop()
                return self._finish(plan, "CANCELLED", i, "CANCELLED", results)
            if res.reason in SAFETY_CANCEL_CAUSE:
                # e-stop / fall / joystick takeover: the plan is cancelled by a higher authority, not failed
                self.on_event("step_failed", step, res.summary or res.reason, res.detail)
                self.skills.stop()
                return self._finish(plan, "CANCELLED", i, res.reason, results)
            self.on_event("step_failed", step, res.summary or res.reason, res.detail)
            self.skills.stop()
            if plan.on_fail == "return_to_start_then_report" and res.reason not in SAFETY_ABORT_REASONS and not cancel.is_set():
                back = Step(id="_on_fail_return", skill="return_to_start", args={"restore_heading": False})
                self.on_event("step_started", back, "returning to the start", {"index": i})
                r2 = self.skills.execute(back, time.monotonic() + 90.0, cancel, lambda *a: None)
                results.append(r2)
                self.on_event("step_done" if r2.ok else "step_failed", back, r2.summary or r2.reason, r2.detail)
            return self._finish(plan, "FAILED", i, res.reason, results)

        return self._finish(plan, "SUCCEEDED", -1, "", results)

    # ------------------------------------------------------------------ summaries
    @staticmethod
    def _describe(step: Step) -> str:
        a = step.args
        s = step.skill
        if s == "turn":
            d = float(a.get("degrees", 0))
            return f"turning {abs(d):.0f} degrees {'left' if d > 0 else 'right'}"
        if s == "move":
            m = float(a.get("meters", 0))
            return f"walking {abs(m):.1f} m {'forward' if m >= 0 else 'backward'}"
        if s == "go_to_object":
            return f"heading to the {a.get('label')}"
        if s == "find_object":
            return f"looking for the {a.get('label')}"
        if s == "return_to_start":
            return "returning to where I started"
        if s == "go_to_pose":
            return "walking to a saved spot"
        if s == "find_person":
            return f"looking for {a.get('who')}"
        if s == "follow_person":
            return f"following {a.get('who')}"
        if s == "look":
            return "taking a look"
        if s == "wait":
            return f"waiting {float(a.get('seconds', 0)):.0f} s"
        if s == "animate":
            return f"doing the {a.get('name')} animation"
        return s

    def _finish(self, plan: Plan, outcome: str, failed_step: int, reason: str, results: List[StepResult]) -> PlanOutcome:
        done_bits = [r.summary for r in results if r.ok and r.summary]
        if outcome == "SUCCEEDED":
            summary = "Done: " + ", ".join(done_bits) + "." if done_bits else "Done."
            replan = False
        elif outcome == "CANCELLED":
            summary = "Stopped" + (" after " + ", ".join(done_bits) if done_bits else "") + "."
            replan = False
        else:
            step = plan.steps[failed_step] if 0 <= failed_step < len(plan.steps) else None
            failed = results[failed_step] if 0 <= failed_step < len(results) else None
            what = failed.summary if failed and failed.summary else f"{reason}"
            head = f"Could not finish step {failed_step + 1}" + (f" ({step.skill})" if step else "") + f": {what}."
            tail = " Completed: " + ", ".join(done_bits) + "." if done_bits else ""
            back = ""
            if len(results) > failed_step + 1 and results[-1].summary and plan.on_fail == "return_to_start_then_report":
                back = " " + results[-1].summary.capitalize() + "."
            summary = head + tail + back
            replan = reason not in SAFETY_ABORT_REASONS and plan.replan_of is None
        out = PlanOutcome(outcome=outcome, failed_step=failed_step, reason=reason, summary=summary, replan_allowed=replan, step_results=results)
        detail: Dict[str, Any] = {"reason": reason, "failed_step": failed_step}
        if outcome == "CANCELLED":
            detail["cause"] = SAFETY_CANCEL_CAUSE.get(reason) or self._cancel_cause()
            if reason in SAFETY_CANCEL_CAUSE:
                what = {"estop": "the emergency stop", "teleop": "the joystick"}[detail["cause"]]
                summary = f"Stopped by {what}" + (" after " + ", ".join(done_bits) if done_bits else "") + "."
                out.summary = summary
        self.on_event({"SUCCEEDED": "plan_done", "FAILED": "plan_failed", "CANCELLED": "plan_cancelled"}[outcome], None, summary, detail)
        return out
