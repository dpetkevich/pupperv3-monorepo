import threading
import time

from pupper_planner.plan_schema import Plan, Step
from pupper_planner.runner import PlanRunner, StepResult


class FakeSkills:
    def __init__(self, results=None, delay=0.0):
        self.results = results or {}
        self.calls = []
        self.marked = 0
        self.stopped = 0
        self.delay = delay

    def mark_start(self):
        self.marked += 1

    def stop(self):
        self.stopped += 1

    def execute(self, step, deadline, cancel, feedback):
        self.calls.append(step.skill)
        t0 = time.monotonic()
        while time.monotonic() - t0 < self.delay:
            if cancel.is_set():
                return StepResult(ok=False, reason="CANCELLED")
            if time.monotonic() >= deadline:
                return StepResult(ok=False, reason="TIMEOUT", summary="took too long")
            time.sleep(0.01)
        r = self.results.get(step.id)
        if r is not None:
            return r
        return StepResult(ok=True, summary=f"did {step.skill}")


def plan(steps, **kw):
    return Plan(steps=[Step(id=f"s{i}", skill=s, args=a) for i, (s, a) in enumerate(steps)], **kw)


def test_success_path_summary_and_events():
    sk = FakeSkills({"s0": StepResult(True, summary="turned 180°"), "s1": StepResult(True, summary="reached the bed (~0.7 m)"),
                     "s2": StepResult(True, summary="back within ~0.4 m of the start")})
    events = []
    out = PlanRunner(sk, on_event=lambda t, s, summ, d: events.append(t)).run(
        plan([("turn", {"degrees": 180}), ("go_to_object", {"label": "bed"}), ("return_to_start", {})]), threading.Event())
    assert out.outcome == "SUCCEEDED"
    assert out.summary == "Done: turned 180°, reached the bed (~0.7 m), back within ~0.4 m of the start."
    assert sk.marked == 1
    assert events == ["step_started", "step_done"] * 3 + ["plan_done"]


def test_replan_does_not_remark_start():
    sk = FakeSkills()
    PlanRunner(sk).run(plan([("stop", {})], replan_of="abc"), threading.Event())
    assert sk.marked == 0


def test_failure_report_names_step_and_allows_replan():
    sk = FakeSkills({"s1": StepResult(False, reason="NOT_FOUND", summary="could not find the bed after looking all around")})
    out = PlanRunner(sk).run(plan([("turn", {"degrees": 180}), ("go_to_object", {"label": "bed"}), ("return_to_start", {})]), threading.Event())
    assert out.outcome == "FAILED" and out.failed_step == 1 and out.reason == "NOT_FOUND"
    assert out.summary.startswith("Could not finish step 2 (go_to_object): could not find the bed")
    assert "Completed: did turn" in out.summary
    assert out.replan_allowed is True
    assert sk.calls == ["turn", "go_to_object"]  # return_to_start not run
    assert sk.stopped == 1


def test_on_fail_return_to_start_runs_except_after_safety_abort():
    sk = FakeSkills({"s0": StepResult(False, reason="LOST", summary="lost the bed"), "_on_fail_return": StepResult(True, summary="back near the start")})
    out = PlanRunner(sk).run(plan([("go_to_object", {"label": "bed"})], on_fail="return_to_start_then_report"), threading.Event())
    assert out.outcome == "FAILED" and sk.calls == ["go_to_object", "return_to_start"]
    assert "Back near the start." in out.summary

    sk = FakeSkills({"s0": StepResult(False, reason="ESTOP")})
    out = PlanRunner(sk).run(plan([("move", {"meters": 1})], on_fail="return_to_start_then_report"), threading.Event())
    assert sk.calls == ["move"] and out.replan_allowed is False
    assert out.outcome == "CANCELLED" and out.reason == "ESTOP" and out.summary == "Stopped by the emergency stop."


def test_plan_cancelled_event_carries_cause():
    causes = {}

    def on_event(t, s, summ, d):
        if t == "plan_cancelled":
            causes[t] = d["cause"]

    for reason, cause in [("TELEOP_TAKEOVER", "teleop"), ("FALLEN", "estop")]:
        sk = FakeSkills({"s0": StepResult(False, reason=reason)})
        out = PlanRunner(sk, on_event=on_event).run(plan([("turn", {"degrees": 90})]), threading.Event())
        assert out.outcome == "CANCELLED" and causes["plan_cancelled"] == cause

    ev = threading.Event()
    ev.set()
    PlanRunner(FakeSkills(), on_event=on_event).run(plan([("stop", {})]), ev, cancel_cause=lambda: "reflex")
    assert causes["plan_cancelled"] == "reflex"
    PlanRunner(FakeSkills(), on_event=on_event).run(plan([("stop", {})]), ev)
    assert causes["plan_cancelled"] == "tool"


def test_cancel_mid_step_stops_and_reports_cancelled():
    sk = FakeSkills(delay=5.0)
    cancel = threading.Event()
    out = {}
    t = threading.Thread(target=lambda: out.update(r=PlanRunner(sk).run(plan([("move", {"meters": 2}), ("turn", {"degrees": 90})]), cancel)))
    t.start()
    time.sleep(0.1)
    cancel.set()
    t.join(2.0)
    assert out["r"].outcome == "CANCELLED" and out["r"].failed_step == 0
    assert sk.calls == ["move"] and sk.stopped == 1


def test_step_timeout():
    sk = FakeSkills(delay=5.0)
    p = plan([("wait", {"seconds": 0})])
    p.steps[0].timeout_s = 0.05
    out = PlanRunner(sk).run(p, threading.Event())
    assert out.outcome == "FAILED" and out.reason == "TIMEOUT"


def test_skill_exception_is_a_failure_not_a_crash():
    class Boom(FakeSkills):
        def execute(self, *a):
            raise RuntimeError("bad")

    out = PlanRunner(Boom()).run(plan([("stop", {})]), threading.Event())
    assert out.outcome == "FAILED" and out.reason.startswith("EXCEPTION:RuntimeError")
