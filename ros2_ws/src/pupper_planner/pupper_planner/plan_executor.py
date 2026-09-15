"""plan_executor node: ExecutePlan action server + state/events publishers + stop/prepare/look services.

Threading: a MultiThreadedExecutor spins the node; ExecutePlan execute callbacks run on the reentrant group and
serialize through ``_run_lock`` so exactly one plan runs at a time. ``mode: replace`` cancels the running plan
before taking the lock; ``mode: queue`` waits behind it. Goal acceptance itself is immediate.
"""
from __future__ import annotations

import json
import math
import threading
import time
from dataclasses import dataclass, field
from typing import Any, Dict, Optional

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import Trigger

from pupper_interfaces.action import ExecutePlan
from pupper_interfaces.msg import ExecutorEvent, ExecutorState, MotionStatus
from pupper_interfaces.srv import Look

from .lat import lat_line
from .phrases import pose_phrase
from .plan_schema import Limits, Plan, PlanRejected, Step, check_replan, load_schema, parse_plan_json
from .runner import PlanOutcome, PlanRunner
from .service_bridge import call_service
from .skills.ros_skills import RosSkills

DEDUPE_WINDOW_S = 5.0


@dataclass
class RunCtx:
    plan: Plan
    goal_handle: Any
    cancel: threading.Event = field(default_factory=threading.Event)
    done: threading.Event = field(default_factory=threading.Event)
    started_mono: float = field(default_factory=time.monotonic)
    step_index: int = 0
    skill: str = ""
    phase: str = ""
    progress: float = 0.0
    outcome: Optional[PlanOutcome] = None
    cancel_cause: str = "tool"     # who set ``cancel``: tool | reflex


class PlanExecutorNode(Node):
    def __init__(self) -> None:
        super().__init__("plan_executor")
        p = self.declare_parameter
        p("google_api_key_file", "")
        p("equirect_width", 800)
        p("equirect_height", 720)
        p("h_fov_deg", 180.0)
        p("v_fov_deg", 180.0)
        p("camera_height_m", 0.2)
        p("look_settle_s", 0.4)
        p("look_max_age_s", 0.6)
        p("animate_wait_s", 8.0)
        p("max_total_m", 15.0)
        p("max_total_s", 300.0)
        p("schema_path", "")
        p("camera_topic", "/camera/image_raw/compressed")
        p("animation_names", [""])

        self.limits = Limits(max_total_m=self.get_parameter("max_total_m").value, max_total_s=self.get_parameter("max_total_s").value)
        names = [n for n in self.get_parameter("animation_names").value if n]
        self.limits.animation_names = names or None
        schema_path = self.get_parameter("schema_path").value or None
        self.schema = load_schema(schema_path)

        self.cg_control = MutuallyExclusiveCallbackGroup()
        self.cg_work = ReentrantCallbackGroup()

        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL, history=HistoryPolicy.KEEP_LAST)
        events_qos = QoSProfile(depth=50, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL, history=HistoryPolicy.KEEP_LAST)
        self.state_pub = self.create_publisher(ExecutorState, "/plan_executor/state", latched)
        self.events_pub = self.create_publisher(ExecutorEvent, "/plan_executor/events", events_qos)

        self._lock = threading.Lock()          # protects _current/_last_* fields
        self._run_lock = threading.Lock()      # one plan at a time
        self._current: Optional[RunCtx] = None
        self._last_result = ""
        self._last_reflex: Optional[RunCtx] = None
        self._replan_parents: Dict[str, Optional[str]] = {}
        self._follow_lock = False
        self._motion: Optional[MotionStatus] = None
        self._motion_mono = 0.0
        self._active_source = "none"

        # vision (optional: no key -> look steps fail with NO_API_KEY, everything else works)
        self.look_skill = self._build_look()

        self.skills = RosSkills(
            self,
            self.look_skill,
            emit_event=self._emit,
            animate_wait_s=self.get_parameter("animate_wait_s").value,
            set_follow_lock=self._set_follow_lock,
            callback_group=self.cg_work,
        )
        self.runner = PlanRunner(self.skills, on_event=self._emit, on_progress=self._on_progress)

        self.motion_stop_client = self.create_client(Trigger, "/motion_server/stop", callback_group=self.cg_work)
        self.motion_prepare_client = self.create_client(Trigger, "/motion_server/prepare", callback_group=self.cg_work)

        self.create_subscription(MotionStatus, "/motion_server/status", self._on_motion, 10, callback_group=self.cg_control)
        self.create_subscription(String, "/cmd_vel_mux/active_source", self._on_active_source, 10, callback_group=self.cg_control)

        self.create_service(Trigger, "/plan_executor/stop", self._srv_stop, callback_group=self.cg_work)
        self.create_service(Trigger, "/plan_executor/prepare", self._srv_prepare, callback_group=self.cg_work)
        self.create_service(Look, "/plan_executor/look", self._srv_look, callback_group=self.cg_work)

        self.action_server = ActionServer(
            self,
            ExecutePlan,
            "/plan_executor/execute_plan",
            execute_callback=self._execute,
            goal_callback=self._goal_cb,
            cancel_callback=self._cancel_cb,
            callback_group=self.cg_work,
        )
        self.create_timer(0.2, self._publish_state, callback_group=self.cg_control)
        self.get_logger().info("plan_executor ready (vision=%s)" % ("on" if self.look_skill and self.look_skill.gemini.available else "off"))

    # ------------------------------------------------------------------ setup
    def _build_look(self):
        try:
            from pupper_vision.equirect import EquirectProjector
            from pupper_vision.gemini_look import GeminiLook, load_api_key
            from pupper_vision.image_buffer import LatestImage

            from .skills.look import LookParams, LookSkill

            key_file = self.get_parameter("google_api_key_file").value or None
            key = load_api_key(key_file)
            projector = EquirectProjector(
                width=int(self.get_parameter("equirect_width").value),
                height=int(self.get_parameter("equirect_height").value),
                h_fov_deg=float(self.get_parameter("h_fov_deg").value),
                v_fov_deg=float(self.get_parameter("v_fov_deg").value),
            )
            latest = LatestImage(self, self.get_parameter("camera_topic").value)
            params = LookParams(
                settle_s=float(self.get_parameter("look_settle_s").value),
                max_age_s=float(self.get_parameter("look_max_age_s").value),
                camera_height_m=float(self.get_parameter("camera_height_m").value),
            )
            if not key:
                self.get_logger().warning("no GOOGLE_API_KEY found; look/go_to_object will fail with NO_API_KEY")
            return LookSkill(latest, projector, GeminiLook(key), params)
        except Exception as e:
            self.get_logger().error(f"vision disabled: {e}")
            return None

    # ------------------------------------------------------------------ subscriptions
    def _on_motion(self, msg: MotionStatus) -> None:
        self._motion = msg
        self._motion_mono = time.monotonic()

    def _on_active_source(self, msg: String) -> None:
        self._active_source = msg.data

    def _set_follow_lock(self, val: bool) -> None:
        self._follow_lock = val

    # ------------------------------------------------------------------ events / state
    def _emit(self, etype: str, step: Optional[Step], summary: str, detail: Dict[str, Any]) -> None:
        with self._lock:
            ctx = self._current
        ev = ExecutorEvent()
        ev.stamp = self.get_clock().now().to_msg()
        ev.plan_id = ctx.plan.plan_id if ctx else ""
        ev.source = ctx.plan.source if ctx else ""
        ev.type = etype
        ev.step_id = step.id if step else ""
        ev.skill = step.skill if step else ""
        ev.summary = summary
        try:
            ev.detail_json = json.dumps(detail, default=str)
        except Exception:
            ev.detail_json = "{}"
        self.events_pub.publish(ev)
        self.get_logger().info(f"event {etype} {ev.step_id} {summary}")

    def _on_progress(self, i: int, n: int, skill: str, phase: str, progress: float) -> None:
        with self._lock:
            ctx = self._current
        if ctx is None:
            return
        ctx.step_index, ctx.skill, ctx.phase, ctx.progress = i, skill, phase, progress
        if ctx.goal_handle is not None:
            fb = ExecutePlan.Feedback()
            fb.step_index, fb.step_count, fb.skill, fb.phase = i, n, skill, phase
            fb.message = f"step {i + 1}/{n} {skill} {phase}"
            fb.state_json = json.dumps(self._state_dict())
            try:
                ctx.goal_handle.publish_feedback(fb)
            except Exception:
                pass

    def _state_dict(self) -> Dict[str, Any]:
        m = self._motion
        with self._lock:
            ctx = self._current
        estop = bool(m.estop_latched) if m else False
        walking = bool(m.controller_active) if m else False
        if ctx is None:
            status = "idle"
        elif ctx.cancel.is_set():
            status = "cancelling"
        else:
            status = "running"
        if estop:
            status = "estop"
        elif ctx is not None and m is not None and not walking and ctx.skill in ("turn", "move", "go_to_pose", "return_to_start", "go_to_object"):
            status = "controller_inactive" if (time.monotonic() - ctx.started_mono) > 4.0 else status
        if m is not None and (time.monotonic() - self._motion_mono) < 2.0:
            pose = pose_phrase(m.start_dx, m.start_dy, m.start_dyaw)
        else:
            pose = "not sure where I am relative to my start"
        target = ""
        if ctx is not None and 0 <= ctx.step_index < len(ctx.plan.steps):
            a = ctx.plan.steps[ctx.step_index].args
            target = str(a.get("label") or a.get("who") or a.get("name") or "")
        return {
            "status": status,
            "plan_id": ctx.plan.plan_id if ctx else "",
            "source": ctx.plan.source if ctx else "",
            "goal": ctx.plan.goal if ctx else "",
            "step_index": ctx.step_index if ctx else 0,
            "step_count": len(ctx.plan.steps) if ctx else 0,
            "current_skill": ctx.skill if ctx else "",
            "current_target": target,
            "progress": ctx.progress if ctx else 0.0,
            "follow_lock": self._follow_lock,
            "walking_active": walking,
            "estop_latched": estop,
            "active_cmd_source": self._active_source,
            "pose_summary": pose,
            "last_result": self._last_result,
        }

    def _publish_state(self) -> None:
        d = self._state_dict()
        msg = ExecutorState()
        msg.stamp = self.get_clock().now().to_msg()
        for k in ("status", "plan_id", "source", "current_skill", "current_target", "active_cmd_source", "pose_summary", "last_result"):
            setattr(msg, k, str(d[k]))
        msg.step_index = int(d["step_index"])
        msg.step_count = int(d["step_count"])
        msg.progress = float(d["progress"])
        msg.follow_lock = bool(d["follow_lock"])
        msg.walking_active = bool(d["walking_active"])
        msg.estop_latched = bool(d["estop_latched"])
        self.state_pub.publish(msg)

    def _cancel_current(self, ctx: RunCtx, cause: Optional[str] = None, note: str = "") -> None:
        """Set the cancel event with a cause. Without an explicit cause, a stop while the reflex tier owns
        /cmd_vel is attributed to the reflex, otherwise to a tool."""
        if not ctx.cancel.is_set():
            ctx.cancel_cause = cause or ("reflex" if self._active_source == "/reflex_cmd_vel" else "tool")
            ctx.cancel.set()
            self.get_logger().info(lat_line("cancel_sent", ctx.plan.plan_id) + f" cause={ctx.cancel_cause} {note}")

    # ------------------------------------------------------------------ services
    def _srv_stop(self, req, resp):
        with self._lock:
            ctx = self._current
        if ctx is not None:
            self._cancel_current(ctx, note="via /plan_executor/stop")
        r = call_service(self.motion_stop_client, Trigger.Request(), timeout_s=1.0)
        resp.success = True
        resp.message = ("cancelled plan " + ctx.plan.plan_id if ctx else "no plan running") + ("; motion stopped" if r is not None else "; motion server did not answer")
        return resp

    def _srv_prepare(self, req, resp):
        if self.motion_prepare_client.service_is_ready():
            r = call_service(self.motion_prepare_client, Trigger.Request(), timeout_s=1.0)
            resp.success = r is not None and bool(r.success)
            resp.message = r.message if r is not None else "motion server did not answer"
        else:
            resp.success = True
            resp.message = "no prepare service; nothing to do"
        return resp

    def _srv_look(self, req, resp):
        if self.look_skill is None:
            resp.ok, resp.text, resp.objects_json = False, "camera not available", "[]"
            return resp
        out = self.look_skill.look(req.prompt or "Describe what you see in one or two sentences.", max_age_s=req.max_age_s, settle_s=0.0)
        resp.ok = out.ok
        resp.text = out.text if out.ok else f"look failed: {out.reason}"
        resp.objects_json = out.objects_json()
        return resp

    # ------------------------------------------------------------------ action server
    def _goal_cb(self, goal_request) -> GoalResponse:
        # Accept immediately; validation errors are delivered as a REJECTED result (a goal rejection carries no reason).
        if len(goal_request.plan_json) > 65536:
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _cancel_cb(self, goal_handle) -> CancelResponse:
        with self._lock:
            ctx = self._current
        if ctx is not None and ctx.goal_handle is goal_handle:
            self._cancel_current(ctx, "tool", "via action cancel")
        else:
            # queued (not yet running) goal: mark it so _execute skips it
            setattr(goal_handle, "_pupper_cancel_early", True)
        return CancelResponse.ACCEPT

    def _result(self, outcome: str, failed_step: int = -1, reason: str = "", summary: str = "", replan: bool = False) -> ExecutePlan.Result:
        r = ExecutePlan.Result()
        r.outcome, r.failed_step, r.reason, r.summary, r.replan_allowed = outcome, int(failed_step), reason, summary, bool(replan)
        r.state_json = json.dumps(self._state_dict())
        return r

    def _execute(self, goal_handle) -> ExecutePlan.Result:
        t_accept = time.monotonic()
        try:
            plan = parse_plan_json(goal_handle.request.plan_json, schema=self.schema, limits=self.limits)
            check_replan(plan, self._replan_parents)
        except PlanRejected as e:
            self.get_logger().warning(f"plan REJECTED: {e.reason}")
            goal_handle.abort()
            return self._result("REJECTED", e.step_index, e.reason, f"I could not run that: {e.reason}")
        self.get_logger().info(lat_line("plan_accepted", plan.plan_id) + f" goal={plan.goal!r} steps={len(plan.steps)} mode={plan.mode} source={plan.source}")

        # dedupe: an LLM plan equivalent to a reflex plan started < 5 s ago
        with self._lock:
            reflex = self._last_reflex
        if plan.source == "llm" and reflex is not None and (t_accept - reflex.started_mono) < DEDUPE_WINDOW_S and reflex.plan.signature() == plan.signature():
            self.get_logger().info(f"plan {plan.plan_id} duplicates reflex plan {reflex.plan.plan_id}; joining it")
            reflex.done.wait()
            oc = reflex.outcome
            goal_handle.succeed()
            return self._result("SUCCEEDED", -1, "already_running", oc.summary if oc else "already done")

        if plan.mode == "replace":
            with self._lock:
                cur = self._current
            if cur is not None and not cur.done.is_set():
                self._cancel_current(cur, "reflex" if plan.source == "reflex" else "tool", f"replaced_by={plan.plan_id}")

        ctx = RunCtx(plan=plan, goal_handle=goal_handle)
        with self._run_lock:
            if getattr(goal_handle, "_pupper_cancel_early", False):
                goal_handle.canceled()
                return self._result("CANCELLED", -1, "CANCELLED", "Cancelled before it started.")
            ctx.started_mono = time.monotonic()
            with self._lock:
                self._current = ctx
                if plan.source == "reflex":
                    self._last_reflex = ctx
                self._replan_parents[plan.plan_id] = plan.replan_of
            self.skills.begin_plan(plan.plan_id)
            self._emit("plan_accepted", None, plan.goal or "starting", {"steps": len(plan.steps)})
            try:
                outcome = self.runner.run(plan, ctx.cancel, cancel_cause=lambda: ctx.cancel_cause)
            except Exception as e:
                self.get_logger().error(f"runner crashed: {e}")
                try:
                    self.skills.stop()
                except Exception:
                    pass
                outcome = PlanOutcome("FAILED", -1, f"EXCEPTION:{type(e).__name__}", f"Something went wrong: {e}", False)
            ctx.outcome = outcome
            with self._lock:
                self._last_result = f"{outcome.outcome}: {outcome.summary}"
                if self._current is ctx:
                    self._current = None
            ctx.done.set()

        if outcome.outcome == "SUCCEEDED":
            goal_handle.succeed()
        elif outcome.outcome == "CANCELLED" and goal_handle.is_cancel_requested:
            goal_handle.canceled()
        else:
            goal_handle.abort()
        return self._result(outcome.outcome, outcome.failed_step, outcome.reason, outcome.summary, outcome.replan_allowed)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PlanExecutorNode()
    executor = MultiThreadedExecutor(num_threads=6)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
