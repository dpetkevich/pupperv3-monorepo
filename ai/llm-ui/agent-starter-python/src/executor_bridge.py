"""ExecutorBridge: the agent's only door into ROS.

One rclpy node spun by a background MultiThreadedExecutor. Every public method is
``async``, never blocks the asyncio loop on ROS, never raises into the LLM, and
returns a short speakable string. Motion itself lives in the plan_executor /
motion_server nodes; this class only submits plans, cancels them, and reads state.
"""

from __future__ import annotations

import asyncio
import functools
import json
import logging
import threading
import time
from datetime import datetime, timezone
from typing import Any, Awaitable, Callable, Optional

import rclpy
from controller_manager_msgs.srv import SwitchController
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger

from pupper_interfaces.action import ExecutePlan
from pupper_interfaces.msg import ExecutorEvent, ExecutorState, MotionStatus
from pupper_interfaces.srv import Look

from animations import ANIMATION_NAMES, get_animation_duration
from plan_schema import PlanError, describe_plan, describe_step, validate_plan
from ros_async import await_ros_future
from speech_render import render_scene, render_state

logger = logging.getLogger("executor_bridge")

WALKING_CONTROLLER = "neural_controller"
ALL_CONTROLLERS = [
    "neural_controller",
    "neural_controller_three_legged",
    "forward_kp_controller",
    "forward_position_controller",
    "forward_kd_controller",
]
STATE_STALE_S = 2.0
ACCEPT_TIMEOUT_S = 1.0
FAST_RESULT_S = 0.3
SERVICE_TIMEOUT_S = 1.0
LOOK_TIMEOUT_S = 8.0
POWER_TIMEOUT_S = 2.0

EventCallback = Callable[[Any], None]


def lat(evt: str, key: str = "") -> None:
    """Latency breadcrumb; joined with the /cmd_vel bag by tools/latency_report.py."""
    logger.info("LAT evt=%s t=%d wall=%s key=%s", evt, time.monotonic_ns(), datetime.now(timezone.utc).isoformat(), key)


def traced(name: str) -> Callable[[Callable[..., Awaitable[str]]], Callable[..., Awaitable[str]]]:
    def deco(fn: Callable[..., Awaitable[str]]) -> Callable[..., Awaitable[str]]:
        @functools.wraps(fn)
        async def wrapper(self: Any, *args: Any, **kwargs: Any) -> str:
            lat("tool_call_start", name)
            try:
                return await fn(self, *args, **kwargs)
            except Exception as e:  # noqa: BLE001 - tools must never raise into the LLM
                logger.exception("tool %s failed", name)
                return f"{name} failed: {type(e).__name__}: {e}"
            finally:
                lat("tool_call_return", name)

        return wrapper

    return deco


class ExecutorBridge:
    def __init__(self, node_name: str = "pupster_agent") -> None:
        if not rclpy.ok():
            rclpy.init()
        self.node = Node(node_name)
        self._callbacks: list[EventCallback] = []
        self.latest_state: Optional[Any] = None
        self._state_mono: float = 0.0
        self.latest_motion: Optional[Any] = None
        self._goal_handle: Optional[Any] = None
        self._goal_lock = threading.Lock()

        latched = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
        )
        events_qos = QoSProfile(
            depth=50,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.node.create_subscription(ExecutorState, "/plan_executor/state", self._on_state, latched)
        self.node.create_subscription(ExecutorEvent, "/plan_executor/events", self._on_event, events_qos)
        self.node.create_subscription(MotionStatus, "/motion_server/status", self._on_motion, 5)

        self._execute_plan = ActionClient(self.node, ExecutePlan, "/plan_executor/execute_plan")
        self._executor_stop = self.node.create_client(Trigger, "/plan_executor/stop")
        self._executor_prepare = self.node.create_client(Trigger, "/plan_executor/prepare")
        self._look = self.node.create_client(Look, "/plan_executor/look")
        self._motion_stop = self.node.create_client(Trigger, "/motion_server/stop")
        self._switch = self.node.create_client(SwitchController, "/controller_manager/switch_controller")
        self._follow_on = self.node.create_client(Trigger, "activate_person_following")
        self._follow_off = self.node.create_client(Trigger, "deactivate_person_following")

        self._speaking_pub = self.node.create_publisher(Bool, "/agent/speaking", 1)
        self._animation_pub = self.node.create_publisher(String, "/animation_controller_py/animation_select", 10)

        self._executor = MultiThreadedExecutor(num_threads=2)
        self._executor.add_node(self.node)
        self._thread = threading.Thread(target=self._spin, name="ros-executor", daemon=True)
        self._thread.start()
        logger.info("ExecutorBridge started")

    # ------------------------------------------------------------------ plumbing
    def _spin(self) -> None:
        try:
            self._executor.spin()
        except Exception:  # noqa: BLE001
            logger.exception("ROS executor thread died")

    def shutdown(self) -> None:
        try:
            self._executor.shutdown(timeout_sec=1.0)
            self.node.destroy_node()
        except Exception:  # noqa: BLE001
            logger.exception("shutdown")

    def on_event(self, callback: EventCallback) -> None:
        """Register a callback for every ExecutorEvent. Called on the ROS thread."""
        self._callbacks.append(callback)

    def _on_state(self, msg: Any) -> None:
        self.latest_state = msg
        self._state_mono = time.monotonic()

    def _on_motion(self, msg: Any) -> None:
        self.latest_motion = msg

    def _on_event(self, msg: Any) -> None:
        for cb in list(self._callbacks):
            try:
                cb(msg)
            except Exception:  # noqa: BLE001
                logger.exception("event callback failed")

    def set_speaking(self, speaking: bool) -> None:
        msg = Bool()
        msg.data = bool(speaking)
        self._speaking_pub.publish(msg)

    async def _call(self, client: Any, request: Any, timeout: float) -> Optional[Any]:
        """Call a service; None if unavailable, timed out, or errored. Never raises."""
        try:
            if not client.service_is_ready():
                logger.warning("service %s not ready", client.srv_name)
                return None
            return await await_ros_future(client.call_async(request), timeout)
        except asyncio.TimeoutError:
            logger.warning("service %s timed out after %.1fs", client.srv_name, timeout)
            return None
        except Exception:  # noqa: BLE001
            logger.exception("service %s failed", client.srv_name)
            return None

    def _state_fresh(self) -> bool:
        return self.latest_state is not None and (time.monotonic() - self._state_mono) <= STATE_STALE_S

    # ------------------------------------------------------------------ tools
    @traced("execute_plan")
    async def execute_plan(self, plan_json: str) -> str:
        try:
            plan = validate_plan(plan_json)
        except PlanError as e:
            logger.warning("plan rejected locally: %s", e)
            return f"Rejected: {e}"

        if not self._execute_plan.server_is_ready():
            return "Rejected: the plan executor is not running."

        goal = ExecutePlan.Goal()
        goal.plan_json = json.dumps(plan, separators=(",", ":"))
        try:
            handle = await await_ros_future(self._execute_plan.send_goal_async(goal), ACCEPT_TIMEOUT_S)
        except asyncio.TimeoutError:
            return "Rejected: the plan executor did not answer in time."
        if not handle.accepted:
            return "Rejected: the executor refused the plan (it may be estopped or busy)."

        with self._goal_lock:
            self._goal_handle = handle
        lat("plan_sent", str(plan.get("goal", ""))[:40])

        result_future = handle.get_result_async()
        result_future.add_done_callback(lambda f: self._on_result(f, handle))
        try:
            wrapped = await await_ros_future(result_future, FAST_RESULT_S)
        except asyncio.TimeoutError:
            return f"Accepted: {describe_plan(plan)}. I'll report when it's done."
        result = wrapped.result
        outcome = str(result.outcome)
        if outcome == "REJECTED":
            return f"Rejected: {result.reason}"
        return f"Done ({outcome.lower()}): {result.summary or describe_plan(plan)}"

    def _on_result(self, future: Any, handle: Any) -> None:
        with self._goal_lock:
            if self._goal_handle is handle:
                self._goal_handle = None
        try:
            r = future.result().result
            logger.info("plan result: %s failed_step=%s reason=%s summary=%s", r.outcome, r.failed_step, r.reason, r.summary)
        except Exception:  # noqa: BLE001
            logger.exception("plan result unavailable")

    @traced("stop")
    async def stop(self) -> str:
        lat("cancel_sent", "stop")
        with self._goal_lock:
            handle = self._goal_handle
        if handle is not None:
            try:
                await await_ros_future(handle.cancel_goal_async(), SERVICE_TIMEOUT_S)
            except Exception:  # noqa: BLE001
                logger.warning("cancel_goal_async did not complete")
        await self._call(self._executor_stop, Trigger.Request(), SERVICE_TIMEOUT_S)
        await self._call(self._motion_stop, Trigger.Request(), SERVICE_TIMEOUT_S)
        await self._call(self._follow_off, Trigger.Request(), SERVICE_TIMEOUT_S)
        await asyncio.sleep(0.2)  # let one state tick land so status() reflects the stop
        return "Stopped. " + await self.status()

    @traced("status")
    async def status(self) -> str:
        if not self._state_fresh():
            return "Executor not responding, so I can't tell you what I'm doing."
        return render_state(self.latest_state)

    @traced("ask_scene")
    async def ask_scene(self, prompt: str) -> str:
        req = Look.Request()
        req.prompt = prompt
        req.max_age_s = 0.0
        resp = await self._call(self._look, req, LOOK_TIMEOUT_S)
        if resp is None:
            return "I couldn't get a look at the scene right now."
        if not resp.ok:
            return f"Look failed: {resp.text or 'unknown reason'}"
        return render_scene(resp.text, resp.objects_json)

    @traced("animate")
    async def animate(self, animation_name: str) -> str:
        if animation_name not in ANIMATION_NAMES:
            return f"Unknown animation '{animation_name}'. Available: {', '.join(ANIMATION_NAMES)}"
        csv_name = ANIMATION_NAMES[animation_name]["csv_name"]
        msg = String()
        msg.data = csv_name
        self._animation_pub.publish(msg)
        duration = get_animation_duration(csv_name)
        await asyncio.sleep(duration)
        return f"Finished the {animation_name} animation."

    @traced("power")
    async def power(self, mode: str) -> str:
        mode = mode.strip().lower()
        req = SwitchController.Request()
        req.strictness = SwitchController.Request.BEST_EFFORT
        if mode == "activate":
            req.activate_controllers = [WALKING_CONTROLLER]
            req.deactivate_controllers = [c for c in ALL_CONTROLLERS if c != WALKING_CONTROLLER]
        elif mode == "deactivate":
            await self.stop()
            req.activate_controllers = []
            req.deactivate_controllers = list(ALL_CONTROLLERS)
        else:
            return "power mode must be 'activate' or 'deactivate'."
        resp = await self._call(self._switch, req, POWER_TIMEOUT_S)
        if resp is None or not resp.ok:
            return f"Could not {mode} the motors."
        return "Motors on, walking controller active." if mode == "activate" else "Motors off."

    async def prepare(self) -> None:
        """Fire-and-forget: pre-activate the walking controller while the user is still talking."""
        try:
            if self._executor_prepare.service_is_ready():
                self._executor_prepare.call_async(Trigger.Request())
        except Exception:  # noqa: BLE001
            logger.exception("prepare")

    @traced("follow_me")
    async def follow_me(self) -> str:
        # The legacy follower needs the walking controller up; /plan_executor/prepare auto-activates it
        # (gated on the e-stop latch) and reports whether it was already active.
        was_active = bool(getattr(getattr(self, "latest_motion", None), "controller_active", False))
        prep = await self._call(self._executor_prepare, Trigger.Request(), POWER_TIMEOUT_S)
        if prep is not None and not prep.success:
            return f"Cannot follow: {prep.message}"
        if not was_active:
            await asyncio.sleep(2.5)  # controller init + fade-in before the follower starts commanding
        resp = await self._call(self._follow_on, Trigger.Request(), POWER_TIMEOUT_S)
        if resp is None:
            return "Person following is not available right now."
        if not resp.success:
            return resp.message or "Could not start following."
        return "Following mode is on. It only tracks a person the camera can currently see; if nobody is in view I stand still."

    @traced("stop_following")
    async def stop_following(self) -> str:
        resp = await self._call(self._follow_off, Trigger.Request(), POWER_TIMEOUT_S)
        if resp is None:
            return "Person following is not available right now."
        return resp.message or ("Stopped following." if resp.success else "Could not stop following.")

    def describe_first_step(self, plan: dict[str, Any]) -> str:
        return describe_step(plan["steps"][0])
