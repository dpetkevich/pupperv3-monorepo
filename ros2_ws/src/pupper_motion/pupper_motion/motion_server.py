"""Motion server node: the ONLY publisher on /motion_cmd_vel.

Owns a 50 Hz control loop that steps one GoalRunner at a time, runs the safety monitor every tick, ramps and
clamps every command, publishes a 0.3 s zero-hold after any goal ends and then goes silent (the cmd_vel_mux
treats any message, even zeros, as "active" for 500 ms).

Actions: TurnDegrees, MoveMeters, GoToPose, ReturnToStart (pupper_interfaces).
Services: /motion_server/stop, /motion_server/prepare, /motion_server/clear_estop, /motion_server/mark_pose.
Topics: /motion_server/status (5 Hz), /motion_server/odom (20 Hz).
"""
import math
import threading
import time
from datetime import datetime, timezone
from typing import Callable, Dict, Optional

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from controller_manager_msgs.srv import ListControllers, SwitchController
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import Empty, Float32, String
from std_srvs.srv import Trigger

from pupper_interfaces.action import GoToPose, MoveMeters, ReturnToStart, TurnDegrees
from pupper_interfaces.msg import MotionStatus
from pupper_interfaces.srv import MarkPose

from .geometry import quat_tilt_deg, wrap_rad
from .limits import Cmd, Envelope, Ramp
from .pose_integrator import PoseIntegrator, PoseIntegratorConfig
from .runners import GoalRunner, GoToPoseRunner, MoveRunner, RobotState, RunnerConfig, TurnRunner
from .safety import SafetyMonitor
from .yaw_tracker import YawTracker, YawTrackerConfig

ALL_CONTROLLERS = [
    "neural_controller",
    "neural_controller_three_legged",
    "forward_kp_controller",
    "forward_kd_controller",
    "forward_position_controller",
]
STRICTNESS_BEST_EFFORT = 1


def _lat(logger, evt: str, key: str = "") -> None:
    logger.info(f"LAT evt={evt} t={time.monotonic_ns()} wall={datetime.now(timezone.utc).isoformat()} key={key}")


class MotionServer(Node):
    def __init__(self) -> None:
        super().__init__("motion_server")
        p = self._declare_params()
        self.cfg = RunnerConfig(
            turn_speed_dps=p["turn_speed_dps"], wz_floor=p["wz_floor"], turn_tol_deg=p["turn_tol_deg"],
            move_speed=p["move_speed"], vx_floor=p["vx_floor"], vy_floor=p["vy_floor"],
            accel_lin=p["accel_lin"], lag_tau=p["lag_tau"], sigma_frac=p["sigma_frac"],
            max_turn_deg=p["max_turn_deg"], max_move_m=p["max_move_m"],
        )
        self.yaw = YawTracker(YawTrackerConfig(alpha=p["yaw_alpha"], jump_deg=p["yaw_jump_deg"], stale_s=p["imu_stale_s"]))
        self.pose = PoseIntegrator(PoseIntegratorConfig(k_vx=p["k_vx"], k_vy=p["k_vy"], lag_tau=p["lag_tau"], sigma_frac=p["sigma_frac"]))
        self.safety = SafetyMonitor(our_source=p["cmd_topic"], higher_priority_sources=list(p["higher_priority_sources"]),
                                    walking_controller=p["walking_controller"], fall_tilt_deg=p["fall_tilt_deg"])
        self.env = Envelope(p["vx_max"], p["vy_max"], p["wz_max"])
        self.ramp = Ramp(p["accel_lin"], p["accel_ang"])
        self.auto_activate = bool(p["auto_activate"])
        self.activation_settle_s = float(p["activation_settle_s"])
        self.zero_hold_s = float(p["zero_hold_s"])
        self.imu_stale_s = float(p["imu_stale_s"])
        self.walking_controller = p["walking_controller"]

        self.lock = threading.Lock()
        self.runner: Optional[GoalRunner] = None
        self.runner_name = ""
        self.runner_done = threading.Event()
        self.goal_in_flight = False
        self._first_nonzero_logged = False
        self.zero_hold_until = 0.0
        self.settle_until = 0.0
        self.last_pub = Cmd()
        self.imu_latency_s = 0.0
        self._imu_rx_mono: Optional[float] = None
        self._last_tick = time.monotonic()
        self._takeovers_seen = 0

        # Threading model (measured, see README): a high-rate subscription on rclpy's MultiThreadedExecutor costs
        # ~100 % of a core at 100 Hz and starves everything else. So the IMU / e-stop / mux-source subscriptions
        # live on a small companion node spun by a SingleThreadedExecutor on its own thread (~25 % of a core at
        # the robot's 260 Hz), the control loop is a plain thread, and the MultiThreadedExecutor only serves
        # actions, services and low-rate timers. Shared state is guarded by self.lock.
        self.imu_node = Node("motion_server_imu")
        self.control_group = ReentrantCallbackGroup()
        self.action_group = ReentrantCallbackGroup()

        self.cmd_pub = self.create_publisher(Twist, p["cmd_topic"], 10)
        self.status_pub = self.create_publisher(MotionStatus, "/motion_server/status", 10)
        self.odom_pub = self.create_publisher(Odometry, "/motion_server/odom", 10)

        sensor_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=50)
        self.imu_node.create_subscription(Imu, p["imu_topic"], self._on_imu, sensor_qos)
        self.imu_node.create_subscription(Empty, "/emergency_stop", self._on_estop, 10)
        self.imu_node.create_subscription(String, "/cmd_vel_mux/active_source", self._on_active_source, 10)
        self.imu_node.create_subscription(Float32, p["imu_latency_topic"], self._on_imu_latency, 10)

        self.list_client = self.create_client(ListControllers, "/controller_manager/list_controllers", callback_group=self.action_group)
        self.switch_client = self.create_client(SwitchController, "/controller_manager/switch_controller", callback_group=self.action_group)

        self.create_service(Trigger, "/motion_server/stop", self._srv_stop, callback_group=self.action_group)
        self.create_service(Trigger, "/motion_server/prepare", self._srv_prepare, callback_group=self.action_group)
        self.create_service(Trigger, "/motion_server/clear_estop", self._srv_clear_estop, callback_group=self.action_group)
        self.create_service(MarkPose, "/motion_server/mark_pose", self._srv_mark_pose, callback_group=self.action_group)

        self._servers = [
            ActionServer(self, TurnDegrees, "/motion_server/turn_degrees", execute_callback=self._exec_turn,
                         goal_callback=self._goal_turn, cancel_callback=self._cancel, callback_group=self.action_group),
            ActionServer(self, MoveMeters, "/motion_server/move_meters", execute_callback=self._exec_move,
                         goal_callback=self._goal_move, cancel_callback=self._cancel, callback_group=self.action_group),
            ActionServer(self, GoToPose, "/motion_server/go_to_pose", execute_callback=self._exec_goto,
                         goal_callback=self._goal_goto, cancel_callback=self._cancel, callback_group=self.action_group),
            ActionServer(self, ReturnToStart, "/motion_server/return_to_start", execute_callback=self._exec_return,
                         goal_callback=self._goal_return, cancel_callback=self._cancel, callback_group=self.action_group),
        ]

        # The control loop runs on its own thread with sleep pacing: an rclpy executor timer can be starved by
        # other callbacks under load, and the velocity stream must never pause mid-goal (mux timeout 500 ms).
        self._control_period = 1.0 / p["control_hz"]
        self._control_thread = threading.Thread(target=self._control_loop, name="motion-control", daemon=True)
        self._control_thread.start()
        self.create_timer(0.2, self._publish_status, callback_group=self.control_group)
        self.create_timer(0.05, self._publish_odom, callback_group=self.control_group)
        self.create_timer(0.5, self._poll_controllers, callback_group=self.action_group)
        self.get_logger().info(f"motion_server up: publishing {p['cmd_topic']}, imu {p['imu_topic']}, auto_activate={self.auto_activate}")

    # ------------------------------------------------------------------ params
    def _declare_params(self) -> Dict:
        defaults = {
            "cmd_topic": "/motion_cmd_vel",
            "imu_topic": "/imu_sensor_broadcaster/imu",
            "imu_latency_topic": "/neural_controller/imu_latency_seconds",
            "higher_priority_sources": ["/teleop_cmd_vel", "/reflex_cmd_vel"],
            "walking_controller": "neural_controller",
            "control_hz": 50.0,
            "auto_activate": True,
            "activation_settle_s": 2.5,
            "zero_hold_s": 0.3,
            "imu_stale_s": 0.5,
            "fall_tilt_deg": 60.0,
            "yaw_alpha": 0.02,
            "yaw_jump_deg": 5.0,
            "k_vx": 1.0,
            "k_vy": 1.0,
            "lag_tau": 0.3,
            "sigma_frac": 0.35,
            "turn_speed_dps": 70.0,
            "wz_floor": 0.5,
            "turn_tol_deg": 3.0,
            "move_speed": 0.45,
            "vx_floor": 0.35,
            "vy_floor": 0.4,
            "vx_max": 0.75,
            "vy_max": 0.5,
            "wz_max": 2.0,
            "accel_lin": 1.0,
            "accel_ang": 4.0,
            "max_turn_deg": 360.0,
            "max_move_m": 5.0,
        }
        out = {}
        for k, v in defaults.items():
            out[k] = self.declare_parameter(k, v).value
        return out

    # ------------------------------------------------------------------ subscriptions
    def _on_imu(self, msg: Imu) -> None:
        q = msg.orientation
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        with self.lock:
            jumps_before = self.yaw.jumps
            self.yaw.update(stamp, (q.x, q.y, q.z, q.w), msg.angular_velocity.z)
            self.safety.on_imu(quat_tilt_deg(q.x, q.y, q.z, q.w), True)
            self._imu_rx_mono = time.monotonic()
            jumped = self.yaw.jumps != jumps_before
        if jumped:
            self.get_logger().warn(f"yaw jump #{self.yaw.jumps} absorbed (fused yaw disagreed with gyro by > {self.yaw.cfg.jump_deg} deg in one step)", throttle_duration_sec=1.0)

    def _on_estop(self, _msg: Empty) -> None:
        with self.lock:
            self.safety.on_estop()
            self._finish_runner_locked("ESTOP")
        self.get_logger().warn("emergency_stop received: latched until the walking controller is re-activated")

    def _on_active_source(self, msg: String) -> None:
        with self.lock:
            before = self.safety.takeover_events
            self.safety.on_active_source(msg.data)
            if self.safety.takeover_events != before:
                self.pose.inflate(1.0)
                self._finish_runner_locked("TELEOP_TAKEOVER")

    def _on_imu_latency(self, msg: Float32) -> None:
        self.imu_latency_s = float(msg.data)

    def _poll_controllers(self) -> None:
        if not self.list_client.service_is_ready():
            return
        fut = self.list_client.call_async(ListControllers.Request())

        def done(f):
            try:
                res = f.result()
            except Exception as e:  # noqa: BLE001
                self.get_logger().warn(f"list_controllers failed: {e}", throttle_duration_sec=10.0)
                return
            states = {c.name: c.state for c in res.controller}
            with self.lock:
                self.safety.on_controller_states(states)

        fut.add_done_callback(done)

    # ------------------------------------------------------------------ control loop
    def _state_locked(self, now: float) -> RobotState:
        imu_ok = self._imu_rx_mono is not None and (now - self._imu_rx_mono) < self.imu_stale_s
        if not imu_ok:
            self.safety.imu_ok = False
        return RobotState(
            t=now,
            yaw=self.yaw.yaw_predicted(self.imu_latency_s),
            gyro_z=self.yaw.gyro_z,
            x=self.pose.x,
            y=self.pose.y,
            speed_est=self.pose.speed_est,
            imu_ok=imu_ok,
        )

    def _control_loop(self) -> None:
        next_t = time.monotonic()
        while rclpy.ok():
            next_t += self._control_period
            try:
                self._tick()
            except Exception as e:  # noqa: BLE001
                self.get_logger().error(f"control loop error: {e}")
                with self.lock:
                    self._finish_runner_locked("ERROR")
            delay = next_t - time.monotonic()
            if delay > 0:
                time.sleep(delay)
            else:
                next_t = time.monotonic()  # fell behind: resynchronise instead of bursting

    def _tick(self) -> None:
        now = time.monotonic()
        dt = min(0.5, max(0.0, now - self._last_tick))
        self._last_tick = now
        if dt > 0.1:
            self.get_logger().warn(f"control loop stalled {dt*1000:.0f} ms", throttle_duration_sec=2.0)
        publish: Optional[Cmd] = None
        with self.lock:
            state = self._state_locked(now)
            cmd = Cmd()
            if self.runner is not None:
                reason = self.safety.check()
                if reason is not None:
                    self._finish_runner_locked(reason)
                else:
                    cmd = self.runner.step(state, dt)
                    if self.runner.done:
                        self._on_runner_done_locked(now)
                        cmd = Cmd()
            if self.runner is not None or now < self.settle_until:
                publish = self.env.clamp(self.ramp.apply(cmd, dt))
                if not publish.is_zero():
                    self.zero_hold_until = now + self.zero_hold_s
                    if not self._first_nonzero_logged:
                        self._first_nonzero_logged = True
                        _lat(self.get_logger(), "first_nonzero_twist", self.runner_name)
            elif now < self.zero_hold_until:
                publish = Cmd()
            integrating = self.safety.controller_active and self.safety.active_source == self.safety.our_source
            pub = publish or Cmd()
            self.pose.step(dt, pub.vx, pub.vy, state.yaw, integrating)
            self.last_pub = pub
        if publish is not None:
            self._publish_cmd(publish)

    def _publish_cmd(self, c: Cmd) -> None:
        msg = Twist()
        msg.linear.x = float(c.vx)
        msg.linear.y = float(c.vy)
        msg.angular.z = float(c.wz)
        self.cmd_pub.publish(msg)

    def _finish_runner_locked(self, reason: str) -> None:
        """Abort the active runner (if any) with `reason`; first zero Twist goes out on the calling thread."""
        if self.runner is None:
            return
        self.runner.finish(False, reason)
        self._on_runner_done_locked(time.monotonic())
        self._publish_cmd(Cmd())

    def _on_runner_done_locked(self, now: float) -> None:
        self.ramp.reset()
        self.zero_hold_until = now + self.zero_hold_s
        _lat(self.get_logger(), "zero_twist_on_exit", self.runner_name)
        self.runner = None
        self.runner_done.set()

    # ------------------------------------------------------------------ status / odom
    def _publish_status(self) -> None:
        m = MotionStatus()
        m.stamp = self.get_clock().now().to_msg()
        with self.lock:
            m.active_goal = self.runner_name if self.runner is not None else ""
            m.estop_latched = self.safety.estop_latched
            m.controller_active = self.safety.controller_active
            m.controller_name = self.walking_controller
            m.teleop_active = self.safety.teleop_active
            m.imu_ok = self.safety.imu_ok
            m.fallen = self.safety.fallen
            m.x, m.y, m.yaw_deg = float(self.pose.x), float(self.pose.y), float(math.degrees(self.pose.yaw))
            if self.pose.has_mark("start"):
                dx, dy, dyaw = self.pose.relative_to("start")
                m.start_dx, m.start_dy, m.start_dyaw = float(dx), float(dy), float(math.degrees(dyaw))
            sig_yaw = self.yaw.sigma_deg(self.yaw.last_stamp)
            m.sigma_xy_m = float(self.pose.sigma_xy(math.radians(sig_yaw)))
            m.sigma_yaw_deg = float(sig_yaw)
        self.status_pub.publish(m)

    def _publish_odom(self) -> None:
        o = Odometry()
        o.header.stamp = self.get_clock().now().to_msg()
        o.header.frame_id = "odom"
        o.child_frame_id = "base_link"
        with self.lock:
            o.pose.pose.position.x = float(self.pose.x)
            o.pose.pose.position.y = float(self.pose.y)
            o.pose.pose.orientation.z = math.sin(self.pose.yaw / 2.0)
            o.pose.pose.orientation.w = math.cos(self.pose.yaw / 2.0)
            sig_yaw = math.radians(self.yaw.sigma_deg(self.yaw.last_stamp))
            sxy = self.pose.sigma_xy(sig_yaw)
            o.twist.twist.linear.x = float(self.pose.cfg.k_vx * self.pose.vx_est)
            o.twist.twist.linear.y = float(self.pose.cfg.k_vy * self.pose.vy_est)
            o.twist.twist.angular.z = float(self.yaw.gyro_z)
        cov = [0.0] * 36
        cov[0] = cov[7] = max(1e-4, sxy * sxy)
        cov[14] = 1e6
        cov[21] = cov[28] = 1e6
        cov[35] = max(1e-4, sig_yaw * sig_yaw)
        o.pose.covariance = cov
        self.odom_pub.publish(o)

    # ------------------------------------------------------------------ services
    def _srv_stop(self, _req, res):
        with self.lock:
            had = self.runner is not None
            self._finish_runner_locked("CANCELLED")
            self.zero_hold_until = time.monotonic() + self.zero_hold_s
        _lat(self.get_logger(), "cancel_sent", "stop_service")
        self._publish_cmd(Cmd())
        res.success = True
        res.message = "stopped active goal" if had else "no active goal; zero published"
        return res

    def _srv_prepare(self, _req, res):
        ok, reason = self._ensure_active(block=False)
        res.success = ok
        res.message = reason or "walking controller active"
        return res

    def _srv_clear_estop(self, _req, res):
        with self.lock:
            self.safety.clear_latches()
        res.success = True
        res.message = "latches cleared"
        return res

    def _srv_mark_pose(self, req, res):
        with self.lock:
            m = self.pose.mark(req.name or "start")
        res.ok = True
        res.x, res.y, res.yaw_deg = float(m.x), float(m.y), float(math.degrees(m.yaw))
        return res

    # ------------------------------------------------------------------ activation
    def _wait_future(self, fut, timeout_s: float):
        ev = threading.Event()
        fut.add_done_callback(lambda _f: ev.set())
        if not ev.wait(timeout_s):
            return None
        try:
            return fut.result()
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(f"service call failed: {e}")
            return None

    def _ensure_active(self, block: bool = True):
        """Make sure the walking controller is active. Returns (ok, reason)."""
        with self.lock:
            if self.safety.controller_known and self.safety.controller_active and not (self.safety.estop_latched or self.safety.fallen):
                return True, ""
            if not self.safety.can_auto_activate():
                return False, "ESTOP"
        if not self.auto_activate:
            return False, "CONTROLLER_INACTIVE"
        if not self.switch_client.wait_for_service(timeout_sec=1.0):
            return False, "CONTROLLER_MANAGER_UNAVAILABLE"
        req = SwitchController.Request()
        req.activate_controllers = [self.walking_controller]
        req.deactivate_controllers = [c for c in ALL_CONTROLLERS if c != self.walking_controller]
        req.strictness = STRICTNESS_BEST_EFFORT
        self.get_logger().info(f"auto-activating {self.walking_controller}")
        res = self._wait_future(self.switch_client.call_async(req), 3.0)
        if res is None or not res.ok:
            return False, "ACTIVATION_FAILED"
        with self.lock:
            self.settle_until = time.monotonic() + self.activation_settle_s
            self.safety.on_controller_states({self.walking_controller: "active"})
        if block:
            time.sleep(self.activation_settle_s)
            self._poll_controllers()
            time.sleep(0.3)
            with self.lock:
                if not self.safety.controller_active:
                    return False, "CONTROLLER_INACTIVE"
        return True, ""

    # ------------------------------------------------------------------ goal gate
    def _accept_goal(self, name: str, invalid: Optional[str]) -> GoalResponse:
        with self.lock:
            if invalid:
                self.get_logger().warn(f"{name} rejected: INVALID ({invalid})")
                return GoalResponse.REJECT
            if self.runner is not None or self.goal_in_flight:
                self.get_logger().warn(f"{name} rejected: BUSY")
                return GoalResponse.REJECT
            if self.safety.estop_latched or self.safety.fallen:
                self.get_logger().warn(f"{name} rejected: ESTOP/FALLEN latched")
                return GoalResponse.REJECT
            if self._imu_rx_mono is None:
                self.get_logger().warn(f"{name} rejected: no IMU yet")
                return GoalResponse.REJECT
            self.goal_in_flight = True
            return GoalResponse.ACCEPT

    def _goal_turn(self, goal: TurnDegrees.Goal) -> GoalResponse:
        bad = None if abs(goal.degrees) <= self.cfg.max_turn_deg else f"|degrees| > {self.cfg.max_turn_deg}"
        return self._accept_goal("TurnDegrees", bad)

    def _goal_move(self, goal: MoveMeters.Goal) -> GoalResponse:
        bad = None if math.hypot(goal.dx, goal.dy) <= self.cfg.max_move_m else f"distance > {self.cfg.max_move_m} m"
        return self._accept_goal("MoveMeters", bad)

    def _goal_goto(self, goal: GoToPose.Goal) -> GoalResponse:
        bad = None
        if goal.frame not in ("start", "odom", ""):
            bad = "frame must be start|odom"
        elif goal.frame in ("start", "") and not self.pose.has_mark("start"):
            bad = "no start mark"
        elif math.hypot(goal.x, goal.y) > 3 * self.cfg.max_move_m:
            bad = "target too far"
        return self._accept_goal("GoToPose", bad)

    def _goal_return(self, goal: ReturnToStart.Goal) -> GoalResponse:
        bad = None if self.pose.has_mark("start") else "no start mark"
        return self._accept_goal("ReturnToStart", bad)

    def _cancel(self, goal_handle) -> CancelResponse:
        with self.lock:
            self._finish_runner_locked("CANCELLED")
        _lat(self.get_logger(), "cancel_sent", self.runner_name)
        return CancelResponse.ACCEPT

    # ------------------------------------------------------------------ execution
    def _run_goal(self, goal_handle, name: str, make_runner: Callable[[], GoalRunner], feedback_msg, fill_feedback, result_msg, fill_result):
        try:
            ok, reason = self._ensure_active(block=True)
            if not ok:
                goal_handle.abort()
                return fill_result(result_msg, False, reason, {})
            with self.lock:
                if self.runner is not None:
                    goal_handle.abort()
                    return fill_result(result_msg, False, "BUSY", {})
                if self.safety.check() is not None:
                    reason = self.safety.check() or "UNSAFE"
                    goal_handle.abort()
                    return fill_result(result_msg, False, reason, {})
                runner = make_runner()
                self.runner = runner
                self.runner_name = name
                self.runner_done.clear()
                self._first_nonzero_logged = False
                self.ramp.reset()
            _lat(self.get_logger(), "goal_started", name)
            while not self.runner_done.wait(0.1):
                if goal_handle.is_cancel_requested:
                    with self.lock:
                        self._finish_runner_locked("CANCELLED")
                    break
                with self.lock:
                    fb = runner.feedback(self._state_locked(time.monotonic()))
                goal_handle.publish_feedback(fill_feedback(feedback_msg, fb))
            self.runner_done.wait(1.0)
            r = runner.result
            assert r is not None
            # canceled() is only a legal transition when the client asked for it; a stop via the service or a
            # safety abort ends as ABORTED with reason CANCELLED / ESTOP / ...
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
            elif r.success:
                goal_handle.succeed()
            else:
                goal_handle.abort()
            self.get_logger().info(
                f"{name} finished: success={r.success} reason={r.reason or '-'} {r.data} "
                f"imu(samples={self.yaw.samples} dups={self.yaw.duplicates} gaps={self.yaw.gaps} jumps={self.yaw.jumps} yaw={self.yaw.yaw_deg:.1f})")
            return fill_result(result_msg, r.success, r.reason, r.data)
        finally:
            with self.lock:
                self.goal_in_flight = False

    def _exec_turn(self, gh):
        g = gh.request

        def ff(m, fb):
            m.remaining_degrees = float(fb.get("remaining_degrees", 0.0))
            m.turned_degrees = float(fb.get("turned_degrees", 0.0))
            return m

        def fr(m, ok, reason, d):
            m.success, m.reason = ok, reason
            m.turned_degrees = float(d.get("turned_degrees", 0.0))
            m.error_degrees = float(d.get("error_degrees", 0.0))
            return m

        return self._run_goal(gh, "TurnDegrees", lambda: TurnRunner(self.cfg, g.degrees, g.speed_dps),
                              TurnDegrees.Feedback(), ff, TurnDegrees.Result(), fr)

    def _exec_move(self, gh):
        g = gh.request

        def ff(m, fb):
            m.remaining_m = float(fb.get("remaining_m", 0.0))
            m.moved_m = float(fb.get("moved_m", 0.0))
            return m

        def fr(m, ok, reason, d):
            m.success, m.reason = ok, reason
            m.moved_m = float(d.get("moved_m", 0.0))
            m.sigma_m = float(d.get("sigma_m", 0.0))
            m.heading_drift_deg = float(d.get("heading_drift_deg", 0.0))
            return m

        return self._run_goal(gh, "MoveMeters", lambda: MoveRunner(self.cfg, g.dx, g.dy, g.speed_mps, g.hold_heading),
                              MoveMeters.Feedback(), ff, MoveMeters.Result(), fr)

    @staticmethod
    def _ff_goto(m, fb):
        m.phase = str(fb.get("phase", ""))
        m.remaining_m = float(fb.get("remaining_m", 0.0))
        m.remaining_deg = float(fb.get("remaining_deg", 0.0))
        return m

    @staticmethod
    def _fr_goto(m, ok, reason, d):
        m.success, m.reason = ok, reason
        m.distance_error_m = float(d.get("distance_error_m", 0.0))
        m.sigma_m = float(d.get("sigma_m", 0.0))
        m.yaw_error_deg = float(d.get("yaw_error_deg", 0.0))
        return m

    def _exec_goto(self, gh):
        g = gh.request

        def make():
            if g.frame in ("start", ""):
                tx, ty, tyaw = self.pose.mark_to_odom("start", g.x, g.y, math.radians(g.yaw_deg))
            else:
                tx, ty, tyaw = g.x, g.y, wrap_rad(math.radians(g.yaw_deg))
            r = GoToPoseRunner(self.cfg, tx, ty, tyaw, g.final_turn)
            r.feedback = _with_phase(r)  # type: ignore[method-assign]
            return r

        return self._run_goal(gh, "GoToPose", make, GoToPose.Feedback(), self._ff_goto, GoToPose.Result(), self._fr_goto)

    def _exec_return(self, gh):
        g = gh.request

        def make():
            tx, ty, tyaw = self.pose.mark_to_odom("start", 0.0, 0.0, 0.0)
            r = GoToPoseRunner(self.cfg, tx, ty, tyaw, g.restore_heading)
            r.feedback = _with_phase(r)  # type: ignore[method-assign]
            return r

        return self._run_goal(gh, "ReturnToStart", make, ReturnToStart.Feedback(), self._ff_goto, ReturnToStart.Result(), self._fr_goto)


def _with_phase(r: GoToPoseRunner):
    base = r.feedback

    def fb(s):
        d = base(s)
        d["phase"] = r.phase
        return d

    return fb


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MotionServer()
    imu_executor = SingleThreadedExecutor()
    imu_executor.add_node(node.imu_node)
    imu_thread = threading.Thread(target=imu_executor.spin, name="motion-imu-spin", daemon=True)
    imu_thread.start()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node._publish_cmd(Cmd())
        imu_executor.shutdown(timeout_sec=0.5)
        node.imu_node.destroy_node()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
