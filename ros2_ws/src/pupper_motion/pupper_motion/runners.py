"""Goal runners: pure control logic stepped at 50 Hz by the motion server.

A runner receives a RobotState snapshot each tick and returns the Cmd to publish. When it finishes it sets
`result` and returns a zero Cmd. Runners never touch ROS.

Sign conventions: yaw/wz left positive (ROS), dx forward, dy left, degrees at the API boundary, radians inside.
"""
import math
from dataclasses import dataclass, field
from typing import Dict, Optional, Tuple

from .geometry import clamp, rotate, wrap_rad
from .limits import Cmd


@dataclass
class RobotState:
    t: float                 # monotonic seconds
    yaw: float               # rad, odom
    gyro_z: float            # rad/s
    x: float                 # m, odom
    y: float
    speed_est: float         # |k * v_est| from the pose integrator
    imu_ok: bool = True


@dataclass
class RunnerResult:
    success: bool
    reason: str = ""         # "" on success, otherwise a short machine token
    data: Dict[str, float] = field(default_factory=dict)


@dataclass
class RunnerConfig:
    turn_speed_dps: float = 70.0
    turn_kp: float = 2.0            # rad/s per rad of error
    wz_floor: float = 0.5           # rad/s below which the policy does not turn
    turn_tol_deg: float = 3.0
    turn_stop_lag_s: float = 0.25    # policy lag + transport delay: stop commanding when the *predicted* stop is in tolerance
    turn_lag_learned: bool = False   # set once a real coast has been measured (then turn_stop_lag_s is trusted)
    settle_s: float = 0.3
    max_correction_passes: int = 4
    stall_gyro: float = 0.05        # rad/s
    stall_s: float = 1.5
    move_speed: float = 0.45
    vx_floor: float = 0.35          # m/s below which the policy does not translate
    vy_floor: float = 0.4
    vx_max: float = 0.75
    vy_max: float = 0.5
    wz_max: float = 2.0
    accel_lin: float = 1.0
    heading_hold_kp: float = 1.0
    heading_hold_wz_max: float = 0.4
    lag_tau: float = 0.3
    sigma_frac: float = 0.35
    timeout_margin_s: float = 5.0
    goto_rebearing_m: float = 0.3
    goto_max_legs: int = 2
    goto_min_move_m: float = 0.15
    max_turn_deg: float = 360.0
    max_move_m: float = 5.0


class GoalRunner:
    name = "runner"

    def __init__(self, cfg: RunnerConfig):
        self.cfg = cfg
        self.result: Optional[RunnerResult] = None
        self.t0: Optional[float] = None

    @property
    def done(self) -> bool:
        return self.result is not None

    def finish(self, success: bool, reason: str = "", **data: float) -> None:
        if self.result is None:
            self.result = RunnerResult(success, reason, {k: float(v) for k, v in data.items()})

    def step(self, s: RobotState, dt: float) -> Cmd:  # pragma: no cover - abstract
        raise NotImplementedError

    def feedback(self, s: RobotState) -> Dict[str, float]:  # pragma: no cover - abstract
        return {}


# ----------------------------------------------------------------------------- turn
class TurnRunner(GoalRunner):
    """Closed-loop yaw turn on the IMU. Cumulative turned angle is integrated from yaw deltas so |target| up
    to 360 works. Overshoot is corrected with up to `max_correction_passes` extra passes after a settle."""

    name = "TurnDegrees"

    def __init__(self, cfg: RunnerConfig, degrees: float, speed_dps: float = 0.0):
        super().__init__(cfg)
        self.target = math.radians(degrees)
        self.speed = math.radians(speed_dps if speed_dps > 0 else cfg.turn_speed_dps)
        self.speed = clamp(self.speed, cfg.wz_floor, cfg.wz_max)
        self.turned = 0.0
        self._last_yaw: Optional[float] = None
        self._phase = "MOVING"
        self._settle_until = 0.0
        self._passes = 0
        self._stall_since: Optional[float] = None
        self._stop_rem = 0.0
        self._stop_gyro = 0.0
        self.measured_lag_s: Optional[float] = None
        self._timeout = abs(self.target) / self.speed + cfg.timeout_margin_s + (cfg.settle_s + 1.0) * (cfg.max_correction_passes + 1)

    def _remaining(self) -> float:
        return self.target - self.turned

    def step(self, s: RobotState, dt: float) -> Cmd:
        if self.done:
            return Cmd()
        if self.t0 is None:
            self.t0 = s.t
            self._last_yaw = s.yaw
        self.turned += wrap_rad(s.yaw - self._last_yaw)
        self._last_yaw = s.yaw
        rem = self._remaining()
        tol = math.radians(self.cfg.turn_tol_deg)

        if s.t - self.t0 > self._timeout:
            self.finish(False, "TIMEOUT", turned_degrees=math.degrees(self.turned), error_degrees=math.degrees(rem))
            return Cmd()

        if self._phase == "MOVING":
            # predicted remaining after the robot coasts through its lag
            rem_pred = rem - s.gyro_z * self.cfg.turn_stop_lag_s
            if abs(rem) <= tol or (math.copysign(1.0, rem_pred) != math.copysign(1.0, rem)) or abs(rem_pred) <= tol:
                self._phase = "SETTLE"
                self._settle_until = s.t + self.cfg.settle_s
                self._stall_since = None
                self._stop_rem, self._stop_gyro = rem, s.gyro_z
                return Cmd()
            wz = math.copysign(clamp(self.cfg.turn_kp * abs(rem), self.cfg.wz_floor, self.speed), rem)
            # stall: commanding a turn but the gyro says we are not rotating
            if abs(s.gyro_z) < self.cfg.stall_gyro:
                if self._stall_since is None:
                    self._stall_since = s.t
                elif s.t - self._stall_since > self.cfg.stall_s:
                    self.finish(False, "STALLED", turned_degrees=math.degrees(self.turned), error_degrees=math.degrees(rem))
                    return Cmd()
            else:
                self._stall_since = None
            return Cmd(0.0, 0.0, wz)

        # SETTLE: wait until the robot has actually stopped rotating (or 1 s), then judge the result
        at_rest = abs(s.gyro_z) < self.cfg.stall_gyro
        if s.t >= self._settle_until and (at_rest or s.t >= self._settle_until + 1.0):
            # learn the real coast so the next pass / next goal predicts it better
            if abs(self._stop_gyro) > 0.1:
                lag = clamp((self._stop_rem - rem) / self._stop_gyro, 0.05, 0.6)
                # first measurement replaces the guess; later ones are blended (cfg is shared across goals)
                self.cfg.turn_stop_lag_s = lag if self.measured_lag_s is None and not self.cfg.turn_lag_learned else 0.5 * self.cfg.turn_stop_lag_s + 0.5 * lag
                self.cfg.turn_lag_learned = True
                self.measured_lag_s = lag
            if abs(rem) <= tol:
                self.finish(True, "", turned_degrees=math.degrees(self.turned), error_degrees=math.degrees(rem))
            elif self._passes < self.cfg.max_correction_passes:
                self._passes += 1
                self._phase = "MOVING"
            else:
                ok = abs(rem) <= 2.0 * tol
                self.finish(ok, "" if ok else "TOLERANCE", turned_degrees=math.degrees(self.turned), error_degrees=math.degrees(rem))
        return Cmd()

    def feedback(self, s: RobotState) -> Dict[str, float]:
        return {"remaining_degrees": math.degrees(self._remaining()), "turned_degrees": math.degrees(self.turned)}


# ----------------------------------------------------------------------------- move
class MoveRunner(GoalRunner):
    """Body-frame translation by dead reckoning. Direction is fixed at goal start; progress is the projection
    of the integrated pose onto that direction. Stops early by the lag-model stopping distance."""

    name = "MoveMeters"

    def __init__(self, cfg: RunnerConfig, dx: float, dy: float, speed_mps: float = 0.0, hold_heading: bool = True):
        super().__init__(cfg)
        self.dist = math.hypot(dx, dy)
        self.ux, self.uy = (dx / self.dist, dy / self.dist) if self.dist > 1e-6 else (1.0, 0.0)
        self.speed = speed_mps if speed_mps > 0 else cfg.move_speed
        self.speed = clamp(self.speed, cfg.vx_floor, cfg.vx_max)
        self.hold_heading = hold_heading
        self._phase = "MOVING"
        self.yaw0 = 0.0
        self.x0 = 0.0
        self.y0 = 0.0
        self._dir_odom: Tuple[float, float] = (1.0, 0.0)
        self.moved = 0.0
        self._timeout = self.dist / max(self.speed, 0.1) + cfg.timeout_margin_s + 2.0 * cfg.lag_tau

    def step(self, s: RobotState, dt: float) -> Cmd:
        if self.done:
            return Cmd()
        if self.t0 is None:
            self.t0 = s.t
            self.yaw0, self.x0, self.y0 = s.yaw, s.x, s.y
            self._dir_odom = rotate(self.ux, self.uy, s.yaw)
            if self.dist < 1e-3:
                self.finish(True, "", moved_m=0.0, sigma_m=0.0, heading_drift_deg=0.0)
                return Cmd()
        self.moved = (s.x - self.x0) * self._dir_odom[0] + (s.y - self.y0) * self._dir_odom[1]
        remaining = self.dist - self.moved

        if s.t - self.t0 > self._timeout:
            self.finish(False, "TIMEOUT", **self._data(s))
            return Cmd()

        wz = 0.0
        if self.hold_heading:
            wz = clamp(self.cfg.heading_hold_kp * wrap_rad(self.yaw0 - s.yaw), -self.cfg.heading_hold_wz_max, self.cfg.heading_hold_wz_max)

        if self._phase == "MOVING":
            stop_dist = s.speed_est * self.cfg.lag_tau
            if remaining <= stop_dist:
                self._phase = "STOPPING"
                return Cmd(0.0, 0.0, wz)
            v = clamp(math.sqrt(2.0 * self.cfg.accel_lin * max(remaining, 0.0)), self.cfg.vx_floor, self.speed)
            vx, vy = v * self.ux, v * self.uy
            if 1e-6 < abs(vx) < self.cfg.vx_floor:
                vx = math.copysign(self.cfg.vx_floor, vx)
            if 1e-6 < abs(vy) < self.cfg.vy_floor:
                vy = math.copysign(self.cfg.vy_floor, vy)
            vy = clamp(vy, -self.cfg.vy_max, self.cfg.vy_max)
            return Cmd(vx, vy, wz)

        # STOPPING: wait for the lag model to settle so `moved` is final
        if s.speed_est < 0.02:
            self.finish(True, "", **self._data(s))
        return Cmd()

    def _data(self, s: RobotState) -> Dict[str, float]:
        return {
            "moved_m": self.moved,
            "sigma_m": self.cfg.sigma_frac * abs(self.moved),
            "heading_drift_deg": math.degrees(wrap_rad(s.yaw - self.yaw0)),
        }

    def feedback(self, s: RobotState) -> Dict[str, float]:
        return {"remaining_m": self.dist - self.moved, "moved_m": self.moved}


# ----------------------------------------------------------------------------- go to pose
class GoToPoseRunner(GoalRunner):
    """Turn towards the target, move, re-bear once if still far, optional final turn. Target in odom."""

    name = "GoToPose"

    def __init__(self, cfg: RunnerConfig, tx: float, ty: float, tyaw: Optional[float], final_turn: bool):
        super().__init__(cfg)
        self.tx, self.ty, self.tyaw = tx, ty, tyaw
        self.final_turn = final_turn and tyaw is not None
        self.phase = "TURN"
        self.sub: Optional[GoalRunner] = None
        self.legs = 0
        self._timeout = (math.hypot(tx, ty) + 20.0) / cfg.vx_floor + 3 * (180.0 / cfg.turn_speed_dps) + cfg.timeout_margin_s

    def _dist(self, s: RobotState) -> float:
        return math.hypot(self.tx - s.x, self.ty - s.y)

    def _bearing_err(self, s: RobotState) -> float:
        return wrap_rad(math.atan2(self.ty - s.y, self.tx - s.x) - s.yaw)

    def _start_phase(self, s: RobotState) -> None:
        d = self._dist(s)
        if self.phase == "TURN":
            if d < self.cfg.goto_min_move_m:
                self.phase = "FINAL_TURN" if self.final_turn else "DONE"
                return self._start_phase(s)
            self.sub = TurnRunner(self.cfg, math.degrees(self._bearing_err(s)))
        elif self.phase == "MOVE":
            self.sub = MoveRunner(self.cfg, min(d, self.cfg.max_move_m), 0.0, 0.0, True)
        elif self.phase == "FINAL_TURN":
            self.sub = TurnRunner(self.cfg, math.degrees(wrap_rad(self.tyaw - s.yaw)))
        else:
            self.sub = None
            self.finish(True, "", **self._data(s))

    def step(self, s: RobotState, dt: float) -> Cmd:
        if self.done:
            return Cmd()
        if self.t0 is None:
            self.t0 = s.t
            self._start_phase(s)
            if self.done:
                return Cmd()
        if s.t - self.t0 > self._timeout:
            self.finish(False, "TIMEOUT", **self._data(s))
            return Cmd()
        assert self.sub is not None
        cmd = self.sub.step(s, dt)
        if not self.sub.done:
            return cmd
        r = self.sub.result
        assert r is not None
        if not r.success and r.reason not in ("TOLERANCE",):
            self.finish(False, r.reason, **self._data(s))
            return Cmd()
        if self.phase == "TURN":
            self.phase = "MOVE"
        elif self.phase == "MOVE":
            if self._dist(s) > self.cfg.goto_rebearing_m and self.legs < self.cfg.goto_max_legs:
                self.legs += 1
                self.phase = "TURN"
            else:
                self.phase = "FINAL_TURN" if self.final_turn else "DONE"
        elif self.phase == "FINAL_TURN":
            self.phase = "DONE"
        self._start_phase(s)
        return Cmd()

    def _data(self, s: RobotState) -> Dict[str, float]:
        yaw_err = math.degrees(wrap_rad(self.tyaw - s.yaw)) if self.tyaw is not None else 0.0
        return {"distance_error_m": self._dist(s), "sigma_m": self.cfg.sigma_frac * self._dist(s), "yaw_error_deg": yaw_err}

    def feedback(self, s: RobotState) -> Dict[str, float]:
        rem_deg = math.degrees(self._bearing_err(s)) if self.phase == "TURN" else (
            math.degrees(wrap_rad(self.tyaw - s.yaw)) if self.phase == "FINAL_TURN" and self.tyaw is not None else 0.0)
        return {"remaining_m": self._dist(s), "remaining_deg": rem_deg}
