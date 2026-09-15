"""Calibrated dead-reckoning.

There is no odometry on Pupper v3. Position is the integral of the *commanded* body velocity passed through a
first-order lag (the policy takes ~0.3 s to reach a commanded speed), scaled by tape-measured calibration
factors k_vx / k_vy, and rotated by the IMU yaw. Uncertainty is tracked honestly and reported alongside.

Frames: `odom` (x, y in metres from where the node started, yaw from YawTracker). Named marks store a pose in
odom; `relative_to(mark)` expresses the current pose in that mark's body frame (dx forward, dy left).
"""
import math
from dataclasses import dataclass, field
from typing import Dict, Optional, Tuple

from .geometry import rotate, wrap_rad


@dataclass
class PoseIntegratorConfig:
    k_vx: float = 1.0        # actual / commanded forward distance (tape calibrated)
    k_vy: float = 1.0        # actual / commanded lateral distance
    lag_tau: float = 0.3     # seconds, first-order response of the policy to a velocity step
    sigma_frac: float = 0.35 # 1-sigma fraction of path length (before calibration)


@dataclass
class Mark:
    x: float
    y: float
    yaw: float
    path_at: float


class PoseIntegrator:
    def __init__(self, cfg: Optional[PoseIntegratorConfig] = None):
        self.cfg = cfg or PoseIntegratorConfig()
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.vx_est = 0.0
        self.vy_est = 0.0
        self.path_m = 0.0
        self.extra_sigma_m = 0.0
        self.marks: Dict[str, Mark] = {}

    # ------------------------------------------------------------------ integration
    def step(self, dt: float, cmd_vx: float, cmd_vy: float, yaw: float, integrating: bool) -> None:
        """Advance one control tick.

        integrating=False means our command is NOT what the controller is executing (teleop took over,
        controller inactive, ...): the velocity estimate decays to zero and no distance is accumulated.
        """
        if dt <= 0.0:
            self.yaw = yaw
            return
        tvx = cmd_vx if integrating else 0.0
        tvy = cmd_vy if integrating else 0.0
        a = min(1.0, dt / self.cfg.lag_tau) if self.cfg.lag_tau > 0 else 1.0
        self.vx_est += (tvx - self.vx_est) * a
        self.vy_est += (tvy - self.vy_est) * a
        self.yaw = yaw
        if integrating:
            bvx = self.cfg.k_vx * self.vx_est
            bvy = self.cfg.k_vy * self.vy_est
            wx, wy = rotate(bvx, bvy, yaw)
            self.x += wx * dt
            self.y += wy * dt
            self.path_m += math.hypot(bvx, bvy) * dt

    @property
    def speed_est(self) -> float:
        return math.hypot(self.cfg.k_vx * self.vx_est, self.cfg.k_vy * self.vy_est)

    def inflate(self, sigma_m: float) -> None:
        self.extra_sigma_m += max(0.0, sigma_m)

    # ------------------------------------------------------------------ marks
    def mark(self, name: str) -> Mark:
        m = Mark(self.x, self.y, self.yaw, self.path_m)
        self.marks[name] = m
        return m

    def has_mark(self, name: str) -> bool:
        return name in self.marks

    def relative_to(self, name: str) -> Tuple[float, float, float]:
        """Current pose expressed in the mark's frame: (dx forward, dy left, dyaw)."""
        m = self.marks[name]
        dx, dy = rotate(self.x - m.x, self.y - m.y, -m.yaw)
        return dx, dy, wrap_rad(self.yaw - m.yaw)

    def mark_to_odom(self, name: str, x: float, y: float, yaw: float) -> Tuple[float, float, float]:
        """A pose given in the mark's frame, expressed in odom."""
        m = self.marks[name]
        wx, wy = rotate(x, y, m.yaw)
        return m.x + wx, m.y + wy, wrap_rad(m.yaw + yaw)

    def sigma_xy(self, sigma_yaw_rad: float, since_mark: str = "start") -> float:
        path_since = self.path_m - (self.marks[since_mark].path_at if since_mark in self.marks else 0.0)
        return self.cfg.sigma_frac * path_since + path_since * abs(sigma_yaw_rad) + self.extra_sigma_m
