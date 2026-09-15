"""Velocity command container, trained-envelope clamp and acceleration ramp."""
from dataclasses import dataclass

from .geometry import clamp


@dataclass
class Cmd:
    vx: float = 0.0  # m/s, +forward
    vy: float = 0.0  # m/s, +left
    wz: float = 0.0  # rad/s, +left (counter-clockwise)

    def is_zero(self, eps: float = 1e-6) -> bool:
        return abs(self.vx) < eps and abs(self.vy) < eps and abs(self.wz) < eps


ZERO = Cmd()


@dataclass
class Envelope:
    """Trained policy envelope (ai/rl/conf/training/default.yaml)."""
    vx_max: float = 0.75
    vy_max: float = 0.5
    wz_max: float = 2.0

    def clamp(self, c: Cmd) -> Cmd:
        return Cmd(clamp(c.vx, -self.vx_max, self.vx_max), clamp(c.vy, -self.vy_max, self.vy_max), clamp(c.wz, -self.wz_max, self.wz_max))


class Ramp:
    """Limits how fast the magnitude of each axis may *increase*. Decreases (including to zero) are instant,
    so stop/cancel/e-stop paths are never delayed by the ramp."""

    def __init__(self, accel_lin: float = 1.0, accel_ang: float = 4.0):
        self.accel_lin = accel_lin
        self.accel_ang = accel_ang
        self.last = Cmd()

    def reset(self) -> None:
        self.last = Cmd()

    @staticmethod
    def _axis(target: float, last: float, max_step: float) -> float:
        if abs(target) <= abs(last) or (target * last) < 0.0:
            # slowing down, stopping or reversing: apply immediately (reversal starts from 0 next tick)
            return target if (target * last) >= 0.0 else 0.0
        step = target - last
        if step > max_step:
            return last + max_step
        if step < -max_step:
            return last - max_step
        return target

    def apply(self, target: Cmd, dt: float) -> Cmd:
        out = Cmd(
            self._axis(target.vx, self.last.vx, self.accel_lin * dt),
            self._axis(target.vy, self.last.vy, self.accel_lin * dt),
            self._axis(target.wz, self.last.wz, self.accel_ang * dt),
        )
        self.last = out
        return out
