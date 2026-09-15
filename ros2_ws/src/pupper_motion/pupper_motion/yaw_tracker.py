"""Heading estimate from the IMU.

Fuses the BNO08x fused rotation vector (absolute yaw, may jump when the magnetometer re-converges)
with the gyro (smooth, drifts) using a complementary filter with a jump rejector.

The 260 Hz IMU broadcaster republishes 100 Hz sensor samples, so identical consecutive samples are
dropped before integration.

Yaw is reported relative to the first accepted sample (odom frame yaw 0 at start), radians, left positive.
"""
import math
from dataclasses import dataclass
from typing import Optional, Tuple

from .geometry import wrap_rad


@dataclass
class YawTrackerConfig:
    alpha: float = 0.02            # per-sample correction gain towards the quaternion yaw; 0 = gyro only
    jump_deg: float = 5.0          # residual larger than this in one step is treated as a magnetometer jump
    jump_sigma_deg: float = 15.0   # sigma inflation right after a jump
    jump_sigma_decay_s: float = 5.0
    stale_s: float = 0.2
    base_sigma_deg: float = 1.0
    gyro_drift_deg_per_s: float = 0.02  # sigma growth while running gyro-only (alpha == 0)
    gap_s: float = 0.06            # a gap longer than this means samples were lost: snap to the quaternion instead
    dedupe: bool = True


class YawTracker:
    def __init__(self, cfg: Optional[YawTrackerConfig] = None):
        self.cfg = cfg or YawTrackerConfig()
        self.reset()

    def reset(self) -> None:
        self.yaw: float = 0.0
        self.gyro_z: float = 0.0
        self.offset: float = 0.0
        self.last_stamp: Optional[float] = None
        self.jumps: int = 0
        self.gaps: int = 0
        self.samples: int = 0
        self.duplicates: int = 0
        self._last_raw: Optional[Tuple[float, float, float, float, float]] = None
        self._last_any_stamp: Optional[float] = None
        self._jump_sigma: float = 0.0
        self._gyro_only_sigma: float = 0.0
        self._last_jump_time: Optional[float] = None

    # ------------------------------------------------------------------ update
    def update(self, stamp_s: float, quat_xyzw: Tuple[float, float, float, float], gyro_z: float) -> bool:
        """Feed one IMU sample. Returns True if the sample was new (not a republished duplicate)."""
        raw = (*quat_xyzw, gyro_z)
        # gap detection uses the time since the last *received* sample (duplicates included): a still robot
        # produces identical samples that are dropped below, and that must not look like lost samples
        gap_dt = (stamp_s - self._last_any_stamp) if self._last_any_stamp is not None else 0.0
        self._last_any_stamp = stamp_s
        if self.cfg.dedupe and raw == self._last_raw:
            self.duplicates += 1
            return False
        self._last_raw = raw

        x, y, z, w = quat_xyzw
        yaw_q = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))

        if self.last_stamp is None:
            self.offset = yaw_q
            self.yaw = 0.0
            self.gyro_z = gyro_z
            self.last_stamp = stamp_s
            self.samples = 1
            return True

        dt = stamp_s - self.last_stamp
        if dt < 0.0:
            dt = 0.0
        if dt > 0.5:
            dt = 0.5  # a gap this long is a dropout; do not integrate across it blindly
        self.last_stamp = stamp_s
        self.samples += 1

        pred = self.yaw + 0.5 * (self.gyro_z + gyro_z) * dt
        self.gyro_z = gyro_z
        resid = wrap_rad((yaw_q - self.offset) - pred)

        if gap_dt > self.cfg.gap_s and self.cfg.alpha > 0.0:
            # We missed samples (callback stall): the gyro integral is unreliable across the gap, the fused
            # quaternion is not. Snap to it rather than treating the disagreement as a magnetometer jump.
            self.gaps += 1
            self.yaw = wrap_rad(yaw_q - self.offset)
            return True

        if self.cfg.alpha <= 0.0:
            self.yaw = wrap_rad(pred)
            self._gyro_only_sigma += self.cfg.gyro_drift_deg_per_s * dt
            return True

        if abs(resid) > math.radians(self.cfg.jump_deg):
            # Magnetometer / fusion jump: keep our smooth estimate, move the offset instead.
            self.offset = wrap_rad(self.offset + resid)
            self.jumps += 1
            self._jump_sigma = self.cfg.jump_sigma_deg
            self._last_jump_time = stamp_s
            self.yaw = wrap_rad(pred)
        else:
            self.yaw = wrap_rad(pred + self.cfg.alpha * resid)
        return True

    # ------------------------------------------------------------------ queries
    @property
    def yaw_deg(self) -> float:
        return math.degrees(self.yaw)

    def is_stale(self, now_s: float) -> bool:
        return self.last_stamp is None or (now_s - self.last_stamp) > self.cfg.stale_s

    def yaw_predicted(self, latency_s: float) -> float:
        """Yaw extrapolated forward by the measurement latency using the last gyro reading."""
        return wrap_rad(self.yaw + self.gyro_z * max(0.0, latency_s))

    def sigma_deg(self, now_s: Optional[float] = None) -> float:
        s = self.cfg.base_sigma_deg + self._gyro_only_sigma
        if self._jump_sigma > 0.0 and self._last_jump_time is not None and now_s is not None:
            age = max(0.0, now_s - self._last_jump_time)
            s += self._jump_sigma * max(0.0, 1.0 - age / self.cfg.jump_sigma_decay_s)
        elif self._jump_sigma > 0.0:
            s += self._jump_sigma
        return s
