"""Angle and quaternion helpers. All yaw values follow ROS/REP-103: radians, counter-clockwise (left) positive."""
import math
from typing import Tuple


def wrap_rad(a: float) -> float:
    """Wrap an angle to [-pi, pi)."""
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def wrap_deg(a: float) -> float:
    """Wrap an angle to [-180, 180)."""
    return (a + 180.0) % 360.0 - 180.0


def quat_to_yaw(x: float, y: float, z: float, w: float) -> float:
    """Yaw (rotation about world +Z) of a unit quaternion, radians, left positive."""
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def quat_tilt_deg(x: float, y: float, z: float, w: float) -> float:
    """Angle in degrees between the body +Z axis and world +Z (0 = level, 90 = on its side)."""
    body_z_world_z = 1.0 - 2.0 * (x * x + y * y)
    return math.degrees(math.acos(max(-1.0, min(1.0, body_z_world_z))))


def rotate(vx: float, vy: float, yaw: float) -> Tuple[float, float]:
    """Rotate a 2-D vector by yaw (body -> world when yaw is the body heading)."""
    c, s = math.cos(yaw), math.sin(yaw)
    return c * vx - s * vy, s * vx + c * vy


def clamp(v: float, lo: float, hi: float) -> float:
    return lo if v < lo else hi if v > hi else v
