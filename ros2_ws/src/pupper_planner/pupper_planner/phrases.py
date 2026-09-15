"""Speakable phrases built from executor/motion state (rclpy-free)."""
from __future__ import annotations

import math


def pose_phrase(dx: float, dy: float, dyaw: float) -> str:
    """Reads after "I'm ...", e.g. "about 1.8 metres from where I started, facing the other way".

    ``dyaw`` is degrees, ROS sign (+left)."""
    d = math.hypot(dx, dy)
    yaw = ((dyaw + 180.0) % 360.0) - 180.0
    if d < 0.3:
        where = "right where I started"
    elif d < 1.0:
        where = "less than a metre from where I started"
    else:
        where = f"about {d:.1f} metres from where I started"
    if abs(yaw) < 20.0:
        facing = "facing the same way as when I started"
    elif abs(yaw) > 160.0:
        facing = "facing the other way"
    else:
        facing = f"turned about {abs(yaw):.0f} degrees to the {'left' if yaw > 0 else 'right'} of my starting heading"
    return f"{where}, {facing}"
