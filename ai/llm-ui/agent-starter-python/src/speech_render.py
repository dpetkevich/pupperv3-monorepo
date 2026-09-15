"""Turn executor state and scene objects into short speakable English."""

from __future__ import annotations

import json
from typing import Any


def heading_words(heading_deg: float) -> str:
    """ROS sign convention: positive = left."""
    h = float(heading_deg)
    side = "left" if h > 0 else "right"
    a = abs(h)
    if a < 10:
        return "straight ahead"
    if a < 35:
        return f"slightly to the {side}"
    if a < 75:
        return f"to the {side}"
    if a < 120:
        return f"far to the {side}"
    return f"behind me to the {side}"


def distance_words(obj: dict[str, Any]) -> str:
    if obj.get("close"):
        return "close by"
    d = obj.get("distance_est_m")
    if d is None:
        return ""
    d = float(d)
    if d < 1.0:
        return "less than a metre away"
    if d < 3.0:
        return f"about {d:.1f} metres away"
    return f"about {d:.0f} metres away"


def render_scene(text: str, objects_json: str) -> str:
    try:
        objects = json.loads(objects_json) if objects_json else []
    except json.JSONDecodeError:
        objects = []
    parts = [text.strip()] if text else []
    if objects:
        items = []
        for o in objects[:8]:
            words = [str(o.get("label", "something")), heading_words(o.get("heading_deg", 0.0))]
            dist = distance_words(o)
            if dist:
                words.append(dist)
            items.append(", ".join(words))
        parts.append("Objects with headings: " + "; ".join(items) + ".")
    return " ".join(parts) if parts else "I could not see anything useful."


def render_state(state: Any) -> str:
    """Render an ExecutorState-like object (attribute access) into 1-2 sentences."""
    status = str(getattr(state, "status", "unknown"))
    pose = str(getattr(state, "pose_summary", "") or "")
    if status == "idle":
        last = str(getattr(state, "last_result", "") or "")
        out = "I'm not moving."
        if last:
            out += f" Last plan: {last}"
        if pose:
            out += f" I'm {pose}."
        return out
    if status == "running":
        i = int(getattr(state, "step_index", 0)) + 1
        n = int(getattr(state, "step_count", 0))
        skill = str(getattr(state, "current_skill", "") or "")
        target = str(getattr(state, "current_target", "") or "")
        prog = float(getattr(state, "progress", 0.0) or 0.0)
        what = f"{skill} {target}".strip()
        out = f"Step {i} of {n}: {what}, {prog * 100:.0f} percent done."
        if getattr(state, "follow_lock", False):
            out += f" I have a lock on {target or 'my target'}."
        if pose:
            out += f" I'm {pose}."
        return out
    if status == "cancelling":
        return "I'm stopping."
    if status == "estop":
        return "Emergency stop is latched. The joystick has to release it before I can move."
    if status == "controller_inactive":
        return "My walking controller is inactive, so I can't move until it is activated."
    if status == "error":
        return f"The executor reported an error: {getattr(state, 'last_result', '') or 'unknown'}"
    return f"Executor status is {status}."
