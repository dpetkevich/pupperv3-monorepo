"""go_to_object / find_object decision logic. rclpy-free: motion and vision are injected callables.

Headings passed in are ROS sign (+left) so ``turn(heading)`` rotates toward the object directly.
"""
from __future__ import annotations

import threading
from dataclasses import dataclass
from typing import Any, Callable, Dict, List, Optional, Tuple

LookFn = Callable[[str], List[Dict[str, Any]]]          # label -> objects (from Look, already filtered to label matches)
TurnFn = Callable[[float], Tuple[bool, str, float]]     # degrees -> (ok, reason, turned)
MoveFn = Callable[[float], Tuple[bool, str, float]]     # metres -> (ok, reason, moved)

HEADING_TOL_DEG = 8.0
MIN_PUSH_M = 0.4
MAX_PUSH_M = 1.5
DEFAULT_PUSH_M = 0.8


@dataclass
class ObjectResult:
    ok: bool
    reason: str            # ARRIVED | NOT_FOUND | MAX_ITERATIONS | CANCELLED | child reason
    summary: str
    detail: Dict[str, Any]


def _best(objects: List[Dict[str, Any]]) -> Optional[Dict[str, Any]]:
    if not objects:
        return None
    # prefer the closest estimate, then the most central
    return sorted(objects, key=lambda o: (o.get("distance_est_m") if o.get("distance_est_m") is not None else 99.0, abs(o.get("heading_deg", 0))))[0]


def find_object(label: str, look: LookFn, turn: TurnFn, cancel: threading.Event, search_step_deg: float = 60.0, max_steps: int = 6) -> ObjectResult:
    """Look, then rotate in ``search_step_deg`` increments (left, ROS +) looking after each turn."""
    searched = 0.0
    for i in range(max_steps + 1):
        if cancel.is_set():
            return ObjectResult(False, "CANCELLED", "stopped while searching", {"searched_deg": searched})
        objs = look(label)
        best = _best(objs)
        if best is not None:
            return ObjectResult(True, "FOUND", f"found the {label}", {"object": best, "searched_deg": searched})
        if i == max_steps or searched + search_step_deg > 360.0 + 1e-6:
            break
        ok, reason, turned = turn(search_step_deg)
        if not ok:
            return ObjectResult(False, reason, f"could not turn while searching for the {label}", {"searched_deg": searched})
        searched += abs(turned)
    return ObjectResult(False, "NOT_FOUND", f"could not find the {label} after looking all around", {"searched_deg": searched})


def go_to_object(
    label: str,
    look: LookFn,
    turn: TurnFn,
    move: MoveFn,
    cancel: threading.Event,
    stop_distance_m: float = 0.8,
    max_iterations: int = 6,
    search: bool = True,
    search_step_deg: float = 60.0,
    feedback: Optional[Callable[[str, float], None]] = None,
) -> ObjectResult:
    fb = feedback or (lambda *a: None)
    walked = 0.0
    last: Optional[Dict[str, Any]] = None
    for it in range(max_iterations):
        if cancel.is_set():
            return ObjectResult(False, "CANCELLED", f"stopped on the way to the {label}", {"walked_m": walked})
        fb("look", it / max_iterations)
        objs = look(label)
        best = _best(objs)
        if best is None:
            if it == 0 and search:
                fr = find_object(label, look, turn, cancel, search_step_deg)
                if not fr.ok:
                    return ObjectResult(False, fr.reason, fr.summary, {**fr.detail, "walked_m": walked})
                best = fr.detail["object"]
            elif last is not None:
                # lost it after moving: one blind push toward the last known heading, then re-look
                return ObjectResult(False, "LOST", f"lost sight of the {label} after walking {walked:.1f} m", {"walked_m": walked, "last": last})
            else:
                return ObjectResult(False, "NOT_FOUND", f"cannot see the {label}", {"walked_m": walked})
        last = best
        heading = float(best.get("heading_deg", 0.0))
        if abs(heading) > HEADING_TOL_DEG:
            fb("turn", it / max_iterations)
            ok, reason, _ = turn(heading)
            if not ok:
                return ObjectResult(False, reason, f"could not turn toward the {label}", {"walked_m": walked})
            if cancel.is_set():
                return ObjectResult(False, "CANCELLED", f"stopped on the way to the {label}", {"walked_m": walked})
        dist = best.get("distance_est_m")
        if best.get("close") or (dist is not None and dist <= stop_distance_m):
            d_txt = f" (~{dist:.1f} m)" if dist is not None else ""
            return ObjectResult(True, "ARRIVED", f"reached the {label}{d_txt}", {"walked_m": walked, "distance_est_m": dist, "iterations": it + 1})
        push = DEFAULT_PUSH_M if dist is None else max(MIN_PUSH_M, min(MAX_PUSH_M, dist - stop_distance_m))
        fb("move", it / max_iterations)
        ok, reason, moved = move(push)
        walked += abs(moved)
        if not ok:
            return ObjectResult(False, reason, f"could not walk toward the {label}", {"walked_m": walked})
    return ObjectResult(False, "MAX_ITERATIONS", f"walked {walked:.1f} m toward the {label} but never got close enough", {"walked_m": walked, "last": last})


def filter_label(objects: List[Dict[str, Any]], label: str) -> List[Dict[str, Any]]:
    """Objects whose label shares a word with the requested label (Gemini paraphrases)."""
    stop = {"the", "a", "an", "my", "your", "our", "his", "her", "their", "that", "this", "some"}
    want = {w for w in label.lower().replace("-", " ").split() if len(w) > 2 and w not in stop}
    out = []
    for o in objects:
        have = set(str(o.get("label", "")).lower().replace("-", " ").split())
        if want & have:
            out.append(o)
    return out
