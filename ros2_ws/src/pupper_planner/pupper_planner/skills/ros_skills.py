"""RosSkills: the ``Skills`` implementation that talks to the motion server, animation controller and camera.

Action/service names (motion server contract):
  /motion_server/turn_degrees  /motion_server/move_meters  /motion_server/go_to_pose  /motion_server/return_to_start
  /motion_server/follow_person /motion_server/find_person  (Phase B)
  /motion_server/mark_pose (MarkPose)  /motion_server/stop (Trigger)
Sign convention: ROS/REP-103 throughout; object headings from LookSkill are already +left.
"""
from __future__ import annotations

import threading
import time
from typing import Any, Callable, Dict, List, Optional

from controller_manager_msgs.srv import SwitchController
from rclpy.action import ActionClient
from std_msgs.msg import String
from std_srvs.srv import Trigger

from pupper_interfaces.action import FindPerson, FollowPerson, GoToPose, MoveMeters, ReturnToStart, TurnDegrees
from pupper_interfaces.srv import MarkPose

from ..action_bridge import send_and_wait
from ..lat import lat_line
from ..plan_schema import Step
from ..runner import StepResult
from ..service_bridge import call_service
from .go_to_object import filter_label, find_object, go_to_object
from .look import LookSkill

ALL_CONTROLLERS = [
    "neural_controller",
    "neural_controller_three_legged",
    "forward_kp_controller",
    "forward_kd_controller",
    "forward_position_controller",
]

EmitEvent = Callable[[str, Optional[Step], str, Dict[str, Any]], None]


def _status_to_reason(status: str, result: Any) -> str:
    if status == "SUCCEEDED":
        return "" if getattr(result, "success", True) else (getattr(result, "reason", "") or "FAILED")
    if status == "CANCELED":
        return "CANCELLED"
    if status in ("TIMEOUT", "UNAVAILABLE", "REJECTED"):
        return status if status != "UNAVAILABLE" else "ACTION_UNAVAILABLE"
    return getattr(result, "reason", "") or status  # ABORTED with a reason from the server


class RosSkills:
    def __init__(
        self,
        node: Any,
        look: Optional[LookSkill],
        emit_event: EmitEvent,
        animate_wait_s: float = 8.0,
        set_follow_lock: Optional[Callable[[bool], None]] = None,
        callback_group: Any = None,
    ) -> None:
        self.node = node
        self.look_skill = look
        self.emit_event = emit_event
        self.animate_wait_s = animate_wait_s
        self.set_follow_lock = set_follow_lock or (lambda _b: None)
        cg = callback_group
        self.turn_client = ActionClient(node, TurnDegrees, "/motion_server/turn_degrees", callback_group=cg)
        self.move_client = ActionClient(node, MoveMeters, "/motion_server/move_meters", callback_group=cg)
        self.pose_client = ActionClient(node, GoToPose, "/motion_server/go_to_pose", callback_group=cg)
        self.return_client = ActionClient(node, ReturnToStart, "/motion_server/return_to_start", callback_group=cg)
        self.follow_client = ActionClient(node, FollowPerson, "/motion_server/follow_person", callback_group=cg)
        self.find_client = ActionClient(node, FindPerson, "/motion_server/find_person", callback_group=cg)
        self.mark_client = node.create_client(MarkPose, "/motion_server/mark_pose", callback_group=cg)
        self.stop_client = node.create_client(Trigger, "/motion_server/stop", callback_group=cg)
        self.switch_client = node.create_client(SwitchController, "/controller_manager/switch_controller", callback_group=cg)
        self.anim_pub = node.create_publisher(String, "/animation_controller_py/animation_select", 10)
        self.current_plan_id = ""
        self._first_goal_sent = False

    # ------------------------------------------------------------ Skills protocol
    def begin_plan(self, plan_id: str) -> None:
        self.current_plan_id = plan_id
        self._first_goal_sent = False

    def mark_start(self) -> None:
        resp = call_service(self.mark_client, MarkPose.Request(name="start"), timeout_s=1.0)
        if resp is None or not resp.ok:
            self.node.get_logger().warning("mark_pose(start) failed; return_to_start will use the last mark")

    def stop(self) -> None:
        self.node.get_logger().info(lat_line("cancel_sent", self.current_plan_id))
        call_service(self.stop_client, Trigger.Request(), timeout_s=1.0)

    def execute(self, step: Step, deadline: float, cancel: threading.Event, feedback: Callable[[str, float], None]) -> StepResult:
        handler = getattr(self, f"_skill_{step.skill}", None)
        if handler is None:
            return StepResult(False, reason=f"UNKNOWN_SKILL:{step.skill}")
        return handler(step, deadline, cancel, feedback)

    # ------------------------------------------------------------ helpers
    def _send(self, client: Any, goal: Any, deadline: float, cancel: threading.Event, on_fb=None):
        if not self._first_goal_sent:
            self._first_goal_sent = True
            self.node.get_logger().info(lat_line("first_goal_sent", self.current_plan_id))
        return send_and_wait(client, goal, on_fb, deadline, cancel)

    def _turn(self, degrees: float, deadline: float, cancel: threading.Event, feedback=None, speed_dps: float = 0.0):
        goal = TurnDegrees.Goal(degrees=float(degrees), speed_dps=float(speed_dps))

        def fb(f):
            if feedback and degrees:
                feedback("turn", min(1.0, abs(f.turned_degrees) / abs(degrees)))

        status, result = self._send(self.turn_client, goal, deadline, cancel, fb)
        reason = _status_to_reason(status, result)
        turned = float(getattr(result, "turned_degrees", 0.0) or 0.0)
        return reason == "", reason, turned

    def _move(self, dx: float, dy: float, deadline: float, cancel: threading.Event, feedback=None, speed: float = 0.0, hold: bool = True):
        goal = MoveMeters.Goal(dx=float(dx), dy=float(dy), speed_mps=float(speed), hold_heading=bool(hold))
        total = (dx * dx + dy * dy) ** 0.5

        def fb(f):
            if feedback and total:
                feedback("move", min(1.0, f.moved_m / total))

        status, result = self._send(self.move_client, goal, deadline, cancel, fb)
        reason = _status_to_reason(status, result)
        moved = float(getattr(result, "moved_m", 0.0) or 0.0)
        return reason == "", reason, moved, result

    # ------------------------------------------------------------ skills
    def _skill_turn(self, step, deadline, cancel, feedback):
        deg = float(step.args["degrees"])
        ok, reason, turned = self._turn(deg, deadline, cancel, feedback, step.args.get("speed_dps", 0.0))
        side = "left" if turned > 0 else "right"
        return StepResult(ok, reason, f"turned {abs(turned):.0f}° {side}" if abs(turned) >= 1 else "", {"turned_degrees": turned})

    def _skill_move(self, step, deadline, cancel, feedback):
        dx = float(step.args["meters"])
        dy = float(step.args.get("lateral_m", 0.0))
        ok, reason, moved, result = self._move(dx, dy, deadline, cancel, feedback, step.args.get("speed_mps", 0.0))
        word = "forward" if dx >= 0 else "backward"
        if dx == 0 and dy != 0:
            word = "left" if dy > 0 else "right"
        sigma = float(getattr(result, "sigma_m", 0.0) or 0.0)
        return StepResult(ok, reason, f"walked {moved:.1f} m {word}" if moved > 0.05 else "", {"moved_m": moved, "sigma_m": sigma})

    def _skill_go_to_pose(self, step, deadline, cancel, feedback):
        a = step.args
        goal = GoToPose.Goal(x=float(a["x"]), y=float(a["y"]), yaw_deg=float(a.get("yaw_deg", 0.0)), frame=a.get("frame", "start"), final_turn="yaw_deg" in a)
        status, result = self._send(self.pose_client, goal, deadline, cancel, lambda f: feedback(f.phase, 0.5))
        reason = _status_to_reason(status, result)
        err = float(getattr(result, "distance_error_m", 0.0) or 0.0)
        return StepResult(reason == "", reason, f"reached the spot (within ~{err:.1f} m)" if reason == "" else "", {"distance_error_m": err})

    def _skill_return_to_start(self, step, deadline, cancel, feedback):
        goal = ReturnToStart.Goal(restore_heading=bool(step.args.get("restore_heading", False)))
        status, result = self._send(self.return_client, goal, deadline, cancel, lambda f: feedback(f.phase, 0.5))
        reason = _status_to_reason(status, result)
        err = float(getattr(result, "distance_error_m", 0.0) or 0.0)
        return StepResult(reason == "", reason, f"back within ~{max(err, 0.1):.1f} m of the start" if reason == "" else "", {"distance_error_m": err})

    def _skill_stop(self, step, deadline, cancel, feedback):
        self.stop()
        return StepResult(True, summary="stopped")

    def _skill_wait(self, step, deadline, cancel, feedback):
        secs = float(step.args["seconds"])
        t0 = time.monotonic()
        while time.monotonic() - t0 < secs:
            if cancel.is_set():
                return StepResult(False, "CANCELLED")
            if time.monotonic() >= deadline:
                return StepResult(False, "TIMEOUT")
            feedback("wait", (time.monotonic() - t0) / max(secs, 1e-6))
            time.sleep(0.05)
        return StepResult(True, summary=f"waited {secs:.0f} s" if secs >= 1 else "")

    def _skill_say(self, step, deadline, cancel, feedback):
        text = str(step.args["text"])
        self.emit_event("say", step, text, {})
        return StepResult(True, summary="", detail={"said": text})

    def _skill_animate(self, step, deadline, cancel, feedback):
        name = str(step.args["name"])
        self.anim_pub.publish(String(data=name))
        t0 = time.monotonic()
        while time.monotonic() - t0 < self.animate_wait_s:
            if cancel.is_set():
                return StepResult(False, "CANCELLED")
            time.sleep(0.05)
        return StepResult(True, summary=f"did the {name} animation", detail={"animation": name})

    def _skill_relax(self, step, deadline, cancel, feedback):
        req = SwitchController.Request()
        req.activate_controllers = []
        req.deactivate_controllers = list(ALL_CONTROLLERS)
        req.strictness = SwitchController.Request.BEST_EFFORT
        resp = call_service(self.switch_client, req, timeout_s=2.0)
        if resp is None or not resp.ok:
            return StepResult(False, "SWITCH_FAILED", "could not relax the motors")
        return StepResult(True, summary="relaxed")

    def _skill_look(self, step, deadline, cancel, feedback):
        if self.look_skill is None:
            return StepResult(False, "NO_VISION", "my camera is not available")
        out = self.look_skill.look(str(step.args["prompt"]), cancel=cancel)
        if not out.ok:
            return StepResult(False, out.reason, "I could not get a look")
        short = " ".join(out.text.split()[:15])
        return StepResult(True, summary=short, detail={"text": out.text, "objects": out.objects})

    def _look_for(self, label: str, stop_distance_m: float, cancel: threading.Event) -> List[Dict[str, Any]]:
        prompt = (
            f"Is there a {label} visible? Answer yes or no and where it is (left, centre, right, and roughly how far in metres), "
            f"then return the JSON array with a box for the {label} only (empty array if not visible)."
        )
        out = self.look_skill.look(prompt, stop_distance_m=stop_distance_m, cancel=cancel)
        if not out.ok:
            if out.reason == "CANCELLED":
                cancel.set()
            return []
        return filter_label(out.objects, label)

    def _skill_go_to_object(self, step, deadline, cancel, feedback):
        if self.look_skill is None:
            return StepResult(False, "NO_VISION", "my camera is not available")
        a = step.args
        label = str(a["label"])
        stop_d = float(a.get("stop_distance_m", 0.8))
        r = go_to_object(
            label,
            look=lambda lbl: self._look_for(lbl, stop_d, cancel),
            turn=lambda deg: self._turn(deg, deadline, cancel),
            move=lambda m: self._move(m, 0.0, deadline, cancel)[:3],
            cancel=cancel,
            stop_distance_m=stop_d,
            max_iterations=int(a.get("max_iterations", 6)),
            search=bool(a.get("search", True)),
            search_step_deg=60.0,
            feedback=feedback,
        )
        return StepResult(r.ok, "" if r.ok else r.reason, r.summary, r.detail)

    def _skill_find_object(self, step, deadline, cancel, feedback):
        if self.look_skill is None:
            return StepResult(False, "NO_VISION", "my camera is not available")
        label = str(step.args["label"])
        r = find_object(
            label,
            look=lambda lbl: self._look_for(lbl, 0.8, cancel),
            turn=lambda deg: self._turn(deg, deadline, cancel),
            cancel=cancel,
            search_step_deg=float(step.args.get("search_step_deg", 60.0)),
        )
        if r.ok:
            obj = r.detail.get("object", {})
            h = float(obj.get("heading_deg", 0.0))
            if abs(h) > 8.0:
                self._turn(h, deadline, cancel)  # face it so the next step starts aligned
        return StepResult(r.ok, "" if r.ok else r.reason, r.summary, r.detail)

    # ---------------------------------------------------------------- Phase A fallbacks (until Phase B lands)
    def _find_person_fallback(self, who: str, deadline, cancel) -> StepResult:
        """No FindPerson action server yet: rotate in 60 deg steps looking for a person with Gemini, then face them."""
        if self.look_skill is None:
            return StepResult(False, "NO_VISION", "my camera is not available")
        r = find_object("person", look=lambda lbl: self._look_for(lbl, 1.0, cancel), turn=lambda deg: self._turn(deg, deadline, cancel), cancel=cancel, search_step_deg=60.0)
        name = who if who not in ("nearest", "speaker") else "you"
        if not r.ok:
            return StepResult(False, r.reason, f"could not find {name}", r.detail)
        obj = r.detail.get("object", {})
        h = float(obj.get("heading_deg", 0.0))
        if abs(h) > 8.0:
            self._turn(h, deadline, cancel)
        return StepResult(True, "", f"found {name}", {"heading_deg": h})

    def _follow_person_fallback(self, who: str) -> StepResult:
        """No FollowPerson action yet: hand over to the legacy person_follower node (needs walking active)."""
        if not hasattr(self, "_legacy_follow_client"):
            self._legacy_follow_client = self.node.create_client(Trigger, "activate_person_following", callback_group=self.stop_client.callback_group)
        resp = call_service(self._legacy_follow_client, Trigger.Request(), timeout_s=2.0)
        if resp is None or not resp.success:
            return StepResult(False, "ACTION_UNAVAILABLE", "following is not available right now")
        name = who if who not in ("nearest", "speaker") else "you"
        return StepResult(True, "", f"following {name} with the camera tracker", {"legacy": True})

    def _skill_find_person(self, step, deadline, cancel, feedback):
        who = str(step.args["who"])
        goal = FindPerson.Goal(name="" if who in ("nearest", "speaker") else who, max_search_deg=0.0, then_follow=False)
        status, result = self._send(self.find_client, goal, deadline, cancel, lambda f: feedback(f.phase, min(1.0, f.searched_deg / 360.0)))
        if status == "UNAVAILABLE":
            return self._find_person_fallback(who, deadline, cancel)
        reason = _status_to_reason(status, result)
        found = bool(getattr(result, "found", False))
        if reason == "" and not found:
            reason = getattr(result, "outcome", "NOT_FOUND") or "NOT_FOUND"
        name = who if who not in ("nearest", "speaker") else "someone"
        return StepResult(found and reason == "", reason, f"found {name}" if found else f"could not find {name}", {"track_id": getattr(result, "track_id", -1)})

    def _skill_follow_person(self, step, deadline, cancel, feedback):
        who = str(step.args["who"])
        goal = FollowPerson.Goal(name="" if who in ("nearest", "speaker") else who, track_id=-1, target_range_m=float(step.args.get("target_range_m", 0.0)), timeout_s=0.0)
        until = step.until
        lost_limit = float(until["lost_for_s"]) if isinstance(until, dict) and "lost_for_s" in until else None
        state = {"lost_s": 0.0}

        def fb(f):
            self.set_follow_lock(bool(f.has_lock))
            state["lost_s"] = float(f.lost_s)
            feedback(f.phase, 1.0 if f.has_lock else 0.0)
            if lost_limit is not None and f.lost_s >= lost_limit:
                cancel.set()

        status, result = self._send(self.follow_client, goal, deadline, cancel, fb)
        self.set_follow_lock(False)
        if status == "UNAVAILABLE":
            return self._follow_person_fallback(who)
        outcome = getattr(result, "outcome", "") or status
        dur = float(getattr(result, "duration_s", 0.0) or 0.0)
        name = who if who not in ("nearest", "speaker") else "you"
        if status in ("TIMEOUT",) or (status == "CANCELED" and (lost_limit is not None and state["lost_s"] >= lost_limit)):
            return StepResult(True, summary=f"followed {name} for {dur:.0f} s", detail={"outcome": outcome})
        if status == "CANCELED":
            return StepResult(False, "CANCELLED", f"stopped following {name}")
        if outcome == "LOST":
            return StepResult(False, "LOST", f"lost {name} after {dur:.0f} s")
        return StepResult(status == "SUCCEEDED", "" if status == "SUCCEEDED" else outcome, f"followed {name} for {dur:.0f} s", {"outcome": outcome})
