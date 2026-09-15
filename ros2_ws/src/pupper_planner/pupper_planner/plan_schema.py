"""Plan JSON v1 loading, validation and the extra executor rules from schema/README.md.

rclpy-free. ``validate`` raises ``PlanRejected`` with a reason precise enough for the LLM to fix the plan once.
"""
from __future__ import annotations

import json
import os
import uuid
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional

import jsonschema

SAFETY_ABORT_REASONS = {"ESTOP", "TELEOP", "TELEOP_TAKEOVER", "FALLEN", "CONTROLLER_INACTIVE"}

# Default per-step timeouts (s) when the plan does not set ``timeout_s``.
DEFAULT_TIMEOUTS: Dict[str, float] = {
    "turn": 30.0,
    "move": 60.0,
    "go_to_pose": 90.0,
    "return_to_start": 90.0,
    "go_to_object": 120.0,
    "find_object": 90.0,
    "look": 15.0,
    "wait": 31.0,
    "say": 2.0,
    "animate": 20.0,
    "relax": 5.0,
    "stop": 5.0,
    "find_person": 60.0,
    "follow_person": 0.0,  # open-ended unless ``until`` says otherwise
}


class PlanRejected(Exception):
    def __init__(self, reason: str, step_index: int = -1) -> None:
        super().__init__(reason)
        self.reason = reason
        self.step_index = step_index


@dataclass
class Step:
    id: str
    skill: str
    args: Dict[str, Any] = field(default_factory=dict)
    timeout_s: Optional[float] = None
    until: Any = None

    def timeout(self) -> float:
        """Effective timeout in seconds; 0 means no deadline."""
        if self.timeout_s is not None:
            return float(self.timeout_s)
        if self.skill == "wait":
            return float(self.args.get("seconds", 0)) + 1.0
        if self.skill == "follow_person":
            u = self.until
            if isinstance(u, dict) and "time_s" in u:
                return float(u["time_s"]) + 2.0
            return 0.0
        return DEFAULT_TIMEOUTS.get(self.skill, 60.0)


@dataclass
class Plan:
    steps: List[Step]
    goal: str = ""
    mode: str = "replace"
    source: str = "llm"
    replan_of: Optional[str] = None
    on_fail: str = "report"
    plan_id: str = field(default_factory=lambda: uuid.uuid4().hex[:8])
    v: int = 1

    def signature(self) -> str:
        """Step-equivalence key used for the reflex/LLM dedupe rule."""
        return json.dumps([[s.skill, s.args] for s in self.steps], sort_keys=True)


@dataclass
class Limits:
    max_total_m: float = 15.0
    max_total_s: float = 300.0
    max_step_timeout_s: float = 120.0
    animation_names: Optional[List[str]] = None  # None = do not check


_SCHEMA_CACHE: Optional[Dict[str, Any]] = None


def load_schema(path: Optional[str] = None) -> Dict[str, Any]:
    """Load plan_v1.json: explicit path > $PUPPER_PLAN_SCHEMA > ament share > source-tree fallback."""
    global _SCHEMA_CACHE
    if path:
        with open(path) as f:
            return json.load(f)
    if _SCHEMA_CACHE is not None:
        return _SCHEMA_CACHE
    candidates = []
    env = os.environ.get("PUPPER_PLAN_SCHEMA")
    if env:
        candidates.append(env)
    try:
        from ament_index_python.packages import get_package_share_directory

        candidates.append(os.path.join(get_package_share_directory("pupper_interfaces"), "schema", "plan_v1.json"))
    except Exception:  # ament not available (tests) or package not installed
        pass
    here = os.path.dirname(os.path.abspath(__file__))
    candidates.append(os.path.normpath(os.path.join(here, "..", "..", "pupper_interfaces", "schema", "plan_v1.json")))
    for c in candidates:
        if os.path.exists(c):
            with open(c) as f:
                _SCHEMA_CACHE = json.load(f)
                return _SCHEMA_CACHE
    raise FileNotFoundError("plan_v1.json not found; tried: " + ", ".join(candidates))


def _known_skills(schema: Dict[str, Any]) -> Dict[str, Dict[str, Any]]:
    out = {}
    for alt in schema["properties"]["steps"]["items"]["oneOf"]:
        out[alt["properties"]["skill"]["const"]] = alt
    return out


def _explain_step(step: Dict[str, Any], idx: int, skills: Dict[str, Dict[str, Any]]) -> str:
    """Produce a precise error for one bad step (instead of jsonschema's opaque oneOf failure)."""
    sid = step.get("id", f"#{idx}") if isinstance(step, dict) else f"#{idx}"
    if not isinstance(step, dict):
        return f"step {sid}: must be an object"
    skill = step.get("skill")
    if skill not in skills:
        return f"step {sid}: unknown skill {skill!r}; valid skills: {', '.join(sorted(skills))}"
    try:
        jsonschema.validate(step, skills[skill])
        return f"step {sid}: invalid"
    except jsonschema.ValidationError as e:
        where = "/".join(str(p) for p in e.absolute_path)
        if e.validator == "additionalProperties":
            allowed = sorted(e.schema.get("properties", {}))
            return f"step {sid} ({skill}): {e.message}; allowed {where or 'keys'}: {', '.join(allowed)}"
        if e.validator == "required":
            return f"step {sid} ({skill}): {e.message}"
        return f"step {sid} ({skill}) {where}: {e.message}"


def validate(plan_dict: Any, schema: Optional[Dict[str, Any]] = None, limits: Optional[Limits] = None) -> Plan:
    """Validate a plan document. Raises ``PlanRejected(reason, step_index)``."""
    schema = schema or load_schema()
    limits = limits or Limits()
    if not isinstance(plan_dict, dict):
        raise PlanRejected("plan must be a JSON object with keys v and steps")
    skills = _known_skills(schema)
    steps_raw = plan_dict.get("steps")
    if isinstance(steps_raw, list):
        for i, s in enumerate(steps_raw):
            try:
                if isinstance(s, dict) and s.get("skill") in skills:
                    jsonschema.validate(s, skills[s["skill"]])
                    continue
            except jsonschema.ValidationError:
                pass
            raise PlanRejected(_explain_step(s, i, skills), i)
    try:
        jsonschema.validate(plan_dict, schema)
    except jsonschema.ValidationError as e:
        where = "/".join(str(p) for p in e.absolute_path) or "plan"
        raise PlanRejected(f"{where}: {e.message}")

    ids = [s["id"] for s in steps_raw]
    if len(set(ids)) != len(ids):
        raise PlanRejected("step ids must be unique")

    steps = [Step(id=s["id"], skill=s["skill"], args=dict(s.get("args", {})), timeout_s=s.get("timeout_s"), until=s.get("until")) for s in steps_raw]

    total_m = 0.0
    total_s = 0.0
    for i, st in enumerate(steps):
        if st.timeout_s is not None and st.timeout_s > limits.max_step_timeout_s:
            raise PlanRejected(f"step {st.id}: timeout_s {st.timeout_s} exceeds {limits.max_step_timeout_s}", i)
        if st.skill == "move":
            total_m += abs(float(st.args.get("meters", 0))) + abs(float(st.args.get("lateral_m", 0)))
        elif st.skill == "go_to_pose":
            total_m += (float(st.args.get("x", 0)) ** 2 + float(st.args.get("y", 0)) ** 2) ** 0.5
        elif st.skill == "animate" and limits.animation_names is not None:
            if st.args.get("name") not in limits.animation_names:
                raise PlanRejected(
                    f"step {st.id}: unknown animation {st.args.get('name')!r}; valid: {', '.join(limits.animation_names)}", i
                )
        total_s += st.timeout()
    if total_m > limits.max_total_m:
        raise PlanRejected(f"plan commands {total_m:.1f} m of translation; the limit is {limits.max_total_m:.0f} m")
    if total_s > limits.max_total_s:
        raise PlanRejected(f"plan could take up to {total_s:.0f} s; the limit is {limits.max_total_s:.0f} s (lower timeouts or fewer steps)")

    return Plan(
        steps=steps,
        goal=str(plan_dict.get("goal", "")),
        mode=plan_dict.get("mode", "replace"),
        source=plan_dict.get("source", "llm"),
        replan_of=plan_dict.get("replan_of"),
        on_fail=plan_dict.get("on_fail", "report"),
    )


def parse_plan_json(text: str, **kw) -> Plan:
    try:
        doc = json.loads(text)
    except json.JSONDecodeError as e:
        raise PlanRejected(f"plan_json is not valid JSON: {e.msg} at char {e.pos}")
    return validate(doc, **kw)


def check_replan(plan: Plan, replan_parents: Dict[str, Optional[str]]) -> None:
    """``replan_parents`` maps plan_id -> its replan_of. Rejects a replan of a replan or of an unknown plan."""
    if plan.replan_of is None:
        return
    if plan.replan_of not in replan_parents:
        raise PlanRejected(f"replan_of {plan.replan_of!r} is not a plan this executor ran")
    if replan_parents[plan.replan_of] is not None:
        raise PlanRejected(f"plan {plan.replan_of} was itself a replan; only one replan per original is allowed")
    if plan.replan_of in replan_parents.values():
        raise PlanRejected(f"plan {plan.replan_of} has already been replanned once")
