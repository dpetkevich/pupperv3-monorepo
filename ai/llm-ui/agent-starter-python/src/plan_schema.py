"""Plan JSON v1 helpers shared by the ROS bridge, the nop bridge and the tests.

The schema itself lives in the ROS interface package
(``pupper_interfaces/schema/plan_v1.json``) so the executor and the agent can
never disagree about it. This module only locates it and renders plans for speech.
"""

from __future__ import annotations

import json
import os
from functools import lru_cache
from pathlib import Path
from typing import Any

import jsonschema

SCHEMA_ENV = "PUPPER_PLAN_SCHEMA"
_REPO_RELATIVE = Path(__file__).resolve().parents[4] / "ros2_ws" / "src" / "pupper_interfaces" / "schema" / "plan_v1.json"

MAX_TOTAL_METERS = 15.0
MAX_TOTAL_SECONDS = 300.0


class PlanError(ValueError):
    """A plan the executor would reject; the message is written for the LLM to fix once."""


def schema_path() -> Path:
    override = os.environ.get(SCHEMA_ENV)
    if override:
        return Path(override)
    try:  # installed on the robot
        from ament_index_python.packages import get_package_share_directory

        return Path(get_package_share_directory("pupper_interfaces")) / "schema" / "plan_v1.json"
    except Exception:
        return _REPO_RELATIVE


@lru_cache(maxsize=1)
def load_schema() -> dict[str, Any]:
    with open(schema_path(), "r", encoding="utf-8") as f:
        return json.load(f)


def _validator() -> jsonschema.Draft7Validator:
    return jsonschema.Draft7Validator(load_schema())


@lru_cache(maxsize=1)
def known_skills() -> tuple[str, ...]:
    variants = load_schema()["properties"]["steps"]["items"]["oneOf"]
    return tuple(v["properties"]["skill"]["const"] for v in variants)


def _explain(error: jsonschema.ValidationError) -> str:
    path = "/".join(str(p) for p in error.absolute_path) or "plan"
    if error.validator == "oneOf" and error.context:
        step = error.instance if isinstance(error.instance, dict) else {}
        skill = step.get("skill")
        skills = known_skills()
        if skill not in skills:
            return f"{path}: unknown skill '{skill}'; known skills: {', '.join(skills)}"
        idx = skills.index(skill)
        for sub in error.context:
            if sub.schema_path and sub.schema_path[0] == idx and sub.validator != "const":
                sub_path = "/".join(str(p) for p in sub.absolute_path)
                return f"{path}: skill '{skill}' rejected at {sub_path or 'step'}: {sub.message}"
        return f"{path}: skill '{skill}' has invalid fields"
    return f"{path}: {error.message}"


def validate_plan(plan_json: str | dict[str, Any]) -> dict[str, Any]:
    """Parse and validate a plan. Returns the plan dict or raises PlanError."""
    if isinstance(plan_json, str):
        try:
            plan = json.loads(plan_json)
        except json.JSONDecodeError as e:
            raise PlanError(f"plan_json is not valid JSON: {e.msg} at position {e.pos}") from e
    else:
        plan = plan_json
    if not isinstance(plan, dict):
        raise PlanError("plan must be a JSON object with 'v' and 'steps'")
    errors = sorted(_validator().iter_errors(plan), key=lambda e: list(e.absolute_path))
    if errors:
        raise PlanError("; ".join(_explain(e) for e in errors[:3]))
    _check_budgets(plan)
    return plan


def _check_budgets(plan: dict[str, Any]) -> None:
    meters = 0.0
    seconds = 0.0
    for step in plan["steps"]:
        args = step.get("args", {})
        skill = step["skill"]
        if skill == "move":
            meters += abs(args.get("meters", 0.0)) + abs(args.get("lateral_m", 0.0))
        elif skill == "go_to_pose":
            meters += (args.get("x", 0.0) ** 2 + args.get("y", 0.0) ** 2) ** 0.5
        elif skill == "wait":
            seconds += args.get("seconds", 0.0)
        seconds += step.get("timeout_s", 0.0)
    if meters > MAX_TOTAL_METERS:
        raise PlanError(f"plan commands {meters:.1f} m of translation; the limit is {MAX_TOTAL_METERS:.0f} m")
    if seconds > MAX_TOTAL_SECONDS:
        raise PlanError(f"plan budgets {seconds:.0f} s; the limit is {MAX_TOTAL_SECONDS:.0f} s")


def describe_step(step: dict[str, Any]) -> str:
    """Short spoken description of one step, e.g. 'turn 180 degrees left'."""
    skill = step.get("skill", "?")
    a = step.get("args", {}) or {}
    if skill == "turn":
        deg = float(a.get("degrees", 0))
        side = "left" if deg > 0 else "right"
        return f"turn {abs(deg):.0f} degrees {side}"
    if skill == "move":
        m = float(a.get("meters", 0))
        lat = float(a.get("lateral_m", 0) or 0)
        parts = []
        if m:
            parts.append(f"walk {abs(m):.1f} metres {'forward' if m > 0 else 'backward'}")
        if lat:
            parts.append(f"{abs(lat):.1f} metres {'left' if lat > 0 else 'right'}")
        return " and ".join(parts) or "hold position"
    if skill == "go_to_pose":
        return f"go to x {a.get('x', 0)}, y {a.get('y', 0)} in the {a.get('frame', 'start')} frame"
    if skill == "return_to_start":
        return "return to where I started"
    if skill == "go_to_object":
        return f"walk to the {a.get('label', 'target')}"
    if skill == "find_object":
        return f"look around for the {a.get('label', 'target')}"
    if skill == "look":
        return "look at the scene"
    if skill == "wait":
        return f"wait {float(a.get('seconds', 0)):.0f} seconds"
    if skill == "say":
        return f"say '{a.get('text', '')}'"
    if skill == "animate":
        return f"do the {a.get('name', '')} animation"
    if skill == "relax":
        return "relax"
    if skill == "stop":
        return "stop"
    if skill == "find_person":
        return f"find {a.get('who', 'someone')}"
    if skill == "follow_person":
        return f"follow {a.get('who', 'someone')}"
    return skill


def describe_plan(plan: dict[str, Any], max_steps: int = 4) -> str:
    steps = [describe_step(s) for s in plan.get("steps", [])]
    if len(steps) > max_steps:
        return ", then ".join(steps[:max_steps]) + f", and {len(steps) - max_steps} more"
    return ", then ".join(steps)
