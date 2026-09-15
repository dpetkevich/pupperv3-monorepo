import json
import os
import re

import pytest

from pupper_planner.plan_schema import Limits, PlanRejected, check_replan, load_schema, parse_plan_json, validate

HERE = os.path.dirname(__file__)
README = os.path.normpath(os.path.join(HERE, "..", "..", "pupper_interfaces", "schema", "README.md"))


def examples():
    txt = open(README).read()
    return [json.loads(m) for m in re.findall(r"```json\n(.*?)```", txt, re.S)]


def test_schema_loads():
    s = load_schema()
    assert s["title"] == "Pupper Plan v1"


@pytest.mark.parametrize("doc", examples())
def test_readme_examples_validate(doc):
    plan = validate(doc)
    assert plan.steps[0].skill == "turn"
    assert plan.mode == "replace"


def test_every_skill_minimal():
    steps = [
        {"id": "a", "skill": "turn", "args": {"degrees": -90}},
        {"id": "b", "skill": "move", "args": {"meters": 1.0}},
        {"id": "c", "skill": "go_to_pose", "args": {"x": 1, "y": 0}},
        {"id": "d", "skill": "return_to_start"},
        {"id": "e", "skill": "go_to_object", "args": {"label": "door"}},
        {"id": "f", "skill": "find_object", "args": {"label": "door"}},
        {"id": "g", "skill": "look", "args": {"prompt": "what is here"}},
        {"id": "h", "skill": "wait", "args": {"seconds": 2}},
        {"id": "i", "skill": "say", "args": {"text": "hi"}},
        {"id": "j", "skill": "animate", "args": {"name": "twerk"}},
        {"id": "k", "skill": "relax"},
        {"id": "l", "skill": "stop"},
    ]
    plan = validate({"v": 1, "steps": steps}, limits=Limits(max_total_s=1000, animation_names=["twerk"]))
    assert len(plan.steps) == 12


def test_unknown_skill_names_step():
    with pytest.raises(PlanRejected) as e:
        validate({"v": 1, "steps": [{"id": "s1", "skill": "fly", "args": {}}]})
    assert "step s1" in e.value.reason and "unknown skill 'fly'" in e.value.reason and "turn" in e.value.reason
    assert e.value.step_index == 0


def test_unknown_arg_names_step_and_allowed_keys():
    with pytest.raises(PlanRejected) as e:
        validate({"v": 1, "steps": [{"id": "s2", "skill": "turn", "args": {"deg": 90}}]})
    assert "step s2 (turn)" in e.value.reason and "degrees" in e.value.reason


def test_over_range_arg():
    with pytest.raises(PlanRejected) as e:
        validate({"v": 1, "steps": [{"id": "s1", "skill": "turn", "args": {"degrees": 1080}}]})
    assert "s1" in e.value.reason and "360" in e.value.reason


def test_total_translation_cap():
    steps = [{"id": f"s{i}", "skill": "move", "args": {"meters": 4}} for i in range(4)]
    with pytest.raises(PlanRejected) as e:
        validate({"v": 1, "steps": steps})
    assert "16.0 m" in e.value.reason and "15 m" in e.value.reason


def test_total_time_cap_and_step_timeout_cap():
    steps = [{"id": f"s{i}", "skill": "go_to_object", "args": {"label": "x"}} for i in range(3)]
    with pytest.raises(PlanRejected):
        validate({"v": 1, "steps": steps})  # 3 * 120 s default > 300
    with pytest.raises(PlanRejected) as e:
        validate({"v": 1, "steps": [{"id": "a", "skill": "wait", "args": {"seconds": 1}, "timeout_s": 500}]})
    assert "timeout" in e.value.reason.lower()


def test_too_many_steps_and_bad_json():
    steps = [{"id": f"s{i}", "skill": "say", "args": {"text": "x"}} for i in range(13)]
    with pytest.raises(PlanRejected):
        validate({"v": 1, "steps": steps})
    with pytest.raises(PlanRejected) as e:
        parse_plan_json("{not json")
    assert "not valid JSON" in e.value.reason


def test_duplicate_ids_and_unknown_animation():
    with pytest.raises(PlanRejected):
        validate({"v": 1, "steps": [{"id": "a", "skill": "stop"}, {"id": "a", "skill": "stop"}]})
    with pytest.raises(PlanRejected) as e:
        validate({"v": 1, "steps": [{"id": "a", "skill": "animate", "args": {"name": "moonwalk"}}]}, limits=Limits(animation_names=["twerk"]))
    assert "moonwalk" in e.value.reason


def test_replan_rules():
    plan = validate({"v": 1, "replan_of": "p1", "steps": [{"id": "a", "skill": "stop"}]})
    check_replan(plan, {"p1": None})
    with pytest.raises(PlanRejected):
        check_replan(plan, {"p1": "p0"})          # p1 was itself a replan
    with pytest.raises(PlanRejected):
        check_replan(plan, {"p1": None, "p2": "p1"})  # p1 already replanned
    with pytest.raises(PlanRejected):
        check_replan(plan, {})                    # unknown


def test_follow_until_and_timeouts():
    plan = validate({"v": 1, "steps": [
        {"id": "a", "skill": "follow_person", "args": {"who": "Amanda"}, "until": {"time_s": 30}},
        {"id": "b", "skill": "follow_person", "args": {"who": "nearest"}},
        {"id": "c", "skill": "wait", "args": {"seconds": 5}},
    ]})
    assert plan.steps[0].timeout() == 32.0
    assert plan.steps[1].timeout() == 0.0
    assert plan.steps[2].timeout() == 6.0
    assert plan.signature() == validate({"v": 1, "steps": plan_dict_steps(plan)}).signature()


def plan_dict_steps(plan):
    return [{"id": s.id, "skill": s.skill, "args": s.args, **({"until": s.until} if s.until else {})} for s in plan.steps]


def test_defaults_applied_when_bridge_omits_them():
    plan = validate({"v": 1, "steps": [{"id": "a", "skill": "stop"}]})
    assert (plan.mode, plan.source, plan.on_fail, plan.replan_of) == ("replace", "llm", "report", None)
