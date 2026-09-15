"""Every ```json block in system_prompt.md must be a valid Plan v1 (and speakable)."""

import json
import re
from pathlib import Path

import pytest

import plan_schema
from plan_schema import PlanError, describe_plan, validate_plan

PROMPT = Path(__file__).resolve().parents[1] / "src" / "system_prompt.md"
BLOCKS = re.findall(r"```json\n(.*?)```", PROMPT.read_text(encoding="utf-8"), re.S)


def test_prompt_has_examples() -> None:
    assert len(BLOCKS) >= 5


@pytest.mark.parametrize("block", BLOCKS, ids=[f"block{i}" for i in range(len(BLOCKS))])
def test_prompt_example_validates(block: str) -> None:
    plan = validate_plan(block)
    assert plan["v"] == 1
    assert describe_plan(plan)


def test_schema_comes_from_interfaces_package() -> None:
    assert plan_schema.schema_path().name == "plan_v1.json"
    assert "pupper_interfaces" in str(plan_schema.schema_path())


def test_execute_plan_docstring_example_validates() -> None:
    import sys
    from unittest.mock import MagicMock

    for m in ("livekit", "livekit.agents", "livekit.agents.llm", "livekit.plugins", "openai", "openai.types",
              "openai.types.beta", "openai.types.beta.realtime", "openai.types.beta.realtime.session"):
        sys.modules.setdefault(m, MagicMock())
    sys.modules["livekit.agents.llm"].function_tool = lambda *a, **k: (a[0] if a and callable(a[0]) else (lambda f: f))
    import importlib

    pupster = importlib.import_module("pupster")
    example = re.search(r"Example, .*?:\n(\{.*?\})\n\nArgs", pupster.EXECUTE_PLAN_DOC, re.S).group(1)
    validate_plan(example)


@pytest.mark.parametrize(
    "bad,fragment",
    [
        ('{"v":1,"steps":[{"id":"s1","skill":"spin","args":{}}]}', "unknown skill 'spin'"),
        ('{"v":1,"steps":[{"id":"s1","skill":"turn","args":{"degrees":720}}]}', "maximum of 360"),
        ("not json", "not valid JSON"),
        ('{"v":1,"steps":[]}', "non-empty"),
        ('{"v":2,"steps":[{"id":"s1","skill":"stop","args":{}}]}', "v"),
    ],
)
def test_rejections_are_precise(bad: str, fragment: str) -> None:
    with pytest.raises(PlanError) as e:
        validate_plan(bad)
    assert fragment in str(e.value)
