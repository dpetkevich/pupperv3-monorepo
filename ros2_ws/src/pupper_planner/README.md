# pupper_planner

`plan_executor` node: runs Plan JSON v1 documents (see `pupper_interfaces/schema`) step by step, one motion goal at a time, and is the single source of truth for what the robot is doing.

- Action `/plan_executor/execute_plan` — goals are always accepted; schema/limit errors come back within ~50 ms as `outcome: REJECTED` with a reason the LLM can fix.
- Topics `/plan_executor/state` (latched, 5 Hz) and `/plan_executor/events` (transient-local; `plan_done|plan_failed|plan_cancelled|say|step_*`). The agent speaks only from these.
- Services `/plan_executor/stop`, `/plan_executor/prepare`, `/plan_executor/look` (Gemini camera look via `pupper_vision`).
- `runner.py` (rclpy-free plan walker, summaries, on_fail), `skills/` (motion wrappers, `go_to_object` loop, `look`), `plan_schema.py`, `action_bridge.py`.

Tests: `pytest test` (rclpy mocked). End-to-end: `ros2_ws/docker/sim_test.sh`.
