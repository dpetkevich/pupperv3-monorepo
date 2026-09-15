# Pupster voice agent

LiveKit Agents (console mode, local mic/speaker) + OpenAI realtime, with a deliberately small tool set. The robot's motion lives in ROS nodes (`plan_executor`, `motion_server`); this process only compiles speech into a plan, hands it over, and relays what the executor reports back.

## Run

On the robot (systemd unit `llm-agent.service` runs `run.sh`):

```bash
python3 src/agent.py console
```

On a laptop with no robot (fake executor, real prompt and voice loop):

```bash
python3 src/agent.py --tool-server nop console
```

Keys live in `.env.local` on the robot only (`OPENAI_API_KEY`; `GOOGLE_API_KEY` is read by the ROS vision node, not by this process).

## Tools exposed to the model

| tool | what it does |
|---|---|
| `execute_plan(plan_json)` | the only way to move: one Plan JSON v1 per command, validated locally then sent to `/plan_executor/execute_plan` |
| `stop()` | cancel the plan, `/plan_executor/stop`, `/motion_server/stop`, stop following; returns `status()` |
| `status()` | renders the latest `/plan_executor/state`; "not responding" if stale |
| `ask_scene(prompt)` | `/plan_executor/look` (Gemini) rendered as text + objects with headings |
| `animate(name)` | publishes to the animation controller and waits for the clip |
| `set_speaker_volume(volume)` | `wpctl` |
| `power(mode)` | activate/deactivate the walking controller |
| `follow_me()` / `stop_following()` | interim Trigger services until Phase B tracking lands |

Executor events (`/plan_executor/events`) are turned into speech by `PlanWatcher`: plan done/failed/cancelled and `say` steps are spoken; step events become throttled system notes so "what are you doing" is answered from executor truth.

## Layout

- `src/agent.py` entrypoint, event wiring, `--tool-server ros|nop`
- `src/pupster.py` persona + tools + session builders
- `src/executor_bridge.py` rclpy node on a background executor thread; async tool implementations
- `src/nop_bridge.py` same interface, fake executor
- `src/plan_watcher.py` executor events → speech / chat context
- `src/plan_schema.py` loads `pupper_interfaces/schema/plan_v1.json`, validates, describes
- `src/ros_async.py` rclpy future → asyncio (never `spin_until_future_complete`)
- `src/speech_render.py` state/scene → short English
- `src/animations.py` animation catalogue
- `src/system_prompt.md` the plan-compilation spec the model follows

## Tests

```bash
python -m pytest tests -q
```

`tests/test_prompt_examples.py` validates every JSON block in the prompt against the shared schema; the bridge tests mock rclpy.
