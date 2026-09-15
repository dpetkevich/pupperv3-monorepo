# Plan JSON v1

Validated by `pupper_planner` with `jsonschema` before a goal is accepted. Beyond the schema the executor also enforces:

- total commanded translation <= 15 m, total planned duration <= 300 s, per-step `timeout_s` <= 120
- `on_fail: return_to_start_then_report` is never run after a safety abort (ESTOP / TELEOP / FALLEN)
- at most one `replan_of` per original plan; a replan of a replan is REJECTED
- `animate.name` must be in the agent's `ANIMATION_NAMES`
- dedupe: an `llm` plan step-equivalent to a `reflex` plan started < 5 s earlier is accepted as `already_running`

Example:

```json
{"v":1,"goal":"walk to the bed and back","mode":"replace","source":"llm","on_fail":"report",
 "steps":[{"id":"s1","skill":"turn","args":{"degrees":180}},
          {"id":"s2","skill":"go_to_object","args":{"label":"bed","stop_distance_m":0.8},"timeout_s":90},
          {"id":"s3","skill":"return_to_start","args":{"restore_heading":true}},
          {"id":"s4","skill":"say","args":{"text":"Back where I started."}}]}
```
