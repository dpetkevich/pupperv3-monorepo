# pupper_interfaces

The contract between the voice agent, the plan executor, the motion server and (Phase B) the person tracker.
Actions in `action/`, services in `srv/`, messages in `msg/`, the Plan JSON schema in `schema/plan_v1.json` (rules in `schema/README.md`).
Sign convention everywhere: ROS/REP-103, turn degrees > 0 = left, dy / lateral_m > 0 = left, dx / meters > 0 = forward.
