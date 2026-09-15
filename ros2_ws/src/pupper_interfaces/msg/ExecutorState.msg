# /plan_executor/state at 5 Hz, latched. The ONLY thing the agent may describe motion from.
builtin_interfaces/Time stamp
string status          # idle | running | cancelling | estop | controller_inactive | error
string plan_id
string source          # llm | reflex | cli
int32 step_index
int32 step_count
string current_skill
string current_target
float32 progress       # 0..1 within the current step
bool follow_lock
bool walking_active
bool estop_latched
string active_cmd_source
string pose_summary    # e.g. "1.8 m from start, heading 172 deg"
string last_result     # outcome + summary of the last finished plan
