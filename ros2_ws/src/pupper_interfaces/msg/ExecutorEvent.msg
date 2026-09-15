# /plan_executor/events, reliable, transient-local, depth 50
builtin_interfaces/Time stamp
string plan_id
string source
string type            # plan_accepted | step_started | step_progress | step_done | step_failed | plan_done | plan_failed | plan_cancelled | reflex | say
string step_id
string skill
string summary         # <= 15 words, speakable
string detail_json
