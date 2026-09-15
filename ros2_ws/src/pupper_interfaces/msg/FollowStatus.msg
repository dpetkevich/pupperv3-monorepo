# /motion_server/follow_status, latched
builtin_interfaces/Time stamp
bool active
bool has_lock
string target_name
int32 track_id
string phase           # IDLE | LOCKED | COASTING | SEARCHING | LOST
float32 heading_deg
float32 range_est_m
float32 lost_s
string last_outcome
