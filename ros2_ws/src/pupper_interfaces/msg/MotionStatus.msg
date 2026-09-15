# /motion_server/status at 5 Hz
builtin_interfaces/Time stamp
string active_goal     # "" | TurnDegrees | MoveMeters | GoToPose | ReturnToStart | FollowPerson | FindPerson
bool estop_latched
bool controller_active
string controller_name
bool teleop_active
bool imu_ok
bool fallen
float32 x              # odom frame, metres
float32 y
float32 yaw_deg
float32 start_dx       # pose relative to the "start" mark (body frame of the mark)
float32 start_dy
float32 start_dyaw
float32 sigma_xy_m
float32 sigma_yaw_deg
