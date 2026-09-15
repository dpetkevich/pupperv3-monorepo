int32 track_id
string state           # TRACKED | COASTING | LOST
float32 heading_deg    # ROS sign: +left
float32 elevation_deg
float32 heading_rate_dps
float32[4] bbox        # x1 y1 x2 y2 in equirect pixels
float32 score
float32 age_s
int32 hits
float32 range_est_m
string name
float32 name_confidence
float32[] embedding    # only on /perception/tracks_raw
