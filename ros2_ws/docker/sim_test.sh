#!/usr/bin/env bash
# End-to-end check of motion_server + plan_executor against the fake plant and the real cmd_vel_mux.
# Runs inside the dev container:  ros2_ws/docker/dev.sh ros2_ws/docker/sim_test.sh
set +u
PIDS=()
cleanup(){ kill "${PIDS[@]}" 2>/dev/null; pkill -f "cmd_vel_mux_node|fake_plant|motion_server|plan_executor" 2>/dev/null; sleep 0.5; pkill -9 -f "cmd_vel_mux_node|fake_plant|motion_server|plan_executor" 2>/dev/null; }
trap cleanup EXIT
source /opt/ros/jazzy/setup.bash
source /ws/install/setup.bash
export ROS_LOCALHOST_ONLY=1
LOG=/ws/log/simtest; rm -rf $LOG; mkdir -p $LOG
ros2 run cmd_vel_mux cmd_vel_mux_node --ros-args -p 'inputs:=["/teleop_cmd_vel","/reflex_cmd_vel","/motion_cmd_vel","/llm_cmd_vel"]' -p timeout_ms:=500 > $LOG/mux.log 2>&1 &
PIDS+=($!)
ros2 run pupper_motion fake_plant > $LOG/plant.log 2>&1 &
PIDS+=($!)
ros2 run pupper_motion motion_server --ros-args -p activation_settle_s:=0.5 > $LOG/motion.log 2>&1 &
PIDS+=($!)
ros2 run pupper_planner plan_executor --ros-args -p google_api_key_file:=/nonexistent > $LOG/executor.log 2>&1 &
PIDS+=($!)
sleep 4
echo "nproc=$(nproc)"; echo "=== nodes ==="; ros2 node list
echo "=== TurnDegrees 90 ==="
nice -n 19 timeout 40 ros2 action send_goal /motion_server/turn_degrees pupper_interfaces/action/TurnDegrees "{degrees: 90.0, speed_dps: 0.0}" | grep -E "Goal accepted|Result|success|reason|turned|error_degrees|Status"
echo "=== MoveMeters 1.0 ==="
nice -n 19 timeout 40 ros2 action send_goal /motion_server/move_meters pupper_interfaces/action/MoveMeters "{dx: 1.0, dy: 0.0, speed_mps: 0.0, hold_heading: true}" | grep -E "Goal accepted|success|reason|moved_m|sigma_m|heading_drift|Status"
echo "=== TurnDegrees 1080 (expect rejected) ==="
nice -n 19 timeout 10 ros2 action send_goal /motion_server/turn_degrees pupper_interfaces/action/TurnDegrees "{degrees: 1080.0, speed_dps: 0.0}" | grep -E "rejected|accepted"
echo "=== ExecutePlan: turn 90, move 1, turn -90, move 1, return_to_start, say ==="
PLAN='{"v":1,"goal":"square-ish","mode":"replace","source":"cli","on_fail":"report","steps":[{"id":"s1","skill":"turn","args":{"degrees":90}},{"id":"s2","skill":"move","args":{"meters":1.0}},{"id":"s3","skill":"turn","args":{"degrees":-90}},{"id":"s4","skill":"move","args":{"meters":1.0}},{"id":"s5","skill":"return_to_start","args":{"restore_heading":true}},{"id":"s6","skill":"say","args":{"text":"Back where I started."}}]}'
nice -n 19 timeout 180 ros2 action send_goal /plan_executor/execute_plan pupper_interfaces/action/ExecutePlan "{plan_json: '$PLAN'}" | grep -E "Goal accepted|outcome|failed_step|reason|summary|replan_allowed|Status"
echo "=== ExecutePlan: invalid (expect REJECTED result) ==="
BAD='{"v":1,"steps":[{"id":"s1","skill":"spin","args":{"degrees":90}}]}'
nice -n 19 timeout 20 ros2 action send_goal /plan_executor/execute_plan pupper_interfaces/action/ExecutePlan "{plan_json: '$BAD'}" | grep -E "outcome|reason"
echo "=== final motion status ==="
timeout 5 ros2 topic echo --once /motion_server/status | grep -E "x:|y:|yaw_deg|start_d|sigma|estop|controller_active|active_goal"
echo "=== executor state ==="
timeout 5 ros2 topic echo --once /plan_executor/state | grep -E "status|last_result|pose_summary"
echo "=== /cmd_vel publishers (expect only the mux) ==="
ros2 topic info -v /cmd_vel | grep -E "Publisher count|Node name"
echo "=== LAT lines ==="; grep -h "LAT evt" $LOG/motion.log $LOG/executor.log | head -20
echo "=== plant timeline ==="; grep -E "yaw=|switch" $LOG/plant.log | head -60
echo "=== mux switching ==="; grep -E "Active source|activated|timed out" $LOG/mux.log | head -30
echo "=== errors in logs ==="; grep -iE "error|traceback|exception" $LOG/*.log | grep -v "no error" | head -20
