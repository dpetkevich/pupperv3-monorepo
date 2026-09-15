#!/usr/bin/env bash
set +u
PIDS=(); cleanup(){ kill "${PIDS[@]}" 2>/dev/null; pkill -f "cmd_vel_mux_node|fake_plant|motion_server|plan_executor" 2>/dev/null; sleep 0.5; pkill -9 -f "cmd_vel_mux_node|fake_plant|motion_server|plan_executor" 2>/dev/null; }; trap cleanup EXIT
source /opt/ros/jazzy/setup.bash; source /ws/install/setup.bash; export ROS_LOCALHOST_ONLY=1
pip3 install -q --break-system-packages py-spy 2>&1 | tail -1
LOG=/ws/log/simprof; rm -rf $LOG; mkdir -p $LOG
ros2 run cmd_vel_mux cmd_vel_mux_node --ros-args -p 'inputs:=["/teleop_cmd_vel","/reflex_cmd_vel","/motion_cmd_vel","/llm_cmd_vel"]' > $LOG/mux.log 2>&1 & PIDS+=($!)
ros2 run pupper_motion fake_plant --ros-args -p start_active:=true > $LOG/plant.log 2>&1 & PIDS+=($!)
ros2 run pupper_motion motion_server --ros-args -p activation_settle_s:=0.5 > $LOG/motion.log 2>&1 & PIDS+=($!)
sleep 4
MS=$(pgrep -f "pupper_motion/motion_server" | head -1); echo "motion_server pid $MS"
echo "=== idle: top ==="; top -b -n 1 | head -12 | tail -6
echo "=== idle: py-spy dump ==="; py-spy dump --pid $MS 2>&1 | grep -E "^Thread|GIL|active|\(.*\.py:[0-9]+\)" | head -40
nice -n 19 timeout 40 ros2 action send_goal /motion_server/turn_degrees pupper_interfaces/action/TurnDegrees "{degrees: 180.0, speed_dps: 0.0}" > $LOG/goal.log 2>&1 &
sleep 2.5
echo "=== during turn: top ==="; top -b -n 1 | head -12 | tail -6
echo "=== during turn: py-spy dump #1 ==="; py-spy dump --pid $MS 2>&1 | grep -E "^Thread|\(.*\.py:[0-9]+\)" | head -60
sleep 1
echo "=== during turn: py-spy top 3s ==="; timeout 4 py-spy record --pid $MS --duration 3 --format raw -o $LOG/prof.txt >/dev/null 2>&1; sort -t' ' -k2 -nr $LOG/prof.txt 2>/dev/null | head -5
py-spy top --pid $MS --duration 3 2>/dev/null | head -25 || true
wait
grep -E "success|turned|error_deg" $LOG/goal.log; grep -c "stalled" $LOG/motion.log
