#!/usr/bin/env bash
# One TurnDegrees goal against the fake plant; prints tracker vs plant truth. Args: degrees [extra motion_server params]
set +u
PIDS=()
cleanup(){ kill "${PIDS[@]}" 2>/dev/null; pkill -f "cmd_vel_mux_node|fake_plant|motion_server|plan_executor" 2>/dev/null; sleep 0.5; pkill -9 -f "cmd_vel_mux_node|fake_plant|motion_server|plan_executor" 2>/dev/null; }
trap cleanup EXIT
source /opt/ros/jazzy/setup.bash; source /ws/install/setup.bash
export ROS_LOCALHOST_ONLY=1
DEG=${1:-90}; shift
LOG=/ws/log/simturn; rm -rf $LOG; mkdir -p $LOG
ros2 run cmd_vel_mux cmd_vel_mux_node --ros-args -p 'inputs:=["/teleop_cmd_vel","/reflex_cmd_vel","/motion_cmd_vel","/llm_cmd_vel"]' -p timeout_ms:=500 > $LOG/mux.log 2>&1 &
PIDS+=($!)
ros2 run pupper_motion fake_plant --ros-args -p start_active:=true > $LOG/plant.log 2>&1 &
PIDS+=($!)
ros2 run pupper_motion motion_server --ros-args -p activation_settle_s:=0.5 "$@" > $LOG/motion.log 2>&1 &
PIDS+=($!)
sleep 3
(timeout 12 ros2 topic hz /motion_cmd_vel -w 200 2>/dev/null | grep -E "average rate|min:" | tail -2 | sed "s/^/motion_cmd_vel /") &
HZPID=$!
T0=$(date +%s.%N)
nice -n 19 timeout 40 ros2 action send_goal /motion_server/turn_degrees pupper_interfaces/action/TurnDegrees "{degrees: $DEG, speed_dps: 0.0}" 2>/dev/null | grep -E "success|turned|error_degrees"
T1=$(date +%s.%N)
echo "goal wall time: $(python3 -c "print(round($T1-$T0,2))") s"
wait $HZPID 2>/dev/null
sleep 1.5
echo "plant truth: $(grep -E 'yaw=' $LOG/plant.log | tail -1)"
grep "finished" $LOG/motion.log; echo "stall warnings: $(grep -c "control loop stalled" $LOG/motion.log)"; grep "control loop stalled" $LOG/motion.log | head -3
grep -E "yaw=" $LOG/plant.log | awk '{print $NF, $(NF-2), $(NF-1)}' | tr '\n' ';' | head -c 600; echo
