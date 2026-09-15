#!/usr/bin/env bash
# Diagnose IMU stream stalls: plant alone, then plant + motion_server, then + a turn goal.
set +u
PIDS=(); cleanup(){ kill "${PIDS[@]}" 2>/dev/null; pkill -f "cmd_vel_mux_node|fake_plant|motion_server|plan_executor" 2>/dev/null; sleep 0.5; pkill -9 -f "cmd_vel_mux_node|fake_plant|motion_server|plan_executor" 2>/dev/null; }; trap cleanup EXIT
source /opt/ros/jazzy/setup.bash; source /ws/install/setup.bash; export ROS_LOCALHOST_ONLY=1
LOG=/ws/log/simimu; rm -rf $LOG; mkdir -p $LOG
ros2 run pupper_motion fake_plant --ros-args -p start_active:=true > $LOG/plant.log 2>&1 & PIDS+=($!)
sleep 2
echo "=== plant alone: imu hz (5 s) ==="; timeout 6 ros2 topic hz /imu_sensor_broadcaster/imu -w 500 2>/dev/null | grep -E "average rate|min:" | tail -2
ros2 run cmd_vel_mux cmd_vel_mux_node --ros-args -p 'inputs:=["/teleop_cmd_vel","/reflex_cmd_vel","/motion_cmd_vel","/llm_cmd_vel"]' > $LOG/mux.log 2>&1 & PIDS+=($!)
ros2 run pupper_motion motion_server --ros-args -p activation_settle_s:=0.5 > $LOG/motion.log 2>&1 & PIDS+=($!)
sleep 3
echo "=== plant + mux + motion_server idle: imu hz (5 s) ==="; timeout 6 ros2 topic hz /imu_sensor_broadcaster/imu -w 500 2>/dev/null | grep -E "average rate|min:" | tail -2
echo "=== during a 180 turn ==="
(timeout 8 ros2 topic hz /imu_sensor_broadcaster/imu -w 500 2>/dev/null | grep -E "average rate|min:" | tail -2) &
nice -n 19 timeout 40 ros2 action send_goal /motion_server/turn_degrees pupper_interfaces/action/TurnDegrees "{degrees: 180.0, speed_dps: 0.0}" 2>/dev/null | grep -E "turned|error_degrees"
wait %1 2>/dev/null; sleep 1
echo "=== motion_server stall warnings ==="; grep -c "control loop stalled" $LOG/motion.log; grep "control loop stalled\|finished" $LOG/motion.log | tail -5
echo "=== plant 1 Hz log stamp deltas > 1.2 s (publisher stalls) ==="; grep -oE "\[[0-9]+\.[0-9]+\]" $LOG/plant.log | tr -d '[]' | awk 'NR>1{d=$1-p; if (d>1.2) printf "gap %.2fs\n", d} {p=$1}' | head
