# Pupper brain — motion server, plan executor, voice agent

Rewrite of the command path (Sept 2026). Plan: `~/.claude/plans/purring-twirling-charm.md`.

```
voice ─► LiveKit console + gpt-realtime ─► tools: execute_plan | stop | status | ask_scene | animate | power | follow_me
                                              ▲ speaks only from /plan_executor/{state,events}
plan_executor (ros2_ws/src/pupper_planner)   ExecutePlan action, Look service, /plan_executor/{stop,prepare}
motion_server (ros2_ws/src/pupper_motion)    TurnDegrees · MoveMeters · GoToPose · ReturnToStart, /motion_server/{stop,prepare,clear_estop,mark_pose}
cmd_vel_mux: /teleop_cmd_vel > /reflex_cmd_vel > /motion_cmd_vel > /llm_cmd_vel > /person_following_cmd_vel  ─► /cmd_vel ─► neural_controller
```

Packages: `pupper_interfaces` (contract: actions/srvs/msgs + `schema/plan_v1.json`), `pupper_motion`, `pupper_vision` (equirect + Gemini look), `pupper_planner`, and the slimmed agent in `ai/llm-ui/agent-starter-python`.

## Develop off-robot

```bash
# build container (once)
docker build -t pupper-ros-dev -f ros2_ws/docker/Dockerfile.dev ros2_ws/docker
# build the brain packages + the mux
ros2_ws/docker/dev.sh colcon build --packages-select pupper_interfaces pupper_motion pupper_vision pupper_planner cmd_vel_mux
# end-to-end sim: real mux + fake plant + motion_server + plan_executor, action goals and a compound plan
ros2_ws/docker/dev.sh bash /ws/docker/sim_test.sh
# one turn, tracker vs truth
ros2_ws/docker/dev.sh bash /ws/docker/sim_turn.sh 90
# pure-python unit tests (no ROS)
.venv-pupper/bin/python -m pytest ros2_ws/src/pupper_motion/test ros2_ws/src/pupper_planner/test ros2_ws/src/pupper_vision/test
.venv-pupper/bin/python -m pytest ai/llm-ui/agent-starter-python/tests --ignore=ai/llm-ui/agent-starter-python/tests/test_agent.py
# iterate on the prompt without a robot
cd ai/llm-ui/agent-starter-python && python3 src/agent.py --tool-server nop console
```

## Deploy to the Pi

```bash
# from the laptop, on branch pupper-brain
rsync -av --delete --exclude '.env.local' --exclude '.venv' --exclude '__pycache__' \
  ros2_ws/src/pupper_interfaces ros2_ws/src/pupper_motion ros2_ws/src/pupper_vision ros2_ws/src/pupper_planner \
  pi@pupper.local:/home/pi/pupperv3-monorepo/ros2_ws/src/
rsync -av ros2_ws/src/neural_controller/launch/config.yaml ros2_ws/src/neural_controller/launch/launch.py \
  pi@pupper.local:/home/pi/pupperv3-monorepo/ros2_ws/src/neural_controller/launch/
rsync -av --delete --exclude '.env.local' --exclude '.venv' --exclude '__pycache__' \
  ai/llm-ui/agent-starter-python/ pi@pupper.local:/home/pi/pupperv3-monorepo/ai/llm-ui/agent-starter-python/

# on the Pi
sudo pip3 install --break-system-packages jsonschema google-genai     # if missing
cd ~/pupperv3-monorepo/ros2_ws && source /opt/ros/jazzy/setup.bash && \
  colcon build --symlink-install --packages-select pupper_interfaces pupper_motion pupper_vision pupper_planner cmd_vel_mux
sudo systemctl restart robot          # whatever unit runs launch.py
sudo systemctl restart llm-agent
```

## First checks on the robot (robot on its side or held, then on the floor)

```bash
source /opt/ros/jazzy/setup.bash && source ~/pupperv3-monorepo/ros2_ws/install/setup.bash
ros2 topic info -v /cmd_vel                       # exactly one publisher: cmd_vel_mux
ros2 topic echo --once /motion_server/status      # imu_ok true, controller_active, estop false
ros2 topic hz /imu_sensor_broadcaster/imu
# M1 (floor, joystick idle):
ros2 action send_goal /motion_server/turn_degrees pupper_interfaces/action/TurnDegrees "{degrees: 90.0, speed_dps: 0.0}"
# M2 after calibration:
ros2 run pupper_motion calibrate_k --quick        # then --write ros2_ws/src/neural_controller/launch/config.yaml
ros2 action send_goal /motion_server/move_meters pupper_interfaces/action/MoveMeters "{dx: 2.0, dy: 0.0, speed_mps: 0.0, hold_heading: true}"
# E1:
ros2 action send_goal /plan_executor/execute_plan pupper_interfaces/action/ExecutePlan \
  "{plan_json: '{\"v\":1,\"source\":\"cli\",\"steps\":[{\"id\":\"s1\",\"skill\":\"turn\",\"args\":{\"degrees\":90}},{\"id\":\"s2\",\"skill\":\"move\",\"args\":{\"meters\":1.0}},{\"id\":\"s3\",\"skill\":\"return_to_start\",\"args\":{\"restore_heading\":true}}]}'}"
# watch what the agent will say:
ros2 topic echo /plan_executor/events
journalctl -u llm-agent -f | grep -E "LAT evt|FUNCTION CALL|plan result"
```

Safety: the PS5 e-stop and teleop always win (mux priority); the motion server latches `/emergency_stop` and falls (IMU tilt > 60°) until the walking controller is observed inactive→active (joystick release) or `/motion_server/clear_estop` is called; every goal end publishes a zero then goes silent after 0.3 s.

## Tuning knobs (config.yaml → motion_server / plan_executor)

`k_vx k_vy lag_tau sigma_frac` (dead-reckoning, from `calibrate_k`), `yaw_alpha` (0.02 fused / 0.0 gyro-only), `turn_speed_dps`, `move_speed`, `vx_floor`, `activation_settle_s`; executor: `look_settle_s`, `look_max_age_s`, `equirect_*`, `camera_height_m`, `google_api_key_file`.
The turn coast (`turn_stop_lag_s`) is learned from the first real turn and kept for the life of the node.

## Phase B / C

Not started: person tracking with identity (`pupper_interfaces` already carries `FollowPerson`, `FindPerson`, `PersonTrack*`, `EnrollPerson`), and the on-Pi keyword reflex tier (`/reflex_cmd_vel` is already in the mux). See the plan.
