# pupper_motion

Motion server: the only publisher on `/motion_cmd_vel`. 50 Hz loop → safety check → one goal runner → ramp → clamp → publish; 0.3 s zero-hold after any goal, then silence.

- `yaw_tracker.py` — IMU heading: gyro-integrated, complementary-fused with the BNO08x rotation vector, magnetometer jumps absorbed (logged as warnings). `yaw_alpha: 0.0` = gyro only.
- `pose_integrator.py` — dead reckoning: commanded velocity through a first-order lag × `k_vx/k_vy`, rotated by yaw; honest `sigma_xy`; named marks (`start`).
- `runners.py` — `TurnRunner` (P-control with floor, predictive stop, learned coast, correction passes), `MoveRunner` (fixed body direction, lag-aware stop), `GoToPoseRunner` (turn → move → re-bear → final turn).
- `safety.py` — e-stop latch, fall latch (tilt > 60°), teleop/reflex takeover, controller state, IMU staleness.
- `motion_server.py` — the node; actions `TurnDegrees MoveMeters GoToPose ReturnToStart`; services `stop prepare clear_estop mark_pose`; topics `status odom`.
- `calibrate_k.py` — tape-measure calibration CLI. `fake_plant.py` — simulated IMU + controller_manager for off-robot testing.

Tests: `pytest test` (no ROS needed). Sim: `ros2_ws/docker/sim_test.sh`.

## Threading model (measured in the dev container, rclpy Jazzy)

| entity on the executor | CPU |
|---|---|
| one 100 Hz IMU subscription, `MultiThreadedExecutor(4)` | ~100 % of a core (busy wait-set rebuild per message; starves everything) |
| one 260 Hz IMU subscription, `MultiThreadedExecutor(4)` | ~130 % |
| one 260 Hz IMU subscription, `SingleThreadedExecutor` | ~25 % |
| timers / idle action servers, either executor | < 10 % |

Hence: IMU / e-stop / mux-source subscriptions on the companion node `motion_server_imu` (single-threaded executor, own thread); the 50 Hz control loop on a plain thread with sleep pacing; the multi-threaded executor only for actions, services and low-rate timers. `ros2_ws/docker/exec_probe.py` reproduces the measurement.
