import math

from pupper_motion.yaw_tracker import YawTracker, YawTrackerConfig


def quat_yaw(yaw):
    return (0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2))


def test_constant_gyro_integrates_and_tracks_quaternion():
    tr = YawTracker()
    yaw = 0.3  # arbitrary absolute start -> reported yaw starts at 0
    wz = 1.0
    for i in range(200):  # 2 s at 100 Hz
        t = i * 0.01
        tr.update(t, quat_yaw(yaw + wz * t), wz)
    assert abs(tr.yaw - 2.0) < 0.02
    assert tr.jumps == 0


def test_duplicates_are_dropped():
    tr = YawTracker()
    q = quat_yaw(0.0)
    tr.update(0.0, q, 0.0)
    assert tr.update(0.004, q, 0.0) is False
    assert tr.update(0.01, quat_yaw(0.001), 0.0) is True
    assert tr.duplicates == 1


def test_magnetometer_jump_is_absorbed():
    tr = YawTracker()
    for i in range(100):
        tr.update(i * 0.01, quat_yaw(0.0), 0.0)
    # the fused quaternion suddenly re-converges 30 degrees away while the gyro says we are still
    for i in range(100, 300):
        tr.update(i * 0.01, quat_yaw(math.radians(30)), 0.0)
    assert tr.jumps == 1
    assert abs(tr.yaw) < math.radians(1.0)
    assert tr.sigma_deg(3.0) > 5.0
    assert tr.sigma_deg(20.0) < 2.0


def test_small_residual_is_fused():
    tr = YawTracker(YawTrackerConfig(alpha=0.02, dedupe=False))
    tr.update(0.0, quat_yaw(0.0), 0.0)
    # quaternion says 2 degrees, gyro says nothing: we should converge to 2 degrees over ~1 s
    for i in range(1, 300):
        tr.update(i * 0.01, quat_yaw(math.radians(2.0)), 0.0)
    assert abs(tr.yaw - math.radians(2.0)) < math.radians(0.2)


def test_gyro_only_mode():
    tr = YawTracker(YawTrackerConfig(alpha=0.0, dedupe=False))
    for i in range(100):
        tr.update(i * 0.01, quat_yaw(1.0), 0.5)  # quaternion static, gyro says 0.5 rad/s
    assert abs(tr.yaw - 0.5 * 0.99) < 0.01


def test_stale_and_prediction():
    tr = YawTracker()
    tr.update(0.0, quat_yaw(0.0), 0.0)
    tr.update(0.01, quat_yaw(0.0), 1.0)
    assert not tr.is_stale(0.1)
    assert tr.is_stale(0.5)
    assert abs(tr.yaw_predicted(0.05) - (tr.yaw + 0.05)) < 1e-9


def test_gap_snaps_to_quaternion():
    tr = YawTracker()
    tr.update(0.0, quat_yaw(0.0), 0.0)
    tr.update(0.01, quat_yaw(0.0), 0.0)
    # 0.4 s of samples lost while the robot turned 20 degrees; last gyro reading still says 0
    tr.update(0.41, quat_yaw(math.radians(20)), 0.0)
    assert abs(tr.yaw - math.radians(20)) < 1e-6
    assert tr.gaps == 1 and tr.jumps == 0
