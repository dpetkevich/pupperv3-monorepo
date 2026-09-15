import math

from pupper_motion.pose_integrator import PoseIntegrator, PoseIntegratorConfig


def run(pi, seconds, vx, vy, yaw, integrating=True, dt=0.02):
    for _ in range(int(seconds / dt)):
        pi.step(dt, vx, vy, yaw, integrating)


def test_forward_two_metres_with_lag():
    pi = PoseIntegrator(PoseIntegratorConfig(lag_tau=0.3))
    run(pi, 4.0, 0.5, 0.0, 0.0)   # 2.0 m commanded; lag removes ~0.15 m
    run(pi, 2.0, 0.0, 0.0, 0.0)   # coast to rest: lag gives it back
    assert abs(pi.x - 2.0) < 0.02
    assert abs(pi.y) < 1e-6
    assert pi.speed_est < 0.01


def test_calibration_scale_and_rotation():
    pi = PoseIntegrator(PoseIntegratorConfig(k_vx=0.8, lag_tau=0.01))
    run(pi, 2.0, 0.5, 0.0, math.pi / 2)  # heading north (+y)
    run(pi, 0.5, 0.0, 0.0, math.pi / 2)
    assert abs(pi.x) < 0.01
    assert abs(pi.y - 0.8) < 0.02


def test_not_integrating_freezes_pose():
    pi = PoseIntegrator()
    run(pi, 1.0, 0.5, 0.0, 0.0)
    x = pi.x
    run(pi, 2.0, 0.5, 0.0, 0.0, integrating=False)
    assert abs(pi.x - x) < 1e-9
    assert pi.speed_est < 0.01


def test_marks_relative_and_to_odom():
    pi = PoseIntegrator(PoseIntegratorConfig(lag_tau=0.01))
    pi.step(0.02, 0.0, 0.0, math.pi / 2, True)
    pi.mark("start")           # at origin, facing +y
    run(pi, 2.0, 0.5, 0.0, math.pi / 2)   # 1 m "forward" = +y in odom
    run(pi, 0.2, 0.0, 0.0, math.pi / 2)
    dx, dy, dyaw = pi.relative_to("start")
    assert abs(dx - 1.0) < 0.02 and abs(dy) < 0.02 and abs(dyaw) < 1e-9
    ox, oy, oyaw = pi.mark_to_odom("start", 0.0, 0.0, 0.0)
    assert abs(ox) < 1e-9 and abs(oy) < 1e-9 and abs(oyaw - math.pi / 2) < 1e-9
    assert pi.sigma_xy(0.0) > 0.3 and pi.sigma_xy(0.0) < 0.4
    pi.inflate(1.0)
    assert pi.sigma_xy(0.0) > 1.3
