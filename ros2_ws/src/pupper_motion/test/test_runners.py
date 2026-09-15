"""Runners stepped against a toy plant: yaw follows wz through a first-order lag; position comes from the
same PoseIntegrator the server uses (so the lag model is consistent)."""
import math

from pupper_motion.limits import Envelope, Ramp
from pupper_motion.pose_integrator import PoseIntegrator, PoseIntegratorConfig
from pupper_motion.runners import GoToPoseRunner, MoveRunner, RobotState, RunnerConfig, TurnRunner


class Plant:
    def __init__(self, k_vx=1.0, yaw_lag=0.15):
        self.t = 0.0
        self.yaw = 0.0
        self.wz = 0.0
        self.yaw_lag = yaw_lag
        self.pi = PoseIntegrator(PoseIntegratorConfig(k_vx=k_vx))
        self.ramp = Ramp()
        self.env = Envelope()
        self.cmds = []

    def state(self):
        return RobotState(self.t, self.yaw, self.wz, self.pi.x, self.pi.y, self.pi.speed_est)

    def run(self, runner, max_s=60.0, dt=0.02, freeze_gyro=False):
        while not runner.done and self.t < max_s:
            cmd = self.env.clamp(self.ramp.apply(runner.step(self.state(), dt), dt))
            self.cmds.append(cmd)
            if not freeze_gyro:
                self.wz += (cmd.wz - self.wz) * min(1.0, dt / self.yaw_lag)
            self.yaw += self.wz * dt
            self.pi.step(dt, cmd.vx, cmd.vy, self.yaw, True)
            self.t += dt
        return runner.result


def test_turn_90_left_hits_tolerance():
    p = Plant()
    r = p.run(TurnRunner(RunnerConfig(), 90.0))
    assert r.success, r
    assert abs(math.degrees(p.yaw) - 90.0) < 3.0
    assert abs(r.data["turned_degrees"] - 90.0) < 3.0
    assert all(c.wz >= 0.0 for c in p.cmds[:10])  # left = positive wz


def test_turn_minus_180_and_360():
    for deg in (-180.0, 360.0):
        p = Plant()
        r = p.run(TurnRunner(RunnerConfig(), deg))
        assert r.success, (deg, r)
        assert abs(r.data["turned_degrees"] - deg) < 3.0


def test_turn_stalls_when_gyro_silent():
    p = Plant()
    r = p.run(TurnRunner(RunnerConfig(), 90.0), freeze_gyro=True)
    assert not r.success and r.reason == "STALLED"


def test_move_two_metres_dead_reckoned():
    p = Plant()
    r = p.run(MoveRunner(RunnerConfig(), 2.0, 0.0))
    assert r.success, r
    assert abs(r.data["moved_m"] - 2.0) < 0.1
    assert abs(p.pi.x - 2.0) < 0.1
    assert r.data["sigma_m"] > 0.5
    assert p.cmds[-1].is_zero()


def test_move_backward_and_strafe_left():
    p = Plant()
    r = p.run(MoveRunner(RunnerConfig(), -1.0, 0.0))
    assert r.success and abs(p.pi.x + 1.0) < 0.1
    p = Plant()
    r = p.run(MoveRunner(RunnerConfig(), 0.0, 1.0))
    assert r.success and abs(p.pi.y - 1.0) < 0.1 and abs(p.pi.x) < 0.05
    assert any(c.vy >= 0.4 for c in p.cmds)  # lateral floor respected, left = +vy


def test_move_speed_floor():
    p = Plant()
    p.run(MoveRunner(RunnerConfig(), 1.0, 0.0, speed_mps=0.1))
    moving = [c.vx for c in p.cmds if c.vx > 0]
    assert min(moving[20:]) >= 0.35 - 1e-9


def test_goto_pose_turn_move_and_return():
    p = Plant()
    r = p.run(GoToPoseRunner(RunnerConfig(), 1.0, 1.0, math.radians(0.0), final_turn=True))
    assert r.success, r
    assert math.hypot(p.pi.x - 1.0, p.pi.y - 1.0) < 0.2
    assert abs(math.degrees(p.yaw)) < 6.0
    # now go back to the origin
    r2 = p.run(GoToPoseRunner(RunnerConfig(), 0.0, 0.0, None, final_turn=False))
    assert r2.success and math.hypot(p.pi.x, p.pi.y) < 0.25


def test_goto_pose_already_there_only_turns():
    p = Plant()
    r = p.run(GoToPoseRunner(RunnerConfig(), 0.05, 0.0, math.radians(90), final_turn=True))
    assert r.success and abs(math.degrees(p.yaw) - 90.0) < 3.0
    assert all(c.vx == 0.0 for c in p.cmds)
