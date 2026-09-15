from pupper_motion.limits import Cmd, Envelope, Ramp


def test_envelope_clamps():
    c = Envelope().clamp(Cmd(2.0, -1.0, 5.0))
    assert (c.vx, c.vy, c.wz) == (0.75, -0.5, 2.0)


def test_ramp_limits_increase_but_not_decrease():
    r = Ramp(accel_lin=1.0, accel_ang=4.0)
    c = r.apply(Cmd(0.5, 0.0, 1.0), 0.02)
    assert abs(c.vx - 0.02) < 1e-9 and abs(c.wz - 0.08) < 1e-9
    for _ in range(100):
        c = r.apply(Cmd(0.5, 0.0, 1.0), 0.02)
    assert abs(c.vx - 0.5) < 1e-9 and abs(c.wz - 1.0) < 1e-9
    c = r.apply(Cmd(), 0.02)
    assert c.is_zero()


def test_ramp_reversal_passes_through_zero():
    r = Ramp()
    for _ in range(100):
        r.apply(Cmd(0.5, 0, 0), 0.02)
    c = r.apply(Cmd(-0.5, 0, 0), 0.02)
    assert c.vx == 0.0
    c = r.apply(Cmd(-0.5, 0, 0), 0.02)
    assert c.vx < 0.0
