from pupper_motion.safety import SafetyMonitor


def ready():
    s = SafetyMonitor()
    s.on_controller_states({"neural_controller": "active"})
    s.on_imu(2.0, True)
    s.on_active_source("/motion_cmd_vel")
    return s


def test_ready_is_clear():
    assert ready().check() is None


def test_estop_latches_until_human_reactivates():
    s = ready()
    s.on_estop()
    assert s.check() == "ESTOP" and not s.can_auto_activate()
    s.on_controller_states({"neural_controller": "active"})  # still active (poll raced the deactivate)
    assert s.check() == "ESTOP"
    s.on_controller_states({"neural_controller": "inactive"})
    assert s.check() == "ESTOP"
    s.on_controller_states({"neural_controller": "active"})  # joystick release re-activated it
    assert s.check() is None


def test_fall_from_tilt():
    s = ready()
    s.on_imu(75.0, True)
    assert s.check() == "FALLEN"
    s.on_imu(1.0, True)
    assert s.check() == "FALLEN"  # latched even after it is set upright
    s.on_controller_states({"neural_controller": "inactive"})
    s.on_controller_states({"neural_controller": "active"})
    assert s.check() is None


def test_teleop_and_reflex_take_priority():
    s = ready()
    s.on_active_source("/teleop_cmd_vel")
    assert s.check() == "TELEOP_TAKEOVER" and s.takeover_events == 1
    s.on_active_source("/reflex_cmd_vel")
    assert s.check() == "TELEOP_TAKEOVER" and s.takeover_events == 1
    s.on_active_source("none")
    assert s.check() is None


def test_controller_inactive_and_imu_stale():
    s = ready()
    s.on_controller_states({"neural_controller": "inactive"})
    assert s.check() == "CONTROLLER_INACTIVE"
    assert s.check(require_controller=False) is None
    s.on_controller_states({"neural_controller": "active"})
    s.on_imu(0.0, False)
    assert s.check() == "IMU_STALE"
