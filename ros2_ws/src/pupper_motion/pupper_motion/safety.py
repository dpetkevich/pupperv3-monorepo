"""Safety monitor: the checks every velocity publisher on Pupper must make (see the plan's non-negotiables).

- /emergency_stop is latching. It clears only when the walking controller is observed going inactive -> active
  (the joystick release button does that; our own auto-activation is gated on the latch so it cannot clear it).
- Falls are detected here from IMU tilt because the policy's body-angle e-stop is internal and never publishes.
- Any higher-priority cmd_vel_mux source (teleop, reflex) active means we must yield: there is no joystick deadman.
"""
from dataclasses import dataclass, field
from typing import Dict, List, Optional


@dataclass
class SafetyMonitor:
    our_source: str = "/motion_cmd_vel"
    higher_priority_sources: List[str] = field(default_factory=lambda: ["/teleop_cmd_vel", "/reflex_cmd_vel"])
    walking_controller: str = "neural_controller"
    fall_tilt_deg: float = 60.0

    estop_latched: bool = False
    fallen: bool = False
    controller_active: bool = False
    controller_known: bool = False
    active_source: str = "none"
    imu_ok: bool = False
    _seen_inactive_since_latch: bool = False
    takeover_events: int = 0

    # ------------------------------------------------------------------ inputs
    def on_estop(self) -> None:
        self.estop_latched = True
        self._seen_inactive_since_latch = False

    def clear_latches(self) -> None:
        self.estop_latched = False
        self.fallen = False
        self._seen_inactive_since_latch = False

    def on_controller_states(self, states: Dict[str, str]) -> None:
        self.controller_known = True
        active = states.get(self.walking_controller) == "active"
        latched = self.estop_latched or self.fallen
        if latched:
            if not active:
                self._seen_inactive_since_latch = True
            elif self._seen_inactive_since_latch:
                # inactive -> active transition while latched: a human re-activated the controller
                self.clear_latches()
        self.controller_active = active

    def on_active_source(self, topic: str) -> None:
        was_ours_or_none = self.active_source in (self.our_source, "none", "")
        self.active_source = topic
        if topic in self.higher_priority_sources and was_ours_or_none:
            self.takeover_events += 1

    def on_imu(self, tilt_deg: float, ok: bool) -> None:
        self.imu_ok = ok
        if ok and tilt_deg > self.fall_tilt_deg:
            if not self.fallen:
                self._seen_inactive_since_latch = False
            self.fallen = True

    # ------------------------------------------------------------------ queries
    @property
    def teleop_active(self) -> bool:
        return self.active_source in self.higher_priority_sources

    def can_auto_activate(self) -> bool:
        return not (self.estop_latched or self.fallen)

    def check(self, require_controller: bool = True, require_imu: bool = True) -> Optional[str]:
        """Return an abort reason, or None if it is safe to keep publishing velocity."""
        if self.estop_latched:
            return "ESTOP"
        if self.fallen:
            return "FALLEN"
        if self.teleop_active:
            return "TELEOP_TAKEOVER"
        if require_controller and self.controller_known and not self.controller_active:
            return "CONTROLLER_INACTIVE"
        if require_imu and not self.imu_ok:
            return "IMU_STALE"
        return None
