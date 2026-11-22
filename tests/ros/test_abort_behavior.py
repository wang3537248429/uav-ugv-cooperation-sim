"""Abort behavior with real ROS message classes and a minimal node harness."""

from std_msgs.msg import Bool
from ap_mission.uav_mission_state_machine import UAVMissionStateMachine


class _FakePublisher:
    def __init__(self):
        self.messages = []

    def publish(self, msg):
        self.messages.append(msg)


class _FakeLogger:
    def __init__(self):
        self.infos = []
        self.errors = []

    def info(self, msg):
        self.infos.append(msg)

    def error(self, msg):
        self.errors.append(msg)


class _AbortHarness:
    def __init__(self, *, elapsed, armed, rc_landing_active=False):
        self._elapsed_value = elapsed
        self.is_armed = armed
        self.current_mode = "STABILIZE"
        self.current_alt = 0.0
        self.hover_pwm = 1550
        self.mission_started = False
        self.mission_start_time = None
        self.landing_strategy = "LAND"
        self._ekf_abort_triggered = True
        self.strike_perm_pub = _FakePublisher()
        self.land_pub = _FakePublisher()
        self.arm_pub = _FakePublisher()
        self.mode_pub = _FakePublisher()
        self.logger = _FakeLogger()
        self.transition_calls = []
        self.rc_commands = []
        self.strike_permission_calls = []
        self._rc_landing_active_value = rc_landing_active

    def _elapsed(self):
        return self._elapsed_value

    def _publish_strike_permission(self, allowed):
        self.strike_permission_calls.append(allowed)

    def _publish_rc(self, roll=1500, pitch=1500, throttle=None, yaw=1500):
        self.rc_commands.append((roll, pitch, throttle, yaw))

    def _rc_landing_active(self):
        return self._rc_landing_active_value

    def _landing_descent_throttle(self):
        return 1400

    def transition(self, state):
        self.transition_calls.append(state)

    def get_logger(self):
        return self.logger


def test_abort_while_disarmed_does_not_command_land():
    harness = _AbortHarness(elapsed=0.5, armed=False)

    UAVMissionStateMachine._handle_abort(harness)

    assert harness.land_pub.messages == []
    assert harness.arm_pub.messages == []
    assert harness.transition_calls == []


def test_abort_while_armed_still_commands_land():
    harness = _AbortHarness(elapsed=0.5, armed=True)

    UAVMissionStateMachine._handle_abort(harness)

    assert len(harness.land_pub.messages) == 1
    assert isinstance(harness.land_pub.messages[0], Bool)
    assert harness.land_pub.messages[0].data is True
