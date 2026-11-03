import math
import threading

import pytest

from sutura.drivers.joint_limits import NED1_JOINT_LIMITS, JointCommandError, validate_joint_command
from sutura.drivers.ned1_driver import Ned1ArmDriver


# Same class names as pyniryo 1.0.5 (pyniryo/api/exceptions.py). The driver matches them by name.
class HostNotReachableException(Exception):
    pass


class NiryoRobotException(Exception):
    pass


class FakeNiryoRobot:
    """Minimal stand-in for pyniryo.NiryoRobot (1.0.5 API, robot software 3.1.1)."""

    def __init__(self, needs_calibration=False, fail_connect=False, move_gate=None):
        self.joints = [0.0, 0.3, -1.3, 0.0, 0.0, 0.0]
        self.needs_calibration = needs_calibration
        self.fail_connect = fail_connect
        self.move_gate = move_gate
        self.raise_on = {}  # method name -> exception raised by that method
        self.calls = []

    def _maybe_raise(self, method):
        if method in self.raise_on:
            raise self.raise_on[method]

    def connect(self, ip):
        self.calls.append(('connect', ip))
        if self.fail_connect:
            raise ConnectionError("unreachable")

    def need_calibration(self):
        return self.needs_calibration

    def calibrate_auto(self):
        self.calls.append(('calibrate_auto',))
        self.needs_calibration = False

    def set_learning_mode(self, enabled):
        self.calls.append(('set_learning_mode', enabled))

    def set_arm_max_velocity(self, pct):
        self.calls.append(('set_arm_max_velocity', pct))

    def get_joints(self):
        self.calls.append(('get_joints',))
        self._maybe_raise('get_joints')
        return list(self.joints)

    def move_joints(self, joints):
        self.calls.append(('move_joints', list(joints)))
        self._maybe_raise('move_joints')
        if self.move_gate is not None:
            self.move_gate.wait(timeout=2.0)
        self.joints = list(joints)

    def move_to_home_pose(self):
        self.calls.append(('move_to_home_pose',))
        self._maybe_raise('move_to_home_pose')
        self.joints = [0.0, 0.3, -1.3, 0.0, 0.0, 0.0]

    def close_connection(self):
        self.calls.append(('close_connection',))


def make_driver(robot=None, **kwargs):
    robot = robot or FakeNiryoRobot()
    driver = Ned1ArmDriver(client_factory=lambda: robot, **kwargs)
    return driver, robot


# ----------------------------------------------------------------------
# Joint limit validation
# ----------------------------------------------------------------------
def test_validate_accepts_zero_pose():
    validate_joint_command([0.0] * 6, NED1_JOINT_LIMITS)


def test_default_margin_accepts_niryo_home_pose():
    # Ned1 rests at joint_3 = -1.3 rad, only 0.04 rad from its -1.34 rad limit.
    validate_joint_command([0.0, 0.3, -1.3, 0.0, 0.0, 0.0], NED1_JOINT_LIMITS,
                           safety_margin_rad=Ned1ArmDriver().safety_margin_rad)


def test_validate_rejects_out_of_range_and_margin():
    upper_j2 = NED1_JOINT_LIMITS[1].upper
    with pytest.raises(JointCommandError, match="joint_2"):
        validate_joint_command([0.0, upper_j2 + 0.01, 0, 0, 0, 0], NED1_JOINT_LIMITS)
    with pytest.raises(JointCommandError, match="joint_2"):
        validate_joint_command([0.0, upper_j2 - 0.01, 0, 0, 0, 0], NED1_JOINT_LIMITS, safety_margin_rad=0.05)


def test_validate_rejects_nan_wrong_length_and_large_step():
    with pytest.raises(JointCommandError):
        validate_joint_command([math.nan] + [0.0] * 5, NED1_JOINT_LIMITS)
    with pytest.raises(JointCommandError):
        validate_joint_command([0.0] * 5, NED1_JOINT_LIMITS)
    with pytest.raises(JointCommandError, match="step"):
        validate_joint_command([0.6] + [0.0] * 5, NED1_JOINT_LIMITS, current=[0.0] * 6, max_step_rad=0.5)


# ----------------------------------------------------------------------
# Connection
# ----------------------------------------------------------------------
def test_connect_configures_robot():
    driver, robot = make_driver(velocity_percent=15)
    assert driver.connect()
    assert ('set_learning_mode', False) in robot.calls
    assert ('set_arm_max_velocity', 15) in robot.calls
    assert driver.get_joint_positions() == robot.joints


def test_connect_calibrates_when_needed():
    driver, robot = make_driver(FakeNiryoRobot(needs_calibration=True))
    assert driver.connect()
    assert ('calibrate_auto',) in robot.calls


def test_connect_refuses_when_calibration_needed_and_disabled():
    driver, robot = make_driver(FakeNiryoRobot(needs_calibration=True), auto_calibrate=False)
    assert not driver.connect()
    assert ('calibrate_auto',) not in robot.calls
    assert not driver.is_connected


def test_connect_failure_is_reported():
    driver, _ = make_driver(FakeNiryoRobot(fail_connect=True))
    assert not driver.connect()
    assert "unreachable" in driver.last_error


def test_invalid_velocity_rejected():
    with pytest.raises(ValueError):
        Ned1ArmDriver(velocity_percent=0)


# ----------------------------------------------------------------------
# Motion
# ----------------------------------------------------------------------
def test_move_valid_command():
    driver, robot = make_driver()
    driver.connect()
    target = [0.1, 0.3, -1.3, 0.0, 0.0, 0.0]
    assert driver.set_joint_positions(target)
    assert robot.joints == target
    assert driver.get_joint_positions() == target


def test_move_rejected_when_not_connected():
    driver, robot = make_driver()
    assert not driver.set_joint_positions([0.0] * 6)
    assert not any(c[0] == 'move_joints' for c in robot.calls)


def test_move_rejected_outside_limits_does_not_reach_robot():
    driver, robot = make_driver(max_step_rad=None)
    driver.connect()
    assert not driver.set_joint_positions([3.5, 0.3, -1.3, 0.0, 0.0, 0.0])
    assert not any(c[0] == 'move_joints' for c in robot.calls)


def test_move_rejected_for_large_step():
    driver, robot = make_driver(max_step_rad=0.2)
    driver.connect()
    assert not driver.set_joint_positions([0.5, 0.3, -1.3, 0.0, 0.0, 0.0])
    assert "max_step_rad" in driver.last_error


def test_torques_are_nan():
    driver, _ = make_driver()
    assert all(math.isnan(t) for t in driver.get_joint_torques())


# ----------------------------------------------------------------------
# Concurrency and emergency stop
# ----------------------------------------------------------------------
def test_state_read_does_not_block_during_motion():
    gate = threading.Event()
    driver, robot = make_driver(FakeNiryoRobot(move_gate=gate))
    driver.connect()
    start = driver.get_joint_positions()
    target = [0.1, 0.3, -1.3, 0.0, 0.0, 0.0]

    worker = threading.Thread(target=driver.set_joint_positions, args=(target,))
    worker.start()
    for _ in range(1000):
        if driver.is_moving:
            break
        threading.Event().wait(0.002)
    assert driver.is_moving
    assert driver.get_joint_positions() == start  # cached value, no blocking
    gate.set()
    worker.join(timeout=2.0)
    assert not driver.is_moving
    assert driver.get_joint_positions() == target


def test_emergency_stop_latches_and_resets():
    driver, robot = make_driver()
    driver.connect()
    driver.emergency_stop()
    assert driver.estop_latched
    assert not driver.set_joint_positions([0.1, 0.3, -1.3, 0.0, 0.0, 0.0])
    assert not driver.move_to_home()
    assert ('set_learning_mode', True) not in robot.calls  # never drop the arm
    driver.reset_emergency_stop()
    assert driver.set_joint_positions([0.1, 0.3, -1.3, 0.0, 0.0, 0.0])


def test_disconnect_closes_connection():
    driver, robot = make_driver()
    driver.connect()
    driver.disconnect()
    assert ('close_connection',) in robot.calls
    assert not driver.is_connected


# ----------------------------------------------------------------------
# pyniryo 1.0.5 exception handling
# ----------------------------------------------------------------------
def test_default_ip_is_ethernet_address():
    assert Ned1ArmDriver().ip_address == "169.254.200.200"


def test_lost_link_during_motion_marks_driver_disconnected():
    driver, robot = make_driver()
    driver.connect()
    robot.raise_on['move_joints'] = HostNotReachableException("Unable to communicate with robot server")
    assert not driver.set_joint_positions([0.1, 0.3, -1.3, 0.0, 0.0, 0.0])
    assert not driver.is_connected
    assert "Unable to communicate" in driver.last_error
    assert not driver.is_moving


def test_lost_link_stops_polling_and_keeps_last_positions():
    driver, robot = make_driver()
    driver.connect()
    last = driver.get_joint_positions()
    robot.raise_on['get_joints'] = HostNotReachableException("down")
    assert driver.get_joint_positions() == last  # failed read returns the cache
    assert not driver.is_connected
    reads = robot.calls.count(('get_joints',))
    robot.raise_on.clear()  # the robot "comes back", but the driver must not poll it by itself
    assert driver.get_joint_positions() == last
    assert robot.calls.count(('get_joints',)) == reads  # no further reads are attempted


def test_os_error_is_a_lost_link():
    driver, robot = make_driver()
    driver.connect()
    robot.raise_on['get_joints'] = ConnectionResetError("reset by peer")
    driver.get_joint_positions()
    assert not driver.is_connected


def test_robot_refusal_keeps_the_link():
    # "Command KO" (NiryoRobotException) is the robot rejecting a command; the socket is fine.
    driver, robot = make_driver()
    driver.connect()
    robot.raise_on['move_joints'] = NiryoRobotException("Command KO : calibration needed")
    assert not driver.set_joint_positions([0.1, 0.3, -1.3, 0.0, 0.0, 0.0])
    assert driver.is_connected
    assert "Command KO" in driver.last_error
    del robot.raise_on['move_joints']
    assert driver.set_joint_positions([0.1, 0.3, -1.3, 0.0, 0.0, 0.0])


def test_home_motion_lost_link():
    driver, robot = make_driver()
    driver.connect()
    robot.raise_on['move_to_home_pose'] = HostNotReachableException("down")
    assert not driver.move_to_home()
    assert not driver.is_connected
