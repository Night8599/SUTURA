"""
Driver for the Niryo Ned (Ned1) robotic arm.

Communication goes through the Niryo TCP server (port 40001) using `pyniryo`.
The team's Ned1 runs robot software 3.1.1, which is supported by pyniryo 1.0.5
(tag v1.0.5 of github.com/NiryoRobotics/pyniryo). Its requirements.txt pins `enum34`
and `opencv-python==4.2.0.32`, so it must be installed without dependencies:

    pip install --no-deps "pyniryo==1.0.5"
    pip install numpy opencv-python

See docs/drivers/ned1.md.

Hardware constraints this driver has to live with:
  * Motion commands are blocking: `move_joints` returns only when the motion ends.
  * The TCP server accepts a single client and a single command at a time, so every
    call is serialized through a lock. While a motion runs, state queries cannot reach
    the robot and the last known joint positions are returned instead.
  * The TCP API has no "stop motion" command. `emergency_stop` therefore latches the
    driver (all further commands are refused) but cannot interrupt a motion already in
    progress. A physical power cut must be within reach during real-world tests.
  * Joints 1-3 are Niryo steppers and do not report torque. `get_joint_torques` returns
    NaN for every joint so that callers cannot mistake it for a real measurement.
"""

import logging
import math
import threading
from typing import Callable

from sutura.drivers.base_driver import BaseArmDriver
from sutura.drivers.joint_limits import (
    NED1_JOINT_LIMITS,
    JointCommandError,
    JointLimit,
    validate_joint_command,
)


def _default_client_factory():
    # Imported lazily so the rest of the package (and the unit tests) work without pyniryo.
    from pyniryo import NiryoRobot
    return NiryoRobot()


# pyniryo 1.0.5 (pyniryo/api/exceptions.py) signals a dead or unusable connection with these
# classes. None of them derives from NiryoRobotException, so they are matched by name: the
# module must not import pyniryo at load time. Everything else (NiryoRobotException "Command KO",
# TcpCommandException) is a refusal by the robot or a bad argument; the link is still alive.
_LINK_ERROR_NAMES = frozenset({
    'HostNotReachableException',
    'ClientNotConnectedException',
    'InvalidAnswerException',
})


def _is_link_error(exc: Exception) -> bool:
    return isinstance(exc, OSError) or type(exc).__name__ in _LINK_ERROR_NAMES


class Ned1ArmDriver(BaseArmDriver):
    """BaseArmDriver implementation for the Niryo Ned1 over its TCP API."""

    NUM_JOINTS = 6

    def __init__(
        self,
        ip_address: str = "169.254.200.200",
        velocity_percent: int = 20,
        auto_calibrate: bool = True,
        safety_margin_rad: float = 0.02,
        max_step_rad: float | None = 0.5,
        joint_limits: tuple[JointLimit, ...] = NED1_JOINT_LIMITS,
        client_factory: Callable[[], object] = _default_client_factory,
        logger: logging.Logger | None = None,
    ):
        """
        Args:
            ip_address: Robot IP (169.254.200.200 over direct Ethernet, the team's setup;
                10.10.10.10 in hotspot mode).
            velocity_percent: Arm max velocity as a percentage of its maximum (1-100).
            auto_calibrate: Run automatic calibration on connect if the robot needs it.
            safety_margin_rad: Margin kept away from each hardware joint limit.
            max_step_rad: Largest joint displacement accepted in a single command.
                None disables the check.
            joint_limits: Joint limits used to validate commands.
            client_factory: Builds the pyniryo client. Override it in tests.
            logger: Logger to use; defaults to a module logger.
        """
        if not 1 <= velocity_percent <= 100:
            raise ValueError(f"velocity_percent must be in [1, 100], got {velocity_percent}.")
        if len(joint_limits) != self.NUM_JOINTS:
            raise ValueError(f"Expected {self.NUM_JOINTS} joint limits, got {len(joint_limits)}.")

        self.ip_address = ip_address
        self.velocity_percent = velocity_percent
        self.auto_calibrate = auto_calibrate
        self.safety_margin_rad = safety_margin_rad
        self.max_step_rad = max_step_rad
        self.joint_limits = joint_limits
        self._client_factory = client_factory
        self._log = logger or logging.getLogger(self.__class__.__name__)

        self._robot = None
        self._io_lock = threading.Lock()          # Serializes every TCP call.
        self._state_lock = threading.Lock()       # Protects the cached state below.
        self._last_positions = [0.0] * self.NUM_JOINTS
        self._is_moving = False
        self._estop_latched = False
        self.is_connected = False
        self.last_error: str | None = None

    # ------------------------------------------------------------------
    # BaseArmDriver interface
    # ------------------------------------------------------------------
    def connect(self) -> bool:
        try:
            robot = self._client_factory()
            self._log.info(f"Connecting to Ned1 at {self.ip_address}...")
            with self._io_lock:
                robot.connect(self.ip_address)
                self._robot = robot

                if self._robot.need_calibration():
                    if not self.auto_calibrate:
                        self._fail("Robot needs calibration and auto_calibrate is disabled.")
                        self._close_quietly()
                        return False
                    self._log.info("Running automatic calibration (the arm will move)...")
                    self._robot.calibrate_auto()

                # Learning mode leaves the motors without torque; motion requires it off.
                self._robot.set_learning_mode(False)
                self._robot.set_arm_max_velocity(int(self.velocity_percent))
                positions = self._read_joints_locked()

            self._set_positions_cache(positions)
            self.is_connected = True
            self._estop_latched = False
            self.last_error = None
            self._log.info(f"Ned1 ready. Joints (rad): {self._format(positions)}")
            return True
        except Exception as exc:  # pyniryo raises several unrelated exception types
            self._fail(f"Connection failed: {exc}")
            self._close_quietly()
            return False

    def disconnect(self) -> None:
        if self._robot is None:
            return
        self._log.info("Disconnecting from Ned1...")
        # Waits for any running motion to finish before closing the socket.
        with self._io_lock:
            self._close_quietly()
        self.is_connected = False

    def set_joint_positions(self, joint_angles: list[float]) -> bool:
        """Moves to the target joint positions. Blocks until the motion ends."""
        if not self.is_connected or self._robot is None:
            self._fail("Driver not connected. Command refused.")
            return False
        if self._estop_latched:
            self._fail("Emergency stop latched. Call reset_emergency_stop() first.")
            return False

        target = [float(a) for a in joint_angles]
        try:
            validate_joint_command(
                target,
                self.joint_limits,
                safety_margin_rad=self.safety_margin_rad,
                current=self.get_joint_positions(),
                max_step_rad=self.max_step_rad,
            )
        except JointCommandError as exc:
            self._fail(f"Command rejected: {exc}")
            return False

        with self._io_lock:
            if self._estop_latched:  # Latched while waiting for the lock.
                self._fail("Emergency stop latched. Command dropped.")
                return False
            with self._state_lock:
                self._is_moving = True
            try:
                self._log.info(f"Moving to (rad): {self._format(target)}")
                self._robot.move_joints(target)
                positions = self._read_joints_locked()
            except Exception as exc:
                self._on_exception("Motion failed", exc)
                return False
            finally:
                with self._state_lock:
                    self._is_moving = False

        self._set_positions_cache(positions)
        self.last_error = None
        return True

    def get_joint_positions(self) -> list[float]:
        """
        Returns the current joint positions in radians.

        During a motion the robot cannot be queried, so the last known positions are
        returned without blocking. The same happens once the link is lost: the last
        known positions are returned and `is_connected` is False.
        """
        if self.is_connected and self._robot is not None and self._io_lock.acquire(blocking=False):
            try:
                self._set_positions_cache(self._read_joints_locked())
            except Exception as exc:
                self._on_exception("Joint read failed", exc)
            finally:
                self._io_lock.release()
        with self._state_lock:
            return list(self._last_positions)

    def get_joint_torques(self) -> list[float]:
        # Ned1 steppers (joints 1-3) provide no torque feedback.
        return [math.nan] * self.NUM_JOINTS

    def emergency_stop(self) -> None:
        """
        Latches the driver so that no further motion is sent.

        Limitation: the Ned1 TCP API cannot interrupt a motion already in progress.
        Learning mode is deliberately NOT enabled, as cutting torque would make the arm fall.
        """
        self._estop_latched = True
        self._log.error("EMERGENCY STOP latched: all further motion commands are refused.")
        if self.is_moving:
            self._log.error("A motion is in progress and cannot be interrupted by software.")

    # ------------------------------------------------------------------
    # Ned1-specific helpers
    # ------------------------------------------------------------------
    @property
    def is_moving(self) -> bool:
        with self._state_lock:
            return self._is_moving

    @property
    def estop_latched(self) -> bool:
        return self._estop_latched

    def reset_emergency_stop(self) -> None:
        self._estop_latched = False
        self._log.warning("Emergency stop reset.")

    def move_to_home(self) -> bool:
        """
        Moves to the Niryo home pose (forearm resting on the shoulder).

        In pyniryo 1.0.5 this is `move_joints(0.0, 0.3, -1.3, 0.0, 0.0, 0.0)`. It is a fixed,
        vendor-defined target, so it is not checked against `max_step_rad`: from a far pose
        the motion can be large.
        """
        if not self.is_connected or self._estop_latched:
            return False
        with self._io_lock:
            with self._state_lock:
                self._is_moving = True
            try:
                self._robot.move_to_home_pose()
                positions = self._read_joints_locked()
            except Exception as exc:
                self._on_exception("Home motion failed", exc)
                return False
            finally:
                with self._state_lock:
                    self._is_moving = False
        self._set_positions_cache(positions)
        return True

    # ------------------------------------------------------------------
    # Internals
    # ------------------------------------------------------------------
    def _read_joints_locked(self) -> list[float]:
        # pyniryo 1.0.5 returns a list of floats; newer versions return a JointsPosition.
        # Both are iterable.
        positions = [float(v) for v in self._robot.get_joints()]
        if len(positions) != self.NUM_JOINTS:
            raise RuntimeError(f"Robot returned {len(positions)} joints.")
        return positions

    def _set_positions_cache(self, positions: list[float]) -> None:
        with self._state_lock:
            self._last_positions = list(positions)

    def _close_quietly(self) -> None:
        if self._robot is not None:
            try:
                self._robot.close_connection()
            except Exception:
                pass
        self._robot = None
        self.is_connected = False

    def _fail(self, message: str) -> None:
        self.last_error = message
        self._log.error(message)

    def _on_exception(self, context: str, exc: Exception) -> None:
        """Records a pyniryo failure. A lost link also marks the driver as disconnected."""
        self._fail(f"{context}: {exc}")
        if _is_link_error(exc):
            self.is_connected = False
            self._log.error("Link to the robot lost. Restart the node once the robot is reachable again.")

    @staticmethod
    def _format(values: list[float]) -> str:
        return "[" + ", ".join(f"{v:.3f}" for v in values) + "]"
