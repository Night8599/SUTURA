import time
from typing import List
from sutura.drivers.base_driver import BaseArmDriver

class MockArmDriver(BaseArmDriver):
    """
    Mock driver for the robotic arm, simulating its behavior for testing purposes.
    This driver provides methods to connect, disconnect, send joint position commands,
    retrieve joint positions and torques, and perform an emergency stop.
    """

    def __init__(self, num_joints: int = 6):
        self.num_joints = num_joints
        self._current_positions = [0.0] * num_joints
        self._current_torques = [0.0] * num_joints
        self.is_connected = False

    def connect(self) -> bool:
        self.is_connected = True
        return True

    def disconnect(self) -> None:
        self.is_connected = False

    def set_joint_positions(self, joint_angles: List[float]) -> bool:
        if not self.is_connected or len(joint_angles) != self.num_joints:
            return False
        # Simulate the robotic arm moving to the desired positions
        self._current_positions = list(joint_angles)
        return True

    def get_joint_positions(self) -> List[float]:
        return self._current_positions

    def get_joint_torques(self) -> List[float]:
        # Simulate some arbitrary torque values for testing purposes
        return [0.1] * self.num_joints

    def emergency_stop(self) -> None:
        self.set_joint_positions(self._current_positions)