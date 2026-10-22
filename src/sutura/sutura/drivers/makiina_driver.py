from typing import List
from sutura.drivers.base_driver import BaseArmDriver


class MakiinaArmDriver(BaseArmDriver):
    """
    Driver for the MAKIINA robotic arm, communicating via CAN bus.
    This driver simulates the behavior of the MAKIINA arm for testing purposes.
    It provides methods to connect, disconnect, send joint position commands,
    retrieve joint positions and torques, and perform an emergency stop.
    """

    def __init__(self, interface: str = "can0", num_joints: int = 6):
        self.interface = interface
        self.num_joints = num_joints
        self.is_connected = False
        self._current_positions = [0.0] * num_joints
        self._current_torques = [0.0] * num_joints

    def connect(self) -> bool:
        print(f"[MAKIINA] Conecting to CAN interface '{self.interface}'...")
        print("[MAKIINA] Verifying connection...")
        self.is_connected = True
        print("[MAKIINA] Driver connected successfully.")
        return True

    def disconnect(self) -> None:
        print(f"[MAKIINA] Disconnecting from CAN interface '{self.interface}'...")
        self.is_connected = False
        print("[MAKIINA] Driver disconnected successfully.")

    def set_joint_positions(self, joint_angles: List[float]) -> bool:
        if not self.is_connected:
            print("[MAKIINA][ERROR] Driver not connected. Cannot send joint positions.")
            return False

        if len(joint_angles) != self.num_joints:
            print(f"[MAKIINA][ERROR] Expected {self.num_joints} joint angles, received {len(joint_angles)}.")
            return False

        self._current_positions = list(joint_angles)
        # Formatação para simular o envio de mensagens no barramento
        angles_str = ", ".join([f"{a:.3f}" for a in joint_angles])
        print(f"[MAKIINA][CAN tx -> {self.interface}] Set Positions (rad): [{angles_str}]")
        return True

    def get_joint_positions(self) -> List[float]:
        return self._current_positions

    def get_joint_torques(self) -> List[float]:
        # Retorna valores simulados de esforço (em N.m) nas articulações
        return [0.15, 0.22, 0.08, 0.05, 0.03, 0.01]

    def emergency_stop(self) -> None:
        print(f"[MAKIINA][CAN tx -> {self.interface}] Emergency Stop Activated!")
        print("[MAKIINA] All joint movements halted immediately.")