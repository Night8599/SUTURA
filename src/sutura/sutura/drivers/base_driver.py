from abc import ABC, abstractmethod
from typing import Dict, List

class BaseArmDriver(ABC):
    """
    Abstract base class for robotic arm drivers. 
    This class defines the interface that all specific arm drivers must implement.
    """

    @abstractmethod
    def connect(self) -> bool:
        """
        Initializes the communication with the robotic arm and prepares it for operation.
        Returns:
            bool: True if the connection was successful, False otherwise.
        """

    @abstractmethod
    def disconnect(self) -> None:
        """
        Closes the communication and safely deactivates the joints.
        Returns:
            None
        """

    @abstractmethod
    def set_joint_positions(self, joint_angles: List[float]) -> bool:
        """
        Sends position commands (in radians) to the joints.
        Args:
            joint_angles: A list of joint angles in radians.
        Returns:
            bool: True if the position commands were sent successfully, False otherwise.
        """

    @abstractmethod
    def get_joint_positions(self) -> List[float]:
        """
        Returns the current positions of the joints in radians.
        Returns:
            List[float]: A list of current joint positions in radians.
        """

    @abstractmethod
    def get_joint_torques(self) -> List[float]:
        """
        Returns the torques read from the joints (used for force detection in the pad).
        Returns:
            List[float]: A list of current joint torques.
        """

    @abstractmethod
    def emergency_stop(self) -> None:
        """
        Performs an immediate emergency stop.
        Returns:
            None
        """