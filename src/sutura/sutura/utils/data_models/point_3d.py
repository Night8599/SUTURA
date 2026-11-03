from dataclasses import dataclass

import numpy as np
from geometry_msgs.msg import Point


@dataclass
class Point3D:
    """Cartesian 3D point in millimeters (x, y, z)."""

    x: float = 0.0
    y: float = 0.0
    z: float = 0.0

    def to_numpy(self) -> np.ndarray:
        return np.array([self.x, self.y, self.z], dtype=np.float64)

    @staticmethod
    def from_numpy(arr: np.ndarray) -> "Point3D":
        return Point3D(x=float(arr[0]), y=float(arr[1]), z=float(arr[2]))

    def to_ros_point(self) -> Point:
        msg = Point()
        msg.x = float(self.x)
        msg.y = float(self.y)
        msg.z = float(self.z)
        return msg

    @staticmethod
    def from_ros_point(msg: Point) -> "Point3D":
        return Point3D(x=msg.x, y=msg.y, z=msg.z)
