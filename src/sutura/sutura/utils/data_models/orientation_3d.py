from dataclasses import dataclass

import numpy as np
from geometry_msgs.msg import Quaternion


@dataclass
class Orientation3D:
    """3D spatial orientation represented as a unit quaternion (x, y, z, w)."""

    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    w: float = 1.0

    def to_numpy(self) -> np.ndarray:
        return np.array([self.x, self.y, self.z, self.w], dtype=np.float64)

    @staticmethod
    def from_numpy(arr: np.ndarray) -> "Orientation3D":
        return Orientation3D(x=float(arr[0]), y=float(arr[1]), z=float(arr[2]), w=float(arr[3]))

    def to_ros_quaternion(self) -> Quaternion:
        msg = Quaternion()
        msg.x = float(self.x)
        msg.y = float(self.y)
        msg.z = float(self.z)
        msg.w = float(self.w)
        return msg

    @staticmethod
    def from_ros_quaternion(msg: Quaternion) -> "Orientation3D":
        return Orientation3D(x=msg.x, y=msg.y, z=msg.z, w=msg.w)

    @staticmethod
    def from_surface_normal(normal_vector: np.ndarray) -> "Orientation3D":
        norm = np.linalg.norm(normal_vector)
        if norm == 0:
            return Orientation3D(0.0, 0.0, 0.0, 1.0)

        normal = normal_vector / norm
        z_axis = np.array([0.0, 0.0, 1.0])

        v = np.cross(z_axis, normal)
        s = np.linalg.norm(v)
        c = np.dot(z_axis, normal)

        if s == 0:
            return Orientation3D(0.0, 0.0, 0.0, 1.0)

        vx = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
        rot_matrix = np.eye(3) + vx + np.matmul(vx, vx) * ((1 - c) / (s ** 2))

        tr = np.trace(rot_matrix)
        if tr > 0:
            S = np.sqrt(tr + 1.0) * 2
            qw = 0.25 * S
            qx = (rot_matrix[2, 1] - rot_matrix[1, 2]) / S
            qy = (rot_matrix[0, 2] - rot_matrix[2, 0]) / S
            qz = (rot_matrix[1, 0] - rot_matrix[0, 1]) / S
        else:
            qw, qx, qy, qz = 1.0, 0.0, 0.0, 0.0

        return Orientation3D(x=qx, y=qy, z=qz, w=qw)
