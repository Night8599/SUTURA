from dataclasses import dataclass, field
import numpy as np
from geometry_msgs.msg import Point, Quaternion, Pose, PoseArray


@dataclass
class Point3D:
    """Representação de um ponto cartesiano no espaço 3D (x, y, z em mm)."""

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


@dataclass
class Orientation3D:
    """Representação de orientação espacial em quatérnio (x, y, z, w)."""

    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    w: float = 1.0  # Identidade (sem rotação)

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
        """Calcula a orientação alinhando o eixo Z local com o vetor normal da superfície."""
        normal = normal_vector / np.linalg.norm(normal_vector)
        z_axis = np.array([0.0, 0.0, 1.0])

        v = np.cross(z_axis, normal)
        s = np.linalg.norm(v)
        c = np.dot(z_axis, normal)

        if s == 0:
            return Orientation3D(0.0, 0.0, 0.0, 1.0)

        vx = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
        rot_matrix = np.eye(3) + vx + np.matmul(vx, vx) * ((1 - c) / (s ** 2))

        # Conversão de matriz de rotação para quatérnio
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


@dataclass
class Pose3D:
    """Combinação de Posição 3D e Orientação 3D (Equivalente ao geometry_msgs/Pose)."""

    position: Point3D = field(default_factory=Point3D)
    orientation: Orientation3D = field(default_factory=Orientation3D)

    def to_ros_pose(self) -> Pose:
        msg = Pose()
        msg.position = self.position.to_ros_point()
        msg.orientation = self.orientation.to_ros_quaternion()
        return msg

    @staticmethod
    def from_ros_pose(msg: Pose) -> "Pose3D":
        return Pose3D(
            position=Point3D.from_ros_point(msg.position),
            orientation=Orientation3D.from_ros_quaternion(msg.orientation)
        )
