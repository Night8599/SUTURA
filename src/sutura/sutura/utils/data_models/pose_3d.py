from dataclasses import dataclass, field

from geometry_msgs.msg import Pose

from sutura.utils.data_models.orientation_3d import Orientation3D
from sutura.utils.data_models.point_3d import Point3D


@dataclass
class Pose3D:
    """Combines a 3D position and orientation."""

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
            orientation=Orientation3D.from_ros_quaternion(msg.orientation),
        )
