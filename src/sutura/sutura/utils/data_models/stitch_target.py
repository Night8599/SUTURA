from dataclasses import dataclass, field

from sutura.utils.data_models.pose_3d import Pose3D


@dataclass
class StitchTarget:
    """Represents full 3D entry and exit target poses for a single suture stitch."""

    stitch_index: int
    entry_pose: Pose3D = field(default_factory=Pose3D)
    exit_pose: Pose3D = field(default_factory=Pose3D)