from dataclasses import dataclass, field

import numpy as np

from sutura.utils.data_models.orientation_3d import Orientation3D
from sutura.utils.data_models.pose_3d import Pose3D


@dataclass
class GridMap2D:
    """2.5D Grid Map representation for tissue surface and wound mapping."""

    resolution_mm: float = 0.5
    width_cells: int = 200
    height_cells: int = 200
    origin_pose: Pose3D = field(default_factory=Pose3D)
    frame_id: str = "camera_frame"

    elevation: np.ndarray = field(init=False)
    wound_mask: np.ndarray = field(init=False)
    confidence: np.ndarray = field(init=False)

    def __post_init__(self):
        shape = (self.height_cells, self.width_cells)
        self.elevation = np.zeros(shape, dtype=np.float32)
        self.wound_mask = np.zeros(shape, dtype=np.uint8)
        self.confidence = np.ones(shape, dtype=np.float32)

    def world_to_grid(self, x_mm: float, y_mm: float) -> tuple[int, int] | None:
        origin_x = self.origin_pose.position.x
        origin_y = self.origin_pose.position.y

        col = int((x_mm - origin_x) / self.resolution_mm)
        row = int((y_mm - origin_y) / self.resolution_mm)

        if 0 <= row < self.height_cells and 0 <= col < self.width_cells:
            return row, col
        return None

    def grid_to_world(self, row: int, col: int) -> tuple[float, float, float]:
        """Converts grid cell indices (row, col) to 3D world coordinates at the cell center."""
        x_mm = self.origin_pose.position.x + (col + 0.5) * self.resolution_mm
        y_mm = self.origin_pose.position.y + (row + 0.5) * self.resolution_mm
        z_mm = float(self.elevation[row, col])
        return x_mm, y_mm, z_mm

    def get_elevation_at_world(self, x_mm: float, y_mm: float) -> float | None:
        """Returns tissue elevation Z in mm at specified world coordinates."""
        coords = self.world_to_grid(x_mm, y_mm)
        if coords:
            row, col = coords
            return float(self.elevation[row, col])
        return None

    def is_wound_at_world(self, x_mm: float, y_mm: float) -> bool:
        """Checks if specified world coordinates belong to the wound ROI."""
        coords = self.world_to_grid(x_mm, y_mm)
        if coords:
            row, col = coords
            return bool(self.wound_mask[row, col] == 1)
        return False

    def compute_surface_normal(self, row: int, col: int) -> Orientation3D:
        r_min, r_max = max(0, row - 1), min(self.height_cells - 1, row + 1)
        c_min, c_max = max(0, col - 1), min(self.width_cells - 1, col + 1)

        dz_dx = (self.elevation[row, c_max] - self.elevation[row, c_min]) / (
            (c_max - c_min) * self.resolution_mm + 1e-6
        )
        dz_dy = (self.elevation[r_max, col] - self.elevation[r_min, col]) / (
            (r_max - r_min) * self.resolution_mm + 1e-6
        )

        normal_vec = np.array([-dz_dx, -dz_dy, 1.0], dtype=np.float64)
        return Orientation3D.from_surface_normal(normal_vec)

