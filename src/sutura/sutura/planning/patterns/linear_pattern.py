import numpy as np

from sutura.planning.patterns.base_pattern import BasePatternGenerator
from sutura.utils.data_models import (
    GridMap2D,
    Point3D,
    Pose3D,
    StitchTarget,
)


class LinearPatternGenerator(BasePatternGenerator):
    """Generates equidistant linear stitch points along a wound ROI in a GridMap2D."""

    def __init__(
        self,
        margin_mm: float = 3.0,
        spacing_mm: float = 5.0,
        entry_offset_mm: float = 4.0,
        entry_angle_deg: float = 90.0,
    ):
        self.margin_mm = margin_mm
        self.spacing_mm = spacing_mm
        self.entry_offset_mm = entry_offset_mm
        self.entry_angle_deg = entry_angle_deg

    def generate_pattern(self, grid_map: GridMap2D) -> list[StitchTarget]:
        """Extracts wound centerline from grid_map and generates StitchTarget list."""
        # 1. Locate all cells belonging to the wound ROI
        wound_rows, wound_cols = np.where(grid_map.wound_mask == 1)
        if len(wound_rows) < 2:
            return []

        # 2. Convert grid cell indices to 2D world coordinates
        world_pts = np.zeros((len(wound_rows), 2), dtype=np.float64)
        for i in range(len(wound_rows)):
            x_w, y_w, _ = grid_map.grid_to_world(wound_rows[i], wound_cols[i])
            world_pts[i] = [x_w, y_w]

        # 3. Compute principal axis of the wound using 2D PCA
        start_2d, end_2d, dir_u = self._fit_centerline(world_pts)
        total_length = float(np.linalg.norm(end_2d - start_2d))

        usable_length = total_length - (2.0 * self.margin_mm)
        if usable_length <= 0:
            return []

        num_stitches = int(usable_length // self.spacing_mm) + 1
        stitch_targets: list[StitchTarget] = []

        # 4. Generate StitchTargets along the centerline
        for i in range(num_stitches):
            dist = self.margin_mm + (i * self.spacing_mm)
            center_2d = start_2d + (dir_u * dist)

            # Query elevation and surface normal at the centerline point
            center_coords = grid_map.world_to_grid(center_2d[0], center_2d[1])
            if center_coords is None:
                continue

            c_row, c_col = center_coords
            normal_orient = grid_map.compute_surface_normal(c_row, c_col)
            normal_q = normal_orient.to_numpy()

            # Convert quaternion to normal vector [nx, ny, nz]
            nz = 1.0 - 2.0 * (normal_q[0] ** 2 + normal_q[1] ** 2)
            nx = 2.0 * (normal_q[0] * normal_q[2] + normal_q[3] * normal_q[1])
            ny = 2.0 * (normal_q[1] * normal_q[2] - normal_q[3] * normal_q[0])
            normal_vec = np.array([nx, ny, nz], dtype=np.float64)
            normal_vec /= np.linalg.norm(normal_vec) + 1e-6

            # Compute lateral vector orthogonal to wound direction and surface normal
            u_3d = np.array([dir_u[0], dir_u[1], 0.0], dtype=np.float64)
            v_3d = np.cross(u_3d, normal_vec)
            v_norm = np.linalg.norm(v_3d)
            if v_norm > 0:
                v_3d /= v_norm

            # 5. Compute entry and exit 3D world coordinates
            entry_xy = center_2d + (v_3d[:2] * self.entry_offset_mm)
            exit_xy = center_2d - (v_3d[:2] * self.entry_offset_mm)

            entry_z = grid_map.get_elevation_at_world(entry_xy[0], entry_xy[1])
            exit_z = grid_map.get_elevation_at_world(exit_xy[0], exit_xy[1])

            if entry_z is None or exit_z is None:
                continue

            entry_pose = Pose3D(
                position=Point3D(x=float(entry_xy[0]), y=float(entry_xy[1]), z=entry_z),
                orientation=normal_orient,
            )
            exit_pose = Pose3D(
                position=Point3D(x=float(exit_xy[0]), y=float(exit_xy[1]), z=exit_z),
                orientation=normal_orient,
            )

            stitch_targets.append(
                StitchTarget(
                    stitch_index=i + 1,
                    entry_pose=entry_pose,
                    exit_pose=exit_pose,
                )
            )

        return stitch_targets

    def _fit_centerline(
        self, pts_2d: np.ndarray
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Extracts start point, end point, and unit direction vector along wound ROI using PCA."""
        mean = np.mean(pts_2d, axis=0)
        centered = pts_2d - mean

        cov = np.cov(centered, rowvar=False)
        eigenvalues, eigenvectors = np.linalg.eigh(cov)

        # Primary direction corresponds to eigenvector with largest eigenvalue
        dir_u = eigenvectors[:, np.argmax(eigenvalues)]
        dir_u /= np.linalg.norm(dir_u)

        # Project points onto primary direction to find extreme endpoints
        projections = centered @ dir_u
        min_proj, max_proj = np.min(projections), np.max(projections)

        start_2d = mean + (min_proj * dir_u)
        end_2d = mean + (max_proj * dir_u)

        # Ensure dir_u points from start to end
        dir_u = (end_2d - start_2d) / (np.linalg.norm(end_2d - start_2d) + 1e-6)

        return start_2d, end_2d, dir_u