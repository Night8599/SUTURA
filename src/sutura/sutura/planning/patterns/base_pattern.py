from abc import ABC, abstractmethod

from sutura.utils.data_models import GridMap2D, StitchTarget


class BasePatternGenerator(ABC):
    """Abstract Base Class for all suture pattern generators operating on a 2.5D GridMap."""

    @abstractmethod
    def generate_pattern(
        self,
        grid_map: GridMap2D,
        margin_mm: float = 3.0,
        spacing_mm: float = 5.0,
        entry_offset_mm: float = 4.0,
    ) -> list[StitchTarget]:
        """Generates an ordered list of StitchTargets derived from the GridMap2D surface.

        Args:
            grid_map: 2.5D grid map containing elevation and wound_mask layers.
            margin_mm: Safety distance in mm from wound boundaries to the first/last stitch.
            spacing_mm: Distance in mm between consecutive stitches along the wound.
            entry_offset_mm: Lateral distance in mm from wound centerline to entry/exit points.

        Returns:
            List of StitchTarget instances containing target entry and exit Pose3D objects.
        """