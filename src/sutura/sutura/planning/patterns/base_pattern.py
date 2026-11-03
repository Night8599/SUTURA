from abc import ABC, abstractmethod
import numpy as np
from sutura.utils.data_models import PointSequence3D


class BasePatternGenerator(ABC):
    """Abstract base class for all suture point pattern generators."""

    @abstractmethod
    def generate_stitch_points(
        self, incision: PointSequence3D
    ) -> PointSequence3D:
        """Calculates entry, exit, and target 3D points along an incision.

        Args:
            incision: 3D point sequence representing the incision beginning and end.

        Returns:
            A PointSequence3D containing the calculated stitch points.
        """
        pass