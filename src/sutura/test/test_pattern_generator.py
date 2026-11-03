import numpy as np
import pytest
from sutura.planning.patterns.linear_pattern import LinearPatternGenerator
from sutura.utils.data_models import GridMap2D


def test_linear_pattern_generator():
    grid = GridMap2D(resolution_mm=0.5, width_cells=100, height_cells=100)

    # Populate a linear wound ROI along X axis (row=50, cols 20 to 80)
    grid.wound_mask[50, 20:80] = 1

    generator = LinearPatternGenerator(
        margin_mm=3.0, spacing_mm=5.0, entry_offset_mm=4.0
    )
    targets = generator.generate_pattern(grid)

    assert len(targets) > 0, "Failed to generate stitch targets!"
    
    # Verify first stitch geometry
    first_stitch = targets[0]
    assert first_stitch.stitch_index == 1
    
    # Entry and exit should be separated by 2 * entry_offset_mm (8.0 mm)
    entry_pos = first_stitch.entry_pose.position.to_numpy()
    exit_pos = first_stitch.exit_pose.position.to_numpy()
    distance = np.linalg.norm(entry_pos - exit_pos)
    
    assert pytest.approx(distance, abs=0.5) == 8.0