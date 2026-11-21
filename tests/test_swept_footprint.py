import math
import random

import pytest

from living_map_writer.entrance import padded_footprint, footprint_min_x
from living_map_writer.swept_footprint import path_min_x, swept_min_x


FOOTPRINT = padded_footprint(
    "[[0.35,0.32],[0.35,-0.32],[-0.35,-0.32],[-0.35,0.32]]", 0.01)


def test_swept_rotation_detects_violation_between_safe_endpoints_with_uncertainty():
    start = (-0.55, 0.0, 0.0)
    end = (-0.55, 0.0, math.pi / 2)
    assert footprint_min_x(start[0], start[2], FOOTPRINT) - 0.05 >= -1.0
    assert footprint_min_x(end[0], end[2], FOOTPRINT) - 0.05 >= -1.0
    assert swept_min_x(start, end, FOOTPRINT, 0.05) < -1.0


def test_swept_curved_translation_checks_all_intermediate_support():
    poses = [(-0.52, -0.5, -0.3), (-0.49, 0.0, 0.2), (-0.52, 0.5, 0.8)]
    exact = path_min_x(poses, FOOTPRINT, 0.05)
    dense = min(
        swept_min_x(a, b, FOOTPRINT, 0.05)
        for a, b in zip(poses, poses[1:])
    )
    assert exact == pytest.approx(dense)


def test_analytic_swept_minimum_matches_dense_reference_for_seeded_paths():
    rng = random.Random(20261120)
    for _ in range(100):
        start = (rng.uniform(-1.5, 0.5), rng.uniform(-1, 1),
                 rng.uniform(-math.pi, math.pi))
        end = (rng.uniform(-1.5, 0.5), rng.uniform(-1, 1),
               rng.uniform(-math.pi, math.pi))
        uncertainty = rng.choice((0.0, 0.02, 0.05))
        analytic = swept_min_x(start, end, FOOTPRINT, uncertainty)
        dense = min(
            footprint_min_x(
                start[0] + (end[0] - start[0]) * i / 2000,
                start[2] + math.atan2(math.sin(end[2] - start[2]),
                                      math.cos(end[2] - start[2])) * i / 2000,
                FOOTPRINT)
            for i in range(2001)
        ) - uncertainty
        assert analytic <= dense + 1e-10
        assert dense - analytic < 0.001


@pytest.mark.parametrize("start,end", [
    ((0.0, 0.0, math.nan), (0.0, 0.0, 0.0)),
    ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0)),
])
def test_swept_pose_validation(start, end):
    if math.isnan(start[2]):
        with pytest.raises(ValueError):
            swept_min_x(start, end, FOOTPRINT)
    else:
        assert math.isfinite(swept_min_x(start, end, FOOTPRINT))
