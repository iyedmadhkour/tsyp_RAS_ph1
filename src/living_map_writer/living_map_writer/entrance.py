"""Writer-only entrance geometry, independent of ROS and the shared SLAM map."""
from __future__ import annotations

import json
import math


def padded_footprint(value: str, padding: float) -> tuple[tuple[float, float], ...]:
    points = json.loads(value)
    if len(points) < 3 or not math.isfinite(padding) or padding < 0:
        raise ValueError("A footprint needs at least three vertices and nonnegative padding")
    result = []
    for point in points:
        if len(point) != 2 or not all(math.isfinite(v) for v in point):
            raise ValueError("Footprint vertices must be finite (x, y) pairs")
        # Same per-axis padding convention as nav2_costmap_2d::padFootprint.
        result.append(tuple(v + math.copysign(padding, v) for v in point))
    return tuple(result)


def footprint_min_x(center_x: float, yaw: float, footprint) -> float:
    """Minimum world x of the complete polygon at a particular heading."""
    c, s = math.cos(yaw), math.sin(yaw)
    return min(center_x + x * c - y * s for x, y in footprint)


def rotation_safe_center_x(entrance_x: float, footprint, goal_tolerance: float,
                           position_uncertainty: float = 0.0) -> float:
    """Allow every heading, including goal-position tolerance toward the entrance.

    The circumradius is the exact maximum support of this polygon over all
    orientations, not an empirically chosen buffer.
    """
    if (not math.isfinite(entrance_x) or not math.isfinite(goal_tolerance)
            or not math.isfinite(position_uncertainty) or goal_tolerance < 0
            or position_uncertainty < 0):
        raise ValueError("Entrance, goal tolerance and position uncertainty must be finite and nonnegative")
    return (entrance_x + max(math.hypot(x, y) for x, y in footprint)
            + goal_tolerance + position_uncertainty)


def entrance_mask(width: int, height: int, resolution: float, origin_x: float,
                  entrance_x: float, position_uncertainty: float = 0.0) -> list[int]:
    """Lethal half-plane for a map-aligned Nav2 StaticLayer.

    Mark every cell touching the forbidden half-plane. Half a cell diagonal
    also covers center sampling when the rolling odom costmap resamples this
    map through a rotated TF. Allowed cells are unknown, so merging with max
    preserves the SLAM map's unknowns and existing obstacles.
    """
    if (width <= 0 or height <= 0 or not all(math.isfinite(v) for v in
            (resolution, origin_x, entrance_x, position_uncertainty))
            or resolution <= 0 or position_uncertainty < 0):
        raise ValueError("Invalid occupancy-grid geometry")
    limit = entrance_x + position_uncertainty + resolution / math.sqrt(2.0)
    columns = [100 if origin_x + x * resolution < limit else -1 for x in range(width)]
    return columns * height
