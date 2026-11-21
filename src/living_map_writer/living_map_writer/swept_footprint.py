"""Continuous entrance-plane checks for interpolated planar robot motion."""
from __future__ import annotations

import math


def shortest_yaw_delta(start: float, end: float) -> float:
    """Return the signed shortest angular difference in [-pi, pi]."""
    if not math.isfinite(start) or not math.isfinite(end):
        raise ValueError("yaw values must be finite")
    return math.atan2(math.sin(end - start), math.cos(end - start))


def swept_min_x(start: tuple[float, float, float],
                end: tuple[float, float, float], footprint,
                position_uncertainty: float = 0.0) -> float:
    """Exact minimum x support while pose is linearly interpolated.

    Translation is linear and yaw follows the shortest angular arc. For each
    polygon vertex, the x coordinate is a linear term plus a sinusoid. Its
    analytic stationary points and both endpoints are evaluated, so a turn
    cannot cross the boundary between sampled poses without being detected.
    The uncertainty value is an axis-aligned bound in x, subtracted from the
    minimum support. This models a bounded translation error, not localization
    performance or arbitrary unmodeled motion between samples.
    """
    values = (*start, *end, position_uncertainty)
    if (not all(math.isfinite(value) for value in values)
            or position_uncertainty < 0 or len(footprint) < 3):
        raise ValueError("poses, uncertainty and polygon are invalid")
    if any(len(point) != 2 or not all(math.isfinite(v) for v in point)
           for point in footprint):
        raise ValueError("footprint vertices must be finite x,y pairs")

    x0, _y0, yaw0 = start
    x1, _y1, yaw1 = end
    dx = x1 - x0
    omega = shortest_yaw_delta(yaw0, yaw1)
    best = math.inf

    for vx, vy in footprint:
        def x_at(t: float) -> float:
            angle = yaw0 + omega * t
            return x0 + dx * t + vx * math.cos(angle) - vy * math.sin(angle)

        candidates = [0.0, 1.0]
        radius = math.hypot(vx, vy)
        if abs(omega) > 1e-15 and radius > 0.0:
            ratio = dx / (omega * radius)
            if -1.0 <= ratio <= 1.0:
                phase = math.atan2(vy, vx)
                alpha = math.asin(ratio)
                low = min(yaw0, yaw0 + omega)
                high = max(yaw0, yaw0 + omega)
                for root in (alpha - phase, math.pi - alpha - phase):
                    first = math.floor((low - root) / (2.0 * math.pi)) - 1
                    last = math.ceil((high - root) / (2.0 * math.pi)) + 1
                    for turn in range(first, last + 1):
                        theta = root + turn * 2.0 * math.pi
                        t = (theta - yaw0) / omega
                        if 0.0 < t < 1.0:
                            candidates.append(t)
        best = min(best, *(x_at(t) for t in candidates))

    return best - position_uncertainty


def path_min_x(poses, footprint, position_uncertainty: float = 0.0) -> float:
    """Minimum swept x support across an ordered sequence of planar poses."""
    if not poses:
        raise ValueError("at least one pose is required")
    if len(poses) == 1:
        return swept_min_x(poses[0], poses[0], footprint, position_uncertainty)
    return min(swept_min_x(a, b, footprint, position_uncertainty)
               for a, b in zip(poses, poses[1:]))
