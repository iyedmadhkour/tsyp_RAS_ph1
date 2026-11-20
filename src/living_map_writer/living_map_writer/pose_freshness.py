"""Small ROS-independent helpers for pose timestamp freshness checks."""
from __future__ import annotations

import math


def pose_stamp_is_fresh(now_s: float, stamp_s: float, max_age_s: float) -> bool:
    """Accept only finite, non-future pose stamps no older than the configured bound."""
    if (not all(math.isfinite(value) for value in (now_s, stamp_s, max_age_s))
            or max_age_s <= 0):
        return False
    age_s = now_s - stamp_s
    return 0.0 <= age_s <= max_age_s
