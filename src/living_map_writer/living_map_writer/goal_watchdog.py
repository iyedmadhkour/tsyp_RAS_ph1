"""Navigation-progress watchdog for a single Nav2 goal.

This is deliberately **not** the STUCK detector in ``progress.py``. The STUCK
detector asks whether the robot is physically immobilised while it is trying to
drive. This watchdog asks a narrower question: is the current navigation goal
making useful progress, or should it be cancelled and retried/replaced?

Evidence, in priority order:

1. ``NavigateToPose`` feedback ``distance_remaining`` (metres of path left).
2. The robot's map pose, when feedback is not available yet.

A goal is healthy while its remaining distance improves by
``min_progress_m`` within the observation window. Rotation-in-place is tolerated
for ``rotation_grace_s``; a prolonged spin does not hide a stall. A large jump
in the measured distance (a SLAM correction or a replanned detour) restarts the
reference instead of being counted as either progress or a stall.

The watchdog never publishes STUCK or DESTROYED. It returns a reason string;
the explorer cancels the action and records a normal navigation failure.
"""
from __future__ import annotations

from dataclasses import dataclass
import math


@dataclass
class WatchdogConfig:
    progress_window_s: float = 20.0
    min_progress_m: float = 0.25
    rotation_grace_s: float = 10.0
    # Cancel a stalled Nav2 goal before the independent 45 s physical STUCK
    # detector can label a navigation stall as robot immobilisation.
    stall_timeout_s: float = 25.0
    goal_timeout_s: float = 150.0
    home_stall_timeout_s: float = 90.0
    home_timeout_s: float = 300.0
    max_recoveries: int = 3
    reset_jump_m: float = 1.0

    def __post_init__(self) -> None:
        numeric = (self.progress_window_s, self.min_progress_m,
                   self.rotation_grace_s, self.stall_timeout_s,
                   self.goal_timeout_s, self.home_stall_timeout_s,
                   self.home_timeout_s, self.max_recoveries,
                   self.reset_jump_m)
        try:
            finite = all(math.isfinite(float(value)) for value in numeric)
        except (TypeError, ValueError, OverflowError):
            finite = False
        if not finite:
            raise ValueError("Watchdog distances, limits and timings must be finite")
        if self.progress_window_s <= 0 or self.min_progress_m <= 0:
            raise ValueError("Watchdog progress window and distance must be positive")
        if self.rotation_grace_s < 0 or self.reset_jump_m <= 0:
            raise ValueError("Watchdog rotation grace and jump threshold are invalid")
        if self.stall_timeout_s < self.progress_window_s:
            raise ValueError("Watchdog stall timeout must cover the progress window")
        if self.home_stall_timeout_s < self.progress_window_s:
            raise ValueError("Home stall timeout must cover the progress window")
        if self.goal_timeout_s <= 0 or self.home_timeout_s <= 0:
            raise ValueError("Watchdog timeouts must be positive")
        if self.max_recoveries < 1:
            raise ValueError("Watchdog recovery limit must be positive")


class GoalWatchdog:
    def __init__(self, cfg: WatchdogConfig | None = None) -> None:
        self.cfg = cfg or WatchdogConfig()
        self.reset()

    def reset(self) -> None:
        self.kind = ""
        self.goal_x = 0.0
        self.goal_y = 0.0
        self.start_at: float | None = None
        self.reference_distance: float | None = None
        self.window_distance: float | None = None
        self.window_started_at: float | None = None
        self.last_progress_at: float | None = None
        self.last_rotation_at: float | None = None
        self.rotation_started_at: float | None = None
        self.max_recoveries_seen = 0
        self.last_distance: float | None = None
        self.jumps = 0
        self.source = "none"

    def start(self, kind: str, goal_x: float, goal_y: float, now: float,
              distance_remaining: float | None = None) -> None:
        self.reset()
        self.kind = kind
        self.goal_x = float(goal_x)
        self.goal_y = float(goal_y)
        self.start_at = float(now)
        self.last_progress_at = float(now)
        self.last_rotation_at = float(now)
        if _finite_positive(distance_remaining):
            self.reference_distance = float(distance_remaining)
            self.last_distance = self.reference_distance
            self.window_distance = self.reference_distance
            self.window_started_at = float(now)
            self.source = "feedback"

    def _limits(self) -> tuple[float, float]:
        if self.kind == "home":
            return self.cfg.home_stall_timeout_s, self.cfg.home_timeout_s
        return self.cfg.stall_timeout_s, self.cfg.goal_timeout_s

    def update(self, now: float, distance_remaining: float | None,
               recoveries: int, x: float, y: float, rotating: bool) -> str | None:
        """Feed one observation; return a cancellation reason or ``None``."""
        now = float(now)
        if self.start_at is None:
            return None

        # Prefer Nav2's path-aware feedback; fall back to straight-line pose
        # distance while the first feedback message is still in flight.
        if _finite_positive(distance_remaining):
            distance = float(distance_remaining)
            self.source = "feedback"
        else:
            distance = math.hypot(self.goal_x - float(x), self.goal_y - float(y))
            self.source = "pose"

        if self.reference_distance is None:
            self.reference_distance = distance
            self.last_distance = distance
            self.window_distance = distance
            self.window_started_at = now
            self.last_progress_at = now
        elif distance > self.last_distance + self.cfg.reset_jump_m:
            # A large worsening is a map correction/replan, not robot failure.
            self.reference_distance = distance
            self.window_distance = distance
            self.window_started_at = now
            self.last_progress_at = now
            self.jumps += 1
        elif self.window_distance is None or self.window_started_at is None:
            self.window_distance = distance
            self.window_started_at = now
        elif now - self.window_started_at <= self.cfg.progress_window_s:
            if distance <= self.window_distance - self.cfg.min_progress_m:
                self.reference_distance = distance
                self.last_progress_at = now
                self.window_distance = distance
                self.window_started_at = now
        else:
            # Sub-threshold movement from a previous window cannot accumulate
            # forever and keep a goal alive. Start a fresh observation window.
            self.window_distance = distance
            self.window_started_at = now
        self.last_distance = distance

        if rotating:
            if self.rotation_started_at is None:
                self.rotation_started_at = now
            # Tolerate a normal turn, but do not let an endless spin mask a stall.
            if now - self.rotation_started_at <= self.cfg.rotation_grace_s:
                self.last_rotation_at = now
        else:
            self.rotation_started_at = None

        self.max_recoveries_seen = max(self.max_recoveries_seen, int(recoveries))
        stall_limit, timeout_limit = self._limits()
        elapsed = now - self.start_at
        stalled_for = now - max(self.last_progress_at or self.start_at,
                                self.last_rotation_at or self.start_at)

        if elapsed >= timeout_limit:
            return "timeout"
        if (self.max_recoveries_seen >= self.cfg.max_recoveries
                and now - (self.last_progress_at or self.start_at)
                >= self.cfg.progress_window_s):
            return "recovery_limit"
        if stalled_for >= stall_limit:
            return "stalled"
        return None

    def snapshot(self, now: float | None = None) -> dict:
        elapsed = None if self.start_at is None or now is None else round(now - self.start_at, 2)
        return {
            "kind": self.kind,
            "source": self.source,
            "elapsed_s": elapsed,
            "reference_distance_m": None if self.reference_distance is None
            else round(self.reference_distance, 3),
            "last_distance_m": None if self.last_distance is None
            else round(self.last_distance, 3),
            "recoveries_seen": self.max_recoveries_seen,
            "distance_jumps": self.jumps,
        }


def _finite_positive(value: float | None) -> bool:
    return value is not None and math.isfinite(float(value)) and float(value) > 0.0
