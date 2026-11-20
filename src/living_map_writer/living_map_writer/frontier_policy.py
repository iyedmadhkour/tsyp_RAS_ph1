"""Bounded retry, cooldown and expiry policy for remembered frontier goals.

Pure Python: the caller supplies monotonic seconds and the current known map
area in square metres. The policy does not decide geometry; it only says which
remembered goals are worth suppressing right now, and why. The existing
geometric selector (``frontier.choose_frontier``) stays authoritative for
reachability, safety and scoring.

Two memories are kept:

* **failure zones** - spatial neighbourhoods around failed goals. A zone gets a
  bounded number of attempts, an exponential cooldown between them, and an
  expiry afterwards. Time or substantial map growth may re-arm it a bounded
  number of times, so a region that becomes reachable later is reconsidered
  while a hopeless area cannot absorb the whole mission.
* **visited goals** - neighbourhoods the Writer already reached. They are
  suppressed briefly, then reconsidered only if the map has grown or enough
  time passed, and permanently after the configured number of visits. A
  visited point is a success, not a failure ban: it prevents driving back to
  the same few cells without suppressing the rest of that boundary.

Before Task 6 the node kept a permanent 0.6 m blacklist per failed goal and a
permanent 1.0 m visited list, with no attempt count, cooldown or expiry.
Archived missions show the same upper-passage frontier being re-attempted
0.4-1.0 s after a failure; see ``docs/results/team4-task6-exploration.md``.
"""
from __future__ import annotations

from collections import Counter
from collections.abc import Iterable
from dataclasses import dataclass
import math


@dataclass
class FrontierPolicyConfig:
    """Thresholds are exposed as ROS parameters by ``frontier_node``."""

    zone_radius_m: float = 1.5
    max_attempts: int = 3
    base_cooldown_s: float = 15.0
    max_cooldown_s: float = 60.0
    exhausted_expiry_s: float = 120.0
    max_rearms: int = 2
    visited_radius_m: float = 1.0
    visited_min_age_s: float = 30.0
    visited_expiry_s: float = 120.0
    max_visits: int = 2
    map_growth_threshold_m2: float = 2.0

    def __post_init__(self) -> None:
        numeric = (self.zone_radius_m, self.max_attempts, self.max_rearms,
                   self.max_visits, self.base_cooldown_s,
                   self.max_cooldown_s, self.exhausted_expiry_s,
                   self.visited_radius_m, self.visited_min_age_s,
                   self.visited_expiry_s, self.map_growth_threshold_m2)
        try:
            finite = all(math.isfinite(float(value)) for value in numeric)
        except (TypeError, ValueError, OverflowError):
            finite = False
        if not finite:
            raise ValueError("Frontier policy distances and timings must be finite")
        if self.zone_radius_m <= 0 or self.visited_radius_m <= 0:
            raise ValueError("Frontier memory radii must be positive")
        if self.max_attempts < 1 or self.max_rearms < 0 or self.max_visits < 1:
            raise ValueError("Attempt, re-arm and visit counts must be nonnegative")
        if min(self.base_cooldown_s, self.max_cooldown_s, self.exhausted_expiry_s,
               self.visited_min_age_s, self.visited_expiry_s) < 0:
            raise ValueError("Frontier timings must be nonnegative")
        if self.map_growth_threshold_m2 <= 0:
            raise ValueError("Map growth threshold must be positive")


@dataclass
class FailureZone:
    x: float
    y: float
    attempts: int = 0
    total_attempts: int = 0
    cooldown_until: float = 0.0
    exhausted: bool = False
    exhausted_until: float = 0.0
    known_area_at_exhaustion: float = 0.0
    rearms: int = 0
    last_failure_at: float = 0.0

    def distance(self, x: float, y: float) -> float:
        return math.hypot(self.x - x, self.y - y)

    def blocks(self, now: float) -> bool:
        return self.exhausted or now < self.cooldown_until

    def cooling(self, now: float) -> bool:
        return not self.exhausted and now < self.cooldown_until


@dataclass
class VisitRecord:
    x: float
    y: float
    count: int = 1
    at: float = 0.0
    known_area_at_visit: float = 0.0

    def distance(self, x: float, y: float) -> float:
        return math.hypot(self.x - x, self.y - y)

    def suppresses(self, now: float, known_area_m2: float,
                   cfg: FrontierPolicyConfig) -> bool:
        if self.count >= cfg.max_visits:
            return True  # visited enough; only a new sortie should retry it
        age = now - self.at
        if age < cfg.visited_min_age_s:
            return True
        if known_area_m2 - self.known_area_at_visit >= cfg.map_growth_threshold_m2:
            return False
        return age < cfg.visited_expiry_s


class FrontierPolicy:
    """Mission-scoped memory of failed and visited frontier goals."""

    def __init__(self, cfg: FrontierPolicyConfig | None = None) -> None:
        self.cfg = cfg or FrontierPolicyConfig()
        self.zones: list[FailureZone] = []
        self.visits: list[VisitRecord] = []
        self.counts: Counter[str] = Counter()

    # -- memory updates -------------------------------------------------
    def zone_for(self, x: float, y: float) -> FailureZone:
        for zone in self.zones:
            if zone.distance(x, y) <= self.cfg.zone_radius_m:
                return zone
        zone = FailureZone(x=x, y=y)
        self.zones.append(zone)
        return zone

    def note_failure(self, x: float, y: float, now: float, known_area_m2: float,
                     reason: str = "aborted") -> FailureZone:
        """Record a real navigation failure (abort, rejected goal, watchdog)."""
        zone = self.zone_for(x, y)
        zone.attempts += 1
        zone.total_attempts += 1
        zone.last_failure_at = now
        self.counts["failures"] += 1
        self.counts[f"failure_{reason}"] += 1
        if zone.attempts >= self.cfg.max_attempts:
            zone.exhausted = True
            zone.exhausted_until = now + self.cfg.exhausted_expiry_s
            zone.known_area_at_exhaustion = known_area_m2
            self.counts["exhausted"] += 1
        else:
            delay = min(self.cfg.base_cooldown_s * 2 ** (zone.attempts - 1),
                        self.cfg.max_cooldown_s)
            zone.cooldown_until = now + delay
            self.counts["cooled"] += 1
        return zone

    def note_cancel(self, x: float, y: float, now: float,
                    reason: str = "cancelled") -> None:
        """A policy/external cancellation is not evidence of unreachability."""
        self.counts["cancels"] += 1
        self.counts[f"cancel_{reason}"] += 1

    def note_success(self, x: float, y: float, now: float,
                     known_area_m2: float) -> VisitRecord:
        for record in self.visits:
            if record.distance(x, y) <= self.cfg.visited_radius_m:
                record.count += 1
                record.at = now
                record.known_area_at_visit = known_area_m2
                self.counts["visits_repeat"] += 1
                return record
        record = VisitRecord(x=x, y=y, at=now, known_area_at_visit=known_area_m2)
        self.visits.append(record)
        self.counts["visits"] += 1
        return record

    # -- time/map-driven expiry -----------------------------------------
    def refresh(self, now: float, known_area_m2: float) -> None:
        """Re-arm exhausted zones after expiry or substantial map growth."""
        for zone in self.zones:
            if not zone.exhausted:
                continue
            grew = (known_area_m2 - zone.known_area_at_exhaustion
                    >= self.cfg.map_growth_threshold_m2)
            expired = now >= zone.exhausted_until
            if not (grew or expired):
                continue
            if zone.rearms >= self.cfg.max_rearms:
                continue  # bounded: stay excluded for this sortie
            zone.rearms += 1
            zone.exhausted = False
            zone.attempts = 0
            zone.cooldown_until = now + self.cfg.base_cooldown_s
            self.counts["rearmed_growth" if grew else "rearmed_time"] += 1

    # -- queries used by the selector -----------------------------------
    def blocked_zones(self, now: float) -> list[FailureZone]:
        return [zone for zone in self.zones if zone.blocks(now)]

    def blacklist(self, now: float) -> list[tuple[float, float]]:
        """Centres whose configured radius must not be selected right now."""
        return [(zone.x, zone.y) for zone in self.blocked_zones(now)]

    def cooling_zones(self, now: float) -> list[FailureZone]:
        return [zone for zone in self.zones if zone.cooling(now)]

    def exhausted_zones(self, now: float) -> list[FailureZone]:
        return [zone for zone in self.zones if zone.exhausted and zone.blocks(now)]

    def visited_points(self, now: float,
                       known_area_m2: float) -> list[tuple[float, float]]:
        return [(record.x, record.y) for record in self.visits
                if record.suppresses(now, known_area_m2, self.cfg)]

    def earliest_retry_s(self, now: float) -> float | None:
        times = [zone.cooldown_until for zone in self.zones
                 if not zone.exhausted and zone.cooldown_until > now]
        return min(times) if times else None

    def earliest_expiry_s(self, now: float,
                          zones: Iterable[FailureZone] | None = None) -> float | None:
        """Return the next time-based re-arm, optionally scoped to relevant zones."""
        candidates = self.zones if zones is None else zones
        times = [zone.exhausted_until for zone in candidates
                 if zone.exhausted and zone.rearms < self.cfg.max_rearms
                 and zone.exhausted_until > now]
        return min(times) if times else None

    def summary(self, now: float | None = None) -> dict:
        data = {
            "failure_zones": len(self.zones),
            "visited_goals": len(self.visits),
            "counts": dict(self.counts),
        }
        if now is not None:
            data["blocked_zones"] = len(self.blocked_zones(now))
            data["cooling_zones"] = len(self.cooling_zones(now))
            data["exhausted_zones"] = len(self.exhausted_zones(now))
        return data
