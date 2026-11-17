"""Writer sortie lifecycle: explore, then come home while it still can.

EXPLORING -> RETURNING when no reachable frontier remains for a few
consecutive checks, all remaining frontier goals exhausted their bounded
retries, or the exploration budget - a stand-in for the battery reserve needed
to drive back - is spent. RETURNING -> HOME on arrival, or RETURN_FAILED after
repeated failed attempts.

``phase`` remains the coarse lifecycle state used by the supervisor. The
explicit ``reason_code`` says why exploration ended, so a temporary frontier
failure is not reported as a fully mapped mine:

    EXPLORED               no frontier cells remain at all
    NO_REACHABLE_FRONTIER  frontier cells remain but none is safe/reachable
    FRONTIERS_EXHAUSTED    remaining candidates used up their bounded retries
    BUDGET                 exploration budget spent
    BATTERY_RESERVE        energy policy requested HOME
    RETURN_FAILED          repeated attempts could not reach HOME
"""
from __future__ import annotations

import math

REASON_TEXT = {
    "EXPLORED": "no reachable frontier left - mine mapped",
    "NO_REACHABLE_FRONTIER": "frontier cells remain but no safe reachable goal",
    "FRONTIERS_EXHAUSTED": "remaining frontier goals exhausted bounded retries",
    "BUDGET": "sortie budget spent; keeping the reserve to get home",
    "BATTERY_RESERVE": "battery reserve reached",
    "RETURN_FAILED": "return to the entrance failed repeatedly",
}


class Sortie:
    def __init__(self, budget_s: float = 420.0, empty_ticks_to_complete: int = 4,
                 max_return_attempts: int = 3) -> None:
        self.budget_s = float(budget_s)
        if not math.isfinite(self.budget_s):
            raise ValueError("Sortie budget must be finite")
        self.empty_ticks_to_complete = max(1, int(empty_ticks_to_complete))
        self.max_return_attempts = max(1, int(max_return_attempts))
        self.phase = "EXPLORING"
        self.reason = ""
        self.reason_code = ""
        self.explored_s = 0.0
        self.empty_ticks = 0
        self.return_attempts_failed = 0

    def spend(self, seconds: float) -> None:
        self.explored_s += max(0.0, float(seconds))

    def budget_spent(self) -> bool:
        return self.budget_s > 0 and self.explored_s >= self.budget_s

    def frontier_found(self) -> None:
        self.empty_ticks = 0

    def wait_for_retry(self) -> None:
        """A cooling frontier is pending, not evidence that the map is complete."""
        if self.phase == "EXPLORING":
            self.empty_ticks = 0

    def no_frontier(self, reason_code: str = "EXPLORED") -> bool:
        """Record a check with no selectable frontier; True when it completes exploration."""
        if self.phase != "EXPLORING":
            return False
        self.empty_ticks += 1
        if self.empty_ticks >= self.empty_ticks_to_complete:
            text = REASON_TEXT.get(reason_code, REASON_TEXT["NO_REACHABLE_FRONTIER"])
            self.start_return(text, reason_code)
            return True
        return False

    def start_return(self, reason: str, reason_code: str = "") -> None:
        if self.phase == "EXPLORING":
            self.phase = "RETURNING"
            self.reason = reason
            self.reason_code = reason_code

    def return_failed(self) -> bool:
        """Count a failed return attempt; True once the Writer must give up."""
        if self.phase != "RETURNING":
            return False
        self.return_attempts_failed += 1
        if self.return_attempts_failed >= self.max_return_attempts:
            self.phase = "RETURN_FAILED"
            self.reason = REASON_TEXT["RETURN_FAILED"]
            self.reason_code = "RETURN_FAILED"
            return True
        return False

    def arrived_home(self) -> None:
        if self.phase == "RETURNING":
            self.phase = "HOME"
