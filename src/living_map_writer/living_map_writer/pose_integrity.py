"""Compare Writer odometry with an independent, frame-aligned motion reference.

This monitor checks relative motion only, so independent sources may have
different origins. Their x/y axes and positive yaw convention must agree. A
reference derived from the same pose estimate is not independent and must not
be configured here.
"""
from __future__ import annotations

from dataclasses import dataclass
from collections import deque
import math


def _angle_delta(a: float, b: float) -> float:
    return math.atan2(math.sin(a - b), math.cos(a - b))


@dataclass(frozen=True)
class PoseSample:
    stamp_s: float
    x: float
    y: float
    yaw: float


class PoseIntegrityMonitor:
    """Latch disagreement between estimated pose motion and a reference.

    Position/yaw allowances come from existing Team 4 simulation assumptions.
    They are not calibrated hardware limits. A fault is sticky until an
    operator acknowledges recovery after paired streams have resumed.
    """

    def __init__(self, position_tolerance_m: float = 0.05,
                 yaw_tolerance_rad: float = 0.02,
                 max_stamp_skew_s: float = 0.05) -> None:
        values = (position_tolerance_m, yaw_tolerance_rad, max_stamp_skew_s)
        if not all(math.isfinite(float(v)) for v in values):
            raise ValueError("pose-integrity tolerances must be finite")
        if position_tolerance_m <= 0 or yaw_tolerance_rad <= 0 or max_stamp_skew_s < 0:
            raise ValueError("pose-integrity tolerances must be positive")
        self.position_tolerance_m = float(position_tolerance_m)
        self.yaw_tolerance_rad = float(yaw_tolerance_rad)
        self.max_stamp_skew_s = float(max_stamp_skew_s)
        self.estimate: PoseSample | None = None
        self.reference: PoseSample | None = None
        self.estimate_history: deque[PoseSample] = deque(maxlen=100)
        self.reference_history: deque[PoseSample] = deque(maxlen=100)
        self.last_pair_stamp_s: float | None = None
        self.anchor: tuple[PoseSample, PoseSample] | None = None
        self.expected_offset: tuple[float, float, float] | None = None
        self.fault_reason: str | None = None
        self.post_fault_good_pairs = 0

    @staticmethod
    def _sample(stamp_s, x, y, yaw) -> PoseSample | None:
        try:
            values = tuple(float(v) for v in (stamp_s, x, y, yaw))
        except (TypeError, ValueError, OverflowError):
            return None
        if not all(math.isfinite(v) for v in values):
            return None
        return PoseSample(*values)

    def observe_estimate(self, stamp_s, x, y, yaw) -> str | None:
        sample = self._sample(stamp_s, x, y, yaw)
        if sample is not None and (self.estimate is None or sample.stamp_s >= self.estimate.stamp_s):
            self.estimate = sample
            self._append_history(self.estimate_history, sample)
        return self._compare()

    def observe_reference(self, stamp_s, x, y, yaw) -> str | None:
        sample = self._sample(stamp_s, x, y, yaw)
        if sample is not None and (self.reference is None or sample.stamp_s >= self.reference.stamp_s):
            self.reference = sample
            self._append_history(self.reference_history, sample)
        return self._compare()

    @staticmethod
    def _append_history(history: deque[PoseSample], sample: PoseSample) -> None:
        if history and sample.stamp_s == history[-1].stamp_s:
            history[-1] = sample
        elif not history or sample.stamp_s > history[-1].stamp_s:
            history.append(sample)

    @staticmethod
    def _interpolate(history: deque[PoseSample], stamp_s: float) -> PoseSample | None:
        before = None
        for after in history:
            if after.stamp_s == stamp_s:
                return after
            if after.stamp_s > stamp_s:
                if before is None:
                    return None
                span = after.stamp_s - before.stamp_s
                if span <= 0:
                    return None
                fraction = (stamp_s - before.stamp_s) / span
                return PoseSample(
                    stamp_s,
                    before.x + fraction * (after.x - before.x),
                    before.y + fraction * (after.y - before.y),
                    before.yaw + fraction * _angle_delta(after.yaw, before.yaw),
                )
            before = after
        return None

    def _synchronized_pair(self) -> tuple[float, PoseSample, PoseSample] | None:
        if self.estimate is None or self.reference is None:
            return None
        if abs(self.estimate.stamp_s - self.reference.stamp_s) > self.max_stamp_skew_s:
            return None
        stamp_s = min(self.estimate.stamp_s, self.reference.stamp_s)
        estimate = self._interpolate(self.estimate_history, stamp_s)
        reference = self._interpolate(self.reference_history, stamp_s)
        if estimate is None or reference is None:
            return None
        return stamp_s, estimate, reference

    def latch_fault(self, reason: str) -> None:
        """Route external freshness/pairing faults through the recovery gate.

        Missing or stale paired data has no numerical disagreement for this
        monitor to report, but it must still require fresh agreement evidence
        and explicit operator acknowledgment before motion resumes.
        """
        if self.fault_reason is None:
            self.fault_reason = str(reason) or "external pose-integrity fault"
            self.post_fault_good_pairs = 0

    def _compare(self) -> str | None:
        synchronized = self._synchronized_pair()
        if synchronized is None:
            return None
        stamp_s, est, ref = synchronized
        if (self.last_pair_stamp_s is not None
                and stamp_s <= self.last_pair_stamp_s):
            return self.fault_reason
        self.last_pair_stamp_s = stamp_s
        if self.anchor is None:
            self.anchor = (est, ref)
            self.expected_offset = (
                est.x - ref.x, est.y - ref.y,
                _angle_delta(est.yaw, ref.yaw))
            return self.fault_reason

        prior_est, prior_ref = self.anchor
        est_dx, est_dy = est.x - prior_est.x, est.y - prior_est.y
        ref_dx, ref_dy = ref.x - prior_ref.x, ref.y - prior_ref.y
        position_error = math.hypot(est_dx - ref_dx, est_dy - ref_dy)
        est_dyaw = _angle_delta(est.yaw, prior_est.yaw)
        ref_dyaw = _angle_delta(ref.yaw, prior_ref.yaw)
        yaw_error = abs(_angle_delta(est_dyaw, ref_dyaw))
        expected_x, expected_y, expected_yaw = self.expected_offset
        offset_error = math.hypot((est.x - ref.x) - expected_x,
                                  (est.y - ref.y) - expected_y)
        yaw_offset_error = abs(_angle_delta(
            _angle_delta(est.yaw, ref.yaw), expected_yaw))

        if self.fault_reason is not None:
            # Recovery evidence is only for operator review; it never clears a
            # fault automatically. Count distinct paired samples whose recent
            # motions agree again.
            if (offset_error <= self.position_tolerance_m
                    and yaw_offset_error <= self.yaw_tolerance_rad):
                self.post_fault_good_pairs += 1
            else:
                self.post_fault_good_pairs = 0
            self.anchor = (est, ref)
            return self.fault_reason

        if position_error > self.position_tolerance_m:
            self.fault_reason = (
                "independent odometry motion disagrees with Writer pose by "
                f"{position_error:.3f} m (allowance "
                f"{self.position_tolerance_m:.3f} m)")
        elif yaw_error > self.yaw_tolerance_rad:
            self.fault_reason = (
                "independent odometry rotation disagrees with Writer pose by "
                f"{yaw_error:.4f} rad (allowance "
                f"{self.yaw_tolerance_rad:.4f} rad)")
        elif (offset_error > self.position_tolerance_m
              or yaw_offset_error > self.yaw_tolerance_rad):
            self.fault_reason = (
                "independent odometry and Writer pose no longer share their "
                f"registered relative transform (translation error "
                f"{offset_error:.3f} m, yaw error {yaw_offset_error:.4f} rad)")
        else:
            est_motion = math.hypot(est_dx, est_dy)
            ref_motion = math.hypot(ref_dx, ref_dy)
            yaw_motion = max(abs(est_dyaw), abs(ref_dyaw))
            if (min(est_motion, ref_motion) > self.position_tolerance_m
                    or yaw_motion > self.yaw_tolerance_rad):
                self.anchor = (est, ref)
        return self.fault_reason

    def paired_fresh(self, now_s: float, max_age_s: float) -> bool:
        return self.freshness_reason(now_s, max_age_s) is None

    def freshness_reason(self, now_s: float, max_age_s: float) -> str | None:
        est, ref = self.estimate, self.reference
        if est is None or ref is None:
            return "estimate or independent reference has not arrived"
        skew = abs(est.stamp_s - ref.stamp_s)
        if skew > self.max_stamp_skew_s:
            return (f"latest pose/reference stamp skew {skew:.3f}s exceeds "
                    f"{self.max_stamp_skew_s:.3f}s")
        est_age, ref_age = now_s - est.stamp_s, now_s - ref.stamp_s
        # ROS callbacks can observe a just-published sample a few milliseconds
        # ahead of the node's current clock reading. Treat that as clock/read
        # scheduling skew only inside the same configured pairing allowance;
        # larger future stamps still fail closed.
        min_age_s = -self.max_stamp_skew_s
        if not min_age_s <= est_age <= max_age_s:
            return (f"Writer pose age {est_age:.3f}s is outside "
                    f"{min_age_s:.3f}..{max_age_s:.3f}s")
        if not min_age_s <= ref_age <= max_age_s:
            return (f"independent reference age {ref_age:.3f}s is outside "
                    f"{min_age_s:.3f}..{max_age_s:.3f}s")
        if self._synchronized_pair() is None:
            return "fresh pose streams could not be aligned at a common timestamp"
        return None

    def diagnostics(self, now_s: float, max_age_s: float) -> dict:
        est, ref = self.estimate, self.reference
        synchronized = self._synchronized_pair()
        return {
            "estimate_stamp_s": est.stamp_s if est is not None else None,
            "reference_stamp_s": ref.stamp_s if ref is not None else None,
            "estimate_age_s": now_s - est.stamp_s if est is not None else None,
            "reference_age_s": now_s - ref.stamp_s if ref is not None else None,
            "latest_stamp_skew_s": (abs(est.stamp_s - ref.stamp_s)
                                    if est is not None and ref is not None else None),
            "max_stamp_skew_s": self.max_stamp_skew_s,
            "max_age_s": max_age_s,
            "synchronized_pair_available": synchronized is not None,
            "last_pair_stamp_s": self.last_pair_stamp_s,
            "freshness_failure": self.freshness_reason(now_s, max_age_s),
        }

    @property
    def recovery_ready(self) -> bool:
        synchronized = self._synchronized_pair()
        return (self.fault_reason is not None and self.post_fault_good_pairs >= 2
                and self.estimate is not None and self.reference is not None
                and synchronized is not None
                and abs(self.estimate.stamp_s - self.reference.stamp_s)
                <= self.max_stamp_skew_s)

    def acknowledge_recovery(self) -> bool:
        if not self.recovery_ready:
            return False
        synchronized = self._synchronized_pair()
        if synchronized is None:
            return False
        _, estimate, reference = synchronized
        self.fault_reason = None
        self.anchor = (estimate, reference)
        self.post_fault_good_pairs = 0
        return True
