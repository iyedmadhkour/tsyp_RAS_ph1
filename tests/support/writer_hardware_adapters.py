"""Test-only mock adapter contracts; these are not production ROS interfaces.

Values use the repository's current canonical ROS semantics where known. The
battery-percent shape and normalized radio probability are fixtures, not Team 1
hardware specifications.
"""
from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Callable


@dataclass(frozen=True)
class Read:
    state: str
    value: object = None
    age_s: float | None = None


class TimedChannel:
    """Small deterministic receiver mock with explicit connection/freshness."""

    def __init__(self, valid: Callable[[object], bool], max_age_s: float = 1.0) -> None:
        if not math.isfinite(max_age_s) or max_age_s <= 0:
            raise ValueError("max_age_s must be finite and positive")
        self.valid = valid
        self.max_age_s = max_age_s
        self.connected = False
        self._value = None
        self._received_at: float | None = None
        self._source_stamp: float | None = None

    def connect(self) -> None:
        self.connected = True

    def disconnect(self) -> None:
        self.connected = False

    def publish(self, value: object, received_at: float,
                source_stamp: float | None = None) -> None:
        self._value = value
        self._received_at = float(received_at)
        self._source_stamp = None if source_stamp is None else float(source_stamp)

    def read(self, now: float) -> Read:
        now = float(now)
        if not math.isfinite(now):
            return Read("invalid")
        if not self.connected:
            return Read("disconnected")
        if self._received_at is None:
            return Read("missing")
        age = now - self._received_at
        if age < 0 or not math.isfinite(age):
            return Read("invalid", age_s=age)
        freshness_age = age
        if self._source_stamp is not None:
            freshness_age = now - self._source_stamp
            if not math.isfinite(freshness_age) or freshness_age < 0:
                return Read("invalid", age_s=freshness_age)
        if freshness_age > self.max_age_s:
            return Read("stale", age_s=freshness_age)
        try:
            valid = bool(self.valid(self._value))
        except (TypeError, ValueError, OverflowError):
            valid = False
        return Read("ready", self._value, freshness_age) if valid else Read("invalid", age_s=freshness_age)


def _finite_vector(value: object, length: int) -> bool:
    return (isinstance(value, (tuple, list)) and len(value) == length
            and all(math.isfinite(float(item)) for item in value))


def _scan_valid(value: object) -> bool:
    if not isinstance(value, (tuple, list)) or not value:
        return False
    # +inf is a conventional no-return value in LaserScan; NaN and -inf are
    # rejected. Range bounds remain the eventual driver contract's concern.
    return all(not math.isnan(float(item)) and float(item) >= 0
               for item in value)


def _odom_valid(value: object) -> bool:
    return (isinstance(value, dict) and value.get("frame_id") == "writer/odom"
            and _finite_vector(value.get("pose"), 3))


@dataclass(frozen=True)
class DeployAck:
    success: bool
    beacon_id: int
    message: str


class MockDeployService:
    """Deterministic service double with delay, timeout, rejection and disconnect."""

    def __init__(self, delay_s: float = 0.0, response: DeployAck | None = None) -> None:
        if not math.isfinite(delay_s) or delay_s < 0:
            raise ValueError("delay_s must be finite and nonnegative")
        self.delay_s = delay_s
        self.response = response or DeployAck(True, 1, "mock acknowledged")
        self.connected = True
        self._start: float | None = None
        self._deadline: float | None = None
        self._terminal: str | None = None

    def begin(self, now: float, timeout_s: float) -> str:
        if not self.connected:
            self._terminal = "disconnected"
        elif not math.isfinite(now) or not math.isfinite(timeout_s) or timeout_s <= 0:
            self._terminal = "invalid"
        else:
            self._start = now
            self._deadline = now + timeout_s
            self._terminal = None
        return self.poll(now)

    def poll(self, now: float) -> str:
        if self._terminal:
            return self._terminal
        if not math.isfinite(now) or (self._start is not None and now < self._start):
            self._terminal = "invalid"
        if not self.connected:
            self._terminal = "disconnected"
        elif self._start is None or self._deadline is None:
            self._terminal = "missing"
        elif now >= self._deadline and self.delay_s > self._deadline - self._start:
            self._terminal = "timeout"
        elif now >= self._start + self.delay_s:
            ack = self.response
            if (not isinstance(ack, DeployAck) or not isinstance(ack.success, bool)
                    or not isinstance(ack.beacon_id, int) or ack.beacon_id < 0
                    or (ack.success and ack.beacon_id == 0)
                    or not isinstance(ack.message, str)):
                self._terminal = "invalid"
            else:
                self._terminal = "acknowledged" if ack.success else "rejected"
        return self._terminal or "pending"


class MockWriterHardware:
    """Isolated mock inputs proposed for later adapters; no ROS publishers."""

    def __init__(self, max_age_s: float = 1.0) -> None:
        self.channels = {
            "lidar": TimedChannel(_scan_valid, max_age_s),
            "imu": TimedChannel(lambda v: _finite_vector(v, 6), max_age_s),
            "odometry": TimedChannel(_odom_valid, max_age_s),
            "radio_quality": TimedChannel(
                lambda v: math.isfinite(float(v)) and 0.0 <= float(v) <= 1.0,
                max_age_s),
            "battery_percent": TimedChannel(
                lambda v: math.isfinite(float(v)) and 0.0 <= float(v) <= 100.0,
                max_age_s),
        }
        self.beacon = MockDeployService()
