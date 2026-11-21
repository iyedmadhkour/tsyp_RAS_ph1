import math
from types import SimpleNamespace

import pytest

from living_map_writer.goal_watchdog import GoalWatchdog, WatchdogConfig
from living_map_writer.demo_fallback import DEMO_WAYPOINTS, GAS_PASSAGE_WAYPOINT
from living_map_writer.node import Writer
from living_map_writer.progress import StuckDetector


@pytest.mark.parametrize("kwargs", [
    {"window_s": math.nan}, {"min_progress_m": math.inf},
    {"linear_cmd_eps": math.nan}, {"linear_cmd_eps": 0.0},
    {"angular_cmd_eps": math.inf}, {"angular_cmd_eps": 0.0},
    {"min_rotation_ratio": math.nan}, {"min_rotation_ratio": 0.0},
    {"min_rotation_ratio": 1.1},
    {"yaw_noise_rad": -0.1}, {"cmd_timeout_s": math.inf},
    {"odom_timeout_s": 0.0}, {"max_step_s": math.nan},
    {"max_speed_mps": -1.0},
])
def test_detector_rejects_invalid_thresholds_and_timeouts(kwargs):
    with pytest.raises(ValueError):
        StuckDetector(**kwargs)


def test_command_buffer_stays_bounded_when_odometry_stops():
    detector = StuckDetector()
    detector.set_active(True)
    detector.odometry(0.0, 0.0, 0.0, 0.0)
    for i in range(1, 2001):
        stamp = i * 0.05
        detector.command(stamp, 0.5, 0.0)

    # Command history outside an attributable odometry interval is discarded;
    # only a bounded, recent interval can remain queued.
    assert len(detector._changes) <= 21
    assert detector.command_state(102.0) == "stale"


def test_long_odometry_gap_is_not_attributed_to_a_command_or_clears_evidence():
    detector = StuckDetector(window_s=10.0)
    detector.set_active(True)
    detector.odometry(0.0, 0.0, 0.0, 0.0)
    detector.command(0.0, 0.5, 0.0)
    detector.odometry(0.5, 0.0, 0.0, 0.0)
    detector.evaluate(0.5)
    detector.command(20.0, 0.5, 0.0)
    detector.odometry(20.0, 0.0, 0.0, 0.0)
    detector.evaluate(20.0)

    assert detector.evidence_s == pytest.approx(0.5)
    assert detector.odom_stale(20.0) is False


def test_watchdog_cancels_stationary_goal_before_integrated_physical_detector():
    detector = StuckDetector(window_s=45.0)
    detector.set_active(True)
    detector.odometry(0.0, 0.0, 0.0, 0.0)
    watchdog = GoalWatchdog(WatchdogConfig())
    watchdog.start("frontier", 5.0, 0.0, 0.0, distance_remaining=5.0)

    verdict = None
    stuck_at = None
    for tick in range(1, 100):
        now = tick * 0.5
        detector.command(now, 0.5, 0.0)
        detector.odometry(now, 0.0, 0.0, 0.0)
        verdict = watchdog.update(now, 5.0, 0, 0.0, 0.0, False)
        if verdict is not None:
            detector.set_active(False)  # cancellation exits NAVIGATING in Writer
            break
        if detector.evaluate(now):
            stuck_at = now
            break

    assert verdict == "stalled"
    assert stuck_at is None
    assert now < 45.0


def test_repeated_watchdog_cancellations_preserve_stall_evidence_between_goals():
    detector = StuckDetector(window_s=45.0)
    writer = SimpleNamespace(enabled=True, failed=False, stuck=False,
                             explorer_state="NAVIGATING", detector=detector)
    writer._detector_active = lambda: Writer._detector_active(writer)
    now = 0.0
    physical_stuck_at = None

    for goal_index in range(3):
        writer.explorer_state = "NAVIGATING"
        Writer._sync_detector(writer)
        detector.odometry(now, 0.0, 0.0, 0.0)
        watchdog = GoalWatchdog(WatchdogConfig())
        watchdog.start("frontier", 5.0, 0.0, now, distance_remaining=5.0)

        for _ in range(60):
            now += 0.5
            detector.command(now, 0.5, 0.0)
            detector.odometry(now, 0.0, 0.0, 0.0)
            verdict = watchdog.update(now, 5.0, 0, 0.0, 0.0, False)
            if detector.evaluate(now):
                physical_stuck_at = now
                break
            if verdict is not None:
                assert verdict == "stalled"
                writer.explorer_state = "SEARCHING"
                Writer._sync_detector(writer)
                now += 1.0  # bounded selector interval before the next goal
                break
        if physical_stuck_at is not None:
            break

    assert physical_stuck_at is not None
    assert physical_stuck_at < 45.0 * 2
    assert goal_index == 1  # second stalled goal, before its own 25 s watchdog timeout


def test_battery_return_keeps_detection_active_and_preserves_evidence():
    detector = StuckDetector(window_s=45.0)
    writer = SimpleNamespace(enabled=True, failed=False, stuck=False,
                             explorer_state="NAVIGATING", detector=detector)
    writer._detector_active = lambda: Writer._detector_active(writer)
    Writer._sync_detector(writer)
    detector.evidence_s = 12.0

    writer.explorer_state = "RETURNING"  # battery reserve requested HOME
    assert Writer._sync_detector(writer)
    assert detector.active
    assert detector.evidence_s == 12.0


def test_operator_pause_and_emergency_states_clear_carried_evidence():
    detector = StuckDetector(window_s=45.0)
    writer = SimpleNamespace(enabled=True, failed=False, stuck=False,
                             explorer_state="SEARCHING", detector=detector)
    writer._detector_active = lambda: Writer._detector_active(writer)
    detector.evidence_s = 20.0
    Writer._sync_detector(writer)
    assert detector.evidence_s == 20.0  # short selector interval preserves it

    writer.enabled = False
    writer.explorer_state = "PAUSED"
    Writer._sync_detector(writer)
    assert detector.evidence_s == 0.0

    writer.enabled = True
    writer.explorer_state = "NAVIGATING"
    Writer._sync_detector(writer)
    detector.evidence_s = 20.0
    writer.failed = True
    Writer._sync_detector(writer)
    assert detector.evidence_s == 0.0


def test_idle_odom_and_commands_do_not_trigger_stuck_but_stationary_drive_does():
    detector = StuckDetector(window_s=3.0)
    detector.set_active(True)
    detector.set_active(False)  # intentionally paused: no physical STUCK evidence
    for tick in range(1, 8):
        now = tick * 0.5
        detector.command(now, 0.0, 0.0)
        detector.odometry(now, 0.0, 0.0, 0.0)
        assert not detector.evaluate(now)

    detector.set_active(True)
    detector.odometry(4.0, 0.0, 0.0, 0.0)
    stuck = False
    for tick in range(8, 16):
        now = tick * 0.5
        detector.command(now, 0.5, 0.0)
        detector.odometry(now, 0.0, 0.0, 0.0)
        stuck = detector.evaluate(now)
        if stuck:
            break
    assert stuck


def test_demo_route_crosses_gas_divider_through_footprint_safe_opening_both_ways():
    wall5_east_edge = 14.75  # world x=3.75, translated by the Writer spawn offset
    wall7_west_edge = 15.825  # world x=4.825, translated by the Writer spawn offset
    footprint_length, footprint_width = 0.63, 0.465

    def crossing_x_bounds(start, end, y):
        fraction = (y - start[1]) / (end[1] - start[1])
        x = start[0] + fraction * (end[0] - start[0])
        heading = math.atan2(end[1] - start[1], end[0] - start[0])
        half_extent_x = (abs(math.cos(heading)) * footprint_length / 2
                         + abs(math.sin(heading)) * footprint_width / 2 + 0.01)
        return x - half_extent_x, x + half_extent_x

    crossings = [(DEMO_WAYPOINTS[4], GAS_PASSAGE_WAYPOINT),
                 (GAS_PASSAGE_WAYPOINT, DEMO_WAYPOINTS[8])]
    for start, end in crossings:
        for y in (1.325, 1.5, 1.675):
            west, east = crossing_x_bounds(start, end, y)
            assert west > wall5_east_edge
            assert east < wall7_west_edge
