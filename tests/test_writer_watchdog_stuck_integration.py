"""Deterministic interaction tests for Task 6 and the current STUCK detector.

These tests compose the pure Python policies with a fake navigation clock. They
do not integrate the unmerged command/odometry PR or emulate DDS delivery.
"""
from living_map_writer.goal_watchdog import GoalWatchdog, WatchdogConfig
from living_map_writer.progress import StuckDetector


def _run_goal(kind="frontier", *, rotating=False, recoveries=0,
              progress_every_s=None, physical_window=45.0):
    watchdog = GoalWatchdog(WatchdogConfig())
    physical = StuckDetector(window_s=physical_window)
    watchdog.start(kind, 10.0, 0.0, 0.0, distance_remaining=10.0)
    verdict = None
    stuck_at = None
    remaining = 10.0
    x = 0.0
    for second in range(0, 181):
        if progress_every_s and second and second % progress_every_s == 0:
            remaining = max(0.1, remaining - 0.4)
            x += 0.4
            if remaining <= 0.25:
                return None, None, watchdog, physical
        verdict = watchdog.update(float(second), remaining, recoveries,
                                  x, 0.0, rotating)
        # The Writer supervisor's detector only receives active driving states.
        # A completed/cancelled navigation goal leaves NAVIGATING and resets it.
        stuck = physical.update(float(second), x, 0.0, driving=verdict is None)
        if stuck:
            stuck_at = second
            break
        if verdict is not None:
            break
    return verdict, stuck_at, watchdog, physical


def test_commanded_but_stationary_frontier_is_cancelled_before_stuck():
    verdict, stuck_at, _, _ = _run_goal()
    assert verdict == "stalled"
    assert stuck_at is None


def test_intentional_pause_resets_stuck_window():
    detector = StuckDetector(window_s=5.0)
    for t in range(4):
        assert not detector.update(float(t), 0.0, 0.0, driving=True)
    for t in range(4, 20):
        assert not detector.update(float(t), 0.0, 0.0, driving=False)
    assert detector.evidence_s == 0.0
    assert not detector.active


def test_long_rotation_is_tolerated_then_watchdog_cancels_before_stuck():
    verdict, stuck_at, _, _ = _run_goal(rotating=True)
    assert verdict == "stalled"
    assert stuck_at is None


def test_short_nav2_recovery_followed_by_progress_is_not_stuck():
    verdict, stuck_at, _, physical = _run_goal(recoveries=2, progress_every_s=5)
    assert verdict is None
    assert stuck_at is None
    assert physical.evidence_s < physical.window_s


def test_home_progress_does_not_trigger_either_detector():
    verdict, stuck_at, _, _ = _run_goal(kind="home", progress_every_s=5)
    assert verdict is None
    assert stuck_at is None


def test_true_immobility_during_return_becomes_stuck_before_home_timeout():
    verdict, stuck_at, watchdog, _ = _run_goal(kind="home")
    assert verdict is None
    assert stuck_at == 45
    assert stuck_at < watchdog.cfg.home_stall_timeout_s


def test_watchdog_falls_back_to_pose_when_nav_feedback_is_missing():
    watchdog = GoalWatchdog(WatchdogConfig())
    watchdog.start("frontier", 5.0, 0.0, 0.0)
    results = [watchdog.update(float(t), None, 0, 0.0, 0.0, False)
               for t in range(0, 27)]
    assert results[-1] == "stalled"
    assert watchdog.source == "pose"


def test_delayed_odometry_and_command_expiry_are_exposed_by_integrated_detector():
    detector = StuckDetector()
    detector.set_active(True)
    detector.command(0.0, 0.5, 0.0)
    assert detector.command_state(2.0) == "stale"
    assert detector.odom_stale(2.0)
