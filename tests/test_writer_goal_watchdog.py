"""Deterministic tests for the navigation-progress watchdog (not STUCK)."""
from pathlib import Path

import pytest
import yaml

from living_map_writer.goal_watchdog import GoalWatchdog, WatchdogConfig
from living_map_writer.progress import StuckDetector


def watchdog(**overrides):
    values = dict(progress_window_s=5.0, min_progress_m=0.25,
                  rotation_grace_s=3.0, stall_timeout_s=10.0,
                  goal_timeout_s=40.0, home_stall_timeout_s=20.0,
                  home_timeout_s=60.0, max_recoveries=2, reset_jump_m=1.0)
    values.update(overrides)
    return GoalWatchdog(WatchdogConfig(**values))


def test_normal_motion_never_trips():
    w = watchdog()
    w.start("frontier", 10.0, 0.0, now=0.0, distance_remaining=10.0)
    verdicts = [w.update(t, 10.0 - 0.3 * t, 0, 0.3 * t, 0.0, False)
                for t in range(0, 31)]
    assert verdicts == [None] * 31


def test_rotation_in_place_is_tolerated_for_the_grace_period():
    w = watchdog()
    w.start("frontier", 10.0, 0.0, now=0.0, distance_remaining=10.0)
    assert [w.update(t, 10.0, 0, 0.0, 0.0, True) for t in range(0, 4)] == [None] * 4


def test_a_long_spin_does_not_mask_a_stall():
    w = watchdog()
    w.start("frontier", 10.0, 0.0, now=0.0, distance_remaining=10.0)
    for t in range(0, 20):
        verdict = w.update(t, 10.0, 0, 0.0, 0.0, True)
    assert verdict == "stalled"


def test_brief_wait_then_motion_is_accepted():
    w = watchdog()
    w.start("frontier", 10.0, 0.0, now=0.0, distance_remaining=10.0)
    for t in range(0, 9):
        assert w.update(t, 10.0, 0, 0.0, 0.0, False) is None
    assert w.update(9.0, 9.5, 0, 0.5, 0.0, False) is None
    assert w.update(10.0, 9.0, 0, 1.0, 0.0, False) is None


def test_genuinely_stalled_goal_is_reported():
    w = watchdog()
    w.start("frontier", 10.0, 0.0, now=0.0, distance_remaining=10.0)
    assert w.update(11.0, 10.0, 0, 0.0, 0.0, False) == "stalled"


def test_default_navigation_watchdog_cancels_before_physical_stuck_detector():
    config = yaml.safe_load((Path(__file__).resolve().parents[1]
                             / "config/nav2/writer.yaml").read_text())
    stall_s = config["/writer/frontier_explorer"]["ros__parameters"][
        "nav_watchdog_stall_s"]
    watchdog = GoalWatchdog(WatchdogConfig(stall_timeout_s=stall_s))
    physical_stuck = StuckDetector()
    watchdog.start("frontier", 5.0, 0.0, now=0.0, distance_remaining=5.0)

    verdict = None
    for rotating in (False, True):
        watchdog.start("frontier", 5.0, 0.0, now=0.0, distance_remaining=5.0)
        physical_stuck.reset()
        for second in range(0, 46):
            verdict = watchdog.update(second, 5.0, 0, 0.0, 0.0, rotating)
            # The action is no longer active once the navigation watchdog cancels.
            stuck = physical_stuck.update(second, 0.0, 0.0, driving=verdict is None)
            if verdict is not None:
                assert verdict == "stalled"
                assert second < physical_stuck.window_s
                assert not stuck
                break
            assert not stuck
        else:
            pytest.fail("navigation watchdog did not preempt the physical STUCK detector")


def test_subthreshold_progress_does_not_accumulate_across_observation_windows():
    w = GoalWatchdog()
    w.start("frontier", 10.0, 0.0, now=0.0, distance_remaining=10.0)
    verdicts = [w.update(t, 10.0 - 0.01 * t, 0, 0.01 * t, 0.0, False)
                for t in range(0, 26)]
    assert verdicts[-1] == "stalled"


def test_repeated_recoveries_without_progress_cancel_the_goal():
    w = watchdog()
    w.start("frontier", 10.0, 0.0, now=0.0, distance_remaining=10.0)
    assert w.update(5.0, 10.0, 2, 0.0, 0.0, False) == "recovery_limit"


def test_recoveries_during_real_progress_do_not_cancel_the_goal():
    w = watchdog()
    w.start("frontier", 10.0, 0.0, now=0.0, distance_remaining=10.0)
    for t in range(0, 16):
        assert w.update(t, 10.0 - 0.5 * t, 3, 0.5 * t, 0.0, False) is None


def test_goal_timeout_is_bounded_even_while_making_progress():
    w = watchdog()
    w.start("frontier", 10.0, 0.0, now=0.0, distance_remaining=10.0)
    assert w.update(41.0, 5.0, 0, 0.0, 0.0, False) == "timeout"


def test_pose_jump_from_a_map_correction_is_not_a_false_stall():
    w = watchdog()
    w.start("frontier", 20.0, 0.0, now=0.0)
    for t in range(0, 5):
        assert w.update(t, None, 0, 12.0, 0.0, False) is None  # 8 m away
    assert w.update(5.0, None, 0, 0.0, 0.0, False) is None      # 8 -> 20 m jump
    assert w.jumps == 1
    assert w.update(9.0, None, 0, 3.0, 0.0, False) is None      # 17 m: progress


def test_home_uses_longer_stall_and_timeout_limits():
    w = watchdog()
    w.start("home", 0.0, 0.0, now=0.0, distance_remaining=10.0)
    assert w.update(15.0, 10.0, 0, 0.0, 0.0, False) is None
    assert w.update(21.0, 10.0, 0, 0.0, 0.0, False) == "stalled"

    w = watchdog()
    w.start("home", 0.0, 0.0, now=0.0, distance_remaining=10.0)
    assert w.update(61.0, 5.0, 0, 0.0, 0.0, False) == "timeout"


@pytest.mark.parametrize("kwargs", [
    dict(progress_window_s=0), dict(min_progress_m=0),
    dict(stall_timeout_s=1, progress_window_s=5),
    dict(max_recoveries=0), dict(reset_jump_m=0),
])
def test_invalid_configuration_is_rejected(kwargs):
    with pytest.raises(ValueError):
        WatchdogConfig(**kwargs)
