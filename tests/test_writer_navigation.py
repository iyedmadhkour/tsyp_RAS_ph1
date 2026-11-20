from concurrent.futures import Future
from collections import Counter
import math
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

pytest.importorskip("rclpy")
pytest.importorskip("nav2_msgs.action")
from living_map_writer.frontier_node import FrontierExplorer
from living_map_writer.frontier_policy import FrontierPolicy
from living_map_writer.sortie import Sortie
from living_map_writer.entrance import padded_footprint
from living_map_writer.pose_freshness import pose_stamp_is_fresh


@pytest.mark.parametrize("wall_gap", [0.40, 0.80])
def test_frontier_selection_leaves_room_for_the_padded_robot_to_turn(wall_gap):
    """A free frontier in a wall-side strip is not necessarily a safe goal."""
    from nav_msgs.msg import OccupancyGrid
    footprint = padded_footprint("[[0.35,0.32],[0.35,-0.32],[-0.35,-0.32],[-0.35,0.32]]", .01)
    radius = max(math.hypot(x, y) for x, y in footprint)
    grid = OccupancyGrid()
    grid.info.width, grid.info.height, grid.info.resolution = 80, 40, 0.05
    # The map currently knows a strip above a wall, with unknown space beyond.
    # The 0.4 m strip admits center-only goals but not the rotation envelope.
    strip_rows = round(wall_gap / grid.info.resolution)
    grid.data = [100 if y == 0 else (0 if y <= strip_rows else -1)
                 for y in range(40) for x in range(80)]
    e = object.__new__(FrontierExplorer)
    e._now_s = lambda: 0.
    e.last_tick_s = None
    e.enabled = True
    e.failed = False
    e.stuck = False
    e.busy = False
    e.map = grid
    e.sortie = Sortie(budget_s=0)
    e.policy = FrontierPolicy()
    e.watchdog = Mock()
    e.counts = Counter()
    e.known_area_m2 = 0.0
    e.published_state = ""
    e.state_pub = Mock()
    e._publish_state = Mock()
    e.nav = Mock()
    e.map_pose = lambda: (2., wall_gap)
    e.minimum_goal_x = -1 + radius + .25
    e.frontier_clearance = radius
    e.get_parameter = lambda key: SimpleNamespace(value={
        "min_cluster_cells": 5, "entrance_x": -1.}[key])
    e._send_goal = Mock()
    e.get_logger = lambda: Mock()
    FrontierExplorer.tick(e)
    assert e._send_goal.called is (wall_gap > radius)
    if e._send_goal.called:
        _, y, kind = e._send_goal.call_args.args
        assert y - .05 >= radius
        assert kind == "frontier"


@pytest.mark.parametrize("stamp,expected", [
    (9.5, True), (9.5001, True), (9.4999, False), (10.0, True),
    (10.001, False), (8.0, False), (math.nan, False),
])
def test_pose_timestamp_freshness_rejects_old_future_and_nonfinite(stamp, expected):
    assert pose_stamp_is_fresh(10.0, stamp, 0.5) is expected
    assert not pose_stamp_is_fresh(10.0, stamp, 0.0)


def test_stale_transform_falls_back_only_to_fresh_odometry():
    from tf2_ros import TransformException

    e = SimpleNamespace(
        _now_s=lambda: 10.0,
        get_parameter=lambda _: SimpleNamespace(value=0.5),
        tf_buffer=Mock(), odom_fallback=(2.0, 3.0),
        odom_fallback_stamp_s=9.7, stale_pose_reported=False,
        get_logger=lambda: Mock(),
    )
    e.tf_buffer.lookup_transform.side_effect = TransformException("missing transform")
    assert FrontierExplorer.map_pose(e) == (2.0, 3.0)
    e.odom_fallback_stamp_s = 9.49
    assert FrontierExplorer.map_pose(e) is None
    assert e.stale_pose_reported


def test_stale_localization_does_not_start_a_frontier_goal():
    e = SimpleNamespace(
        _now_s=lambda: 10.0, last_tick_s=None, enabled=True, failed=False,
        stuck=False, busy=False, map=object(), nav=SimpleNamespace(server_is_ready=lambda: True),
        sortie=Sortie(budget_s=600), policy=Mock(), known_area_m2=0.0,
        counts=Counter(), _publish_state=Mock(), map_pose=lambda: None,
        _publish_summary=Mock(), _send_goal=Mock(), get_logger=lambda: Mock(),
        last_stale_pose_summary_s=None,
    )
    e.sortie.phase = "EXPLORING"
    FrontierExplorer.tick(e)
    assert e.counts["pose_unavailable"] == 1
    e._send_goal.assert_not_called()
    e.policy.visited_points.assert_not_called()


def test_stale_localization_does_not_break_active_goal_watchdog():
    e = SimpleNamespace(
        active_goal={"x": 8.0, "y": 9.0, "watchdog_reason": None,
                     "cancel_reason": None, "last_watchdog_pose": (2.0, 3.0)},
        map_pose=lambda: None, angular_z=0.0, feedback_distance=4.0,
        feedback_recoveries=0, watchdog=Mock(), counts=Counter(),
        _cancel=Mock(), get_logger=lambda: Mock(),
    )
    e.watchdog.update.return_value = None

    FrontierExplorer._update_watchdog(e, 12.0)

    e.watchdog.update.assert_called_once_with(12.0, 4.0, 0, 2.0, 3.0, False)
    assert e.counts["watchdog_pose_unavailable"] == 1


def test_transit_doorway_does_not_require_in_place_rotation():
    from living_map_writer.frontier import choose_frontier
    width, height, res = 100, 60, .05
    data = [100] * (width * height)
    for y in range(1, height - 1):
        for x in range(1, width - 1):
            # Two rooms joined by a 0.85 m doorway. A safe distant frontier
            # must remain available even though spinning in the door is unsafe.
            data[y * width + x] = 100 if x == 50 and not 22 <= y <= 38 else 0
    for y in range(15, 45):
        for x in range(95, 100):
            data[y * width + x] = -1
    goal = choose_frontier(data, width, height, res, 0, 0, 1., 1.5,
                           goal_clearance=.49 + res / math.sqrt(2))
    assert goal is not None and goal[1] > 4.
    assert choose_frontier(data, width, height, res, 0, 0, 1., 1.5,
                           clearance=.49 + res / math.sqrt(2)) is None


@pytest.mark.parametrize("kind,return_requested,expected_cancel", [
    ("frontier", True, True),
    ("frontier", False, False),
    ("home", True, False),
])
def test_return_during_pending_goal_acceptance(kind, return_requested, expected_cancel):
    """The return callback can run before send_goal_async supplies a handle."""
    e = SimpleNamespace(sortie=Sortie(budget_s=0), goal_handle=None,
                        failed=False, stuck=False, enabled=True, busy=True,
                        get_logger=lambda: Mock(), _goal_failed=Mock(), _done=Mock())
    e._cancel = lambda reason: FrontierExplorer._cancel(e, reason)
    if return_requested:
        FrontierExplorer.on_return_request(e, SimpleNamespace(data=True))
    handle = Mock(accepted=True)
    accepted = Future()
    accepted.set_result(handle)
    FrontierExplorer._goal_sent(e, accepted, 2.0, 0.0, kind)
    assert handle.cancel_goal_async.called is expected_cancel
    # Cancellation must still drain the action result before another goal starts.
    handle.get_result_async.assert_called_once()
    assert e.busy


@pytest.mark.parametrize("flag", ["failed", "stuck", "paused"])
def test_pending_goal_acceptance_respects_emergency_and_pause(flag):
    e = SimpleNamespace(sortie=Sortie(), failed=flag == "failed",
                        stuck=flag == "stuck", enabled=flag != "paused",
                        _done=Mock())
    handle = Mock(accepted=True)
    accepted = Future()
    accepted.set_result(handle)
    FrontierExplorer._goal_sent(e, accepted, 2.0, 0.0, "frontier")
    handle.cancel_goal_async.assert_called_once()
    handle.get_result_async.assert_called_once()


def _completed(status):
    future = Future()
    future.set_result(SimpleNamespace(status=status))
    return future


def _explorer_for_result(policy, sortie=None):
    explorer = SimpleNamespace(
        busy=True, goal_handle=Mock(), failed=False, stuck=False, enabled=True,
        active_goal=None, sortie=sortie or Sortie(budget_s=0), policy=policy,
        counts=Counter(), watchdog=Mock(), known_area_m2=50.0,
        _now_s=lambda: 100.0, _publish_state=Mock(), _publish_summary=Mock(),
        get_logger=lambda: Mock(),
    )
    explorer._goal_failed = lambda x, y, kind, reason="aborted": (
        FrontierExplorer._goal_failed(explorer, x, y, kind, reason))
    return explorer


def test_external_cancellation_is_not_recorded_as_a_frontier_failure():
    policy = FrontierPolicy()
    e = _explorer_for_result(policy)
    e.active_goal = {"x": 3.0, "y": 4.0, "kind": "frontier",
                     "watchdog_reason": None,
                     "cancel_reason": "battery reserve reached; heading home"}

    FrontierExplorer._done(e, _completed(5), 3.0, 4.0, "frontier")

    assert policy.zones == []
    assert e.counts["frontier_cancelled"] == 1
    assert e.sortie.phase == "EXPLORING"


def test_unexplained_frontier_cancellation_is_paced_without_becoming_failure():
    """External cancellation cannot immediately resubmit the same frontier."""
    policy = FrontierPolicy()
    e = _explorer_for_result(policy)
    e.active_goal = {"x": 3.0, "y": 4.0, "kind": "frontier",
                     "watchdog_reason": None, "cancel_reason": None}

    FrontierExplorer._done(e, _completed(5), 3.0, 4.0, "frontier")

    assert len(policy.zones) == 1
    assert policy.zones[0].attempts == 0
    assert policy.zones[0].cooldown_until == 115.0
    assert e.counts["frontier_cancelled"] == 1
    assert policy.counts["cancel_cancelled"] == 1
    assert policy.earliest_retry_s(100.0) == 115.0
    assert policy.blacklist(114.9) == [(3.0, 4.0)]
    assert policy.blacklist(115.0) == []  # eligible for selection after cooldown
    assert policy.summary()["failure_zones"] == 0
    assert policy.summary()["cancellation_only_zones"] == 1
    assert policy.summary(100.0)["next_retry_at_s"] == 115.0


def test_watchdog_cancellation_is_recorded_as_a_frontier_failure():
    policy = FrontierPolicy()
    e = _explorer_for_result(policy)
    e.active_goal = {"x": 3.0, "y": 4.0, "kind": "frontier",
                     "watchdog_reason": "stalled",
                     "cancel_reason": "navigation watchdog: stalled"}

    FrontierExplorer._done(e, _completed(5), 3.0, 4.0, "frontier")

    assert len(policy.zones) == 1
    assert policy.zones[0].attempts == 1
    assert e.counts["frontier_failed"] == 1
    assert e.sortie.phase == "EXPLORING"


def test_successful_frontier_goal_records_a_visit_not_a_failure():
    policy = FrontierPolicy()
    e = _explorer_for_result(policy)
    e.active_goal = {"x": 3.0, "y": 4.0, "kind": "frontier",
                     "watchdog_reason": None, "cancel_reason": None}

    FrontierExplorer._done(e, _completed(4), 3.0, 4.0, "frontier")

    assert policy.zones == []
    assert len(policy.visits) == 1
    assert policy.visits[0].count == 1
    assert e.counts["frontier_succeeded"] == 1


def test_aborted_frontier_goal_records_a_bounded_failure():
    policy = FrontierPolicy()
    e = _explorer_for_result(policy)
    e.active_goal = {"x": 3.0, "y": 4.0, "kind": "frontier",
                     "watchdog_reason": None, "cancel_reason": None}

    FrontierExplorer._done(e, _completed(6), 3.0, 4.0, "frontier")

    assert len(policy.zones) == 1
    assert policy.zones[0].attempts == 1
    assert e.counts["frontier_failed"] == 1


def test_nav2_server_unavailable_does_not_send_a_goal_or_move_the_sortie():
    e = object.__new__(FrontierExplorer)
    e.enabled, e.failed, e.stuck, e.busy = True, False, False, False
    e.sortie = Sortie(budget_s=0)
    e.last_tick_s = None
    e.known_area_m2 = 0.0
    e._now_s = lambda: 1.0
    e._publish_state = Mock()
    e.policy = Mock()
    e.nav = Mock()
    e.nav.server_is_ready.return_value = False
    e.map = object()

    FrontierExplorer.tick(e)

    e.nav.send_goal_async.assert_not_called()
    assert e.sortie.phase == "EXPLORING"
    e.policy.refresh.assert_called_once()


def test_home_navigation_failures_are_bounded_and_set_return_failed():
    sortie = Sortie(budget_s=0, max_return_attempts=2)
    sortie.start_return("battery reserve reached", "BATTERY_RESERVE")
    e = SimpleNamespace(sortie=sortie, counts=Counter(), watchdog=Mock(),
                        active_goal=None, _now_s=lambda: 10.0,
                        get_logger=lambda: Mock(), _publish_summary=Mock())
    e._goal_failed = lambda x, y, kind, reason="aborted": (
        FrontierExplorer._goal_failed(e, x, y, kind, reason))

    FrontierExplorer._goal_failed(e, 0.0, 0.0, "home", "aborted")
    assert sortie.phase == "RETURNING"
    FrontierExplorer._goal_failed(e, 0.0, 0.0, "home", "aborted")

    assert sortie.phase == "RETURN_FAILED"
    assert sortie.reason_code == "RETURN_FAILED"
    assert e.counts["home_failed"] == 2


def test_unexplained_home_cancellation_consumes_bounded_return_attempts():
    sortie = Sortie(budget_s=0, max_return_attempts=2)
    sortie.start_return("battery reserve reached", "BATTERY_RESERVE")
    e = _explorer_for_result(FrontierPolicy(), sortie)
    e.active_goal = {"x": 0.0, "y": 0.0, "kind": "home",
                     "watchdog_reason": None, "cancel_reason": None}

    FrontierExplorer._done(e, _completed(5), 0.0, 0.0, "home")
    assert sortie.phase == "RETURNING"
    assert sortie.return_attempts_failed == 1
    assert e.counts["home_failed"] == 1
    assert e.counts["home_cancelled"] == 0

    e.active_goal = {"x": 0.0, "y": 0.0, "kind": "home",
                     "watchdog_reason": None, "cancel_reason": None}
    FrontierExplorer._done(e, _completed(5), 0.0, 0.0, "home")
    assert sortie.phase == "RETURN_FAILED"
    assert sortie.reason_code == "RETURN_FAILED"
