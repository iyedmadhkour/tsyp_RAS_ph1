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
    e.navigation_uncertain = False
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
    e.scan_is_fresh = lambda now: True
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

    map_to_odom = SimpleNamespace(
        header=SimpleNamespace(stamp=SimpleNamespace(sec=10, nanosec=0)),
        transform=SimpleNamespace(
            translation=SimpleNamespace(x=5.0, y=-2.0),
            rotation=SimpleNamespace(x=0.0, y=0.0, z=math.sqrt(.5), w=math.sqrt(.5))),
    )
    e = SimpleNamespace(
        _now_s=lambda: 10.0,
        get_parameter=lambda _: SimpleNamespace(value=0.5),
        tf_buffer=Mock(), odom_fallback=(2.0, 3.0),
        odom_fallback_stamp_s=9.7, stale_pose_reported=False,
        get_logger=lambda: Mock(),
    )
    def lookup(target, source, _time):
        if source == "writer/base_link":
            raise TransformException("missing base transform")
        return map_to_odom

    e.tf_buffer.lookup_transform.side_effect = lookup
    assert FrontierExplorer.map_pose(e) == pytest.approx((2.0, 0.0))
    e.odom_fallback_stamp_s = 9.49
    assert FrontierExplorer.map_pose(e) is None
    assert e.stale_pose_reported


def test_stale_localization_does_not_start_a_frontier_goal():
    e = SimpleNamespace(
        _now_s=lambda: 10.0, last_tick_s=None, enabled=True, failed=False,
        stuck=False, navigation_uncertain=False, busy=False, map=object(),
        nav=SimpleNamespace(server_is_ready=lambda: True), scan_is_fresh=lambda now: True,
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


def test_stale_localization_cancels_active_goal_without_assuming_it_stopped():
    e = SimpleNamespace(
        active_goal={"x": 8.0, "y": 9.0, "watchdog_reason": None,
                     "cancel_reason": None},
        goal_handle=Mock(), busy=True, navigation_uncertain=False,
        map_pose=lambda: None, angular_z=0.0, feedback_distance=4.0,
        feedback_recoveries=0, watchdog=Mock(), counts=Counter(),
        scan_is_fresh=lambda now: True, _cancel=Mock(), _publish_summary=Mock(),
        get_parameter=lambda _: SimpleNamespace(value=3.0), get_logger=lambda: Mock(),
    )

    FrontierExplorer._update_watchdog(e, 12.0)

    e.watchdog.update.assert_not_called()
    e._cancel.assert_called_once_with("localization unavailable")
    assert e.counts["watchdog_pose_unavailable"] == 1


def test_stale_lidar_cancels_active_goal_and_does_not_assume_stop():
    e = SimpleNamespace(
        active_goal={"x": 8.0, "y": 9.0, "watchdog_reason": None,
                     "cancel_reason": None},
        goal_handle=Mock(), busy=True, navigation_uncertain=False,
        map_pose=lambda: (1.0, 2.0), angular_z=0.0, feedback_distance=4.0,
        feedback_recoveries=0, watchdog=Mock(), counts=Counter(),
        scan_is_fresh=lambda now: False, _cancel=Mock(), _publish_summary=Mock(),
        get_parameter=lambda _: SimpleNamespace(value=3.0), get_logger=lambda: Mock(),
    )
    FrontierExplorer._update_watchdog(e, 12.0)
    e.watchdog.update.assert_not_called()
    e._cancel.assert_called_once_with("LiDAR data stale")
    assert e.counts["watchdog_scan_unavailable"] == 1


@pytest.mark.parametrize("pose,scan_fresh,expected_reason", [
    (None, True, "localization unavailable"),
    ((1.0, 2.0), False, "LiDAR data stale"),
])
def test_fast_safety_monitor_cancels_active_goal_when_inputs_stale(
        pose, scan_fresh, expected_reason):
    e = SimpleNamespace(
        busy=True, active_goal={"cancel_reason": None, "watchdog_reason": None},
        goal_handle=Mock(), navigation_uncertain=False, counts=Counter(),
        _now_s=lambda: 10.0, map_pose=lambda: pose,
        scan_is_fresh=lambda _now: scan_fresh, stale_scan_reported=False,
        get_logger=lambda: Mock(), _cancel=Mock(), _publish_summary=Mock(),
    )

    FrontierExplorer._monitor_navigation_safety(e)

    e._cancel.assert_called_once_with(expected_reason)
    assert e.busy


def test_fast_safety_monitor_latches_stale_pose_reference_between_goals():
    from living_map_writer.pose_integrity import PoseIntegrityMonitor

    integrity = PoseIntegrityMonitor()
    integrity.observe_estimate(1.0, 0.0, 0.0, 0.0)
    integrity.observe_reference(1.0, 1.0, 0.0, 0.0)
    e = SimpleNamespace(
        _now_s=lambda: 10.0, busy=False, active_goal=None,
        pose_integrity_required=True, pose_integrity=integrity,
        pose_integrity_fault=False,
        _pose_integrity_ready=lambda now: integrity.paired_fresh(now, 0.5),
        get_parameter=lambda key: SimpleNamespace(value={
            "pose_integrity_axes_aligned": True,
            "pose_integrity_reference_max_age_s": 0.5,
        }[key]),
        _latch_pose_integrity_fault=Mock(),
    )

    FrontierExplorer._monitor_navigation_safety(e)

    e._latch_pose_integrity_fault.assert_called_once()
    assert "age" in e._latch_pose_integrity_fault.call_args.args[0]


def test_fast_safety_monitor_waits_for_first_pose_pair_during_startup():
    from living_map_writer.pose_integrity import PoseIntegrityMonitor

    integrity = PoseIntegrityMonitor()
    e = SimpleNamespace(
        _now_s=lambda: 10.0, busy=False, active_goal=None,
        pose_integrity_required=True, pose_integrity=integrity,
        pose_integrity_fault=False,
        _pose_integrity_ready=lambda _now: False,
        _latch_pose_integrity_fault=Mock(),
    )

    FrontierExplorer._monitor_navigation_safety(e)

    e._latch_pose_integrity_fault.assert_not_called()


def test_out_of_order_odometry_does_not_replace_the_last_accepted_fallback():
    from nav_msgs.msg import Odometry

    e = object.__new__(FrontierExplorer)
    e.odom_fallback = (1.0, 2.0)
    e.odom_fallback_stamp_s = 10.0
    e.angular_z = 0.0
    e.counts = Counter()
    stale = Odometry()
    stale.header.stamp.sec = 9
    stale.pose.pose.position.x = 90.0
    FrontierExplorer.on_odom(e, stale)
    assert e.odom_fallback == (1.0, 2.0)
    assert e.odom_fallback_stamp_s == 10.0
    assert e.counts["odom_out_of_order"] == 1


def test_invalid_odometry_position_does_not_replace_the_last_fallback():
    from nav_msgs.msg import Odometry

    e = object.__new__(FrontierExplorer)
    e.odom_fallback = (1.0, 2.0)
    e.odom_fallback_stamp_s = 10.0
    e.angular_z = 0.0
    e.counts = Counter()
    msg = Odometry()
    msg.header.stamp.sec = 10
    msg.header.stamp.nanosec = 1
    msg.pose.pose.position.x = math.nan
    FrontierExplorer.on_odom(e, msg)
    assert e.odom_fallback == (1.0, 2.0)
    assert e.odom_fallback_stamp_s == 10.0
    assert e.counts["odom_invalid"] == 1


def test_out_of_order_lidar_does_not_refresh_frozen_scan_age():
    from sensor_msgs.msg import LaserScan

    e = object.__new__(FrontierExplorer)
    e.last_scan_stamp_s = 10.0
    e.counts = Counter()
    e.stale_scan_reported = False
    e.get_parameter = lambda _: SimpleNamespace(value=0.5)
    old = LaserScan()
    old.header.stamp.sec = 9
    FrontierExplorer.on_scan(e, old)
    assert e.last_scan_stamp_s == 10.0
    assert e.counts["scan_out_of_order"] == 1
    assert FrontierExplorer.scan_is_fresh(e, 10.6) is False


def test_stale_home_waits_for_fresh_pose_and_lidar():
    e = SimpleNamespace(
        _now_s=lambda: 10.0, last_tick_s=None, enabled=True, failed=False,
        stuck=False, navigation_uncertain=False, busy=False, map=object(),
        nav=SimpleNamespace(server_is_ready=lambda: True),
        sortie=Sortie(budget_s=600), policy=Mock(), known_area_m2=0.0,
        counts=Counter(), _publish_state=Mock(), stale_scan_reported=False,
        map_pose=lambda: None,
        scan_is_fresh=lambda now: True, _publish_summary=Mock(),
        _send_goal=Mock(), get_logger=lambda: Mock(),
        last_stale_pose_summary_s=None,
    )
    e.sortie.start_return("battery reserve reached", "BATTERY_RESERVE")
    FrontierExplorer.tick(e)
    e._send_goal.assert_not_called()

    e.map_pose = lambda: (0.0, 0.0)
    e.scan_is_fresh = lambda now: False
    FrontierExplorer.tick(e)
    e._send_goal.assert_not_called()


def test_stale_pose_prevents_new_frontier_goal_even_with_fresh_scan():
    e = SimpleNamespace(
        _now_s=lambda: 10.0, last_tick_s=None, enabled=True, failed=False,
        stuck=False, navigation_uncertain=False, busy=False, map=object(),
        nav=SimpleNamespace(server_is_ready=lambda: True),
        sortie=Sortie(budget_s=600), policy=Mock(), known_area_m2=0.0,
        counts=Counter(), _publish_state=Mock(), map_pose=lambda: None,
        scan_is_fresh=lambda now: True, _publish_summary=Mock(),
        _send_goal=Mock(), get_logger=lambda: Mock(),
        last_stale_pose_summary_s=None,
    )
    e.sortie.phase = "EXPLORING"
    FrontierExplorer.tick(e)
    e._send_goal.assert_not_called()
    e.policy.visited_points.assert_not_called()


def test_cancel_service_response_does_not_clear_busy_before_terminal_result():
    cancel_future = Future()
    e = SimpleNamespace(
        active_goal={"x": 1.0, "y": 2.0, "kind": "frontier",
                     "watchdog_reason": None, "cancel_reason": None},
        goal_handle=Mock(), cancel_future=None, busy=True,
        navigation_uncertain=False, navigation_uncertain_reason=None,
        counts=Counter(),
        get_logger=lambda: Mock(), _publish_summary=Mock(), _publish_state=Mock(),
    )
    e._on_cancel_response = lambda future: FrontierExplorer._on_cancel_response(e, future)
    e._latch_navigation_uncertain = lambda reason: (
        FrontierExplorer._latch_navigation_uncertain(e, reason))
    e.goal_handle.cancel_goal_async.return_value = cancel_future
    FrontierExplorer._cancel(e, "test stale input")
    assert e.active_goal["cancel_response"] == "pending"
    assert e.busy
    cancel_future.set_result(SimpleNamespace(goals_canceling=[]))
    assert e.active_goal["cancel_response"] == "not_confirmed"
    assert e.busy
    assert e.navigation_uncertain


def test_missing_cancel_terminal_result_times_out_without_clearing_active_goal(
        monkeypatch):
    now = [10.0]
    monkeypatch.setattr("living_map_writer.frontier_node.time.monotonic",
                        lambda: now[0])
    e = SimpleNamespace(
        active_goal={"x": 1.0, "y": 2.0, "kind": "frontier",
                     "watchdog_reason": None, "cancel_reason": "stale LiDAR",
                     "cancel_sent_at": 10.0, "action_timeout_reported": False},
        goal_handle=Mock(), busy=True, navigation_uncertain=False,
        navigation_uncertain_reason=None, counts=Counter(),
        get_parameter=lambda _: SimpleNamespace(value=3.0),
        get_logger=lambda: Mock(), _publish_summary=Mock(), _publish_state=Mock(),
    )
    e._latch_navigation_uncertain = lambda reason: (
        FrontierExplorer._latch_navigation_uncertain(e, reason))

    now[0] = 12.9
    FrontierExplorer._update_watchdog(e, 100.0)
    assert not e.navigation_uncertain
    assert e.busy

    now[0] = 13.1
    FrontierExplorer._update_watchdog(e, 100.0)
    assert e.navigation_uncertain
    assert e.busy
    assert e.active_goal is not None
    assert e.active_goal["action_timeout_reported"]
    assert e.counts["navigation_result_unknown"] == 1

    FrontierExplorer._update_watchdog(e, 101.0)
    assert e.counts["navigation_result_unknown"] == 1


def test_steady_action_deadline_monitor_latches_without_ros_clock_reads(monkeypatch):
    now = [50.0]
    monkeypatch.setattr("living_map_writer.frontier_node.time.monotonic",
                        lambda: now[0])
    e = SimpleNamespace(
        active_goal={"x": 1.0, "y": 2.0, "kind": "frontier",
                     "cancel_reason": "stale LiDAR", "cancel_sent_at": 50.0,
                     "action_timeout_reported": False},
        goal_handle=Mock(), busy=True, navigation_uncertain=False,
        navigation_uncertain_reason=None, counts=Counter(),
        get_parameter=lambda _: SimpleNamespace(value=3.0),
        get_logger=lambda: Mock(), _publish_summary=Mock(), _publish_state=Mock(),
    )
    e._latch_navigation_uncertain = lambda reason: (
        FrontierExplorer._latch_navigation_uncertain(e, reason))

    # This callback uses time.monotonic directly and never reads ROS/sim time.
    now[0] = 53.1
    FrontierExplorer._monitor_action_response_deadlines(e)

    assert e.navigation_uncertain
    assert e.busy
    assert e.active_goal is not None
    assert "terminal action result" in e.navigation_uncertain_reason


def test_missing_goal_acceptance_response_times_out_conservatively(monkeypatch):
    now = [20.0]
    monkeypatch.setattr("living_map_writer.frontier_node.time.monotonic",
                        lambda: now[0])
    e = SimpleNamespace(
        active_goal={"x": 1.0, "y": 2.0, "kind": "frontier",
                     "watchdog_reason": None, "cancel_reason": None,
                     "goal_response_started_at": 20.0,
                     "action_timeout_reported": False},
        goal_handle=None, busy=True, navigation_uncertain=False,
        navigation_uncertain_reason=None, counts=Counter(),
        get_parameter=lambda _: SimpleNamespace(value=3.0),
        get_logger=lambda: Mock(), _publish_summary=Mock(), _publish_state=Mock(),
    )
    e._latch_navigation_uncertain = lambda reason: (
        FrontierExplorer._latch_navigation_uncertain(e, reason))

    now[0] = 23.1
    FrontierExplorer._update_watchdog(e, 100.0)

    assert e.navigation_uncertain
    assert e.busy
    assert e.active_goal is not None
    assert "acceptance response timed out" in e.navigation_uncertain_reason


def test_late_terminal_result_resolves_cancel_uncertainty_without_failure():
    policy = FrontierPolicy()
    e = _explorer_for_result(policy)
    e.navigation_uncertain = True
    e.navigation_uncertain_reason = "cancel response timed out"
    e.active_goal = {"x": 1.0, "y": 2.0, "kind": "frontier",
                     "watchdog_reason": None, "cancel_reason": "stale LiDAR"}

    FrontierExplorer._done(e, _completed(5), 1.0, 2.0, "frontier")

    assert not e.busy
    assert e.active_goal is None
    assert not e.navigation_uncertain
    assert e.navigation_uncertain_reason is None
    assert e.counts["navigation_uncertainty_resolved"] == 1
    assert e.counts["frontier_failed"] == 0
    assert policy.summary(100.0)["failure_zones"] == 0


@pytest.mark.parametrize("result", [RuntimeError("lost result"), 2, 3, 99])
def test_unknown_or_nonterminal_nav2_result_latches_without_clearing_active_goal(result):
    e = _explorer_for_result(FrontierPolicy())
    e.active_goal = {"x": 1.0, "y": 2.0, "kind": "frontier",
                     "watchdog_reason": None, "cancel_reason": "stale pose"}
    future = Future()
    if isinstance(result, Exception):
        future.set_exception(result)
    else:
        future.set_result(SimpleNamespace(status=result))

    FrontierExplorer._done(e, future, 1.0, 2.0, "frontier")

    assert e.busy
    assert e.goal_handle is not None
    assert e.active_goal is not None
    assert e.navigation_uncertain
    assert e.counts["navigation_result_unknown"] == 1
    assert e.counts["frontier_failed"] == 0


def test_lost_goal_acceptance_response_does_not_assume_goal_was_rejected():
    e = _explorer_for_result(FrontierPolicy())
    e.active_goal = {"x": 1.0, "y": 2.0, "kind": "frontier",
                     "watchdog_reason": None, "cancel_reason": None}
    accepted = Future()
    accepted.set_exception(RuntimeError("acceptance response lost"))

    FrontierExplorer._goal_sent(e, accepted, 1.0, 2.0, "frontier")

    assert e.busy
    assert e.active_goal is not None
    assert e.navigation_uncertain
    assert e.counts["navigation_result_unknown"] == 1


def test_stale_input_cancel_latched_during_goal_acceptance_is_sent_when_accepted():
    cancel_future = Future()
    handle = Mock(accepted=True)
    handle.cancel_goal_async.return_value = cancel_future
    active = {"x": 1.0, "y": 2.0, "kind": "frontier",
              "watchdog_reason": None, "cancel_reason": "LiDAR data stale"}
    e = SimpleNamespace(
        goal_handle=None, active_goal=active, failed=False, stuck=False,
        enabled=True, navigation_uncertain=False,
        sortie=SimpleNamespace(phase="EXPLORING"),
        _on_cancel_response=lambda future: FrontierExplorer._on_cancel_response(e, future),
        _cancel=lambda reason: FrontierExplorer._cancel(e, reason),
        get_logger=lambda: Mock(), _publish_summary=Mock(), _publish_state=Mock(),
    )
    accepted = Future()
    accepted.set_result(handle)

    FrontierExplorer._goal_sent(e, accepted, 1.0, 2.0, "frontier")

    handle.cancel_goal_async.assert_called_once()
    assert active["cancel_reason"] == "LiDAR data stale"
    assert active["cancel_response"] == "pending"
    handle.get_result_async.assert_called_once()


def test_unconfirmed_cancel_response_is_resolved_by_terminal_canceled_result():
    policy = FrontierPolicy()
    e = _explorer_for_result(policy)
    e.active_goal = {"x": 1.0, "y": 2.0, "kind": "frontier",
                     "watchdog_reason": None, "cancel_reason": "stale LiDAR",
                     "cancel_response": "not_confirmed"}

    FrontierExplorer._done(e, _completed(5), 1.0, 2.0, "frontier")

    assert not e.busy
    assert e.active_goal is None
    assert e.counts["frontier_failed"] == 0
    assert policy.counts["cancels"] == 1
    assert policy.summary(100.0)["failure_zones"] == 0


def test_cancel_race_with_succeeded_terminal_result_records_success():
    policy = FrontierPolicy()
    e = _explorer_for_result(policy)
    e.active_goal = {"x": 1.0, "y": 2.0, "kind": "frontier",
                     "watchdog_reason": None, "cancel_reason": "stale LiDAR",
                     "cancel_response": "accepted"}

    FrontierExplorer._done(e, _completed(4), 1.0, 2.0, "frontier")

    assert not e.busy
    assert e.counts["frontier_succeeded"] == 1
    assert e.counts["frontier_failed"] == 0
    assert policy.summary(100.0)["failure_zones"] == 0


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
                        navigation_uncertain=False,
                        active_goal={"x": 2.0, "y": 0.0, "kind": kind,
                                     "watchdog_reason": None, "cancel_reason": None},
                        get_logger=lambda: Mock(), _publish_summary=Mock(),
                        _publish_state=Mock(), _goal_failed=Mock(), _done=Mock())
    e._cancel = lambda reason: FrontierExplorer._cancel(e, reason)
    e._on_cancel_response = lambda future: FrontierExplorer._on_cancel_response(e, future)
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
                        navigation_uncertain=False, goal_handle=None, cancel_future=None,
                        active_goal={"x": 2.0, "y": 0.0, "kind": "frontier",
                                     "watchdog_reason": None, "cancel_reason": None},
                        get_logger=lambda: Mock(), _publish_summary=Mock(),
                        _publish_state=Mock(), _done=Mock())
    e._cancel = lambda reason: FrontierExplorer._cancel(e, reason)
    e._on_cancel_response = lambda future: FrontierExplorer._on_cancel_response(e, future)
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
        navigation_uncertain=False, navigation_uncertain_reason=None,
        cancel_future=None, active_goal=None,
        sortie=sortie or Sortie(budget_s=0), policy=policy,
        counts=Counter(), watchdog=Mock(), known_area_m2=50.0,
        _now_s=lambda: 100.0, _publish_state=Mock(), _publish_summary=Mock(),
        get_logger=lambda: Mock(),
    )
    explorer._goal_failed = lambda x, y, kind, reason="aborted": (
        FrontierExplorer._goal_failed(explorer, x, y, kind, reason))
    explorer._latch_navigation_uncertain = lambda reason, **kwargs: (
        FrontierExplorer._latch_navigation_uncertain(explorer, reason, **kwargs))
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


def test_pose_integrity_fault_remains_latched_after_terminal_cancel_result():
    policy = FrontierPolicy()
    e = _explorer_for_result(policy)
    e.navigation_uncertain = True
    e.navigation_uncertain_reason = "pose integrity disagreement"
    e.pose_integrity_fault = True
    e.active_goal = {"x": 3.0, "y": 4.0, "kind": "frontier",
                     "watchdog_reason": None,
                     "cancel_reason": "pose integrity disagreement"}

    FrontierExplorer._done(e, _completed(5), 3.0, 4.0, "frontier")

    assert e.navigation_uncertain
    assert e.pose_integrity_fault
    assert e.counts["pose_integrity_goal_cancelled"] == 1
    assert e.counts["navigation_uncertainty_resolved"] == 0
    assert policy.zones == []  # integrity lockout is not a frontier failure


def test_required_independent_pose_reference_fails_closed_before_goal():
    from living_map_writer.pose_integrity import PoseIntegrityMonitor

    e = SimpleNamespace(
        pose_integrity_required=True,
        pose_integrity=PoseIntegrityMonitor(),
        get_parameter=lambda key: SimpleNamespace(value={
            "pose_integrity_axes_aligned": True,
            "pose_integrity_reference_max_age_s": 0.5,
        }[key]),
    )
    assert not FrontierExplorer._pose_integrity_ready(e, 10.0)


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
    e.navigation_uncertain = False
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
    e._latch_navigation_uncertain = lambda reason: (
        FrontierExplorer._latch_navigation_uncertain(e, reason))

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
