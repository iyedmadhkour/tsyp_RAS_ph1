from concurrent.futures import Future
import math
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

pytest.importorskip("rclpy")
pytest.importorskip("nav2_msgs.action")
from living_map_writer.frontier_node import FrontierExplorer
from living_map_writer.sortie import Sortie
from living_map_writer.entrance import padded_footprint


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
    e = SimpleNamespace(
        _now_s=lambda: 0., last_tick_s=None, enabled=True, failed=False,
        stuck=False, busy=False, map=grid, sortie=Sortie(budget_s=0),
        _publish_state=Mock(), nav=Mock(), map_pose=lambda: (2., wall_gap),
        minimum_goal_x=-1 + radius + .25, frontier_clearance=radius,
        blacklist=[], visited=[],
        get_parameter=lambda key: SimpleNamespace(value={
            "min_cluster_cells": 5, "entrance_x": -1.}[key]),
        _send_goal=Mock(), get_logger=lambda: Mock())
    FrontierExplorer.tick(e)
    assert e._send_goal.called is (wall_gap > radius)
    if e._send_goal.called:
        _, y, kind = e._send_goal.call_args.args
        assert y - .05 >= radius
        assert kind == "frontier"


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
