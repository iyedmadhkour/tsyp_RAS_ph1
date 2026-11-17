import math
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

import pytest
import yaml

from living_map_writer.entrance import (
    entrance_mask, footprint_min_x, padded_footprint, rotation_safe_center_x,
)
from living_map_writer.frontier import choose_frontier

ROOT = Path(__file__).resolve().parents[1]
CONFIG = yaml.safe_load((ROOT / "config/nav2/writer.yaml").read_text())
PARAMS = CONFIG["/writer/frontier_explorer"]["ros__parameters"]
FOOTPRINT = padded_footprint(PARAMS["footprint"], PARAMS["footprint_padding"])


@pytest.mark.parametrize("yaw", [0, math.pi / 2, math.pi, -math.pi / 2, math.pi / 4])
def test_containment_uses_rotated_vertices_not_robot_center(yaw):
    support = -footprint_min_x(0, yaw, FOOTPRINT)
    assert footprint_min_x(-1 + support, yaw, FOOTPRINT) == pytest.approx(-1)
    assert footprint_min_x(-1 + support - 0.001, yaw, FOOTPRINT) < -1
    assert footprint_min_x(-1 + support + 0.001, yaw, FOOTPRINT) > -1


def test_recorded_violation_pose_is_rejected_even_though_center_is_inside():
    assert -0.724 > -1
    assert footprint_min_x(-0.724, 0, FOOTPRINT) < -1
    assert footprint_min_x(-0.724, math.pi / 4, FOOTPRINT) < -1


@pytest.mark.parametrize("degrees", range(0, 360, 5))
def test_frontier_limit_contains_all_rotations_even_at_goal_tolerance(degrees):
    minimum = rotation_safe_center_x(-1, FOOTPRINT, PARAMS["goal_tolerance_xy"])
    actual_x = minimum - PARAMS["goal_tolerance_xy"]
    assert footprint_min_x(actual_x, math.radians(degrees), FOOTPRINT) >= -1 - 1e-12
    # The home goal remains safe for every heading and the same tolerance.
    assert footprint_min_x(-PARAMS["goal_tolerance_xy"], math.radians(degrees), FOOTPRINT) > -1


def test_frontiers_near_entrance_leave_room_to_rotate_and_stop():
    width, height, resolution, origin = 50, 20, 0.1, -2.0
    data = [-1] * (width * height)
    for y in range(3, 17):
        for x in range(12, 45):
            data[y * width + x] = 0 if 4 <= y < 16 else 100
    old = choose_frontier(data, width, height, resolution, origin, 0, 0, 1,
                          min_cluster=2, min_x=-1)
    assert old is not None and old[1] < -0.5
    minimum = rotation_safe_center_x(-1, FOOTPRINT, PARAMS["goal_tolerance_xy"])
    goal = choose_frontier(data, width, height, resolution, origin, 0, 0, 1,
                           min_cluster=2, min_x=-1, boundary_clearance=minimum + 1)
    assert goal is not None and goal[1] >= minimum


def test_reachability_cannot_use_outside_as_a_shortcut():
    width = height = 12
    data = [100] * (width * height)
    for y in range(1, 11):
        for x in range(1, 11):
            data[y * width + x] = 0
    for x in range(2, 11):
        data[5 * width + x] = 100
    for x in range(6, 10):
        data[1 * width + x] = -1
    args = (data, width, height, 1, 0, 0, 7.5, 8.5)
    assert choose_frontier(*args, clearance=0, min_cluster=2) is not None
    assert choose_frontier(*args, clearance=0, min_cluster=2, min_x=2) is None


@pytest.mark.parametrize("origin", [-2.0, -2.013, -2.047])
def test_mask_conservatively_covers_cell_edges_and_rotated_resampling(origin):
    res, width, height = 0.05, 100, 4
    mask = entrance_mask(width, height, res, origin, -1)
    for x in range(width):
        left = origin + x * res
        # A source cell is free only when even the center of a rotated local
        # costmap cell cannot sample it across the boundary.
        if left < -1 + res / math.sqrt(2):
            assert all(mask[y * width + x] == 100 for y in range(height))
        else:
            assert mask[x] == -1  # Do not turn shared unknown space into free space.
    assert mask[width:] == mask[:width] * (height - 1)


def test_nav2_costmaps_share_geometry_and_cannot_clear_the_virtual_wall():
    slam = yaml.safe_load((ROOT / "config/slam/writer.yaml").read_text())
    for kind in ("local", "global"):
        p = CONFIG[f"/writer/{kind}_costmap/{kind}_costmap"]["ros__parameters"]
        assert p["footprint"] == PARAMS["footprint"]
        assert p["footprint_padding"] == PARAMS["footprint_padding"]
        # The mask's half-cell-diagonal guard covers rolling-grid resampling
        # at this resolution; changes must retain this invariant.
        assert p["resolution"] <= slam["slam_toolbox"]["ros__parameters"]["resolution"]
        assert p["use_maximum"] is True
        assert p["plugins"].index("obstacle_layer") < p["plugins"].index("entrance_layer")
        assert p["plugins"].index("entrance_layer") < p["plugins"].index("inflation_layer")
        assert p["entrance_layer"]["map_topic"] == "/writer/entrance_mask"
        assert p["entrance_layer"]["footprint_clearing_enabled"] is False
    goal = CONFIG["/writer/controller_server"]["ros__parameters"]["goal_checker"]
    assert goal["xy_goal_tolerance"] == PARAMS["goal_tolerance_xy"]
    assert CONFIG["/writer/entrance_boundary"]["ros__parameters"]["entrance_x"] == PARAMS["entrance_x"]


def test_unsafe_home_and_frontier_goals_never_reach_nav2():
    pytest.importorskip("rclpy")
    pytest.importorskip("nav2_msgs.action")
    from living_map_writer.frontier_node import FrontierExplorer
    e = SimpleNamespace(minimum_goal_x=rotation_safe_center_x(-1, FOOTPRINT, 0.25),
                        get_logger=lambda: Mock(), _goal_failed=Mock(), nav=Mock())
    for kind in ("home", "frontier"):
        FrontierExplorer._send_goal(e, -0.724, 0, kind)
        e._goal_failed.assert_called_with(-0.724, 0, kind, "entrance_rejected")
    e.nav.send_goal_async.assert_not_called()


def test_mask_publisher_preserves_shared_map_and_metadata():
    pytest.importorskip("rclpy")
    pytest.importorskip("nav_msgs.msg")
    from nav_msgs.msg import OccupancyGrid
    from living_map_writer.entrance_boundary import EntranceBoundary
    msg = OccupancyGrid()
    msg.header.frame_id = "map"
    msg.info.width, msg.info.height, msg.info.resolution = 60, 4, 0.05
    msg.info.origin.position.x = -2
    msg.info.origin.orientation.w = 1
    msg.data = [-1, 0, 100] * 80
    original = list(msg.data)
    node = SimpleNamespace(mask_pub=Mock(), get_parameter=lambda _: SimpleNamespace(value=-1.0))
    EntranceBoundary.on_map(node, msg)
    mask = node.mask_pub.publish.call_args.args[0]
    assert list(msg.data) == original
    assert mask.info == msg.info and mask.header == msg.header
    assert mask is not msg and mask.info is not msg.info
    assert set(mask.data) == {-1, 100}
