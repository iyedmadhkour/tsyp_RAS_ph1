"""Deterministic coverage of the node's no-candidate reason-code paths.

The Gazebo missions can exercise a real failure, but they cannot force every
combination of cooling, exhausted, visited-only and genuinely empty maps. These
tests call the same selector and classifier with a small synthetic grid.
"""
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

pytest.importorskip("rclpy")
pytest.importorskip("nav2_msgs.action")
from nav_msgs.msg import OccupancyGrid

from living_map_writer.frontier_node import FrontierExplorer
from living_map_writer.frontier_policy import FrontierPolicy, FrontierPolicyConfig
from living_map_writer.sortie import Sortie


def explorer(policy=None, frontier=True):
    grid = OccupancyGrid()
    grid.info.width = grid.info.height = 10
    grid.info.resolution = 1.0
    if frontier:
        # Known free interior surrounded by unknown space: one connected ring
        # of frontier cells, all reachable from the robot at (1, 1).
        grid.data = [0 if 1 <= x <= 8 and 1 <= y <= 8 else -1
                     for y in range(10) for x in range(10)]
    else:
        grid.data = [0] * (grid.info.width * grid.info.height)

    node = object.__new__(FrontierExplorer)
    node.map = grid
    node.policy = policy or FrontierPolicy(FrontierPolicyConfig(zone_radius_m=10.0))
    node.sortie = Sortie(budget_s=0, empty_ticks_to_complete=2)
    node.frontier_clearance = 0.1
    node.minimum_goal_x = 0.0
    node.known_area_m2 = 50.0
    node.last_wait_log_key = None
    node._publish_summary = Mock()
    node.get_logger = lambda: Mock()
    node._now_s = lambda: 100.0
    node.get_parameter = lambda key: SimpleNamespace(value={
        "min_cluster_cells": 2, "entrance_x": -100.0}[key])
    node._choose = lambda rx, ry, blacklist, visited, stats=None: (
        FrontierExplorer._choose(node, rx, ry, blacklist, visited, stats))
    return node


def selected(node):
    stats = {}
    candidate = node._choose(1.0, 1.0, (), (), stats)
    assert candidate is not None, stats
    return candidate[1], candidate[2], stats


def test_cooling_failure_waits_instead_of_completing():
    node = explorer()
    x, y, _ = selected(node)
    node.policy.note_failure(x, y, 100.0, 50.0)

    node._no_candidate(100.0, 1.0, 1.0, (), {})

    assert node.sortie.phase == "EXPLORING"
    assert node.sortie.empty_ticks == 0
    assert node._publish_summary.call_count == 0
    assert node.last_wait_log_key is not None


def test_exhausted_zone_completes_with_frontiers_exhausted_code():
    node = explorer()
    x, y, _ = selected(node)
    for now in (100.0, 101.0, 102.0):
        node.policy.note_failure(x, y, now, 50.0)
    assert node.policy.zones[0].exhausted

    node._no_candidate(103.0, 1.0, 1.0, (), {})
    assert node.sortie.phase == "EXPLORING"
    node._no_candidate(105.0, 1.0, 1.0, (), {})

    assert node.sortie.phase == "RETURNING"
    assert node.sortie.reason_code == "FRONTIERS_EXHAUSTED"


def test_visited_only_frontier_reports_no_reachable_frontier():
    node = explorer(FrontierPolicy(FrontierPolicyConfig(
        zone_radius_m=10.0, visited_radius_m=10.0)))
    x, y, _ = selected(node)
    node.policy.note_success(x, y, 100.0, 50.0)
    visited = node.policy.visited_points(100.0, 50.0)
    assert visited == [(x, y)]

    node._no_candidate(100.0, 1.0, 1.0, visited, {})
    node._no_candidate(102.0, 1.0, 1.0, visited, {})

    assert node.sortie.phase == "RETURNING"
    assert node.sortie.reason_code == "NO_REACHABLE_FRONTIER"


def test_genuinely_empty_map_reports_explored():
    node = explorer(frontier=False)

    node._no_candidate(100.0, 1.0, 1.0, (), {})
    node._no_candidate(102.0, 1.0, 1.0, (), {})

    assert node.sortie.phase == "RETURNING"
    assert node.sortie.reason_code == "EXPLORED"
