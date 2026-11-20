import math
import random

from living_map_writer.frontier import (
    _dilated_occupied_rows, choose_frontier, clusters, frontier_cells,
)


def test_frontier_cells_match_four_neighbor_reference_on_seeded_maps():
    rng = random.Random(20261120)
    for width, height in ((9, 7), (31, 19)):
        data = [rng.choice((-1, 0, 20, 21, 65, 100))
                for _ in range(width * height)]
        expected = []
        for y in range(1, height - 1):
            for x in range(1, width - 1):
                index = y * width + x
                if (0 <= data[index] <= 20 and
                    (data[index - 1] < 0 or data[index + 1] < 0
                     or data[index - width] < 0 or data[index + width] < 0)):
                    expected.append((x, y))
        assert frontier_cells(data, width, height) == expected


def test_bit_row_obstacle_dilation_matches_original_circular_offsets():
    rng = random.Random(117)
    for width, height, resolution, clearance in (
            (13, 9, 0.1, 0.0), (21, 17, 0.05, 0.35),
            (27, 14, 0.05, 0.49), (16, 11, 0.2, 0.7)):
        data = [rng.choice((-1, 0, 20, 21, 65, 100))
                for _ in range(width * height)]
        # Exercise clipping at every map edge and corner.
        for x, y in ((0, 0), (width - 1, 0), (0, height - 1),
                     (width - 1, height - 1)):
            data[y * width + x] = 100
        radius = math.ceil(clearance / resolution)
        offsets = [(dx, dy) for dy in range(-radius, radius + 1)
                   for dx in range(-radius, radius + 1)
                   if math.hypot(dx, dy) * resolution <= clearance]
        expected = set()
        for index, value in enumerate(data):
            if value > 20:
                x, y = index % width, index // width
                expected.update((x + dx, y + dy) for dx, dy in offsets)
        actual_rows = _dilated_occupied_rows(data, width, height,
                                              resolution, clearance)
        actual = {(x, y) for y, row in enumerate(actual_rows)
                  for x in range(width) if (row >> x) & 1}
        expected_in_map = {(x, y) for x, y in expected
                           if 0 <= x < width and 0 <= y < height}
        assert actual == expected_in_map
def test_frontier_found():
 w=h=7;d=[100]*(w*h)
 for y in range(2,5):
  for x in range(2,5):d[y*w+x]=0
 for x in range(1,6):d[1*w+x]=-1
 assert choose_frontier(d,w,h,1,0,0,3,3,min_cluster=2) is not None


def test_goal_is_free_cell_on_concave_frontier():
    w = h = 11
    data = [-1] * (w * h)
    for y in range(2, 9):
        for x in range(2, 9):
            if x <= 3 or y >= 7:
                data[y * w + x] = 0
    goal = choose_frontier(data, w, h, 1, 0, 0, 2.5, 7.5)
    assert goal is not None
    assert data[int(goal[2]) * w + int(goal[1])] == 0


def test_blacklisted_goal_does_not_stop_exploration():
    w = h = 11
    data = [-1] * (w * h)
    for y in range(2, 9):
        for x in range(2, 9):
            data[y * w + x] = 0
    first = choose_frontier(data, w, h, 1, 0, 0, 5.5, 5.5)
    second = choose_frontier(data, w, h, 1, 0, 0, 5.5, 5.5,
                             blacklist=[(first[1], first[2])])
    assert second is not None
    assert second[1:3] != first[1:3]


def test_unreachable_frontier_is_ignored():
    w = h = 11
    data = [100] * (w * h)
    data[5 * w + 2] = 0
    for y in range(2, 9):
        for x in range(6, 9):
            data[y * w + x] = 0
    for y in range(2, 9):
        data[y * w + 9] = -1
    assert choose_frontier(data, w, h, 1, 0, 0, 2.5, 5.5) is None


def test_small_unknown_patch_under_robot_does_not_block_exploration():
    w = h = 21
    data = [-1] * (w * h)
    for y in range(2, 19):
        for x in range(2, 19):
            data[y * w + x] = 0
    data[10 * w + 10] = -1
    goal = choose_frontier(data, w, h, 0.1, 0, 0, 1.05, 1.05)
    assert goal is not None
    assert data[int(goal[2] / 0.1) * w + int(goal[1] / 0.1)] == 0


def test_frontier_that_survives_a_visit_is_skipped():
    # Free room (x 2..17) with two frontiers: an open west edge (x=1 unknown)
    # that never resolves, and an east edge (x=18 unknown).
    w, h = 20, 9
    data = [100] * (w * h)
    for y in range(2, 7):
        data[y * w + 1] = -1
        data[y * w + 18] = -1
        for x in range(2, 18):
            data[y * w + x] = 0
    near_west = choose_frontier(data, w, h, 1, 0, 0, 3.5, 4.5, min_cluster=2, clearance=0.3)
    assert near_west[1] < 5
    # The robot reached that west goal, yet the frontier is still there.
    after_visit = choose_frontier(data, w, h, 1, 0, 0, 3.5, 4.5, min_cluster=2, clearance=0.3,
                                  visited=[(near_west[1], near_west[2])])
    assert after_visit is not None and after_visit[1] > 15


def test_visited_point_does_not_discard_connected_unexplored_boundary():
    """One visited section of a room perimeter must leave other goals available."""
    width = height = 21
    data = [-1] * (width * height)
    for y in range(2, 19):
        for x in range(2, 19):
            data[y * width + x] = 0
    assert len(clusters(frontier_cells(data, width, height))) == 1
    first = choose_frontier(data, width, height, 0.1, 0, 0, 1.05, 1.05)
    assert first is not None
    visited = [first[1:3]]
    second = choose_frontier(data, width, height, 0.1, 0, 0, *first[1:3],
                             visited=visited, visited_radius=0.5)
    assert second is not None
    assert math.dist(second[1:3], first[1:3]) >= 0.5
    assert data[int(second[2] / 0.1) * width + int(second[1] / 0.1)] == 0


def test_fully_visited_boundary_has_no_new_goal():
    """Suppress already visited cells without falling back to repeat goals."""
    width = height = 11
    data = [-1] * (width * height)
    for y in range(2, 9):
        for x in range(2, 9):
            data[y * width + x] = 0
    visited = [(x + 0.5, y + 0.5) for x, y in frontier_cells(data, width, height)]
    assert choose_frontier(data, width, height, 1, 0, 0, 5.5, 5.5,
                           visited=visited) is None


def test_frontiers_behind_the_entrance_are_not_explored():
    from living_map_writer.frontier import choose_frontier
    width, height = 40, 20
    # Free 1.2 m corridor with walls along its sides; unknown beyond both ends.
    data = [-1] * (width * height)
    for y in range(3, 17):
        for x in range(5, 35):
            data[y * width + x] = 0 if 4 <= y < 16 else 100  # walls along both sides
    # Robot near the west end, so the west frontier is closer and would win.
    west = choose_frontier(data, width, height, 0.1, 0.0, 0.0, 1.5, 1.0, min_cluster=2)
    assert west is not None and west[1] < 1.0
    bounded = choose_frontier(data, width, height, 0.1, 0.0, 0.0, 1.5, 1.0, min_cluster=2, min_x=1.0)
    assert bounded is not None and bounded[1] > 3.0


def test_blacklist_radius_is_configurable():
    """Task 6 failure zones suppress a neighbourhood, not just one 0.6 m cell."""
    width = height = 9
    data = [-1] * (width * height)
    for y in range(2, 7):
        for x in range(2, 7):
            data[y * width + x] = 0
    first = choose_frontier(data, width, height, 1, 0, 0, 4.5, 4.5,
                            min_cluster=2, clearance=0.1)
    assert first is not None
    # The old 0.6 m radius leaves neighbouring cells of the same cluster.
    near = choose_frontier(data, width, height, 1, 0, 0, 4.5, 4.5,
                           min_cluster=2, clearance=0.1,
                           blacklist=[first[1:3]], blacklist_radius=0.6)
    assert near is not None
    # A zone wider than the whole cluster suppresses every candidate.
    assert choose_frontier(data, width, height, 1, 0, 0, 4.5, 4.5,
                           min_cluster=2, clearance=0.1,
                           blacklist=[first[1:3]], blacklist_radius=10.0) is None


def test_frontier_search_reports_selection_and_rejection_counts():
    width = height = 9
    data = [-1] * (width * height)
    for y in range(2, 7):
        for x in range(2, 7):
            data[y * width + x] = 0
    stats = {}
    assert choose_frontier(data, width, height, 1, 0, 0, 4.5, 4.5,
                           min_cluster=2, clearance=0.1, stats=stats) is not None
    assert stats["raw_frontier_cells"] == 16
    assert stats["clusters"] == 1 and stats["selected"] is True
    assert stats["clusters_too_small"] == 0
    assert stats["clusters_without_eligible_goal"] == 0

    visited = [(x + 0.5, y + 0.5) for x, y in frontier_cells(data, width, height)]
    stats = {}
    assert choose_frontier(data, width, height, 1, 0, 0, 4.5, 4.5,
                           visited=visited, stats=stats) is None
    assert stats["filtered_frontier_cells"] == 0
    assert stats["selected"] is False
