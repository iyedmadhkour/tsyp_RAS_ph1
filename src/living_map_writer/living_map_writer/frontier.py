from __future__ import annotations

from collections import deque
import math


def frontier_cells(data, width, height, free_max=20):
    return [
        (x, y)
        for y in range(1, height - 1)
        for x in range(1, width - 1)
        if 0 <= data[y * width + x] <= free_max
        and any(data[(y + dy) * width + x + dx] < 0
                for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)))
    ]


def clusters(cells):
    remaining = set(cells)
    result = []
    while remaining:
        seed = min(remaining)
        remaining.remove(seed)
        queue = [seed]
        cluster = [seed]
        while queue:
            x, y = queue.pop()
            for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1),
                           (1, 1), (1, -1), (-1, 1), (-1, -1)):
                neighbor = (x + dx, y + dy)
                if neighbor in remaining:
                    remaining.remove(neighbor)
                    queue.append(neighbor)
                    cluster.append(neighbor)
        result.append(cluster)
    return result


def choose_frontier(data, width, height, resolution, origin_x, origin_y,
                    robot_x, robot_y, min_cluster=5, blacklist=(), clearance=0.35,
                    visited=(), visited_radius=1.0, min_x=None, boundary_clearance=0.0,
                    goal_clearance=None, blacklist_radius=0.6, stats=None):
    """Choose an actual frontier cell reachable through free, robot-sized space.

    ``visited`` holds frontier goals the robot already reached. Only cells
    within ``visited_radius`` of these goals are excluded. Cluster the remaining
    cells so a visited section cannot hide a connected, unexplored boundary.
    ``blacklist`` holds remembered failure-zone centres; ``blacklist_radius``
    is the neighbourhood suppressed around each one (Task 6 makes that
    configurable instead of the previous hard-coded 0.6 m).

    ``min_x + boundary_clearance`` bounds both candidate goals and the reachable
    free-cell search. Callers supply the padded footprint's rotation radius and
    goal-position tolerance; outside space must not connect two inside regions.
    ``goal_clearance`` can require extra turning room at goals without assuming
    the robot must be able to rotate in place at every transit cell. Nav2 checks
    the actual footprint along the resulting navigation path.

    ``stats`` is an optional dict filled with candidate counts and rejection
    reasons so the explorer can explain why a frontier was selected or skipped.
    """
    stats = {} if stats is None else stats
    stats.clear()
    radius = math.ceil(clearance / resolution)
    offsets = [(dx, dy) for dy in range(-radius, radius + 1)
               for dx in range(-radius, radius + 1)
               if math.hypot(dx, dy) * resolution <= clearance]
    blocked = set()
    for index, value in enumerate(data):
        if value > 20:
            x, y = index % width, index // width
            blocked.update((x + dx, y + dy) for dx, dy in offsets)
    goal_blocked = blocked
    if goal_clearance is not None:
        goal_radius = math.ceil(goal_clearance / resolution)
        goal_offsets = [(dx, dy) for dy in range(-goal_radius, goal_radius + 1)
                        for dx in range(-goal_radius, goal_radius + 1)
                        if math.hypot(dx, dy) * resolution <= goal_clearance]
        goal_blocked = set()
        for index, value in enumerate(data):
            if value > 20:
                x, y = index % width, index // width
                goal_blocked.update((x + dx, y + dy) for dx, dy in goal_offsets)
    raw_cells = frontier_cells(data, width, height)
    free = {(i % width, i // width) for i, value in enumerate(data)
            if 0 <= value <= 20 and (i % width, i // width) not in blocked
            and (min_x is None or origin_x + (i % width + 0.5) * resolution
                 >= min_x + boundary_clearance)}
    start = (math.floor((robot_x - origin_x) / resolution),
             math.floor((robot_y - origin_y) / resolution))
    if start not in free:
        # SLAM can leave a small unknown patch beneath the laser/base.
        if not free:
            stats.update(raw_frontier_cells=len(raw_cells), filtered_frontier_cells=0,
                         clusters=0, clusters_too_small=0,
                         clusters_without_eligible_goal=0, selected=False)
            return None
        nearest = min(free, key=lambda p: ((p[0] - start[0])**2 + (p[1] - start[1])**2, p))
        if math.dist(start, nearest) * resolution > clearance:
            stats.update(raw_frontier_cells=len(raw_cells), filtered_frontier_cells=0,
                         clusters=0, clusters_too_small=0,
                         clusters_without_eligible_goal=0, selected=False)
            return None
        start = nearest
    reachable = {start}
    queue = deque([start])
    while queue:
        x, y = queue.popleft()
        for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            neighbor = (x + dx, y + dy)
            if neighbor in free and neighbor not in reachable:
                reachable.add(neighbor)
                queue.append(neighbor)

    unvisited_cells = [
        (x, y) for x, y in raw_cells
        if (min_x is None or origin_x + (x + 0.5) * resolution >= min_x + boundary_clearance)
        and not any(
            (origin_x + (x + 0.5) * resolution - vx)**2
            + (origin_y + (y + 0.5) * resolution - vy)**2 <= visited_radius**2
            for vx, vy in visited
        )
    ]
    cluster_list = clusters(unvisited_cells)
    stats.update(raw_frontier_cells=len(raw_cells),
                 filtered_frontier_cells=len(unvisited_cells),
                 clusters=len(cluster_list),
                 clusters_too_small=0,
                 clusters_without_eligible_goal=0,
                 selected=False)
    best = None
    for cluster in cluster_list:
        if len(cluster) < min_cluster:
            stats["clusters_too_small"] += 1
            continue
        cx = sum(x for x, _ in cluster) / len(cluster)
        cy = sum(y for _, y in cluster) / len(cluster)
        eligible = []
        for x, y in cluster:
            wx = origin_x + (x + 0.5) * resolution
            wy = origin_y + (y + 0.5) * resolution
            if (x, y) in reachable and (x, y) not in goal_blocked and not any(
                (wx - bx)**2 + (wy - by)**2 < blacklist_radius**2 for bx, by in blacklist
            ) and math.hypot(wx - robot_x, wy - robot_y) >= 0.5:
                eligible.append((x, y))
        if not eligible:
            stats["clusters_without_eligible_goal"] += 1
            continue
        x, y = min(eligible, key=lambda p: ((p[0] - cx)**2 + (p[1] - cy)**2, p))
        wx = origin_x + (x + 0.5) * resolution
        wy = origin_y + (y + 0.5) * resolution
        score = len(eligible) * resolution - 0.7 * math.hypot(wx - robot_x, wy - robot_y)
        if best is None or score > best[0]:
            best = (score, wx, wy, len(cluster))
    if best is not None:
        stats.update(selected=True, selected_x=best[1], selected_y=best[2],
                     selected_score=best[0], selected_cluster_cells=best[3])
    return best
