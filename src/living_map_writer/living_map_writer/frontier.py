from __future__ import annotations

from collections import deque
import math


def frontier_cells(data, width, height, free_max=20):
    result = []
    for y in range(1, height - 1):
        row = y * width
        above, below = row - width, row + width
        for x in range(1, width - 1):
            index = row + x
            if (0 <= data[index] <= free_max
                    and (data[index - 1] < 0 or data[index + 1] < 0
                         or data[above + x] < 0 or data[below + x] < 0)):
                result.append((x, y))
    return result


def _dilated_occupied_rows(data, width, height, resolution, clearance):
    """Return exact circular obstacle dilation as one integer bit row per map row.

    Only in-map cells are queried by the selector, so keeping each row as a bit
    set avoids allocating and hashing all obstacle-neighbour coordinate tuples.
    The per-row spans use the same cell-center distance predicate as the former
    offset set, preserving the occupied-cell clearance exactly.
    """
    radius = math.ceil(clearance / resolution)
    spans = []
    for dy in range(-radius, radius + 1):
        valid_dx = [dx for dx in range(-radius, radius + 1)
                    if math.hypot(dx, dy) * resolution <= clearance]
        if valid_dx:
            spans.append((dy, max(abs(dx) for dx in valid_dx)))

    rows = [0] * height
    for y in range(height):
        row = y * width
        for x in range(width):
            if data[row + x] <= 20:
                continue
            for dy, span in spans:
                target_y = y + dy
                if not 0 <= target_y < height:
                    continue
                left, right = max(0, x - span), min(width, x + span + 1)
                if left < right:
                    rows[target_y] |= ((1 << (right - left)) - 1) << left
    return rows


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
    blocked_rows = _dilated_occupied_rows(data, width, height, resolution, clearance)
    goal_blocked_rows = blocked_rows
    if goal_clearance is not None:
        goal_blocked_rows = _dilated_occupied_rows(
            data, width, height, resolution, goal_clearance)
    raw_cells = frontier_cells(data, width, height)
    free = set()
    boundary_x = None if min_x is None else min_x + boundary_clearance
    for y in range(height):
        row = y * width
        blocked = blocked_rows[y]
        for x in range(width):
            value = data[row + x]
            if (0 <= value <= 20 and not (blocked >> x) & 1
                    and (boundary_x is None
                         or origin_x + (x + 0.5) * resolution >= boundary_x)):
                free.add((x, y))
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
            if (x, y) in reachable and not (goal_blocked_rows[y] >> x) & 1 and not any(
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
