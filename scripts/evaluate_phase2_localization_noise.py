#!/usr/bin/env python3
"""Seeded, ROS-free bounded localization-error experiments (not sensor data)."""
from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import random
import statistics
import subprocess

from living_map_writer.entrance import padded_footprint, rotation_safe_center_x
from living_map_writer.frontier import choose_frontier
from living_map_writer.goal_watchdog import GoalWatchdog, WatchdogConfig
from living_map_writer.progress import StuckDetector
from living_map_writer.swept_footprint import path_min_x
import yaml

ROOT = Path(__file__).resolve().parents[1]


def run(seed: int, trials: int, xy_bound: float, yaw_bound_deg: float) -> dict:
    rng = random.Random(seed)
    cfg_path = ROOT / "config/nav2/writer.yaml"
    params = yaml.safe_load(cfg_path.read_text())[
        "/writer/frontier_explorer"]["ros__parameters"]
    footprint = padded_footprint(params["footprint"], float(params["footprint_padding"]))
    entrance_x = float(params["entrance_x"])
    allowance = float(params["entrance_position_uncertainty_xy"])
    min_goal_x = rotation_safe_center_x(entrance_x, footprint,
        float(params["goal_tolerance_xy"]), allowance)
    yaw_bound = math.radians(yaw_bound_deg)

    # Fixed occupancy layout: a known rectangular room and unknown north edge.
    width, height, resolution = 120, 100, .05
    grid = [100 if y == 10 or x == 8 or x == 111 else
            (0 if 10 < y < 82 and 8 < x < 112 else -1)
            for y in range(height) for x in range(width)]
    goals = []
    safe_goals = 0
    for _ in range(trials):
        noise_x = rng.uniform(-xy_bound, xy_bound)
        noise_y = rng.uniform(-xy_bound, xy_bound)
        _yaw_error = rng.uniform(-yaw_bound, yaw_bound)
        result = choose_frontier(grid, width, height, resolution, -1.0, -2.5,
            1.5 + noise_x, 1.5 + noise_y,
            goal_clearance=.50, min_x=entrance_x,
            boundary_clearance=min_goal_x - entrance_x)
        if result is not None:
            goal = [round(result[1], 6), round(result[2], 6)]
            goals.append(goal)
            safe_goals += goal[0] >= min_goal_x

    # Check continuous footprint containment over a deliberately boundary-
    # adjacent synthetic route, with sampled XY error plus the configured
    # translation allowance. The goal center includes worst-heading clearance.
    center_x = min_goal_x
    route = []
    for i in range(trials + 1):
        fraction = i / trials
        route.append((center_x + rng.uniform(-xy_bound, xy_bound),
                      -0.5 + fraction,
                      rng.uniform(-yaw_bound, yaw_bound)))
    swept = path_min_x(route, footprint, allowance)

    watchdog_moving_false_cancels = 0
    watchdog_stationary_cancel_times = []
    stuck_moving_false = 0
    stuck_stationary_detected = 0
    stuck_stationary_detection_times = []
    for _ in range(trials):
        wd = GoalWatchdog(WatchdogConfig())
        wd.start("frontier", 5.0, 0.0, 0.0)
        cancel = None
        for sec in range(1, 51):
            true_x = min(5.0, 0.1 * sec)
            observed_x = true_x + rng.uniform(-xy_bound, xy_bound)
            observed_y = rng.uniform(-xy_bound, xy_bound)
            cancel = wd.update(float(sec), None, 0, observed_x, observed_y, False)
            if cancel is not None:
                break
        watchdog_moving_false_cancels += cancel is not None

        wd = GoalWatchdog(WatchdogConfig())
        wd.start("frontier", 5.0, 0.0, 0.0)
        cancel_time = None
        for sec in range(1, 31):
            cancel = wd.update(float(sec), None, 0,
                rng.uniform(-xy_bound, xy_bound),
                rng.uniform(-xy_bound, xy_bound), False)
            if cancel is not None:
                cancel_time = sec
                break
        if cancel_time is not None:
            watchdog_stationary_cancel_times.append(cancel_time)

        detector = StuckDetector()
        detector.set_active(True)
        detector_stuck = False
        for step in range(501):
            t = step * .1
            true_x = min(5.0, t * .1)
            detector.command(t, .1, 0.0)
            detector.odometry(t, true_x + rng.uniform(-xy_bound, xy_bound),
                              rng.uniform(-xy_bound, xy_bound),
                              rng.uniform(-yaw_bound, yaw_bound))
            detector_stuck = detector.evaluate(t) or detector_stuck
            if detector_stuck:
                break
        stuck_moving_false += detector_stuck

        detector = StuckDetector()
        detector.set_active(True)
        detector_stuck = False
        detection_time = None
        for step in range(501):
            t = step * .1
            detector.command(t, .1, 0.0)
            detector.odometry(t, rng.uniform(-xy_bound, xy_bound),
                              rng.uniform(-xy_bound, xy_bound),
                              rng.uniform(-yaw_bound, yaw_bound))
            detector_stuck = detector.evaluate(t)
            if detector_stuck:
                detection_time = t
                break
        if detection_time is not None:
            stuck_stationary_detected += 1
            stuck_stationary_detection_times.append(round(detection_time, 2))

    writer_source = ROOT / "src/living_map_writer/living_map_writer/frontier_node.py"
    return {
        "experiment": "seeded_bounded_pose_error_software_model",
        "classification": "SIMULATION_ASSUMPTION; not measured sensor performance",
        "commit": subprocess.run(["git", "rev-parse", "HEAD"], cwd=ROOT,
            check=True, capture_output=True, text=True).stdout.strip(),
        "seed": seed, "trials": trials,
        "disturbance_bounds": {"x_m": xy_bound, "y_m": xy_bound,
                               "yaw_deg": yaw_bound_deg},
        "config": {"entrance_x_m": entrance_x,
                   "configured_xy_allowance_m": allowance,
                   "safe_goal_center_x_m": min_goal_x,
                   "footprint": params["footprint"],
                   "footprint_padding_m": params["footprint_padding"],
                   "config_sha256": hashlib.sha256(cfg_path.read_bytes()).hexdigest(),
                   "writer_source_sha256": hashlib.sha256(writer_source.read_bytes()).hexdigest()},
        "frontier_selection": {"map_shape": [width, height], "resolution_m": resolution,
            "attempted": trials, "selected": len(goals), "safe_selected": safe_goals,
            "goal_variants": sorted({tuple(goal) for goal in goals})},
        "swept_entrance_check": {"pose_samples": len(route),
            "minimum_footprint_x_m": swept, "entrance_x_m": entrance_x,
            "contained": swept >= entrance_x},
        "watchdog": {"moving_false_cancels": watchdog_moving_false_cancels,
            "stationary_cancelled": len(watchdog_stationary_cancel_times),
            "stationary_cancel_median_s": statistics.median(
                watchdog_stationary_cancel_times) if watchdog_stationary_cancel_times else None},
        "stuck_detector": {"moving_false_stuck": stuck_moving_false,
            "stationary_stuck_detected": stuck_stationary_detected,
            "stationary_detection_median_s": statistics.median(
                stuck_stationary_detection_times) if stuck_stationary_detection_times else None},
        "scope_note": "The Nav watchdog and command/odometry STUCK detector are exercised independently;"
                      " this model does not simulate the Gazebo localization or motor interface.",
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seed", type=int, default=20261120)
    parser.add_argument("--trials", type=int, default=100)
    parser.add_argument("--xy-bound-m", type=float, default=.05)
    parser.add_argument("--yaw-bound-deg", type=float, default=5.0)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    if args.trials < 1 or args.xy_bound_m < 0 or args.yaw_bound_deg < 0:
        parser.error("trial count must be positive and disturbance bounds nonnegative")
    result = run(args.seed, args.trials, args.xy_bound_m, args.yaw_bound_deg)
    rendered = json.dumps(result, indent=2, sort_keys=True) + "\n"
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(rendered)
    print(rendered, end="")
    if (not result["swept_entrance_check"]["contained"]
            or result["frontier_selection"]["selected"] != args.trials
            or result["frontier_selection"]["safe_selected"] != args.trials
            or result["watchdog"]["moving_false_cancels"]
            or result["stuck_detector"]["moving_false_stuck"]
            or result["stuck_detector"]["stationary_stuck_detected"] != args.trials):
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
