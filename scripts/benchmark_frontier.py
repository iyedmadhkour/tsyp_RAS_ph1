#!/usr/bin/env python3
"""Repeatably benchmark the production frontier selector on captured Gazebo maps."""
from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import platform
import statistics
import subprocess
import time

import numpy as np
import yaml

from living_map_writer.entrance import padded_footprint, rotation_safe_center_x
from living_map_writer.frontier import choose_frontier


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_DATASET = ROOT / "docs/results/evidence/team4-phase2-baseline-20261120/maps.json"


def _commit() -> str:
    try:
        return subprocess.run(["git", "rev-parse", "HEAD"], cwd=ROOT,
                              capture_output=True, text=True, check=True).stdout.strip()
    except (OSError, subprocess.CalledProcessError):
        return "unknown"


def _source_digest() -> str:
    source = ROOT / "src/living_map_writer/living_map_writer/frontier.py"
    return hashlib.sha256(source.read_bytes()).hexdigest()


def run(dataset_path: Path, trials: int, warmup: int) -> dict:
    dataset = json.loads(dataset_path.read_text())
    config_path = ROOT / "config/nav2/writer.yaml"
    config = yaml.safe_load(config_path.read_text())
    params = config["/writer/frontier_explorer"]["ros__parameters"]
    footprint = padded_footprint(params["footprint"], float(params["footprint_padding"]))
    radius = max(math.hypot(x, y) for x, y in footprint)
    minimum_goal_x = rotation_safe_center_x(
        float(params["entrance_x"]), footprint,
        float(params["goal_tolerance_xy"]),
        float(params.get("entrance_position_uncertainty_xy", 0.0)))
    measured = []

    for entry in dataset["maps"]:
        path = dataset_path.parent / entry["file"]
        payload = path.read_bytes()
        digest = hashlib.sha256(payload).hexdigest()
        if digest != entry["sha256"]:
            raise ValueError(f"dataset hash mismatch for {path}")
        with np.load(path, allow_pickle=False) as archive:
            grid = archive["data"]
            resolution = float(archive["resolution"])
            origin_x, origin_y = map(float, archive["origin"])
        height, width = grid.shape
        data = grid.ravel().tolist()
        robot_x, robot_y = entry["robot_pose_map_xy_interpolated"]
        kwargs = {
            "min_cluster": int(params.get("min_cluster_cells", 5)),
            # Match FrontierExplorer._choose: the transit clearance default is
            # 0.35 m; goal clearance uses the padded polygon radius plus half a
            # cell diagonal.
            "clearance": 0.35,
            "goal_clearance": radius + resolution / math.sqrt(2.0),
            "blacklist": (),
            "blacklist_radius": float(params["frontier_zone_radius_m"]),
            "visited": (),
            "visited_radius": float(params["frontier_visited_radius_m"]),
            "min_x": float(params["entrance_x"]),
            "boundary_clearance": minimum_goal_x - float(params["entrance_x"]),
        }

        def select():
            stats: dict = {}
            result = choose_frontier(
                data, width, height, resolution, origin_x, origin_y,
                robot_x, robot_y, stats=stats, **kwargs)
            return result, stats

        for _ in range(warmup):
            select()
        durations = []
        result = None
        stats = {}
        for _ in range(trials):
            started = time.perf_counter_ns()
            result, stats = select()
            durations.append((time.perf_counter_ns() - started) / 1e6)
        ordered = sorted(durations)
        p95 = ordered[min(len(ordered) - 1, math.ceil(0.95 * len(ordered)) - 1)]
        measured.append({
            "map_file": entry["file"],
            "map_sha256": digest,
            "sim_s": entry["sim_s"],
            "shape_width_height": [width, height],
            "resolution_m": resolution,
            "known_cells": entry["known_cells"],
            "occupied_cells": entry["occupied_cells"],
            "robot_pose_map_xy": [robot_x, robot_y],
            "selected_xy": None if result is None else [result[1], result[2]],
            "selection_stats": stats,
            "trial_count": trials,
            "warmup_count": warmup,
            "median_ms": statistics.median(durations),
            "p95_ms": p95,
            "min_ms": min(durations),
            "max_ms": max(durations),
            "trial_ms": durations,
        })

    return {
        "benchmark": "living_map_writer.frontier.choose_frontier",
        "source_commit": _commit(),
        "frontier_source_sha256": _source_digest(),
        "python": platform.python_version(),
        "numpy": np.__version__,
        "config_sha256": hashlib.sha256(config_path.read_bytes()).hexdigest(),
        "dataset": str(dataset_path),
        "dataset_description": dataset["dataset"],
        "random_seed": None,
        "random_seed_note": "Captured Gazebo OccupancyGrid data; no synthetic noise or RNG is used.",
        "maps": measured,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset", type=Path, default=DEFAULT_DATASET)
    parser.add_argument("--trials", type=int, default=15)
    parser.add_argument("--warmup", type=int, default=2)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    if args.trials < 3 or args.warmup < 0:
        parser.error("--trials must be at least 3 and --warmup cannot be negative")
    result = run(args.dataset.resolve(), args.trials, args.warmup)
    rendered = json.dumps(result, indent=2, sort_keys=True) + "\n"
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(rendered)
    print(rendered, end="")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
