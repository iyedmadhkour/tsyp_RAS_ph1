#!/usr/bin/env python3
"""Compare the optimized selector with its pre-optimization implementation.

The three captured Gazebo maps are paired with deterministic synthetic maps.
Both implementations receive byte-identical arrays and safety arguments.
"""
from __future__ import annotations

import argparse
import hashlib
import importlib.util
import json
import math
from pathlib import Path
import statistics
import subprocess
import time

import numpy as np
import yaml

from living_map_writer.entrance import padded_footprint, rotation_safe_center_x
from living_map_writer.frontier import choose_frontier

ROOT = Path(__file__).resolve().parents[1]
BASE_COMMIT = "8714209b2045f45963fde0715815d58d76f789d4"


def _baseline_selector():
    source = subprocess.run(
        ["git", "show", f"{BASE_COMMIT}:src/living_map_writer/living_map_writer/frontier.py"],
        cwd=ROOT, check=True, capture_output=True, text=True).stdout
    # Load source from memory: no generated source file is left in the checkout.
    spec = importlib.util.spec_from_loader("frontier_baseline_runtime", loader=None)
    module = importlib.util.module_from_spec(spec)
    exec(compile(source, f"git:{BASE_COMMIT}/frontier.py", "exec"), module.__dict__)
    return module.choose_frontier


def _synthetic_maps(seed: int):
    rng = np.random.default_rng(seed)
    for name, width, height, resolution, density in (
            ("synthetic_rooms_192x128", 192, 128, .05, .0008),
            ("synthetic_corridors_320x200", 320, 200, .05, .0012),
            ("synthetic_dense_384x256", 384, 256, .04, .0015)):
        grid = np.full((height, width), -1, dtype=np.int8)
        grid[4:height - 4, 4:width - 4] = 0
        # Reproducible walls with door gaps and obstacle clusters produce many
        # frontier candidates without changing the real footprint constraints.
        for x in range(40, width - 12, max(48, width // 5)):
            grid[5:height - 5, x] = 100
            gap = int(rng.integers(12, height - 12))
            grid[max(5, gap - 3):gap + 4, x] = 0
        count = int(width * height * density)
        ys = rng.integers(8, height - 8, size=count)
        xs = rng.integers(8, width - 8, size=count)
        for x, y in zip(xs, ys):
            grid[y, x] = 100
        yield {"name": name, "array": grid, "resolution": resolution,
               "origin": (-2.0, -1.0), "robot_xy": (1.0, 1.0),
               "seed": seed}


def _captured_maps(path: Path):
    dataset = json.loads(path.read_text())
    for entry in dataset["maps"]:
        data_path = path.parent / entry["file"]
        payload = data_path.read_bytes()
        digest = hashlib.sha256(payload).hexdigest()
        if digest != entry["sha256"]:
            raise ValueError(f"captured map hash mismatch: {data_path}")
        with np.load(data_path, allow_pickle=False) as archive:
            grid = archive["data"].astype(np.int8, copy=False)
            resolution = float(archive["resolution"])
            origin = tuple(float(v) for v in archive["origin"])
        yield {"name": entry["file"], "array": grid,
               "resolution": resolution, "origin": origin,
               "robot_xy": tuple(entry["robot_pose_map_xy_interpolated"]),
               "sha256": digest}


def _median_call(selector, values, kwargs, trials, warmup):
    def call():
        stats = {}
        result = selector(values, stats=stats, **kwargs)
        return result, stats
    expected = call()
    for _ in range(warmup):
        call()
    durations = []
    for _ in range(trials):
        start = time.perf_counter_ns()
        actual = call()
        durations.append((time.perf_counter_ns() - start) / 1e6)
        if actual != expected:
            raise AssertionError("selector output changed between identical trials")
    return expected, statistics.median(durations), sorted(durations)[math.ceil(.95 * trials) - 1]


def run(dataset: Path, seed: int, trials: int, warmup: int):
    params = yaml.safe_load((ROOT / "config/nav2/writer.yaml").read_text())[
        "/writer/frontier_explorer"]["ros__parameters"]
    footprint = padded_footprint(params["footprint"], float(params["footprint_padding"]))
    radius = max(math.hypot(x, y) for x, y in footprint)
    entrance = float(params["entrance_x"])
    safe_goal_x = rotation_safe_center_x(
        entrance, footprint, float(params["goal_tolerance_xy"]),
        float(params.get("entrance_position_uncertainty_xy", 0)))
    baseline = _baseline_selector()
    cases = [*list(_captured_maps(dataset)), *list(_synthetic_maps(seed))]
    results = []
    for case in cases:
        grid = case["array"]
        height, width = grid.shape
        resolution = case["resolution"]
        kwargs = dict(width=width, height=height, resolution=resolution,
                      origin_x=case["origin"][0], origin_y=case["origin"][1],
                      robot_x=case["robot_xy"][0], robot_y=case["robot_xy"][1],
                      min_cluster=int(params.get("min_cluster_cells", 5)),
                      clearance=.35, goal_clearance=radius + resolution / math.sqrt(2),
                      blacklist=(), blacklist_radius=float(params["frontier_zone_radius_m"]),
                      visited=(), visited_radius=float(params["frontier_visited_radius_m"]),
                      min_x=entrance, boundary_clearance=safe_goal_x - entrance)
        values = grid.ravel().tolist()
        before, baseline_ms, baseline_p95 = _median_call(
            baseline, values, kwargs, trials, warmup)
        after, current_ms, current_p95 = _median_call(
            choose_frontier, values, kwargs, trials, warmup)
        if before != after:
            raise AssertionError(f"goal/diagnostic mismatch for {case['name']}")
        results.append({"name": case["name"], "shape_width_height": [width, height],
                        "resolution_m": resolution, "input_sha256": case.get("sha256"),
                        "synthetic_seed": case.get("seed"),
                        "selected": before[0],
                        "selector_stats": before[1], "baseline_median_ms": baseline_ms,
                        "optimized_median_ms": current_ms, "baseline_p95_ms": baseline_p95,
                        "optimized_p95_ms": current_p95,
                        "median_reduction_percent": 100 * (baseline_ms-current_ms)/baseline_ms,
                        "identical_selection_and_diagnostics": True})
    return {"baseline_commit": BASE_COMMIT,
            "optimized_commit": subprocess.run(["git", "rev-parse", "HEAD"], cwd=ROOT,
                check=True, capture_output=True, text=True).stdout.strip(),
            "seed": seed, "trials": trials, "warmup": warmup,
            "selector_config_sha256": hashlib.sha256(
                (ROOT / "config/nav2/writer.yaml").read_bytes()).hexdigest(),
            "maps": results}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset", type=Path, default=ROOT /
        "docs/results/evidence/team4-phase2-baseline-20261120/maps.json")
    parser.add_argument("--seed", type=int, default=20261120)
    parser.add_argument("--trials", type=int, default=7)
    parser.add_argument("--warmup", type=int, default=1)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    if args.trials < 3 or args.warmup < 0:
        parser.error("--trials must be >= 3 and --warmup >= 0")
    report = run(args.dataset.resolve(), args.seed, args.trials, args.warmup)
    rendered = json.dumps(report, indent=2, sort_keys=True) + "\n"
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(rendered)
    print(rendered, end="")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
