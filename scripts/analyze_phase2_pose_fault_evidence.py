#!/usr/bin/env python3
"""Audit fresh/stale pose fault injection against Gazebo ground truth."""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import re
import sys
import xml.etree.ElementTree as ET

import yaml

from analyze_phase2_stale_ground_truth import _parse_pose_stream


def _phase_poses(path: Path, spawn: list[float]):
    raw = _parse_pose_stream(path)
    return [(t, x - spawn[0], y - spawn[1], yaw - spawn[5])
            for t, x, y, yaw in raw]


def _continuous_bound(poses, vmax: float, radius: float, uncertainty: float):
    gaps = [b[0] - a[0] for a, b in zip(poses, poses[1:])]
    lower = min(min(a[1], b[1]) - vmax * (b[0] - a[0]) - radius - uncertainty
                for a, b in zip(poses, poses[1:]))
    return lower, max(gaps, default=math.inf)


def _frozen_stream_audit(run: Path, start_s: float, end_s: float):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    bag = run / "bag"
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(bag / "bag_0.mcap"), storage_id="mcap"),
                rosbag2_py.ConverterOptions("cdr", "cdr"))
    types = {topic.name: get_message(topic.type)
             for topic in reader.get_all_topics_and_types()}
    sim_s = None
    odom = []
    command_count = 0
    command_nonzero = 0
    while reader.has_next():
        topic, payload, _receive_stamp = reader.read_next()
        message = deserialize_message(payload, types[topic])
        if topic == "/clock":
            sim_s = message.clock.sec + message.clock.nanosec / 1e9
        elif topic == "/writer/odom":
            stamp = message.header.stamp.sec + message.header.stamp.nanosec / 1e9
            if start_s <= stamp <= end_s:
                p = message.pose.pose.position
                odom.append((stamp, p.x, p.y))
        elif topic == "/writer/cmd_vel" and sim_s is not None and start_s <= sim_s <= end_s:
            command_count += 1
            if abs(message.linear.x) > .02 or abs(message.angular.z) > .1:
                command_nonzero += 1
    if len(odom) < 10:
        raise ValueError(f"frozen pose interval has too few odometry samples: {len(odom)}")
    positions = [(x, y) for _, x, y in odom]
    pose_range = max(math.hypot(x1-x2, y1-y2)
                     for x1, y1 in positions for x2, y2 in positions)
    return {
        "odom_samples": len(odom),
        "odom_stamp_span_s": odom[-1][0] - odom[0][0],
        "frozen_odom_position_range_m": pose_range,
        "cmd_vel_samples": command_count,
        "nonzero_cmd_vel_samples": command_nonzero,
        "freshly_stamped_pose_stream": odom[-1][0] - odom[0][0] >= 1.0,
        "reported_pose_frozen": pose_range <= 1e-6,
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("run", type=Path)
    parser.add_argument("--writer-config", type=Path, required=True)
    parser.add_argument("--world", type=Path, required=True)
    parser.add_argument("--model", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()

    config = yaml.safe_load(args.writer_config.read_text())
    params = config["/writer/frontier_explorer"]["ros__parameters"]
    polygon = json.loads(params["footprint"])
    pad = float(params["footprint_padding"])
    padded = [(x + math.copysign(pad, x), y + math.copysign(pad, y))
              for x, y in polygon]
    radius = max(math.hypot(x, y) for x, y in padded)
    uncertainty = float(params["entrance_position_uncertainty_xy"])
    entrance_x = float(params["entrance_x"])

    world = ET.parse(args.world).getroot()
    include = next(e for e in world.iter("include")
                   if any(c.tag == "name" and c.text == "writer" for c in e))
    spawn = [float(v) for v in include.findtext("pose").split()]
    model = ET.parse(args.model).getroot()
    vmax = float(model.findtext(".//plugin/max_linear_velocity"))

    phase_results = {}
    pose_quality = True
    for phase in ("stale-recovery", "fresh-stamped-freeze"):
        path = args.run / phase / "ground-truth-pose.txt"
        poses = _phase_poses(path, spawn)
        lower, gap = _continuous_bound(poses, vmax, radius, uncertainty)
        phase_results[phase] = {
            "ground_truth_samples": len(poses),
            "simulation_start_s": poses[0][0],
            "simulation_end_s": poses[-1][0],
            "max_ground_truth_gap_s": gap,
            "conservative_swept_footprint_lower_x_m": lower,
            "entrance_plane_x_m": entrance_x,
            "continuous_containment": lower >= entrance_x,
        }
        pose_quality &= len(poses) >= 100 and gap <= .05 and lower >= entrance_x

    recovery = args.run / "stale-recovery"
    recovery_summary = json.loads((recovery / "summary.json").read_text())
    mode_log = (recovery / "pose-adapter.log").read_text(errors="replace")
    mode_lines = (recovery / "mode-timeline.txt").read_text(errors="replace")
    required_modes = ["stale", "delayed", "missing", "pass"]
    mode_seen = all(f"POSE_FAULT_MODE={mode}" in mode_log for mode in required_modes)
    cancellations = recovery_summary["localization_cancel_events"] >= 3
    no_goal_while_invalid = all(
        int(a) == int(b) for a, b in re.findall(
            r"goals_before=(\d+)\s+goals_while_pose_invalid=(\d+)", mode_lines))
    recovery_seen = len(re.findall(r"goals_after_recovery=", mode_lines)) >= 2
    home = recovery_summary["home_observed"]

    frozen_dir = args.run / "fresh-stamped-freeze"
    frozen_fault = (frozen_dir / "fault-injection.txt").read_text()
    start_s = float(re.search(r"frozen_start_sim_s=([0-9.eE+-]+)", frozen_fault).group(1))
    frozen_end = float(re.search(r"frozen_end_sim_s=([0-9.eE+-]+)", frozen_fault).group(1))
    frozen_stream = _frozen_stream_audit(frozen_dir, start_s, frozen_end)
    frozen_log = (frozen_dir / "pose-adapter.log").read_text(errors="replace")
    stuck = "POSE_FAULT_MODE=frozen" in frozen_log and json.loads(
        (frozen_dir / "summary.json").read_text())["stuck_observed"]

    result = {
        "scenario": "dedicated_stale_delayed_missing_and_frozen_pose_gazebo",
        "result": "PASS" if (mode_seen and cancellations and no_goal_while_invalid
                               and recovery_seen and home and stuck and pose_quality
                               and frozen_stream["freshly_stamped_pose_stream"]
                               and frozen_stream["reported_pose_frozen"]
                               and frozen_stream["nonzero_cmd_vel_samples"] > 0) else "INCONCLUSIVE",
        "simulation_only": True,
        "pose_fault_modes_seen": mode_seen,
        "localization_cancellations_at_least_three": cancellations,
        "no_new_goal_while_pose_invalid": no_goal_while_invalid,
        "navigation_resumed_after_valid_pose": recovery_seen,
        "home_after_missing_pose_recovery": home,
        "frozen_pose_ended_in_stuck": stuck,
        "frozen_stream": frozen_stream,
        "footprint_assumptions": {
            "footprint": params["footprint"], "padding_m": pad,
            "translation_uncertainty_m": uncertainty,
            "max_linear_speed_m_s": vmax,
            "basis": "Gazebo spawn-to-map registration; full padded-footprint circumradius covers every heading; vmax bounds intersample center motion",
        },
        "phases": phase_results,
        "limitations": [
            "0.5 s pose age, 0.35 s scan age and 1.5 s injected delay are simulation assumptions, not hardware limits.",
            "Frozen fresh-stamped odometry is detected conservatively through the existing no-progress STUCK path; this cannot prove whether a physical base moved.",
            "Gazebo ground truth does not measure real localization error, motor stopping, or physical boundary containment.",
        ],
    }
    args.output.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n")
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0 if result["result"] == "PASS" else 2


if __name__ == "__main__":
    raise SystemExit(main())
