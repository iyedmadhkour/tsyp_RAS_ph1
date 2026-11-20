#!/usr/bin/env python3
"""Audit a stale-input Gazebo run using synchronized world-frame poses.

This report uses Gazebo model poses and the independent world_odom publisher,
then bounds motion between observations with the simulator's configured maximum
linear speed and the full orientation-independent padded footprint radius.
"""
from __future__ import annotations

import argparse
import json
import math
import re
from pathlib import Path
import xml.etree.ElementTree as ET

import yaml


def _number(block: str, key: str, default: float = 0.0) -> float:
    match = re.search(rf"\b{re.escape(key)}:\s*([-+0-9.eE]+)", block)
    return float(match.group(1)) if match else default


def _parse_pose_stream(path: Path):
    text = path.read_text(encoding="utf-8", errors="replace")
    # Each Gazebo Pose message begins with `pose {`; extract nested braces.
    blocks = []
    start = 0
    while True:
        start = text.find("pose {", start)
        if start < 0:
            break
        depth = 0
        end = start
        for end in range(start, len(text)):
            if text[end] == "{":
                depth += 1
            elif text[end] == "}":
                depth -= 1
                if depth == 0:
                    blocks.append(text[start:end + 1])
                    start = end + 1
                    break
        else:
            raise ValueError("unterminated Gazebo pose message")

    poses = []
    for block in blocks:
        frame_match = re.search(r'key:\s*"frame_id"\s*value:\s*"([^"]+)"', block)
        child_match = re.search(r'key:\s*"child_frame_id"\s*value:\s*"([^"]+)"', block)
        if (frame_match is None or frame_match.group(1) != "world"
                or child_match is None or child_match.group(1) != "writer/base_link"):
            raise ValueError("Gazebo pose frame must be world -> writer/base_link")
        header = re.search(r"header\s*\{(.*?)\n\s*\}", block, re.S)
        stamp_text = header.group(1) if header else ""
        sec = _number(stamp_text, "sec")
        nsec = _number(stamp_text, "nsec")
        position = re.search(r"position\s*\{(.*?)\n\s*\}", block, re.S)
        orientation = re.search(r"orientation\s*\{(.*?)\n\s*\}", block, re.S)
        if position is None or orientation is None:
            continue
        x, y = _number(position.group(1), "x"), _number(position.group(1), "y")
        z, w = _number(orientation.group(1), "z"), _number(orientation.group(1), "w", 1.0)
        yaw = math.atan2(2.0 * w * z, 1.0 - 2.0 * z * z)
        poses.append((sec + nsec / 1e9, x, y, yaw))
    if not poses:
        raise ValueError(f"no valid Gazebo model poses in {path}")
    return poses


def _events(path: Path):
    return [json.loads(line) for line in path.read_text().splitlines() if line.strip()]


def main() -> int:
    """Create a machine-readable, conservative stale-outage safety audit."""
    parser = argparse.ArgumentParser()
    parser.add_argument("run_dir", type=Path)
    parser.add_argument("--writer-config", type=Path, default=Path("config/nav2/writer.yaml"))
    parser.add_argument("--world", type=Path, default=Path("src/living_map_sim/worlds/mine.sdf"))
    parser.add_argument("--model", type=Path, default=Path("src/living_map_sim/models/writer/model.sdf"))
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()

    run = args.run_dir
    raw = _parse_pose_stream(run / "ground-truth-pose.txt")
    events = _events(run / "observations.jsonl")
    config = yaml.safe_load(args.writer_config.read_text())
    params = config["/writer/frontier_explorer"]["ros__parameters"]
    footprint = json.loads(params["footprint"])
    pad = float(params["footprint_padding"])
    footprint = [(x + math.copysign(pad, x), y + math.copysign(pad, y))
                 for x, y in footprint]
    radius = max(math.hypot(x, y) for x, y in footprint)
    uncertainty = float(params["entrance_position_uncertainty_xy"])
    entrance_x = float(params["entrance_x"])

    world_root = ET.parse(args.world).getroot()
    world_model = next(e for e in world_root.iter("include")
                       if any(c.tag == "name" and c.text == "writer" for c in e))
    spawn = [float(v) for v in world_model.findtext("pose").split()]
    model_root = ET.parse(args.model).getroot()
    max_linear = float(model_root.findtext(".//plugin/max_linear_velocity"))

    # The Writer is spawned at the map origin. The recorded initial Nav2 pose
    # is (0,0), and live map->writer/odom is identity before/during/after fault.
    # These independent checks determine world->map translation and yaw.
    map_dx, map_dy, map_yaw = -spawn[0], -spawn[1], -spawn[5]
    log = (run / "launch.log").read_text(errors="replace")
    initial_pose_confirmed = bool(re.search(
        r"Begin navigating from current location \(0\.00, 0\.00\)", log))
    tf_events = [e for e in events if e.get("event") == "map_to_odom" and e.get("data")]
    tf_max = max((max(abs(float(e["data"][k])) for k in ("x", "y", "yaw"))
                  for e in tf_events), default=math.inf)
    if not initial_pose_confirmed or tf_max > 1e-6:
        raise ValueError("map/world alignment not supported by initial pose and TF observations")

    poses = []
    for t, wx, wy, yaw in raw:
        c, s = math.cos(map_yaw), math.sin(map_yaw)
        poses.append((t, map_dx + c * wx - s * wy,
                      map_dy + s * wx + c * wy, yaw + map_yaw))
    if any(b[0] <= a[0] for a, b in zip(poses, poses[1:])):
        raise ValueError("Gazebo pose stamps are not strictly increasing")

    time_gaps = [b[0] - a[0] for a, b in zip(poses, poses[1:])]
    continuous_lower = math.inf
    for a, b in zip(poses, poses[1:]):
        # Any center motion during one sample interval is at most vmax*dt;
        # the padded polygon's circumradius covers every orientation, including
        # the entire tested yaw uncertainty, not merely sampled headings.
        lower = min(a[1], b[1]) - max_linear * (b[0] - a[0]) - radius - uncertainty
        continuous_lower = min(continuous_lower, lower)

    fault_lines = (run / "fault-injection.txt").read_text().splitlines()
    fault = {line.split("=", 1)[0]: float(line.split("=", 1)[1])
             for line in fault_lines if "=" in line
             and line.split("=", 1)[0] in {"paused_sim_s", "resumed_sim_s"}}
    pause_s, resume_s = fault["paused_sim_s"], fault["resumed_sim_s"]
    markers = {e["event"]: e.get("sim_s") for e in events
               if e.get("event") in {"goal_cancel_requested", "goal_cancel_terminal_logged"}}
    cancel_s = markers.get("goal_cancel_requested")
    terminal_s = markers.get("goal_cancel_terminal_logged")

    def between(start: float, end: float):
        return [p for p in poses if start <= p[0] <= end]

    outage_poses = between(pause_s, resume_s)
    if not outage_poses:
        raise ValueError("no ground-truth poses span the injected outage")
    start = min(outage_poses, key=lambda p: abs(p[0] - pause_s))
    end = min(outage_poses, key=lambda p: abs(p[0] - resume_s))
    cmds = [e for e in events if e.get("event") == "writer_cmd_vel"
            and e.get("sim_s") is not None]
    outage_cmds = [e for e in cmds if pause_s <= e["sim_s"] <= resume_s]
    post_cancel_nonzero = [e for e in outage_cmds if cancel_s is not None
                           and e["sim_s"] > cancel_s + 0.15
                           and (abs(e["data"]["linear_x"]) > 1e-9
                                or abs(e["data"]["angular_z"]) > 1e-9)]
    cancel_pose = (min(poses, key=lambda p: abs(p[0] - cancel_s))
                   if cancel_s is not None else None)
    terminal_pose = (min(poses, key=lambda p: abs(p[0] - terminal_s))
                     if terminal_s is not None else None)
    after_zero = (min(poses, key=lambda p: abs(p[0] - (terminal_s + 0.15)))
                  if terminal_s is not None else None)

    def displacement(a, b):
        return math.hypot(b[1] - a[1], b[2] - a[2])

    max_tf_drift = max((math.hypot(float(e["data"]["x"]), float(e["data"]["y"]))
                        for e in tf_events), default=math.inf)

    world_events = [e for e in events if e.get("event") == "gazebo_world_odom"]
    matched = []
    # Compare independently forwarded Gazebo world odometry against raw model
    # poses by their simulation stamps. This checks source/frame synchronization.
    raw_by_stamp = {round(p[0], 6): p for p in raw}
    for e in world_events:
        d = e.get("data", {})
        stamp = round(float(d.get("stamp_s", -100.0)), 6)
        p = raw_by_stamp.get(stamp)
        if p is not None and d.get("frame_id") == "world":
            matched.append((math.hypot(p[1] - float(d["x"]),
                                       p[2] - float(d["y"])),
                            abs(math.atan2(math.sin(p[3] - float(d["yaw"])),
                                           math.cos(p[3] - float(d["yaw"]))))))

    passed = (max(time_gaps) <= 0.05 and continuous_lower >= entrance_x
              and max_tf_drift <= 1e-6 and len(matched) >= 100
              and max(v[0] for v in matched) <= 1e-8
              and max(v[1] for v in matched) <= 1e-6
              and cancel_s is not None and terminal_s is not None
              and not post_cancel_nonzero)
    result = {
        "result": "PASS" if passed else "INCONCLUSIVE",
        "scenario": "stale_input_gazebo_world_pose_swept_footprint_audit",
        "evidence_source": (
            "/model/writer/pose (Gazebo model state), cross-checked against "
            "/writer/world_odom"),
        "frames": {
            "world": "world",
            "map_registration": (
                "Writer spawn pose maps to map (0,0,0); launch Nav2 start "
                "pose and map->odom TF checked"),
            "world_to_map_translation_m": [map_dx, map_dy],
            "world_to_map_yaw_rad": map_yaw,
            "initial_nav_pose_confirmed": initial_pose_confirmed,
            "max_map_to_odom_translation_m": max_tf_drift,
            "max_map_to_odom_component_abs": tf_max,
        },
        "sampled_ground_truth": {
            "samples": len(poses),
            "start_sim_s": poses[0][0],
            "end_sim_s": poses[-1][0],
            "max_sample_gap_s": max(time_gaps),
            "min_sample_gap_s": min(time_gaps),
        },
        "footprint_bound": {
            "configured_footprint": params["footprint"],
            "padding_m": pad,
            "circumradius_m": radius,
            "translation_uncertainty_m": uncertainty,
            "yaw_uncertainty_covered": "all headings by circumradius",
            "max_linear_speed_m_s": max_linear,
            "entrance_x_m": entrance_x,
            "conservative_continuous_lower_x_m": continuous_lower,
            "continuous_containment": continuous_lower >= entrance_x,
        },
        "fault_interval": {
            "bridge_paused_sim_s": pause_s,
            "bridge_resumed_sim_s": resume_s,
            "cancel_request_observed_sim_s": cancel_s,
            "cancel_terminal_observed_sim_s": terminal_s,
            "cancel_observation_uncertainty_s": 0.15,
            "ground_truth_samples": len(outage_poses),
            "map_center_start_at_pause_m": list(start[1:]),
            "map_center_end_before_recovery_m": list(end[1:]),
            "displacement_m": displacement(start, end),
            "yaw_change_rad": math.atan2(
                math.sin(end[3] - start[3]), math.cos(end[3] - start[3])),
            "nearest_cancel_pose_sim_s": cancel_pose[0] if cancel_pose else None,
            "motion_pause_to_cancel_request_m": (
                displacement(start, cancel_pose) if cancel_pose else None),
            "nearest_terminal_pose_sim_s": (
                terminal_pose[0] if terminal_pose else None),
            "motion_terminal_result_to_recovery_m": (
                displacement(terminal_pose, end) if terminal_pose else None),
            "motion_terminal_to_150ms_after_m": (
                displacement(terminal_pose, after_zero)
                if terminal_pose and after_zero else None),
            "command_messages_during_outage": len(outage_cmds),
            "nonzero_commands_after_cancel_settled_window": len(post_cancel_nonzero),
            "interpretation": (
                "Observed motion occurred during the stale-input outage. "
                "Ground truth records residual movement after cancellation/zero "
                "cmd_vel; no nonzero cmd_vel was observed after the settled "
                "cancellation window. Cancellation does not imply an "
                "instantaneous or hardware-guaranteed stop."),
        },
        "world_odom_crosscheck": {
            "matched_sim_stamps": len(matched),
            "max_position_residual_m": max(v[0] for v in matched) if matched else None,
            "max_yaw_residual_rad": max(v[1] for v in matched) if matched else None,
        },
        "limitations": [
            "Simulation evidence only; motion limits and the 5 cm allowance "
            "are assumptions, not Team 1 measured hardware limits.",
            "The ROS observer records cancel log events within an approximately "
            "0.15 s polling window.",
            "Nav2 cancel and zero cmd_vel do not prove a physical motor stop; "
            "a hardware lease/acknowledgment contract is required.",
        ],
    }
    output = args.output or run / "ground-truth-audit.json"
    output.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n")
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0 if passed else 2


if __name__ == "__main__":
    raise SystemExit(main())
