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


def _jsonl(path: Path):
    return [json.loads(line) for line in path.read_text().splitlines() if line.strip()]


def _phase_poses(path: Path, spawn: list[float]):
    raw = _parse_pose_stream(path)
    return [(t, x - spawn[0], y - spawn[1], yaw - spawn[5])
            for t, x, y, yaw in raw]


def _continuous_bound(poses, vmax: float, radius: float, uncertainty: float):
    gaps = [b[0] - a[0] for a, b in zip(poses, poses[1:])]
    lower = min(min(a[1], b[1]) - vmax * (b[0] - a[0]) - radius - uncertainty
                for a, b in zip(poses, poses[1:]))
    return lower, max(gaps, default=math.inf)


def _frozen_phase_pass(phase: str, info: dict, position_tolerance: float,
                       yaw_tolerance: float) -> bool:
    stream = info["frozen_stream"]
    valid_fault = (
        info["fresh_stamped_pose_mismatch_latched"]
        and info["detection_delay_after_injection_s"] is not None
        and info["no_new_frontier_goal_while_locked"]
        and info["explicit_recovery_acknowledged"]
        and info["home_after_operator_ack"]
        and stream["freshly_stamped_pose_stream"]
        and stream["reported_pose_frozen"]
        and info["lockout_preceded_any_center_boundary_crossing"]
    )
    if not valid_fault:
        return False
    if phase.endswith("rotation"):
        return (
            stream["independent_reference_yaw_range_rad"] > yaw_tolerance
            and info["pre_fault_motion_measurement"] is not None
            and info["pre_fault_motion_measurement"] > yaw_tolerance
            and info["lockout_rotation_disagreement_rad"] is not None
            and info["lockout_rotation_disagreement_rad"] > yaw_tolerance
            and info["pre_fault_rotation_evidence"]
        )
    return (
        info["pre_fault_motion_measurement"] is not None
        and info["pre_fault_motion_measurement"] > 0.02
        and info["lockout_translation_disagreement_m"] is not None
        and info["lockout_translation_disagreement_m"] > position_tolerance
    )


def _frozen_stream_audit(run: Path, start_s: float, end_s: float,
                         freeze_rotation: bool):
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
    world_odom = []
    command_count = 0
    command_nonzero = 0
    linear_command_count = 0
    angular_command_count = 0
    while reader.has_next():
        topic, payload, _receive_stamp = reader.read_next()
        message = deserialize_message(payload, types[topic])
        if topic == "/clock":
            sim_s = message.clock.sec + message.clock.nanosec / 1e9
        elif topic == "/writer/odom":
            stamp = message.header.stamp.sec + message.header.stamp.nanosec / 1e9
            if start_s <= stamp <= end_s:
                p = message.pose.pose.position
                q = message.pose.pose.orientation
                yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
                odom.append((stamp, p.x, p.y, yaw))
        elif topic == "/writer/world_odom":
            stamp = message.header.stamp.sec + message.header.stamp.nanosec / 1e9
            if start_s <= stamp <= end_s:
                p = message.pose.pose.position
                q = message.pose.pose.orientation
                yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
                world_odom.append((stamp, p.x, p.y, yaw))
        elif topic == "/writer/cmd_vel" and sim_s is not None and start_s <= sim_s <= end_s:
            command_count += 1
            if abs(message.linear.x) > .1:
                linear_command_count += 1
            if abs(message.angular.z) > .1:
                angular_command_count += 1
            if abs(message.linear.x) > .02 or abs(message.angular.z) > .1:
                command_nonzero += 1
    if len(odom) < 10:
        raise ValueError(f"frozen pose interval has too few odometry samples: {len(odom)}")
    positions = [(x, y) for _, x, y, _ in odom]
    pose_range = max(math.hypot(x1-x2, y1-y2)
                     for x1, y1 in positions for x2, y2 in positions)
    yaws = [yaw for _, _, _, yaw in odom]
    yaw_range = max(abs(math.atan2(math.sin(a-b), math.cos(a-b)))
                    for a in yaws for b in yaws)
    world_positions = [(x, y) for _, x, y, _ in world_odom]
    world_position_range = (max(math.hypot(x1-x2, y1-y2)
                                for x1, y1 in world_positions
                                for x2, y2 in world_positions)
                            if world_positions else 0.0)
    world_yaws = [yaw for _, _, _, yaw in world_odom]
    world_yaw_range = (max(abs(math.atan2(math.sin(a-b), math.cos(a-b)))
                           for a in world_yaws for b in world_yaws)
                       if world_yaws else 0.0)
    return {
        "odom_samples": len(odom),
        "odom_stamp_span_s": odom[-1][0] - odom[0][0],
        "frozen_odom_position_range_m": pose_range,
        "frozen_odom_yaw_range_rad": yaw_range,
        "cmd_vel_samples": command_count,
        "nonzero_cmd_vel_samples": command_nonzero,
        "nonzero_linear_cmd_vel_samples": linear_command_count,
        "nonzero_angular_cmd_vel_samples": angular_command_count,
        "independent_reference_position_range_m": world_position_range,
        "independent_reference_yaw_range_rad": world_yaw_range,
        "freshly_stamped_pose_stream": odom[-1][0] - odom[0][0] >= 1.0,
        "reported_pose_frozen": (yaw_range <= 1e-6 if freeze_rotation
                                  else pose_range <= 1e-6),
    }


def _stationary_stream_audit(run: Path, start_s: float, end_s: float):
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
    estimate, reference = [], []
    command_samples = nonzero_commands = 0
    while reader.has_next():
        topic, payload, _receive_stamp = reader.read_next()
        message = deserialize_message(payload, types[topic])
        if topic == "/clock":
            sim_s = message.clock.sec + message.clock.nanosec / 1e9
        elif topic in ("/writer/odom", "/writer/world_odom"):
            stamp = message.header.stamp.sec + message.header.stamp.nanosec / 1e9
            if not start_s <= stamp <= end_s:
                continue
            p, q = message.pose.pose.position, message.pose.pose.orientation
            yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                             1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            target = estimate if topic == "/writer/odom" else reference
            target.append((stamp, p.x, p.y, yaw))
        elif topic == "/writer/cmd_vel" and sim_s is not None and start_s <= sim_s <= end_s:
            command_samples += 1
            if abs(message.linear.x) > .02 or abs(message.angular.z) > .1:
                nonzero_commands += 1

    def ranges(samples):
        positions = [(x, y) for _, x, y, _ in samples]
        yaws = [yaw for _, _, _, yaw in samples]
        position = (max(math.hypot(x1-x2, y1-y2)
                        for x1, y1 in positions for x2, y2 in positions)
                    if positions else math.inf)
        angle = (max(abs(math.atan2(math.sin(a-b), math.cos(a-b)))
                     for a in yaws for b in yaws) if yaws else math.inf)
        return position, angle

    estimate_position, estimate_yaw = ranges(estimate)
    reference_position, reference_yaw = ranges(reference)
    span = min(estimate[-1][0] - estimate[0][0] if estimate else 0.0,
               reference[-1][0] - reference[0][0] if reference else 0.0)
    passed = (
        len(estimate) >= 10 and len(reference) >= 10 and span >= 0.5
        and estimate_position <= 0.05 and estimate_yaw <= 0.02
        and reference_position <= 0.05 and reference_yaw <= 0.02
        and command_samples > 0 and nonzero_commands == 0
    )
    return {
        "window_sim_start_s": start_s, "window_sim_end_s": end_s,
        "estimate_samples": len(estimate), "reference_samples": len(reference),
        "common_observation_span_s": span,
        "estimate_position_range_m": estimate_position,
        "estimate_yaw_range_rad": estimate_yaw,
        "reference_position_range_m": reference_position,
        "reference_yaw_range_rad": reference_yaw,
        "cmd_vel_samples": command_samples,
        "nonzero_cmd_vel_samples": nonzero_commands,
        "result": "PASS" if passed else "INCONCLUSIVE",
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
    position_tolerance = float(params["pose_integrity_position_tolerance_m"])
    yaw_tolerance = float(params["pose_integrity_yaw_tolerance_rad"])

    world = ET.parse(args.world).getroot()
    include = next(e for e in world.iter("include")
                   if any(c.tag == "name" and c.text == "writer" for c in e))
    spawn = [float(v) for v in include.findtext("pose").split()]
    model = ET.parse(args.model).getroot()
    vmax = float(model.findtext(".//plugin/max_linear_velocity"))

    phase_results = {}
    phase_names = ("stale-recovery", "fresh-stamped-frozen-translation",
                   "fresh-stamped-frozen-rotation")
    pose_quality = True
    stationary_results = {}
    for phase in phase_names:
        stationary_dir = args.run / phase
        stationary_window = (stationary_dir / "stationary-window.txt").read_text()
        stationary_start = float(re.search(
            r"start_sim_s=([0-9.eE+-]+)", stationary_window).group(1))
        stationary_end = float(re.search(
            r"end_sim_s=([0-9.eE+-]+)", stationary_window).group(1))
        stationary_results[phase] = _stationary_stream_audit(
            stationary_dir, stationary_start, stationary_end)
        path = args.run / phase / "ground-truth-pose.txt"
        poses = _phase_poses(path, spawn)
        lower, gap = _continuous_bound(poses, vmax, radius, uncertainty)
        center_min_pose = min(poses, key=lambda p: p[1])
        crossing = next((p[0] for p in poses if p[1] < entrance_x), None)
        phase_results[phase] = {
            "ground_truth_samples": len(poses),
            "simulation_start_s": poses[0][0],
            "simulation_end_s": poses[-1][0],
            "max_ground_truth_gap_s": gap,
            "minimum_center_x_m": center_min_pose[1],
            "minimum_center_x_at_s": center_min_pose[0],
            "first_center_boundary_crossing_s": crossing,
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
    cancellations = (recovery_summary["localization_cancel_events"]
                     + recovery_summary.get("pose_integrity_lockouts", 0) >= 3)
    no_goal_while_invalid = all(
        int(a) == int(b) for a, b in re.findall(
            r"goals_after_fault_mode=(\d+)\s+goals_after_lockout_window=(\d+)",
            mode_lines)) and len(re.findall(r"goals_after_fault_mode=", mode_lines)) >= 3
    recovery_seen = len(re.findall(r"goals_after_recovery=", mode_lines)) >= 2
    home = recovery_summary["home_observed"]

    frozen_results = {}
    for phase in phase_names[1:]:
        frozen_dir = args.run / phase
        frozen_fault = (frozen_dir / "fault-injection.txt").read_text()
        start_s = float(re.search(
            r"frozen_start_sim_s=([0-9.eE+-]+)", frozen_fault).group(1))
        lockout_s = float(re.search(
            r"lockout_sim_s=([0-9.eE+-]+)", frozen_fault).group(1))
        frozen_end = float(re.search(
            r"frozen_end_sim_s=([0-9.eE+-]+)", frozen_fault).group(1))
        frozen_stream = _frozen_stream_audit(
            frozen_dir, start_s, frozen_end, phase.endswith("rotation"))
        log = (frozen_dir / "launch.log").read_text(errors="replace")
        timeline = (frozen_dir / "mode-timeline.txt").read_text(errors="replace")
        summary = json.loads((frozen_dir / "summary.json").read_text())
        motion = re.search(
            r"pre_fault_motion=sim_s=([0-9.eE+-]+); (displacement_m|yaw_range_rad)=([0-9.eE+-]+)",
            timeline)
        motion_value = float(motion.group(3)) if motion else None
        translation_error = re.search(
            r"independent odometry motion disagrees with Writer pose by ([0-9.]+) m",
            log)
        rotation_error = re.search(
            r"independent odometry rotation disagrees with Writer pose by ([0-9.]+) rad",
            log)
        state_events = [e for e in _jsonl(frozen_dir / "observations.jsonl")
                        if e.get("event") == "state"
                        and e.get("data") == "NAVIGATION_UNCERTAIN"
                        and e.get("sim_s") is not None
                        and e["sim_s"] >= start_s]
        lockout_event = min(state_events, key=lambda e: e["sim_s"]) if state_events else None
        lockout_delta_s = lockout_s - start_s
        goal_counts = re.search(
            r"frontier_goals_at_lockout=(\d+)\s+frontier_goals_after_lockout_window=(\d+)",
            timeline)
        no_goals_while_locked = bool(goal_counts and goal_counts.group(1) == goal_counts.group(2))
        recovery_ack = (frozen_dir / "recovery-ack.txt").is_file()
        first_crossing = phase_results[phase]["first_center_boundary_crossing_s"]
        frozen_results[phase] = {
            "fresh_stamped_pose_mismatch_latched": bool(
                "pose integrity could not be established" in log and lockout_s >= start_s),
            "lockout_sim_time_s": lockout_s,
            "detection_delay_after_injection_s": lockout_delta_s,
            "lockout_preceded_any_center_boundary_crossing": (
                first_crossing is None or lockout_event is not None
                and lockout_event["sim_s"] < first_crossing),
            "center_boundary_crossing_s": first_crossing,
            "no_new_frontier_goal_while_locked": no_goals_while_locked,
            "explicit_recovery_acknowledged": recovery_ack,
            "home_after_operator_ack": summary["home_observed"],
            "frozen_stream": frozen_stream,
            "pre_fault_motion_measurement": motion_value,
            "lockout_translation_disagreement_m": (
                float(translation_error.group(1)) if translation_error else None),
            "lockout_rotation_disagreement_rad": (
                float(rotation_error.group(1)) if rotation_error else None),
            "pre_fault_rotation_evidence": (
                bool(motion and motion.group(2) == "yaw_range_rad")
                if phase.endswith("rotation") else None),
        }

    coordinate_events = []
    coordinate_valid_events = []
    fault_coordinate_events = []
    for phase in phase_names:
        phase_dir = args.run / phase
        if phase == "stale-recovery":
            fault_text = (phase_dir / "fault-injection.txt").read_text()
            fault_start_s = float(re.search(
                r"paused_sim_s=([0-9.eE+-]+)", fault_text).group(1))
        else:
            fault_text = (phase_dir / "fault-injection.txt").read_text()
            fault_start_s = float(re.search(
                r"frozen_start_sim_s=([0-9.eE+-]+)", fault_text).group(1))
        for event in _jsonl(phase_dir / "observations.jsonl"):
            if (event.get("event") != "map_to_odom"
                    or not isinstance(event.get("data"), dict)
                    or event.get("sim_s") is None):
                continue
            coordinate_events.append(event)
            target = (coordinate_valid_events
                      if float(event["sim_s"]) < fault_start_s
                      else fault_coordinate_events)
            target.append(event)

    def coordinate_ranges(events):
        return {
            "samples": len(events),
            "maximum_translation_from_registered_odom_m": max((
                math.hypot(float(e["data"]["x"]), float(e["data"]["y"]))
                for e in events), default=math.inf),
            "maximum_absolute_yaw_from_registered_odom_rad": max((
                abs(float(e["data"]["yaw"])) for e in events), default=math.inf),
        }

    coordinate_alignment = coordinate_ranges(coordinate_events)
    coordinate_alignment_before_faults = coordinate_ranges(coordinate_valid_events)
    coordinate_fault_drift = coordinate_ranges(fault_coordinate_events)

    frozen_detection_pass = all(
        _frozen_phase_pass(phase, info, position_tolerance, yaw_tolerance)
        for phase, info in frozen_results.items())
    frozen_detection_pass &= all(
        phase_results[phase]["first_center_boundary_crossing_s"] is None
        and phase_results[phase]["continuous_containment"]
        for phase in phase_names[1:])
    coordinate_alignment_ok = (
        coordinate_alignment["samples"] > 0
        and coordinate_alignment["maximum_translation_from_registered_odom_m"] <= 0.05
        and coordinate_alignment["maximum_absolute_yaw_from_registered_odom_rad"] <= 0.02)

    result = {
        "scenario": "pose_integrity_fault_injection_and_safe_recovery_gazebo",
        "result": "PASS" if (mode_seen and cancellations and no_goal_while_invalid
                               and recovery_seen and home and pose_quality
                               and frozen_detection_pass and coordinate_alignment_ok
                               and all(x["result"] == "PASS"
                                       for x in stationary_results.values())) else "INCONCLUSIVE",
        "simulation_only": True,
        "pose_fault_modes_seen": mode_seen,
        "localization_cancellations_at_least_three": cancellations,
        "no_new_goal_while_pose_invalid": no_goal_while_invalid,
        "navigation_resumed_after_valid_pose": recovery_seen,
        "home_after_missing_pose_recovery": home,
        "stationary_robot": stationary_results,
        "frozen_pose_faults": frozen_results,
        "coordinate_frame_alignment": coordinate_alignment,
        "coordinate_frame_alignment_before_faults": coordinate_alignment_before_faults,
        "coordinate_frame_drift_during_faults": coordinate_fault_drift,
        "footprint_assumptions": {
            "footprint": params["footprint"], "padding_m": pad,
            "translation_uncertainty_m": uncertainty,
            "max_linear_speed_m_s": vmax,
            "basis": "Gazebo spawn-to-map registration; full padded-footprint circumradius covers every heading; vmax bounds intersample center motion",
        },
        "phases": phase_results,
        "limitations": [
            "0.5 s pose age, 0.35 s scan age and 1.5 s injected delay are simulation assumptions, not hardware limits.",
            "Pose reference axes, 50 ms timestamp pairing, 5 cm position agreement and 0.02 rad yaw agreement are explicit simulation assumptions pending Team 1 confirmation.",
            "Gazebo world odometry is independently derived from simulation physics; a hardware reference must be independently sourced or goals remain blocked.",
            "Nav2 cancellation and the simulated zero command do not prove physical motor stopping.",
            "The 0.05 m and 0.02 rad disagreement thresholds reuse existing uncertainty/noise settings; hardware values require measurement and approval.",
        ],
    }
    args.output.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n")
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0 if result["result"] == "PASS" else 2


if __name__ == "__main__":
    raise SystemExit(main())
