#!/usr/bin/env python3
"""ROS helper for the bounded PR #1 external-cancel Gazebo scenario."""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import re
import sys
import time

import rclpy
from action_msgs.srv import CancelGoal
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from rclpy.qos import (QoSProfile, DurabilityPolicy, ReliabilityPolicy,
                       HistoryPolicy, qos_profile_sensor_data)
from std_msgs.msg import Bool, String
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener
import yaml



def _qos_state() -> QoSProfile:
    return QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                      reliability=ReliabilityPolicy.RELIABLE,
                      history=HistoryPolicy.KEEP_LAST)


def _yaw(q: Quaternion) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def _cancel() -> int:
    rclpy.init()
    node = rclpy.create_node("team4_pr1_external_cancel_probe")
    try:
        client = node.create_client(CancelGoal,
                                    "/writer/navigate_to_pose/_action/cancel_goal")
        if not client.wait_for_service(timeout_sec=10.0):
            raise RuntimeError("Nav2 cancel service was not available within 10 s")
        request = CancelGoal.Request()
        # ROS action convention: zero UUID and zero timestamp requests all goals.
        request.goal_info.goal_id.uuid = [0] * 16
        request.goal_info.stamp.sec = 0
        request.goal_info.stamp.nanosec = 0
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
        if not future.done():
            raise RuntimeError("Nav2 cancel service did not respond within 5 s")
        response = future.result()
        count = len(response.goals_canceling)
        print(json.dumps({"goals_canceling": count,
                          "return_code": int(response.return_code)}), flush=True)
        return count
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def _request_return() -> None:
    rclpy.init()
    node = rclpy.create_node("team4_pr1_return_probe")
    try:
        publisher = node.create_publisher(Bool, "/living_map/writer_return_request",
                                          _qos_state())
        deadline = time.monotonic() + 10.0
        next_publish = 0.0
        published = False
        while time.monotonic() < deadline:
            if time.monotonic() >= next_publish:
                # Transient-local durability retains the request while the
                # ROS graph discovers the Writer subscriber.
                publisher.publish(Bool(data=True))
                published = True
                next_publish = time.monotonic() + 0.5
            rclpy.spin_once(node, timeout_sec=0.1)
            if published and publisher.get_subscription_count() > 0:
                break
        matches = publisher.get_subscription_count()
        if matches == 0 or not published:
            raise RuntimeError("Writer return-request subscriber was not discovered")
        rclpy.spin_once(node, timeout_sec=0.5)
        print(json.dumps({"return_request_published": True,
                          "matched_subscribers": matches}),
              flush=True)
    finally:
        node.destroy_node()
        rclpy.shutdown()


def _observe(output: Path, launch_log: Path, config: Path) -> None:
    from living_map_writer.entrance import padded_footprint, footprint_min_x
    from living_map_writer.swept_footprint import swept_min_x

    params = yaml.safe_load(config.read_text())["/writer/frontier_explorer"]["ros__parameters"]
    footprint = padded_footprint(params["footprint"],
                                 float(params["footprint_padding"]))
    entrance_x = float(params["entrance_x"])
    position_uncertainty = float(params.get("entrance_position_uncertainty_xy", 0.0))
    rclpy.init()
    node = rclpy.create_node("team4_pr1_cancel_observer")
    tf_buffer = Buffer()
    tf_listener = TransformListener(tf_buffer, node)
    output.parent.mkdir(parents=True, exist_ok=True)
    stream = output.open("a", encoding="utf-8", buffering=1)
    state = ""
    stuck = False
    sim_s: float | None = None
    last_health = time.monotonic()
    minimum_footprint_x = math.inf
    minimum_uncertain_footprint_x = math.inf
    minimum_swept_footprint_x = math.inf
    minimum_swept_uncertain_footprint_x = math.inf
    odom_samples = 0
    boundary_transform_missing = 0
    snapshot_transform_samples = 0
    maximum_tf_alignment_lag_s = 0.0
    odom_gap_count = 0
    unmapped_motion_samples = 0
    maximum_odom_gap_s = 0.0
    last_odom_stamp_s: float | None = None
    last_raw_pose: tuple[float, float, float] | None = None
    path_length_m = 0.0
    last_xy: tuple[float, float] | None = None
    last_pose: tuple[float, float, float] | None = None
    log_offset = 0

    def emit(event: str, data=None) -> None:
        stream.write(json.dumps({"event": event, "sim_s": sim_s, "data": data},
                                sort_keys=True) + "\n")

    def on_clock(message: Clock) -> None:
        nonlocal sim_s
        sim_s = message.clock.sec + message.clock.nanosec / 1e9

    def on_state(message: String) -> None:
        nonlocal state
        state = message.data
        emit("state", state)

    def on_stuck(message: Bool) -> None:
        nonlocal stuck
        stuck = bool(message.data)
        emit("stuck", stuck)

    def on_summary(message: String) -> None:
        try:
            payload = json.loads(message.data)
        except (ValueError, TypeError):
            emit("summary_invalid", message.data)
        else:
            emit("summary", payload)

    def on_odom(message: Odometry) -> None:
        nonlocal minimum_footprint_x, minimum_uncertain_footprint_x
        nonlocal minimum_swept_footprint_x, minimum_swept_uncertain_footprint_x
        nonlocal odom_samples, path_length_m, last_xy, last_pose
        nonlocal boundary_transform_missing, snapshot_transform_samples, odom_gap_count
        nonlocal maximum_odom_gap_s, last_odom_stamp_s
        nonlocal maximum_tf_alignment_lag_s, unmapped_motion_samples, last_raw_pose
        odom_stamp = message.header.stamp.sec + message.header.stamp.nanosec / 1e9
        if last_odom_stamp_s is not None:
            gap = odom_stamp - last_odom_stamp_s
            if gap > 0.5:  # SIMULATION ASSUMPTION: five 10 Hz odometry periods.
                odom_gap_count += 1
                maximum_odom_gap_s = max(maximum_odom_gap_s, gap)
        last_odom_stamp_s = odom_stamp
        odom_x = float(message.pose.pose.position.x)
        odom_y = float(message.pose.pose.position.y)
        odom_yaw = _yaw(message.pose.pose.orientation)
        raw_pose = (odom_x, odom_y, odom_yaw)
        transform = None
        try:
            # SLAM publishes map->odom less frequently than odometry. Query its
            # latest transform and admit only a bounded simulation-age skew.
            transform = tf_buffer.lookup_transform("map", "writer/odom", Time())
        except TransformException:
            pass
        if transform is not None:
            transform_stamp = transform.header.stamp.sec + transform.header.stamp.nanosec / 1e9
            if transform_stamp > 0.0:
                alignment_lag = abs(odom_stamp - transform_stamp)
                maximum_tf_alignment_lag_s = max(maximum_tf_alignment_lag_s, alignment_lag)
                if alignment_lag > 0.5:  # SIMULATION ASSUMPTION: five 10 Hz pose periods.
                    transform = None
        if transform is not None:
            tx = float(transform.transform.translation.x)
            ty = float(transform.transform.translation.y)
            transform_yaw = _yaw(transform.transform.rotation)
        else:
            # The controlled cooldown harness records this immutable frame
            # alignment while holding the Writer still. It safely bridges the
            # brief SLAM deactivation/static-TF handover in this simulation.
            snapshot_path = output.parent / "map-to-odom.json"
            try:
                snapshot = json.loads(snapshot_path.read_text())
            except (OSError, ValueError):
                if (last_raw_pose is not None
                        and (math.hypot(odom_x-last_raw_pose[0], odom_y-last_raw_pose[1]) > .002
                             or abs(math.atan2(math.sin(odom_yaw-last_raw_pose[2]),
                                               math.cos(odom_yaw-last_raw_pose[2]))) > .005)):
                    # Simulation instrumentation floor: ideal odometry jitter
                    # below 2 mm / 0.005 rad is not treated as actual motion.
                    unmapped_motion_samples += 1
                last_raw_pose = raw_pose
                boundary_transform_missing += 1
                return
            tx = float(snapshot["x"])
            ty = float(snapshot["y"])
            transform_yaw = float(snapshot["yaw"])
            snapshot_transform_samples += 1
        last_raw_pose = raw_pose
        c, s = math.cos(transform_yaw), math.sin(transform_yaw)
        x = tx + c * odom_x - s * odom_y
        y = ty + s * odom_x + c * odom_y
        yaw = transform_yaw + odom_yaw
        minimum_footprint_x = min(minimum_footprint_x,
                                  footprint_min_x(x, yaw, footprint))
        minimum_uncertain_footprint_x = min(
            minimum_uncertain_footprint_x,
            footprint_min_x(x - position_uncertainty, yaw, footprint))
        pose = (x, y, yaw)
        if last_pose is None:
            minimum_swept_footprint_x = min(minimum_swept_footprint_x,
                                             footprint_min_x(x, yaw, footprint))
            minimum_swept_uncertain_footprint_x = min(
                minimum_swept_uncertain_footprint_x,
                footprint_min_x(x - position_uncertainty, yaw, footprint))
        else:
            minimum_swept_footprint_x = min(
                minimum_swept_footprint_x,
                swept_min_x(last_pose, pose, footprint))
            minimum_swept_uncertain_footprint_x = min(
                minimum_swept_uncertain_footprint_x,
                swept_min_x(last_pose, pose, footprint, position_uncertainty))
        last_pose = pose
        xy = (x, y)
        if last_xy is not None:
            path_length_m += math.hypot(x - last_xy[0], y - last_xy[1])
        last_xy = xy
        odom_samples += 1

    node.create_subscription(Clock, "/clock", on_clock, 10)
    node.create_subscription(String, "/living_map/writer_explorer_state",
                             on_state, _qos_state())
    node.create_subscription(Bool, "/living_map/writer_stuck", on_stuck,
                             _qos_state())
    node.create_subscription(String, "/living_map/writer_exploration_summary",
                             on_summary, _qos_state())
    node.create_subscription(Odometry, "/writer/odom", on_odom,
                             qos_profile_sensor_data)
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            try:
                with launch_log.open("r", encoding="utf-8", errors="replace") as log:
                    log.seek(log_offset)
                    lines = log.readlines()
                    log_offset = log.tell()
            except FileNotFoundError:
                lines = []
            for line in lines:
                if "Frontier selected x=" in line:
                    match = re.search(r"Frontier selected x=([-+0-9.eE]+) y=([-+0-9.eE]+)", line)
                    if match:
                        emit("frontier_selected", {"x": float(match.group(1)),
                                                    "y": float(match.group(2))})
                if "frontier goal at (" in line and "cancelled externally" in line:
                    match = re.search(r"frontier goal at \(([-+0-9.eE]+),([-+0-9.eE]+)\) cancelled externally \(([^)]*)\)", line)
                    if match:
                        emit("frontier_cancelled", {"x": float(match.group(1)),
                                                     "y": float(match.group(2)),
                                                     "reason": match.group(3)})
            if time.monotonic() - last_health >= 0.5:
                emit("health", {"state": state, "stuck": stuck,
                                 "minimum_footprint_x": minimum_footprint_x,
                                 "minimum_uncertain_footprint_x": minimum_uncertain_footprint_x,
                                 "minimum_swept_footprint_x": minimum_swept_footprint_x,
                                 "minimum_swept_uncertain_footprint_x": minimum_swept_uncertain_footprint_x,
                                 "position_uncertainty_xy": position_uncertainty,
                                 "entrance_x": entrance_x,
                                 "boundary_crossed": minimum_uncertain_footprint_x < entrance_x,
                                 "odom_samples": odom_samples,
                                 "boundary_transform_missing": boundary_transform_missing,
                                 "snapshot_transform_samples": snapshot_transform_samples,
                                 "maximum_tf_alignment_lag_s": maximum_tf_alignment_lag_s,
                                 "odom_gap_count": odom_gap_count,
                                 "unmapped_motion_samples": unmapped_motion_samples,
                                 "maximum_odom_gap_s": maximum_odom_gap_s,
                                 "path_length_m": path_length_m})
                last_health = time.monotonic()
    except KeyboardInterrupt:
        pass
    finally:
        emit("final", {"state": state, "stuck": stuck,
                       "minimum_footprint_x": minimum_footprint_x,
                       "minimum_uncertain_footprint_x": minimum_uncertain_footprint_x,
                       "minimum_swept_footprint_x": minimum_swept_footprint_x,
                       "minimum_swept_uncertain_footprint_x": minimum_swept_uncertain_footprint_x,
                       "position_uncertainty_xy": position_uncertainty,
                       "entrance_x": entrance_x,
                       "boundary_crossed": minimum_uncertain_footprint_x < entrance_x,
                       "odom_samples": odom_samples,
                       "boundary_transform_missing": boundary_transform_missing,
                       "snapshot_transform_samples": snapshot_transform_samples,
                       "maximum_tf_alignment_lag_s": maximum_tf_alignment_lag_s,
                       "odom_gap_count": odom_gap_count,
                       "unmapped_motion_samples": unmapped_motion_samples,
                       "maximum_odom_gap_s": maximum_odom_gap_s,
                       "path_length_m": path_length_m})
        stream.close()
        node.destroy_node()
        rclpy.shutdown()


def _hold_after_cancel(launch_log: Path, paused_file: Path, resume_file: Path,
                       transform_file: Path, cancel_first: bool) -> None:
    """Pause exploration immediately after an external cancel, then await resume."""
    rclpy.init()
    node = rclpy.create_node("team4_pr1_cancel_hold_controller")
    publisher = node.create_publisher(Bool, "/living_map/writer_enabled", _qos_state())
    tf_buffer = Buffer()
    tf_listener = TransformListener(tf_buffer, node)
    state = ""
    cancel_seen = False
    log_offset = 0

    def on_state(message: String) -> None:
        nonlocal state
        state = message.data

    node.create_subscription(String, "/living_map/writer_explorer_state", on_state,
                             _qos_state())
    paused_file.parent.mkdir(parents=True, exist_ok=True)
    try:
        if cancel_first:
            client = node.create_client(
                CancelGoal, "/writer/navigate_to_pose/_action/cancel_goal")
            if not client.wait_for_service(timeout_sec=60.0):
                raise RuntimeError("Nav2 cancel service did not become available")
            goal_deadline = time.monotonic() + 150.0
            selected = False
            while rclpy.ok() and time.monotonic() < goal_deadline and not selected:
                rclpy.spin_once(node, timeout_sec=0.01)
                try:
                    with launch_log.open("r", encoding="utf-8", errors="replace") as log:
                        log.seek(log_offset)
                        lines = log.readlines()
                        log_offset = log.tell()
                except FileNotFoundError:
                    lines = []
                selected = any("Frontier selected x=" in line for line in lines)
            if not selected:
                raise RuntimeError("no first frontier goal appeared before cancellation bound")
            request = CancelGoal.Request()
            request.goal_info.goal_id.uuid = [0] * 16
            request.goal_info.stamp.sec = 0
            request.goal_info.stamp.nanosec = 0
            future = client.call_async(request)
            rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
            if not future.done():
                raise RuntimeError("first-goal cancel request timed out")
            response = future.result()
            count = len(response.goals_canceling)
            (paused_file.parent / "cancel-response.json").write_text(
                json.dumps({"goals_canceling": count,
                            "return_code": int(response.return_code)}) + "\n")
            if count < 1:
                raise RuntimeError("first frontier goal was no longer active at cancellation")
            print(f"first_goal_cancel_confirmed count={count}", flush=True)
        deadline = time.monotonic() + 120.0
        while rclpy.ok() and time.monotonic() < deadline and not cancel_seen:
            rclpy.spin_once(node, timeout_sec=0.05)
            try:
                with launch_log.open("r", encoding="utf-8", errors="replace") as log:
                    log.seek(log_offset)
                    lines = log.readlines()
                    log_offset = log.tell()
            except FileNotFoundError:
                lines = []
            cancel_seen = any("frontier goal at (" in line
                              and "cancelled externally (no reason)" in line
                              for line in lines)
        if not cancel_seen:
            raise RuntimeError("timed out waiting for unexplained external cancellation")
        if publisher.get_subscription_count() == 0:
            raise RuntimeError("Writer enabled-state subscriber was not discovered")
        pause_deadline = time.monotonic() + 8.0
        while rclpy.ok() and state != "PAUSED" and time.monotonic() < pause_deadline:
            publisher.publish(Bool(data=False))
            rclpy.spin_once(node, timeout_sec=0.05)
        if state != "PAUSED":
            raise RuntimeError(f"Writer did not confirm PAUSED state (state={state!r})")
        tf_deadline = time.monotonic() + 8.0
        transform = None
        while rclpy.ok() and time.monotonic() < tf_deadline:
            try:
                transform = tf_buffer.lookup_transform("map", "writer/odom", Time())
                break
            except TransformException:
                rclpy.spin_once(node, timeout_sec=0.05)
        if transform is None:
            raise RuntimeError("could not capture current map-to-odom transform")
        q = transform.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        transform_file.write_text(json.dumps({
            "x": transform.transform.translation.x,
            "y": transform.transform.translation.y,
            "z": transform.transform.translation.z,
            "yaw": yaw,
        }) + "\n")
        paused_file.write_text(json.dumps({"confirmed_state": state,
                                           "wall_time": time.time()}) + "\n")
        print("controlled_pause_confirmed", flush=True)
        while rclpy.ok() and not resume_file.exists():
            rclpy.spin_once(node, timeout_sec=0.1)
        if not rclpy.ok():
            return
        resume_deadline = time.monotonic() + 8.0
        while rclpy.ok() and state != "SEARCHING" and time.monotonic() < resume_deadline:
            publisher.publish(Bool(data=True))
            rclpy.spin_once(node, timeout_sec=0.05)
        if state != "SEARCHING":
            raise RuntimeError(f"Writer did not resume exploration (state={state!r})")
        print("controlled_resume_confirmed", flush=True)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="mode", required=True)
    sub.add_parser("cancel-all")
    sub.add_parser("request-return")
    observe = sub.add_parser("observe")
    observe.add_argument("--output", type=Path, required=True)
    observe.add_argument("--launch-log", type=Path, required=True)
    observe.add_argument("--config", type=Path, required=True)
    hold = sub.add_parser("hold-after-cancel")
    hold.add_argument("--launch-log", type=Path, required=True)
    hold.add_argument("--paused-file", type=Path, required=True)
    hold.add_argument("--resume-file", type=Path, required=True)
    hold.add_argument("--transform-file", type=Path, required=True)
    hold.add_argument("--cancel-first-frontier", action="store_true")
    args = parser.parse_args()
    if args.mode == "cancel-all":
        _cancel()
        return 0
    if args.mode == "request-return":
        _request_return()
        return 0
    if args.mode == "observe":
        _observe(args.output, args.launch_log, args.config)
        return 0
    _hold_after_cancel(args.launch_log, args.paused_file, args.resume_file,
                       args.transform_file, args.cancel_first_frontier)
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except Exception as exc:
        print(f"FAIL: {exc}", file=sys.stderr, flush=True)
        sys.exit(1)
