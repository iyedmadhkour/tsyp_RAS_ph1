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

    params = yaml.safe_load(config.read_text())["/writer/frontier_explorer"]["ros__parameters"]
    footprint = padded_footprint(params["footprint"],
                                 float(params["footprint_padding"]))
    entrance_x = float(params["entrance_x"])
    rclpy.init()
    node = rclpy.create_node("team4_pr1_cancel_observer")
    output.parent.mkdir(parents=True, exist_ok=True)
    stream = output.open("a", encoding="utf-8", buffering=1)
    state = ""
    stuck = False
    sim_s: float | None = None
    last_health = time.monotonic()
    minimum_footprint_x = math.inf
    odom_samples = 0
    path_length_m = 0.0
    last_xy: tuple[float, float] | None = None
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
        nonlocal minimum_footprint_x, odom_samples, path_length_m, last_xy
        x = float(message.pose.pose.position.x)
        y = float(message.pose.pose.position.y)
        minimum_footprint_x = min(minimum_footprint_x,
                                  footprint_min_x(x, _yaw(message.pose.pose.orientation),
                                                  footprint))
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
                                 "entrance_x": entrance_x,
                                 "boundary_crossed": minimum_footprint_x < entrance_x,
                                 "odom_samples": odom_samples,
                                 "path_length_m": path_length_m})
                last_health = time.monotonic()
    except KeyboardInterrupt:
        pass
    finally:
        emit("final", {"state": state, "stuck": stuck,
                       "minimum_footprint_x": minimum_footprint_x,
                       "entrance_x": entrance_x,
                       "boundary_crossed": minimum_footprint_x < entrance_x,
                       "odom_samples": odom_samples,
                       "path_length_m": path_length_m})
        stream.close()
        node.destroy_node()
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
    args = parser.parse_args()
    if args.mode == "cancel-all":
        _cancel()
        return 0
    if args.mode == "request-return":
        _request_return()
        return 0
    _observe(args.output, args.launch_log, args.config)
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except Exception as exc:
        print(f"FAIL: {exc}", file=sys.stderr, flush=True)
        sys.exit(1)
