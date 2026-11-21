#!/usr/bin/env python3
"""Test-only scan/odometry relay for controlled Gazebo pose fault injection.

This replaces living_map/scan_frames only in a validation run. It preserves
the normal Writer scan and all three robot odometry/TF topics while allowing
the Writer pose stream to be stale, delayed, missing, translation-frozen, or
rotation-frozen with fresh timestamps. It must never drive a physical robot.
"""
from __future__ import annotations

from collections import deque
from copy import deepcopy
import math
import time

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from sensor_msgs.msg import LaserScan
from tf2_ros import TransformBroadcaster


SPAWNS = {
    "writer": (-11.0, 0.0),
    "executor": (-13.0, -2.0),
    "firebot": (-13.0, 2.0),
}
MODES = {"pass", "stale", "delayed", "missing",
         "frozen_translation", "frozen_rotation"}


class PoseFaultAdapter(Node):
    def __init__(self) -> None:
        super().__init__("pose_fault_frames", namespace="/living_map")
        self.declare_parameter("writer_mode", "pass")
        self.declare_parameter("delay_s", 1.5)
        self.mode = str(self.get_parameter("writer_mode").value)
        if self.mode not in MODES:
            raise ValueError(f"writer_mode must be one of {sorted(MODES)}")
        self.delay_s = float(self.get_parameter("delay_s").value)
        if not math.isfinite(self.delay_s) or self.delay_s <= 0.5:
            raise ValueError("delay_s must exceed the 0.5 s simulation freshness assumption")
        self.tf_broadcaster = TransformBroadcaster(self)
        self.scan_publishers = {
            robot: self.create_publisher(LaserScan, f"/{robot}/scan", 10)
            for robot in SPAWNS
        }
        self.odom_publishers = {
            robot: self.create_publisher(Odometry, f"/{robot}/odom", 20)
            for robot in SPAWNS
        }
        for robot in SPAWNS:
            self.create_subscription(
                LaserScan, f"/{robot}/scan_raw",
                lambda msg, name=robot: self._scan(name, msg), 10)
            self.create_subscription(
                Odometry, f"/{robot}/world_odom",
                lambda msg, name=robot: self._odom(name, msg), 20)
        self.last_writer: Odometry | None = None
        self.stale_sample: Odometry | None = None
        self.frozen_pose = None
        self.delayed: deque[Odometry] = deque(maxlen=500)
        self.create_timer(0.02, self._apply_parameters)
        self.create_timer(0.02, self._release_delayed)
        self.create_timer(0.1, self._repeat_stale)
        self.get_logger().info(
            f"Pose fault relay ready: writer_mode={self.mode}, delay_s={self.delay_s:g}")

    def _apply_parameters(self) -> None:
        mode = str(self.get_parameter("writer_mode").value)
        if mode not in MODES:
            self.get_logger().error(f"Invalid writer_mode requested: {mode}")
            return
        if mode == self.mode:
            return
        self.mode = mode
        if mode == "stale" and self.last_writer is not None:
            self.stale_sample = deepcopy(self.last_writer)
        elif mode in {"frozen_translation", "frozen_rotation"} and self.last_writer is not None:
            p = self.last_writer.pose.pose.position
            q = self.last_writer.pose.pose.orientation
            self.frozen_pose = (p.x, p.y, p.z, q.x, q.y, q.z, q.w)
        elif mode == "delayed":
            self.delayed.clear()
        self.get_logger().warning(f"POSE_FAULT_MODE={mode}")

    def _scan(self, robot: str, msg: LaserScan) -> None:
        msg.header.frame_id = f"{robot}/laser"
        self.scan_publishers[robot].publish(msg)

    def _odom(self, robot: str, incoming: Odometry) -> None:
        msg = deepcopy(incoming)
        msg.header.frame_id = f"{robot}/odom"
        msg.child_frame_id = f"{robot}/base_link"
        msg.pose.pose.position.x -= SPAWNS[robot][0]
        msg.pose.pose.position.y -= SPAWNS[robot][1]
        msg.pose.pose.position.z = 0.0
        if robot != "writer":
            self._publish_odom(robot, msg)
            return
        self.last_writer = deepcopy(msg)
        if self.mode == "missing":
            return
        if self.mode == "stale":
            return  # The cached entry sample is repeated below with its old stamp.
        if self.mode == "delayed":
            self.delayed.append(msg)
            return
        if self.mode == "frozen_translation" and self.frozen_pose is not None:
            p = msg.pose.pose.position
            p.x, p.y, p.z = self.frozen_pose[:3]
            msg.twist.twist.linear.x = 0.0
            msg.twist.twist.linear.y = 0.0
        elif self.mode == "frozen_rotation" and self.frozen_pose is not None:
            q = msg.pose.pose.orientation
            q.x, q.y, q.z, q.w = self.frozen_pose[3:]
            msg.twist.twist.angular.z = 0.0
        self._publish_odom(robot, msg)

    def _repeat_stale(self) -> None:
        if self.mode == "stale" and self.stale_sample is not None:
            self._publish_odom("writer", deepcopy(self.stale_sample))

    def _release_delayed(self) -> None:
        if self.mode != "delayed":
            return
        now_s = self.get_clock().now().nanoseconds / 1e9
        while self.delayed:
            oldest = self.delayed[0]
            stamp_s = oldest.header.stamp.sec + oldest.header.stamp.nanosec / 1e9
            if now_s - stamp_s < self.delay_s:
                break
            self._publish_odom("writer", self.delayed.popleft())

    def _publish_odom(self, robot: str, msg: Odometry) -> None:
        self.odom_publishers[robot].publish(msg)
        transform = TransformStamped()
        transform.header = msg.header
        transform.child_frame_id = msg.child_frame_id
        transform.transform.translation.x = msg.pose.pose.position.x
        transform.transform.translation.y = msg.pose.pose.position.y
        transform.transform.translation.z = msg.pose.pose.position.z
        transform.transform.rotation = msg.pose.pose.orientation
        self.tf_broadcaster.sendTransform(transform)


def main() -> None:
    rclpy.init()
    node = PoseFaultAdapter()
    try:
        rclpy.spin(node)
    except ExternalShutdownException:
        pass
    finally:
        node.destroy_node()
        # Launch teardown sends SIGINT, which may already have shut the
        # default context down before finally runs. try_shutdown is idempotent
        # and avoids reporting normal teardown as an adapter failure.
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
