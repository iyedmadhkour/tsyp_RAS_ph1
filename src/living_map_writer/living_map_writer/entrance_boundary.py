"""Publish a private entrance wall for the Writer's native Nav2 costmaps."""
import copy

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile

from .entrance import entrance_mask


class EntranceBoundary(Node):
    def __init__(self):
        super().__init__("entrance_boundary", namespace="/writer")
        self.declare_parameter("entrance_x", -1.0)
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.mask_pub = self.create_publisher(OccupancyGrid, "/writer/entrance_mask", qos)
        self.create_subscription(OccupancyGrid, "/map", self.on_map, qos)

    def on_map(self, msg):
        q = msg.info.origin.orientation
        # Nav2 StaticLayer requires a grid aligned with its header frame.
        if msg.header.frame_id != "map" or any(abs(v) > 1e-6 for v in (q.x, q.y, q.z)):
            raise ValueError("Writer entrance mask requires a map-aligned SLAM grid")
        mask = OccupancyGrid()
        mask.header = copy.deepcopy(msg.header)
        mask.info = copy.deepcopy(msg.info)
        mask.data = entrance_mask(msg.info.width, msg.info.height, msg.info.resolution,
                                  msg.info.origin.position.x,
                                  float(self.get_parameter("entrance_x").value))
        self.mask_pub.publish(mask)


def main():
    rclpy.init()
    node = EntranceBoundary()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
