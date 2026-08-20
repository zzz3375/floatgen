#!/usr/bin/env python3
# Copyright 2026 floatgen
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Publish the drone flight path as nav_msgs/Path in the world (ENU) frame.

Subscribes to /fmu/out/vehicle_local_position (PX4 local NED, origin = home)
and converts to the gz world ENU frame so the path aligns with wind turbines
and other world objects in RViz2.

Usage:
    python3 scripts/flight_path_publisher.py config/wind_farm.yaml
"""

import math
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from px4_msgs.msg import VehicleLocalPosition

import yaml


class FlightPathPublisher(Node):

    def __init__(self, config_path):
        super().__init__('flight_path_publisher')
        with open(config_path) as f:
            cfg = yaml.safe_load(f)

        # Home in ENU (same origin as the gz world).
        self.home_x = cfg['drone_home']['x']
        self.home_y = cfg['drone_home']['y']
        self.home_z = cfg['drone_home']['z']

        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST)

        self.sub = self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position',
            self.pos_cb, qos)

        self.pub = self.create_publisher(Path, '/drone_path', 10)

        self.path = Path()
        self.path.header.frame_id = 'world'
        self.last_seq = -1

    def pos_cb(self, msg):
        if msg.xy_valid and msg.z_valid:
            px = msg.y + self.home_x  # east  → world X
            py = msg.x + self.home_y  # north → world Y
            pz = -msg.z + self.home_z  # up   → world Z

            pose = PoseStamped()
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.header.frame_id = 'world'
            pose.pose.position.x = float(px)
            pose.pose.position.y = float(py)
            pose.pose.position.z = float(pz)
            pose.pose.orientation.w = 1.0

            self.path.poses.append(pose)
            self.path.header.stamp = pose.header.stamp
            self.pub.publish(self.path)


def main(args=None):
    rclpy.init(args=args)
    config = sys.argv[1] if len(sys.argv) > 1 else \
        '/home/zzz2204/ros2_ws/src/floatgen/config/wind_farm.yaml'
    node = FlightPathPublisher(config)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
