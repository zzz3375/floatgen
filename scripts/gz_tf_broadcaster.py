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

"""Publish TF for the gz x500_mid360 drone in the wind_farm SITL.

The gz world frame (ENU) is the RViz fixed frame. gz-sim's OdometryPublisher
plugin (added to the x500_mid360 model) publishes the MODEL's world pose as
gz.msgs.OdometryWithCovariance on the custom topic /x500_mid360/odom_with_cov
(redirected so PX4's gz_bridge never sees it); ros_gz_bridge forwards it as
nav_msgs/Odometry with header.frame_id = "world". This node broadcasts that as
the dynamic world -> model transform and attaches the static frames (TF
composition keeps everything consistent under rotation):

    world -> x500_mid360                (dynamic, from odometry)
    x500_mid360 -> base_link            (static, +0.24 z, x500_base model pose)
    base_link -> camera_link            (static, front camera mount)
    base_link -> mid360_link            (static, top lidar mount)
    <link> -> <link>/<sensor>           (static, identity)

The last group matches the frame_id strings gz-sensors puts in the camera /
gpu_lidar messages (<model>::<link>::<sensor>, converted to '/' by the
bridge), so the point cloud and image resolve in the TF tree.

Usage:
    python3 gz_tf_broadcaster.py
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster


MODEL = 'x500_mid360'
# PX4's gz_bridge spawns the model as <model>_<px4_instance> (0 by default).
# When loaded via model://, the gz-sensor scoped names carry this spawned
# name, so the camera/lidar frame_ids are x500_mid360_0/<link>/<sensor>.
SPAWNED_MODEL = '{}_0'.format(MODEL)
# Custom odometry topic from the model's OdometryPublisher plugin: the
# with-covariance output is redirected here so PX4's gz_bridge never consumes it.
ODOM_TOPIC = '/x500_mid360/odom_with_cov'
MODEL_FRAME = MODEL                      # model origin frame (odometry reports this pose)
BASE_LINK = '{}/base_link'.format(MODEL)

# base_link pose within the model frame. x500_base carries <pose>0 0 .24 0 0 0</pose>,
# which is applied when merged into x500_mid360, so base_link sits 0.24 m above the
# model origin. The gz OdometryPublisher reports the MODEL world pose, so the TF tree is
# world -> model -> base_link (composition handles the offset under drone rotation).
MODEL_TO_BASE_LINK = ((0.0, 0.0, 0.24), (0.0, 0.0, 0.0))

# Sensor mounts relative to base_link ((x y z), (r p y)). Link <pose> is
# model-relative, so these are base_link_pose + mount_pose, matching
# PX4-Autopilot Tools/simulation/gz/models/x500_mid360/model.sdf:
#   camera_link (0.12 0 0.242), mid360_link (0 0 0.59) in the model frame.
SENSOR_MOUNTS = {
    '{}/camera_link'.format(MODEL): ((0.12, 0.0, 0.002), (0.0, 0.0, 0.0)),
    '{}/mid360_link'.format(MODEL): ((0.0, 0.0, 0.35), (0.0, 0.0, 0.0)),
}

# gz-sensors publishes sensor data with frame = <model>::<link>::<sensor>;
# the bridge maps '::' to '/'. PX4's gz_bridge spawns the model as
# <model>_<instance> (x500_mid360_0 by default), and when the model is loaded
# via the model:// URI the sensor scope carries that spawned name. Each sensor
# sits at the origin of its link, so these are identity transforms.
SENSOR_FRAMES = {
    '{}/camera_link/camera'.format(SPAWNED_MODEL): '{}/camera_link'.format(MODEL),
    '{}/mid360_link/mid360'.format(SPAWNED_MODEL): '{}/mid360_link'.format(MODEL),
}


def _transform(parent, child, xyz, rpy):
    t = TransformStamped()
    t.header.frame_id = parent
    t.child_frame_id = child
    t.transform.translation.x = xyz[0]
    t.transform.translation.y = xyz[1]
    t.transform.translation.z = xyz[2]
    t.transform.rotation.x = rpy[0]
    t.transform.rotation.y = rpy[1]
    t.transform.rotation.z = rpy[2]
    t.transform.rotation.w = 1.0
    return t


class GzTfBroadcaster(Node):

    def __init__(self):
        super().__init__('gz_tf_broadcaster')
        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_broadcaster = StaticTransformBroadcaster(self)

        static = [_transform(MODEL_FRAME, BASE_LINK, *MODEL_TO_BASE_LINK)]
        static += [_transform(BASE_LINK, child, xyz, rpy)
                   for child, (xyz, rpy) in SENSOR_MOUNTS.items()]
        static += [_transform(parent, child, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0))
                   for child, parent in SENSOR_FRAMES.items()]
        self.static_broadcaster.sendTransform(static)

        # EMA low-pass filter to suppress high-frequency odometry jitter.
        # alpha ∈ (0, 1]: smaller → smoother, 1.0 → no filtering (passthrough).
        self.declare_parameter('filter_alpha', 0.25)
        self._alpha = self.get_parameter('filter_alpha').value
        self._prev_pos = None
        self._prev_quat = None

        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(Odometry, ODOM_TOPIC, self.odom_cb, qos)
        self.get_logger().info(
            'publishing TF from {} (fixed frame: world, filter alpha={})'.format(
                ODOM_TOPIC, self._alpha))

    def odom_cb(self, msg):
        pos = msg.pose.pose.position
        quat = msg.pose.pose.orientation

        if self._prev_pos is None:
            self._prev_pos = [pos.x, pos.y, pos.z]
            self._prev_quat = [quat.x, quat.y, quat.z, quat.w]
        else:
            a = self._alpha
            # EMA on position
            self._prev_pos[0] += a * (pos.x - self._prev_pos[0])
            self._prev_pos[1] += a * (pos.y - self._prev_pos[1])
            self._prev_pos[2] += a * (pos.z - self._prev_pos[2])
            # Shortest-path linear blend on quaternion (NLerp):
            # flip sign if needed so we interpolate the short way around.
            qx, qy, qz, qw = self._prev_quat
            dot = qx * quat.x + qy * quat.y + qz * quat.z + qw * quat.w
            sign = -1.0 if dot < 0.0 else 1.0
            qx += a * (sign * quat.x - qx)
            qy += a * (sign * quat.y - qy)
            qz += a * (sign * quat.z - qz)
            qw += a * (sign * quat.w - qw)
            norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
            self._prev_quat = [qx / norm, qy / norm, qz / norm, qw / norm]

        t = TransformStamped()
        t.header = msg.header
        t.child_frame_id = MODEL_FRAME
        t.transform.translation.x = self._prev_pos[0]
        t.transform.translation.y = self._prev_pos[1]
        t.transform.translation.z = self._prev_pos[2]
        t.transform.rotation.x = self._prev_quat[0]
        t.transform.rotation.y = self._prev_quat[1]
        t.transform.rotation.z = self._prev_quat[2]
        t.transform.rotation.w = self._prev_quat[3]
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = GzTfBroadcaster()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    node.destroy_node()
    try:
        rclpy.shutdown()
    except Exception:
        pass


if __name__ == '__main__':
    main()
