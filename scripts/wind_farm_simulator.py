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

"""Wind-farm inspection simulator for PX4 SITL (gz wind_farm world).

Drives the PX4 x500 through an offboard orbit around the first wind turbine
(wind_turbine_1_1 at the farm origin), then returns to launch and lands:

    stream setpoints -> arm -> offboard -> TAKEOFF -> hold -> plan cycle
    -> orbit waypoints -> RTL -> land -> disarm

All setpoints are position setpoints (OffboardControlMode.position +
TrajectorySetpoint) in the PX4 local NED frame, home = drone spawn point
(PX4_GZ_MODEL_POSE in the ENU gz world). Configuration comes from
config/wind_farm.yaml (same file the world layout was generated from).

Waypoint ordering is computed by cycle_planner — after reaching the holding
altitude the orbit is reordered so the first waypoint is nearest to the
drone's current hover position.

Usage:
    ros2 run floatgen wind_farm_simulator
"""

import math
import os
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint
from px4_msgs.msg import VehicleCommand, VehicleLocalPosition, VehicleStatus

import yaml

# cycle_planner lives alongside this script; ensure it is importable when
# invoked via ros2 run (where CWD is not the scripts directory).
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from cycle_planner import CyclePlanner


# PX4 VehicleStatus constants (px4_msgs v1.15.4)
ARMING_STATE_DISARMED = 1
ARMING_STATE_ARMED = 2
NAVIGATION_STATE_OFFBOARD = 14
NAVIGATION_STATE_AUTO_RTL = 5

# VehicleCommand constants
CMD_DO_SET_MODE = 176
CMD_COMPONENT_ARM_DISARM = 400
CMD_NAV_RETURN_TO_LAUNCH = 20
PX4_CUSTOM_MAIN_MODE_OFFBOARD = 6.0


def ned_from_enu(enu, home_enu):
    """ENU world position -> PX4 local NED.

    gz worlds are ENU (x=east, y=north, z=up); PX4 local is NED
    (x=north, y=east, z=down), home = drone spawn point.
    """
    return (enu[1] - home_enu[1], enu[0] - home_enu[0], -(enu[2] - home_enu[2]))


class WindFarmSimulator(Node):

    def __init__(self, config_path):
        super().__init__('wind_farm_simulator')
        with open(config_path) as f:
            self.cfg = yaml.safe_load(f)

        self.qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT,
                              history=HistoryPolicy.KEEP_LAST)

        self.offboard_pub = self.create_publisher(OffboardControlMode,
                                                  '/fmu/in/offboard_control_mode', 10)
        self.setpoint_pub = self.create_publisher(TrajectorySetpoint,
                                                  '/fmu/in/trajectory_setpoint', 10)
        self.command_pub = self.create_publisher(VehicleCommand,
                                                 '/fmu/in/vehicle_command', 10)

        self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status',
                                 self.status_cb, self.qos)
        self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position',
                                 self.local_pos_cb, self.qos)

        self.status = None
        self.local_pos = None
        self.last_status_time = 0.0
        self.last_local_pos_time = 0.0

        # --- mission plan (NED) -------------------------------------------
        home_enu = (self.cfg['drone_home']['x'], self.cfg['drone_home']['y'],
                    self.cfg['drone_home']['z'])
        self.home_enu = home_enu
        turb = self.cfg['turbines'][0]
        self.center_ned = ned_from_enu((turb['x'], turb['y'], 0.0), home_enu)
        self.get_logger().info(
            f"inspection target '{turb['name']}' at NED "
            f"({self.center_ned[0]:.1f}, {self.center_ned[1]:.1f})")

        insp = self.cfg['inspection']
        self.orbit_radius = insp['orbit_radius']
        self.orbit_z = -insp['orbit_altitude']  # NED down
        self.num_wp = insp['num_waypoints']
        self.wp_tol = insp['waypoint_tolerance']
        self.wp_timeout = insp['waypoint_timeout']
        self.yaw_towards = insp['yaw_towards_turbine']

        # orbit waypoints around the turbine centre (initial order; reordered
        # from the drone's hover position after takeoff — see TAKEOFF state)
        self.cycle = CyclePlanner(
            center_ned=self.center_ned,
            radius=self.orbit_radius,
            altitude_z=self.orbit_z,
            num_waypoints=self.num_wp,
        )
        self.waypoints = self.cycle.waypoints

        # --- state machine -------------------------------------------------
        self.state = 'STREAM'          # STREAM -> ARM -> OFFBOARD -> TAKEOFF -> FLY -> RTL -> LAND -> DONE
        self.state_since = time.monotonic()
        self.waypoint_idx = 0
        self.takeoff_pos = None        # xy position held during vertical climb
        self.target = None             # current NED setpoint
        self.armed = False
        self.offboard = False
        self.finished = False
        self.last_arm_send = 0.0       # monotonic time of the last ARM command

        self.timer = self.create_timer(0.05, self.control_loop)  # 20 Hz

    # -- subscriptions ------------------------------------------------------
    def status_cb(self, msg):
        self.status = msg
        self.last_status_time = time.monotonic()

    def local_pos_cb(self, msg):
        self.local_pos = msg
        self.last_local_pos_time = time.monotonic()

    # -- helpers ------------------------------------------------------------
    def now_us(self):
        return int(self.get_clock().now().nanoseconds / 1000)

    def publish_offboard_mode(self):
        msg = OffboardControlMode()
        msg.timestamp = self.now_us()
        msg.position = True
        self.offboard_pub.publish(msg)

    def publish_setpoint(self, ned):
        msg = TrajectorySetpoint()
        msg.timestamp = self.now_us()
        msg.position = [float(ned[0]), float(ned[1]), float(ned[2])]
        if self.yaw_towards:
            msg.yaw = math.atan2(self.center_ned[1] - ned[1],
                                 self.center_ned[0] - ned[0])
        else:
            msg.yaw = 0.0
        msg.velocity = [float('nan')] * 3
        msg.yawspeed = 0.0
        self.setpoint_pub.publish(msg)

    def send_command(self, command, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.timestamp = self.now_us()
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.command = command
        msg.param1 = float(param1)
        msg.param2 = float(param2)
        msg.param3 = 0.0
        msg.param4 = 0.0
        msg.param5 = 0.0
        msg.param6 = 0.0
        msg.param7 = 0.0
        msg.confirmation = False
        msg.from_external = True
        self.command_pub.publish(msg)

    def elapsed(self):
        return time.monotonic() - self.state_since

    def log_state(self, s):
        self.get_logger().info(f'[{s}]')

    # -- main loop ----------------------------------------------------------
    def control_loop(self):
        now = time.monotonic()
        if self.status is None or self.local_pos is None:
            self.get_logger().warn('waiting for vehicle status/local position...',
                                   throttle_duration_sec=5.0)
            # still stream a harmless hover setpoint once data is available
            return

        pos = (self.local_pos.x, self.local_pos.y, self.local_pos.z)

        if self.state == 'STREAM':
            # send a hover setpoint at the current position for the pre-arm phase
            self.target = (pos[0], pos[1], min(pos[2] - 0.5, -0.5))
            self.publish_offboard_mode()
            self.publish_setpoint(self.target)
            if self.elapsed() > 2.0:
                self.send_command(CMD_COMPONENT_ARM_DISARM, param1=1.0)
                self.log_state('ARM (command sent)')
                self.state = 'ARM'
                self.state_since = now
                self.last_arm_send = now

        elif self.state == 'ARM':
            self.publish_offboard_mode()
            self.publish_setpoint(self.target)
            if self.status.arming_state == ARMING_STATE_ARMED:
                self.log_state('ARMED -> OFFBOARD (command sent)')
                self.send_command(CMD_DO_SET_MODE, param1=1.0,
                                  param2=PX4_CUSTOM_MAIN_MODE_OFFBOARD)
                self.state = 'OFFBOARD'
                self.state_since = now
            elif self.elapsed() > 30.0:
                self.get_logger().error('arming timeout, aborting')
                self.state = 'DONE'
                self.state_since = now
            elif self.elapsed() - self.last_arm_send > 2.0:
                # EKF may still be converging (GPS/height fusion); keep
                # requesting arming until the health checks pass
                self.send_command(CMD_COMPONENT_ARM_DISARM, param1=1.0)
                self.last_arm_send = now
                self.get_logger().info('ARM (retry, waiting for health checks)')

        elif self.state == 'OFFBOARD':
            self.publish_offboard_mode()
            self.publish_setpoint(self.target)
            if self.status.nav_state == NAVIGATION_STATE_OFFBOARD:
                self.log_state('OFFBOARD active, climbing to altitude')
                self.takeoff_pos = (pos[0], pos[1])
                self.target = (pos[0], pos[1], self.orbit_z)
                self.state = 'TAKEOFF'
                self.state_since = now
            elif self.elapsed() > 10.0:
                self.get_logger().error('offboard mode timeout, aborting')
                self.state = 'DONE'
                self.state_since = now

        elif self.state == 'TAKEOFF':
            self.publish_offboard_mode()
            self.publish_setpoint(self.target)
            if abs(pos[2] - self.target[2]) < 1.5:
                # reorder orbit so the first waypoint is nearest to the
                # drone's current hover position
                self.cycle.reorder_from((pos[0], pos[1]))
                self.waypoints = self.cycle.waypoints
                k = self.cycle.nearest_index((pos[0], pos[1]))
                self.log_state(
                    f'altitude reached ({-self.orbit_z:.1f}m), '
                    f'phase index k={k}, flying to next waypoint (k+1)')
                self.waypoint_idx = 0
                self.target = self.waypoints[0]
                self.state = 'FLY'
                self.state_since = now
            elif self.elapsed() > self.wp_timeout:
                self.get_logger().error('takeoff timeout, aborting')
                self.state = 'DONE'
                self.state_since = now

        elif self.state == 'FLY':
            # If PX4 dropped out of offboard mode (watchdog timeout), try to
            # re-enter before the drone drifts too far.
            # if self.status.nav_state != NAVIGATION_STATE_OFFBOARD:
            #     self.get_logger().warn(
            #         f'lost offboard (nav_state={self.status.nav_state}), '
            #         f'requesting again')
            #     self.send_command(CMD_DO_SET_MODE, param1=1.0,
            #                       param2=PX4_CUSTOM_MAIN_MODE_OFFBOARD)
            #  zzz3375: 这个强制转offboard不需要，去除掉
            self.publish_offboard_mode()
            self.publish_setpoint(self.target)
            dx = pos[0] - self.target[0]
            dy = pos[1] - self.target[1]
            dz = pos[2] - self.target[2]
            dist_h = math.hypot(dx, dy)
            if dist_h < self.wp_tol and abs(dz) < 1.5:
                self.waypoint_idx += 1
                self.state_since = now
                if self.waypoint_idx >= len(self.waypoints):
                    self.log_state(f'ORBIT COMPLETE ({self.num_wp} waypoints) -> RTL')
                    self.send_command(CMD_NAV_RETURN_TO_LAUNCH)
                    self.state = 'RTL'
                    self.state_since = now
                else:
                    self.target = self.waypoints[self.waypoint_idx]
                    self.log_state(
                        f'waypoint {self.waypoint_idx}/{self.num_wp} '
                        f'({self.target[0]:.1f}, {self.target[1]:.1f}, {self.target[2]:.1f})')
            elif self.elapsed() > self.wp_timeout:
                self.get_logger().error(f'waypoint {self.waypoint_idx} timeout, aborting')
                self.state = 'DONE'
                self.state_since = now

        elif self.state == 'RTL':
            # stop streaming offboard setpoints: PX4 keeps RTL mode
            if self.status.nav_state == NAVIGATION_STATE_AUTO_RTL:
                self.log_state('RTL active, waiting for landing')
                self.state = 'LAND'
                self.state_since = now
            elif self.elapsed() > 60.0:
                self.get_logger().error('RTL timeout')
                self.state = 'DONE'
                self.state_since = now

        elif self.state == 'LAND':
            # PX4 auto-disarms after a landed touchdown, and the local
            # altitude reference can drift a couple of metres, so accept
            # either a disarmed state or a near-home altitude.
            landed = (self.status.arming_state == ARMING_STATE_DISARMED
                      or pos[2] > -1.0)
            if landed and self.elapsed() > 5.0:
                self.log_state('LANDED -> DISARM')
                if self.status.arming_state == ARMING_STATE_ARMED:
                    self.send_command(CMD_COMPONENT_ARM_DISARM, param1=0.0)
                self.state = 'DONE'
                self.state_since = now

        elif self.state == 'DONE':
            self.get_logger().info('mission finished')
            self.timer.cancel()
            self.finished = True


def main(args=None):
    rclpy.init(args=args)
    config = sys.argv[1] if len(sys.argv) > 1 else \
        '/home/zzz2204/ros2_ws/src/floatgen/config/wind_farm.yaml'
    node = WindFarmSimulator(config)
    try:
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
