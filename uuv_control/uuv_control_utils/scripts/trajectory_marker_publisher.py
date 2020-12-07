#!/usr/bin/env python3
# Copyright (c) 2020 The Plankton Authors.
# All rights reserved.
#
# This source code is derived from UUV Simulator
# (https://github.com/uuvsimulator/uuv_simulator)
# Copyright (c) 2016-2019 The UUV Simulator Authors
# licensed under the Apache license, Version 2.0
# cf. 3rd-party-licenses.txt file in the root directory of this source tree.
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

import os
import traceback
import yaml
from datetime import datetime

from std_msgs.msg import Bool
from nav_msgs.msg import Path
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from uuv_control_msgs.msg import Trajectory, TrajectoryPoint, WaypointSet
import uuv_trajectory_generator
import uuv_waypoints
import rclpy
from rclpy.node import Node

from plankton_utils.time import is_sim_time

class TrajectoryMarkerPublisher(Node):
    def __init__(self, name, **kwargs):
        super().__init__(name,
                        allow_undeclared_parameters=True, 
                        automatically_declare_parameters_from_overrides=True,
                        **kwargs)

        self.get_logger().info('Starting trajectory and waypoint marker publisher')

        self._trajectory_sub = self.create_subscription(
            Trajectory, 'trajectory', self._update_trajectory, 10)

        self._waypoints_sub = self.create_subscription(
            WaypointSet,'waypoints', self._update_waypoints, 10)

        # Vehicle state flags
        self._is_auto_on = False
        self._is_traj_tracking_on = False
        self._is_station_keeping_on = False

        self._automatic_mode_sub = self.create_subscription(
            Bool, 'automatic_on', self._update_auto_mode, 10)

        self._traj_tracking_mode_sub = self.create_subscription(
            Bool, 'trajectory_tracking_on', self._update_traj_tracking_mode, 10)

        self._station_keeping_mode_sub = self.create_subscription(
            Bool, 'station_keeping_on', self._update_station_keeping_mode, 10)

        self._reference_sub = self.create_subscription(
            TrajectoryPoint, 'reference', self._reference_callback, 10)

        # Waypoint set received
        self._waypoints = None
        # Trajectory received
        self._trajectory = None

        self._output_dir = None
        if self.has_parameter('output_dir'):
            self._output_dir = self.get_parameter('output_dir').get_parameter_value().string_value
            if not os.path.isdir(self._output_dir):
                self.get_logger().error('Invalid output directory, not saving the files, dir=', self._output_dir)
                self._output_dir = None
            else:
                self._output_dir = os.path.join(self._output_dir, self.get_namespace().replace('/', ''))
                if not os.path.isdir(self._output_dir):
                    os.makedirs(self._output_dir)

        # Visual marker publishers
        self._trajectory_path_pub = self.create_publisher(
            Path, 'trajectory_marker', 1)

        self._waypoint_markers_pub = self.create_publisher(
            MarkerArray, 'waypoint_markers', 1)

        self._waypoint_path_pub = self.create_publisher(
            Path, 'waypoint_path_marker', 1)

        self._reference_marker_pub = self.create_publisher(
            Marker, 'reference_marker', 1)

        self._update_markers_timer = self.create_timer(
            0.5, self._update_markers)

        self.get_logger().info('Trajectory and waypoint marker publisher ready')

    # =========================================================================
    def _update_markers(self):
        if self._waypoints is None:
            waypoint_path_marker = Path()
            t_msg = self.get_clock().now().to_msg()
            waypoint_path_marker.header.stamp = t_msg
            waypoint_path_marker.header.frame_id = 'world'

            waypoint_marker = MarkerArray()
            marker = Marker()
            marker.header.stamp = t_msg
            marker.header.frame_id = 'world'
            marker.id = 0
            marker.type = Marker.SPHERE
            marker.action = 3
        else:
            waypoint_path_marker = self._waypoints.to_path_marker(self.get_clock().now())
            waypoint_path_marker.header.frame_id = self._waypoints.inertial_frame_id
            waypoint_marker = self._waypoints.to_marker_list(self.get_clock().now())

        self._waypoint_path_pub.publish(waypoint_path_marker)
        self._waypoint_markers_pub.publish(waypoint_marker)

        traj_marker = Path()
        traj_marker.header.stamp = self.get_clock().now().to_msg()
        traj_marker.header.frame_id = 'world'

        if self._trajectory is not None:
            traj_marker.header.frame_id = self._trajectory.header.frame_id
            for pnt in self._trajectory.points:
                p_msg = PoseStamped()
                p_msg.header.stamp = pnt.header.stamp
                p_msg.header.frame_id = self._trajectory.header.frame_id
                p_msg.pose = pnt.pose
                traj_marker.poses.append(p_msg)

        self._trajectory_path_pub.publish(traj_marker)
        return True

    # =========================================================================
    def _update_trajectory(self, msg):
        self._trajectory = msg

    # =========================================================================
    def _update_waypoints(self, msg):
        self._waypoints = uuv_waypoints.WaypointSet()
        self._waypoints.from_message(msg)

    # =========================================================================
    def _update_auto_mode(self, msg):
        self._is_auto_on = msg.data

    #==========================================================================
    def _update_station_keeping_mode(self, msg):
        self._is_station_keeping_on = msg.data

    # =========================================================================
    def _update_traj_tracking_mode(self, msg):
        self._is_traj_tracking_on = msg.data

    # =========================================================================
    def _reference_callback(self, msg):
        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = msg.header.frame_id
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.MODIFY

        marker.pose.position = msg.pose.position
        marker.scale.x = 0.3
        marker.scale.y = 0.3
        marker.scale.z = 0.3

        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0

        self._reference_marker_pub.publish(marker)


# =============================================================================
if __name__ == '__main__':
    print('Starting trajectory and waypoint marker publisher')
    rclpy.init()

    try:
        sim_time_param = is_sim_time()

        node = TrajectoryMarkerPublisher(
            'trajectory_marker_publisher', 
            parameter_overrides=[sim_time_param])
        rclpy.spin(node)
    except Exception as e:
        print('Caught exception' + repr(e))
        print(traceback.print_exc())
    finally:
        if rclpy.ok():
            rclpy.shutdown()
    print('Exiting')


