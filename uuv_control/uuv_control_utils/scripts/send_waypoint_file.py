#!/usr/bin/env python3
# Copyright (c) 2016 The UUV Simulator Authors.
# All rights reserved.
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

import rclpy
import sys
from uuv_control_msgs.srv import InitWaypointsFromFile
from std_msgs.msg import String, Time
from time_utils import time_in_float_sec

def main():
    rclpy.init()
    node = rclpy.create_node('send_waypoint_file')
    node.get_logger().info('Send a waypoint file, namespace=%s', node.get_namespace())

    # if rospy.is_shutdown():
    #     rospy.logerr('ROS master not running!')
    #     sys.exit(-1)

    if node.has_parameter('~filename'):
        filename = node.get_parameter('~filename').get_parameter_value().string_value
    else:
        raise RuntimeError('No filename found')

    # If no start time is provided: start *now*.
    start_time = time_in_float_sec(node.get_clock().now())
    start_now = True
    if node.has_parameter('~start_time'):
        start_time = node.get_parameter('~start_time').get_parameter_value().double_value
        if start_time < 0.0:
            node.get_logger().error('Negative start time, setting it to 0.0')
            start_time = 0.0
            start_now = True
        else:
            start_now = False
    else:
        start_now = True

    node.get_logger().info('Start time=%.2f s' % start_time)

    interpolator = node.get_parameter('~interpolator', 'lipb').get_parameter_value().string_value

    try:
        init_wp = node.create_client(
            InitWaypointsFromFile,
            'init_waypoints_from_file')
    except Exception as e:
        node.get_logger().error('Service call failed, error=%s', str(e))

    try:
        ready = init_wp.wait_for_service('init_waypoints_from_file', timeout_sec=30)
        if not ready:
            raise RuntimeError('Service not available! Closing node...')
        
    req = InitWaypointsFromFile.Request()
    req.start_time = rclpy.time.Time(start_time).to_msg())
    req.start_now = start_now
    req.filename = String(filename),
    req.interpolator = String(interpolator)

    success = init_wp.call(req)

    if success:
        node.get_logger().info('Waypoints file successfully received, '
                      'filename=%s', filename)
    else:
        node.get_logger().info('Failed to send waypoints')

if __name__ == '__main__':
    main()