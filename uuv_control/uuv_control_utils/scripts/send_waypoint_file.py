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

import sys

import rclpy
import rclpy.time

from uuv_control_msgs.srv import InitWaypointsFromFile
from std_msgs.msg import String, Time

from plankton_utils import time_in_float_sec
from plankton_utils import float_sec_to_int_sec_nano

def main():
    rclpy.init()
    node = rclpy.create_node('send_waypoint_file',
                            allow_undeclared_parameters=True, 
                            automatically_declare_parameters_from_overrides=True)


    sim_time = rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)
    self.set_parameters([sim_time])
    node.get_logger().info('Send a waypoint file, namespace=%s', % node.get_namespace())

    # if rospy.is_shutdown():
    #     rospy.logerr('ROS master not running!')
    #     sys.exit(-1)

    if node.has_parameter('filename'):
        filename = node.get_parameter('filename').get_parameter_value().string_value
    else:
        raise RuntimeError('No filename found')

    #Important...ensure the clock has been updated when using sim time
    while node.get_clock().now() == rclpy.time.Time():
        rclpy.spin_once(node)

    # If no start time is provided: start *now*.
    start_time = time_in_float_sec(node.get_clock().now())
    start_now = True
    if node.has_parameter('start_time'):
        start_time = node.get_parameter('start_time').value
        if start_time < 0.0:
            node.get_logger().error('Negative start time, setting it to 0.0')
            start_time = 0.0
            start_now = True
        else:
            start_now = False

    node.get_logger().info('Start time=%.2f s' % start_time)

    interpolator = node.get_parameter('interpolator', 'lipb').get_parameter_value().string_value

    srv_name = 'init_waypoints_from_file'
    
    init_wp = node.create_client(InitWaypointsFromFile, srv_name)

    ready = init_wp.wait_for_service('init_waypoints_from_file', timeout_sec=30)
    if not ready:
        raise RuntimeError('Service not available! Closing node...')
        
    (sec, nsec) = float_sec_to_int_sec_nano(start_time)

    req = InitWaypointsFromFile.Request()
    req.start_time = rclpy.time.Time(seconds=sec, nanoseconds=nsec).to_msg())
    req.start_now = start_now
    req.filename = String(data=filename),
    req.interpolator = String(data=interpolator)

    future = init_wp.call_async(req)
    rclpy.spin_until_future_complete(self, future)
    try:
        response = future.result()
    except Exception as e:
        node.get_logger().error('Service call ' + srv_name + ' failed, error=' + str(e)):
    else:
        node.get_logger().info('Waypoints file successfully received, '
                      'filename=%s', filename)

    # success = init_wp.call(req)

    # if success:
    #     node.get_logger().info('Waypoints file successfully received, '
    #                   'filename=%s', filename)
    # else:
    #     node.get_logger().info('Failed to send waypoints')

#==============================================================================
if __name__ == '__main__':
    try:    
        main()
    except Exception as e:
        print('Caught exception: ' + str(e))
    finally:
        if rclpy.ok():
            rclpy.shutdown()
