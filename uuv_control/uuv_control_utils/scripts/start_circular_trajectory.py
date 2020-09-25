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

import rclpy
import sys
from numpy import pi

from uuv_control_msgs.srv import InitCircularTrajectory

from geometry_msgs.msg import Point
from std_msgs.msg import Time

from plankton_utils.time import time_in_float_sec
from plankton_utils import float_sec_to_int_sec_nano

def main():
    rclpy.init()
    node = rclpy.create_node('start_circular_trajectory',
                            allow_undeclared_parameters=True, 
                            automatically_declare_parameters_from_overrides=True)

    sim_time = rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)
    node.set_parameters([sim_time])

    node.get_logger().info('Starting the circular trajectory creator')
    
    #Important...ensure the clock has been updated when using sim time
    while node.get_clock().now() == rclpy.time.Time():
        rclpy.spin_once(node)

    # If no start time is provided: start *now*.
    start_time = time_in_float_sec(node.get_clock().now())
    start_now = False
    if node.has_parameter('start_time'):
        start_time = node.get_parameter('start_time').value
        if start_time < 0.0:
            node.get_logger().warn('Negative start time, setting it to 0.0')
            start_time = 0.0
            start_now = True
    else:
        start_now = True

    param_labels = ['radius', 'center', 'n_points', 'heading_offset',
                    'duration', 'max_forward_speed']
    params = dict()

    for label in param_labels:
        if not node.has_parameter(label):
            node.get_logger().error('{} must be provided for the trajectory generation!'.format(label))
            sys.exit(-1)

        params[label] = node.get_parameter(label).value

    if len(params['center']) != 3:
        node.get_logger().error('Center of circle must have 3 components (x, y, z)')
        sys.exit(-1)

    if params['n_points'] <= 2:
        node.get_logger().error('Number of points must be at least 2')
        sys.exit(-1)

    if params['max_forward_speed'] <= 0:
        node.get_logger().error('Velocity limit must be positive')
        sys.exit(-1)

    srv_name = 'start_circular_trajectory'
    traj_gen = node.create_client(InitCircularTrajectory, srv_name)
        
    if not traj_gen.wait_for_service(timeout_sec=20):
        node.get_logger().error('Service %s not available! Closing node...' %(traj_gen.srv_name))
        sys.exit(-1)

    node.get_logger().info('Generating trajectory that starts at t={} s'.format(start_time))

    #Convert the time value
    (sec, nsec) = float_sec_to_int_sec_nano(start_time)

    req = InitCircularTrajectory.Request()
    req.start_time = rclpy.time.Time(seconds=sec, nanoseconds=nsec).to_msg()
    req.start_now = start_now
    req.radius = params['radius'],
    req.center = Point(params['center'][0], params['center'][1], params['center'][2])
    req.is_clockwise = False
    req.angle_offset = 0.0
    req.n_points = params['n_points']
    req.heading_offset = params['heading_offset'] * pi / 180
    req.max_forward_speed = params['max_forward_speed']
    req.duration = params['duration']

    future = traj_gen.call_async(req)
    rclpy.spin_until_future_complete(self, future)
    try:
        response = future.result()
    except Exception as e:
        node.get_logger().error('Service call ' + srv_name + ' failed, error=' + str(e)):
    else:
        node.get_logger().info('Trajectory successfully generated!')

    #success = traj_gen.call(req)

    # if success:
    #     print('Trajectory successfully generated!')
    # else:
    #     print('Failed')

#==============================================================================
if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print('Something went wrong: ' + str(e))
    finally:
        if rclpy.ok():
            rclpy.shutdown()
