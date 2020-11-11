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
import traceback
from uuv_control_msgs.srv import InitHelicalTrajectory
from numpy import pi
from geometry_msgs.msg import Point

from plankton_utils.time import time_in_float_sec
from plankton_utils.time import float_sec_to_int_sec_nano
from plankton_utils.time import is_sim_time

def main(): 
    rclpy.init()

    sim_time_param = is_sim_time()
    
    # Compared to ROS 1 version, the node name was changed
    node = rclpy.create_node(
        'start_helical_trajectory',
        allow_undeclared_parameters=True, 
        automatically_declare_parameters_from_overrides=True,
        parameter_overrides=[sim_time_param])
        
    node.get_logger().info('Starting the helical trajectory creator')

    # Ensure the clock has been updated when using sim time
    while node.get_clock().now() == rclpy.time.Time(clock_type=node.get_clock().clock_type):
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
                    'duration', 'n_turns', 'delta_z', 'max_forward_speed']
    params = dict()

    for label in param_labels:
        if not node.has_parameter(label):
            node.get_logger().error('{} must be provided for the trajectory generation!'.format(label))
            sys.exit(-1)

        params[label] = node.get_parameter(label).value

    if len(params['center']) != 3:
        raise RuntimeError(
        'Center of circle must have 3 components (x, y, z):' + str(params['center']))

    if params['n_points'] <= 2:
        raise RuntimeError(
        'Number of points must be at least 2' + str(params['n_points']))

    if params['max_forward_speed'] <= 0:
        raise RuntimeError('Velocity limit must be positive' + str(params['max_forward_speed']))

    srv_name = 'start_helical_trajectory'
    traj_gen = node.create_client(InitHelicalTrajectory, 'start_helical_trajectory')

    if not traj_gen.wait_for_service(timeout_sec=20):
        raise RuntimeError('Service %s not available! Closing node...' %(traj_gen.srv_name))

    node.get_logger().info('Generating trajectory that starts at t={} s'.format(start_time))

    # Convert the time value
    (sec, nsec) = float_sec_to_int_sec_nano(start_time)

    req = InitHelicalTrajectory.Request()
    req.start_time = rclpy.time.Time(seconds=sec, nanoseconds=nsec).to_msg()
    req.start_now = start_now
    req.radius = float(params['radius'])
    req.center = Point(x=float(params['center'][0]), 
                       y=float(params['center'][1]), 
                       z=float(params['center'][2]))
    req.is_clockwise = False
    req.angle_offset = 0.0
    req.n_points = params['n_points']
    req.heading_offset = float(params['heading_offset'] * pi / 180)
    req.max_forward_speed = float(params['max_forward_speed'])
    req.duration = float(params['duration'])
    # As partial turns can be given, it is a float
    req.n_turns = float(params['n_turns'])
    req.delta_z = float(params['delta_z'])

    future = traj_gen.call_async(req)
    rclpy.spin_until_future_complete(node, future)
    try:
        response = future.result()
    except Exception as e:
        node.get_logger().error('Service call ' + srv_name + ' failed, error=' + str(e))
    else:
        node.get_logger().info('Trajectory successfully generated!')

    node.destroy_node()

# =============================================================================
if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print('Exception caught: ' + repr(e))
        print(traceback.print_exc())
    finally:
        if rclpy.ok():
            rclpy.shutdown()
