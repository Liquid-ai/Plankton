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
from __future__ import print_function
import rclpy
import sys
from uuv_control_msgs.srv import InitCircularTrajectory
from numpy import pi
from geometry_msgs.msg import Point
from std_msgs.msg import Time
from time_utils import time_in_float_sec

def main():
    print('Starting the circular trajectory creator')

    rclpy.init()
    node = rclpy.create_node('start_circular_trajectory')

    # if rospy.is_shutdown():
    #     print('ROS master not running!')
    #     sys.exit(-1)

    # If no start time is provided: start *now*.
    start_time = time_in_float_sec(node.get_clock().now()) #rospy.Time.now().to_sec()
    start_now = False
    if node.has_parameter('~start_time'):
        start_time = node.get_parameter('~start_time').get_parameter_value().double_value
        if start_time < 0.0:
            print('Negative start time, setting it to 0.0')
            start_time = 0.0
            start_now = True
    else:
        start_now = True

    param_labels = ['radius', 'center', 'n_points', 'heading_offset',
                    'duration', 'max_forward_speed']
    params = dict()

    for label in param_labels:
        if not node.has_parameter('~' + label):
            print('{} must be provided for the trajectory generation!'.format(label))
            sys.exit(-1)

        params[label] = node.get_parameter('~' + label).value

    if len(params['center']) != 3:
        print('Center of circle must have 3 components (x, y, z)')
        sys.exit(-1)

    if params['n_points'] <= 2:
        print('Number of points must be at least 2')
        sys.exit(-1)

    if params['max_forward_speed'] <= 0:
        print('Velocity limit must be positive')
        sys.exit(-1)

    try:
        traj_gen = node.create_client(InitCircularTrajectory, 'start_circular_trajectory')
    except Exception as e:
        print('Service call failed, error={}'.format(e))
        sys.exit(-1) 
        
    if not traj_gen.wait_for_service(timeout_sec=20):
        print('Service %s not available! Closing node...' %(traj_gen.srv_name))
        sys.exit(-1)

    print('Generating trajectory that starts at t={} s'.format(start_time))

    req = InitCircularTrajectory.Request()
    req.start_time = node.get_clock().now().to_msg()
    req.start_now = start_now
    req.radius = params['radius'],
    req.center = Point(params['center'][0], params['center'][1], params['center'][2])
    req.is_clockwise = False
    req.angle_offset = 0.0
    req.n_points = params['n_points']
    req.heading_offset = params['heading_offset'] * pi / 180
    req.max_forward_speed = params['max_forward_speed']
    req.duration = params['duration']

    success = traj_gen.call(req)

    if success:
        print('Trajectory successfully generated!')
    else:
        print('Failed')


if __name__ == '__main__':
    main()