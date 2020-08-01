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
from numpy import pi
from uuv_world_ros_plugins_msgs.srv import *
from time_utils import time_in_float_sec

def main():
    print('Starting current perturbation node')

    rclpy.init()
    node = rclpy.create_node('set_timed_current_perturbation')

    print('Programming the generation of a current perturbation')
    # if rospy.is_shutdown():
    #     print('ROS master not running!')
    #     sys.exit(-1)

    starting_time = 0.0
    if node.has_parameter('~starting_time'):
        starting_time = node.get_parameter('~starting_time').get_parameter_value().double_value
        if starting_time < 0.0:
            print('Negative starting time, setting it to 0.0')
            starting_time = 0.0

    print('Starting time=', starting_time)

    end_time = -1
    if node.has_parameter('~end_time'):
        end_time = node.get_parameter('~end_time').get_parameter_value().double_value
        if end_time != -1 and end_time <= starting_time:
            raise RuntimeError('End time is smaller than the starting time')

    print('End time=', (end_time if end_time != -1 else 'Inf.'))

    vel = 0.0
    if node.has_parameter('~current_velocity'):
        vel = node.get_parameter('~current_velocity').double_value

    print('Current velocity [m/s]=', vel)

    horz_angle = 0.0
    if node.has_parameter('~horizontal_angle'):
        horz_angle = node.get_parameter('~horizontal_angle').get_parameter_value().double_value
        horz_angle *= pi / 180

    print('Current horizontal angle [deg]=', horz_angle * 180 / pi)

    vert_angle = 0.0
    if node.has_parameter('~vertical_angle'):
        vert_angle = node.get_parameter('~vertical_angle').get_parameter_value().double_value
        vert_angle *= pi / 180

    print('Current vertical angle [deg]=', horz_angle * 180 / pi)



    try:
        set_current = node.create_client(SetCurrentVelocity, '/hydrodynamics/set_current_velocity')
    except Exception as e:
        print('Service call failed, error=', e)
        sys.exit(-1) 
        
    if not set_current.wait_for_service(timeout_sec=20)
        print('Current velocity services not available! Closing node...')
        sys.exit(-1)

    # Wait to set the current model
    rate = node.create_rate(100)
    if rospy.get_time() < starting_time:
        while rospy.get_time() < starting_time:
            rate.sleep()

    print('Applying current model...')
    req = SetCurrentVelocity.Request()
    req.vel = vel
    req.horz_angle = horz_angle
    req.vert_angle = vert_angle
    if set_current(vel, horz_angle, vert_angle):
        print('Current velocity changed successfully at %f s! vel= %f m/s' % (rospy.get_time(), vel))
    else:
        print('Failed to change current velocity')

    if end_time != -1:
        while time_in_float_sec(node.get_clock().now()) < end_time:
            rate.sleep()

        req.vel = 0
        req.horz_angle = horz_angle
        req.vert_angle = vert_angle
        print('TIMEOUT, setting current velocity to zero...')
        set_current.call(req)

    print('Leaving node...')


if __name__ == '__main__':
    main()