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
from gazebo_msgs.srv import ApplyBodyWrench
from geometry_msgs.msg import Point, Wrench, Vector3

def main():
    print('Apply programmed perturbation to vehicle', rospy.get_namespace())
    #rospy.init_node('set_body_wrench')

    rclpy.init()
    node = rclpy.create_node('set_body_wrench')

    # if rospy.is_shutdown():
    #     print('ROS master not running!')
    #     sys.exit(-1)

    starting_time = 0.0
    if node.has_parameter('~starting_time'):
        starting_time = node.get_parameter('~starting_time').get_parameter_value().double_value

    print('Starting time= {} s'.format(starting_time))

    duration = 0.0
    if node.has_parameter('~duration'):
        duration = node.get_parameter('~duration').get_parameter_value().double_value

    if duration == 0.0:
        print('Duration not set, leaving node...')
        sys.exit(-1)

    print('Duration [s]=', ('Inf.' if duration < 0 else duration))

    force = [0, 0, 0]
    if node.has_parameter('~force'):
        force = node.get_parameter('~force').value
        print(force)
        if len(force) != 3:
            raise RuntimeError('Invalid force vector')

    print('Force [N]=', force)

    torque = [0, 0, 0]
    if node.has_parameter('~torque'):
        torque = node.get_parameter('~torque').value
        if len(torque) != 3:
            raise RuntimeError('Invalid torque vector')

    print('Torque [N]=', torque)

    try:
        apply_wrench = node.create_client(ApplyBodyWrench, '/gazebo/apply_body_wrench')
    except rospy.ServiceException as e:
        print('Service call failed, error=', e)
        sys.exit(-1)
    
    try:
        ready = apply_wrench.wait_for_service(timeout_sec=10)
        if not ready:
            raise RuntimeError('')
    except:
        print('Service /gazebo/apply_body_wrench not available! Closing node...')
        sys.exit(-1)
    ns = node.get_namespace().replace('/', '')

    body_name = '%s/base_link' % ns

    if starting_time >= 0:
        rate = node.create_rate(100)
        while node.get_clock().now().nanoseconds < starting_time * 1e9:
            rate.sleep()

    apw = ApplyBodyWrench.Request()
    apw.body_name = body_name
    apw.reference_frame = 'world'
    apw.reference_point = Point(0, 0, 0)
    apw.wrench = Wrench()
    apw.wrench.force = Vector3(*force)
    apw.wrench.torque = Vector3(*torque)
    apw.start_time = node.get_clock().now()
    apw.duration = rclpy.time.Duration(duration)

    success = apply_wrench.call(apw)
        
    if success:
        print('Body wrench perturbation applied!')
        print('\tFrame: ', body_name)
        print('\tDuration [s]: ', duration)
        print('\tForce [N]: ', force)
        print('\tTorque [Nm]: ', torque)
    else:
        print('Failed!')

if __name__ == '__main__':
    main()