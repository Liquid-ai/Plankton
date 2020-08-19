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
from uuv_gazebo_ros_plugins_msgs.srv import SetThrusterState
from time_utils import time_in_float_sec

def build_service_name(ns, thruster_id, service_name) -> str :
    return '/%s/thrusters/id_%d/%s' % (ns, thruster_id, service_name)

def main():
    rclpy.init('set_thrusters_states')
    node = rclpy.create_node()

    print('Set the state of thrusters for vehicle, namespace=', node.get_namespace())

    # if rospy.is_shutdown():
    #     rospy.ROSException('ROS master not running!')

    starting_time = 0.0
    if node.has_parameter('~starting_time'):
        starting_time = node.get_parameter('~starting_time').get_parameter_value().double_value

    print('Starting time={} s'.format(starting_time))

    duration = 0.0
    if node.has_parameter('~duration'):
        duration = node.get_parameter('~duration').get_parameter_value().double_value

    if duration == 0.0:
        raise RuntimeError('Duration not set, leaving node...')

    print('Duration [s]=', ('Inf.' if duration < 0 else duration))

    if node.has_parameter('~is_on'):
        is_on = bool(node.get_parameter('~is_on').get_parameter_value().bool_value)
    else:
        raise RuntimeError('State flag not provided')

    if node.has_parameter('~thruster_id'):
        thruster_id = node.get_parameter('~thruster_id').get_parameter_value().integer_value()
    else:
        raise RuntimeError('Thruster ID not given')

    if thruster_id < 0:
        raise RuntimeError('Invalid thruster ID')

    print('Setting state of thruster #{} as {}'.format(thruster_id, 'ON' if is_on else 'OFF'))

    vehicle_name = node.get_namespace().replace('/', '')

    srv_name = build_service_name(vehicle_name, thruster_id, 'set_thruster_state')

    try:
        set_state = node.create_client(SetThrusterState, srv_name)
    except Exception as e:
        raise RuntimeError('Service call failed, error=' + e)

    if not set_state.wait_for_service(timeout_sec=2):
        raise RuntimeError('Service %s not available! Closing node...' %(srv_name))

    rate = node.create_rate(100)
    while time_in_float_sec(node.get_clock().now()) < starting_time:
        rate.sleep()

    req = SetThrusterState.Request()
    req.on = is_on
    success = set_state.call(req)

    if success:
        print('Time={} s'.format(time_in_float_sec(node.get_clock().now().get_time())))
        print('Current state of thruster #{}={}'.format(thruster_id, 'ON' if is_on else 'OFF'))

    if duration > 0:
        rate = node.create_rate(100)
        while time_in_float_sec(node.get_clock().now()) < starting_time + duration:
            rate.sleep()

        req.on = not is_on
        success = set_state.call(req)

        if success:
            print('Time={} s'.format(time_in_float_sec(node.get_clock().now().get_time())))
            print('Returning to previous state of thruster #{}={}'.format(thruster_id, 'ON' if not is_on else 'OFF'))

    print('Leaving node...')


if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print('Something went wrong: ' + str(e))