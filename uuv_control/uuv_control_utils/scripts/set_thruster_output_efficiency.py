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
from uuv_gazebo_ros_plugins_msgs.srv import SetThrusterEfficiency
from time_utils import time_in_float_sec

def main():
    rclpy.init()
    node = rclpy.create_node('set_thrusters_states')

    print('Set the thruster output efficiency for vehicle, namespace=', self.get_namespace())

    # if rospy.is_shutdown():
    #     raise rospy.ROSException('ROS master not running!')

    starting_time = 0.0
    if node.has_parameter('~starting_time'):
        starting_time = node.get_parameter('~starting_time').get_parameter_value().double_value

    print('Starting time= {} s'.format(starting_time))

    duration = 0.0
    if node.has_parameter('~duration'):
        duration = node.get_parameter('~duration').get_parameter_value().double_value

    if duration == 0.0:
        raise RuntimeError('Duration not set, leaving node...')

    print('Duration [s]=', ('Inf.' if duration < 0 else duration))

    if node.has_parameter('~efficiency'):
        efficiency = node.get_parameter('~efficiency').get_parameter_value().double_value
        if efficiency < 0 or efficiency > 1:
            raise RuntimeError('Invalid thruster output efficiency, leaving node...')
    else:
        raise RuntimeError('Thruster output efficiency not set, leaving node...')

    if node.has_parameter('~thruster_id'):
        thruster_id = node.get_parameter('~thruster_id').get_parameter_value().integer_value
    else:
        raise RuntimeError('Thruster ID not given')

    if thruster_id < 0:
        raise RuntimeError('Invalid thruster ID')

    print('Setting thruster output efficiency #{} to {}'.format(thruster_id, 100 * efficiency))

    vehicle_name = node.get_namespace().replace('/', '')

    srv_name = '/%s/thrusters/%d/set_thrust_force_efficiency' % (vehicle_name, thruster_id)
   
    try:
        set_eff = node.create_client(SetThrusterEfficiency, srv_name)
    except Exception as e:
        print('Service call failed, error=' + str(e))
        
    if not set_eff.wait_for_service(timeout_sec=2):
        raise RuntimeError('Service %s not available! Closing node...' %(srv_name))


    rate = node.create_rate(100)
    while time_in_float_sec(node.get_clock().now()) < starting_time:
        rate.sleep()

    req = SetThrusterEfficiency.Request()
    req.efficiency = efficiency
    success = set_eff.call(efficiency)

    if success:
        print('Time={} s'.format(rospy.get_time()))
        print('Current thruster output efficiency #{}={}'.format(thruster_id, efficiency * 100))

    if duration > 0:
        rate = node.create_rate.Rate(100)
        while time_in_float_sec(node.get_clock().now()) < starting_time + duration:
            rate.sleep()

        req.efficiency = 1.0
        success = set_eff.call(efficiency)

        if success:
            print('Time={} s'.format(time_in_float_sec(node.get_clock().now())))
            print('Returning to previous thruster output efficiency #{}={}'.format(thruster_id, efficiency * 100))

    print('Leaving node...')
    
if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print('Something went wrong: ' + str(e))