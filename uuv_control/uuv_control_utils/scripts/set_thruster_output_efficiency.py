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
import threading

from uuv_gazebo_ros_plugins_msgs.srv import SetThrusterEfficiency
from plankton_utils.time import time_in_float_sec


def build_service_name(ns, thruster_id, service_name) -> str :
    return '/%s/thrusters/id_%d/%s' % (ns, thruster_id, service_name)

#==============================================================================
def main():
    rclpy.init()
    node = rclpy.create_node('set_thrusters_states',
                            allow_undeclared_parameters=True, 
                            automatically_declare_parameters_from_overrides=True)

    node.get_logger().info('Set the thruster output efficiency for vehicle, namespace=' + self.get_namespace())
    
    sim_time = rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)
    node.set_parameters([sim_time])

    starting_time = 0.0
    if node.has_parameter('starting_time'):
        starting_time = float(node.get_parameter('starting_time').value)

    node.get_logger().info('Starting time= {} s'.format(starting_time))

    duration = 0.0
    if node.has_parameter('duration'):
        duration = float(node.get_parameter('duration').value)

    if duration <= 0.0:
        raise RuntimeError('Duration not set, leaving node...')

    node.get_logger().info('Duration [s]=', ('Inf.' if duration < 0 else str(duration)))

    if node.has_parameter('efficiency'):
        efficiency = float(node.get_parameter('efficiency').value)
        if efficiency < 0 or efficiency > 1:
            raise RuntimeError('Invalid thruster output efficiency, leaving node...')
    else:
        raise RuntimeError('Thruster output efficiency not set, leaving node...')

    thruster_id = -1
    if node.has_parameter('thruster_id'):
        if node.get_parameter('thruster_id').type_ == rclpy.Parameter.Type.INTEGER:
            thruster_id = node.get_parameter('thruster_id').get_parameter_value().integer_value
    else:
        raise RuntimeError('Thruster ID not given')

    if thruster_id < 0:
        raise RuntimeError('Invalid thruster ID')

    node.get_logger().info('Setting thruster output efficiency #{} to {}'.format(thruster_id, 100 * efficiency))

    vehicle_name = node.get_namespace().replace('/', '')

    srv_name = build_service_name(vehicle_name, thruster_id, 'set_thrust_force_efficiency')
   
    set_eff = node.create_client(SetThrusterEfficiency, srv_name)
        
    if not set_eff.wait_for_service(timeout_sec=2):
        raise RuntimeError('Service %s not available! Closing node...' %(srv_name))

    FREQ = 100
    rate = node.create_rate(FREQ)
    thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    thread.start()
    while time_in_float_sec(node.get_clock().now()) < starting_time:
        #Just a guard for really short timeouts
        if 1.0 / FREQ < starting_time: 
            rate.sleep()

    # rate = node.create_rate(100)
    # while time_in_float_sec(node.get_clock().now()) < starting_time:
    #     rate.sleep()

    req = SetThrusterEfficiency.Request()
    req.efficiency = efficiency
    #success = set_eff.call(efficiency)

    future = set_eff.call_async(efficiency)

    #NB : spining is done from another thread    
    while not future.done():
        try:
            response = future.result()
        except Exception as e:
            node.get_logger().info('Service call ' + srv_name + ' failed, error=' + str(e)):
        else:
            node.get_logger().info('Time={} s'.format(rtime_in_float_sec(node.get_clock().now())))
            node.get_logger().info('Current thruster output efficiency #{}={}'.format(thruster_id, efficiency * 100))

    if duration > 0:
        while time_in_float_sec(node.get_clock().now()) < starting_time + duration:
            if 1.0 / FREQ < starting_time + duration:
                rate.sleep()

        # rate = node.create_rate.Rate(100)
        # while time_in_float_sec(node.get_clock().now()) < starting_time + duration:
        #     rate.sleep()

        req.efficiency = 1.0
        success = set_eff.call(efficiency)

        while not future.done():
            try:
                response = future.result()
            except Exception as e:
                node.get_logger().info('Service call ' + srv_name + ' failed, error=' + str(e)):
            else:
                node.get_logger().info('Time={} s'.format(time_in_float_sec(node.get_clock().now())))
                node.get_logger().infoint(
                    'Returning to previous thruster output efficiency #{}={}'.format(thruster_id, efficiency * 100))

        # if success:
        #     print('Time={} s'.format(time_in_float_sec(node.get_clock().now())))
        #     print('Returning to previous thruster output efficiency #{}={}'.format(thruster_id, efficiency * 100))

    node.get_logger().info('Leaving node...')

    node.destroy_node()
    rclpy.shutdown()
    thread.join()
    
#==============================================================================
if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print('Something went wrong: ' + str(e))
    finally:
        if rclpy.ok():
            rclpy.shutdown()
