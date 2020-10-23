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
import traceback
import threading

import rclpy
import sys
from numpy import pi
from uuv_world_ros_plugins_msgs.srv import *
from plankton_utils.time import time_in_float_sec
from plankton_utils.time import is_sim_time


def main():
    rclpy.init()

    sim_time_param = is_sim_time()

    node = rclpy.create_node(
        'set_timed_current_perturbation',
        allow_undeclared_parameters=True, 
        automatically_declare_parameters_from_overrides=True,
        parameter_overrides=[sim_time_param])

    node.get_logger().info('Starting current perturbation node')
    node.get_logger().info('Programming the generation of a current perturbation')

    starting_time = 0.0
    if node.has_parameter('starting_time'):
        starting_time = node.get_parameter('starting_time').value
        if starting_time < 0.0:
            node.get_logger().warn('Negative starting time, setting it to 0.0')
            starting_time = 0.0

    node.get_logger().info('Starting time={} s'.format(starting_time))

    end_time = -1
    if node.has_parameter('end_time'):
        end_time = node.get_parameter('end_time').value
        if end_time > 0 and end_time <= starting_time:
            raise RuntimeError('End time is smaller than the starting time')

    node.get_logger().info('End time=%s' % (end_time if end_time > 0 else 'Inf.'))

    vel = 0.0
    if node.has_parameter('current_velocity'):
        vel = float(node.get_parameter('current_velocity').value)

    node.get_logger().info('Current velocity [m/s]={}'.format(vel))

    horz_angle = 0.0
    if node.has_parameter('horizontal_angle'):
        horz_angle = float(node.get_parameter('horizontal_angle').value)
        horz_angle *= pi / 180

    node.get_logger().info('Current horizontal angle [deg]={}'.format( horz_angle * 180 / pi))

    vert_angle = 0.0
    if node.has_parameter('vertical_angle'):
        vert_angle = float(node.get_parameter('vertical_angle').value)
        vert_angle *= pi / 180

    node.get_logger().info('Current vertical angle [deg]={}'.format(horz_angle * 180 / pi))

    # Create client service
    srv_name = '/hydrodynamics/set_current_velocity' 
    set_current = node.create_client(SetCurrentVelocity, srv_name)
        
    if not set_current.wait_for_service(timeout_sec=20):
        node.get_logger().error('%s service not available! Closing node...' %(srv_name))
        sys.exit(-1)

    # Wait to set the current model
    FREQ = 100
    rate = node.create_rate(FREQ)
    thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    thread.start()

    # Make sure the clock has been updated when using sim time
    while node.get_clock().now() == rclpy.time.Time(clock_type=node.get_clock().clock_type):
        pass
        
    while time_in_float_sec(node.get_clock().now()) < starting_time:
        if 1.0 / FREQ < starting_time: 
            rate.sleep()

    node.get_logger().info('Applying current model...')
    req = SetCurrentVelocity.Request()
    req.velocity = vel
    req.horizontal_angle = horz_angle
    req.vertical_angle = vert_angle

    future = set_current.call_async(req)

    # NB : spining is done from another thread  
    while not future.done():
        pass

    try:
        response = future.result()
    except Exception as e:
        node.get_logger().error('Service call ' + srv_name + ' failed, error=' + repr(e))
    else:
        if response.success:
            t = time_in_float_sec(node.get_clock().now())
            node.get_logger().info('Current velocity changed successfully at %f s! vel= %f m/s' % (t, vel))
        else:
            node.get_logger().error('Service call ' + srv_name + ' return a "failed" value')

    # Returning to previous state
    if end_time > 0:
        while time_in_float_sec(node.get_clock().now()) < end_time:
            if 1.0 / FREQ < end_time: 
                rate.sleep()

        req.velocity = 0.0
        req.horizontal_angle = horz_angle
        req.vertical_angle = vert_angle
        node.get_logger().info('TIMEOUT, setting current velocity to zero...')

        future = set_current.call_async(req)

        # NB : spining is done from another thread    
        while not future.done():
            pass
            
        try:
            response = future.result()
        except Exception as e:
            node.get_logger().error('Service call ' + srv_name + ' failed, error=' + repr(e))

    node.get_logger().info('Leaving node...')

    node.destroy_node()
    rclpy.shutdown()
    thread.join()

# =============================================================================
if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print('Something went wrong: ' + repr(e))
        print(traceback.print_exc())
    finally:
        if rclpy.ok():
            rclpy.shutdown()
