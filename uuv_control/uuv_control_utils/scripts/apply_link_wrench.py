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
import time
import threading

import rclpy
from geometry_msgs.msg import Point, Wrench, Vector3

from gazebo_msgs.srv import ApplyLinkWrench

from plankton_utils.time import time_in_float_sec
from plankton_utils.time import float_sec_to_int_sec_nano
from plankton_utils.time import is_sim_time

def main():
    rclpy.init()

    sim_time_param = is_sim_time()

    node = rclpy.create_node('set_link_wrench',
                            allow_undeclared_parameters=True, 
                            automatically_declare_parameters_from_overrides=True, 
                            parameter_overrides=[sim_time_param])

    node.get_logger().info('Apply programmed perturbation to vehicle ' + node.get_namespace())

    starting_time = 0.0
    if node.has_parameter('starting_time'):
        starting_time = float(node.get_parameter('starting_time').value)

    node.get_logger().info('Starting time in = {} s'.format(starting_time))

    duration = 0.0
    if node.has_parameter('duration'):
        duration = float(node.get_parameter('duration').value)
    
    # Compare to eps ?
    if duration == 0.0:
        node.get_logger().info('Duration not set, leaving node...')
        sys.exit(-1)

    node.get_logger().info('Duration [s]=' + ('Inf.' if duration < 0 else str(duration)))

    force = [0.0, 0.0, 0.0]
    if node.has_parameter('force'):
        force = node.get_parameter('force').value
        if len(force) != 3:
            raise RuntimeError('Invalid force vector')
        # Ensure type is float
        force = [float(elem) for elem in force]

    node.get_logger().info('Force [N]=' + str(force))

    torque = [0.0, 0.0, 0.0]
    if node.has_parameter('torque'):
        torque = node.get_parameter('torque').value
        if len(torque) != 3:
            raise RuntimeError('Invalid torque vector')
        # Ensure type is float
        torque = [float(elem) for elem in torque]

    node.get_logger().info('Torque [N]=' + str(torque))

    gazebo_ns = 'gazebo'
    if node.has_parameter('gazebo_ns'):
        gazebo_ns = node.get_parameter('gazebo_ns').value
    if not gazebo_ns.startswith('/'):
        gazebo_ns = '/' + gazebo_ns

    service_name = '{}/apply_link_wrench'.format(gazebo_ns)
    try:
        apply_wrench = node.create_client(ApplyLinkWrench, service_name)
    except Exception as e:
        node.get_logger().error('Creation of service ' + service_name + ' failed, error=' + str(e))
        sys.exit(-1)
    
    try:
        ready = apply_wrench.wait_for_service(timeout_sec=10)
        if not ready:
            raise RuntimeError('')
    except:
        node.get_logger().error('Service ' + service_name + ' not available! Closing node...')
        sys.exit(-1)

    ns = node.get_namespace().replace('/', '')

    body_name = '%s/base_link' % ns

    FREQ = 100
    rate = node.create_rate(FREQ)
    thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    thread.start()
    while time_in_float_sec(node.get_clock().now()) < starting_time:
        # Just a guard for really short timeouts
        if 1.0 / FREQ < starting_time: 
            rate.sleep()

    (d_secs, d_nsecs) = float_sec_to_int_sec_nano(duration)

    apw = ApplyLinkWrench.Request()
    apw.link_name = body_name
    apw.reference_frame = 'world'
    apw.reference_point = Point(x=0.0, y=0.0, z=0.0)
    apw.wrench = Wrench()
    apw.wrench.force = Vector3(x=force[0], y=force[1], z=force[2])
    apw.wrench.torque = Vector3(x=torque[0], y=torque[1], z=torque[2])
    apw.start_time = node.get_clock().now().to_msg()
    apw.duration = rclpy.time.Duration(seconds=d_secs, nanoseconds=d_nsecs).to_msg()

    future = apply_wrench.call_async(apw)

    # NB : spining is done from another thread
    while rclpy.ok():
        if future.done():
            try:
                response = future.result()
            except Exception as e:
                node.get_logger().error(
                    'Could not apply link wrench, exception thrown: %r' % (e,))
            else:
                if response.success:
                    message = '\nLink wrench perturbation applied!'
                    message += '\n\tFrame: '+ body_name
                    message += '\n\tDuration [s]: ' + str(duration)
                    message += '\n\tForce [N]: ' + str(force)
                    message += '\n\tTorque [Nm]: ' + str(torque)
                    node.get_logger().info(message)
                else:
                    message = 'Could not apply link wrench'
                    message += '\n\tError is: %s' % (response.status_message)
                    node.get_logger().error(message)
            finally:
                break

    node.destroy_node()


# =============================================================================
if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print('Caught exception: ' + str(e))
    finally:
        if rclpy.ok():
            rclpy.shutdown()
