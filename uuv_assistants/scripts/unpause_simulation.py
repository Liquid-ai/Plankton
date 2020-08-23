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
from std_srvs.srv import Empty
import time
import sys


def main():
    rclpy.init()

    node = rclpy.create_node('unpause_simulation',
                            allow_undeclared_parameters=True, 
                            automatically_declare_parameters_from_overrides=True)

    #Default sim_time to True
    sim_time = rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)
    node.set_parameters([sim_time])

    # if not rclpy.ok():
    #     rospy.ROSException('ROS master is not running!')

    timeout = 0.0
    if node.has_parameter('timeout'):
        timeout = node.get_parameter('timeout').get_parameter_value().double_value
        if timeout <= 0:
            raise RuntimeError('Unpause time must be a positive floating point value')

    print('Unpause simulation - Time = {} s'.format(timeout))

    # start_time = time.time()
    # while time.time() - start_time < timeout:
    #     time.sleep(0.1)
    if(timeout > 0):
        time.sleep(timeout)

    start_time = time.time()
    try:
        # Handle for retrieving model properties
        unpause = node.create_client(Empty, '/gazebo/unpause_physics')
        unpause.wait_for_service(timeout_sec=100)
        if(not ready):
            raise rclpy.exceptions.InvalidServiceNameException('service is unavailable')
    except rclpy.exceptions.InvalidServiceNameException:
        print('/gazebo/unpause_physics service is unavailable')
        sys.exit()
    
    node.get_logger().info(
        'The Gazebo "unpause_physics" service was available {} s after the timeout'.format(time.time() - start_time))

    req = Empty.Request()
    future = unpause.call_async(req)
    rclpy.spin_until_future_complete(self, future)
    if future.result() is not None:
        prop = future.result()
        if prop.succes:
            print('Simulation unpaused...')
        else
            node.get_logger().error('Failed to unpaused the simulation')

#==============================================================================
if __name__ == '__main__':
    main()
