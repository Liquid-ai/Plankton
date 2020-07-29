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

    node = rclpy.create_node('unpause_simulation')

    # if not rclpy.ok():
    #     rospy.ROSException('ROS master is not running!')

    timeout = 0.0
    if node.has_parameter('~timeout'):
        timeout = node.get_parameter('~timeout').get_parameter_value().double_value
        if timeout <= 0:
            raise RuntimeError('Unpause time must be a positive floating point value')

    print('Unpause simulation - Time = {} s'.format(timeout))

    # ?
    start_time = time.time()
    while time.time() - start_time < timeout:
        time.sleep(0.1)

    try:
        # Handle for retrieving model properties
        unpause = node.create_client(Empty, '/gazebo/unpause_physics')
        unpause.wait_for_service(timeout_sec=100)
        if(not ready):
            raise rclpy.exceptions.InvalidServiceNameException('service is unavailable')
    except rclpy.exceptions.InvalidServiceNameException:
        print('/gazebo/unpause_physics service is unavailable')
        sys.exit()

    req = Empty.Request()
    unpause.call(req)
    print('Simulation unpaused...')

if __name__ == '__main__':
    main()
