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
from std_srvs.srv import Empty
import time
import sys

from plankton_utils.time import is_sim_time

def main():
    rclpy.init()

    sim_time_param = is_sim_time()

    node = rclpy.create_node('unpause_simulation',
                            allow_undeclared_parameters=True, 
                            automatically_declare_parameters_from_overrides=True,
                            parameter_overrides=[sim_time_param])

    timeout = 0.0
    if node.has_parameter('timeout'):
        timeout = node.get_parameter('timeout').get_parameter_value().double_value
        if timeout <= 0:
            raise RuntimeError('Unpause time must be a positive floating point value')

    print('Unpause simulation - Time = {} s'.format(timeout))

    if(timeout > 0):
        time.sleep(timeout)

     # Handle for retrieving model properties
    unpause = node.create_client(Empty, '/gazebo/unpause_physics')
    unpause.wait_for_service(timeout_sec=100)
    if(not ready):
        raise Exception('Service %s is unavailable' % unpause.srv_name)
        sys.exit()
    
    node.get_logger().info(
        'The Gazebo "unpause_physics" service was available {} s after the timeout'.format(timeout))

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
