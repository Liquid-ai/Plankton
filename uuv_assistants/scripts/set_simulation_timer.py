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
import time

#==============================================================================
def main():
    rclpy.init()

    node = rclpy.create_node('set_simulation_timer', 
                             allow_undeclared_parameters=True, 
                             automatically_declare_parameters_from_overrides=True)

    timeout = 0.0
    if node.has_parameter('timeout'):
        timeout = node.get_parameter('timeout').get_parameter_value().double_value
        if timeout <= 0:
            raise RuntimeError('Termination time must be a positive floating point value (X.Y)')

    node.get_logger().info('Starting simulation timer - Timeout = {} s'.format(timeout))

    if(timeout > 0):
        time.sleep(timeout)

    print('Simulation timeout - Killing simulation...')

    rclpy.shutdown()


#==============================================================================
if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print('Caught exception: ' + str(e))
        print('Exiting')
