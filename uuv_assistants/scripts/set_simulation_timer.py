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
import rclpy.clock
import time
import threading

from plankton_utils.time import time_in_float_sec
from plankton_utils.time import is_sim_time

#==============================================================================
def main():
    rclpy.init()

    sim_time_param = is_sim_time()

    node = rclpy.create_node('set_simulation_timer', 
                             allow_undeclared_parameters=True, 
                             automatically_declare_parameters_from_overrides=True,
                             parameter_overrides=[sim_time_param])

    timeout = 0.0
    if node.has_parameter('timeout'):
        timeout = node.get_parameter('timeout').get_parameter_value().double_value
        if timeout <= 0:
            raise RuntimeError('Termination time must be a positive floating point value (X.Y)')

    node.get_logger().info('Starting simulation timer - Timeout = {} s'.format(timeout))

    # Note that node's clock will be initialized from the /clock topic during the spin in sim tim mode
    # Behaviour of Rate changed in ROS 2. To wake it up, it needs to be triggered from a separate 
    # thread. Maybe a rclpy.spin_once + time.sleep() would be enough
    FREQ = 100
    rate = node.create_rate(FREQ)
    thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    thread.start()
    while time_in_float_sec(node.get_clock().now()) < timeout:
        # Just a guard for really short timeouts
        if 1.0 / FREQ < timeout: 
            rate.sleep()

    node.get_logger().info('Simulation timeout - Killing simulation...')

    # destroy_node() prevents a further warning on exit
    node.destroy_node()
    rclpy.shutdown()
    thread.join()

#==============================================================================
if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print('Caught exception: ' + str(e))
        print('Exiting')
