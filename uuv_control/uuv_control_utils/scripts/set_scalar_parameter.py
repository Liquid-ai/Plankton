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
import sys
from uuv_gazebo_ros_plugins_msgs.srv import SetFloat

def main():
    rclpy.init()
    node = rclpy.create_node('set_thrusters_states')

    node.get_logger().info('Set scalar parameter, namespace=' + node.get_namespace())


    services = ['set_fluid_density', 'set_added_mass_scaling',
                'set_damping_scaling', 'set_volume_scaling',
                'set_volume_offset', 'set_added_mass_offset',
                'set_linear_damping_offset', 'set_nonlinear_damping_offset',
                'set_linear_forward_speed_damping_offset']

    assert node.has_parameter('service_name')
    assert node.has_parameter('data')

    service_name = node.get_parameter('service_name').get_parameter_value().string_value
    assert service_name in services, 'Possible service names are=' + str(services)

    data = node.get_parameter('data').value

    service = node.create_client(SetFloat, service_name)
    
    if not service.wait_for_service(timeout_sec=30)
        raise RuntimeError("Service %s not running" % (service.srv_name))

    node.get_logger().info('Set scalar parameter, service=%s, data=%.2f' % (service_name, data))

    req = SetFloat.Request()
    req.data = data

    future = set_model.call_async(req)
    rclpy.spin_until_future_complete(self, future)
    if future.result() is not None:
        res = future.result()
        if res.succes:
            node.get_logger().info('Parameter set successfully')
        else:
            node.get_logger().info('Error setting scalar parameter')

    # output = service.call(data)
    # if output.success:
    #     node.get_logger().info('Parameter set successfully')
    # else:
    #     node.get_logger().info('Error setting scalar parameter')
        
    node.get_logger().info(output.message)
    node.get_logger().info('Leaving node...')

#==============================================================================
if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print('Exception caught: ' + str(e))
    finally:
        if rclpy.ok():
            rclpy.shutdown()
