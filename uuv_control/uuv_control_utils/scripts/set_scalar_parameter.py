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
import rclpy
import sys
from uuv_gazebo_ros_plugins_msgs.srv import SetFloat

def main():
    rclpy.init()
    node = rclpy.create_node('set_thrusters_states')

    print('Set scalar parameter, namespace=' + node.get_namespace())

    # if rospy.is_shutdown():
    #     rospy.ROSException('ROS master not running!')

    services = ['set_fluid_density', 'set_added_mass_scaling',
                'set_damping_scaling', 'set_volume_scaling',
                'set_volume_offset', 'set_added_mass_offset',
                'set_linear_damping_offset', 'set_nonlinear_damping_offset',
                'set_linear_forward_speed_damping_offset']

    assert node.has_parameter('~service_name')
    assert node.has_parameter('~data')

    service_name = node.get_parameter('~service_name').get_parameter_value().string_value
    assert service_name in services, 'Possible service names are=' + str(services)

    data = node.get_parameter('~data').value

    

    service = node.create_client(SetFloat, service_name)
    
    service.wait_for_service(timeout_sec=30)
        raise RuntimeError("Service %s not running" % (service.srv_name))

    print('Set scalar parameter, service=%s, data=%.2f' % (service_name, data))

    req = SetFloat.Request()
    req.data = data
    output = service.call(data)

    if output.success:
        print('Parameter set successfully')
    else:
        print('Error setting scalar parameter')
        
    print(output.message)
    print('Leaving node...')


if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print('Exception caught: ' + str(e))