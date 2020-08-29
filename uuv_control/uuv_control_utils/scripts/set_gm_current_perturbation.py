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
#from __future__ import print_function
import rclpy
import sys
from numpy import pi
from uuv_world_ros_plugins_msgs.srv import *

def main():
    rclpy.init()
    node = rclpy.create_node('set_gm_current_perturbation')
    
    node.get_logger().info('Starting current perturbation node')
    node.get_logger().info('Programming the generation of a current perturbation')

    params = ['component', 'mean', 'min', 'max', 'noise', 'mu']
    values = dict()
    for p in params:
        assert node.has_parameter(p)
        values[p] = node.get_parameter(p).value

    assert values['component'] in ['velocity', 'horz_angle', 'vert_angle']
    if values['component'] == 'velocity':
        assert values['mean'] > 0
    else:
        values['min'] *= pi / 180.0
        values['max'] *= pi / 180.0
        values['mean'] *= pi / 180.0

    assert values['min'] < values['max']
    assert values['noise'] >= 0
    assert values['mu'] >= 0

    set_model = node.create_client(
        SetCurrentModel,
        '/hydrodynamics/set_current_%s_model' % values['component'])
        
    if not set_model.wait_for_service(timeout_sec=30):
        raise RuntimeError("Service %s not running" % (set_model.srv_name))

    req = SetCurrentModel.Request()
    req.mean = values['mean']
    req.min = values['min']
    req.max = values['max']
    req.noise =values['noise']
    req.mu =  values['mu']

    future = set_model.call_async(req)
    rclpy.spin_until_future_complete(self, future)
    if future.result() is not None:
        prop = future.result()
        if prop.succes:
            node.get_logger().info(('Model for <{}> set successfully!'.format(values['component']))
        else:
            node.get_logger().info(('Error setting model!')

#==============================================================================
if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print('Caught exception: ' + str(e))
    finally:
        if rclpy.ok():
            rclpy.shutdown()