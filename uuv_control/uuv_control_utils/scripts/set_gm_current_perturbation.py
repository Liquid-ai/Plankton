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
import traceback
from numpy import pi
from uuv_world_ros_plugins_msgs.srv import *
from plankton_utils.time import is_sim_time

def main():
    rclpy.init()

    sim_time_param = is_sim_time()
    
    node = rclpy.create_node(
        'set_gm_current_perturbation', 
        allow_undeclared_parameters=True, 
        automatically_declare_parameters_from_overrides=True, 
        parameter_overrides=[sim_time_param]
    )
    
    node.get_logger().info('Starting current perturbation node')
    node.get_logger().info('Programming the generation of a current perturbation')

    params = ['component', 'mean', 'min', 'max', 'noise', 'mu']
    values = dict()
    for p in params:
        assert node.has_parameter(p), 'All parameters %s are required' % (str(params))     
        values[p] = node.get_parameter(p).value 
        # Assert the numeric type is float
        if type(values[p]) != str:
            values[p] = float(values[p])

    choices = ['velocity', 'horz_angle', 'vert_angle']
    assert values['component'] in choices, ('Component values must be one of ' + str(choices))
    if values['component'] == 'velocity':
        # Just a guess that it should be >= instead of >
        assert values['mean'] >= 0, '"mean" parameter must be >= 0'

    else:
        values['min'] *= pi / 180.0
        values['max'] *= pi / 180.0
        values['mean'] *= pi / 180.0

    assert values['min'] <= values['max'], '"min" parameter must be smaller than "max" parameter'
    assert values['noise'] >= 0, '"noise" parameter must be >= 0'
    assert values['mu'] >= 0, '"mu" parameter must be >= 0'

    set_model = node.create_client(
        SetCurrentModel,
        '/hydrodynamics/set_current_%s_model' % values['component'])
        
    if not set_model.wait_for_service(timeout_sec=30):
        raise RuntimeError("Service %s not running" % (set_model.srv_name))

    req = SetCurrentModel.Request()
    req.mean = values['mean']
    req.min = values['min']
    req.max = values['max']
    req.noise = values['noise']
    req.mu =  values['mu']

    future = set_model.call_async(req)
    rclpy.spin_until_future_complete(node, future)
    if future.result() is not None:
        prop = future.result()
        if prop.success:
            node.get_logger().info('Model for <{}> set successfully!'.format(values['component']))
        else:
            node.get_logger().info('Error setting model!')
    
    node.destroy_node()

# =============================================================================
if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print('Caught exception: ' + repr(e))
        print(traceback.print_exc())
    finally:
        if rclpy.ok():
            rclpy.shutdown()
