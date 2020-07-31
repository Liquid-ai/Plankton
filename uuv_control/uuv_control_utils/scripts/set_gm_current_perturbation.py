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
import sys
from numpy import pi
from uuv_world_ros_plugins_msgs.srv import *

def main():
    print('Starting current perturbation node')

    rclpy.init()
    node = rclpy.create_node('set_gm_current_perturbation')

    print('Programming the generation of a current perturbation')
    # if rospy.is_shutdown():
    #     print('ROS master not running!')
    #     sys.exit(-1)

    params = ['component', 'mean', 'min', 'max', 'noise', 'mu']
    values = dict()
    for p in params:
        assert node.has_parameter('~' + p)
        values[p] = node.get_parameter('~' + p).value

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
    if set_model.call(req):
        print('Model for <{}> set successfully!'.format(values['component']))
    else:
        print('Error setting model!')

if __name__ == '__main__':
    main()