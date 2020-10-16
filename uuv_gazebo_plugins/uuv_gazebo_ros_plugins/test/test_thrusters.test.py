#!/usr/bin/env python
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
import array
import numpy as np
import os
import pathlib
import pytest
import rclpy
import unittest
import xacro
import sys
from time import sleep

from uuv_gazebo_ros_plugins_msgs.msg import FloatStamped
from uuv_gazebo_ros_plugins_msgs.srv import GetThrusterConversionFcn, \
    SetThrusterState, GetThrusterState, SetThrusterEfficiency, \
    GetThrusterEfficiency

import launch
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
import launch_testing.actions

from ament_index_python.packages import get_package_share_directory


class TestThrusters(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        # Initialize the ROS context for the test node
        rclpy.init()

    # =========================================================================
    @classmethod
    def tearDownClass(cls):
        # Shutdown the ROS context
        rclpy.shutdown()
        os.system('killall -9 gzserver')

    # =========================================================================
    def setUp(self):
        # Create a ROS node for tests
        self.node = rclpy.create_node(
            'test_thrusters',
            allow_undeclared_parameters=True, 
            automatically_declare_parameters_from_overrides=True
        )

        self.thruster_input_pub = dict()
        for i in range(3):
            self.thruster_input_pub[i] = self.node.create_publisher(
                FloatStamped, self.build_topic_name('/vehicle/thrusters', i, 'input'), 1)

    # =========================================================================
    def tearDown(self):
        self.node.destroy_node()

     # =========================================================================
    def build_topic_name(self, prefix, id, suffix):
        return prefix + '/id_' + str(id) + '/' + suffix

    # =========================================================================
    def create_client(self, srv_type, srv_name):
        s = self.node.create_client(srv_type, srv_name)
        
        if not s.wait_for_service(timeout_sec=10):
            self.fail('service %s not available...' % srv_name)

        return s

    # =========================================================================
    def service_request(self, service, **kwargs):
        req = service.srv_type.Request()
        for key, value in kwargs.items():
            try:
                getattr(req, key)
                setattr(req, key, value)
            except:
                print('Non existing attribute %s' % key)
                pass

        future = service.call_async(req)

        rclpy.spin_until_future_complete(self.node, future)

        return future.result()

    # TODO Migrate the test 
    # =========================================================================
    # def test_input_output_topics_exist(self):
    #     pub = self.node.create_publisher(
    #         FloatStamped, 
    #         self.build_topic_name('/vehicle/thrusters', 0, 'input'),
    #         1
    #     )

    #     # pub = rospy.Publisher(FloatStamped'/vehicle/thrusters/0/input', ,
    #     #                       queue_size=1)

    #     for k in self.thruster_input_pub:
    #         # Publishing set point to rotor velocity
    #         input_message = FloatStamped()
    #         input_message.header.stamp = self.node.get_clock().now().to_msg() # rospy.Time.now()
    #         input_message.data = 0.2

    #         self.thruster_input_pub[k].publish(input_message)
    #         sleep(1)
            
    #         output = rospy.wait_for_message('/vehicle/thrusters/%d/thrust' % k,
    #                                         FloatStamped, timeout=30)
    #         self.assertIsNot(output.data, 0.0)

    #         # Turning thruster off
    #         input_message.data = 0.0
    #         pub.publish(input_message)

    # =========================================================================
    def test_conversion_fcn_parameters(self):
        # Testing thruster #0 - basic/proportional model
        s_get = self.create_client(
            GetThrusterConversionFcn, 
            self.build_topic_name(
                '/vehicle/thrusters', 
                 0, 
                'get_thruster_conversion_fcn'
            )
        )

        fcn = self.service_request(s_get)

        self.assertEqual(fcn.fcn.function_name, 'Basic')
        self.assertEqual(len(fcn.fcn.tags), len(fcn.fcn.data))
        self.assertEqual(len(fcn.fcn.tags), 1)
        self.assertIn('rotor_constant', fcn.fcn.tags)
        self.assertEqual(fcn.fcn.data[0], 0.001)

        # Testing thruster #1 - Bessa/nonlinear model
        s_get = self.create_client(
            GetThrusterConversionFcn, 
            self.build_topic_name(
                '/vehicle/thrusters', 1,
                'get_thruster_conversion_fcn')
        )

        fcn = self.service_request(s_get)

        bessa_tags = ['rotor_constant_l', 'rotor_constant_r', 'delta_l',
                      'delta_r']
        bessa_params = [0.001, 0.001, -0.01, 0.01]
        self.assertEqual(fcn.fcn.function_name, 'Bessa')
        self.assertEqual(len(fcn.fcn.tags), len(fcn.fcn.data))
        self.assertEqual(len(fcn.fcn.tags), 4)
        for t, p in zip(fcn.fcn.tags, fcn.fcn.data):
            self.assertIn(t, bessa_tags)
            self.assertEqual(p, bessa_params[bessa_tags.index(t)])

        # Testing thruster #2 - Linear interpolation
        s_get = self.create_client(
            GetThrusterConversionFcn, 
            self.build_topic_name(
                '/vehicle/thrusters', 2,
                'get_thruster_conversion_fcn')
        )

        fcn = self.service_request(s_get)

        self.assertEqual(fcn.fcn.function_name, 'LinearInterp')
        self.assertEqual(len(fcn.fcn.tags), len(fcn.fcn.data))
        self.assertEqual(len(fcn.fcn.tags), 0)
        self.assertEqual(len(fcn.fcn.lookup_table_input),
                         len(fcn.fcn.lookup_table_output))
        self.assertListEqual([-0.1, 0.0, 0.1],
                             list(fcn.fcn.lookup_table_input))
        self.assertListEqual([-0.01, 0.0, 0.01],
                             list(fcn.fcn.lookup_table_output))

    # =========================================================================
    def test_change_thruster_state(self):
        
        for i in range(3):
            s_set = self.create_client(
                SetThrusterState, 
                self.build_topic_name(
                    '/vehicle/thrusters', i, 'set_thruster_state')
            )
            set_state = self.service_request(s_set, on=False)
            self.assertTrue(set_state.success)

            # Test that thruster is off
            s_get = self.create_client(
                GetThrusterState, 
                self.build_topic_name(
                    '/vehicle/thrusters', i, 'get_thruster_state')
            )

            get_state = self.service_request(s_get)
            self.assertFalse(get_state.is_on)

            # Turn thruster on again
            set_state = self.service_request(s_set, on=True)
            self.assertTrue(set_state.success)

            get_state = self.service_request(s_get)
            self.assertTrue(get_state.is_on)

    # =========================================================================
    def test_change_thrust_efficiency(self):
        for i in range(3):
            s_set = self.create_client(
                SetThrusterEfficiency, 
                self.build_topic_name(
                    '/vehicle/thrusters', i, 'set_thrust_force_efficiency')
            )

            set_efficiency = self.service_request(s_set, efficiency=0.5)
            self.assertTrue(set_efficiency.success)

            # Test that thruster is off
            s_get = self.create_client(
                GetThrusterEfficiency, 
                self.build_topic_name(
                    '/vehicle/thrusters', i, 'get_thrust_force_efficiency')
            )

            get_efficiency = self.service_request(s_get)
            self.assertEqual(get_efficiency.efficiency, 0.5)

            # Turn thruster on again
            set_efficiency = self.service_request(s_set, efficiency=1.0)
            self.assertTrue(set_efficiency.success)

            get_efficiency = self.service_request(s_get)
            self.assertEqual(get_efficiency.efficiency, 1.0)

    # =========================================================================
    def test_change_dyn_state_efficiency(self):
        for i in range(3):            
            s_set = self.create_client(
                SetThrusterEfficiency, 
                self.build_topic_name(
                    '/vehicle/thrusters', i, 'set_dynamic_state_efficiency')
            )

            set_efficiency = self.service_request(s_set, efficiency=0.5)
            self.assertTrue(set_efficiency.success)

            # Test that thruster is off
            s_get = self.create_client(
                GetThrusterEfficiency, 
                self.build_topic_name(
                    '/vehicle/thrusters', i, 'get_dynamic_state_efficiency')
            )

            get_efficiency = self.service_request(s_get)
            self.assertEqual(get_efficiency.efficiency, 0.5)

            # Turn thruster on again
            set_efficiency = self.service_request(s_set, efficiency=1.0)
            self.assertTrue(set_efficiency.success)

            get_efficiency = self.service_request(s_get)
            self.assertEqual(get_efficiency.efficiency, 1.0)


# =============================================================================
@pytest.mark.rostest
def generate_test_description():
    # Set env...not sure if the tests actually connect to the right Gazebo...
    os.environ['GAZEBO_MASTER_URI'] = 'http://localhost:3002'

    file_path = pathlib.Path(__file__)
    # Here, parent first removes the file name
    parent_file_path = pathlib.Path(__file__).parent 

    # Gazebo
    gazebo_launch = os.path.join(
        get_package_share_directory('gazebo_ros'),
        'launch',
        'gazebo.launch.py'
    )

    gazebo_world = os.path.join(
        str(parent_file_path),
        'worlds',
        'test_empty.world',
    )

    if not pathlib.Path(gazebo_launch).exists() or not pathlib.Path(gazebo_world).exists():
        exc = 'Launch file ' + gazebo_launch + ' or ' + gazebo_world + ' does not exist'
        raise Exception(exc)
        
    launch_args = [('world', gazebo_world), ('paused', 'false'), 
                   ('gui', 'false'),        ('verbose', 'true'),]
    gazebo_launch_desc = IncludeLaunchDescription(
            AnyLaunchDescriptionSource(gazebo_launch), launch_arguments=launch_args)

    # Upload vehicle
    upload_launch = os.path.join(
        str(parent_file_path),
        'models',
        'thrusters',
        'test_upload_thrusters.launch.py'
    )

    if not pathlib.Path(upload_launch).exists():
        exc = 'Launch file ' + upload_launch + ' does not exist'
        raise Exception(exc)

    upload_launch_desc = IncludeLaunchDescription(
            AnyLaunchDescriptionSource(upload_launch))

    return (
        launch.LaunchDescription([
            gazebo_launch_desc,
            upload_launch_desc,
            launch_testing.actions.ReadyToTest(),
        ])
    )