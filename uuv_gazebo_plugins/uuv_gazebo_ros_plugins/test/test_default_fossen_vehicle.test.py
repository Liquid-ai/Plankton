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
from geometry_msgs.msg import Vector3, Inertia
from uuv_gazebo_ros_plugins_msgs.msg import UnderwaterObjectModel
from uuv_gazebo_ros_plugins_msgs.srv import *

import rclpy
import xacro
import launch
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
import launch_testing.actions
from ament_index_python.packages import get_package_share_directory

import array
import unittest
import pytest
import os
import pathlib


class TestDefaultFossenVehicle(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        # Initialize the ROS context for the test node
        rclpy.init()

    # =========================================================================
    @classmethod
    def tearDownClass(cls):
        # Shutdown the ROS context
        rclpy.shutdown()
        # Needed to force Gazebo 9 to shutdown when the tests finish
        os.system('killall -9 gzserver')

    # =========================================================================
    def setUp(self):
        # Create a ROS node for tests
        self.node = rclpy.create_node(
            'test_default_fossen_vehicle',
            allow_undeclared_parameters=True, 
            automatically_declare_parameters_from_overrides=True
        )

    # =========================================================================
    def tearDown(self):
        self.node.destroy_node()

    # =========================================================================
    def create_client(self, srv_type, srv_name):
        s = self.node.create_client(srv_type, srv_name)
        
        if not s.wait_for_service(timeout_sec=10):
            self.fail('service %s not available...' % srv_name)

        return s

    # =========================================================================
    # @pytest.mark.skip()
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

    # =========================================================================
    def test_get_model_parameters(self):
        srv_name = '/vehicle/get_model_properties'
        s_get = self.create_client(GetModelProperties, srv_name)

        models = self.service_request(s_get)

        self.assertEqual(len(models.link_names), 1)
        self.assertEqual(len(models.models), 1)

        # Test the name of the link
        self.assertEqual(
            models.link_names[0], 'vehicle/base_link',
            'Link name is invalid, name=' + str(models.link_names[0]))

        # Test message types
        self.assertIsInstance(models.models[0].added_mass, array.array)
        self.assertIsInstance(models.models[0].linear_damping, array.array)
        self.assertIsInstance(models.models[0].linear_damping_forward_speed, array.array)
        self.assertIsInstance(models.models[0].quadratic_damping, array.array)
        self.assertIsInstance(models.models[0].volume, float)
        self.assertIsInstance(models.models[0].bbox_length, float)
        self.assertIsInstance(models.models[0].bbox_width, float)
        self.assertIsInstance(models.models[0].bbox_height, float)
        self.assertIsInstance(models.models[0].fluid_density, float)
        self.assertIsInstance(models.models[0].neutrally_buoyant, bool)
        self.assertIsInstance(models.models[0].cob, Vector3)
        self.assertIsInstance(models.models[0].inertia, Inertia)

        # Test size of the parameter lists
        # Generate index numbers for the diagonal elements of the matrices
        d_idxs = [i*6 + j for i, j in zip(range(6), range(6))]
        self.assertEqual(len(models.models[0].added_mass), 36)
        for i in range(len(models.models[0].added_mass)):
            if i in d_idxs:
                self.assertEqual(models.models[0].added_mass[i], 1.0)
            else:
                self.assertEqual(models.models[0].added_mass[i], 0.0)

        self.assertEqual(len(models.models[0].linear_damping), 36)
        for i in range(len(models.models[0].linear_damping)):
            if i in d_idxs:
                self.assertEqual(models.models[0].linear_damping[i], 1.0)
            else:
                self.assertEqual(models.models[0].linear_damping[i], 0.0)

        self.assertEqual(len(models.models[0].linear_damping_forward_speed), 36)
        for i in range(len(models.models[0].linear_damping_forward_speed)):
            if i in d_idxs:
                self.assertEqual(models.models[0].linear_damping_forward_speed[i], 1.0)
            else:
                self.assertEqual(models.models[0].linear_damping_forward_speed[i], 0.0)

        self.assertEqual(len(models.models[0].quadratic_damping), 36)
        for i in range(len(models.models[0].quadratic_damping)):
            if i in d_idxs:
                self.assertEqual(models.models[0].quadratic_damping[i], 1.0)
            else:
                self.assertEqual(models.models[0].quadratic_damping[i], 0.0)

        # Tests if some of the parameters match to the ones given in the URDF
        self.assertEqual(models.models[0].fluid_density, 1028.0)
        self.assertEqual(models.models[0].volume, 1.0)
        self.assertEqual(models.models[0].bbox_height, 1.0)
        self.assertEqual(models.models[0].bbox_length, 1.0)
        self.assertEqual(models.models[0].bbox_width, 1.0)

    # =========================================================================
    def test_set_fluid_density(self):
        srv_name = '/vehicle/get_fluid_density'
   
        s_get = self.create_client(GetFloat, srv_name) 

        get_func = self.service_request(s_get)
    
        self.assertEqual(get_func.data, 1028.0)

        #
        srv_name = '/vehicle/set_fluid_density'
        s_set = self.create_client(SetFloat, srv_name)

        set_func = self.service_request(s_set, data=1025.0)
        self.assertTrue(set_func.success)

        get_func = self.service_request(s_get)
        
        self.assertEqual(get_func.data, 1025.0)

        set_func = self.service_request(s_set, data=1028.0)
        self.assertTrue(set_func.success)

    # =========================================================================
    def test_volume_offset(self):
        s_set = self.create_client(SetFloat, '/vehicle/set_volume_offset')
        
        s_get = self.create_client(GetFloat, '/vehicle/get_volume_offset')

        get_model = self.create_client(GetModelProperties, '/vehicle/get_model_properties')

        # Test that offset has changed
        get_func = self.service_request(s_get)
        self.assertEqual(get_func.data, 0.0)

        set_func = self.service_request(s_set, data=1.0)
        self.assertTrue(set_func.success)

        get_func = self.service_request(s_get)
        self.assertEqual(get_func.data, 1.0)

        # Test that the actual volume has NOT changed, offset should only
        # be used during the computation of forces
        get_model_func = self.service_request(get_model)
        self.assertEqual(get_model_func.models[0].volume, 1.0)

        set_func = self.service_request(s_set, data=0.0)
        self.assertTrue(set_func.success)

    # =========================================================================
    def test_added_mass_scaling(self):
        s_set = self.create_client(SetFloat, '/vehicle/set_added_mass_scaling')

        s_get = self.create_client(GetFloat, '/vehicle/get_added_mass_scaling')

        get_func = self.service_request(s_get)
        self.assertEqual(get_func.data, 1.0)

        set_func = self.service_request(s_set, data=0.8)
        self.assertTrue(set_func.success)

        get_func = self.service_request(s_get)
        self.assertEqual(get_func.data, 0.8)

        set_func = self.service_request(s_set, data=1.0)
        self.assertTrue(set_func.success)

    # =========================================================================
    def test_damping_scaling(self):
        s_set = self.create_client(SetFloat, '/vehicle/set_damping_scaling')

        s_get = self.create_client(GetFloat, '/vehicle/get_damping_scaling')

        get_func = self.service_request(s_get)
        self.assertEqual(get_func.data, 1.0)

        set_func = self.service_request(s_set, data=0.8)
        self.assertTrue(set_func.success)

        get_func = self.service_request(s_get)
        self.assertEqual(get_func.data, 0.8)

        set_func = self.service_request(s_set, data=1.0)
        self.assertTrue(set_func.success)

    # =========================================================================
    def test_volume_scaling(self):
        s_set = self.create_client(SetFloat, '/vehicle/set_volume_scaling')

        s_get = self.create_client(GetFloat, '/vehicle/get_volume_scaling')

        get_func = self.service_request(s_get)
        self.assertEqual(get_func.data, 1.0)

        set_func = self.service_request(s_set, data=0.8)
        self.assertTrue(set_func.success)

        get_func = self.service_request(s_get)
        self.assertEqual(get_func.data, 0.8)

        set_func = self.service_request(s_set, data=1.0)
        self.assertTrue(set_func.success)

    # =========================================================================
    def test_added_mass_offset(self):
        s_set = self.create_client(SetFloat, '/vehicle/set_added_mass_offset')

        s_get = self.create_client(GetFloat, '/vehicle/get_added_mass_offset')

        get_func = self.service_request(s_get)
        self.assertEqual(get_func.data, 0.0)

        set_func = self.service_request(s_set, data=1.0)
        self.assertTrue(set_func.success)

        get_func = self.service_request(s_get)
        self.assertEqual(get_func.data, 1.0)

        set_func = self.service_request(s_set, data=0.0)
        self.assertTrue(set_func.success)

    # =========================================================================
    def test_linear_damping_offset(self):
        s_set = self.create_client(SetFloat, '/vehicle/set_linear_damping_offset')

        s_get = self.create_client(GetFloat, '/vehicle/get_linear_damping_offset')

        get_func = self.service_request(s_get)
        self.assertEqual(get_func.data, 0.0)

        set_func = self.service_request(s_set, data=1.0)
        self.assertTrue(set_func.success)

        get_func = self.service_request(s_get)
        self.assertEqual(get_func.data, 1.0)

        set_func = self.service_request(s_set, data=0.0)
        self.assertTrue(set_func.success)

    # =========================================================================
    def test_linear_forward_speed_damping_offset(self):
        s_set = self.create_client(SetFloat, '/vehicle/set_linear_forward_speed_damping_offset')

        s_get = self.create_client(GetFloat, '/vehicle/get_linear_forward_speed_damping_offset')

        get_func = self.service_request(s_get)
        self.assertEqual(get_func.data, 0.0)

        set_func = self.service_request(s_set, data=1.0)
        self.assertTrue(set_func.success)

        get_func = self.service_request(s_get)
        self.assertEqual(get_func.data, 1.0)

        set_func = self.service_request(s_set, data=0.0)
        self.assertTrue(set_func.success)

    # =========================================================================
    def test_linear_forward_speed_damping_offset(self):
        s_set = self.create_client(SetFloat, '/vehicle/set_nonlinear_damping_offset')

        s_get = self.create_client(GetFloat, '/vehicle/get_nonlinear_damping_offset')

        get_func = self.service_request(s_get)
        self.assertEqual(get_func.data, 0.0)

        set_func = self.service_request(s_set, data=1.0)
        self.assertTrue(set_func.success)

        get_func = self.service_request(s_get)
        self.assertEqual(get_func.data, 1.0)

        set_func = self.service_request(s_set, data=0.0)
        self.assertTrue(set_func.success)


# =============================================================================
@pytest.mark.rostest
def generate_test_description():
    # Set env...not sure if the tests actually connect to the right Gazebo...
    # os.environ['GAZEBO_MASTER_URI'] ='http://localhost:3000'

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
    
    # gazebo_launch_desc = launch.actions.ExecuteProcess(
    #     cmd=['gzserver', '--verbose', 'world', gazebo_world, 
    #         '-s', 'libgazebo_ros_init.so', 'libgazebo_ros_properties.so', 'libgazebo_ros_factory.so', 'libgazebo_ros_state.so'],
    #     output='screen'
    # )

    # Upload vehicle
    upload_launch = os.path.join(
        str(parent_file_path),
        'models',
        'default_fossen_vehicle',
        'test_upload_default_fossen_vehicle.launch.py'
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
            # Start tests right away - no need to wait for anything
            launch_testing.actions.ReadyToTest(),
        ])
    )
