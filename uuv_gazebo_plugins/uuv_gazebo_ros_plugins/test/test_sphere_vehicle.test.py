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

from geometry_msgs.msg import Vector3, Inertia
from uuv_gazebo_ros_plugins_msgs.msg import UnderwaterObjectModel
from uuv_gazebo_ros_plugins_msgs.srv import *

import launch
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
import launch_testing.actions

from ament_index_python.packages import get_package_share_directory


# Sphere model radius
RADIUS = 0.1
# Damping coefficient
CD = 0.5


class TestSphereVehicle(unittest.TestCase):
    # def tearDownClass(self):
        # FIXME Temporary solution to avoid gzserver lingering after the
        # simulation node is killed (Gazebo 9.1)
        # os.system('killall -9 gzserver')
    
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
            'test_sphere_vehicle',
            allow_undeclared_parameters=True, 
            automatically_declare_parameters_from_overrides=True
        )

    # =========================================================================
    def tearDown(self):
        self.node.destroy_node()

    # =========================================================================
    def create_service(self, srv_type, srv_name):
        s = self.node.create_client(srv_type, srv_name)
        
        if not s.wait_for_service(timeout_sec=10):
            self.fail('service %s not available...' % srv_name)

        return s

    # =========================================================================
    # @pytest.mark.skip()
    def service_request(self, service, **kwargs):#data=None):
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
        # rospy.wait_for_service('/vehicle/get_model_properties')

        # get_models = rospy.ServiceProxy('/vehicle/get_model_properties',
        #                                 GetModelProperties)

        s_models = self.create_service(GetModelProperties, '/vehicle/get_model_properties')

        models = self.service_request(s_models)
        # models = get_models()

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
        self.assertEqual(len(models.models[0].added_mass), 36)
        self.assertEqual(len(models.models[0].linear_damping), 36)
        self.assertEqual(len(models.models[0].linear_damping_forward_speed), 36)
        self.assertEqual(len(models.models[0].quadratic_damping), 36)

        # Tests if some of the parameters match to the ones given in the URDF
        self.assertEqual(models.models[0].fluid_density, 1028.0)
        self.assertEqual(models.models[0].volume, 0.009727626)
        self.assertEqual(models.models[0].bbox_height, 1.0)
        self.assertEqual(models.models[0].bbox_length, 1.0)
        self.assertEqual(models.models[0].bbox_width, 1.0)

    # =========================================================================
    def test_added_mass_coefs(self):
        # rospy.wait_for_service('/vehicle/get_model_properties')

        # get_models = rospy.ServiceProxy('/vehicle/get_model_properties',
        #                                 GetModelProperties)

        s_models = self.create_service(GetModelProperties, '/vehicle/get_model_properties')

        models = self.service_request(s_models)

        # models = get_models()

        d_idxs = [i*6 + j for i, j in zip(range(3), range(3))]
        sphere_ma = 2.0 / 3.0 * models.models[0].fluid_density * np.pi * \
            RADIUS**3.0
        for i in range(len(models.models[0].added_mass)):
            if i in d_idxs:
                self.assertLess(abs(models.models[0].added_mass[i] - sphere_ma), 0.001)
            else:
                self.assertEqual(models.models[0].added_mass[i], 0.0)

    # ==============================================================
    def test_nonlinear_damping_coefs(self):
        # rospy.wait_for_service('/vehicle/get_model_properties')

        # get_models = rospy.ServiceProxy('/vehicle/get_model_properties',
        #                                 GetModelProperties)

        s_models = self.create_service(GetModelProperties, '/vehicle/get_model_properties')

        models = self.service_request(s_models)

        # models = get_models()

        area_section = np.pi * RADIUS**2
        dq = -0.5 * models.models[0].fluid_density * CD * area_section

        d_idxs = [i*6 + j for i, j in zip(range(3), range(3))]
        for i in range(len(models.models[0].quadratic_damping)):
            if i in d_idxs:
                self.assertLess(abs(models.models[0].quadratic_damping[i] - dq), 0.001)
            else:
                self.assertEqual(models.models[0].quadratic_damping[i], 0.0)


# if __name__ == '__main__':
#     import rosunit
#     rosunit.unitrun(PKG, NAME, TestSphereVehicle)

# =============================================================================
@pytest.mark.rostest
def generate_test_description():
    # Set env
    os.environ['GAZEBO_MASTER_URI'] ='http://localhost:3001'

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
        
    launch_args = [('world', gazebo_world), ('paused', 'false'), ('gui', 'false'), ]
    gazebo_launch_desc = IncludeLaunchDescription(
            AnyLaunchDescriptionSource(gazebo_launch), launch_arguments=launch_args)

    # Upload vehicle
    upload_launch = os.path.join(
        str(parent_file_path),
        'models',
        'sphere_vehicle',
        'test_upload_sphere_vehicle.launch.py'
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