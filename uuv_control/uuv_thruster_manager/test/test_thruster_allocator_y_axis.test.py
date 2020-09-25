#!/usr/bin/env python3
# Copyright (c) 2016-2019 The UUV Simulator Authors.
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
import numpy as np
import unittest
import time

import rclpy
from geometry_msgs.msg import WrenchStamped

from uuv_thruster_manager.srv import GetThrusterManagerConfig
from uuv_thruster_manager.srv import ThrusterManagerInfo
from uuv_thruster_manager.srv import GetThrusterCurve
from uuv_thruster_manager.srv import SetThrusterManagerConfig

from uuv_gazebo_ros_plugins_msgs.msg import FloatStamped

import launch
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch import LaunchDescription
import launch_testing.actions
import launch_ros
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import pathlib
import xacro

import pytest
# Other imports


NS = 'test_vehicle'

AXIS = 'y'

AXIS_X_TAM = np.array([
    [1, 0, 0, 0, 0, 0],
    [0.87758256, 0, -0.47942554, 0.47942554, 0.47942554, 0.87758256],
    [0.87758256, 0.47942554, 0, -0.47942554, 0.87758256, -0.87758256]
]).T

AXIS_Y_TAM = np.array([
    [0, 0.87758256, 0.47942554, 0, 0.47942554, -0.87758256],
    [0, 1, 0, 0, 0, 1],
    [-0.47942554, 0.87758256, 0, -0.87758256, -0.47942554, 0.47942554]
]).T

AXIS_Z_TAM = np.array([
    [0, -0.47942554, 0.87758256, 0, 0.87758256, 0.47942554],
    [0.47942554, 0, 0.87758256, -0.87758256, -0.87758256, 0.47942554],
    [0., 0., 1., 1., 0., 0.]
]).T


def ensure_path_exists(full_path):
    dir_name = os.path.dirname(full_path)
    if dir_name:
        try:
            os.makedirs(dir_name)
        except OSError:
            print ("Creation of the directory %s failed" % dir_name)


class TestThrusterAllocator(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        # Initialize the ROS context for the test node
        rclpy.init()

    # =========================================================================
    @classmethod
    def tearDownClass(cls):
        # Shutdown the ROS context
        rclpy.shutdown()

    # =========================================================================
    def setUp(self):
        # Create a ROS node for tests (node name in isolated env does not matter)
        self.node = rclpy.create_node('test_thrusters_allocator',
                        allow_undeclared_parameters=True, 
                        automatically_declare_parameters_from_overrides=True)

    # =========================================================================
    def tearDown(self):
        self.node.destroy_node()

    # =========================================================================
    def test_services_exist(self):
        services = list()
        services.append(self.node.create_client(
            ThrusterManagerInfo, '/{}/thruster_manager/get_thrusters_info'.format(NS)
        ))
        services.append(self.node.create_client(
            GetThrusterCurve, '/{}/thruster_manager/get_thruster_curve'.format(NS)
        ))
        services.append(self.node.create_client(
            SetThrusterManagerConfig, '/{}/thruster_manager/set_config'.format(NS)
        ))
        services.append(self.node.create_client(
            GetThrusterManagerConfig, '/{}/thruster_manager/get_config'.format(NS)
        ))

        for srv in services:
            self.assertTrue(srv.wait_for_service(timeout_sec=20), 'service %s not ready' % srv.srv_name)

    # =========================================================================
    def test_config(self):
        #axis = self.node.get_parameter('axis').value
        # ref_config = rospy.get_param('/{}/thruster_manager'.format(NS))
        
        srv_name = '/{}/thruster_manager/get_config'.format(NS)
        cli = self.node.create_client(GetThrusterManagerConfig, srv_name)
        
        if not cli.wait_for_service(timeout_sec=30):
            self.fail('service %s not available...' % srv_name)
        
        # srv = rospy.ServiceProxy('/{}/thruster_manager/get_config'.format(NS), GetThrusterManagerConfig)
        # tm_config = srv()

        req = GetThrusterManagerConfig.Request()
        future = cli.call_async(req)
        
        rclpy.spin_until_future_complete(self.node, future)
        
        tm_config = future.result()
        
        self.assertEqual(tm_config.tf_prefix, '/test_vehicle/')
        self.assertEqual(tm_config.base_link, 'base_link')
        self.assertEqual(tm_config.thruster_frame_base, 'thruster_')
        self.assertEqual(tm_config.thruster_topic_suffix, '/input')
        self.assertEqual(tm_config.timeout, -1)
        self.assertEqual(tm_config.max_thrust, 1000.0)
        self.assertEqual(tm_config.n_thrusters, 3)
        
        if AXIS == 'x':
            tam_flat = AXIS_X_TAM.flatten()
        elif AXIS == 'y':
            tam_flat = AXIS_Y_TAM.flatten()
        elif AXIS == 'z':
            tam_flat = AXIS_Z_TAM.flatten()

        self.assertEqual(len(tm_config.allocation_matrix), tam_flat.size, tam_flat)
        for x, y in zip(tam_flat, tm_config.allocation_matrix):
            self.assertAlmostEqual(x, y)


# =============================================================================
@pytest.mark.rostest
def generate_test_description():
    #path_to_test = os.path.dirname(__file__)
    file_path = pathlib.Path(__file__)
    # Here, parent first removes the file name
    parent_file_path = pathlib.Path(__file__).parent 

    thruster_manager_launch = os.path.join(
        get_package_share_directory('uuv_thruster_manager'),
        'launch',
        'thruster_manager.launch'
    )

    thruster_manager_yaml = os.path.join(
        str(parent_file_path),
        'test_vehicle_thruster_manager_proportional.yaml'
    )

    xacro_file = os.path.join(
        str(parent_file_path),
        'test_vehicle_y_axis.urdf.xacro'
    )

    output = os.path.join(
        str(parent_file_path),
        'robot_description_y_axis.urdf'
    )

    doc = xacro.process(xacro_file)
    ensure_path_exists(output)
    try:
        with open(output, 'w') as file_out:
            file_out.write(doc)
    except IOError as e:
        print("Failed to open output: ", exc=e)

    args = output.split()

    # ('axis', 'x')
    launch_args = [('model_name', 'test_vehicle'), 
        ('output_dir', '/tmp'), ('config_file', thruster_manager_yaml), ('reset_tam', 'true'), ('urdf_file', output)]
    thruster_manager_launch_desc = IncludeLaunchDescription(
            AnyLaunchDescriptionSource(thruster_manager_launch), launch_arguments=launch_args)

    joint_state_publisher = launch_ros.actions.Node(
        node_namespace = 'test_vehicle',
        package="joint_state_publisher",
        node_executable="joint_state_publisher",
        node_name="joint_state_publisher",
        #parameters=[{'source_list':'test_vehicle/joint_states'}],
        #remappings=[('joint_states', '/test_vehicle/joint_states')],
        arguments=args,
        output='screen',
        parameters=[{'use_sim_time':False}, {'rate': 100}],
    )

    robot_state_description = launch_ros.actions.Node(
        node_namespace = 'test_vehicle',
        package='robot_state_publisher',
        node_executable='robot_state_publisher',
        #parameters=[{'robot_description', doc}]
        # TODO To replace in foxy with parameters=[{'robot_description', Command('ros2 run xacro...')}]
        arguments=args,
        output='screen',
        parameters=[{'use_sim_time':False}], # Use subst here
    )

    return (
        launch.LaunchDescription([           
            joint_state_publisher,
            robot_state_description,
            thruster_manager_launch_desc,
            # Start tests right away - no need to wait for anything
            launch_testing.actions.ReadyToTest(),
        ])
    )