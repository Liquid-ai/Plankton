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
import unittest
import numpy as np
import rclpy
from uuv_thrusters import ThrusterManager

import launch
from launch import LaunchDescription
import launch_testing.actions
import launch_ros
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import pathlib
import xacro
import pytest
import threading
import time


REFERENCE_TAM = np.array([
    [1, 0, 0, 0, 0, 0],
    [0.87758256, 0, -0.47942554, 0.47942554, 0.47942554, 0.87758256],
    [0.87758256, 0.47942554, 0, -0.47942554, 0.87758256, -0.87758256]
]).T


def ensure_path_exists(full_path):
    dir_name = os.path.dirname(full_path)
    if dir_name:
        try:
            os.makedirs(dir_name)
        except OSError:
            print ("Creation of the directory %s failed" % dir_name)


class TestThrusterManagerProportionalCorrect(unittest.TestCase):
    _manager = None
    _thread = None
    
    @classmethod
    def setUpClass(cls):
        parent_file_path = pathlib.Path(__file__).parent 
        thruster_manager_yaml = os.path.join(
            str(parent_file_path),
            'test_vehicle_thruster_manager_proportional.yaml'
        )

        xacro_file = os.path.join(
        str(parent_file_path),
        'test_vehicle_x_axis.urdf.xacro'
        )
  
        doc = xacro.process(xacro_file)
           
        # Initialize the ROS context for the test node
        # FIXME Temp workaround TF for listener subscribing to relative topic
        rclpy.init(args=['--ros-args', '--params-file', thruster_manager_yaml, 
            '-r', '__ns:=/test_vehicle', '-r', 'tf:=/tf', '-p', 'robot_description:=%s' % doc])

        # Name alias...why cls is not working ?
        _class = TestThrusterManagerProportionalCorrect
        
        _class._manager = ThrusterManager('test_thruster_manager_proportional_correct')
        _class._thread = threading.Thread(target=rclpy.spin, args=(_class._manager,), daemon=True)
        _class._thread.start()

        # Wait for the async initialization of the manager to finish
        while not _class._manager.ready:
            time.sleep(0.1)

    # =========================================================================
    @classmethod
    def tearDownClass(cls):
        _class = TestThrusterManagerProportionalCorrect

        _class._manager.destroy_node()
        # Shutdown the ROS context
        rclpy.shutdown()
        _class._thread.join()

    # =========================================================================
    # def setUp(self):
        
    # # =========================================================================
    # def tearDown(self):
    #     # self.MANAGER.destroy_node()
    #     # self.thread.join()

    # =========================================================================
    def test_initialization(self):    
        _class = TestThrusterManagerProportionalCorrect
        self.assertEqual(_class._manager.namespace, '/test_vehicle/')

        self.assertEqual(_class._manager.config['tf_prefix'].value, 'test_vehicle')
        self.assertEqual(_class._manager.config['base_link'].value, 'base_link')
        self.assertEqual(_class._manager.config['thruster_topic_prefix'].value, 'thrusters/')        
        self.assertEqual(_class._manager.config['thruster_frame_base'].value, 'thruster_')
        self.assertEqual(_class._manager.config['thruster_topic_suffix'].value, '/input')
        self.assertEqual(_class._manager.config['timeout'].value, -1)
        self.assertEqual(_class._manager.config['max_thrust'].value, 1000.0)
        self.assertEqual(_class._manager.config['update_rate'].value, 50)

        self.assertEqual(_class._manager.n_thrusters, 3)

        self.assertEqual(REFERENCE_TAM.shape, _class._manager.configuration_matrix.shape)        

        self.assertTrue(np.isclose(REFERENCE_TAM, _class._manager.configuration_matrix).all())

    # =========================================================================
    def test_thrusters(self):
        _class = TestThrusterManagerProportionalCorrect

        self.assertEqual(len(_class._manager.thrusters), 3)

        for i in range(len(_class._manager.thrusters)):
            self.assertEqual(_class._manager.thrusters[i].index, i)
            self.assertEqual(_class._manager.thrusters[i].topic, 'thrusters/id_{}/input'.format(i))
            self.assertEqual(_class._manager.thrusters[i].LABEL, 'proportional')
            self.assertTrue(np.isclose(REFERENCE_TAM[:, i].flatten(), _class._manager.thrusters[i].tam_column).all())

    # =========================================================================
    def test_processing_gen_forces(self):
        _class = TestThrusterManagerProportionalCorrect

        for _ in range(10):
            gen_force = np.random.rand(6) * 100
            thrust_forces = _class._manager.compute_thruster_forces(gen_force)

            ref_thrust_forces = np.linalg.pinv(REFERENCE_TAM).dot(gen_force)

            self.assertTrue(np.isclose(ref_thrust_forces, thrust_forces).all())


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
        'test_vehicle_x_axis.urdf.xacro'
    )

    doc = xacro.process(xacro_file)

    joint_state_publisher = launch_ros.actions.Node(
        namespace = 'test_vehicle',
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        output='screen',
        parameters=[{'use_sim_time':False}, {'rate': 100}],
    )

    robot_state_description = launch_ros.actions.Node(
        namespace = 'test_vehicle',
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time':False}, {'robot_description': doc}], # Use subst here
    )

    return (
        launch.LaunchDescription([           
            joint_state_publisher,
            robot_state_description,
            launch_testing.actions.ReadyToTest(),
        ])
    )
