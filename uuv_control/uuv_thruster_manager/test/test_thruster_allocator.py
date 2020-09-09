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

from uuv_thruster_manager.srv import GetThrusterManagerConfig, ThrusterManagerInfo
from uuv_gazebo_ros_plugins_msgs.msg import FloatStamped

import launch
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch import LaunchDescription
import launch_testing.actions
import launch_ros
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory, get_package_prefix
import os
import pathlib
import xacro

import pytest
# Other imports


NS = 'test_vehicle'

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
        # Create a ROS node for tests
        self.node = rclpy.create_node('test_thrusters',
                        allow_undeclared_parameters=True, 
                        automatically_declare_parameters_from_overrides=True)


    # =========================================================================
    def tearDown(self):
        self.node.destroy_node()

    # =========================================================================
    # def test_services_exist(self):
    #     srvs = [
    #         'thruster_manager/get_thrusters_info',
    #         'thruster_manager/get_thruster_curve',
    #         'thruster_manager/set_config',
    #         'thruster_manager/get_config'
    #     ]

    #     for srv in srvs:
    #         rospy.wait_for_service('/{}/{}'.format(NS, srv), timeout=1000)

    # =========================================================================
    def test_config(self):
        axis = self.node.get_parameter('axis')
        # ref_config = rospy.get_param('/{}/thruster_manager'.format(NS))
        
        srv_name = '/{}/thruster_manager/get_config'.format(NS)
        cli = self.node.create_client(GetThrusterManagerConfig, srv_name)
        
        if not cli.wait_for_service(timeout_sec=10):
            self.fail('service %s not available...' % srv_name)
        
        # srv = rospy.ServiceProxy('/{}/thruster_manager/get_config'.format(NS), GetThrusterManagerConfig)
        # tm_config = srv()

        req = GetThrusterManagerConfig.Request()
        future = cli.call_async(req)

        while future.done():
            rclpy.spin_once(self.node)
        
        tm_config = future.result()
        
        self.assertEqual(tm_config.tf_prefix, '/test_vehicle/')
        self.assertEqual(tm_config.base_link, 'base_link')
        self.assertEqual(tm_config.thruster_frame_base, 'thruster_')
        self.assertEqual(tm_config.thruster_topic_suffix, '/input')
        self.assertEqual(tm_config.timeout, -1)
        self.assertEqual(tm_config.max_thrust, 1000.0)
        self.assertEqual(tm_config.n_thrusters, 3)

        if axis == 'x':
            tam_flat = AXIS_X_TAM.flatten()
        elif axis == 'y':
            tam_flat = AXIS_Y_TAM.flatten()
        elif axis == 'z':
            tam_flat = AXIS_Z_TAM.flatten()

        self.assertEqual(len(tm_config.allocation_matrix), tam_flat.size)
        for x, y in zip(tam_flat, tm_config.allocation_matrix):
            self.assertAlmostEqual(x, y)


# if __name__ == '__main__':
#     import rosunit
#     rosunit.unitrun(PKG, 'test_thruster_allocator', TestThrusterAllocator)
    









    # <rosparam param="axis">"x"</rosparam>

    # <group ns="test_vehicle">
    #     <param name="robot_description"
    #         command="$(find xacro)/xacro '$(find uuv_thruster_manager)/test/test_vehicle_x_axis.urdf.xacro' --inorder"/>

    #     <rosparam 
    #         command="load"
    #         file="$(find uuv_thruster_manager)/test/test_vehicle_thruster_manager_proportional.yaml"/>                
    # </group>    

    # <include file="$(find uuv_thruster_manager)/launch/thruster_manager.launch">
    #     <arg name="model_name" value="test_vehicle"/>
    #     <arg name="output_dir" value="/tmp" />
    #     <arg name="config_file" value="$(find uuv_thruster_manager)/test/test_vehicle_thruster_manager_proportional.yaml" />        
    #     <arg name="reset_tam" value="true"/>
    # </include>

    # <node name="joint_state_publisher" pkg="joint_state_publisher" type="joint_state_publisher" ns="test_vehicle"/>

    # <node name="robot_state_publisher" 
    #     pkg="robot_state_publisher" 
    #     type="robot_state_publisher" 
    #     respawn="true" 
    #     output="screen"
    #     ns="test_vehicle">
    #     <param name="robot_description" value="/test_vehicle/robot_description" />
    #     <param name="publish_frequency" value="5" />
    # </node>


def open_output(output_filename):
    if output_filename is None:
        return sys.stdout
    else:
        dir_name = os.path.dirname(output_filename)
        if dir_name:
            try:
                os.makedirs(dir_name)
            except os.error:
                # errors occur when dir_name exists or creation failed
                # ignore error here; opening of file will fail if directory is still missing
                pass

        try:
            return open(output_filename, 'w')
        except IOError as e:
            print("Failed to open output:", exc=e)


@pytest.mark.rostest
def generate_test_description():
    # Normally, talker publishes on the 'chatter' topic and listener listens on the
    # 'chatter' topic, but we want to show how to use remappings to munge the data so we
    # will remap these topics when we launch the nodes and insert our own node that can
    # change the data as it passes through
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

    launch_args = [('axis', 'x'), ('model_name', 'test_vehicle'), ('output_dir', '/tmp'), ('config_file', thruster_manager_yaml), ('reset_tam', 'true')]
    thruster_manager_launch_desc = IncludeLaunchDescription(
            AnyLaunchDescriptionSource(thruster_manager_launch), launch_arguments=launch_args)
    
    # thruster_manager_launch_desc = launch_ros.actions.Node(
    #     node_namespace = "test_vehicle",
    #     package="uuv_thruster_manager",
    #     node_executable="thruster_allocator.py",
    #     node_name="joint_state_publisher",
    #     parameters=[{'axis': 'x', 'model_name': 'test_vehicle', 'output_dir': '/tmp', 'config_file': thruster_manager_yaml, 'reset_tam': 'false'}]
    # )
    
    # config = os.path.join(
    #     get_package_share_directory('uuv_thruster_manager'),
    #     'config',
    #     'test_vehicle_thruster_manager_proportional.yaml'
    # )

    #doc = xacro.process_file(xacro_file, mappings={'simulate_obstacles' : 'false'})

    joint_state_publisher = launch_ros.actions.Node(
        node_namespace = "test_vehicle",
        package="joint_state_publisher",
        node_executable="joint_state_publisher",
        node_name="joint_state_publisher",
        #parameters=[config]
    )

    xacro_file = os.path.join(
        str(parent_file_path),
        'test_vehicle_x_axis.urdf.xacro'
    ) #os.path.join(
    #     str(parent_file_path),
    #     'test',
    #     'test_vehicle_x_axis.urdf.xacro'
    # )

    output = os.path.join(
        str(parent_file_path),
        'robot_description.urdf'
    )

    doc = xacro.process(xacro_file)
    file_out = open_output(output)
    file_out.write(doc)
    file_out.close()
    #print(doc)

    robot_state_description = launch_ros.actions.Node(
        node_namespace = "test_vehicle",
        package='robot_state_publisher',
        node_executable='robot_state_publisher',
        #parameters=[{'robot_description', doc}]
        # To replace in foxy with parameters=[{'robot_description', Command('ros2 run xacro...')}]
        arguments=[output]
    )



    # listener_node = launch_ros.actions.Node(
    #     executable=sys.executable,
    #     arguments=[os.path.join(path_to_test, 'listener.py')],
    #     additional_env={'PYTHONUNBUFFERED': '1'},
    #     remappings=[('chatter', 'listener_chatter')]
    # )

    return (
        launch.LaunchDescription([
            thruster_manager_launch_desc,
            joint_state_publisher,
            robot_state_description,
            # Start tests right away - no need to wait for anything
            launch_testing.actions.ReadyToTest(),
        ])
        # {
        #     'talker': talker_node,
        #     'listener': listener_node,
        # }
    )


# class TestTalkerListenerLink(unittest.TestCase):

#     @classmethod
#     def setUpClass(cls):
#         # Initialize the ROS context for the test node
#         rclpy.init()

#     @classmethod
#     def tearDownClass(cls):
#         # Shutdown the ROS context
#         rclpy.shutdown()

#     def setUp(self):
#         # Create a ROS node for tests
#         self.node = rclpy.create_node('test_talker_listener_link')

#     def tearDown(self):
#         self.node.destroy_node()

#     def test_talker_transmits(self, launch_service, talker, proc_output):
#         # Expect the talker to publish strings on '/talker_chatter' and also write to stdout
#         msgs_rx = []

#         sub = self.node.create_subscription(
#             std_msgs.msg.String,
#             'talker_chatter',
#             lambda msg: msgs_rx.append(msg),
#             10
#         )
#         try:
#             # Wait until the talker transmits two messages over the ROS topic
#             end_time = time.time() + 10
#             while time.time() < end_time:
#                 rclpy.spin_once(self.node, timeout_sec=0.1)
#                 if len(msgs_rx) > 2:
#                     break

#             self.assertGreater(len(msgs_rx), 2)

#             # Make sure the talker also output the same data via stdout
#             for msg in msgs_rx:
#                 proc_output.assertWaitFor(
#                     expected_output=msg.data, process=talker
#                 )
#         finally:
#             self.node.destroy_subscription(sub)

#     def test_listener_receives(self, launch_service, listener, proc_output):
#         pub = self.node.create_publisher(
#             std_msgs.msg.String,
#             'listener_chatter',
#             10
#         )
#         try:
#             # Publish a unique message on /chatter and verify that the listener
#             # gets it and prints it
#             msg = std_msgs.msg.String()
#             msg.data = str(uuid.uuid4())
#             for _ in range(10):
#                 pub.publish(msg)
#                 success = proc_output.waitFor(
#                     expected_output=msg.data,
#                     process=listener,
#                     timeout=1.0,
#                 )
#                 if success:
#                     break
#             assert success, 'Waiting for output timed out'
#         finally:
#             self.node.destroy_publisher(pub)

#     def test_fuzzy_data(self, launch_service, listener, proc_output):
#         # This test shows how to insert a node in between the talker and the listener to
#         # change the data.  Here we're going to change 'Hello World' to 'Aloha World'
#         def data_mangler(msg):
#             msg.data = msg.data.replace('Hello', 'Aloha')
#             return msg

#         republisher = launch_testing_ros.DataRepublisher(
#             self.node,
#             'talker_chatter',
#             'listener_chatter',
#             std_msgs.msg.String,
#             data_mangler
#         )
#         try:
#             # Spin for a few seconds until we've republished some mangled messages
#             end_time = time.time() + 10
#             while time.time() < end_time:
#                 rclpy.spin_once(self.node, timeout_sec=0.1)
#                 if republisher.get_num_republished() > 2:
#                     break

#             self.assertGreater(republisher.get_num_republished(), 2)

#             # Sanity check that we're changing 'Hello World'
#             proc_output.assertWaitFor('Aloha World')

#             # Check for the actual messages we sent
#             for msg in republisher.get_republished():
#                 proc_output.assertWaitFor(msg.data, listener)
#         finally:
#             republisher.shutdown()