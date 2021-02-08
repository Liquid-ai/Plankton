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

import math
import numpy
import rclpy

from rcl_interfaces.msg import ParameterDescriptor

from uuv_PID import PIDRegulator

import geometry_msgs.msg as geometry_msgs
from nav_msgs.msg import Odometry

import tf_quaternion.transformations as transf

from rcl_interfaces.msg import SetParametersResult
from rclpy.parameter import Parameter
from rclpy.node import Node

from plankton_utils.time import time_in_float_sec_from_msg
from plankton_utils.time import is_sim_time

class PositionControllerNode(Node):
    def __init__(self, name, **kwargs):
        super().__init__(name, **kwargs)
        self.get_logger().info('PositionControllerNode: initializing node')

        self.config = {}

        self.pos_des = numpy.zeros(3)
        self.quat_des = numpy.array([0, 0, 0, 1])

        self.initialized = False

        # Initialize pids with default parameters
        self.pid_rot = PIDRegulator(1, 0, 0, 1)
        self.pid_pos = PIDRegulator(1, 0, 0, 1)

        self._declare_and_fill_map("pos_p", 1., "p component of pid for position", self.config)
        self._declare_and_fill_map("pos_i", 0.0, "i component of pid for position.", self.config)
        self._declare_and_fill_map("pos_d", 0.0, "d component of pid for position.", self.config)
        self._declare_and_fill_map("pos_sat", 10.0, "saturation of pid for position.", self.config)

        self._declare_and_fill_map("rot_p", 1., "p component of pid for orientation.", self.config)
        self._declare_and_fill_map("rot_i", 0.0, "i component of pid for orientation.", self.config)
        self._declare_and_fill_map("rot_d", 0.0, "d component of pid for orientation.", self.config)
        self._declare_and_fill_map("rot_sat", 3.0, "saturation of pid for orientation.", self.config)

        self.set_parameters_callback(self.callback_params)

        self.create_pids(self.config)

        # ROS infrastructure
        self.sub_cmd_pose = self.create_subscription(geometry_msgs.PoseStamped, 'cmd_pose', self.cmd_pose_callback, 10)
        self.sub_odometry = self.create_subscription(Odometry, 'odom', self.odometry_callback, 10)
        
        self.pub_cmd_vel = self.create_publisher(geometry_msgs.Twist, 'cmd_vel', 10)        

    #==============================================================================
    def cmd_pose_callback(self, msg):
        """Handle updated set pose callback."""
        # Just store the desired pose. The actual control runs on odometry callbacks
        p = msg.pose.position
        q = msg.pose.orientation
        self.pos_des = numpy.array([p.x, p.y, p.z])
        self.quat_des = numpy.array([q.x, q.y, q.z, q.w])

    #==============================================================================
    def odometry_callback(self, msg):
        """Handle updated measured velocity callback."""
        if not bool(self.config):
            return

        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        p = numpy.array([p.x, p.y, p.z])
        q = numpy.array([q.x, q.y, q.z, q.w])

        if not self.initialized:
            # If this is the first callback: Store and hold latest pose.
            self.pos_des  = p
            self.quat_des = q
            self.initialized = True

        # Compute control output:
        t = time_in_float_sec_from_msg(msg.header.stamp)

        # Position error
        e_pos_world = self.pos_des - p
        e_pos_body = transf.quaternion_matrix(q).transpose()[0:3,0:3].dot(e_pos_world)

        # Error quaternion wrt body frame
        e_rot_quat = transf.quaternion_multiply(transf.quaternion_conjugate(q), self.quat_des)

        if numpy.linalg.norm(e_pos_world[0:2]) > 5.0:
            # special case if we are far away from goal:
            # ignore desired heading, look towards goal position
            heading = math.atan2(e_pos_world[1],e_pos_world[0])
            quat_des = numpy.array([0, 0, math.sin(0.5*heading), math.cos(0.5*heading)])
            e_rot_quat = transf.quaternion_multiply(transf.quaternion_conjugate(q), quat_des)
            
        # Error angles
        e_rot = numpy.array(transf.euler_from_quaternion(e_rot_quat))

        # print(e_pos_body.shape)
        # print(e_rot.shape)

        v_linear = self.pid_pos.regulate(e_pos_body, t)
        v_angular = self.pid_rot.regulate(e_rot, t)

        # Convert and publish vel. command:
        cmd_vel = geometry_msgs.Twist()
        cmd_vel.linear = geometry_msgs.Vector3(x=v_linear[0], y=v_linear[1], z=v_linear[2])
        cmd_vel.angular = geometry_msgs.Vector3(x=v_angular[0], y=v_angular[1], z=v_angular[2])
        self.pub_cmd_vel.publish(cmd_vel)

    #==============================================================================
    def callback_params(self, data):
        """Handle updated configuration values."""
        for parameter in data:
            #if parameter.name == "name":
            #if parameter.type_ == Parameter.Type.DOUBLE:
            self.config[parameter.name] = parameter.value

        # Config has changed, reset PID controllers
        self.create_pids(self.config)

        self.get_logger().warn("Parameters dynamically changed...")
        return SetParametersResult(successful=True)

    #==============================================================================
    def create_pids(self, config):
        self.pid_pos = PIDRegulator(config['pos_p'], config['pos_i'], config['pos_d'], config['pos_sat'])
        self.pid_rot = PIDRegulator(config['rot_p'], config['rot_i'], config['rot_d'], config['rot_sat'])

    #==============================================================================
    def _declare_and_fill_map(self, key, default_value, description, map):
        param = self.declare_parameter(key, default_value, ParameterDescriptor(description=description))
        map[key] = param.value

#==============================================================================
def main():
    print('Starting PositionControl.py')
    rclpy.init()

    try:
        sim_time_param = is_sim_time()

        node = PositionControllerNode('position_control', parameter_overrides=[sim_time_param])
        rclpy.spin(node)
    except Exception as e:
        print('Caught exception: ' + str(e))
    finally:
        if rclpy.ok():
            rclpy.shutdown()
        print('Exiting')

#==============================================================================
if __name__ == '__main__':
    main()
