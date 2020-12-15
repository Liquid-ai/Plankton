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

import numpy
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
import geometry_msgs.msg as geometry_msgs
from nav_msgs.msg import Odometry

# Modules included in this package
from uuv_PID import PIDRegulator
import tf_quaternion.transformations as transf
from plankton_utils.time import time_in_float_sec_from_msg
from plankton_utils.time import is_sim_time


class VelocityControllerNode(Node):
    def __init__(self, name, **kwargs):
        print('VelocityControllerNode: initializing node')
        super().__init__(name, **kwargs)

        self.config = {}

        self.v_linear_des = numpy.zeros(3)
        self.v_angular_des = numpy.zeros(3)

        # Initialize pids with default parameters
        self.pid_angular = PIDRegulator(1, 0, 0, 1)
        self.pid_linear = PIDRegulator(1, 0, 0, 1)

        #Declared parameters are overriden with yaml values
        self._declare_and_fill_map("linear_p", 1., "p component of pid for linear vel.", self.config)
        self._declare_and_fill_map("linear_i", 0.0, "i component of pid for linear vel.", self.config)
        self._declare_and_fill_map("linear_d", 0., "d component of pid for linear vel.", self.config)
        self._declare_and_fill_map("linear_sat", 1., "saturation of pid for linear vel.", self.config)

        self._declare_and_fill_map("angular_p", 1., "p component of pid for angular vel.", self.config)
        self._declare_and_fill_map("angular_i", 0.0, "i component of pid for angular vel.", self.config)
        self._declare_and_fill_map("angular_d", 0.0, "d component of pid for angular vel.", self.config)
        self._declare_and_fill_map("angular_sat", 3.0, "saturation of pid for angular vel.", self.config)
        
        self._declare_and_fill_map(
            "odom_vel_in_world", True, "Is odometry velocity supplied in world frame? (gazebo)", self.config)
        
        self.set_parameters_callback(self.callback_params)
        
        self.create_pids(self.config)

        # ROS infrastructure
        self.sub_cmd_vel = self.create_subscription(geometry_msgs.Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        self.sub_odometry = self.create_subscription(Odometry, 'odom', self.odometry_callback, 10)
        self.pub_cmd_accel = self.create_publisher( geometry_msgs.Accel, 'cmd_accel', 10)

    #==============================================================================
    def cmd_vel_callback(self, msg):
        """Handle updated set velocity callback."""
        # Just store the desired velocity. The actual control runs on odometry callbacks
        v_l = msg.linear
        v_a = msg.angular
        self.v_linear_des = numpy.array([v_l.x, v_l.y, v_l.z])
        self.v_angular_des = numpy.array([v_a.x, v_a.y, v_a.z])

    #==============================================================================
    def odometry_callback(self, msg):
        """Handle updated measured velocity callback."""
        if not bool(self.config):
            return

        linear = msg.twist.twist.linear
        angular = msg.twist.twist.angular
        v_linear = numpy.array([linear.x, linear.y, linear.z])
        v_angular = numpy.array([angular.x, angular.y, angular.z])

        if self.config['odom_vel_in_world']:
            # This is a temp. workaround for gazebo's pos3d plugin not behaving properly:
            # Twist should be provided wrt child_frame, gazebo provides it wrt world frame
            # see http://docs.ros.org/api/nav_msgs/html/msg/Odometry.html
            xyzw_array = lambda o: numpy.array([o.x, o.y, o.z, o.w])
            q_wb = xyzw_array(msg.pose.pose.orientation)
            R_bw = transf.quaternion_matrix(q_wb)[0:3, 0:3].transpose()

            v_linear = R_bw.dot(v_linear)
            v_angular = R_bw.dot(v_angular)
        
        # Compute compute control output:
        t = time_in_float_sec_from_msg(msg.header.stamp)
        
        e_v_linear = (self.v_linear_des - v_linear)
        e_v_angular = (self.v_angular_des - v_angular)
        
        a_linear = self.pid_linear.regulate(e_v_linear, t)
        a_angular = self.pid_angular.regulate(e_v_angular, t)

        # Convert and publish accel. command:
        cmd_accel = geometry_msgs.Accel()
        cmd_accel.linear = geometry_msgs.Vector3(x=a_linear[0], y=a_linear[1], z=a_linear[2])
        cmd_accel.angular = geometry_msgs.Vector3(x=a_angular[0], y=a_angular[1], z=a_angular[2])
        self.pub_cmd_accel.publish(cmd_accel)

    #==============================================================================
    def callback_params(self, data):
        for parameter in data:
            self.config[parameter.name] = parameter.value
        
        # config has changed, reset PID controllers
        self.create_pids(self.config)

        self.get_logger().warn("Parameters dynamically changed...")
        return SetParametersResult(successful=True)

    #==============================================================================
    def create_pids(self, config):
        self.pid_linear = PIDRegulator(
            config['linear_p'], config['linear_i'], config['linear_d'], config['linear_sat'])
        self.pid_angular = PIDRegulator(
            config['angular_p'], config['angular_i'], config['angular_d'], config['angular_sat'])

    #==============================================================================
    def _declare_and_fill_map(self, key, default_value, description, map):
        param = self.declare_parameter(key, default_value, ParameterDescriptor(description=description))
        map[key] = param.value


#==============================================================================
def main():
    print('Starting VelocityControl.py')
    rclpy.init()

    try:
        sim_time_param = is_sim_time()

        node = VelocityControllerNode('velocity_control', parameter_overrides=[sim_time_param])
        rclpy.spin(node)
    except Exception as e:
        print('Caught exception: ' + str(e))
    finally:
        rclpy.shutdown()
    print('Exiting')


#==============================================================================
if __name__ == '__main__':
    main()
