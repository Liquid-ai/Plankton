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

import math
import numpy
import rclpy
#import tf.transformations as trans
from PID import PIDRegulator

#from dynamic_reconfigure.server import Server
#from uuv_control_cascaded_pid.cfg import PositionControlConfig
import geometry_msgs.msg as geometry_msgs
from nav_msgs.msg import Odometry
#TODO numpy msg...
from rospy.numpy_msg import numpy_msg

from rcl_interfaces.msg import SetParametersResult
from rclpy.parameter import Parameter
from rclpy.node import Node

def time_in_float_sec(time: Time):
    f_time = time.seconds_nanoseconds[0] + time.seconds_nanoseconds[1] / 1e9
    return f_time

class PositionControllerNode(Node):
    def __init__(self, node_name):
        super().__init__(node_name)
        print('PositionControllerNode: initializing node')

        self.config = {}

        self.pos_des = numpy.zeros(3)
        self.quat_des = numpy.array([0, 0, 0, 1])

        self.initialized = False

        # Initialize pids with default parameters
        self.pid_rot = PIDRegulator(1, 0, 0, 1)
        self.pid_pos = PIDRegulator(1, 0, 0, 1)

        self.declare_parameter("pos_p", 1.)
        self.declare_parameter("pos_i", 0.0)
        self.declare_parameter("pos_d", 0.0)
        self.declare_parameter("pos_sat", 10.0)

        self.declare_parameter("rot_p", 1.)
        self.declare_parameter("rot_i", 0.0)
        self.declare_parameter("rot_d", 0.0)
        self.declare_parameter("rot_sat", 3.0)

        # ROS infrastructure
        self.sub_cmd_pose = self.create_subscription(numpy_msg(geometry_msgs.PoseStamped), 'cmd_pose', self.cmd_pose_callback, 10)
        self.sub_odometry = self.create_subscription(numpy_msg(Odometry), 'odom', self.odometry_callback, 10)
        self.pub_cmd_vel = self.create_publisher(geometry_msgs.Twist, 'cmd_vel', 10)
        #self.srv_reconfigure = Server(PositionControlConfig, self.config_callback)
        self.add_on_set_parameters_callback(self.cb_params)

    def cmd_pose_callback(self, msg):
        """Handle updated set pose callback."""
        # Just store the desired pose. The actual control runs on odometry callbacks
        p = msg.pose.position
        q = msg.pose.orientation
        self.pos_des = numpy.array([p.x, p.y, p.z])
        self.quat_des = numpy.array([q.x, q.y, q.z, q.w])

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
        t = time_in_float_sec(msg.header.stamp)

        # Position error
        e_pos_world = self.pos_des - p
        e_pos_body = trans.quaternion_matrix(q).transpose()[0:3,0:3].dot(e_pos_world)

        # Error quaternion wrt body frame
        e_rot_quat = trans.quaternion_multiply(trans.quaternion_conjugate(q), self.quat_des)

        if numpy.linalg.norm(e_pos_world[0:2]) > 5.0:
            # special case if we are far away from goal:
            # ignore desired heading, look towards goal position
            heading = math.atan2(e_pos_world[1],e_pos_world[0])
            quat_des = numpy.array([0, 0, math.sin(0.5*heading), math.cos(0.5*heading)])
            e_rot_quat = trans.quaternion_multiply(trans.quaternion_conjugate(q), quat_des)

        # Error angles
        e_rot = numpy.array(trans.euler_from_quaternion(e_rot_quat))

        v_linear = self.pid_pos.regulate(e_pos_body, t)
        v_angular = self.pid_rot.regulate(e_rot, t)

        # Convert and publish vel. command:
        cmd_vel = geometry_msgs.Twist()
        cmd_vel.linear = geometry_msgs.Vector3(*v_linear)
        cmd_vel.angular = geometry_msgs.Vector3(*v_angular)
        self.pub_cmd_vel.publish(cmd_vel)

    def config_callback(self, config, level):
        """Handle updated configuration values."""
        # Config has changed, reset PID controllers
        self.pid_pos = PIDRegulator(config['pos_p'], config['pos_i'], config['pos_d'], config['pos_sat'])
        self.pid_rot = PIDRegulator(config['rot_p'], config['rot_i'], config['rot_d'], config['rot_sat'])

        self.config = config

        return config


   def cb_params(self, data):
        for parameter in data:
            #if parameter.name == "name":
            if parameter.type_ == Parameter.Type.DOUBLE:
                self.config[parameter.name] = parameter.value

        self.get_logger().warn("Parameters dynamically changed...")
        return SetParametersResult(successful=True)

def main():
    print('Starting PositionControl.py')
    rclpy.init()

    try:
        node = PositionControllerNode('position_control')
        rclpy.spin(node)
    except Exception as e:
        print('Caught exception: ' + str(e))
    print('Exiting')

if __name__ == '__main__':
    main()
