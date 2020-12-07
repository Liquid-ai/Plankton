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
# import tf
# import tf.transformations as trans
from PIDRegulator import PIDRegulator

from dynamic_reconfigure.server import Server
from uuv_control_cascaded_pid.cfg import PositionControlConfig
import geometry_msgs.msg as geometry_msgs
from nav_msgs.msg import Odometry

#TODO numpy_msg...
from rospy.numpy_msg import numpy_msg

from rclpy.node import Node

#It seems that the file is not used...

class PositionControllerNode(Node):
    def __init__(self, name):
        print('PositionControllerNode: initializing node')

        super().__init__(name)
        self.config = {}

        self.pos_des = numpy.zeros(3)
        self.quat_des = numpy.array([0, 0, 0, 1])

        self.initialized = False

        # Initialize pids with default parameters
        self.pid_depth = PIDRegulator(1, 0, 0, 1)
        self.pid_heading = PIDRegulator(1, 0, 0, 1)
        self.pid_forward = PIDRegulator(1, 0, 0, 1)

        # ROS infrastructure
        #self.listener = tf.TransformListener()
        self.sub_cmd_pose = self.create_subscription(numpy_msg(geometry_msgs.PoseStamped), 'cmd_pose', self.cmd_pose_callback, 10)
        self.sub_odometry = self.create_subscription(numpy_msg(Odometry), 'odom', self.odometry_callback, 10)
        self.pub_cmd_vel = self.create_publisher(geometry_msgs.Twist,'cmd_vel', 10)
        self.srv_reconfigure = Server(PositionControlConfig, self.config_callback)

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
        t = msg.header.stamp.to_sec()

        # Position error wrt. body frame
        e_pos = trans.quaternion_matrix(q).transpose()[0:3,0:3].dot(self.pos_des - p)

        vz = self.pid_depth.regulate(e_pos[2], t)
        vx = self.pid_forward.regulate(numpy.linalg.norm(e_pos[0:2]), t)
        wx = self.pid_heading.regulate(numpy.arctan2(), t)

        v_linear = numpy.array([vx, 0, vz])
        v_angular = numpy.array([0, 0, wz])

        # Convert and publish vel. command:
        cmd_vel = geometry_msgs.Twist()
        cmd_vel.linear = geometry_msgs.Vector3(*v_linear)
        cmd_vel.angular = geometry_msgs.Vector3(*v_angular)

        self.pub_cmd_vel.publish(cmd_vel)

    def config_callback(self, config, level):
        """Handle updated configuration values."""
        # Config has changed, reset PID controllers
        self.pid_forward = PIDRegulator(config['forward_p'], config['forward_i'], config['forward_d'], config['forward_sat'])
        self.pid_depth = PIDRegulator(config['depth_p'], config['depth_i'], config['depth_d'], config['depth_sat'])
        self.pid_heading = PIDRegulator(config['heading_p'], config['heading_i'], config['heading_d'], config['heading_sat'])

        self.config = config

        return config


if __name__ == '__main__':
    print('starting PositionControl.py')
    rospy.init_node('position_control')

    try:
        node = PositionControllerNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        print('caught exception')
    print('exiting')
