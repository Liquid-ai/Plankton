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
from __future__ import print_function
import numpy
import rclpy
# import tf
# import tf.transformations as trans

from sensor_msgs.msg import Joy
from uuv_gazebo_ros_plugins_msgs.msg import FloatStamped
from uuv_thrusters.models import Thruster
from rclpy.numpy_msg import numpy_msg
from rclpy.node import Node


class FinnedUUVControllerNode(Node):
    def __init__(self):
        super.__init__('finned_uuv_teleop')

        print('FinnedUUVControllerNode: initializing node')
        
        self._ready = False

        # Test if any of the needed parameters are missing
        param_labels = ['n_fins', 'gain_roll', 'gain_pitch', 'gain_yaw',
                        'thruster_model', 'fin_topic_prefix',
                        'fin_topic_suffix', 'thruster_topic',
                        'axis_thruster', 'axis_roll', 'axis_pitch', 'axis_yaw']

        #TODO Remove ~ ?
        for label in param_labels:
            if not self.has_parameter('~%s' % label):
                raise rospy.ROSException('Parameter missing, label=%s' % label)

        # Number of fins
        self._n_fins = self.get_parameter('~n_fins').get_parameter_value().integer_value

        # Thruster joy axis gain
        self._thruster_joy_gain = 1
        if self.has_parameter('~thruster_joy_gain'):
            self._thruster_joy_gain = self.get_parameter('~thruster_joy_gain').get_parameter_value().double_value

        # Read the vector for contribution of each fin on the change on
        # orientation
        gain_roll = self.get_parameter('~gain_roll').get_parameter_value().double_array_value
        gain_pitch = self.get_parameter('~gain_pitch').get_parameter_value().double_array_value
        gain_yaw = self.get_parameter('~gain_yaw').get_parameter_value().double_array_value

        if len(gain_roll) != self._n_fins or len(gain_pitch) != self._n_fins \
            or len(gain_yaw) != self._n_fins:
            raise rclpy.exceptions.InvalidParameterValueException('Input gain vectors must have length '
                                     'equal to the number of fins')

        # Create the command angle to fin angle mapping
        self._rpy_to_fins = numpy.vstack((gain_roll, gain_pitch, gain_yaw)).T

        # Read the joystick mapping
        self._joy_axis = dict(axis_thruster=self.get_parameter('~axis_thruster').get_parameter_value().integer_value,
                              axis_roll=self.get_parameter('~axis_roll').get_parameter_value().integer_value,
                              axis_pitch=self.get_parameter('~axis_pitch').get_parameter_value().integer_value,
                              axis_yaw=self.get_parameter('~axis_yaw').get_parameter_value().integer_value)

        # Subscribe to the fin angle topics
        self._pub_cmd = list()
        self._fin_topic_prefix = self.get_parameter('~fin_topic_prefix').get_parameter_value().string_value
        self._fin_topic_suffix = self.get_parameter('~fin_topic_suffix').get_parameter_value().string_value
        for i in range(self._n_fins):
            topic = self._fin_topic_prefix + str(i) + self._fin_topic_suffix
            self._pub_cmd.append(
              self.create_publisher(FloatStamped, topic, 10))

        # Create the thruster model object
        try:
            self._thruster_topic = self.get_parameter('~thruster_topic').get_parameter_value().string_value
            self._thruster_params = self.get_parameter('~thruster_model').value
            if 'max_thrust' not in self._thruster_params:
                raise rclpy.exceptions.ParameterException('No limit to thruster output was given')
            self._thruster_model = Thruster.create_thruster(
                        self._thruster_params['name'], 0,
                        self._thruster_topic, None, None,
                        **self._thruster_params['params'])
        except:
            raise RuntimeError('Thruster model could not be initialized')
        
        # Subscribe to the joystick topic
        self.sub_joy = self.create_subscription(numpy_msg(Joy), 'joy',
                                        self.joy_callback)

        self._ready = True

    def joy_callback(self, msg):
        """Handle callbacks with joystick state."""

        if not self._ready:
            return

        try:
            thrust = max(0, msg.axes[self._joy_axis['axis_thruster']]) * \
                self._thruster_params['max_thrust'] * \
                self._thruster_joy_gain

            cmd_roll = msg.axes[self._joy_axis['axis_roll']]
            if abs(cmd_roll) < 0.2:
                cmd_roll = 0.0

            cmd_pitch = msg.axes[self._joy_axis['axis_pitch']]
            if abs(cmd_pitch) < 0.2:
                cmd_pitch = 0.0

            cmd_yaw = msg.axes[self._joy_axis['axis_yaw']]
            if abs(cmd_yaw) < 0.2:
                cmd_yaw = 0.0

            rpy = numpy.array([cmd_roll, cmd_pitch, cmd_yaw])
            fins = self._rpy_to_fins.dot(rpy)

            self._thruster_model.publish_command(thrust)

            for i in range(self._n_fins):
                cmd = FloatStamped()
                cmd.data = fins[i]
                self._pub_cmd[i].publish(cmd)

            if not self._ready:
                return
        except Exception as e:
            print('Error occurred while parsing joystick input, check '
                  'if the joy_id corresponds to the joystick ' 
                  'being used. message={}'.format(e))

def main(args=None):
    print('starting FinnedUUVControllerNode.py')
    #rospy.init_node('finned_uuv_teleop')
    rclpy.init(args=args)

    try:
        node = FinnedUUVControllerNode()
        rclpy.spin(node)
    except rospy.ROSInterruptException:
        print('caught exception')
    print('exiting')

if __name__ == '__main__':
    main()
