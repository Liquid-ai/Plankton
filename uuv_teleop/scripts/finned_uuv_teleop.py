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
from __future__ import print_function
import numpy
import rclpy
# import tf
# import tf.transformations as trans

from sensor_msgs.msg import Joy
from uuv_gazebo_ros_plugins_msgs.msg import FloatStamped
from uuv_thrusters.models import Thruster

from plankton_utils.param_helper import parse_nested_params_to_dict

from rclpy.node import Node


class FinnedUUVControllerNode(Node):
    def __init__(self):
        super().__init__('finned_uuv_teleop',
                      allow_undeclared_parameters=True, 
                      automatically_declare_parameters_from_overrides=True)

        #Default sim_time to True
        sim_time = rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)
        self.set_parameters([sim_time])

        self.get_logger().info('FinnedUUVControllerNode: initializing node')
        
        self._ready = False

        # Test if any of the needed parameters are missing
        param_labels = ['n_fins', 'gain_roll', 'gain_pitch', 'gain_yaw',
                        'thruster_model.max_thrust', 'thruster_model.name',
                        'fin_topic_prefix',
                        'fin_topic_suffix', 'thruster_topic',
                        'axis_thruster', 'axis_roll', 'axis_pitch', 'axis_yaw']

        for label in param_labels:
            if not self.has_parameter('%s' % label):
                raise Exception('Parameter missing, label=%s' % label)

        # Number of fins
        self._n_fins = self.get_parameter('n_fins').get_parameter_value().integer_value

        # Thruster joy axis gain
        self._thruster_joy_gain = 1
        if self.has_parameter('thruster_joy_gain'):
            self._thruster_joy_gain = self.get_parameter('thruster_joy_gain').value

        # Read the vector for contribution of each fin on the change on
        # orientation
        gain_roll = self.get_parameter('gain_roll').value
        gain_pitch = self.get_parameter('gain_pitch').value
        gain_yaw = self.get_parameter('gain_yaw').value

        if len(gain_roll) != self._n_fins or len(gain_pitch) != self._n_fins \
            or len(gain_yaw) != self._n_fins:
            raise Exception('Input gain vectors must have length '
                                     'equal to the number of fins')

        # Create the command angle to fin angle mapping
        self._rpy_to_fins = numpy.vstack((gain_roll, gain_pitch, gain_yaw)).T

        # Read the joystick mapping
        self._joy_axis = dict(axis_thruster=self.get_parameter('axis_thruster').get_parameter_value().integer_value,
                              axis_roll=self.get_parameter('axis_roll').get_parameter_value().integer_value,
                              axis_pitch=self.get_parameter('axis_pitch').get_parameter_value().integer_value,
                              axis_yaw=self.get_parameter('axis_yaw').get_parameter_value().integer_value)

        # Subscribe to the fin angle topics
        self._pub_cmd = list()
        self._fin_topic_prefix = self.get_parameter('fin_topic_prefix').get_parameter_value().string_value
        self._fin_topic_suffix = self.get_parameter('fin_topic_suffix').get_parameter_value().string_value
        for i in range(self._n_fins):
            topic = self.build_topic_name(self._fin_topic_prefix, i, self._fin_topic_suffix)
            #topic = self._fin_topic_prefix + str(i) + self._fin_topic_suffix
            self._pub_cmd.append(self.create_publisher(FloatStamped, topic, 10))

        # Create the thruster model object
        try:
            self._thruster_topic = self.get_parameter('thruster_topic').get_parameter_value().string_value
            self._thruster_params = self.get_parameters_by_prefix('thruster_model')
            self._thruster_params = parse_nested_params_to_dict(self._thruster_params, '.')
            
            if 'params' not in self._thruster_params:
                raise Exception('No thruster params given')
            #Should not happen
            if 'max_thrust' not in self._thruster_params:
                raise Exception('No limit to thruster output was given')
            self._thruster_model = Thruster.create_thruster(
                        self, self._thruster_params['name'].value, 0,
                        self._thruster_topic, None, None,
                        **{key: val.value for key, val in self._thruster_params['params'].items()})
        except Exception as e:
            raise RuntimeError('Thruster model could not be initialized: ' + str(e))
        
        # Subscribe to the joystick topic
        self.sub_joy = self.create_subscription(Joy, 'joy', self.joy_callback, 10)

        self._ready = True

    #==========================================================================
    def joy_callback(self, msg):
        """Handle callbacks with joystick state."""

        if not self._ready:
            return

        try:
            thrust = max(0, msg.axes[self._joy_axis['axis_thruster']]) * \
                self._thruster_params['max_thrust'].value * \
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
            self.get_logger().error('Error occurred while parsing joystick input, check '
                  'if the joy_id corresponds to the joystick ' 
                  'being used. message={}'.format(e))

    # =========================================================================
    def build_topic_name(self, _fin_topic_prefix, id, _fin_topic_suffix):
        return _fin_topic_prefix + 'id_' + str(id) + _fin_topic_suffix


# ==============================================================================
def main(args=None):
    print('starting FinnedUUVControllerNode.py')

    rclpy.init(args=args)

    try:
        node = FinnedUUVControllerNode()
        rclpy.spin(node)
    except rclpy.exceptions.ROSInterruptException as excep:
        print('Caught ROSInterruptException exception: ' + str(excep))
    except Exception as e:
        print('Caught exception: ' + str(e))
    finally:
        if rclpy.ok():
            rclpy.shutdown()
    print('Exiting')

#==========================================================================
if __name__ == '__main__':
    main()
