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
import os
import rclpy
import numpy as np
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist, Accel, Vector3
from sensor_msgs.msg import Joy
from rclpy.node import Node
from plankton_utils.param_helper import parse_nested_params_to_dict
from plankton_utils.time import is_sim_time


class VehicleTeleop(Node):
    def __init__(self, name, **kwargs):
        super().__init__(name,
                        allow_undeclared_parameters=True, 
                        automatically_declare_parameters_from_overrides=True,
                        **kwargs)

        # Load the mapping for each input
        self._axes = dict(x=4, y=3, z=1,
                          roll=2, pitch=5, yaw=0,
                          xfast=-1, yfast=-1, zfast=-1,
                          rollfast=-1, pitchfast=-1, yawfast=-1)
        # Load the gain for each joystick axis input
        # (default values for the XBox 360 controller)
        self._axes_gain = dict(x=3, y=3, z=0.5,
                               roll=0.5, pitch=0.5, yaw=0.5,
                               xfast=6, yfast=6, zfast=1,
                               rollfast=2, pitchfast=2, yawfast=2)

        mapping = self.get_parameters_by_prefix('mapping')
        
        if len(mapping) != 0:       
            mapping = parse_nested_params_to_dict(mapping, '.')
            for tag in self._axes:
                if tag not in mapping:
                    self.get_logger().debug('Tag not found in axes mapping, tag=%s' % tag)
                else:
                    if 'axis' in mapping[tag]:
                        self._axes[tag] = mapping[tag]['axis'].get_parameter_value().integer_value
                    if 'gain' in mapping[tag]:
                        self._axes_gain[tag] = mapping[tag]['gain'].get_parameter_value().double_value

        # Dead zone: Force values close to 0 to 0
        # (Recommended for imprecise controllers)
        self._deadzone = 0.5
        if self.has_parameter('deadzone'):
            self._deadzone = self.get_parameter('deadzone').get_parameter_value().double_value

        # Default for the RB button of the XBox 360 controller
        self._deadman_button = -1
        if self.has_parameter('deadman_button'):
            self._deadman_button = self.get_parameter('deadman_button').get_parameter_value().integer_value

       
        # If these buttons are pressed, the arm will not move
        if self.has_parameter('exclusion_buttons'):
            self._exclusion_buttons = self.get_parameter('exclusion_buttons').value
            if type(self._exclusion_buttons) == str:
                # Eloquent type inference problem...if a list is passed in the launch file from
                # arg element to param with a value-sep attribute, it is not correctly parsed 
                # (you get a list with 1 str value). Let's provide a custom parser:
                self._exclusion_buttons = str(self._exclusion_buttons).split(',')
            if type(self._exclusion_buttons) in [float, int]:
                self._exclusion_buttons = [int(self._exclusion_buttons)]
            elif type(self._exclusion_buttons) == list:
                for n in self._exclusion_buttons:
                    if type(n) is str:
                        try:
                            int(n)
                        except:
                            raise Exception(
                                'Exclusion buttons must be an integer index or an equivalent str value')
                    elif type(n) not in [float, int]:
                        raise Exception(
                            'Exclusion buttons must be an integer index to '
                            'the joystick button')

                # Ensure we only get int values
                self._exclusion_buttons = [int(n) for n in self._exclusion_buttons]
        else:
            self._exclusion_buttons = list()

        # Default for the start button of the XBox 360 controller
        self._home_button = 7
        if self.has_parameter('home_button'):
            self._home_button = self.get_parameter('home_button').get_parameter_value().integer_value

        self._msg_type = 'twist'
        if self.has_parameter('type'):
            self._msg_type = self.get_parameter('type').get_parameter_value().string_value
            if self._msg_type not in ['twist', 'accel']:
                raise Exception('Teleoperation output must be either '
                                         'twist or accel')

        if self._msg_type == 'twist':
            self._output_pub = self.create_publisher(Twist, 'output', 1)
        else:
            self._output_pub = self.create_publisher(Accel, 'output', 1)

        self._home_pressed_pub = self.create_publisher(Bool, 'home_pressed', 1)  

        # Joystick topic subscriber
        self._joy_sub = self.create_subscription(Joy, 'joy', self._joy_callback, 10)

    #==============================================================================
    def _parse_joy(self, joy=None):
        if self._msg_type == 'twist':
            cmd = Twist()
        else:
            cmd = Accel()
        if joy is not None:
            # Linear velocities:
            l = Vector3(x=0.0, y=0.0, z=0.0)

            if self._axes['x'] > -1 and abs(joy.axes[self._axes['x']]) > self._deadzone:
                l.x += self._axes_gain['x'] * joy.axes[self._axes['x']]

            if self._axes['y'] > -1 and abs(joy.axes[self._axes['y']]) > self._deadzone:
                l.y += self._axes_gain['y'] * joy.axes[self._axes['y']]

            if self._axes['z'] > -1 and abs(joy.axes[self._axes['z']]) > self._deadzone:
                l.z += self._axes_gain['z'] * joy.axes[self._axes['z']]

            if self._axes['xfast'] > -1 and abs(joy.axes[self._axes['xfast']]) > self._deadzone:
                l.x += self._axes_gain['xfast'] * joy.axes[self._axes['xfast']]

            if self._axes['yfast'] > -1 and abs(joy.axes[self._axes['yfast']]) > self._deadzone:
                l.y += self._axes_gain['yfast'] * joy.axes[self._axes['yfast']]

            if self._axes['zfast'] > -1 and abs(joy.axes[self._axes['zfast']]) > self._deadzone:
                l.z += self._axes_gain['zfast'] * joy.axes[self._axes['zfast']]

            # Angular velocities:
            a = Vector3(x=0.0, y=0.0, z=0.0)

            if self._axes['roll'] > -1 and abs(joy.axes[self._axes['roll']]) > self._deadzone:
                a.x += self._axes_gain['roll'] * joy.axes[self._axes['roll']]

            if self._axes['rollfast'] > -1 and abs(joy.axes[self._axes['rollfast']]) > self._deadzone:
                a.x += self._axes_gain['rollfast'] * joy.axes[self._axes['rollfast']]

            if self._axes['pitch'] > -1 and abs(joy.axes[self._axes['pitch']]) > self._deadzone:
                a.y += self._axes_gain['pitch'] * joy.axes[self._axes['pitch']]

            if self._axes['pitchfast'] > -1 and abs(joy.axes[self._axes['pitchfast']]) > self._deadzone:
                a.y += self._axes_gain['pitchfast'] * joy.axes[self._axes['pitchfast']]

            if self._axes['yaw'] > -1 and abs(joy.axes[self._axes['yaw']]) > self._deadzone:
                a.z += self._axes_gain['yaw'] * joy.axes[self._axes['yaw']]

            if self._axes['yawfast'] > -1 and abs(joy.axes[self._axes['yawfast']]) > self._deadzone:
                a.z += self._axes_gain['yawfast'] * joy.axes[self._axes['yawfast']]

            cmd.linear = l
            cmd.angular = a
        else:
            cmd.linear =  Vector3(x=0.0, y=0.0, z=0.0)
            cmd.angular = Vector3(x=0.0, y=0.0, z=0.0)
        return cmd

    #==============================================================================
    def _joy_callback(self, joy):
        # If any exclusion buttons are pressed, do nothing
        try:
            for n in self._exclusion_buttons:
                if joy.buttons[n] == 1:
                    cmd = self._parse_joy()
                    self._output_pub.publish(cmd)
                    return

            if self._deadman_button != -1:
                if joy.buttons[self._deadman_button] == 1:
                    cmd = self._parse_joy(joy)
                else:
                    cmd = self._parse_joy()
            else:
                cmd = self._parse_joy(joy)
            
            self._output_pub.publish(cmd)
            self._home_pressed_pub.publish(
                Bool(data=bool(joy.buttons[self._home_button])))
        except Exception as e:
            self.get_logger().error('Error occurred while parsing joystick input,'
                  ' check if the joy_id corresponds to the joystick ' 
                  'being used. message="{}"'.format(e))

#==============================================================================
def main(args=None):
    # Start the node
    name = os.path.splitext(os.path.basename(__file__))[0]
    rclpy.init()
    
    try:
        sim_time_param = is_sim_time()
        teleop = VehicleTeleop(name, parameter_overrides=[sim_time_param])
        teleop.get_logger().info('Starting [%s] node' % name)

        rclpy.spin(teleop)
    except Exception as e:
        print('Caught exception: ' + str(e))

    teleop.get_logger().info('Shutting down [%s] node' % name)
    rclpy.shutdown()

#==============================================================================
if __name__ == '__main__':
    main()
