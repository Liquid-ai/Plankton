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
import numpy as np
import os
import yaml

from geometry_msgs.msg import Wrench, WrenchStamped
import rclpy
from rclpy.node import Node
import tf2_py as tf2
import tf2_ros

#from tf2_py import LookupException
from tf_quaternion.transformations import quaternion_matrix
from uuv_thrusters.models import Thruster
from uuv_auv_control_allocator.msg import AUVCommand
from uuv_gazebo_ros_plugins_msgs.msg import FloatStamped
from .fin_model import FinModel
from plankton_utils.params_helper import parse_nested_params_to_dict

#TODO Refactor
class ActuatorManager(Node):
    MAX_FINS = 4

    def __init__(self, name, **kwargs):
        super().__init__(name,
                        allow_undeclared_parameters=True, 
                        automatically_declare_parameters_from_overrides=True,
                        **kwargs)

        # Acquiring the namespace of the vehicle
        self.namespace = self.get_namespace().replace('/', '')
        self.get_logger().info('Initialize control allocator for vehicle <%s>' % self.namespace)  

        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer, self)
        tf_trans_ned_to_enu = None

        try:
            if self.namespace != '':
                target = '{}/base_link'.format(self.namespace)
                source = '{}/base_link_ned'.format(self.namespace)
            else:
                target = 'base_link'
                source = 'base_link_ned'
            self.get_logger().info('Lookup transfrom from %s to %s' % (source, target))
            tf_trans_ned_to_enu = self.tf_buffer.lookup_transform().lookup_transform(
                target, source, rclpy.time.Time(), rclpy.time.Duration(seconds=1))
        except Exception as e:
            self.get_logger().warning('No transform found between base_link and base_link_ned'
                  ' for vehicle {}, message={}'.format(self.namespace, e))
            self.base_link_ned_to_enu = None

        if tf_trans_ned_to_enu is not None:
            self.base_link_ned_to_enu = quaternion_matrix(
                (tf_trans_ned_to_enu.transform.rotation.x,
                 tf_trans_ned_to_enu.transform.rotation.y,
                 tf_trans_ned_to_enu.transform.rotation.z,
                 tf_trans_ned_to_enu.transform.rotation.w))[0:3, 0:3]

            self.get_logger().warning('base_link transform NED to ENU=\n{}'.format(
                self.base_link_ned_to_enu))
        
        self.base_link = self.get_parameter('base_link', 'base_link').get_parameter_value().string_value

        # Retrieve the thruster configuration parameters if available
        thruster_config = self.get_parameters_by_prefix('thruster_config')
        if len(thruster_config) == 0:
            raise RuntimeError('Thruster configuration not available') 
        self.thruster_config = parse_nested_params_to_dict(self.thruster_config, '.', True)


        # Check if all necessary thruster model parameter are available
        thruster_params = ['conversion_fcn_params', 'conversion_fcn', 
            'topic_prefix', 'topic_suffix', 'frame_base', 'max_thrust']
        for p in thruster_params: 
            if p not in self.thruster_config:
                raise RuntimeError(
                    'Parameter <%s> for thruster conversion function is missing' % p)

        # Setting up the thruster topic name
        self.thruster_topic = build_topic_name(self.namespace, 
            self.thruster_config['topic_prefix'], 0, 
            self.thruster_config['topic_suffix'])
        
        self.thruster = None

        # Retrieve the fin configuration if available
        fin_config = self.get_parameters_by_prefix('fin_config')
        if len(fin_config) == 0:
            raise RuntimeError('Fin configuration is not available')
        
        
        self.fin_config = parse_nested_params_to_dict(self.fin_config, '.', True)

        # Check if all necessary fin parameters are available
        fin_params = ['fluid_density', 'lift_coefficient', 'fin_area', 
            'topic_prefix', 'topic_suffix', 'frame_base']
        
        for p in fin_params:
            if p not in self.fin_config:
                raise RuntimeError(
                    'Parameter <%s> for fin configuration is missing' % p)
        
        self.fin_lower_limit = -np.pi / 2
        if 'lower_limit' in self.fin_config:
            self.fin_lower_limit = self.fin_config['lower_limit']

        self.fin_upper_limit = np.pi / 2
        if 'upper_limit' in self.fin_config:
            self.fin_upper_limit = self.fin_config['upper_limit']

        if self.fin_config['lower_limit'] >= self.fin_config['upper_limit']:
            raise RuntimeError('Fin angle limits are invalid')

        self.fins = dict()
                
        self.n_fins = 0

        if not self.find_actuators():
            raise RuntimeError('No thruster and/or fins found')

    # =========================================================================
    def find_actuators(self):
        """Calculate the control allocation matrix, if one is not given."""
        
        self.ready = False
        self.get_logger().infos('ControlAllocator: updating thruster poses')

        base = '%s/%s' % (self.namespace, self.base_link)

        frame = '%s/%s%d' % (self.namespace, self.thruster_config['frame_base'], 0)

        self.get_logger().info('Lookup: Thruster transform found %s -> %s' % (base, frame))
        trans = self.tf_buffer.lookup_transform(base, frame, rclpy.time.Time(), rclpy.time.Duration(seconds=1))
        pos = np.array([trans.transform.translation.x,
                           trans.transform.translation.y,
                           trans.transform.translation.z])
        quat = np.array([trans.transform.rotation.x,
                            trans.transform.rotation.y,
                            trans.transform.rotation.z,
                            trans.transform.rotation.w])
        self.get_logger().info('Thruster transform found %s -> %s' % (base, frame))
        self.get_logger().info('pos=' + str(pos))
        self.get_logger().info('rot=' + str(quat))
                
        # Read transformation from thruster
        #params = {key: val.value for key, val in params.items()}
        self.thruster = Thruster.create_thruster(
            self.thruster_config['conversion_fcn'], 0, 
            self.thruster_topic, pos, quat, 
            **self.thruster_config['conversion_fcn_params'])

        for i in range(self.MAX_FINS):
            try:
                frame = '%s/%s%d' % (self.namespace, self.fin_config['frame_base'], i)
                
                self.get_logger().info('Lookup: Fin transform found %s -> %s' % (base, frame))
                trans = self.tf_buffer.lookup_transform(base, frame, rclpy.time.Time(), rclpy.time.Duration(seconds=1))
                pos = np.array([trans.transform.translation.x,
                                   trans.transform.translation.y,
                                   trans.transform.translation.z])
                quat = np.array([trans.transform.rotation.x,
                                    trans.transform.rotation.y,
                                    trans.transform.rotation.z,
                                    trans.transform.rotation.w])                
                self.get_logger().info('Fin transform found %s -> %s' % (base, frame))
                self.get_logger().info('pos=' + str(pos))
                self.get_logger().info('quat=' + str(quat))

                fin_topic = build_topic_name(self.namespace, 
                    self.fin_config['topic_prefix'], i, self.fin_config['topic_suffix'])

                self.fins[i] = FinModel(
                    i,
                    pos,
                    quat,
                    fin_topic,
                    self)

            except (tf2.LookupException, tf2.ConnectivityException, tf2.ExtrapolationException):
                self.get_logger().info('Could not get transform from %s to %s ' % (base, frame))
                break

        self.n_fins = len(self.fins.keys())
        self.get_logger().info('# fins found: %d' % len(self.fins.keys()))
        
        for i in range(self.n_fins):
            self.get_logger().info(i)
            self.get_logger().info(self.fins[i].pos)
            self.get_logger().info(self.fins[i].rot)

        self.ready = True
        return True

    # =========================================================================
    def compute_control_force(self, thrust, delta, u):
        actuator_model = self.thruster.tam_column.reshape((6, 1)) * thrust
        for i in self.fins:
            f_lift = (0.5 * self.fin_config['fluid_density'] * 
                self.fin_config['lift_coefficient'] * self.fin_config['fin_area'] *  
                delta[i] * u**2)
        
            tau = np.zeros(6)
            tau[0:3] = f_lift * self.fins[i].lift_vector
            tau[3::] = np.cross(self.fins[i].pos, f_lift)
            
            actuator_model += tau
        return actuator_model

    # =========================================================================
    def publish_commands(self, command):
        self.thruster.publish_command(command[0])

        for i in range(self.n_fins):
            self.fins[i].publish_command(command[i + 1])

    # =========================================================================
    def build_topic_name(self, namespace, topic_prefix, id, topic_prefix):
        return '/%s/%s/id_%d/%s' %  (namespace, topic_prefix, 0, topic_suffix)
