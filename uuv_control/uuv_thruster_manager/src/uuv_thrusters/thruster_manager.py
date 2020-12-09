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
from os.path import isdir, join
import time
from time import sleep
import xml.etree.ElementTree as etree
import yaml

from geometry_msgs.msg import Wrench
from std_msgs.msg import String
import tf2_ros
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSProfile

from .models import Thruster
from plankton_utils.param_helper import parse_nested_params_to_dict
from plankton_utils.param_helper import get_parameter_or_helper
from tf_quaternion import transformations
from uuv_gazebo_ros_plugins_msgs.msg import FloatStamped

import threading

class ThrusterManager(Node):
    """
    The thruster manager generates the thruster allocation matrix using the
    TF information and publishes the thruster forces assuming the the thruster
    topics are named in the following pattern

    <thruster_topic_prefix>/id_<index>/<thruster_topic_suffix>

    Thruster frames should also be named as follows

    <thruster_frame_base>_<index>
    """

    MAX_THRUSTERS = 16
        
    def __init__(self, name, **kwargs):
        """Class constructor."""
        super().__init__(name,
                        allow_undeclared_parameters=True, 
                        automatically_declare_parameters_from_overrides=True,
                        **kwargs)

        # This flag will be set to true once the thruster allocation matrix is
        # available
        self._ready = False
    
        # Acquiring the namespace of the vehicle
        self.namespace = self.get_namespace()
        if self.namespace != '':
            if self.namespace[-1] != '/':
                self.namespace += '/'

            if self.namespace[0] != '/':
                self.namespace = '/' + self.namespace

        # Load all parameters
        self.config = self.get_parameters_by_prefix('thruster_manager')
        
        if len(self.config) == 0:
            raise RuntimeError('Thruster manager parameters not '
                                     'initialized for uuv_name=' +
                                     self.namespace)

        self.config = parse_nested_params_to_dict(self.config, '.')
        
        # Indicates whether we could get the robot description for the thrust axes
        self.use_robot_descr = False
        self.axes = {}
        self.robot_description_subscription = None
        
        if self.config['update_rate'].value < 0:
            self.config['update_rate'].value = 50.0

        self.base_link_ned_to_enu = None

        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Initialize some variables
        self.output_dir = None
        self.n_thrusters = 0 
        # Thruster objects used to calculate the right angular velocity command
        self.thrusters = list()
        # Thrust forces vector
        self.thrust = None
        # Thruster allocation matrix: transform thruster inputs to force/torque
        self.configuration_matrix = None
        self.inverse_configuration_matrix = None

        self.init_future = rclpy.Future()
        self.init_thread = threading.Thread(target=self._init_async, daemon=True)
        self.init_thread.start()

    # =========================================================================
    def _init_async(self):
        try:
            self._init_async_impl()
        except Exception as e:
            self.get_logger().warn('Caught exception: ' + repr(e))

    # =========================================================================
    def _init_async_impl(self):
        # try to retrieve the thrust axes from the robot description
        timeout_robot_desc = get_parameter_or_helper(self, 'timeout_robot_description', 5).value
        self.get_axis_from_robot_description(timeout_robot_desc)

        # try to retrieve some tf transforms
        tf_trans_ned_to_enu = None
        
        try:
            if self.namespace != '':
                target = '{}base_link'.format(self.namespace)
                target = target[1::]
                source = '{}base_link_ned'.format(self.namespace)
            else:
                target = 'base_link'
                source = 'base_link_ned'
            source = source[1::]
            tf_trans_ned_to_enu = self.tf_buffer.lookup_transform(
                target, source, rclpy.time.Time(), rclpy.time.Duration(seconds=10))
        except Exception as e:
            self.get_logger().warn('No transform found between base_link and base_link_ned'
                  ' for vehicle {}, message={}'.format(self.namespace, e))
            self.base_link_ned_to_enu = None
    
        if tf_trans_ned_to_enu is not None:
            self.base_link_ned_to_enu = transformations.quaternion_matrix(
                (tf_trans_ned_to_enu.transform.rotation.x,
                 tf_trans_ned_to_enu.transform.rotation.y,
                 tf_trans_ned_to_enu.transform.rotation.z,
                 tf_trans_ned_to_enu.transform.rotation.w))[0:3, 0:3]

            self.get_logger().info('base_link transform NED to ENU=\n' + str(self.base_link_ned_to_enu))

        self.get_logger().info(
          'ThrusterManager::update_rate=' + str(self.config['update_rate'].value))

        # Set the tf_prefix parameter
        #TODO probably remove
        #self.set_parameters(['thruster_manager/tf_prefix'], [self.namespace])
        # param_tf_prefix = rclpy.parameter.Parameter('thruster_manager.tf_prefix', rclpy.Parameter.Type.STRING, self.namespace)
        # self.set_parameters([param_tf_prefix])

        # Retrieve the output file path to store the TAM
        # matrix for future use
        self.output_dir = None
        if self.has_parameter('output_dir'):
            self.output_dir = self.get_parameter('output_dir').get_parameter_value().string_value
            if not isdir(self.output_dir):
                raise RuntimeError(
                    'Invalid output directory, output_dir=' + self.output_dir)
            self.get_logger().info('output_dir=' + self.output_dir)

        # Number of thrusters
        self.n_thrusters = 0
    
        # Thruster objects used to calculate the right angular velocity command
        self.thrusters = list()

        # Thrust forces vector
        self.thrust = None

        # Thruster allocation matrix: transform thruster inputs to force/torque
        self.configuration_matrix = None
        
        if self.has_parameter('tam'):
            tam = self.get_parameter('tam').value
            tam = numpy.array(tam)
            # Reshape the array. #TODO Unsure
            self.configuration_matrix = numpy.reshape(tam, (6, -1))
            # Set number of thrusters from the number of columns
            self.n_thrusters = self.configuration_matrix.shape[1]
            # Create publishing topics to each thruster
            params = self.config['conversion_fcn_params']
            conv_fcn = self.config['conversion_fcn'].value
            if type(params) == list and type(conv_fcn) == list:
                if len(params) != self.n_thrusters or len(conv_fcn) != self.n_thrusters:
                    raise RuntimeError('Lists conversion_fcn and '
                                             'conversion_fcn_params must have '
                                             'the same number of items as of '
                                             'thrusters')
            
            for i in range(self.n_thrusters):
                # topic = self.config['thruster_topic_prefix'].value + 'id_' + str(i) + \
                #     self.config['thruster_topic_suffix'].value
                topic = self.build_topic_name(self.config['thruster_topic_prefix'].value, 
                                         i, 
                                         self.config['thruster_topic_suffix'].value)
                if list not in [type(params), type(conv_fcn)]:
                    # Unpack parameters to values
                    deduced_params = {key: val.value for key, val in params.items()}
                    thruster = Thruster.create_thruster(
                        self, conv_fcn, i, topic, None, None,
                        **deduced_params)
                else:
                    # Unpack parameters to values
                    deduced_params = {key: val.value for key, val in params[i].items()}
                    thruster = Thruster.create_thruster(
                        self, conv_fcn[i], i, topic, None, None,
                        **deduced_params)

                if thruster is None:
                    RuntimeError('Invalid thruster conversion '
                                       'function=%s'
                                       % self.config['conversion_fcn'].value)
                self.thrusters.append(thruster)

            self.get_logger().info('Thruster allocation matrix provided!')
            self.get_logger().info('TAM=')
            self.get_logger().info(str(self.configuration_matrix))
            self.thrust = numpy.zeros(self.n_thrusters)

        if not self.update_tam():
            raise RuntimeError('No thrusters found')

        # (pseudo) inverse: force/torque to thruster inputs
        self.inverse_configuration_matrix = None
        if self.configuration_matrix is not None:
            self.inverse_configuration_matrix = numpy.linalg.pinv(
                self.configuration_matrix)

        # If an output directory was provided, store matrix for further use
        if self.output_dir is not None:
            with open(join(self.output_dir, 'TAM.yaml'), 'w') as yaml_file:
                yaml_file.write(
                    yaml.safe_dump(
                        dict(tam=self.configuration_matrix.tolist())))

        self._ready = True
        self.init_future.set_result(True)
        self.get_logger().info('ThrusterManager: ready')

    #==============================================================================
    def parse_urdf(self, urdf_str):
        root = etree.fromstring(urdf_str)
        for joint in root.findall('joint'):
            if joint.get('type') == 'fixed':
                continue
            axis_str_list = joint.find('axis').get('xyz').split()
            child = joint.find('child').get('link')
            # nb: the following was removed for tf
            # if child[0]!='/':
            #     child = '/'+child

            self.axes[child] = numpy.array([float(axis_str_list[0]),
                                            float(axis_str_list[1]),
                                            float(axis_str_list[2]), 0.0])


    #==============================================================================
    def update_tam(self, recalculate=False):
        """Calculate the thruster allocation matrix, if one is not given."""
        if self.configuration_matrix is not None and not recalculate:
            self._ready = True
            self.get_logger().info('TAM provided, skipping...')
            # self.get_logger().info('ThrusterManager: ready')
            return True

        self._ready = False
        self.get_logger().info('ThrusterManager: updating thruster poses')
        # Small margin to make sure we get thruster frames via tf
        now = self.get_clock().now() + rclpy.time.Duration(nanoseconds=int(0.2 * 1e9))

        base = self.namespace + self.config['base_link'].value
        # Strip /
        base = base[1:]

        self.thrusters = list()

        equal_thrusters = True
        idx_thruster_model = 0

        if type(self.config['conversion_fcn_params']) == list and \
            type(self.config['conversion_fcn'].value) == list:
            if len(self.config['conversion_fcn_params']) != len(
                self.config['conversion_fcn'].value):
                raise RuntimeError(
                    'Lists of conversion_fcn_params and conversion_fcn'
                    ' must have equal length')
            equal_thrusters = False

        self.get_logger().info('conversion_fcn=' + str(self.config['conversion_fcn'].value))
        self.get_logger().info('conversion_fcn_params=' + str(self.config['conversion_fcn_params']))
        
        for i in range(self.MAX_THRUSTERS):
            frame = self.namespace + \
                self.config['thruster_frame_base'].value + str(i)
            # Strip /
            frame = frame[1:]
            try:
                # try to get thruster pose with respect to base frame via tf
                self.get_logger().info('transform: ' + base + ' -> ' + frame)
                now = self.get_clock().now() + rclpy.time.Duration(nanoseconds=int(0.2 * 1e9))
                # self.tf_buffer.can_transform(base, frame,
                #                                now, timeout=rclpy.time.Duration(seconds=1))

                transformObject = self.tf_buffer.lookup_transform(base, frame, now, timeout=rclpy.duration.Duration(seconds=5))
                pos = numpy.array([transformObject.transform.translation.x,
                           transformObject.transform.translation.y,
                           transformObject.transform.translation.z])
                quat = numpy.array([transformObject.transform.rotation.x,
                            transformObject.transform.rotation.y,
                            transformObject.transform.rotation.z,
                            transformObject.transform.rotation.w])

                # topic = self.config['thruster_topic_prefix'].value + 'id_' + str(i) + \
                #     self.config['thruster_topic_suffix'].value
                topic = self.build_topic_name(self.config['thruster_topic_prefix'].value,
                                              i,
                                              self.config['thruster_topic_suffix'].value)

                # If not using robot_description, thrust_axis=None which will
                # result in the thrust axis being the x-axis,i.e. (1,0,0)
                thrust_axis = None if not self.use_robot_descr else self.axes[frame]

                if equal_thrusters:
                    params = self.config['conversion_fcn_params']
                    # Unpack parameters to values
                    params = {key: val.value for key, val in params.items()}
                    thruster = Thruster.create_thruster(
                        self, self.config['conversion_fcn'].value,
                        i, topic, pos, quat, thrust_axis, **params)
                else:
                    if idx_thruster_model >= len(self.config['conversion_fcn'].value):
                        raise RuntimeError('Number of thrusters found and '
                                                 'conversion_fcn are different')
                    params = self.config['conversion_fcn_params'][idx_thruster_model]
                    # Unpack parameters to values
                    params = {key: val.value for key, val in params.items()}
                    conv_fcn = self.config['conversion_fcn'][idx_thruster_model].value
                    thruster = Thruster.create_thruster(
                        self,
                        conv_fcn,
                        i, topic, pos, quat, thrust_axis,
                        **params)
                    idx_thruster_model += 1
                if thruster is None:
                    RuntimeError('Invalid thruster conversion '
                                       'function=%s'
                                       % self.config['conversion_fcn'].value)
                self.thrusters.append(thruster)
            except Exception as e:
                self.get_logger().warn('could not get transform from: ' + base)
                self.get_logger().warn('to: ' + frame)
                self.get_logger().warn('Except: ' + repr(e))
                break

        self.get_logger().info(str(self.thrusters))
        if len(self.thrusters) == 0:
            return False

        # Set the number of thrusters found
        self.n_thrusters = len(self.thrusters)

        # Fill the thrust vector
        self.thrust = numpy.zeros(self.n_thrusters)

        # Fill the thruster allocation matrix
        self.configuration_matrix = numpy.zeros((6, self.n_thrusters))

        for i in range(self.n_thrusters):
            self.configuration_matrix[:, i] = self.thrusters[i].tam_column

        # Eliminate small values
        self.configuration_matrix[numpy.abs(
            self.configuration_matrix) < 1e-3] = 0.0

        self.get_logger().info('TAM= %s' % str(self.configuration_matrix))

        # Once we know the configuration matrix we can compute its
        # (pseudo-)inverse:
        self.inverse_configuration_matrix = numpy.linalg.pinv(
            self.configuration_matrix)

        # If an output directory was provided, store matrix for further use
        if self.output_dir is not None and not recalculate:
            with open(join(self.output_dir, 'TAM.yaml'), 'w') as yaml_file:
                yaml_file.write(
                    yaml.safe_dump(
                        dict(tam=self.configuration_matrix.tolist())))
            self.get_logger().info('TAM saved in <{}>'.format(join(self.output_dir, 'TAM.yaml')))
        elif recalculate:
            self.get_logger().info('Recalculate flag on, matrix will not be stored in TAM.yaml')
        else:
            self.get_logger().error('Invalid output directory for the TAM matrix, dir='.format(
                self.output_dir))

        self._ready = True
        self.get_logger().info('TAM updated')
        return True

    #==============================================================================
    def command_thrusters(self):
        """Publish the thruster input into their specific topic."""
        if self.thrust is None:
            return
        for i in range(self.n_thrusters):
            self.thrusters[i].publish_command(self.thrust[i])

    #==============================================================================
    def publish_thrust_forces(self, control_forces, control_torques,
                              frame_id=None):
        if not self._ready:
            return

        if frame_id is not None:
            if self.config['base_link'].value != frame_id:
                assert self.base_link_ned_to_enu is not None, 'Transform from'
                ' base_link_ned to base_link could not be found'
                if 'base_link_ned' not in self.config['base_link'].value:
                    control_forces = numpy.dot(self.base_link_ned_to_enu,
                                               control_forces)
                    control_torques = numpy.dot(self.base_link_ned_to_enu,
                                                control_torques)
                else:
                    control_forces = numpy.dot(self.base_link_ned_to_enu.T,
                                               control_forces)
                    control_torques = numpy.dot(self.base_link_ned_to_enu.T,
                                                control_torques)

        gen_forces = numpy.hstack(
            (control_forces, control_torques)).transpose()
        self.thrust = self.compute_thruster_forces(gen_forces)
        self.command_thrusters()

    #==============================================================================
    def compute_thruster_forces(self, gen_forces):
        """Compute desired thruster forces using the inverse configuration
        matrix.
        """
        # Calculate individual thrust forces
        thrust = self.inverse_configuration_matrix.dot(gen_forces)
        # Obey limit on max thrust by applying a constant scaling factor to all
        # thrust forces
        limitation_factor = 1.0
        if type(self.config['max_thrust'].value) == list:
            if len(self.config['max_thrust'].value) != self.n_thrusters:
                raise RuntimeError('max_thrust list must have the length'
                                         ' equal to the number of thrusters')
            max_thrust = self.config['max_thrust'].value
        else:
            max_thrust = [self.config['max_thrust'].value for _ in range(self.n_thrusters)]
        for i in range(self.n_thrusters):
            if abs(thrust[i]) > max_thrust[i]:
                thrust[i] = numpy.sign(thrust[i]) * max_thrust[i]
        return thrust

    # =========================================================================
    @property
    def ready(self):
        return self._ready

    # =========================================================================
    def build_topic_name(self, prefix, id, suffix):
        return prefix + 'id_' + str(id) + suffix

    # =========================================================================
    def get_axis_from_robot_description(self, timeout=5):
        '''Must be called in a separate thread. Waits until the robot 
        description has been received or the timeout has expired
        '''

        robot_description_content = ''

        def robot_desc_cb(msg):
            nonlocal robot_description_content
            robot_description_content = msg.data

        latched_qos = QoSProfile(
                depth=1,
                durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL)
        self.robot_description_subscription = self.create_subscription(
                String, 'robot_description', robot_desc_cb, latched_qos)

        self.get_logger().info(
            'Waiting for the robot_description in topic: {}...'.format(self.robot_description_subscription.topic_name))

        current = time.time()
        while time.time() < current + timeout and robot_description_content == '':
            sleep(0.1)

        self.use_robot_descr = False
        self.axes = {}
        if robot_description_content != '':  
                self.get_logger().info('Initializing thrust axis with the robot description')  
                self.use_robot_descr = True
                self.parse_urdf(robot_description_content)
        else:
            self.get_logger().info('No robot description provided, defaulting thrust axis')  