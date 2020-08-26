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

import numpy
from os.path import isdir, join
from time import sleep
import xml.etree.ElementTree as etree
import yaml

from geometry_msgs.msg import Wrench
import tf2_ros
import rclpy
from rclpy.node import Node

from .models import Thruster
from plankton_utils.param_handler import parse_nested_params_to_dict
from tf_quaternion import transformations
from uuv_gazebo_ros_plugins_msgs.msg import FloatStamped

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
        
    def __init__(self, node_name):
        """Class constructor."""
        super().__init__(node_name,
                        allow_undeclared_parameters=True, 
                        automatically_declare_parameters_from_overrides=True)

        #Default sim_time to True
        sim_time = rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)
        self.set_parameters([sim_time])

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

        self.config = self.get_parameters_by_prefix('thruster_manager')
        
        if len(self.config) == 0:
        #if not self.has_parameter('thruster_manager'):
            raise RuntimeError('Thruster manager parameters not '
                                     'initialized for uuv_name=' +
                                     self.namespace)

        self.config = parse_nested_params_to_dict(self.config, '.')
        # Load all parameters
        #self.config = self.get_parameter('thruster_manager').value

        robot_description_param = self.namespace + 'robot_description'
        self.use_robot_descr = False
        self.axes = {}
        if self.has_parameter(robot_description_param):
            self.use_robot_descr = True
            self.parse_urdf(self.get_parameter(robot_description_param).value)

        if self.config['update_rate'].value < 0:
            self.config['update_rate'].value = 50.0

        self.base_link_ned_to_enu = None

        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer, self)
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
                target, source, rclpy.time.Time(), rclpy.time.Duration(seconds=1))
        except Exception as e:
            self.get_logger().info('No transform found between base_link and base_link_ned'
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
        #TODO probably comment
        #self.set_parameters(['thruster_manager/tf_prefix'], [self.namespace])
        param_tf_prefix = rclpy.parameter.Parameter('thruster_manager.tf_prefix', rclpy.Parameter.Type.STRING, self.namespace)
        self.set_parameters([param_tf_prefix])

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

        #TODO Close check...transform to dict
        # Thruster allocation matrix: transform thruster inputs to force/torque
        self.configuration_matrix = None
        # tam = self.get_parameter('tam')
        # if len(tam) != 0:
            #tam = parse_nested_params_to_dict(tam, '.')
        if self.has_parameter('tam'):
            tam = self.get_parameter('tam').value
            tam = numpy.array(tam)
            #reshape the array. #TODO Unsure
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
            #TODO !!!!!!!!! Check if the topic name has to be corrected somewhere !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
            for i in range(self.n_thrusters):
                topic = self.config['thruster_topic_prefix'].value + 'id_' + str(i) + \
                    self.config['thruster_topic_suffix'].value
                if list not in [type(params), type(conv_fcn)]:
                    #Unpack parameters to values
                    deduced_params = {key: val.value for key, val in params.items()}
                    thruster = Thruster.create_thruster(
                        self, conv_fcn, i, topic, None, None,
                        **deduced_params)
                else:
                    #Unpack parameters to values
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
        # else:
        #     self.get_logger().info('Invalid output directory for the TAM matrix, dir=' + str(self.output_dir))

        self.ready = True
        self.get_logger().info('ThrusterManager: ready')

    #==============================================================================
    def parse_urdf(self, urdf_str):
        root = etree.fromstring(urdf_str)
        for joint in root.findall('joint'):
            if joint.get('type') == 'fixed':
                continue
            axis_str_list = joint.find('axis').get('xyz').split()
            child = joint.find('child').get('link')
            if child[0]!='/':
                child = '/'+child

            self.axes[child] = numpy.array([float(axis_str_list[0]),
                                            float(axis_str_list[1]),
                                            float(axis_str_list[2]), 0.0])


    #==============================================================================
    def update_tam(self, recalculate=False):
        """Calculate the thruster allocation matrix, if one is not given."""
        if self.configuration_matrix is not None and not recalculate:
            self.ready = True
            self.get_logger().info('TAM provided, skipping...')
            self.get_logger().info('ThrusterManager: ready')
            return True

        self.ready = False
        self.get_logger().info('ThrusterManager: updating thruster poses')
        # Small margin to make sure we get thruster frames via tf
        now = self.get_clock().now() + rclpy.time.Duration(nanoseconds=int(0.2 * 1e9))

        base = self.namespace + self.config['base_link'].value

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

        #listener = tf.TransformListener()
        #sleep(0.1)

        
        for i in range(self.MAX_THRUSTERS):
            frame = self.namespace + \
                self.config['thruster_frame_base'].value + str(i)
            try:
                # try to get thruster pose with respect to base frame via tf
                self.get_logger().info('transform: ' + base + ' -> ' + frame)
                now = self.get_clock().now() + rclpy.time.Duration(nanoseconds=int(0.2 * 1e9))
                self.tf_buffer.can_transform(base, frame,
                                               now, timeout=rclpy.time.Duration(seconds=1))
                [pos, quat] = self.tf_buffer.lookup_transform(base, frame, now)

                topic = self.config['thruster_topic_prefix'].value + 'id_' + str(i) + \
                    self.config['thruster_topic_suffix'].value

                # If not using robot_description, thrust_axis=None which will
                # result in the thrust axis being the x-axis,i.e. (1,0,0)
                thrust_axis = None if not self.use_robot_descr else self.axes[frame]

                if equal_thrusters:
                    params = self.config['conversion_fcn_params']
                    #Unpack parameters to values
                    params = {key: val.value for key, val in params.items()}
                    thruster = Thruster.create_thruster(
                        self, self.config['conversion_fcn'].value,
                        i, topic, pos, quat, self.axes[frame], **params)
                else:
                    if idx_thruster_model >= len(self.config['conversion_fcn'].value):
                        raise RuntimeError('Number of thrusters found and '
                                                 'conversion_fcn are different')
                    params = self.config['conversion_fcn_params'][idx_thruster_model]
                    #Unpack parameters to values
                    params = {key: val.value for key, val in params.items()}
                    conv_fcn = self.config['conversion_fcn'][idx_thruster_model].value
                    thruster = Thruster.create_thruster(
                        self,
                        conv_fcn,
                        i, topic, pos, quat, self.axes[frame],
                        **params)
                    idx_thruster_model += 1
                if thruster is None:
                    RuntimeError('Invalid thruster conversion '
                                       'function=%s'
                                       % self.config['conversion_fcn'].value)
                self.thrusters.append(thruster)
            except Exception:
                self.get_logger().info('could not get transform from: ' + base)
                self.get_logger().info('to: ' + frame)
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

        self.get_logger().info('TAM= %s', str(self.configuration_matrix))

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

        self.ready = True
        self.get_logger().info('ThrusterManager: ready')
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
        if not self.ready:
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
