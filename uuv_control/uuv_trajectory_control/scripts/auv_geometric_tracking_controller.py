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
from copy import deepcopy
import math
import numpy as np
import traceback
import threading

import rclpy
import tf2_ros
from rclpy.node import Node

import geometry_msgs.msg as geometry_msgs
from nav_msgs.msg import Odometry

import uuv_control_msgs.msg as uuv_control_msgs
from uuv_thrusters.models import Thruster
from uuv_gazebo_ros_plugins_msgs.msg import FloatStamped
from uuv_control_msgs.msg import TrajectoryPoint
from uuv_control_interfaces import DPControllerLocalPlanner
#from tf_quaternion.transformations import quaternion_matrix
from tf_quaternion.transformations import quaternion_multiply
from tf_quaternion.transformations import quaternion_inverse
from tf_quaternion.transformations import euler_from_quaternion

from plankton_utils.param_helper import parse_nested_params_to_dict, \
                                        get_parameter_or_helper
from plankton_utils.time import is_sim_time
from plankton_utils.time import time_in_float_sec
from utils.transform import get_world_ned_to_enu


class AUVGeometricTrackingController(Node):
    def __init__(self, name, world_ned_to_enu=None,**kwargs):
        super().__init__(name,
                        allow_undeclared_parameters=True, 
                        automatically_declare_parameters_from_overrides=True,
                        **kwargs)

        # sim_time = rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)
        # self.set_parameters([sim_time])

        self.namespace = self.get_namespace().replace('/', '')
        self.get_logger().info('Initialize control for vehicle <%s>' % self.namespace)

        self.local_planner = DPControllerLocalPlanner(self, full_dof=True, thrusters_only=False,
            stamped_pose_only=False, tf_trans_world_ned_to_enu=world_ned_to_enu)

        self.base_link = self.get_parameter_or_helper('base_link', 'base_link').get_parameter_value().string_value

        # Reading the minimum thrust generated
        self.min_thrust = self.get_parameter_or_helper('min_thrust', 0.0).value
        assert self.min_thrust >= 0
        self.get_logger().info('Min. thrust [N]=%.2f' % self.min_thrust)

        # Reading the maximum thrust generated
        self.max_thrust = self.get_parameter_or_helper('max_thrust', 0.0).value
        assert self.max_thrust > 0 and self.max_thrust > self.min_thrust
        self.get_logger().info('Max. thrust [N]=%.2f' % self.max_thrust)

        # Reading the thruster topic
        self.thruster_topic = self.get_parameter_or_helper('thruster_topic', 'thrusters/id_0/input').get_parameter_value().string_value
        assert len(self.thruster_topic) > 0

        # Reading the thruster gain
        self.p_gain_thrust = self.get_parameter_or_helper('thrust_p_gain', 0.0).value
        assert self.p_gain_thrust > 0

        self.d_gain_thrust = self.get_parameter_or_helper('thrust_d_gain', 0.0).value
        assert self.d_gain_thrust >= 0

        # Reading the roll gain
        self.p_roll = self.get_parameter_or_helper('p_roll', 0.0).value
        assert self.p_roll > 0

        # Reading the pitch P gain
        self.p_pitch = self.get_parameter_or_helper('p_pitch', 0.0).value
        assert self.p_pitch > 0

        # Reading the pitch D gain
        self.d_pitch = self.get_parameter_or_helper('d_pitch', 0.0).value
        assert self.d_pitch >= 0

        # Reading the yaw P gain
        self.p_yaw = self.get_parameter_or_helper('p_yaw', 0.0).value
        assert self.p_yaw > 0

        # Reading the yaw D gain
        self.d_yaw = self.get_parameter_or_helper('d_yaw', 0.0).value
        assert self.d_yaw >= 0

        # Reading the saturation for the desired pitch
        self.desired_pitch_limit = self.get_parameter_or_helper('desired_pitch_limit', 15 * np.pi / 180).value
        assert self.desired_pitch_limit > 0

        # Reading the saturation for yaw error
        self.yaw_error_limit = self.get_parameter_or_helper('yaw_error_limit', 1.0).value
        assert self.yaw_error_limit > 0

        # Reading the number of fins
        self.n_fins = self.get_parameter_or_helper('n_fins', 0).get_parameter_value().integer_value
        assert self.n_fins > 0

        # Reading the mapping for roll commands
        self.map_roll = self.get_parameter_or_helper('map_roll', [0, 0, 0, 0]).value
        assert isinstance(self.map_roll, list)
        assert len(self.map_roll) == self.n_fins

        # Reading the mapping for the pitch commands
        self.map_pitch = self.get_parameter_or_helper('map_pitch', [0, 0, 0, 0]). value
        assert isinstance(self.map_pitch, list)
        assert len(self.map_pitch) == self.n_fins
        
        # Reading the mapping for the yaw commands
        self.map_yaw = self.get_parameter_or_helper('map_yaw', [0, 0, 0, 0]).value
        assert isinstance(self.map_yaw, list)
        assert len(self.map_yaw) == self.n_fins

        # Retrieve the thruster configuration parameters
        self.thruster_config = self.get_parameters_by_prefix('thruster_config')
        #Parse parameters to dictionary and unpack params to values
        self.thruster_config = parse_nested_params_to_dict(self.thruster_config, '.', True)

        # Check if all necessary thruster model parameter are available
        thruster_params = ['conversion_fcn_params', 'conversion_fcn',
            'topic_prefix', 'topic_suffix', 'frame_base', 'max_thrust']
        for p in thruster_params:
            if p not in self.thruster_config:
                raise RuntimeError(
                    'Parameter <%s> for thruster conversion function is '
                    'missing' % p)

        # Setting up the thruster topic name
        self.thruster_topic = self.build_thruster_topic_name(self.namespace,
            self.thruster_config['topic_prefix'], 0,
            self.thruster_config['topic_suffix'])
        # self.thruster_topic = '/%s/%s/id_%d/%s' %  (self.namespace,
        #     self.thruster_config['topic_prefix'], 0,
        #     self.thruster_config['topic_suffix'])

        self.max_fin_angle = self.get_parameter_or_helper('max_fin_angle', 0.0).value
        assert self.max_fin_angle > 0

        # Reading the fin input topic prefix
        self.fin_topic_prefix = self.get_parameter_or_helper('fin_topic_prefix', 'fins').get_parameter_value().string_value
        self.fin_topic_suffix = self.get_parameter_or_helper('fin_topic_suffix', 'input').get_parameter_value().string_value

        self.rpy_to_fins = np.vstack((self.map_roll, self.map_pitch, self.map_yaw)).T
       
        self.pub_cmd = list()
        self.odometry_sub = None
        self.reference_pub = None
        self.error_pub = None

        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self._ready = False
        self.init_future = rclpy.Future()
        self.init_thread = threading.Thread(target=self._init_async, daemon=True)
        self.init_thread.start()

    # =========================================================================
    def _init_async(self):
        try:
            self._init_async_impl()
        except Exception as e:
            self.get_logger().error('Caught exception: ' + repr(e))
            traceback.print_exc()
    
    # =========================================================================
    def _init_async_impl(self):
        base = '%s/%s' % (self.namespace, self.base_link)

        frame = '%s/%s%d' % (self.namespace, self.thruster_config['frame_base'], 0)

        self.get_logger().info('Lookup: Thruster transform found %s -> %s' % (base, frame))
        trans = self.tf_buffer.lookup_transform(base, frame, rclpy.time.Time(), rclpy.time.Duration(seconds=5))
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
        # params = {key: val.value for key, val in params.items()}
        self.thruster = Thruster.create_thruster(
            self,
            self.thruster_config['conversion_fcn'], 0,
            self.thruster_topic, pos, quat,
            **self.thruster_config['conversion_fcn_params'])

        self.get_logger().info('Thruster configuration=\n' + str(self.thruster_config))
        self.get_logger().info('Thruster input topic=' + self.thruster_topic)

        self.pub_cmd = list()

        for i in range(self.n_fins):
            topic = self.build_fin_topic_name(self.fin_topic_prefix, i, self.fin_topic_suffix)
            #topic = '%s/id_%d/%s' % (self.fin_topic_prefix, i, self.fin_topic_suffix)
            self.pub_cmd.append(
              self.create_publisher(FloatStamped, topic, 10))

        self.odometry_sub = self.create_subscription(
            Odometry, 'odom', self.odometry_callback, 10)

        self.reference_pub = self.create_publisher(
            TrajectoryPoint, 'reference', 1)

        # Publish error (for debugging)
        self.error_pub = self.create_publisher(
            TrajectoryPoint, 'error', 1)

        self._ready = True
        self.get_logger().info('AUV geometric tracking controller: ready')
        self.init_future.set_result(True)

    #==========================================================================
    @property
    def ready(self):
        return self._ready

    #==========================================================================
    @staticmethod
    def unwrap_angle(t):
        return math.atan2(math.sin(t),math.cos(t))

    #==========================================================================
    @staticmethod
    def vector_to_np(v):
        return np.array([v.x, v.y, v.z])

    #==========================================================================
    @staticmethod
    def quaternion_to_np(q):
        return np.array([q.x, q.y, q.z, q.w])

    #==========================================================================
    def odometry_callback(self, msg):
        """Handle odometry callback: The actual control loop."""

        # Update local planner's vehicle position and orientation
        pos = [msg.pose.pose.position.x,
               msg.pose.pose.position.y,
               msg.pose.pose.position.z]

        quat = [msg.pose.pose.orientation.x,
                msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z,
                msg.pose.pose.orientation.w]

        self.local_planner.update_vehicle_pose(pos, quat)

        # Compute the desired position
        t = time_in_float_sec(self.get_clock().now())
        des = self.local_planner.interpolate(t)

        # Publish the reference
        ref_msg = TrajectoryPoint()
        ref_msg.header.stamp = self.get_clock().now().to_msg()
        ref_msg.header.frame_id = self.local_planner.inertial_frame_id
        ref_msg.pose.position = geometry_msgs.Point(**self.to_dict_vect3(*des.p))
        ref_msg.pose.orientation = geometry_msgs.Quaternion(**self.to_dict_quat(*des.q))
        ref_msg.velocity.linear = geometry_msgs.Vector3(**self.to_dict_vect3(*des.vel[0:3]))
        ref_msg.velocity.angular = geometry_msgs.Vector3(**self.to_dict_vect3(*des.vel[3::]))

        self.reference_pub.publish(ref_msg)

        p = self.vector_to_np(msg.pose.pose.position)
        forward_vel = self.vector_to_np(msg.twist.twist.linear)
        ref_vel = des.vel[0:3]

        q = self.quaternion_to_np(msg.pose.pose.orientation)
        rpy = euler_from_quaternion(q, axes='sxyz')

        # Compute tracking errors wrt world frame:
        e_p = des.p - p
        abs_pos_error = np.linalg.norm(e_p)
        abs_vel_error = np.linalg.norm(ref_vel - forward_vel)

        # Generate error message
        error_msg = TrajectoryPoint()
        error_msg.header.stamp = self.get_clock().now().to_msg()
        error_msg.header.frame_id = self.local_planner.inertial_frame_id
        error_msg.pose.position = geometry_msgs.Point(**self.to_dict_vect3(*e_p))
        error_msg.pose.orientation = geometry_msgs.Quaternion(
            **self.to_dict_quat(*quaternion_multiply(quaternion_inverse(q), des.q)))
        error_msg.velocity.linear = geometry_msgs.Vector3(
            **self.to_dict_vect3(*(des.vel[0:3] - self.vector_to_np(msg.twist.twist.linear))))
        error_msg.velocity.angular = geometry_msgs.Vector3(
            **self.to_dict_vect3(*(des.vel[3::] - self.vector_to_np(msg.twist.twist.angular))))

        # Based on position tracking error: Compute desired orientation
        pitch_des = -math.atan2(e_p[2], np.linalg.norm(e_p[0:2]))
        # Limit desired pitch angle:
        pitch_des = max(-self.desired_pitch_limit, min(pitch_des, self.desired_pitch_limit))

        yaw_des = math.atan2(e_p[1], e_p[0])
        yaw_err = self.unwrap_angle(yaw_des - rpy[2])

        # Limit yaw effort
        yaw_err = min(self.yaw_error_limit, max(-self.yaw_error_limit, yaw_err))

        # Roll: P controller to keep roll == 0
        roll_control = self.p_roll * rpy[0]

        # Pitch: P controller to reach desired pitch angle
        pitch_control = self.p_pitch * self.unwrap_angle(pitch_des - rpy[1]) + self.d_pitch * (des.vel[4] - msg.twist.twist.angular.y)

        # Yaw: P controller to reach desired yaw angle
        yaw_control = self.p_yaw * yaw_err + self.d_yaw * (des.vel[5] - msg.twist.twist.angular.z)

        # Limit thrust
        thrust = min(self.max_thrust, self.p_gain_thrust * np.linalg.norm(abs_pos_error) + self.d_gain_thrust * abs_vel_error)
        thrust = max(self.min_thrust, thrust)

        rpy = np.array([roll_control, pitch_control, yaw_control])

        # In case the world_ned reference frame is used, the convert it back
        # to the ENU convention to generate the reference fin angles
        rtf = deepcopy(self.rpy_to_fins)
        if self.local_planner.inertial_frame_id == 'world_ned':
            rtf[:, 1] *= -1
            rtf[:, 2] *= -1
        # Transform orientation command into fin angle set points
        fins = rtf.dot(rpy)

        # Check for saturation
        max_angle = max(np.abs(fins))
        if max_angle >= self.max_fin_angle:
            fins = fins * self.max_fin_angle / max_angle

        thrust_force = self.thruster.tam_column * thrust
        self.thruster.publish_command(thrust_force[0])

        cmd = FloatStamped()
        for i in range(self.n_fins):
            cmd.header.stamp = self.get_clock().now().to_msg()
            cmd.header.frame_id = '%s/fin%d' % (self.namespace, i)
            cmd.data = min(fins[i], self.max_fin_angle)
            cmd.data = max(cmd.data, -self.max_fin_angle)
            self.pub_cmd[i].publish(cmd)

        self.error_pub.publish(error_msg)

    #==========================================================================
    def get_parameter_or_helper(self, name, default_value):
        return get_parameter_or_helper(self, name, default_value)

    # =========================================================================
    def build_thruster_topic_name(self, namespace, topic_prefix, id, topic_suffix) -> str:
        return '/%s/%s/id_%d/%s' %  (namespace, topic_prefix, id, topic_suffix)

    # =========================================================================
    def build_fin_topic_name(self, topic_prefix, id, topic_suffix) -> str:
        return '%s/id_%d/%s' % (topic_prefix, id, topic_suffix)

    # =========================================================================
    def to_dict_vect3(self, *args) -> dict:
        return { 'x': args[0], 'y': args[1], 'z': args[2] }

    # =========================================================================
    def to_dict_quat(self, *args) -> dict:
        return { 'x': args[0], 'y': args[1], 'z': args[2], 'w': args[3] }


#==============================================================================
def main():
    print('Starting AUV trajectory tracker')
    rclpy.init()

    try:
        sim_time_param = is_sim_time()

        tf_world_ned_to_enu = get_world_ned_to_enu(sim_time_param)
        
        node = AUVGeometricTrackingController(
            'auv_geometric_tracking_controller',
            world_ned_to_enu=tf_world_ned_to_enu, 
            parameter_overrides=[sim_time_param])
        rclpy.spin(node)
        
    except Exception as e:
        print('caught exception: ' + repr(e))
        traceback.print_exc()
    finally:
        if rclpy.ok():
            rclpy.shutdown()

#==============================================================================
if __name__ == '__main__':
    main()
