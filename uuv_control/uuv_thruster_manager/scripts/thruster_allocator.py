#!/usr/bin/env python3
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
from __future__ import print_function
import numpy
import rclpy
#import tf
#import tf.transformations as trans
from os.path import isdir, join
from copy import deepcopy
import yaml
#import tf2_ros
from uuv_thrusters import ThrusterManager
from geometry_msgs.msg import Wrench, WrenchStamped
from uuv_thruster_manager.srv import *
from plankton_utils.time import time_in_float_sec


class ThrusterAllocatorNode(ThrusterManager):
    """The thruster allocator node allows a client node 
    to command the thrusters.
    """

    def __init__(self, node_name):
        """Class constructor."""
        ThrusterManager.__init__(self, node_name)
 
        self.last_update = time_in_float_sec(self.get_clock().now())
   
        # Subscriber to the wrench to be applied on the UUV
        self.input_sub = self.create_subscription(Wrench, 'thruster_manager/input',
                                          self.input_callback, 10)

    
        # To deliver the wrench input with an option to use another body frame
        # (options: base_link and base_link_ned), use the wrench stamped
        # message
        self.input_stamped_sub = self.create_subscription(
            WrenchStamped, 'thruster_manager/input_stamped', 
            self.input_stamped_callback, 10)
        self.thruster_info_service = self.create_service(
            ThrusterManagerInfo, 'thruster_manager/get_thrusters_info',
            self.get_thruster_info)
        self.curve_calc_service = self.create_service(
            GetThrusterCurve, 'thruster_manager/get_thruster_curve',
            self.get_thruster_curve)
        self.set_thruster_manager_config_service = self.create_service(
            SetThrusterManagerConfig, 'thruster_manager/set_config',
            self.set_config)
        self.get_thruster_manager_config_service = self.create_service(
            GetThrusterManagerConfig, 'thruster_manager/get_config',
            self.get_config)

        #rate = self.create_rate(self.config['update_rate'].value)
        self.timer_function = self.create_timer(
            1.0 / self.config['update_rate'].value, self.spin_function)

    # ==============================================================================
    def spin_function(self):
        if self.config['timeout'].value > 0:
            # If a timeout is set, zero the outputs to the thrusters if
            # there is no command signal for the length of timeout
            if time_in_float_sec(self.get_clock().now()) - self.last_update > self.config['timeout'].value:
                self.get_logger().info('Turning thrusters off - inactive for too long')
                if self.thrust is not None:
                    self.thrust.fill(0)
                    self.command_thrusters()        
            

    # ==============================================================================
    def get_thruster_info(self, request, response):
        """Return service callback with thruster information."""
        response.n_thrusters = n_thrusters
        response.allocation_matrix = self.configuration_matrix.flatten().tolist()
        response.reference_frame = self.namespace + self.config['base_link'].value

        return response
        # return ThrusterManagerInfoResponse(
        #     self.n_thrusters,
        #     self.configuration_matrix.flatten().tolist(),
        #     self.namespace + self.config['base_link'].value)

    # ==============================================================================
    def get_thruster_curve(self, request, response):
        """Return service callback for computation of thruster curve."""

        if self.n_thrusters == 0:
            response.input = []
            response.thrust = []
        
        # TODO Get thruster index, for the case the vehicle has different
        # models
        input_values, thrust_values = self.thrusters[0].get_curve(
            request.min, request.max, request.n_points)

        response.input = input_values
        response.thrust = thrust_values

        return response

        # if self.n_thrusters == 0:
        #     return GetThrusterCurveResponse([], [])
        # # TODO Get thruster index, for the case the vehicle has different
        # # models
        # input_values, thrust_values = self.thrusters[0].get_curve(
        #     request.min, request.max, request.n_points)
        # return GetThrusterCurveResponse(input_values, thrust_values)

    # ==============================================================================
    def set_config(self, request, response):
        old_config = deepcopy(self.config)
        self.ready = False
        self.config['base_link'].value = request.base_link
        self.config['thruster_frame_base'].value = request.thruster_frame_base
        self.config['thruster_topic_prefix'].value = request.thruster_topic_prefix
        self.config['thruster_topic_suffix'].value = request.thruster_topic_suffix
        self.config['timeout'] = request.timeout
        self.get_logger().info('New configuration:\n')
        for key in self.config:
            self.get_logger().info(key, '=', self.config[key].value)
        if not self.update_tam(recalculate=True):
            self.get_logger().info('Configuration parameters are invalid, going back to old configuration...')
            self.config = old_config
            self.update_tam(recalculate=True)

        response.success = True
        return response

        # return SetThrusterManagerConfigResponse(True)

    # ==============================================================================
    def get_config(self, request, response):
        response.tf_prefix = self.namespace
        response.base_link = self.config['base_link'].value
        response.thruster_frame_base = self.config['thruster_frame_base'].value
        response.thruster_topic_prefix = self.config['thruster_topic_prefix'].value
        response.thruster_topic_suffix = self.config['thruster_topic_suffix'].value
        response.timeout = self.config['timeout'].value
        response.max_thrust = self.config['max_thrust'].value
        response.n_thrusters = self.n_thrusters
        response.allocation_matrix = self.configuration_matrix.flatten().tolist()

        return response
        # return GetThrusterManagerConfigResponse(
        #     self.namespace,
        #     self.config['base_link'].value,
        #     self.config['thruster_frame_base'].value,
        #     self.config['thruster_topic_prefix'].value,
        #     self.config['thruster_topic_suffix'].value,
        #     self.config['timeout'].value,
        #     self.config['max_thrust'].value,
        #     self.n_thrusters,
        #     self.configuration_matrix.flatten().tolist())

    # ==============================================================================
    def input_callback(self, msg):
        """
        Callback to the subscriber that receiver the wrench to be applied on
        UUV's BODY frame.
        @param msg Wrench message
        """
        if not self.ready:
            return

        force = numpy.array((msg.force.x, msg.force.y, msg.force.z))
        torque = numpy.array((msg.torque.x, msg.torque.y, msg.torque.z))

        # This mode assumes that the wrench is given wrt thruster manager
        # configured base_link reference
        self.publish_thrust_forces(force, torque)

        self.last_update = time_in_float_sec(self.get_clock().now())
    
    # ==============================================================================
    def input_stamped_callback(self, msg):
        """
        Callback to the subscriber that receiver the stamped wrench to be
        applied on UUV's BODY frame.
        @param msg Stamped wrench message
        """
        if not self.ready:
            return

        force = numpy.array(
            (msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z))
        torque = numpy.array(
            (msg.wrench.torque.x, msg.wrench.torque.y, msg.wrench.torque.z))

        # Send the frame ID for the requested wrench
        self.publish_thrust_forces(force, torque, msg.header.frame_id.split('/')[-1])
        self.last_update = time_in_float_sec(self.get_clock().now())

# ==============================================================================
def main():
    rclpy.init()

    try:
        node = ThrusterAllocatorNode('thruster_allocator')
        rclpy.spin(node)

    except Exception as e:
        print('ThrusterAllocatorNode::Exception ' + str(e))
    finally:
        if rclpy.ok()
            rclpy.shutdown()
    print('Leaving ThrusterAllocatorNode')

# ==============================================================================
if __name__ == '__main__':
    main()
