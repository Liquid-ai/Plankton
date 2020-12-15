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
import numpy as np
import rclpy
import traceback

from utils.transform import get_world_ned_to_enu
from uuv_control_msgs.srv import *
from uuv_control_interfaces.dp_controller_base import DPControllerBase
from plankton_utils.time import is_sim_time

class ROVUnderActuatedPIDController(DPControllerBase):
    """
    This is an abstract class for PID-based controllers. The base class method
    update_controller must be overridden in other for a controller to work.
    """

    def __init__(self, name, *args, **kwargs):
        # Start the super class
        DPControllerBase.__init__(self, name, *args, **kwargs)
        self._logger.info('Initializing: Underactuated PID controller')
        # Proportional gains
        self._Kp = np.zeros(shape=(4, 4))
        # Derivative gains
        self._Kd = np.zeros(shape=(4, 4))
        # Integral gains
        self._Ki = np.zeros(shape=(4, 4))
        # Integrator component
        self._int = np.zeros(4)
        # Error for the vehicle pose
        self._error_pose = np.zeros(4)

        if self.has_parameter('Kp'):
            Kp_diag = self.get_parameter('Kp').value
            if len(Kp_diag) == 4:
                self._Kp = np.diag(Kp_diag)
            else:
                raise RuntimeError('Kp matrix error: 4 coefficients '
                                         'needed')

        self._logger.info('Kp=' + str([self._Kp[i, i] for i in range(4)]))

        if self.has_parameter('Kd'):
            Kd_diag = self.get_parameter('Kd').value
            if len(Kd_diag) == 4:
                self._Kd = np.diag(Kd_diag)
            else:
                raise RuntimeError('Kd matrix error: 4 coefficients '
                                         'needed')

        self._logger.info('Kd=' + str([self._Kd[i, i] for i in range(4)]))

        if self.has_parameter('Ki'):
            Ki_diag = self.get_parameter('Ki').value
            if len(Ki_diag) == 4:
                self._Ki = np.diag(Ki_diag)
            else:
                raise RuntimeError('Ki matrix error: 4 coefficients '
                                         'needed')

        self._logger.info('Ki=' + str([self._Ki[i, i] for i in range(4)]))

        srv_name = 'set_pid_params'
        self._services[srv_name] = self.create_service(
            SetPIDParams,
            srv_name,
            self.set_pid_params_callback)

        srv_name = 'get_pid_params'
        self._services[srv_name] = self.create_service(
            GetPIDParams,
            srv_name,
            self.get_pid_params_callback)

        self._is_init = True
        self._logger.info('Underactuated PID controller ready!')

    # =========================================================================
    def _reset_controller(self):
        super(DPPIDControllerBase, self)._reset_controller()
        self._error_pose = np.zeros(4)
        self._int = np.zeros(4)

    # =========================================================================
    def set_pid_params_callback(self, request, response):
        kp = request.kp
        kd = request.kd
        ki = request.ki
        if len(kp) != 4 or len(kd) != 4 or len(ki) != 4:
            response.success = False
        else:
            self._Kp = np.diag(kp)
            self._Ki = np.diag(ki)
            self._Kd = np.diag(kd)
            response.success = True
        return response

    # =========================================================================
    def get_pid_params_callback(self, request, response):
        response.kp = [self._Kp[i, i] for i in range(4)]
        response.kd = [self._Kd[i, i] for i in range(4)]
        response.ki = [self._Ki[i, i] for i in range(4)]

        return response

    # =========================================================================
    def update_controller(self):
        if not self.odom_is_init:
            return False
        if not self._is_init:
            return False
        # Update integrator
        cur_error_pose = np.array([self.error_pose_euler[0],
                                   self.error_pose_euler[1],
                                   self.error_pose_euler[2],
                                   self.error_pose_euler[5]])
        self._int += 0.5 * (cur_error_pose + self._error_pose) * self._dt
        # Store current pose error
        self._error_pose = cur_error_pose
        error_vel = np.array([self._errors['vel'][0],
                              self._errors['vel'][1],
                              self._errors['vel'][2],
                              self._errors['vel'][5]])
        ua_tau = np.dot(self._Kp, cur_error_pose) \
                        + np.dot(self._Kd, error_vel) \
                        + np.dot(self._Ki, self._int)
        self._tau = np.array([ua_tau[0], ua_tau[1], ua_tau[2], 0, 0, ua_tau[3]])
        self.publish_control_wrench(self._tau)
        return True


# =============================================================================
if __name__ == '__main__':
    print('Starting Underactuated PID Controller')
    rclpy.init()

    try:
        sim_time_param = is_sim_time()

        tf_world_ned_to_enu = get_world_ned_to_enu(sim_time_param)

        node = ROVUnderActuatedPIDController(
            'rov_ua_pid_controller',
            world_ned_to_enu=tf_world_ned_to_enu,
            parameter_overrides=[sim_time_param])

        rclpy.spin(node)
    except Exception as e:
        print('Caught exception: ' + repr(e))
        traceback.print_exc()
    finally:
        if rclpy.ok():
            rclpy.shutdown()
    print('Exiting')
