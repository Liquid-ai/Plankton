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
import rclpy
import numpy as np
import traceback

from utils.transform import get_world_ned_to_enu
from uuv_control_interfaces import DPControllerBase
from uuv_control_msgs.srv import *
from plankton_utils.time import time_in_float_sec
from plankton_utils.time import is_sim_time


class ROV_NMB_SMController(DPControllerBase):
    """
    Model-free sliding mode controller based on the work published in [1] and
    [2], or model-free high order sliding mode controller.

    [1] Garcia-Valdovinos, Luis Govinda, et al. "Modelling, design and robust
        control of a remotely operated underwater vehicle." International
        Journal of Advanced Robotic Systems 11.1 (2014): 1.
    [2] Salgado-Jimenez, Tomas, Luis G. Garcia-Valdovinos, and Guillermo
        Delgado-Ramirez. "Control of ROVs using a Model-free 2nd-Order Sliding
        Mode Approach." Sliding Mode Control (2011): 347-368.
    """

    _LABEL = 'Model-free Sliding Mode Controller'

    def __init__(self, name, **kwargs):
        DPControllerBase.__init__(self, name, is_model_based=False, **kwargs)
  
        self._logger.info('Initializing: ' + self._LABEL)
        self._first_pass = True
        self._t_init = 0.0
        self._s_linear_b_init = np.array([0, 0, 0])
        self._s_angular_b_init = np.array([0, 0, 0])

        # 'K' gains (help in the initial transient)
        self._K = np.zeros(6)
        # Derivative gains
        self._Kd = np.zeros(6)
        # Robustness gains
        self._Ki = np.zeros(6)
        # Overall proportional gains
        self._slope = np.zeros(6)

        if self.has_parameter('K'):
            coefs = self.get_parameter('K').value
            if len(coefs) == 6:
                self._K = np.array(coefs)
            else:
                raise RuntimeError('K coefficients: 6 coefficients '
                                         'needed')

        self._logger.info('K=' + str(self._K))

        if self.has_parameter('Kd'):
            coefs = self.get_parameter('Kd').value
            if len(coefs) == 6:
                self._Kd = np.array(coefs)
            else:
                raise RuntimeError('Kd coefficients: 6 coefficients '
                                         'needed')

        self._logger.info('Kd=' + str(self._Kd))

        if self.has_parameter('Ki'):
            coefs = self.get_parameter('Ki').value
            if len(coefs) == 6:
                self._Ki = np.array(coefs)
            else:
                raise RuntimeError('Ki coeffcients: 6 coefficients '
                                         'needed')
        self._logger.info('Ki=' + str(self._Ki))

        if self.has_parameter('slope'):
            coefs = self.get_parameter('slope').value
            if len(coefs) == 6:
                self._slope = np.array(coefs)
            else:
                raise RuntimeError('Slope coefficients: 6 coefficients '
                                         'needed')

        self._logger.info('slope=' + str(self._slope))

        self._sat_epsilon = 0.8
        if self.has_parameter('sat_epsilon'):
            self._sat_epsilon = max(0.0, self.get_parameter('sat_epsilon').value)

        self._logger.info('Saturation limits=' + str(self._sat_epsilon))

        self._summ_sign_sn_linear_b = np.array([0, 0, 0])
        self._summ_sign_sn_angular_b = np.array([0, 0, 0])

        self._prev_sign_sn_linear_b = np.array([0, 0, 0])
        self._prev_sign_sn_angular_b = np.array([0, 0, 0])

        self._tau = np.zeros(6)
    
        srv_name = 'set_sm_controller_params'
        self._services[srv_name] = self.create_service(
            SetSMControllerParams,
            srv_name,
            self.set_sm_controller_params_callback,
            callback_group=self._callback_group)
        
        srv_name = 'get_sm_controller_params'
        self._services[srv_name] = self.create_service(
            GetSMControllerParams,
            srv_name,
            self.get_sm_controller_params_callback,
            callback_group=self._callback_group)

        self._is_init = True
        self._logger.info(self._LABEL + ' ready')

    # =========================================================================
    def _reset_controller(self):
        super(ROV_NMB_SMController, self)._reset_controller()
        self._first_pass = True
        self._t_init = 0.0
        self._s_linear_b_init = np.array([0, 0, 0])
        self._s_angular_b_init = np.array([0, 0, 0])
        self._prev_t = time_in_float_sec(self.get_clock().now())
        self._summ_sign_sn_linear_b = np.array([0, 0, 0])
        self._summ_sign_sn_angular_b = np.array([0, 0, 0])
        self._prev_sign_sn_linear_b = np.array([0, 0, 0])
        self._prev_sign_sn_angular_b = np.array([0, 0, 0])
        self._tau = np.zeros(6)

    # =========================================================================
    def set_sm_controller_params_callback(self, request, response):
        response.success = True
        return response
        #return SetSMControllerParamsResponse(True)

    # =========================================================================
    def get_sm_controller_params_callback(self, request, response):
        response.k = self._K.tolist()
        response.kd = self._Kd.tolist()
        response.ki = self._Ki.tolist()
        response.slope = self._slope.tolist()

        return response

    # =========================================================================
    def update_controller(self):
        if not self._is_init:
            return False
        t = time_in_float_sec(self.get_clock().now())

        dt = t - self._prev_t
        if self._prev_t < 0.0:
            dt = 0.0

        # Get trajectory errors (reference - actual)
        e_p_linear_b = self._errors['pos']
        e_v_linear_b = self._errors['vel'][0:3]

        e_p_angular_b = self.error_orientation_rpy # check this
        e_v_angular_b = self._errors['vel'][3:6]

        # Compute sliding surface s wrt body frame
        s_linear_b = -e_v_linear_b - np.multiply(self._slope[0:3],
                                                e_p_linear_b)
        s_angular_b = -e_v_angular_b - np.multiply(self._slope[3:6],
                                                  e_p_angular_b)

        # Compute exponential decay for transient improvement
        if self._first_pass == True:
            self._t_init, self._s_linear_b_init, self._s_angular_b_init = t, s_linear_b, s_angular_b
            self._first_pass = False

        sd_linear_b = np.multiply(self._s_linear_b_init,
                                  np.exp(-self._K[0:3] * (t - self._t_init)))
        sd_angular_b = np.multiply(self._s_angular_b_init,
                                   np.exp(-self._K[3:6] * (t - self._t_init)))

        # Compute sliding surface sn wrt body frame
        sn_linear_b = s_linear_b - sd_linear_b
        sn_angular_b = s_angular_b - sd_angular_b

        # Compute summation sign(sn) wrt body frame
        if self._prev_t > 0.0 and dt > 0.0:
            self._summ_sign_sn_linear_b = self._summ_sign_sn_linear_b + 0.5 * (
            self.sat(sn_linear_b, self._sat_epsilon) + self._prev_sign_sn_linear_b) * dt
            self._summ_sign_sn_angular_b = self._summ_sign_sn_angular_b + 0.5 * (
            self.sat(sn_angular_b, self._sat_epsilon) + self._prev_sign_sn_angular_b) * dt

        # Compute extended error wrt body frame
        sr_linear_b = sn_linear_b + np.multiply(self._Ki[0:3],
                                                self._summ_sign_sn_linear_b)
        sr_angular_b = sn_angular_b + np.multiply(self._Ki[3:6],
                                                  self._summ_sign_sn_angular_b)

        # Compute required forces and torques wrt body frame
        force_b = -np.multiply(self._Kd[0:3], sr_linear_b)
        torque_b = -np.multiply(self._Kd[3:6], sr_angular_b)

        self._tau = np.hstack((force_b, torque_b))

        self.publish_control_wrench(self._tau)

        self._prev_sign_sn_linear_b = self.sat(sn_linear_b, self._sat_epsilon)
        self._prev_sign_sn_angular_b = self.sat(sn_angular_b, self._sat_epsilon)
        self._prev_t = t

    # =========================================================================
    @staticmethod
    def sat(value, epsilon=0.5):
        assert epsilon >= 0, 'Saturation constant must be greate or equal to zero'
        if epsilon == 0:
            return np.sign(value)

        vec = value / epsilon
        output = np.zeros(vec.size)
        for i in range(output.size):
            if vec[i] > epsilon:
                output[i] = 1
            elif vec[i] < -epsilon:
                output[i] = -1
            else:
                output[i] = vec[i]
        return output


# =============================================================================
def main():
    print('Starting Non-model-based Sliding Mode Controller')
    rclpy.init()

    try:
        sim_time_param = is_sim_time()

        tf_world_ned_to_enu = get_world_ned_to_enu(sim_time_param)
        
        node = ROV_NMB_SMController(
            'rov_nmb_sm_controller',
            world_ned_to_enu=tf_world_ned_to_enu,
            parameter_overrides=[sim_time_param])
        executor = rclpy.executors.MultiThreadedExecutor()
        executor.add_node(node)
        executor.spin()
        # rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print('Caught exception: ' + repr(e))
        traceback.print_exc()
    finally:
        if rclpy.ok():
            rclpy.shutdown()
        
    print('Exiting')


# =============================================================================
if __name__ == '__main__':
    main()
