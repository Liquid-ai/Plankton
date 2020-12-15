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
from uuv_control_interfaces.vehicle import cross_product_operator
from plankton_utils.time import time_in_float_sec
from plankton_utils.time import is_sim_time


class ROV_MB_SMController(DPControllerBase):
    _LABEL = 'Model-based Sliding Mode Controller'

    def __init__(self, name, **kwargs):
        DPControllerBase.__init__(self, name, True, **kwargs)
        self._logger.info('Initializing: ' + self._LABEL)

        # Lambda - Slope of the Sliding Surface
        self._lambda = np.zeros(6)
        # Rho Constant - Vector of positive terms for assuring sliding surface reaching condition
        self._rho_constant = np.zeros(6)
        # k - PD gain (P term = k * lambda , D term = k)
        self._k = np.zeros(6)
        # c - slope of arctan (the greater, the more similar with the sign function)
        self._c = np.zeros(6)
        # Adapt slope - Adaptation gain for the estimation of uncertainties
        # and disturbances upper boundaries
        # adapt_slope = [proportional to surface distance, prop. to square
        # of pose errors, prop. to square of velocity errors]
        self._adapt_slope = np.zeros(3)
        # Rho_0 - rho_adapt treshold for drift prevention
        self._rho_0 = np.zeros(6)
        # Drift prevent - Drift prevention slope
        self._drift_prevent = 0

        if self.has_parameter('lambda'):
            coeffs = self.get_parameter('lambda').value
            coeffs = [float(c) for c in coeffs]
            if len(coeffs) == 6:
                self._lambda = np.array(coeffs)
            else:
                raise RuntimeError('lambda coefficients: 6 coefficients '
                                         'needed')
        print('lambda=', self._lambda)

        if self.has_parameter('rho_constant'):
            coeffs = self.get_parameter('rho_constant').value
            coeffs = [float(c) for c in coeffs]
            if len(coeffs) == 6:
                self._rho_constant = np.array(coeffs)
            else:
                raise RuntimeError('rho_constant coefficients: 6 coefficients '
                                         'needed')
        print('rho_constant=', self._rho_constant)

        if self.has_parameter('k'):
            coeffs = self.get_parameter('k').value
            coeffs = [float(c) for c in coeffs]
            if len(coeffs) == 6:
                self._k = np.array(coeffs)
            else:
                raise RuntimeError('k coefficients: 6 coefficients '
                                         'needed')
        print('k=', self._k)

        if self.has_parameter('c'):
            coeffs = self.get_parameter('c').value
            coeffs = [float(c) for c in coeffs]
            if len(coeffs) == 6:
                self._c = np.array(coeffs)
            else:
                raise RuntimeError('c coefficients: 6 coefficients '
                                         'needed')
        print('c=', self._c)

        if self.has_parameter('adapt_slope'):
            coeffs = self.get_parameter('adapt_slope').value
            coeffs = [float(c) for c in coeffs]
            if len(coeffs) == 3:
                self._adapt_slope = np.array(coeffs)
            else:
                raise RuntimeError('adapt_slope coefficients: 6 coefficients '
                                         'needed')
        print('adapt_slope=', self._adapt_slope)

        if self.has_parameter('rho_0'):
            coeffs = self.get_parameter('rho_0').value
            coeffs = [float(c) for c in coeffs]
            if len(coeffs) == 6:
                self._rho_0 = np.array(coeffs)
            else:
                raise RuntimeError('rho_0 coefficients: 6 coefficients '
                                         'needed')
        print('rho_0=', self._rho_0)

        if self.has_parameter('drift_prevent'):
            scalar = self.get_parameter('drift_prevent').value
            coeffs = [float(c) for c in coeffs]
            if not isinstance(scalar, list):
                self._drift_prevent = scalar
            else:
                raise RuntimeError('drift_prevent needs to be a scalar value')

        print('drift_prevent=', self._drift_prevent)

        # Enable(1) / disable(0) integral term in the sliding surface
        if self.has_parameter('enable_integral_term'):
            self._sliding_int = self.get_parameter('enable_integral_term').get_parameter_value().integer_value
        else:
            self._sliding_int = 0

        # Enable(1) / disable(0) adaptive uncertainty upper boundaries for
        # robust control
        if self.has_parameter('adaptive_bounds'):
            self._adaptive_bounds = self.get_parameter('adaptive_bounds').get_parameter_value().integer_value
        else:
            self._adaptive_bounds = 1

        # Enable(1) / disable(0) constant uncertainty upper boundaries for
        # robust control
        if self.has_parameter('constant_bound'):
            self._constant_bound = self.get_parameter('constant_bound').get_parameter_value().integer_value
        else:
            self._constant_bound = 1

        # Enable(1) / disable(0) equivalent control term
        if self.has_parameter('ctrl_eq'):
            self._ctrl_eq = self.get_parameter('ctrl_eq').get_parameter_value().integer_value
        else:
            self._ctrl_eq = 1

        # Enable(1) / disable(0) linear control term
        if self.has_parameter('ctrl_lin'):
            self._ctrl_lin = self.get_parameter('ctrl_lin').get_parameter_value().integer_value
        else:
            self._ctrl_lin = 1

        # Enable(1) / disable(0) robust control term
        if self.has_parameter('ctrl_robust'):
            self._ctrl_robust = self.get_parameter('ctrl_robust').get_parameter_value().integer_value
        else:
            self._ctrl_robust = 1
        # Integrator component
        self._int = np.zeros(6)
        # Error for the vehicle pose
        self._error_pose = np.zeros(6)
        # Sliding Surface
        self._s_b = np.zeros(6)
        # Time derivative of the rotation matrix
        self._rotBtoI_dot = np.zeros(shape=(3, 3), dtype=float)
        # Linear acceleration estimation
        self._accel_linear_estimate_b = np.zeros(3)
        # Angular acceleration estimation
        self._accel_angular_estimate_b = np.zeros(3)
        # Acceleration estimation
        self._accel_estimate_b = np.zeros(6)
        # adaptive term of uncertainties upper bound estimation
        self._rho_adapt = np.zeros(6)
        # Upper bound for model uncertainties and disturbances
        self._rho_total = np.zeros(6)
        # Equivalent control
        self._f_eq = np.zeros(6)
        # Linear term of controller
        self._f_lin = np.zeros(6)
        # Robust control
        self._f_robust = np.zeros(6)
        # Total control
        self._tau = np.zeros(6)

        srv_name = 'set_mb_sm_controller_params'
        self._services[srv_name] = self.create_service(
            SetMBSMControllerParams,
            srv_name,
            self.set_mb_sm_controller_params_callback)

        srv_name = 'get_mb_sm_controller_params'
        self._services[srv_name] = self.create_service(
            GetMBSMControllerParams,
            srv_name,
            self.get_mb_sm_controller_params_callback)
        self._is_init = True
        self._logger.info(self._LABEL + ' ready')

    # =========================================================================
    def _reset_controller(self):
        super(ROV_MB_SMController, self)._reset_controller()
        self._sliding_int = 0
        self._adaptive_bounds = 0
        self._constant_bound = 0
        self._ctrl_eq = 0
        self._ctrl_lin = 0
        self._ctrl_robust = 0
        self._prev_t = 0
        self._int = np.zeros(6)
        self._error_pose = np.zeros(6)
        self._s_b = np.zeros(6)
        self._rotBtoI_dot = np.zeros(shape=(3, 3), dtype=float)
        self._accel_linear_estimate_b = np.zeros(3)
        self._accel_angular_estimate_b = np.zeros(3)
        self._accel_estimate_b = np.zeros(6)
        self._rho_adapt = np.zeros(6)
        self._rho_total = np.zeros(6)
        self._f_eq = np.zeros(6)
        self._f_lin = np.zeros(6)
        self._f_robust = np.zeros(6)
        self._tau = np.zeros(6)

    # =========================================================================
    def set_mb_sm_controller_params_callback(self, request, response):
        # What does this function do ?
        reponse.success = True
        return response

    # =========================================================================
    def get_mb_sm_controller_params_callback(self, request, response):
        # Not sure if it is still required to convert to lists
        response.lambda_array = self._lambda.tolist()
        response.rho_constant = self._rho_constant.tolist()
        response.k = self._k.tolist()
        response.c = self._c.tolist()
        response.adapt_slope = self._adapt_slope.tolist()
        response.rho_0 = self._rho_0.tolist()
        response.drift_prevent = self._drift_prevent

        return response

    # =========================================================================
    def update_controller(self):
        if not self._is_init:
            return False
        t = time_in_float_sec(self.get_clock().now())

        dt = t - self._prev_t
        if self._prev_t < 0.0:
            dt = 0.0

        # Update integrator
        self._int += 0.5 * (self.error_pose_euler - self._error_pose) * self._dt
        # Store current pose error
        self._error_pose = self.error_pose_euler

        # Get trajectory errors (reference - actual)
        e_p_linear_b = self._errors['pos']
        e_v_linear_b = self._errors['vel'][0:3]

        e_p_angular_b = self.error_orientation_rpy
        e_v_angular_b = self._errors['vel'][3:6]

        e_p_b = np.hstack((e_p_linear_b, e_p_angular_b))
        e_v_b = np.hstack((e_v_linear_b, e_v_angular_b))

        # Compute sliding surface s wrt body frame
        self._s_b = -e_v_b - np.multiply(self._lambda, e_p_b) \
                    - self._sliding_int * np.multiply(np.square(self._lambda)/4, self._int)

        # Acceleration estimate
        self._rotBtoI_dot = np.dot(cross_product_operator(self._vehicle_model._vel[3:6]), self._vehicle_model.rotBtoI)
        self._accel_linear_estimate_b = np.dot(
            self._vehicle_model.rotItoB, (self._reference['acc'][0:3] - \
                                          np.dot(self._rotBtoI_dot, self._vehicle_model._vel[0:3]))) + \
                                          np.multiply(self._lambda[0:3], e_v_linear_b) + \
                                          self._sliding_int * np.multiply(np.square(self._lambda[0:3]) / 4, e_p_linear_b)
        self._accel_angular_estimate_b = np.dot(self._vehicle_model.rotItoB, (self._reference['acc'][3:6] -
                                                np.dot(self._rotBtoI_dot, self._vehicle_model._vel[3:6]))) + \
                                                np.multiply(self._lambda[3:6], e_v_angular_b) + \
                                                self._sliding_int * np.multiply(np.square(self._lambda[3:6]) / 4,
                                                                                e_p_angular_b)
        self._accel_estimate_b = np.hstack((self._accel_linear_estimate_b, self._accel_angular_estimate_b))

        # Equivalent control
        acc = self._vehicle_model.to_SNAME(self._accel_estimate_b)
        self._f_eq = self._vehicle_model.compute_force(acc, use_sname=False)

        # Linear control
        self._f_lin = - np.multiply(self._k, self._s_b)

        # Uncertainties / disturbances upper boundaries for robust control
        self._rho_total = self._adaptive_bounds * self._rho_adapt + self._constant_bound * self._rho_constant

        # Adaptation law
        self._rho_adapt = self._rho_adapt + \
                          (self._adapt_slope[0] * np.abs(self._s_b) +
                          (self._adapt_slope[1] * np.abs(self._s_b) * np.abs(e_p_b) * np.abs(e_p_b)) +
                          (self._adapt_slope[2] * np.abs(self._s_b) * np.abs(e_v_b) * np.abs(e_v_b)) +
                           self._drift_prevent * (self._rho_0 - self._rho_adapt)) * dt

        # Robust control
        self._f_robust = - np.multiply(self._rho_total, (2 / np.pi) * np.arctan(np.multiply(self._c, self._s_b)))

        # Compute required forces and torques wrt body frame
        self._tau = self._ctrl_eq * self._f_eq + self._ctrl_lin * self._f_lin + self._ctrl_robust * self._f_robust

        self.publish_control_wrench(self._tau)

        self._prev_t = t
        

# =============================================================================
def main():
    print('Starting Model-based Sliding Mode Controller')
    rclpy.init()

    try:
        sim_time_param = is_sim_time()

        tf_world_ned_to_enu = get_world_ned_to_enu(sim_time_param)

        node = ROV_MB_SMController(
            'rov_mb_sm_controller', 
            parameter_overrides=[sim_time_param],
            world_ned_to_enu=tf_world_ned_to_enu)
        rclpy.spin(node)
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
