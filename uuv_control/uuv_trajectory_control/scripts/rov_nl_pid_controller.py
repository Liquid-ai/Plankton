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

from geometry_msgs.msg import Wrench, Vector3

from utils.transform import get_world_ned_to_enu
from uuv_control_interfaces import DPPIDControllerBase
from tf_quaternion.transformations import quaternion_matrix
from plankton_utils.time import is_sim_time


class ROV_NLPIDController(DPPIDControllerBase):
    """MIMO Nonlinear PID controller with acceleration feedback for the dynamic
    positioning of underwater vehicles.

    References
    ----------

    - Fossen, Thor I. Handbook of Marine Craft Hydrodynamics and Motion
    Control (April 8, 2011).
    """

    _LABEL = 'MIMO Nonlinear PID Controller with Acceleration Feedback'

    def __init__(self, name, **kwargs):
        DPPIDControllerBase.__init__(self, name, True, **kwargs)
        self._logger.info('Initializing: ' + self._LABEL)
        # Feedback acceleration gain
        self._Hm = np.eye(6)
        if self.has_parameter('Hm'):
            hm = self.get_parameter('Hm').value
            if len(hm) == 6:
                self._Hm = self._vehicle_model.Mtotal +  np.diag(hm)
            else:
                raise RuntimeError('Invalid feedback acceleration gain coefficients')

        self._tau = np.zeros(6)
        # Acceleration feedback term
        self._accel_ff = np.zeros(6)
        # PID control vector
        self._pid_control = np.zeros(6)
        self._is_init = True
        self._logger.info(self._LABEL + ' ready')

    # =========================================================================
    def _reset_controller(self):
        super(ROV_NLPIDController, self)._reset_controller()
        self._accel_ff = np.zeros(6)
        self._pid_control = np.zeros(6)

    # =========================================================================
    def update_controller(self):
        if not self._is_init:
            return False
        # Calculating the feedback acceleration vector for the control forces
        # from last iteration
        acc = self._vehicle_model.compute_acc(
            self._vehicle_model.to_SNAME(self._tau), use_sname=False)
        self._accel_ff = np.dot(self._Hm, acc)
        # Update PID control action
        self._pid_control = self.update_pid()
        # Publish control forces and torques
        self._tau = self._pid_control - self._accel_ff + \
            self._vehicle_model.restoring_forces
        self.publish_control_wrench(self._tau)
        return True


# =============================================================================
def main():
    rclpy.init()

    try:
        sim_time_param = is_sim_time()

        tf_world_ned_to_enu = get_world_ned_to_enu(sim_time_param)

        node = ROV_NLPIDController(
            'rov_nl_pid_controller', 
            world_ned_to_enu=tf_world_ned_to_enu,
            parameter_overrides=[sim_time_param])
        rclpy.spin(node)
    except Exception as e:
        print('Caught exception: ' + repr(e))
        traceback.print_exc()
    finally:
        if rclpy.ok():
            rclpy.shutdown()
    print('exiting')    


# =============================================================================
if __name__ == '__main__':
    main()
