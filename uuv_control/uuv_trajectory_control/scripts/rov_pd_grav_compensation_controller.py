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
from plankton_utils.time import is_sim_time

class ROV_PD_GComp_Controller(DPControllerBase):
    """
    PD controller with compensation of restoring forces for the dynamic 
    positioning of ROVs.
    """

    _LABEL = 'PD controller with compensation of restoring forces'
    def __init__(self, name, **kwargs):
        # Start the super class
        DPControllerBase.__init__(self, name, is_model_based=True, **kwargs)
        self._logger.info('Initializing: ' + self._LABEL)
        # Proportional gains
        self._Kp = np.zeros(shape=(6, 6))
        # Derivative gains
        self._Kd = np.zeros(shape=(6, 6))
        # Error for the vehicle pose
        self._error_pose = np.zeros(6)

        self._tau = np.zeros(6)

        if self.has_parameter('Kp'):
            Kp_diag = self.get_parameter('Kp').value
            if len(Kp_diag) == 6:
                self._Kp = np.diag(Kp_diag)
            else:
                raise RuntimeError('Kp matrix error: 6 coefficients '
                                         'needed')

        self._logger.info('Kp=' + str([self._Kp[i, i] for i in range(6)]))

        if self.has_parameter('Kd'):
            Kd_diag = self.get_parameter('Kd').value
            if len(Kd_diag) == 6:
                self._Kd = np.diag(Kd_diag)
            else:
                raise RuntimeError('Kd matrix error: 6 coefficients '
                                         'needed')

        self._logger.info('Kd=' + str([self._Kd[i, i] for i in range(6)]))
                
        self._is_init = True
        self._logger.info(self._LABEL + ' ready')

    # =========================================================================
    def _reset_controller(self):
        super(DPControllerBase, self)._reset_controller()
        self._error_pose = np.zeros(6)

    # =========================================================================
    def update_controller(self):
        if not self._is_init:
            return False
        
        self._vehicle_model._update_restoring(use_sname=True)

        self._tau = np.dot(self._Kp, self.error_pose_euler) \
            + np.dot(self._Kd, self._errors['vel']) \
            + self._vehicle_model.restoring_forces            

        self.publish_control_wrench(self._tau)


# =============================================================================
def main():
    print('Starting PD controller with compensation of restoring forces')
    rclpy.init()

    try:
        sim_time_param = is_sim_time()

        tf_world_ned_to_enu = get_world_ned_to_enu(sim_time_param)

        node = ROV_PD_GComp_Controller(
            'rov_pd_grav_compensation_controller',
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

# =============================================================================
if __name__ == '__main__':
    main()
