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
from uuv_control_interfaces import DPPIDControllerBase
from plankton_utils.time import is_sim_time


class ROV_PIDController(DPPIDControllerBase):
    """PID controller for the dynamic positioning of ROVs."""

    _LABEL = 'PID'
    def __init__(self, name, **kwargs):
        self._tau = np.zeros(6)
        DPPIDControllerBase.__init__(self, name, False, **kwargs)
        self._is_init = True

    # =========================================================================
    def update_controller(self):
        if not self._is_init:
            return False
        # Update PID control action
        self._tau = self.update_pid()
        self.publish_control_wrench(self._tau)
        return True


# =============================================================================
def main():
    print('Starting PID')
    rclpy.init()

    try:
        sim_time_param = is_sim_time()

        tf_world_ned_to_enu = get_world_ned_to_enu(sim_time_param)

        node = ROV_PIDController(
            'rov_pid_controller',
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
