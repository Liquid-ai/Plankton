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
from .thruster import Thruster


class ThrusterCustom(Thruster):
    """Class describing a custom conversion curve between the command input,
    usually the angular velocity, and the correspondent output thrust force.
    Here the inverse of the conversion function can be computed so that the
    command for the desired thrust force is retrieved.
    The input vector corresponds to sampled values for the command input, and
    the output vector corresponds to the sampled values for the correspondent
    thrust forces.
    This information is usually available in the datasheet of the thruster's
    manufacturer.

    > *Input arguments*
    
    * `index` (*type:* `int`): Thruster's ID.
    * `topic` (*type:* `str`): Thruster's command topic.
    * `pos` (*type:* `numpy.array` or `list`): Position vector 
    of the thruster with respect to the vehicle's frame.
    * `orientation` (*type:* `numpy.array` or `list`): Quaternion 
    with the orientation of the thruster with respect to the vehicle's
    frame as `(qx, qy, qz, qw)`.
    * `axis` (*type:* `numpy.array`): Axis of rotation of the thruster.
    * `input` (*type:* `list` or `numpy.array`): Vector samples of 
    angular velocities to be interpolated with the vector samples
    of thrust force output.
    * `output` (*type:* `list` or `numpy.array`): Vector samples
    of thrust force output.
    """
    LABEL = 'custom'

    def __init__(self, node, *args, **kwargs):
        """Class constructor."""
        super(ThrusterCustom, self).__init__(node, *args)

        if 'input' not in kwargs or 'output' not in kwargs:
            RuntimeError('Thruster input/output sample points not given')
        # Vector of sample values for each angular velocity
        self._input = kwargs['input']
        # Vector of sample values for each thrust force relative to the input
        self._output = kwargs['output']

    #==============================================================================
    def get_command_value(self, thrust):
        """Compute the angular velocity necessary 
        for the desired thrust force.
        
        > *Input arguments*
        
        * `thrust` (*type:* `float`): Thrust force magnitude in N
        
        > *Returns*
        
        `float`: Angular velocity set-point for the thruster in rad/s 
        """
        return numpy.interp(thrust, self._output, self._input)

    #==============================================================================
    def get_thrust_value(self, command):
        """Computes the thrust force for the given angular velocity
        set-point.
        
        > *Input arguments*
        
        * `command` (*type:* `float`): Angular velocity set-point for 
        the thruster in rad/s 
        
        > *Returns*

        `thrust` (*type:* `float`): Thrust force magnitude in N
        """
        return numpy.interp(command, self._input, self._output)
