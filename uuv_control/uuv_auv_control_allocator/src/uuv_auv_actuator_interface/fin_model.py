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
from tf_quaternion.transformations import quaternion_matrix
from uuv_gazebo_ros_plugins_msgs.msg import FloatStamped


class FinModel:
    def __init__(self, index, pos, quat, topic, node:rclpy.node.Node):
        self.node = node
        self.id = index
        self.pos = pos
        self.quat = quat
        self.topic = topic
        self.rot = quaternion_matrix(quat)[0:3, 0:3]
    
        unit_z = self.rot[:, 2]
        # Surge velocity wrt vehicle's body frame
        surge_vel = np.array([1, 0, 0])
        fin_surge_vel = surge_vel - np.dot(surge_vel, unit_z) / np.linalg.norm(unit_z)**2 * unit_z        
        # Compute the lift and drag vectors
        self.lift_vector = -1 * np.cross(unit_z, fin_surge_vel) / np.linalg.norm(np.cross(unit_z, fin_surge_vel))
        self.drag_vector = -1 * surge_vel / np.linalg.norm(surge_vel)       
        
        self.pub = self.node.create_publisher(FloatStamped, self.topic, 3)
    
    # =========================================================================
    def publish_command(self, delta):
        msg = FloatStamped()
        msg.header.stamp = self.node.get_clock().now()
        msg.data = delta
        self.pub.publish(msg)
