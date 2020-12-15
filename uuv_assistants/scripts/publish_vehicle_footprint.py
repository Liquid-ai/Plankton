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
from copy import deepcopy
from tf_quaternion.transformations import euler_from_quaternion
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PolygonStamped, Point32
from visualization_msgs.msg import Marker
import numpy as np
from rclpy.node import Node

from plankton_utils.time import is_sim_time

class VehicleFootprint(Node):
    MARKER = np.array([[0, 0.75], [-0.5, -0.25], [0.5, -0.25]])
    
    #==========================================================================
    def __init__(self, name, **kwargs):
        super().__init__(name,
                        allow_undeclared_parameters=True, 
                        automatically_declare_parameters_from_overrides=True,
                        **kwargs)

        self.get_logger().info('Generate RViz footprint and markers for 2D visualization')

        #Default sim_time to True
        # sim_time = rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)
        # self.set_parameters([sim_time])

        self._namespace = self.get_namespace().replace('/', '')

        self._scale_footprint = 10

        if self.has_parameter('scale_footprint'):
            scale = self.get_parameter('scale_footprint').get_parameter_value().double_value
            if scale > 0:
                self._scale_footprint = scale
            else:
                self.get_logger().error('Scale factor should be greater than zero')
        
        self.get_logger().info('Footprint marker scale factor= ' + str(self._scale_footprint))

        self._scale_label = 10

        if self.has_parameter('_scale_label'):
            scale = self.get_parameter('_scale_label').get_parameter_value().double_value
            if scale > 0:
                self._scale_label = scale
            else:
                self.get_logger().error('Scale factor should be greater than zero')
        
        self.get_logger().info('Label marker scale factor = ' + str(self._scale_label))

        self._label_x_offset = 60
        if self.has_parameter('label_x_offset'):
            self._label_x_offset = self.get_parameter('label_x_offset').get_parameter_value().double_value
            
        self._label_marker = Marker()
        self._label_marker.header.frame_id = 'world'
        self._label_marker.header.stamp = self.get_clock().now().to_msg()
        self._label_marker.ns = self._namespace
        self._label_marker.type = Marker.TEXT_VIEW_FACING
        self._label_marker.text = self._namespace
        self._label_marker.action = Marker.ADD
        self._label_marker.pose.orientation.x = 0.0
        self._label_marker.pose.orientation.y = 0.0
        self._label_marker.pose.orientation.z = 0.0
        self._label_marker.pose.orientation.w = 1.0
        self._label_marker.scale.x = 0.0
        self._label_marker.scale.y = 0.0
        self._label_marker.scale.z = self._scale_label
        self._label_marker.color.a = 1.0
        self._label_marker.color.r = 0.0
        self._label_marker.color.g = 1.0
        self._label_marker.color.b = 0.0

        # Odometry subscriber (remap this topic in the launch file if necessary)
        self._odom_sub = self.create_subscription(Odometry, 'odom', self.odometry_callback, 1)
        # Footprint marker publisher (remap this topic in the launch file if necessary)
        self._footprint_pub = self.create_publisher(PolygonStamped, 'footprint', 1)
        # Vehicle label marker (remap this topic in the launch file if necessary)
        self._label_pub = self.create_publisher(Marker, 'label', 1)
    
    # =========================================================================
    @staticmethod
    def rot(alpha):
        return np.array([[np.cos(alpha), -np.sin(alpha)],
                         [np.sin(alpha), np.cos(alpha)]])

    #==========================================================================
    def odometry_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        orientation = np.array([msg.pose.pose.orientation.x,
                                msg.pose.pose.orientation.y,
                                msg.pose.pose.orientation.z,
                                msg.pose.pose.orientation.w])

        yaw = euler_from_quaternion(orientation)[2]

        # Generating the vehicle footprint marker
        new_marker = self._scale_footprint * deepcopy(self.MARKER)

        points = list()
        for i in range(new_marker.shape[0]):
            new_marker[i, :] = np.dot(self.rot(yaw - np.pi / 2), new_marker[i, :])
            new_marker[i, 0] += x
            new_marker[i, 1] += y

            p = Point32()
            p.x = new_marker[i, 0]
            p.y = new_marker[i, 1]
            points.append(p)

        new_poly = PolygonStamped()
        new_poly.header.stamp = self.get_clock().now()
        new_poly.header.frame_id = 'world'
        new_poly.polygon.points = points

        self._footprint_pub.publish(new_poly)

        # Generating the label marker
        self._label_marker.pose.position.x = msg.pose.pose.position.x + self._label_x_offset
        self._label_marker.pose.position.y = msg.pose.pose.position.y
        self._label_marker.pose.position.z = msg.pose.pose.position.z 

        self._label_pub.publish(self._label_marker)


# =============================================================================
def main():
    print('Generate RViz footprint and markers for 2D visualization')
    #TODO Add rclpy instruction

    try:
        sim_time_param = is_sim_time()
        
        world_pub = WorldPublisher('publish_world_models', parameter_overrides=[sim_time_param])
        node = VehicleFootprint('generate_vehicle_footprint')
        rclpy.spin(node)
    except rclpy.exceptions.ROSInterruptException as e:
        print('Caught exception: ' + str(e))
    print('Exiting')

#==============================================================================
if __name__ == '__main__':
    main()
