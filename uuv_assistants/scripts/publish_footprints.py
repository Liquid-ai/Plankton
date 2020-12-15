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
from __future__ import print_function
import rclpy
import rostopic
import rosgraph
import numpy as np
from copy import deepcopy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PolygonStamped, Point32
from tf_quaternion.transformations import euler_from_quaternion
from gazebo_msgs.srv import GetWorldProperties, GetModelProperties
from rclpy.node import Node

from plankton_utils.time import is_sim_time

# TODO Needs a rework
class FootprintsPublisher(Node):
    def __init__(self, name, **kwargs):
        super().__init__(name, **kwargs)

        self.vehicle_pub = dict()
        self.odom_sub = dict()
        self.get_world_props = None
        self.get_model_props = None
        self.marker = np.array([[0, 0.75], [-0.5, -0.25], [0.5, -0.25]])

        self.update_timer = self.create_timer(10, update_vehicle_list)
        #update_timer = rospy.Timer(rospy.Duration(10), update_vehicle_list)

    def rot(alpha):
        return np.array([[np.cos(alpha), -np.sin(alpha)],
                     [np.sin(alpha), np.cos(alpha)]])

    def odometry_callback(self, msg, name):
        #global vehicle_pub
        if name not in self.vehicle_pub:
            return
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        orientation = np.array([msg.pose.pose.orientation.x,
                                msg.pose.pose.orientation.y,
                                msg.pose.pose.orientation.z,
                                msg.pose.pose.orientation.w])

        yaw = euler_from_quaternion(orientation)[2]

        new_marker = deepcopy(marker)
        points = list()
        for i in range(new_marker.shape[0]):
            new_marker[i, :] = np.dot(rot(yaw-np.pi/2), new_marker[i, :])
            new_marker[i, 0] += x
            new_marker[i, 1] += y

            p = Point32()
            p.x = new_marker[i, 0]
            p.y = new_marker[i, 1]
            points.append(p)

        new_poly = PolygonStamped()
        new_poly.header.stamp = self.get_clock().now().to_msg() #rospy.Time.now()
        new_poly.header.frame_id = 'world'
        new_poly.polygon.points = points
        self.get
        vehicle_pub[name].publish(new_poly)

    def get_topics(self, name):
        #TODO Not sure if correct...
        return [t, type for t, type in self.get_topic_names_and_types() if not t.startswith('/rosout') and name in t]
        #return [t for t, _ in rosgraph.Master('/{}/'.format(name)).getPublishedTopics('') if not t.startswith('/rosout') and name in t]

    def sub_odometry_topic(self, name):
        odom_topic_sub = None
        for t, msg_class in get_topics(name):
            #msg_class, _, _ = rostopic.get_topic_class(t)
            if len(msg_class) != 1:
                self.get_logger().info(
                        "Warning: ignoring topic '%s', which has more than one type: [%s]"
                        % (t, ', '.join(msg_class)))
            if msg_class[0] == Odometry:
                odom_topic_sub = self.create_subscription(
                    Odometry, t, lambda msg: odometry_callback(msg, name), 10)
        return odom_topic_sub

    def update_vehicle_list(self):
        """Call list of models in the Gazebo simulation and filter out the
        marine crafts.
        """
        #global get_world_props
        if get_world_props is None:
            try:
                # Handle for world properties update function
                self.get_world_props = self.create_client(GetWorldProperties, '/gazebo/get_world_properties')
                while not self.get_world_props.wait_for_service(timeout_sec=2):
                    self.get_logger().info('waiting for /gazebo/get_world_properties service')
            except:
                print('/gazebo/get_world_properties service is unavailable')

        #global get_model_props
        if get_model_props is None:
            try:
                # Handle for retrieving model properties
                self.get_model_props = self.create_client(GetModelProperties, '/gazebo/get_model_properties')
                while not self.get_model_props.wait_for_service(timeout_sec=2):
                    self.get_logger().info('waiting for /gazebo/get_model_properties service')
            except:
                print('/gazebo/get_model_properties service is unavailable')
        try:
            #global vehicle_pub
            #global odom_sub
            #What happens if request is slower than this callback rate call ?
            reqW = GetWorldProperties.Request()
            msg = self.get_world_props.call(reqW)
            #msg = self.get_world_props()
            for model in msg.model_names:
                reqM = GetModelProperties.Request()
                reqM.model_name = model
                model_properties = self.get_model_props.call(reqM)
                #model_properties = self.get_model_props(model)
                if not model_properties.is_static and \
                self.has_parameter('/{}/robot_description'.format(model)) and \
                model not in self.vehicle_pub:
                self.odom_sub[model] = sub_odometry_topic(model)
                self.vehicle_pub[model] = self.create_publisher(PolygonStamped, '/{}/footprint'.format(model), 1)
        except Exception as e:
            print('Service call failed: {}'.format(e))

def main():
    print('Start publishing vehicle footprints to RViz')
    rclpy.init()

    try:
        sim_time_param = is_sim_time()
        
        node = FootprintsPublisher('publish_footprints', parameter_overrides=[sim_time_param])
        rclpy.spin(node)
    except rclpy.exceptions.ROSInterruptException:
        print('caught exception')
    print('exiting')


if __name__ == '__main__':
    main()
