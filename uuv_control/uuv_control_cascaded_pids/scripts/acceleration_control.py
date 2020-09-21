#!/usr/bin/env python3
# Copyright (c) 2020 The Plankton Authors.
# All rights reserved.
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
import numpy
import rclpy

#from dynamic_reconfigure.server import Server
from geometry_msgs.msg import Accel
from geometry_msgs.msg import Wrench
#from rospy.numpy_msg import numpy_msg
from rclpy.node import Node

class AccelerationControllerNode(Node):
    def __init__(self, node_name):

        super().__init__(node_name,
                        allow_undeclared_parameters=True, 
                        automatically_declare_parameters_from_overrides=True)

        #Default sim_time to True
        sim_time = rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)
        self.set_parameters([sim_time])

        self.get_logger().info('AccelerationControllerNode: initializing node')

        self.ready = False
        self.mass = 1.
        self.inertial_tensor = numpy.identity(3)
        self.mass_inertial_matrix = numpy.zeros((6, 6))

        # ROS infrastructure
        self.sub_accel = self.create_subscription(Accel, 'cmd_accel', self.accel_callback, 10)
        self.sub_force = self.create_subscription(Accel, 'cmd_force', self.force_callback, 10)
        # self.sub_accel = self.create_subscription(
        #   numpy_msg(Accel), 'cmd_accel', self.accel_callback, 10)
        # self.sub_force = self.create_subscription(
        #   numpy_msg(Accel), 'cmd_force', self.force_callback, 10)
        self.pub_gen_force = self.create_publisher(Wrench, 'thruster_manager/input', 1)

        self.get_logger().info(str(self.get_parameters(['/'])))

        if not self.has_parameter("pid.mass"):
            raise RuntimeError("UUV's mass was not provided")

        # if not self.has_parameter("pid.inertial"):
        #     raise RuntimeError("UUV's inertial was not provided")

        self.mass = self.get_parameter("pid.mass").value
        self.inertial = self.get_parameters_by_prefix("pid.inertial")

        if len(self.inertial) == 0:
          raise RuntimeError("UUV's inertial was not provided")

        #self.get_logger().info(str(inertial))

        # update mass, moments of inertia
        self.inertial_tensor = numpy.array(
          [[self.inertial['ixx'].value, self.inertial['ixy'].value, self.inertial['ixz'].value],
           [self.inertial['ixy'].value, self.inertial['iyy'].value, self.inertial['iyz'].value],
           [self.inertial['ixz'].value, self.inertial['iyz'].value, self.inertial['izz'].value]])
        self.mass_inertial_matrix = numpy.vstack((
          numpy.hstack((self.mass*numpy.identity(3), numpy.zeros((3, 3)))),
          numpy.hstack((numpy.zeros((3, 3)), self.inertial_tensor))))

        self.get_logger().info(str(self.mass_inertial_matrix))
        self.ready = True

    #==============================================================================
    def force_callback(self, msg):
        if not self.ready:
            return

        # extract 6D force / torque vector from message
        force = numpy.array((
          msg.accel.linear.x, msg.accel.linear.y, msg.accel.linear.z))
        torque = numpy.array((
          msg.accel.angular.x, msg.accel.angular.y, msg.accel.angular.z))
        force_torque = numpy.hstack((force, torque)).transpose()

        force_msg = Wrench()
        force_msg.force.x = force[0]
        force_msg.force.y = force[1]
        force_msg.force.z = force[2]

        force_msg.torque.x = torque[0]
        force_msg.torque.y = torque[1]
        force_msg.torque.z = torque[2]

        self.pub_gen_force.publish(force_msg)

    #==============================================================================
    def accel_callback(self, msg):
        if not self.ready:
            return

        # extract 6D accelerations (linear, angular) from message
        linear = numpy.array((msg.linear.x, msg.linear.y, msg.linear.z))
        angular = numpy.array((msg.angular.x, msg.angular.y, msg.angular.z))
        accel = numpy.hstack((linear, angular)).transpose()

        # convert acceleration to force / torque
        force_torque = self.mass_inertial_matrix.dot(accel)

        force_msg = Wrench()
        force_msg.force.x = force_torque[0]
        force_msg.force.y = force_torque[1]
        force_msg.force.z = force_torque[2]

        force_msg.torque.x = force_torque[3]
        force_msg.torque.y = force_torque[4]
        force_msg.torque.z = force_torque[5]

        self.pub_gen_force.publish(force_msg)

#==============================================================================
def main():
  print('starting acceleration_control.py')
  #rospy.init_node('acceleration_control')

  rclpy.init()

  try:
      node = AccelerationControllerNode('acceleration_control')
      rclpy.spin(node)
  except Exception as e:
    print('Caught exception:' + str(e))
  finally:
    if rclpy.ok():
      rclpy.shutdown()
    print('Exiting')

#==============================================================================
if __name__ == '__main__':
    main()
