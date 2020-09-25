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

import logging
import numpy as np
import sys
import threading

import rclpy
from uuv_world_ros_plugins_msgs.srv import *
from gazebo_msgs.srv import ApplyLinkWrench
from uuv_gazebo_ros_plugins_msgs.srv import SetThrusterState, SetThrusterEfficiency
from geometry_msgs.msg import Point, WrenchStamped, Vector3
from rclpy.node import Node

from plankton_utils.param_helper import parse_nested_params_to_dict
from plankton_utils.time import time_in_float_sec, float_sec_to_int_sec_nano


class DisturbanceManager(Node):

    def __init__(self, node_name):
        super().__init__(node_name,
                        allow_undeclared_parameters=True, 
                        automatically_declare_parameters_from_overrides=True)

        sim_time = rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)
        self.set_parameters([sim_time])

        self._logger = logging.getLogger('dp_local_planner')
        out_hdlr = logging.StreamHandler(sys.stdout)
        out_hdlr.setFormatter(logging.Formatter('%(asctime)s | %(levelname)s | %(module)s | %(message)s'))
        out_hdlr.setLevel(logging.INFO)
        self._logger.addHandler(out_hdlr)
        self._logger.setLevel(logging.INFO)

        self.thread = threading.Thread(target=rclpy.spin, args=(self,), daemon=True)
        self.thread.start()

        # Load disturbances and check for missing information
        specs = dict(current=['starting_time', 'velocity', 'horizontal_angle',
                              'vertical_angle'],
                     wrench=['starting_time', 'duration', 'force', 'torque'],
                     thruster_state=['starting_time', 'thruster_id', 'is_on',
                                     'duration'],
                     propeller_efficiency=['starting_time', 'thruster_id', 'duration',
                                    'efficiency'],
                     thrust_efficiency=['starting_time', 'thruster_id', 'duration',
                                 'efficiency'])

        thruster_ids = list()

        #if self.has_parameter('disturbances'):
        #self._logger.info(str(self.get_parameters(['/'])))
        self._disturbances = self.get_parameters_by_prefix('disturbances')
        self._disturbances = parse_nested_params_to_dict(self._disturbances, unpack_value=True)
        if self._disturbances != {}:
        #if type(self._disturbances) != list:
        #    raise RuntimeError('Current specifications must be '
        #                            'given as a list of dict')
            #for i in range(len(self._disturbances)):
            for key in self._disturbances.keys():
                item = self._disturbances[key]
                if type(item) != dict:
                    raise RuntimeError('Disturbance description must be'
                                                ' given as a dict')
                if 'type' not in item:
                    raise RuntimeError('Type of disturbance not '
                                                'specified')
                if item['type'] not in specs:
                    raise RuntimeError(
                            'Invalid type of disturbance, value=%s' % item['type'])
                for spec in specs[item['type']]:
                    if spec not in item:
                        raise RuntimeError(
                                'Invalid current model specification, '
                                'missing tag=%s' % spec)
            
                if item['type'] == 'thruster_state':
                    thruster_ids.append(item['thruster_id'])

                # Create flag to indicate that perturbation has been applied
                self._disturbances[key]['is_applied'] = False
                self._disturbances[key]['ended'] = False

        else:
            raise RuntimeError('No disturbance specifications given')

        # List all disturbances to be applied
        #for i in range(len(self._disturbances)):
        cpt = 0
        for key in self._disturbances.keys():
            self._logger.info('Disturbance #%d: %s' % (cpt, self._disturbances[key]))
            cpt = cpt + 1

        self._body_force = np.zeros(3)
        self._body_torque = np.zeros(3)
        self._body_wrench_msg = WrenchStamped()

        # For body wrench disturbances, publish a topic
        self._wrench_topic = self.create_publisher(
            WrenchStamped, 'wrench_perturbation', 1)

        vehicle_name = self.get_namespace().replace('/', '')

        # Obtain service proxy
        self._service_cb = dict()
        service_list = list()
        try:
            service_list.append(self.create_client(
                SetCurrentVelocity, '/hydrodynamics/set_current_velocity'))
            self._service_cb['current_velocity'] = service_list[-1]
            service_list.append(self.create_client(
                ApplyBodyWrench, '/gazebo/apply_body_wrench'))
            self._service_cb['wrench'] = service_list[-1]
         
            self._service_cb['thrusters'] = dict()          
            for item in self._disturbances.values():
                if item['type'] == 'thruster_state':
                    thruster_id = item['thruster_id']
                    if 'state' not in self._service_cb['thrusters']:
                        self._service_cb['thrusters']['state'] = dict()
                    service_list.append(self.create_client(
                        SetThrusterState,
                        create_service_name(vehicle_name, thruster_id, 'set_thruster_state')))
                        #'/%s/thrusters/%d/set_thruster_state' % (vehicle_name, item['thruster_id'])))
                    self._service_cb['thrusters']['state'][thruster_id] = service_list[-1]
                elif item['type'] == 'propeller_efficiency':
                    if 'propeller_efficiency' not in self._service_cb['thrusters']:
                        self._service_cb['thrusters']['propeller_efficiency'] = dict()
                    service_list.append(self.create_client(
                        SetThrusterEfficiency,
                        create_service_name(vehicle_name, thruster_id, 'set_dynamic_state_efficiency')))
                        #'/%s/thrusters/%d/set_dynamic_state_efficiency' % (vehicle_name, item['thruster_id'])))
                    self._service_cb['thrusters']['propeller_efficiency'][thruster_id] = service_list[-1]
                elif item['type'] == 'thrust_efficiency':
                    if 'thrust_efficiency' not in self._service_cb['thrusters']:
                        self._service_cb['thrusters']['thrust_efficiency'] = dict()
                    service_list.append(self.create_client(
                        SetThrusterEfficiency,
                        create_service_name(vehicle_name, thruster_id, 'set_thrust_force_efficiency')))
                        #'/%s/thrusters/%d/set_thrust_force_efficiency' % (vehicle_name, item['thruster_id'])))
                    self._service_cb['thrusters']['thrust_efficiency'][thruster_id] = service_list[-1]
        except Exception as e:
            self._logger.info('Service call failed, error=%s' % str(e))
            sys.exit(-1)
            
        # Test if services are reachable
        try:
            #self._service_cb.values()
            # services = ['/hydrodynamics/set_current_velocity',
            #             '/gazebo/apply_body_wrench']
            # for item in self._disturbances:
            #     if item['type'] == 'thruster_state':
            #         services.append('/%s/thrusters/%d/set_thruster_state' % (vehicle_name, item['thruster_id']))
            #     elif item['type'] == 'propeller_efficiency':
            #         services.append('/%s/thrusters/%d/set_dynamic_state_efficiency' % (vehicle_name, item['thruster_id']))
            #     elif item['type'] == 'thrust_efficiency':
            #         services.append('/%s/thrusters/%d/set_thrust_force_efficiency' % (vehicle_name, item['thruster_id']))
            for s in service_list:
                ready = s.wait_for_service(timeout_sec=10)
                if not ready:
                    raise RuntimeError('Service %s not ready' % (s.srv_name))
        except Exception as e:
            self._logger.error('Some services are not available! message=' + str(e))
            self._logger.error('Closing node...')
            sys.exit(-1)
       
        self._wrench_timer = self.create_timer(0.1, self._publish_wrench_disturbance)
        
        #rate = self.create_rate(100)
        #FREQ = 100
        rate = node.create_rate(100)
        while rclpy.ok():
            t = time_in_float_seconds(self.get_clock().now())
            #for i in range(len(self._disturbances)):
            for d in self._disturbances.values():
                #d = self._disturbances[i]
                if t > d['starting_time'] and not d['is_applied']:
                    ###########################################################
                    if d['type'] == 'current':
                        self.set_current(d['velocity'], d['horizontal_angle'],
                                         d['vertical_angle'])
                    ###########################################################
                    elif d['type'] == 'wrench':
                        self.set_link_wrench(d['force'],
                                             d['torque'],
                                             -1,
                                             d['starting_time'])
                    ###########################################################
                    elif d['type'] == 'thruster_state':
                        self.set_thruster_state(d['thruster_id'], bool(d['is_on']))
                    ###########################################################
                    elif d['type'] == 'propeller_efficiency':
                        self.set_propeller_efficiency(d['thruster_id'], d['efficiency'])
                    ###########################################################
                    elif d['type'] == 'thrust_efficiency':
                        self.set_thrust_efficiency(d['thruster_id'], d['efficiency'])
                    # Set applied flag to true
                    #self._disturbances[i]['is_applied'] = True
                    d['is_applied'] = True

                    if 'duration' in d:
                        if d['duration'] == -1:
                            #self._disturbances[i]['ended'] = True
                            d['ended'] = True
                    else:
                        #self._disturbances[i]['ended'] = True
                        d['ended'] = True
                elif d['is_applied'] and 'duration' in d and not d['ended']:
                    if d['duration'] > 0:
                        if self.get_clock().now().nanoseconds > int((d['starting_time'] + d['duration']) * 1e9):
                            ###########################################################
                            if d['type'] == 'current':
                                # Set current to zero
                                self.set_current(0, d['horizontal_angle'],
                                                 d['vertical_angle'])
                            ###########################################################
                            elif d['type'] == 'wrench':
                                # Cancel out force and torque
                                self.set_link_wrench([-1 * d['force'][n] for n in range(3)],
                                                     [-1 * d['torque'][n] for n in range(3)],
                                                     -1,
                                                     time_in_float_sec(self.get_clock().now()))
                            ###########################################################
                            elif d['type'] == 'thruster_state':
                                self.set_thruster_state(d['thruster_id'], not bool(d['is_on']))
                            ###########################################################
                            elif d['type'] == 'propeller_efficiency':
                                self.set_propeller_efficiency(d['thruster_id'], 1.0)
                            ###########################################################
                            elif d['type'] == 'thrust_efficiency':
                                self.set_thrust_efficiency(d['thruster_id'], 1.0)

                            #self._disturbances[i]['ended'] = True
                            d['ended'] = True
            rate.sleep()

    # =========================================================================
    def __del__(self):
        if rclpy.ok(): 
            self.destroy_node()
            rclpy.shutdown()
            self.thread.join()

    # =========================================================================
    def _publish_wrench_disturbance(self, event):
        msg = WrenchStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'world'
        msg.wrench.force = Vector3(x=self._body_force[0], y=self._body_force[1], z=self._body_force[2])
        msg.wrench.torque = Vector3(x=self._body_torque[0], y=self._body_torque[1], z=self._body_torque[2])
        # Publish the applied body wrench
        self._wrench_topic.publish(msg)
        return True

    # =========================================================================
    def set_current(self, velocity, horizontal_angle, vertical_angle):
        self._logger.info('Appying current velocity model...')
        req = SetCurrentVelocity.Request()
        req.velocity = velocity
        req.horizontal_angle = horizontal_angle
        req.vertical_angle = vertical_angle

        future = self._service_cb['current_velocity'].call_async(req)

        self.wait_for_service_completion(
            future,
            'Current velocity changed successfully at %f s! vel= %f m/s' % (time_in_float_sec(self.get_clock().now()), velocity),
            'Failed to change current velocity')
      
        # if self._service_cb['current_velocity'].call(req):
        #     self._logger.info('Current velocity changed successfully at %f s! vel= %f m/s' % (time_in_float_sec(self.get_clock().now()), velocity))
        # else:
        #     self._logger.error('Failed to change current velocity')

    # =========================================================================
    def set_link_wrench(self, force, torque, duration, starting_time):
        ns = self.get_namespace().replace('/', '')
        body_name = '%s/base_link' % ns

        req = ApplyLinkWrench.Request()

        self._body_force = np.array([self._body_force[i] + force[i] for i in range(3)])
        self._body_torque = np.array([self._body_torque[i] + torque[i] for i in range(3)])

        self._body_wrench_msg = WrenchStamped()
        self._body_wrench_msg.header.stamp = self.get_clock().now().to_msg()
        self._body_wrench_msg.header.frame_id = 'world'
        self._body_wrench_msg.wrench.force = Vector3(x=self._body_force[0], 
                                                     y=self._body_force[1], 
                                                     z=self._body_force[2])
        self._body_wrench_msg.wrench.torque = Vector3(x=self._body_torque[0], 
                                                      y=self._body_torque[1], 
                                                      z=self._body_torque[2])

        (secs, nsecs) = float_sec_to_int_sec_nano(starting_time)
        (d_secs, d_nsecs) = float_sec_to_int_sec_nano(duration)
        req.body_name = body_name
        req.reference_frame = 'world'
        req.reference_point = Point(0, 0, 0)
        req.wrench = self._body_wrench_msg.wrench
        req.start_time = rclpy.time.Time(seconds=secs, nanoseconds=nsecs).to_msg()
        req.duration = rclpy.time.Duration(seconds=d_secs, nanoseconds=d_nsecs).to_msg()

        future = self._service_cb['wrench'].call_async(req)
        wait_for_service_completion(
            future,
            'Link wrench perturbation applied!, body_name=%s, t=%.2f s' % (body_name, time_in_float_sec(self.get_clock().now())),
            'Failed to apply link wrench!, body_name=%s, t=%.2f s' % (body_name, time_in_float_sec(self.get_clock().now())))

        # if success:
        #     self._logger.info('Body wrench perturbation applied!, body_name=%s, t=%.2f s' % (body_name, time_in_float_sec(self.get_clock().now())))
        # else:
        #     self._logger.error('Failed to apply body wrench!, body_name=%s, t=%.2f s' % (body_name, time_in_float_sec(self.get_clock().now())))

    # =========================================================================
    def set_thruster_state(self, thruster_id, is_on):
        req = SetThrusterState.Request()
        req.on = is_on

        future = self._service_cb['thrusters']['state'][thruster_id].call_async(req)
        time = time_in_float_sec(self.get_clock().now())
        wait_for_service_completion(future,
            'Setting state of thruster #%d, state=%s, t=%.2f s' % (thruster_id, 'ON' if is_on else 'OFF', time),
            'Setting state of thruster #%d failed! t=%.2f s' % (thruster_id, time))

        # if self._service_cb['thrusters']['state'][thruster_id].call(req):
            
        #     self._logger.info('Setting state of thruster #%d, state=%s, t=%.2f s' % (thruster_id, 'ON' if is_on else 'OFF', time))
        # else:
        #     time = time_in_float_sec(self.get_clock().now())
        #     self._logger.error('Setting state of thruster #%d failed! t=%.2f s' % (thruster_id, time))

    # =========================================================================
    def set_propeller_efficiency(self, thruster_id, eff):
        req = SetThrusterEfficiency.Request()
        req.efficiency = eff

        future = self._service_cb['thrusters']['propeller_efficiency'][thruster_id].call_async(req)
        time = time_in_float_sec(self.get_clock().now())
        wait_for_service_completion(future,
            'Setting propeller efficiency of thruster #%d, eff=%s, t=%.2f s' % (thruster_id, eff, time),
            'Setting propeller efficiency of thruster #%d failed! t=%.2f s' % (thruster_id, time))

        # if self._service_cb['thrusters']['propeller_efficiency'][thruster_id].call(req):
        #     time = time_in_float_sec(self.get_clock().now())
        #     self._logger.info('Setting propeller efficiency of thruster #%d, eff=%s, t=%.2f s' % (thruster_id, eff, time))
        # else:
        #     time = time_in_float_sec(self.get_clock().now())
        #     self._logger.error('Setting propeller efficiency of thruster #%d failed! t=%.2f s' % (thruster_id, time))

    # =========================================================================
    def set_thrust_efficiency(self, thruster_id, eff):
        req = SetThrusterEfficiency.Request()
        req.efficiency = eff

        future = self._service_cb['thrusters']['thrust_efficiency'][thruster_id].call_async(req)
        time = time_in_float_sec(self.get_clock().now())
        wait_for_service_completion(
            future,
            'Setting thrust efficiency of thruster #%d, eff=%s, t=%.2f s' % (thruster_id, eff, time),
            'Setting thrust efficiency of thruster #%d failed! t=%.2f s' % (thruster_id, time))

        # if self._service_cb['thrusters']['thrust_efficiency'][thruster_id].call(req):
        #     time = time_in_float_sec(self.get_clock().now())
        #     self._logger.info('Setting thrust efficiency of thruster #%d, eff=%s, t=%.2f s' % (thruster_id, eff, time))
        # else:
        #     time = time_in_float_sec(self.get_clock().now())
        #     self._logger.error('Setting thrust efficiency of thruster #%d failed! t=%.2f s' % (thruster_id, time))

    # =========================================================================
    def wait_for_service_completion(future, success_msg, error_msg):
        while not future.done():
            try:
                response = future.result()
            except Exception as e:
                self._logger.error(error_msg)
            else:
                self._logger.info(success_msg)

    # =========================================================================
    def build_service_name(self, ns, thruster_id, service_name) -> str :
        return '/%s/thrusters/id_%d/%s' % (ns, thruster_id, service_name)


# =============================================================================
def main():
    print('Starting disturbance manager')
    
    rclpy.init()

    try:
        node = DisturbanceManager('disturbance_manager')
        rclpy.spin(node)
    except rclpy.exceptions.ROSInterruptException as rosInter:
        print('Caught ROSInterruptException exception' + str(rosInter))
    except Exception as e:
        print('Caught exception: '+str(e))
    # finally:
    #     if rclpy.ok():
    #         rclpy.shutdown()
            # node.destroy_node()
            # node.thread.join()
            
    print('exiting')


# =============================================================================
if __name__ == '__main__':
    main()
