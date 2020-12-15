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
import sys
from tf_quaternion.transformations import quaternion_from_euler
from visualization_msgs.msg import Marker, MarkerArray
from gazebo_msgs.srv import GetEntityState
from rclpy.node import Node
from plankton_utils.param_helper import parse_nested_params_to_dict
from plankton_utils.time import is_sim_time


class WorldPublisher(Node):
    def __init__(self, name, **kwargs):
        super().__init__(name,
                        allow_undeclared_parameters=True, 
                        automatically_declare_parameters_from_overrides=True,
                        **kwargs)

        self.get_logger().info(name + ': start publishing vehicle footprints to RViz')

        self._model_paths = dict()

        try:
            # Handle for retrieving model properties
            service_name = '/gazebo/get_entity_state'
            self.get_entity_state = self.create_client(GetEntityState, service_name)
            ready = self.get_entity_state.wait_for_service(timeout_sec=100)
            if not ready:
                raise RuntimeError('service is unavailable')
        except RuntimeError:
            self.get_logger().error('%s service is unavailable' % service_name)
            sys.exit()
     
        meshes = self.get_parameters_by_prefix('meshes')
        if meshes != None:
            if type(meshes) != dict:
                raise RuntimeError('A list of mesh filenames is required')
    
            meshes = parse_nested_params_to_dict(meshes, '.')

            self.add_meshes(meshes)

        self._mesh_topic = self.create_publisher(MarkerArray, '/world_models', 1)

        self.timer = self.create_timer(10, self.publish_meshes)

    # =========================================================================
    def add_meshes(self, models):
        for model in models:
            if model in self._model_paths:
                self.get_logger().info('Model %s already exists' % model)
                continue

            new_model = dict()

            new_model['position'] = [0., 0., 0.]
            new_model['orientation'] = [0., 0., 0., 1.]
            new_model['scale'] = [1., 1., 1.]

            # It would probably be cleaner not to consider maps of maps, but rather 
            # to create a function to parse arg1.arg2.arg3
            if 'pose' in models[model]:
                if 'position' in models[model]['pose']:
                    val = models[model]['pose']['position'].value
                    if len(val) == 3:
                        new_model['position'] = models[model]['pose']['position'].value
                if 'orientation' in models[model]['pose']:
                    if len(models[model]['pose']['orientation'].value) == 3:
                        new_model['orientation'] = quaternion_from_euler(*models[model]['pose']['orientation'].value)

            if 'scale' in models[model]:
                if len(models[model]['scale'].value) == 3:
                    new_model['scale'] = models[model]['scale'].value

            if 'mesh' in models[model]:
                new_model['mesh'] = models[model]['mesh'].value

                if 'model' in models[model]:
                    model_name = models[model]['model'].value
                    req = GetEntityState.Request()
                    req._name = model_name
                    req.reference_frame = ''
                    self.get_logger().debug('Asking to: ' + self.get_entity_state.srv_name + 
                                            ' model: ' + model_name)
                    future = self.get_entity_state.call_async(req)
                    rclpy.spin_until_future_complete(self, future)
                    if future.result() is not None:
                        prop = future.result()
                        
                        if prop.success:
                            new_model['position'] = [prop.state.pose.position.x,
                                                    prop.state.pose.position.y,
                                                    prop.state.pose.position.z]
                            new_model['orientation'] = [prop.state.pose.orientation.x,
                                                        prop.state.pose.orientation.y,
                                                        prop.state.pose.orientation.z,
                                                        prop.state.pose.orientation.w]
                        else:
                            self.get_logger().info('Model %s not found in the current Gazebo scenario' % model)
                    else:
                        raise RuntimeError('Exception while calling service: %s' % future.exception())
                else:
                    self.get_logger().info('Information about the model %s for the mesh %s could not be '
                          'retrieved' % (model, models[model]['mesh'].value))
            elif 'plane' in models[model]:
                new_model['plane'] = [1, 1, 1]
                if 'plane' in models[model]:
                    if len(models[model]['plane'].value) == 3:
                        new_model['plane'] = models[model]['plane'].value
                    else:
                        self.get_logger().warn('Invalid scale vector for ' + model)
            else:
                continue

            self._model_paths[model] = new_model
            self.get_logger().info(
                '\nNew model being published: %s' % model +
                '\n\t Position: ' + str(self._model_paths[model]['position']) +
                '\n\t Orientation: ' + str(self._model_paths[model]['orientation'])+
                '\n\t Scale: ' + str(self._model_paths[model]['scale'])
            )

    # =========================================================================
    def publish_meshes(self):
        markers = MarkerArray()
        i = 0
        total_models = len(self._model_paths.keys())
        for model in self._model_paths:
            marker = Marker()

            if 'mesh' in self._model_paths[model]:
                marker.type = Marker.MESH_RESOURCE
                marker.mesh_resource = self._model_paths[model]['mesh']
                marker.scale.x = self._model_paths[model]['scale'][0]
                marker.scale.y = self._model_paths[model]['scale'][1]
                marker.scale.z = self._model_paths[model]['scale'][2]
            elif 'plane' in self._model_paths[model]:
                marker.type = Marker.CUBE
                marker.scale.x = self._model_paths[model]['plane'][0]
                marker.scale.y = self._model_paths[model]['plane'][1]
                marker.scale.z = self._model_paths[model]['plane'][2]

            marker.header.frame_id = 'world'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = ''
            marker.id = i
            marker.action = Marker.ADD
            marker.pose.position.x = self._model_paths[model]['position'][0]
            marker.pose.position.y = self._model_paths[model]['position'][1]
            marker.pose.position.z = self._model_paths[model]['position'][2]
            marker.pose.orientation.x = self._model_paths[model]['orientation'][0]
            marker.pose.orientation.y = self._model_paths[model]['orientation'][1]
            marker.pose.orientation.z = self._model_paths[model]['orientation'][2]
            marker.pose.orientation.w = self._model_paths[model]['orientation'][3]
            marker.color.a = 0.2
            marker.color.r = 0.0
            marker.color.g = 0.0
            marker.color.b = 1.0 - float(i) / total_models

            markers.markers.append(marker)
            i += 1

        self._mesh_topic.publish(markers)
        
        
# =============================================================================
def main():
    print('Start publishing vehicle footprints to RViz')

    rclpy.init()

    try:
        sim_time_param = is_sim_time()

        world_pub = WorldPublisher('publish_world_models', parameter_overrides=[sim_time_param])
        rclpy.spin(world_pub)
    except Exception as e:
        print('Caught exception: ' + str(e))
    print('Exiting')


# =============================================================================
if __name__ == '__main__':
    main()
