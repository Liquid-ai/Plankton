// Copyright (c) 2020 The Plankton Authors.
// All rights reserved.
//
// This source code is derived from UUV Simulator
// (https://github.com/uuvsimulator/uuv_simulator)
// Copyright (c) 2016-2019 The UUV Simulator Authors
// licensed under the Apache 2 license
// cf. 3rd-party-licenses.txt file in the root directory of this source tree.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/// \file SphericalCoordinatesROSInterfacePlugin.hh

#ifndef __SC_ROS_INTERFACE_PLUGIN_HH__
#define __SC_ROS_INTERFACE_PLUGIN_HH__

#include <gazebo/gazebo.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/SphericalCoordinates.hh>
#include <gazebo/physics/World.hh>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/vector3.hpp>

#include <gazebo_ros/node.hpp>

#include <uuv_world_ros_plugins_msgs/srv/set_origin_spherical_coord.hpp>
#include <uuv_world_ros_plugins_msgs/srv/get_origin_spherical_coord.hpp>
#include <uuv_world_ros_plugins_msgs/srv/transform_to_spherical_coord.hpp>
#include <uuv_world_ros_plugins_msgs/srv/transform_from_spherical_coord.hpp>

#include <map>
#include <string>

namespace gazebo
{

class SphericalCoordinatesROSInterfacePlugin : public WorldPlugin
{
  /// \brief Constructor
  public: SphericalCoordinatesROSInterfacePlugin();

  /// \brief Destructor
  public: virtual ~SphericalCoordinatesROSInterfacePlugin();

  /// \brief Load module and read parameters from SDF.
  public: void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf);

  /// \brief Service call that returns the origin in WGS84 standard
  public: bool GetOriginSphericalCoord(
      const uuv_world_ros_plugins_msgs::srv::GetOriginSphericalCoord::Request::SharedPtr _req,
      uuv_world_ros_plugins_msgs::srv::GetOriginSphericalCoord::Response::SharedPtr _res);

  /// \brief Service call that returns the origin in WGS84 standard
  public: bool SetOriginSphericalCoord(
      const uuv_world_ros_plugins_msgs::srv::SetOriginSphericalCoord::Request::SharedPtr _req,
      uuv_world_ros_plugins_msgs::srv::SetOriginSphericalCoord::Response::SharedPtr _res);

  /// \brief Service call to transform from Cartesian to spherical coordinates
  public: bool TransformToSphericalCoord(
      const uuv_world_ros_plugins_msgs::srv::TransformToSphericalCoord::Request::SharedPtr _req,
      uuv_world_ros_plugins_msgs::srv::TransformToSphericalCoord::Response::SharedPtr _res);

  /// \brief Service call to transform from spherical to Cartesian coordinates
  public: bool TransformFromSphericalCoord(
      const uuv_world_ros_plugins_msgs::srv::TransformFromSphericalCoord::Request::SharedPtr _req,
      uuv_world_ros_plugins_msgs::srv::TransformFromSphericalCoord::Response::SharedPtr _res);

  /// \brief Pointer to this ROS node's handle.
  protected: gazebo_ros::Node::SharedPtr myRosNode;

  /// \brief Connection for callbacks on update world.
  protected: event::ConnectionPtr rosPublishConnection;

  /// \brief Pointer to world
  protected: physics::WorldPtr world;

  /// \brief All underwater world services
  protected: std::map<std::string, rclcpp::ServiceBase::SharedPtr> worldServices;
};

}

#endif // __SC_ROS_INTERFACE_PLUGIN_HH__
