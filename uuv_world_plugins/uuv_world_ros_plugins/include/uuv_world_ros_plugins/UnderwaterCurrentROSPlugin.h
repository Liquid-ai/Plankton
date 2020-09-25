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

/// \file UnderwaterCurrentROSPlugin.hh
/// \brief Publishes the constant flow velocity in ROS messages and creates a
/// service to alter the flow model in runtime

#ifndef __UNDERWATER_CURRENT_ROS_PLUGIN_HH__
#define __UNDERWATER_CURRENT_ROS_PLUGIN_HH__

#include <map>
#include <string>

// Gazebo plugin
#include <uuv_world_plugins/UnderwaterCurrentPlugin.h>

#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/World.hh>

#include <gazebo_ros/node.hpp>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

#include <uuv_world_ros_plugins_msgs/srv/set_current_model.hpp>
#include <uuv_world_ros_plugins_msgs/srv/get_current_model.hpp>
#include <uuv_world_ros_plugins_msgs/srv/set_current_velocity.hpp>
#include <uuv_world_ros_plugins_msgs/srv/set_current_direction.hpp>
#include <uuv_world_ros_plugins_msgs/srv/set_origin_spherical_coord.hpp>
#include <uuv_world_ros_plugins_msgs/srv/get_origin_spherical_coord.hpp>

namespace uuv_simulator_ros
{
  class UnderwaterCurrentROSPlugin : public gazebo::UnderwaterCurrentPlugin
  {
    /// \brief Class constructor
    public: UnderwaterCurrentROSPlugin();

    /// \brief Class destructor
    public: virtual ~UnderwaterCurrentROSPlugin();

    /// \brief Load module and read parameters from SDF.
    public: void Load(gazebo::physics::WorldPtr _world,
        sdf::ElementPtr _sdf);

    /// \brief Service call to update the parameters for the velocity
    /// Gauss-Markov process model
    public: void UpdateCurrentVelocityModel(
        const uuv_world_ros_plugins_msgs::srv::SetCurrentModel::Request::SharedPtr _req,
        uuv_world_ros_plugins_msgs::srv::SetCurrentModel::Response::SharedPtr _res);

    /// \brief Service call to update the parameters for the horizontal angle
    /// Gauss-Markov process model
    public: void UpdateCurrentHorzAngleModel(
        const uuv_world_ros_plugins_msgs::srv::SetCurrentModel::Request::SharedPtr _req,
        uuv_world_ros_plugins_msgs::srv::SetCurrentModel::Response::SharedPtr _res);

    /// \brief Service call to update the parameters for the vertical angle
    /// Gauss-Markov process model
    public: void UpdateCurrentVertAngleModel(
        const uuv_world_ros_plugins_msgs::srv::SetCurrentModel::Request::SharedPtr _req,
        uuv_world_ros_plugins_msgs::srv::SetCurrentModel::Response::SharedPtr _res);

    /// \brief Service call to read the parameters for the velocity
    /// Gauss-Markov process model
    public: void GetCurrentVelocityModel(
        const uuv_world_ros_plugins_msgs::srv::GetCurrentModel::Request::SharedPtr _req,
        uuv_world_ros_plugins_msgs::srv::GetCurrentModel::Response::SharedPtr _res);

    /// \brief Service call to read the parameters for the horizontal angle
    /// Gauss-Markov process model
    public: void GetCurrentHorzAngleModel(
        const uuv_world_ros_plugins_msgs::srv::GetCurrentModel::Request::SharedPtr _req,
        uuv_world_ros_plugins_msgs::srv::GetCurrentModel::Response::SharedPtr _res);

    /// \brief Service call to read the parameters for the vertical angle
    /// Gauss-Markov process model
    public: void GetCurrentVertAngleModel(
        const uuv_world_ros_plugins_msgs::srv::GetCurrentModel::Request::SharedPtr _req,
        uuv_world_ros_plugins_msgs::srv::GetCurrentModel::Response::SharedPtr _res);

    /// \brief Service call to update the mean value of the flow velocity
    public: void UpdateCurrentVelocity(
        const uuv_world_ros_plugins_msgs::srv::SetCurrentVelocity::Request::SharedPtr _req,
        uuv_world_ros_plugins_msgs::srv::SetCurrentVelocity::Response::SharedPtr _res);

    /// \brief Service call to update the mean value of the horizontal angle
    public: void UpdateHorzAngle(
        const uuv_world_ros_plugins_msgs::srv::SetCurrentDirection::Request::SharedPtr _req,
        uuv_world_ros_plugins_msgs::srv::SetCurrentDirection::Response::SharedPtr _res);

    /// \brief Service call to update the mean value of the vertical angle
    public: void UpdateVertAngle(
        const uuv_world_ros_plugins_msgs::srv::SetCurrentDirection::Request::SharedPtr _req,
        uuv_world_ros_plugins_msgs::srv::SetCurrentDirection::Response::SharedPtr _res);

    /// \brief Publishes ROS topics
    private: void OnUpdateCurrentVel();

    /// \brief All underwater world services
    private: std::map<std::string, rclcpp::ServiceBase::SharedPtr> worldServices;

    /// \brief Pointer to this ROS node's handle.
    private: gazebo_ros::Node::SharedPtr myRosNode;

    /// \brief Connection for callbacks on update world.
    private: gazebo::event::ConnectionPtr rosPublishConnection;

    /// \brief Publisher for the flow velocity in the world frame
    private: rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr myFlowVelocityPub;

    /// \brief Period after which we should publish a message via ROS.
    private: gazebo::common::Time rosPublishPeriod;

    /// \brief Last time we published a message via ROS.
    private: gazebo::common::Time lastRosPublishTime;
  };
}

#endif  // __UNDERWATER_CURRENT_ROS_PLUGIN_HH__
