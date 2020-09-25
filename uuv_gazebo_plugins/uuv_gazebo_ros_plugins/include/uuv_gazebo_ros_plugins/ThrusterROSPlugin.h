// Copyright (c) 2020 The Plankton Authors.
// All rights reserved.
//
// This source code is derived from UUV Simulator
// (https://github.com/uuvsimulator/uuv_simulator)
// Copyright (c) 2016-2019 The UUV Simulator Authors
// licensed under the Apache license, Version 2.0
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

#ifndef __THRUSTER_ROS_PLUGIN_HH__
#define __THRUSTER_ROS_PLUGIN_HH__

#include <map>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>

#include <gazebo/common/Plugin.hh>

#include <gazebo_ros/node.hpp>

#include <uuv_gazebo_plugins/ThrusterPlugin.h>

#include <uuv_gazebo_ros_plugins_msgs/msg/float_stamped.hpp>
#include <uuv_gazebo_ros_plugins_msgs/srv/set_thruster_state.hpp>
#include <uuv_gazebo_ros_plugins_msgs/srv/get_thruster_state.hpp>
#include <uuv_gazebo_ros_plugins_msgs/srv/set_thruster_efficiency.hpp>
#include <uuv_gazebo_ros_plugins_msgs/srv/get_thruster_efficiency.hpp>
#include <uuv_gazebo_ros_plugins_msgs/srv/get_thruster_conversion_fcn.hpp>

namespace uuv_simulator_ros
{
  class ThrusterROSPlugin : public gazebo::ThrusterPlugin
  {
    /// \brief Constrcutor.
    public: ThrusterROSPlugin();

    /// \brief Destructor.
    public: ~ThrusterROSPlugin();

    /// \brief Load module and read parameters from SDF.
    public: void Load(gazebo::physics::ModelPtr _parent, sdf::ElementPtr _sdf);

    /// \brief Publish thruster state via ROS.
    public: void RosPublishStates();

    /// \brief Set new set point (desired thrust [N]) for thruster.
    public: void SetThrustReference(
        const uuv_gazebo_ros_plugins_msgs::msg::FloatStamped::SharedPtr _msg);

    /// \brief Return the ROS publish period.
    public: gazebo::common::Time  GetRosPublishPeriod();

    /// \brief Set the ROS publish frequency (Hz).
    public: void SetRosPublishRate(double _hz);

    /// \brief Initialize Module.
    public: virtual void Init();

    /// \brief Reset Module.
    public: virtual void Reset();

    /// \brief Set the thrust efficiency factor
    public: void SetThrustForceEfficiency(
      const uuv_gazebo_ros_plugins_msgs::srv::SetThrusterEfficiency::Request::SharedPtr _req,
      uuv_gazebo_ros_plugins_msgs::srv::SetThrusterEfficiency::Response::SharedPtr _res);

    /// \brief Get the thrust efficiency factor
    public: void GetThrustForceEfficiency(
      const uuv_gazebo_ros_plugins_msgs::srv::GetThrusterEfficiency::Request::SharedPtr _req,
      uuv_gazebo_ros_plugins_msgs::srv::GetThrusterEfficiency::Response::SharedPtr _res);

    /// \brief Set the dynamic state efficiency factor
    public: void SetDynamicStateEfficiency(
      const uuv_gazebo_ros_plugins_msgs::srv::SetThrusterEfficiency::Request::SharedPtr _req,
      uuv_gazebo_ros_plugins_msgs::srv::SetThrusterEfficiency::Response::SharedPtr _res);

      /// \brief Get the dynamic state efficiency factor
    public: void GetDynamicStateEfficiency(
      const uuv_gazebo_ros_plugins_msgs::srv::GetThrusterEfficiency::Request::SharedPtr _req,
        uuv_gazebo_ros_plugins_msgs::srv::GetThrusterEfficiency::Response::SharedPtr _res);

    /// \brief Turn thruster on/off
    public: void SetThrusterState(
      const  uuv_gazebo_ros_plugins_msgs::srv::SetThrusterState::Request::SharedPtr _req,
      uuv_gazebo_ros_plugins_msgs::srv::SetThrusterState::Response::SharedPtr _res);

    /// \brief Get thruster state
    public: void GetThrusterState(
      const uuv_gazebo_ros_plugins_msgs::srv::GetThrusterState::Request::SharedPtr _req,
      uuv_gazebo_ros_plugins_msgs::srv::GetThrusterState::Response::SharedPtr _res);

    /// \brief Get thruster conversion function parameters
    public: void GetThrusterConversionFcn(
      const uuv_gazebo_ros_plugins_msgs::srv::GetThrusterConversionFcn::Request::SharedPtr _req,
      uuv_gazebo_ros_plugins_msgs::srv::GetThrusterConversionFcn::Response::SharedPtr _res);

    /// \brief Map of thruster services
    //TODO Replace with map<string, ServiceBase::SharedPtr>
    //private: std::map<std::string, rclcpp::Service:: ::ServiceServer> services;
    rclcpp::Service<uuv_gazebo_ros_plugins_msgs::srv::SetThrusterEfficiency>::SharedPtr mySet_thrust_force_efficiencySrv;
    rclcpp::Service<uuv_gazebo_ros_plugins_msgs::srv::GetThrusterEfficiency>::SharedPtr myGet_thrust_force_efficiencySrv;
    rclcpp::Service<uuv_gazebo_ros_plugins_msgs::srv::SetThrusterEfficiency>::SharedPtr mySet_dynamic_state_efficiencySrv;
    rclcpp::Service<uuv_gazebo_ros_plugins_msgs::srv::GetThrusterEfficiency>::SharedPtr myGet_dynamic_state_efficiency;
    rclcpp::Service<uuv_gazebo_ros_plugins_msgs::srv::SetThrusterState>::SharedPtr mySet_thruster_state;
    rclcpp::Service<uuv_gazebo_ros_plugins_msgs::srv::GetThrusterState>::SharedPtr myGet_thruster_state;
    rclcpp::Service<uuv_gazebo_ros_plugins_msgs::srv::GetThrusterConversionFcn>::SharedPtr myGet_thruster_conversion_fcn;

    /// \brief Pointer to this ROS node's handle.
    protected: gazebo_ros::Node::SharedPtr myRosNode;

    /// \brief Subscriber reacting to new reference thrust set points.
    private: rclcpp::Subscription<uuv_gazebo_ros_plugins_msgs::msg::FloatStamped>::SharedPtr mySubThrustReference;

    /// \brief Publisher for current actual thrust.
    private: rclcpp::Publisher<uuv_gazebo_ros_plugins_msgs::msg::FloatStamped>::SharedPtr myPubThrust;

    /// \brief Publisher for current actual thrust as wrench.
    private: rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr myPubThrustWrench;

    /// \brief Publisher for the thruster state
    private: rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr myPubThrusterState;

    /// \brief Publisher for the thrust force efficiency
    private: rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr myPubThrustForceEff;

    /// \brief Publisher for the dynamic state efficiency
    private: rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr myPubDynamicStateEff;

    /// \brief Connection for callbacks on update world.
    private: gazebo::event::ConnectionPtr rosPublishConnection;

    /// \brief Period after which we should publish a message via ROS.
    private: gazebo::common::Time rosPublishPeriod;

    /// \brief Last time we published a message via ROS.
    private: gazebo::common::Time lastRosPublishTime;
  };
}

#endif  // __THRUSTER_ROS_PLUGIN_HH__
