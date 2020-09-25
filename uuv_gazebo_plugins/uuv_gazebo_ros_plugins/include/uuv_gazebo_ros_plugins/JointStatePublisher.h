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

/// \file JointStatePublisher.hh A Gazebo ROS plugin for publishing the joint
/// states of a robot (position, velocity and effort). Build similar to the
/// class in the GazeboRosJointStatePublisher, but including more information

#ifndef __JOINT_STATE_PUBLISHER_HH__
#define __JOINT_STATE_PUBLISHER_HH__

#include <sstream>
#include <string>
#include <vector>

#include <gazebo/gazebo.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Event.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo/physics/World.hh>

#include <gazebo_ros/node.hpp>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>


namespace uuv_simulator_ros
{
class JointStatePublisher : public gazebo::ModelPlugin
{
  public: JointStatePublisher();

  public: ~JointStatePublisher();

  public: void Load(gazebo::physics::ModelPtr _parent, sdf::ElementPtr _sdf);

  public: void OnUpdate(const gazebo::common::UpdateInfo &_info);

  public: void PublishJointStates();

  private: bool IsIgnoredJoint(std::string _jointName);

  private: gazebo::physics::WorldPtr world;

  private: gazebo::physics::ModelPtr myModel;

  private: gazebo::event::ConnectionPtr updateConnection;

  protected: gazebo_ros::Node::SharedPtr myRosNode;

  private: std::string myRobotNamespace;

  private: std::vector<std::string> movingJoints;

  private: double updateRate;

  private: double updatePeriod;

  private: gazebo::common::Time lastUpdate;

  private: rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr myJointStatePub;
};
}

#endif  // __JOINT_STATE_PUBLISHER_HH__
