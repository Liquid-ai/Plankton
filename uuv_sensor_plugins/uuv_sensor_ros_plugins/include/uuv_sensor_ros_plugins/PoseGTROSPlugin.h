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

// This source code is derived from gazebo_ros_pkgs
//   (https://github.com/ros-simulation/gazebo_ros_pkgs)
// * Copyright 2012 Open Source Robotics Foundation,
// licensed under the Apache-2.0 license,
// cf. 3rd-party-licenses.txt file in the root directory of this source tree.
//
// The original code was modified to:
// - be more consistent with other sensor plugins within uuv_simulator,
// - adhere to Gazebo's coding standards.

#ifndef __UUV_POSE_GT_SENSOR_ROS_PLUGIN_HH__
#define __UUV_POSE_GT_SENSOR_ROS_PLUGIN_HH__

#include <gazebo/gazebo.hh>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <gazebo/physics/physics.hh>

#include <uuv_sensor_ros_plugins/ROSBaseModelPlugin.h>

#include <memory>

namespace gazebo
{
  class PoseGTROSPlugin : public ROSBaseModelPlugin
  {
    /// \brief Class constructor
    public: PoseGTROSPlugin();

    /// \brief Class destructor
    public: ~PoseGTROSPlugin();

    /// \brief Load the plugin
    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

    /// \brief Update sensor measurement
    protected: virtual bool OnUpdate(const common::UpdateInfo& _info);

    protected: void PublishNEDOdomMessage(common::Time _time,
      ignition::math::Pose3d _pose, ignition::math::Vector3d _linVel,
      ignition::math::Vector3d _angVel);

    protected: void PublishOdomMessage(common::Time _time,
      ignition::math::Pose3d _pose, ignition::math::Vector3d _linVel,
      ignition::math::Vector3d _angVel);

    protected: void UpdateNEDTransform();

    protected: rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr nedOdomPub;

    /// \brief Pose offset
    protected: ignition::math::Pose3d offset;

    protected: std::string nedFrameID;

    protected: ignition::math::Pose3d nedTransform;

    protected: bool nedTransformIsInit;

    protected: bool publishNEDOdom;

    //protected: tf2_ros::Buffer tfBuffer;
    std::unique_ptr<tf2_ros::Buffer> myTfBuffer;

    protected: std::shared_ptr<tf2_ros::TransformListener> tfListener;

    protected: ignition::math::Vector3d lastLinVel;
    protected: ignition::math::Vector3d lastAngVel;
    protected: ignition::math::Vector3d linAcc;
    protected: ignition::math::Vector3d angAcc;
    protected: ignition::math::Vector3d lastRefLinVel;
    protected: ignition::math::Vector3d lastRefAngVel;
    protected: ignition::math::Vector3d refLinAcc;
    protected: ignition::math::Vector3d refAngAcc;

    /// \brief publisher for transporting measurement messages.
    protected: rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr rosSensorOutputPub;
    
  };
}

#endif // __UUV_POSE_GT_SENSOR_ROS_PLUGIN_HH__
