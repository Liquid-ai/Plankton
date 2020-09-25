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

#ifndef __UUV_RPT_ROS_PLUGIN_HH__
#define __UUV_RPT_ROS_PLUGIN_HH__

#include <gazebo/gazebo.hh>

#include <rclcpp/rclcpp.hpp>

#include <uuv_sensor_ros_plugins/ROSBaseModelPlugin.h>
#include <uuv_sensor_ros_plugins_msgs/msg/position_with_covariance_stamped.hpp>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include "SensorRpt.pb.h"

namespace gazebo
{
  class RPTROSPlugin :  public ROSBaseModelPlugin
  {
    /// \brief Class constructor
    public: RPTROSPlugin();

    /// \brief Class destructor
    public: virtual ~RPTROSPlugin();

    /// \brief Load the plugin
    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

    /// \brief Update sensor measurement
    protected: virtual bool OnUpdate(const common::UpdateInfo& _info);

    /// \brief Latest measured position.
    protected: ignition::math::Vector3d position;

    /// \brief Store message since many attributes do not change (cov.).
    protected: uuv_sensor_ros_plugins_msgs::msg::PositionWithCovarianceStamped myRosMessage;

    /// \brief publisher for transporting measurement messages.
    protected: rclcpp::Publisher<uuv_sensor_ros_plugins_msgs::msg::PositionWithCovarianceStamped>::SharedPtr rosSensorOutputPub;
    
  };
}

#endif // __UUV_RPT_ROS_PLUGIN_HH__
