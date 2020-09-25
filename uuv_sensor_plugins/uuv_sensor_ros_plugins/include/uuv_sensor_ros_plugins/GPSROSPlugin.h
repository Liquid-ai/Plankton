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

#ifndef __GPS_SENSOR_ROS_PLUGIN_HH__
#define __GPS_SENSOR_ROS_PLUGIN_HH__

#include <gazebo/gazebo.hh>
#include <gazebo/sensors/sensors.hh>

#include <uuv_sensor_ros_plugins/ROSBaseSensorPlugin.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

namespace gazebo
{
  class GPSROSPlugin : public ROSBaseSensorPlugin
  {
    /// \brief Class constructor
    public: GPSROSPlugin();

    /// \brief Class destructor
    public: virtual ~GPSROSPlugin();

    /// \brief Load module and read parameters from SDF.
    public: virtual void Load(sensors::SensorPtr _parent,
      sdf::ElementPtr _sdf);

    /// \brief Update GPS ROS message
    public: bool OnUpdateGPS();

    /// \brief Pointer to the parent sensor
    protected: sensors::GpsSensorPtr gazeboGPSSensor;

    /// \brief Output GPS ROS message
    protected: sensor_msgs::msg::NavSatFix gpsMessage;

    /// \brief publisher for transporting measurement messages.
    protected: rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr rosSensorOutputPub;
  };
}

#endif // __GPS_SENSOR_ROS_PLUGIN_HH__
