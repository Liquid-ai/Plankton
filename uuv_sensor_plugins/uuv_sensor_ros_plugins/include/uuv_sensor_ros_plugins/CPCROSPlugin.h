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

#ifndef __UUV_CHEMICAL_PARTICLE_CONCENTRATION_ROS_PLUGIN_HH__
#define __UUV_CHEMICAL_PARTICLE_CONCENTRATION_ROS_PLUGIN_HH__

#include <gazebo/gazebo.hh>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>

#include <uuv_sensor_ros_plugins/ROSBaseModelPlugin.h>
#include <uuv_sensor_ros_plugins/ROSBaseModelPlugin.h>

#include <uuv_sensor_ros_plugins_msgs/msg/chemical_particle_concentration.hpp>
#include <uuv_sensor_ros_plugins_msgs/msg/salinity.hpp>

namespace gazebo
{
  class CPCROSPlugin : public ROSBaseModelPlugin
  {
    /// \brief Class constructor
    public: CPCROSPlugin();

    /// \brief Class destructor
    public: virtual ~CPCROSPlugin();

    /// \brief Load the plugin
    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

    /// \brief Update sensor measurement
    protected: virtual bool OnUpdate(const common::UpdateInfo& _info);

    /// \brief Update callback from simulator.
    protected: virtual void OnPlumeParticlesUpdate(
      const sensor_msgs::msg::PointCloud::SharedPtr _msg);

    /// \brief Input topic for the plume particle point cloud
    protected: rclcpp::Subscription<sensor_msgs::msg::PointCloud>::SharedPtr particlesSub;

    /// \brief Output topic for salinity measurements based on the particle concentration
    protected: rclcpp::Publisher<uuv_sensor_ros_plugins_msgs::msg::Salinity>::SharedPtr salinityPub;

    /// \brief Flag to ensure the cloud and measurement update don't coincide
    protected: bool updatingCloud;

    /// \brief Gamma velocity parameter for the smoothing function
    protected: double gamma;

    /// \brief Sensor gain
    protected: double gain;

    // \brief Radius of the kernel to identify particles that will be taken into
    // account in the concentration computation
    protected: double smoothingLength;

    //protected: rclcpp::Clock myTimer;

    /// \brief Last update from the point cloud callback
    //protected: ros::Time lastUpdateTimestamp;
    protected: rclcpp::Time myLastUpdateTimeStamp;

    /// \brief Output measurement topic
    protected: uuv_sensor_ros_plugins_msgs::msg::ChemicalParticleConcentration
      outputMsg;

    /// \brief Output salinity measurement message
    protected: uuv_sensor_ros_plugins_msgs::msg::Salinity salinityMsg;

    protected: double waterSalinityValue;

    protected: double plumeSalinityValue;

    /// \brief publisher for transporting measurement messages.
    protected: rclcpp::Publisher<uuv_sensor_ros_plugins_msgs::msg::ChemicalParticleConcentration>::SharedPtr rosSensorOutputPub;
    
  };
}

#endif // __UUV_CHEMICAL_PARTICLE_CONCENTRATION_ROS_PLUGIN_HH__
