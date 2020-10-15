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

#ifndef __ROS_BASE_PLUGIN_HH__
#define __ROS_BASE_PLUGIN_HH__

#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/Noise.hh>

#include <gazebo_ros/node.hpp>

#include <uuv_sensor_ros_plugins/Common.h>
#include <uuv_sensor_ros_plugins_msgs/srv/change_sensor_state.hpp>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

#include <chrono>
#include <random>
#include <string>
#include <map>


namespace gazebo
{
  class ROSBasePlugin
  {
    /// \brief Class constructor
    public: ROSBasePlugin();

    /// \brief Class destructor
    public: virtual ~ROSBasePlugin();

    /// \brief Initialize base plugin
    public: bool InitBasePlugin(sdf::ElementPtr _sdf);

    /// \brief Update callback from simulation.
    public: virtual bool OnUpdate(const common::UpdateInfo&) = 0;

    /// \brief Add noise normal distribution to the list
    public: bool AddNoiseModel(const std::string& _name, double _sigma);

    /// \brief Robot namespace
    protected: std::string myRobotNamespace;

    /// \brief Name of the sensor's output topic
    protected: std::string sensorOutputTopic;

    /// \brief Pointer to the world.
    protected: physics::WorldPtr world;

    /// \brief Pointer to the update event connection.
    protected: event::ConnectionPtr updateConnection;

    /// \brief (Simulation) time when the last sensor measurement was generated.
    protected: common::Time lastMeasurementTime;

    /// \brief Sensor update rate
    protected: double updateRate;

    /// \brief Noise standard deviation
    protected: double noiseSigma;

    /// \brief Noise amplitude
    protected: double noiseAmp;

    /// \brief Flag set to true if the Gazebo sensors messages are supposed
    /// to be published as well (it can avoid unnecessary overhead in case)
    /// the sensor messages needed are only ROS messages
    protected: bool gazeboMsgEnabled;

    /// \brief Pseudo random number generator
    protected: std::default_random_engine rndGen;

    /// \brief Normal distribution describing the noise models
    protected: std::map<std::string, std::normal_distribution<double>>
      noiseModels;

    /// \brief Flag to control the generation of output messages
    protected: std_msgs::msg::Bool isOn;

    /// \brief ROS node handle for communication with ROS
    protected: gazebo_ros::Node::SharedPtr myRosNode;

    /// \brief Gazebo's node handle for transporting measurement  messages.
    protected: transport::NodePtr gazeboNode;

    /// \brief Gazebo's publisher for transporting measurement messages.
    //protected: rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr rosSensorOutputPub;

    /// \brief Gazebo's publisher for transporting measurement messages.
    protected: transport::PublisherPtr gazeboSensorOutputPub;

    /// \brief Service server object
    protected: rclcpp::Service<uuv_sensor_ros_plugins_msgs::srv::ChangeSensorState>::SharedPtr myChangeSensorSrv;

    /// \brief ROS publisher for the switchable sensor data
    protected: rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr myPluginStatePub;

    /// \brief Pose of the reference frame wrt world frame
    protected: ignition::math::Pose3d referenceFrame;

    /// \brief ROS subscriber for the TF static reference frame
    protected: rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr myTfStaticSub;

    /// \brief Frame ID of the reference frame
    protected: std::string referenceFrameID;

    /// \brief Flag set to true if reference frame initialized
    protected: bool isReferenceInit;

    /// \brief Reference link
    protected: physics::LinkPtr referenceLink;

    /// \brief Returns true if the plugin is activated
    protected: bool IsOn();

    /// \brief Publish the current state of the plugin
    protected: void PublishState();

    /// \brief Change sensor state (ON/OFF)
    protected: void ChangeSensorState(
        const uuv_sensor_ros_plugins_msgs::srv::ChangeSensorState::Request::SharedPtr _req,
        uuv_sensor_ros_plugins_msgs::srv::ChangeSensorState::Response::SharedPtr _res);

    /// \brief Callback function for the static TF message
    protected: void GetTFMessage(const tf2_msgs::msg::TFMessage::SharedPtr _msg);// tf::tfMessage::ConstPtr &_msg);

    /// \brief Returns noise value for a function with zero mean from the
    /// default Gaussian noise model
    protected: double GetGaussianNoise(double _amp);

    /// \brief Returns noise value for a function with zero mean from a
    /// Gaussian noise model according to the model name
    protected: double GetGaussianNoise(const std::string& _name, double _amp);

    /// \brief Enables generation of simulated measurement if the timeout
    /// since the last update has been reached
    protected: bool EnableMeasurement(const common::UpdateInfo& _info) const;

    /// \brief Updates the pose of the reference frame wrt the world frame
    protected: void UpdateReferenceFramePose();
  };
}

#endif // __ROS_BASE_PLUGIN_HH__
