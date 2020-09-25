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

/// \file UnderwaterObjectROSPlugin.hh Publishes the underwater object's
/// Gazebo topics and parameters into ROS standards

#ifndef __UNDERWATER_OBJECT_ROS_PLUGIN_HH__
#define __UNDERWATER_OBJECT_ROS_PLUGIN_HH__

#include <map>
#include <string>

#include <gazebo/gazebo.hh>
#include <gazebo/common/Plugin.hh>

#include <gazebo_ros/node.hpp>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <std_msgs/msg/bool.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>

#include <uuv_gazebo_plugins/UnderwaterObjectPlugin.h>

#include <uuv_gazebo_ros_plugins_msgs/srv/set_use_global_current_vel.hpp>
#include <uuv_gazebo_ros_plugins_msgs/msg/underwater_object_model.hpp>
#include <uuv_gazebo_ros_plugins_msgs/srv/get_model_properties.hpp>
#include <uuv_gazebo_ros_plugins_msgs/srv/set_float.hpp>
#include <uuv_gazebo_ros_plugins_msgs/srv/get_float.hpp>


namespace uuv_simulator_ros
{
  class UnderwaterObjectROSPlugin : public gazebo::UnderwaterObjectPlugin
  {
    /// \brief Constructor
    public: UnderwaterObjectROSPlugin();

    /// \brief Destructor
    public: virtual ~UnderwaterObjectROSPlugin();

    /// \brief Load module and read parameters from SDF.
    public: void Load(gazebo::physics::ModelPtr _parent, sdf::ElementPtr _sdf);

    /// \brief Initialize Module.
    public: virtual void Init();

    /// \brief Reset Module.
    public: virtual void Reset();

    /// \brief Update the simulation state.
    /// \param[in] _info Information used in the update event.
    public: virtual void Update(const gazebo::common::UpdateInfo &_info);

    /// \brief Update the local current velocity, this data will be used only
    /// if the useGlobalCurrent flag is set to false.
    public: void UpdateLocalCurrentVelocity(
      const geometry_msgs::msg::Vector3::SharedPtr _msg);

    /// \brief Set flag to use the global current velocity topic input
    public: void SetUseGlobalCurrentVel(
      const uuv_gazebo_ros_plugins_msgs::srv::SetUseGlobalCurrentVel::Request::SharedPtr _req,
      uuv_gazebo_ros_plugins_msgs::srv::SetUseGlobalCurrentVel::Response::SharedPtr _res);

    /// \brief Return the model properties, along with parameters from the
    /// hydrodynamic and hydrostatic models
    public: void GetModelProperties(
      const uuv_gazebo_ros_plugins_msgs::srv::GetModelProperties::Request::SharedPtr _req,
      uuv_gazebo_ros_plugins_msgs::srv::GetModelProperties::Response::SharedPtr _res);

    /// \brief Set the scaling factor for the added-mass matrix
    public: void SetScalingAddedMass(
      const uuv_gazebo_ros_plugins_msgs::srv::SetFloat::Request::SharedPtr _req,
      uuv_gazebo_ros_plugins_msgs::srv::SetFloat::Response::SharedPtr _res);

    /// \brief Return current scaling factor for the added-mass matrix
    public: void GetScalingAddedMass(
      const uuv_gazebo_ros_plugins_msgs::srv::GetFloat::Request::SharedPtr _req,
      uuv_gazebo_ros_plugins_msgs::srv::GetFloat::Response::SharedPtr _res);

    /// \brief Set a scaling factor for the overall damping matrix
    public: void SetScalingDamping(
      const uuv_gazebo_ros_plugins_msgs::srv::SetFloat::Request::SharedPtr _req,
      uuv_gazebo_ros_plugins_msgs::srv::SetFloat::Response::SharedPtr _res);

    /// \brief Return the scaling factor for the overall damping matrix
    public: void GetScalingDamping(
      const uuv_gazebo_ros_plugins_msgs::srv::GetFloat::Request::SharedPtr _req,
      uuv_gazebo_ros_plugins_msgs::srv::GetFloat::Response::SharedPtr _res);

    /// \brief Set scaling factor for the model's volume used for buoyancy
    /// force computation
    public: void SetScalingVolume(
      const uuv_gazebo_ros_plugins_msgs::srv::SetFloat::Request::SharedPtr _req,
      uuv_gazebo_ros_plugins_msgs::srv::SetFloat::Response::SharedPtr _res);

    /// \brief Get scaling factor for the model's volume used for buoyancy
    /// force computation
    public: void GetScalingVolume(
      const uuv_gazebo_ros_plugins_msgs::srv::GetFloat::Request::SharedPtr _req,
      uuv_gazebo_ros_plugins_msgs::srv::GetFloat::Response::SharedPtr _res);

    /// \brief Set new fluid density (this will alter the value for the
    /// buoyancy force)
    public: void SetFluidDensity(
      const uuv_gazebo_ros_plugins_msgs::srv::SetFloat::Request::SharedPtr _req,
      uuv_gazebo_ros_plugins_msgs::srv::SetFloat::Response::SharedPtr _res);

    /// \brief Get current value for the fluid density
    public: void GetFluidDensity(
      const uuv_gazebo_ros_plugins_msgs::srv::GetFloat::Request::SharedPtr _req,
      uuv_gazebo_ros_plugins_msgs::srv::GetFloat::Response::SharedPtr _res);

    /// \brief Set offset factor for the model's volume (this will alter the
    /// value for the buoyancy force)
    public: void SetOffsetVolume(
      const uuv_gazebo_ros_plugins_msgs::srv::SetFloat::Request::SharedPtr _req,
      uuv_gazebo_ros_plugins_msgs::srv::SetFloat::Response::SharedPtr _res);

    /// \brief Return the offset factor for the model's volume
    public: void GetOffsetVolume(
      const uuv_gazebo_ros_plugins_msgs::srv::GetFloat::Request::SharedPtr _req,
      uuv_gazebo_ros_plugins_msgs::srv::GetFloat::Response::SharedPtr _res);

    /// \brief Set the offset factor for the added-mass matrix
    public: void SetOffsetAddedMass(
      const uuv_gazebo_ros_plugins_msgs::srv::SetFloat::Request::SharedPtr _req,
      uuv_gazebo_ros_plugins_msgs::srv::SetFloat::Response::SharedPtr _res);

    /// \brief Return the offset factor for the added-mass matrix
    public: void GetOffsetAddedMass(
      const uuv_gazebo_ros_plugins_msgs::srv::GetFloat::Request::SharedPtr _req,
      uuv_gazebo_ros_plugins_msgs::srv::GetFloat::Response::SharedPtr _res);

    /// \brief Set the offset factor for the linear damping matrix
    public: void SetOffsetLinearDamping(
      const uuv_gazebo_ros_plugins_msgs::srv::SetFloat::Request::SharedPtr _req,
      uuv_gazebo_ros_plugins_msgs::srv::SetFloat::Response::SharedPtr _res);

    /// \brief Return the offset factor for the linear damping matrix
    public: void GetOffsetLinearDamping(
      const uuv_gazebo_ros_plugins_msgs::srv::GetFloat::Request::SharedPtr _req,
      uuv_gazebo_ros_plugins_msgs::srv::GetFloat::Response::SharedPtr _res);

    /// \brief Set the offset factor for the linear forward speed damping
    /// matrix
    public: void SetOffsetLinearForwardSpeedDamping(
      const uuv_gazebo_ros_plugins_msgs::srv::SetFloat::Request::SharedPtr _req,
      uuv_gazebo_ros_plugins_msgs::srv::SetFloat::Response::SharedPtr _res);

    /// \brief Return the offset factor for the linear forward speed damping
    /// matrix
    public: void GetOffsetLinearForwardSpeedDamping(
      const uuv_gazebo_ros_plugins_msgs::srv::GetFloat::Request::SharedPtr _req,
      uuv_gazebo_ros_plugins_msgs::srv::GetFloat::Response::SharedPtr _res);

    /// \brief Set the offset factor for the nonlinear damping
    /// matrix
    public: void SetOffsetNonLinearDamping(
      const uuv_gazebo_ros_plugins_msgs::srv::SetFloat::Request::SharedPtr _req,
      uuv_gazebo_ros_plugins_msgs::srv::SetFloat::Response::SharedPtr _res);

    /// \brief Return the offset factor for the nonlinear damping
    /// matrix
    public: void GetOffsetNonLinearDamping(
      const uuv_gazebo_ros_plugins_msgs::srv::GetFloat::Request::SharedPtr _req,
      uuv_gazebo_ros_plugins_msgs::srv::GetFloat::Response::SharedPtr _res);

    /// \brief Publish restoring force
    /// \param[in] _link Pointer to the link where the force information will
    /// be extracted from
    protected: virtual void PublishRestoringForce(
      gazebo::physics::LinkPtr _link);

    /// \brief Publish hydrodynamic wrenches
    /// \param[in] _link Pointer to the link where the force information will
    /// be extracted from
    protected: virtual void PublishHydrodynamicWrenches(
      gazebo::physics::LinkPtr _link);

    /// \brief Returns the wrench message for debugging topics
    /// \param[in] _force Force vector
    /// \param[in] _torque Torque vector
    /// \param[in] _output Stamped wrench message to be updated
    protected: virtual void GenWrenchMsg(
      ignition::math::Vector3d _force, ignition::math::Vector3d _torque,
      geometry_msgs::msg::WrenchStamped &_output);

    /// \brief Sets the topics used for publishing the intermediate data during
    /// the simulation
    /// \param[in] _link Pointer to the link
    /// \param[in] _hydro Pointer to the hydrodynamic model
    protected: virtual void InitDebug(gazebo::physics::LinkPtr _link,
      gazebo::HydrodynamicModelPtr _hydro);

    /// \brief Publishes the current velocity marker
    protected: virtual void PublishCurrentVelocityMarker();

    /// \brief Publishes the state of the vehicle (is submerged)
    protected: virtual void PublishIsSubmerged();

    /// \brief Pointer to this ROS node's handle.
    protected: gazebo_ros::Node::SharedPtr myRosNode;

    /// \brief Subscriber to locally calculated current velocity
    private: rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr mySubLocalCurVel;

    /// \brief Publisher for current actual thrust.
    private: std::map<std::string, rclcpp::PublisherBase::SharedPtr> myRosHydroPub;
    // private: rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr myCurrent_velocity_marker;
    // private: rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr myUsing_global_current_velocity;
    // private: rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr myIs_submerged;

    /// \brief Map of services
    // private: std::map<std::string, ros::ServiceServer> services;
    private: std::map<std::string, rclcpp::ServiceBase::SharedPtr> services;

    private: geometry_msgs::msg::TransformStamped nedTransform;

    private: std::unique_ptr<tf2_ros::TransformBroadcaster> tfBroadcaster;
  };
}

#endif  // __UNDERWATER_OBJECT_ROS_PLUGIN_HH__
