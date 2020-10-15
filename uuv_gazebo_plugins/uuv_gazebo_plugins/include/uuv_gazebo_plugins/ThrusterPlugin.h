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

/// \file ThrusterPlugin.hh
/// \brief Model plugin for description of the thruster dynamics

#ifndef __UUV_GAZEBO_PLUGINS_THRUSTER_PLUGIN_HH__
#define __UUV_GAZEBO_PLUGINS_THRUSTER_PLUGIN_HH__

#include <boost/shared_ptr.hpp>

#include <map>
#include <string>
#include <memory>

#include <gazebo/gazebo.hh>
#include <gazebo/transport/TransportTypes.hh>

#include <sdf/sdf.hh>

#include <uuv_gazebo_plugins/ThrusterConversionFcn.h>
#include <uuv_gazebo_plugins/Dynamics.h>

#include "Double.pb.h"

namespace gazebo
{
/// \brief Definition of a pointer to the floating point message
typedef const boost::shared_ptr<const uuv_gazebo_plugins_msgs::msgs::Double>
ConstDoublePtr;

/// \brief Class for the thruster plugin
class ThrusterPlugin : public ModelPlugin
{
  /// \brief Constructor
  public: ThrusterPlugin();

  /// \brief Destructor
  public: virtual ~ThrusterPlugin();

  // Documentation inherited.
  public: virtual void Load(physics::ModelPtr _model,
                            sdf::ElementPtr _sdf);

  // Documentation inherited.
  public: virtual void Init();

  /// \brief Custom plugin reset behavior.
  public: virtual void Reset();

  /// \brief Update the simulation state.
  /// \param[in] _info Information used in the update event.
  public: void Update(const common::UpdateInfo &_info);

  /// \brief Callback for the input topic subscriber
  protected: void UpdateInput(ConstDoublePtr &_msg);

  /// \brief Builds the topic prefix name
  protected: std::string BuildTopicPrefix(const std::string& pluginNamespace, int id);

  /// \brief Thruster dynamic model
  protected: std::shared_ptr<Dynamics> thrusterDynamics;

  /// \brief Thruster conversion function
  protected: std::shared_ptr<ConversionFunction> conversionFunction;

  /// \brief Update event
  protected: event::ConnectionPtr updateConnection;

  /// \brief Pointer to the thruster link
  protected: physics::LinkPtr thrusterLink;

  /// \brief Gazebo node
  protected: transport::NodePtr node;

  /// \brief Subscriber to the reference signal topic.
  protected: transport::SubscriberPtr commandSubscriber;

  /// \brief Publisher to the output thrust topic
  protected: transport::PublisherPtr thrustTopicPublisher;

  /// \brief Input command, typically desired angular velocity of the
  ///        rotor.
  protected: double inputCommand;

  /// \brief Latest thrust force in [N]
  protected: double thrustForce;

  /// \brief Time stamp of latest thrust force
  protected: common::Time thrustForceStamp;

  /// \brief Optional: The rotor joint, used for visualization
  protected: physics::JointPtr joint;

  /// \brief: Optional: Commands less than this value will be clamped.
  protected: double clampMin;

  /// \brief: Optional: Commands greater than this value will be clamped.
  protected: double clampMax;

  /// \brief: Optional: Minimum thrust force output
  protected: double thrustMin;

  /// \brief: Optional: Maximum thrust force output
  protected: double thrustMax;

  /// \brief Thruster ID, used to generated topic names automatically
  protected: int thrusterID;

  /// \brief Thruster topics prefix
  protected: std::string myTopicPrefix;

  /// \brief: Optional: Gain factor: Desired angular velocity = command * gain
  protected: double gain;

  /// \brief Optional: Flag to indicate if the thruster is turned on or off
  protected: bool isOn;

  /// \brief Optional: Output thrust efficiency factor of the thruster
  protected: double thrustEfficiency;

  /// \brief Optional: Propeller angular velocity efficiency term
  protected: double propellerEfficiency;

  /// \brief The axis about which the thruster rotates
  protected: ignition::math::Vector3d thrusterAxis;
};
}
#endif  // __UUV_GAZEBO_PLUGINS_THRUSTER_PLUGIN_HH__
