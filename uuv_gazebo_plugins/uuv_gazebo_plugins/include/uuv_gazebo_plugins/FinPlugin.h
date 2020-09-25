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

/// \file FinPlugin.hh
/// \brief Model plugin for description of a submarine's fin.

#ifndef __UUV_GAZEBO_PLUGINS_FIN_PLUGIN_HH__
#define __UUV_GAZEBO_PLUGINS_FIN_PLUGIN_HH__

//TODO remove boost dependence
#include <boost/scoped_ptr.hpp>
#include <gazebo/gazebo.hh>
#include <sdf/sdf.hh>

#include <gazebo/msgs/msgs.hh>
#include <uuv_gazebo_plugins/Dynamics.h>
#include <uuv_gazebo_plugins/LiftDragModel.h>

#include "Double.pb.h"

namespace gazebo {

/// \brief Definition of a pointer to the floating point message
typedef const boost::shared_ptr<const uuv_gazebo_plugins_msgs::msgs::Double>
ConstDoublePtr;

class FinPlugin : public ModelPlugin
{
    /// \brief Constructor
    public: FinPlugin();

    /// \brief Destructor
    public: virtual ~FinPlugin();

    // Documentation inherited.
    public: virtual void Load(physics::ModelPtr _model,
                              sdf::ElementPtr _sdf);

    // Documentation inherited.
    public: virtual void Init();

    /// \brief Update the simulation state.
    /// \param[in] _info Information used in the update event.
    public: void OnUpdate(const common::UpdateInfo &_info);

    /// \brief Callback for the input topic subscriber
    protected: void UpdateInput(ConstDoublePtr &_msg);

    /// \brief Reads current velocity topic
    protected: void UpdateCurrentVelocity(ConstVector3dPtr &_msg);

    /// \brief Helper function that builds and return the plugin's topic prefix
    protected: std::string BuildTopicPrefix(const std::string& rosNamespace, int id);

    /// \brief Fin dynamic model
    protected: std::shared_ptr<Dynamics> dynamics;

    /// \brief Lift&Drag model
    protected: std::shared_ptr<LiftDrag> liftdrag;

    /// \brief Update event
    protected: event::ConnectionPtr updateConnection;

    /// \brief Gazebo node
    protected: transport::NodePtr node;

    /// \brief The fin joint
    protected: physics::JointPtr joint;

    /// \brief The fin link
    protected: physics::LinkPtr link;

    /// \brief Subscriber to the reference signal topic.
    protected: transport::SubscriberPtr commandSubscriber;

    /// \brief Publisher to the output thrust topic
    protected: transport::PublisherPtr anglePublisher;

    /// \brief Force component calculated from the lift and drag module
    protected: ignition::math::Vector3d finForce;

    /// \brief Latest input command.
    protected: double inputCommand;

    /// \brief Fin ID
    protected: int finID;

    /// \brief Topic prefix
    protected: std::string myTopicPrefix;

    /// \brief Latest fin angle in [rad].
    protected: double angle;

    /// \brief Time stamp of latest thrust force
    protected: common::Time angleStamp;

    /// \brief Subcriber to current message
    protected: transport::SubscriberPtr currentSubscriber;

    /// \brief Current velocity vector read from topic
    protected: ignition::math::Vector3d currentVelocity;
};
}

#endif
