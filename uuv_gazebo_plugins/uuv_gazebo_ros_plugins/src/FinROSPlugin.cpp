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

#include <uuv_gazebo_ros_plugins/FinROSPlugin.h>

#include <string>

#include <gazebo/physics/Base.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>

namespace uuv_simulator_ros
{
//=============================================================================
///////////////////////////////////////////////////////////////////////////////
FinROSPlugin::FinROSPlugin()
{
  this->rosPublishPeriod = gazebo::common::Time(0.05);
  this->lastRosPublishTime = gazebo::common::Time(0.0);
}

//=============================================================================
///////////////////////////////////////////////////////////////////////////////
FinROSPlugin::~FinROSPlugin()
{
#if GAZEBO_MAJOR_VERSION >= 8
  this->rosPublishConnection.reset();
#else
  gazebo::event::Events::DisconnectWorldUpdateBegin(
        this->rosPublishConnection);
#endif
}

//=============================================================================
///////////////////////////////////////////////////////////////////////////////
void FinROSPlugin::SetReference(
    const uuv_gazebo_ros_plugins_msgs::msg::FloatStamped::SharedPtr _msg)
{
  if (std::isnan(_msg->data))
  {
    RCLCPP_WARN(myRosNode->get_logger(), "FinROSPlugin: Ignoring nan command");
    return;
  }

  this->inputCommand = _msg->data;
}

//=============================================================================
///////////////////////////////////////////////////////////////////////////////
gazebo::common::Time FinROSPlugin::GetRosPublishPeriod()
{
  return this->rosPublishPeriod;
}

//=============================================================================
///////////////////////////////////////////////////////////////////////////////
void FinROSPlugin::SetRosPublishRate(double _hz)
{
  if (_hz > 0.0)
    this->rosPublishPeriod = 1.0 / _hz;
  else
    this->rosPublishPeriod = 0.;
}

//=============================================================================
///////////////////////////////////////////////////////////////////////////////
void FinROSPlugin::Init()
{
  FinPlugin::Init();
}

//=============================================================================
///////////////////////////////////////////////////////////////////////////////
void FinROSPlugin::Reset()
{
  this->lastRosPublishTime.Set(0, 0);
}

//=============================================================================
///////////////////////////////////////////////////////////////////////////////
void FinROSPlugin::Load(gazebo::physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
  try {
    FinPlugin::Load(_parent, _sdf);
  } catch(gazebo::common::Exception &_e)
  {
    gzerr << "Error loading plugin."
          << "Please ensure that your model is correct."
          << '\n';
    return;
  }

  if (!rclcpp::ok())
  {
    gzerr << "Not loading plugin since ROS has not been "
          << "properly initialized.  Try starting gazebo with ros plugin:\n"
          << "  gazebo -s libgazebo_ros_api_plugin.so\n";
    return;
  }

  myRosNode = gazebo_ros::Node::CreateWithArgs(_sdf->Get<std::string>("name"));// gazebo_ros::Node::Get(_sdf);
  std::string info = "[FinROSPlugin] Namespace: " + std::string(myRosNode->get_namespace());
  RCLCPP_INFO(myRosNode->get_logger(), info.c_str());
  //this->rosNode.reset(new ros::NodeHandle(""));

  mySubReference = myRosNode->create_subscription<
    uuv_gazebo_ros_plugins_msgs::msg::FloatStamped
    >(this->commandSubscriber->GetTopic(), 10,
      std::bind(&FinROSPlugin::SetReference, this, std::placeholders::_1));

  myPubState = myRosNode->create_publisher<
    uuv_gazebo_ros_plugins_msgs::msg::FloatStamped
    >(this->anglePublisher->GetTopic(), 10);

  std::string wrenchTopic;
  if (_sdf->HasElement("wrench_topic"))
    wrenchTopic = _sdf->Get<std::string>("wrench_topic");
  else
    wrenchTopic = myTopicPrefix + "/wrench_topic";

  myPubFinForce =
    myRosNode->create_publisher<geometry_msgs::msg::WrenchStamped>(wrenchTopic, 10);

  // std::stringstream stream;
  // stream << _parent->GetName() << "/fins/" << this->finID <<
  //   "/get_lift_drag_params";
  std::string getLiftDragSrvName = myTopicPrefix + "/get_lift_drag_params";
  myServicesById["get_lift_drag_params"] = myRosNode->create_service<uuv_gazebo_ros_plugins_msgs::srv::GetListParam>(
    getLiftDragSrvName, std::bind(&FinROSPlugin::GetLiftDragParams, this, std::placeholders::_1, std::placeholders::_2));

  gzmsg << "Fin #" << this->finID << " initialized" << std::endl
    << "\t- Link: " << this->link->GetName() << std::endl
    << "\t- Robot model: " << _parent->GetName() << std::endl
    << "\t- Input command topic: " <<
      this->commandSubscriber->GetTopic() << std::endl
    << "\t- Output topic: " <<
      this->anglePublisher->GetTopic() << std::endl;

  this->rosPublishConnection = gazebo::event::Events::ConnectWorldUpdateBegin(
        std::bind(&FinROSPlugin::RosPublishStates, this));
}

/////////////////////////////////////////////////
void FinROSPlugin::RosPublishStates()
{
  // Limit publish rate according to publish period
  if (this->angleStamp - this->lastRosPublishTime >=
      this->rosPublishPeriod)
  {
    this->lastRosPublishTime = this->angleStamp;

    // Publish the current angle of attack
    uuv_gazebo_ros_plugins_msgs::msg::FloatStamped state_msg;
    state_msg.header.stamp = myRosNode->get_clock()->now();// ros::Time().now();
    state_msg.header.frame_id = this->link->GetName();
    state_msg.data = this->angle;
    myPubState->publish(state_msg);

    // Publish the lift and drag forces
    geometry_msgs::msg::WrenchStamped msg;
    msg.header.stamp = myRosNode->get_clock()->now(); //ros::Time().now();
    msg.header.frame_id = this->link->GetName();
    msg.wrench.force.x = this->finForce.X();
    msg.wrench.force.y = this->finForce.Y();
    msg.wrench.force.z = this->finForce.Z();

    myPubFinForce->publish(msg);
  }
}

/////////////////////////////////////////////////
void FinROSPlugin::GetLiftDragParams(
  const uuv_gazebo_ros_plugins_msgs::srv::GetListParam::Request::SharedPtr /*_req*/,
  uuv_gazebo_ros_plugins_msgs::srv::GetListParam::Response::SharedPtr _res)
{
  _res->description = this->liftdrag->GetType();
  for (auto& item : this->liftdrag->GetListParams())
  {
    _res->tags.push_back(item.first);
    _res->data.push_back(item.second);
  }
}

GZ_REGISTER_MODEL_PLUGIN(FinROSPlugin)
}
