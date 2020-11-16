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

#include <uuv_gazebo_ros_plugins/ThrusterROSPlugin.h>

#include <string>

#include <gazebo/physics/Base.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>

#include <uuv_gazebo_ros_plugins_msgs/msg/float_stamped.hpp>

namespace uuv_simulator_ros
{
//=============================================================================
///////////////////////////////////////////////////////////////////////////////
ThrusterROSPlugin::ThrusterROSPlugin()
{
  this->rosPublishPeriod = gazebo::common::Time(0.05);
  this->lastRosPublishTime = gazebo::common::Time(0.0);
}

//=============================================================================
///////////////////////////////////////////////////////////////////////////////
ThrusterROSPlugin::~ThrusterROSPlugin()
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
void ThrusterROSPlugin::SetThrustReference(
    const uuv_gazebo_ros_plugins_msgs::msg::FloatStamped::SharedPtr _msg)
{
  if (std::isnan(_msg->data))
  {
    RCLCPP_WARN(myRosNode->get_logger(), "ThrusterROSPlugin: Ignoring nan command");
    return;
  }

  this->inputCommand = _msg->data;
}

//=============================================================================
///////////////////////////////////////////////////////////////////////////////
gazebo::common::Time ThrusterROSPlugin::GetRosPublishPeriod()
{
  return this->rosPublishPeriod;
}

//=============================================================================
///////////////////////////////////////////////////////////////////////////////
void ThrusterROSPlugin::SetRosPublishRate(double _hz)
{
  if (_hz > 0.0)
    this->rosPublishPeriod = 1.0 / _hz;
  else
    this->rosPublishPeriod = 0.;
}

//=============================================================================
///////////////////////////////////////////////////////////////////////////////
void ThrusterROSPlugin::Init()
{
  ThrusterPlugin::Init();
}

//=============================================================================
///////////////////////////////////////////////////////////////////////////////
void ThrusterROSPlugin::Reset()
{
  this->lastRosPublishTime.Set(0, 0);
}

//=============================================================================
///////////////////////////////////////////////////////////////////////////////
void ThrusterROSPlugin::Load(gazebo::physics::ModelPtr _parent,
                             sdf::ElementPtr _sdf)
{
  using std::placeholders::_1;
  using std::placeholders::_2;

  try {
    ThrusterPlugin::Load(_parent, _sdf);

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

  // std::string nodeNamespace = myTopicPrefix[myTopicPrefix.size() - 1] == '/' ? 
  //   myTopicPrefix.substr(0, myTopicPrefix.size() - 1) : myTopicPrefix;

  //TODO probably change
  //easy way : no namespace, topic prefix for each topic
  //consistent way, namespace in the SDF, topic prefix without /namespace/. Take care, topics are also created in the parent class
  try {
    myRosNode = gazebo_ros::Node::CreateWithArgs(_sdf->Get<std::string>("name"));//, nodeNamespace);

    gzmsg << "[ThrusterROSPlugin] Node created with name: " << myRosNode->get_name() 
        << ", with ns: " << myRosNode->get_namespace() << "\n";
    
    //this->rosNode.reset(new ros::NodeHandle(""));
    //this->services["set_thrust_force_efficiency"] =
    mySet_thrust_force_efficiencySrv = 
      myRosNode->create_service<uuv_gazebo_ros_plugins_msgs::srv::SetThrusterEfficiency>(
        myTopicPrefix + "set_thrust_force_efficiency",
        std::bind(&ThrusterROSPlugin::SetThrustForceEfficiency, this, _1, _2));

    //this->services["get_thrust_force_efficiency"] =
    myGet_thrust_force_efficiencySrv = 
      myRosNode->create_service<uuv_gazebo_ros_plugins_msgs::srv::GetThrusterEfficiency>(
        myTopicPrefix + "get_thrust_force_efficiency",
        std::bind(&ThrusterROSPlugin::GetThrustForceEfficiency, this, _1, _2));

    //this->services["set_dynamic_state_efficiency"] =
    mySet_dynamic_state_efficiencySrv = 
      myRosNode->create_service<uuv_gazebo_ros_plugins_msgs::srv::SetThrusterEfficiency>(
        myTopicPrefix + "set_dynamic_state_efficiency",
        std::bind(&ThrusterROSPlugin::SetDynamicStateEfficiency, this, _1, _2));

    //this->services["get_dynamic_state_efficiency"] =
    myGet_dynamic_state_efficiency =
      myRosNode->create_service<uuv_gazebo_ros_plugins_msgs::srv::GetThrusterEfficiency>(
        myTopicPrefix + "get_dynamic_state_efficiency",
        std::bind(&ThrusterROSPlugin::GetDynamicStateEfficiency, this, _1, _2));

    //this->services["set_thruster_state"] =
    mySet_thruster_state =
      myRosNode->create_service<uuv_gazebo_ros_plugins_msgs::srv::SetThrusterState>(
        myTopicPrefix + "set_thruster_state",
        std::bind(&ThrusterROSPlugin::SetThrusterState, this, _1, _2));

    //this->services["get_thruster_state"] =
    myGet_thruster_state = 
      myRosNode->create_service<uuv_gazebo_ros_plugins_msgs::srv::GetThrusterState>(
        myTopicPrefix + "get_thruster_state",
        std::bind(&ThrusterROSPlugin::GetThrusterState, this, _1, _2));

    //this->services["get_thruster_conversion_fcn"] =
    myGet_thruster_conversion_fcn = 
      myRosNode->create_service<uuv_gazebo_ros_plugins_msgs::srv::GetThrusterConversionFcn>(
        myTopicPrefix + "get_thruster_conversion_fcn",
        std::bind(&ThrusterROSPlugin::GetThrusterConversionFcn, this, _1, _2));
  
    mySubThrustReference = myRosNode->create_subscription<
      uuv_gazebo_ros_plugins_msgs::msg::FloatStamped
      >(this->commandSubscriber->GetTopic(), 10,
        std::bind(&ThrusterROSPlugin::SetThrustReference, this, _1));

  
    myPubThrust = myRosNode->create_publisher<
      uuv_gazebo_ros_plugins_msgs::msg::FloatStamped
      >(this->thrustTopicPublisher->GetTopic(), 10);

    myPubThrustWrench =
      myRosNode->create_publisher<geometry_msgs::msg::WrenchStamped>(
        this->thrustTopicPublisher->GetTopic() + "_wrench", 10);
      //

    myPubThrusterState = myRosNode->create_publisher<std_msgs::msg::Bool>(
      myTopicPrefix + "is_on", 1);

    myPubThrustForceEff = myRosNode->create_publisher<std_msgs::msg::Float64>(
      myTopicPrefix + "thrust_efficiency", 1);

    myPubDynamicStateEff = myRosNode->create_publisher<std_msgs::msg::Float64>(
      myTopicPrefix + "dynamic_state_efficiency", 1);

    gzmsg << "Thruster #" << this->thrusterID << " initialized" << std::endl
      << "\t- Link: " << this->thrusterLink->GetName() << std::endl
      << "\t- Robot model: " << _parent->GetName() << std::endl
      << "\t- Input command topic: " <<
        this->commandSubscriber->GetTopic() << std::endl
      << "\t- Thrust output topic: " <<
        this->thrustTopicPublisher->GetTopic() << std::endl;

    this->rosPublishConnection = gazebo::event::Events::ConnectWorldUpdateBegin(
      std::bind(&ThrusterROSPlugin::RosPublishStates, this));
  } 
  catch(std::exception& e) 
  {
    gzerr << "Exception when loading plugin: " << e.what() << "\n";
  }
}

//=============================================================================
///////////////////////////////////////////////////////////////////////////////
void ThrusterROSPlugin::RosPublishStates()
{
  // Limit publish rate according to publish period
  if (this->thrustForceStamp - this->lastRosPublishTime >=
      this->rosPublishPeriod)
  {
    this->lastRosPublishTime = this->thrustForceStamp;

    // Publish the thrust force magnitude
    uuv_gazebo_ros_plugins_msgs::msg::FloatStamped thrustMsg;
    thrustMsg.header.stamp = myRosNode->now();//ros::Time().now();
    thrustMsg.header.frame_id = this->thrusterLink->GetName();
    thrustMsg.data = this->thrustForce;
    myPubThrust->publish(thrustMsg);

    // Publish the thrust force vector wrt the thruster frame
    geometry_msgs::msg::WrenchStamped thrustWrenchMsg;
    thrustWrenchMsg.header.stamp = myRosNode->now();//ros::Time().now();
    thrustWrenchMsg.header.frame_id = this->thrusterLink->GetName();
    ignition::math::Vector3d thrustVector = this->thrustForce * this->thrusterAxis;
    thrustWrenchMsg.wrench.force.x = thrustVector.X();
    thrustWrenchMsg.wrench.force.y = thrustVector.Y();
    thrustWrenchMsg.wrench.force.z = thrustVector.Z();
    myPubThrustWrench->publish(thrustWrenchMsg);

    // Publish the thruster current state (ON or OFF)
    std_msgs::msg::Bool isOnMsg;
    isOnMsg.data = this->isOn;
    myPubThrusterState->publish(isOnMsg);

    // Publish thrust output efficiency
    std_msgs::msg::Float64 thrustEffMsg;
    thrustEffMsg.data = this->thrustEfficiency;
    myPubThrustForceEff->publish(thrustEffMsg);

    // Publish dynamic state efficiency
    std_msgs::msg::Float64 dynStateEffMsg;
    dynStateEffMsg.data = this->propellerEfficiency;
    myPubDynamicStateEff->publish(dynStateEffMsg);
  }
}

/////////////////////////////////////////////////
void ThrusterROSPlugin::SetThrustForceEfficiency(
  const uuv_gazebo_ros_plugins_msgs::srv::SetThrusterEfficiency::Request::SharedPtr _req,
  uuv_gazebo_ros_plugins_msgs::srv::SetThrusterEfficiency::Response::SharedPtr _res)
{
  if (_req->efficiency < 0.0 || _req->efficiency > 1.0)
  {
    _res->success = false;
  }
  else
  {
    this->thrustEfficiency = _req->efficiency;
    _res->success = true;
    gzmsg << "Setting thrust efficiency at thruster " <<
      this->thrusterLink->GetName() << "=" << _req->efficiency  * 100
      << "%" << std::endl;
  }
}

/////////////////////////////////////////////////
void ThrusterROSPlugin::GetThrustForceEfficiency(
  const uuv_gazebo_ros_plugins_msgs::srv::GetThrusterEfficiency::Request::SharedPtr /*_req*/,
  uuv_gazebo_ros_plugins_msgs::srv::GetThrusterEfficiency::Response::SharedPtr _res)
{
  _res->efficiency = this->thrustEfficiency;
}

/////////////////////////////////////////////////
void ThrusterROSPlugin::SetDynamicStateEfficiency(
  const uuv_gazebo_ros_plugins_msgs::srv::SetThrusterEfficiency::Request::SharedPtr _req,
  uuv_gazebo_ros_plugins_msgs::srv::SetThrusterEfficiency::Response::SharedPtr _res)
{
  if (_req->efficiency < 0.0 || _req->efficiency > 1.0)
  {
    _res->success = false;
  }
  else
  {
    this->propellerEfficiency = _req->efficiency;
    _res->success = true;
    gzmsg << "Setting propeller efficiency at thruster " <<
      this->thrusterLink->GetName() << "=" << _req->efficiency * 100
      << "%" << std::endl;
  }
}

/////////////////////////////////////////////////
void ThrusterROSPlugin::GetDynamicStateEfficiency(
  const uuv_gazebo_ros_plugins_msgs::srv::GetThrusterEfficiency::Request::SharedPtr /*_req*/,
  uuv_gazebo_ros_plugins_msgs::srv::GetThrusterEfficiency::Response::SharedPtr _res)
{
  _res->efficiency = this->propellerEfficiency;
}

/////////////////////////////////////////////////
void ThrusterROSPlugin::SetThrusterState(
  const uuv_gazebo_ros_plugins_msgs::srv::SetThrusterState::Request::SharedPtr _req,
  uuv_gazebo_ros_plugins_msgs::srv::SetThrusterState::Response::SharedPtr _res)
{
  this->isOn = _req->on;
  gzmsg << "Turning thruster " << this->thrusterLink->GetName() << " " <<
    (this->isOn ? "ON" : "OFF") << std::endl;
  _res->success = true;
}

/////////////////////////////////////////////////
void ThrusterROSPlugin::GetThrusterState(
  const uuv_gazebo_ros_plugins_msgs::srv::GetThrusterState::Request::SharedPtr /*_req*/,
  uuv_gazebo_ros_plugins_msgs::srv::GetThrusterState::Response::SharedPtr _res)
{
  _res->is_on = this->isOn;
}

/////////////////////////////////////////////////
void ThrusterROSPlugin::GetThrusterConversionFcn(
  const uuv_gazebo_ros_plugins_msgs::srv::GetThrusterConversionFcn::Request::SharedPtr /*_req*/,
  uuv_gazebo_ros_plugins_msgs::srv::GetThrusterConversionFcn::Response::SharedPtr _res)
{
  _res->fcn.function_name = this->conversionFunction->GetType();

  double param;

  if (!_res->fcn.function_name.compare("Basic"))
  {
    gzmsg << "ThrusterROSPlugin::GetThrusterConversionFcn::Basic" << std::endl;
    _res->fcn.tags.push_back("rotor_constant");
    this->conversionFunction->GetParam("rotor_constant", param);
    _res->fcn.data.push_back(param);
  }
  else if (!_res->fcn.function_name.compare("Bessa"))
  {
    gzmsg << "ThrusterROSPlugin::GetThrusterConversionFcn::Bessa" << std::endl;
    _res->fcn.tags.push_back("rotor_constant_l");
    this->conversionFunction->GetParam("rotor_constant_l", param);
    _res->fcn.data.push_back(param);

    _res->fcn.tags.push_back("rotor_constant_r");
    this->conversionFunction->GetParam("rotor_constant_r", param);
    _res->fcn.data.push_back(param);

    _res->fcn.tags.push_back("delta_l");
    this->conversionFunction->GetParam("delta_l", param);
    _res->fcn.data.push_back(param);

    _res->fcn.tags.push_back("delta_r");
    this->conversionFunction->GetParam("delta_r", param);
    _res->fcn.data.push_back(param);
  }
  else if (!_res->fcn.function_name.compare("LinearInterp"))
  {
    gzmsg << "ThrusterROSPlugin::GetThrusterConversionFcn::LinearInterp" << std::endl;
    std::map<double, double> table = this->conversionFunction->GetTable();

    for (auto& item : table)
    {
      gzmsg << item.first << " " << item.second << std::endl;
      _res->fcn.lookup_table_input.push_back(item.first);
      _res->fcn.lookup_table_output.push_back(item.second);
    }
  }
}

/////////////////////////////////////////////////
GZ_REGISTER_MODEL_PLUGIN(ThrusterROSPlugin)
}
