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

#include <uuv_gazebo_ros_plugins/LinearBatteryROSPlugin.h>

#include <gazebo/plugins/LinearBatteryPlugin.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/common/Plugin.hh>

#include <chrono>

namespace gazebo
{
/////////////////////////////////////////////////
LinearBatteryROSPlugin::LinearBatteryROSPlugin()
{
}

/////////////////////////////////////////////////
LinearBatteryROSPlugin::~LinearBatteryROSPlugin()
{
}

/////////////////////////////////////////////////
void LinearBatteryROSPlugin::Load(physics::ModelPtr _parent,
  sdf::ElementPtr _sdf)
{
  try
  {
    LinearBatteryPlugin::Load(_parent, _sdf);
  }
  catch(common::Exception &_e)
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

  // if (_sdf->HasElement("namespace"))
  //   myRobotNamespace = _sdf->Get<std::string>("namespace");

  double updateRate = 2;
  if (_sdf->HasElement("update_rate"))
    updateRate = _sdf->Get<double>("update_rate");

  if (updateRate <= 0.0)
  {
    gzmsg << "Invalid update rate, setting it to 2 Hz, rate=" << updateRate
      << std::endl;
    updateRate = 2;
  }

  //Creates the node including the namespace read in the SDF
  myRosNode =  gazebo_ros::Node::Get(_sdf);
  //this->rosNode.reset(new ros::NodeHandle(this->robotNamespace));

  myBatteryStatePub = myRosNode->create_publisher<sensor_msgs::msg::BatteryState>
    ("battery_state", 0);

  myUpdateTimer = myRosNode->create_wall_timer(
    //duration in seconds
    std::chrono::duration<double, std::ratio<1>>(1. / updateRate),
    std::bind(&LinearBatteryROSPlugin::PublishBatteryState, this));

  gzmsg << "ROS Battery Plugin for link <" << this->link->GetName()
    << "> initialized\n"
    << "\t- Initial charge [Ah]=" << this->q0 << '\n'
    << "\t- Update rate [Hz]=" << updateRate
    << std::endl;
}

/////////////////////////////////////////////////
void LinearBatteryROSPlugin::PublishBatteryState()
{
  myBatteryStateMsg.header.stamp = myRosNode->now();//::Time().now();
  myBatteryStateMsg.header.frame_id = this->link->GetName();

  myBatteryStateMsg.charge = this->q;
  myBatteryStateMsg.percentage = this->q / this->q0;
  myBatteryStateMsg.voltage = this->battery->Voltage();
  myBatteryStateMsg.design_capacity = this->q0;

  myBatteryStateMsg.power_supply_status =
    sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_DISCHARGING;
  myBatteryStateMsg.power_supply_health =
    sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_GOOD;
  myBatteryStateMsg.power_supply_technology =
    sensor_msgs::msg::BatteryState::POWER_SUPPLY_TECHNOLOGY_UNKNOWN;
  myBatteryStateMsg.present = true;

  // Publish battery message
  myBatteryStatePub->publish(myBatteryStateMsg);
}

/////////////////////////////////////////////////
void LinearBatteryROSPlugin::Init()
{
  LinearBatteryPlugin::Init();
}

/////////////////////////////////////////////////
void LinearBatteryROSPlugin::Reset()
{
  LinearBatteryPlugin::Reset();
}

GZ_REGISTER_MODEL_PLUGIN(LinearBatteryROSPlugin)
}
