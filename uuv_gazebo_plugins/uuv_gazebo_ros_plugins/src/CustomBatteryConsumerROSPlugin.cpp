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

#include <uuv_gazebo_ros_plugins/CustomBatteryConsumerROSPlugin.h>

namespace gazebo
{
/////////////////////////////////////////////////
CustomBatteryConsumerROSPlugin::CustomBatteryConsumerROSPlugin()
{
  this->isDeviceOn = true;
}

/////////////////////////////////////////////////
CustomBatteryConsumerROSPlugin::~CustomBatteryConsumerROSPlugin()
{
}

/////////////////////////////////////////////////
void CustomBatteryConsumerROSPlugin::Load(physics::ModelPtr _parent,
  sdf::ElementPtr _sdf)
{
  if (!rclcpp::ok())
  {
    gzerr << "Not loading plugin since ROS has not been "
          << "properly initialized.  Try starting gazebo with ros plugin:\n"
          << "  gazebo -s libgazebo_ros_api_plugin.so\n";
    return;
  }

  //Create the ROS node with the plugin's name
  //TODO Check uniqueness of the node
  myRosNode = gazebo_ros::Node::CreateWithArgs(_sdf->Get<std::string>("name"));  //gazebo_ros::Node::Get(_sdf);
  // new ros::NodeHandle(""));

  GZ_ASSERT(_sdf->HasElement("link_name"), "Battery link name (link_name) is missing");
  this->linkName = _sdf->Get<std::string>("link_name");
  physics::LinkPtr link = _parent->GetLink(this->linkName);
  GZ_ASSERT(link, "Consumer plugin: battery link was NULL");

  GZ_ASSERT(_sdf->HasElement("battery_name"), "Battery name is missing");
  this->batteryName = _sdf->Get<std::string>("battery_name");
  this->battery = link->Battery(this->batteryName);
  GZ_ASSERT(this->battery, "Battery was NULL");

  GZ_ASSERT(_sdf->HasElement("power_load"), "Power load is missing");
  this->powerLoad = _sdf->Get<double>("power_load");

  GZ_ASSERT(this->powerLoad > 0, "Power load must be greater than zero");

  // Adding consumer
  this->consumerID = this->battery->AddConsumer();

  if (_sdf->HasElement("topic_device_state"))
  {
    std::string topicName = _sdf->Get<std::string>("topic_device_state");
    if (!topicName.empty())
        myDeviceStateSub = myRosNode->create_subscription<std_msgs::msg::Bool>(
          topicName, 1,
          std::bind(&CustomBatteryConsumerROSPlugin::UpdateDeviceState,
          this, std::placeholders::_1));
  }
  else
  {
    // In the case the device is always on, then set the power load only once
    this->UpdatePowerLoad(this->powerLoad);
  }

  gzmsg << "CustomBatteryConsumerROSPlugin::Device <"
    << _sdf->Get<std::string>("name") 
    << "> added as battery consumer for the battery <" 
    << this->linkName << ">" << std::endl
    << "\t- ID=" << this->consumerID << std::endl
    << "\t- Power load [W]=" << this->powerLoad
    << std::endl;
}

/////////////////////////////////////////////////
void CustomBatteryConsumerROSPlugin::UpdateDeviceState(
  const std_msgs::msg::Bool::SharedPtr _msg)
{
  this->isDeviceOn = _msg->data;
  if (this->isDeviceOn)
    this->UpdatePowerLoad(this->powerLoad);
  else
    this->UpdatePowerLoad(0.0);
}

/////////////////////////////////////////////////
void CustomBatteryConsumerROSPlugin::UpdatePowerLoad(double _powerLoad)
{
  if (!this->battery->SetPowerLoad(this->consumerID, _powerLoad))
    gzerr << "Error setting the consumer power load" << std::endl;
}

GZ_REGISTER_MODEL_PLUGIN(CustomBatteryConsumerROSPlugin)
}
