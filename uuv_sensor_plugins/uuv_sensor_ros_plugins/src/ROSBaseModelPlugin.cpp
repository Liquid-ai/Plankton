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

#include <uuv_sensor_ros_plugins/ROSBaseModelPlugin.h>

#include <memory>

namespace gazebo
{

/////////////////////////////////////////////////
ROSBaseModelPlugin::ROSBaseModelPlugin()
{
  // Initialize local NED frame
  this->localNEDFrame = ignition::math::Pose3d::Zero;
  this->localNEDFrame.Rot() = ignition::math::Quaterniond(
    ignition::math::Vector3d(M_PI, 0, 0));
  // Initialize the local NED frame
  //this->tfLocalNEDFrame.setOrigin(tf2::Vector3(0.0, 0.0, 0.0));
  tf2::Quaternion quat;
  quat.setRPY(M_PI, 0.0, 0.0);
  //this->tfLocalNEDFrame.setRotation(quat);
    //tf2::createQuaternionFromRPY(M_PI, 0.0, 0.0));

  //Refactor with tf2::convert
  // tf2::Stamped<tf2::Transform> tf2Tr;
  // tf2Tr.setOrigin(tf2::Vector3(0.0, 0.0, 0.0));
  // tf2Tr.setRotation(quat);
  // tf2::convert(tf2Tr, tfLocalNEDFrameMsg);

  geometry_msgs::msg::Vector3 translation; translation.x = 0.0;translation.y = 0.0;translation.z = 0.0;
    tfLocalNEDFrameMsg.transform.set__translation(translation);
  this->tfLocalNEDFrameMsg.transform.rotation.x = quat.x();
  this->tfLocalNEDFrameMsg.transform.rotation.y = quat.y();
  this->tfLocalNEDFrameMsg.transform.rotation.z = quat.z();
  this->tfLocalNEDFrameMsg.transform.rotation.w = quat.w();

  // NB: initialization of TF broadcaster removed as the ros node is not initialized)
}

/////////////////////////////////////////////////
ROSBaseModelPlugin::~ROSBaseModelPlugin()
{
}

/////////////////////////////////////////////////
void ROSBaseModelPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  // Initialize model pointer
  this->model = _model;

  // Store world pointer
  this->world = this->model->GetWorld();

  std::string linkName;
  GZ_ASSERT(_sdf->HasElement("link_name"), "No link name provided");
  GetSDFParam<std::string>(_sdf, "link_name", linkName, "");
  GZ_ASSERT(!linkName.empty(), "Link name string is empty");

  // Get flag to enable generation of Gazebo messages
  GetSDFParam<bool>(_sdf, "enable_local_ned_frame", this->enableLocalNEDFrame,
    true);

  if (_sdf->HasElement("reference_link_name"))
  {
    std::string refLinkName;
    GetSDFParam<std::string>(_sdf, "reference_link_name", refLinkName, "");
    if (!refLinkName.empty())
    {
      this->referenceLink = this->model->GetLink(refLinkName);
      GZ_ASSERT(this->referenceLink != NULL, "Invalid reference link");
      this->referenceFrameID = refLinkName;
    }
  }

  // Get sensor link
  this->link = this->model->GetLink(linkName);
  GZ_ASSERT(this->link != NULL, "Invalid link pointer");

  // Set the frame IDs for the local NED frame
  this->tfLocalNEDFrameMsg.header.frame_id = this->link->GetName();
  this->tfLocalNEDFrameMsg.child_frame_id = this->link->GetName() + "_ned";

  //Call to base class to initialize common stuff
  this->InitBasePlugin(_sdf);

  if(!myRosNode)
    throw std::runtime_error(std::string(myRosNode->get_name()) + ": ROS node NULL");

  // Initialize TF broadcaster
  this->tfBroadcaster.reset(new tf2_ros::TransformBroadcaster(myRosNode));

  // Bind the sensor update callback function to the world update event
  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
      std::bind(&ROSBasePlugin::OnUpdate, this, std::placeholders::_1));
}

/////////////////////////////////////////////////
bool ROSBaseModelPlugin::OnUpdate(const common::UpdateInfo&)
{
  return true;
}

/////////////////////////////////////////////////
void ROSBaseModelPlugin::SendLocalNEDTransform()
{
  geometry_msgs::msg::TransformStamped msg;
  auto time = myRosNode->now();
  //builtin_interfaces::msg::Time msgTime;
  
  this->tfLocalNEDFrameMsg.header.stamp = time;    
  tf2::TimePoint(std::chrono::nanoseconds(time.nanoseconds()));
  //tf2::transformStampedTFToMsg(this->tfLocalNEDFrame, msg);
  this->tfBroadcaster->sendTransform(msg);  
}

}
