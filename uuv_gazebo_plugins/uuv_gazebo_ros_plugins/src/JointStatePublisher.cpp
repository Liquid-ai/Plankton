// Copyright (c) 2016 The UUV Simulator Authors.
// All rights reserved.
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

#include <uuv_gazebo_ros_plugins/JointStatePublisher.h>

#include <boost/bind.hpp>

namespace uuv_simulator_ros
{
GZ_REGISTER_MODEL_PLUGIN(JointStatePublisher)

//=============================================================================
///////////////////////////////////////////////////////////////////////////////
JointStatePublisher::JointStatePublisher()
{
    myModel = NULL;
    this->world = NULL;
}

//=============================================================================
///////////////////////////////////////////////////////////////////////////////
JointStatePublisher::~JointStatePublisher()
{
  rclcpp::shutdown();
    //this->node->shutdown();
}

//=============================================================================
///////////////////////////////////////////////////////////////////////////////
void JointStatePublisher::Load(gazebo::physics::ModelPtr _parent,
  sdf::ElementPtr _sdf)
{
  myModel = _parent;

  GZ_ASSERT(myModel != NULL, "Invalid model pointer");

  this->world = myModel->GetWorld();

  if (!rclcpp::is_initialized())
  {
    gzerr << "ROS was not initialized. Closing plugin..." << std::endl;
    return;
  }

  myNode = rclcpp::Node::make_unique(myRobotNamespace);
  // this->node = boost::shared_ptr<ros::NodeHandle>(
  //   new ros::NodeHandle(this->robotNamespace));
  // Retrieve the namespace used to publish the joint states
  if (_sdf->HasElement("robotNamespace"))
    myRobotNamespace = _sdf->Get<std::string>("robotNamespace");
  else
    myRobotNamespace = myModel->GetName();

  gzmsg << "JointStatePublisher::robotNamespace="
    << myRobotNamespace << std::endl;

  if (myRobotNamespace[0] != '/')
    myRobotNamespace = "/" + myRobotNamespace;

  if (_sdf->HasElement("updateRate"))
    this->updateRate = _sdf->Get<double>("updateRate");
  else
    this->updateRate = 50;

  gzmsg << "JointStatePublisher::Retrieving moving joints:" << std::endl;
  this->movingJoints.clear();
  double upperLimit, lowerLimit;
  for (auto &joint : myModel->GetJoints())
  {
#if GAZEBO_MAJOR_VERSION >= 8
  lowerLimit = joint->LowerLimit(0);
  upperLimit = joint->UpperLimit(0);
#else
  lowerLimit = joint->GetLowerLimit(0).Radian();
  upperLimit = joint->GetUpperLimit(0).Radian();
#endif
    if (lowerLimit  == 0 && upperLimit == 0)
      continue;
    else if (joint->GetType() == gazebo::physics::Base::EntityType::FIXED_JOINT)
      continue;
    else
    {
      this->movingJoints.push_back(joint->GetName());
      gzmsg << "\t- " << joint->GetName() << std::endl;
    }
  }

  GZ_ASSERT(this->updateRate > 0, "Update rate must be positive");

  // Setting the update period
  this->updatePeriod = 1.0 / this->updateRate;

  // Advertise the joint states topic
  //TODO check namespace...why was the NodeHandle prefix repeated ?
  myJointStatePub =
    myNode->create_publisher<sensor_msgs::msg::JointState>(
      myRobotNamespace + "/joint_states", 1);
#if GAZEBO_MAJOR_VERSION >= 8
  this->lastUpdate = this->world->SimTime();
#else
  this->lastUpdate = this->world->GetSimTime();
#endif
  // Connect the update function to the Gazebo callback
  this->updateConnection = gazebo::event::Events::ConnectWorldUpdateBegin(
    boost::bind(&JointStatePublisher::OnUpdate, this, _1));
}

//=============================================================================
///////////////////////////////////////////////////////////////////////////////
void JointStatePublisher::OnUpdate(const gazebo::common::UpdateInfo & /*_info*/)
{
#if GAZEBO_MAJOR_VERSION >= 8
  gazebo::common::Time simTime = this->world->SimTime();
#else
  gazebo::common::Time simTime = this->world->GetSimTime();
#endif
  if (simTime - this->lastUpdate >= this->updatePeriod)
  {
    this->PublishJointStates();
    this->lastUpdate = simTime;
  }
}

//=============================================================================
///////////////////////////////////////////////////////////////////////////////
void JointStatePublisher::PublishJointStates()
{
  //ros::Time stamp = ros::Time::now();
  sensor_msgs::msg::JointState jointState;

  jointState.header.stamp = myNode->get_clock()->now();
  // Resize containers
  jointState.name.resize(myModel->GetJointCount());
  jointState.position.resize(myModel->GetJointCount());
  jointState.velocity.resize(myModel->GetJointCount());
  jointState.effort.resize(myModel->GetJointCount());

  int i = 0;
  for (auto &joint : myModel->GetJoints())
  {
    if (!this->IsIgnoredJoint(joint->GetName()))
    {
      jointState.name[i] = joint->GetName();
#if GAZEBO_MAJOR_VERSION >= 8
      jointState.position[i] = joint->Position(0);
#else
      jointState.position[i] = joint->GetAngle(0).Radian();
#endif
      jointState.velocity[i] = joint->GetVelocity(0);
      jointState.effort[i] = joint->GetForce(0);
    }
      else
    {
      jointState.name[i] = joint->GetName();
      jointState.position[i] = 0.0;
      jointState.velocity[i] = 0.0;
      jointState.effort[i] = 0.0;
    }

    ++i;
  }

  myJointStatePub->publish(jointState);
}

//=============================================================================
///////////////////////////////////////////////////////////////////////////////
bool JointStatePublisher::IsIgnoredJoint(std::string _jointName)
{
  if (this->movingJoints.empty()) return true;
  for (auto joint : this->movingJoints)
    if (_jointName.compare(joint) == 0)
      return false;
  return true;
}
}
