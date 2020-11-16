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

#include <uuv_gazebo_ros_plugins/JointStatePublisher.h>


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
  //rclcpp::shutdown();
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

  if (!rclcpp::ok())
  {
    gzerr << "ROS was not initialized. Closing plugin..." << std::endl;
    return;
  }

  // this->node = boost::shared_ptr<ros::NodeHandle>(
  //   new ros::NodeHandle(this->robotNamespace));
  // Retrieve the namespace used to publish the joint states
  // if (_sdf->HasElement("robotNamespace"))
  //   myRobotNamespace = _sdf->Get<std::string>("robotNamespace");
  // else
  //   myRobotNamespace = myModel->GetName();


  //Creates a node, including the robot namespace directly read in the SDF
  myRosNode =  gazebo_ros::Node::Get(_sdf);
  //myRosNode =  gazebo_ros::Node::CreateWithArgs("joint_state_publisher", myRobotNamespace);

  if(std::string(myRosNode->get_namespace()).empty())
    gzerr << "JointStatePublisher: robot namespace is empty" << std::endl;
  else
    gzmsg << "JointStatePublisher::robot namespace = " << myRosNode->get_namespace() << std::endl;


  // if (myRobotNamespace[0] != '/')
  //   myRobotNamespace = "/" + myRobotNamespace;

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
  myJointStatePub =
    myRosNode->create_publisher<sensor_msgs::msg::JointState>(
      /*myRobotNamespace +*/ "joint_states", 1);
#if GAZEBO_MAJOR_VERSION >= 8
  this->lastUpdate = this->world->SimTime();
#else
  this->lastUpdate = this->world->GetSimTime();
#endif
  // Connect the update function to the Gazebo callback
  this->updateConnection = gazebo::event::Events::ConnectWorldUpdateBegin(
    std::bind(&JointStatePublisher::OnUpdate, this, std::placeholders::_1));
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
  sensor_msgs::msg::JointState jointState;

  jointState.header.stamp = myRosNode->get_clock()->now();
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
