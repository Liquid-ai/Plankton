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

  if (_sdf->HasElement("updateRate"))
    this->updateRate = _sdf->Get<double>("updateRate");
  else
    this->updateRate = 50;

  const auto FIXED_JOINT{gazebo::physics::Base::EntityType::FIXED_JOINT
                        + gazebo::physics::Base::EntityType::JOINT};

  std::set<std::string> moving_joint_names;

  for (auto &joint : myModel->GetJoints())
  {
    // do not consider fixed joints
    if (joint->GetType() == FIXED_JOINT)
      continue;

    jointState.name.push_back(joint->GetName());
    

#if GAZEBO_MAJOR_VERSION >= 8
      const auto lowerLimit{joint->LowerLimit(0)};
      const auto upperLimit{joint->UpperLimit(0)};
#else
      const auto lowerLimit{joint->GetLowerLimit(0).Radian()};
      const auto upperLimit{joint->GetUpperLimit(0).Radian()};
#endif

      if (lowerLimit == 0. && upperLimit == 0.)
      {
        gzmsg << "\t- " << joint->GetName() << " (0-bounded)" << std::endl;
        continue;
      }

    moving_joint_names.insert(joint->GetName());
    moving_joints++;
    gzmsg << "\t- " << joint->GetName() << std::endl;
  }

  const auto nJoints{jointState.name.size()};
  jointState.position.resize(nJoints, 0);
  jointState.velocity.resize(nJoints, 0);

  // put moving joints in front, we only care about them
  std::partition(jointState.name.begin(), jointState.name.end(), [&](const auto &name)
  {return moving_joint_names.find(name) != moving_joint_names.end();});

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

  jointState.header.stamp = myRosNode->get_clock()->now();

  // only first moving_joints are moving
  for (size_t i = 0; i < moving_joints; ++i)
  {
    const auto & joint{myModel->GetJoint(jointState.name[i])};

#if GAZEBO_MAJOR_VERSION >= 8
      jointState.position[i] = joint->Position(0);
#else
      jointState.position[i] = joint->GetAngle(0).Radian();
#endif
      jointState.velocity[i] = joint->GetVelocity(0);
      // not implemented anyway
      // jointState.effort[i] = joint.GetForce(0);
    }

  myJointStatePub->publish(jointState);
}

}
