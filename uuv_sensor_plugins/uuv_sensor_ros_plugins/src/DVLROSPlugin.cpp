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

#include <uuv_sensor_ros_plugins/DVLROSPlugin.h>

#include <tf2_ros/create_timer_ros.h>

namespace gazebo
{
/////////////////////////////////////////////////
DVLROSPlugin::DVLROSPlugin() : ROSBaseModelPlugin()
{
  this->beamTransformsInitialized = false;
}

/////////////////////////////////////////////////
DVLROSPlugin::~DVLROSPlugin()
{ }

/////////////////////////////////////////////////
void DVLROSPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  using std::placeholders::_1;
  using std::placeholders::_2;
  using std::placeholders::_3;
  using std::placeholders::_4;

  ROSBaseModelPlugin::Load(_model, _sdf);

  //std::shared_ptr<rclcpp::Clock> clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
  myBuffer.reset(new tf2_ros::Buffer(myRosNode->get_clock()));
  myTransformListener.reset(new tf2_ros::TransformListener(*myBuffer, myRosNode, false));
  //TODO Totally unsure. New in Eloquent 
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
    myRosNode->get_node_base_interface(),
    myRosNode->get_node_timers_interface()
  );
  myBuffer->setCreateTimerInterface(timer_interface);
  myBuffer->setUsingDedicatedThread(true);
  //

  // Load the link names for all the beams
  std::string beamLinkName;
  GetSDFParam<std::string>(_sdf, "beam_link_name_0", beamLinkName, "");
  GZ_ASSERT(!beamLinkName.empty(), "Beam 0 link name empty");
  this->beamsLinkNames.push_back(beamLinkName);

  GetSDFParam<std::string>(_sdf, "beam_link_name_1", beamLinkName, "");
  GZ_ASSERT(!beamLinkName.empty(), "Beam 1 link name empty");
  this->beamsLinkNames.push_back(beamLinkName);

  GetSDFParam<std::string>(_sdf, "beam_link_name_2", beamLinkName, "");
  GZ_ASSERT(!beamLinkName.empty(), "Beam 2 link name empty");
  this->beamsLinkNames.push_back(beamLinkName);

  GetSDFParam<std::string>(_sdf, "beam_link_name_3", beamLinkName, "");
  GZ_ASSERT(!beamLinkName.empty(), "Beam 3 link name empty");
  this->beamsLinkNames.push_back(beamLinkName);

  // Load the beam output topic names
  std::string beamTopic;
  GetSDFParam<std::string>(_sdf, "beam_topic_0", beamTopic, "");
  GZ_ASSERT(!beamTopic.empty(), "Beam 0 topic name empty");
  this->beamTopics.push_back(beamTopic);

  GetSDFParam<std::string>(_sdf, "beam_topic_1", beamTopic, "");
  GZ_ASSERT(!beamTopic.empty(), "Beam 1 topic name empty");
  this->beamTopics.push_back(beamTopic);

  GetSDFParam<std::string>(_sdf, "beam_topic_2", beamTopic, "");
  GZ_ASSERT(!beamTopic.empty(), "Beam 2 topic name empty");
  this->beamTopics.push_back(beamTopic);

  GetSDFParam<std::string>(_sdf, "beam_topic_3", beamTopic, "");
  GZ_ASSERT(!beamTopic.empty(), "Beam 3 topic name empty");
  this->beamTopics.push_back(beamTopic);

  // Create beam subscribers
  this->beamSub0.reset(new message_filters::Subscriber<sensor_msgs::msg::Range>(
    myRosNode, this->beamTopics[0]));
  this->beamSub1.reset(new message_filters::Subscriber<sensor_msgs::msg::Range>(
    myRosNode, this->beamTopics[1]));
  this->beamSub2.reset(new message_filters::Subscriber<sensor_msgs::msg::Range>(
    myRosNode, this->beamTopics[2]));
  this->beamSub3.reset(new message_filters::Subscriber<sensor_msgs::msg::Range>(
    myRosNode, this->beamTopics[3]));

  for (int i = 0; i < 4; i++)
    this->dvlBeamMsgs.push_back(uuv_sensor_ros_plugins_msgs::msg::DVLBeam());

  // Synchronize the beam topics
  this->syncBeamMessages.reset(new message_filters::TimeSynchronizer<
    sensor_msgs::msg::Range, sensor_msgs::msg::Range,
    sensor_msgs::msg::Range, sensor_msgs::msg::Range>(
      *this->beamSub0.get(), *this->beamSub1.get(), *this->beamSub2.get(),
      *this->beamSub3.get(), 10));

  // Set synchronized callback function for the DVL beams
  this->syncBeamMessages->registerCallback(
    std::bind(&DVLROSPlugin::OnBeamCallback, this, _1, _2, _3, _4));

  // Initialize the default DVL output
  this->rosSensorOutputPub =
    myRosNode->create_publisher<uuv_sensor_ros_plugins_msgs::msg::DVL>(
      this->sensorOutputTopic, 1);

  this->twistPub =
    myRosNode->create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>(
      this->sensorOutputTopic + "_twist", 1);

  // Initialize ROS messages headers
  if (this->enableLocalNEDFrame)
  {
    // Use the local NED frame format
    this->dvlROSMsg.header.frame_id = this->tfLocalNEDFrameMsg.child_frame_id;
    this->twistROSMsg.header.frame_id = this->tfLocalNEDFrameMsg.child_frame_id;
  }
  else
  {
    // Use the link's frame ID
    this->dvlROSMsg.header.frame_id = this->link->GetName();
    this->twistROSMsg.header.frame_id = this->link->GetName();
  }

  double variance = this->noiseSigma * this->noiseSigma;

  // Set covariance
  for (int i = 0; i < 9; i++)
      this->dvlROSMsg.velocity_covariance[i] = 0.0;

  this->dvlROSMsg.velocity_covariance[0] = variance;
  this->dvlROSMsg.velocity_covariance[4] = variance;
  this->dvlROSMsg.velocity_covariance[8] = variance;

  for (int i = 0; i < 36; i++)
    this->twistROSMsg.twist.covariance[i] = 0.0;

  this->twistROSMsg.twist.covariance[0] = variance;
  this->twistROSMsg.twist.covariance[7] = variance;
  this->twistROSMsg.twist.covariance[14] = variance;
  this->twistROSMsg.twist.covariance[21] = -1;  // not available
  this->twistROSMsg.twist.covariance[28] = -1;  // not available
  this->twistROSMsg.twist.covariance[35] = -1;  // not available

  if (this->gazeboMsgEnabled)
  {
    this->gazeboSensorOutputPub =
      this->gazeboNode->Advertise<sensor_msgs::msgs::Dvl>(
        myRobotNamespace + "/" + this->sensorOutputTopic, 1);
  }
}

/////////////////////////////////////////////////
bool DVLROSPlugin::OnUpdate(const common::UpdateInfo& _info)
{
  // Publish sensor state
  this->PublishState();

  if (!this->EnableMeasurement(_info))
    return false;

  if (this->enableLocalNEDFrame)
    this->SendLocalNEDTransform();

  ignition::math::Vector3d bodyVel;

  if (!this->UpdateBeamTransforms())
    return false;

  // Read true body velocity
  // TODO Temporary solution to generate DVL message, use beams in the future
  // instead
#if GAZEBO_MAJOR_VERSION >= 8
  bodyVel = this->link->RelativeLinearVel();
#else
  bodyVel = this->link->GetRelativeLinearVel().Ign();
#endif

  bodyVel.X() += this->GetGaussianNoise(this->noiseAmp);
  bodyVel.Y() += this->GetGaussianNoise(this->noiseAmp);
  bodyVel.Z() += this->GetGaussianNoise(this->noiseAmp);

  if (this->enableLocalNEDFrame)
    bodyVel = this->localNEDFrame.Rot().RotateVector(bodyVel);

  if (this->gazeboMsgEnabled)
  {
    sensor_msgs::msgs::Dvl dvlGazeboMsg;
    double variance = this->noiseSigma * this->noiseSigma;

    for (int i = 0; i < 9; i++)
    {
      if (i == 0 || i == 4 || i == 8)
        dvlGazeboMsg.add_linear_velocity_covariance(variance);
      else
        dvlGazeboMsg.add_linear_velocity_covariance(0.0);
    }

    // Publish simulated measurement
    gazebo::msgs::Vector3d* v = new gazebo::msgs::Vector3d();
    v->set_x(bodyVel.X());
    v->set_y(bodyVel.Y());
    v->set_z(bodyVel.Z());
    dvlGazeboMsg.set_allocated_linear_velocity(v);
    this->gazeboSensorOutputPub->Publish(dvlGazeboMsg);
  }

  // Publish ROS DVL message
  this->dvlROSMsg.header.stamp.sec = _info.simTime.sec;
  this->dvlROSMsg.header.stamp.nanosec = _info.simTime.nsec;

  this->dvlROSMsg.altitude = this->altitude;

  this->dvlROSMsg.beams = this->dvlBeamMsgs;

  this->dvlROSMsg.velocity.x = bodyVel.X();
  this->dvlROSMsg.velocity.y = bodyVel.Y();
  this->dvlROSMsg.velocity.z = bodyVel.Z();
  this->rosSensorOutputPub->publish(this->dvlROSMsg);

  this->twistROSMsg.header.stamp = this->dvlROSMsg.header.stamp;

  this->twistROSMsg.twist.twist.linear.x = bodyVel.X();
  this->twistROSMsg.twist.twist.linear.y = bodyVel.Y();
  this->twistROSMsg.twist.twist.linear.z = bodyVel.Z();

  this->twistPub->publish(this->twistROSMsg);

  // Read the current simulation time
  #if GAZEBO_MAJOR_VERSION >= 8
    this->lastMeasurementTime = this->world->SimTime();
  #else
    this->lastMeasurementTime = this->world->GetSimTime();
  #endif
  return true;
}

/////////////////////////////////////////////////
void DVLROSPlugin::OnBeamCallback(const sensor_msgs::msg::Range::ConstSharedPtr& _range0,
  const sensor_msgs::msg::Range::ConstSharedPtr& _range1,
  const sensor_msgs::msg::Range::ConstSharedPtr& _range2,
  const sensor_msgs::msg::Range::ConstSharedPtr& _range3)
{
  if (_range0->range == _range0->min_range &&
      _range1->range == _range1->min_range &&
      _range2->range == _range2->min_range &&
      _range3->range == _range3->min_range)
  {
    this->altitude = ALTITUDE_OUT_OF_RANGE;
    return;
  }


  if (_range0->range == _range0->max_range &&
      _range1->range == _range1->max_range &&
      _range2->range == _range2->max_range &&
      _range3->range == _range3->max_range)
  {
    this->altitude = ALTITUDE_OUT_OF_RANGE;
    return;
  }

  // TODO Compute the altitude taking into account the vehicle's orientation
  this->altitude =
    0.25 * (_range0->range + _range1->range + _range2->range + _range3->range);

  this->dvlBeamMsgs[0].range = _range0->range;
  this->dvlBeamMsgs[1].range = _range1->range;
  this->dvlBeamMsgs[2].range = _range2->range;
  this->dvlBeamMsgs[3].range = _range3->range;
}

/////////////////////////////////////////////////
bool DVLROSPlugin::UpdateBeamTransforms()
{
  if (this->beamPoses.size() == 4)
    return true;

  geometry_msgs::msg::TransformStamped beamTransform;
  std::string targetFrame, sourceFrame;
  bool success = true;

  for (size_t i = 0; i < this->beamsLinkNames.size(); i++)
  {
    sourceFrame = this->beamsLinkNames[i];
    if (!this->enableLocalNEDFrame)
      targetFrame = this->link->GetName();
    else
      targetFrame = tfLocalNEDFrameMsg.child_frame_id;
    try
    {
      beamTransform = myBuffer->lookupTransform(
        targetFrame, sourceFrame, rclcpp::Time(0));
    }
    catch(tf2::TransformException &ex)
    {
      success = false;
      break;
    }

    ignition::math::Pose3d pose;
    pose.Pos() = ignition::math::Vector3d(
      beamTransform.transform.translation.x,
      beamTransform.transform.translation.y,
      beamTransform.transform.translation.z);
    pose.Rot() = ignition::math::Quaterniond(
      beamTransform.transform.rotation.w,
      beamTransform.transform.rotation.x,
      beamTransform.transform.rotation.y,
      beamTransform.transform.rotation.z);

    this->dvlBeamMsgs[i].pose = geometry_msgs::msg::PoseStamped();
    this->dvlBeamMsgs[i].pose.header.stamp = myRosNode->now();
    this->dvlBeamMsgs[i].pose.header.frame_id = sourceFrame;

    this->dvlBeamMsgs[i].pose.pose.position.x = beamTransform.transform.translation.x;
    this->dvlBeamMsgs[i].pose.pose.position.y = beamTransform.transform.translation.y;
    this->dvlBeamMsgs[i].pose.pose.position.z = beamTransform.transform.translation.z;

    this->dvlBeamMsgs[i].pose.pose.orientation.x = beamTransform.transform.rotation.x;
    this->dvlBeamMsgs[i].pose.pose.orientation.y = beamTransform.transform.rotation.y;
    this->dvlBeamMsgs[i].pose.pose.orientation.z = beamTransform.transform.rotation.z;
    this->dvlBeamMsgs[i].pose.pose.orientation.w = beamTransform.transform.rotation.w;

    this->beamPoses.push_back(pose);
  }
  return success;
}

/////////////////////////////////////////////////
GZ_REGISTER_MODEL_PLUGIN(DVLROSPlugin)
}
