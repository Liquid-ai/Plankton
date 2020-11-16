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

#include <uuv_gazebo_ros_plugins/UnderwaterObjectROSPlugin.h>

#include <gazebo/physics/Base.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/physics/Link.hh>

namespace uuv_simulator_ros
{
/////////////////////////////////////////////////
UnderwaterObjectROSPlugin::UnderwaterObjectROSPlugin()
{
}

/////////////////////////////////////////////////
UnderwaterObjectROSPlugin::~UnderwaterObjectROSPlugin()
{
  //rclcpp::shutdown();
  //this->rosNode->shutdown();
}

/////////////////////////////////////////////////
void UnderwaterObjectROSPlugin::Load(gazebo::physics::ModelPtr _parent,
                             sdf::ElementPtr _sdf)
{
  using std::placeholders::_1;
  using std::placeholders::_2;

  if (!rclcpp::ok())
  {
    gzerr << "Not loading plugin since ROS has not been "
          << "properly initialized.  Try starting gazebo with ros plugin:\n"
          << "  gazebo -s libgazebo_ros_api_plugin.so\n";
    return;
  }

  //TODO change to be consistent with namespace handling
  //-add namespace
  //-remove topic prefix + "/"
  myRosNode =  gazebo_ros::Node::CreateWithArgs(_sdf->Get<std::string>("name"));//, _parent->GetName());
  gzmsg << "[UnderwaterObjectROSPlugin]: Node created. Name: " << myRosNode->get_name() 
    << ", namespace: " << myRosNode->get_namespace() << std::endl;

  //TODO Not sure about the behaviour of TransformBroadcaster with ROS 2...
  tfBroadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(myRosNode);
  //this->rosNode.reset(new ros::NodeHandle(""));

  try
  {
    UnderwaterObjectPlugin::Load(_parent, _sdf);
  }
  catch(gazebo::common::Exception &_e)
  {
    gzerr << "Error loading plugin."
          << "Please ensure that your model is correct."
          << '\n';
    return;
  }

  mySubLocalCurVel = myRosNode->create_subscription<geometry_msgs::msg::Vector3>(
    _parent->GetName() + "/current_velocity", 10,
    std::bind(&UnderwaterObjectROSPlugin::UpdateLocalCurrentVelocity,
    this, _1));

  this->services["set_use_global_current_velocity"] =
    myRosNode->create_service<uuv_gazebo_ros_plugins_msgs::srv::SetUseGlobalCurrentVel>(
      _parent->GetName() + "/set_use_global_current_velocity",
      std::bind(&UnderwaterObjectROSPlugin::SetUseGlobalCurrentVel, this, _1, _2));

  this->services["set_added_mass_scaling"] =
    myRosNode->create_service<uuv_gazebo_ros_plugins_msgs::srv::SetFloat>(
      _parent->GetName() + "/set_added_mass_scaling",
      std::bind(&UnderwaterObjectROSPlugin::SetScalingAddedMass, this, _1, _2));

  this->services["get_added_mass_scaling"] =
    myRosNode->create_service<uuv_gazebo_ros_plugins_msgs::srv::GetFloat>(
      _parent->GetName() + "/get_added_mass_scaling",
      std::bind(&UnderwaterObjectROSPlugin::GetScalingAddedMass, this, _1, _2));

  this->services["set_damping_scaling"] =
    myRosNode->create_service<uuv_gazebo_ros_plugins_msgs::srv::SetFloat>(
        _parent->GetName() + "/set_damping_scaling",
        std::bind(&UnderwaterObjectROSPlugin::SetScalingDamping, this, _1, _2));

  this->services["get_damping_scaling"] =
    myRosNode->create_service<uuv_gazebo_ros_plugins_msgs::srv::GetFloat>(
      _parent->GetName() + "/get_damping_scaling",
        std::bind(&UnderwaterObjectROSPlugin::GetScalingDamping, this, _1, _2));

  this->services["set_volume_scaling"] =
    myRosNode->create_service<uuv_gazebo_ros_plugins_msgs::srv::SetFloat>(
        _parent->GetName() + "/set_volume_scaling",
        std::bind(&UnderwaterObjectROSPlugin::SetScalingVolume, this, _1, _2));

  this->services["get_volume_scaling"] =
    myRosNode->create_service<uuv_gazebo_ros_plugins_msgs::srv::GetFloat>(
        _parent->GetName() + "/get_volume_scaling",
        std::bind(&UnderwaterObjectROSPlugin::GetScalingVolume, this, _1, _2));

  this->services["set_fluid_density"] =
    myRosNode->create_service<uuv_gazebo_ros_plugins_msgs::srv::SetFloat>(
        _parent->GetName() + "/set_fluid_density",
        std::bind(&UnderwaterObjectROSPlugin::SetFluidDensity, this, _1, _2));

  this->services["get_fluid_density"] =
    myRosNode->create_service<uuv_gazebo_ros_plugins_msgs::srv::GetFloat>(
        _parent->GetName() + "/get_fluid_density",
        std::bind(&UnderwaterObjectROSPlugin::GetFluidDensity, this, _1, _2));

  this->services["set_volume_offset"] =
    myRosNode->create_service<uuv_gazebo_ros_plugins_msgs::srv::SetFloat>(
      _parent->GetName() + "/set_volume_offset",
      std::bind(&UnderwaterObjectROSPlugin::SetOffsetVolume, this, _1, _2));

  this->services["get_volume_offset"] =
    myRosNode->create_service<uuv_gazebo_ros_plugins_msgs::srv::GetFloat>(
      _parent->GetName() + "/get_volume_offset",
      std::bind(&UnderwaterObjectROSPlugin::GetOffsetVolume, this, _1, _2));

  this->services["set_added_mass_offset"] =
    myRosNode->create_service<uuv_gazebo_ros_plugins_msgs::srv::SetFloat>(
      _parent->GetName() + "/set_added_mass_offset",
      std::bind(&UnderwaterObjectROSPlugin::SetOffsetAddedMass, this, _1, _2));

  this->services["get_added_mass_offset"] =
    myRosNode->create_service<uuv_gazebo_ros_plugins_msgs::srv::GetFloat>(
      _parent->GetName() + "/get_added_mass_offset",
      std::bind(&UnderwaterObjectROSPlugin::GetOffsetAddedMass, this, _1, _2));

  this->services["set_linear_damping_offset"] =
    myRosNode->create_service<uuv_gazebo_ros_plugins_msgs::srv::SetFloat>(
      _parent->GetName() + "/set_linear_damping_offset",
      std::bind(&UnderwaterObjectROSPlugin::SetOffsetLinearDamping, this, _1, _2));

  this->services["get_linear_damping_offset"] =
    myRosNode->create_service<uuv_gazebo_ros_plugins_msgs::srv::GetFloat>(
      _parent->GetName() + "/get_linear_damping_offset",
      std::bind(&UnderwaterObjectROSPlugin::GetOffsetLinearDamping, this, _1, _2));

  this->services["set_linear_forward_speed_damping_offset"] =
    myRosNode->create_service<uuv_gazebo_ros_plugins_msgs::srv::SetFloat>(
      _parent->GetName() + "/set_linear_forward_speed_damping_offset",
      std::bind(&UnderwaterObjectROSPlugin::SetOffsetLinearForwardSpeedDamping, this, _1, _2));

  this->services["get_linear_forward_speed_damping_offset"] =
    myRosNode->create_service<uuv_gazebo_ros_plugins_msgs::srv::GetFloat>(
      _parent->GetName() + "/get_linear_forward_speed_damping_offset",
      std::bind(&UnderwaterObjectROSPlugin::GetOffsetLinearForwardSpeedDamping, this, _1, _2));

  this->services["set_nonlinear_damping_offset"] =
    myRosNode->create_service<uuv_gazebo_ros_plugins_msgs::srv::SetFloat>(
      _parent->GetName() + "/set_nonlinear_damping_offset",
      std::bind(&UnderwaterObjectROSPlugin::SetOffsetNonLinearDamping, this, _1, _2));

  this->services["get_nonlinear_damping_offset"] =
    myRosNode->create_service<uuv_gazebo_ros_plugins_msgs::srv::GetFloat>(
      _parent->GetName() + "/get_nonlinear_damping_offset",
      std::bind(&UnderwaterObjectROSPlugin::GetOffsetNonLinearDamping, this, _1, _2));

  this->services["get_model_properties"] =
    myRosNode->create_service<uuv_gazebo_ros_plugins_msgs::srv::GetModelProperties>(
      _parent->GetName() + "/get_model_properties",
      std::bind(&UnderwaterObjectROSPlugin::GetModelProperties, this, _1, _2));

  myRosHydroPub["current_velocity_marker"] =
  //myCurrent_velocity_marker =
    myRosNode->create_publisher<visualization_msgs::msg::Marker>
    (_parent->GetName() + "/current_velocity_marker", 0);

  myRosHydroPub["using_global_current_velocity"] =
  //myUsing_global_current_velocity =
    myRosNode->create_publisher<std_msgs::msg::Bool>
    (_parent->GetName() + "/using_global_current_velocity", 0);

  myRosHydroPub["is_submerged"] =
  //myIs_submerged =
    myRosNode->create_publisher<std_msgs::msg::Bool>
    (_parent->GetName() + "/is_submerged", 0);

  this->nedTransform.header.frame_id = this->model->GetName() + "/base_link";
  this->nedTransform.child_frame_id = this->model->GetName() + "/base_link_ned";
  this->nedTransform.transform.translation.x = 0;
  this->nedTransform.transform.translation.y = 0;
  this->nedTransform.transform.translation.z = 0;
  tf2::Quaternion quat;
  quat.setRPY(M_PI, 0, 0);
  this->nedTransform.transform.rotation.x = quat.x();
  this->nedTransform.transform.rotation.y = quat.y();
  this->nedTransform.transform.rotation.z = quat.z();
  this->nedTransform.transform.rotation.w = quat.w();
}

/////////////////////////////////////////////////
void UnderwaterObjectROSPlugin::Init()
{
  UnderwaterObjectPlugin::Init();
}

/////////////////////////////////////////////////
void UnderwaterObjectROSPlugin::Reset()
{ }

/////////////////////////////////////////////////
void UnderwaterObjectROSPlugin::Update(const gazebo::common::UpdateInfo &_info)
{
  UnderwaterObjectPlugin::Update(_info);

  this->nedTransform.header.stamp = myRosNode->now();//ros::Time::now();
  this->tfBroadcaster->sendTransform(this->nedTransform);
}

/////////////////////////////////////////////////
void UnderwaterObjectROSPlugin::InitDebug(gazebo::physics::LinkPtr _link,
  gazebo::HydrodynamicModelPtr _hydro)
{
  UnderwaterObjectPlugin::InitDebug(_link, _hydro);

  // Publish the stamped wrench topics if the debug flag is on
  for (std::map<std::string,
    gazebo::transport::PublisherPtr>::iterator it = this->hydroPub.begin();
    it != this->hydroPub.end(); ++it)
  {
    myRosHydroPub[it->first] =
      myRosNode->create_publisher<geometry_msgs::msg::WrenchStamped>(
        it->second->GetTopic(), 10);
      gzmsg << "ROS TOPIC: " << it->second->GetTopic() << std::endl;
  }
}

/////////////////////////////////////////////////
void UnderwaterObjectROSPlugin::PublishRestoringForce(
  gazebo::physics::LinkPtr _link)
{
  // Call base class method
  UnderwaterObjectPlugin::PublishRestoringForce(_link);

  // Publish data in a ROS topic
  if (this->models.count(_link))
  {
    if (!this->models[_link]->GetDebugFlag())
      return;

    ignition::math::Vector3d restoring = this->models[_link]->GetStoredVector(
      RESTORING_FORCE);

    geometry_msgs::msg::WrenchStamped msg;
    this->GenWrenchMsg(restoring,
      ignition::math::Vector3d(0, 0, 0), msg);
    auto tmpPub = std::static_pointer_cast<
      rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>>(myRosHydroPub[_link->GetName() + "/restoring"]);
    tmpPub->publish(msg);
  }
}

/////////////////////////////////////////////////
void UnderwaterObjectROSPlugin::PublishIsSubmerged()
{
  if (this->baseLinkName.empty())
    gzwarn << "Base link name string is empty" << std::endl;
  std_msgs::msg::Bool isSubmerged;
  isSubmerged.data = this->models[this->model->GetLink(this->baseLinkName)]->IsSubmerged();

  auto tmpPub = std::static_pointer_cast<
      rclcpp::Publisher<std_msgs::msg::Bool>>(myRosHydroPub["is_submerged"]);
  tmpPub->publish(isSubmerged);
}

/////////////////////////////////////////////////
void UnderwaterObjectROSPlugin::PublishCurrentVelocityMarker()
{
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = "world";
  marker.header.stamp = myRosNode->now();//ros::Time();
  marker.ns = this->model->GetName() + "/current_velocity_marker";
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::ARROW;
  // Creating the arrow marker for the current velocity information
  // (orientation only, magnitude has to be read from the topic)
  if (this->flowVelocity.Length() > 0)
  {
    marker.action = visualization_msgs::msg::Marker::ADD;
    ignition::math::Pose3d pose;
#if GAZEBO_MAJOR_VERSION >= 8
    pose = this->model->WorldPose();
#else
    pose = this->model->GetWorldPose().Ign();
#endif
    double yaw = std::atan2(this->flowVelocity.Y(), this->flowVelocity.X());
    double pitch = std::atan2(
      this->flowVelocity.Z(),
      std::sqrt(std::pow(this->flowVelocity.X(), 2) +
        std::pow(this->flowVelocity.Y(), 2)));

    ignition::math::Quaterniond qt(0.0, -pitch, yaw);
    marker.pose.position.x = pose.Pos().X();
    marker.pose.position.y = pose.Pos().Y();
    marker.pose.position.z = pose.Pos().Z() + 1.5;
    marker.pose.orientation.x = qt.X();
    marker.pose.orientation.y = qt.Y();
    marker.pose.orientation.z = qt.Z();
    marker.pose.orientation.w = qt.W();
    marker.scale.x = 1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.color.a = 1.0;
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;
  }
  else
  {
    marker.action = visualization_msgs::msg::Marker::DELETE;
  }
  // Publish current velocity RViz marker
  std::static_pointer_cast<
      rclcpp::Publisher<visualization_msgs::msg::Marker>>(myRosHydroPub["current_velocity_marker"])->publish(marker);
  //this->rosHydroPub["current_velocity_marker"].publish(marker);
  // Publishing flag for usage of global current velocity
  std_msgs::msg::Bool useGlobalMsg;
  useGlobalMsg.data = this->useGlobalCurrent;
  std::static_pointer_cast<
      rclcpp::Publisher<std_msgs::msg::Bool>>(myRosHydroPub["using_global_current_velocity"])->publish(useGlobalMsg);
  //this->rosHydroPub["using_global_current_velocity"].publish(useGlobalMsg);
}

/////////////////////////////////////////////////
void UnderwaterObjectROSPlugin::PublishHydrodynamicWrenches(
  gazebo::physics::LinkPtr _link)
{
  // Call base class method
  UnderwaterObjectPlugin::PublishRestoringForce(_link);

  // Publish data in a ROS topic
  if (this->models.count(_link))
  {
    if (!this->models[_link]->GetDebugFlag())
      return;
    geometry_msgs::msg::WrenchStamped msg;
    ignition::math::Vector3d force, torque;

    // Publish wrench generated by the acceleration of fluid around the object
    force = this->models[_link]->GetStoredVector(UUV_ADDED_MASS_FORCE);
    torque = this->models[_link]->GetStoredVector(UUV_ADDED_MASS_TORQUE);

    this->GenWrenchMsg(force, torque, msg);
    std::static_pointer_cast<
      rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>>(myRosHydroPub[_link->GetName() + "/added_mass"])->publish(msg);
    //this->rosHydroPub[_link->GetName() + "/added_mass"].publish(msg);

    // Publish wrench generated by the fluid damping
    force = this->models[_link]->GetStoredVector(UUV_DAMPING_FORCE);
    torque = this->models[_link]->GetStoredVector(UUV_DAMPING_TORQUE);

    this->GenWrenchMsg(force, torque, msg);
    std::static_pointer_cast<
      rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>>(myRosHydroPub[_link->GetName() + "/damping"])->publish(msg);
    //this->rosHydroPub[_link->GetName() + "/damping"].publish(msg);

    // Publish wrench generated by the Coriolis forces
    force = this->models[_link]->GetStoredVector(UUV_ADDED_CORIOLIS_FORCE);
    torque = this->models[_link]->GetStoredVector(UUV_ADDED_CORIOLIS_TORQUE);

    this->GenWrenchMsg(force, torque, msg);

    std::static_pointer_cast<
      rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>>(myRosHydroPub[_link->GetName() + "/added_coriolis"])->publish(msg);
    //this->rosHydroPub[_link->GetName() + "/added_coriolis"].publish(msg);
  }
}

/////////////////////////////////////////////////
void UnderwaterObjectROSPlugin::GenWrenchMsg(
  ignition::math::Vector3d _force, ignition::math::Vector3d _torque,
  geometry_msgs::msg::WrenchStamped &_output)
{
  _output.wrench.force.x = _force.X();
  _output.wrench.force.y = _force.Y();
  _output.wrench.force.z = _force.Z();

  _output.wrench.torque.x = _torque.X();
  _output.wrench.torque.y = _torque.Y();
  _output.wrench.torque.z = _torque.Z();
#if GAZEBO_MAJOR_VERSION >= 8
  _output.header.stamp = rclcpp::Time(this->world->SimTime().sec, this->world->SimTime().nsec);//ros::Time(this->world->SimTime().Double());
#else
  _output.header.stamp = rclcpp::Time(this->world->SimTime().sec, this->world->SimTime().nsec);//ros::Time(this->world->GetSimTime().Double());
#endif
}

/////////////////////////////////////////////////
void UnderwaterObjectROSPlugin::UpdateLocalCurrentVelocity(
  const geometry_msgs::msg::Vector3::SharedPtr _msg)
{
  if (!this->useGlobalCurrent)
  {
    this->flowVelocity.X() = _msg->x;
    this->flowVelocity.Y() = _msg->y;
    this->flowVelocity.Z() = _msg->z;
  }
}

/////////////////////////////////////////////////
void UnderwaterObjectROSPlugin::SetUseGlobalCurrentVel(
  const uuv_gazebo_ros_plugins_msgs::srv::SetUseGlobalCurrentVel::Request::SharedPtr _req,
  uuv_gazebo_ros_plugins_msgs::srv::SetUseGlobalCurrentVel::Response::SharedPtr _res)
{
  if (_req->use_global == this->useGlobalCurrent)
    _res->success = true;
  else
  {
    this->useGlobalCurrent = _req->use_global;
    this->flowVelocity.X() = 0;
    this->flowVelocity.Y() = 0;
    this->flowVelocity.Z() = 0;
    if (this->useGlobalCurrent)
      gzmsg << this->model->GetName() <<
        "::Now using global current velocity" << std::endl;
    else
      gzmsg << this->model->GetName() <<
        "::Using the current velocity under the namespace " <<
        this->model->GetName() << std::endl;
    _res->success = true;
  }
}

/////////////////////////////////////////////////
void UnderwaterObjectROSPlugin::GetModelProperties(
  const uuv_gazebo_ros_plugins_msgs::srv::GetModelProperties::Request::SharedPtr /*_req*/,
  uuv_gazebo_ros_plugins_msgs::srv::GetModelProperties::Response::SharedPtr _res)
{
  for (std::map<gazebo::physics::LinkPtr,
       gazebo::HydrodynamicModelPtr>::iterator it = models.begin();
       it != models.end(); ++it)
  {
    gazebo::physics::LinkPtr link = it->first;
    gazebo::HydrodynamicModelPtr hydro = it->second;

    _res->link_names.push_back(link->GetName());

    uuv_gazebo_ros_plugins_msgs::msg::UnderwaterObjectModel model;
    double param;
    std::vector<double> mat;

    hydro->GetParam("volume", param);
    model.volume = param;

    hydro->GetParam("fluid_density", param);
    model.fluid_density = param;

    hydro->GetParam("bbox_height", param);
    model.bbox_height = param;

    hydro->GetParam("bbox_length", param);
    model.bbox_length = param;

    hydro->GetParam("bbox_width", param);
    model.bbox_width = param;

    hydro->GetParam("added_mass", mat);
    model.added_mass = mat;

    hydro->GetParam("linear_damping", mat);
    model.linear_damping = mat;

    hydro->GetParam("linear_damping_forward_speed", mat);
    model.linear_damping_forward_speed = mat;

    hydro->GetParam("quadratic_damping", mat);
    model.quadratic_damping = mat;

    model.neutrally_buoyant = hydro->IsNeutrallyBuoyant();

    hydro->GetParam("center_of_buoyancy", mat);
    model.cob.x = mat[0];
    model.cob.y = mat[1];
    model.cob.z = mat[2];
#if GAZEBO_MAJOR_VERSION >= 8
    model.inertia.m = link->GetInertial()->Mass();
    model.inertia.ixx = link->GetInertial()->IXX();
    model.inertia.ixy = link->GetInertial()->IXY();
    model.inertia.ixz = link->GetInertial()->IXZ();
    model.inertia.iyy = link->GetInertial()->IYY();
    model.inertia.iyz = link->GetInertial()->IYZ();
    model.inertia.izz = link->GetInertial()->IZZ();

    model.inertia.com.x = link->GetInertial()->CoG().X();
    model.inertia.com.y = link->GetInertial()->CoG().Y();
    model.inertia.com.z = link->GetInertial()->CoG().Z();
#else
    model.inertia.m = link->GetInertial()->GetMass();
    model.inertia.ixx = link->GetInertial()->GetIXX();
    model.inertia.ixy = link->GetInertial()->GetIXY();
    model.inertia.ixz = link->GetInertial()->GetIXZ();
    model.inertia.iyy = link->GetInertial()->GetIYY();
    model.inertia.iyz = link->GetInertial()->GetIYZ();
    model.inertia.izz = link->GetInertial()->GetIZZ();

    model.inertia.com.x = link->GetInertial()->GetCoG().x;
    model.inertia.com.y = link->GetInertial()->GetCoG().y;
    model.inertia.com.z = link->GetInertial()->GetCoG().z;
#endif
    _res->models.push_back(model);
  }
}

/////////////////////////////////////////////////
void UnderwaterObjectROSPlugin::SetScalingAddedMass(
  const uuv_gazebo_ros_plugins_msgs::srv::SetFloat::Request::SharedPtr _req,
  uuv_gazebo_ros_plugins_msgs::srv::SetFloat::Response::SharedPtr _res)

{
  if (_req->data < 0)
  {
    _res->success = false;
    _res->message = "Scaling factor cannot be negative";
  }
  else
  {
    for (std::map<gazebo::physics::LinkPtr,
       gazebo::HydrodynamicModelPtr>::iterator it = models.begin();
       it != models.end(); ++it)
    {
      gazebo::HydrodynamicModelPtr hydro = it->second;
      hydro->SetParam("scaling_added_mass", _req->data);
    }
    _res->success = true;
    _res->message = "All links set with new added-mass scaling factor";
  }
}

/////////////////////////////////////////////////
void UnderwaterObjectROSPlugin::GetScalingAddedMass(
  const uuv_gazebo_ros_plugins_msgs::srv::GetFloat::Request::SharedPtr,
  uuv_gazebo_ros_plugins_msgs::srv::GetFloat::Response::SharedPtr _res)

{
  models.begin()->second->GetParam("scaling_added_mass", _res->data);
}

/////////////////////////////////////////////////
void UnderwaterObjectROSPlugin::SetScalingDamping(
  const uuv_gazebo_ros_plugins_msgs::srv::SetFloat::Request::SharedPtr _req,
  uuv_gazebo_ros_plugins_msgs::srv::SetFloat::Response::SharedPtr _res)

{
  if (_req->data < 0)
  {
    _res->success = false;
    _res->message = "Scaling factor cannot be negative";
  }
  else
  {
    for (std::map<gazebo::physics::LinkPtr,
       gazebo::HydrodynamicModelPtr>::iterator it = models.begin();
       it != models.end(); ++it)
    {
      gazebo::HydrodynamicModelPtr hydro = it->second;
      hydro->SetParam("scaling_damping", _req->data);
    }
    _res->success = true;
    _res->message = "All links set with new damping scaling factor";
  }
}

/////////////////////////////////////////////////
void UnderwaterObjectROSPlugin::GetScalingDamping(
  const uuv_gazebo_ros_plugins_msgs::srv::GetFloat::Request::SharedPtr /*_req*/,
  uuv_gazebo_ros_plugins_msgs::srv::GetFloat::Response::SharedPtr _res)

{
  models.begin()->second->GetParam("scaling_damping", _res->data);
}

/////////////////////////////////////////////////
void UnderwaterObjectROSPlugin::SetScalingVolume(
  const uuv_gazebo_ros_plugins_msgs::srv::SetFloat::Request::SharedPtr _req,
  uuv_gazebo_ros_plugins_msgs::srv::SetFloat::Response::SharedPtr _res)

{
  if (_req->data < 0)
  {
    _res->success = false;
    _res->message = "Scaling factor cannot be negative";
  }
  else
  {
    for (std::map<gazebo::physics::LinkPtr,
       gazebo::HydrodynamicModelPtr>::iterator it = models.begin();
       it != models.end(); ++it)
    {
      gazebo::HydrodynamicModelPtr hydro = it->second;
      hydro->SetParam("scaling_volume", _req->data);
    }
    _res->success = true;
    _res->message = "All links set with new volume scaling factor";
  }
}

/////////////////////////////////////////////////
void UnderwaterObjectROSPlugin::GetScalingVolume(
  const uuv_gazebo_ros_plugins_msgs::srv::GetFloat::Request::SharedPtr /*_req*/,
  uuv_gazebo_ros_plugins_msgs::srv::GetFloat::Response::SharedPtr _res)

{
  models.begin()->second->GetParam("scaling_volume", _res->data);
}

/////////////////////////////////////////////////
void UnderwaterObjectROSPlugin::SetFluidDensity(
  const uuv_gazebo_ros_plugins_msgs::srv::SetFloat::Request::SharedPtr _req,
  uuv_gazebo_ros_plugins_msgs::srv::SetFloat::Response::SharedPtr _res)

{
  if (_req->data < 0)
  {
    _res->success = false;
    _res->message = "Scaling factor cannot be negative";
  }
  else
  {
    for (std::map<gazebo::physics::LinkPtr,
       gazebo::HydrodynamicModelPtr>::iterator it = models.begin();
       it != models.end(); ++it)
    {
      gazebo::HydrodynamicModelPtr hydro = it->second;
      hydro->SetParam("fluid_density", _req->data);
    }
    _res->success = true;
    _res->message = "All links set with new fluid density";
  }
}

/////////////////////////////////////////////////
void UnderwaterObjectROSPlugin::GetFluidDensity(
  const uuv_gazebo_ros_plugins_msgs::srv::GetFloat::Request::SharedPtr /*_req*/,
  uuv_gazebo_ros_plugins_msgs::srv::GetFloat::Response::SharedPtr _res)

{
  models.begin()->second->GetParam("fluid_density", _res->data);
}

/////////////////////////////////////////////////
void UnderwaterObjectROSPlugin::SetOffsetVolume(
  const uuv_gazebo_ros_plugins_msgs::srv::SetFloat::Request::SharedPtr _req,
  uuv_gazebo_ros_plugins_msgs::srv::SetFloat::Response::SharedPtr _res)

{
  for (std::map<gazebo::physics::LinkPtr,
     gazebo::HydrodynamicModelPtr>::iterator it = models.begin();
     it != models.end(); ++it)
  {
    gazebo::HydrodynamicModelPtr hydro = it->second;
    hydro->SetParam("offset_volume", _req->data);
  }
  _res->success = true;
  _res->message = "All links set with new volume offset";

}

/////////////////////////////////////////////////
void UnderwaterObjectROSPlugin::GetOffsetVolume(
  const uuv_gazebo_ros_plugins_msgs::srv::GetFloat::Request::SharedPtr /*_req*/,
  uuv_gazebo_ros_plugins_msgs::srv::GetFloat::Response::SharedPtr _res)

{
  models.begin()->second->GetParam("offset_volume", _res->data);
}

/////////////////////////////////////////////////
void UnderwaterObjectROSPlugin::SetOffsetAddedMass(
  const uuv_gazebo_ros_plugins_msgs::srv::SetFloat::Request::SharedPtr _req,
  uuv_gazebo_ros_plugins_msgs::srv::SetFloat::Response::SharedPtr _res)

{
  for (std::map<gazebo::physics::LinkPtr,
     gazebo::HydrodynamicModelPtr>::iterator it = models.begin();
     it != models.end(); ++it)
  {
    gazebo::HydrodynamicModelPtr hydro = it->second;
    hydro->SetParam("offset_added_mass", _req->data);
  }
  _res->success = true;
  _res->message = "All links set with new added-mass identity offset";

}

/////////////////////////////////////////////////
void UnderwaterObjectROSPlugin::GetOffsetAddedMass(
  const uuv_gazebo_ros_plugins_msgs::srv::GetFloat::Request::SharedPtr /*_req*/,
  uuv_gazebo_ros_plugins_msgs::srv::GetFloat::Response::SharedPtr _res)

{
  models.begin()->second->GetParam("offset_added_mass", _res->data);
}

/////////////////////////////////////////////////
void UnderwaterObjectROSPlugin::SetOffsetLinearDamping(
  const uuv_gazebo_ros_plugins_msgs::srv::SetFloat::Request::SharedPtr _req,
  uuv_gazebo_ros_plugins_msgs::srv::SetFloat::Response::SharedPtr _res)

{
  for (std::map<gazebo::physics::LinkPtr,
     gazebo::HydrodynamicModelPtr>::iterator it = models.begin();
     it != models.end(); ++it)
  {
    gazebo::HydrodynamicModelPtr hydro = it->second;
    hydro->SetParam("offset_linear_damping", _req->data);
  }
  _res->success = true;
  _res->message = "All links set with new linear damping identity offset";

}

/////////////////////////////////////////////////
void UnderwaterObjectROSPlugin::GetOffsetLinearDamping(
  const uuv_gazebo_ros_plugins_msgs::srv::GetFloat::Request::SharedPtr /*_req*/,
  uuv_gazebo_ros_plugins_msgs::srv::GetFloat::Response::SharedPtr _res)

{
  models.begin()->second->GetParam("offset_linear_damping", _res->data);
}

/////////////////////////////////////////////////
void UnderwaterObjectROSPlugin::SetOffsetLinearForwardSpeedDamping(
  const uuv_gazebo_ros_plugins_msgs::srv::SetFloat::Request::SharedPtr _req,
  uuv_gazebo_ros_plugins_msgs::srv::SetFloat::Response::SharedPtr _res)

{
  for (std::map<gazebo::physics::LinkPtr,
     gazebo::HydrodynamicModelPtr>::iterator it = models.begin();
     it != models.end(); ++it)
  {
    gazebo::HydrodynamicModelPtr hydro = it->second;
    hydro->SetParam("offset_lin_forward_speed_damping", _req->data);
  }
  _res->success = true;
  _res->message = "All links set with new linear forward speed damping identity offset";

}

/////////////////////////////////////////////////
void UnderwaterObjectROSPlugin::GetOffsetLinearForwardSpeedDamping(
  const uuv_gazebo_ros_plugins_msgs::srv::GetFloat::Request::SharedPtr /*_req*/,
  uuv_gazebo_ros_plugins_msgs::srv::GetFloat::Response::SharedPtr _res)

{
  models.begin()->second->GetParam("offset_lin_forward_speed_damping", _res->data);
}

/////////////////////////////////////////////////
void UnderwaterObjectROSPlugin::SetOffsetNonLinearDamping(
  const uuv_gazebo_ros_plugins_msgs::srv::SetFloat::Request::SharedPtr _req,
  uuv_gazebo_ros_plugins_msgs::srv::SetFloat::Response::SharedPtr _res)

{
  for (std::map<gazebo::physics::LinkPtr,
     gazebo::HydrodynamicModelPtr>::iterator it = models.begin();
     it != models.end(); ++it)
  {
    gazebo::HydrodynamicModelPtr hydro = it->second;
    hydro->SetParam("offset_nonlin_damping", _req->data);
  }
  _res->success = true;
  _res->message = "All links set with new nonlinear damping identity offset";
}

/////////////////////////////////////////////////
void UnderwaterObjectROSPlugin::GetOffsetNonLinearDamping(
  const uuv_gazebo_ros_plugins_msgs::srv::GetFloat::Request::SharedPtr /*_req*/,
  uuv_gazebo_ros_plugins_msgs::srv::GetFloat::Response::SharedPtr _res)

{
  models.begin()->second->GetParam("offset_nonlin_damping", _res->data);
}

GZ_REGISTER_MODEL_PLUGIN(UnderwaterObjectROSPlugin)
}
