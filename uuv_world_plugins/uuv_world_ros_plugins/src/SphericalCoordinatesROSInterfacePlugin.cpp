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

/// \file SphericalCoordinatesROSInterfacePlugin.cc

#include <uuv_world_ros_plugins/SphericalCoordinatesROSInterfacePlugin.h>

//TODO Remove Gazebo gazebo < 9 references

namespace gazebo
{

/////////////////////////////////////////////////
SphericalCoordinatesROSInterfacePlugin::SphericalCoordinatesROSInterfacePlugin()
{ }

/////////////////////////////////////////////////
SphericalCoordinatesROSInterfacePlugin::~SphericalCoordinatesROSInterfacePlugin()
{
#if GAZEBO_MAJOR_VERSION >= 8
  this->rosPublishConnection.reset();
#else
  event::Events::DisconnectWorldUpdateBegin(this->rosPublishConnection);
#endif

  //this->rosNode->shutdown();
}

/////////////////////////////////////////////////
void SphericalCoordinatesROSInterfacePlugin::Load(
  physics::WorldPtr _world, sdf::ElementPtr _sdf)
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

  GZ_ASSERT(_world != NULL, "World pointer is invalid");
  GZ_ASSERT(_sdf != NULL, "SDF pointer is invalid");

  this->world = _world;

  //TODO Think about adding the "gazebo" namespace in the SDF : <ros><namespace>
  myRosNode =  gazebo_ros::Node::Get(_sdf);
  //this->rosNode.reset(new ros::NodeHandle(""));

  gzmsg << "[SphericalCoordinatesROSInterfacePlugin] Node created with name: " << 
    myRosNode->get_name() << " with namespace: " << myRosNode->get_namespace() << std::endl;

  // Advertise the service to get origin of the world in spherical coordinates
  this->worldServices["get_origin_spherical_coordinates"] =
    myRosNode->create_service<uuv_world_ros_plugins_msgs::srv::GetOriginSphericalCoord>(
      "/gazebo/get_origin_spherical_coordinates",
      std::bind(&SphericalCoordinatesROSInterfacePlugin::GetOriginSphericalCoord, this, _1, _2));

  // Advertise the service to get origin of the world in spherical coordinates
  this->worldServices["set_origin_spherical_coordinates"] =
    myRosNode->create_service<uuv_world_ros_plugins_msgs::srv::SetOriginSphericalCoord>(
      "/gazebo/set_origin_spherical_coordinates",
      std::bind(&SphericalCoordinatesROSInterfacePlugin::SetOriginSphericalCoord, this, _1, _2));

  this->worldServices["transform_to_spherical_coord"] =
    myRosNode->create_service<uuv_world_ros_plugins_msgs::srv::TransformToSphericalCoord>(
      "/gazebo/transform_to_spherical_coordinates",
      std::bind(&SphericalCoordinatesROSInterfacePlugin::TransformToSphericalCoord, this, _1, _2));

  this->worldServices["transform_from_spherical_coord"] =
    myRosNode->create_service<uuv_world_ros_plugins_msgs::srv::TransformFromSphericalCoord>(
      "/gazebo/transform_from_spherical_coordinates",
      std::bind(&SphericalCoordinatesROSInterfacePlugin::TransformFromSphericalCoord, this, _1, _2));

#if GAZEBO_MAJOR_VERSION >= 8
  gzmsg << "Spherical coordinates reference=" << std::endl
    << "\t- Latitude [degrees]="
    << this->world->SphericalCoords()->LatitudeReference().Degree()
    << std::endl
    << "\t- Longitude [degrees]="
    << this->world->SphericalCoords()->LongitudeReference().Degree()
    << std::endl
    << "\t- Altitude [m]="
    << this->world->SphericalCoords()->GetElevationReference()
    << std::endl;
#else
  gzmsg << "Spherical coordinates reference=" << std::endl
    << "\t- Latitude [degrees]="
    << this->world->GetSphericalCoordinates()->LatitudeReference().Degree()
    << std::endl
    << "\t- Longitude [degrees]="
    << this->world->GetSphericalCoordinates()->LongitudeReference().Degree()
    << std::endl
    << "\t- Altitude [m]="
    << this->world->GetSphericalCoordinates()->GetElevationReference()
    << std::endl;
#endif
}

/////////////////////////////////////////////////
bool SphericalCoordinatesROSInterfacePlugin::TransformToSphericalCoord(
    const uuv_world_ros_plugins_msgs::srv::TransformToSphericalCoord::Request::SharedPtr _req,
    uuv_world_ros_plugins_msgs::srv::TransformToSphericalCoord::Response::SharedPtr _res)
{
  ignition::math::Vector3d cartVec = ignition::math::Vector3d(
    _req->input.x, _req->input.y, _req->input.z);

#if GAZEBO_MAJOR_VERSION >= 8
  ignition::math::Vector3d scVec =
    this->world->SphericalCoords()->SphericalFromLocal(cartVec);
#else
  ignition::math::Vector3d scVec =
    this->world->GetSphericalCoordinates()->SphericalFromLocal(cartVec);
#endif
  _res->latitude_deg = scVec.X();
  _res->longitude_deg = scVec.Y();
  _res->altitude = scVec.Z();
  return true;
}

/////////////////////////////////////////////////
bool SphericalCoordinatesROSInterfacePlugin::TransformFromSphericalCoord(
    const uuv_world_ros_plugins_msgs::srv::TransformFromSphericalCoord::Request::SharedPtr _req,
    uuv_world_ros_plugins_msgs::srv::TransformFromSphericalCoord::Response::SharedPtr _res)
{
  ignition::math::Vector3d scVec = ignition::math::Vector3d(
    _req->latitude_deg, _req->longitude_deg, _req->altitude);
#if GAZEBO_MAJOR_VERSION >= 8
  ignition::math::Vector3d cartVec =
    this->world->SphericalCoords()->LocalFromSpherical(scVec);
#else
  ignition::math::Vector3d cartVec =
    this->world->GetSphericalCoordinates()->LocalFromSpherical(scVec);
#endif
  _res->output.x = cartVec.X();
  _res->output.y = cartVec.Y();
  _res->output.z = cartVec.Z();
  return true;
}

/////////////////////////////////////////////////
bool SphericalCoordinatesROSInterfacePlugin::GetOriginSphericalCoord(
    const uuv_world_ros_plugins_msgs::srv::GetOriginSphericalCoord::Request::SharedPtr /*_req*/,
    uuv_world_ros_plugins_msgs::srv::GetOriginSphericalCoord::Response::SharedPtr _res)
{
#if GAZEBO_MAJOR_VERSION >= 8
  _res->latitude_deg =
    this->world->SphericalCoords()->LatitudeReference().Degree();
  _res->longitude_deg =
    this->world->SphericalCoords()->LongitudeReference().Degree();
  _res->altitude =
    this->world->SphericalCoords()->GetElevationReference();
#else
  _res->latitude_deg =
    this->world->GetSphericalCoordinates()->LatitudeReference().Degree();
  _res->longitude_deg =
    this->world->GetSphericalCoordinates()->LongitudeReference().Degree();
  _res->altitude =
    this->world->GetSphericalCoordinates()->GetElevationReference();
#endif
  return true;
}

/////////////////////////////////////////////////
bool SphericalCoordinatesROSInterfacePlugin::SetOriginSphericalCoord(
    const uuv_world_ros_plugins_msgs::srv::SetOriginSphericalCoord::Request::SharedPtr _req,
    uuv_world_ros_plugins_msgs::srv::SetOriginSphericalCoord::Response::SharedPtr _res)
{
  ignition::math::Angle angle;
  angle.Degree(_req->latitude_deg);
#if GAZEBO_MAJOR_VERSION >= 8
  this->world->SphericalCoords()->SetLatitudeReference(angle);
#else
  this->world->GetSphericalCoordinates()->SetLatitudeReference(angle);
#endif

  angle.Degree(_req->longitude_deg);
#if GAZEBO_MAJOR_VERSION >= 8
  this->world->SphericalCoords()->SetLongitudeReference(angle);
  this->world->SphericalCoords()->SetElevationReference(_req->altitude);
#else
  this->world->GetSphericalCoordinates()->SetLongitudeReference(angle);
  this->world->GetSphericalCoordinates()->SetElevationReference(_req->altitude);
#endif
  _res->success = true;
  return true;
}

/////////////////////////////////////////////////
GZ_REGISTER_WORLD_PLUGIN(SphericalCoordinatesROSInterfacePlugin)
}
