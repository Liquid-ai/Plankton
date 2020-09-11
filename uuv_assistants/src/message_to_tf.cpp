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
//
// This source code is derived from hector_localization
//   (https://github.com/tu-darmstadt-ros-pkg/hector_localization)
// Copyright (c) 2012, Johannes Meyer, TU Darmstadt,
// licensed under the BSD 3-Clause license,
// cf. 3rd-party-licenses.txt file in the root directory of this source tree.
//
// The original code was modified to:
// - be more consistent with other sensor plugins within uuv_simulator,
// - adhere to Gazebo's coding standards.

#include "rclcpp/rclcpp.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


std::string g_odometry_topic;
std::string g_pose_topic;
std::string g_imu_topic;
std::string g_topic;
std::string g_frame_id;
std::string g_footprint_frame_id;
std::string g_position_frame_id;
std::string g_stabilized_frame_id;
std::string g_child_frame_id;

bool g_publish_roll_pitch;

std::string g_tf_prefix;

tf2_ros::TransformBroadcaster *g_transform_broadcaster; //revert to not static version
rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr g_pose_publisher;
rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr g_euler_publisher;

#ifndef TF2_MATRIX3x3_H
  typedef btScalar tfScalar;
  namespace tf { typedef btMatrix3x3 Matrix3x3; }
#endif

void addTransform(std::vector<geometry_msgs::msg::TransformStamped>& transforms, const tf2::Stamped<tf2::Transform>& tf, std::string child_frame_id)
{
  // Might work. Might not. It probably do work though.
  auto new_msg = tf2::toMsg(tf);
  new_msg.child_frame_id = child_frame_id;
  transforms.push_back(new_msg);
}

namespace tf2 
{
  //Specialization for Point msg
  static inline void fromMsg(const geometry_msgs::msg::Point& msgIn, tf2::Vector3& out) 
  {
    out = Vector3(msgIn.x, msgIn.y, msgIn.z);
  }
}

std::string stripSlash(const std::string & in)
{
  if (in.size() && in[0] == '/')
  {
    return in.substr(1);
  }
  return in;
}

void sendTransform(geometry_msgs::msg::Pose const &pose, const std_msgs::msg::Header& header, std::string child_frame_id = "")
{
  std::vector<geometry_msgs::msg::TransformStamped> transforms;

  tf2::Stamped<tf2::Transform> tf;

  auto time_stamp = header.stamp;
  // Manual conversion
  tf.stamp_ = tf2::TimePoint(
    std::chrono::seconds(time_stamp.sec) +
    std::chrono::nanoseconds(time_stamp.nanosec));

  tf.frame_id_ = header.frame_id;
  if (!g_frame_id.empty()) tf.frame_id_ = g_frame_id;

  // No idea what I'm doing
  tf.frame_id_ = stripSlash(tf.frame_id_);

  if (!g_child_frame_id.empty()) child_frame_id = g_child_frame_id;
  if (child_frame_id.empty()) child_frame_id = "base_link";

  tf2::Quaternion orientation;
  tf2::fromMsg(pose.orientation, orientation);
  tf2Scalar yaw, pitch, roll;
  tf2::Matrix3x3(orientation).getEulerYPR(yaw, pitch, roll);
  tf2::Vector3 position;
  tf2::fromMsg(pose.position, position);

  // position intermediate transform (x,y,z)
  if( !g_position_frame_id.empty() && child_frame_id != g_position_frame_id) {
    tf.setOrigin(tf2::Vector3(position.x(), position.y(), position.z() ));
    tf.setRotation(tf2::Quaternion(0.0, 0.0, 0.0, 1.0));
    addTransform(transforms, tf, stripSlash(g_position_frame_id));
  }

  // footprint intermediate transform (x,y,yaw)
  if (!g_footprint_frame_id.empty() && child_frame_id != g_footprint_frame_id) {
    tf.setOrigin(tf2::Vector3(position.x(), position.y(), 0.0));
    tf2::Quaternion quat;
    quat.setRPY(0.0, 0.0, yaw);
    tf.setRotation(quat);
    addTransform(transforms, tf, stripSlash(g_footprint_frame_id));

    yaw = 0.0;
    position.setX(0.0);
    position.setY(0.0);
    tf.frame_id_ = stripSlash(g_footprint_frame_id);
  }

  // stabilized intermediate transform (z)
  if (!g_footprint_frame_id.empty() && child_frame_id != g_stabilized_frame_id) {
    tf.setOrigin(tf2::Vector3(0.0, 0.0, position.z()));
    tf.setBasis(tf2::Matrix3x3::getIdentity());
    addTransform(transforms, tf, stripSlash(g_stabilized_frame_id));

    position.setZ(0.0);
    tf.frame_id_ = stripSlash(g_stabilized_frame_id);
  }

  // base_link transform (roll, pitch)
  if (g_publish_roll_pitch) {
    tf.setOrigin(position);
    tf2::Quaternion quat;
    quat.setRPY(roll, pitch, yaw);
    tf.setRotation(quat);
    addTransform(transforms, tf, stripSlash(child_frame_id));
  }

  g_transform_broadcaster->sendTransform(transforms);

  // publish pose message
  if (g_pose_publisher) {
    geometry_msgs::msg::PoseStamped pose_stamped;
    pose_stamped.pose = pose;
    pose_stamped.header = header;
    g_pose_publisher->publish(pose_stamped);
  }

  // publish pose message
  if (g_euler_publisher) {
    geometry_msgs::msg::Vector3Stamped euler_stamped;
    euler_stamped.vector.x = roll;
    euler_stamped.vector.y = pitch;
    euler_stamped.vector.z = yaw;
    euler_stamped.header = header;
    g_euler_publisher->publish(euler_stamped);
  }
}

void odomCallback(nav_msgs::msg::Odometry::SharedPtr odometry) {
  sendTransform(odometry->pose.pose, odometry->header, odometry->child_frame_id);
}

void poseCallback(geometry_msgs::msg::PoseStamped::SharedPtr pose) {
  sendTransform(pose->pose, pose->header);
}

void tfCallback(geometry_msgs::msg::TransformStamped::SharedPtr tf) {
  geometry_msgs::msg::Pose pose;
  pose.position.x = tf->transform.translation.x;
  pose.position.y = tf->transform.translation.y;
  pose.position.z = tf->transform.translation.z;
  pose.orientation = tf->transform.rotation;

  sendTransform(pose, tf->header);
}

void imuCallback(sensor_msgs::msg::Imu::SharedPtr imu) {
  std::vector<geometry_msgs::msg::TransformStamped> transforms;
  std::string child_frame_id;

  tf2::Stamped<tf2::Transform> tf;

// TODO
  auto time_stamp = imu->header.stamp;
  tf.stamp_ = tf2::TimePoint(
    std::chrono::seconds(time_stamp.sec) +
    std::chrono::nanoseconds(time_stamp.nanosec));

  tf.frame_id_ = stripSlash(g_stabilized_frame_id);
  if (!g_child_frame_id.empty()) child_frame_id = g_child_frame_id;
  if (child_frame_id.empty()) child_frame_id = "base_link";

  tf2::Quaternion orientation;
  tf2::fromMsg(imu->orientation, orientation);
  tf2Scalar yaw, pitch, roll;
  tf2::Matrix3x3(orientation).getEulerYPR(yaw, pitch, roll);
  tf2::Quaternion rollpitch;
  rollpitch.setRPY(roll, pitch, 0.0);

  // base_link transform (roll, pitch)
  if (g_publish_roll_pitch) {
    tf.setOrigin(tf2::Vector3(0.0, 0.0, 0.0));
    tf.setRotation(rollpitch);
    addTransform(transforms, tf, stripSlash(child_frame_id));
  }

  if (!transforms.empty()) g_transform_broadcaster->sendTransform(transforms);

  // publish pose message
  if (g_pose_publisher) {
    geometry_msgs::msg::PoseStamped pose_stamped;
    pose_stamped.header.stamp = imu->header.stamp;
    pose_stamped.header.frame_id = g_stabilized_frame_id;
    pose_stamped.pose.orientation = tf2::toMsg(rollpitch);
    g_pose_publisher->publish(pose_stamped);
  }
}

// Disabled multicallback as this is not seemingly possible easily with ros2

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  //Add node options to auto declare parameters from launch files and cmd line
  auto node = rclcpp::Node::make_shared("message_to_tf",
                              rclcpp::NodeOptions().allow_undeclared_parameters(true).
                              automatically_declare_parameters_from_overrides(true)
  );

  g_footprint_frame_id = "base_footprint";
  g_stabilized_frame_id = "base_stabilized";
  // g_position_frame_id = "base_position";
  // g_child_frame_id = "base_link";

  node->get_parameter("odometry_topic", g_odometry_topic);
  node->get_parameter("pose_topic", g_pose_topic);
  node->get_parameter("imu_topic", g_imu_topic);
  //node->get_parameter("topic", g_topic); // Not possible anymore
  node->get_parameter("frame_id", g_frame_id);
  node->get_parameter("footprint_frame_id", g_footprint_frame_id);
  node->get_parameter("position_frame_id", g_position_frame_id);
  node->get_parameter("stabilized_frame_id", g_stabilized_frame_id);
  node->get_parameter("child_frame_id", g_child_frame_id);

  // get topic from the commandline
  // Currently useless + cmd line args are not removed (use 
  // init_and_remove_arguments instead of init)
  // if (argc > 1) {
  //     g_topic = argv[1];
  //     g_odometry_topic.clear();
  //     g_pose_topic.clear();
  //     g_imu_topic.clear();
  // }

  g_publish_roll_pitch = true;
  node->get_parameter("publish_roll_pitch", g_publish_roll_pitch);

  g_transform_broadcaster = new tf2_ros::TransformBroadcaster(node); // TODO Check

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub1;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub2;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub3;

  int subscribers = 0;
  if (!g_odometry_topic.empty()) {
      sub1 = node->create_subscription<nav_msgs::msg::Odometry>(g_odometry_topic, 10, &odomCallback);
      subscribers++;
  }
  if (!g_pose_topic.empty()) {
      sub2 = node->create_subscription<geometry_msgs::msg::PoseStamped>(g_pose_topic, 10, &poseCallback);
      subscribers++;
  }
  if (!g_imu_topic.empty()) {
      sub3 = node->create_subscription<sensor_msgs::msg::Imu>(g_imu_topic, 10, &imuCallback);
      subscribers++;
  }
  if (!g_topic.empty()) {
     //
      RCLCPP_FATAL(node->get_logger(), "Multicallbacks from ROS 1 not yet implemented");
      //sub4 = node.subscribe(g_topic, 10, &multiCallback);
      subscribers++;
  }

  if (subscribers == 0) {
    RCLCPP_FATAL(node->get_logger(), "Usage: rosrun message_to_tf message_to_tf <topic>");
    return 1;
  } else if (subscribers > 1) {
    RCLCPP_FATAL(node->get_logger(), "More than one of the parameters odometry_topic, pose_topic, imu_topic and topic are set.\n"
              "Please specify exactly one of them or simply add the topic name to the command line.");
    return 1;
  }

  bool publish_pose = true;
  node->get_parameter("publish_pose", publish_pose);
  if (publish_pose) {
    std::string publish_pose_topic;
    node->get_parameter("publish_pose_topic", publish_pose_topic);

    if (!publish_pose_topic.empty())
      g_pose_publisher = node->create_publisher<geometry_msgs::msg::PoseStamped>(publish_pose_topic, 10);
    else
      g_pose_publisher = node->create_publisher<geometry_msgs::msg::PoseStamped>("~/pose", 10);
  }

  bool publish_euler = true;
  node->get_parameter("publish_euler", publish_euler);
  if (publish_euler) {
    std::string publish_euler_topic;
    node->get_parameter("publish_euler_topic", publish_euler_topic);

    if (!publish_euler_topic.empty())
      g_euler_publisher = node->create_publisher<geometry_msgs::msg::Vector3Stamped>(publish_euler_topic, 10);
    else
      g_euler_publisher = node->create_publisher<geometry_msgs::msg::Vector3Stamped>("~/euler", 10);
      //g_euler_publisher = priv_nh.advertise<geometry_msgs::msg::Vector3Stamped>("~/euler", 10); //TODO to check
  }

  rclcpp::spin(node);
  delete g_transform_broadcaster;

  rclcpp::shutdown();
  return 0;
}
