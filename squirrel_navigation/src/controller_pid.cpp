// Copyright (c) 2017, Federico Boniardi and Wolfram Burgard
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
// * Redistributions of source code must retain the above copyright
//   notice, this list of conditions and the following disclaimer.
//
// * Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in
//   the documentation and/or other materials provided with the
//   distribution.
//
// * Neither the name of the University of Freiburg nor the names of
//   its contributors may be used to endorse or promote products
//   derived from this software without specific prior written
//   permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
// INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
// HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
// STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
// OF THE POSSIBILITY OF SUCH DAMAGE.

#include "squirrel_navigation/controller_pid.h"
#include "squirrel_navigation/utils/math_utils.h"

#include <ros/console.h>
#include <ros/node_handle.h>

#include <std_msgs/Header.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <tf/tf.h>

#include <angles/angles.h>

#include <cmath>
#include <string>

namespace squirrel_navigation {

void ControllerPID::initialize(const std::string& name) {
  if (init_)
    return;
  // Initialize the parameter server.
  ros::NodeHandle pnh("~/" + name);
  dsrv_.reset(new dynamic_reconfigure::Server<ControllerPIDConfig>(pnh));
  dsrv_->setCallback(
      boost::bind(&ControllerPID::reconfigureCallback, this, _1, _2));
  // Publish the linear twist.
  cmd_pub_ = pnh.advertise<visualization_msgs::MarkerArray>("cmd_raw", 1);
  // Initialization successful.
  init_ = true;
  ROS_INFO_STREAM(
      "squirrel_navigation/ControllerPID: initialization successful.");
}

void ControllerPID::reset(const ros::Time& start) {
  last_stamp_.reset(new ros::Time(start));
  errIx_ = errIy_ = errIa_ = 0.;
}

void ControllerPID::computeCommand(
    const ros::Time& stamp, const geometry_msgs::Pose& pose,
    const geometry_msgs::Pose& ref_pose, const geometry_msgs::Twist& vel,
    const geometry_msgs::Twist& ref_vel, geometry_msgs::Twist* twist) {
  // Compute deltas...
  const double dt  = stamp.toSec() - last_stamp_->toSec();
  const double dx  = math::delta<0>(pose, ref_pose);
  const double dy  = math::delta<1>(pose, ref_pose);
  const double da  = math::delta<2>(pose, ref_pose);
  const double dvx = math::delta<0>(vel, ref_vel);
  const double dvy = math::delta<1>(vel, ref_vel);
  const double dva = math::delta<2>(vel, ref_vel);
  // Update the integral term.
  errIx_ += dx * dt;
  errIy_ += dy * dt;
  errIa_ = angles::normalize_angle(errIa_ + da * dt);
  // Compute the command.
  twist->linear.x =
      params_.kP_lin * dx + params_.kI_lin * errIx_ + params_.kD_lin * dvx;
  twist->linear.y =
      params_.kP_lin * dy + params_.kI_lin * errIy_ + params_.kD_lin * dvy;
  twist->angular.z =
      params_.kP_ang * da + params_.kI_ang * errIa_ + params_.kD_ang * dva;
  // Update last stamp.
  *last_stamp_ = stamp;
  // Visualize the twist.
  publishTwist(pose, *twist, stamp);
}

void ControllerPID::reconfigureCallback(
    ControllerPIDConfig& config, uint32_t level) {
  params_.global_frame_id  = config.global_frame_id;
  params_.kP_lin           = config.kP_lin;
  params_.kI_lin           = config.kI_lin;
  params_.kD_lin           = config.kD_lin;
  params_.kP_ang           = config.kP_ang;
  params_.kI_ang           = config.kI_ang;
  params_.kD_ang           = config.kD_ang;
  params_.visualize_topics = config.visualize_topics;
}

void ControllerPID::publishTwist(
    const geometry_msgs::Pose& actuation_pose, const geometry_msgs::Twist& cmd,
    const ros::Time& stamp) const {
  if (!params_.visualize_topics)
    return;
  // Common header.
  std_msgs::Header header;
  header.frame_id = params_.global_frame_id;
  header.stamp    = stamp;
  // Actuation pose quantities.
  const geometry_msgs::Point& actuation_point = actuation_pose.position;
  const double yaw = tf::getYaw(actuation_pose.orientation);
  // Twist quantities.
  const double dir    = std::atan2(cmd.linear.y, cmd.linear.x);
  const double length = std::hypot(cmd.linear.x, cmd.linear.y);
  const double angle  = yaw + M_PI / 2;
  // Marker for linear command.
  visualization_msgs::Marker marker_lin_cmd;
  marker_lin_cmd.id               = 0;
  marker_lin_cmd.header           = header;
  marker_lin_cmd.ns               = "cmd_raw";
  marker_lin_cmd.type             = visualization_msgs::Marker::ARROW;
  marker_lin_cmd.action           = visualization_msgs::Marker::MODIFY;
  marker_lin_cmd.pose.position    = actuation_point;
  marker_lin_cmd.pose.orientation = tf::createQuaternionMsgFromYaw(dir);
  marker_lin_cmd.scale.x          = length;
  marker_lin_cmd.scale.y          = 0.035;
  marker_lin_cmd.scale.z          = 0.05;
  marker_lin_cmd.color.r          = 1.0;
  marker_lin_cmd.color.g          = 0.0;
  marker_lin_cmd.color.b          = 0.0;
  marker_lin_cmd.color.a          = 0.5;
  // Marker for angular command.
  visualization_msgs::Marker marker_ang_cmd;
  marker_ang_cmd.id               = 1;
  marker_ang_cmd.header           = header;
  marker_ang_cmd.ns               = "cmd_raw";
  marker_ang_cmd.type             = visualization_msgs::Marker::ARROW;
  marker_ang_cmd.action           = visualization_msgs::Marker::MODIFY;
  marker_ang_cmd.pose.position.x  = actuation_point.x + 0.22 * std::cos(yaw);
  marker_ang_cmd.pose.position.y  = actuation_point.y + 0.22 * std::sin(yaw);
  marker_ang_cmd.pose.orientation = tf::createQuaternionMsgFromYaw(angle);
  marker_ang_cmd.scale.x          = cmd.angular.z;
  marker_ang_cmd.scale.y          = 0.035;
  marker_ang_cmd.scale.z          = 0.05;
  marker_ang_cmd.color.r          = 1.0;
  marker_ang_cmd.color.g          = 0.0;
  marker_ang_cmd.color.b          = 0.0;
  marker_ang_cmd.color.a          = 0.5;
  // Publish the message.
  visualization_msgs::MarkerArray marker_cmd;
  marker_cmd.markers = {marker_lin_cmd, marker_ang_cmd};
  cmd_pub_.publish(marker_cmd);
}

ControllerPID::Params ControllerPID::Params::defaultParams() {
  Params params;
  params.global_frame_id = "/map";
  params.kP_lin = params.kP_ang = 3.0;
  params.kI_lin = params.kI_ang = 0.0001;
  params.kD_lin = params.kD_ang = 0.0001;
  params.visualize_topics       = true;
  return params;
}

}  // namespace squirrel_navigation
