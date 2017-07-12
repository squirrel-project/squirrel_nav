// The MIT License (MIT)
//
// Copyright (c) 2017 Federico Boniardi and Wolfram Burgard
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include "squirrel_navigation/controller_pid.h"
#include "squirrel_navigation/utils/math_utils.h"

#include <ros/console.h>
#include <ros/node_handle.h>

#include <angles>

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
  // Initialization successful.
  init_ = true;
  ROS_INFO_STREAM(
      "squirrel_navigation::ControllerPID: initialization successful.");
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
  errIa_ = angles::normalize_angle(errIa_ * da * dt);
  // Compute the command.
  twist->linear.x =
      params_.kP_lin * dx + params_.kI_lin * errIx_ + params_.kD_lin * dvx;
  twist->linear.y =
      params_.kP_lin * dy + params_.kI_lin * errIy_ + params_.kD_lin * dvy;
  twist->angular.z =
      params_.kP_ang * da + params_.kI_ang * errIa_ + params_.kD_ang * dva;
  // Update last stamp.
  *last_stamp_ = stamp;
}

void ControllerPID::reconfigureCallback(
    ControllerPIDConfig& config, uint32_t level) {
  params_.kP_lin = config.kP_lin;
  params_.kI_lin = config.kI_lin;
  params_.kD_lin = config.kD_lin;
  params_.kP_ang = config.kP_ang;
  params_.kI_ang = config.kI_ang;
  params_.kD_ang = config.kD_ang;
}

ControllerPID::Params ControllerPID::Params::defaultParams() {
  Params params;
  params.kP_lin = params.kP_ang = 3.;
  params.kI_lin = params.kI_ang = 0.01;
  params.kD_lin = params.kD_ang = 0.01;
  return params;
}

}  // namespace squirrel_navigation
