// LocalPlanner.cpp --- 
// 
// Filename: LocalPlanner.cpp
// Description: 
// Author: Federico Boniardi
// Maintainer: 
// Created: Sat Feb  6 19:29:11 2016 (+0100)
// Version: 
// Last-Updated: 
//           By: 
//     Update #: 0
// URL: 
// Keywords: 
// Compatibility: 
// 
// 

// Commentary: 
// 
// 
// 
// 

// Change Log:
// 
// 
// 
// 
// Copyright (c) 2016, Federico Boniardi
// All rights reserved.
// 
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
// 
// * Redistributions of source code must retain the above copyright notice, this
//   list of conditions and the following disclaimer.
// 
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
// 
// * Neither the name of the University of Freiburg nor the names of its
//   contributors may be used to endorse or promote products derived from
//   this software without specific prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// 
// 

// Code:

#include "squirrel_navigation/LocalPlanner.h"

PLUGINLIB_DECLARE_CLASS(squirrel_navigation, LocalPlanner, squirrel_navigation::LocalPlanner, nav_core::BaseLocalPlanner)

namespace squirrel_navigation {

LocalPlanner::LocalPlanner( void ) :
    controller_(nullptr),
    tf_(nullptr),
    trajectory_(nullptr),
    goal_(nullptr),
    max_linear_vel_(0.5),
    max_angular_vel_(0.7),
    xy_goal_tolerance_(0.05),
    yaw_goal_tolerance_(0.05),
    verbose_(false)
{  
  ROS_INFO("squirrel_navigation::LocalPlanner started");
}

LocalPlanner::~LocalPlanner( void )
{
  TrajectoryPlanner::deleteTrajectory();

  if ( goal_ )
    delete goal_;

  if ( controller_ )
    delete controller_;
  
  odom_sub_.shutdown();
  traj_pub_.shutdown();
}

void LocalPlanner::initialize( std::string name, tf::TransformListener* tf, costmap_2d::Costmap2DROS* costmap_ros )
{
  name_ = name;
  
  tf_ = tf;
  costmap_ros_ = costmap_ros;
  trajectory_ = TrajectoryPlanner::getTrajectory();

  if ( not controller_ )
    controller_ = new ControllerPD;
  
  ros::NodeHandle pnh("~/"+name);
  pnh.param<bool>("verbose", verbose_, false);  
  pnh.param<double>("max_linear_vel", max_linear_vel_, 0.5);
  pnh.param<double>("max_angular_vel", max_angular_vel_, 0.7);
  pnh.param<double>("xy_goal_tolerance", xy_goal_tolerance_, 0.05);
  pnh.param<double>("yaw_goal_tolerance", yaw_goal_tolerance_, 0.05);
  
  if ( max_linear_vel_ <= 0 ) {
    ROS_WARN_STREAM(name << ": max_rlinear_vel has been chosen to be non positive. Reverting to 0.5 (m/s)");
    max_linear_vel_ = 0.5;
  }
  
  if ( max_angular_vel_ <= 0 ) {
    ROS_WARN_STREAM(name << ": max_rotation_vel has been chosen to be non positive. Reverting to 0.7 (rad/s)");
    max_angular_vel_ = 0.7;
  }

  trajectory_->setVelocityBounds(max_linear_vel_,max_angular_vel_);
  
  ros::NodeHandle pnh_c("~/"+name+"/controller_gains");
  pnh_c.param<double>("P_linear", gains_.Pxy, 1.0);
  pnh_c.param<double>("P_angular", gains_.Pyaw, 1.0);
  pnh_c.param<double>("D_linear", gains_.Dxy, 0.1);
  pnh_c.param<double>("D_angular", gains_.Dyaw, 0.1);

  controller_->setGains(gains_);
  
  ros::NodeHandle nh;
  odom_sub_ = nh.subscribe<nav_msgs::Odometry>("odom", 1, &LocalPlanner::odometryCallback_, this);
  traj_pub_ = pnh.advertise<visualization_msgs::Marker>("marker", 10);
}

bool LocalPlanner::computeVelocityCommands( geometry_msgs::Twist& cmd_vel )
{
  std::unique_lock<std::mutex> lock(guard_);
  
  tf::Stamped<tf::Pose> tf_robot_pose;
  costmap_ros_->getRobotPose(tf_robot_pose);
  
  double odom_vx = odom_.twist.twist.linear.x,
      odom_vy = odom_.twist.twist.linear.y,
      odom_vyaw = odom_.twist.twist.angular.z;
  
  TrajectoryPlanner::Profile robot_pose, ref_pose;

  robot_pose.x = tf_robot_pose.getOrigin().getX();
  robot_pose.y = tf_robot_pose.getOrigin().getY();
  robot_pose.yaw = tf::getYaw(tf_robot_pose.getRotation());
  robot_pose.vx = std::cos(robot_pose.yaw)*odom_vx - std::sin(robot_pose.yaw)*odom_vy;
  robot_pose.vy = std::sin(robot_pose.yaw)*odom_vx + std::cos(robot_pose.yaw)*odom_vy;
  robot_pose.vyaw = odom_vyaw;

  ref_pose = trajectory_->getProfile(odom_.header.stamp);
  controller_->computeCommands(ref_pose,robot_pose,cmd_);
  normalizeCommands_();

  cmd_vel.linear.x = std::cos(-robot_pose.yaw)*cmd_[0] - std::sin(-robot_pose.yaw)*cmd_[1];
  cmd_vel.linear.y = std::sin(-robot_pose.yaw)*cmd_[0] + std::cos(-robot_pose.yaw)*cmd_[1];
  cmd_vel.angular.z = cmd_[2];
  
  publishTrajectoryPose_(ref_pose,odom_.header.stamp);  
  
  return true;
}

bool LocalPlanner::isGoalReached( void )
{
  tf::Stamped<tf::Pose> tf_robot_pose;
  costmap_ros_->getRobotPose(tf_robot_pose);

  geometry_msgs::Pose robot_pose;
  robot_pose.position.x = tf_robot_pose.getOrigin().getX();
  robot_pose.position.y = tf_robot_pose.getOrigin().getY();
  robot_pose.orientation = tf::createQuaternionMsgFromYaw(tf::getYaw(tf_robot_pose.getRotation()));  
  
  if ( not goal_ ) {
    return false;
  } else if ( linearDistance(goal_->position,robot_pose.position) < xy_goal_tolerance_ and
              angularDistance(goal_->orientation,robot_pose.orientation) < yaw_goal_tolerance_ ) {
    delete goal_;
    goal_ = nullptr;
    trajectory_->deactivate();
    return true;
  } else {
    return false;
  }
}

bool LocalPlanner::setPlan( const std::vector<geometry_msgs::PoseStamped>& plan )
{
  if ( not goal_ )
    goal_ = new geometry_msgs::Pose;

  *goal_ = plan.back().pose;
  
  return true;
}

void LocalPlanner::odometryCallback_( const nav_msgs::Odometry::ConstPtr& odom )
{
  odom_ = *odom;
}

}  // namespace squirrel_navigation

// 
// LocalPlanner.cpp ends here
