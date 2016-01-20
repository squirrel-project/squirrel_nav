// Autolocalization.cpp --- 
// 
// Filename: Autolocalization.cpp
// Description: Localisation on startup 
// Author: Federico Boniardi
// Maintainer: boniardi@cs.uni-freiburg.de
// Created: Mon Nov 24 10:20:05 2014 (+0100)
// Version: 0.1.0
// Last-Updated: Wed Nov 26 16:10:57 2014 (+0100)
//           By: Federico Boniardi
//     Update #: 1
// URL: 
// Keywords: 
// Compatibility: 
//   ROS Hydro, ROS Indigo
// 

// Commentary: 
//   Tested on: - ROS Hydro on Ubuntu 12.04
//              - ROS Indigo on Ubuntu 14.04
// 

// Copyright (c) 2014, Federico Boniardi
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
// * Neither the name of the {organization} nor the names of its
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

#include "squirrel_navigation/Autolocalization.h"

namespace squirrel_navigation {

Autolocalization::Autolocalization( void ) :
    private_nh_("~"),
    state_(STOP),
    is_localized_(false),
    glob_start_(false),
    pose_start_(false),
    amcl_start_(false),
    free_dir_(0.0),
    i_distance_sensors_(0),
    th_err_(0.0)
{
  private_nh_.param("global_localization", global_localization_, false);
  private_nh_.param("max_angular_vel", max_vel_ang_, 0.1);
  private_nh_.param("max_linear_vel", max_vel_lin_, 0.1);
  private_nh_.param("tolerance_var_x", tolerance_var_x_, 0.05);
  private_nh_.param("tolerance_var_y", tolerance_var_y_, 0.05);
  private_nh_.param("tolerance_var_th", tolerance_var_th_, 0.05);
  private_nh_.param("calibration", calibration_, false);
  private_nh_.param("initial_pose_topic", initial_pose_topic_, std::string("/initialpose"));
  private_nh_.param("amcl_pose_topic", amcl_pose_topic_, std::string("/squirrel_localizer_pose"));
  private_nh_.param("odom_topic", odom_topic_, std::string("/odom"));
  private_nh_.param("distance_sensors_topic", distance_sensors_topic_, std::string("/distance_sensors"));

  odom_sub_ = public_nh_.subscribe(odom_topic_, 1000, &Autolocalization::getOdomPose, this);
  distance_sensors_sub_ = public_nh_.subscribe(distance_sensors_topic_, 1000, &Autolocalization::getDistanceSensorsValues, this);
  initial_pose_sub_ = public_nh_.subscribe<geometry_msgs::PoseWithCovarianceStamped>(initial_pose_topic_, 1000,
                                                                                     boost::bind(&Autolocalization::getPoseWithCovariance, this, _1, initial_pose_topic_));
  amcl_pose_sub_ = public_nh_.subscribe<geometry_msgs::PoseWithCovarianceStamped>(amcl_pose_topic_, 1000,
                                                                                  boost::bind(&Autolocalization::getPoseWithCovariance, this, _1, amcl_pose_topic_));
  twist_pub_ = public_nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);

  node_name_ = ros::this_node::getName();
  
  setInitialCovariance();
  
  // Distribute particles in the whole free space
  if ( global_localization_ ) {
    std_srvs::Empty::Request empty_req;
    std_srvs::Empty::Response empty_res;
    ros::service::call("/global_localization", empty_req, empty_res);
  }

  // Be sure not to forget to stop robot
  std::signal(SIGINT, interruptCallback);
}

Autolocalization::~Autolocalization( void )
{
  amcl_pose_sub_.shutdown();
  initial_pose_sub_.shutdown();
}

void Autolocalization::spin( void )
{
  ros::Rate lr(20);  
  while ( !_SIGINT_caught && !is_localized_ ) {
    try {
      ros::spinOnce();
      checkCovariance(amcl_pose_);
      move();
      lr.sleep();
    } catch ( std::runtime_error& err ) {
      ROS_ERROR("%s: %s", node_name_.c_str(), err.what());
      clearCostmapAndShutdown();
      std::exit(1);
    }
  }
  clearCostmapAndShutdown();
}

void Autolocalization::waitForStarting( void )
{
  ros::Rate lr(1);
  if ( global_localization_ ) {
    while ( ros::ok() && !glob_start_ ) {
      ros::spinOnce();
      ROS_WARN("%s: waiting for odometry...", node_name_.c_str());
      lr.sleep();
    }
    return;
  } else {
    while ( ros::ok() && !pose_start_ ) {
      ros::spinOnce();
      if ( amcl_start_ ) {
        checkCovariance(amcl_pose_);
      }
      ROS_WARN("%s: waiting for initial pose...", node_name_.c_str());
      lr.sleep();
    }
    return;
  }
}

void Autolocalization::getPoseWithCovariance( const geometry_msgs::PoseWithCovarianceStampedConstPtr& pose_msg, std::string topic )
{
  if ( topic.compare(initial_pose_topic_) == 0 ) {
    pose_start_ = true;
    ROS_INFO("%s: got initial pose", node_name_.c_str());
  }

  if ( topic.compare(amcl_pose_topic_) == 0 ) {
    amcl_start_ = true;
  }
  
  amcl_pose_.header.seq = pose_msg->header.seq;
  amcl_pose_.header.frame_id = pose_msg->header.frame_id;
  amcl_pose_.header.stamp = pose_msg->header.stamp;

  amcl_pose_.pose.pose.position.x = pose_msg->pose.pose.position.x;
  amcl_pose_.pose.pose.position.y = pose_msg->pose.pose.position.y;
  amcl_pose_.pose.pose.orientation.x = pose_msg->pose.pose.orientation.x;
  amcl_pose_.pose.pose.orientation.y = pose_msg->pose.pose.orientation.y;
  amcl_pose_.pose.pose.orientation.z = pose_msg->pose.pose.orientation.z;
  amcl_pose_.pose.pose.orientation.w = pose_msg->pose.pose.orientation.w;

  amcl_pose_.pose.covariance = pose_msg->pose.covariance;
}

void Autolocalization::getOdomPose( const nav_msgs::OdometryConstPtr& odom_msg )
{
  glob_start_ = true;
  odom_.x = odom_msg->pose.pose.position.x;
  odom_.y = odom_msg->pose.pose.position.y;
  odom_.theta = tf::getYaw(odom_msg->pose.pose.orientation);
}

void Autolocalization::getDistanceSensorsValues( const sensor_msgs::PointCloudConstPtr& distance_sensors_msg )
{
  distance_values_.clear();
  for (unsigned int i=0; i<distance_sensors_msg->points.size(); ++i) {
    double rho = std::sqrt(std::pow(distance_sensors_msg->points[i].x,2) + std::pow(distance_sensors_msg->points[i].y,2));
    double theta = std::atan2(distance_sensors_msg->points[i].y, distance_sensors_msg->points[i].x);
    polar_t p = {rho, theta};
    distance_values_.push_back(p);
  } 
}

void Autolocalization::checkCovariance( geometry_msgs::PoseWithCovarianceStamped amcl_pose ) 
{
  double var_x = amcl_pose.pose.covariance[0];
  double var_y = amcl_pose.pose.covariance[7];
  double var_th = amcl_pose.pose.covariance[35];

  if ( calibration_ ) {
    ROS_INFO("%s: variances (x,y,theta): (%f, %f, %f)", node_name_.c_str(), var_x, var_y, var_th);
    return;
  }
  
  if ( var_x <= tolerance_var_x_ &&
       var_y <= tolerance_var_y_ &&
       var_th <= tolerance_var_th_ ) {
    is_localized_ = true;
    ROS_INFO("%s: robot is localized.", node_name_.c_str());
  } else {
    is_localized_ = false;
  }
}

void Autolocalization::clearCostmapAndShutdown( void )
{
  stop();
  
  std_srvs::Empty::Request empty_req;
  std_srvs::Empty::Response empty_res;  
  if ( ros::service::call("/move_base/clear_costmaps", empty_req, empty_res) ) {
    ROS_INFO("%s: resetting costmap...", node_name_.c_str());
  }
  ros::shutdown();
}

void Autolocalization::setInitialCovariance( void )
{
  amcl_pose_.pose.covariance[0] = std::numeric_limits<double>::max();
  amcl_pose_.pose.covariance[7] = std::numeric_limits<double>::max();
  amcl_pose_.pose.covariance[35] = std::numeric_limits<double>::max();
}

void Autolocalization::calcCmdVel( void )
{
  switch ( state_ ) {
    case STOP: {
      odom_start_.x = odom_.x;
      odom_start_.y = odom_.y;
      odom_start_.theta = odom_.theta;
      cmd_vel_.linear.x = 0.0;
      cmd_vel_.linear.y = 0.0;
      cmd_vel_.angular.z = 0.0;
      break;
    }
    case ROTATING: {
      cmd_vel_.linear.x = 0.0;
      cmd_vel_.linear.y = 0.0;
      cmd_vel_.angular.z = max_vel_ang_;
      break;
    }
    case LINEAR: {
      cmd_vel_.linear.x = max_vel_lin_ * std::cos(free_dir_);
      cmd_vel_.linear.y = max_vel_lin_ * std::sin(free_dir_);
      cmd_vel_.angular.z = 0.0;      
      break;
    }
    default: {
      break;
    }
  }      
}

void Autolocalization::move( void )
{
  double d_xy = std::sqrt(std::pow(odom_.x-odom_start_.x, 2) + std::pow(odom_.x-odom_start_.x,2));
  double d_th = angles::normalize_angle(odom_.theta - odom_start_.theta);
  switch ( state_ ) {
    case STOP: {
      calcCmdVel();
      twist_pub_.publish(cmd_vel_);
      state_ = ROTATING;
      th_err_ = 0.0;
      break;
    }
    case ROTATING: {
      calcCmdVel();
      twist_pub_.publish(cmd_vel_);
      if ( std::fabs(d_th) > 0.05 ) {
        th_err_ = 0.05;
      }
      if ( std::fabs(d_th) < th_err_ ) {
        state_ = STOP;
        calcCmdVel();
        twist_pub_.publish(cmd_vel_);
        state_ = LINEAR;
      } 
      break;
    }
    case LINEAR: {
      if ( d_xy > 0.1 ) {
        state_ = STOP;
        calcCmdVel();
        twist_pub_.publish(cmd_vel_);
      } else {
        checkCollisions(distance_values_);
        calcCmdVel();
        twist_pub_.publish(cmd_vel_);
      }
      break;
    }
    default: {
      break;
    }
  }        
}

void Autolocalization::stop( void )
{
  cmd_vel_.linear.x = 0.0;
  cmd_vel_.linear.y = 0.0;
  cmd_vel_.angular.z = 0.0;
  twist_pub_.publish(cmd_vel_);  
}

void Autolocalization::checkCollisions( std::vector<polar_t> distance_values )
{
  if ( distance_values[i_distance_sensors_].rho < 0.45 ||
       distance_values[(i_distance_sensors_+1)%9].rho < 0.45 ||
       distance_values[(i_distance_sensors_-1)%9].rho < 0.45) {
    int i_min = 5;
    for (unsigned int i=0; i<distance_values.size(); ++i) {
      if ( distance_values[i_min].rho > distance_values[i].rho ) {
        i_min = i;
      }
    }
    i_distance_sensors_ = (i_min+4)%9;
    free_dir_ = distance_values[i_distance_sensors_].theta;
  }
}

}  // namespace squirrel_navigation

// 
// Autolocalization.cpp ends here
