// Autolocalization.h --- 
// 
// Filename: Autolocalization.h
// Description: Localisation on startup 
// Author: Federico Boniardi
// Maintainer: boniardi@cs.uni-freiburg.de
// Created: Mon Nov 24 10:20:05 2014 (+0100)
// Version: 0.1.0
// Last-Updated: Mon Dec 8 21:28:29 2014 (+0100)
//           By: Federico Boniardi
//     Update #: 2
// URL: 
// Keywords: 
// Compatibility: 
//   ROS Hydro, ROS Indigo
// 

// Commentary: 
//   Tested on: - ROS Hydro on Ubuntu 12.04
//              - ROS Indigo on Ubuntu 14.04
// 

// Change Log:
// 
// 
// 
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

#ifndef SQUIRREL_NAVIGATION_ROBOTLOCALIZER_H_
#define SQUIRREL_NAVIGATION_ROBOTLOCALIZER_H_

#include <ros/ros.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud.h>

#include <tf/tf.h>

#include <vector>

#include "Common.h"

namespace squirrel_navigation {

class Autolocalization
{
 public:
  Autolocalization( void );
  virtual ~Autolocalization( void );
  void spin( void );
  void waitForStarting( void );
 private:
  typedef struct { double rho; double theta; } polar_t;
  typedef enum {STOP, ROTATING, LINEAR} state_t;

  void getPoseWithCovariance( const geometry_msgs::PoseWithCovarianceStampedConstPtr&, std::string);
  void getOdomPose( const nav_msgs::OdometryConstPtr& );
  void getDistanceSensorsValues( const sensor_msgs::PointCloudConstPtr& );
  void clearCostmapAndShutdown( void );
  void checkCovariance( geometry_msgs::PoseWithCovarianceStamped );
  void setInitialCovariance( void );
  void calcCmdVel( void );
  void move( void );
  void stop( void );
  void checkCollisions( std::vector<polar_t> );
  
  ros::NodeHandle private_nh_, public_nh_;
  
  ros::Subscriber amcl_pose_sub_, initial_pose_sub_, odom_sub_, distance_sensors_sub_;

  ros::Publisher twist_pub_;

  std::string node_name_;
  
  geometry_msgs::Twist cmd_vel_;

  geometry_msgs::PoseWithCovarianceStamped amcl_pose_;

  geometry_msgs::Pose2D odom_, odom_start_, odom_first;
  
  bool is_localized_, glob_start_, pose_start_, amcl_start_;

  std::vector<polar_t> distance_values_;
  
  state_t state_;

  double free_dir_, th_err_;

  int i_distance_sensors_;
  
  // Parameters
  bool global_localization_, calibration_;
  double max_vel_ang_, max_vel_lin_;
  double tolerance_var_x_, tolerance_var_y_, tolerance_var_th_;
  std::string initial_pose_topic_, amcl_pose_topic_, odom_topic_, distance_sensors_topic_; 
};

}  // namespace squirrel_navigation

#endif /* SQUIRREL_NAVIGATION_ROBOTLOCALIZER_H_ */

// 
// Autolocalization.h ends here
