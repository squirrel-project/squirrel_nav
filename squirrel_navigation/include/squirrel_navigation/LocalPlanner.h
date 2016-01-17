// LocalPlanner.h --- 
// 
// Filename: LocalPlanner.h
// Description: Local planner based on robotino_local_planner
// Author: indorewala@servicerobotics.eu
// Maintainer: Federico Boniardi (boniardi@cs.uni-freiburg.de)
// Created: Fri Nov 14 01:10:39 2014 (+0100)
// Version: 0.1.0
// Last-Updated: Wed Nov 26 15:47:36 2014 (+0100)
//           By: Federico Boniardi
//     Update #: 3
// URL: 
// Keywords: 
// Compatibility: 
//   ROS Hydro, ROS Indigo
// 

// Commentary: 
//   
//   The code therein is adapted from the package robotino_local_planner
//   by indorewala@servicerobotics.eu, available at
//
//     http://svn.openrobotino.org/robotino-ros-pkg/branches/robotino_navigation/robotino_navigation/
//
//   and licensed under BSD software license. Further documentation available at
//
//     http://wiki.ros.org/robotino_local_planner
//
//     
//   Tested on: - ROS Hydro on Ubuntu 12.04
//               - ROS Indigo on Ubuntu 14.04
//    RGBD source: ASUS Xtion pro
// 

// Code:

#ifndef SQUIRREL_NAVIGATION_LOCALPLANNER_H_
#define SQUIRREL_NAVIGATION_LOCALPLANNER_H_

#include <ros/ros.h>

#include <nav_core/base_local_planner.h>

#include <costmap_2d/costmap_2d_ros.h>

#include <tf/transform_listener.h>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point32.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Bool.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <string>
#include <vector>

#include "squirrel_navigation/Common.h"

namespace squirrel_navigation {
  
class LocalPlanner : public nav_core::BaseLocalPlanner {
 public:
  LocalPlanner( void );
  ~LocalPlanner( void );

  bool computeVelocityCommands( geometry_msgs::Twist& );
  void initialize( std::string, tf::TransformListener*, costmap_2d::Costmap2DROS* );
  bool isGoalReached( void );
  bool setPlan( const std::vector<geometry_msgs::PoseStamped>& );

 private:  
  typedef enum { ROTATING_TO_START, MOVING, ROTATING_TO_GOAL, FINISHED } state_t;
  typedef enum { DIJKSTRA, LATTICE } planner_t;
  
  tf::TransformListener* tf_;

  std::vector<geometry_msgs::PoseStamped> global_plan_;

  geometry_msgs::PoseStamped base_odom_;
  
  nav_msgs::Path global_plan_msg_;
  
  ros::Subscriber odom_sub_, update_sub_;
  
  ros::Publisher next_heading_pub_;
  
  state_t state_;

  boost::mutex odom_lock_;

  int curr_heading_index_, next_heading_index_;

  planner_t planner_type_;
  
  // Parameters
  double heading_lookahead_;
  double max_linear_vel_, min_linear_vel_;
  double max_rotation_vel_, min_rotation_vel_, max_in_place_rotation_vel_;
  double yaw_goal_tolerance_, xy_goal_tolerance_;
  int num_window_points_;

  // Name and references
  std::string name_;

  bool move_( geometry_msgs::Twist& );
  bool rotateToGoal_( geometry_msgs::Twist& );
  bool rotateToStart_( geometry_msgs::Twist& );
  double calLinearVel_( void );
  double calRotationVel_( double );
  double linearDistance_( geometry_msgs::Point, geometry_msgs::Point );
  double mapToMinusPIToPI_( double );
  void computeNextHeadingIndex_( void );
  void odomCallback_( const nav_msgs::OdometryConstPtr& );
  void plannerUpdateCallback_( const std_msgs::Bool::ConstPtr& );
  void publishNextHeading_( bool show = true );
};

} // namespace squirrel_navigation

#endif /* SQUIRREL_NAVIGATION_LOCALPLANNER_H_ */

// 
// LocalPlanner.h ends here
