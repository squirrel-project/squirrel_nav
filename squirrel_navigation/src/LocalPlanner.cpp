// LocalPlanner.cpp --- 
// 
// Filename: LocalPlanner.cpp
// Description: Local planner based on robotino robotino_local_planner
// Author: indorewala@servicerobotics.eu
// Maintainer: Federico Boniardi (boniardi@cs.uni-freiburg.de)
// Created: Fri Nov 14 01:09:32 2014 (+0100)
// Version: 0.1.0
// Last-Updated: Fri Dec 5 17:10:34 2014 (+0100)
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
//              - ROS Indigo on Ubuntu 14.04
//   RGBD source: ASUS Xtion pro
//

// Code:

#include "squirrel_navigation/LocalPlanner.h"

#include <pluginlib/class_list_macros.h>

#include <cmath>

PLUGINLIB_DECLARE_CLASS(squirrel_navigation, LocalPlanner, squirrel_navigation::LocalPlanner, nav_core::BaseLocalPlanner)

namespace squirrel_navigation {

LocalPlanner::LocalPlanner( void ) :
    tf_(NULL),
    state_(FINISHED),
    curr_heading_index_(0),
    next_heading_index_(0),
    max_linear_vel_(0.0),
    min_linear_vel_(0.0),
    max_rotation_vel_(0.0),
    min_rotation_vel_(0.0),
    num_window_points_(10)
{
  ROS_INFO("%s/%s: LocalPlanner started", ros::this_node::getNamespace().c_str(), ros::this_node::getName().c_str());
}

LocalPlanner::~LocalPlanner( void )
{
  odom_sub_.shutdown();
  next_heading_pub_.shutdown();
}

void LocalPlanner::initialize( std::string name, tf::TransformListener* tf, costmap_2d::Costmap2DROS* costmap_ros )
{
  tf_ = tf;
  
  ros::NodeHandle private_nh("~/" + name);
  private_nh.param("heading_lookahead", heading_lookahead_, 0.3);
  private_nh.param("max_linear_vel", max_linear_vel_, 0.3);
  private_nh.param("min_linear_vel", min_linear_vel_, 0.1);
  private_nh.param("max_rotation_vel", max_rotation_vel_, 1.0);
  private_nh.param("min_rotation_vel", min_rotation_vel_, 0.3);
  private_nh.param("yaw_goal_tolerance", yaw_goal_tolerance_, 0.05);
  private_nh.param("xy_goal_tolerance", xy_goal_tolerance_, 0.10);
  private_nh.param("num_window_points", num_window_points_, 10);
  
  ros::NodeHandle global_node;
  odom_sub_ = global_node.subscribe<nav_msgs::Odometry>("odom", 1, &LocalPlanner::odomCallback, this);
  next_heading_pub_ = private_nh.advertise<visualization_msgs::Marker>("marker", 10);
}

bool LocalPlanner::computeVelocityCommands( geometry_msgs::Twist& cmd_vel )
{
  // Set all values of cmd_vel to zero
  cmd_vel.linear.x = 0.0;
  cmd_vel.linear.y = 0.0;
  cmd_vel.linear.z = 0.0;

  cmd_vel.angular.x = 0.0;
  cmd_vel.angular.y = 0.0;
  cmd_vel.angular.z = 0.0;

  // Set the default return value as false
  bool ret = false;

  // We need to compute the next heading point from the global plan
  computeNextHeadingIndex();

  switch(state_)
  {
    case ROTATING_TO_START:
      ret = rotateToStart( cmd_vel );
      break;
    case MOVING:
      ret = move( cmd_vel );
      break;
    case ROTATING_TO_GOAL:
      ret = rotateToGoal( cmd_vel );
      break;
    default:
      return true;
  }

  return ret;
}

bool LocalPlanner::isGoalReached( void )
{
  return (state_ == FINISHED);
}

bool LocalPlanner::setPlan( const std::vector<geometry_msgs::PoseStamped>& global_plan )
{
  if ( global_plan_.empty() ) {
    state_ = ROTATING_TO_START;
  } 

  global_plan_msg_.header.seq = 0;
  global_plan_msg_.header.stamp = ros::Time::now();
  global_plan_msg_.header.frame_id = std::string("map");
  
  // Make our copy of the global plan
  global_plan_.clear();
  global_plan_ = global_plan;
  global_plan_msg_.poses = global_plan_;
  
  curr_heading_index_ = 0;
  next_heading_index_ = 0;
  
  return true;
}

void LocalPlanner::odomCallback( const nav_msgs::OdometryConstPtr& msg )
{
  //we assume that the odometry is published in the frame of the base
  boost::mutex::scoped_lock lock(odom_lock_);
  base_odom_.header = msg->header;
  base_odom_.pose.position = msg->pose.pose.position;
  base_odom_.pose.orientation = msg->pose.pose.orientation;
}

void LocalPlanner::publishNextHeading( bool show )
{
  const geometry_msgs::PoseStamped& next_pose = global_plan_[next_heading_index_];

  visualization_msgs::Marker marker;
  marker.id = 0;
  marker.header.stamp = ros::Time::now();
  marker.header.frame_id= next_pose.header.frame_id;
  marker.ns = "waypoints";
  marker.type = visualization_msgs::Marker::CYLINDER;

  if(show) {
    marker.action = visualization_msgs::Marker::MODIFY;
    marker.pose = next_pose.pose;
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.2;
    marker.color.a = 0.5;
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
  } else {
    marker.action = visualization_msgs::Marker::DELETE;
  }
  next_heading_pub_.publish(marker);
}

bool LocalPlanner::rotateToStart( geometry_msgs::Twist& cmd_vel )
{
  geometry_msgs::PoseStamped rotate_goal;

  ros::Time now = ros::Time::now();
  global_plan_[next_heading_index_].header.stamp = now;
  
  try {
    boost::mutex::scoped_lock lock(odom_lock_);
    tf_->waitForTransform( base_odom_.header.frame_id, global_plan_[next_heading_index_].header.frame_id, now, ros::Duration( TRANSFORM_TIMEOUT ) );
    tf_->transformPose( base_odom_.header.frame_id, global_plan_[next_heading_index_], rotate_goal );
  } catch(tf::LookupException& ex) {
    ROS_ERROR("%s/%s: Lookup Error: %s\n", ros::this_node::getNamespace().c_str(), ros::this_node::getName().c_str(), ex.what());
    return false;
  } catch(tf::ConnectivityException& ex) {
    ROS_ERROR("%s/%s: Connectivity Error: %s\n", ros::this_node::getNamespace().c_str(), ros::this_node::getName().c_str(), ex.what());
    return false;
  } catch(tf::ExtrapolationException& ex) {
    ROS_ERROR("%s/%s: Extrapolation Error: %s\n", ros::this_node::getNamespace().c_str(), ros::this_node::getName().c_str(), ex.what());
    return false;
  }

  // Create a vector between the current odom pose to the next heading pose
  double x = rotate_goal.pose.position.x - base_odom_.pose.position.x;
  double y = rotate_goal.pose.position.y - base_odom_.pose.position.y;

  // Calculate the rotation between the current odom and the vector created above
  double rotation = (::atan2(y,x) - tf::getYaw(base_odom_.pose.orientation ) );

  rotation = mapToMinusPIToPI( rotation );

  if( fabs( rotation ) < yaw_goal_tolerance_ ) {
    state_ = MOVING;
    return true;
  }

  cmd_vel.angular.z = calRotationVel( rotation );

  return true;
}

bool LocalPlanner::move( geometry_msgs::Twist& cmd_vel )
{
  publishNextHeading();

  geometry_msgs::PoseStamped move_goal;
  ros::Time now = ros::Time::now();
  global_plan_[next_heading_index_].header.stamp = now;

  try {
    boost::mutex::scoped_lock lock(odom_lock_);
    tf_->waitForTransform( base_odom_.header.frame_id, global_plan_[next_heading_index_].header.frame_id, now, ros::Duration( TRANSFORM_TIMEOUT ) );
    tf_->transformPose( base_odom_.header.frame_id, global_plan_[next_heading_index_], move_goal );
  } catch(tf::LookupException& ex) {
    ROS_ERROR("%s/%s: Lookup Error: %s\n", ros::this_node::getNamespace().c_str(), ros::this_node::getName().c_str(), ex.what());
    return false;
  } catch(tf::ConnectivityException& ex) {
    ROS_ERROR("%s/%s: Connectivity Error: %s\n", ros::this_node::getNamespace().c_str(), ros::this_node::getName().c_str(), ex.what());
    return false;
  } catch(tf::ExtrapolationException& ex) {
    ROS_ERROR("%s/%s: Extrapolation Error: %s\n", ros::this_node::getNamespace().c_str(), ros::this_node::getName().c_str(), ex.what());
    return false;
  }

  // Create a vector between the current odom pose to the next heading pose
  double x = move_goal.pose.position.x - base_odom_.pose.position.x;
  double y = move_goal.pose.position.y - base_odom_.pose.position.y;

  // Calculate the rotation between the current odom and the vector created above
  double rotation = (::atan2(y,x) - tf::getYaw(base_odom_.pose.orientation ) );

  rotation = mapToMinusPIToPI( rotation );

  double vel_th = fabs( rotation ) < yaw_goal_tolerance_ ? 0.0 : calRotationVel(rotation);
  double vel_x = calLinearVel() * ( rotation < 0.5*PI/2 && rotation > -0.5*PI);
  
  cmd_vel.linear.x = vel_x;
  cmd_vel.angular.z = vel_th;
  
  // The distance from the robot's current pose to the next heading pose
  double distance_to_next_heading = linearDistance(base_odom_.pose.position, move_goal.pose.position );

  // We are approaching the goal position, slow down
  if( next_heading_index_ == (int) global_plan_.size()-1) {
    // Reached the goal, now we can stop and rotate the robot to the goal position
    if( distance_to_next_heading < xy_goal_tolerance_ ) {
      cmd_vel.linear.x = 0.0;
      cmd_vel.angular.z = 0.0;
      ROS_INFO("rotate to goal");
      state_ = ROTATING_TO_GOAL;
      return true;
    }
  }
  return true;
}

bool LocalPlanner::rotateToGoal( geometry_msgs::Twist& cmd_vel )
{
  geometry_msgs::PoseStamped rotate_goal;
  
  ros::Time now = ros::Time::now();
  global_plan_[next_heading_index_].header.stamp = now;
  
  try {
    boost::mutex::scoped_lock lock(odom_lock_);
    tf_->waitForTransform( base_odom_.header.frame_id, global_plan_[next_heading_index_].header.frame_id, now, ros::Duration( TRANSFORM_TIMEOUT ) );
    tf_->transformPose( base_odom_.header.frame_id, global_plan_[next_heading_index_], rotate_goal );
  } catch(tf::LookupException& ex) {
    ROS_ERROR("%s/%s: Lookup Error: %s\n", ros::this_node::getNamespace().c_str(), ros::this_node::getName().c_str(), ex.what());
    return false;
  } catch(tf::ConnectivityException& ex) {
    ROS_ERROR("%s/%s: Connectivity Error: %s\n", ros::this_node::getNamespace().c_str(), ros::this_node::getName().c_str(), ex.what());
    return false;
  } catch(tf::ExtrapolationException& ex) {
    ROS_ERROR("%s/%s: Extrapolation Error: %s\n", ros::this_node::getNamespace().c_str(), ros::this_node::getName().c_str(), ex.what());
    return false;
  }
  
  double rotation = tf::getYaw( rotate_goal.pose.orientation ) -
      tf::getYaw( base_odom_.pose.orientation );

  if( fabs( rotation ) < yaw_goal_tolerance_ ) {
    state_ = FINISHED;
    if ( global_plan_.size() > 0 ) {
      global_plan_.clear();
    } 
    ROS_INFO("%s/%s: Goal reached", ros::this_node::getNamespace().c_str(), ros::this_node::getName().c_str());
    return true;
  }
  
  cmd_vel.angular.z = calRotationVel( rotation );

  return true;
}

void LocalPlanner::computeNextHeadingIndex( void )
{
  geometry_msgs::PoseStamped next_heading_pose;

  for( unsigned int i = curr_heading_index_; i < global_plan_.size() - 1; ++i ) {
    boost::mutex::scoped_lock lock(odom_lock_);
    ros::Time now = ros::Time::now();
    global_plan_[i].header.stamp = now;

    try {
      tf_->waitForTransform( base_odom_.header.frame_id, global_plan_[i].header.frame_id, now, ros::Duration( TRANSFORM_TIMEOUT ) );
      tf_->transformPose( base_odom_.header.frame_id, global_plan_[i], next_heading_pose );
    } catch(tf::LookupException& ex) {
      ROS_ERROR("%s/%s:Lookup Error: %s\n", ros::this_node::getNamespace().c_str(), ros::this_node::getName().c_str(), ex.what());
      return;
    } catch(tf::ConnectivityException& ex) {
      ROS_ERROR("%s/%s: Connectivity Error: %s\n", ros::this_node::getNamespace().c_str(), ros::this_node::getName().c_str(), ex.what());
      return;
    } catch(tf::ExtrapolationException& ex) {
      ROS_ERROR("%s/%s: Extrapolation Error: %s\n", ros::this_node::getNamespace().c_str(), ros::this_node::getName().c_str(), ex.what());
      return;
    }

    double dist = linearDistance( base_odom_.pose.position,
                                  next_heading_pose.pose.position );

    if( dist > heading_lookahead_) {
      next_heading_index_ = i;
      return;
    } else {
      curr_heading_index_++;
    }
  }
  next_heading_index_ = global_plan_.size() - 1;
}

double LocalPlanner::calLinearVel( void )
{
  double vel = 0.0;

  int effective_num_window_points =  std::min(num_window_points_, int(global_plan_.size()-1));
  
  if( next_heading_index_ < effective_num_window_points ) {
    return vel;
  }
  
  unsigned int beg_index = next_heading_index_ - effective_num_window_points;

  double straight_dist = linearDistance(global_plan_[beg_index].pose.position,
                                        global_plan_[next_heading_index_].pose.position);

  double path_dist = 0.0;

  for(unsigned int i = beg_index; i < (unsigned int)next_heading_index_; ++i) {
    path_dist = path_dist + linearDistance(global_plan_[i].pose.position, global_plan_[i + 1].pose.position);
  }

  double diff = path_dist - straight_dist;

  vel = 0.001 * ( 1 / diff );

  if( vel > max_linear_vel_ )
    vel = max_linear_vel_;

  if( vel < min_linear_vel_ )
    vel = min_linear_vel_;

  return vel;
}

double LocalPlanner::calRotationVel( double rotation )
{
  double vel = 0.0;

  int sign = 1;

  if( rotation < 0.0 )
    sign = -1;

  if( fabs(rotation) > max_rotation_vel_) {
    vel = sign * max_rotation_vel_;
  } else if ( fabs(rotation) < min_rotation_vel_ ) {
    vel = sign * min_rotation_vel_;
  } else {
    vel = rotation;
  }

  return vel;
}

double LocalPlanner::linearDistance( geometry_msgs::Point p1, geometry_msgs::Point p2 )
{
  return std::sqrt( std::pow( p2.x - p1.x, 2) + std::pow( p2.y - p1.y, 2)  );
}

double LocalPlanner::mapToMinusPIToPI( double angle )
{
  double angle_overflow = static_cast<double>( static_cast<int>(angle / PI ) );

  if( angle_overflow > 0.0 ) {
    angle_overflow = ceil( angle_overflow / 2.0 );
  } else {
    angle_overflow = floor( angle_overflow / 2.0 );
  }

  angle -= 2 * PI * angle_overflow;
  return angle;
}

} // Namespace squirrel_navigation

// 
// LocalPlanner.cpp ends here
