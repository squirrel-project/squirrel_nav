// LocalPlanner.cpp --- 
// 
// Filename: LocalPlanner.cpp
// Description: Local planner based on robotino robotino_local_planner
// Author: indorewala@servicerobotics.eu
// Maintainer: Federico Boniardi (boniardi@cs.uni-freiburg.de)
// Created: Fri Nov 14 01:09:32 2014 (+0100)
// Version: 0.1.0
// Last-Updated: Wed Feb 25 15:58:57 2015 (+0100)
//           By: Federico Boniardi
//     Update #: 4
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
    trajectory_planner_(NULL),
    tf_(NULL),
    state_(FINISHED),
    planner_type_(DIJKSTRA),
    curr_heading_index_(0),
    next_heading_index_(0),
    max_linear_vel_(0.0),
    min_linear_vel_(0.0),
    max_rotation_vel_(0.0),
    max_in_place_rotation_vel_(0.0),
    min_rotation_vel_(0.0),
    num_window_points_(10)
{
  ROS_INFO("squirrel_navigation::LocalPlanner started");
}

LocalPlanner::~LocalPlanner( void )
{
  if ( trajectory_planner_ )
    delete trajectory_planner_;
  odom_sub_.shutdown();
  next_heading_pub_.shutdown();
  update_sub_.shutdown();
}

void LocalPlanner::initialize( std::string name, tf::TransformListener* tf, costmap_2d::Costmap2DROS* costmap_ros )
{
  name_ = name;
  
  tf_ = tf;

  if ( not trajectory_planner_ )
    trajectory_planner_ = new base_local_planner::TrajectoryPlannerROS(name+"/trajectory_planner",tf,costmap_ros);

  ros::NodeHandle pnh("~/"+name);
  pnh.param<bool>("verbose", verbose_, false);
  
  ros::NodeHandle pnh_tt("~/" + name+"/trajectory_tracker");
  pnh_tt.param<double>("heading_lookahead", heading_lookahead_, 0.3);
  pnh_tt.param<double>("max_linear_vel", max_linear_vel_, 0.2);
  pnh_tt.param<double>("min_linear_vel", min_linear_vel_, 0.0);
  pnh_tt.param<double>("max_rotation_vel", max_rotation_vel_, 0.5);
  pnh_tt.param<double>("max_in_place_rotation_vel", max_in_place_rotation_vel_, 1.0);
  pnh_tt.param<double>("min_rotation_vel", min_rotation_vel_, 0.0);
  pnh_tt.param<double>("yaw_goal_tolerance", yaw_goal_tolerance_, 0.05);
  pnh_tt.param<double>("xy_goal_tolerance", xy_goal_tolerance_, 0.10);
  pnh_tt.param<int>("num_window_points", num_window_points_, 10);

  if ( max_rotation_vel_ <= 0 ) {
    ROS_WARN("max_rotation_vel has been chosen to be non positive. Reverting to 0.3 (rad/s)");
    max_rotation_vel_ = 0.3;
  }
  
  ros::NodeHandle nh;
  odom_sub_ = nh.subscribe<nav_msgs::Odometry>("odom", 1, &LocalPlanner::odomCallback_, this);
  update_sub_ = nh.subscribe<std_msgs::Bool>("/plan_with_footprint", 1, &LocalPlanner::plannerUpdateCallback_, this);
  next_heading_pub_ = pnh_tt.advertise<visualization_msgs::Marker>("marker", 10);
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

  switch ( planner_type_ ) {

    case DIJKSTRA: {
      // We need to compute the next heading point from the global plan  
      computeNextHeadingIndex_();
      switch( state_ ) {
        case ROTATING_TO_START:
          ret = rotateToStart_( cmd_vel );
          break;
        case MOVING:
          ret = move_( cmd_vel );
          break;
        case ROTATING_TO_GOAL:
          ret = rotateToGoal_( cmd_vel );
          break;
        default:
          return true;
      }
      return true;
    }

    case LATTICE: {
      trajectory_planner_->computeVelocityCommands(cmd_vel);
      return true;
    }
  }      
  
  return ret;
}

bool LocalPlanner::isGoalReached( void )
{
  if ( state_ == FINISHED ) {
    ROS_INFO("%s: Goal reached.", name_.c_str());
    return true;
  } else {
    return false;
  }
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

  trajectory_planner_->setPlan(global_plan); 
      
  return true;
}

bool LocalPlanner::move_( geometry_msgs::Twist& cmd_vel )
{
  publishNextHeading_();

  geometry_msgs::PoseStamped move__goal;
  ros::Time now = ros::Time::now();
  global_plan_[next_heading_index_].header.stamp = now;

  try {
    boost::mutex::scoped_lock lock(odom_lock_);
    tf_->waitForTransform( base_odom_.header.frame_id, global_plan_[next_heading_index_].header.frame_id, now, ros::Duration( TRANSFORM_TIMEOUT ) );
    tf_->transformPose( base_odom_.header.frame_id, global_plan_[next_heading_index_], move__goal );
  } catch ( tf::LookupException& ex ) {
    ROS_ERROR("%s: Lookup Error: %s", name_.c_str(), ex.what());
    return false;
  } catch ( tf::ConnectivityException& ex ) {
    ROS_ERROR("%s: Connectivity Error: %s", name_.c_str(), ex.what());
    return false;
  } catch ( tf::ExtrapolationException& ex ) {
    ROS_ERROR("%s: Extrapolation Error: %s", name_.c_str(), ex.what());
    return false;
  }

  // Create a vector between the current odom pose to the next heading pose
  double x = move__goal.pose.position.x - base_odom_.pose.position.x;
  double y = move__goal.pose.position.y - base_odom_.pose.position.y;

  // Calculate the rotation between the current odom and the vector created above
  double rotation = (::atan2(y,x) - tf::getYaw(base_odom_.pose.orientation ) );

  rotation = angles::normalize_angle( rotation );

  double vel_th = fabs( rotation ) < yaw_goal_tolerance_ ? 0.0 : calRotationVel_(rotation);
  double vel_x = calLinearVel_() * cutOff_( rotation );
  
  cmd_vel.linear.x = vel_x;
  cmd_vel.angular.z = vel_th;
  
  // The distance from the robot's current pose to the next heading pose
  double distance_to_next_heading = linearDistance_(base_odom_.pose.position, move__goal.pose.position );

  // We are approaching the goal position, slow down
  if( next_heading_index_ == (int) global_plan_.size()-1) {
    // Reached the goal, now we can stop and rotate the robot to the goal position
    if( distance_to_next_heading < xy_goal_tolerance_ ) {
      cmd_vel.linear.x = 0.0;
      cmd_vel.angular.z = 0.0;
      state_ = ROTATING_TO_GOAL;
      return true;
    }
  }
  return true;
}

bool LocalPlanner::rotateToGoal_( geometry_msgs::Twist& cmd_vel )
{
  geometry_msgs::PoseStamped rotate_goal;
  
  ros::Time now = ros::Time::now();
  global_plan_[next_heading_index_].header.stamp = now;
  
  try {
    boost::mutex::scoped_lock lock(odom_lock_);
    tf_->waitForTransform( base_odom_.header.frame_id, global_plan_[next_heading_index_].header.frame_id, now, ros::Duration( TRANSFORM_TIMEOUT ) );
    tf_->transformPose( base_odom_.header.frame_id, global_plan_[next_heading_index_], rotate_goal );
  } catch ( tf::LookupException& ex ) {
    ROS_ERROR("%s: Lookup Error: %s", name_.c_str(), ex.what());
    return false;
  } catch ( tf::ConnectivityException& ex ) {
    ROS_ERROR("%s: Connectivity Error: %s", name_.c_str(), ex.what());
    return false;
  } catch ( tf::ExtrapolationException& ex ) {
    ROS_ERROR("%s: Extrapolation Error: %s", name_.c_str(), ex.what());
    return false;
  }
  
  double rotation = tf::getYaw( rotate_goal.pose.orientation ) -
      tf::getYaw( base_odom_.pose.orientation );

  rotation = angles::normalize_angle(rotation);
  
  if( fabs( rotation ) < yaw_goal_tolerance_ ) {
    if ( global_plan_.size() > 0 ) {
      global_plan_.clear();
    }
    state_ = FINISHED;
    cmd_vel.angular.z = 0.0;
    return true;
  }
  
  cmd_vel.angular.z = ( max_in_place_rotation_vel_ / max_rotation_vel_ ) * calRotationVel_( rotation ) ;

  return true;
}

bool LocalPlanner::rotateToStart_( geometry_msgs::Twist& cmd_vel )
{
  geometry_msgs::PoseStamped rotate_goal;

  ros::Time now = ros::Time::now();
  global_plan_[next_heading_index_].header.stamp = now;
  
  try {
    boost::mutex::scoped_lock lock(odom_lock_);
    tf_->waitForTransform( base_odom_.header.frame_id, global_plan_[next_heading_index_].header.frame_id, now, ros::Duration( TRANSFORM_TIMEOUT ) );
    tf_->transformPose( base_odom_.header.frame_id, global_plan_[next_heading_index_], rotate_goal );
  } catch ( tf::LookupException& ex ) {
    ROS_ERROR("%s: Lookup Error: %s", name_.c_str(), ex.what());
    return false;
  } catch ( tf::ConnectivityException& ex ) {
    ROS_ERROR("%s: Connectivity Error: %s", name_.c_str(), ex.what());
    return false;
  } catch ( tf::ExtrapolationException& ex ) {
    ROS_ERROR("%s: Extrapolation Error: %s", name_.c_str(), ex.what());
    return false;
  }

  // Create a vector between the current odom pose to the next heading pose
  double x = rotate_goal.pose.position.x - base_odom_.pose.position.x;
  double y = rotate_goal.pose.position.y - base_odom_.pose.position.y;

  // Calculate the rotation between the current odom and the vector created above
  double rotation = (::atan2(y,x) - tf::getYaw(base_odom_.pose.orientation ) );

  rotation = angles::normalize_angle( rotation );

  if( fabs( rotation ) < yaw_goal_tolerance_ ) {
    state_ = MOVING;
    return true;
  }

  cmd_vel.angular.z = ( max_in_place_rotation_vel_ / max_rotation_vel_ ) * calRotationVel_( rotation );

  return true;
}

double LocalPlanner::calLinearVel_( void )
{
  double vel = 0.0;

  int effective_num_window_points =  std::min(num_window_points_, int(global_plan_.size()-1));
  
  if( next_heading_index_ < effective_num_window_points ) {
    return vel;
  }
  
  unsigned int beg_index = next_heading_index_ - effective_num_window_points;

  double straight_dist = linearDistance_(global_plan_[beg_index].pose.position,
                                        global_plan_[next_heading_index_].pose.position);

  double path_dist = 0.0;

  for(unsigned int i = beg_index; i < (unsigned int)next_heading_index_; ++i) {
    path_dist = path_dist + linearDistance_(global_plan_[i].pose.position, global_plan_[i + 1].pose.position);
  }

  double diff = path_dist - straight_dist;

  vel = 0.001 * ( 1 / diff );

  if( vel > max_linear_vel_ )
    vel = max_linear_vel_;

  if( vel < min_linear_vel_ )
    vel = min_linear_vel_;

  return vel;
}

double LocalPlanner::calRotationVel_( double rotation )
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

double LocalPlanner::linearDistance_( geometry_msgs::Point p1, geometry_msgs::Point p2 )
{
  return std::sqrt( std::pow( p2.x - p1.x, 2) + std::pow( p2.y - p1.y, 2)  );
}

void LocalPlanner::computeNextHeadingIndex_( void )
{
  geometry_msgs::PoseStamped next_heading_pose;

  for( unsigned int i = curr_heading_index_; i < global_plan_.size() - 1; ++i ) {
    boost::mutex::scoped_lock lock(odom_lock_);
    ros::Time now = ros::Time::now();
    global_plan_[i].header.stamp = now;

    try {
      tf_->waitForTransform( base_odom_.header.frame_id, global_plan_[i].header.frame_id, now, ros::Duration( TRANSFORM_TIMEOUT ) );
      tf_->transformPose( base_odom_.header.frame_id, global_plan_[i], next_heading_pose );
    } catch ( tf::LookupException& ex ) {
      ROS_ERROR("%s: Lookup Error: %s", name_.c_str(), ex.what());
      return;
    } catch ( tf::ConnectivityException& ex ) {
      ROS_ERROR("%s: Connectivity Error: %s", name_.c_str(), ex.what());
      return;
    } catch ( tf::ExtrapolationException& ex ) {
      ROS_ERROR("%s: Extrapolation Error: %s", name_.c_str(), ex.what());
      return;
    }

    double dist = linearDistance_( base_odom_.pose.position,
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

void LocalPlanner::odomCallback_( const nav_msgs::OdometryConstPtr& msg )
{
  //we assume that the odometry is published in the frame of the base
  boost::mutex::scoped_lock lock(odom_lock_);
  base_odom_.header = msg->header;
  base_odom_.pose.position = msg->pose.pose.position;
  base_odom_.pose.orientation = msg->pose.pose.orientation;
}

void LocalPlanner::plannerUpdateCallback_( const std_msgs::Bool::ConstPtr& use_footprint_msg )
{
  if ( use_footprint_msg->data == true ) {
    planner_type_ = LATTICE;
    if ( verbose_ )
      ROS_INFO_STREAM(name_ << ": Using TrajectoryPlanner as controller.");
  } else {
    planner_type_ = DIJKSTRA;
    if ( verbose_ )
      ROS_INFO_STREAM(name_ << ": Using TrajectoryTracker as controller.");
  }
}

void LocalPlanner::publishNextHeading_( bool show )
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

} // Namespace squirrel_navigation

// 
// LocalPlanner.cpp ends here
