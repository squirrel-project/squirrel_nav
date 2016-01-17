// ab_filter.h --- 
// 
// Filename: ab_filter.h
// Description: alpha/beta filtering
// Author: Federico Boniardi
// Maintainer: boniardi@informatik.uni-freiburg.de
// Created: Tue Jan 12 18:47:00 2016 (+0100)
// Version: 0.0.1 
//

// Code:

#ifndef LASER_SCAN_MATCHER_AB_FILTER_H_
#define LASER_SCAN_MATCHER_AB_FILTER_H_

#include <ros/time.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Header.h>

#include <tf/tf.h>

#include <cmath>

namespace scan_tools {

class AlphaBetaFilter
{
 public:
  AlphaBetaFilter( void );
  AlphaBetaFilter( double, double );
  virtual ~AlphaBetaFilter( void );

  void init( const ros::Time& );
  geometry_msgs::Twist operator()( const geometry_msgs::PoseStamped& );
  
 private:
  double alpha_, beta_;
  
  geometry_msgs::Pose x_;
  geometry_msgs::Twist v_;
  ros::Time t_;
};

}  // namespace scan_tools

#endif /* LASER_SCAN_MATCHER_AB_FILTER_H_ */

// 
// ab_filter.h ends here
