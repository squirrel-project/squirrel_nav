// ab_filter.cpp --- 
// 
// Filename: ab_filter.cpp
// Description: alpha/beta filtering
// Author: Federico Boniardi
// Maintainer: boniardi@informatik.uni-freiburg.de
// Created: Tue Jan 12 18:51:48 2016 (+0100)
// Version: 0.0.1
// 

// Code:

#include <laser_scan_matcher/ab_filter.h>

namespace scan_tools {

AlphaBetaFilter::AlphaBetaFilter( void ) :
    alpha_(0.1),
    beta_(2*(2-alpha_)-4*std::sqrt(1-alpha_))
{
  x_.orientation = tf::createQuaternionMsgFromYaw(0.0);

  ros::NodeHandle pnh("~");
  pnh.param<double>("alpha", alpha_, alpha_);
  pnh.param<double>("beta", beta_, beta_);
}

AlphaBetaFilter::AlphaBetaFilter( double alpha, double beta ) :
    alpha_(alpha),
    beta_(beta)
{
  x_.orientation = tf::createQuaternionMsgFromYaw(0.0);
}

AlphaBetaFilter::~AlphaBetaFilter( void )
{
  // Empty
}

void AlphaBetaFilter::init( const ros::Time& start )
{
  t_ = start;
}

geometry_msgs::Twist AlphaBetaFilter::operator()( const geometry_msgs::PoseStamped& X )
{
  double dt = (X.header.stamp - t_).toSec();

  double vx, vy, vth, x, y, th;
  
  x = x_.position.x;
  y = x_.position.y;
  th = tf::getYaw(x_.orientation);

  vx = v_.linear.x;
  vy = v_.linear.y;
  vth = v_.angular.z;
  
  double dx = X.pose.position.x - x - dt * vx;
  double dy = X.pose.position.y - y - dt * vy;
  double dth = tf::getYaw(X.pose.orientation) - th - dt * vth;
  
  x_.position.x = x + dt * vx + alpha_ * dx;
  x_.position.y = y + dt * vy + alpha_ * dy;
  x_.orientation = tf::createQuaternionMsgFromYaw(th + dt * vth + alpha_ * dth); 
  
  v_.linear.x = vx + (beta_/dt) * dx;
  v_.linear.y = vy + (beta_/dt) * dy;
  v_.angular.z = vth + (beta_/dt) * dth;

  t_ = X.header.stamp;
  
  // Computing the twist
  double curr_th = tf::getYaw(X.pose.orientation);
  geometry_msgs::Twist twist;
  twist.linear.x = std::cos(-curr_th)*v_.linear.x - std::sin(-curr_th)*v_.linear.y;
  twist.linear.y = std::sin(-curr_th)*v_.linear.x + std::cos(-curr_th)*v_.linear.y;
  twist.angular.z = v_.angular.z;
  
  return twist;
}

}  // namespace scan_tools

// 
// ab_filter.cpp ends here
