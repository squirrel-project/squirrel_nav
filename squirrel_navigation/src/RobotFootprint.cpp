// RobotFootprint.cpp --- 
// 
// Filename: RobotFootprint.cpp
// Description: 
// Author: Federico Boniardi
// Maintainer: boniardi@informatik.uni-freiburg.de
// Created: Fri Nov 20 13:57:41 2015 (+0100)
// Version: 0.1.0
// Last-Updated: Mon Mar 21 16:02:23 2016 (+0100)
//           By: Federico Boniardi
//     Update #: 1
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

// Code:

#include "squirrel_navigation/RobotFootprint.h"

namespace squirrel_navigation {

RobotFootprint::RobotFootprint( void )
{
  ros::NodeHandle pnh("~");
  footprint_pub_ = pnh.advertise<geometry_msgs::PolygonStamped>("footprint",1);
}

RobotFootprint::~RobotFootprint( void )
{
  // Empty
}

void RobotFootprint::updateCurrentFootprint( double robot_x, double robot_y, double robot_yaw,  const std::vector<geometry_msgs::Point>& footprint_spec )
{  
  const double cos_th = std::cos(robot_yaw);
  const double sin_th = std::sin(robot_yaw);

  geometry_msgs::PolygonStamped footprint_msg;
  footprint_msg.header.frame_id = "/map";
  footprint_msg.header.stamp = ros::Time::now();
      
  footprint_.clear();
  footprint_.reserve(footprint_spec.size());
  footprint_msg.polygon.points.reserve(footprint_spec.size());
  for (unsigned int i = 0; i < footprint_spec.size(); ++i)  {
    const double px = robot_x + (footprint_spec[i].x * cos_th - footprint_spec[i].y * sin_th);
    const double py = robot_y + (footprint_spec[i].x * sin_th + footprint_spec[i].y * cos_th);
    footprint_.emplace_back(px,py);

    geometry_msgs::Point32 p;
    p.x = px;
    p.y = py;
    footprint_msg.polygon.points.push_back(p);
  }

  footprint_pub_.publish(footprint_msg);
}

bool RobotFootprint::isInside( double px, double py ) const
{
  return isInsideFootprint(footprint_,CGAL_Point2D(px,py),ckern_);
}

}  // namespace squirrel_navigation

// 
// RobotFootprint.cpp ends here
