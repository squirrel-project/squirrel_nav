// FootprintLayer.cpp --- 
// 
// Filename: FootprintLayer.cpp
// Description: 
// Author: Federico Boniardi
// Maintainer: boniardi@informatik.uni-freiburg.de
// Created: Fri Nov 20 13:57:41 2015 (+0100)
// Version: 0.1.0
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
//   /*********************************************************************
// *
// * Software License Agreement (BSD License)
// *
// *  Copyright (c) 2008, 2013, Willow Garage, Inc.
// *  All rights reserved.
// *
// *  Redistribution and use in source and binary forms, with or without
// *  modification, are permitted provided that the following conditions
// *  are met:
// *
// *   * Redistributions of source code must retain the above copyright
// *     notice, this list of conditions and the following disclaimer.
// *   * Redistributions in binary form must reproduce the above
// *     copyright notice, this list of conditions and the following
// *     disclaimer in the documentation and/or other materials provided
// *     with the distribution.
// *   * Neither the name of Willow Garage, Inc. nor the names of its
// *     contributors may be used to endorse or promote products derived
// *     from this software without specific prior written permission.
// *
// *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// *  POSSIBILITY OF SUCH DAMAGE.
// *
// * Author: Eitan Marder-Eppstein
// *         David V. Lu!!
// *********************************************************************/
// 
//

// Code:

#include "squirrel_navigation/FootprintLayer.h"

PLUGINLIB_EXPORT_CLASS(squirrel_navigation::FootprintLayer, costmap_2d::Layer)

namespace squirrel_navigation {

void FootprintLayer::onInitialize( void )
{
  ros::NodeHandle pnh("~/" + name_), nh;
  current_ = false;

  dsrv_ = nullptr;

  footprint_pub_ = pnh.advertise<geometry_msgs::PolygonStamped>("footprint", 1);

  if ( not dsrv_ )
    dsrv_ = new dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>(pnh);

  dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>::CallbackType cb = boost::bind(&FootprintLayer::reconfigureCallback, this, _1, _2);
  dsrv_->setCallback(cb);

  current_ = true;
}

FootprintLayer::~FootprintLayer( void )
{
  if ( dsrv_ )
    delete dsrv_;
}

void FootprintLayer::updateBounds( double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y, double* max_x, double* max_y )
{
  if ( not enabled_)
    return;

  const std::vector<geometry_msgs::Point>& footprint_spec = getFootprint();

  const double cos_th = std::cos(robot_yaw);
  const double sin_th = std::sin(robot_yaw);

  geometry_msgs::PolygonStamped footprint_msg;  
  footprint_msg.header.stamp = ros::Time::now();
  footprint_msg.header.frame_id = layered_costmap_->getGlobalFrameID();
  footprint_msg.polygon.points.reserve(footprint_spec.size());
    
  footprint_.reserve(footprint_spec.size());

  for (unsigned int i = 0; i < footprint_spec.size(); ++i)  {
    geometry_msgs::Point32 new_pt;
    new_pt.x = robot_x + (footprint_spec[i].x * cos_th - footprint_spec[i].y * sin_th);
    new_pt.y = robot_y + (footprint_spec[i].x * sin_th + footprint_spec[i].y * cos_th);
    footprint_msg.polygon.points.push_back(new_pt);
    footprint_.emplace_back(new_pt.x,new_pt.y);

    const double px = new_pt.x, py = new_pt.y;
    *min_x = std::min(px,*min_x);
    *min_y = std::min(py,*min_y);
    *max_x = std::max(px,*max_x);
    *max_y = std::max(py,*max_y);    
  }

  footprint_pub_.publish(footprint_msg);
}

void FootprintLayer::updateCosts( costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j )
{
  if ( not enabled_)
    return;

  std::vector<geometry_msgs::Point> footprint_points = toPointVector(footprint_);
  master_grid.setConvexPolygonCost(footprint_points, costmap_2d::FREE_SPACE);
}

void FootprintLayer::reconfigureCallback( costmap_2d::GenericPluginConfig &config, uint32_t level )
{
  enabled_ = config.enabled;
}

}  // namespace squirrel_navigation

// 
// FootprintLayer.cpp ends here
