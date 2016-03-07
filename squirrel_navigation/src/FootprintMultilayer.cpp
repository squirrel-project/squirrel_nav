// FootprintMultilayer.cpp --- 
// 
// Filename: FootprintMultilayer.cpp
// Description: 
// Author: Federico Boniardi
// Maintainer: 
// Created: Mon Mar  7 13:10:28 2016 (+0100)
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

#include "squirrel_navigation/FootprintMultilayer.h"

PLUGINLIB_EXPORT_CLASS(squirrel_navigation::FootprintMultilayer, costmap_2d::Layer)

namespace squirrel_navigation {

void FootprintMultilayer::onInitialize( void )
{
  ros::NodeHandle pnh("~/"+name_), nh;
  current_ = false;

  getFootprints(pnh);  
  for (size_t i=0; i<footprints_.size(); ++i)
    footprints_pubs_[i] = pnh.advertise<geometry_msgs::PolygonStamped>(footprints_ids_[i]+"_footprint",1);
  
  dsrv_ = new dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>(pnh);
  dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>::CallbackType cb = boost::bind(&FootprintMultilayer::reconfigureCallback, this, _1, _2);
  dsrv_->setCallback(cb);

  current_ = true;
}

FootprintMultilayer::~FootprintMultilayer( void )
{
  if ( dsrv_ )
    delete dsrv_;
}

void FootprintMultilayer::updateBounds( double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y, double* max_x, double* max_y )
{
  if ( not enabled_)
    return;

  const size_t n = footprints_specs_.size();
  
  ros::Time now = ros::Time::now();

  std::vector<geometry_msgs::PolygonStamped> footprints_msgs(n);  
  for (size_t i=0; i<n; ++i) {
    footprints_msgs[i].header.frame_id = layered_costmap_->getGlobalFrameID();
    footprints_msgs[i].header.stamp = now;
    footprints_msgs[i].polygon.points.clear();

    const double cos_th = cos(robot_yaw);
    const double sin_th = sin(robot_yaw);

    footprints_msgs[i].polygon.points.reserve(n);
    footprints_[i].reserve(footprints_specs_[i].size());

    for (size_t j=0; j<footprints_specs_[i].size(); ++j)  {
      geometry_msgs::Point32 new_pt;
      new_pt.x = robot_x + (footprints_specs_[i][j].x * cos_th - footprints_specs_[i][j].y * sin_th);
      new_pt.y = robot_y + (footprints_specs_[i][j].x * sin_th + footprints_specs_[i][j].y * cos_th);
      footprints_msgs[i].polygon.points.push_back(new_pt);
      footprints_[i].emplace_back(new_pt.x,new_pt.y);

      double px = new_pt.x, py = new_pt.y;
      *min_x = std::min(px,*min_x);
      *min_y = std::min(py,*min_y);
      *max_x = std::max(px,*max_x);
      *max_y = std::max(py,*max_y);
    }

    footprints_pubs_[i].publish( footprints_msgs[i] );
  }
}

void FootprintMultilayer::updateCosts( costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j )
{
  return;
}

void FootprintMultilayer::getFootprints( const ros::NodeHandle& pnh )
{
  XmlRpc::XmlRpcValue footprints_list, ids_list;
  pnh.getParam("footprints", footprints_list);
  pnh.getParam("footprints_id", ids_list);

  ROS_ASSERT( footprints_list.getType() == XmlRpc::XmlRpcValue::TypeArray );
  ROS_ASSERT( ids_list.getType() == XmlRpc::XmlRpcValue::TypeArray );
  ROS_ASSERT( footprints_list.size() == ids_list.size() );
  
  const size_t n = footprints_list.size();

  // Looping on layers
  footprints_specs_.resize(n);
  footprints_ids_.reserve(n);
  for (size_t i=0; i<n; ++i) {
    ROS_ASSERT( footprints_list[i].getType() == XmlRpc::XmlRpcValue::TypeArray );
    ROS_ASSERT( ids_list[i].getType == XmlRpcValue::TypeString );
    footprints_ids_.emplace_back(ids_list[i]);
    // Looping on polygon
    footprints_specs_[i].reserve(footprints_list[i].size());
    for (size_t j=0; j<footprints_list[i].size(); ++i) {
      ROS_ASSERT( (footprints_list[i][j].getType() == XmlRpc::XmlRpcValue::TypeArray) and (footprints_list[i][j].size() == 2) );
      geometry_msgs::Point new_pt;
      new_pt.x = footprints_list[i][j][0];
      new_pt.y = footprints_list[i][j][1];
      footprints_specs_[i].push_back(new_pt);
    }
  }
 
  footprints_pubs_.resize(n);
  footprints_.resize(n);
}

void FootprintMultilayer::reconfigureCallback( costmap_2d::GenericPluginConfig &config, uint32_t level )
{
  enabled_ = config.enabled;
}

}  // namespace squirrel_navigation

// 
// FootprintMultilayer.cpp ends here
