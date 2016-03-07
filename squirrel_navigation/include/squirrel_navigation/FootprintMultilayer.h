// FootprintMultilayer.h --- 
// 
// Filename: FootprintMultilayer.h
// Description: 
// Author: Federico Boniardi
// Maintainer: 
// Created: Mon Mar  7 12:42:57 2016 (+0100)
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

#ifndef SQUIRREL_NAVIGATION_FOOTPRINTMULTILAYER_H_
#define SQUIRREL_NAVIGATION_FOOTPRINTMULTILAYER_H_

#include <ros/ros.h>

#include <costmap_2d/layer.h>
#include <costmap_2d/layered_costmap.h>
#include <costmap_2d/costmap_math.h>
#include <costmap_2d/footprint.h>
#include <costmap_2d/GenericPluginConfig.h>

#include <dynamic_reconfigure/server.h>

#include <pluginlib/class_list_macros.h>

#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/PolygonStamped.h>
#include <nav_msgs/OccupancyGrid.h>

#include <string>
#include <sstream>

#include "squirrel_navigation/Common.h"

namespace squirrel_navigation {

class FootprintMultilayer : public costmap_2d::Layer
{
 public:
  virtual void onInitialize( void );
  virtual ~FootprintMultilayer( void );
  virtual void updateBounds( double, double, double, double*, double*, double*, double* );
  virtual void updateCosts( costmap_2d::Costmap2D&, int, int, int, int );

  inline bool insideFootprint( size_t layer, double px, double py )
  {
    return isInsideFootprint(footprints_[layer],CGAL_Point2D(px,py),ckern_);
  }
  
 private:
  std::vector<ros::Publisher> footprints_pubs_;
  std::vector<std::string> footprints_ids_;
  std::vector< std::vector<geometry_msgs::Point> > footprints_specs_;

  std::vector<Footprint> footprints_;
  CGAL_Kernel ckern_;
  
  dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>* dsrv_;
  
  void getFootprints( const ros::NodeHandle& );
  void reconfigureCallback( costmap_2d::GenericPluginConfig &config, uint32_t level );
};

}  // namespace squirrel_navigation

#endif /* SQUIRREL_NAVIGATION_FOOTPRINTMULTILAYER_H_ */

// 
// FootprintMultilayer.h ends here
