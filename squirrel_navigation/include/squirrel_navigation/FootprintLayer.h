// FootprintLayer.h --- 
// 
// Filename: FootprintLayer.h
// Description: 
// Author: Federico Boniardi
// Maintainer: boniardi@informatik.uni-freiburg.de
// Created: Fri Nov 20 13:57:41 2015 (+0100)
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

#ifndef SQUIRREL_NAVIGATION_FOOTPRINT_LAYER_H_
#define SQUIRREL_NAVIGATION_FOOTPRINT_LAYER_H_

#include <ros/ros.h>

#include <costmap_2d/layer.h>
#include <costmap_2d/layered_costmap.h>
#include <costmap_2d/costmap_math.h>
#include <costmap_2d/GenericPluginConfig.h>

#include <dynamic_reconfigure/server.h>

#include <nav_msgs/OccupancyGrid.h>

#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/PolygonStamped.h>

namespace squirrel_navigation {

class FootprintLayer : public costmap_2d::Layer
{
public:
  virtual void onInitialize( void );
  virtual ~FootprintLayer( void );
  virtual void updateBounds( double, double, double, double*, double*, double*, double* );
  virtual void updateCosts( costmap_2d::Costmap2D&, int, int, int, int );

private:
  ros::Publisher footprint_pub_;
  geometry_msgs::PolygonStamped footprint_; ///< Storage for polygon being published.
  dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>* dsrv_;

  void publishFootprint( void );
  void reconfigureCB( costmap_2d::GenericPluginConfig &config, uint32_t level );
};

}  // namespace squirrel_navigation

#endif // SQUIRREL_NAVIGATION_FOOTPRINT_LAYER_H_

// 
// FootprintLayer.h ends here
