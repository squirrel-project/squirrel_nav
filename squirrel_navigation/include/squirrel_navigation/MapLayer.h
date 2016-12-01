// MapLayer.h ---
//
// Filename: MapLayer.h
// Description: Costmap layer related to static obstacles
// Author: Federico Boniardi
// Maintainer: boniardi@cs.uni-freiburg.de
// Created: Tue Feb 3 10:49:46 2015 (+0100)
// Version: 0.1.0
// Last-Updated: Tue Feb 3 15:34:16 2015 (+0100)
//           By: Federico Boniardi
//     Update #: 1
// URL:
// Keywords:
// Compatibility:
//   ROS Hydro, ROS Indigo
//

// Commentary:
//   The code therein is an integration of costmap_2d::InflationLayer into
//   costmap_2d::StaticLayer. Both source codes are distributed by the authors
//   under BSD license, that is below reported
//
//     /*********************************************************************
//      *
//      * Software License Agreement (BSD License)
//      *
//      *  Copyright (c) 2008, 2013, Willow Garage, Inc.
//      *  All rights reserved.
//      *
//      *  Redistribution and use in source and binary forms, with or without
//      *  modification, are permitted provided that the following conditions
//      *  are met:
//      *
//      *   * Redistributions of source code must retain the above copyright
//      *     notice, this list of conditions and the following disclaimer.
//      *   * Redistributions in binary form must reproduce the above
//      *     copyright notice, this list of conditions and the following
//      *     disclaimer in the documentation and/or other materials provided
//      *     with the distribution.
//      *   * Neither the name of Willow Garage, Inc. nor the names of its
//      *     contributors may be used to endorse or promote products derived
//      *     from this software without specific prior written permission.
//      *
//      *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
//      *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
//      *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
//      *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
//      *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
//      *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
//      *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
//      *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
//      *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
//      *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
//      *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
//      *  POSSIBILITY OF SUCH DAMAGE.
//      *
//      * Author: Eitan Marder-Eppstein
//      *         David V. Lu!!
//      *********************************************************************/

#ifndef SQUIRREL_NAVIGATION_MAPLAYER_H_
#define SQUIRREL_NAVIGATION_MAPLAYER_H_

#include <ros/ros.h>

#include <costmap_2d/GenericPluginConfig.h>
#include <costmap_2d/costmap_layer.h>
#include <costmap_2d/costmap_math.h>
#include <costmap_2d/layered_costmap.h>
#include <costmap_2d/static_layer.h>

#include <dynamic_reconfigure/server.h>

#include <map_msgs/OccupancyGridUpdate.h>
#include <nav_msgs/OccupancyGrid.h>

#include <message_filters/subscriber.h>

#include <pluginlib/class_list_macros.h>

#include <algorithm>
#include <cmath>
#include <set>

#include "squirrel_navigation/CellData.h"
#include "squirrel_navigation/InflatedLayer.h"
#include "squirrel_navigation/MapLayerPluginConfig.h"

namespace squirrel_navigation {

class MapLayer : public InflatedLayer {
 public:
  MapLayer(void);
  virtual ~MapLayer(void);

  virtual void onInitialize(void);
  virtual void activate(void);
  virtual void deactivate(void);
  virtual void reset(void);

  virtual void updateBounds(
      double, double, double, double*, double*, double*, double*);
  virtual void updateCosts(costmap_2d::Costmap2D&, int, int, int, int);
  virtual void matchSize(void);
  inline bool isDiscretized(void) { return true; };

 private:
  std::string global_frame_;
  bool subscribe_to_updates_;
  bool map_received_;
  bool has_updated_data_;
  unsigned int x_, y_, width_, height_;
  bool track_unknown_space_;
  bool use_maximum_;
  bool trinary_costmap_;
  ros::Subscriber map_sub_, map_update_sub_;

  unsigned char lethal_threshold_, unknown_cost_value_;

  mutable boost::recursive_mutex lock_;
  dynamic_reconfigure::Server<MapLayerPluginConfig>* dsrv_;

  void mapCallback_(const nav_msgs::OccupancyGridConstPtr&);
  void mapUpdateCallback_(const map_msgs::OccupancyGridUpdateConstPtr&);
  void reconfigureCallback_(MapLayerPluginConfig&, uint32_t);
  unsigned char interpretValue_(unsigned char);
};

}  // namespace squirrel_navigation

#endif  // SQUIRREL_NAVIGATION_MAPLAYER_H_
