// MapLayer.h --- 
// 
// Filename: MapLayer.h
// Description: Costmap layer related to static obstacles
// Author: Federico Boniardi
// Maintainer: boniardi@cs.uni-freiburg.de
// Created: Tue Feb 3 10:49:46 2015 (+0100)
// Version: 0.1.0
// Last-Updated: 
//           By: Federico Boniardi
//     Update #: 0
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
//
//   Tested on: - ROS Hydro on Ubuntu 12.04
//              - ROS Indigo on Ubuntu 14.04
//      
//      

// Code:

#ifndef MAPLAYER_H_
#define MAPLAYER_H_

#include <ros/ros.h>

#include <costmap_2d/costmap_layer.h>
#include <costmap_2d/layered_costmap.h>
#include <costmap_2d/GenericPluginConfig.h>

#include <dynamic_reconfigure/server.h>

#include <nav_msgs/OccupancyGrid.h>
#include <map_msgs/OccupancyGridUpdate.h>

#include <message_filters/subscriber.h>

#include "squirrel_navigation/CellData.h"
#include "squirrel_navigation/MapLayerPluginConfig.h"

#include <algorithm>
#include <set>

namespace squirrel_navigation {

class MapLayer : public costmap_2d::CostmapLayer
{
 public:
  MapLayer( void );
  virtual ~MapLayer( void );
  virtual void onInitialize( void );
  virtual void activate( void );
  virtual void deactivate( void );
  virtual void reset( void );

  virtual void updateBounds( double, double, double, double*, double*, double*, double* );
  virtual void updateCosts( costmap_2d::Costmap2D&, int, int, int, int );

  virtual void inflationMatchSize( void );
  virtual void mapMatchSize( void );
  
  inline bool isDiscretized( void ) {
    return true;
  }
  
  inline unsigned char computeCost(double distance) const
  {
    unsigned char cost = 0;
    if ( distance == 0 ) {
      cost = costmap_2d::LETHAL_OBSTACLE;
    } else if ( distance * resolution_ <= inscribed_radius_ ) {
      cost = costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
    } else {
      double euclidean_distance = distance * resolution_;
      double factor = exp(-1.0 * weight_ * (euclidean_distance - inscribed_radius_));
      cost = (unsigned char)((costmap_2d::INSCRIBED_INFLATED_OBSTACLE - 1) * factor);
    }
    return cost;
  }

 protected:
  virtual void onFootprintChanged( void );
  boost::shared_mutex* access_;
  
 private:
  inline double distanceLookup(int mx, int my, int src_x, int src_y)
  {
    unsigned int dx = abs(mx - src_x);
    unsigned int dy = abs(my - src_y);
    return cached_distances_[dx][dy];
  }

  inline unsigned char costLookup(int mx, int my, int src_x, int src_y)
  {
    unsigned int dx = std::abs(mx - src_x);
    unsigned int dy = std::abs(my - src_y);
    return cached_costs_[dx][dy];
  }

  void computeCaches( void );
  void deleteKernels( void );
  void inflate_area( int, int, int, int, unsigned char* );

  unsigned int cellDistance(double world_dist)
  {
    return layered_costmap_->getCostmap()->cellDistance(world_dist);
  }

  inline void enqueue( unsigned char*, unsigned int, unsigned int, unsigned int, unsigned int, unsigned int );
  
  void incomingMap( const nav_msgs::OccupancyGridConstPtr& );
  void incomingUpdate( const map_msgs::OccupancyGridUpdateConstPtr& );
  void reconfigureCB( MapLayerPluginConfig&, uint32_t );

  unsigned char interpretValue( unsigned char );

  std::string global_frame_;
  bool subscribe_to_updates_;
  bool map_received_;
  bool has_updated_data_;
  unsigned int x_,y_,width_,height_;
  bool track_unknown_space_;
  bool use_maximum_;
  bool trinary_costmap_;
  ros::Subscriber map_sub_, map_update_sub_;

  unsigned char lethal_threshold_, unknown_cost_value_;

  std::set<unsigned int> obstacles_index_;
  
  mutable boost::recursive_mutex lock_;
  dynamic_reconfigure::Server<MapLayerPluginConfig> *dsrv_;

  // Inflation related members
  double robot_link_radius_;
  double inflation_radius_, inscribed_radius_, weight_;
  unsigned int cell_inflation_radius_;
  unsigned int cached_cell_inflation_radius_;
  std::priority_queue<CellData> inflation_queue_;

  bool *seen_, need_reinflation_;

  unsigned char** cached_costs_;
  double** cached_distances_;
};
  
}  // namespace squirrel_navigation

#endif /* MAPLAYER_H_ */

// 
// MapLayer.h ends here
