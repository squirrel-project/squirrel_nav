// InflationLayer.h ---
//
// Filename: InflationLayer.h
// Description:
// Author: Federico Boniardi
// Maintainer:
// Created: Mi Jan 20 15:09:36 2016 (+0100)
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
// /*********************************************************************
//  *
//  * Software License Agreement (BSD License)
//  *
//  *  Copyright (c) 2008, 2013, Willow Garage, Inc.
//  *  All rights reserved.
//  *
//  *  Redistribution and use in source and binary forms, with or without
//  *  modification, are permitted provided that the following conditions
//  *  are met:
//  *
//  *   * Redistributions of source code must retain the above copyright
//  *     notice, this list of conditions and the following disclaimer.
//  *   * Redistributions in binary form must reproduce the above
//  *     copyright notice, this list of conditions and the following
//  *     disclaimer in the documentation and/or other materials provided
//  *     with the distribution.
//  *   * Neither the name of Willow Garage, Inc. nor the names of its
//  *     contributors may be used to endorse or promote products derived
//  *     from this software without specific prior written permission.
//  *
//  *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
//  *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
//  *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
//  *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
//  *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
//  *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
//  *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
//  *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
//  *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
//  *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
//  *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
//  *  POSSIBILITY OF SUCH DAMAGE.
//  *
//  * Author: Eitan Marder-Eppstein
//  *         David V. Lu!!
//  *********************************************************************/

#ifndef SQUIRREL_NAVIGATION_INFLATION_LAYER_H_
#define SQUIRREL_NAVIGATION_INFLATION_LAYER_H_

#include "squirrel_navigation/CellData.h"
#include "squirrel_navigation/InflationLayerPluginConfig.h"

#include <ros/ros.h>

#include <costmap_2d/costmap_math.h>
#include <costmap_2d/footprint.h>
#include <costmap_2d/layer.h>
#include <costmap_2d/layered_costmap.h>

#include <dynamic_reconfigure/server.h>

#include <boost/thread.hpp>

#include <pluginlib/class_list_macros.h>

#include <algorithm>
#include <boost/thread.hpp>
#include <queue>

namespace squirrel_navigation {

class InflationLayer : public costmap_2d::Layer {
 public:
  InflationLayer(void);
  virtual ~InflationLayer(void);

  virtual void onInitialize(void);
  virtual void updateBounds(
      double robot_x, double robot_y, double robot_yaw, double* min_x,
      double* min_y, double* max_x, double* max_y);
  virtual void updateCosts(
      costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i,
      int max_j);
  virtual void matchSize(void);
  virtual void reset(void) { onInitialize(); };
  void setInflationParameters(
      double inscribed_radius, double inflation_radius,
      double cost_scaling_factor);

  virtual bool isDiscretized(void) { return true; };

  inline unsigned char computeCost(double distance) const {
    unsigned char cost = 0;
    if (distance == 0)
      cost = costmap_2d::LETHAL_OBSTACLE;
    else if (distance * resolution_ <= inscribed_radius_)
      cost = costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
    else {
      // make sure cost falls off by Euclidean distance
      double euclidean_distance = distance * resolution_;
      double factor =
          exp(-1.0 * weight_ * (euclidean_distance - inscribed_radius_));
      cost =
          (unsigned char)((costmap_2d::INSCRIBED_INFLATED_OBSTACLE - 1) * factor);
    }
    return cost;
  }

 protected:
  virtual void onFootprintChanged(void);
  boost::recursive_mutex* inflation_access_;

 private:
  double inflation_radius_, inscribed_radius_, weight_;
  unsigned int cell_inflation_radius_;
  unsigned int cached_cell_inflation_radius_;
  std::priority_queue<CellData> inflation_queue_;

  double resolution_;

  bool inscribed_radius_from_footprint_;

  bool* seen_;
  int seen_size_;

  unsigned char** cached_costs_;
  double** cached_distances_;
  double last_min_x_, last_min_y_, last_max_x_, last_max_y_;

  dynamic_reconfigure::Server<InflationLayerPluginConfig>* dsrv_;

  bool need_reinflation_;  ///< Indicates that the entire costmap should be
                           ///reinflated next time around.

  void reconfigureCB(InflationLayerPluginConfig& config, uint32_t level);
  void computeCaches(void);
  void deleteKernels(void);
  void inflate_area(
      int min_i, int min_j, int max_i, int max_j, unsigned char* master_grid);
  inline void enqueue(
      unsigned char* grid, unsigned int index, unsigned int mx, unsigned int my,
      unsigned int src_x, unsigned int src_y);

  inline double distanceLookup(int mx, int my, int src_x, int src_y) {
    unsigned int dx = abs(mx - src_x);
    unsigned int dy = abs(my - src_y);
    return cached_distances_[dx][dy];
  };

  inline unsigned char costLookup(int mx, int my, int src_x, int src_y) {
    unsigned int dx = abs(mx - src_x);
    unsigned int dy = abs(my - src_y);
    return cached_costs_[dx][dy];
  };

  unsigned int cellDistance(double world_dist) {
    return layered_costmap_->getCostmap()->cellDistance(world_dist);
  };
};

}  // namespace costmap_2d

#endif  // SQUIRREL_NAVIGATION_INFLATION_LAYER_H_
