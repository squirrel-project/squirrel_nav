// MultiInflatedLayer.h ---
//
// Filename: MultiInflatedLayer.h
// Description: Obstacle inflator to simulate robot's 3D configuration space
// Author: Federico Boniardi
// Maintainer: boniardi@cs.uni-freiburg.de
// Created: Wed Feb 4 10:05:31 2015 (+0100)
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
//   The code therein is based on costmap_2d::InflationLayer,
//   distributed by its authors under BSD license which is below reported
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

#ifndef SQUIRREL_NAVIGATION_MULTIINFLATEDLAYER_H_
#define SQUIRREL_NAVIGATION_MULTIINFLATEDLAYER_H_

#include "squirrel_navigation/CellData.h"

#include <ros/ros.h>

#include <costmap_2d/layer.h>
#include <costmap_2d/layered_costmap.h>
#include <costmap_2d/obstacle_layer.h>

#include <cmath>
#include <map>
#include <queue>

namespace squirrel_navigation {

class MultiInflatedLayer : public costmap_2d::ObstacleLayer {
 public:
  MultiInflatedLayer(void);
  virtual ~MultiInflatedLayer(void);

  virtual void malloc_layers(void);

  virtual void updateInflatedBounds(
      double, double, double, double*, double*, double*, double*);
  virtual void updateInflatedCosts(
      costmap_2d::Costmap2D& master_grid, int, int, int, int);
  virtual void matchInflatedSize(void);

  inline unsigned char computeCost(double distance, int l) const {
    unsigned char cost = 0;
    if (distance == 0) {
      cost = costmap_2d::LETHAL_OBSTACLE;
    } else if (distance * resolution_ <= inscribed_radii_[l]) {
      cost = costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
    } else {
      double euclidean_distance = distance * resolution_;
      double factor =
          exp(-1.0 * weights_[l] * (euclidean_distance - inscribed_radii_[l]));
      cost =
          (unsigned char)((costmap_2d::INSCRIBED_INFLATED_OBSTACLE - 1) * factor);
    }
    return cost;
  }

 protected:
  virtual void onFootprintChanged(void);
  virtual void computeCaches(void);

  virtual unsigned int cellDistance(double world_dist) {
    return layered_costmap_->getCostmap()->cellDistance(world_dist);
  }

  std::map<unsigned int, unsigned int> obstacles_;

  unsigned int num_layers_;
  std::vector<double> inflation_radii_, inscribed_radii_, weights_;

  bool *seen_, need_reinflation_;
  std::vector<unsigned int> cell_inflation_radii_;
  std::vector<unsigned int> cached_cell_inflation_radii_;

  unsigned int max_cell_inflation_radius_;
  unsigned int max_cached_cell_inflation_radius_;

  boost::shared_mutex* access_;

 private:
  inline double distanceLookup(int mx, int my, int src_x, int src_y, int l) {
    unsigned int dx = std::abs(mx - src_x);
    unsigned int dy = std::abs(my - src_y);
    return cached_distances_[l][dx][dy];
  }

  inline unsigned char costLookup(int mx, int my, int src_x, int src_y, int l) {
    unsigned int dx = abs(mx - src_x);
    unsigned int dy = abs(my - src_y);
    return cached_costs_[l][dx][dy];
  }

  void deleteKernels(void);
  void inflate_area(int, int, int, int, unsigned char*);

  inline void enqueue(
      unsigned char*, unsigned int, unsigned int, unsigned int, unsigned int,
      unsigned int);

  std::priority_queue<CellData> inflation_queue_;

  unsigned char*** cached_costs_;
  double*** cached_distances_;
};

}  // namespace squirrel_navigation

#endif  // SQUIRREL_NAVIGATION_MULTIINFLATEDLAYER_H_
