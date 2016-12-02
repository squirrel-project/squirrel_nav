// InflatedLayer.h ---
//
// Filename: InflatedLayer.h
// Description: Obstacle inflator to simulate robot's configuration space
// Author: Federico Boniardi
// Maintainer: boniardi@cs.uni-freiburg.de
// Created: Tue Feb  3 13:46:17 2015 (+0100)
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

#ifndef SQUIRREL_NAVIGATION_INFLATEDLAYER_H_
#define SQUIRREL_NAVIGATION_INFLATEDLAYER_H_

#include <ros/ros.h>

#include <costmap_2d/costmap_layer.h>
#include <costmap_2d/costmap_math.h>
#include <costmap_2d/footprint.h>
#include <costmap_2d/layered_costmap.h>

#include <pluginlib/class_list_macros.h>

#include <cmath>
#include <queue>
#include <set>

#include "squirrel_navigation/CellData.h"

namespace squirrel_navigation {

class InflatedLayer : public costmap_2d::CostmapLayer {
 public:
  InflatedLayer(void);
  virtual ~InflatedLayer(void);

  virtual void updateInflatedBounds(
      double, double, double, double*, double*, double*, double*);
  virtual void updateInflatedCosts(costmap_2d::Costmap2D&, int, int, int, int);
  virtual void matchInflatedSize(void);
  unsigned char computeCost(double) const;

 protected:
  std::set<unsigned int> obstacles_;

  double inflation_radius_, inscribed_radius_, weight_;
  bool *seen_, need_reinflation_;

  unsigned int cell_inflation_radius_;
  unsigned int cached_cell_inflation_radius_;

  boost::shared_mutex* access_;

  virtual void onFootprintChanged(void);
  virtual void computeCaches(void);
  virtual unsigned int cellDistance(double world_dist);

 private:
  std::priority_queue<CellData> inflation_queue_;

  unsigned char** cached_costs_;
  double** cached_distances_;

  void deleteKernels_(void);
  double distanceLookup_(int, int, int, int);
  unsigned char costLookup_(int, int, int, int);
  void enqueue_(
      unsigned char*, unsigned int, unsigned int, unsigned int, unsigned int,
      unsigned int);
};

}  // namespace squirrel_navigation

#endif  // SQUIRREL_NAVIGATION_INFLATEDLAYER_H_
