// MultiInflatedLayer.cpp ---
//
// Filename: MultiInflatedLayer.cpp
// Description: Obstacle inflator to simulate robot's 3D configuration space
// Author: Federico Boniardi
// Maintainer: boniardi@cs.uni-freiburg.de
// Created: Wed Feb 4 10:04:34 2015 (+0100)
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

#include "squirrel_navigation/MultiInflatedLayer.h"

#include <costmap_2d/costmap_math.h>
#include <costmap_2d/footprint.h>

#include <algorithm>
#include <cmath>

namespace squirrel_navigation {

MultiInflatedLayer::MultiInflatedLayer(void)
    : max_cell_inflation_radius_(0),
      max_cached_cell_inflation_radius_(0),
      cached_distances_(NULL),
      cached_costs_(NULL) {
  access_ = new boost::shared_mutex();
}

MultiInflatedLayer::~MultiInflatedLayer(void) {
  deleteKernels();

  if (cached_costs_ != NULL) {
    delete[] cached_costs_;
  }

  if (cached_distances_ != NULL) {
    delete[] cached_distances_;
  }
}

void MultiInflatedLayer::malloc_layers(void) {
  if (cached_costs_ != NULL) {
    delete[] cached_costs_;
  }
  cached_costs_ = new unsigned char**[num_layers_];

  if (cached_distances_ != NULL) {
    delete[] cached_distances_;
  }
  cached_distances_ = new double**[num_layers_];

  weights_.resize(num_layers_, 0.0);
  cell_inflation_radii_.resize(num_layers_, 0.0);
  cached_cell_inflation_radii_.resize(num_layers_, 0.0);
}

void MultiInflatedLayer::matchInflatedSize(void) {
  boost::unique_lock<boost::shared_mutex> lock(*access_);
  costmap_2d::Costmap2D* costmap = layered_costmap_->getCostmap();
  resolution_                    = costmap->getResolution();

  for (unsigned int l = 0; l < num_layers_; ++l) {
    cell_inflation_radii_[l] = cellDistance(inflation_radii_[l]);
  }
  max_cell_inflation_radius_ = *std::max_element(
      cell_inflation_radii_.begin(), cell_inflation_radii_.end());

  computeCaches();

  unsigned int size_x = costmap->getSizeInCellsX(),
               size_y = costmap->getSizeInCellsY();
  if (seen_) {
    delete seen_;
  }

  seen_ = new bool[size_x * size_y];
}

void MultiInflatedLayer::updateInflatedBounds(
    double robot_x, double robot_y, double robot_yaw, double* min_x,
    double* min_y, double* max_x, double* max_y) {
  if (need_reinflation_) {
    *min_x            = -std::numeric_limits<float>::max();
    *min_y            = -std::numeric_limits<float>::max();
    *max_x            = std::numeric_limits<float>::max();
    *max_y            = std::numeric_limits<float>::max();
    need_reinflation_ = false;
  }
}

void MultiInflatedLayer::onFootprintChanged(void) {
  for (unsigned int l = 0; l < num_layers_; ++l) {
    cell_inflation_radii_[l] = cellDistance(inflation_radii_[l]);
  }
  max_cell_inflation_radius_ = *std::max_element(
      cell_inflation_radii_.begin(), cell_inflation_radii_.end());
  computeCaches();
  need_reinflation_ = true;
}

void MultiInflatedLayer::updateInflatedCosts(
    costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i,
    int max_j) {
  boost::unique_lock<boost::shared_mutex> lock(*access_);

  ROS_ASSERT_MSG(
      inflation_queue_.empty(),
      "The inflation queue must be empty at the beginning of inflation");

  unsigned char* master_array = master_grid.getCharMap();
  unsigned int size_x         = master_grid.getSizeInCellsX(),
               size_y         = master_grid.getSizeInCellsY();

  memset(seen_, false, size_x * size_y * sizeof(bool));

  min_i -= max_cell_inflation_radius_;
  min_j -= max_cell_inflation_radius_;
  max_i += max_cell_inflation_radius_;
  max_j += max_cell_inflation_radius_;

  min_i = std::max(0, min_i);
  min_j = std::max(0, min_j);
  max_i = std::min(int(size_x), max_i);
  max_j = std::min(int(size_y), max_j);

  for (int j = min_j; j < max_j; j++) {
    for (int i = min_i; i < max_i; i++) {
      int index           = master_grid.getIndex(i, j);
      unsigned char cost  = master_array[index];
      bool to_be_inflated = obstacles_.find(index) != obstacles_.end();
      if (cost == costmap_2d::LETHAL_OBSTACLE && to_be_inflated) {
        enqueue(master_array, index, i, j, i, j);
      }
    }
  }

  while (!inflation_queue_.empty()) {
    const CellData& current_cell = inflation_queue_.top();

    unsigned int index = current_cell.index_;
    unsigned int mx    = current_cell.x_;
    unsigned int my    = current_cell.y_;
    unsigned int sx    = current_cell.src_x_;
    unsigned int sy    = current_cell.src_y_;

    inflation_queue_.pop();

    if (mx > 0) {
      enqueue(master_array, index - 1, mx - 1, my, sx, sy);
    }
    if (my > 0) {
      enqueue(master_array, index - size_x, mx, my - 1, sx, sy);
    }
    if (mx < size_x - 1) {
      enqueue(master_array, index + 1, mx + 1, my, sx, sy);
    }
    if (my < size_y - 1) {
      enqueue(master_array, index + size_x, mx, my + 1, sx, sy);
    }
  }
}

inline void MultiInflatedLayer::enqueue(
    unsigned char* grid, unsigned int index, unsigned int mx, unsigned int my,
    unsigned int src_x, unsigned int src_y) {
  unsigned int src_index = getIndex(src_x, src_y);
  if (!seen_[index]) {
    double distance =
        distanceLookup(mx, my, src_x, src_y, obstacles_[src_index]);

    if (distance > cell_inflation_radii_[obstacles_[src_index]]) {
      return;
    }

    unsigned char cost =
        costLookup(mx, my, src_x, src_y, obstacles_[src_index]);
    unsigned char old_cost = grid[index];

    if (old_cost == costmap_2d::NO_INFORMATION &&
        cost >= costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
      grid[index] = cost;
    } else {
      grid[index] = std::max(old_cost, cost);
    }
    seen_[index] = true;
    CellData data(distance, index, mx, my, src_x, src_y);
    inflation_queue_.push(data);
  }
}

void MultiInflatedLayer::computeCaches(void) {
  for (unsigned int l = 0; l < num_layers_; ++l) {
    if (cell_inflation_radii_[l] == 0) {
      continue;
    }

    if (cell_inflation_radii_[l] != cached_cell_inflation_radii_[l]) {
      if (cached_cell_inflation_radii_[l] > 0) {
        deleteKernels();
      }

      cached_costs_[l]     = new unsigned char*[cell_inflation_radii_[l] + 2];
      cached_distances_[l] = new double*[cell_inflation_radii_[l] + 2];

      for (unsigned int i = 0; i <= cell_inflation_radii_[l] + 1; ++i) {
        cached_costs_[l][i] = new unsigned char[cell_inflation_radii_[l] + 2];
        cached_distances_[l][i] = new double[cell_inflation_radii_[l] + 2];
        for (unsigned int j = 0; j <= cell_inflation_radii_[l] + 1; ++j) {
          cached_distances_[l][i][j] = hypot(i, j);
        }
      }

      cached_cell_inflation_radii_[l] = cell_inflation_radii_[l];
    }

    for (unsigned int i = 0; i <= cell_inflation_radii_[l] + 1; ++i) {
      for (unsigned int j = 0; j <= cell_inflation_radii_[l] + 1; ++j) {
        cached_costs_[l][i][j] = computeCost(cached_distances_[l][i][j], l);
      }
    }
  }
}

void MultiInflatedLayer::deleteKernels(void) {
  if (cached_distances_ != NULL) {
    for (unsigned int l = 0; l < num_layers_; ++l) {
      if (cached_distances_[l] != NULL) {
        for (unsigned int i = 0; i <= cached_cell_inflation_radii_[l] + 1;
             ++i) {
          delete[] cached_distances_[l][i];
        }
        delete[] cached_distances_[l];
      }
    }
  }

  if (cached_costs_ != NULL) {
    for (unsigned int l = 0; l < num_layers_; ++l) {
      if (cached_costs_[l] != NULL) {
        for (unsigned int i = 0; i <= cached_cell_inflation_radii_[l] + 1;
             ++i) {
          delete[] cached_costs_[l][i];
        }
        delete[] cached_costs_[l];
      }
    }
  }
}

}  // namespace squirrel_navigation
