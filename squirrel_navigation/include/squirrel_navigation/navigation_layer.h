// Copyright (c) 2017, Federico Boniardi and Wolfram Burgard
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
// * Redistributions of source code must retain the above copyright
//   notice, this list of conditions and the following disclaimer.
//
// * Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in
//   the documentation and/or other materials provided with the
//   distribution.
//
// * Neither the name of the University of Freiburg nor the names of
//   its contributors may be used to endorse or promote products
//   derived from this software without specific prior written
//   permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
// INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
// HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
// STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
// OF THE POSSIBILITY OF SUCH DAMAGE.

#ifndef SQUIRREL_NAVIGATION_NAVIGATION_LAYER_H_
#define SQUIRREL_NAVIGATION_NAVIGATION_LAYER_H_

#include "squirrel_navigation/NavigationLayerConfig.h"

#include <ros/service.h>

#include <dynamic_reconfigure/server.h>

#include <costmap_2d/costmap_layer.h>

#include <costmap_2d_strip/obstacle_layer.h>
#include <costmap_2d_strip/static_layer.h>
#include <costmap_2d_strip/voxel_layer.h>

#include <squirrel_navigation_msgs/ClearCostmapRegion.h>
#include <squirrel_navigation_msgs/GetObstaclesMap.h>
#include <squirrel_navigation_msgs/GetPathClearance.h>

#include <memory>
#include <mutex>

namespace squirrel_navigation {

class NavigationLayer : public costmap_2d::CostmapLayer {
 public:
  class Params {
   public:
    static Params defaultParams();

    bool use_kinect, use_laser_scan;
  };

 public:
  NavigationLayer() : params_(Params::defaultParams()) {}
  NavigationLayer(const Params& params) : params_(params) {}
  virtual ~NavigationLayer() {}

  // Initialization function.
  void onInitialize() override;

  // Update the costmap.
  void updateBounds(
      double robot_x, double robot_y, double robot_yaw, double* min_x,
      double* min_y, double* max_x, double* max_y) override;
  void updateCosts(
      costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i,
      int max_j) override;

  // On/Off/Reset iterface.
  void activate();
  void deactivate();
  void reset();

  // Whether the map is discrete or not.
  inline bool isDiscretized() { return true; }

  // Update guard.
  inline std::mutex& mutex() const { return update_mtx_; }

  // Parameters read/write.
  inline const Params& params() const { return params_; }
  inline void setParams(const Params& params) { params_ = params; }
  inline Params& params() { return params_; }

 private:
  // Update the parameters and service callbacks
  void reconfigureCallback(NavigationLayerConfig& config, uint32_t level);
  bool clearCostmapRegionCallback(
      squirrel_navigation_msgs::ClearCostmapRegion::Request& req,
      squirrel_navigation_msgs::ClearCostmapRegion::Response& res);
  bool getObstaclesMapCallback(
      squirrel_navigation_msgs::GetObstaclesMap::Request& req,
      squirrel_navigation_msgs::GetObstaclesMap::Response& res);
  bool getPathClearanceCallback(
      squirrel_navigation_msgs::GetPathClearance::Request& req,
      squirrel_navigation_msgs::GetPathClearance::Response& res);

  // Compute the obstacle map.
  size_t getObstaclesMap(
      std::vector<bool>* obstacles_indicator,
      std::vector<geometry_msgs::Point32>* obstacles_positions) const;

  // Costs update and utilities.
  void resetMasterCostmapLocally(
      costmap_2d::Costmap2D& master_grid, unsigned int stride, int min_i,
      int min_j, int max_i, int max_j);
  void mergeCostmaps(
      unsigned char* laser_costmap, unsigned char* kinect_costmap,
      unsigned char* static_costmap, unsigned int stride, int min_i, int min_j,
      int max_i, int max_j);

 private:
  Params params_;
  std::unique_ptr<dynamic_reconfigure::Server<NavigationLayerConfig>> dsrv_;

  squirrel_navigation::ObstacleLayer laser_layer_;
  squirrel_navigation::VoxelLayer kinect_layer_;
  squirrel_navigation::StaticLayer static_layer_;

  ros::ServiceServer clear_costmap_srv_, obstacles_map_srv_,
      path_clearance_srv_;

  mutable std::mutex update_mtx_;
};

}  // namespace squirrel_navigation

#endif /* SQUIRREL_NAVIGATION_NAVIGATION_LAYER_H_ */
