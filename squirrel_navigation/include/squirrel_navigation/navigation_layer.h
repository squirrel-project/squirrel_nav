// The MIT License (MIT)
//
// Copyright (c) 2017 Federico Boniardi and Wolfram Burgard
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#ifndef SQUIRREL_NAVIGATION_NAVIGATION_LAYER_H_
#define SQUIRREL_NAVIGATION_NAVIGATION_LAYER_H_

#include <ros/ros.h>

#include <dynamic_reconfigure/server.h>

#include <costmap_2d/costmap_layer.h>

#include <std_msgs/Bool.h>
#include <squirrel_navigation/NavigationLayer.cfg>

#include <costmap_2d_strip/obstacle_layer.h>
#include <costmap_2d_strip/static_layer.h>
#include <costmap_2d_strip/voxel_layer.h>

#include <memory>

namespace squirrel_navigation {

class NavigationLayer : public costmap_2d::CostmapLayer {
 public:
  class Params{
    static Params defaultParams();

    bool use_kinect, use_laser_scan; 
  };

 public:
  NavigationLayer() : params_(Params::defaultParams()) {}
  NavigationLayer(const Params& params) : params_(params) {}
  virtual ~NavigationLayer() {}

  // Initialization function.
  void onInitialize();

  // Update the costmap.
  void updateBounds(
      double robot_x, double robot_y, double robot_yaw, double* min_x,
      double* min_y, double* max_x, double* max_y);
  void updateCosts(
      costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i,
      int max_j);

  // On/Off/Reset iterface.
  void activate();
  void deactivate();
  void reset();

  // Whether the map is discrete or not.
  bool isDiscretized() { return true; }

  // Parameters read/write.
  inline const Params& params() const { return params_ }
  inline void setParams(const Params& params) { params_ = params; }
  inline Params& params() { return params_; }

 private:
  // Update the parameters.
  void reconfigureCallback(NavigationLayer& config, uint32_t level);

  // Costs update.
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
};

}  // namespace squirrel_navigation

#endif /* SQUIRREL_NAVIGATION_NAVIGATION_LAYER_H_ */
