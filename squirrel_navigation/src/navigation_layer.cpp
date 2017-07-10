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

#include "squirrel_navigation/navigation_layer.h"

#include <pluginlib/class_list_macros.h>

#include <vector>

PLUGINLIB_EXPORT_CLASS(squirrel_navigation::NavigationLayer, costmap_2d::Layer);

namespace squirrel_navigation {

void NavigationLayer::onInitialize() {
  // Initialize paramter server
  ros::NodeHandle pnh("~/" + name_);
  dsrv_.reset(new dynamic_reconfigure::Server<NavigationLayerConfig>(pnh));
  dsrv_->setCallback(
      boost::bind(&NavigationLayer::reconfigureCallback, this, _1, _2));
  // Initialize the layers.
  laser_layer_.initialize(layered_costmap_, name_ + "/laser_layer", tf_);
  kinect_layer_.initialize(layered_costmap_, name_ + "/kinect_layer", tf_);
  static_layer_.initialize(layered_costmap_, name_ + "/static_layer", tf_);
  NavigationLayer::matchSize();
  current_ = true;
  enabled_ = true;
}

void NavigationLayer::updateBounds(
    double robot_x, double robot_y, double robot_yaw, double* min_x,
    double* min_y, double* max_x, double* max_y) {
  // Get bounds for laser.
  double laser_min_x, laser_min_y, laser_max_x, laser_max_y;
  laser_layer_.updateBounds(
      robot_x, robot_y, robot_yaw, &laser_min_x, &laser_min_y, &laser_max_x,
      &laser_max_y);
  current_ = current_ && laser_layer_.currentStatus();
  // Get bounds for kinect.
  double kinect_min_x, kinect_min_y, kinect_max_x, kinect_max_y;
  kinect_layer_.updateBounds(
      robot_x, robot_y, robot_yaw, &kinect_min_x, &kinect_min_y, &kinect_max_x,
      &kinect_max_y);
  current_ = current_ && laser_layer_.currentStatus();
  // Get the boundaries.
  *min_x = std::min(laser_min_x, kinect_min_x);
  *min_y = std::min(laser_min_y, kinect_min_y);
  *max_x = std::max(laser_max_x, kinect_max_x);
  *max_y = std::max(laser_max_y, kinect_max_y);
}

void NavigationLayer::updateCosts(
    costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i,
    int max_j) {
  unsigned char* laser_costmap  = laser_layer_.costmap();
  unsigned char* kinect_costmap = kinect_layer_.costmap();
  unsigned char* static_costmap = static_layer_.costmap();
  // Merge the two costmaps.
  const unsigned int stride = master_grid.getSizeInCellsX();
  mergeCostmaps(
      laser_costmap, kinect_costmap, static_costmap, stride, min_i, min_j,
      max_i, max_j);
  resetMasterCostmapLocally(master_grid, stride, min_i, min_j, max_i, max_j);
  updateWithOverwrite(master_grid, min_i, min_j, max_i, max_j);
}

void NavigationLayer::activate() {
  laser_layer_.activate();
  kinect_layer_.activate();
  static_layer_.activate();
  current_ = true;
}

void NavigationLayer::deactivate() {
  laser_layer_.deactivate();
  kinect_layer_.deactivate();
  static_layer_.deactivate();
}

void NavigationLayer::reset() {
  deactivate();
  laser_layer_.reset();
  kinect_layer_.reset();
  static_layer_.reset();
  activate();
  current_ = true;
}

void NavigationLayer::reconfigureCallback(
    NavigationLayerConfig& config, uint32_t level) {
  kinect_layer_.enabled() = config.use_kinect;
  laser_layer_.enabled()  = config.use_laser_scan;
}

void NavigationLayer::resetMasterCostmapLocally(
    costmap_2d::Costmap2D& master_grid, unsigned int stride, int min_i,
    int min_j, int max_i, int max_j) {
  unsigned char* master_costmap = master_grid.getCharMap();
  for (int j = min_j; j < max_j; ++j) {
    unsigned int it = stride * j + min_i;
    for (int i = min_i; i < max_i; ++i) {
      master_costmap[it] = costmap_2d::FREE_SPACE;
      it++;
    }
  }
}

void NavigationLayer::mergeCostmaps(
    unsigned char* laser_costmap, unsigned char* kinect_costmap,
    unsigned char* static_costmap, unsigned int stride, int min_i, int min_j,
    int max_i, int max_j) {
  const std::set<unsigned int>& floor_indices = kinect_layer_.floorIndices();
  for (const auto index : floor_indices)
    costmap_[index] = laser_costmap[index] = costmap_2d::FREE_SPACE;
  for (int j = min_j; j < max_j; ++j) {
    unsigned int it = stride * j + min_i;
    for (int i = min_i; i < max_i; ++i) {
      costmap_[it] = std::max(laser_costmap[it], kinect_costmap[it]);
      costmap_[it] = std::max(static_costmap[it], costmap_[it]);
      it++;
    }
  }
}

NavigationLayer::Params NavigationLayer::Params::defaultParams() {
  Params params;
  params.use_kinect     = true;
  params.use_laser_scan = true;
  return params;
}

}  // namespace squirrel_navigation
