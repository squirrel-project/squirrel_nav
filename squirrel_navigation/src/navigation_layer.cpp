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

#include "squirrel_navigation/navigation_layer.h"

#include <pluginlib/class_list_macros.h>

#include <thread>
#include <vector>

PLUGINLIB_EXPORT_CLASS(squirrel_navigation::NavigationLayer, costmap_2d::Layer);

namespace squirrel_navigation {

void NavigationLayer::onInitialize() {
  // Initialize paramter server
  ros::NodeHandle pnh("~/" + name_);
  dsrv_.reset(new dynamic_reconfigure::Server<NavigationLayerConfig>(pnh));
  dsrv_->setCallback(
      boost::bind(&NavigationLayer::reconfigureCallback, this, _1, _2));
  // Initialize the static layer to fit the map.
  static_layer_.initialize(layered_costmap_, name_ + "/StaticLayer", tf_);
  laser_layer_.initialize(layered_costmap_, name_ + "/LaserLayer", tf_);
  kinect_layer_.initialize(layered_costmap_, name_ + "/DepthCameraLayer", tf_);
  // Align size of kinect and laser layer.
  static_layer_.matchSize<ObstacleLayer>(&laser_layer_);
  static_layer_.matchSize<VoxelLayer>(&kinect_layer_);
  // Initialize services.
  clear_costmap_srv_ = pnh.advertiseService(
      "clearCostmapRegion", &NavigationLayer::clearCostmapRegionCallback, this);
  obstacles_map_srv_ = pnh.advertiseService(
      "getObstaclesMap", &NavigationLayer::getObstaclesMapCallback, this);
  path_clearance_srv_ = pnh.advertiseService(
      "getPathClearance", &NavigationLayer::getPathClearanceCallback, this);
  // Finalize intialization.
  current_ = true;
  enabled_ = true;
  // Initialization Successful.
  ROS_INFO("squirrel_navigation/NavigationLayer: Initializations successful.");
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

bool NavigationLayer::clearCostmapRegionCallback(
    squirrel_navigation_msgs::ClearCostmapRegion::Request& req,
    squirrel_navigation_msgs::ClearCostmapRegion::Response& res) {
  // Wait for costmap to be disabled.
  req.sleep.sleep();
  if (kinect_layer_.enabled() || laser_layer_.enabled())
    return false;
  // Getting clearing bounding box.
  const int inf = std::numeric_limits<double>::max();
  int min_i = inf, min_j = inf, max_i = -inf, max_j = -inf;
  for (const auto& point : req.region.points) {
    int p_i, p_j;
    worldToMapEnforceBounds(point.x, point.y, p_i, p_j);
    min_i = std::min(min_i, p_i);
    min_j = std::min(min_j, p_j);
    max_i = std::max(max_i, p_i);
    max_j = std::max(max_j, p_j);
  }
  // Clear the master grid.
  costmap_2d::Costmap2D* master_costmap = layered_costmap_->getCostmap();
  for (int i = min_i; i < max_i; ++i)
    for (int j = min_j; j < max_j; ++j)
      master_costmap->setCost(i, j, costmap_2d::FREE_SPACE);
  return true;
}

bool NavigationLayer::getObstaclesMapCallback(
    squirrel_navigation_msgs::GetObstaclesMap::Request& req,
    squirrel_navigation_msgs::GetObstaclesMap::Response& res) {
  std::vector<bool> obstacles_indicator;
  std::vector<geometry_msgs::Point32> obstacles_positions;
  const int nobstacles =
      getObstaclesMap(&obstacles_indicator, &obstacles_positions);
  res.obstacles_indicator.resize(nobstacles);
  res.obstacles_positions.resize(nobstacles);
  for (int i = 0; i < nobstacles; ++i) {
    res.obstacles_indicator[i] = obstacles_indicator[i];
    res.obstacles_positions[i] = obstacles_positions[i];
  }
  // Set the map origin.
  const costmap_2d::Costmap2D* master_costmap = layered_costmap_->getCostmap();
  res.map_origin.x                            = master_costmap->getOriginX();
  res.map_origin.y                            = master_costmap->getOriginY();
  res.map_origin.theta                        = 0.;
  return true;
}

bool NavigationLayer::getPathClearanceCallback(
    squirrel_navigation_msgs::GetPathClearance::Request& req,
    squirrel_navigation_msgs::GetPathClearance::Response& res) {
  std::vector<bool> obstacles_indicator;
  std::vector<geometry_msgs::Point32> obstacles_positions;
  getObstaclesMap(&obstacles_indicator, &obstacles_positions);
  // Resize the storage.
  const int nwaypoints = req.plan.poses.size();
  res.proximity_map.reserve(nwaypoints);
  res.proximities.resize(nwaypoints);
  geometry_msgs::Point32 closest_obstacle;
  // Get the proximity informations.
  res.clearance = std::numeric_limits<double>::max();
  for (int i = 0; i < nwaypoints; ++i) {
    const auto& waypoint = req.plan.poses[i];
    // Get the clearance of the waypoint.
    res.proximities[i] = std::numeric_limits<double>::max();
    for (int o = 0; o < obstacles_positions.size(); ++o) {
      const double dist = std::hypot(
          waypoint.pose.position.x - obstacles_positions[o].x,
          waypoint.pose.position.y - obstacles_positions[o].y);
      if (dist < res.proximities[i]) {
        res.proximities[i]   = dist;
        res.proximity_map[i] = obstacles_positions[o];
      }
    }
    // Update the proximity map.
    if (res.proximities[i] < res.clearance) {
      res.clearance          = res.proximities[i];
      res.clearance_waypoint = i;
    }
  }
  return true;
}

size_t NavigationLayer::getObstaclesMap(
    std::vector<bool>* obstacles_indicator,
    std::vector<geometry_msgs::Point32>* obstacles_positions) const {
  std::unique_lock<std::mutex> lock(update_mtx_);
  // Costmap size.
  const costmap_2d::Costmap2D* master_costmap = layered_costmap_->getCostmap();
  const size_t size_x = master_costmap->getSizeInCellsX();
  const size_t size_y = master_costmap->getSizeInCellsY();
  // Storage.
  obstacles_indicator->resize(size_x * size_y);
  obstacles_positions->reserve(size_x * size_y);
  geometry_msgs::Point32 point;
  // Check for obstacles.
  size_t nobstacles = 0;
  double px, py;
  for (size_t x = 0; x < size_x; ++x)
    for (size_t y = 0; y < size_y; ++y) {
      const unsigned int index       = master_costmap->getIndex(x, y);
      obstacles_indicator->at(index) = false;
      if (master_costmap->getCost(x, y) == costmap_2d::FREE_SPACE) {
        obstacles_indicator->at(index) = true;
        master_costmap->mapToWorld(x, y, px, py);
        obstacles_positions->emplace_back(point);
        nobstacles++;
      }
    }
  return nobstacles;
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
