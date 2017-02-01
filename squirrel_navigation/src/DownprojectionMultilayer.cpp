// DownprojectionMultilayer.cpp ---
//
// Filename: DownprojectionMultilayer.cpp
// Description: Dynamic mapping of obstacles with RGBD
//              and Laser sensors
// Author: Federico Boniardi
// Maintainer: boniardi@cs.uni-freiburg.de
// Created: Wed Nov 19 18:57:41 2014 (+0100)
// Version: 0.1.0
// Last-Updated: Tue Jan 19 16:07:03 2016 (+0100)
//           By: Federico Boniardi
//     Update #: 8
// URL:
// Keywords:
// Compatibility:
//   ROS Hydro, ROS Indigo
//

// Commentary:
//   The code therein is an integration of costmap_2d::InflationLayer into
//   costmap_2d::VoxelLayer. Both source codes are distributed by the authors
//   under BSD license which is below reported
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

#include "squirrel_navigation/DownprojectionMultilayer.h"

PLUGINLIB_EXPORT_CLASS(
    squirrel_navigation::DownprojectionMultilayer, costmap_2d::Layer)

namespace squirrel_navigation {

DownprojectionMultilayer::DownprojectionMultilayer(void)
    : voxel_grid_(0, 0, 0),
      max_obstacle_height_(1.0),
      min_obstacle_height_(0.0),
      obstacles_persistence_(60.0),
      dsrv_(NULL),
      kinect_tilt_h_("kinect_tilt_joint"),
      kinect_pan_h_("kinect_pan_joint") {
  costmap_               = nullptr;
  costmap_update_handle_ = CostmapUpdateHandle::getHandle();
}

DownprojectionMultilayer::~DownprojectionMultilayer(void) {
  if (dsrv_)
    delete dsrv_;

  CostmapUpdateHandle::releaseHandle();
}

void DownprojectionMultilayer::onInitialize(void) {
  ObstacleLayer::onInitialize();
  footprint_.onInitialize();
  matchInflatedSize();
}

void DownprojectionMultilayer::updateBounds(
    double robot_x, double robot_y, double robot_yaw, double* min_x,
    double* min_y, double* max_x, double* max_y) {
  footprint_.updateCurrentMultiFootprint(robot_x, robot_y, robot_yaw);

  if (rolling_window_)
    updateOrigin(
        robot_x - getSizeInMetersX() / 2, robot_y - getSizeInMetersY() / 2);

  if (!enabled_)
    return;

  if (!costmap_update_handle_->performUpdate()) {
    if (verbose_)
      ROS_INFO(
          "%s/%s: Skipping costmap's update.",
          ros::this_node::getName().c_str(), name_.c_str());
    return;
  }

  if (kinect_tilt_h_.skipData() or kinect_pan_h_.skipData()) {
    if (verbose_)
      ROS_INFO(
          "%s/%s: Skipping costmap's update. Kinect is moving.",
          ros::this_node::getName().c_str(), name_.c_str());
    return;
  }

  // This function doesnot compile if older version of ROS+Ubuntu are used
  useExtraBounds(min_x, min_y, max_x, max_y);

  bool current = true;
  std::vector<costmap_2d::Observation> observations, clearing_observations;

  current  = current && getMarkingObservations(observations);
  current  = current && getClearingObservations(clearing_observations);
  current_ = current;

  for (unsigned int i = 0; i < clearing_observations.size(); ++i)
    raytraceFreespace(clearing_observations[i], min_x, min_y, max_x, max_y);

  ros::Time now = ros::Time::now();
  std::set<unsigned int> index_free_space;
  std::map<unsigned int, bool> free_space_lock;

  if (obstacles_persistence_ > 0)
    for (std::map<unsigned int, ros::Time>::iterator i =
             clearing_index_stamped_.begin();
         i != clearing_index_stamped_.end(); ++i)
      if (i->second.toSec() < now.toSec() - obstacles_persistence_)
        costmap_[i->first] = costmap_2d::FREE_SPACE;

  for (std::vector<costmap_2d::Observation>::const_iterator it =
           observations.begin();
       it != observations.end(); ++it) {
    const costmap_2d::Observation& obs = *it;

    const pcl::PointCloud<pcl::PointXYZ>& cloud = *(obs.cloud_);

    const double sq_obstacle_range = obs.obstacle_range_ * obs.obstacle_range_;

    for (unsigned int i = 0; i < cloud.points.size(); ++i) {
      if (cloud.points[i].z > max_obstacle_height_ or
          cloud.points[i].z < -min_obstacle_height_)
        continue;

      unsigned int l = layer(cloud.points[i].z);

      const double sq_dist_orig = (cloud.points[i].x - obs.origin_.x) *
                                      (cloud.points[i].x - obs.origin_.x) +
                                  (cloud.points[i].y - obs.origin_.y) *
                                      (cloud.points[i].y - obs.origin_.y);

      if (sq_dist_orig >= sq_obstacle_range and
          cloud.points[i].z >= min_obstacle_height_)
        continue;

      const double sq_dist_robot =
          (cloud.points[i].x - robot_x) * (cloud.points[i].x - robot_x) +
          (cloud.points[i].y - robot_y) * (cloud.points[i].y - robot_y);
      const double sq_in_radius = std::pow(footprint_.inscribedRadius(l), 2);
      const double sq_circ_radius =
          std::pow(footprint_.circumscribedRadius(l), 2);

      if (sq_dist_robot <= sq_in_radius) {
        continue;
      } else if (sq_dist_robot <= sq_circ_radius) {
        if (footprint_.isInside(cloud.points[i].x, cloud.points[i].y, l)) {
          continue;
        }
      }

      unsigned int mx, my, mz;
      if (cloud.points[i].z < origin_z_) {
        if (!worldToMap3D(
                cloud.points[i].x, cloud.points[i].y, origin_z_, mx, my, mz)) {
          continue;
        }
      } else if (!worldToMap3D(
                     cloud.points[i].x, cloud.points[i].y, cloud.points[i].z,
                     mx, my, mz)) {
        continue;
      }

      unsigned int index = getIndex(mx, my);

      if (cloud.points[i].z < min_obstacle_height_ && !free_space_lock[index]) {
        index_free_space.insert(index);
        touch(
            (double)cloud.points[i].x, (double)cloud.points[i].y, min_x, min_y,
            max_x, max_y);
      } else if (
          cloud.points[i].z >= min_obstacle_height_ &&
          voxel_grid_.markVoxelInMap(mx, my, mz, mark_threshold_)) {
        index_free_space.erase(index);
        free_space_lock[index]         = true;
        clearing_index_stamped_[index] = now;
        costmap_[index]                = costmap_2d::LETHAL_OBSTACLE;
        if (!observed_[index]) {
          obstacles_[index] = l;
          observed_[index]  = true;
        } else {
          obstacles_[index] =
              inscribed_radii_[l] > inscribed_radii_[obstacles_[index]]
                  ? l
                  : obstacles_[index];
        }
        touch(
            (double)cloud.points[i].x, (double)cloud.points[i].y, min_x, min_y,
            max_x, max_y);
      }
    }
  }

  for (std::set<unsigned int>::iterator f = index_free_space.begin();
       f != index_free_space.end(); ++f)
    costmap_[*f] = costmap_2d::FREE_SPACE;

  updateInflatedBounds(robot_x, robot_y, robot_yaw, min_x, min_y, max_x, max_y);
}

void DownprojectionMultilayer::updateOrigin(
    double new_origin_x, double new_origin_y) {
  int cell_ox, cell_oy;
  cell_ox = int((new_origin_x - origin_x_) / resolution_);
  cell_oy = int((new_origin_y - origin_y_) / resolution_);

  double new_grid_ox, new_grid_oy;
  new_grid_ox = origin_x_ + cell_ox * resolution_;
  new_grid_oy = origin_y_ + cell_oy * resolution_;

  int size_x = size_x_;
  int size_y = size_y_;

  int lower_left_x, lower_left_y, upper_right_x, upper_right_y;
  lower_left_x  = std::min(std::max(cell_ox, 0), size_x);
  lower_left_y  = std::min(std::max(cell_oy, 0), size_y);
  upper_right_x = std::min(std::max(cell_ox + size_x, 0), size_x);
  upper_right_y = std::min(std::max(cell_oy + size_y, 0), size_y);

  unsigned int cell_size_x = upper_right_x - lower_left_x;
  unsigned int cell_size_y = upper_right_y - lower_left_y;

  unsigned char* local_map      = new unsigned char[cell_size_x * cell_size_y];
  unsigned int* local_voxel_map = new unsigned int[cell_size_x * cell_size_y];
  unsigned int* voxel_map       = voxel_grid_.getData();

  copyMapRegion(
      costmap_, lower_left_x, lower_left_y, size_x_, local_map, 0, 0,
      cell_size_x, cell_size_x, cell_size_y);
  copyMapRegion(
      voxel_map, lower_left_x, lower_left_y, size_x_, local_voxel_map, 0, 0,
      cell_size_x, cell_size_x, cell_size_y);

  resetMaps();

  origin_x_ = new_grid_ox;
  origin_y_ = new_grid_oy;

  int start_x = lower_left_x - cell_ox;
  int start_y = lower_left_y - cell_oy;

  copyMapRegion(
      local_map, 0, 0, cell_size_x, costmap_, start_x, start_y, size_x_,
      cell_size_x, cell_size_y);
  copyMapRegion(
      local_voxel_map, 0, 0, cell_size_x, voxel_map, start_x, start_y, size_x_,
      cell_size_x, cell_size_y);

  delete[] local_map;
  delete[] local_voxel_map;
}

void DownprojectionMultilayer::updateCosts(
    costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i,
    int max_j) {
  if (!enabled_)
    return;

  if (combination_method_ == 0) {
    updateWithOverwrite(master_grid, min_i, min_j, max_i, max_j);
  } else {
    updateWithMax(master_grid, min_i, min_j, max_i, max_j);
  }

  updateInflatedCosts(master_grid, min_i, min_j, max_i, max_j);

  obstacles_.clear();
  observed_.clear();
}

bool DownprojectionMultilayer::isDiscretized(void) { return true; }

void DownprojectionMultilayer::matchSize(void) {
  costmap_2d::ObstacleLayer::matchSize();
  voxel_grid_.resize(size_x_, size_y_, size_z_);
  ROS_ASSERT(voxel_grid_.sizeX() == size_x_ && voxel_grid_.sizeY() == size_y_);
}

void DownprojectionMultilayer::reset(void) {
  deactivate();
  resetMaps();
  voxel_grid_.reset();
  activate();
}

void DownprojectionMultilayer::setupDynamicReconfigure(ros::NodeHandle& nh) {
  {
    boost::unique_lock<boost::shared_mutex> lock(*access_);
    current_          = true;
    seen_             = NULL;
    need_reinflation_ = false;

    dynamic_reconfigure::Server<
        DownprojectionMultilayerPluginConfig>::CallbackType cb =
        boost::bind(&DownprojectionMultilayer::reconfigureCB, this, _1, _2);

    if (dsrv_ != NULL) {
      dsrv_->clearCallback();
      dsrv_->setCallback(cb);
    } else {
      dsrv_ =
          new dynamic_reconfigure::Server<DownprojectionMultilayerPluginConfig>(
              nh);
      dsrv_->setCallback(cb);
    }
  }
}

void DownprojectionMultilayer::resetMaps(void) {
  costmap_2d::Costmap2D::resetMaps();
  voxel_grid_.reset();
}

void DownprojectionMultilayer::reconfigureCB(
    DownprojectionMultilayerPluginConfig& config, uint32_t level) {
  num_layers_ = config.num_layers;
  malloc_layers();

  max_obstacle_height_ = config.max_obstacle_height;
  min_obstacle_height_ = config.min_obstacle_height;

  size_z_                = config.z_voxels;
  origin_z_              = config.origin_z;
  z_resolution_          = config.z_resolution;
  unknown_threshold_     = config.unknown_threshold + (VOXEL_BITS - size_z_);
  mark_threshold_        = config.mark_threshold;
  obstacles_persistence_ = config.obstacles_persistence;

  if (obstacles_persistence_ > 0) {
    ROS_INFO("Obstacle persistence: %f", obstacles_persistence_);
  } else if (obstacles_persistence_ == 0) {
    ROS_WARN("Obstacle_persistence is chosen to be 0(s). Reset to 60.0(s).");
  }

  robot_link_radii_ = ParameterParser::array<double>(config.robot_link_radii);
  layers_levels_    = ParameterParser::array<double>(config.layers_levels);

  std::vector<double> weights =
      ParameterParser::array<double>(config.cost_scaling_factors);
  std::vector<double> inflation_radii =
      ParameterParser::array<double>(config.inflation_radii);
  std::vector<double> inscribed_radii =
      ParameterParser::array<double>(config.inscribed_radii);

  if (robot_link_radii_.size() != num_layers_ ||
      weights.size() != num_layers_ || layers_levels_.size() != num_layers_ ||
      inflation_radii.size() != num_layers_ ||
      inscribed_radii.size() != num_layers_) {
    ROS_ERROR(
        "Invalid parameter for squirrel_navigation::DownprojectionMultilayer");
    ros::shutdown();
  }

  if (!std::equal(weights_.begin(), weights_.end(), weights.data()) ||
      !std::equal(
          inflation_radii_.begin(), inflation_radii.end(),
          inflation_radii.data())) {

    inflation_radii_ = inflation_radii;

    cell_inflation_radii_.clear();
    for (unsigned int i = 0; i < num_layers_; ++i) {
      cell_inflation_radii_.push_back(cellDistance(inflation_radii_[i]));
    }
    max_cell_inflation_radius_ = *std::max_element(
        cell_inflation_radii_.begin(), cell_inflation_radii_.end());

    inscribed_radii_ = inscribed_radii;

    weights_ = weights;

    need_reinflation_ = true;

    computeCaches();
  }

  if (enabled_ != config.enabled) {
    enabled_          = config.enabled;
    need_reinflation_ = true;
  }

  combination_method_ = config.combination_method;

  verbose_ = config.verbose;

  matchSize();
}

void DownprojectionMultilayer::clearNonLethal(
    double wx, double wy, double w_size_x, double w_size_y,
    bool clear_no_info) {
  unsigned int mx, my;
  if (!worldToMap(wx, wy, mx, my)) {
    return;
  }

  double start_x = wx - w_size_x / 2;
  double start_y = wy - w_size_y / 2;
  double end_x   = start_x + w_size_x;
  double end_y   = start_y + w_size_y;

  start_x = std::max(origin_x_, start_x);
  start_y = std::max(origin_y_, start_y);

  end_x = std::min(origin_x_ + getSizeInMetersX(), end_x);
  end_y = std::min(origin_y_ + getSizeInMetersY(), end_y);

  unsigned int map_sx, map_sy, map_ex, map_ey;

  if (!worldToMap(start_x, start_y, map_sx, map_sy) ||
      !worldToMap(end_x, end_y, map_ex, map_ey)) {
    return;
  }

  unsigned int index     = getIndex(map_sx, map_sy);
  unsigned char* current = &costmap_[index];
  for (unsigned int j = map_sy; j <= map_ey; ++j) {
    for (unsigned int i = map_sx; i <= map_ex; ++i) {
      if (*current != costmap_2d::LETHAL_OBSTACLE) {
        if (clear_no_info || *current != costmap_2d::NO_INFORMATION) {
          *current = costmap_2d::FREE_SPACE;
          voxel_grid_.clearVoxelColumn(index);
        }
      }
      current++;
      index++;
    }
    current += size_x_ - (map_ex - map_sx) - 1;
    index += size_x_ - (map_ex - map_sx) - 1;
  }
}

void DownprojectionMultilayer::raytraceFreespace(
    const costmap_2d::Observation& clearing_observation, double* min_x,
    double* min_y, double* max_x, double* max_y) {
  if (clearing_observation.cloud_->points.size() == 0) {
    return;
  }

  double sensor_x, sensor_y, sensor_z;
  double ox = clearing_observation.origin_.x;
  double oy = clearing_observation.origin_.y;
  double oz = clearing_observation.origin_.z;

  if (!worldToMap3DFloat(ox, oy, oz, sensor_x, sensor_y, sensor_z)) {
    ROS_WARN_THROTTLE(
        1.0,
        "The origin for the sensor at (%.2f, %.2f, %.2f) is out of map bounds. "
        "So, the costmap cannot raytrace for it.",
        ox, oy, oz);
    return;
  }

  double map_end_x = origin_x_ + getSizeInMetersX();
  double map_end_y = origin_y_ + getSizeInMetersY();

  for (unsigned int i = 0; i < clearing_observation.cloud_->points.size();
       ++i) {
    double wpx = clearing_observation.cloud_->points[i].x;
    double wpy = clearing_observation.cloud_->points[i].y;
    double wpz = clearing_observation.cloud_->points[i].z;

    double distance     = linearDistance(ox, oy, oz, wpx, wpy, wpz);
    double scaling_fact = 1.0;
    scaling_fact        = std::max(
        std::min(scaling_fact, (distance - 2 * resolution_) / distance), 0.0);
    wpx = scaling_fact * (wpx - ox) + ox;
    wpy = scaling_fact * (wpy - oy) + oy;
    wpz = scaling_fact * (wpz - oz) + oz;

    double a = wpx - ox;
    double b = wpy - oy;
    double c = wpz - oz;
    double t = 1.0;

    if (wpz > max_obstacle_height_) {
      t = std::max(0.0, std::min(t, (max_obstacle_height_ - 0.01 - oz) / c));
    } else if (wpz < origin_z_) {
      t = std::min(t, (origin_z_ - oz) / c);
    }

    if (wpx < origin_x_) {
      t = std::min(t, (origin_x_ - ox) / a);
    }

    if (wpy < origin_y_) {
      t = std::min(t, (origin_y_ - oy) / b);
    }

    if (wpx > map_end_x) {
      t = std::min(t, (map_end_x - ox) / a);
    }

    if (wpy > map_end_y) {
      t = std::min(t, (map_end_y - oy) / b);
    }

    wpx = ox + a * t;
    wpy = oy + b * t;
    wpz = oz + c * t;

    double point_x, point_y, point_z;
    if (worldToMap3DFloat(wpx, wpy, wpz, point_x, point_y, point_z)) {
      unsigned int cell_raytrace_range =
          cellDistance(clearing_observation.raytrace_range_);

      voxel_grid_.clearVoxelLineInMap(
          sensor_x, sensor_y, sensor_z, point_x, point_y, point_z, costmap_,
          unknown_threshold_, mark_threshold_, costmap_2d::FREE_SPACE,
          costmap_2d::NO_INFORMATION, cell_raytrace_range);

      updateRaytraceBounds(
          ox, oy, wpx, wpy, clearing_observation.raytrace_range_, min_x, min_y,
          max_x, max_y);
    }
  }
}

}  // namespace squirrel_navigation
