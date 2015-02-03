// ObstaclesLayer.cpp --- 
// 
// Filename: ObstaclesLayer.cpp
// Description: Dynamic mapping of obstacles with RGBD
//              and Laser sensors
// Author: Federico Boniardi
// Maintainer: boniardi@cs.uni-freiburg.de
// Created: Wed Nov 19 18:57:41 2014 (+0100)
// Version: 0.1.0
// Last-Updated: Tue Feb 3 10:52:14 2015 (+0100)
//           By: Federico Boniardi
//     Update #: 4
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
//
//   Tested on: - ROS Hydro on Ubuntu 12.04
//              - ROS Indigo on Ubuntu 14.04
//
//      

// Code:

#include "squirrel_navigation/ObstaclesLayer.h"

#include <pluginlib/class_list_macros.h>
#include <pcl_conversions/pcl_conversions.h>

#define VOXEL_BITS 16

PLUGINLIB_EXPORT_CLASS(squirrel_navigation::ObstaclesLayer, costmap_2d::Layer)

namespace squirrel_navigation {

ObstaclesLayer::ObstaclesLayer( void ) :
    voxel_grid_(0, 0, 0),
    robot_link_radius_(0.5),
    max_obstacle_height_(1.0),
    min_obstacle_height_(0.0),
    obstacles_persistence_(60.0),
    inflation_radius_(0.0),
    weight_(0.0),
    cell_inflation_radius_(0),
    cached_cell_inflation_radius_(0),
    dsrv_(NULL)
{
  access_ = new boost::shared_mutex();
  costmap_ = NULL;
}

ObstaclesLayer::~ObstaclesLayer( void )
{
  if ( dsrv_ ) {
    delete dsrv_;
  }
}

void ObstaclesLayer::onInitialize( void )
{
   ObstacleLayer::onInitialize();
  inflationMatchSize();
}

void ObstaclesLayer::updateBounds( double robot_x, double robot_y, double robot_yaw,
                                   double* min_x, double* min_y, double* max_x, double* max_y )
{
  if ( rolling_window_ ) {
    updateOrigin(robot_x - getSizeInMetersX() / 2, robot_y - getSizeInMetersY() / 2);
  }
  if (!enabled_) {
    return;
  }
  // This function doesn't compile with the current libraries in the Robotino
  // useExtraBounds(min_x, min_y, max_x, max_y);
  
  bool current = true;
  std::vector<costmap_2d::Observation> observations, clearing_observations;

  current = current && getMarkingObservations(observations);
  current = current && getClearingObservations(clearing_observations);
  current_ = current;

  for (unsigned int i = 0; i < clearing_observations.size(); ++i) {
    raytraceFreespace(clearing_observations[i], min_x, min_y, max_x, max_y);
  }

  ros::Time now = ros::Time::now();
  std::set<unsigned int> index_free_space;

  if ( obstacles_persistence_ > 0 ) {
    for (std::map<unsigned int, ros::Time>::iterator i=clearing_index_stamped_.begin(); i!=clearing_index_stamped_.end(); ++i) {
      if ( i->second.toSec() < now.toSec()-obstacles_persistence_ ) {
        costmap_[i->first] = costmap_2d::FREE_SPACE;
      }
    }
  }
  
  for (std::vector<costmap_2d::Observation>::const_iterator it = observations.begin(); it != observations.end(); ++it) {
    const costmap_2d::Observation& obs = *it;

    const pcl::PointCloud<pcl::PointXYZ>& cloud = *(obs.cloud_);
    
    double sq_obstacle_range = obs.obstacle_range_ * obs.obstacle_range_;

    for (unsigned int i = 0; i < cloud.points.size(); ++i) {
      if ( cloud.points[i].z > max_obstacle_height_ ) {
        continue;
      }
      
      double sq_dist_orig = (cloud.points[i].x - obs.origin_.x) * (cloud.points[i].x - obs.origin_.x)
          + (cloud.points[i].y - obs.origin_.y) * (cloud.points[i].y - obs.origin_.y)
          + (cloud.points[i].z - obs.origin_.z) * (cloud.points[i].z - obs.origin_.z);

      if ( sq_dist_orig >= sq_obstacle_range ) {
        continue;
      }

      double sq_dist_robot = (cloud.points[i].x - robot_x) * (cloud.points[i].x - robot_x)
          + (cloud.points[i].y - robot_y) * (cloud.points[i].y - robot_y);

      if ( sq_dist_orig <= std::pow(robot_link_radius_, 2) ) {
        continue;
      }
      
      unsigned int mx, my, mz;
      if ( cloud.points[i].z < origin_z_ ) {
        if ( !worldToMap3D(cloud.points[i].x, cloud.points[i].y, origin_z_, mx, my, mz) ) {
          continue;
        }
      } else if ( !worldToMap3D(cloud.points[i].x, cloud.points[i].y, cloud.points[i].z, mx, my, mz) ) {
        continue;
      }

      unsigned int index = getIndex(mx, my);
      index_free_space.insert(index);
      obstacles_index_.erase(index);

      if ( cloud.points[i].z > min_obstacle_height_ ) {
        clearing_index_stamped_[index] = now;
        index_free_space.erase(index);
      }
      
      if ( voxel_grid_.markVoxelInMap(mx, my, mz, mark_threshold_) ) {
        costmap_[index] = costmap_2d::LETHAL_OBSTACLE;
        obstacles_index_.insert(index);
        touch((double)cloud.points[i].x, (double)cloud.points[i].y, min_x, min_y, max_x, max_y);
      }
    }
  }

  for (std::set<unsigned int>::iterator i=index_free_space.begin(); i!=index_free_space.end(); ++i) {
    costmap_[*i] = costmap_2d::FREE_SPACE; 
  }
 
  footprint_layer_.updateBounds(robot_x, robot_y, robot_yaw, min_x, min_y, max_x, max_y);

  if( need_reinflation_ ) {
    *min_x = -std::numeric_limits<float>::max();
    *min_y = -std::numeric_limits<float>::max();
    *max_x = std::numeric_limits<float>::max();
    *max_y = std::numeric_limits<float>::max();
    need_reinflation_ = false;
  }

}

void ObstaclesLayer::updateOrigin(double new_origin_x, double new_origin_y)
{
  int cell_ox, cell_oy;
  cell_ox = int((new_origin_x - origin_x_) / resolution_);
  cell_oy = int((new_origin_y - origin_y_) / resolution_);

  double new_grid_ox, new_grid_oy;
  new_grid_ox = origin_x_ + cell_ox * resolution_;
  new_grid_oy = origin_y_ + cell_oy * resolution_;

  int size_x = size_x_;
  int size_y = size_y_;

  int lower_left_x, lower_left_y, upper_right_x, upper_right_y;
  lower_left_x = std::min(std::max(cell_ox, 0), size_x);
  lower_left_y = std::min(std::max(cell_oy, 0), size_y);
  upper_right_x = std::min(std::max(cell_ox + size_x, 0), size_x);
  upper_right_y = std::min(std::max(cell_oy + size_y, 0), size_y);

  unsigned int cell_size_x = upper_right_x - lower_left_x;
  unsigned int cell_size_y = upper_right_y - lower_left_y;

  unsigned char* local_map = new unsigned char[cell_size_x * cell_size_y];
  unsigned int* local_voxel_map = new unsigned int[cell_size_x * cell_size_y];
  unsigned int* voxel_map = voxel_grid_.getData();

  copyMapRegion(costmap_, lower_left_x, lower_left_y, size_x_, local_map, 0, 0, cell_size_x, cell_size_x, cell_size_y);
  copyMapRegion(voxel_map, lower_left_x, lower_left_y, size_x_, local_voxel_map, 0, 0, cell_size_x, cell_size_x,
                cell_size_y);

  resetMaps();

  origin_x_ = new_grid_ox;
  origin_y_ = new_grid_oy;

  int start_x = lower_left_x - cell_ox;
  int start_y = lower_left_y - cell_oy;

  copyMapRegion(local_map, 0, 0, cell_size_x, costmap_, start_x, start_y, size_x_, cell_size_x, cell_size_y);
  copyMapRegion(local_voxel_map, 0, 0, cell_size_x, voxel_map, start_x, start_y, size_x_, cell_size_x, cell_size_y);

  delete[] local_map;
  delete[] local_voxel_map;
}

void ObstaclesLayer::updateCosts( costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j )
{
  if ( !enabled_ ) {
    return;
  }

  footprint_layer_.updateCosts(*this, min_i, min_j, max_i, max_j);

  if(combination_method_==0) {
    updateWithOverwrite(master_grid, min_i, min_j, max_i, max_j);
  } else {
    updateWithMax(master_grid, min_i, min_j, max_i, max_j);
  }
  
  boost::unique_lock < boost::shared_mutex > lock(*access_);

  //make sure the inflation queue is empty at the beginning of the cycle (should always be true)
  ROS_ASSERT_MSG(inflation_queue_.empty(), "The inflation queue must be empty at the beginning of inflation");

  unsigned char* master_array = master_grid.getCharMap();
  unsigned int size_x = master_grid.getSizeInCellsX(), size_y = master_grid.getSizeInCellsY();

  memset(seen_, false, size_x * size_y * sizeof(bool));
  
  // We need to include in the inflation cells outside the bounding
  // box min_i...max_j, by the amount cell_inflation_radius_.  Cells
  // up to that distance outside the box can still influence the costs
  // stored in cells inside the box.
  min_i -= cell_inflation_radius_;
  min_j -= cell_inflation_radius_;
  max_i += cell_inflation_radius_;
  max_j += cell_inflation_radius_;

  min_i = std::max( 0, min_i );
  min_j = std::max( 0, min_j );
  max_i = std::min( int( size_x  ), max_i );
  max_j = std::min( int( size_y  ), max_j );

  for (int j = min_j; j < max_j; j++) {
    for (int i = min_i; i < max_i; i++) {
      int index = master_grid.getIndex(i, j);
      unsigned char cost = master_array[index];
      bool is_obstacle = obstacles_index_.find(index) != obstacles_index_.end();
      if ( cost == costmap_2d::LETHAL_OBSTACLE && is_obstacle ) {
        enqueue(master_array, index, i, j, i, j);
      }
    }
  }

  while (!inflation_queue_.empty())
  {
    //get the highest priority cell and pop it off the priority queue
    const CellData& current_cell = inflation_queue_.top();

    unsigned int index = current_cell.index_;
    unsigned int mx = current_cell.x_;
    unsigned int my = current_cell.y_;
    unsigned int sx = current_cell.src_x_;
    unsigned int sy = current_cell.src_y_;

    //pop once we have our cell info
    inflation_queue_.pop();

    //attempt to put the neighbors of the current cell onto the queue
    if ( mx > 0 ) {
      enqueue(master_array, index - 1, mx - 1, my, sx, sy);
    }
    if ( my > 0 ) {
      enqueue(master_array, index - size_x, mx, my - 1, sx, sy);
    }
    if ( mx < size_x - 1 ) {
      enqueue(master_array, index + 1, mx + 1, my, sx, sy);
    }
    if ( my < size_y - 1 ) {
      enqueue(master_array, index + size_x, mx, my + 1, sx, sy);
    }
  }
}

bool ObstaclesLayer::isDiscretized( void )
{
  return true;
}

void ObstaclesLayer::obstaclesMatchSize( void )
{
  costmap_2d::ObstacleLayer::matchSize();
  voxel_grid_.resize(size_x_, size_y_, size_z_);
  ROS_ASSERT(voxel_grid_.sizeX() == size_x_ && voxel_grid_.sizeY() == size_y_);
}

void ObstaclesLayer::inflationMatchSize( void )
{
  boost::unique_lock < boost::shared_mutex > lock(*access_);
  costmap_2d::Costmap2D* costmap = layered_costmap_->getCostmap();
  cell_inflation_radius_ = cellDistance(inflation_radius_);
  computeCaches();
  
  unsigned int size_x = costmap->getSizeInCellsX(), size_y = costmap->getSizeInCellsY();

  if ( seen_ ) {
    delete seen_;
  }
  seen_ = new bool[size_x * size_y]; 
}

void ObstaclesLayer::reset( void )
{
  deactivate();
  resetMaps();
  voxel_grid_.reset();
  activate();
}

void ObstaclesLayer::setupDynamicReconfigure( ros::NodeHandle& nh )
{
  {
    boost::unique_lock < boost::shared_mutex > lock(*access_);
    current_ = true;
    seen_ = NULL;
    need_reinflation_ = false;

    dynamic_reconfigure::Server<ObstaclesLayerPluginConfig>::CallbackType cb = boost::bind(
        &ObstaclesLayer::reconfigureCB, this, _1, _2);

    if ( dsrv_ != NULL ) {
      dsrv_->clearCallback();
      dsrv_->setCallback(cb);
    } else {
      dsrv_ = new dynamic_reconfigure::Server<ObstaclesLayerPluginConfig>(ros::NodeHandle("~/"+name_));
      dsrv_->setCallback(cb);
    }
  }
}

void ObstaclesLayer::resetMaps( void )
{
  costmap_2d::Costmap2D::resetMaps();
  voxel_grid_.reset();
}

void ObstaclesLayer::reconfigureCB( ObstaclesLayerPluginConfig &config, uint32_t level )
{
  enabled_ = config.enabled;

  if ( enabled_ != config.enabled ) {
    enabled_ = config.enabled;
    need_reinflation_ = true;
  }
  
  max_obstacle_height_ = config.max_obstacle_height;
  min_obstacle_height_ = config.min_obstacle_height;
  robot_link_radius_ = config.robot_link_radius;
  size_z_ = config.z_voxels;
  origin_z_ = config.origin_z;
  z_resolution_ = config.z_resolution;
  unknown_threshold_ = config.unknown_threshold + (VOXEL_BITS - size_z_);
  mark_threshold_ = config.mark_threshold;
  
  if ( obstacles_persistence_ > 0 ) {
    ROS_INFO_STREAM("obstacle persistence: " << obstacles_persistence_);
  } else if ( obstacles_persistence_ == 0) {
    std::string ns = ros::this_node::getNamespace();
    ROS_WARN("/%s: obstacle_persistance is chosen to be 0(s). Reset to 60.0(s). ", ns.c_str());
  } 

  inscribed_radius_ = robot_link_radius_;

  if ( weight_ != config.cost_scaling_factor || inflation_radius_ != config.inflation_radius ) {
    inflation_radius_ = config.inflation_radius;
    cell_inflation_radius_ = cellDistance(inflation_radius_);
    weight_ = config.cost_scaling_factor;
    need_reinflation_ = true;
    computeCaches();
  }

  combination_method_ = config.combination_method;
  
  obstaclesMatchSize();
}

void ObstaclesLayer::clearNonLethal(double wx, double wy, double w_size_x, double w_size_y, bool clear_no_info)
{
  unsigned int mx, my;
  if (!worldToMap(wx, wy, mx, my)) {
    return;
  }
  
  double start_x = wx - w_size_x / 2;
  double start_y = wy - w_size_y / 2;
  double end_x = start_x + w_size_x;
  double end_y = start_y + w_size_y;

  start_x = std::max(origin_x_, start_x);
  start_y = std::max(origin_y_, start_y);

  end_x = std::min(origin_x_ + getSizeInMetersX(), end_x);
  end_y = std::min(origin_y_ + getSizeInMetersY(), end_y);

  unsigned int map_sx, map_sy, map_ex, map_ey;

  if (!worldToMap(start_x, start_y, map_sx, map_sy) || !worldToMap(end_x, end_y, map_ex, map_ey)) {
    return;
  }
  
  unsigned int index = getIndex(map_sx, map_sy);
  unsigned char* current = &costmap_[index];
  for (unsigned int j = map_sy; j <= map_ey; ++j) {
    for (unsigned int i = map_sx; i <= map_ex; ++i) {
      if ( *current != costmap_2d::LETHAL_OBSTACLE ) {
        if ( clear_no_info || *current != costmap_2d::NO_INFORMATION ) {
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

void ObstaclesLayer::raytraceFreespace(const costmap_2d::Observation& clearing_observation,
                                       double* min_x, double* min_y, double* max_x, double* max_y)
{
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
        "The origin for the sensor at (%.2f, %.2f, %.2f) is out of map bounds. So, the costmap cannot raytrace for it.",
        ox, oy, oz);
    return;
  }

  double map_end_x = origin_x_ + getSizeInMetersX();
  double map_end_y = origin_y_ + getSizeInMetersY();

  for (unsigned int i = 0; i < clearing_observation.cloud_->points.size(); ++i) {
    double wpx = clearing_observation.cloud_->points[i].x;
    double wpy = clearing_observation.cloud_->points[i].y;
    double wpz = clearing_observation.cloud_->points[i].z;

    double distance = dist(ox, oy, oz, wpx, wpy, wpz);
    double scaling_fact = 1.0;
    scaling_fact = std::max(std::min(scaling_fact, (distance - 2 * resolution_) / distance), 0.0);
    wpx = scaling_fact * (wpx - ox) + ox;
    wpy = scaling_fact * (wpy - oy) + oy;
    wpz = scaling_fact * (wpz - oz) + oz;

    double a = wpx - ox;
    double b = wpy - oy;
    double c = wpz - oz;
    double t = 1.0;

    if ( wpz > max_obstacle_height_ ) {
      t = std::max(0.0, std::min(t, (max_obstacle_height_ - 0.01 - oz) / c));
    } else if ( wpz < origin_z_ ) {
      t = std::min(t, (origin_z_ - oz) / c);
    }

    if ( wpx < origin_x_ ) {
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
      unsigned int cell_raytrace_range = cellDistance(clearing_observation.raytrace_range_);

      voxel_grid_.clearVoxelLineInMap(sensor_x, sensor_y, sensor_z, point_x, point_y, point_z, costmap_,
                                      unknown_threshold_, mark_threshold_, costmap_2d::FREE_SPACE,
                                      costmap_2d::NO_INFORMATION,cell_raytrace_range);

      updateRaytraceBounds(ox, oy, wpx, wpy, clearing_observation.raytrace_range_, min_x, min_y, max_x, max_y);
    }
  }
}

inline void ObstaclesLayer::enqueue( unsigned char* grid, unsigned int index, unsigned int mx, unsigned int my,
                                     unsigned int src_x, unsigned int src_y )
{ 
  //set the cost of the cell being inserted
  if ( !seen_[index] ) {
    //we compute our distance table one cell further than the inflation radius dictates so we can make the check below
    double distance = distanceLookup(mx, my, src_x, src_y);

    //we only want to put the cell in the queue if it is within the inflation radius of the obstacle point
    if (distance > cell_inflation_radius_) {
      return;
    }

    //assign the cost associated with the distance from an obstacle to the cell
    unsigned char cost = costLookup(mx, my, src_x, src_y);
    unsigned char old_cost = grid[index];

    if ( old_cost == costmap_2d::NO_INFORMATION && cost >= costmap_2d::INSCRIBED_INFLATED_OBSTACLE ) {
      grid[index] = cost;
    } else {
      grid[index] = std::max(old_cost, cost);
    }
    //push the cell data onto the queue and mark
    seen_[index] = true;
    CellData data(distance, index, mx, my, src_x, src_y);
    inflation_queue_.push(data);
  }
}

void ObstaclesLayer::computeCaches( void )
{
  if ( cell_inflation_radius_ == 0 ) {
    return;
  }

  //based on the inflation radius... compute distance and cost caches
  if ( cell_inflation_radius_ != cached_cell_inflation_radius_ ) {
    if( cached_cell_inflation_radius_ > 0 ) {
      deleteKernels();
    }

    cached_costs_ = new unsigned char*[cell_inflation_radius_ + 2];
    cached_distances_ = new double*[cell_inflation_radius_ + 2];

    for (unsigned int i = 0; i <= cell_inflation_radius_ + 1; ++i) {
      cached_costs_[i] = new unsigned char[cell_inflation_radius_ + 2];
      cached_distances_[i] = new double[cell_inflation_radius_ + 2];
      for (unsigned int j = 0; j <= cell_inflation_radius_ + 1; ++j) {
        cached_distances_[i][j] = hypot(i, j);
      }
    }

    cached_cell_inflation_radius_ = cell_inflation_radius_;
  }

  for (unsigned int i = 0; i <= cell_inflation_radius_ + 1; ++i) {
    for (unsigned int j = 0; j <= cell_inflation_radius_ + 1; ++j) {
      cached_costs_[i][j] = computeCost(cached_distances_[i][j]);
    }
  }
}

void ObstaclesLayer::deleteKernels()
{
  if ( cached_distances_ != NULL ) {
    for (unsigned int i = 0; i <= cached_cell_inflation_radius_ + 1; ++i) {
      delete[] cached_distances_[i];
    }
    delete[] cached_distances_;
  }

  if (cached_costs_ != NULL) {
    for (unsigned int i = 0; i <= cached_cell_inflation_radius_ + 1; ++i) {
      delete[] cached_costs_[i];
    }
    delete[] cached_costs_;
  }
}

void ObstaclesLayer::onFootprintChanged( void )
{
  inscribed_radius_ = robot_link_radius_;
  cell_inflation_radius_ = cellDistance( inflation_radius_ );
  computeCaches();
  need_reinflation_ = true;
}

}  // namespace squirrel_navigation

// 
// ObstaclesLayer.cpp ends here
