// MapLayer.cpp --- 
// 
// Filename: MapLayer.cpp
// Description: Costmap layer related to static obstacles
// Author: Federico Boniardi
// Maintainer: boniardi@cs.uni-freiburg.de
// Created: Tue Feb 3 10:49:46 2015 (+0100)
// Version: 0.1.0
// Last-Updated: 
//           By: Federico Boniardi
//     Update #: 0
// URL: 
// Keywords: 
// Compatibility: 
//   ROS Hydro, ROS Indigo
// 

// Commentary: 
//   The code therein is an integration of costmap_2d::InflationLayer into
//   costmap_2d::StaticLayer. Both source codes are distributed by the authors
//   under BSD license, that is below reported
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

#include "squirrel_navigation/MapLayer.h"

#include <costmap_2d/static_layer.h>
#include <costmap_2d/costmap_math.h>

#include <pluginlib/class_list_macros.h>

#include <cmath>

PLUGINLIB_EXPORT_CLASS(squirrel_navigation::MapLayer, costmap_2d::Layer)

namespace squirrel_navigation {

MapLayer::MapLayer( void ) :
    inflation_radius_(0.0),
    weight_(0.0),
    cell_inflation_radius_(0.0),
    cached_cell_inflation_radius_(0.0),
    dsrv_(NULL)
{
  access_ = new boost::shared_mutex();
}

MapLayer::~MapLayer( void )
{
  deleteKernels();
  if( dsrv_ ) {
    delete dsrv_;
  }
}

void MapLayer::onInitialize( void )
{
  ros::NodeHandle nh("~/" + name_), g_nh;
  current_ = true;

  global_frame_ = layered_costmap_->getGlobalFrameID();

  std::string map_topic;
  nh.param("map_topic", map_topic, std::string("map"));
  nh.param("subscribe_to_updates", subscribe_to_updates_, false);
  
  nh.param("track_unknown_space", track_unknown_space_, true);
  nh.param("use_maximum", use_maximum_, false);

  int temp_lethal_threshold, temp_unknown_cost_value;
  nh.param("lethal_cost_threshold", temp_lethal_threshold, int(100));
  nh.param("unknown_cost_value", temp_unknown_cost_value, int(-1));
  nh.param("trinary_costmap", trinary_costmap_, true);

  lethal_threshold_ = std::max(std::min(temp_lethal_threshold, 100), 0);
  unknown_cost_value_ = temp_unknown_cost_value;

  ROS_INFO("Requesting the map...");
  map_sub_ = g_nh.subscribe(map_topic, 1, &MapLayer::incomingMap, this);
  map_received_ = false;
  has_updated_data_ = false;

  ros::Rate r(10);
  while ( !map_received_ && g_nh.ok() ) {
    ros::spinOnce();
    r.sleep();
  }

  ROS_INFO("Received a %d X %d map at %f m/pix", getSizeInCellsX(), getSizeInCellsY(), getResolution());
  
  if( subscribe_to_updates_ ) {
    ROS_INFO("Subscribing to updates");
    map_update_sub_ = g_nh.subscribe(map_topic + "_updates", 10, &MapLayer::incomingUpdate, this);
  }

  if( dsrv_ ) {
    delete dsrv_;
  }

  {
    boost::unique_lock < boost::shared_mutex > lock(*access_);
    current_ = true;
    seen_ = NULL;
    need_reinflation_ = false;
    
    dynamic_reconfigure::Server<MapLayerPluginConfig>::CallbackType cb = boost::bind(
        &MapLayer::reconfigureCB, this, _1, _2);
    
    if( dsrv_ != NULL ){
      dsrv_->clearCallback();
      dsrv_->setCallback(cb);
    }
    else
    {
      dsrv_ = new dynamic_reconfigure::Server<MapLayerPluginConfig>(ros::NodeHandle("~/" + name_));
      dsrv_->setCallback(cb);
    }
  }

  inflationMatchSize();
}

void MapLayer::reconfigureCB( MapLayerPluginConfig &config, uint32_t level )
{
  if ( weight_ != config.cost_scaling_factor || inflation_radius_ != config.inflation_radius ) {
    inflation_radius_ = config.inflation_radius;
    cell_inflation_radius_ = cellDistance(inflation_radius_);
    weight_ = config.cost_scaling_factor;
    need_reinflation_ = true;
    computeCaches();
  }

  robot_link_radius_ = config.robot_link_radius;
  inscribed_radius_ = robot_link_radius_;
  
  if ( config.enabled != enabled_ ) {
    enabled_ = config.enabled;
    has_updated_data_ = true;
    x_ = y_ = 0;
    width_ = size_x_;
    height_ = size_y_;
    need_reinflation_ = true;
  }
}

void MapLayer::inflationMatchSize( void )
{
  boost::unique_lock < boost::shared_mutex > lock(*access_);
  costmap_2d::Costmap2D* costmap = layered_costmap_->getCostmap();
  resolution_ = costmap->getResolution();
  cell_inflation_radius_ = cellDistance(inflation_radius_);
  computeCaches();
  
  unsigned int size_x = costmap->getSizeInCellsX(), size_y = costmap->getSizeInCellsY();
  if ( seen_ ) {
    delete seen_;
  }
  seen_ = new bool[size_x * size_y];
}

void MapLayer::mapMatchSize( void )
{
  Costmap2D* master = layered_costmap_->getCostmap();
  resizeMap(master->getSizeInCellsX(), master->getSizeInCellsY(), master->getResolution(),
            master->getOriginX(), master->getOriginY());
}

void MapLayer::onFootprintChanged( void )
{
  inscribed_radius_ = robot_link_radius_;
  cell_inflation_radius_ = cellDistance( inflation_radius_ );
  computeCaches();
  need_reinflation_ = true;
}

unsigned char MapLayer::interpretValue( unsigned char value )
{
  if ( track_unknown_space_ && value == unknown_cost_value_ ) {
    return costmap_2d::NO_INFORMATION;
  } else if ( value >= lethal_threshold_ ) {
    return costmap_2d::LETHAL_OBSTACLE;
  } else if ( trinary_costmap_ ) {
    return costmap_2d::FREE_SPACE;
  }
  
  double scale = (double) value / lethal_threshold_;
  return scale * costmap_2d::LETHAL_OBSTACLE;
}

void MapLayer::incomingMap(const nav_msgs::OccupancyGridConstPtr& new_map)
{
  unsigned int size_x = new_map->info.width, size_y = new_map->info.height;

  ROS_DEBUG("Received a %d X %d map at %f m/pix", size_x, size_y, new_map->info.resolution);

  Costmap2D* master = layered_costmap_->getCostmap();
  if ( master->getSizeInCellsX() != size_x || master->getSizeInCellsY() != size_y ||
       master->getResolution() != new_map->info.resolution || master->getOriginX() != new_map->info.origin.position.x ||
       master->getOriginY() != new_map->info.origin.position.y || !layered_costmap_->isSizeLocked() ){
    ROS_INFO("Resizing costmap to %d X %d at %f m/pix", size_x, size_y, new_map->info.resolution);
    layered_costmap_->resizeMap(size_x, size_y, new_map->info.resolution, new_map->info.origin.position.x, new_map->info.origin.position.y, true);
  } else if ( size_x_ != size_x || size_y_ != size_y ||
              resolution_ != new_map->info.resolution ||
              origin_x_ != new_map->info.origin.position.x ||
              origin_y_ != new_map->info.origin.position.y ) {
    mapMatchSize();
  }

  unsigned int index = 0;

  for (unsigned int i = 0; i < size_y; ++i) {
    for (unsigned int j = 0; j < size_x; ++j) {
      unsigned char value = new_map->data[index];
      unsigned char costmap_value = interpretValue(value);
      if ( costmap_value == costmap_2d::LETHAL_OBSTACLE ) {
        obstacles_index_.insert(index);
      }
      costmap_[index] = costmap_value;
      ++index;
    }
  }
  
  x_ = y_ = 0;
  width_ = size_x_;
  height_ = size_y_;
  map_received_ = true;
  has_updated_data_ = true;
}

void MapLayer::incomingUpdate(const map_msgs::OccupancyGridUpdateConstPtr& update) {
  unsigned int di = 0;
  for (unsigned int y = 0; y < update->height ; y++) {
    unsigned int index_base = (update->y + y) * update->width;
    for (unsigned int x = 0; x < update->width ; x++) {
      unsigned int index = index_base + x + update->x;
      costmap_[index] = interpretValue( update->data[di++] );
    }
  }
  
  x_ = update->x;
  y_ = update->y;
  width_ = update->width;
  height_ = update->height;
  has_updated_data_ = true;
}

void MapLayer::activate( void )
{
  onInitialize();
}

void MapLayer::deactivate( void )
{
  map_sub_.shutdown();
  if (subscribe_to_updates_)
    map_update_sub_.shutdown();
}

void MapLayer::reset( void )
{
  deactivate();
  activate();
}

void MapLayer::updateBounds(double robot_x, double robot_y, double robot_yaw,
                            double* min_x, double* min_y, double* max_x, double* max_y)
{
  if ( !map_received_ || !(has_updated_data_ || has_extra_bounds_) ) {
    return;
  }
  
  useExtraBounds(min_x, min_y, max_x, max_y);

  double mx, my;
  
  mapToWorld(x_, y_, mx, my);
  *min_x = std::min(mx, *min_x);
  *min_y = std::min(my, *min_y);
  
  mapToWorld(x_ + width_, y_ + height_, mx, my);
  *max_x = std::max(mx, *max_x);
  *max_y = std::max(my, *max_y);
  
  has_updated_data_ = false;

  if( need_reinflation_ ) {
    *min_x = -std::numeric_limits<float>::max();
    *min_y = -std::numeric_limits<float>::max();
    *max_x = std::numeric_limits<float>::max();
    *max_y = std::numeric_limits<float>::max();
    need_reinflation_ = false;
  }
}

void MapLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j)
{
  if (!map_received_) {
    return;
  }

  if (!use_maximum_) {
    updateWithTrueOverwrite(master_grid, min_i, min_j, max_i, max_j);
  } else {
    updateWithMax(master_grid, min_i, min_j, max_i, max_j);
  }

  boost::unique_lock < boost::shared_mutex > lock(*access_);
  
  ROS_ASSERT_MSG(inflation_queue_.empty(), "The inflation queue must be empty at the beginning of inflation");

  unsigned char* master_array = master_grid.getCharMap();
  unsigned int size_x = master_grid.getSizeInCellsX(), size_y = master_grid.getSizeInCellsY();

  memset(seen_, false, size_x * size_y * sizeof(bool));

  min_i -= cell_inflation_radius_;
  min_j -= cell_inflation_radius_;
  max_i += cell_inflation_radius_;
  max_j += cell_inflation_radius_;

  min_i = std::max( 0, min_i );
  min_j = std::max( 0, min_j );
  max_i = std::min( int( size_x ), max_i );
  max_j = std::min( int( size_y ), max_j );

  for (int j = min_j; j < max_j; j++) {
    for (int i = min_i; i < max_i; i++) {
      int index = master_grid.getIndex(i, j);
      unsigned char cost = master_array[index];
      bool is_static_obstacle = obstacles_index_.find(index) != obstacles_index_.end();
      if (cost == costmap_2d::LETHAL_OBSTACLE && is_static_obstacle ) {
        enqueue(master_array, index, i, j, i, j);
      }
    }
  }

  while ( !inflation_queue_.empty() ) {
    const CellData& current_cell = inflation_queue_.top();
    
    unsigned int index = current_cell.index_;
    unsigned int mx = current_cell.x_;
    unsigned int my = current_cell.y_;
    unsigned int sx = current_cell.src_x_;
    unsigned int sy = current_cell.src_y_;

    inflation_queue_.pop();

    if ( mx > 0 ) {
      enqueue(master_array, index - 1, mx - 1, my, sx, sy);
    }
    
    if ( my > 0 ) {
      enqueue(master_array, index - size_x, mx, my - 1, sx, sy);
    }

    if ( mx < size_x - 1 ) {
      enqueue(master_array, index + 1, mx + 1, my, sx, sy);
    }

    if (my < size_y - 1) {
      enqueue(master_array, index + size_x, mx, my + 1, sx, sy);
    }
  }
}

inline void MapLayer::enqueue(unsigned char* grid, unsigned int index, unsigned int mx, unsigned int my,
                              unsigned int src_x, unsigned int src_y)
{
  if ( !seen_[index] ) {
    double distance = distanceLookup(mx, my, src_x, src_y);

    if ( distance > cell_inflation_radius_ ) {
      return;
    }

    unsigned char cost = costLookup(mx, my, src_x, src_y);
    unsigned char old_cost = grid[index];

    if ( old_cost == costmap_2d::NO_INFORMATION && cost >= costmap_2d::INSCRIBED_INFLATED_OBSTACLE ) {
      grid[index] = cost;
    } else {
      grid[index] = std::max(old_cost, cost);
    }
    seen_[index] = true;
    CellData data(distance, index, mx, my, src_x, src_y);
    inflation_queue_.push(data);
  }
}

void MapLayer::computeCaches( void )
{
  if( cell_inflation_radius_ == 0 ) {
    return;
  }
    
  if( cell_inflation_radius_ != cached_cell_inflation_radius_ ) {
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

void MapLayer::deleteKernels( void )
{
  if ( cached_distances_ != NULL ) {
    for (unsigned int i = 0; i <= cached_cell_inflation_radius_ + 1; ++i) {
      delete[] cached_distances_[i];
    }
    delete[] cached_distances_;
  }

  if ( cached_costs_ != NULL ) {
    for (unsigned int i = 0; i <= cached_cell_inflation_radius_ + 1; ++i) {
      delete[] cached_costs_[i];
    }
    delete[] cached_costs_;
  }
}

}  // namespace squirrel_navigation

// 
// MapLayer.cpp ends here
