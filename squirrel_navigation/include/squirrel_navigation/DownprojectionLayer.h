 // DownprojectionLayer.h --- 
// 
// Filename: DownprojectionLayer.h
// Description: Dynamic mapping of obstacles with RGBD
//              and Laser sensors
// Author: Federico Boniardi
// Maintainer: boniardi@cs.uni-freiburg.de
// Created: Wed Nov 19 18:57:41 2014 (+0100)
// Version: 0.1.0
// Last-Updated: Tue Nov 24 14:35:57 2015 (+0100)
//           By: Federico Boniardi
//     Update #: 4
// URL: 
// Keywords: 
// Compatibility: 
//   ROS Hydro, ROS Indigo
// 

// Commentary: 
//   The code therein is an improvement of costmap_2d::VoxelLayer which
//   is distributed by the authors under BSD license, that is below reported
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
//   RGBD source: ASUS Xtion pro
//      
//      

// Code:

#ifndef SQUIRREL_NAVIGATION_DOWNPROJECTIONLAYER_H_
#define SQUIRREL_NAVIGATION_DOWNPROJECTIONLAYER_H_

#include <ros/ros.h>

#include <dynamic_reconfigure/server.h>

#include <costmap_2d/observation_buffer.h>
#include <costmap_2d/VoxelGrid.h>
#include <costmap_2d/obstacle_layer.h>

#include <dynamixel_msgs/JointState.h>
#include <std_msgs/Float64.h>

#include <voxel_grid/voxel_grid.h>

#include <pluginlib/class_list_macros.h>

#include <pcl_conversions/pcl_conversions.h>

#include <cmath>
#include <map>
#include <set>

#include "squirrel_navigation/FootprintLayer.h"
#include "squirrel_navigation/DownprojectionLayerPluginConfig.h"
#include "squirrel_navigation/JointHandle.h"
#include "squirrel_navigation/Common.h"

namespace squirrel_navigation {

class DownprojectionLayer : public costmap_2d::ObstacleLayer
{
public:
  DownprojectionLayer( void );
  virtual ~DownprojectionLayer( void );

  virtual void onInitialize( void );
  virtual void updateBounds( double, double, double, double*, double*, double*, double* );
  virtual void updateOrigin( double, double );
  virtual void matchSize( void );
  virtual void reset( void );
  inline bool isDiscretized( void ) { return true; };
  
protected:
  FootprintLayer footprint_layer_;

  virtual void setupDynamicReconfigure( ros::NodeHandle& );
  virtual void resetMaps( void );

private:
  dynamic_reconfigure::Server<DownprojectionLayerPluginConfig> *dsrv_;

  // time based costmap layer
  std::map<unsigned int, ros::Time> clearing_index_stamped_;

  ros::Publisher voxel_pub_, clearing_endpoints_pub_;
  
  voxel_grid::VoxelGrid voxel_grid_;
  double z_resolution_, origin_z_;

  unsigned int unknown_threshold_, mark_threshold_, size_z_;

  sensor_msgs::PointCloud clearing_endpoints_;
  
  double robot_diameter_, robot_height_;
  double floor_threshold_,  obstacles_persistence_;

  // TiltHandle kinect_th_;
  std::map<std::string,JointHandle> kinect_jh_;
  
  void reconfigureCallback_( DownprojectionLayerPluginConfig& , uint32_t );
  void clearNonLethal_( double, double, double, double, bool );
  void raytraceFreespace_( const costmap_2d::Observation&, double*, double*, double*, double* );

  inline bool worldToMap3DFloat_( double wx, double wy, double wz, double& mx, double& my, double& mz )
  {
    if ( wx < origin_x_ or wy < origin_y_ or wz < origin_z_ ) {
      return false;
    }

    mx = ((wx - origin_x_) / resolution_);
    my = ((wy - origin_y_) / resolution_);
    mz = ((wz - origin_z_) / z_resolution_);

    if (mx < size_x_ and my < size_y_ and mz < size_z_) {
      return true;
    }
    
    return false;
  };

  inline bool worldToMap3D_( double wx, double wy, double wz, unsigned int& mx, unsigned int& my, unsigned int& mz )
  {
    if ( wx < origin_x_ or wy < origin_y_ or wz < origin_z_ ) {
      return false;
    }
    
    mx = (int)((wx - origin_x_) / resolution_);
    my = (int)((wy - origin_y_) / resolution_);
    mz = (int)((wz - origin_z_) / z_resolution_);

    if (mx < size_x_ and my < size_y_ and mz < size_z_) {
      return true;
    }

    return false;
  };

  inline void mapToWorld3D_( unsigned int mx, unsigned int my, unsigned int mz, double& wx, double& wy, double& wz )
  {
    wx = origin_x_ + (mx + 0.5) * resolution_;
    wy = origin_y_ + (my + 0.5) * resolution_;
    wz = origin_z_ + (mz + 0.5) * z_resolution_;
  };

  inline double dist_( double x0, double y0, double z0, double x1, double y1, double z1 )
  {
    return std::sqrt((x1-x0)*(x1-x0) + (y1-y0)*(y1-y0) + (z1-z0)*(z1-z0));
  };
};

}  // namespace squirrel_navigation

#endif  // SQUIRREL_NAVIGATION_DOWNPROJECTIONLAYER_H_

// 
// DownprojectionLayer.h ends here
