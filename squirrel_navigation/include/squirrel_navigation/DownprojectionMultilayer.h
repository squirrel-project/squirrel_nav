// DownprojectionMultilayer.h ---
//
// Filename: DownprojectionMultilayer.h
// Description: Dynamic mapping of obstacles with RGBD
//              and Laser sensors
// Author: Federico Boniardi
// Maintainer: boniardi@cs.uni-freiburg.de
// Created: Wed Feb 4 10:14:10 2015 (+0100)
// Version: 0.1.0
// Last-Updated: Tue Nov 24 14:36:10 2015 (+0100)
//           By: Federico Boniardi
//     Update #: 6
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

#ifndef SQUIRREL_NAVIGATION_DOWNPROJECTIONMULTILAYER_H_
#define SQUIRREL_NAVIGATION_DOWNPROJECTIONMULTILAYER_H_

#include <ros/ros.h>

#include <costmap_2d/layered_costmap.h>
#include <costmap_2d/observation_buffer.h>
#include <costmap_2d/obstacle_layer.h>

#include <voxel_grid/voxel_grid.h>

#include <dynamic_reconfigure/server.h>

#include <pluginlib/class_list_macros.h>

#include <pcl_conversions/pcl_conversions.h>

#include "squirrel_navigation/Common.h"
#include "squirrel_navigation/CostmapUpdateHandle.h"
#include "squirrel_navigation/DownprojectionMultilayerPluginConfig.h"
#include "squirrel_navigation/JointHandle.h"
#include "squirrel_navigation/MultiInflatedLayer.h"
#include "squirrel_navigation/RobotMultiFootprint.h"

#include <algorithm>
#include <cmath>
#include <map>
#include <sstream>
#include <vector>

namespace squirrel_navigation {

class DownprojectionMultilayer : public MultiInflatedLayer {
 public:
  DownprojectionMultilayer(void);
  virtual ~DownprojectionMultilayer(void);
  virtual void onInitialize(void);
  virtual void updateBounds(
      double, double, double, double*, double*, double*, double*);
  virtual void updateCosts(costmap_2d::Costmap2D&, int, int, int, int);
  void updateOrigin(double, double);
  bool isDiscretized(void);
  virtual void matchSize(void);
  virtual void reset(void);

 protected:
  RobotMultiFootprint footprint_;

  virtual void setupDynamicReconfigure(ros::NodeHandle&);
  virtual void resetMaps(void);

  class ParameterParser {
   public:
    template <typename T>
    static inline std::vector<T> array(std::string input) {
      std::vector<T> output;
      std::stringstream ss(input);
      std::string token;

      for (unsigned int i = 0; std::getline(ss, token, ','); ++i) {
        output.resize(output.size() + 1);
        std::stringstream ss(token);
        ss >> output[i];
      }

      return output;
    };
  };

 private:
  dynamic_reconfigure::Server<DownprojectionMultilayerPluginConfig>* dsrv_;

  // time based costmap layer
  std::map<unsigned int, ros::Time> clearing_index_stamped_;

  ros::NodeHandle nh_;
  ros::Publisher voxel_pub_, clearing_endpoints_pub_;
  ros::Subscriber tilt_state_sub_, tilt_command_sub_;

  voxel_grid::VoxelGrid voxel_grid_;
  double z_resolution_, origin_z_;

  bool verbose_;
  unsigned int unknown_threshold_, mark_threshold_, size_z_;
  double max_obstacle_height_, min_obstacle_height_, obstacles_persistence_;

  sensor_msgs::PointCloud clearing_endpoints_;

  std::vector<double> robot_link_radii_;
  std::vector<double> layers_levels_;
  std::map<unsigned int, bool> observed_;

  CostmapUpdateHandle* costmap_update_handle_;
  JointHandle kinect_tilt_h_, kinect_pan_h_;

  void reconfigureCB(DownprojectionMultilayerPluginConfig&, uint32_t);
  void clearNonLethal(double, double, double, double, bool);
  virtual void raytraceFreespace(
      const costmap_2d::Observation&, double*, double*, double*, double*);

  inline bool worldToMap3DFloat(
      double wx, double wy, double wz, double& mx, double& my, double& mz) {
    if (wx < origin_x_ or wy < origin_y_ or wz < origin_z_)
      return false;

    mx = ((wx - origin_x_) / resolution_);
    my = ((wy - origin_y_) / resolution_);
    mz = ((wz - origin_z_) / z_resolution_);

    if (mx < size_x_ and my < size_y_ and mz < size_z_)
      return true;

    return false;
  };

  inline bool worldToMap3D(
      double wx, double wy, double wz, unsigned int& mx, unsigned int& my,
      unsigned int& mz) {
    if (wx < origin_x_ or wy < origin_y_ or wz < origin_z_)
      return false;

    mx = (int)((wx - origin_x_) / resolution_);
    my = (int)((wy - origin_y_) / resolution_);
    mz = (int)((wz - origin_z_) / z_resolution_);

    if (mx < size_x_ and my < size_y_ and mz < size_z_)
      return true;

    return false;
  };

  inline void mapToWorld3D(
      unsigned int mx, unsigned int my, unsigned int mz, double& wx, double& wy,
      double& wz) {
    wx = origin_x_ + (mx + 0.5) * resolution_;
    wy = origin_y_ + (my + 0.5) * resolution_;
    wz = origin_z_ + (mz + 0.5) * z_resolution_;
  };

  inline double linearDistance(
      double x0, double y0, double z0, double x1, double y1, double z1) {
    const double dx = x1 - x0, dy = y1 - y0, dz = z1 - z0;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
  };

  inline unsigned int layer(double z) {
    if (z < layers_levels_.front()) {
      return 0;
    } else if (z >= layers_levels_.back()) {
      return (unsigned int)num_layers_ - 1;
    } else {
      for (unsigned int i = 0; i < num_layers_ - 1; ++i) {
        if (z >= layers_levels_[i] && z < layers_levels_[i + 1]) {
          return i;
        }
      }
    }
  };
};

}  // namespace squirrel_navigation

#endif  // SQUIRREL_NAVIGATION_DOWNPROJECTIONMULTILAYER_H_
