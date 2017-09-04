// Copyright (c) 2017, Federico Boniardi and Wolfram Burgard
// All rights reserved.
// 
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
// 
// * Redistributions of source code must retain the above copyright notice, this
//   list of conditions and the following disclaimer.
// 
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
// 
// * Neither the name of the University of Freiburg nor the names of its
//   contributors may be used to endorse or promote products derived from
//   this software without specific prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#ifndef SQUIRREL_POINTCLOUD_FILTER_POINTCLOUD_FILTER_H_
#define SQUIRREL_POINTCLOUD_FILTER_POINTCLOUD_FILTER_H_

#include "squirrel_pointcloud_filter/PointCloudFilterConfig.h"

#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>

#include <dynamic_reconfigure/server.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/filters/filter.h>
#include <pcl_ros/filters/voxel_grid.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

#include <tf/transform_listener.h>

#include <array>
#include <memory>
#include <string>

namespace squirrel_pointcloud_filter {

class PointCloudFilter {
 public:
  class Params {
   public:
    static Params defaultParams();

    std::string global_frame_id;
    bool nanfree, do_ground_segmentation, do_voxel_filter;
    bool ground_filter_voxelized, ground_pcls_voxelized;
    double update_rate;
    std::array<double, 3> resolutions_xyz;
    double ground_threshold;
  };

 public:
  PointCloudFilter() : params_(Params::defaultParams()) { initialize(); }
  PointCloudFilter(const Params& params) : params_(params) { initialize(); }
  virtual ~PointCloudFilter() {}

  // Spinner.
  void spin();

 private:
  // Initialize.
  void initialize();

  // Callbacks.
  void reconfigureCallback(PointCloudFilterConfig& config, uint32_t level);
  void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg_in);

  // Filter the ground.
  void segmentGround(
      const pcl::PointCloud<pcl::PointXYZ>& cloud,
      pcl::PointCloud<pcl::PointXYZ>* ground,
      pcl::PointCloud<pcl::PointXYZ>* nonground) const;

 private:
  Params params_;
  std::unique_ptr<dynamic_reconfigure::Server<PointCloudFilterConfig>> dsrv_;

  ros::Publisher voxel_pcl_pub_, ground_pcl_pub_, nonground_pcl_pub_;
  ros::Subscriber pcl_sub_;

  tf::TransformListener tfl_;
  
  std::unique_ptr<pcl::VoxelGrid<pcl::PointXYZ>> voxel_filter_;
};

}  // namespace squirrel_pointcloud_filter

#endif /* SQUIRREL_POINTCLOUD_FILTER_POINTCLOUD_FILTER_H_ */
