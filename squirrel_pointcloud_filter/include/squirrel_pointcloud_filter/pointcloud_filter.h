// The MIT License (MIT)
//
// Copyright (c) 2017 Federico Boniardi and Wolfram Burgard
//
// Permission is hereby granted, free of charge, to any person obtaining
// a copy of this software and associated documentation files (the
// "Software"), to deal in the Software without restriction, including
// without limitation the rights to use, copy, modify, merge, publish,
// distribute, sublicense, and/or sell copies of the Software, and to
// permit persons to whom the Software is furnished to do so, subject to
// the following conditions:
//
// The above copyright notice and this permission notice shall be
// included in all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
// EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
// MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
// NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE
// LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
// OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
// WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

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

#include <memory>
#include <array>

namespace squirrel_pointcloud_filter {

class PointCloudFilter {
 public:
  class Params {
   public:
    static Params defaultParams();

    bool nanfree;
    double update_rate;
    std::array<double, 3> resolutions_xyz;
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
  
 private:
  Params params_;
  std::unique_ptr<dynamic_reconfigure::Server<PointCloudFilterConfig>> dsrv_;

  ros::Publisher pcl_pub_;
  ros::Subscriber pcl_sub_;
  
  std::unique_ptr<pcl::VoxelGrid<pcl::PointXYZ>> voxel_filter_;
};

}  // namespace squirrel_pointcloud_filter

#endif /* SQUIRREL_POINTCLOUD_FILTER_POINTCLOUD_FILTER_H_ */
