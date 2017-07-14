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

#include "squirrel_pointcloud_filter/pointcloud_filter.h"

#include <stdexcept>
#include <string>
#include <vector>

namespace squirrel_pointcloud_filter {

void PointCloudFilter::spin() {
  ros::Rate lr(params_.update_rate);
  while (ros::ok())
    try {
      ros::spinOnce();
      lr.sleep();
    } catch (const std::runtime_error& err) {
      const std::string& node_name = ros::this_node::getName();
      ROS_ERROR_STREAM(node_name << ": " << err.what());
    }
}

void PointCloudFilter::initialize() {
  ros::NodeHandle pnh("~"), gnh;
  // Initialize the parameter server.
  dsrv_.reset(new dynamic_reconfigure::Server<PointCloudFilterConfig>(pnh));
  dsrv_->setCallback(
      boost::bind(&PointCloudFilter::reconfigureCallback, this, _1, _2));
  // Initialize publisher and subscribers.
  pcl_pub_ = gnh.advertise<sensor_msgs::PointCloud2>("/cloud_out", 1);
  pcl_sub_ = gnh.subscribe(
      "/cloud_in", 1, &PointCloudFilter::pointCloudCallback, this);
}

void PointCloudFilter::reconfigureCallback(
    PointCloudFilterConfig& config, uint32_t level) {
  params_.nanfree            = config.nanfree;
  params_.update_rate        = config.update_rate_hz;
  params_.resolutions_xyz[0] = config.resolution_x;
  params_.resolutions_xyz[1] = config.resolution_y;
  params_.resolutions_xyz[2] = config.resolution_z;
}

void PointCloudFilter::pointCloudCallback(
    const sensor_msgs::PointCloud2::ConstPtr& msg_in) {
  typedef pcl::PointCloud<pcl::PointXYZ> PCLPointCloud;
  // Input pointcloud.
  PCLPointCloud::Ptr pointcloud_raw(new PCLPointCloud);
  pcl::fromROSMsg(*msg_in, *pointcloud_raw);
  // Filter NaN if needed.
  if (!params_.nanfree) {
    std::vector<int> nan_filter;
    pcl::removeNaNFromPointCloud(*pointcloud_raw, *pointcloud_raw, nan_filter);
  }
  // Apply voxel filter.
  PCLPointCloud::Ptr pointcloud_filtered(new PCLPointCloud);
  voxel_filter_.reset(new pcl::VoxelGrid<pcl::PointXYZ>);
  voxel_filter_->setInputCloud(pointcloud_raw);
  voxel_filter_->setLeafSize(
      params_.resolutions_xyz[0], params_.resolutions_xyz[1],
      params_.resolutions_xyz[2]);
  voxel_filter_->filter(*pointcloud_filtered);
  // Publish the filtered pointcloud.
  sensor_msgs::PointCloud2 msg_out;
  msg_out.header = msg_in->header;
  pcl::toROSMsg(*pointcloud_filtered, msg_out);
  pcl_pub_.publish(msg_out);
  // Info.
  ROS_INFO_STREAM_ONCE(
      ros::this_node::getName() << ": Subscribing to the pointcloud.");
}

PointCloudFilter::Params PointCloudFilter::Params::defaultParams() {
  Params params;
  params.nanfree         = true;
  params.update_rate     = 10.0;
  params.resolutions_xyz = {0.05, 0.05, 0.05};
  return params;
}

}  // namespace squirrel_pointcloud_filter
