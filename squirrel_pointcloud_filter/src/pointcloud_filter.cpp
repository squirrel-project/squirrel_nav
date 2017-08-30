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
  for (ros::Rate lr(params_.update_rate); ros::ok(); lr.sleep())
    try {
      ros::spinOnce();
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
  pcl_sub_ = gnh.subscribe(
      "/cloud_in", 1, &PointCloudFilter::pointCloudCallback, this);
  voxel_pcl_pub_ =
      pnh.advertise<sensor_msgs::PointCloud2>("/voxelized_cloud_out", 1);
  ground_pcl_pub_ =
      pnh.advertise<sensor_msgs::PointCloud2>("/ground_cloud_out", 1);
  nonground_pcl_pub_ =
      pnh.advertise<sensor_msgs::PointCloud2>("/nonground_cloud_out", 1);
  // Wait for ros crap to boot.
  ros::Duration(0.5).sleep();
}

void PointCloudFilter::reconfigureCallback(
    PointCloudFilterConfig& config, uint32_t level) {
  params_.global_frame_id        = config.global_frame_id;
  params_.nanfree                = config.nanfree;
  params_.update_rate            = config.update_rate_hz;
  params_.do_voxel_filter        = config.do_voxel_filter;
  params_.ground_pcls_voxelized  = config.ground_pcls_voxelized;
  params_.do_ground_segmentation = config.do_ground_segmentation;
  params_.resolutions_xyz[0]     = config.resolution_x;
  params_.resolutions_xyz[1]     = config.resolution_y;
  params_.resolutions_xyz[2]     = config.resolution_z;
  params_.ground_threshold       = config.ground_threshold;
}

void PointCloudFilter::pointCloudCallback(
    const sensor_msgs::PointCloud2::ConstPtr& msg_in) {
  ROS_INFO_STREAM_ONCE(
      ros::this_node::getName() << ": Subscribing to the pointcloud.");
  // Input pointcloud.
  pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud_raw(
      new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*msg_in, *pointcloud_raw);
  // Filter NaN if needed.
  if (!params_.nanfree) {
    std::vector<int> nan_filter;
    pcl::removeNaNFromPointCloud(*pointcloud_raw, *pointcloud_raw, nan_filter);
  }
  // Apply voxel filter.
  pcl::PointCloud<pcl::PointXYZ>::Ptr voxelized_pointcloud(
      new pcl::PointCloud<pcl::PointXYZ>);
  if (params_.do_voxel_filter) {
    voxel_filter_.reset(new pcl::VoxelGrid<pcl::PointXYZ>);
    voxel_filter_->setInputCloud(pointcloud_raw);
    voxel_filter_->setLeafSize(
        params_.resolutions_xyz[0], params_.resolutions_xyz[1],
        params_.resolutions_xyz[2]);
    voxel_filter_->filter(*voxelized_pointcloud);
    // Publish the filtered pointcloud.
    sensor_msgs::PointCloud2 voxelized_pointcloud_msg;
    voxelized_pointcloud_msg.header = msg_in->header;
    pcl::toROSMsg(*voxelized_pointcloud, voxelized_pointcloud_msg);
    voxel_pcl_pub_.publish(voxelized_pointcloud_msg);
  }
  // Apply ground filter.
  if (params_.do_ground_segmentation) {
    pcl::PointCloud<pcl::PointXYZ> tf_pointcloud, ground_pointcloud,
        nonground_pointcloud;
    if (params_.ground_filter_voxelized && params_.do_voxel_filter)
      pcl_ros::transformPointCloud<pcl::PointXYZ>(
          params_.global_frame_id, *voxelized_pointcloud, tf_pointcloud, tfl_);
    else
      pcl_ros::transformPointCloud<pcl::PointXYZ>(
          params_.global_frame_id, *pointcloud_raw, tf_pointcloud, tfl_);
    // Segment the pointcloud.
    segmentGround(tf_pointcloud, &ground_pointcloud, &nonground_pointcloud);
    // Publish the ground pointcloud.
    sensor_msgs::PointCloud2 ground_pointcloud_msg;
    ground_pointcloud_msg.header.frame_id = params_.global_frame_id;
    ground_pointcloud_msg.header.stamp    = msg_in->header.stamp;
    pcl::toROSMsg(ground_pointcloud, ground_pointcloud_msg);
    ground_pcl_pub_.publish(ground_pointcloud_msg);
    // Publish the nonground pointcloud.
    sensor_msgs::PointCloud2 nonground_pointcloud_msg;
    nonground_pointcloud_msg.header.frame_id = params_.global_frame_id;
    nonground_pointcloud_msg.header.stamp    = msg_in->header.stamp;
    pcl::toROSMsg(nonground_pointcloud, nonground_pointcloud_msg);
    nonground_pcl_pub_.publish(nonground_pointcloud_msg);
  }
}

void PointCloudFilter::segmentGround(
    const pcl::PointCloud<pcl::PointXYZ>& cloud,
    pcl::PointCloud<pcl::PointXYZ>* ground,
    pcl::PointCloud<pcl::PointXYZ>* nonground) const {
  const int npoints = cloud.points.size();
  ground->header = nonground->header = cloud.header;
  ground->points.reserve(npoints);
  nonground->points.reserve(npoints);
  for (const auto& point : cloud.points) {
    if (std::abs(point.z) <= params_.ground_threshold)
      ground->points.emplace_back(point);
    else if (point.z > params_.ground_threshold)
      nonground->points.emplace_back(point);
  }
  ground->points.shrink_to_fit();
  nonground->points.shrink_to_fit();
}

PointCloudFilter::Params PointCloudFilter::Params::defaultParams() {
  Params params;
  params.nanfree                 = true;
  params.do_voxel_filter         = true;
  params.resolutions_xyz         = {0.05, 0.05, 0.05};
  params.do_ground_segmentation  = false;
  params.ground_filter_voxelized = false;
  params.ground_threshold        = 0.05;
  params.update_rate             = 10.0;
  params.global_frame_id         = "/map";
  return params;
}

}  // namespace squirrel_pointcloud_filter
