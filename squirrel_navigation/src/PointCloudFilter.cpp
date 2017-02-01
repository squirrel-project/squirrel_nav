// Copyright (c) 2014, Federico Boniardi
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
// * Neither the name of the {organization} nor the names of its
//   contributors may be used to endorse or promote products derived from
//   this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#include "squirrel_navigation/PointCloudFilter.h"

namespace squirrel_navigation {

PointCloudFilter::PointCloudFilter(void)
    : pointcloud_in_topic_("/cloud_in"),
      pointcloud_out_topic_("/cloud_out"),
      pointcloud_size_("4500"),
      nodename_(ros::this_node::getName()),
      seq_(0),
      nanfree_(false) {
  ros::NodeHandle pnh("~");
  pnh.param<std::string>("pointcloud_in", pointcloud_in_topic_, "/cloud_in");
  pnh.param<std::string>("pointcloud_out", pointcloud_out_topic_, "/cloud_out");
  pnh.param<std::string>("pointcloud_size", pointcloud_size_, "4500");
  pnh.param<bool>("nan_free", nanfree_, false);

  getFilterStep_();

  sub_ = nh_.subscribe(
      pointcloud_in_topic_, 1, &PointCloudFilter::pointCloud2Callback_, this);

  pointcloud_pub_ =
      nh_.advertise<sensor_msgs::PointCloud2>(pointcloud_out_topic_, 1);
}

PointCloudFilter::~PointCloudFilter(void) {
  sub_.shutdown();
  pointcloud_pub_.shutdown();
  nh_.shutdown();
}

void PointCloudFilter::spin(double hz) {
  ros::Rate lr(hz);
  while (ros::ok()) {
    try {
      ros::spinOnce();
      lr.sleep();
    } catch (std::runtime_error& err) {
      ROS_ERROR("%s: %s", nodename_.c_str(), err.what());
    }
  }
}

void PointCloudFilter::pointCloud2Callback_(
    const sensor_msgs::PointCloud2::ConstPtr& pointcloud_in_msg) {
  ROS_DEBUG("%s: filtering pointcloud... ", nodename_.c_str());

  pcl::PointCloud<pcl::PointXYZ> pointcloud_raw;
  pcl::fromROSMsg(*pointcloud_in_msg, pointcloud_raw);

  if (not nanfree_) {
    std::vector<int> nan_filter;
    pcl::removeNaNFromPointCloud(pointcloud_raw, pointcloud_raw, nan_filter);
  }

  pcl::PointCloud<pcl::PointXYZ> pointcloud_filtered;

  for (unsigned int i = 0; i < pointcloud_raw.points.size(); i += filter_step_)
    pointcloud_filtered.points.push_back(pointcloud_raw[i]);

  sensor_msgs::PointCloud2 pointcloud_out_msg;
  toROSMsg(pointcloud_filtered, pointcloud_out_msg);
  pointcloud_out_msg.header.seq      = seq_++;
  pointcloud_out_msg.header.stamp    = ros::Time::now();
  pointcloud_out_msg.header.frame_id = pointcloud_in_msg->header.frame_id;

  pointcloud_pub_.publish(pointcloud_out_msg);

  ROS_DEBUG("%s: pointcloud published", nodename_.c_str());
}

void PointCloudFilter::getFilterStep_(void) {
  if (pointcloud_size_.compare("full") == 0) {
    ROS_WARN(
        "%s: chosen 'full pointcloud'. Consider reducing size of the "
        "pointcloud.",
        nodename_.c_str());
    filter_step_ = 1;
  } else {
    int size;
    std::istringstream(pointcloud_size_) >> size;
    filter_step_ = std::max(300000 / std::max(1, size), 1);
  }
}

}  // namespace squirrel_navigation
