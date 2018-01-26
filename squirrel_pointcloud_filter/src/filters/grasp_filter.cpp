// Copyright (c) 2017, Federico Boniardi and Wolfram Burgard
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
// * Redistributions of source code must retain the above copyright
//   notice, this list of conditions and the following disclaimer.
//
// * Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in
//   the documentation and/or other materials provided with the
//   distribution.
//
// * Neither the name of the University of Freiburg nor the names of
//   its contributors may be used to endorse or promote products
//   derived from this software without specific prior written
//   permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
// INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
// HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
// STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
// OF THE POSSIBILITY OF SUCH DAMAGE.

#include "squirrel_pointcloud_filter/filters/grasp_filter.h"
#include "squirrel_pointcloud_filter/tf_pcl_utils.h"

#include <ros/duration.h>
#include <ros/init.h>
#include <ros/node_handle.h>
#include <ros/time.h>

namespace squirrel_pointcloud_filter {

GraspFilter::GraspFilter(const std::string& name)
    : dsrv_(nullptr), initialized_(false) {
  initialize(name);
}

void GraspFilter::initialize(const std::string& name) {
  if (!initialized_) {
    ros::NodeHandle nh("~/" + name), gnh;

    dsrv_.reset(new dynamic_reconfigure::Server<GraspFilterConfig>(nh));
    dsrv_->setCallback(
        boost::bind(&GraspFilter::reconfigureCallback, this, _1, _2));

    if (!const_object_dim_)
      sub_ = gnh.subscribe(
          object_topic_, 1, &GraspFilter::objectSizeCallback, this);

    initialized_ = true;
    ROS_INFO_STREAM_NAMED(
        ros::this_node::getName(), "GraspFilter succesfully initialized.");
  }
}

void GraspFilter::apply(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& pointcloud) const {
  if (!enabled_)
    return;

  std::unique_lock<std::mutex> lock(mtx_);

  const ros::Time& now = ros::Time::now();

  tf::StampedTransform tf_pclframe2object;
  try {
    tfl_.waitForTransform(
        pointcloud_frame_, object_frame_, now, ros::Duration(0.05));
    tfl_.lookupTransform(
        pointcloud_frame_, object_frame_, now, tf_pclframe2object);
  } catch (const tf::TransformException& ex) {
    ROS_ERROR_STREAM_NAMED(ros::this_node::getName(), ex.what());
    return;
  }

  const tf::Vector3& object_position = tf_pclframe2object.getOrigin();

  const auto in_ellipsoid = [this](const tf::Vector3& p) {
    const float ellipsoid_eq = std::pow(p.getX() / sizes_[0], 2) +
                               std::pow(p.getY() / sizes_[1], 2) +
                               std::pow(p.getZ() / sizes_[2], 2);
    return ellipsoid_eq <= 1;
  };

  const int num_init_points = pointcloud->points.size();
  for (auto it = pointcloud->points.begin(); it != pointcloud->points.end();) {
    if (in_ellipsoid(*it - object_position))
      it = pointcloud->points.erase(it);
    else
      ++it;
  }

  if (pointcloud->points.size() != num_init_points) {
    pointcloud->height = 1;
    pointcloud->width  = pointcloud->points.size();
  }
}

void GraspFilter::reconfigureCallback(
    GraspFilterConfig& config, uint32_t level) {
  object_frame_     = config.object_frame;
  enabled_          = config.enabled;
  const_object_dim_ = config.use_const_object_dim;
  object_topic_     = config.object_topic;
  sizes_            = {config.size_x, config.size_y, config.size_z};
}

void GraspFilter::objectSizeCallback(
    const std_msgs::Float32MultiArray::ConstPtr& params) {
  sizes_ = {params->data[0], params->data[1], params->data[2]};
}

const std::string GraspFilter::tag = "GraspFilter";

}  // namespace squirrel_pointcloud_filter
