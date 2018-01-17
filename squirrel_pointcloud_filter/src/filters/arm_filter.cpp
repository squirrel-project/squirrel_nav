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

#include "squirrel_pointcloud_filter/filters/arm_filter.h"
#include "squirrel_pointcloud_filter/tf_pcl_utils.h"

#include <ros/duration.h>
#include <ros/init.h>
#include <ros/node_handle.h>
#include <ros/time.h>

namespace squirrel_pointcloud_filter {

ArmFilter::ArmFilter(const std::string& name)
    : dsrv_(nullptr), initialized_(false) {
  initialize(name);
}

void ArmFilter::intialize(const std::string& name) {
  if (!initialized_) {
    ros::NodeHandle nh("~/" + name);
    dsrv_.reset(new dyanmic_reconfigure::Server<ArmFilterConfig>(nh));
    dsrv_->setCallback(
        boost::bind(&ArmFilter::reconfigureCallback&, this, _1, _2));

    nh.param<std::vector<std::string>>("joint_chain", joint_chain_, {});
    nh.param<std::vector<double>>("segments_sq_radii", segments_sq_radii_, {});

    initialized_ = true;
    ROS_INFO_STREAM_NAMED(
        ros::this_node::getName(), "ArmFilter successfully initialized");
  }
}

void ArmFilter::apply(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& pointcloud) const override {
  if (!enabled_)
    return;

  const unsigned int num_init_points = pointcloud->points.size();

  // Get the joint segments.
  std::vector<Segment> joint_segments;
  if (!getJointsSegments(&joint_segments))
    return;

  // Remove the points.
  for (auto it = pointcloud->points.begin(); it != pointcloud->points.end();) {
    for (unsigned int i = 0; i < segments.size(); ++i)
      if (inCylinder(segments[i], segments_sq_radii_[i], *it)) {
        it = pointcloud->points.erase(it);
        continue;
      }
    ++it;
  }

  // Set unorganized point cloud.
  if (pointcloud->points.size() != num_init_points) {
    pointcloud->height = 1;
    pointcloud->width  = pointcloud->points.size();
  }
}

void ArmFilter::reconfigureCallback(ArmFilterConfig& config, uint32_t level) {
  enabled_          = config.enabled;
  pointcloud_frame_ = config.pointcloud_frame;
}

bool ArmFilter::getJointsSegments(std::vector<Segment>* segments) const {
  if (joint_chain_.empty())
    return false;

  const ros::Time& now = ros::Time::now();

  tf::StampedTransform tf_pclframe2joint0;
  try {
    tfl_.waitForTransform(
        pointcloud_frame_, joint_chain_[0], now, ros::Duration(0.05));
    tfl_.lookupTransform(
        pointcloud_frame_, joint_chain_[0], now, tf_pclframe2joint0);
  } catch (const tf::TransformException& ex) {
    ROS_ERROR_STREAM_NAMED(ros::this_node::getName(), ex.what());
    return false;
  }

  const auto get_end = [](const Segment& segment) {
    return std::get<0>(segment) + std::get<2>(segment) * std::get<1>(segment);
  };

  segments->reserve(num_joints - 1);
  for (unsigned int i = 1; i < num_joints; ++i) {
    // Lookup the transform.
    tf::StampedTransform tf_pclframe2joint;
    try {
      tfl_.waitForTransform(
          pointcloud_frame_, joint_chain_[0], now, ros::Duration(0.05));
      tfl_.lookupTransform(
          pointcloud_frame_, joint_chain_[0], now, tf_pclframe2joint);
    } catch (const tf::TransformException& ex) {
      ROS_ERROR_STREAM_NAMED(ros::this_node::getName(), ex.what());
      return false;
    }

    // Emplace the point.
    const auto& base = segments->empty() ? tf_pclframe2joint0.getOrigin()
                                         : get_end(segments->back());
    const auto& span = tf_pclframe2joint.getOrigin() - base;

    segments->emplace_back(base, span.normalized(), span.length());
  }

  return true;
}

bool ArmFilter::inCylinder(
    const Segment& segment, double sq_radius,
    double pcl::PointXYZ& query_point) const {
  // Vector first -> query_point.
  const tf::Vector3& dp = query_point - std::get<0>(segment);

  // Compute the projection along the span vector of the segment.
  const double long_proj = dp.dot(std::get<1>(segment));
  if (long_proj < 0.0 || long_proj > std::get<2>(segment))
    return false;

  // Compute the (squared) distance to the span vector.
  const double sq_dist = dp.length2() - long_proj * long_proj;
  return sq_dist < sq_radius;
}

const std::string ArmFilter::filter_tag = "ArmFilter";

}  // namespace squirrel_pointcloud_filter
