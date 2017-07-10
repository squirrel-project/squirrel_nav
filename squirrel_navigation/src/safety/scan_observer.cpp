// The MIT License (MIT)
//
// Copyright (c) 2017 Federico Boniardi and Wolfram Burgard
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include "squirrel_navigation/safety/scan_observer.h"

#include <ros/init.h>

#include <visualization_msgs/MarkerArray.h>

#include <tf/tf.h>

#include <cmath>
#include <thread>

namespace squirrel_navigation {

namespace safety {

void ScanObserver::initialize(const std::string& name) {
  if (init_)
    return;
  // Initialize the parameter server.
  ros::NodeHandle pnh("~/" + name), nh;
  dsrv_.reset(new dynamic_reconfigure::Server<ScanObserverConfig>(pnh));
  dsrv_->setCallback(
      boost::bind(&ScanObserver::reconfigureCallback, this, _1, _2));
  // Initialize the alpha/beta-filter and internal observer.
  ab_filter_.initialize(name + "/ab_filter");
  // Publishers subscribers.
  scan_sub_ = nh.subscribe(
      params_.scan_topic, 1, &ScanObserver::laserScanCallback, this);
  rangevels_pub_ =
      pnh.advertise<visualization_msgs::MarkerArray>("ranges_velocities", 1);
  // Initialization successful.
  init_ = true;
  ROS_INFO_STREAM(
      "squirrel_navigation::safety::ScanObserver: Initialization successful.");
}

bool ScanObserver::safe() const {
  std::unique_lock<std::mutex> lock(mtx_);
  for (int i = 0; i < ab_filter_.stateDimension(); ++i)
    if (!safeCheck(i)) {
      if (params_.verbose)
        ROS_WARN_STREAM(
            "squirrel_navigation::safety::ScanObserver: Unsafe scan.");
      return false || !params_.enabled;
    }
  return true || !params_.enabled;
}

bool ScanObserver::safeCheck(int i) const {
  if (ranges_[i] > params_.unsafe_range ||
      (std::abs(rangevelocities_[i]) <= params_.max_safety_rangevel &&
       rangevelocities_[i] < 0.))
    return true;
  return false;
}

void ScanObserver::reconfigureCallback(
    ScanObserverConfig& config, uint32_t level) {
  params_.enabled             = config.enabled;
  params_.scan_topic          = config.scan_topic;
  params_.max_safety_rangevel = config.max_safety_rangevel;
  params_.unsafe_range        = config.unsafe_range;
  params_.verbose             = config.verbose;
}

void ScanObserver::laserScanCallback(
    const sensor_msgs::LaserScan::ConstPtr& scan) {
  ROS_INFO_STREAM_ONCE(
      "squirrel_navigation::safety::ScanObserver: Subscribed to LaserScan.");
  // Get scan paramters.
  params_.scan_angle_increment = scan->angle_increment;
  params_.scan_angle_min       = scan->angle_min;
  params_.scan_frame_id        = scan->header.frame_id;
  // Update the internal state vector.
  updateState(scan->ranges, scan->header.stamp);
}

void ScanObserver::updateState(
    const std::vector<float>& ranges, const ros::Time& stamp) {
  std::unique_lock<std::mutex> lock(mtx_);
  // Update the dimension of the filter (only once).
  static const int nranges = ranges.size();
  ab_filter_.setStateDimension(nranges);
  // Compute the velocities.
  const Eigen::MatrixXf& state = ab_filter_(ranges, stamp);
  ranges_.resize(nranges);
  rangevelocities_.resize(nranges);
  for (int i = 0; i < nranges; ++i) {
    ranges_[i]          = ranges[i];
    rangevelocities_[i] = state(1, i);
  }
  publishMarkers(stamp);
}

geometry_msgs::Pose ScanObserver::computeMarkerPose(int i) const {
  const double vel_dir = rangevelocities_[i] >= 0. ? 0. : -M_PI;
  const double angle =
      params_.scan_angle_min + i * params_.scan_angle_increment;
  // Computing the pose.
  geometry_msgs::Pose marker_pose;
  marker_pose.position.x  = ranges_[i] * std::cos(angle);
  marker_pose.position.y  = ranges_[i] * std::sin(angle);
  marker_pose.position.z  = 0.0;
  marker_pose.orientation = tf::createQuaternionMsgFromYaw(angle + vel_dir);
  return marker_pose;
}

void ScanObserver::publishMarkers(const ros::Time& stamp) const {
  visualization_msgs::MarkerArray marker_array;
  marker_array.markers.reserve(ab_filter_.stateDimension());
  for (int i = 0; i < ab_filter_.stateDimension(); ++i) {
    visualization_msgs::Marker marker;
    marker.id              = i;
    marker.header.stamp    = stamp;
    marker.header.frame_id = params_.scan_frame_id;
    marker.ns              = "range_velocities";
    marker.type            = visualization_msgs::Marker::ARROW;
    marker.action          = visualization_msgs::Marker::MODIFY;
    marker.pose            = computeMarkerPose(i);
    marker.scale.x         = std::abs(rangevelocities_[i]);
    marker.scale.y         = 0.1;
    marker.scale.z         = 0.0;
    marker.color.r         = safeCheck(i);
    marker.color.g         = !safeCheck(i);
    marker.color.b         = 0.0;
    marker.color.a         = 0.5;
    marker_array.markers.emplace_back(marker);
  }
  rangevels_pub_.publish(marker_array);
}

ScanObserver::Params ScanObserver::Params::defaultParams() {
  Params params;
  params.enabled             = true;
  params.scan_topic          = "/scan";
  params.max_safety_rangevel = 0.5;
  params.unsafe_range        = 0.75;
  params.verbose             = false;
  return params;
}

}  // namespace safety

}  // namespace squirrel_navigation
