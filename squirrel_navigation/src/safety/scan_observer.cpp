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

#include "squirrel_navigation/safety/scan_observer.h"

#include <ros/init.h>

#include <squirrel_navigation_msgs/ScanSafetyObservation.h>
#include <visualization_msgs/MarkerArray.h>

#include <tf/tf.h>

#include <cmath>
#include <thread>

namespace squirrel_navigation {
namespace safety {

const std::string ScanObserver::tag = "ScanSafetyObserver";

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
  safety_log_pub_ =
      pnh.advertise<squirrel_navigation_msgs::ScanSafetyObservation>(
          "scan_safety_log", 1);
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
            "squirrel_navigation::safety::ScanObserver: Unsafe scan,"
            "someone's approaching the robot?.");
      return false || !params_.enabled;
    }
  return true || !params_.enabled;
}

bool ScanObserver::safeCheck(int i) const {
  if ((ranges_[i] > params_.unsafe_range) ||
      (std::abs(rangevelocities_[i]) <= params_.max_safety_rangevel) &&
      (rangevelocities_[i] < 0.))
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
    ranges_[i]          = state(i, 0);
    rangevelocities_[i] = state(i, 1);
  }
  publishMarkers(stamp);
  publishMessage(stamp);
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

double ScanObserver::computeMarkerArrowLength(int i) const {
  const double length = 0.5 * std::abs(rangevelocities_[i]);
  return std::max(length, 1e-3);
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
    marker.scale.x         = computeMarkerArrowLength(i);
    marker.scale.y         = 0.1;
    marker.scale.z         = 0.1;
    marker.color.r         = safeCheck(i);
    marker.color.g         = !safeCheck(i);
    marker.color.b         = 0.0;
    marker.color.a         = 0.5;
    marker_array.markers.emplace_back(marker);
  }
  rangevels_pub_.publish(marker_array);
}

void ScanObserver::publishMessage(const ros::Time& stamp) const {
  squirrel_navigation_msgs::ScanSafetyObservation safety_log;
  safety_log.header.stamp    = stamp;
  safety_log.header.frame_id = params_.scan_frame_id;
  safety_log.is_safe         = true;
  safety_log.safe_readings.resize(ranges_.size());
  for (int i = 0; i < (int)ranges_.size(); ++i) {
    safety_log.safe_readings[i] = safeCheck(i);
    if (!safety_log.safe_readings[i])
      safety_log.is_safe = false;
  }
  safety_log.range_velocities          = rangevelocities_;
  safety_log.max_safety_range_velocity = params_.max_safety_rangevel;
  safety_log.unsafe_range              = params_.unsafe_range;
  safety_log_pub_.publish(safety_log);
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
