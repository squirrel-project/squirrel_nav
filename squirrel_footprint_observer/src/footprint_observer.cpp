// Copyright (C) 2017  Federico Boniardi and Wolfram Burgard
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.

#include "squirrel_footprint_observer/footprint_observer.h"
#include "squirrel_footprint_observer/footprint_utils.h"
#include "squirrel_footprint_observer/parameter_parser.h"

#include <stdexcept>
#include <string>
#include <thread>

namespace squirrel_footprint_observer {

FootprintObserver::FootprintObserver() : enabled_(true) {
  const std::string& node_name = ros::this_node::getName();
  ros::NodeHandle pnh("~");
  // Read base footprint.
  if (pnh.hasParam("base_radius")) {
    double base_radius;
    pnh.getParam("base_radius", base_radius);
    if (base_radius <= 0.) {
      ROS_ERROR_STREAM(node_name << ": Robot radius is not positive.");
      ros::shutdown();
    } else {
      approximatedCircle<kBaseNumPoints>(base_radius, &base_footprint_);
      ROS_INFO_STREAM(
          node_name << ": Base footprint successfully initialized.");
    }
  } else if (pnh.hasParam("base_footprint")) {
    std::string footprint_str;
    pnh.getParam("base_footprint", footprint_str);
    if (!parameter_parser::parseFootprintParameter(
            footprint_str, &base_footprint_)) {
      ROS_ERROR_STREAM(node_name << ": Please specify a valid footprint.");
      ros::shutdown();
    }
  } else {
    ROS_ERROR_STREAM(
        node_name << ": Please specify the base radius or the base footprint");
    ros::shutdown();
  }
  // Read the single joint footprint.
  if (pnh.hasParam("joint_radius")) {
    double joint_radius;
    pnh.getParam("joint_radius", joint_radius);
    if (joint_radius <= 0.) {
      ROS_ERROR_STREAM(node_name << ": Joint radius is not positive.");
      ros::shutdown();
    } else {
      approximatedCircle<kJointNumPoints>(joint_radius, &joint_footprint_);
      ROS_INFO_STREAM(
          node_name << ": Joint footprint successfully initialized.");
    }
  }
  // Read the arm joint chain.
  if (pnh.hasParam("joint_chain")) {
    pnh.getParam("joint_chain", joint_chain_);
    if (joint_chain_.empty()) {
      ROS_ERROR_STREAM(node_name << ": Please provide a valid joint link.");
      ros::shutdown();
    }
  }
  // Read the robot frame id.
  pnh.param<std::string>(
      "base_frame_id", footprint_.header.frame_id, "/base_link");
  // Initialize the footprint services, publisher and subscribers.
  enable_sub_ =
      pnh.subscribe("enable", 1, &FootprintObserver::enableCallback, this);
  footprint_pub_ = pnh.advertise<geometry_msgs::PolygonStamped>("footprint", 1);
  footprint_srv_ = pnh.advertiseService(
      "getFootprint", &FootprintObserver::footprintServiceCallback, this);
  ROS_INFO_STREAM(node_name << ": FootprintObserver successfully initialized.");
}

void FootprintObserver::spin(double hz) {
  const std::string& node_name = ros::this_node::getName();
  for (ros::Rate lr(hz); ros::ok(); lr.sleep()) {
    try {
      ros::spinOnce();
      if (!enabled_)
        continue;
      updateFootprint(ros::Time::now());
      footprint_pub_.publish(footprint_);
    } catch (const std::runtime_error& err) {
      ROS_ERROR_STREAM(node_name << ": " << err.what());
    }
  }
}

void FootprintObserver::updateFootprint(const ros::Time& stamp) {
  std::unique_lock<std::mutex> lock(update_mtx);
  const size_t njoints        = joint_chain_.size();
  const size_t footprint_size = kBaseNumPoints + njoints * kJointNumPoints;
  std::vector<Point2D> full_footprint;
  full_footprint.reserve(footprint_size);
  // Add the downprojected joints to the full footprint.
  for (size_t i = 0; i < njoints; ++i) {
    tf::StampedTransform tf_base2joint;
    const std::string& base_frame_id  = footprint_.header.frame_id;
    const std::string& joint_frame_id = joint_chain_[i];
    try {
      tfl_.waitForTransform(
          base_frame_id, joint_frame_id, stamp, ros::Duration(0.05));
      tfl_.lookupTransform(base_frame_id, joint_frame_id, stamp, tf_base2joint);
    } catch (const tf::TransformException& ex) {
      const std::string& node_name = ros::this_node::getName();
      ROS_ERROR_STREAM(node_name << ": " << ex.what());
      ROS_WARN_STREAM(node_name << ": Skipping footprint update.");
      return;
    }
    // Transform the joint footprint according to the joint position.
    const double joint_x = tf_base2joint.getOrigin().getX();
    const double joint_y = tf_base2joint.getOrigin().getY();
    const Vector2D joint_position(joint_x, joint_y);
    for (size_t i = 0; i < kJointNumPoints; ++i)
      full_footprint.emplace_back(joint_footprint_[i] + joint_position);
  }
  // Add the base to the full footprint.
  for (size_t i = 0; i < kBaseNumPoints; ++i)
    full_footprint.emplace_back(base_footprint_[i]);
  // Compute convex hull.
  std::vector<Point2D> ch_footprint;
  CGAL::convex_hull_2(
      full_footprint.begin(), full_footprint.end(),
      std::back_inserter(ch_footprint));
  // Update the current footprint.
  footprint_.polygon.points.resize(ch_footprint.size());
  for (size_t i = 0; i < ch_footprint.size(); ++i) {
    footprint_.polygon.points[i].x = ch_footprint[i].x();
    footprint_.polygon.points[i].y = ch_footprint[i].y();
  }
}

void FootprintObserver::enableCallback(const std_msgs::Bool::ConstPtr& msg) {
  enabled_ = msg->data;
}

bool FootprintObserver::footprintServiceCallback(
    squirrel_footprint_observer_msgs::get_footprint::Request& req,
    squirrel_footprint_observer_msgs::get_footprint::Response& res) {
  updateFootprint(ros::Time::now());
  res.footprint = footprint_.polygon;
  return true;
}

}  // namespace squirrel_footprint_observer
