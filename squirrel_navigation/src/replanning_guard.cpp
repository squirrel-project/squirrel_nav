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
// OF THE POSSIBILITY OF SUCH DAMAGE

#include "squirrel_navigation/replanning_guard.h"
#include "squirrel_navigation/utils/math_utils.h"

#include <costmap_2d/cost_values.h>

#include <ros/console.h>

#include <tf/tf.h>

namespace squirrel_navigation {

void ReplanningGuard::initialize(const costmap_2d::Costmap2D& costmap) {
  if (init_)
    return;
  costmap_model_.reset(new base_local_planner::CostmapModel(costmap));
  // Initialization successful.
  init_ = true;
  ROS_INFO_STREAM(
      "squirrel_navigation::ReplanningGuard: Initialization successful.");
}

void ReplanningGuard::checkCollisionsWithRadius(double radius, double lookahead) {
  const std::vector<geometry_msgs::Point> empty_footprint;
  // values 0.22 are the default radius, anyway those values are ignored.
  checkCollisionsWithFootprint(empty_footprint, radius, radius, lookahead);
}

void ReplanningGuard::checkCollisionsWithFootprint(
    const std::vector<geometry_msgs::Point>& footprint, double inscribed_radius,
    double circumscribed_radius, double lookahead) {
  replanning_flag_ = false;
  // Check initialization.
  if (!init_) {
    ROS_ERROR_STREAM_ONCE(
        "squirrel_navigation::ReplanningGuard: The replanning checker is not "
        "initialized.");
    replanning_flag_ = true;
  }
  // Check the path for collisions.
  const int nwaypoints     = waypoints_.size();
  double cum_lin_lookahead = 0.;
  for (int i = 0; cum_lin_lookahead < lookahead && i < nwaypoints - 1; ++i) {
    cum_lin_lookahead +=
        math::linearDistance2D(waypoints_[i], waypoints_[i + 1]);
    // Extract waypoint coordinate.
    const double x             = waypoints_[i].pose.position.x;
    const double y             = waypoints_[i].pose.position.x;
    const double a             = tf::getYaw(waypoints_[i].pose.orientation);
    // Evaluate waypoint cost.
    const double waypoint_cost = costmap_model_->footprintCost(
        x, y, a, footprint, inscribed_radius, circumscribed_radius);
    if (waypoint_cost < 0. || waypoint_cost == costmap_2d::LETHAL_OBSTACLE ||
        waypoint_cost >= costmap_2d::INSCRIBED_INFLATED_OBSTACLE - 1) {
      replanning_flag_ = true;
      break;
    }
  }
}

const std::vector<geometry_msgs::PoseStamped>& ReplanningGuard::waypoints()
    const {
  return waypoints_;
}

void ReplanningGuard::setWaypoints(
    const std::vector<geometry_msgs::PoseStamped>& waypoints) {
  waypoints_ = waypoints;
}

}  // namespace squirrel_navigation
