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

#ifndef SQUIRREL_NAVIGATION_REPLANNING_GUARD_H_
#define SQUIRREL_NAVIGATION_REPLANNING_GUARD_H_

#include <base_local_planner/costmap_model.h>
#include <costmap_2d/costmap_2d.h>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>

#include <limits>
#include <memory>
#include <vector>

namespace squirrel_navigation {

class ReplanningGuard {
 public:
  ReplanningGuard() : replanning_flag_(false), init_(false), enabled_(false) {}
  virtual ~ReplanningGuard() {}

  // Initialization the costamp model.
  void initialize(const costmap_2d::Costmap2D& costmap_ros);

  // Check if waypoints are collision free.
  void checkCollisionsWithRadius(
      double radius,
      double lookahead = std::numeric_limits<double>::infinity());
  void checkCollisionsWithFootprint(
      const std::vector<geometry_msgs::Point>& footprint,
      double inscribed_radius, double circumscribed_radius,
      double lookahead = std::numeric_limits<double>::infinity());

  // Utilities.
  bool empty() const { return waypoints_.empty(); }
  size_t size() const { return waypoints_.size(); }
  void clear() { waypoints_.clear(), replanning_flag_ = true; }

  // Read/write the waypoints.
  const std::vector<geometry_msgs::PoseStamped>& waypoints() const;
  void setWaypoints(const std::vector<geometry_msgs::PoseStamped>& waypoints);

  // Read/write
  bool replanningFlag() const { return replanning_flag_; }
  void setReplanningFlag(bool flag) { replanning_flag_ = flag; }

  // Set enable/disable.
  bool enabled() const { return enabled_; }
  void setEnabled(bool enabled) { enabled_ = enabled; }

 private:
  std::vector<geometry_msgs::PoseStamped> waypoints_;
  bool replanning_flag_;
  bool enabled_;

  std::unique_ptr<base_local_planner::CostmapModel> costmap_model_;

  bool init_;
};

}  // namespace squirrel_navigation

#endif /* SQUIRREL_NAVIGATION_REPLANNING_GUARD_H_ */
