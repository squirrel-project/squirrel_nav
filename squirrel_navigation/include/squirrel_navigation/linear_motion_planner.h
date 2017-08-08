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

#ifndef SQUIRREL_NAVIGATION_MOTION_PLANNER_H_
#define SQUIRREL_NAVIGATION_MOTION_PLANNER_H_

#include "squirrel_navigation/LinearMotionPlannerConfig.h"
#include "squirrel_navigation/utils/motion_planner.h"

#include <ros/publisher.h>
#include <ros/subscriber.h>
#include <ros/time.h>

#include <dynamic_reconfigure/server.h>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>

#include <angles/angles.h>

#include <cmath>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace squirrel_navigation {

class LinearMotionPlanner : public utils::MotionPlanner {
 public:
  class Params {
   public:
    static Params defaultParams();

    double max_linear_velocity, max_angular_velocity;
    double linear_smoother, angular_smoother;
    double time_scaler;
    double lookahead;
    int waypoints_heading_lookahead;
  };

 public:
  LinearMotionPlanner() : params_(Params::defaultParams()) { init_ = false; }
  LinearMotionPlanner(const Params& params) : params_(params) { init_ = false; }
  virtual ~LinearMotionPlanner() {}

  // Create/update a new motion trajectory.
  void initialize(const std::string& name) override;
  void reset(
      const std::vector<geometry_msgs::PoseStamped>& waypoints,
      const ros::Time& start) override;
  void update(
      const std::vector<geometry_msgs::PoseStamped>& waypoints,
      const ros::Time& stamp) override;

  // Get reference for motion control.
  void computeReference(
      const ros::Time& stamp, geometry_msgs::Pose* pose,
      geometry_msgs::Twist* twist) override;

  // Get the forward trajectory.
  std::vector<geometry_msgs::PoseStamped> trajectory() const override;

  // Waypoint(s) getters.
  const std::vector<geometry_msgs::PoseStamped>& waypoints() const override;

  // Get start/goal.
  const geometry_msgs::PoseStamped& start() const;
  const geometry_msgs::PoseStamped& goal() const;

  // Mutex getter.
  inline std::mutex& mutex() const { return update_mtx_; }

  // Parameters read/write.
  inline const Params& params() const { return params_; }
  inline void setParams(const Params& params) { params_ = params; }
  inline Params& params() { return params_; }

 private:
  // Reconfigure the parameters.
  void reconfigureCallback(LinearMotionPlannerConfig& config, uint32_t level);

  // Compute next waypoint.
  std::vector<geometry_msgs::PoseStamped>::const_iterator
      computeHeadingWaypoint(const ros::Time& stamp) const;
  int computeHeadingWaypointIndex(const ros::Time& stamp) const;

  // Smooth up the planned trajectory.
  void smoothTrajectory(
      const std::vector<geometry_msgs::PoseStamped>& waypoints, int begin,
      int end, std::vector<geometry_msgs::PoseStamped>* smooth_waypoints) const;
  void smoothTrajectoryInPlace(
      int begin, int end, std::vector<geometry_msgs::PoseStamped>* waypoints);

  // Define the velocity profile.
  double computeSafetyVelocity(double linear_delta, double angular_delta) const;

 private:
  Params params_;
  std::unique_ptr<dynamic_reconfigure::Server<LinearMotionPlannerConfig>> dsrv_;

  std::vector<geometry_msgs::PoseStamped>::const_iterator heading_waypoint_;
  std::vector<geometry_msgs::PoseStamped> waypoints_;

  mutable std::mutex update_mtx_;
};

}  // namespace squirrel_navigation

#endif /* SQUIRREL_NAVIGATION_MOTION_PLANNER_H_ */
