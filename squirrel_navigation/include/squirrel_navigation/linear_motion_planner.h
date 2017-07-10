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
    int waypoints_heading_lookahead;
  };

 public:
  LinearMotionPlanner() : params_(Params::defaultParams()) { init_ = false; }
  LinearMotionPlanner(const Params& params) : params_(params) { init_ = false; }
  virtual ~LinearMotionPlanner() {}

  // Create/update a new motion trajectory.
  void initialize(const std::string& name) override;
  void reset(const std::vector<geometry_msgs::PoseStamped>& waypoints) override;
  void update(
      const std::vector<geometry_msgs::PoseStamped>& waypoints) override;

  // Get reference for motion control.
  void computeReference(
      const ros::Time& stamp, geometry_msgs::Pose* pose,
      geometry_msgs::Twist* twist) override;

  // Waypoint(s) getters.
  const std::vector<geometry_msgs::PoseStamped>& waypoints() const;
  const geometry_msgs::PoseStamped& operator()(int i) const;
  const geometry_msgs::PoseStamped& operator[](int i) const;

  // Mutex getter.
  inline std::mutex& mutex() const { return update_mtx_; }

  // Set initial start time.
  inline const ros::Time* start() const { return start_.get(); }
  inline void setStart(const ros::Time& t) { start_.reset(new ros::Time(t)); }

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
      const std::vector<geometry_msgs::PoseStamped>& waypoints,
      std::vector<geometry_msgs::PoseStamped>* smooth_waypoints) const;

  // Define the velocity profile.
  double computeSafetyVelocity(double linear_delta, double angular_delta) const;

 private:
  Params params_;
  std::unique_ptr<dynamic_reconfigure::Server<LinearMotionPlannerConfig>> dsrv_;

  std::vector<geometry_msgs::PoseStamped> waypoints_;
  std::unique_ptr<ros::Time> start_;

  mutable std::mutex update_mtx_;
};

}  // namespace squirrel_navigation

#endif /* SQUIRREL_NAVIGATION_MOTION_PLANNER_H_ */
