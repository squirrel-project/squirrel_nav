// Copyright (c) 2016, Federico Boniardi
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
// * Neither the name of the University of Freiburg nor the names of its
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

#ifndef SQUIRREL_NAVIGATION_TRAJECTORYPLANNER_H_
#define SQUIRREL_NAVIGATION_TRAJECTORYPLANNER_H_

#include <ros/console.h>
#include <ros/node_handle.h>
#include <ros/publisher.h>
#include <ros/time.h>

#include <dynamic_reconfigure/server.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>

#include <tf/tf.h>

#include <angles/angles.h>

#include "squirrel_navigation/TrajectoryPlannerConfig.h"

#include <cassert>
#include <cmath>
#include <mutex>
#include <thread>
#include <vector>

namespace squirrel_navigation {

class TrajectoryPlanner {
 protected:
  TrajectoryPlanner(void);

 public:
  class Profile {
   public:
    Profile(void) : x(0), y(0), yaw(0), vx(0), vy(0), vyaw(0){};
    ~Profile(void){};

    double x, y, yaw, vx, vy, vyaw;
  };

  class Pose2D {
   public:
    Pose2D(void) : x(0), y(0), yaw(0), t(0){};
    virtual ~Pose2D(void){};

    static inline geometry_msgs::Pose toPoseMsg(const Pose2D& p) {
      geometry_msgs::Pose pose;
      pose.position.x  = p.x;
      pose.position.y  = p.y;
      pose.orientation = tf::createQuaternionMsgFromYaw(p.yaw);
      return pose;
    }

    double x, y, yaw, t;
  };

  static TrajectoryPlanner* getTrajectory(void);
  static void deleteTrajectory(void);

  void setVelocityBounds(double, double);

  bool makeTrajectory(
      const geometry_msgs::PoseStamped&,
      std::vector<geometry_msgs::PoseStamped>&, size_t, bool);

  geometry_msgs::Pose getGoal(void) const;
  size_t getNodePose(ros::Time&, geometry_msgs::PoseStamped&) const;
  Profile getProfile(const ros::Time&);
  bool lostRobot(const Profile&, const Profile&);

  inline std::vector<Pose2D>* getPoses(void) { return poses_; };
  inline bool isActive(void) { return (t0_ != nullptr); };
  inline void deactivate(void) {
    if (t0_) {
      delete t0_;
      t0_ = nullptr;
    }
  };

 public:
  static TrajectoryPlanner* trajectory_ptr_;
  static ros::Time* t0_;
  static std::vector<Pose2D>* poses_;

  double max_linear_vel_, max_angular_vel_;

  // Parameters
  static dynamic_reconfigure::Server<TrajectoryPlannerConfig>* dsrv_;
  double xy_smoother_, yaw_smoother_;
  double time_scaler_;
  int heading_lookahead_;

  std::mutex guard_;

  bool initTrajectory(const std::vector<geometry_msgs::PoseStamped>&, bool);
  bool updateTrajectory(
      const std::vector<geometry_msgs::PoseStamped>&, size_t, bool);

  size_t matchIndex(double) const;
  void reconfigureCallback(TrajectoryPlannerConfig&, uint32_t);

  inline double timeIncrement(const Pose2D& p1, const Pose2D& p2) const {
    double dl, da;

    dl = std::hypot(p1.x - p2.x, p1.y - p2.y);
    da = std::abs(angles::normalize_angle(p1.yaw - p2.yaw));

    return (1 + time_scaler_) *
           std::max(dl / max_linear_vel_, da / max_angular_vel_);
  };

  inline double linearDistance(const Profile& p1, const Profile& p2) {
    return std::hypot(p1.x - p2.x, p1.y - p2.y);
  };

  inline double angularDistance(const Profile& p1, const Profile& p2) {
    return std::abs(angles::normalize_angle(p1.yaw - p2.yaw));
  };
};

}  // namespace squirrel_navigation

#endif /* SQUIRREL_NAVIGATION_TRAJECTORYPLANNER_H_ */
