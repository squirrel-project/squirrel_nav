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

#include "squirrel_navigation/TrajectoryPlanner.h"

namespace squirrel_navigation {

TrajectoryPlanner::TrajectoryPlanner(void)
    : max_linear_vel_(1.0),
      max_angular_vel_(2 * M_PI),
      xy_smoother_(0.75),
      yaw_smoother_(0.1),
      time_scaler_(0.75),
      heading_lookahead_(5) {
  ROS_INFO("squirrel_navigation::TrajectoryPlanner started");

  ros::NodeHandle pnh("~/trajectory_planner");
  if (not dsrv_)
    dsrv_ = new dynamic_reconfigure::Server<TrajectoryPlannerConfig>(pnh);
  dynamic_reconfigure::Server<TrajectoryPlannerConfig>::CallbackType cb =
      boost::bind(&TrajectoryPlanner::reconfigureCallback, this, _1, _2);
  dsrv_->setCallback(cb);

  // pnh.param<double>("xy_smoother", xy_smoother_, 0.75);
  // pnh.param<double>("yaw_smoother", yaw_smoother_, 0.1);
  // pnh.param<double>("time_scaler", time_scaler_, 0.75);
  // pnh.param<int>("heading_lookahead", heading_lookahead_, 5);

  if (time_scaler_ <= 0) {
    ROS_WARN(
        "TrajectoryPlanner: time_scaler should be positive. Revert to default "
        "parameter (0.75)");
    time_scaler_ = 0.75;
  }
}

TrajectoryPlanner* TrajectoryPlanner::getTrajectory(void) {
  if (not trajectory_ptr_) {
    trajectory_ptr_ = new TrajectoryPlanner;

    if (not poses_)
      poses_ = new std::vector<Pose2D>;
  }

  return trajectory_ptr_;
}

void TrajectoryPlanner::deleteTrajectory(void) {
  if (dsrv_)
    delete dsrv_;

  if (t0_)
    delete t0_;

  if (poses_)
    delete poses_;

  if (trajectory_ptr_)
    delete trajectory_ptr_;
}

void TrajectoryPlanner::setVelocityBounds(
    double linear_vel, double angular_vel) {
  max_linear_vel_  = linear_vel;
  max_angular_vel_ = angular_vel;
}

bool TrajectoryPlanner::makeTrajectory(
    const geometry_msgs::PoseStamped& start,
    std::vector<geometry_msgs::PoseStamped>& plan, size_t i, bool smooth) {
  if (plan.size() <= 0)
    return false;

  if (i <= 0) {
    plan[0].pose.orientation =
        start.pose.orientation;  // ROS planner does not consider this
    return initTrajectory(plan, smooth);
  } else {
    return updateTrajectory(plan, i, smooth);
  }
}

geometry_msgs::Pose TrajectoryPlanner::getGoal(void) const {
  geometry_msgs::Pose goal;
  goal.position.x  = poses_->back().x;
  goal.position.y  = poses_->back().y;
  goal.orientation = tf::createQuaternionMsgFromYaw(poses_->back().yaw);
  return goal;
}

size_t TrajectoryPlanner::getNodePose(
    ros::Time& t, geometry_msgs::PoseStamped& p) const {
  double tau = t.toSec() - t0_->toSec();

  size_t i = matchIndex(tau);

  p.pose.position.x  = (*poses_)[i].x;
  p.pose.position.y  = (*poses_)[i].y;
  p.pose.orientation = tf::createQuaternionMsgFromYaw((*poses_)[i].yaw);

  return i;
}

TrajectoryPlanner::Profile TrajectoryPlanner::getProfile(const ros::Time& t) {
  std::unique_lock<std::mutex> lock(guard_);

  double tau = t.toSec() - t0_->toSec();

  Profile out;

  if (tau < 0.0) {
    out.x   = poses_->front().x;
    out.y   = poses_->front().y;
    out.yaw = poses_->front().yaw;
    return out;
  } else if (tau >= poses_->back().t) {
    out.x   = poses_->back().x;
    out.y   = poses_->back().y;
    out.yaw = poses_->back().yaw;
    return out;
  }

  size_t i = matchIndex(tau) - 1;

  double dt = (*poses_)[i + 1].t - (*poses_)[i].t;
  double mu = (tau - (*poses_)[i].t) / dt;

  out.x   = (*poses_)[i].x + mu * ((*poses_)[i + 1].x - (*poses_)[i].x);
  out.y   = (*poses_)[i].y + mu * ((*poses_)[i + 1].y - (*poses_)[i].y);
  out.yaw = angles::normalize_angle(
      (*poses_)[i].yaw +
      mu * angles::normalize_angle((*poses_)[i + 1].yaw - (*poses_)[i].yaw));
  out.vx = ((*poses_)[i + 1].x - (*poses_)[i].x) / dt;
  out.vy = ((*poses_)[i + 1].y - (*poses_)[i].y) / dt;
  out.vyaw =
      angles::normalize_angle((*poses_)[i + 1].yaw - (*poses_)[i].yaw) / dt;

  return out;
}

bool TrajectoryPlanner::lostRobot(
    const TrajectoryPlanner::Profile& ref,
    const TrajectoryPlanner::Profile& odom) {
  const double dl = linearDistance(ref, odom);
  const double da = angularDistance(ref, odom);
  return (dl > 0.3 or da > 1.57);
}

bool TrajectoryPlanner::initTrajectory(
    const std::vector<geometry_msgs::PoseStamped>& plan, bool smooth) {
  const size_t n = plan.size();

  if (n < 1)
    return false;

  if (not t0_)
    t0_ = new ros::Time(ros::Time::now());

  poses_->resize(n + 1);

  const double xy_smoother  = smooth ? xy_smoother_ : 0.0;
  const double yaw_smoother = smooth ? yaw_smoother_ : 0.0;

  for (size_t i = 0; i < n; ++i) {
    if (i == 0) {
      (*poses_)[i].x   = plan[i].pose.position.x;
      (*poses_)[i].y   = plan[i].pose.position.y;
      (*poses_)[i].yaw = tf::getYaw(plan[i].pose.orientation);
    } else {
      double da = angles::normalize_angle(
          (*poses_)[i - 1].yaw - tf::getYaw(plan[i].pose.orientation));

      (*poses_)[i].x = xy_smoother * (*poses_)[i - 1].x +
                       (1 - xy_smoother) * plan[i].pose.position.x;
      (*poses_)[i].y = xy_smoother * (*poses_)[i - 1].y +
                       (1 - xy_smoother) * plan[i].pose.position.y;
      (*poses_)[i].yaw = angles::normalize_angle(
          tf::getYaw(plan[i].pose.orientation) + yaw_smoother * da);
      (*poses_)[i].t =
          (*poses_)[i - 1].t + timeIncrement((*poses_)[i], (*poses_)[i - 1]);
    }
  }

  (*poses_)[n].x   = plan[n - 1].pose.position.x;
  (*poses_)[n].y   = plan[n - 1].pose.position.y;
  (*poses_)[n].yaw = tf::getYaw(plan[n - 1].pose.orientation);
  (*poses_)[n].t =
      (*poses_)[n - 1].t + timeIncrement((*poses_)[n], (*poses_)[n - 1]);

  return true;
}

bool TrajectoryPlanner::updateTrajectory(
    const std::vector<geometry_msgs::PoseStamped>& plan, size_t init,
    bool smooth) {
  const size_t n = plan.size();

  if ((n <= heading_lookahead_ + 1) or
      (init >= poses_->size() - heading_lookahead_))
    return true;

  std::vector<Pose2D> new_poses(n - heading_lookahead_ + 1);
  new_poses.insert(new_poses.begin(), poses_->begin(), poses_->begin() + init);

  const double xy_smoother  = smooth ? xy_smoother_ : 0.0;
  const double yaw_smoother = smooth ? yaw_smoother_ : 0.0;

  for (size_t i = init, pi = heading_lookahead_;
       i < init + n - heading_lookahead_; ++i, ++pi) {
    double da = new_poses[i - 1].yaw - tf::getYaw(plan[pi].pose.orientation);

    new_poses[i].x = xy_smoother * new_poses[i - 1].x +
                     (1 - xy_smoother) * plan[pi].pose.position.x;
    new_poses[i].y = xy_smoother * new_poses[i - 1].y +
                     (1 - xy_smoother) * plan[pi].pose.position.y;
    new_poses[i].yaw = angles::normalize_angle(
        tf::getYaw(plan[pi].pose.orientation) + yaw_smoother * da);
    new_poses[i].t =
        new_poses[i - 1].t + timeIncrement(new_poses[i], new_poses[i - 1]);
  }

  size_t last         = init + n - heading_lookahead_;
  new_poses[last].x   = plan.back().pose.position.x;
  new_poses[last].y   = plan.back().pose.position.y;
  new_poses[last].yaw = tf::getYaw(plan.back().pose.orientation);
  new_poses[last].t   = new_poses[last - 1].t +
                      timeIncrement(new_poses[last], new_poses[last - 1]);

  delete poses_;
  poses_  = new std::vector<Pose2D>;
  *poses_ = new_poses;

  return true;
}

size_t TrajectoryPlanner::matchIndex(double t) const {
  if (t <= 0.0)
    return 0;
  if (t > poses_->back().t)
    return poses_->size();

  size_t i = poses_->size() / 2, l = 0, u = poses_->size();
  while (not(t > (*poses_)[i - 1].t and t <= (*poses_)[i].t)) {
    if (t > (*poses_)[i].t)
      l = i;
    else
      u = i;
    i   = (u + l) / 2;
  }

  return i;
};

void TrajectoryPlanner::reconfigureCallback(
    TrajectoryPlannerConfig& config, uint32_t level) {
  xy_smoother_       = config.xy_smoother;
  yaw_smoother_      = config.yaw_smoother;
  time_scaler_       = config.time_scaler;
  heading_lookahead_ = config.heading_lookahead;
}

TrajectoryPlanner* TrajectoryPlanner::trajectory_ptr_ = nullptr;

ros::Time* TrajectoryPlanner::t0_ = nullptr;

std::vector<TrajectoryPlanner::Pose2D>* TrajectoryPlanner::poses_ = nullptr;

dynamic_reconfigure::Server<TrajectoryPlannerConfig>* TrajectoryPlanner::dsrv_ =
    nullptr;

}  // namespace squirrel_navigation
