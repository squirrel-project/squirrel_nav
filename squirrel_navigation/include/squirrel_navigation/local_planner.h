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

#ifndef SQUIRREL_NAVIGATION_LOCAL_PLANNER_H_
#define SQUIRREL_NAVIGATION_LOCAL_PLANNER_H_

#include "squirrel_navigation/LocalPlannerConfig.h"
#include "squirrel_navigation/controller_pid.h"
#include "squirrel_navigation/linear_motion_planner.h"
#include "squirrel_navigation/safety/scan_observer.h"

#include <ros/console.h>
#include <ros/publisher.h>
#include <ros/subscriber.h>

#include <base_local_planner/costmap_model.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <nav_core/base_local_planner.h>

#include <pluginlib/class_list_macros.h>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <squirrel_navigation_msgs/BrakeRobot.h>
#include <std_srvs/Empty.h>

#include <dynamic_reconfigure/server.h>

#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <mutex>
#include <string>
#include <vector>

namespace squirrel_navigation {

class LocalPlanner : public nav_core::BaseLocalPlanner {
 public:
  class Params {
   public:
    static Params defaultParams();

    std::string odom_topic, footprint_topic;
    double goal_ang_tolerance, goal_lin_tolerance;
    double max_safe_lin_velocity, max_safe_ang_velocity;
    double max_safe_lin_displacement, max_safe_ang_displacement;
    std::vector<std::string> safety_observers;
    bool collision_based_replanning;
    double replanning_lin_lookahead, replanning_ang_lookahead;
    double replanning_path_length_ratio;
    bool visualize_topics;
    bool verbose;
  };

 public:
  LocalPlanner() : params_(Params::defaultParams()), init_(false) {}
  LocalPlanner(const Params& params) : params_(params), init_(false) {}
  virtual ~LocalPlanner() {}

  // Initialization with full map/costmap structure.
  void initialize(
      std::string name, tf::TransformListener* tfl,
      costmap_2d::Costmap2DROS* costmap_ros) override;

  // Compute velocity commands to input in the robot.
  bool computeVelocityCommands(geometry_msgs::Twist& cmd) override;

  // Check accomplishment of goal reaching task.
  bool isGoalReached() override;

  // Update the current path.
  bool setPlan(
      const std::vector<geometry_msgs::PoseStamped>& waypoints) override;

  // Parameter read/write utilities.
  inline const Params& params() const { return params_; }
  inline void setParams(const Params& params) { params_ = params; }
  inline Params& params() { return params_; }

 private:
  class BaseBrake {
   public:
    BaseBrake() : enable_stamp_(nullptr) {}
    virtual ~BaseBrake() {}

   public:
    bool spin(geometry_msgs::Twist* cmd);

    void enable(const ros::Duration& duration);
    void disable();

   private:
    std::unique_ptr<ros::Time> enable_stamp_;
    ros::Duration enable_time_;
  };

 private:
  // Callbacks.
  void footprintCallback(const geometry_msgs::PolygonStamped::ConstPtr& msg);
  void odomCallback(const nav_msgs::Odometry::ConstPtr& odom);
  void reconfigureCallback(LocalPlannerConfig& config, uint32_t level);

  bool brakeRobotCallback(
      squirrel_navigation_msgs::BrakeRobot::Request& req,
      squirrel_navigation_msgs::BrakeRobot::Response& res);
  bool unbrakeRobotCallback(
      std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);

  // Visualization utils.
  void publishTrajectory(const ros::Time& stamp) const;
  void publishFootprints(const ros::Time& stamp);
  void publishReference(
      const geometry_msgs::Pose& ref_pose, const ros::Time& stamp) const;
  void publishTwist(
      const geometry_msgs::PoseStamped& actuation_pose,
      const geometry_msgs::Twist& cmd) const;

  // Twist tranformation utilities.
  void twistToGlobalFrame(
      const geometry_msgs::Twist& robot_twist,
      geometry_msgs::Twist* map_twist) const;
  void twistToRobotFrame(
      const geometry_msgs::Twist& map_twist,
      geometry_msgs::Twist* robot_twist) const;
  void safeVelocityCommands(
      const geometry_msgs::Twist& twist,
      geometry_msgs::Twist* safe_twist) const;
  
  // Check if path is collision free.
  bool isTrajectorySafe(
      const std::vector<geometry_msgs::PoseStamped>& waypoints) const;
  bool needReplanning(
      const std::vector<geometry_msgs::PoseStamped>& old_waypoints,
      const std::vector<geometry_msgs::PoseStamped>& new_waypoints) const;

  // Check if a new goal is input.
  bool newGoal(const geometry_msgs::Pose& pose) const;

 private:
  Params params_;
  std::unique_ptr<dynamic_reconfigure::Server<LocalPlannerConfig>> dsrv_;

  std::string odom_topic_;

  std::unique_ptr<utils::Controller> controller_;
  std::unique_ptr<utils::MotionPlanner> motion_planner_;

  std::vector<std::unique_ptr<safety::Observer>> safety_observers_;

  geometry_msgs::TwistStamped robot_twist_;
  geometry_msgs::PoseStamped robot_pose_;
  std::unique_ptr<geometry_msgs::Pose> current_goal_;
  std::shared_ptr<tf::TransformListener> tfl_;
  std::shared_ptr<costmap_2d::Costmap2DROS> costmap_ros_;

  ros::Publisher ref_pub_, traj_pub_, footprints_pub_, cmd_pub_;
  ros::Subscriber odom_sub_, footprint_sub_;
  ros::ServiceServer brake_srv_, unbrake_srv_;

  std::vector<geometry_msgs::Point> footprint_;
  double inscribed_radius_, circumscribed_radius_;
  std::unique_ptr<base_local_planner::CostmapModel> costmap_model_;

  int last_nwaypoints_;
  
  bool init_;

  BaseBrake base_brake_;
  
  mutable std::mutex state_mtx_;
};

}  // namespace squirrel_navigation

#endif /* SQUIRREL_NAVIGATION_LOCAL_PLANNER_H_ */
