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

#ifndef SQUIRREL_NAVIGATION_LOCAL_PLANNER_H_
#define SQUIRREL_NAVIGATION_LOCAL_PLANNER_H_

#include "squirrel_navigation/controller_pid.h"
#include "squirrel_navigation/linear_motion_planner.h"
#include "squirrel_navigation/safety/scan_observer.h"

#include <ros/console.h>
#include <ros/publisher.h>
#include <ros/subscriber.h>

#include <nav_core/base_local_planner.h>

#include <pluginlib/class_list_macros.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <squirrel_navigation/LocalPlannerConfig.h>

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

    double goal_ang_tolerance, goal_lin_tolerance;
    double max_safe_lin_velocity, max_safe_ang_velocity;
    double max_safe_lin_displacement, max_safe_ang_displacement;
    std::vector<std::string> safety_observers;
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
  // Callbacks.
  void odomCallback(const nav_msgs::Odometry::ConstPtr& odom);
  void reconfigureCallback(LocalPlannerConfig& config, uint32_t level);

  // Visualization utils.
  void publishTrajectory(const ros::Time& stamp) const;
  void publishReference(
      const geometry_msgs::Pose& ref_pose, const ros::Time& stamp) const;

  // Twist tranformation utilities.
  void robotToGlobalFrame(
      const geometry_msgs::Twist& robot_twist,
      geometry_msgs::Twist* map_twist) const;
  void globalToRobotFrame(
      const geometry_msgs::Twist& map_twist,
      geometry_msgs::Twist* robot_twist) const;
  void safeVelocityCommands(
      const geometry_msgs::Twist& twist,
      geometry_msgs::Twist* safe_twist) const;

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

  ros::Publisher ref_pub_, traj_pub_;
  ros::Subscriber odom_sub_;

  bool init_;
  
  mutable std::mutex state_mtx_;
};

}  // namespace squirrel_navigation

#endif /* SQUIRREL_NAVIGATION_LOCAL_PLANNER_H_ */
