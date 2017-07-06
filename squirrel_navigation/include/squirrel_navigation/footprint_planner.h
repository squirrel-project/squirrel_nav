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

#ifndef SQUIRREL_NAVIGATION_FOOTPRINT_PLANNER_H_
#define SQUIRREL_NAVIGATION_FOOTPRINT_PLANNER_H_

#include <ros/console.h>
#include <ros/publisher.h>
#include <ros/time.h>

#include <base_local_planner/costmap_model.h>
#include <nav_core/base_global_planner.h>

#include <costmap_2d/costmap_2d_ros.h>

#include <dynamic_reconfigure/server.h>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/PoseStamped.h>
#include <squirrel_navigation/FootPrintPluginConfig.h>

#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/geometric/PathSimplifier.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>

#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace squirrel_navigation {

class FootPrintPlanner : public nav_core::BaseGlobalPlanner {
 public:
  class Params {
   public:
    static Params defaultParams();

    std::string footprint_topic;
    double collision_check_resolution;
    double max_planning_time, max_simplification_time;
    double map_resolution;
    double range;
    bool verbose;
  };

 public:
  FootPrintPlanner() : params_(Params::defaultParams()) {}
  FootPrintPlanner(const Params& params) : params_(params) {}
  virtual ~FootPrintPlanner() {}

  // Initialize the internal observers.
  virtual void initialize(
      std::string name, costmap_2d::Costmap2DROS* costmap_ros) override;

  // Interface to create a collision free path.
  virtual bool makePlan(
      const geometry_msgs::PoseStamped& start,
      const geometry_msgs::PoseStamped& goal,
      std::vector<geometry_msgs::PoseStamped>& waypoints) override;

  // Parameters read/write utilities.
  inline const Params& params() const { return params_; }
  inline void setParams(const Params& params) { params_ = params; }
  inline Params& params() { return params_; }
  
 private:
  // Callbacks.
  void reconfigureCallback(FootPrintPlanner& config, uint32_t level);
  void footprintCallback(const geometry_msgs::Polygon::ConstPtr& footprint);

  // OMPL related utilities.
  void initializeOMPLPlanner();
  bool checkValidState();

  // Utilities.
  void publishPath(
      const std::vector<geometry_msgs::PoseStamped>& waypoints,
      const ros::Time& stamp) const;
  void convertOMPLStatesToWayPoints(
      const std::vector<ompl::base::State*>& ompl_states,
      std::vector<geometry_msgs::PoseStamped>* waypoints) const;

 private:
  Params params_;
  std::unique_ptr<dynamic_reconfigure::Server<FootPrintPlannerConfig>> dsrv_;

  std::vector<geometry_msgs::Point> footprint_;
  std::unique_ptr<base_local_planner::CostmapModel> costmap_model_;
  std::shared_ptr<costmap_2d::Costmap2DROS> costmap_ros_;

  std::shared_ptr<ompl::base::RRTstar> ompl_rrt_planner_;
  std::shared_ptr<ompl::base::StateSpace> ompl_state_space_;
  std::unique_ptr<ompl::base::SimpleSetup> ompl_simple_setup_;
  std::unique_ptr<ompl::base::RealVectorBounds> bounds_;

  ros::Subscriber footprint_sub_;
  ros::Publisher plan_pub_, waypoints_pub_;

  bool ompl_need_reinitialization_;

  mutable std::mutex footprint_mtx_;
};

}  // namespace squirrel_navigation

#endif /* SQUIRREL_NAVIGATION_FOOTPRINT_PLANNER_H_ */
