// Copyright (c) 2017, Federico Boniardi and Wolfram Burgard
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
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#ifndef SQUIRREL_NAVIGATION_FOOTPRINT_PLANNER_H_
#define SQUIRREL_NAVIGATION_FOOTPRINT_PLANNER_H_

#include "squirrel_navigation/FootprintPlannerConfig.h"

#include <ros/console.h>
#include <ros/publisher.h>
#include <ros/subscriber.h>
#include <ros/time.h>

#include <base_local_planner/costmap_model.h>
#include <nav_core/base_global_planner.h>

#include <costmap_2d/costmap_2d_ros.h>

#include <dynamic_reconfigure/server.h>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/MarkerArray.h>

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

class FootprintPlanner : public nav_core::BaseGlobalPlanner {
 public:
  class Params {
   public:
    static Params defaultParams();

    std::string footprint_topic;
    double collision_check_resolution;
    double max_planning_time, max_simplification_time;
    double waypoints_resolution;
    double range;
    bool verbose;
  };

 public:
  FootprintPlanner() : params_(Params::defaultParams()), init_(false) {}
  FootprintPlanner(const Params& params) : params_(params), init_(false) {}
  virtual ~FootprintPlanner() {}

  // Initialize the internal observers.
  virtual void initialize(
      std::string name, costmap_2d::Costmap2DROS* costmap_ros) override;

  // Interface to create a collision free path.
  virtual bool makePlan(
      const geometry_msgs::PoseStamped& start,
      const geometry_msgs::PoseStamped& goal,
      std::vector<geometry_msgs::PoseStamped>& waypoints) override;

  // Mutex getter.
  inline std::mutex& mutex() const { return footprint_mtx_; }

  // Parameters read/write utilities.
  inline const Params& params() const { return params_; }
  inline void setParams(const Params& params) { params_ = params; }
  inline Params& params() { return params_; }

 private:
  // Callbacks.
  void reconfigureCallback(FootprintPlannerConfig& config, uint32_t level);
  void footprintCallback(
      const geometry_msgs::PolygonStamped::ConstPtr& footprint);

  // OMPL related utilities.
  void initializeOMPLPlanner();
  bool checkValidState(
      const ompl::base::SpaceInformation* ompl_simple_setup,
      const ompl::base::State* state);

  // Utilities.
  void initializeFootprintMarker();
  void publishPath(
      const std::vector<geometry_msgs::PoseStamped>& waypoints,
      const ros::Time& stamp);
  void convertOMPLStatesToWayPoints(
      const std::vector<ompl::base::State*>& ompl_states,
      std::vector<geometry_msgs::PoseStamped>* waypoints);

 private:
  Params params_;
  std::unique_ptr<dynamic_reconfigure::Server<FootprintPlannerConfig>> dsrv_;

  visualization_msgs::Marker footprint_marker_;
  std::vector<geometry_msgs::Point> footprint_;
  std::unique_ptr<base_local_planner::CostmapModel> costmap_model_;
  std::shared_ptr<costmap_2d::Costmap2DROS> costmap_ros_;

  std::unique_ptr<ompl::geometric::SimpleSetup> ompl_simple_setup_;
  std::unique_ptr<ompl::base::RealVectorBounds> ompl_bounds_;
  ompl::base::PlannerPtr ompl_planner_;        
  ompl::base::StateSpacePtr ompl_state_space_; 

  ros::Subscriber footprint_sub_;
  ros::Publisher plan_pub_, waypoints_pub_, footprints_pub_;

  bool init_;
  bool ompl_need_reinitialization_;

  mutable std::mutex footprint_mtx_;
};

}  // namespace squirrel_navigation

#endif /* SQUIRREL_NAVIGATION_FOOTPRINT_PLANNER_H_ */
