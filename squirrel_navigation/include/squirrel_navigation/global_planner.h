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

#ifndef SQUIRREL_NAVIGATION_GLOBAL_PLANNER_H_
#define SQUIRREL_NAVIGATION_GLOBAL_PLANNER_H_

#include "squirrel_navigation/GlobalPlannerConfig.h"
#include "squirrel_navigation/footprint_planner.h"

#include <ros/publisher.h>
#include <ros/subscriber.h>

#include <nav_core/base_global_planner.h>
#include <navfn/navfn_ros.h>

#include <dynamic_reconfigure/server.h>

#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Bool.h>

#include <memory>
#include <string>
#include <vector>

namespace squirrel_navigation {

class GlobalPlanner : public nav_core::BaseGlobalPlanner {
 public:
  class Params {
   public:
    static Params defaultParams();

    bool plan_with_footprint;
    bool plan_with_constant_heading;
    bool visualize_topics;
    double heading;
    bool verbose;
  };

 public:
  GlobalPlanner();
  GlobalPlanner(const Params& params);
  virtual ~GlobalPlanner() {}

  // Initialize the state observers.
  void initialize(
      std::string name, costmap_2d::Costmap2DROS* costmap_ros) override;

  // Compute a collision free path.
  bool makePlan(
      const geometry_msgs::PoseStamped& start,
      const geometry_msgs::PoseStamped& goal,
      std::vector<geometry_msgs::PoseStamped>& waypoints) override;

  // Parameters read/write utilities.
  inline const Params& params() const { return params_; }
  inline void setParams(const Params& params) { params_ = params; }
  inline Params& params() { return params_; }

 private:
  // Callbacks.
  void reconfigureCallback(GlobalPlannerConfig& config, uint32_t level);
  
  // Publishing utilities.
  void publishPlan(
      const std::vector<geometry_msgs::PoseStamped>& waypoints,
      const ros::Time& stamp) const;
  void publishWaypoints(
      const std::vector<geometry_msgs::PoseStamped>& waypoints,
      const ros::Time& stamp) const;
  void publishFootprints(
      const std::vector<geometry_msgs::PoseStamped>& waypoints,
      const ros::Time& stamp);

 private:
  Params params_;
  std::unique_ptr<dynamic_reconfigure::Server<GlobalPlannerConfig>> dsrv_;

  std::unique_ptr<navfn::NavfnROS> dijkstra_planner_;
  std::unique_ptr<FootprintPlanner> footprint_planner_;

  ros::Publisher plan_pub_, waypoints_pub_, footprints_pub_;
  
  bool init_;
  
  int last_nwaypoints_;

  std::shared_ptr<costmap_2d::Costmap2DROS> costmap_ros_;
};

}  // namespace squirrel_navigation

#endif /* SQUIRREL_NAVIGATION_GLOBAL_PLANNER_H_ */
