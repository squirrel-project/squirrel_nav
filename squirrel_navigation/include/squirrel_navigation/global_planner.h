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

#ifndef SQUIRREL_NAVIGATION_GLOBAL_PLANNER_H_
#define SQUIRREL_NAVIGATION_GLOBAL_PLANNER_H_

#include "squirrel_navigation/footprint_planner.h"

#include <ros/subscriber.h>

#include <nav_core/base_global_planner.h>
#include <navfn/navfn_ros.h>

#include <dynamic_reconfigure/server.h>

#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <geometry_msgs/Polygon.h>
#include <nav_msgs/Path.h>
#include <squirrel_navigation/GlobalPlannerConfig.h>

#include <memory>
#include <string>
#include <vector>

namespace squirrel_navigation {

class GlobalPlanner : public nav_core::BaseGlobalPlanner {
 public:
  class Params {
   public:
    static Params defaultParams();

    std::string footprint_topic;
    bool plan_with_footprint;
    bool plan_with_constant_heading;
    double heading;
    bool verbose;
  };

 public:
  GlobalPlanner() : params_(Params::defaultParams()), init_(false) {}
  GlobalPlanner(const Params& params) : params_(params), init_(false) {}
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
  // Reconfigure the callback.
  void reconfigureCallback(GlobalPlannerConfig& config, uint32_t level);

 private:
  Params params_;
  std::unique_ptr<dynamic_reconfigure::Server<GlobalPlannerConfig>> dsrv_;

  std::unique_ptr<navfn::NavfnROS> dijkstra_planner_;
  std::unique_ptr<FootprintPlanner> footprint_planner_;

  bool init_;
  
  std::shared_ptr<costmap_2d::Costmap2DROS> costmap_ros_;
};

}  // namespace squirrel_navigation

#endif /* SQUIRREL_NAVIGATION_GLOBAL_PLANNER_H_ */
