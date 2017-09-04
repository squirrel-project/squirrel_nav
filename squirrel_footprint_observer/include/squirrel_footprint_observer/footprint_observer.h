// Copyright (C) 2017  Federico Boniardi and Wolfram Burgard
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.

#ifndef SQUIRREL_FOOTPRINT_OBSERVER_FOOTPRINT_OBSERVER_H_
#define SQUIRREL_FOOTPRINT_OBSERVER_FOOTPRINT_OBSERVER_H_

#include "squirrel_footprint_observer/FootprintObserverConfig.h"
#include "squirrel_footprint_observer/geometry_types.h"

#include <ros/ros.h>

#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <dynamic_reconfigure/server.h>

#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/PolygonStamped.h>
#include <squirrel_footprint_observer_msgs/dump_footprint.h>
#include <squirrel_footprint_observer_msgs/get_footprint.h>
#include <std_msgs/Bool.h>

#include <memory>
#include <mutex>
#include <vector>

namespace squirrel_footprint_observer {

class FootprintObserver {
 public:
  FootprintObserver();
  virtual ~FootprintObserver() {}

  // Spinner.
  void spin(double hz = 10.);

 private:
  // Reconfigure utilities.
  void reconfigureCallback(FootprintObserverConfig& config, uint32_t level);
  void enableCallback(const std_msgs::Bool::ConstPtr& msg);

  // Service callbacks.
  bool getFootprintServiceCallback(
      squirrel_footprint_observer_msgs::get_footprint::Request& req,
      squirrel_footprint_observer_msgs::get_footprint::Response& res);
  bool dumpFootprintServiceCallback(
      squirrel_footprint_observer_msgs::dump_footprint::Request& req,
      squirrel_footprint_observer_msgs::dump_footprint::Response& res);

  // Update the footprint.
  void updateFootprint(const ros::Time& stamp);

  // Mutex getter.
  inline std::mutex& mutex() const { return update_mtx_; }

 private:
  std::unique_ptr<dynamic_reconfigure::Server<FootprintObserverConfig>> dsrv_;

  std::vector<Point2D> base_footprint_;
  std::vector<Point2D> joint_footprint_;
  std::vector<std::string> joint_chain_;

  bool enabled_;

  geometry_msgs::PolygonStamped footprint_;

  ros::Publisher footprint_pub_;
  ros::Subscriber enable_sub_;
  ros::ServiceServer footprint_get_srv_, footprint_dump_srv_;
  tf::TransformListener tfl_;

  mutable std::mutex update_mtx_;
};

namespace {

constexpr size_t kBaseNumPoints  = 20;
constexpr size_t kJointNumPoints = 10;

}  // namespace

}  // namespace squirrel_footprint_observer

#endif /* SQUIRREL_FOOTPRINT_OBSERVER_FOOTPRINT_OBSERVER_H_ */
