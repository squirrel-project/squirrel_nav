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

#ifndef SQUIRREL_FOOTPRINT_OBSERVER_ARM_FOLDING_OBSERVER_H_
#define SQUIRREL_FOOTPRINT_OBSERVER_ARM_FOLDING_OBSERVER_H_

#include <sensor_msgs/JointState.h>

#include <ros/init.h>
#include <ros/subscriber.h>
#include <ros/node_handle.h>

#include <map>
#include <string>
#include <vector>

namespace squirrel_footprint_observer {
namespace {

constexpr double kJointValueTolerance = 1e-1;

}

class ArmFoldingObserver {
 public:
  ArmFoldingObserver() : joint_index_map_(nullptr) {}
  virtual ~ArmFoldingObserver() {}

  void initialize(const std::string& name = "");
  
  inline bool folded() const { return !plan_with_footprint_; }

  // Spinners.
  inline void spin() { ros::spin(); }
  inline void spinOnce() { ros::spinOnce(); }
  
 private:
  void jointStatesCallback(
      const sensor_msgs::JointState::ConstPtr& joint_state);

  void toggleFootprintPlanner(bool on_off) const;
  void buildJointIndexMap(const std::vector<std::string>& joint_msg_names);
  
 private:
  bool verbose_;
  std::string joint_states_topic_;
  std::map<std::string, double> joints_values_;

  bool plan_with_footprint_;
  std::unique_ptr<std::map<std::string, int>> joint_index_map_;

  std::map<std::string, bool> joints_exists_;

  ros::Subscriber joint_states_sub_;

  ros::NodeHandle gnh_;
};

}  // namespace squirrel_footprint_observer

#endif /* SQUIRREL_FOOTPRINT_OBSERVER_ARM_FOLDING_OBSERVER_H_ */
