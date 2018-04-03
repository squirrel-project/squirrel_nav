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

#include "squirrel_footprint_observer/arm_folding_observer.h"

#include <dynamic_reconfigure/BoolParameter.h>
#include <dynamic_reconfigure/Config.h>
#include <dynamic_reconfigure/Reconfigure.h>

#include <ros/service.h>

#include <iomanip>
#include <stdexcept>
#include <utility>

namespace squirrel_footprint_observer {

void ArmFoldingObserver::initialize(const std::string &name) {
  ros::NodeHandle nh("~/" + name);

  std::vector<std::string> joint_names;
  std::vector<double> joint_values;

  nh.param<bool>("verbose", verbose_, false);
  nh.param<std::string>("joint_states_topic", joint_states_topic_, "/states");
  nh.param<std::vector<std::string>>("joint_names", joint_names, {});
  nh.param<std::vector<double>>("ref_joint_states", joint_values, {});

  if (joint_names.size() != joint_values.size()) {
    ROS_ERROR_STREAM_NAMED(
        "arm_folding_observer",
        "'joint_names' and 'ref_joint_states' not of the same size.");
    ros::shutdown();
  }

  // Zip the vectors.
  for (unsigned int i = 0; i < joint_names.size(); ++i)
    joints_values_.emplace(joint_names[i], joint_values[i]);

  // Subscribe to the joint states.
  joint_states_sub_ = gnh_.subscribe(
      joint_states_topic_, 1, &ArmFoldingObserver::jointStatesCallback, this);
}

void ArmFoldingObserver::jointStatesCallback(
    const sensor_msgs::JointState::ConstPtr &joint_state) {
  if (verbose_)
    ROS_INFO_STREAM_ONCE_NAMED("arm_folding_observer",
                               "Subscribed to '" << joint_states_topic_
                                                 << "'.");
  if (!joint_index_map_)
    buildJointIndexMap(joint_state->name);

  gnh_.getParamCached("/move_base/GlobalPlanner/plan_with_footprint",
                      plan_with_footprint_);

  const auto &positions = joint_state->position;

  bool plan_with_footprint = false;
  for (const auto &joint_value : joints_values_) {
    if (!joints_exists_[joint_value.first])
      continue;

    const std::string &joint_name = joint_value.first;
    const double joint_ref_value = joint_value.second;
    const int joint_index = joint_index_map_->at(joint_name);
    const double joint_cur_value = positions[joint_index];

    if (std::abs(joint_ref_value - joint_cur_value) > kJointValueTolerance) {
      plan_with_footprint = true;
      break;
    }
  }

  if (plan_with_footprint != plan_with_footprint_)
    toggleFootprintPlanner(plan_with_footprint);
}

void ArmFoldingObserver::toggleFootprintPlanner(bool on_off) const {
  dynamic_reconfigure::BoolParameter bool_param;
  bool_param.name = "plan_with_footprint";
  bool_param.value = on_off;

  dynamic_reconfigure::Reconfigure reconfigure;
  reconfigure.request.config.bools.emplace_back(bool_param);

  ros::service::waitForService("/move_base/GlobalPlanner/set_parameters");
  
  if (ros::service::call("/move_base/GlobalPlanner/set_parameters",
                         reconfigure)) {
    ROS_INFO_STREAM_COND_NAMED(verbose_, "arm_folding_observer",
                               "Set 'plan_with_footprint' to " << std::boolalpha
                                                               << on_off);
  } else {
    ROS_WARN_STREAM_NAMED("arm_folding_observer",
                          "Tried to set 'plan_with_footprint' to"
                              << std::boolalpha << on_off
                              << ". Didn't work...");
  }
}

void ArmFoldingObserver::buildJointIndexMap(
    const std::vector<std::string> &joint_msg_names) {
  joint_index_map_.reset(new std::map<std::string, int>);

  const auto begin = joint_msg_names.begin();
  const auto end = joint_msg_names.end();

  for (const auto &joint_value : joints_values_) {
    const auto &joint_name = joint_value.first;

    auto it = std::find(begin, end, joint_name);

    if (it == end) {
      ROS_WARN_STREAM_NAMED("arm_folding_observer",
                            joint_name << "does not appear to be published.");
      joints_exists_.emplace(joint_name, false);
    } else {
      joint_index_map_->emplace(joint_name, (int)std::distance(begin, it));
      joints_exists_.emplace(joint_name, true);
    }
  }
}

} // namespace squirrel_footprint_observer
