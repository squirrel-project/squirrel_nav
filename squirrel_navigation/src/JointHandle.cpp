// Copyright (c) 2015, Federico Boniardi
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

#include "squirrel_navigation/JointHandle.h"

namespace squirrel_navigation {

JointHandle::JointHandle(void)
    : info_(false), moving_(false), verbose_(false), command_(false) {
  // Empty
}

JointHandle::JointHandle(const std::string& name)
    : name_(name),
      info_(false),
      moving_(false),
      verbose_(false),
      command_(false) {
  ros::NodeHandle pnh("~/" + name_);
  pnh.param<std::string>("command_topic", command_topic_, name_ + "/command");
  pnh.param<std::string>("state_topic", state_topic_, name_ + "/state");
  pnh.param<double>("navigation_angle", navigation_angle_, 0.6);
  pnh.param<bool>("use_navigation_angle", use_navigation_angle_, true);
  pnh.param<bool>("verbose", verbose_, false);

  cur_angle_ = navigation_angle_;

  state_sub_ =
      nh_.subscribe(state_topic_, 1, &JointHandle::stateCallback_, this);
  command_sub_ =
      nh_.subscribe(command_topic_, 1, &JointHandle::commandCallback_, this);
}

JointHandle::~JointHandle(void) {
  // Empty
}

void JointHandle::stateCallback_(
    const dynamixel_msgs::JointState::ConstPtr& joint_state) {
  moving_    = joint_state->is_moving;
  cur_angle_ = joint_state->current_pos;
  if (verbose_)
    ROS_INFO_STREAM(
        ros::this_node::getName() << "/" << name_ << ": joint is moving.");
}

void JointHandle::commandCallback_(const std_msgs::Float64::ConstPtr& cmd) {
  command_ = cmd->data;
  if (verbose_)
    ROS_INFO_STREAM(
        ros::this_node::getName() << "/" << name_ << ": got motion command.");
}

}  // namespace squirrel_navigation
