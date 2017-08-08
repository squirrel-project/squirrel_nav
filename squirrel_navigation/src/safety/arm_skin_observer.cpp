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

#include "squirrel_navigation/safety/arm_skin_observer.h"

#include <ros/node_handle.h>

#include <thread>

namespace squirrel_navigation {
namespace safety {

const std::string ArmSkinObserver::tag = "ArmSkinObserver";

ArmSkinObserver::ArmSkinObserver()
    : params_(Params::defaultParams()), hit_(false) {
  init_ = false;
}

ArmSkinObserver::ArmSkinObserver(const Params& params)
    : params_(params), hit_(false) {
  init_ = false;
}

void ArmSkinObserver::initialize(const std::string& name) {
  if (init_)
    return;
  // Initialize the parameter server.
  ros::NodeHandle pnh("~/" + name), nh;
  dsrv_.reset(new dynamic_reconfigure::Server<ArmSkinObserverConfig>(pnh));
  dsrv_->setCallback(
      boost::bind(&ArmSkinObserver::reconfigureCallback, this, _1, _2));
  // Subscriber.
  bump_sub_ =
      nh.subscribe(params_.topic_name, 1, &ArmSkinObserver::bumpCallback, this);
  // Intiialization successful.
  init_ = true;
  ROS_INFO_STREAM(
      "squirrel_navigation::safety::ArmSkinObserver: Initialization "
      "successful.");
}

bool ArmSkinObserver::safe() const {
  std::unique_lock<std::mutex> lock(mtx_);
  if (params_.verbose && params_.enabled && hit_)
    ROS_WARN_STREAM("squirrel_navigation::safety::ArmSkinObserver: Arm hit.");
  return !hit_ || !params_.enabled;
}

void ArmSkinObserver::reconfigureCallback(
    ArmSkinObserverConfig& config, uint32_t level) {
  params_.verbose    = config.verbose;
  params_.topic_name = config.topic_name;
  params_.enabled    = config.enabled;
}

void ArmSkinObserver::bumpCallback(const std_msgs::Bool::ConstPtr& bump) {
  std::unique_lock<std::mutex> lock(mtx_);
  hit_ = bump->data;
}

ArmSkinObserver::Params ArmSkinObserver::Params::defaultParams() {
  Params params;
  params.verbose    = false;
  params.topic_name = "/arm_skin";
  params.enabled    = true;
  return params;
}

}  // namespace safety
}  // namespace squirrel_navigation
