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

#include "squirrel_navigation/safety/arm_skin_observer.h"

#include <ros/node_handle.h>

#include <thread>

namespace squirrel_navigation {
namespace safety {

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
