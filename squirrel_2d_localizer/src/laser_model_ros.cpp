// The MIT License (MIT)
//
// Copyright (c) 2016-2017 Federico Boniardi and Wolfram Burgard
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

#include "squirrel_2d_localizer/laser_model_ros.h"

#include <ros/node_handle.h>

namespace squirrel_2d_localizer {

LaserModelROS::LaserModelROS(const LaserModel::Params& params)
    : LaserModel(params), dsrv_(nullptr) {
  initialize();
}

void LaserModelROS::initialize() {
  ros::NodeHandle pnh("~/laser_model");
  dsrv_.reset(new dynamic_reconfigure::Server<LaserModelConfig>(pnh));
  dsrv_->setCallback(
      boost::bind(&LaserModelROS::reconfigureCallback, this, _1, _2));
}

void LaserModelROS::reconfigureCallback(
    LaserModelConfig& config, uint32_t level) {
  params_.endpoints_min_distance = config.beams_min_distance;
}

}  // namespace squirrel_2d_localizer
