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

#ifndef SQUIRREL_2D_LOCALIZER_LASER_MODEL_ROS_H_
#define SQUIRREL_2D_LOCALIZER_LASER_MODEL_ROS_H_

#include "squirrel_2d_localizer/LaserModelConfig.h"
#include "squirrel_2d_localizer/laser_model.h"

#include <dynamic_reconfigure/server.h>

#include <memory>

namespace squirrel_2d_localizer {

class LaserModelROS : public LaserModel {
 public:
  LaserModelROS() : LaserModel(), dsrv_(nullptr) { initialize(); }
  LaserModelROS(const LaserModel::Params& params);
  virtual ~LaserModelROS() {}

 private:
  void initialize();
  void reconfigureCallback(LaserModelConfig& config, uint32_t level);

 private:
  std::unique_ptr<dynamic_reconfigure::Server<LaserModelConfig>> dsrv_;
};

}  // namespace squirrel_2d_localizer

#endif /* SQUIRREL_2D_LOCALIZER_LASER_MODEL_ROS_H_ */
