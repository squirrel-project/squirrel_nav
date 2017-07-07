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

#ifndef SQUIRREL_NAVIGATION_SAFETY_ARM_SKIN_OBSERVER_H_
#define SQUIRREL_NAVIGATION_SAFETY_ARM_SKIN_OBSERVER_H_

#include "squirrel_navigation/safety/observer.h"

#include <ros/console.h>
#include <ros/subscriber.h>

#include <dynamic_reconfigure/server.h>

#include <squirrel_navigation/ArmSkinObserverConfig.h>
#include <std_msgs/Bool.h>

#include <memory>
#include <mutex>

namespace squirrel_navigation {

namespace safety {

class ArmSkinObserver : public Observer {
 public:
  class Params {
    static Params defaultParams();

    std::string topic_name;
    bool enabled;
    bool verbose;
  };

 public:
  ArmSkinObserver() : params_(Params::defaultParams()), hit_(false) {}
  ArmSkinObserver(const Params& params) : params_(params),hit_(false) {}
  virtual ~ArmSkinObserver() {}

  // Initialize the internal observer.
  void initialize(const std::string& name);

  // Query safety of the current scan.
  bool safe() const override;

  // Parameter read/write utilities.
  inline const Params& params() const { return params_; }
  inline void setParams(const Params& params) { params_ = params; }
  inline Params& params() { return params_; }

 private:
  // Callbacks.
  void reconfigureCallback(ArnSkinObserverConfig& config, uint32_t level);
  void bumpCallback(const std_msgs::Bool::ConstPtr& bump);

 private:
  Params params_;
  std::unique_ptr<dynamic_reconfigure::Server<ArmSkinObserverConfig>> dsrv_;

  bool hit_;
  
  ros::Subscriber bump_sub_;

  mutable std::mutex mtx_;
};

}  // namespace safety

}  // namespace squirrel_navigation

#endif /* SQUIRREL_NAVIGATION_SAFETY_ARM_SKIN_OBSERVER_H_ */
