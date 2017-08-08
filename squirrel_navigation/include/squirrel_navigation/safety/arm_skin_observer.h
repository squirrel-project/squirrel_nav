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

#ifndef SQUIRREL_NAVIGATION_SAFETY_ARM_SKIN_OBSERVER_H_
#define SQUIRREL_NAVIGATION_SAFETY_ARM_SKIN_OBSERVER_H_

#include "squirrel_navigation/safety/observer.h"
#include "squirrel_navigation/ArmSkinObserverConfig.h"

#include <ros/console.h>
#include <ros/subscriber.h>

#include <dynamic_reconfigure/server.h>

#include <std_msgs/Bool.h>

#include <memory>
#include <mutex>

namespace squirrel_navigation {
namespace safety {

class ArmSkinObserver : public Observer {
 public:
  class Params {
   public:
    static Params defaultParams();

    std::string topic_name;
    bool enabled;
    bool verbose;
  };

 public:
  ArmSkinObserver();
  ArmSkinObserver(const Params& params);
  virtual ~ArmSkinObserver() {}

  // Initialize the internal observer.
  void initialize(const std::string& name);

  // Query safety of the current scan.
  bool safe() const override;

  // Parameter read/write utilities.
  inline const Params& params() const { return params_; }
  inline void setParams(const Params& params) { params_ = params; }
  inline Params& params() { return params_; }

 public:
  static const std::string tag;
  
 private:
  // Callbacks.
  void reconfigureCallback(ArmSkinObserverConfig& config, uint32_t level);
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
