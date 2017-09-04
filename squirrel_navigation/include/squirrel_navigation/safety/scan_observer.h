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

#ifndef SQUIRREL_NAVIGATION_SAFETY_SCAN_OBSERVER_H_
#define SQUIRREL_NAVIGATION_SAFETY_SCAN_OBSERVER_H_

#include "squirrel_navigation/ScanObserverConfig.h"
#include "squirrel_navigation/safety/observer.h"
#include "squirrel_navigation/utils/alpha_beta_filter.h"

#include <ros/console.h>
#include <ros/publisher.h>
#include <ros/subscriber.h>

#include <geometry_msgs/Pose.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>

#include <tf/transform_listener.h>

#include <dynamic_reconfigure/server.h>

#include <memory>
#include <mutex>
#include <string>

namespace squirrel_navigation {
namespace safety {

class ScanObserver : public Observer {
 public:
  class Params {
   public:
    static Params defaultParams();

    bool enabled;
    std::string scan_topic;
    double max_safety_rangevel;
    double unsafe_range;
    bool verbose;
    // Scan parameters.
    double scan_angle_increment;
    double scan_angle_min;
    std::string scan_frame_id;
  };

 public:
  ScanObserver() : params_(Params::defaultParams()) { init_ = false; }
  ScanObserver(const Params& params) : params_(params) { init_ = false; }
  virtual ~ScanObserver() {}

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
  // Safety checks, no mutex lock.
  bool safeCheck(int i) const;

  // Callbacks.
  void reconfigureCallback(ScanObserverConfig& config, uint32_t level);
  void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& scan);

  // Update the internal state.
  void updateState(const std::vector<float>& ranges, const ros::Time& stamp);

  // Publish visualization markers.
  geometry_msgs::Pose computeMarkerPose(int i) const;
  double computeMarkerArrowLength(int i) const;
  void publishMarkers(const ros::Time& stamp) const;
  void publishMessage(const ros::Time& stamp) const;
  
 private:
  Params params_;
  std::unique_ptr<dynamic_reconfigure::Server<ScanObserverConfig>> dsrv_;

  utils::AlphaBetaFilter ab_filter_;

  std::vector<float> ranges_, rangevelocities_;

  ros::Subscriber scan_sub_;
  ros::Publisher rangevels_pub_, safety_log_pub_;

  mutable std::mutex mtx_;
};

}  // namespace safety
}  // namespace squirrel_navigation

#endif /* SQUIRREL_NAVIGATION_SAFETY_SCAN_OBSERVER_H_ */
