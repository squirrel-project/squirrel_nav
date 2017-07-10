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

#ifndef SQUIRREL_NAVIGATION_SAFETY_SCAN_OBSERVER_H_
#define SQUIRREL_NAVIGATION_SAFETY_SCAN_OBSERVER_H_

#include "squirrel_navigation/safety/observer.h"
#include "squirrel_navigation/utils/alpha_beta_filter.h"

#include <ros/console.h>
#include <ros/publisher.h>
#include <ros/subscriber.h>

#include <geometry_msgs/Pose.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <squirrel_navigation/ScanObserverConfig.h>

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
  ScanObserver() : params_(Params::defaultParams()) {}
  ScanObserver(const Params& params) : params_(params) {}
  virtual ~ScanObserver() {}

  // Initialize the internal observer.
  void initialize(const std::string& name);

  // Query safety of the current scan.
  bool safe() const override;
  
  // Parameter read/write utilities.
  inline const Params& params() const { return params_; }
  inline void setParams(const Params& params) { params_ = params; }
  inline Params& params() { return params_; }

 private:
  // Safety checks, no mutex lock.
  bool safeCheck(int i) const; 
  
  // Callbacks.
  void reconfigureCallback(ScanObserverConfig& config, uint32_t level);
  void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
  void pointCloudCallback(const sensor_msgs::PointCloud::ConstPtr& cloud);

  // Update the internal state.
  void updateState(const std::vector<float>& ranges, const ros::Time& stamp);

  // Publish visualization markers.
  geometry_msgs::Pose computeMarkerPose(int i) const;
  void publishMarkers(const ros::Time& stamp) const;
  
 private:
  Params params_;
  std::unique_ptr<dynamic_reconfigure::Server<ScanObserverConfig>> dsrv_;

  utils::AlphaBetaFilter ab_filter_;

  std::vector<float> ranges_, rangevelocities_;

  ros::Subscriber scan_sub_;
  ros::Publisher rangevels_pub_;

  mutable std::mutex mtx_;
};

}  // namespace safety

}  // namespace squirrel_navigation

#endif /* SQUIRREL_NAVIGATION_SAFETY_SCAN_OBSERVER_H_ */
