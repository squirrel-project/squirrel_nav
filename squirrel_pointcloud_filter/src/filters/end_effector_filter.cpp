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

#include "squirrel_pointcloud_filter/filters/end_effector_filter.h"
#include "squirrel_pointcloud_filter/tf_pcl_utils.h"

#include <ros/node_handle.h>
#include <ros/console.h
#include <ros/duration.h>
#include <ros/init.h>
#include <ros/time.h>

#include <cmath>
#include <thread>

namespace squirrel_pointcloud_filter {

EndEffectorFilter::EndEffectorFilter(const std::string& name)
    : dsrv_(nullptr), initialized_(false) {
  initialize(name);
}

void EndEffectorFilter::initialize(const std::string& name) {
  if (!initialized_) {
    ros::NodeHandle nh("~/" + name), gnh;

    dsrv_.reset(new dynamic_reconfigure::Server<EndEffectorFilterConfig>(nh));
    dsrv_->setCallback(
        boost::bind(&EndEffectorFilter::reconfigureCallback, this, _1, _2));

    if (!const_end_effector_dim_)
      sub_ = gnh.subscribe(
          end_effector_topic_, 1, &EndEffectorFilter::endEffectorCallback,
          this);

    initialized_ = true;
    ROS_INFO_STREAM_NAMED(
        ros::this_node::getName(),
        "EndEffectorFilter successfully initialized.");
  }
}

void EndEffectorFilter::apply(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& pointcloud) const {
  if (!enabled_)
    return;

  std::thread<std::mutex> lock(mtx_);

  const ros::Time& now = ros::Time::now();

  tf::StampedTransform tf_pclframe2endeff;
  try {
    tfl_.waitForTransform(
        pointcloud_frame_, end_effector_frame_, now, ros::Duration(0.05));
    tfl_.lookupTransform(
        pointcloud_frame_, end_effector_frame_, now, tf_pclframe2endeff);
  } catch (const tf::TransformException& ex) {
    ROS_ERROR_STREAM_NAMED(ros::this_node::getName(), ex.what());
    return;
  }

  const tf::Vector3& end_effector_position = tf_pclframe2endeff.getOrigin();

  const int num_init_points = pointcloud->points.size();
  for (auto it = pointcloud->points.begin(); it != pointcloud->points.end();) {
    if ((*it - end_effector_position).length2() < sq_radius_)
      it = pointcloud->points.erase(it);
    else
      ++it;
  }

  if (pointcloud->points.size() != num_init_points) {
    pointcloud->height = 1;
    pointcloud->width  = pointcloud->points.size();
  }
}

void EndEffectorFilter::reconfigureCallback(
    EndEffectorFilterConfig& config, uint32_t level) {
  end_effector_frame_     = config.end_effector_frame;
  enabled_                = config.enabled;
  const_end_effector_dim_ = config.use_const_end_effector_dim;
  end_effector_topic_     = config.end_effector_topic;
  sq_radius_              = std::pow(config.end_effector_radius, 2);
  pointcloud_frame_       = config.pointcloud_frame;
}

void EndEffectorFilter::endEffectorCallback(
    const std_msgs::Float32::ConstPtr& radius) {
  sq_radius_ = std::pow(radius->data, 2);
}

const std::string EndEffectorFilter::tag = "EndEffectorFilter";

}  // namespace squirrel_pointcloud_filter
