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

#ifndef SQUIRREL_NAVIGATION_UTILS_MATH_UTILS_H_
#define SQUIRREL_NAVIGATION_UTILS_MATH_UTILS_H_

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Twist.h>

#include <tf/tf.h>

#include <angles/angles.h>

#include <cmath>
#include <vector>

namespace squirrel_navigation {
namespace math {

// SE2 transforms.
inline geometry_msgs::Point applyTransform2D(
    const geometry_msgs::Pose& tf_pose, const geometry_msgs::Point& point) {
  const double c = std::cos(tf::getYaw(tf_pose.orientation));
  const double s = std::sin(tf::getYaw(tf_pose.orientation));
  geometry_msgs::Point output;
  output.x = c * point.x - s * point.y + tf_pose.position.x;
  output.y = s * point.x + c * point.y + tf_pose.position.y;
  return output;
}

inline geometry_msgs::Point applyTransform2D(
    const tf::Transform& tf, const geometry_msgs::Point& point) {
  geometry_msgs::Pose tf_pose;
  tf::poseTFToMsg(tf, tf_pose);
  return applyTransform2D(tf_pose, point);
}

// Simple increment utilities.
template <int N>
inline double delta(
    const geometry_msgs::Pose& q1, const geometry_msgs::Pose& q2) {
  if (N < 1)
    return q2.position.x - q1.position.x;
  else if (N == 1)
    return q2.position.y - q1.position.y;
  else
    return angles::normalize_angle(
        tf::getYaw(q2.orientation) - tf::getYaw(q1.orientation));
}

template <int N>
inline double delta(
    const geometry_msgs::Twist& t1, const geometry_msgs::Twist& t2) {
  if (N < 1)
    return t2.linear.x - t1.linear.x;
  else if (N == 1)
    return t2.linear.y - t1.linear.y;
  else
    return t2.angular.z - t1.angular.z;
}

// Linear distance in 2D.
inline double linearDistance2D(
    const geometry_msgs::Point32& p1, const geometry_msgs::Point32& p2) {
  return std::hypot(p1.x - p2.x, p1.y - p2.y);
}

inline double linearDistance2D(
    const geometry_msgs::Point32& p1, const geometry_msgs::Point& p2) {
  return std::hypot(p1.x - p2.x, p1.y - p2.y);
}

inline double linearDistance2D(
    const geometry_msgs::Point& p1, const geometry_msgs::Point32& p2) {
  return std::hypot(p1.x - p2.x, p1.y - p2.y);
}

inline double linearDistance2D(
    const geometry_msgs::Point& p1, const geometry_msgs::Point& p2) {
  return std::hypot(p1.x - p2.x, p1.y - p2.y);
}

inline double linearDistance2D(
    const geometry_msgs::Pose& q1, const geometry_msgs::Pose& q2) {
  return linearDistance2D(q1.position, q2.position);
}

inline double linearDistance2D(
    const geometry_msgs::PoseStamped& q1,
    const geometry_msgs::PoseStamped& q2) {
  return linearDistance2D(q1.pose, q2.pose);
}

// Yaw distance.
inline double angularDistanceYaw(
    const geometry_msgs::Quaternion& q1, const geometry_msgs::Quaternion& q2) {
  return std::abs(angles::normalize_angle(tf::getYaw(q1) - tf::getYaw(q2)));
}

inline double angularDistanceYaw(
    const geometry_msgs::Pose& q1, const geometry_msgs::Pose& q2) {
  return angularDistanceYaw(q1.orientation, q2.orientation);
}

inline double angularDistanceYaw(
    const geometry_msgs::PoseStamped& q1,
    const geometry_msgs::PoseStamped& q2) {
  return angularDistanceYaw(q1.pose, q2.pose);
}

// Linear interpolation
inline geometry_msgs::Point linearInterpolation2D(
    const geometry_msgs::Point& p1, const geometry_msgs::Point& p2,
    double alpha) {
  geometry_msgs::Point out;
  out.x = p1.x + alpha * (p2.x - p1.x);
  out.y = p1.y + alpha * (p2.y - p1.y);
  return out;
}

inline geometry_msgs::Point linearInterpolation2D(
    const geometry_msgs::Pose& q1, const geometry_msgs::Pose& q2,
    double alpha) {
  return linearInterpolation2D(q1.position, q2.position, alpha);
}

inline geometry_msgs::Point linearInterpolation2D(
    const geometry_msgs::PoseStamped& q1, const geometry_msgs::PoseStamped& q2,
    double alpha) {
  return linearInterpolation2D(q1.pose, q2.pose, alpha);
}

// Angular interpolation
inline geometry_msgs::Quaternion slerpYaw(
    const geometry_msgs::Quaternion& q1, const geometry_msgs::Quaternion& q2,
    double alpha) {
  const double a1    = tf::getYaw(q1);
  const double da    = angles::normalize_angle(tf::getYaw(q2) - a1);
  const double slerp = angles::normalize_angle(a1 + alpha * da);
  return tf::createQuaternionMsgFromYaw(slerp);
}

inline geometry_msgs::Quaternion slerpYaw(
    const geometry_msgs::Pose& q1, const geometry_msgs::Pose& q2,
    double alpha) {
  return slerpYaw(q1.orientation, q2.orientation, alpha);
}

inline geometry_msgs::Quaternion slerpYaw(
    const geometry_msgs::PoseStamped& q1, const geometry_msgs::PoseStamped& q2,
    double alpha) {
  return slerpYaw(q1.pose, q2.pose, alpha);
}

inline double pathLength(const std::vector<geometry_msgs::Point>& waypoints) {
  double length = 0.;
  for (unsigned int i = 1; i < waypoints.size(); ++i)
    length += linearDistance2D(waypoints[i - 1], waypoints[i]);
  return length;
}

inline double pathLength(const std::vector<geometry_msgs::Pose>& waypoints) {
  double length = 0.;
  for (unsigned int i = 1; i < waypoints.size(); ++i)
    length += linearDistance2D(waypoints[i - 1], waypoints[i]);
  return length;
}

inline double pathLength(
    const std::vector<geometry_msgs::PoseStamped>& waypoints) {
  double length = 0.;
  for (unsigned int i = 1; i < waypoints.size(); ++i)
    length += linearDistance2D(waypoints[i - 1], waypoints[i]);
  return length;
}

}  // namespace math
}  // namespace squirrel_navigation

#endif /* SQUIRREL_NAVIGATION_UTILS_MATH_UTILS_H_ */
