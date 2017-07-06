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

#ifndef SQUIRREL_NAVIGATION_MATH_UTILS_H_
#define SQUIRREL_NAVIGATION_MATH_UTILS_H_

#include <geometry_msgs/Point.h
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Twist.h>

#include <tf/tf.h>

#include <angles/angles.h>

#include <cmath>

namespace squirrel_navigation {

namespace math {

// Simple utilities.
template <int N>
inline double delta(
    const geometry_msgs::Pose& q1, const geometry_msgs::Pose& q2) const {
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
  return std::abs(angle::normalize_angle(tf::getYaw(q1) - tf::getYaw(q2)));
}

inline double angularDistanceYaw(
    const geometry_msgs::Pose& q1, const geometry_msgs::Pose& q2) {
  return angularDistanceYaw(q1.orientation, q2.orientation);
}

inline double angularDistanceYaw(
    const geometry_msgs::PoseStamped& q1,
    const geometry_msgs::PoseStamped& q2) {
  return angularDistance(q1.pose, q2.pose);
}

// Linear interpolation
inline geometry_msgs::Point linearInterpolation2D(
    const geometry_msgs::Point& p1, const geometry_msgs::Point& p2,
    double alpha) {
  const double dx = p2.x - p1.x, dy = p2.y - p1.y;
  geometry_msgs::Point out;
  out.x = p1.x + alpha * dx;
  out.y = p1.y + alpha * dy;
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
inline geometry_msgs::Quaternion slerp2D(
    const geometry_msgs::Quaternion& q1, const geometry_msgs::Quaternion& q2,
    double alpha) {
  const double da    = angles::normalize_angle(tf::getYaw(q2) - tf::getYaw(q1));
  const double slerp = angles::normalize_angle(a1 + alpha * da);
  return tf::createQuaternionMsgFromYaw(slerp);
}

inline geometry_msgs::Quaternion slerp2D(
    const geometry_msgs::Pose& q1, const geometry_msgs::Pose& q2,
    double alpha) {
  return slerp2D(q1.orientation, q2.orientation, alpha);
}

inline geometry_msgs::Quaternion slerp2D(
    const geometry_msgs::PoseStamped& q1, const geometry_msgs::PoseStamped& q2,
    double alpha) {
  return slerp2D(q1.pose, q2.pose, alpha);
}

}  // namespace math

}  // namespace squirrel_navigation

#endif /* SQUIRREL_NAVIGATION_MATH_UTILS_H_ */
