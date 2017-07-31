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

}  // namespace math
}  // namespace squirrel_navigation

#endif /* SQUIRREL_NAVIGATION_UTILS_MATH_UTILS_H_ */
