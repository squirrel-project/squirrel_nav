// The MIT License (MIT)
//
// Copyright (c) 2016 Federico Boniardi
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

#ifndef SQUIRREL_2D_LOCALIZER_SE2_TYPES_H_
#define SQUIRREL_2D_LOCALIZER_SE2_TYPES_H_

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <g2o/core/eigen_types.h>
#include <g2o/types/slam2d/se2.h>

#include <geometry_msgs/Pose.h>

#include <tf/tf.h>

namespace squirrel_2d_localizer {

typedef g2o::SE2 Pose2d;
typedef g2o::SE2 Transform2d;

namespace ros_conversions {

inline Pose2d fromROSMsg(const geometry_msgs::Pose& p) {
  Pose2d output(p.position.x, p.position.y, tf::getYaw(p.orientation));
  return output;
}

inline Pose2d fromTFMsg(const tf::Transform& tf) {
  Pose2d output(
      tf.getOrigin().getX(), tf.getOrigin().getY(),
      tf::getYaw(tf.getRotation()));
  return output;
}

inline geometry_msgs::Pose toROSMsg(const Pose2d& p) {
  geometry_msgs::Pose output;
  output.position.x  = p[0];
  output.position.y  = p[1];
  output.orientation = tf::createQuaternionMsgFromYaw(p[2]);
  return output;
}

inline tf::Transform toTFMsg(const Pose2d& p) {
  tf::Vector3 translation(p[0], p[1], 0.);
  tf::Quaternion rotation;
  rotation.setRPY(0., 0., p[2]);
  return tf::Transform(rotation, translation);
}

}  // namespace ros_conversions

}  // namespace squirrel_2d_localizer

#endif /* SQUIRREL_2D_LOCALIZER_SE2_TYPES_H_ */
