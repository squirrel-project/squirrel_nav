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

#include "squirrel_2d_localizer/math_types.h"

#include <geometry_msgs/Pose.h>

#include <tf/tf.h>

namespace squirrel_2d_localizer {

class Pose2d {
 public:
  Pose2d() : translation_(0., 0.), rotation_(0.) {}
  Pose2d(double x, double y, double a);
  Pose2d(const Vector<2>& translation, double a);
  Pose2d(const Pose2d& model) = default;
  virtual ~Pose2d() {}

  const Vector<2>& translation() const { return translation_; }
  Vector<2>& translation() { return translation_; }

  const Rotation2d& rotation() const { return rotation_; }
  Rotation2d& rotation() { return rotation_; }

  double operator[](const size_t i) const;
  double& operator[](const size_t i);
  
  Pose2d& operator=(const Pose2d& rhs);

  Pose2d operator*(const Pose2d& rhs) const;
  Pose2d& operator*=(const Pose2d& rhs);

  Pose2d inverse() const;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  
 private:
  Vector<2> translation_;
  Rotation2d rotation_;
};

typedef Pose2d Transform2d;

namespace ros_conversions {

template <typename TypeSE2>
TypeSE2 fromROSMsgTo(const geometry_msgs::Pose& p) {
  TypeSE2 output(p.position.x, p.position.y, tf::getYaw(p.orientation));
  return output;
}

template <typename TypeSE2>
TypeSE2 fromTFMsgTo(const tf::Transform& tf) {
  TypeSE2 output(
      tf.getOrigin().getX(), tf.getOrigin().getY(),
      tf::getYaw(tf.getRotation()));
  return output;
}

template <typename TypeSE2>
geometry_msgs::Pose toROSMsgFrom(const TypeSE2& p) {
  geometry_msgs::Pose output;
  output.position.x  = p[0];
  output.position.y  = p[1];
  output.orientation = tf::createQuaternionMsgFromYaw(p[2]);
  return output;
}

template <typename TypeSE2>
tf::Transform toTFMsgFrom(const TypeSE2& p) {
  tf::Vector3 translation(p[0], p[1], 0.);
  tf::Quaternion rotation;
  rotation.setRPY(0., 0., p[2]);
  return tf::Transform(rotation, translation);
}

}  // namespace ros_conversions

}  // namespace squirrel_2d_localizer

#endif /* SQUIRREL_2D_LOCALIZER_SE2_TYPES_H_ */
