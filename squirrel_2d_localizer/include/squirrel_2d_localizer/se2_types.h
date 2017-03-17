// Copyright (c) 2016-2017, Federico Boniardi and Wolfram Burgard
// All rights reserved.
// 
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
// 
// * Redistributions of source code must retain the above copyright notice, this
//   list of conditions and the following disclaimer.
// 
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
// 
// * Neither the name of the University of Freiburg nor the names of its
//   contributors may be used to endorse or promote products derived from
//   this software without specific prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

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
