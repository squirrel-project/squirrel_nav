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

#include "squirrel_2d_localizer/se2_types.h"

#include <angles/angles.h>

namespace squirrel_2d_localizer {

Pose2d::Pose2d(double x, double y, double a)
    : translation_(x, y), rotation_(a) {}

Pose2d::Pose2d(const Eigen::Vector2d& translation, double a)
    : translation_(translation), rotation_(a) {}

Pose2d::Pose2d(const Eigen::Vector3d& pose_vec)
    : translation_(pose_vec(0), pose_vec(1)), rotation_(pose_vec(2)) {}

double Pose2d::operator[](size_t i) const {
  if (i == 0)
    return translation_[0];
  else if (i == 1)
    return translation_[1];
  else
    return rotation_.angle();
}

double& Pose2d::operator[](size_t i) {
  if (i == 0)
    return translation_[0];
  else if (i == 1)
    return translation_[1];
  else
    return rotation_.angle();
}

Pose2d& Pose2d::operator=(const Pose2d& rhs) {
  if (this != &rhs) {
    translation_[0]   = rhs[0];
    translation_[1]   = rhs[1];
    rotation_.angle() = rhs[2];
  }
  return *this;
}

Pose2d Pose2d::operator*(const Pose2d& rhs) const {
  Pose2d output(
      rotation_ * rhs.translation() + translation_, rotation_.angle() + rhs[2]);
  return output;
}

Pose2d& Pose2d::operator*=(const Pose2d& rhs) {
  translation_ = rotation_ * rhs.translation() + translation_;
  rotation_.angle() += rhs[2];
  return *this;
}

Pose2d Pose2d::inverse() const {
  Pose2d inv(rotation_.inverse() * (-translation_), -(*this)[2]);
  return inv;
}

Eigen::Vector3d Pose2d::toVector() const {
  return Eigen::Vector3d(
      translation_(0), translation_(1),
      angles::normalize_angle(rotation_.angle()));
}

void Pose2d::fromVector(const Eigen::Vector3d& xya) {
  translation_(0) = xya(0);
  translation_(1) = xya(1);
  rotation_       = Rotation2d(xya(2));
}

}  // namespace squirrel_2d_localizer
