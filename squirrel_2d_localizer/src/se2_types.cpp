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

#include "squirrel_2d_localizer/se2_types.h"

#include <angles/angles.h>

namespace squirrel_2d_localizer {

Pose2d::Pose2d(double x, double y, double a)
    : translation_({x, y}), rotation_(a) {}

Pose2d::Pose2d(const Vector<2>& translation, double a)
    : translation_(translation), rotation_(a) {}

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
      rotation_ * rhs.translation() + translation_,
      rotation_.angle() + rhs[2]);
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

}  // namespace squirrel_2d_localizer
