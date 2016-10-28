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

#ifndef SQUIRREL_2D_LOCALIZER_MOTION_MODEL_H_
#define SQUIRREL_2D_LOCALIZER_MOTION_MODEL_H_

#include "squirrel_2d_localizer/math_types.h"
#include "squirrel_2d_localizer/particle_types.h"
#include "squirrel_2d_localizer/se2_types.h"

#include <memory>
#include <mutex>
#include <random>

namespace squirrel_2d_localizer {

class MotionModel {
 public:
  typedef std::unique_ptr<MotionModel> Ptr;
  typedef std::unique_ptr<MotionModel const> ConstPtr;

  struct Params {
    double noise_xx, noise_xy, noise_xa, noise_yy, noise_ya, noise_aa;
    double noise_magnitude;
  };

 public:
  MotionModel() : rnd_eng_(std::rand()) { setDefaultParams(); }
  MotionModel(const Params& motion_params);
  virtual ~MotionModel() {}

  void propagateParticles(
      const Transform2d& motion, std::vector<Particle>* particles) const;

  inline Params& params() { return motion_params_; }

 private:
  inline void setDefaultParams() {
    motion_params_.noise_xx        = 1.;
    motion_params_.noise_xy        = 0.;
    motion_params_.noise_xa        = 0.;
    motion_params_.noise_yy        = 1.;
    motion_params_.noise_ya        = 0.;
    motion_params_.noise_aa        = 1.;
    motion_params_.noise_magnitude = 0.5;
  }

 private:
  Params motion_params_;

  mutable std::mt19937 rnd_eng_;
  mutable std::mutex mtx_;
};

}  // namespace squirrel_2d_localizer

#endif /* SQUIRREL_2D_LOCALIZER_MOTION_MODEL_H_ */
