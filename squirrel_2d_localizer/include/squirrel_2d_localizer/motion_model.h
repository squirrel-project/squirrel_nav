// The MIT License (MIT)
//
// Copyright (c) 2016-2017 Federico Boniardi and Wolfram Burgard
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
  class Params {
   public:
    static Params defaultParams();

    double noise_xx, noise_xy, noise_xa, noise_yy, noise_ya, noise_aa;
    double noise_magnitude;
  };

 public:
  MotionModel() : params_(Params::defaultParams()), rnd_eng_(std::rand()) {}
  MotionModel(const Params& params) : params_(params), rnd_eng_(std::rand()) {}
  virtual ~MotionModel() {}

  // Sample from the proposal distribution.
  void sampleProposal(
      const Transform2d& motion, std::vector<Particle>* particles) const;

  inline const Params& params() const { return params_; }
  inline void setParams(const Params& params) { params_ = params; }
  inline Params& params() { return params_; }

 protected:
  Params params_;

 private:
  mutable std::mt19937 rnd_eng_;
  mutable std::mutex mtx_;
};

}  // namespace squirrel_2d_localizer

#endif /* SQUIRREL_2D_LOCALIZER_MOTION_MODEL_H_ */
