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

  void sampleProposal(
      const Transform2d& motion, std::vector<Particle>* particles) const;

  inline const Params& params() const { return motion_params_; }
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
