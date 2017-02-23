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

#ifndef SQUIRREL_2D_LOCALIZER_LASER_MODEL_H_
#define SQUIRREL_2D_LOCALIZER_LASER_MODEL_H_

#include "squirrel_2d_localizer/endpoint_types.h"
#include "squirrel_2d_localizer/grid_map.h"
#include "squirrel_2d_localizer/latent_model_likelihood_field.h"
#include "squirrel_2d_localizer/particle_types.h"
#include "squirrel_2d_localizer/se2_types.h"

#include <Eigen/StdVector>

#include <cmath>
#include <memory>
#include <mutex>
#include <vector>

namespace squirrel_2d_localizer {

class LaserModel {
 public:
  typedef std::unique_ptr<LaserModel> Ptr;
  typedef std::unique_ptr<LaserModel const> ConstPtr;

  struct Params {
    double endpoints_min_distance;
    double range_min, range_max;
    double angle_min, angle_max;
    Pose2d tf_r2l;
  };

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

 public:
  LaserModel() { setDefaultParams(); }
  LaserModel(const Params& laser_params) : laser_params_(laser_params) {}
  virtual ~LaserModel() {}

  void computeParticlesLikelihood(
      const GridMap& grid_map,
      const LatentModelLikelihoodField& likelihood_field,
      const std::vector<float>& measurement, std::vector<Particle>* particles);

  inline Params& params() { return laser_params_; }
  inline const Params& params() const { return laser_params_; }

 private:
  inline void setDefaultParams() {
    laser_params_.endpoints_min_distance = 0.5;
    laser_params_.range_min              = 0.;
    laser_params_.range_max              = 6.;
    laser_params_.angle_min              = -0.5 * M_PI;
    laser_params_.angle_max              = 0.5 * M_PI;
  }

  void prepareLaserReadings(const std::vector<float>& measurement);

 private:
  std::vector<EndPoint2d, Eigen::aligned_allocator<EndPoint2d>>
      eff_measurement_;

  Params laser_params_;

  std::mutex mtx_;
};

}  // namespace squirrel_2d_localizer

#endif /* SQUIRREL_2D_LOCALIZER_LASER_MODEL_H_ */
