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

#ifndef SQUIRREL_2D_LOCALIZER_LASER_MODEL_H_
#define SQUIRREL_2D_LOCALIZER_LASER_MODEL_H_

#include "squirrel_2d_localizer/endpoint_types.h"
#include "squirrel_2d_localizer/grid_map.h"
#include "squirrel_2d_localizer/latent_model_likelihood_field.h"
#include "squirrel_2d_localizer/particle_types.h"
#include "squirrel_2d_localizer/se2_types.h"

#include <Eigen/StdVector>

#include <cmath>
#include <mutex>
#include <vector>

namespace squirrel_2d_localizer {

class LaserModel {
 public:
  class Params {
   public:
    static Params defaultParams();

    double endpoints_min_distance;
    double range_min, range_max;
    double angle_min, angle_max;
    Pose2d tf_r2l;
  };

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

 public:
  LaserModel() : params_(Params::defaultParams()) {}
  LaserModel(const Params& params) : params_(params) {}
  virtual ~LaserModel() {}

  // Compute the particle likelihood.
  void computeParticlesLikelihood(
      const GridMap& grid_map,
      const LatentModelLikelihoodField& likelihood_field,
      const std::vector<float>& measurement, std::vector<Particle>* particles);

  // Paramters read/write utilities.
  inline const Params& params() const { return params_; }
  inline void setParams(const Params& params) { params_ = params; }
  inline Params& params() { return params_; }

 private:
  // Compute the effective reading used in the localizer.
  void prepareLaserReadings(const std::vector<float>& measurement);

 private:
  Params params_;

  std::vector<EndPoint2d, Eigen::aligned_allocator<EndPoint2d>>
      eff_measurement_;

  mutable std::mutex mtx_;
};

}  // namespace squirrel_2d_localizer

#endif /* SQUIRREL_2D_LOCALIZER_LASER_MODEL_H_ */
