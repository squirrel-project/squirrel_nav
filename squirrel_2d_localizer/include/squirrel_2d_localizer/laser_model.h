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

#ifndef SQUIRREL_2D_LOCALIZER_LASER_MODEL_H_
#define SQUIRREL_2D_LOCALIZER_LASER_MODEL_H_

#include "squirrel_2d_localizer/endpoint_types.h"
#include "squirrel_2d_localizer/grid_map.h"
#include "squirrel_2d_localizer/likelihood_field.h"
#include "squirrel_2d_localizer/particle_types.h"
#include "squirrel_2d_localizer/se2_types.h"

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

 public:
  LaserModel() { setDefaultParams(); }
  LaserModel(const Params& laser_params) : laser_params_(laser_params) {}
  virtual ~LaserModel() {}

  void computeParticlesLikelihood(
      const GridMap& grid_map, const LikelihoodField& likelihood_field,
      const std::vector<float>& measurement, std::vector<Particle>* particles);

  inline Params& params() { return laser_params_; }

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
  std::vector<EndPoint2d> eff_measurement_;

  Params laser_params_;

  std::mutex mtx_;
};

}  // namespace squirrel_2d_localizer

#endif /* SQUIRREL_2D_LOCALIZER_LASER_MODEL_H_ */
