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

#ifndef SQUIRREL_2D_LOCALIZER_LOCALIZER_H_
#define SQUIRREL_2D_LOCALIZER_LOCALIZER_H_

#include "squirrel_2d_localizer/grid_map.h"
#include "squirrel_2d_localizer/laser_model.h"
#include "squirrel_2d_localizer/latent_model_likelihood_field.h"
#include "squirrel_2d_localizer/motion_model.h"

#include <memory>

namespace squirrel_2d_localizer {

class Localizer {
 public:
  typedef std::unique_ptr<Localizer> Ptr;
  typedef std::unique_ptr<Localizer const> ConstPtr;

  struct Params {
    int num_particles;
    double min_lin_update, min_ang_update;
    double init_stddev_x, init_stddev_y, init_stddev_a;
  };

 public:
  Localizer() { setDefaultParams(); }
  Localizer(const Params& loc_params) : loc_params_(loc_params) {}
  virtual ~Localizer() {}

  void initialize(
      GridMap::Ptr& map, LatentModelLikelihoodField::Ptr& likelihood_field,
      LaserModel::Ptr& laser_model, MotionModel::Ptr& motion_model);

  void resetPose(const Pose2d& init_pose = Pose2d(0., 0., 0.));
  bool updateFilter(
      const Transform2d& motion, const std::vector<float>& scan,
      const Transform2d& extra_correction = Pose2d(0., 0., 0.));

  const std::vector<Particle>& particles() const { return particles_; }
  const Pose2d& pose() const { return pose_; }
  const Matrix<3, 3>& covariance() const { return covariance_; }

  inline Params& params() { return loc_params_; }

  inline GridMap* gridMap() { return map_.get(); }
  inline LaserModel* laserModel() { return laser_model_.get(); }
  inline MotionModel* motionModel() { return motion_model_.get(); }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

 public:
  mutable std::mutex init_mtx;

 private:
  inline void setDefaultParams() {
    loc_params_.num_particles  = 250;
    loc_params_.min_lin_update = 1.0;
    loc_params_.min_ang_update = 1.0;
  }

 private:
  GridMap::Ptr map_;
  LatentModelLikelihoodField::Ptr likelihood_field_;
  LaserModel::Ptr laser_model_;
  MotionModel::Ptr motion_model_;

  std::vector<Particle> particles_;
  Pose2d pose_;
  Matrix<3, 3> covariance_;

  Params loc_params_;

  double cum_lin_motion_, cum_ang_motion_;
};

}  // namespace squirrel_2d_localizer

#endif /* SQUIRREL_2D_LOCALIZER_LOCALIZER_H_ */
