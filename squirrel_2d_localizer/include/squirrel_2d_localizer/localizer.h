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

 private:
  inline void setDefaultParams() {
    loc_params_.num_particles  = 1000;
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

  std::mutex init_mtx_;
};

}  // namespace squirrel_2d_localizer

#endif /* SQUIRREL_2D_LOCALIZER_LOCALIZER_H_ */
