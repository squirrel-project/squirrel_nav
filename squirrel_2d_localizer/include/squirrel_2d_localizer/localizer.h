// The MIT License (MIT)
//
// Copyright (c) 2017 Federico Boniardi and Wolfram Burgard
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
  class Params {
   public:
    static Params defaultParams();

    int num_particles;
    double min_lin_update, min_ang_update;
    double init_stddev_x, init_stddev_y, init_stddev_a;
  };

 public:
  Localizer() : params_(Params::defaultParams()) {}
  Localizer(const Params& params) : params_(params) {}
  virtual ~Localizer() {}

  // Initialize the localizer.
  void initialize(
      std::unique_ptr<GridMap>& map,
      std::unique_ptr<LatentModelLikelihoodField>& likelihood_field,
      std::unique_ptr<LaserModel>& laser_model,
      std::unique_ptr<MotionModel>& motion_model);

  // Update the localizer.
  void resetPose(const Pose2d& init_pose = Pose2d(0., 0., 0.));
  void resetParticles(const std::vector<Particle>& particles);
  bool updateFilter(
      const Transform2d& motion, const std::vector<float>& scan,
      const Transform2d& extra_correction = Pose2d(0., 0., 0.),
      bool force_update = false);

  // Get Particle filter's stuff.
  const std::vector<Particle>& particles() const { return particles_; }
  const Pose2d& pose() const { return pose_; }
  const Eigen::Matrix3d& covariance() const { return covariance_; }

  // Get the update guard.
  std::mutex& mutex() const { return init_mtx_; }

  // Get the localizer's objects.
  inline GridMap* gridMap() { return map_.get(); }
  inline LaserModel* laserModel() { return laser_model_.get(); }
  inline MotionModel* motionModel() { return motion_model_.get(); }

  // Parameters read/write utilities.
  inline const Params& params() const { params_; }
  inline void setParams(const Params& params) { params_ = params; }
  inline Params& params() { return params_; }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

 private:
  Params params_;

  std::unique_ptr<GridMap> map_;
  std::unique_ptr<LatentModelLikelihoodField> likelihood_field_;
  std::unique_ptr<LaserModel> laser_model_;
  std::unique_ptr<MotionModel> motion_model_;

  std::vector<Particle> particles_;
  Pose2d pose_;
  Eigen::Matrix3d covariance_;

  double cum_lin_motion_, cum_ang_motion_;

  mutable std::mutex init_mtx_;
};

}  // namespace squirrel_2d_localizer

#endif /* SQUIRREL_2D_LOCALIZER_LOCALIZER_H_ */
