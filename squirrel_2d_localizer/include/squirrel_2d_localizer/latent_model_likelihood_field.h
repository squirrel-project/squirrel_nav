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

#ifndef SQUIRREL_2D_LOCALIZER_LATENT_MODEL_LIKELIHOOD_FIELD_H_
#define SQUIRREL_2D_LOCALIZER_LATENT_MODEL_LIKELIHOOD_FIELD_H_

#include "squirrel_2d_localizer/grid_map.h"
#include "squirrel_2d_localizer/se2_types.h"

#include <memory>

namespace squirrel_2d_localizer {

class LatentModelLikelihoodField {
 public:
  typedef std::unique_ptr<LatentModelLikelihoodField> Ptr;
  typedef std::unique_ptr<LatentModelLikelihoodField const> ConstPtr;

  struct Params {
    double uniform_hit;
    double observation_sigma;
  };

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  
 public:
  LatentModelLikelihoodField();
  LatentModelLikelihoodField(const Params& params) : lmlf_params_(params) {};
  virtual ~LatentModelLikelihoodField() {}

  void initialize(const GridMap& occupancy_gridmap);

  double likelihood(int i, int j) const;

  inline Params& params() { return lmlf_params_; }
  inline const Params& parmas() const { return lmlf_params_; }
  
 private:
  inline bool inside(int i, int j) const {
    return i >= 0 && i < likelihood_cache_.rows() && j >= 0 &&
           j < likelihood_cache_.cols();
  }

  inline void setDefaultParams() {
    lmlf_params_.uniform_hit       = 0.1;
    lmlf_params_.observation_sigma = 1.0;
  }

 private:
  Matrix<> likelihood_cache_;
  int likelihood_cache_rows_, likelihood_cache_cols_;

  Params lmlf_params_;
};

}  // namespace squirrel_2d_localizer

#endif /* SQUIRREL_2D_LOCALIZER_LATENT_MODEL_LIKELIHOOD_FIELD_H_ */
