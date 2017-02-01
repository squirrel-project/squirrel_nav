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
