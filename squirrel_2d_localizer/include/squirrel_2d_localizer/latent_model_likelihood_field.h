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

#ifndef SQUIRREL_2D_LOCALIZER_LATENT_MODEL_LIKELIHOOD_FIELD_H_
#define SQUIRREL_2D_LOCALIZER_LATENT_MODEL_LIKELIHOOD_FIELD_H_

#include "squirrel_2d_localizer/grid_map.h"
#include "squirrel_2d_localizer/se2_types.h"

namespace squirrel_2d_localizer {

class LatentModelLikelihoodField {
 public:
  class Params {
   public:
    static Params defaultParams();
    
    double uniform_hit;
    double observation_sigma;
  };

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  
 public:
  LatentModelLikelihoodField() : params_(Params::defaultParams()) {}
  LatentModelLikelihoodField(const Params& params) : params_(params) {};
  virtual ~LatentModelLikelihoodField() {}

  // Initialize the likelihood fields.
  void initialize(const GridMap& occupancy_gridmap);

  // Compute the likelihood value.
  double likelihood(int i, int j) const;

  // Paramters read/write utilites.
  inline const Params& params() const { return params_; }
  inline void setParams(const Params& params) { params_ = params; }
  inline Params& params() { return params_; }
  
 private:
  bool inside(int i, int j) const;

 private:
  Params params_;

  Eigen::MatrixXd likelihood_cache_;
  int likelihood_cache_rows_, likelihood_cache_cols_;
};

}  // namespace squirrel_2d_localizer

#endif /* SQUIRREL_2D_LOCALIZER_LATENT_MODEL_LIKELIHOOD_FIELD_H_ */
