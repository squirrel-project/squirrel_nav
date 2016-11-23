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

#ifndef SQUIRREL_2D_LOCALIZER_LIKELIHOOD_FIELD_H_
#define SQUIRREL_2D_LOCALIZER_LIKELIHOOD_FIELD_H_

#include "squirrel_2d_localizer/distance_transform.h"
#include "squirrel_2d_localizer/grid_map.h"
#include "squirrel_2d_localizer/se2_types.h"

#include <memory>

namespace squirrel_2d_localizer {

class LikelihoodField {
 public:
  typedef std::unique_ptr<LikelihoodField> Ptr;
  typedef std::unique_ptr<LikelihoodField const> ConstPtr;

  struct Params {
    double saturation_distance;
    double observation_sigma;
  };

 public:
  LikelihoodField() { setDefaultParams(); }
  LikelihoodField(const Params& lf_params) : lf_params_(lf_params) {}
  virtual ~LikelihoodField() {}

  void initialize(const GridMap& gridmap);

  double likelihood(int i, int j) const;

 private:
  inline bool inside(int i, int j) const {
    return i >= 0 && i < likelihood_cache_.rows() && j >= 0 &&
           j < likelihood_cache_.cols();
  }

  inline void setDefaultParams() {
    lf_params_.saturation_distance = 1.0;
    lf_params_.observation_sigma   = 0.5;
  }

  inline Params& params() { return lf_params_; }

 private:
  Matrix<> likelihood_cache_;

  double sq_saturation_distance_;

  Params lf_params_;
};

}  // namespace squirrel_2d_localizer

#endif /* SQUIRREL_2D_LOCALIZER_LIKELIHOOD_FIELD_H_ */
