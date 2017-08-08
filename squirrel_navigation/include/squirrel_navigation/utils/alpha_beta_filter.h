// Copyright (c) 2017, Federico Boniardi and Wolfram Burgard
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
// * Redistributions of source code must retain the above copyright
//   notice, this list of conditions and the following disclaimer.
//
// * Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in
//   the documentation and/or other materials provided with the
//   distribution.
//
// * Neither the name of the University of Freiburg nor the names of
//   its contributors may be used to endorse or promote products
//   derived from this software without specific prior written
//   permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
// INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
// HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
// STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
// OF THE POSSIBILITY OF SUCH DAMAGE.

#ifndef SQUIRREL_NAVIGATION_UTILS_ALPHA_BETA_FILTER_H_
#define SQUIRREL_NAVIGATION_UTILS_ALPHA_BETA_FILTER_H_

#include "squirrel_navigation/AlphaBetaFilterConfig.h"

#include <ros/time.h>

#include <dynamic_reconfigure/server.h>

#include <Eigen/Core>

#include <memory>
#include <utility>

namespace squirrel_navigation {
namespace utils {

class AlphaBetaFilter {
 public:
  class Params {
   public:
    static Params defaultParams();

    double alpha, beta;
  };

 public:
  AlphaBetaFilter();
  AlphaBetaFilter(const Params& params);
  virtual ~AlphaBetaFilter() {}

  // Initialization.
  void initialize(const std::string& name);

  // State dimension.
  int stateDimension() const { return dim_; }
  void setStateDimension(int n) { dim_ = n; }

  // Apply the filter.
  Eigen::MatrixXf operator()(
      const std::vector<float>& x, const ros::Time& stamp);

  // Parameter read/write utilities.
  inline const Params& params() const { return params_; }
  inline void setParams(const Params& params) { params_ = params; }
  inline Params& params() { return params_; }

 private:
  class State {
   public:
    State(const Eigen::VectorXf& x0, const Eigen::VectorXf& v0, double t0);
    State(const State& state) = default;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    Eigen::VectorXf x, v;
    double t;
  };

 private:
  void reconfigureCallback(AlphaBetaFilterConfig& config, uint32_t level);
  Eigen::VectorXf stdVectorToEigenVector(const std::vector<float>& x) const;

 private:
  Params params_;
  std::unique_ptr<dynamic_reconfigure::Server<AlphaBetaFilterConfig>> dsrv_;

  size_t dim_;
  std::unique_ptr<State> state_;

  bool init_;
};

}  // namespace utils
}  // namespace squirrel_navigation

#endif /* SQUIRREL_NAVIGATION_UTILS_ALPHA_BETA_FILTER_H_ */
