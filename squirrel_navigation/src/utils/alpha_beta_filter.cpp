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

#include "squirrel_navigation/utils/alpha_beta_filter.h"

#include <cassert>
#include <cmath>

namespace squirrel_navigation {
namespace utils {

AlphaBetaFilter::AlphaBetaFilter()
    : params_(Params::defaultParams()), state_(nullptr), init_(false) {}

AlphaBetaFilter::AlphaBetaFilter(const Params& params)
    : params_(params), state_(nullptr), init_(false) {}

void AlphaBetaFilter::initialize(const std::string& name) {
  if (init_)
    return;
  ros::NodeHandle pnh("~/" + name);
  dsrv_.reset(new dynamic_reconfigure::Server<AlphaBetaFilterConfig>(pnh));
  dsrv_->setCallback(
      boost::bind(&AlphaBetaFilter::reconfigureCallback, this, _1, _2));
  init_ = true;
}

Eigen::MatrixXf AlphaBetaFilter::operator()(
    const std::vector<float>& x, const ros::Time& stamp) {
  assert(dim_ == x.size());
  if (!state_) {
    state_.reset(new State(
        stdVectorToEigenVector(x), Eigen::VectorXf::Zero(dim_), stamp.toSec()));
  } else {
    const double dt          = stamp.toSec() - state_->t;
    const Eigen::VectorXf xp = state_->x + dt * state_->v;
    const Eigen::VectorXf rc = stdVectorToEigenVector(x) - xp;
    // Update the state.
    state_->x = xp + params_.alpha * rc;
    state_->v = state_->v + (params_.beta / dt) * rc;
    state_->t = stamp.toSec();
  }
  Eigen::MatrixXf output(dim_, 2);
  output.col(0) = state_->x;
  output.col(1) = state_->v;
  return output;
}

void AlphaBetaFilter::reconfigureCallback(
    AlphaBetaFilterConfig& config, uint32_t level) {
  params_.alpha = config.alpha;
  params_.beta  = config.beta;
}

Eigen::VectorXf AlphaBetaFilter::stdVectorToEigenVector(
    const std::vector<float>& x) const {
  const int n = x.size();
  Eigen::VectorXf out(n);
  for (int i = 0; i < n; ++i)
    out(i)   = std::isfinite(x[i]) ? x[i] : 0.0;
  return out;
}

AlphaBetaFilter::Params AlphaBetaFilter::Params::defaultParams() {
  Params params;
  params.alpha = 0.1;
  params.beta  = 2 * (2 - params.alpha) - 4 * std::sqrt(1. - params.alpha);
  return params;
}

AlphaBetaFilter::State::State(
    const Eigen::VectorXf& x0, const Eigen::VectorXf& v0, double t0)
    : x(x0), v(v0), t(t0) {}

}  // namespace utils
}  // namespace squirrel_navigation
