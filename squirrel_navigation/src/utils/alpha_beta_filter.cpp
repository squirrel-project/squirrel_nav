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

#include "squirrel_navigation/utils/alpha_beta_filter.h"

#include <cassert>
#include <cmath>

namespace squirrel_navigation {

namespace utils {

void AlphaBetaFilter::initialize(const std::string& name) {
  ros::NodeHandle pnh("~/" + name);
  dsrv_.reset(new dynamic_reconfigure::Server<AlphaBetaFilterConfig>(pnh));
  dsrv_->setCallback(
      boost::bind(&AlphaBetaFilter::reconfigureCallback, this, _1, _2));
}

Eigen::Matrix<> AlphaBetaFilter::operator()(
    const std::vector<float>& x, const ros::Time& stamp) {
  assert(dim_ == x.size());
  if (!state_) {
    Eigen::VectorXf x0(dim_);
    for (int i = 0; i < (int)dim_; ++i)
      x0(i)    = x[i];
    state_.reset(new State(x, Eigen::VectorXd::Zero(dim_), stamp.toSec()));
  } else {
    const double dt          = stamp.toSec() - state_->t;
    const Eigen::VectorXf xp = state_->x + dt * state_->v;
    const Eigen::VectorXf rc = x - xp;
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

AlphaBetaFilter::Params AlphBetaFilter::Params::defaultParams() {
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
