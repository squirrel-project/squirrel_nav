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

#ifndef SQUIRREL_NAVIGATION_UTILS_ALPHA_BETA_FILTER_H_
#define SQUIRREL_NAVIGATION_UTILS_ALPHA_BETA_FILTER_H_

#include <ros/time.h>

#include <dynamic_reconfigure/server.h>

#include <squirrel_navigation/AlphaBetaFilterConfig.h>

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
