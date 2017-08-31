/**
 * @file convolutional_network_test.cpp
 * @author Marcus Edel
 *
 * Tests the convolutional neural network.
 *
 * mlpack is free software; you may redistribute it and/or modify it under the
 * terms of the 3-clause BSD license.  You should have received a copy of the
 * 3-clause BSD license along with mlpack.  If not, see
 * http://www.opensource.org/licenses/BSD-3-Clause for more information.
 */
#include <mlpack/core.hpp>

#include <mlpack/core/optimizers/rmsprop/rmsprop.hpp>
#include <mlpack/methods/ann/layer/layer.hpp>
#include <mlpack/methods/ann/ffn.hpp>

#include <boost/test/unit_test.hpp>
#include "test_tools.hpp"

using namespace mlpack;
using namespace mlpack::ann;
using namespace mlpack::optimization;


BOOST_AUTO_TEST_SUITE(ConvolutionalNetworkTest);

/**
 * Train the vanilla network on a larger dataset.
 */
BOOST_AUTO_TEST_CASE(VanillaNetworkTest)
{
  arma::mat X;
  X.load("mnist_first250_training_4s_and_9s.arm");

  // Normalize each point since these are images.
  arma::uword nPoints = X.n_cols;
  for (arma::uword i = 0; i < nPoints; i++)
  {
    X.col(i) /= norm(X.col(i), 2);
  }

  // Build the target matrix.
  arma::mat Y = arma::zeros<arma::mat>(1, nPoints);
  for (size_t i = 0; i < nPoints; i++)
  {
    if (i < nPoints / 2)
    {
      Y(i) = 4;
    }
    else
    {
      Y(i) = 9;
    }
  }

  /*
   * Construct a convolutional neural network with a 28x28x1 input layer,
   * 24x24x8 convolution layer, 12x12x8 pooling layer, 8x8x12 convolution layer
   * and a 4x4x12 pooling layer which is fully connected with the output layer.
   * The network structure looks like:
   *
   * Input    Convolution  Pooling      Convolution  Pooling      Output
   * Layer    Layer        Layer        Layer        Layer        Layer
   *
   *          +---+        +---+        +---+        +---+
   *          | +---+      | +---+      | +---+      | +---+
   * +---+    | | +---+    | | +---+    | | +---+    | | +---+    +---+
   * |   |    | | |   |    | | |   |    | | |   |    | | |   |    |   |
   * |   +--> +-+ |   +--> +-+ |   +--> +-+ |   +--> +-+ |   +--> |   |
   * |   |      +-+   |      +-+   |      +-+   |      +-+   |    |   |
   * +---+        +---+        +---+        +---+        +---+    +---+
   */
  FFN<NegativeLogLikelihood<> > model;

  model.Add<Convolution<> >(1, 8, 5, 5, 1, 1, 0, 0, 28, 28);
  model.Add<ReLULayer<> >();
  model.Add<MaxPooling<> >(8, 8, 2, 2);
  model.Add<Convolution<> >(8, 12, 2, 2);
  model.Add<ReLULayer<> >();
  model.Add<MaxPooling<> >(2, 2, 2, 2);
  model.Add<Linear<> >(192, 20);
  model.Add<ReLULayer<> >();
  model.Add<Linear<> >(20, 30);
  model.Add<ReLULayer<> >();
  model.Add<Linear<> >(30, 10);
  model.Add<LogSoftMax<> >();

  RMSprop<decltype(model)> opt(model, 0.01, 0.88, 1e-8, 5000, -1);

  model.Train(std::move(X), std::move(Y), opt);

  arma::mat predictionTemp;
  model.Predict(X, predictionTemp);
  arma::mat prediction = arma::zeros<arma::mat>(1, predictionTemp.n_cols);

  for (size_t i = 0; i < predictionTemp.n_cols; ++i)
  {
    prediction(i) = arma::as_scalar(arma::find(
        arma::max(predictionTemp.col(i)) == predictionTemp.col(i), 1)) + 1;
  }

  size_t error = 0;
  for (size_t i = 0; i < X.n_cols; i++)
  {
    if (prediction(i) == Y(i))
    {
      error++;
    }
  }

  double classificationError = 1 - double(error) / X.n_cols;
  BOOST_REQUIRE_LE(classificationError, 0.2);
}

BOOST_AUTO_TEST_SUITE_END();
