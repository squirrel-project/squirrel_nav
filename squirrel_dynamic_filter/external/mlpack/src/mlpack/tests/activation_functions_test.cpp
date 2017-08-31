/**
 * @file activation_functions_test.cpp
 * @author Marcus Edel
 * @author Dhawal Arora
 *
 * Tests for the various activation functions.
 *
 * mlpack is free software; you may redistribute it and/or modify it under the
 * terms of the 3-clause BSD license.  You should have received a copy of the
 * 3-clause BSD license along with mlpack.  If not, see
 * http://www.opensource.org/licenses/BSD-3-Clause for more information.
 */
#include <mlpack/core.hpp>

#include <mlpack/methods/ann/layer/layer.hpp>
#include <mlpack/methods/ann/activation_functions/logistic_function.hpp>
#include <mlpack/methods/ann/activation_functions/identity_function.hpp>
#include <mlpack/methods/ann/activation_functions/softsign_function.hpp>
#include <mlpack/methods/ann/activation_functions/tanh_function.hpp>
#include <mlpack/methods/ann/activation_functions/rectifier_function.hpp>
#include <mlpack/methods/ann/activation_functions/softplus_function.hpp>

#include <boost/test/unit_test.hpp>
#include "test_tools.hpp"

using namespace mlpack;
using namespace mlpack::ann;

BOOST_AUTO_TEST_SUITE(ActivationFunctionsTest);

// Generate dataset for activation function tests.
const arma::colvec activationData("-2 3.2 4.5 -100.2 1 -1 2 0");

/*
 * Implementation of the activation function test.
 *
 * @param input Input data used for evaluating the activation function.
 * @param target Target data used to evaluate the activation.
 *
 * @tparam ActivationFunction Activation function used for the check.
 */
template<class ActivationFunction>
void CheckActivationCorrect(const arma::colvec input, const arma::colvec target)
{
  // Test the activation function using a single value as input.
  for (size_t i = 0; i < target.n_elem; i++)
  {
    BOOST_REQUIRE_CLOSE(ActivationFunction::fn(input.at(i)),
        target.at(i), 1e-3);
  }

  // Test the activation function using the entire vector as input.
  arma::colvec activations;
  ActivationFunction::fn(input, activations);
  for (size_t i = 0; i < activations.n_elem; i++)
  {
    BOOST_REQUIRE_CLOSE(activations.at(i), target.at(i), 1e-3);
  }
}

/*
 * Implementation of the activation function derivative test.
 *
 * @param input Input data used for evaluating the activation function.
 * @param target Target data used to evaluate the activation.
 *
 * @tparam ActivationFunction Activation function used for the check.
 */
template<class ActivationFunction>
void CheckDerivativeCorrect(const arma::colvec input, const arma::colvec target)
{
  // Test the calculation of the derivatives using a single value as input.
  for (size_t i = 0; i < target.n_elem; i++)
  {
    BOOST_REQUIRE_CLOSE(ActivationFunction::deriv(input.at(i)),
        target.at(i), 1e-3);
  }

  // Test the calculation of the derivatives using the entire vector as input.
  arma::colvec derivatives;
  ActivationFunction::deriv(input, derivatives);
  for (size_t i = 0; i < derivatives.n_elem; i++)
  {
    BOOST_REQUIRE_CLOSE(derivatives.at(i), target.at(i), 1e-3);
  }
}

/*
 * Implementation of the activation function inverse test.
 *
 * @param input Input data used for evaluating the activation function.
 * @param target Target data used to evaluate the activation.
 *
 * @tparam ActivationFunction Activation function used for the check.
 */
template<class ActivationFunction>
void CheckInverseCorrect(const arma::colvec input)
{
    // Test the calculation of the inverse using a single value as input.
  for (size_t i = 0; i < input.n_elem; i++)
  {
    BOOST_REQUIRE_CLOSE(ActivationFunction::inv(ActivationFunction::fn(
        input.at(i))), input.at(i), 1e-3);
  }

  // Test the calculation of the inverse using the entire vector as input.
  arma::colvec activations;
  ActivationFunction::fn(input, activations);
  ActivationFunction::inv(activations, activations);

  for (size_t i = 0; i < input.n_elem; i++)
  {
    BOOST_REQUIRE_CLOSE(activations.at(i), input.at(i), 1e-3);
  }
}

/*
 * Implementation of the HardTanH activation function test. The function is
 * implemented as a HardTanH Layer in hard_tanh.hpp
 *
 * @param input Input data used for evaluating the HardTanH activation function.
 * @param target Target data used to evaluate the HardTanH activation.
 */
void CheckHardTanHActivationCorrect(const arma::colvec input,
                                    const arma::colvec target)
{
  HardTanH<> htf;

  // Test the activation function using the entire vector as input.
  arma::colvec activations;
  htf.Forward(std::move(input), std::move(activations));
  for (size_t i = 0; i < activations.n_elem; i++)
  {
    BOOST_REQUIRE_CLOSE(activations.at(i), target.at(i), 1e-3);
  }
}

/*
 * Implementation of the HardTanH activation function derivative test. The
 * derivative is implemented as HardTanH Layer in hard_tanh.hpp
 *
 * @param input Input data used for evaluating the HardTanH activation function.
 * @param target Target data used to evaluate the HardTanH activation.
 */
void CheckHardTanHDerivativeCorrect(const arma::colvec input,
                                    const arma::colvec target)
{
  HardTanH<> htf;

  // Test the calculation of the derivatives using the entire vector as input.
  arma::colvec derivatives;

  // This error vector will be set to 1 to get the derivatives.
  arma::colvec error = arma::ones<arma::colvec>(input.n_elem);
  htf.Backward(std::move(input), std::move(error), std::move(derivatives));

  for (size_t i = 0; i < derivatives.n_elem; i++)
  {
    BOOST_REQUIRE_CLOSE(derivatives.at(i), target.at(i), 1e-3);
  }
}

/*
 * Implementation of the LeakyReLU activation function test. The function is
 * implemented as LeakyReLU layer in the file leaky_relu.hpp
 *
 * @param input Input data used for evaluating the LeakyReLU activation function.
 * @param target Target data used to evaluate the LeakyReLU activation.
 */
void CheckLeakyReLUActivationCorrect(const arma::colvec input,
                                     const arma::colvec target)
{
  LeakyReLU<> lrf;

  // Test the activation function using the entire vector as input.
  arma::colvec activations;
  lrf.Forward(std::move(input), std::move(activations));
  for (size_t i = 0; i < activations.n_elem; i++)
  {
    BOOST_REQUIRE_CLOSE(activations.at(i), target.at(i), 1e-3);
  }
}

/*
 * Implementation of the LeakyReLU activation function derivative test.
 * The derivative function is implemented as LeakyReLU layer in the file
 * leaky_relu_layer.hpp
 *
 * @param input Input data used for evaluating the LeakyReLU activation function.
 * @param target Target data used to evaluate the LeakyReLU activation.
 */
void CheckLeakyReLUDerivativeCorrect(const arma::colvec input,
                                     const arma::colvec target)
{
  LeakyReLU<> lrf;

  // Test the calculation of the derivatives using the entire vector as input.
  arma::colvec derivatives;

  // This error vector will be set to 1 to get the derivatives.
  arma::colvec error = arma::ones<arma::colvec>(input.n_elem);
  lrf.Backward(std::move(input), std::move(error), std::move(derivatives));
  for (size_t i = 0; i < derivatives.n_elem; i++)
  {
    BOOST_REQUIRE_CLOSE(derivatives.at(i), target.at(i), 1e-3);
  }
}

/*
 * Implementation of the ELU activation function test. The function is
 * implemented as ELU layer in the file elu.hpp
 *
 * @param input Input data used for evaluating the ELU activation function.
 * @param target Target data used to evaluate the ELU activation.
 */
void CheckELUActivationCorrect(const arma::colvec input,
                                     const arma::colvec target)
{
  ELU<> lrf;

  // Test the activation function using the entire vector as input.
  arma::colvec activations;
  lrf.Forward(std::move(input), std::move(activations));
  for (size_t i = 0; i < activations.n_elem; i++)
  {
    BOOST_REQUIRE_CLOSE(activations.at(i), target.at(i), 1e-3);
  }
}

/*
 * Implementation of the ELU activation function derivative test. The function
 * is implemented as ELU layer in the file elu.hpp
 *
 * @param input Input data used for evaluating the ELU activation function.
 * @param target Target data used to evaluate the ELU activation.
 */
void CheckELUDerivativeCorrect(const arma::colvec input,
                                     const arma::colvec target)
{
  ELU<> lrf;

  // Test the calculation of the derivatives using the entire vector as input.
  arma::colvec derivatives;

  // This error vector will be set to 1 to get the derivatives.
  arma::colvec error = arma::ones<arma::colvec>(input.n_elem);
  lrf.Backward(std::move(input), std::move(error), std::move(derivatives));
  for (size_t i = 0; i < derivatives.n_elem; i++)
  {
    BOOST_REQUIRE_CLOSE(derivatives.at(i), target.at(i), 1e-3);
  }
}

/**
 * Basic test of the tanh function.
 */
BOOST_AUTO_TEST_CASE(TanhFunctionTest)
{
  const arma::colvec desiredActivations("-0.96402758 0.9966824 0.99975321 -1 \
                                         0.76159416 -0.76159416 0.96402758 0");

  const arma::colvec desiredDerivatives("0.07065082 0.00662419 0.00049352 0 \
                                         0.41997434 0.41997434 0.07065082 1");

  CheckActivationCorrect<TanhFunction>(activationData, desiredActivations);
  CheckDerivativeCorrect<TanhFunction>(desiredActivations, desiredDerivatives);
  CheckInverseCorrect<TanhFunction>(desiredActivations);
}

/**
 * Basic test of the logistic function.
 */
BOOST_AUTO_TEST_CASE(LogisticFunctionTest)
{
  const arma::colvec desiredActivations("1.19202922e-01 9.60834277e-01 \
                                         9.89013057e-01 3.04574e-44 \
                                         7.31058579e-01 2.68941421e-01 \
                                         8.80797078e-01 0.5");

  const arma::colvec desiredDerivatives("0.10499359 0.03763177 0.01086623 \
                                         3.04574e-44 0.19661193 0.19661193 \
                                         0.10499359 0.25");

  CheckActivationCorrect<LogisticFunction>(activationData, desiredActivations);
  CheckDerivativeCorrect<LogisticFunction>(desiredActivations,
      desiredDerivatives);
  CheckInverseCorrect<LogisticFunction>(activationData);
}

/**
 * Basic test of the softsign function.
 */
BOOST_AUTO_TEST_CASE(SoftsignFunctionTest)
{
  const arma::colvec desiredActivations("-0.66666667 0.76190476 0.81818182 \
                                         -0.99011858 0.5 -0.5 0.66666667 0");

  const arma::colvec desiredDerivatives("0.11111111 0.05668934 0.03305785 \
                                         9.7642e-05 0.25 0.25 0.11111111 1");

  CheckActivationCorrect<SoftsignFunction>(activationData, desiredActivations);
  CheckDerivativeCorrect<SoftsignFunction>(desiredActivations,
      desiredDerivatives);
  CheckInverseCorrect<SoftsignFunction>(desiredActivations);
}

/**
 * Basic test of the identity function.
 */
BOOST_AUTO_TEST_CASE(IdentityFunctionTest)
{
  const arma::colvec desiredDerivatives = arma::ones<arma::colvec>(
      activationData.n_elem);

  CheckActivationCorrect<IdentityFunction>(activationData, activationData);
  CheckDerivativeCorrect<IdentityFunction>(activationData, desiredDerivatives);
}

/**
 * Basic test of the rectifier function.
 */
BOOST_AUTO_TEST_CASE(RectifierFunctionTest)
{
  const arma::colvec desiredActivations("0 3.2 4.5 0 1 0 2 0");

  const arma::colvec desiredDerivatives("0 1 1 0 1 0 1 0");

  CheckActivationCorrect<RectifierFunction>(activationData, desiredActivations);
  CheckDerivativeCorrect<RectifierFunction>(desiredActivations,
      desiredDerivatives);
}

/**
 * Basic test of the LeakyReLU function.
 */
BOOST_AUTO_TEST_CASE(LeakyReLUFunctionTest)
{
  const arma::colvec desiredActivations("-0.06 3.2 4.5 -3.006 \
                                         1 -0.03 2 0");

  const arma::colvec desiredDerivatives("0.03 1 1 0.03 \
                                         1 0.03 1 1");

  CheckLeakyReLUActivationCorrect(activationData, desiredActivations);
  CheckLeakyReLUDerivativeCorrect(desiredActivations, desiredDerivatives);
}

/**
 * Basic test of the HardTanH function.
 */
BOOST_AUTO_TEST_CASE(HardTanHFunctionTest)
{
  const arma::colvec desiredActivations("-1 1 1 -1 \
                                         1 -1 1 0");

  const arma::colvec desiredDerivatives("0 0 0 0 \
                                         1 1 0 1");

  CheckHardTanHActivationCorrect(activationData, desiredActivations);
  CheckHardTanHDerivativeCorrect(activationData, desiredDerivatives);
}

/**
 * Basic test of the ELU function.
 */
BOOST_AUTO_TEST_CASE(ELUFunctionTest)
{
  const arma::colvec desiredActivations("-0.86466471 3.2 4.5 -1.0 \
                                         1 -0.63212055 2 0");

  const arma::colvec desiredDerivatives("0.13533529 1 1 0 \
                                         1 0.36787945 1 1");

  CheckELUActivationCorrect(activationData, desiredActivations);
  CheckELUDerivativeCorrect(desiredActivations, desiredDerivatives);
}

/**
 * Basic test of the softplus function.
 */
BOOST_AUTO_TEST_CASE(SoftplusFunctionTest)
{
  const arma::colvec desiredActivations("0.12692801 3.23995333 4.51104774 \
                                         0 1.31326168 0.31326168 2.12692801 \
                                         0.69314718");

  const arma::colvec desiredDerivatives("0.53168946 0.96231041 0.98913245 \
                                         0.5 0.78805844 0.57768119 0.89349302\
                                         0.66666666");

  CheckActivationCorrect<SoftplusFunction>(activationData, desiredActivations);
  CheckDerivativeCorrect<SoftplusFunction>(desiredActivations,
      desiredDerivatives);
  CheckInverseCorrect<SoftplusFunction>(desiredActivations);
}

BOOST_AUTO_TEST_SUITE_END();
