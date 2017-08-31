/**
 * @file test_tools.hpp
 * @author Ryan Curtin
 *
 * This file includes some useful macros for tests.
 *
 * mlpack is free software; you may redistribute it and/or modify it under the
 * terms of the 3-clause BSD license.  You should have received a copy of the
 * 3-clause BSD license along with mlpack.  If not, see
 * http://www.opensource.org/licenses/BSD-3-Clause for more information.
 */
#ifndef MLPACK_TESTS_TEST_TOOLS_HPP
#define MLPACK_TESTS_TEST_TOOLS_HPP

#include <mlpack/core.hpp>
#include <boost/version.hpp>

// This is only necessary for pre-1.36 Boost.Test.
#if BOOST_VERSION < 103600

#include <boost/test/floating_point_comparison.hpp>
#include <boost/test/auto_unit_test.hpp>

// This depends on other macros.  Probably not a great idea... but it works, and
// we only need it for ancient Boost versions.
#define BOOST_REQUIRE_GE( L, R ) \
    BOOST_REQUIRE_EQUAL( (L >= R), true )

#define BOOST_REQUIRE_NE( L, R ) \
    BOOST_REQUIRE_EQUAL( (L != R), true )

#define BOOST_REQUIRE_LE( L, R ) \
    BOOST_REQUIRE_EQUAL( (L <= R), true )

#define BOOST_REQUIRE_LT( L, R ) \
    BOOST_REQUIRE_EQUAL( (L < R), true )

#define BOOST_REQUIRE_GT( L, R ) \
    BOOST_REQUIRE_EQUAL( (L > R), true )

#endif

// Require the approximation L to be within a relative error of E respect to the
// actual value R.
#define REQUIRE_RELATIVE_ERR( L, R, E ) \
    BOOST_REQUIRE_LE( std::abs((R) - (L)), (E) * std::abs(R))

// Check the values of two matrices.
inline void CheckMatrices(arma::mat& a, arma::mat& b)
{
  BOOST_REQUIRE_EQUAL(a.n_rows, b.n_rows);
  BOOST_REQUIRE_EQUAL(a.n_cols, b.n_cols);

  for (size_t i = 0; i < a.n_elem; ++i)
  {
    if (std::abs(a[i]) < 1e-5)
      BOOST_REQUIRE_SMALL(b[i], 1e-5);
    else
      BOOST_REQUIRE_CLOSE(a[i], b[i], 1e-5);
  }
}

// Check the values of two unsigned matrices.
inline void CheckMatrices(arma::Mat<size_t>& a, arma::Mat<size_t>& b)
{
  BOOST_REQUIRE_EQUAL(a.n_rows, b.n_rows);
  BOOST_REQUIRE_EQUAL(a.n_cols, b.n_cols);

  for (size_t i = 0; i < a.n_elem; ++i)
    BOOST_REQUIRE_EQUAL(a[i], b[i]);
}

#endif
