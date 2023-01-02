/*
 * Copyright (c) 2011-2023, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/master/LICENSE
 *
 * This file is provided under the following "BSD-style" License:
 *   Redistribution and use in source and binary forms, with or
 *   without modification, are permitted provided that the following
 *   conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 *   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef DART_UNITTESTS_GTESTUTILS_HPP_
#define DART_UNITTESTS_GTESTUTILS_HPP_

#include <gtest/gtest.h>

#include <Eigen/Dense>
#include "dart/math/Geometry.hpp"
#include "dart/math/MathTypes.hpp"

//==============================================================================
#define EXPECT_VECTOR_DOUBLE_EQ(vec1, vec2)                                    \
  if (!::dart::test::equals(vec1, vec2))                                       \
  {                                                                            \
    std::stringstream ss;                                                      \
    ss << "Expected equality of these vectors:\n"                              \
       << "  Expected: " << vec1.transpose() << "\n"                           \
       << "  Actual  : " << vec2.transpose() << "\n";                          \
    GTEST_NONFATAL_FAILURE_(ss.str().c_str());                                 \
  }                                                                            \
  do                                                                           \
  {                                                                            \
  } while (0)
// do {} while (0) is to require semicolon after the macro

//==============================================================================
#define EXPECT_MATRIX_DOUBLE_EQ(mat1, mat2)                                    \
  if (!::dart::test::equals(mat1, mat2))                                       \
  {                                                                            \
    std::stringstream ss;                                                      \
    ss << "Expected equality of these matrices:\n"                             \
       << "  Expected:\n"                                                      \
       << "  " << mat1.matrix() << "\n"                                        \
       << "  Actual  :\n"                                                      \
       << "  " << mat2.matrix() << "\n";                                       \
    GTEST_NONFATAL_FAILURE_(ss.str().c_str());                                 \
  }                                                                            \
  do                                                                           \
  {                                                                            \
  } while (0)

//==============================================================================
#define EXPECT_ROTATION_DOUBLE_EQ(rot1, rot2)                                  \
  if (!::dart::test::rotationEquals(rot1, rot2))                               \
  {                                                                            \
    std::stringstream ss;                                                      \
    ss << "Expected equality of these rotations:\n"                            \
       << "  Expected:\n"                                                      \
       << "  " << rot1 << "\n"                                                 \
       << "  Actual  :\n"                                                      \
       << "  " << rot2 << "\n";                                                \
    GTEST_NONFATAL_FAILURE_(ss.str().c_str());                                 \
  }                                                                            \
  do                                                                           \
  {                                                                            \
  } while (0)

//==============================================================================
#define EXPECT_TRANSFORM_DOUBLE_EQ(tf1, tf2)                                   \
  if (!::dart::test::equals(tf1, tf2))                                         \
  {                                                                            \
    std::stringstream ss;                                                      \
    ss << "Expected equality of these transforms:\n"                           \
       << "  Expected:\n"                                                      \
       << "  " << tf1.matrix() << "\n"                                         \
       << "  Actual  :\n"                                                      \
       << "  " << tf2.matrix() << "\n";                                        \
    GTEST_NONFATAL_FAILURE_(ss.str().c_str());                                 \
  }                                                                            \
  do                                                                           \
  {                                                                            \
  } while (0)

//==============================================================================
#define EXPECT_VECTOR_NEAR(vec1, vec2, abs_error)                              \
  if (!::dart::test::equals(vec1, vec2, abs_error))                            \
  {                                                                            \
    std::stringstream ss;                                                      \
    ss << "The element-wise difference between:\n"                             \
       << vec1.transpose() << "\n"                                             \
       << "and\n"                                                              \
       << vec2.transpose() << "\n"                                             \
       << "exceeds " << abs_error << ".\n";                                    \
    GTEST_NONFATAL_FAILURE_(ss.str().c_str());                                 \
  }                                                                            \
  do                                                                           \
  {                                                                            \
  } while (0)

//==============================================================================
#define EXPECT_MATRIX_NEAR(mat1, mat2, abs_error)                              \
  if (!::dart::test::equals(mat1, mat2, abs_error))                            \
  {                                                                            \
    std::stringstream ss;                                                      \
    ss << "The element-wise difference between:\n"                             \
       << mat1.matrix() << "\n"                                                \
       << "and\n"                                                              \
       << mat2.matrix() << "\n"                                                \
       << "exceeds " << abs_error << ".\n";                                    \
    GTEST_NONFATAL_FAILURE_(ss.str().c_str());                                 \
  }                                                                            \
  do                                                                           \
  {                                                                            \
  } while (0)

//==============================================================================
#define EXPECT_ROTATION_NEAR(rot1, rot2, abs_error)                            \
  if (!::dart::test::rotationEquals(rot1, rot2, abs_error))                    \
  {                                                                            \
    std::stringstream ss;                                                      \
    ss << "The distance between:\n"                                            \
       << rot1 << "\n"                                                         \
       << "and\n"                                                              \
       << rot2 << "\n"                                                         \
       << "exceeds " << abs_error << ".\n";                                    \
    GTEST_NONFATAL_FAILURE_(ss.str().c_str());                                 \
  }                                                                            \
  do                                                                           \
  {                                                                            \
  } while (0)

//==============================================================================
#define EXPECT_TRANSFORM_NEAR(tf1, tf2, abs_error)                             \
  if (!::dart::test::equals(tf1, tf2, abs_error))                              \
  {                                                                            \
    std::stringstream ss;                                                      \
    ss << "The distance between:\n"                                            \
       << tf1.matrix() << "\n"                                                 \
       << "and\n"                                                              \
       << tf2.matrix() << "\n"                                                 \
       << "exceeds " << abs_error << ".\n";                                    \
    GTEST_NONFATAL_FAILURE_(ss.str().c_str());                                 \
  }                                                                            \
  do                                                                           \
  {                                                                            \
  } while (0)

namespace dart {
namespace test {

namespace detail {

template <typename DerivedA, typename DerivedB, typename Enable = void>
struct EqualsImpl
{
  static bool run(
      const Eigen::DenseBase<DerivedA>& expected,
      const Eigen::DenseBase<DerivedB>& actual,
      typename DerivedA::Scalar tol)
  {
    // Get the matrix sizes and sanity check the call
    const std::size_t n1 = expected.cols(), m1 = expected.rows();
    const std::size_t n2 = actual.cols(), m2 = actual.rows();
    if (m1 != m2 || n1 != n2)
      return false;

    // Check each index
    for (std::size_t i = 0; i < m1; i++)
    {
      for (std::size_t j = 0; j < n1; j++)
      {
        if (std::isnan(expected(i, j)) ^ std::isnan(actual(i, j)))
        {
          return false;
        }
        else if (std::abs(expected(i, j)) > 1)
        {
          // Test relative error for values that are larger than 1
          if (std::abs((expected(i, j) - actual(i, j)) / expected(i, j)) > tol)
            return false;
        }
        else if (std::abs(expected(i, j) - actual(i, j)) > tol)
        {
          return false;
        }
      }
    }

    // If no problems, the two matrices are equal
    return true;
  }
};

// Workaround to resolve: "fpclassify': ambiguous call to overloaded function
// Reference: https://stackoverflow.com/a/61646279
#ifdef _WIN32
template <typename DerivedA, typename DerivedB>
struct EqualsImpl<
    DerivedA,
    DerivedB,
    std::enable_if_t<std::is_integral<typename DerivedA::Scalar>::value>>
{
  static bool run(
      const Eigen::DenseBase<DerivedA>& expected,
      const Eigen::DenseBase<DerivedB>& actual,
      typename DerivedA::Scalar tol)
  {
    // Get the matrix sizes and sanity check the call
    const std::size_t n1 = expected.cols(), m1 = expected.rows();
    const std::size_t n2 = actual.cols(), m2 = actual.rows();
    if (m1 != m2 || n1 != n2)
      return false;

    // Check each index
    for (std::size_t i = 0; i < m1; i++)
    {
      for (std::size_t j = 0; j < n1; j++)
      {
        if (std::isnan(static_cast<double>(expected(i, j)))
            ^ std::isnan(static_cast<double>(actual(i, j))))
        {
          return false;
        }
        else if (std::abs(expected(i, j)) > 1)
        {
          // Test relative error for values that are larger than 1
          if (std::abs((expected(i, j) - actual(i, j)) / expected(i, j)) > tol)
            return false;
        }
        else if (std::abs(expected(i, j) - actual(i, j)) > tol)
        {
          return false;
        }
      }
    }

    // If no problems, the two matrices are equal
    return true;
  }
};
#endif

} // namespace detail

//==============================================================================
/// Returns true if the two matrices are equal within the given bound
template <typename T1, typename T2>
bool equals(
    const T1& expected,
    const T2& actual,
    typename T1::Scalar tol = static_cast<typename T1::Scalar>(1e-5))
{
  return detail::EqualsImpl<T1, T2>::run(expected, actual, tol);
}

//==============================================================================
inline bool rotationEquals(
    const Eigen::Matrix3d& rot1, const Eigen::Matrix3d& rot2, double tol = 1e-5)
{
  const Eigen::Matrix3d rotError = rot1.transpose() * rot2;
  const Eigen::Vector3d error = dart::math::logMap(rotError);
  return (error.norm() < tol);
}

//==============================================================================
inline bool equals(
    const Eigen::Isometry3d& tf1,
    const Eigen::Isometry3d& tf2,
    double tol = 1e-5)
{
  auto se3 = dart::math::logMap(tf1.inverse() * tf2);
  auto norm = se3.norm();

  return (norm < tol);
}

} // namespace test
} // namespace dart

#endif // DART_UNITTESTS_GTESTUTILS_HPP_
