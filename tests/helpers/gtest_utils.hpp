/*
 * Copyright (c) 2011, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/main/LICENSE
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

#include "dart/math/geometry.hpp"
#include "dart/math/helpers.hpp"
#include "dart/math/math_types.hpp"

#include <Eigen/Dense>
#include <gtest/gtest.h>

#include <type_traits>

#include <cmath>

//==============================================================================
#define EXPECT_VECTOR_DOUBLE_EQ(vec1, vec2)                                    \
  if (!::dart::test::equals(vec1, vec2)) {                                     \
    std::stringstream ss;                                                      \
    ss << "Expected equality of these vectors:\n"                              \
       << "  Expected: " << vec1.transpose() << "\n"                           \
       << "  Actual  : " << vec2.transpose() << "\n";                          \
    GTEST_NONFATAL_FAILURE_(ss.str().c_str());                                 \
  }                                                                            \
  do {                                                                         \
  } while (0)
// do {} while (0) is to require semicolon after the macro

//==============================================================================
#define EXPECT_MATRIX_DOUBLE_EQ(mat1, mat2)                                    \
  if (!::dart::test::equals(mat1, mat2)) {                                     \
    std::stringstream ss;                                                      \
    ss << "Expected equality of these matrices:\n"                             \
       << "  Expected:\n"                                                      \
       << "  " << mat1.matrix() << "\n"                                        \
       << "  Actual  :\n"                                                      \
       << "  " << mat2.matrix() << "\n";                                       \
    GTEST_NONFATAL_FAILURE_(ss.str().c_str());                                 \
  }                                                                            \
  do {                                                                         \
  } while (0)

//==============================================================================
#define EXPECT_ROTATION_DOUBLE_EQ(rot1, rot2)                                  \
  if (!::dart::test::rotationEquals(rot1, rot2)) {                             \
    std::stringstream ss;                                                      \
    ss << "Expected equality of these rotations:\n"                            \
       << "  Expected:\n"                                                      \
       << "  " << rot1 << "\n"                                                 \
       << "  Actual  :\n"                                                      \
       << "  " << rot2 << "\n";                                                \
    GTEST_NONFATAL_FAILURE_(ss.str().c_str());                                 \
  }                                                                            \
  do {                                                                         \
  } while (0)

//==============================================================================
#define EXPECT_TRANSFORM_DOUBLE_EQ(tf1, tf2)                                   \
  if (!::dart::test::equals(tf1, tf2)) {                                       \
    std::stringstream ss;                                                      \
    ss << "Expected equality of these transforms:\n"                           \
       << "  Expected:\n"                                                      \
       << "  " << tf1.matrix() << "\n"                                         \
       << "  Actual  :\n"                                                      \
       << "  " << tf2.matrix() << "\n";                                        \
    GTEST_NONFATAL_FAILURE_(ss.str().c_str());                                 \
  }                                                                            \
  do {                                                                         \
  } while (0)

//==============================================================================
#define EXPECT_VECTOR_NEAR(vec1, vec2, abs_error)                              \
  if (!::dart::test::equals(vec1, vec2, abs_error)) {                          \
    std::stringstream ss;                                                      \
    ss << "The element-wise difference between:\n"                             \
       << vec1.transpose() << "\n"                                             \
       << "and\n"                                                              \
       << vec2.transpose() << "\n"                                             \
       << "exceeds " << abs_error << ".\n";                                    \
    GTEST_NONFATAL_FAILURE_(ss.str().c_str());                                 \
  }                                                                            \
  do {                                                                         \
  } while (0)

//==============================================================================
#define EXPECT_MATRIX_NEAR(mat1, mat2, abs_error)                              \
  if (!::dart::test::equals(mat1, mat2, abs_error)) {                          \
    std::stringstream ss;                                                      \
    ss << "The element-wise difference between:\n"                             \
       << mat1.matrix() << "\n"                                                \
       << "and\n"                                                              \
       << mat2.matrix() << "\n"                                                \
       << "exceeds " << abs_error << ".\n";                                    \
    GTEST_NONFATAL_FAILURE_(ss.str().c_str());                                 \
  }                                                                            \
  do {                                                                         \
  } while (0)

//==============================================================================
#define EXPECT_ROTATION_NEAR(rot1, rot2, abs_error)                            \
  if (!::dart::test::rotationEquals(rot1, rot2, abs_error)) {                  \
    std::stringstream ss;                                                      \
    ss << "The distance between:\n"                                            \
       << rot1 << "\n"                                                         \
       << "and\n"                                                              \
       << rot2 << "\n"                                                         \
       << "exceeds " << abs_error << ".\n";                                    \
    GTEST_NONFATAL_FAILURE_(ss.str().c_str());                                 \
  }                                                                            \
  do {                                                                         \
  } while (0)

//==============================================================================
#define EXPECT_TRANSFORM_NEAR(tf1, tf2, abs_error)                             \
  if (!::dart::test::equals(tf1, tf2, abs_error)) {                            \
    std::stringstream ss;                                                      \
    ss << "The distance between:\n"                                            \
       << tf1.matrix() << "\n"                                                 \
       << "and\n"                                                              \
       << tf2.matrix() << "\n"                                                 \
       << "exceeds " << abs_error << ".\n";                                    \
    GTEST_NONFATAL_FAILURE_(ss.str().c_str());                                 \
  }                                                                            \
  do {                                                                         \
  } while (0)

namespace dart {
namespace test {

//==============================================================================
/// Returns true if the two matrices are equal within the given bound
template <typename T1, typename T2>
bool equals(
    const T1& expected,
    const T2& actual,
    std::common_type_t<typename T1::Scalar, typename T2::Scalar, double> tol
    = static_cast<
        std::common_type_t<typename T1::Scalar, typename T2::Scalar, double>>(
        1e-5))
{
  if (expected.rows() != actual.rows() || expected.cols() != actual.cols()) {
    return false;
  }

  using CommonScalar
      = std::common_type_t<typename T1::Scalar, typename T2::Scalar, double>;

  for (Eigen::Index i = 0; i < expected.rows(); ++i) {
    for (Eigen::Index j = 0; j < expected.cols(); ++j) {
      const CommonScalar lhs = static_cast<CommonScalar>(expected(i, j));
      const CommonScalar rhs = static_cast<CommonScalar>(actual(i, j));

      if (std::isnan(lhs) && std::isnan(rhs)) {
        continue;
      }

      if (!dart::math::isApprox(lhs, rhs, tol, tol)) {
        return false;
      }
    }
  }

  return true;
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
