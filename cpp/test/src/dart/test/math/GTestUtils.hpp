/*
 * Copyright (c) 2011-2021, The DART development contributors
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

#pragma once

#include <tuple>

#include <Eigen/Core>
#include <gtest/gtest.h>

#include "dart/common/stl_utility.hpp"
#include "dart/math/constant.hpp"

//==============================================================================
#define EXPECT_S_EQ(val1, val2)                                                \
  if constexpr (::std::is_same_v<Scalar, float>) {                             \
    EXPECT_FLOAT_EQ(val1, val2);                                               \
  } else {                                                                     \
    EXPECT_DOUBLE_EQ(val1, val2);                                              \
  }                                                                            \
  do {                                                                         \
  } while (0)
// do {} while (0) is to require semicolon after the macro

//==============================================================================
#define EXPECT_VECTOR2S_EQ(expected, actual)                                   \
  EXPECT_S_EQ((expected)[0], (actual)[0]);                                     \
  EXPECT_S_EQ((expected)[1], (actual)[1]);                                     \
  do {                                                                         \
  } while (0)
// do {} while (0) is to require semicolon after the macro

//==============================================================================
#define EXPECT_VECTOR3S_EQ(expected, actual)                                   \
  EXPECT_S_EQ((expected)[0], (actual)[0]);                                     \
  EXPECT_S_EQ((expected)[1], (actual)[1]);                                     \
  EXPECT_S_EQ((expected)[2], (actual)[2]);                                     \
  do {                                                                         \
  } while (0)
// do {} while (0) is to require semicolon after the macro

//==============================================================================
#define EXPECT_MATRIX3S_EQ(expected, actual)                                   \
  {                                                                            \
    auto _mat3_a = expected;                                                   \
    auto _mat3_b = actual;                                                     \
    for (auto _i = 0; _i < 3; ++_i) {                                          \
      for (auto _j = 0; _j < 3; ++_j) {                                        \
        auto _a = _mat3_a(_i, _j);                                             \
        auto _b = _mat3_b(_i, _j);                                             \
        EXPECT_S_EQ(_a, _b);                                                   \
      }                                                                        \
    }                                                                          \
  }                                                                            \
  do {                                                                         \
  } while (0)
// do {} while (0) is to require semicolon after the macro

//==============================================================================
#define EXPECT_TRANSFORM3S_EQ(expected, actual)                                \
  {                                                                            \
    auto _mat3_a = expected;                                                   \
    auto _mat3_b = actual;                                                     \
    for (auto _i = 0; _i < 4; ++_i) {                                          \
      for (auto _j = 0; _j < 4; ++_j) {                                        \
        auto _a = _mat3_a(_i, _j);                                             \
        auto _b = _mat3_b(_i, _j);                                             \
        EXPECT_S_EQ(_a, _b);                                                   \
      }                                                                        \
    }                                                                          \
  }                                                                            \
  do {                                                                         \
  } while (0)
// do {} while (0) is to require semicolon after the macro

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

namespace dart::test {

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
    for (std::size_t i = 0; i < m1; i++) {
      for (std::size_t j = 0; j < n1; j++) {
        if (std::isnan(expected(i, j)) ^ std::isnan(actual(i, j))) {
          return false;
        } else if (std::abs(expected(i, j)) > 1) {
          // Test relative error for values that are larger than 1
          if (std::abs((expected(i, j) - actual(i, j)) / expected(i, j)) > tol)
            return false;
        } else if (std::abs(expected(i, j) - actual(i, j)) > tol) {
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
    for (std::size_t i = 0; i < m1; i++) {
      for (std::size_t j = 0; j < n1; j++) {
        if (std::isnan(static_cast<double>(expected(i, j)))
            ^ std::isnan(static_cast<double>(actual(i, j)))) {
          return false;
        } else if (std::abs(expected(i, j)) > 1) {
          // Test relative error for values that are larger than 1
          if (std::abs((expected(i, j) - actual(i, j)) / expected(i, j)) > tol)
            return false;
        } else if (std::abs(expected(i, j) - actual(i, j)) > tol) {
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
template <typename Scalar>
Scalar eps_for_diff()
{
  if constexpr (std::is_same_v<Scalar, float>) {
    return 1e-2;
  } else if constexpr (std::is_same_v<Scalar, double>) {
    return 1e-3;
  } else if constexpr (std::is_same_v<Scalar, long double>) {
    return 1e-4;
  } else {
    common::static_assert_no_match();
    return {};
  }
}

//==============================================================================
template <typename Scalar>
Scalar tol_for_equals()
{
  if constexpr (std::is_same_v<Scalar, float>) {
    return 1e-3;
  } else if constexpr (std::is_same_v<Scalar, double>) {
    return 1e-6;
  } else if constexpr (std::is_same_v<Scalar, long double>) {
    return 1e-9;
  } else {
    return {};
  }
}

//==============================================================================
/// Returns true if the two matrices are equal within the given bound
template <typename T1, typename T2>
bool equals(
    const T1& expected,
    const T2& actual,
    typename T1::Scalar tol = tol_for_equals<typename T1::Scalar>())
{
  const bool result = detail::EqualsImpl<T1, T2>::run(expected, actual, tol);
  if (!result) {
    std::cout << "Expected: \n" << expected << std::endl;
    std::cout << "Actual  : \n" << actual << std::endl;
  }
  return result;
}

//==============================================================================
template <class T, class S>
struct Case
{
  using type = T;

  static std::string GetParam()
  {
    return S::str;
  }
};

//==============================================================================
template <class TupleType, class TupleParam, std::size_t I>
struct make_case
{
  static constexpr std::size_t N = std::tuple_size<TupleParam>::value;

  using type = Case<
      typename std::tuple_element<I / N, TupleType>::type,
      typename std::tuple_element<I % N, TupleParam>::type>;
};

//==============================================================================
template <class T1, class T2, class Is>
struct make_combinations;

//==============================================================================
template <class TupleType, class TupleParam, std::size_t... Is>
struct make_combinations<TupleType, TupleParam, std::index_sequence<Is...>>
{
  using tuples
      = std::tuple<typename make_case<TupleType, TupleParam, Is>::type...>;
};

//==============================================================================
template <class TupleTypes, class... Params>
using Combinations_t = typename make_combinations<
    TupleTypes,
    std::tuple<Params...>,
    std::make_index_sequence<
        (std::tuple_size<TupleTypes>::value) * (sizeof...(Params))>>::tuples;

} // namespace dart::test
