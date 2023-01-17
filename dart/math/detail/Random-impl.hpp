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

#pragma once

#include <dart/math/Fwd.hpp>
#include <dart/math/Random.hpp>

namespace dart::math {

namespace detail {

//==============================================================================
template <typename FloatType>
using UniformRealDist = std::uniform_real_distribution<FloatType>;

template <typename IntType>
using UniformIntDist = std::uniform_int_distribution<IntType>;

template <typename FloatType>
using NormalRealDist = std::normal_distribution<FloatType>;

//==============================================================================
template <template <typename...> class C, typename... Ts>
std::true_type is_base_of_template_impl(const C<Ts...>*);

template <template <typename...> class C>
std::false_type is_base_of_template_impl(...);

template <template <typename...> class C, typename T>
using is_base_of_template
    = decltype(is_base_of_template_impl<C>(std::declval<T*>()));

template <typename T>
using is_base_of_matrix = is_base_of_template<::Eigen::MatrixBase, T>;

template <typename T>
constexpr bool is_base_of_matrix_v = is_base_of_matrix<T>::value;

//==============================================================================
/// Check whether \c T can be used for std::uniform_int_distribution<T>
/// Reference:
/// https://en.cppreference.com/w/cpp/numeric/random/uniform_int_distribution
template <typename T, typename Enable = void>
struct is_compatible_to_uniform_int_distribution : std::false_type
{
  // Define nothing
};

// clang-format off

template <typename T>
struct is_compatible_to_uniform_int_distribution<
    T, typename std::enable_if<
        std::is_same<typename std::remove_cv<T>::type, short>::value
        || std::is_same<typename std::remove_cv<T>::type, int>::value
        || std::is_same<typename std::remove_cv<T>::type, long>::value
        || std::is_same<typename std::remove_cv<T>::type, long long>::value
        || std::is_same<typename std::remove_cv<T>::type, unsigned short>::value
        || std::is_same<typename std::remove_cv<T>::type, unsigned int>::value
        || std::is_same<typename std::remove_cv<T>::type, unsigned long>::value
        || std::is_same<typename std::remove_cv<T>::type, unsigned long long>::value
        >::type
    > : std::true_type
{
  // Define nothing
};

template <typename T>
constexpr bool is_compatible_to_uniform_int_distribution_v = is_compatible_to_uniform_int_distribution<T>::value;

// clang-format on

//==============================================================================
template <typename T, typename Generator>
[[nodiscard]] T genUniformScalar(
    const T& min, const T& max, Generator& generator)
{
  if constexpr (std::is_floating_point_v<T>) {
    UniformRealDist<T> dist(min, max);
    return dist(generator);
  } else if constexpr (is_compatible_to_uniform_int_distribution_v<T>) {
    UniformIntDist<T> dist(min, max);
    return dist(generator);
  }
}

//==============================================================================
template <typename Derived, typename Generator>
[[nodiscard]] typename Derived::PlainObject genUniformMatrix(
    const ::Eigen::MatrixBase<Derived>& min,
    const ::Eigen::MatrixBase<Derived>& max,
    Generator& generator)
{
  // Dynamic size matrix
  if constexpr (
      !Derived::IsVectorAtCompileTime
      && Derived::SizeAtCompileTime == ::Eigen::Dynamic) {
    return Derived::PlainObject::NullaryExpr(
        min.rows(), min.cols(), [&](const int i, const int j) {
          return genUniformScalar(min(i, j), max(i, j), generator);
        });
  } // Fixed size matrix
  else if constexpr (
      !Derived::IsVectorAtCompileTime
      && Derived::SizeAtCompileTime != ::Eigen::Dynamic) {
    return Derived::PlainObject::NullaryExpr([&](const int i, const int j) {
      return genUniformScalar(min(i, j), max(i, j), generator);
    });
  } // Dynamic size vector
  else if constexpr (
      Derived::IsVectorAtCompileTime
      && Derived::SizeAtCompileTime == ::Eigen::Dynamic) {
    return Derived::PlainObject::NullaryExpr(min.size(), [&](const int i) {
      return genUniformScalar(min[i], max[i], generator);
    });
  } // Fixed size vector
  else if constexpr (
      Derived::IsVectorAtCompileTime
      && Derived::SizeAtCompileTime != ::Eigen::Dynamic) {
    return Derived::PlainObject::NullaryExpr([&](const int i) {
      return genUniformScalar(min[i], max[i], generator);
    });
  }
}

//==============================================================================
template <typename T, typename Generator>
[[nodiscard]] T genUniform(const T& min, const T& max, Generator& generator)
{
  // Scalar type
  if constexpr (std::is_arithmetic_v<T>) {
    return genUniformScalar(min, max, generator);
  } // Matrix type
  else if constexpr (is_base_of_matrix_v<T>) {
    return genUniformMatrix(min, max, generator);
  } // Unsupported type
  else {
    static_assert(
        std::is_arithmetic_v<T> || is_base_of_matrix_v<T>,
        "genUniform() is only supported for arithmetic types and Eigen types");
  }
}

//==============================================================================
template <typename T, typename Generator>
[[nodiscard]] T genNormalScalar(
    const T& mean, const T& sigma, Generator& generator)
{
  // Real number type
  if constexpr (std::is_floating_point_v<T>) {
    NormalRealDist<T> dist(mean, sigma);
    return dist(generator);
  } // Integer type
  else if constexpr (is_compatible_to_uniform_int_distribution_v<T>) {
    const float realNumber = genNormalScalar<float>(mean, sigma, generator);
    return std::round(realNumber);
  }
  // Unsupported type
  else {
    static_assert(
        std::is_floating_point_v<T>,
        "genNormalScalar() is only supported for floating point types");
  }
}

//==============================================================================
template <typename Derived, typename Generator>
[[nodiscard]] typename Derived::PlainObject genNormalMatrix(
    const ::Eigen::MatrixBase<Derived>& mean,
    const ::Eigen::MatrixBase<Derived>& sigma,
    Generator& generator)
{
  // Dynamic size matrix
  if constexpr (
      !Derived::IsVectorAtCompileTime
      && Derived::SizeAtCompileTime == ::Eigen::Dynamic) {
    return Derived::PlainObject::NullaryExpr(
        mean.rows(), mean.cols(), [&](const int i, const int j) {
          return genNormalScalar(mean(i, j), sigma(i, j), generator);
        });
  } // Fixed size matrix
  else if constexpr (
      !Derived::IsVectorAtCompileTime
      && Derived::SizeAtCompileTime != ::Eigen::Dynamic) {
    return Derived::PlainObject::NullaryExpr([&](const int i, const int j) {
      return genNormalScalar(mean(i, j), sigma(i, j), generator);
    });
  } // Dynamic size vector
  else if constexpr (
      Derived::IsVectorAtCompileTime
      && Derived::SizeAtCompileTime == ::Eigen::Dynamic) {
    return Derived::PlainObject::NullaryExpr(mean.size(), [&](const int i) {
      return genNormalScalar(mean[i], sigma[i], generator);
    });
  } // Fixed size vector
  else if constexpr (
      Derived::IsVectorAtCompileTime
      && Derived::SizeAtCompileTime != ::Eigen::Dynamic) {
    return Derived::PlainObject::NullaryExpr([&](const int i) {
      return genNormalScalar(mean[i], sigma[i], generator);
    });
  }
}

//==============================================================================
template <typename T, typename Generator>
[[nodiscard]] T genNormal(const T& mean, const T& sigma, Generator& generator)
{
  // Scalar type
  if constexpr (std::is_arithmetic_v<T>) {
    return genNormalScalar(mean, sigma, generator);
  } // Matrix type
  else if constexpr (is_base_of_matrix_v<T>) {
    return genNormalMatrix(mean, sigma, generator);
  } // Unsupported type
  else {
    static_assert(
        std::is_arithmetic_v<T> || is_base_of_matrix_v<T>,
        "genNormal() is only supported for arithmetic types and Eigen types");
  }
}

} // namespace detail

//==============================================================================
template <typename Generator>
Random<Generator>& Random<Generator>::GetInstance()
{
  static Random<Generator> instance;
  return instance;
}

//==============================================================================
template <typename Generator>
Random<Generator>::Random(uint32_t seed) : mSeed(seed), mGenerator(seed)
{
  // Do nothing
}

//==============================================================================
template <typename Generator>
uint32_t Random<Generator>::getSeed() const
{
  return mSeed;
}

//==============================================================================
template <typename Generator>
void Random<Generator>::setSeed(uint32_t seed)
{
  if (seed == mSeed) {
    return;
  }
  mSeed = seed;
  mGenerator.seed(mSeed);
}

//==============================================================================
template <typename Generator>
template <typename T>
T Random<Generator>::uniform(const T& min, const T& max)
{
  return detail::genUniform(min, max, mGenerator);
}

//==============================================================================
template <typename Generator>
template <typename FixedSizeT>
FixedSizeT Random<Generator>::uniform(
    typename FixedSizeT::Scalar min, typename FixedSizeT::Scalar max)
{
  return detail::genUniformMatrix(
      FixedSizeT::Constant(min), FixedSizeT::Constant(max), mGenerator);
}

//==============================================================================
template <typename Generator>
template <typename DynamicSizeVectorT>
DynamicSizeVectorT Random<Generator>::uniform(
    int size,
    typename DynamicSizeVectorT::Scalar min,
    typename DynamicSizeVectorT::Scalar max)
{
  return detail::genUniformMatrix(
      DynamicSizeVectorT::Constant(size, min),
      DynamicSizeVectorT::Constant(size, max),
      mGenerator);
}

//==============================================================================
template <typename Generator>
template <typename DynamicSizeMatrixT>
DynamicSizeMatrixT Random<Generator>::uniform(
    int rows,
    int cols,
    typename DynamicSizeMatrixT::Scalar min,
    typename DynamicSizeMatrixT::Scalar max)
{
  return detail::genUniformMatrix(
      DynamicSizeMatrixT::Constant(rows, cols, min),
      DynamicSizeMatrixT::Constant(rows, cols, max),
      mGenerator);
}

//==============================================================================
template <typename Generator>
template <typename T>
T Random<Generator>::normal(const T& mean, const T& sigma)
{
  return detail::genNormal(mean, sigma, mGenerator);
}

//==============================================================================
template <typename Generator>
template <typename FixedSizeT>
FixedSizeT Random<Generator>::normal(
    typename FixedSizeT::Scalar mean, typename FixedSizeT::Scalar sigma)
{
  return detail::genNormalMatrix(
      FixedSizeT::Constant(mean), FixedSizeT::Constant(sigma), mGenerator);
}

//==============================================================================
template <typename Generator>
template <typename DynamicSizeVectorT>
DynamicSizeVectorT Random<Generator>::normal(
    int size,
    typename DynamicSizeVectorT::Scalar mean,
    typename DynamicSizeVectorT::Scalar sigma)
{
  return detail::genNormalMatrix(
      DynamicSizeVectorT::Constant(size, mean),
      DynamicSizeVectorT::Constant(size, sigma),
      mGenerator);
}

//==============================================================================
template <typename Generator>
template <typename DynamicSizeMatrixT>
DynamicSizeMatrixT Random<Generator>::normal(
    int rows,
    int cols,
    typename DynamicSizeMatrixT::Scalar mean,
    typename DynamicSizeMatrixT::Scalar sigma)
{
  return detail::genNormalMatrix(
      DynamicSizeMatrixT::Constant(rows, cols, mean),
      DynamicSizeMatrixT::Constant(rows, cols, sigma),
      mGenerator);
}

//==============================================================================
template <typename T>
T Uniform(const T& min, const T& max)
{
  auto& rng = Random<>::GetInstance();
  return rng.uniform(min, max);
}

//==============================================================================
template <typename FixedSizeT>
FixedSizeT Uniform(
    typename FixedSizeT::Scalar min, typename FixedSizeT::Scalar max)
{
  auto& rng = Random<>::GetInstance();
  return rng.uniform<FixedSizeT>(min, max);
}

//==============================================================================
template <typename DynamicSizeVectorT>
DynamicSizeVectorT Uniform(
    int size,
    typename DynamicSizeVectorT::Scalar min,
    typename DynamicSizeVectorT::Scalar max)
{
  auto& rng = Random<>::GetInstance();
  return rng.uniform<DynamicSizeVectorT>(size, min, max);
}

//==============================================================================
template <typename DynamicSizeMatrixT>
DynamicSizeMatrixT Uniform(
    int rows,
    int cols,
    typename DynamicSizeMatrixT::Scalar min,
    typename DynamicSizeMatrixT::Scalar max)
{
  auto& rng = Random<>::GetInstance();
  return rng.uniform<DynamicSizeMatrixT>(rows, cols, min, max);
}

//==============================================================================
template <typename T>
T Normal(const T& min, const T& max)
{
  auto& rng = Random<>::GetInstance();
  return rng.normal(min, max);
}

//==============================================================================
template <typename FixedSizeT>
FixedSizeT Normal(
    typename FixedSizeT::Scalar mean, typename FixedSizeT::Scalar sigma)
{
  auto& rng = Random<>::GetInstance();
  return rng.normal<FixedSizeT>(mean, sigma);
}

//==============================================================================
template <typename DynamicSizeVectorT>
DynamicSizeVectorT Normal(
    int size,
    typename DynamicSizeVectorT::Scalar mean,
    typename DynamicSizeVectorT::Scalar sigma)
{
  auto& rng = Random<>::GetInstance();
  return rng.normal<DynamicSizeVectorT>(size, mean, sigma);
}

//==============================================================================
template <typename DynamicSizeMatrixT>
DynamicSizeMatrixT Normal(
    int rows,
    int cols,
    typename DynamicSizeMatrixT::Scalar mean,
    typename DynamicSizeMatrixT::Scalar sigma)
{
  auto& rng = Random<>::GetInstance();
  return rng.normal<DynamicSizeMatrixT>(rows, cols, mean, sigma);
}

} // namespace dart::math
