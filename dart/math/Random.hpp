/*
 * Copyright (c) 2011-2018, The DART development contributors
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

#ifndef DART_MATH_RANDOM_HPP_
#define DART_MATH_RANDOM_HPP_

#include <random>

#include <Eigen/Core>

#include "dart/math/detail/TypeTraits.hpp"

namespace dart {
namespace math {

class Random
{
public:
  /// Sets the seed value.
  static void setSeed(unsigned int seed);

  /// Changes the seed value using the default random device and returns the new
  /// seed value.
  static unsigned int changeSeed();

  /// Returns the seed value.
  static unsigned int getSeed();

  /// Returns a random number from a uniform distribution where the type of
  /// number is floating-point.
  template <typename S>
  static S uniform(
      S min,
      S max,
      typename std::enable_if<std::is_floating_point<S>::value>::type* = 0);

  /// Returns a random number from a uniform distribution where the type of
  /// number is integer.
  template <typename S>
  static S uniform(
      S min,
      S max,
      typename std::
          enable_if<detail::is_compatible_to_uniform_int_distribution<S>::
                        value>::type* = nullptr);

  /// Returns a random vector/matrix from a uniform distribution where the type
  /// of elements can be either floating-point or integer.
  template <typename Derived>
  static typename Derived::PlainObject uniform(
      const Eigen::MatrixBase<Derived>& min,
      const Eigen::MatrixBase<Derived>& max);

  /// Returns a random number from a normal distribution where the type of
  /// number is floating-point.
  template <typename S>
  static S normal(
      S mean,
      S sigma,
      typename std::enable_if<std::is_floating_point<S>::value>::
          type* = nullptr);

  /// Returns a random number from a normal distribution where the type of
  /// number is integer.
  template <typename S>
  static S normal(
      S mean,
      S sigma,
      typename std::
          enable_if<detail::is_compatible_to_uniform_int_distribution<S>::
                        value>::type* = nullptr);

private:
  using GeneratorType = std::mt19937;

  template <typename FloatType>
  using UniformRealDist = std::uniform_real_distribution<FloatType>;

  template <typename FloatType>
  using NormalRealDist = std::normal_distribution<FloatType>;

  template <typename IntType>
  using UniformIntDist = std::uniform_int_distribution<IntType>;

  /// Returns a mutable reference to the seed.
  static uint32_t& getSeedMutable();

  /// Returns a mutable reference to the random generator
  static GeneratorType& getRandGenerator();
};

} // namespace math
} // namespace dart

#include "dart/math/detail/Random-impl.hpp"

#endif // DART_MATH_RANDOM_HPP_
