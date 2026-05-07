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

#pragma once

#include <dart/simd/config.hpp>

#include <concepts>
#include <limits>
#include <numbers>
#include <type_traits>

namespace dart::simd {

namespace detail {

template <std::size_t W>
struct HasFullIntegerSimd : std::true_type
{
};

#if defined(DART_SIMD_AVX) && !defined(DART_SIMD_AVX2)
template <>
struct HasFullIntegerSimd<8> : std::false_type
{
};
#endif

} // namespace detail

/// Mathematical constants for SIMD operations
/// All constants are provided as constexpr template variables for float/double
template <typename T>
struct MathConstants
{
  static_assert(
      std::floating_point<T>, "MathConstants requires floating point type");

  // Pi and related constants
  static constexpr T pi = std::numbers::pi_v<T>;
  static constexpr T twoPi = T(2) * std::numbers::pi_v<T>;
  static constexpr T halfPi = std::numbers::pi_v<T> / T(2);
  static constexpr T quarterPi = std::numbers::pi_v<T> / T(4);
  static constexpr T invPi = std::numbers::inv_pi_v<T>;
  static constexpr T invTwoPi = std::numbers::inv_pi_v<T> / T(2);
  static constexpr T twoOverPi = T(2) * std::numbers::inv_pi_v<T>;
  static constexpr T fourOverPi = T(4) * std::numbers::inv_pi_v<T>;

  // e and related constants
  static constexpr T e = std::numbers::e_v<T>;
  static constexpr T log2E = std::numbers::log2e_v<T>;
  static constexpr T log10E = std::numbers::log10e_v<T>;
  static constexpr T ln2 = std::numbers::ln2_v<T>;
  static constexpr T ln10 = std::numbers::ln10_v<T>;
  static constexpr T invLn2 = std::numbers::log2e_v<T>;

  // Square roots
  static constexpr T sqrt2 = std::numbers::sqrt2_v<T>;
  static constexpr T invSqrt2 = T(1) / std::numbers::sqrt2_v<T>;
  static constexpr T sqrtPi = T(1.77245385090551602729816748334114518);
  static constexpr T invSqrtPi = std::numbers::inv_sqrtpi_v<T>;
  static constexpr T sqrt2Pi = T(2.50662827463100050241576528481104525);

  // Special values
  static constexpr T infinity = std::numeric_limits<T>::infinity();
  static constexpr T negInfinity = -std::numeric_limits<T>::infinity();
  static constexpr T nan = std::numeric_limits<T>::quiet_NaN();
  static constexpr T epsilon = std::numeric_limits<T>::epsilon();
  static constexpr T minPositive = std::numeric_limits<T>::min();
  static constexpr T maxValue = std::numeric_limits<T>::max();
  static constexpr T lowest = std::numeric_limits<T>::lowest();

  // Range reduction constants for sin/cos (extended precision)
  // These are used for argument reduction: x = x - n * (c1 + c2 + c3)
  // where c1 + c2 + c3 = pi/2 with high precision
  static constexpr T pio2_1 = []() {
    if constexpr (std::same_as<T, float>) {
      return T(0.78515625);
    } else {
      return T(7.85398125648498535156e-1);
    }
  }();

  static constexpr T pio2_2 = []() {
    if constexpr (std::same_as<T, float>) {
      return T(2.4187564849853515625e-4);
    } else {
      return T(3.77489470793079817668e-8);
    }
  }();

  static constexpr T pio2_3 = []() {
    if constexpr (std::same_as<T, float>) {
      return T(3.77489497744594108e-8);
    } else {
      return T(2.69515142907905952645e-15);
    }
  }();

  // Range reduction constants for exp
  static constexpr T expHi = []() {
    if constexpr (std::same_as<T, float>) {
      return T(88.3762626647949f);
    } else {
      return T(709.43613930310391424428);
    }
  }();

  static constexpr T expLo = []() {
    if constexpr (std::same_as<T, float>) {
      return T(-88.3762626647949f);
    } else {
      return T(-709.43613930310391424428);
    }
  }();

  // Log constants
  static constexpr T logLo = []() {
    if constexpr (std::same_as<T, float>) {
      return T(1.1754943508e-38f); // FLT_MIN
    } else {
      return T(2.2250738585072014e-308); // DBL_MIN
    }
  }();
};

// Convenience aliases
template <typename T>
inline constexpr T Pi = MathConstants<T>::pi;

template <typename T>
inline constexpr T TwoPi = MathConstants<T>::twoPi;

template <typename T>
inline constexpr T HalfPi = MathConstants<T>::halfPi;

template <typename T>
inline constexpr T InvPi = MathConstants<T>::invPi;

template <typename T>
inline constexpr T FourOverPi = MathConstants<T>::fourOverPi;

template <typename T>
inline constexpr T E = MathConstants<T>::e;

template <typename T>
inline constexpr T Ln2 = MathConstants<T>::ln2;

template <typename T>
inline constexpr T InvLn2 = MathConstants<T>::invLn2;

template <typename T>
inline constexpr T Log2E = MathConstants<T>::log2E;

template <typename T>
inline constexpr T Sqrt2 = MathConstants<T>::sqrt2;

template <typename T>
inline constexpr T InvSqrt2 = MathConstants<T>::invSqrt2;

template <typename T>
inline constexpr T Infinity = MathConstants<T>::infinity;

template <typename T>
inline constexpr T NaN = MathConstants<T>::nan;

} // namespace dart::simd
