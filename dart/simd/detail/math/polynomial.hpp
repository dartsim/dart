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

#include <type_traits>

#include <cmath>
#include <cstddef>

#if defined(__GNUC__) || defined(__clang__)
  #define DART_MATH_INLINE __attribute__((always_inline)) inline
#elif defined(_MSC_VER)
  #define DART_MATH_INLINE __forceinline
#else
  #define DART_MATH_INLINE inline
#endif

namespace dart::simd {

template <typename T, std::size_t W>
struct Vec;

template <typename T, std::size_t W>
Vec<T, W> fmadd(const Vec<T, W>& a, const Vec<T, W>& b, const Vec<T, W>& c);

namespace detail {

template <typename Value, std::size_t N>
DART_MATH_INLINE Value estrinImpl(const Value& x, const Value (&coeff)[N])
{
  constexpr std::size_t nRec = (N - 1) / 2;
  constexpr std::size_t nFma = N / 2;

  Value coeffRec[nRec + 1];

  for (std::size_t i = 0; i < nFma; ++i) {
    coeffRec[i] = fmadd(x, coeff[2 * i + 1], coeff[2 * i]);
  }

  if constexpr (nRec == nFma) {
    coeffRec[nRec] = coeff[N - 1];
  }

  if constexpr (nRec == 0) {
    return coeffRec[0];
  } else {
    Value x2 = x * x;
    return estrinImpl(x2, coeffRec);
  }
}

template <typename Value, std::size_t N>
DART_MATH_INLINE Value hornerImpl(const Value& x, const Value (&coeff)[N])
{
  Value accum = coeff[N - 1];
  for (std::size_t i = 1; i < N; ++i) {
    accum = fmadd(x, accum, coeff[N - 1 - i]);
  }
  return accum;
}

template <std::size_t N>
DART_MATH_INLINE float estrinScalar(float x, const float (&coeff)[N])
{
  if constexpr (N == 1) {
    return coeff[0];
  } else if constexpr (N == 2) {
    return std::fma(x, coeff[1], coeff[0]);
  } else {
    constexpr std::size_t nFma = N / 2;
    constexpr std::size_t nRec = (N - 1) / 2;
    float coeffRec[nRec + 1];

    for (std::size_t i = 0; i < nFma; ++i) {
      coeffRec[i] = std::fma(x, coeff[2 * i + 1], coeff[2 * i]);
    }
    if constexpr (nRec == nFma) {
      coeffRec[nRec] = coeff[N - 1];
    }

    float x2 = x * x;
    return estrinScalar(x2, coeffRec);
  }
}

template <std::size_t N>
DART_MATH_INLINE double estrinScalar(double x, const double (&coeff)[N])
{
  if constexpr (N == 1) {
    return coeff[0];
  } else if constexpr (N == 2) {
    return std::fma(x, coeff[1], coeff[0]);
  } else {
    constexpr std::size_t nFma = N / 2;
    constexpr std::size_t nRec = (N - 1) / 2;
    double coeffRec[nRec + 1];

    for (std::size_t i = 0; i < nFma; ++i) {
      coeffRec[i] = std::fma(x, coeff[2 * i + 1], coeff[2 * i]);
    }
    if constexpr (nRec == nFma) {
      coeffRec[nRec] = coeff[N - 1];
    }

    double x2 = x * x;
    return estrinScalar(x2, coeffRec);
  }
}

template <std::size_t N>
DART_MATH_INLINE float hornerScalar(float x, const float (&coeff)[N])
{
  float accum = coeff[N - 1];
  for (std::size_t i = 1; i < N; ++i) {
    accum = std::fma(x, accum, coeff[N - 1 - i]);
  }
  return accum;
}

template <std::size_t N>
DART_MATH_INLINE double hornerScalar(double x, const double (&coeff)[N])
{
  double accum = coeff[N - 1];
  for (std::size_t i = 1; i < N; ++i) {
    accum = std::fma(x, accum, coeff[N - 1 - i]);
  }
  return accum;
}

} // namespace detail

template <typename Value, typename... Cs>
[[nodiscard]] DART_MATH_INLINE Value estrin(const Value& x, Cs... cs)
{
  using scalar_t = typename Value::scalar_type;
  Value coeffs[]{Value::broadcast(static_cast<scalar_t>(cs))...};
  return detail::estrinImpl(x, coeffs);
}

template <typename... Cs>
[[nodiscard]] DART_MATH_INLINE float estrin(float x, Cs... cs)
{
  float coeffs[]{static_cast<float>(cs)...};
  return detail::estrinScalar(x, coeffs);
}

template <typename... Cs>
[[nodiscard]] DART_MATH_INLINE double estrin(double x, Cs... cs)
{
  double coeffs[]{static_cast<double>(cs)...};
  return detail::estrinScalar(x, coeffs);
}

template <typename Value, typename... Cs>
[[nodiscard]] DART_MATH_INLINE Value horner(const Value& x, Cs... cs)
{
  using scalar_t = typename Value::scalar_type;
  Value coeffs[]{Value::broadcast(static_cast<scalar_t>(cs))...};
  return detail::hornerImpl(x, coeffs);
}

template <typename... Cs>
[[nodiscard]] DART_MATH_INLINE float horner(float x, Cs... cs)
{
  float coeffs[]{static_cast<float>(cs)...};
  return detail::hornerScalar(x, coeffs);
}

template <typename... Cs>
[[nodiscard]] DART_MATH_INLINE double horner(double x, Cs... cs)
{
  double coeffs[]{static_cast<double>(cs)...};
  return detail::hornerScalar(x, coeffs);
}

template <typename Value>
[[nodiscard]] DART_MATH_INLINE Value square(const Value& x)
{
  return x * x;
}

[[nodiscard]] DART_MATH_INLINE float square(float x)
{
  return x * x;
}

[[nodiscard]] DART_MATH_INLINE double square(double x)
{
  return x * x;
}

} // namespace dart::simd
