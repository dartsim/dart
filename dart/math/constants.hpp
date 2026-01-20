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

#ifndef DART_MATH_CONSTANTS_HPP_
#define DART_MATH_CONSTANTS_HPP_

#include <limits>

#if defined(__has_include)
  #if __has_include(<numbers>)
    #include <numbers>
    #if defined(__cpp_lib_math_constants) && __cpp_lib_math_constants >= 201907L
      #define DART_MATH_HAS_STD_NUMBERS 1
    #endif
  #endif
#endif

#ifndef DART_MATH_HAS_STD_NUMBERS
  #define DART_MATH_HAS_STD_NUMBERS 0
#endif

#include "dart/common/deprecated.hpp"
#include "dart/common/diagnostics.hpp"

namespace dart {
namespace math {

namespace detail {

#if DART_MATH_HAS_STD_NUMBERS
inline constexpr long double pi_ld = std::numbers::pi_v<long double>;
inline constexpr long double phi_ld = std::numbers::phi_v<long double>;
#else
inline constexpr long double pi_ld = 3.141592653589793238462643383279502884L;
inline constexpr long double phi_ld = 1.618033988749894848204586834365638118L;
#endif

inline constexpr long double two_pi_ld = 2.0L * pi_ld;
inline constexpr long double half_pi_ld = 0.5L * pi_ld;
inline constexpr long double pi_sq_ld = pi_ld * pi_ld;

} // namespace detail

template <typename T>
inline constexpr T pi_v = static_cast<T>(detail::pi_ld);

template <typename T>
inline constexpr T phi_v = static_cast<T>(detail::phi_ld);

template <typename T>
inline constexpr T two_pi_v = static_cast<T>(detail::two_pi_ld);

template <typename T>
inline constexpr T half_pi_v = static_cast<T>(detail::half_pi_ld);

template <typename T>
inline constexpr T pi_sq_v = static_cast<T>(detail::pi_sq_ld);

template <typename T>
inline constexpr T inf_v = std::numeric_limits<T>::infinity();

template <typename T>
inline constexpr T max_v = std::numeric_limits<T>::max();

template <typename T>
inline constexpr T min_v = std::numeric_limits<T>::lowest();

template <typename T>
inline constexpr T eps_v = std::numeric_limits<T>::epsilon();

inline constexpr double pi = pi_v<double>;
inline constexpr double phi = phi_v<double>;
inline constexpr double two_pi = two_pi_v<double>;
inline constexpr double half_pi = half_pi_v<double>;
inline constexpr double pi_sq = pi_sq_v<double>;
inline constexpr double inf = inf_v<double>;
inline constexpr double max_val
    = max_v<double>; // renamed to max once legacy APIs are removed
inline constexpr double min_val
    = min_v<double>; // renamed to min once legacy APIs are removed
inline constexpr double eps = eps_v<double>;

/// Deprecated wrapper around the new dart::math variable templates (removed in
/// DART 7.1).
template <typename T>
struct DART_DEPRECATED(7.0) constants
{
  static constexpr T pi()
  {
    return pi_v<T>;
  }

  static constexpr T two_pi()
  {
    return two_pi_v<T>;
  }

  static constexpr T half_pi()
  {
    return half_pi_v<T>;
  }

  static constexpr T pi_sqr()
  {
    return pi_sq_v<T>;
  }

  static constexpr T phi()
  {
    return phi_v<T>;
  }

  static constexpr T inf()
  {
    return inf_v<T>;
  }

  static constexpr T max()
  {
    return max_v<T>;
  }

  /// std::numeric_limits<T>::lowest() (most negative finite value)
  static constexpr T min()
  {
    return min_v<T>;
  }

  static constexpr T eps()
  {
    return eps_v<T>;
  }
};

#ifdef DART_SUPPRESS_DEPRECATED_BEGIN
DART_SUPPRESS_DEPRECATED_BEGIN
#endif
using constantsf = constants<float>;
using constantsd = constants<double>;
#ifdef DART_SUPPRESS_DEPRECATED_END
DART_SUPPRESS_DEPRECATED_END
#endif

} // namespace math
} // namespace dart

#undef DART_MATH_HAS_STD_NUMBERS

#endif // DART_MATH_CONSTANTS_HPP_
