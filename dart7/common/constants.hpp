/*
 * Copyright (c) 2011-2025, The DART development contributors
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

#include <concepts>
#include <numbers>
#include <type_traits>

namespace dart7 {
inline namespace constants {

/// Type-safe access to pi for arbitrary arithmetic types.
template <std::floating_point T>
inline constexpr T pi_v = std::numbers::pi_v<T>;

/// Double-precision pi overload for convenience.
inline constexpr double pi = pi_v<double>;

/// Float-precision pi overload for convenience.
inline constexpr float pi_f = pi_v<float>;

/// 2*pi generalized for any floating point type.
template <std::floating_point T>
inline constexpr T twoPi_v = static_cast<T>(2) * pi_v<T>;

inline constexpr double twoPi = twoPi_v<double>;
inline constexpr float twoPi_f = twoPi_v<float>;

/// pi/2 generalized for any floating point type.
template <std::floating_point T>
inline constexpr T halfPi_v = pi_v<T> / static_cast<T>(2);

inline constexpr double halfPi = halfPi_v<double>;
inline constexpr float halfPi_f = halfPi_v<float>;

/// pi/4 generalized for any floating point type.
template <std::floating_point T>
inline constexpr T quarterPi_v = pi_v<T> / static_cast<T>(4);

inline constexpr double quarterPi = quarterPi_v<double>;
inline constexpr float quarterPi_f = quarterPi_v<float>;

/// Degrees-to-radians conversion factors.
template <std::floating_point T>
inline constexpr T degToRad_v = pi_v<T> / static_cast<T>(180);

inline constexpr double degToRad = degToRad_v<double>;
inline constexpr float degToRad_f = degToRad_v<float>;

/// Radians-to-degrees conversion factors.
template <std::floating_point T>
inline constexpr T radToDeg_v = static_cast<T>(180) / pi_v<T>;

inline constexpr double radToDeg = radToDeg_v<double>;
inline constexpr float radToDeg_f = radToDeg_v<float>;

/// Convert degrees to radians for any floating point type.
template <std::floating_point T>
constexpr T toRadians(T degrees)
{
  return degrees * degToRad_v<T>;
}

/// Convert radians to degrees for any floating point type.
template <std::floating_point T>
constexpr T toDegrees(T radians)
{
  return radians * radToDeg_v<T>;
}

} // namespace constants
} // namespace dart7
