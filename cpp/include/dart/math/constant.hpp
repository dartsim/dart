/*
 * Copyright (c) 2011-2022, The DART development contributors:
 * https://github.com/dartsim/dart/blob/main/LICENSE
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#pragma once

#include <limits>
#include <type_traits>

namespace dart::math {

/// Returns default epsilon
template <typename S = double>
static constexpr S eps()
{
  if constexpr (std::is_same_v<S, float>) {
    return 1e-6;
  } else if constexpr (std::is_same_v<S, double>) {
    return 1e-12;
  } else if constexpr (std::is_same_v<S, long double>) {
    return 1e-18;
  } else {
    return 0;
  }
}

template <typename T = double>
constexpr T default_absolute_tolerance()
{
  if constexpr (std::is_same_v<T, float>) {
    return 1e-3;
  } else if constexpr (std::is_same_v<T, double>) {
    return 1e-6;
  } else if constexpr (std::is_same_v<T, long double>) {
    return 1e-9;
  } else {
    return 0;
  }
}

template <typename T = double>
constexpr T atol()
{
  return default_absolute_tolerance<T>();
}

template <typename T = double>
constexpr T default_relative_tolerance()
{
  if constexpr (std::is_same_v<T, float>) {
    return 1e-2;
  } else if constexpr (std::is_same_v<T, double>) {
    return 1e-4;
  } else if constexpr (std::is_same_v<T, long double>) {
    return 1e-8;
  } else {
    return 0;
  }
}

template <typename T = double>
constexpr T rtol()
{
  return default_relative_tolerance<T>();
}

template <typename T = double>
constexpr T e()
{
  return 2.718281828459045235360287471352662498L;
}

template <typename S = double>
static constexpr S pi()
{
  return static_cast<S>(
      3.141592653589793238462643383279502884197169399375105820974944592L);
}

/// Pi/2 for type T.
template <typename T = double>
constexpr T half_pi()
{
  return 0.5L
         * 3.141592653589793238462643383279502884197169399375105820974944592L;
}

/// Pi/4 for type T.
template <typename T = double>
constexpr T quarter_pi()
{
  return 0.25L
         * 3.141592653589793238462643383279502884197169399375105820974944592L;
}

/// 2*Pi for type T.
template <typename T = double>
constexpr T two_pi()
{
  return 2.0L
         * 3.141592653589793238462643383279502884197169399375105820974944592L;
}

/// 4*Pi for type T.
template <typename T = double>
constexpr T four_pi()
{
  return 4.0L
         * 3.141592653589793238462643383279502884197169399375105820974944592L;
}

/// 1/Pi for type T.
template <typename T = double>
constexpr T inv_pi()
{
  return 1.0L
         / 3.141592653589793238462643383279502884197169399375105820974944592L;
}

/// 1/(2*Pi) for type T.
template <typename T = double>
constexpr T inv_two_pi()
{
  return 1.0L
         / (2.0L
            * 3.141592653589793238462643383279502884197169399375105820974944592L);
}

/// 1/(4*Pi) for type T.
template <typename T = double>
constexpr T inv_four_pi()
{
  return 1.0L
         / (4.0L
            * 3.141592653589793238462643383279502884197169399375105820974944592L);
}

/// The golden ratio
template <typename T = double>
constexpr T phi()
{
  return 1.618033988749894848204586834365638117720309179805762862135448623L;
}

//------------------------------------------------------------------------------
// Physics
//------------------------------------------------------------------------------

/// Gravity.
template <typename T = double>
constexpr T gravity()
{
  return -9.8L; // TODO: Find higher precision value
}

/// Water density.
template <typename T = double>
constexpr T water_density()
{
  return 1000L; // TODO: Find higher precision value
}

/// Speed of sound in water at 20 degrees celcius.
template <typename T = double>
constexpr T speed_of_sound_in_water()
{
  return 1482.0L; // TODO: Find higher precision value
}

} // namespace dart::math
