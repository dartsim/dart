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

#include "dart/common/Macros.hpp"

#include <limits>

#include <cmath>

/* constants */

/* Modern C++ constant for 1/sqrt(2) - replaces M_SQRT1_2 macro */
namespace dart::lcpsolver::constants {
template <typename T>
inline constexpr T sqrt1_2 = T(0.7071067811865475244008443621048490L);
} // namespace dart::lcpsolver::constants

// Legacy M_SQRT1_2 macro for backward compatibility
#ifndef M_SQRT1_2
  #define M_SQRT1_2 REAL(0.7071067811865475244008443621048490)
#endif

/* floating point data type, vector, matrix and quaternion types */

#define dDOUBLE 1

namespace dart::lcpsolver {

#if defined(dSINGLE)
using dReal = float;
  #ifdef dDOUBLE
    #error You can only #define dSINGLE or dDOUBLE, not both.
  #endif // dDOUBLE
#elif defined(dDOUBLE)
using dReal = double;
#else
  #error You must #define dSINGLE or dDOUBLE
#endif

//==============================================================================
// Template Type Traits for Scalar Types
//==============================================================================

/// Type traits for scalar types used in LCP solver
/// Provides precision-specific constants and math functions
template <typename Scalar>
struct ScalarTraits
{
  /// Machine epsilon for the scalar type
  static constexpr Scalar epsilon();

  /// Positive infinity for the scalar type
  static constexpr Scalar infinity();

  /// Square root
  static Scalar sqrt(Scalar x);

  /// Absolute value
  static Scalar abs(Scalar x);

  /// Reciprocal (1/x)
  static Scalar reciprocal(Scalar x);

  /// Reciprocal square root (1/sqrt(x))
  static Scalar recip_sqrt(Scalar x);

  /// Sine
  static Scalar sin(Scalar x);

  /// Cosine
  static Scalar cos(Scalar x);
};

/// Specialization for float (32-bit)
template <>
struct ScalarTraits<float>
{
  static constexpr float epsilon()
  {
    return 1e-7f;
  }

  static constexpr float inf()
  {
    return std::numeric_limits<float>::infinity();
  }

  static float abs(float x)
  {
    return std::fabs(x);
  }
};

/// Specialization for double (64-bit)
template <>
struct ScalarTraits<double>
{
  static constexpr double epsilon()
  {
    return 1e-14;
  }

  static constexpr double inf()
  {
    return std::numeric_limits<double>::infinity();
  }

  static double abs(double x)
  {
    return std::fabs(x);
  }
};

/// Round an integer up to a multiple of 4, except that 0 and 1 are unmodified
/// (used to compute matrix leading dimensions)
constexpr int padding(int a)
{
  return (a > 1) ? ((((a)-1) | 3) + 1) : a;
}

/// Reciprocal (1/x) - template function for type-safe division
template <typename S>
inline constexpr S reciprocal(S x)
{
  return S(1) / x;
}

/* precision dependent scalar math functions */

#if defined(dSINGLE)

  #define REAL(x) (x##f) /* form a constant */

#elif defined(dDOUBLE)

  #define REAL(x) (x)

#else
  #error You must #define dSINGLE or dDOUBLE
#endif

// dInfinity - Positive infinity for dReal type (replaces odeconfig.h macro)
inline constexpr dReal dInfinity = ScalarTraits<dReal>::inf();

// Legacy dPAD macro - now defined as constexpr function
constexpr int dPAD(int a)
{
  return padding(a);
}

} // namespace dart::lcpsolver
