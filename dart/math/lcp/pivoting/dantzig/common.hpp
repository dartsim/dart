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

/*
 * This file contains code derived from Open Dynamics Engine (ODE).
 * Original copyright notice:
 *
 * Open Dynamics Engine, Copyright (C) 2001,2002 Russell L. Smith.
 * All rights reserved.  Email: russ@q12.org   Web: www.q12.org
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of EITHER:
 *   (1) The GNU Lesser General Public License as published by the Free
 *       Software Foundation; either version 2.1 of the License, or (at
 *       your option) any later version. The text of the GNU Lesser
 *       General Public License is included with this library in the
 *       file LICENSE.TXT.
 *   (2) The BSD-style license that is included with this library in
 *       the file LICENSE-BSD.TXT.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the files
 * LICENSE.TXT and LICENSE-BSD.TXT for more details.
 */

#pragma once

#include "dart/common/macros.hpp"
#include "dart/math/lcp/lcp_utils.hpp"

#include <Eigen/Core>

#include <limits>

#include <cmath>

//==============================================================================
// Memory Allocation Macros
//==============================================================================

#ifndef EFFICIENT_ALIGNMENT
  #define EFFICIENT_ALIGNMENT 16
#endif

/* utility */

/* round something up to be a multiple of the EFFICIENT_ALIGNMENT */

#define dEFFICIENT_SIZE(x)                                                     \
  (((x) + (EFFICIENT_ALIGNMENT - 1)) & ~((size_t)(EFFICIENT_ALIGNMENT - 1)))
#define dEFFICIENT_PTR(p) ((void*)dEFFICIENT_SIZE((size_t)(p)))
#define dOFFSET_EFFICIENTLY(p, b) ((void*)((size_t)(p) + dEFFICIENT_SIZE(b)))

/* alloca aligned to the EFFICIENT_ALIGNMENT. note that this can waste
 * up to 15 bytes per allocation, depending on what alloca() returns.
 */

#define dALLOCA16(n)                                                           \
  ((char*)dEFFICIENT_PTR(alloca((n) + (EFFICIENT_ALIGNMENT - 1))))

// misc defines
#define ALLOCA dALLOCA16

namespace dart::math {

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
  static Scalar reciprocalSqrt(Scalar x);

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

  static float sqrt(float x)
  {
    return std::sqrt(x);
  }

  static float abs(float x)
  {
    return std::fabs(x);
  }

  static float reciprocal(float x)
  {
    return 1.0f / x;
  }

  static float reciprocalSqrt(float x)
  {
    return 1.0f / std::sqrt(x);
  }

  static float sin(float x)
  {
    return std::sin(x);
  }

  static float cos(float x)
  {
    return std::cos(x);
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

  static double sqrt(double x)
  {
    return std::sqrt(x);
  }

  static double abs(double x)
  {
    return std::fabs(x);
  }

  static double reciprocal(double x)
  {
    return 1.0 / x;
  }

  static double reciprocalSqrt(double x)
  {
    return 1.0 / std::sqrt(x);
  }

  static double sin(double x)
  {
    return std::sin(x);
  }

  static double cos(double x)
  {
    return std::cos(x);
  }
};

/// Reciprocal (1/x) - template function for type-safe division
template <typename S>
inline constexpr S reciprocal(S x)
{
  return S(1) / x;
}

//==============================================================================
// Matrix Multiplication Functions (Phase 14.3 - SIMD Optimization)
//==============================================================================

// Hybrid threshold for switching between raw pointer and Eigen SIMD
// Based on benchmark results showing Eigen advantage at size >= 12
// Threshold tuning (Phase 15.3): Tested 8, 10, 12, 16 → 10 is optimal
constexpr int MATRIX_MULTIPLY_SIMD_THRESHOLD = 10;

/// Matrix multiplication: A = B * C
/// Sizes: A:p×r B:p×q C:q×r (all row-major)
/// Uses hybrid approach: raw pointers for small matrices, Eigen SIMD for larger
template <typename Scalar>
inline void Multiply0(
    Scalar* A, const Scalar* B, const Scalar* C, int p, int q, int r)
{
  DART_ASSERT(p > 0 && q > 0 && r > 0 && A && B && C);

  // Use Eigen SIMD for medium/large matrices
  if (q >= MATRIX_MULTIPLY_SIMD_THRESHOLD
      && r >= MATRIX_MULTIPLY_SIMD_THRESHOLD) {
    using Matrix = Eigen::
        Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;
    Eigen::Map<Matrix> mapA(A, p, r);
    Eigen::Map<const Matrix> mapB(B, p, q);
    Eigen::Map<const Matrix> mapC(C, q, r);
    mapA.noalias() = mapB * mapC;
  } else {
    // Raw pointer implementation for small matrices
    for (int i = 0; i < p; ++i) {
      for (int j = 0; j < r; ++j) {
        Scalar sum = Scalar(0);
        for (int k = 0; k < q; ++k) {
          sum += B[i * q + k] * C[k * r + j];
        }
        A[i * r + j] = sum;
      }
    }
  }
}

/// Matrix multiplication: A = B' * C (B transposed)
/// Sizes: A:p×r B:q×p C:q×r (all row-major, but B is logically transposed)
template <typename Scalar>
inline void Multiply1(
    Scalar* A, const Scalar* B, const Scalar* C, int p, int q, int r)
{
  DART_ASSERT(p > 0 && q > 0 && r > 0 && A && B && C);

  if (q >= MATRIX_MULTIPLY_SIMD_THRESHOLD
      && r >= MATRIX_MULTIPLY_SIMD_THRESHOLD) {
    using Matrix = Eigen::
        Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;
    Eigen::Map<Matrix> mapA(A, p, r);
    Eigen::Map<const Matrix> mapB(B, q, p); // Note: q×p for transpose
    Eigen::Map<const Matrix> mapC(C, q, r);
    mapA.noalias() = mapB.transpose() * mapC;
  } else {
    // Raw pointer: B transposed access
    for (int i = 0; i < p; ++i) {
      for (int j = 0; j < r; ++j) {
        Scalar sum = Scalar(0);
        for (int k = 0; k < q; ++k) {
          sum += B[k * p + i] * C[k * r + j]; // B[k,i] transposed
        }
        A[i * r + j] = sum;
      }
    }
  }
}

/// Matrix multiplication: A = B * C' (C transposed)
/// Sizes: A:p×r B:p×q C:r×q (all row-major, but C is logically transposed)
template <typename Scalar>
inline void Multiply2(
    Scalar* A, const Scalar* B, const Scalar* C, int p, int q, int r)
{
  DART_ASSERT(p > 0 && q > 0 && r > 0 && A && B && C);

  if (q >= MATRIX_MULTIPLY_SIMD_THRESHOLD
      && r >= MATRIX_MULTIPLY_SIMD_THRESHOLD) {
    using Matrix = Eigen::
        Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;
    Eigen::Map<Matrix> mapA(A, p, r);
    Eigen::Map<const Matrix> mapB(B, p, q);
    Eigen::Map<const Matrix> mapC(C, r, q); // Note: r×q for transpose
    mapA.noalias() = mapB * mapC.transpose();
  } else {
    // Raw pointer: C transposed access
    for (int i = 0; i < p; ++i) {
      for (int j = 0; j < r; ++j) {
        Scalar sum = Scalar(0);
        for (int k = 0; k < q; ++k) {
          sum += B[i * q + k] * C[j * q + k]; // C[j,k] transposed
        }
        A[i * r + j] = sum;
      }
    }
  }
}

} // namespace dart::math
