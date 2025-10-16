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

/// @file lcp.h
/// @brief Linear Complementarity Problem (LCP) solver using Dantzig's algorithm

#pragma once

#include "dart/lcpsolver/dantzig/common.h"

#include <Eigen/Core>

#include <cstdio>
#include <cstdlib>

namespace dart {
namespace lcpsolver {

/// Solve the Linear Complementarity Problem using Dantzig's algorithm
///
/// Given (A,b,lo,hi), solve the LCP problem: A*x = b+w, where each x(i),w(i)
/// satisfies one of:
///   (1) x = lo, w >= 0
///   (2) x = hi, w <= 0
///   (3) lo < x < hi, w = 0
///
/// @param n Dimension of the problem (A is n×n matrix)
/// @param A Coefficient matrix of dimension n×n (may be modified)
/// @param x Solution vector (output)
/// @param b Right-hand side vector
/// @param w Complementarity vector (output)
/// @param nub Number of unbounded variables (first nub variables have infinite
/// bounds)
/// @param lo Lower bounds (must satisfy lo(i) <= 0)
/// @param hi Upper bounds (must satisfy hi(i) >= 0)
/// @param findex Friction index array (nullptr if not used). When findex[i] >=
/// 0,
///               the constraint is "special" and bounds are updated:
///               hi[i] = abs(hi[i] * x[findex[i]]), lo[i] = -hi[i]
/// @param earlyTermination If true, solver may terminate early
/// @return True if solution found, false otherwise
///
/// @note lo and hi can be +/- dInfinity as needed
/// @note The first nub variables are unbounded (hi and lo assumed to be +/-
/// dInfinity)
/// @note The original data (A,b) may be modified by this function
/// @note For friction approximation, the first nub variables must have findex <
/// 0
template <typename Scalar>
bool SolveLCP(
    int n,
    Scalar* A,
    Scalar* x,
    Scalar* b,
    Scalar* w,
    int nub,
    Scalar* lo,
    Scalar* hi,
    int* findex,
    bool earlyTermination = false);

/// Backward compatible LCP solver using dReal (double precision)
/// @see SolveLCP for detailed documentation
// TODO: Remove in favor of template <typename Scalar> SolveLCP
bool dSolveLCP(
    int n,
    dReal* A,
    dReal* x,
    dReal* b,
    dReal* outer_w,
    int nub,
    dReal* lo,
    dReal* hi,
    int* findex,
    bool earlyTermination);

// Explicit template instantiation for float only
// Note: double has a full specialization in lcp.cpp
extern template bool SolveLCP<float>(
    int n,
    float* A,
    float* x,
    float* b,
    float* w,
    int nub,
    float* lo,
    float* hi,
    int* findex,
    bool earlyTermination);

//==============================================================================
// Eigen API - Modern C++ interface
//==============================================================================

/// Solve LCP with Eigen types (Matrix version)
///
/// This is a modern C++ interface that accepts Eigen matrices and vectors
/// directly, providing type safety and convenience.
///
/// @param A Coefficient matrix (n×n), will be modified
/// @param x Solution vector (output)
/// @param b Right-hand side vector
/// @param w Complementarity vector (output, can be nullptr)
/// @param lo Lower bounds vector (must satisfy lo(i) <= 0)
/// @param hi Upper bounds vector (must satisfy hi(i) >= 0)
/// @param nub Number of unbounded variables (default: 0)
/// @param findex Friction index array (nullptr if not used)
/// @param earlyTermination Allow early termination (default: false)
/// @return True if solution found, false otherwise
///
/// @note This function extracts raw pointers from Eigen types and calls the
///       pointer-based SolveLCP implementation
template <typename Derived1, typename Derived2, typename Derived3>
inline bool SolveLCP(
    Eigen::MatrixBase<Derived1>& A,
    Eigen::MatrixBase<Derived2>& x,
    const Eigen::MatrixBase<Derived3>& b,
    Eigen::MatrixBase<Derived2>* w,
    const Eigen::MatrixBase<Derived2>& lo,
    const Eigen::MatrixBase<Derived2>& hi,
    int nub = 0,
    int* findex = nullptr,
    bool earlyTermination = false)
{
  using Scalar = typename Derived2::Scalar;

  DART_ASSERT(A.rows() == A.cols());
  DART_ASSERT(x.size() == A.rows());
  DART_ASSERT(b.size() == A.rows());
  DART_ASSERT(lo.size() == A.rows());
  DART_ASSERT(hi.size() == A.rows());
  DART_ASSERT(!w || w->size() == A.rows());

  const int n = static_cast<int>(A.rows());

  // Make a copy of b since the pointer version may modify it
  Eigen::Matrix<Scalar, Eigen::Dynamic, 1> b_copy = b;

  return SolveLCP<Scalar>(
      n,
      A.derived().data(),
      x.derived().data(),
      b_copy.data(),
      w ? w->derived().data() : nullptr,
      nub,
      const_cast<Scalar*>(lo.derived().data()),
      const_cast<Scalar*>(hi.derived().data()),
      findex,
      earlyTermination);
}

/// Solve LCP with Eigen types (no w output version)
///
/// Convenience overload that doesn't require a w output parameter
template <typename Derived1, typename Derived2, typename Derived3>
inline bool SolveLCP(
    Eigen::MatrixBase<Derived1>& A,
    Eigen::MatrixBase<Derived2>& x,
    const Eigen::MatrixBase<Derived3>& b,
    const Eigen::MatrixBase<Derived2>& lo,
    const Eigen::MatrixBase<Derived2>& hi,
    int nub = 0,
    int* findex = nullptr,
    bool earlyTermination = false)
{
  return SolveLCP(A, x, b, nullptr, lo, hi, nub, findex, earlyTermination);
}

} // namespace lcpsolver
} // namespace dart
