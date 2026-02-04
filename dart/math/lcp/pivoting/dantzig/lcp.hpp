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

#include "dart/export.hpp"
#include "dart/math/lcp/pivoting/dantzig/common.hpp"

#include <Eigen/Core>

#include <cstdio>
#include <cstdlib>

namespace dart::math {

/// Solve the Linear Complementarity Problem using Dantzig's algorithm
///
/// Given (A,b,lo,hi), solve the LCP problem: A*x = b+w, where each x(i),w(i)
/// satisfies one of:
///   (1) x = lo, w >= 0
///   (2) x = hi, w <= 0
///   (3) lo < x < hi, w = 0
///
/// @tparam Scalar Floating-point type (float or double)
/// @param n Dimension of the problem (A is n×n matrix)
/// @param A Coefficient matrix of dimension n×n (may be modified)
/// @param x Solution vector (output)
/// @param b Right-hand side vector (may be modified)
/// @param w Complementarity vector (output, can be nullptr to skip)
/// @param nub Number of unbounded variables (first nub variables have infinite
/// bounds)
/// @param lo Lower bounds (must satisfy lo(i) <= 0)
/// @param hi Upper bounds (must satisfy hi(i) >= 0)
/// @param findex Friction index array (nullptr if not used). When findex[i] >=
/// 0,
///               the constraint is "special" and bounds are updated:
///               hi[i] = abs(hi[i] * x[findex[i]]), lo[i] = -hi[i]
/// @param earlyTermination If true, solver may terminate early (default: false)
/// @return True if solution found, false otherwise
///
/// @note lo and hi can be +/- infinity (ScalarTraits<Scalar>::inf()) as needed
/// @note The first nub variables are unbounded (hi and lo assumed to be +/-
///       infinity)
/// @note For friction approximation, the first nub variables must have findex <
///       0
///
/// @code
/// // Example usage:
/// constexpr int n = 6;
/// double A[n*n], x[n], b[n], w[n], lo[n], hi[n];
/// // ... initialize A, b, lo, hi ...
/// bool success = SolveLCP<double>(n, A, x, b, w, 0, lo, hi, nullptr);
/// @endcode
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

} // namespace dart::math

// Template implementations for header-only usage
#include "dart/math/lcp/pivoting/dantzig/lcp-impl.hpp"
