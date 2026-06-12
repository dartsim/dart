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

#ifndef DART_MATH_NONNEGATIVELEASTSQUARES_HPP_
#define DART_MATH_NONNEGATIVELEASTSQUARES_HPP_

#include <Eigen/Core>

#include <cstddef>

namespace dart {
namespace math {

/// Solves the nonnegative least squares problem
///
///   minimize    || A * x - b ||^2
///   subject to  x >= 0
///
/// using the Lawson-Hanson active-set method. The problem always has a
/// minimizer; for rank-deficient \c A the minimizer may not be unique, in
/// which case the solver deterministically returns one of them.
///
/// \param[in] A Matrix of size (m x n). Must contain only finite values.
/// \param[in] b Vector of size m. Must contain only finite values.
/// \param[out] x Solution vector of size n. Resized as needed. Set to zero
/// on invalid inputs; when the iteration cap is exhausted it contains the
/// last (nonnegative) iterate instead.
/// \param[in] tolerance Nonnegativity/optimality tolerance on the dual
/// vector. A non-positive value selects an automatic tolerance scaled by the
/// magnitudes of \c A and \c b with an absolute floor of 1 on the \c b
/// factor; callers that need relative accuracy for very small \c b should
/// pass an explicit tolerance.
/// \param[in] maxIterations Iteration cap for the active-set loop. Zero
/// selects an automatic cap proportional to the number of columns.
/// \return True when the optimality conditions were met within the iteration
/// cap; false on invalid inputs or when the iteration cap was exhausted.
bool solveNonNegativeLeastSquares(
    const Eigen::MatrixXd& A,
    const Eigen::VectorXd& b,
    Eigen::VectorXd& x,
    double tolerance = -1.0,
    std::size_t maxIterations = 0);

} // namespace math
} // namespace dart

#endif // DART_MATH_NONNEGATIVELEASTSQUARES_HPP_
