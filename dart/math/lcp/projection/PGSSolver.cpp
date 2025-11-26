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

#include "dart/math/lcp/projection/PGSSolver.hpp"

#include "dart/math/lcp/dantzig/Common.hpp"

#include <limits>

namespace dart::math {

PGSSolver::PGSSolver()
{
  mDefaultOptions.maxIterations = 30;
  mDefaultOptions.absoluteTolerance = 1e-6;
  mDefaultOptions.relativeTolerance = 1e-3;
  mDefaultOptions.validateSolution = true;
}

//==============================================================================
LcpResult PGSSolver::solve(
    const Eigen::MatrixXd& A,
    const Eigen::VectorXd& b,
    Eigen::VectorXd& x,
    const LcpOptions& options)
{
  LcpResult result;

  if (A.rows() != A.cols() || A.rows() != b.size()) {
    result.status = LcpSolverStatus::InvalidProblem;
    result.message = "Matrix dimensions inconsistent";
    return result;
  }

  const int n = static_cast<int>(b.size());
  Eigen::VectorXd currentX = options.warmStart ? x : Eigen::VectorXd::Zero(n);
  Eigen::VectorXd w = Eigen::VectorXd::Zero(n);

  bool converged = true;
  for (int iter = 0;
       iter
       < (options.maxIterations > 0 ? options.maxIterations
                                    : mDefaultOptions.maxIterations);
       ++iter) {
    converged = true;
    for (int i = 0; i < n; ++i) {
      double newX = b[i];
      for (int j = 0; j < n; ++j) {
        if (j == i)
          continue;
        newX += A(i, j) * currentX[j];
      }
      newX /= A(i, i);
      const double clampedX = std::max(0.0, newX);
      if (std::abs(clampedX - currentX[i]) > options.absoluteTolerance)
        converged = false;
      currentX[i] = clampedX;
    }
    if (converged)
      break;
  }

  x = currentX;
  w = A * x + b;

  result.iterations = options.maxIterations > 0 ? options.maxIterations
                                                : mDefaultOptions.maxIterations;
  result.complementarity = (x.array() * w.array()).abs().maxCoeff();
  result.residual = (w.array().min(Eigen::ArrayXd::Zero(n)))
                        .matrix()
                        .lpNorm<Eigen::Infinity>();
  result.status
      = converged ? LcpSolverStatus::Success : LcpSolverStatus::Failed;

  if (options.validateSolution) {
    const double tol
        = std::max(options.complementarityTolerance, options.absoluteTolerance);
    const bool feasible = w.minCoeff() >= -options.absoluteTolerance
                          && x.minCoeff() >= -options.absoluteTolerance
                          && (x.array() * w.array()).abs().maxCoeff() <= tol;
    result.validated = true;
    if (!feasible)
      result.status = LcpSolverStatus::NumericalError;
  }

  return result;
}

//==============================================================================
std::string PGSSolver::getName() const
{
  return "PGS";
}

//==============================================================================
std::string PGSSolver::getCategory() const
{
  return "Projection";
}

} // namespace dart::math
