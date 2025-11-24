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

#include "dart/math/lcp/pivoting/LemkeSolver.hpp"

#include "dart/math/lcp/Lemke.hpp"

namespace dart {
namespace math {

//==============================================================================
LemkeSolver::LemkeSolver()
{
  // Lemke is an exact solver, no need for iterations or tolerances
  mDefaultOptions.maxIterations = 0;       // Not applicable
  mDefaultOptions.validateSolution = true; // Always validate
}

//==============================================================================
LcpResult LemkeSolver::solve(
    const Eigen::MatrixXd& A,
    const Eigen::VectorXd& b,
    Eigen::VectorXd& x,
    const LcpOptions& options)
{
  LcpResult result;

  // Check problem dimensions
  if (A.rows() != A.cols() || A.rows() != b.size()) {
    result.status = LcpSolverStatus::InvalidProblem;
    result.message = "Matrix dimensions inconsistent";
    return result;
  }

  // Call the legacy Lemke solver
  const int exitCode = Lemke(A, b, &x);

  // Interpret exit code
  result.iterations = 1; // Pivoting methods don't have iterations in same sense
  if (exitCode == 0) {
    result.status = LcpSolverStatus::Success;

    // Validate solution if requested
    if (options.validateSolution) {
      const bool isValid = validate(A, x, b);
      result.validated = true;
      if (!isValid) {
        result.status = LcpSolverStatus::NumericalError;
        result.message = "Solution validation failed";
      }
    }

    // Compute complementarity error
    Eigen::VectorXd w = A * x + b;
    result.complementarity = (x.array() * w.array()).abs().sum();
    result.residual = (A * x - (-b)).norm();
  } else {
    result.status = LcpSolverStatus::Failed;
    result.message = "Lemke algorithm failed (no solution found)";
  }

  return result;
}

//==============================================================================
std::string LemkeSolver::getName() const
{
  return "Lemke";
}

//==============================================================================
std::string LemkeSolver::getCategory() const
{
  return "Pivoting";
}

} // namespace math
} // namespace dart
