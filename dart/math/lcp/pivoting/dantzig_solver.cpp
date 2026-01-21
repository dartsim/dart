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

#include "dart/math/lcp/pivoting/dantzig_solver.hpp"

#include "dart/math/lcp/lcp_validation.hpp"
#include "dart/math/lcp/pivoting/dantzig/common.hpp"
#include "dart/math/lcp/pivoting/dantzig/lcp.hpp"

#include <algorithm>
#include <iterator>
#include <limits>
#include <vector>

#include <cmath>

namespace dart::math {

//==============================================================================
DantzigSolver::DantzigSolver()
{
  mDefaultOptions.warmStart = true;
}

//==============================================================================
LcpResult DantzigSolver::solve(
    const LcpProblem& problem, Eigen::VectorXd& x, const LcpOptions& options)
{
  LcpResult result;

  const auto& A = problem.A;
  const auto& b = problem.b;
  const auto& lo = problem.lo;
  const auto& hi = problem.hi;
  const auto& findex = problem.findex;

  std::string problemMessage;
  if (!detail::validateProblem(problem, &problemMessage)) {
    result.status = LcpSolverStatus::InvalidProblem;
    result.message = problemMessage;
    return result;
  }

  const auto n = std::ssize(b);
  if (n == 0) {
    x.resize(0);
    result.status = LcpSolverStatus::Success;
    result.iterations = 0;
    result.residual = 0.0;
    result.complementarity = 0.0;
    result.validated = options.validateSolution;
    return result;
  }

  if (x.size() != n)
    x = Eigen::VectorXd::Zero(n);
  const int nSkip = padding(static_cast<int>(n));

  std::vector<double> Adata(n * nSkip, 0.0);
  std::vector<double> xdata(n, 0.0);
  std::vector<double> wdata(n, 0.0);
  std::vector<double> bdata(n, 0.0);
  std::vector<double> loData(n, 0.0);
  std::vector<double> hiData(n, 0.0);
  std::vector<int> findexData(n, -1);

  for (int i = 0; i < n; ++i) {
    bdata[i] = b[i];
    loData[i] = lo[i];
    hiData[i] = hi[i];
    findexData[i] = findex[i];
    xdata[i] = (options.warmStart && std::isfinite(x[i])) ? x[i] : 0.0;
    for (int j = 0; j < n; ++j) {
      Adata[i * nSkip + j] = A(i, j);
    }
  }

  const bool success = SolveLCP<double>(
      static_cast<int>(n),
      Adata.data(),
      xdata.data(),
      bdata.data(),
      wdata.data(),
      0,
      loData.data(),
      hiData.data(),
      findexData.data(),
      options.earlyTermination);

  x = Eigen::VectorXd::Map(xdata.data(), n);
  const Eigen::VectorXd w = A * x - b;
  result.iterations = 1;

  Eigen::VectorXd loEff;
  Eigen::VectorXd hiEff;
  std::string boundsMessage;
  const bool boundsOk = detail::computeEffectiveBounds(
      lo, hi, findex, x, loEff, hiEff, &boundsMessage);
  if (!boundsOk) {
    result.status = LcpSolverStatus::InvalidProblem;
    result.message = boundsMessage;
    return result;
  }

  const double absTol = (options.absoluteTolerance > 0)
                            ? options.absoluteTolerance
                            : mDefaultOptions.absoluteTolerance;
  result.residual = detail::naturalResidualInfinityNorm(x, w, loEff, hiEff);
  result.complementarity
      = detail::complementarityInfinityNorm(x, w, loEff, hiEff, absTol);
  result.status = success ? LcpSolverStatus::Success : LcpSolverStatus::Failed;

  if (options.validateSolution && result.status == LcpSolverStatus::Success) {
    const double compTol = (options.complementarityTolerance > 0)
                               ? options.complementarityTolerance
                               : mDefaultOptions.complementarityTolerance;
    const double validationTol = std::max(absTol, compTol);
    std::string validationMessage;
    const bool feasible = detail::validateSolution(
        x, w, loEff, hiEff, validationTol, &validationMessage);
    result.validated = true;
    if (!feasible) {
      result.status = LcpSolverStatus::NumericalError;
      result.message = validationMessage;
    }
  }

  return result;
}

//==============================================================================
std::string DantzigSolver::getName() const
{
  return "Dantzig";
}

//==============================================================================
std::string DantzigSolver::getCategory() const
{
  return "Pivoting";
}

} // namespace dart::math
