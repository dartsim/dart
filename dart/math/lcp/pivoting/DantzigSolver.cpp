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

#include "dart/math/lcp/pivoting/DantzigSolver.hpp"

#include "dart/math/lcp/pivoting/dantzig/Common.hpp"
#include "dart/math/lcp/pivoting/dantzig/Lcp.hpp"

#include <limits>
#include <vector>

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

  if (A.rows() != A.cols() || A.rows() != b.size() || lo.size() != b.size()
      || hi.size() != b.size() || findex.size() != b.size()) {
    result.status = LcpSolverStatus::InvalidProblem;
    result.message = "Matrix dimensions inconsistent";
    return result;
  }

  const int n = static_cast<int>(b.size());
  const int nSkip = padding(n);

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
    xdata[i] = options.warmStart ? x[i] : 0.0;
    for (int j = 0; j < n; ++j) {
      Adata[i * nSkip + j] = A(i, j);
    }
  }

  const bool success = SolveLCP<double>(
      n,
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
  result.complementarity = (x.array() * w.array()).abs().maxCoeff();
  result.residual = (w.array().min(Eigen::ArrayXd::Zero(n)))
                        .matrix()
                        .lpNorm<Eigen::Infinity>();
  result.status = success ? LcpSolverStatus::Success : LcpSolverStatus::Failed;

  if (options.validateSolution) {
    const double tol
        = std::max(options.complementarityTolerance, options.absoluteTolerance);
    const bool feasible = w.minCoeff() >= -options.absoluteTolerance
                          && (x - lo).minCoeff() >= -options.absoluteTolerance
                          && (hi - x).minCoeff() >= -options.absoluteTolerance
                          && (x.array() * w.array()).abs().maxCoeff() <= tol;
    result.validated = true;
    if (!feasible)
      result.status = LcpSolverStatus::NumericalError;
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
