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

#include "dart/constraint/BoxedLcpSolver.hpp"

#include "dart/math/lcp/dantzig/Common.hpp"

#include <algorithm>
#include <vector>

namespace dart::constraint {

//==============================================================================
math::LcpOptions BoxedLcpSolver::getDefaultOptions() const
{
  return mDefaultOptions;
}

//==============================================================================
void BoxedLcpSolver::setDefaultOptions(const math::LcpOptions& options)
{
  mDefaultOptions = options;
}

//==============================================================================
math::LcpResult BoxedLcpSolver::solve(
    const Eigen::MatrixXd& A,
    const Eigen::VectorXd& b,
    const Eigen::VectorXd& lo,
    const Eigen::VectorXd& hi,
    const Eigen::VectorXi& findex,
    Eigen::VectorXd& x,
    const math::LcpOptions& options)
{
  math::LcpResult result;

  const bool dimensionMismatch = (A.rows() != A.cols()) || (A.rows() != b.size())
                                 || (lo.size() != b.size())
                                 || (hi.size() != b.size())
                                 || (findex.size() != b.size());
  if (dimensionMismatch) {
    result.status = math::LcpSolverStatus::InvalidProblem;
    result.message = "Matrix/vector dimensions inconsistent";
    return result;
  }

  const int n = static_cast<int>(b.size());
  const int nSkip = math::padding(n);

  std::vector<double> Adata(static_cast<std::size_t>(n * nSkip), 0.0);
  std::vector<double> xdata(static_cast<std::size_t>(n), 0.0);
  std::vector<double> bdata(static_cast<std::size_t>(n), 0.0);
  std::vector<double> loData(static_cast<std::size_t>(n), 0.0);
  std::vector<double> hiData(static_cast<std::size_t>(n), 0.0);
  std::vector<int> findexData(static_cast<std::size_t>(n), -1);

  for (int r = 0; r < n; ++r) {
    xdata[static_cast<std::size_t>(r)] = options.warmStart ? x[r] : 0.0;
    bdata[static_cast<std::size_t>(r)] = b[r];
    loData[static_cast<std::size_t>(r)] = lo[r];
    hiData[static_cast<std::size_t>(r)] = hi[r];
    findexData[static_cast<std::size_t>(r)] = findex[r];
    for (int c = 0; c < n; ++c) {
      Adata[static_cast<std::size_t>(r * nSkip + c)] = A(r, c);
    }
  }

  const bool success = solve(
      n,
      Adata.data(),
      xdata.data(),
      bdata.data(),
      0,
      loData.data(),
      hiData.data(),
      findexData.data(),
      options.earlyTermination);

  x = Eigen::Map<Eigen::VectorXd>(xdata.data(), n);

  Eigen::VectorXd w = A * x + b;
  result.iterations = 1;
  result.complementarity = (x.array() * w.array()).abs().maxCoeff();
  result.residual
      = (w.array().min(Eigen::ArrayXd::Zero(n))).matrix().lpNorm<Eigen::Infinity>();
  result.status = (success && !x.hasNaN()) ? math::LcpSolverStatus::Success
                                           : math::LcpSolverStatus::Failed;

  if (options.validateSolution) {
    const double tol = std::max(
        options.complementarityTolerance, options.absoluteTolerance);
    bool valid = true;

    if (w.minCoeff() < -options.absoluteTolerance)
      valid = false;
    if ((x.array() - lo.array()).minCoeff() < -options.absoluteTolerance)
      valid = false;
    if ((hi.array() - x.array()).minCoeff() < -options.absoluteTolerance)
      valid = false;
    if ((x.array() * w.array()).abs().maxCoeff() > tol)
      valid = false;

    result.validated = true;
    if (!valid) {
      result.status = math::LcpSolverStatus::NumericalError;
      result.message = "Solution validation failed";
    }
  }

  return result;
}

} // namespace dart::constraint
