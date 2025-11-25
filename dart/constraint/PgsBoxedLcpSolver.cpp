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

#include "dart/constraint/PgsBoxedLcpSolver.hpp"

#include "dart/math/Constants.hpp"
#include "dart/math/lcp/dantzig/Common.hpp"
#include "dart/math/lcp/dantzig/Matrix.hpp"
#include "dart/math/lcp/dantzig/Misc.hpp"

#include <Eigen/Dense>

#include <vector>

#include <cmath>
#include <cstring>

#define PGS_EPSILON 10e-9

namespace dart {
namespace constraint {

//==============================================================================
PgsBoxedLcpSolver::Option::Option(
    int maxIteration,
    double deltaXTolerance,
    double relativeDeltaXTolerance,
    double epsilonForDivision,
    bool randomizeConstraintOrder)
  : mMaxIteration(maxIteration),
    mDeltaXThreshold(deltaXTolerance),
    mRelativeDeltaXTolerance(relativeDeltaXTolerance),
    mEpsilonForDivision(epsilonForDivision),
    mRandomizeConstraintOrder(randomizeConstraintOrder)
{
  // Do nothing
}

//==============================================================================
const std::string& PgsBoxedLcpSolver::getType() const
{
  return getStaticType();
}

//==============================================================================
const std::string& PgsBoxedLcpSolver::getStaticType()
{
  static const std::string type = "PgsBoxedLcpSolver";
  return type;
}

//==============================================================================
math::LcpResult PgsBoxedLcpSolver::solve(
    const Eigen::MatrixXd& A,
    const Eigen::VectorXd& b,
    const Eigen::VectorXd& lo,
    const Eigen::VectorXd& hi,
    const Eigen::VectorXi& findex,
    Eigen::VectorXd& x,
    const math::LcpOptions& options)
{
  math::LcpResult result;

  const bool dimensionMismatch
      = (A.rows() != A.cols()) || (A.rows() != b.size())
        || (lo.size() != b.size()) || (hi.size() != b.size())
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

  Option previousOption = mOption;
  Option appliedOption = mOption;
  if (options.maxIterations > 0)
    appliedOption.mMaxIteration = options.maxIterations;
  if (options.absoluteTolerance > 0)
    appliedOption.mDeltaXThreshold = options.absoluteTolerance;
  if (options.relativeTolerance > 0)
    appliedOption.mRelativeDeltaXTolerance = options.relativeTolerance;

  setOption(appliedOption);

  for (int r = 0; r < n; ++r) {
    bdata[static_cast<std::size_t>(r)] = b[r];
    xdata[static_cast<std::size_t>(r)] = options.warmStart ? x[r] : 0.0;
    loData[static_cast<std::size_t>(r)] = lo[r];
    hiData[static_cast<std::size_t>(r)] = hi[r];
    findexData[static_cast<std::size_t>(r)] = findex[r];
    for (int c = 0; c < n; ++c) {
      Adata[static_cast<std::size_t>(r * nSkip + c)] = A(r, c);
    }
  }

  const int nub = 0;

  // Allocate w vector for LCP solver
  std::vector<double> w(static_cast<std::size_t>(n), 0.0);

  const bool success = math::SolveLCP<double>(
      n,
      Adata.data(),
      xdata.data(),
      bdata.data(),
      w.data(),
      nub,
      loData.data(),
      hiData.data(),
      findexData.data(),
      options.earlyTermination);

  x = Eigen::Map<Eigen::VectorXd>(xdata.data(), n);

  // Restore the original PGS option so per-call overrides don't leak.
  setOption(previousOption);

  Eigen::VectorXd w = A * x + b;
  result.iterations = appliedOption.mMaxIteration;
  result.complementarity = (x.array() * w.array()).abs().maxCoeff();
  result.residual = (w.array().min(Eigen::ArrayXd::Zero(n)))
                        .matrix()
                        .lpNorm<Eigen::Infinity>();
  result.status = (success && !x.hasNaN()) ? math::LcpSolverStatus::Success
                                           : math::LcpSolverStatus::Failed;

  if (options.validateSolution) {
    const double tol
        = std::max(options.complementarityTolerance, options.absoluteTolerance);
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

#if DART_BUILD_MODE_DEBUG
//==============================================================================
bool PgsBoxedLcpSolver::canSolve(const Eigen::MatrixXd& A)
{
  const int n = static_cast<int>(A.rows());

  // Return false if A has zero-diagonal or A is nonsymmetric matrix
  for (auto i = 0; i < n; ++i) {
    if (A(i, i) < PGS_EPSILON)
      return false;

    for (auto j = 0; j < n; ++j) {
      if (std::abs(A(i, j) - A(j, i)) > PGS_EPSILON)
        return false;
    }
  }

  return true;
}
#endif

//==============================================================================
void PgsBoxedLcpSolver::setOption(const PgsBoxedLcpSolver::Option& option)
{
  mOption = option;
}

//==============================================================================
const PgsBoxedLcpSolver::Option& PgsBoxedLcpSolver::getOption() const
{
  return mOption;
}

} // namespace constraint
} // namespace dart
