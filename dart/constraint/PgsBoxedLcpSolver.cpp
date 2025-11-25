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

#include <cmath>
#include <cstring>
#include <vector>

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

  const bool success = solveLegacy(
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

  // Restore the original PGS option so per-call overrides don't leak.
  setOption(previousOption);

  Eigen::VectorXd w = A * x + b;
  result.iterations = appliedOption.mMaxIteration;
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

//==============================================================================
bool PgsBoxedLcpSolver::solveLegacy(
    int n,
    double* A,
    double* x,
    double* b,
    int nub,
    double* lo,
    double* hi,
    int* findex,
    bool /*earlyTermination*/)
{
  const int nskip = math::padding(n);

  // If all the variables are unbounded then we can just factor, solve, and
  // return.R
  if (nub >= n) {
    mCacheD.resize(n);
    std::fill(mCacheD.begin(), mCacheD.end(), 0);

    math::dFactorLDLT(A, mCacheD.data(), n, nskip);
    math::dSolveLDLT(A, mCacheD.data(), b, n, nskip);
    std::memcpy(x, b, n * sizeof(double));

    return true;
  }

  mCacheOrder.clear();
  mCacheOrder.reserve(n);

  bool possibleToTerminate = true;
  for (int i = 0; i < n; ++i) {
    // mOrderCacheing
    if (A[nskip * i + i] < mOption.mEpsilonForDivision) {
      x[i] = 0.0;
      continue;
    }

    mCacheOrder.push_back(i);

    // Initial loop
    const double* A_ptr = A + nskip * i;
    const double old_x = x[i];

    double new_x = b[i];

    for (int j = 0; j < i; ++j)
      new_x -= A_ptr[j] * x[j];

    for (int j = i + 1; j < n; ++j)
      new_x -= A_ptr[j] * x[j];

    new_x /= A[nskip * i + i];

    if (findex[i] >= 0) {
      const double hi_tmp = hi[i] * x[findex[i]];
      const double lo_tmp = -hi_tmp;

      if (new_x > hi_tmp)
        x[i] = hi_tmp;
      else if (new_x < lo_tmp)
        x[i] = lo_tmp;
      else
        x[i] = new_x;
    } else {
      if (new_x > hi[i])
        x[i] = hi[i];
      else if (new_x < lo[i])
        x[i] = lo[i];
      else
        x[i] = new_x;
    }

    // Test
    if (possibleToTerminate) {
      const double deltaX = std::abs(x[i] - old_x);
      if (deltaX > mOption.mDeltaXThreshold)
        possibleToTerminate = false;
    }
  }

  if (possibleToTerminate) {
    return true;
  }

  // Normalizing
  for (const auto& index : mCacheOrder) {
    const double dummy = 1.0 / A[nskip * index + index];
    b[index] *= dummy;
    for (int j = 0; j < n; ++j)
      A[nskip * index + j] *= dummy;
  }

  for (int iter = 1; iter < mOption.mMaxIteration; ++iter) {
    if (mOption.mRandomizeConstraintOrder) {
      if ((iter & 7) == 0) {
        for (std::size_t i = 1; i < mCacheOrder.size(); ++i) {
          const int tmp = mCacheOrder[i];
          const int swapi = math::dRandInt(i + 1);
          mCacheOrder[i] = mCacheOrder[swapi];
          mCacheOrder[swapi] = tmp;
        }
      }
    }

    possibleToTerminate = true;

    // Single loop
    for (const auto& index : mCacheOrder) {
      const double* A_ptr = A + nskip * index;
      double new_x = b[index];
      const double old_x = x[index];

      for (int j = 0; j < index; j++)
        new_x -= A_ptr[j] * x[j];

      for (int j = index + 1; j < n; j++)
        new_x -= A_ptr[j] * x[j];

      if (findex[index] >= 0) {
        const double hi_tmp = hi[index] * x[findex[index]];
        const double lo_tmp = -hi_tmp;

        if (new_x > hi_tmp)
          x[index] = hi_tmp;
        else if (new_x < lo_tmp)
          x[index] = lo_tmp;
        else
          x[index] = new_x;
      } else {
        if (new_x > hi[index])
          x[index] = hi[index];
        else if (new_x < lo[index])
          x[index] = lo[index];
        else
          x[index] = new_x;
      }

      if (possibleToTerminate
          && std::abs(x[index]) > mOption.mEpsilonForDivision) {
        const double relativeDeltaX = std::abs((x[index] - old_x) / x[index]);
        if (relativeDeltaX > mOption.mRelativeDeltaXTolerance)
          possibleToTerminate = false;
      }
    }

    if (possibleToTerminate)
      break;
  }

  return possibleToTerminate;
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
