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

  // Projected Gauss-Seidel iteration
  mCacheOrder.clear();
  mCacheOrder.reserve(n);

  bool possibleToTerminate = true;
  for (int i = 0; i < n; ++i) {
    if (Adata[nSkip * i + i] < mOption.mEpsilonForDivision) {
      xdata[static_cast<std::size_t>(i)] = 0.0;
      continue;
    }

    mCacheOrder.push_back(i);

    const double* APtr = Adata.data() + nSkip * i;
    const double oldX = xdata[static_cast<std::size_t>(i)];

    double newX = bdata[static_cast<std::size_t>(i)];
    for (int j = 0; j < i; ++j)
      newX -= APtr[j] * xdata[static_cast<std::size_t>(j)];
    for (int j = i + 1; j < n; ++j)
      newX -= APtr[j] * xdata[static_cast<std::size_t>(j)];

    newX /= Adata[nSkip * i + i];

    if (findexData[static_cast<std::size_t>(i)] >= 0) {
      const double hiTmp = hiData[static_cast<std::size_t>(i)]
                           * xdata[static_cast<std::size_t>(
                               findexData[static_cast<std::size_t>(i)])];
      const double loTmp = -hiTmp;

      if (newX > hiTmp)
        xdata[static_cast<std::size_t>(i)] = hiTmp;
      else if (newX < loTmp)
        xdata[static_cast<std::size_t>(i)] = loTmp;
      else
        xdata[static_cast<std::size_t>(i)] = newX;
    } else {
      if (newX > hiData[static_cast<std::size_t>(i)])
        xdata[static_cast<std::size_t>(i)] = hiData[static_cast<std::size_t>(i)];
      else if (newX < loData[static_cast<std::size_t>(i)])
        xdata[static_cast<std::size_t>(i)] = loData[static_cast<std::size_t>(i)];
      else
        xdata[static_cast<std::size_t>(i)] = newX;
    }

    if (possibleToTerminate) {
      const double deltaX = std::abs(
          xdata[static_cast<std::size_t>(i)] - oldX);
      if (deltaX > mOption.mDeltaXThreshold)
        possibleToTerminate = false;
    }
  }

  if (!possibleToTerminate) {
    // Normalizing
    for (const auto& index : mCacheOrder) {
      const double dummy = 1.0 / Adata[nSkip * index + index];
      bdata[static_cast<std::size_t>(index)] *= dummy;
      for (int j = 0; j < n; ++j)
        Adata[static_cast<std::size_t>(nSkip * index + j)] *= dummy;
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
        const double* APtr = Adata.data() + nSkip * index;
        double newX = bdata[static_cast<std::size_t>(index)];
        const double oldX = xdata[static_cast<std::size_t>(index)];

        for (int j = 0; j < index; j++)
          newX -= APtr[j] * xdata[static_cast<std::size_t>(j)];

        for (int j = index + 1; j < n; j++)
          newX -= APtr[j] * xdata[static_cast<std::size_t>(j)];

        if (findexData[static_cast<std::size_t>(index)] >= 0) {
          const double hiTmp = hiData[static_cast<std::size_t>(index)]
                               * xdata[static_cast<std::size_t>(
                                   findexData[static_cast<std::size_t>(index)])];
          const double loTmp = -hiTmp;

          if (newX > hiTmp)
            xdata[static_cast<std::size_t>(index)] = hiTmp;
          else if (newX < loTmp)
            xdata[static_cast<std::size_t>(index)] = loTmp;
          else
            xdata[static_cast<std::size_t>(index)] = newX;
        } else {
          if (newX > hiData[static_cast<std::size_t>(index)])
            xdata[static_cast<std::size_t>(index)]
                = hiData[static_cast<std::size_t>(index)];
          else if (newX < loData[static_cast<std::size_t>(index)])
            xdata[static_cast<std::size_t>(index)]
                = loData[static_cast<std::size_t>(index)];
          else
            xdata[static_cast<std::size_t>(index)] = newX;
        }

        if (possibleToTerminate
            && std::abs(xdata[static_cast<std::size_t>(index)])
                   > mOption.mEpsilonForDivision) {
          const double relativeDeltaX = std::abs(
              (xdata[static_cast<std::size_t>(index)] - oldX)
              / xdata[static_cast<std::size_t>(index)]);
          if (relativeDeltaX > mOption.mRelativeDeltaXTolerance)
            possibleToTerminate = false;
        }
      }

      if (possibleToTerminate)
        break;
    }
  }

  const bool success = possibleToTerminate;

  x = Eigen::Map<Eigen::VectorXd>(xdata.data(), n);

  // Restore the original PGS option so per-call overrides don't leak.
  setOption(previousOption);

  Eigen::VectorXd wVec = A * x + b;
  result.iterations = appliedOption.mMaxIteration;
  result.complementarity = (x.array() * wVec.array()).abs().maxCoeff();
  result.residual = (wVec.array().min(Eigen::ArrayXd::Zero(n)))
                        .matrix()
                        .lpNorm<Eigen::Infinity>();
  result.status = (success && !x.hasNaN()) ? math::LcpSolverStatus::Success
                                           : math::LcpSolverStatus::Failed;

  if (options.validateSolution) {
    const double tol
        = std::max(options.complementarityTolerance, options.absoluteTolerance);
    bool valid = true;

    if (wVec.minCoeff() < -options.absoluteTolerance)
      valid = false;
    if ((x.array() - lo.array()).minCoeff() < -options.absoluteTolerance)
      valid = false;
    if ((hi.array() - x.array()).minCoeff() < -options.absoluteTolerance)
      valid = false;
    if ((x.array() * wVec.array()).abs().maxCoeff() > tol)
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
