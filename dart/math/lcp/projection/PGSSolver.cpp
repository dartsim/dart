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
#include "dart/math/lcp/dantzig/Misc.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace dart::math {

PGSSolver::PGSSolver()
{
  mDefaultOptions.maxIterations = 30;
  mDefaultOptions.absoluteTolerance = 1e-6;
  mDefaultOptions.relativeTolerance = 1e-3;
  mDefaultOptions.validateSolution = true;
  mDefaultOptions.warmStart = true;
}

//==============================================================================
void PGSSolver::setParameters(const Parameters& params)
{
  mParameters = params;
}

//==============================================================================
const PGSSolver::Parameters& PGSSolver::getParameters() const
{
  return mParameters;
}

//==============================================================================
LcpResult PGSSolver::solve(
    const Eigen::MatrixXd& A,
    const Eigen::VectorXd& b,
    Eigen::VectorXd& x,
    const LcpOptions& options)
{
  const int n = static_cast<int>(b.size());
  Eigen::VectorXd lo = Eigen::VectorXd::Zero(n);
  Eigen::VectorXd hi
      = Eigen::VectorXd::Constant(n, ScalarTraits<double>::inf());
  Eigen::VectorXi findex = Eigen::VectorXi::Constant(n, -1);
  return solve(A, b, lo, hi, findex, x, options);
}

//==============================================================================
LcpResult PGSSolver::solve(
    const Eigen::MatrixXd& A,
    const Eigen::VectorXd& b,
    const Eigen::VectorXd& lo,
    const Eigen::VectorXd& hi,
    const Eigen::VectorXi& findex,
    Eigen::VectorXd& x,
    const LcpOptions& options)
{
  LcpResult result;

  const bool dimensionMismatch
      = (A.rows() != A.cols()) || (A.rows() != b.size())
        || (lo.size() != b.size()) || (hi.size() != b.size())
        || (findex.size() != b.size());
  if (dimensionMismatch) {
    result.status = LcpSolverStatus::InvalidProblem;
    result.message = "Matrix/vector dimensions inconsistent";
    return result;
  }

  const Parameters* params = options.customOptions
                                 ? static_cast<const Parameters*>(
                                     options.customOptions)
                                 : &mParameters;

  const int n = static_cast<int>(b.size());
  const int nSkip = padding(n);
  const int maxIterations = std::max(
      1, (options.maxIterations > 0) ? options.maxIterations
                                     : mDefaultOptions.maxIterations);
  const double absTolerance = (options.absoluteTolerance > 0)
                                  ? options.absoluteTolerance
                                  : mDefaultOptions.absoluteTolerance;
  const double relativeTolerance = (options.relativeTolerance > 0)
                                       ? options.relativeTolerance
                                       : mDefaultOptions.relativeTolerance;

  std::vector<double> Adata(static_cast<std::size_t>(n * nSkip), 0.0);
  std::vector<double> xdata(static_cast<std::size_t>(n), 0.0);
  std::vector<double> bdata(static_cast<std::size_t>(n), 0.0);
  std::vector<double> loData(static_cast<std::size_t>(n), 0.0);
  std::vector<double> hiData(static_cast<std::size_t>(n), 0.0);
  std::vector<int> findexData(static_cast<std::size_t>(n), -1);

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

  std::vector<int> order;
  order.reserve(static_cast<std::size_t>(n));

  bool possibleToTerminate = true;
  for (int i = 0; i < n; ++i) {
    if (Adata[static_cast<std::size_t>(nSkip * i + i)]
        < params->epsilonForDivision) {
      xdata[static_cast<std::size_t>(i)] = 0.0;
      continue;
    }

    order.push_back(i);

    const double* APtr = Adata.data() + nSkip * i;
    const double oldX = xdata[static_cast<std::size_t>(i)];

    double newX = bdata[static_cast<std::size_t>(i)];
    for (int j = 0; j < i; ++j)
      newX -= APtr[j] * xdata[static_cast<std::size_t>(j)];
    for (int j = i + 1; j < n; ++j)
      newX -= APtr[j] * xdata[static_cast<std::size_t>(j)];

    newX /= Adata[static_cast<std::size_t>(nSkip * i + i)];

    if (findexData[static_cast<std::size_t>(i)] >= 0) {
      const double hiTmp
          = hiData[static_cast<std::size_t>(i)]
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
        xdata[static_cast<std::size_t>(i)]
            = hiData[static_cast<std::size_t>(i)];
      else if (newX < loData[static_cast<std::size_t>(i)])
        xdata[static_cast<std::size_t>(i)]
            = loData[static_cast<std::size_t>(i)];
      else
        xdata[static_cast<std::size_t>(i)] = newX;
    }

    if (possibleToTerminate) {
      const double deltaX = std::abs(xdata[static_cast<std::size_t>(i)] - oldX);
      if (deltaX > absTolerance)
        possibleToTerminate = false;
    }
  }

  int iterationsUsed = 1;

  if (!possibleToTerminate && maxIterations > 1) {
    // Normalize to avoid repeated division in the main iteration loop.
    for (const auto& index : order) {
      const double invDiag
          = 1.0 / Adata[static_cast<std::size_t>(nSkip * index + index)];
      bdata[static_cast<std::size_t>(index)] *= invDiag;
      for (int j = 0; j < n; ++j)
        Adata[static_cast<std::size_t>(nSkip * index + j)] *= invDiag;
    }

    for (int iter = 1; iter < maxIterations; ++iter) {
      if (params->randomizeConstraintOrder && (iter & 7) == 0) {
        for (std::size_t i = 1; i < order.size(); ++i) {
          const int swapIndex
              = dRandInt(static_cast<int>(i + 1)); // NOLINT
          std::swap(order[i], order[static_cast<std::size_t>(swapIndex)]);
        }
      }

      possibleToTerminate = true;

      for (const auto& index : order) {
        const double* APtr = Adata.data() + nSkip * index;
        double newX = bdata[static_cast<std::size_t>(index)];
        const double oldX = xdata[static_cast<std::size_t>(index)];

        for (int j = 0; j < index; j++)
          newX -= APtr[j] * xdata[static_cast<std::size_t>(j)];

        for (int j = index + 1; j < n; j++)
          newX -= APtr[j] * xdata[static_cast<std::size_t>(j)];

        if (findexData[static_cast<std::size_t>(index)] >= 0) {
          const double hiTmp
              = hiData[static_cast<std::size_t>(index)]
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
                   > params->epsilonForDivision) {
          const double relativeDeltaX = std::abs(
              (xdata[static_cast<std::size_t>(index)] - oldX)
              / xdata[static_cast<std::size_t>(index)]);
          if (relativeDeltaX > relativeTolerance)
            possibleToTerminate = false;
        }
      }

      ++iterationsUsed;

      if (possibleToTerminate)
        break;
    }
  }

  x = Eigen::Map<Eigen::VectorXd>(xdata.data(), n);

  Eigen::VectorXd wVec = A * x - b;
  result.iterations = iterationsUsed;
  result.complementarity = (x.array() * wVec.array()).abs().maxCoeff();
  result.residual = (wVec.array().min(Eigen::ArrayXd::Zero(n)))
                        .matrix()
                        .lpNorm<Eigen::Infinity>();
  result.status = (possibleToTerminate && !x.hasNaN())
                      ? LcpSolverStatus::Success
                      : LcpSolverStatus::Failed;

  if (options.validateSolution) {
    const double tol = std::max(
        options.complementarityTolerance > 0 ? options.complementarityTolerance
                                             : mDefaultOptions
                                                   .complementarityTolerance,
        absTolerance);
    bool valid = true;

    if (wVec.minCoeff() < -absTolerance)
      valid = false;
    if ((x.array() - lo.array()).minCoeff() < -absTolerance)
      valid = false;
    if ((hi.array() - x.array()).minCoeff() < -absTolerance)
      valid = false;
    if ((x.array() * wVec.array()).abs().maxCoeff() > tol)
      valid = false;

    result.validated = true;
    if (!valid) {
      result.status = LcpSolverStatus::NumericalError;
      result.message = "Solution validation failed";
    }
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
