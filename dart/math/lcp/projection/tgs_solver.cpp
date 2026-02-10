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

#include "dart/math/lcp/projection/tgs_solver.hpp"

#include "dart/math/lcp/lcp_validation.hpp"
#include "dart/math/lcp/pivoting/dantzig/common.hpp"

#include <algorithm>
#include <limits>
#include <vector>

#include <cmath>

namespace dart::math {

TgsSolver::TgsSolver()
{
  mDefaultOptions.maxIterations = 100;
  mDefaultOptions.absoluteTolerance = 1e-6;
  mDefaultOptions.relativeTolerance = 1e-3;
  mDefaultOptions.validateSolution = true;
  mDefaultOptions.warmStart = true;
}

void TgsSolver::setParameters(const Parameters& params)
{
  mParameters = params;
}

const TgsSolver::Parameters& TgsSolver::getParameters() const
{
  return mParameters;
}

LcpResult TgsSolver::solve(
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

  if (x.size() != n) {
    x = Eigen::VectorXd::Zero(n);
  }

  const Parameters* params
      = options.customOptions
            ? static_cast<const Parameters*>(options.customOptions)
            : &mParameters;

  const int nSkip = padding(n);
  const int maxIterations = std::max(
      1,
      (options.maxIterations > 0) ? options.maxIterations
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
    const double initVal
        = (options.warmStart && std::isfinite(x[r])) ? x[r] : 0.0;
    xdata[static_cast<std::size_t>(r)] = initVal;
    loData[static_cast<std::size_t>(r)] = lo[r];
    hiData[static_cast<std::size_t>(r)] = hi[r];
    findexData[static_cast<std::size_t>(r)] = findex[r];
    for (int c = 0; c < n; ++c) {
      Adata[static_cast<std::size_t>(r * nSkip + c)] = A(r, c);
    }
  }

  std::vector<int> order;
  order.reserve(static_cast<std::size_t>(n));

  for (int i = 0; i < n; ++i) {
    if (Adata[static_cast<std::size_t>(nSkip * i + i)]
        >= params->epsilonForDivision) {
      order.push_back(i);
    } else {
      xdata[static_cast<std::size_t>(i)] = 0.0;
    }
  }

  std::vector<double> invDiag(static_cast<std::size_t>(n), 0.0);
  for (const auto& index : order) {
    invDiag[static_cast<std::size_t>(index)]
        = 1.0 / Adata[static_cast<std::size_t>(nSkip * index + index)];
  }

  int iterationsUsed = 0;
  bool converged = false;

  // TGS uses relaxation = 1.0 (standard Gauss-Seidel) with soft regularization
  const double relaxation = 1.0;

  for (int iter = 0; iter < maxIterations; ++iter) {
    ++iterationsUsed;

    bool possibleToTerminate = true;

    for (const auto& index : order) {
      const std::size_t idx = static_cast<std::size_t>(index);
      const double* APtr = Adata.data() + nSkip * index;

      double newX = bdata[idx];
      for (int j = 0; j < index; ++j) {
        newX -= APtr[j] * xdata[static_cast<std::size_t>(j)];
      }
      for (int j = index + 1; j < n; ++j) {
        newX -= APtr[j] * xdata[static_cast<std::size_t>(j)];
      }
      newX *= invDiag[idx];

      const double oldX = xdata[idx];
      newX = oldX + relaxation * (newX - oldX);

      double loVal = loData[idx];
      double hiVal = hiData[idx];
      if (findexData[idx] >= 0) {
        const double fricLimit = std::abs(
            hiData[idx] * xdata[static_cast<std::size_t>(findexData[idx])]);
        loVal = -fricLimit;
        hiVal = fricLimit;
      }

      newX = std::clamp(newX, loVal, hiVal);
      xdata[idx] = newX;

      if (possibleToTerminate && std::abs(newX) > params->epsilonForDivision) {
        const double relativeDelta = std::abs((newX - oldX) / newX);
        if (relativeDelta > relativeTolerance) {
          possibleToTerminate = false;
        }
      }
    }

    if (possibleToTerminate) {
      converged = true;
      break;
    }
  }

  x = Eigen::Map<Eigen::VectorXd>(xdata.data(), n);

  Eigen::VectorXd wVec = A * x - b;
  result.iterations = iterationsUsed;

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

  result.residual = detail::naturalResidualInfinityNorm(x, wVec, loEff, hiEff);
  result.complementarity = detail::complementarityInfinityNorm(
      x, wVec, loEff, hiEff, absTolerance);

  if (x.hasNaN()) {
    result.status = LcpSolverStatus::Failed;
  } else if (!converged) {
    result.status = LcpSolverStatus::MaxIterations;
  } else {
    result.status = LcpSolverStatus::Success;
  }

  if (options.validateSolution && result.status == LcpSolverStatus::Success) {
    const double compTol = (options.complementarityTolerance > 0)
                               ? options.complementarityTolerance
                               : mDefaultOptions.complementarityTolerance;
    const double validationTol = std::max(absTolerance, compTol);

    std::string validationMessage;
    const bool valid = detail::validateSolution(
        x, wVec, loEff, hiEff, validationTol, &validationMessage);
    result.validated = true;
    if (!valid) {
      result.status = LcpSolverStatus::NumericalError;
      result.message = validationMessage.empty() ? "Solution validation failed"
                                                 : validationMessage;
    }
  }

  return result;
}

std::string TgsSolver::getName() const
{
  return "Tgs";
}

std::string TgsSolver::getCategory() const
{
  return "Projection";
}

} // namespace dart::math
