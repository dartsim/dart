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
#include <span>
#include <string>
#include <vector>

#include <cmath>

namespace dart::math {
namespace {

bool validateParameters(
    const TgsSolver::Parameters& params, std::string* message)
{
  if (!std::isfinite(params.epsilonForDivision)
      || params.epsilonForDivision <= 0.0) {
    if (message) {
      *message = "TGS epsilon_for_division must be positive";
    }
    return false;
  }
  return true;
}

} // namespace

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
  std::string parameterMessage;
  if (!validateParameters(*params, &parameterMessage)) {
    result.status = LcpSolverStatus::InvalidProblem;
    result.message = parameterMessage;
    return result;
  }

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

  const auto problemSize = static_cast<std::size_t>(n);
  const auto paddedSize = static_cast<std::size_t>(nSkip);

  std::vector<double> Adata(problemSize * paddedSize, 0.0);
  std::vector<double> xdata(problemSize, 0.0);
  std::vector<double> bdata(problemSize, 0.0);
  std::vector<double> loData(problemSize, 0.0);
  std::vector<double> hiData(problemSize, 0.0);
  std::vector<int> findexData(problemSize, -1);

  std::span<double> aValues{Adata};
  std::span<double> xValues{xdata};
  std::span<double> bValues{bdata};
  std::span<double> loValues{loData};
  std::span<double> hiValues{hiData};
  std::span<int> findexValues{findexData};

  auto aRow = [&](int row) -> std::span<double> {
    return aValues.subspan(
        static_cast<std::size_t>(row) * paddedSize, problemSize);
  };

  auto projectIntoBounds = [&](double value, std::size_t idx) {
    if (findexValues[idx] >= 0) {
      const auto frictionIndex = static_cast<std::size_t>(findexValues[idx]);
      const double frictionLimit
          = std::abs(hiValues[idx] * xValues[frictionIndex]);
      return detail::projectToBounds(value, -frictionLimit, frictionLimit);
    }

    return detail::projectToBounds(value, loValues[idx], hiValues[idx]);
  };

  for (int r = 0; r < n; ++r) {
    const auto idx = static_cast<std::size_t>(r);
    std::span<double> row = aRow(r);
    bValues[idx] = b[r];
    const double initVal
        = (options.warmStart && std::isfinite(x[r])) ? x[r] : 0.0;
    xValues[idx] = initVal;
    loValues[idx] = lo[r];
    hiValues[idx] = hi[r];
    findexValues[idx] = findex[r];
    for (int c = 0; c < n; ++c) {
      row[static_cast<std::size_t>(c)] = A(r, c);
    }
  }

  std::vector<int> order;
  order.reserve(static_cast<std::size_t>(n));

  for (int i = 0; i < n; ++i) {
    const auto idx = static_cast<std::size_t>(i);
    const std::span<double> row = aRow(i);
    if (row[idx] >= params->epsilonForDivision) {
      order.push_back(i);
    } else {
      xValues[idx] = 0.0;
    }
  }

  std::vector<double> invDiag(problemSize, 0.0);
  std::span<double> invDiagValues{invDiag};
  for (const auto& index : order) {
    const auto idx = static_cast<std::size_t>(index);
    const std::span<double> row = aRow(index);
    invDiagValues[idx] = 1.0 / row[idx];
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
      const std::span<double> row = aRow(index);

      double newX = bValues[idx];
      for (int j = 0; j < index; ++j) {
        newX -= row[static_cast<std::size_t>(j)]
                * xValues[static_cast<std::size_t>(j)];
      }
      for (int j = index + 1; j < n; ++j) {
        newX -= row[static_cast<std::size_t>(j)]
                * xValues[static_cast<std::size_t>(j)];
      }
      newX *= invDiagValues[idx];

      const double oldX = xValues[idx];
      newX = std::lerp(oldX, newX, relaxation);
      newX = projectIntoBounds(newX, idx);
      xValues[idx] = newX;

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
