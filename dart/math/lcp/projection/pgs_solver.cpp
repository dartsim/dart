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

#include "dart/math/lcp/projection/pgs_solver.hpp"

#include "dart/math/lcp/lcp_validation.hpp"
#include "dart/math/lcp/pivoting/dantzig/common.hpp"
#include "dart/math/lcp/pivoting/dantzig/misc.hpp"

#include <algorithm>
#include <iterator>
#include <limits>
#include <span>
#include <string>
#include <vector>

#include <cmath>

namespace dart::math {
namespace {

bool validateParameters(
    const PgsSolver::Parameters& params, std::string* message)
{
  if (!std::isfinite(params.epsilonForDivision)
      || params.epsilonForDivision <= 0.0) {
    if (message) {
      *message = "PGS epsilon_for_division must be positive";
    }
    return false;
  }
  return true;
}

} // namespace

//==============================================================================
void PgsSolver::Scratch::reserve(int size)
{
  if (size <= 0) {
    return;
  }
  const auto problemSize = static_cast<std::size_t>(size);
  const auto paddedSize = static_cast<std::size_t>(padding(size));
  Adata.reserve(problemSize * paddedSize);
  xdata.reserve(problemSize);
  bdata.reserve(problemSize);
  loData.reserve(problemSize);
  hiData.reserve(problemSize);
  w.reserve(problemSize);
  loEff.reserve(problemSize);
  hiEff.reserve(problemSize);
  findexData.reserve(problemSize);
  order.reserve(problemSize);
}

//==============================================================================
void PgsSolver::Scratch::clear() noexcept
{
  Adata.clear();
  xdata.clear();
  bdata.clear();
  loData.clear();
  hiData.clear();
  w.clear();
  loEff.clear();
  hiEff.clear();
  findexData.clear();
  order.clear();
}

//==============================================================================
PgsSolver::PgsSolver()
{
  mDefaultOptions.maxIterations = 30;
  mDefaultOptions.absoluteTolerance = 1e-6;
  mDefaultOptions.relativeTolerance = 1e-3;
  mDefaultOptions.validateSolution = true;
  mDefaultOptions.warmStart = true;
}

//==============================================================================
void PgsSolver::setParameters(const Parameters& params)
{
  mParameters = params;
}

//==============================================================================
const PgsSolver::Parameters& PgsSolver::getParameters() const
{
  return mParameters;
}

//==============================================================================
LcpResult PgsSolver::solve(
    const LcpProblem& problem, Eigen::VectorXd& x, const LcpOptions& options)
{
  Scratch scratch;
  return solve(problem, x, scratch, options);
}

//==============================================================================
LcpResult PgsSolver::solve(
    const LcpProblem& problem,
    Eigen::VectorXd& x,
    Scratch& scratch,
    const LcpOptions& options)
{
  LcpResult result;

  std::string problemMessage;
  if (!detail::validateProblem(problem, &problemMessage)) {
    result.status = LcpSolverStatus::InvalidProblem;
    result.message = problemMessage;
    return result;
  }

  const auto n = std::ssize(problem.b);
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

  return solve(
      problem.A,
      problem.b,
      problem.lo,
      problem.hi,
      problem.findex,
      x,
      scratch,
      options);
}

//==============================================================================
LcpResult PgsSolver::solve(
    const Eigen::Ref<const Eigen::MatrixXd>& A,
    const Eigen::Ref<const Eigen::VectorXd>& b,
    const Eigen::Ref<const Eigen::VectorXd>& lo,
    const Eigen::Ref<const Eigen::VectorXd>& hi,
    const Eigen::Ref<const Eigen::VectorXi>& findex,
    Eigen::Ref<Eigen::VectorXd> x,
    Scratch& scratch,
    const LcpOptions& options)
{
  LcpResult result;

  std::string problemMessage;
  if (!detail::validateProblemView(A, b, lo, hi, findex, &problemMessage)) {
    result.status = LcpSolverStatus::InvalidProblem;
    result.message = problemMessage;
    return result;
  }

  const Eigen::Index n = b.size();
  if (x.size() != n) {
    result.status = LcpSolverStatus::InvalidProblem;
    result.message = "Solution vector dimensions inconsistent";
    return result;
  }

  if (n == 0) {
    result.status = LcpSolverStatus::Success;
    result.iterations = 0;
    result.residual = 0.0;
    result.complementarity = 0.0;
    result.validated = options.validateSolution;
    return result;
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
  const double relaxation = (options.relaxation > 0)
                                ? options.relaxation
                                : mDefaultOptions.relaxation;
  if (!std::isfinite(relaxation) || relaxation <= 0.0 || relaxation > 2.0) {
    result.status = LcpSolverStatus::InvalidProblem;
    result.message = "Relaxation parameter must be in (0, 2]";
    return result;
  }

  const auto problemSize = static_cast<std::size_t>(n);
  const auto paddedSize = static_cast<std::size_t>(nSkip);

  scratch.Adata.assign(problemSize * paddedSize, 0.0);
  scratch.xdata.assign(problemSize, 0.0);
  scratch.bdata.assign(problemSize, 0.0);
  scratch.loData.assign(problemSize, 0.0);
  scratch.hiData.assign(problemSize, 0.0);
  scratch.findexData.assign(problemSize, -1);

  std::span<double> aValues{scratch.Adata};
  std::span<double> xValues{scratch.xdata};
  std::span<double> bValues{scratch.bdata};
  std::span<double> loValues{scratch.loData};
  std::span<double> hiValues{scratch.hiData};
  std::span<int> findexValues{scratch.findexData};

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
    xValues[idx] = (options.warmStart && std::isfinite(x[r])) ? x[r] : 0.0;
    loValues[idx] = lo[r];
    hiValues[idx] = hi[r];
    findexValues[idx] = findex[r];
    for (int c = 0; c < n; ++c) {
      row[static_cast<std::size_t>(c)] = A(r, c);
    }
  }

  auto& order = scratch.order;
  order.clear();
  order.reserve(static_cast<std::size_t>(n));

  bool possibleToTerminate = true;
  for (int i = 0; i < n; ++i) {
    const auto idx = static_cast<std::size_t>(i);
    std::span<double> row = aRow(i);
    if (row[idx] < params->epsilonForDivision) {
      xValues[idx] = 0.0;
      continue;
    }

    order.push_back(i);

    const double oldX = xValues[idx];

    double newX = bValues[idx];
    for (int j = 0; j < i; ++j) {
      newX -= row[static_cast<std::size_t>(j)]
              * xValues[static_cast<std::size_t>(j)];
    }
    for (int j = i + 1; j < n; ++j) {
      newX -= row[static_cast<std::size_t>(j)]
              * xValues[static_cast<std::size_t>(j)];
    }

    newX /= row[idx];

    newX = std::lerp(oldX, newX, relaxation);
    xValues[idx] = projectIntoBounds(newX, idx);

    if (possibleToTerminate) {
      const double deltaX = std::abs(xValues[idx] - oldX);
      if (deltaX > absTolerance) {
        possibleToTerminate = false;
      }
    }
  }

  int iterationsUsed = 1;

  if (!possibleToTerminate && maxIterations > 1) {
    // Normalize to avoid repeated division in the main iteration loop.
    for (const auto& index : order) {
      const auto idx = static_cast<std::size_t>(index);
      std::span<double> row = aRow(index);
      const double invDiag = 1.0 / row[idx];
      bValues[idx] *= invDiag;
      for (int j = 0; j < n; ++j) {
        row[static_cast<std::size_t>(j)] *= invDiag;
      }
    }

    for (int iter = 1; iter < maxIterations; ++iter) {
      if (params->randomizeConstraintOrder && (iter & 7) == 0) {
        for (std::size_t i = 1; i < order.size(); ++i) {
          const int swapIndex = dRandInt(static_cast<int>(i + 1)); // NOLINT
          std::swap(order[i], order[static_cast<std::size_t>(swapIndex)]);
        }
      }

      possibleToTerminate = true;

      for (const auto& index : order) {
        const auto idx = static_cast<std::size_t>(index);
        const std::span<double> row = aRow(index);
        double newX = bValues[idx];
        const double oldX = xValues[idx];

        for (int j = 0; j < index; j++) {
          newX -= row[static_cast<std::size_t>(j)]
                  * xValues[static_cast<std::size_t>(j)];
        }

        for (int j = index + 1; j < n; j++) {
          newX -= row[static_cast<std::size_t>(j)]
                  * xValues[static_cast<std::size_t>(j)];
        }

        newX = std::lerp(oldX, newX, relaxation);
        xValues[idx] = projectIntoBounds(newX, idx);

        if (possibleToTerminate
            && std::abs(xValues[idx]) > params->epsilonForDivision) {
          const double relativeDeltaX
              = std::abs((xValues[idx] - oldX) / xValues[idx]);
          if (relativeDeltaX > relativeTolerance) {
            possibleToTerminate = false;
          }
        }
      }

      ++iterationsUsed;

      if (possibleToTerminate) {
        break;
      }
    }
  }

  x = Eigen::Map<const Eigen::VectorXd>(scratch.xdata.data(), n);

  scratch.w.resize(problemSize);
  Eigen::Map<Eigen::VectorXd> wVec(scratch.w.data(), n);
  wVec.noalias() = A * x;
  wVec -= b;
  result.iterations = iterationsUsed;

  scratch.loEff.resize(problemSize);
  scratch.hiEff.resize(problemSize);
  Eigen::Map<Eigen::VectorXd> loEff(scratch.loEff.data(), n);
  Eigen::Map<Eigen::VectorXd> hiEff(scratch.hiEff.data(), n);
  std::string boundsMessage;
  const bool boundsOk = detail::computeEffectiveBoundsInto(
      lo, hi, findex, x, loEff, hiEff, &boundsMessage);
  if (!boundsOk) {
    result.status = LcpSolverStatus::InvalidProblem;
    result.message = boundsMessage;
    return result;
  }

  result.residual
      = detail::naturalResidualInfinityNormView(x, wVec, loEff, hiEff);
  result.complementarity = detail::complementarityInfinityNormView(
      x, wVec, loEff, hiEff, absTolerance);
  if (x.hasNaN()) {
    result.status = LcpSolverStatus::Failed;
  } else if (!possibleToTerminate) {
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
    const bool valid = detail::validateSolutionView(
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

//==============================================================================
std::string PgsSolver::getName() const
{
  return "Pgs";
}

//==============================================================================
std::string PgsSolver::getCategory() const
{
  return "Projection";
}

} // namespace dart::math
