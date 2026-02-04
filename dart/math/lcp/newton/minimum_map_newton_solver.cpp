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

#include "dart/math/lcp/newton/minimum_map_newton_solver.hpp"

#include "dart/math/lcp/lcp_validation.hpp"
#include "dart/math/lcp/pivoting/dantzig_solver.hpp"

#include <Eigen/QR>

#include <algorithm>
#include <iterator>
#include <limits>
#include <vector>

namespace dart::math {
namespace {

double matrixInfinityNorm(const Eigen::MatrixXd& A)
{
  if (A.size() == 0) {
    return 0.0;
  }

  return A.cwiseAbs().rowwise().sum().maxCoeff();
}

double vectorInfinityNorm(const Eigen::VectorXd& v)
{
  return v.size() > 0 ? v.cwiseAbs().maxCoeff() : 0.0;
}

} // namespace

//==============================================================================
MinimumMapNewtonSolver::MinimumMapNewtonSolver()
{
  mDefaultOptions.maxIterations = 50;
  mDefaultOptions.absoluteTolerance = 1e-6;
  mDefaultOptions.relativeTolerance = 1e-4;
  mDefaultOptions.complementarityTolerance = 1e-6;
  mDefaultOptions.validateSolution = true;
  mDefaultOptions.warmStart = false;
}

//==============================================================================
void MinimumMapNewtonSolver::setParameters(const Parameters& params)
{
  mParameters = params;
}

//==============================================================================
const MinimumMapNewtonSolver::Parameters&
MinimumMapNewtonSolver::getParameters() const
{
  return mParameters;
}

//==============================================================================
LcpResult MinimumMapNewtonSolver::solve(
    const LcpProblem& problem, Eigen::VectorXd& x, const LcpOptions& options)
{
  LcpResult result;

  std::string problemMessage;
  if (!detail::validateProblem(problem, &problemMessage)) {
    result.status = LcpSolverStatus::InvalidProblem;
    result.message = problemMessage;
    return result;
  }

  const auto& A = problem.A;
  const auto& b = problem.b;
  const auto& lo = problem.lo;
  const auto& hi = problem.hi;
  const auto& findex = problem.findex;

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

  const double absTol = (options.absoluteTolerance > 0.0)
                            ? options.absoluteTolerance
                            : mDefaultOptions.absoluteTolerance;
  const bool standardBounds
      = (lo.array().abs().maxCoeff() <= absTol)
        && (hi.array() == std::numeric_limits<double>::infinity()).all()
        && (findex.array() < 0).all();
  if (!standardBounds) {
    DantzigSolver fallback;
    return fallback.solve(problem, x, options);
  }

  if (x.size() != n || !options.warmStart || !x.allFinite()) {
    x = Eigen::VectorXd::Zero(n);
  } else {
    x = x.cwiseMax(0.0);
  }

  const int maxIterations = std::max(
      1,
      (options.maxIterations > 0) ? options.maxIterations
                                  : mDefaultOptions.maxIterations);
  const double relTol = (options.relativeTolerance > 0.0)
                            ? options.relativeTolerance
                            : mDefaultOptions.relativeTolerance;
  const double compTol = (options.complementarityTolerance > 0.0)
                             ? options.complementarityTolerance
                             : mDefaultOptions.complementarityTolerance;

  const Parameters* params
      = options.customOptions
            ? static_cast<const Parameters*>(options.customOptions)
            : &mParameters;

  bool converged = false;
  bool lineSearchFailed = false;
  int iterationsUsed = 0;

  Eigen::VectorXd y(n);
  Eigen::VectorXd H(n);
  Eigen::VectorXd dx(n);

  for (int iter = 0; iter < maxIterations; ++iter) {
    iterationsUsed = iter + 1;

    y = A * x - b;
    H = x.cwiseMin(y);

    const double scale = std::max(
        1.0,
        std::max(
            vectorInfinityNorm(b),
            matrixInfinityNorm(A) * vectorInfinityNorm(x)));
    const double tol = std::max(absTol, relTol * scale);

    if (H.cwiseAbs().maxCoeff() <= tol) {
      converged = true;
      break;
    }

    std::vector<int> active;
    std::vector<int> freeSet;
    active.reserve(static_cast<std::size_t>(n));
    freeSet.reserve(static_cast<std::size_t>(n));

    for (int i = 0; i < n; ++i) {
      if (y[i] < x[i]) {
        active.push_back(i);
      } else {
        freeSet.push_back(i);
      }
    }

    dx.setZero();
    for (const int idx : freeSet) {
      dx[idx] = -x[idx];
    }

    if (!active.empty()) {
      const auto aSize = std::ssize(active);
      const auto fSize = std::ssize(freeSet);
      Eigen::MatrixXd A_AA(aSize, aSize);
      Eigen::MatrixXd A_AF(aSize, fSize);
      Eigen::VectorXd rhs(aSize);
      Eigen::VectorXd dxF(fSize);

      for (int r = 0; r < aSize; ++r) {
        const int i = active[r];
        rhs[r] = -y[i];
        for (int c = 0; c < aSize; ++c) {
          A_AA(r, c) = A(i, active[c]);
        }
        for (int c = 0; c < fSize; ++c) {
          A_AF(r, c) = A(i, freeSet[c]);
        }
      }

      for (int c = 0; c < fSize; ++c) {
        dxF[c] = dx[freeSet[c]];
      }

      rhs -= A_AF * dxF;

      const Eigen::VectorXd dxA = A_AA.colPivHouseholderQr().solve(rhs);
      if (!dxA.allFinite()) {
        result.status = LcpSolverStatus::NumericalError;
        result.message = "Newton solve produced non-finite step";
        return result;
      }

      for (int r = 0; r < aSize; ++r) {
        dx[active[r]] = dxA[r];
      }
    }

    double step = 1.0;
    bool accepted = false;
    const double Hnorm = H.cwiseAbs().maxCoeff();

    for (int ls = 0; ls < params->maxLineSearchSteps; ++ls) {
      const Eigen::VectorXd xCandidate = (x + step * dx).cwiseMax(0.0);
      const Eigen::VectorXd yCandidate = A * xCandidate - b;
      const Eigen::VectorXd HCandidate = xCandidate.cwiseMin(yCandidate);
      const double normCandidate = HCandidate.cwiseAbs().maxCoeff();
      const double target = Hnorm * (1.0 - params->sufficientDecrease * step);

      if (normCandidate <= target || normCandidate < Hnorm) {
        x = xCandidate;
        accepted = true;
        break;
      }

      step *= params->stepReduction;
      if (step < params->minStep) {
        break;
      }
    }

    if (!accepted) {
      lineSearchFailed = true;
      result.message = "Line search failed to reduce residual";
      break;
    }
  }

  result.iterations = iterationsUsed;
  const Eigen::VectorXd w = A * x - b;

  Eigen::VectorXd loEff;
  Eigen::VectorXd hiEff;
  std::string boundsMessage;
  if (!detail::computeEffectiveBounds(
          lo, hi, findex, x, loEff, hiEff, &boundsMessage)) {
    result.status = LcpSolverStatus::InvalidProblem;
    result.message = boundsMessage;
    return result;
  }

  result.residual = detail::naturalResidualInfinityNorm(x, w, loEff, hiEff);
  result.complementarity
      = detail::complementarityInfinityNorm(x, w, loEff, hiEff, absTol);

  if (converged) {
    result.status = LcpSolverStatus::Success;
  } else if (lineSearchFailed) {
    result.status = LcpSolverStatus::Failed;
  } else {
    result.status = LcpSolverStatus::MaxIterations;
  }

  if (options.validateSolution && result.status == LcpSolverStatus::Success) {
    const double validationTol = std::max(absTol, compTol);
    std::string validationMessage;
    const bool valid = detail::validateSolution(
        x, w, loEff, hiEff, validationTol, &validationMessage);
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
std::string MinimumMapNewtonSolver::getName() const
{
  return "MinimumMapNewton";
}

//==============================================================================
std::string MinimumMapNewtonSolver::getCategory() const
{
  return "Newton";
}

} // namespace dart::math
