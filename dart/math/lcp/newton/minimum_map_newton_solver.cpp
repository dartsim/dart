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
#include "dart/math/lcp/projection/pgs_solver.hpp"

#include <Eigen/QR>

#include <algorithm>
#include <iterator>
#include <limits>
#include <ranges>
#include <string>
#include <vector>

#include <cmath>

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

void computeMinimumMapResidualAndGradient(
    const Eigen::MatrixXd& A,
    const Eigen::VectorXd& b,
    const Eigen::VectorXd& x,
    Eigen::VectorXd& H,
    Eigen::VectorXd& grad)
{
  const Eigen::VectorXd y = A * x - b;
  H = x.cwiseMin(y);
  grad.setZero(x.size());

  for (Eigen::Index i = 0; i < x.size(); ++i) {
    if (y[i] < x[i]) {
      grad.noalias() += H[i] * A.row(i).transpose();
    } else {
      grad[i] += H[i];
    }
  }
}

double minimumMapMerit(
    const Eigen::MatrixXd& A,
    const Eigen::VectorXd& b,
    const Eigen::VectorXd& x)
{
  Eigen::VectorXd H(x.size());
  Eigen::VectorXd grad(x.size());
  computeMinimumMapResidualAndGradient(A, b, x, H, grad);
  return 0.5 * H.squaredNorm();
}

bool validateParameters(
    const MinimumMapNewtonSolver::Parameters& params, std::string* message)
{
  if (params.maxLineSearchSteps <= 0) {
    if (message) {
      *message = "Minimum-map Newton max_line_search_steps must be positive";
    }
    return false;
  }
  if (!std::isfinite(params.stepReduction) || params.stepReduction <= 0.0
      || params.stepReduction >= 1.0) {
    if (message) {
      *message = "Minimum-map Newton step_reduction must be in (0, 1)";
    }
    return false;
  }
  if (!std::isfinite(params.sufficientDecrease)
      || params.sufficientDecrease < 0.0 || params.sufficientDecrease >= 1.0) {
    if (message) {
      *message = "Minimum-map Newton sufficient_decrease must be in [0, 1)";
    }
    return false;
  }
  if (!std::isfinite(params.minStep) || params.minStep <= 0.0) {
    if (message) {
      *message = "Minimum-map Newton min_step must be positive";
    }
    return false;
  }
  if (params.maxGradientDescentWarmStartSteps < 0) {
    if (message) {
      *message
          = "Minimum-map Newton max_gradient_descent_warm_start_steps must "
            "be non-negative";
    }
    return false;
  }
  if (params.maxGradientDescentLineSearchSteps <= 0) {
    if (message) {
      *message
          = "Minimum-map Newton max_gradient_descent_line_search_steps must "
            "be positive";
    }
    return false;
  }
  if (!std::isfinite(params.gradientDescentStepReduction)
      || params.gradientDescentStepReduction <= 0.0
      || params.gradientDescentStepReduction >= 1.0) {
    if (message) {
      *message
          = "Minimum-map Newton gradient_descent_step_reduction must be in "
            "(0, 1)";
    }
    return false;
  }
  if (!std::isfinite(params.gradientDescentSufficientDecrease)
      || params.gradientDescentSufficientDecrease < 0.0
      || params.gradientDescentSufficientDecrease >= 1.0) {
    if (message) {
      *message
          = "Minimum-map Newton gradient_descent_sufficient_decrease must be "
            "in [0, 1)";
    }
    return false;
  }
  if (!std::isfinite(params.gradientDescentMinStep)
      || params.gradientDescentMinStep <= 0.0) {
    if (message) {
      *message
          = "Minimum-map Newton gradient_descent_min_step must be positive";
    }
    return false;
  }
  if (params.maxPgsWarmStartIterations < 0) {
    if (message) {
      *message
          = "Minimum-map Newton max_pgs_warm_start_iterations must be "
            "non-negative";
    }
    return false;
  }
  if (!std::isfinite(params.pgsWarmStartRelaxation)
      || params.pgsWarmStartRelaxation <= 0.0
      || params.pgsWarmStartRelaxation > 2.0) {
    if (message) {
      *message
          = "Minimum-map Newton pgs_warm_start_relaxation must be in (0, 2]";
    }
    return false;
  }
  return true;
}

void runPgsWarmStart(
    const LcpProblem& problem,
    Eigen::VectorXd& x,
    const LcpOptions& options,
    const MinimumMapNewtonSolver::Parameters& params)
{
  if (params.maxPgsWarmStartIterations <= 0) {
    return;
  }

  const double initialMerit = minimumMapMerit(problem.A, problem.b, x);
  Eigen::VectorXd candidate = x;
  LcpOptions pgsOptions = options;
  pgsOptions.maxIterations = params.maxPgsWarmStartIterations;
  pgsOptions.relaxation = params.pgsWarmStartRelaxation;
  pgsOptions.warmStart = true;
  pgsOptions.validateSolution = false;
  pgsOptions.customOptions = nullptr;

  PgsSolver pgs;
  pgs.solve(problem, candidate, pgsOptions);

  if (candidate.allFinite()
      && minimumMapMerit(problem.A, problem.b, candidate) < initialMerit) {
    x = candidate;
  }
}

void runGradientDescentWarmStart(
    const Eigen::MatrixXd& A,
    const Eigen::VectorXd& b,
    Eigen::VectorXd& x,
    const MinimumMapNewtonSolver::Parameters& params,
    double absTol,
    double relTol)
{
  if (params.maxGradientDescentWarmStartSteps <= 0) {
    return;
  }

  Eigen::VectorXd H(x.size());
  Eigen::VectorXd grad(x.size());

  for (int iter = 0; iter < params.maxGradientDescentWarmStartSteps; ++iter) {
    computeMinimumMapResidualAndGradient(A, b, x, H, grad);

    const double scale = std::max(
        1.0,
        std::max(
            vectorInfinityNorm(b),
            matrixInfinityNorm(A) * vectorInfinityNorm(x)));
    const double tol = std::max(absTol, relTol * scale);
    const double Hnorm = H.cwiseAbs().maxCoeff();
    if (Hnorm <= tol || !grad.allFinite()) {
      break;
    }

    const double merit = 0.5 * H.squaredNorm();
    double step = 1.0;
    bool accepted = false;

    for (int ls = 0; ls < params.maxGradientDescentLineSearchSteps; ++ls) {
      const Eigen::VectorXd xCandidate = (x - step * grad).cwiseMax(0.0);
      Eigen::VectorXd HCandidate(x.size());
      Eigen::VectorXd gradCandidate(x.size());
      computeMinimumMapResidualAndGradient(
          A, b, xCandidate, HCandidate, gradCandidate);
      const double candidateMerit = 0.5 * HCandidate.squaredNorm();
      const double target
          = merit * (1.0 - params.gradientDescentSufficientDecrease * step);

      if (candidateMerit <= target || candidateMerit < merit) {
        x = xCandidate;
        accepted = true;
        break;
      }

      step *= params.gradientDescentStepReduction;
      if (step < params.gradientDescentMinStep) {
        break;
      }
    }

    if (!accepted) {
      break;
    }
  }
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
  if (!problem.isStandardLcp(absTol)) {
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
  std::string parameterMessage;
  if (!validateParameters(*params, &parameterMessage)) {
    result.status = LcpSolverStatus::InvalidProblem;
    result.message = parameterMessage;
    return result;
  }

  Eigen::VectorXd fastW;
  if (!options.warmStart
      && detail::trySolveStrictInteriorStandardLcpLltFirst(
          problem, absTol, std::max(absTol, compTol), x, &fastW)) {
    Eigen::VectorXd loEff;
    Eigen::VectorXd hiEff;
    std::string boundsMessage;
    if (!detail::computeEffectiveBounds(
            lo, hi, findex, x, loEff, hiEff, &boundsMessage)) {
      result.status = LcpSolverStatus::InvalidProblem;
      result.message = boundsMessage;
      return result;
    }

    result.status = LcpSolverStatus::Success;
    result.iterations = 0;
    result.residual
        = detail::naturalResidualInfinityNorm(x, fastW, loEff, hiEff);
    result.complementarity
        = detail::complementarityInfinityNorm(x, fastW, loEff, hiEff, absTol);
    result.validated = options.validateSolution;
    return result;
  }

  runPgsWarmStart(problem, x, options, *params);
  runGradientDescentWarmStart(A, b, x, *params, absTol, relTol);

  bool converged = false;
  bool lineSearchFailed = false;
  int iterationsUsed = 0;

  Eigen::VectorXd y(n);
  Eigen::VectorXd H(n);
  Eigen::VectorXd dx(n);

  for ([[maybe_unused]] const auto iter : std::views::iota(0, maxIterations)) {
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

    for (const auto i : std::views::iota(0, static_cast<int>(n))) {
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

      for (const auto r : std::views::iota(0, static_cast<int>(aSize))) {
        const int i = active[r];
        rhs[r] = -y[i];
        for (const auto c : std::views::iota(0, static_cast<int>(aSize))) {
          A_AA(r, c) = A(i, active[c]);
        }
        for (const auto c : std::views::iota(0, static_cast<int>(fSize))) {
          A_AF(r, c) = A(i, freeSet[c]);
        }
      }

      for (const auto c : std::views::iota(0, static_cast<int>(fSize))) {
        dxF[c] = dx[freeSet[c]];
      }

      rhs -= A_AF * dxF;

      const Eigen::VectorXd dxA = A_AA.colPivHouseholderQr().solve(rhs);
      if (!dxA.allFinite()) {
        result.status = LcpSolverStatus::NumericalError;
        result.message = "Newton solve produced non-finite step";
        return result;
      }

      for (const auto r : std::views::iota(0, static_cast<int>(aSize))) {
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
