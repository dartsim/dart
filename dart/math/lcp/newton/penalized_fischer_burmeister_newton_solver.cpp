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

#include "dart/math/lcp/newton/penalized_fischer_burmeister_newton_solver.hpp"

#include "dart/math/lcp/lcp_validation.hpp"
#include "dart/math/lcp/pivoting/dantzig_solver.hpp"
#include "dart/math/lcp/projection/pgs_solver.hpp"

#include <Eigen/QR>

#include <algorithm>
#include <iterator>
#include <limits>
#include <string>

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

void computePenalizedFbFunction(
    const Eigen::VectorXd& x,
    const Eigen::VectorXd& y,
    double epsilon,
    double lambda,
    Eigen::VectorXd& phi,
    Eigen::VectorXd& p,
    Eigen::VectorXd& q)
{
  const Eigen::Index n = x.size();
  phi.resize(n);
  p.resize(n);
  q.resize(n);

  const double penaltyScale = 1.0 - lambda;

  for (Eigen::Index i = 0; i < n; ++i) {
    const double xi = x[i];
    const double yi = y[i];
    const double r = std::hypot(xi, yi, epsilon);
    const double invR = (r > 0.0) ? (1.0 / r) : 0.0;
    const double maxY = std::max(0.0, yi);

    phi[i] = lambda * (r - xi - yi) - penaltyScale * xi * maxY;

    const double baseP = lambda * (xi * invR - 1.0);
    const double baseQ = lambda * (yi * invR - 1.0);

    if (yi > 0.0) {
      p[i] = baseP - penaltyScale * yi;
      q[i] = baseQ - penaltyScale * xi;
    } else {
      p[i] = baseP;
      q[i] = baseQ;
    }
  }
}

void computePenalizedFbResidualAndGradient(
    const Eigen::MatrixXd& A,
    const Eigen::VectorXd& b,
    const Eigen::VectorXd& x,
    double smoothingEpsilon,
    double lambda,
    Eigen::VectorXd& phi,
    Eigen::VectorXd& grad)
{
  const Eigen::VectorXd y = A * x - b;
  Eigen::VectorXd p;
  Eigen::VectorXd q;
  computePenalizedFbFunction(x, y, smoothingEpsilon, lambda, phi, p, q);
  grad = p.cwiseProduct(phi) + A.transpose() * q.cwiseProduct(phi);
}

double penalizedFbMerit(
    const Eigen::MatrixXd& A,
    const Eigen::VectorXd& b,
    const Eigen::VectorXd& x,
    double smoothingEpsilon,
    double lambda)
{
  Eigen::VectorXd phi(x.size());
  Eigen::VectorXd grad(x.size());
  computePenalizedFbResidualAndGradient(
      A, b, x, smoothingEpsilon, lambda, phi, grad);
  return 0.5 * phi.squaredNorm();
}

bool validateParameters(
    const PenalizedFischerBurmeisterNewtonSolver::Parameters& params,
    std::string* message)
{
  if (!std::isfinite(params.smoothingEpsilon)
      || params.smoothingEpsilon <= 0.0) {
    if (message) {
      *message
          = "Penalized Fischer-Burmeister Newton smoothing_epsilon must be "
            "positive";
    }
    return false;
  }
  if (!std::isfinite(params.lambda) || params.lambda <= 0.0
      || params.lambda > 1.0) {
    if (message) {
      *message = "Penalized Fischer-Burmeister Newton lambda must be in (0, 1]";
    }
    return false;
  }
  if (params.maxLineSearchSteps <= 0) {
    if (message) {
      *message
          = "Penalized Fischer-Burmeister Newton max_line_search_steps must "
            "be positive";
    }
    return false;
  }
  if (!std::isfinite(params.stepReduction) || params.stepReduction <= 0.0
      || params.stepReduction >= 1.0) {
    if (message) {
      *message
          = "Penalized Fischer-Burmeister Newton step_reduction must be in "
            "(0, 1)";
    }
    return false;
  }
  if (!std::isfinite(params.sufficientDecrease)
      || params.sufficientDecrease < 0.0 || params.sufficientDecrease >= 1.0) {
    if (message) {
      *message
          = "Penalized Fischer-Burmeister Newton sufficient_decrease must be "
            "in [0, 1)";
    }
    return false;
  }
  if (!std::isfinite(params.minStep) || params.minStep <= 0.0) {
    if (message) {
      *message
          = "Penalized Fischer-Burmeister Newton min_step must be positive";
    }
    return false;
  }
  if (params.maxGradientDescentWarmStartSteps < 0) {
    if (message) {
      *message
          = "Penalized Fischer-Burmeister Newton "
            "max_gradient_descent_warm_start_steps must be non-negative";
    }
    return false;
  }
  if (params.maxGradientDescentLineSearchSteps <= 0) {
    if (message) {
      *message
          = "Penalized Fischer-Burmeister Newton "
            "max_gradient_descent_line_search_steps must be positive";
    }
    return false;
  }
  if (!std::isfinite(params.gradientDescentStepReduction)
      || params.gradientDescentStepReduction <= 0.0
      || params.gradientDescentStepReduction >= 1.0) {
    if (message) {
      *message
          = "Penalized Fischer-Burmeister Newton "
            "gradient_descent_step_reduction must be in (0, 1)";
    }
    return false;
  }
  if (!std::isfinite(params.gradientDescentSufficientDecrease)
      || params.gradientDescentSufficientDecrease < 0.0
      || params.gradientDescentSufficientDecrease >= 1.0) {
    if (message) {
      *message
          = "Penalized Fischer-Burmeister Newton "
            "gradient_descent_sufficient_decrease must be in [0, 1)";
    }
    return false;
  }
  if (!std::isfinite(params.gradientDescentMinStep)
      || params.gradientDescentMinStep <= 0.0) {
    if (message) {
      *message
          = "Penalized Fischer-Burmeister Newton "
            "gradient_descent_min_step must be positive";
    }
    return false;
  }
  if (params.maxPgsWarmStartIterations < 0) {
    if (message) {
      *message
          = "Penalized Fischer-Burmeister Newton "
            "max_pgs_warm_start_iterations must be non-negative";
    }
    return false;
  }
  if (!std::isfinite(params.pgsWarmStartRelaxation)
      || params.pgsWarmStartRelaxation <= 0.0
      || params.pgsWarmStartRelaxation > 2.0) {
    if (message) {
      *message
          = "Penalized Fischer-Burmeister Newton "
            "pgs_warm_start_relaxation must be in (0, 2]";
    }
    return false;
  }
  return true;
}

void runPgsWarmStart(
    const LcpProblem& problem,
    Eigen::VectorXd& x,
    const LcpOptions& options,
    const PenalizedFischerBurmeisterNewtonSolver::Parameters& params)
{
  if (params.maxPgsWarmStartIterations <= 0) {
    return;
  }

  const double initialMerit = penalizedFbMerit(
      problem.A, problem.b, x, params.smoothingEpsilon, params.lambda);
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
      && penalizedFbMerit(
             problem.A,
             problem.b,
             candidate,
             params.smoothingEpsilon,
             params.lambda)
             < initialMerit) {
    x = candidate;
  }
}

void runGradientDescentWarmStart(
    const Eigen::MatrixXd& A,
    const Eigen::VectorXd& b,
    Eigen::VectorXd& x,
    const PenalizedFischerBurmeisterNewtonSolver::Parameters& params,
    double absTol,
    double relTol)
{
  if (params.maxGradientDescentWarmStartSteps <= 0) {
    return;
  }

  Eigen::VectorXd phi(x.size());
  Eigen::VectorXd grad(x.size());

  for (int iter = 0; iter < params.maxGradientDescentWarmStartSteps; ++iter) {
    computePenalizedFbResidualAndGradient(
        A, b, x, params.smoothingEpsilon, params.lambda, phi, grad);

    const double scale = std::max(
        1.0,
        std::max(
            vectorInfinityNorm(b),
            matrixInfinityNorm(A) * vectorInfinityNorm(x)));
    const double tol = std::max(absTol, relTol * scale);
    const double phiNorm = phi.cwiseAbs().maxCoeff();
    if (phiNorm <= tol || !grad.allFinite()) {
      break;
    }

    const double merit = 0.5 * phi.squaredNorm();
    double step = 1.0;
    bool accepted = false;

    for (int ls = 0; ls < params.maxGradientDescentLineSearchSteps; ++ls) {
      const Eigen::VectorXd xCandidate = (x - step * grad).cwiseMax(0.0);
      Eigen::VectorXd phiCandidate(x.size());
      Eigen::VectorXd gradCandidate(x.size());
      computePenalizedFbResidualAndGradient(
          A,
          b,
          xCandidate,
          params.smoothingEpsilon,
          params.lambda,
          phiCandidate,
          gradCandidate);
      const double candidateMerit = 0.5 * phiCandidate.squaredNorm();
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
PenalizedFischerBurmeisterNewtonSolver::PenalizedFischerBurmeisterNewtonSolver()
{
  mDefaultOptions.maxIterations = 50;
  mDefaultOptions.absoluteTolerance = 1e-6;
  mDefaultOptions.relativeTolerance = 1e-4;
  mDefaultOptions.complementarityTolerance = 1e-6;
  mDefaultOptions.validateSolution = true;
  mDefaultOptions.warmStart = false;
}

//==============================================================================
void PenalizedFischerBurmeisterNewtonSolver::setParameters(
    const Parameters& params)
{
  mParameters = params;
}

//==============================================================================
const PenalizedFischerBurmeisterNewtonSolver::Parameters&
PenalizedFischerBurmeisterNewtonSolver::getParameters() const
{
  return mParameters;
}

//==============================================================================
LcpResult PenalizedFischerBurmeisterNewtonSolver::solve(
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
  Eigen::VectorXd phi(n);
  Eigen::VectorXd p(n);
  Eigen::VectorXd q(n);

  for (int iter = 0; iter < maxIterations; ++iter) {
    iterationsUsed = iter + 1;

    y = A * x - b;
    computePenalizedFbFunction(
        x, y, params->smoothingEpsilon, params->lambda, phi, p, q);

    const double scale = std::max(
        1.0,
        std::max(
            vectorInfinityNorm(b),
            matrixInfinityNorm(A) * vectorInfinityNorm(x)));
    const double tol = std::max(absTol, relTol * scale);

    if (phi.cwiseAbs().maxCoeff() <= tol) {
      converged = true;
      break;
    }

    Eigen::MatrixXd J = A;
    for (int i = 0; i < n; ++i) {
      J.row(i) *= q[i];
    }
    for (int i = 0; i < n; ++i) {
      J(i, i) += p[i];
    }

    const Eigen::VectorXd dx = J.colPivHouseholderQr().solve(-phi);
    if (!dx.allFinite()) {
      result.status = LcpSolverStatus::NumericalError;
      result.message = "Newton solve produced non-finite step";
      return result;
    }

    double step = 1.0;
    bool accepted = false;
    const double phiNorm = phi.cwiseAbs().maxCoeff();

    for (int ls = 0; ls < params->maxLineSearchSteps; ++ls) {
      const Eigen::VectorXd xCandidate = (x + step * dx).cwiseMax(0.0);
      const Eigen::VectorXd yCandidate = A * xCandidate - b;
      Eigen::VectorXd phiCandidate;
      Eigen::VectorXd pCandidate;
      Eigen::VectorXd qCandidate;
      computePenalizedFbFunction(
          xCandidate,
          yCandidate,
          params->smoothingEpsilon,
          params->lambda,
          phiCandidate,
          pCandidate,
          qCandidate);
      const double normCandidate = phiCandidate.cwiseAbs().maxCoeff();
      const double target = phiNorm * (1.0 - params->sufficientDecrease * step);

      if (normCandidate <= target || normCandidate < phiNorm) {
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
std::string PenalizedFischerBurmeisterNewtonSolver::getName() const
{
  return "PenalizedFischerBurmeisterNewton";
}

//==============================================================================
std::string PenalizedFischerBurmeisterNewtonSolver::getCategory() const
{
  return "Newton";
}

} // namespace dart::math
