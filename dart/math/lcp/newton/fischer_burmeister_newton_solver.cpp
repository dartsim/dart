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

#include "dart/math/lcp/newton/fischer_burmeister_newton_solver.hpp"

#include "dart/math/lcp/lcp_validation.hpp"
#include "dart/math/lcp/pivoting/dantzig_solver.hpp"

#include <Eigen/QR>

#include <algorithm>
#include <iterator>
#include <limits>

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

void computeFbFunction(
    const Eigen::VectorXd& x,
    const Eigen::VectorXd& y,
    double epsilon,
    Eigen::VectorXd& phi,
    Eigen::VectorXd& p,
    Eigen::VectorXd& q)
{
  const Eigen::Index n = x.size();
  phi.resize(n);
  p.resize(n);
  q.resize(n);

  for (Eigen::Index i = 0; i < n; ++i) {
    const double xi = x[i];
    const double yi = y[i];
    const double r = std::sqrt(xi * xi + yi * yi + epsilon * epsilon);
    const double invR = (r > 0.0) ? (1.0 / r) : 0.0;
    phi[i] = r - xi - yi;
    p[i] = xi * invR - 1.0;
    q[i] = yi * invR - 1.0;
  }
}

} // namespace

//==============================================================================
FischerBurmeisterNewtonSolver::FischerBurmeisterNewtonSolver()
{
  mDefaultOptions.maxIterations = 50;
  mDefaultOptions.absoluteTolerance = 1e-6;
  mDefaultOptions.relativeTolerance = 1e-4;
  mDefaultOptions.complementarityTolerance = 1e-6;
  mDefaultOptions.validateSolution = true;
  mDefaultOptions.warmStart = false;
}

//==============================================================================
void FischerBurmeisterNewtonSolver::setParameters(const Parameters& params)
{
  mParameters = params;
}

//==============================================================================
const FischerBurmeisterNewtonSolver::Parameters&
FischerBurmeisterNewtonSolver::getParameters() const
{
  return mParameters;
}

//==============================================================================
LcpResult FischerBurmeisterNewtonSolver::solve(
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
  Eigen::VectorXd phi(n);
  Eigen::VectorXd p(n);
  Eigen::VectorXd q(n);

  for (int iter = 0; iter < maxIterations; ++iter) {
    iterationsUsed = iter + 1;

    y = A * x - b;
    computeFbFunction(x, y, params->smoothingEpsilon, phi, p, q);

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
      computeFbFunction(
          xCandidate,
          yCandidate,
          params->smoothingEpsilon,
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
std::string FischerBurmeisterNewtonSolver::getName() const
{
  return "FischerBurmeisterNewton";
}

//==============================================================================
std::string FischerBurmeisterNewtonSolver::getCategory() const
{
  return "Newton";
}

} // namespace dart::math
