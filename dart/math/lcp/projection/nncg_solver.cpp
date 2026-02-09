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

#include "dart/math/lcp/projection/nncg_solver.hpp"

#include "dart/math/lcp/lcp_validation.hpp"
#include "dart/math/lcp/projection/pgs_solver.hpp"

#include <algorithm>
#include <iterator>
#include <limits>

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

} // namespace

//==============================================================================
NncgSolver::NncgSolver()
{
  mDefaultOptions.maxIterations = 200;
  mDefaultOptions.absoluteTolerance = 1e-6;
  mDefaultOptions.relativeTolerance = 1e-3;
  mDefaultOptions.complementarityTolerance = 1e-6;
  mDefaultOptions.relaxation = 1.0;
  mDefaultOptions.validateSolution = true;
  mDefaultOptions.warmStart = false;
}

//==============================================================================
void NncgSolver::setParameters(const Parameters& params)
{
  mParameters = params;
}

//==============================================================================
const NncgSolver::Parameters& NncgSolver::getParameters() const
{
  return mParameters;
}

//==============================================================================
LcpResult NncgSolver::solve(
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

  if (x.size() != n || !options.warmStart || !x.allFinite()) {
    x = Eigen::VectorXd::Zero(n);
  }

  const int maxIterations = std::max(
      1,
      (options.maxIterations > 0) ? options.maxIterations
                                  : mDefaultOptions.maxIterations);
  const double absTol = (options.absoluteTolerance > 0.0)
                            ? options.absoluteTolerance
                            : mDefaultOptions.absoluteTolerance;
  const double relTol = (options.relativeTolerance > 0.0)
                            ? options.relativeTolerance
                            : mDefaultOptions.relativeTolerance;
  const double compTolOpt = (options.complementarityTolerance > 0.0)
                                ? options.complementarityTolerance
                                : mDefaultOptions.complementarityTolerance;

  const Parameters* params
      = options.customOptions
            ? static_cast<const Parameters*>(options.customOptions)
            : &mParameters;

  if (params->pgsIterations <= 0) {
    result.status = LcpSolverStatus::InvalidProblem;
    result.message = "NNCG requires at least one PGS iteration";
    return result;
  }

  if (!std::isfinite(params->restartThreshold)
      || params->restartThreshold <= 0.0) {
    result.status = LcpSolverStatus::InvalidProblem;
    result.message = "NNCG restart threshold must be positive";
    return result;
  }

  const int restartInterval = std::max(0, params->restartInterval);

  PgsSolver pgs;
  LcpOptions pgsOptions = pgs.getDefaultOptions();
  pgsOptions.maxIterations = params->pgsIterations;
  pgsOptions.absoluteTolerance = absTol;
  pgsOptions.relativeTolerance = relTol;
  pgsOptions.complementarityTolerance = compTolOpt;
  pgsOptions.relaxation = options.relaxation;
  pgsOptions.warmStart = true;
  pgsOptions.validateSolution = false;
  pgsOptions.customOptions = nullptr;

  auto applyPgs = [&](Eigen::VectorXd& xValue) -> LcpResult {
    return pgs.solve(problem, xValue, pgsOptions);
  };

  Eigen::VectorXd xPgs = x;
  LcpResult pgsResult = applyPgs(xPgs);
  if (pgsResult.status == LcpSolverStatus::InvalidProblem
      || pgsResult.status == LcpSolverStatus::NumericalError) {
    return pgsResult;
  }

  Eigen::VectorXd r = xPgs - x;
  x = xPgs;

  Eigen::VectorXd w;
  Eigen::VectorXd loEff;
  Eigen::VectorXd hiEff;
  double tol = 0.0;
  double compTol = 0.0;
  double residual = 0.0;
  double complementarity = 0.0;

  auto updateMetrics = [&](const Eigen::VectorXd& xValue) -> bool {
    w = A * xValue - b;
    std::string boundsMessage;
    if (!detail::computeEffectiveBounds(
            lo, hi, findex, xValue, loEff, hiEff, &boundsMessage)) {
      result.status = LcpSolverStatus::InvalidProblem;
      result.message = boundsMessage;
      return false;
    }

    const double scale = std::max(
        1.0,
        std::max(
            vectorInfinityNorm(b),
            matrixInfinityNorm(A) * vectorInfinityNorm(xValue)));
    tol = std::max(absTol, relTol * scale);
    compTol = std::max(compTolOpt, relTol * scale);
    residual = detail::naturalResidualInfinityNorm(xValue, w, loEff, hiEff);
    complementarity
        = detail::complementarityInfinityNorm(xValue, w, loEff, hiEff, compTol);
    return true;
  };

  bool converged = false;
  int iterationsUsed = 0;

  if (!updateMetrics(x)) {
    return result;
  }

  if (residual <= tol && complementarity <= compTol) {
    converged = true;
  }

  Eigen::VectorXd p = -r;

  for (int iter = 0; iter < maxIterations && !converged; ++iter) {
    iterationsUsed = iter + 1;

    Eigen::VectorXd xCandidate = x + p;
    pgsResult = applyPgs(xCandidate);
    if (pgsResult.status == LcpSolverStatus::InvalidProblem
        || pgsResult.status == LcpSolverStatus::NumericalError) {
      return pgsResult;
    }

    Eigen::VectorXd rNew = xCandidate;
    pgsResult = applyPgs(rNew);
    if (pgsResult.status == LcpSolverStatus::InvalidProblem
        || pgsResult.status == LcpSolverStatus::NumericalError) {
      return pgsResult;
    }
    rNew -= xCandidate;

    const double rNormSq = r.squaredNorm();
    const double rNewNormSq = rNew.squaredNorm();
    double beta = 0.0;
    if (rNormSq > 0.0) {
      beta = rNewNormSq / rNormSq;
    }

    bool restart = false;
    if (!std::isfinite(beta) || beta > params->restartThreshold) {
      restart = true;
    }
    if (restartInterval > 0 && ((iter + 1) % restartInterval == 0)) {
      restart = true;
    }

    if (restart) {
      p = -rNew;
    } else {
      p = -rNew + beta * p;
    }

    x = xCandidate;
    r = rNew;

    if (!updateMetrics(x)) {
      return result;
    }

    if (residual <= tol && complementarity <= compTol) {
      converged = true;
    }
  }

  result.iterations = iterationsUsed;
  result.residual = residual;
  result.complementarity = complementarity;

  if (converged) {
    result.status = LcpSolverStatus::Success;
  } else {
    result.status = LcpSolverStatus::MaxIterations;
  }

  if (options.validateSolution && result.status == LcpSolverStatus::Success) {
    const double validationTol = std::max(tol, compTol);
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
std::string NncgSolver::getName() const
{
  return "NNCG";
}

//==============================================================================
std::string NncgSolver::getCategory() const
{
  return "Projection";
}

} // namespace dart::math
