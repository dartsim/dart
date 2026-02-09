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

#include "dart/math/lcp/other/admm_solver.hpp"

#include "dart/math/lcp/lcp_validation.hpp"

#include <Eigen/Cholesky>

#include <algorithm>

#include <cmath>

namespace dart::math {

AdmmSolver::AdmmSolver()
{
  mDefaultOptions.maxIterations = 200;
  mDefaultOptions.absoluteTolerance = 1e-6;
  mDefaultOptions.relativeTolerance = 1e-4;
  mDefaultOptions.validateSolution = true;
  mDefaultOptions.warmStart = true;
}

void AdmmSolver::setParameters(const Parameters& params)
{
  mParameters = params;
}

const AdmmSolver::Parameters& AdmmSolver::getParameters() const
{
  return mParameters;
}

LcpResult AdmmSolver::solve(
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

  const auto n = b.size();
  if (n == 0) {
    x.resize(0);
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

  const int maxIterations = std::max(
      1,
      (options.maxIterations > 0) ? options.maxIterations
                                  : mDefaultOptions.maxIterations);
  const double absTolerance = (options.absoluteTolerance > 0)
                                  ? options.absoluteTolerance
                                  : mDefaultOptions.absoluteTolerance;

  double rho = params->rhoInit;
  const double muProx = params->muProx;

  if (x.size() != n) {
    x = Eigen::VectorXd::Zero(n);
  } else if (!options.warmStart) {
    x.setZero();
  }

  Eigen::VectorXd z = x;
  Eigen::VectorXd y = Eigen::VectorXd::Zero(n);
  Eigen::VectorXd xPrev = x;
  Eigen::VectorXd zPrev = z;

  Eigen::MatrixXd M = A;
  M.diagonal().array() += rho + muProx;

  Eigen::LLT<Eigen::MatrixXd> llt(M);
  if (llt.info() != Eigen::Success) {
    result.status = LcpSolverStatus::NumericalError;
    result.message = "Cholesky factorization failed";
    return result;
  }

  int iterationsUsed = 0;
  bool converged = false;

  for (int iter = 0; iter < maxIterations; ++iter) {
    ++iterationsUsed;
    xPrev = x;
    zPrev = z;

    // x-update: x = (A + (rho + muProx)*I)^{-1} * (rho*z - y + b)
    Eigen::VectorXd rhs = rho * z - y + b;
    x = llt.solve(rhs);

    // z-update: z = clamp(x + y/rho, lo, hi) with findex handling
    Eigen::VectorXd xTilde = x + y / rho;
    for (Eigen::Index i = 0; i < n; ++i) {
      double loVal = lo[i];
      double hiVal = hi[i];
      if (findex[i] >= 0) {
        const double fricLimit = std::abs(hi[i] * z[findex[i]]);
        loVal = -fricLimit;
        hiVal = fricLimit;
      }
      z[i] = std::clamp(xTilde[i], loVal, hiVal);
    }

    // y-update (dual): y = y + rho*(x - z)
    y += rho * (x - z);

    const double primalRes = (x - z).lpNorm<Eigen::Infinity>();
    const double dualRes = rho * (z - zPrev).lpNorm<Eigen::Infinity>();

    if (primalRes < absTolerance && dualRes < absTolerance) {
      converged = true;
      break;
    }

    if (params->adaptiveRho && iter > 0 && (iter % 10) == 0) {
      const double ratio = primalRes / (dualRes + 1e-10);
      if (ratio > params->adaptiveRhoTolerance) {
        rho *= 2.0;
        y *= 0.5;
        M = A;
        M.diagonal().array() += rho + muProx;
        llt.compute(M);
      } else if (ratio < 1.0 / params->adaptiveRhoTolerance) {
        rho *= 0.5;
        y *= 2.0;
        M = A;
        M.diagonal().array() += rho + muProx;
        llt.compute(M);
      }
    }
  }

  x = z;

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

std::string AdmmSolver::getName() const
{
  return "Admm";
}

std::string AdmmSolver::getCategory() const
{
  return "Other";
}

} // namespace dart::math
