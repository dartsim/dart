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

#include "dart/math/lcp/other/interior_point_solver.hpp"

#include "dart/math/lcp/lcp_validation.hpp"
#include "dart/math/lcp/pivoting/dantzig_solver.hpp"

#include <Eigen/Dense>

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

double computeStepSize(
    const Eigen::VectorXd& x, const Eigen::VectorXd& dx, double stepScale)
{
  double alpha = 1.0;
  for (Eigen::Index i = 0; i < x.size(); ++i) {
    if (dx[i] < 0.0) {
      alpha = std::min(alpha, -x[i] / dx[i]);
    }
  }
  alpha = std::min(1.0, stepScale * alpha);
  if (!std::isfinite(alpha)) {
    return 0.0;
  }
  return alpha;
}

double clampValue(double value, double lo, double hi)
{
  return std::min(std::max(value, lo), hi);
}

} // namespace

//==============================================================================
InteriorPointSolver::InteriorPointSolver()
{
  mDefaultOptions.maxIterations = 50;
  mDefaultOptions.absoluteTolerance = 1e-6;
  mDefaultOptions.relativeTolerance = 1e-4;
  mDefaultOptions.complementarityTolerance = 1e-6;
  mDefaultOptions.validateSolution = true;
  mDefaultOptions.warmStart = false;
}

//==============================================================================
void InteriorPointSolver::setParameters(const Parameters& params)
{
  mParameters = params;
}

//==============================================================================
const InteriorPointSolver::Parameters& InteriorPointSolver::getParameters()
    const
{
  return mParameters;
}

//==============================================================================
LcpResult InteriorPointSolver::solve(
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

  const double absTol = (options.absoluteTolerance > 0.0)
                            ? options.absoluteTolerance
                            : mDefaultOptions.absoluteTolerance;
  const double relTol = (options.relativeTolerance > 0.0)
                            ? options.relativeTolerance
                            : mDefaultOptions.relativeTolerance;
  const double compTolOpt = (options.complementarityTolerance > 0.0)
                                ? options.complementarityTolerance
                                : mDefaultOptions.complementarityTolerance;
  const bool standardBounds
      = (lo.array().abs().maxCoeff() <= absTol)
        && (hi.array() == std::numeric_limits<double>::infinity()).all()
        && (findex.array() < 0).all();
  if (!standardBounds) {
    DantzigSolver fallback;
    return fallback.solve(problem, x, options);
  }

  int maxIterations = (options.maxIterations > 0)
                          ? options.maxIterations
                          : mDefaultOptions.maxIterations;
  if (maxIterations < 1) {
    maxIterations = 1;
  }

  const Parameters* params
      = options.customOptions
            ? static_cast<const Parameters*>(options.customOptions)
            : &mParameters;
  const double sigma = clampValue(params->sigma, 1e-6, 0.999);
  const double stepScale = clampValue(params->stepScale, 1e-3, 1.0);

  const double initScale = std::max(1.0, vectorInfinityNorm(b));
  const double minVal = std::max(1e-8, std::min(1.0, 1e-6 * initScale));

  if (x.size() != n || !options.warmStart || !x.allFinite()) {
    x = Eigen::VectorXd::Constant(n, minVal);
  } else {
    x = x.cwiseMax(minVal);
  }

  Eigen::VectorXd w = (A * x - b).cwiseMax(minVal);

  Eigen::VectorXd loEff = Eigen::VectorXd::Zero(n);
  Eigen::VectorXd hiEff
      = Eigen::VectorXd::Constant(n, std::numeric_limits<double>::infinity());

  bool converged = false;
  bool numericalError = false;
  int iterationsUsed = 0;

  for (int iter = 0; iter < maxIterations; ++iter) {
    iterationsUsed = iter + 1;

    const Eigen::VectorXd r = A * x - w - b;
    const double mu = sigma * (x.dot(w) / static_cast<double>(n));
    const Eigen::VectorXd c
        = x.cwiseProduct(w) - Eigen::VectorXd::Constant(n, mu);

    const Eigen::VectorXd wEval = A * x - b;
    const double scale = std::max(
        1.0,
        std::max(
            vectorInfinityNorm(b),
            matrixInfinityNorm(A) * vectorInfinityNorm(x)));
    const double tol = std::max(absTol, relTol * scale);
    const double compTol = std::max(compTolOpt, relTol * scale);

    result.residual
        = detail::naturalResidualInfinityNorm(x, wEval, loEff, hiEff);
    result.complementarity
        = detail::complementarityInfinityNorm(x, wEval, loEff, hiEff, compTol);

    if (result.residual <= tol && result.complementarity <= compTol) {
      converged = true;
      break;
    }

    Eigen::MatrixXd M = x.asDiagonal() * A;
    M.diagonal().array() += w.array();
    const Eigen::VectorXd rhs = -(c + x.cwiseProduct(r));

    Eigen::FullPivLU<Eigen::MatrixXd> lu(M);
    if (!lu.isInvertible()) {
      numericalError = true;
      result.message = "Interior-point solver: singular Newton system";
      break;
    }

    const Eigen::VectorXd dx = lu.solve(rhs);
    if (!dx.allFinite()) {
      numericalError = true;
      result.message = "Interior-point solver: invalid Newton step";
      break;
    }

    const Eigen::VectorXd dw = A * dx + r;
    if (!dw.allFinite()) {
      numericalError = true;
      result.message = "Interior-point solver: invalid dual step";
      break;
    }

    const double alphaPrimal = computeStepSize(x, dx, stepScale);
    const double alphaDual = computeStepSize(w, dw, stepScale);
    const double alpha = std::min(alphaPrimal, alphaDual);
    if (alpha <= 0.0 || !std::isfinite(alpha)) {
      numericalError = true;
      result.message = "Interior-point solver: invalid step length";
      break;
    }

    x += alpha * dx;
    w += alpha * dw;
    x = x.cwiseMax(minVal);
    w = w.cwiseMax(minVal);
  }

  result.iterations = iterationsUsed;

  if (numericalError) {
    result.status = LcpSolverStatus::NumericalError;
    return result;
  }

  const Eigen::VectorXd wEval = A * x - b;
  const double scale = std::max(
      1.0,
      std::max(
          vectorInfinityNorm(b),
          matrixInfinityNorm(A) * vectorInfinityNorm(x)));
  const double tol = std::max(absTol, relTol * scale);
  const double compTol = std::max(compTolOpt, relTol * scale);

  result.residual = detail::naturalResidualInfinityNorm(x, wEval, loEff, hiEff);
  result.complementarity
      = detail::complementarityInfinityNorm(x, wEval, loEff, hiEff, compTol);

  if (converged) {
    result.status = LcpSolverStatus::Success;
  } else {
    result.status = LcpSolverStatus::MaxIterations;
  }

  if (options.validateSolution && result.status == LcpSolverStatus::Success) {
    const double validationTol = std::max(tol, compTol);
    std::string validationMessage;
    const bool feasible = detail::validateSolution(
        x, wEval, loEff, hiEff, validationTol, &validationMessage);
    result.validated = true;
    if (!feasible) {
      result.status = LcpSolverStatus::NumericalError;
      result.message = validationMessage.empty() ? "Solution validation failed"
                                                 : validationMessage;
    }
  }

  return result;
}

//==============================================================================
std::string InteriorPointSolver::getName() const
{
  return "InteriorPoint";
}

//==============================================================================
std::string InteriorPointSolver::getCategory() const
{
  return "Other";
}

} // namespace dart::math
