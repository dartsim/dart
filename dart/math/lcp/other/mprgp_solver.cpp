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

#include "dart/math/lcp/other/mprgp_solver.hpp"

#include "dart/math/lcp/lcp_validation.hpp"
#include "dart/math/lcp/pivoting/dantzig_solver.hpp"

#include <Eigen/Cholesky>
#include <Eigen/Dense>

#include <algorithm>
#include <iterator>
#include <limits>
#include <string>
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

bool isStandardLcp(
    const Eigen::VectorXd& lo,
    const Eigen::VectorXd& hi,
    const Eigen::VectorXi& findex,
    double absTol)
{
  return (lo.array().abs().maxCoeff() <= absTol)
         && (hi.array() == std::numeric_limits<double>::infinity()).all()
         && (findex.array() < 0).all();
}

bool isSymmetric(const Eigen::MatrixXd& A, double tol)
{
  if (A.rows() != A.cols()) {
    return false;
  }

  return (A - A.transpose()).cwiseAbs().maxCoeff() <= tol;
}

} // namespace

//==============================================================================
MprgpSolver::MprgpSolver()
{
  mDefaultOptions.maxIterations = 200;
  mDefaultOptions.absoluteTolerance = 1e-6;
  mDefaultOptions.relativeTolerance = 1e-4;
  mDefaultOptions.complementarityTolerance = 1e-6;
  mDefaultOptions.validateSolution = true;
  mDefaultOptions.warmStart = false;
}

//==============================================================================
void MprgpSolver::setParameters(const Parameters& params)
{
  mParameters = params;
}

//==============================================================================
const MprgpSolver::Parameters& MprgpSolver::getParameters() const
{
  return mParameters;
}

//==============================================================================
LcpResult MprgpSolver::solve(
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
  if (!isStandardLcp(lo, hi, findex, absTol)) {
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
  const double compTolOpt = (options.complementarityTolerance > 0.0)
                                ? options.complementarityTolerance
                                : mDefaultOptions.complementarityTolerance;

  const Parameters* params
      = options.customOptions
            ? static_cast<const Parameters*>(options.customOptions)
            : &mParameters;

  if (!std::isfinite(params->epsilonForDivision)
      || params->epsilonForDivision <= 0.0) {
    result.status = LcpSolverStatus::InvalidProblem;
    result.message = "Division epsilon must be positive";
    return result;
  }

  if (!std::isfinite(params->symmetryTolerance)
      || params->symmetryTolerance < 0.0) {
    result.status = LcpSolverStatus::InvalidProblem;
    result.message = "Symmetry tolerance must be non-negative";
    return result;
  }

  if (!isSymmetric(A, params->symmetryTolerance)) {
    DantzigSolver fallback;
    return fallback.solve(problem, x, options);
  }

  if (params->checkPositiveDefinite) {
    Eigen::LLT<Eigen::MatrixXd> llt(A);
    if (llt.info() != Eigen::Success) {
      DantzigSolver fallback;
      return fallback.solve(problem, x, options);
    }
  }

  Eigen::VectorXd w;
  Eigen::VectorXd loEff;
  Eigen::VectorXd hiEff;
  double tol = 0.0;
  double compTol = 0.0;
  double residual = 0.0;
  double complementarity = 0.0;

  auto updateMetrics = [&]() -> bool {
    w = A * x - b;
    std::string boundsMessage;
    if (!detail::computeEffectiveBounds(
            lo, hi, findex, x, loEff, hiEff, &boundsMessage)) {
      result.status = LcpSolverStatus::InvalidProblem;
      result.message = boundsMessage;
      return false;
    }

    const double scale = std::max(
        1.0,
        std::max(
            vectorInfinityNorm(b),
            matrixInfinityNorm(A) * vectorInfinityNorm(x)));
    tol = std::max(absTol, relTol * scale);
    compTol = std::max(compTolOpt, relTol * scale);
    residual = detail::naturalResidualInfinityNorm(x, w, loEff, hiEff);
    complementarity
        = detail::complementarityInfinityNorm(x, w, loEff, hiEff, compTol);
    return true;
  };

  if (!updateMetrics()) {
    return result;
  }

  bool converged = (residual <= tol && complementarity <= compTol);
  int iterationsUsed = 0;

  Eigen::VectorXd pg(n);
  Eigen::VectorXd p(n);
  Eigen::VectorXd pPrev(n);
  Eigen::VectorXd pgPrev(n);
  Eigen::VectorXd Ap(n);
  std::vector<int> active(n, 0);
  std::vector<int> activePrev(n, 0);
  bool resetDirection = true;

  for (int iter = 0; iter < maxIterations && !converged; ++iter) {
    iterationsUsed = iter + 1;

    w = A * x - b;
    pg = w;

    for (int i = 0; i < n; ++i) {
      if (x[i] <= 0.0 && w[i] > 0.0) {
        active[i] = 1;
        x[i] = 0.0;
        pg[i] = 0.0;
      } else {
        active[i] = 0;
      }
    }

    if (pg.cwiseAbs().maxCoeff() <= params->epsilonForDivision) {
      result.status = LcpSolverStatus::Failed;
      result.message = "Projected gradient vanished before convergence";
      return result;
    }

    const bool activeChanged = (active != activePrev);

    if (resetDirection || activeChanged) {
      p = -pg;
    } else {
      const double denom = pgPrev.squaredNorm();
      const double beta = (denom > params->epsilonForDivision)
                              ? pg.dot(pg - pgPrev) / denom
                              : 0.0;
      p = -pg + beta * pPrev;
    }

    for (int i = 0; i < n; ++i) {
      if (active[i]) {
        p[i] = 0.0;
      }
    }

    Ap = A * p;
    double denom = p.dot(Ap);
    if (!std::isfinite(denom) || denom <= params->epsilonForDivision) {
      p = -pg;
      Ap = A * p;
      denom = p.dot(Ap);
      if (!std::isfinite(denom) || denom <= params->epsilonForDivision) {
        result.status = LcpSolverStatus::NumericalError;
        result.message = "MPRGP direction is not positive definite";
        return result;
      }
    }

    const double stepNumerator = -pg.dot(p);
    double alpha = stepNumerator / denom;
    if (!std::isfinite(alpha) || alpha <= 0.0) {
      resetDirection = true;
      continue;
    }

    double alphaMax = std::numeric_limits<double>::infinity();
    for (int i = 0; i < n; ++i) {
      if (p[i] < 0.0) {
        alphaMax = std::min(alphaMax, -x[i] / p[i]);
      }
    }

    const double step = std::min(alpha, alphaMax);
    if (!std::isfinite(step) || step <= 0.0) {
      resetDirection = true;
      continue;
    }

    x += step * p;
    x = x.cwiseMax(0.0);

    const bool hitBound = std::isfinite(alphaMax) && (step >= alphaMax - 1e-12);
    resetDirection = hitBound;

    pPrev = p;
    pgPrev = pg;
    activePrev = active;

    if (!updateMetrics()) {
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
std::string MprgpSolver::getName() const
{
  return "MPRGP";
}

//==============================================================================
std::string MprgpSolver::getCategory() const
{
  return "Other";
}

} // namespace dart::math
