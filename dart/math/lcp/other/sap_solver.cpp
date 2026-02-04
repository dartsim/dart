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

#include "dart/math/lcp/other/sap_solver.hpp"

#include "dart/math/lcp/lcp_validation.hpp"

#include <Eigen/Cholesky>

#include <algorithm>

#include <cmath>

namespace dart::math {

namespace {

double computeBarrierCost(
    const Eigen::VectorXd& x,
    const Eigen::VectorXd& lo,
    const Eigen::VectorXd& hi,
    double reg)
{
  double cost = 0.0;
  for (Eigen::Index i = 0; i < x.size(); ++i) {
    if (x[i] < lo[i]) {
      const double diff = lo[i] - x[i];
      cost += 0.5 * diff * diff / reg;
    } else if (x[i] > hi[i]) {
      const double diff = x[i] - hi[i];
      cost += 0.5 * diff * diff / reg;
    }
  }
  return cost;
}

void computeBarrierGradient(
    const Eigen::VectorXd& x,
    const Eigen::VectorXd& lo,
    const Eigen::VectorXd& hi,
    double reg,
    Eigen::VectorXd& grad)
{
  for (Eigen::Index i = 0; i < x.size(); ++i) {
    if (x[i] < lo[i]) {
      grad[i] += (x[i] - lo[i]) / reg;
    } else if (x[i] > hi[i]) {
      grad[i] += (x[i] - hi[i]) / reg;
    }
  }
}

void addBarrierHessianDiagonal(
    const Eigen::VectorXd& x,
    const Eigen::VectorXd& lo,
    const Eigen::VectorXd& hi,
    double reg,
    Eigen::VectorXd& diagAdd)
{
  for (Eigen::Index i = 0; i < x.size(); ++i) {
    if (x[i] < lo[i] || x[i] > hi[i]) {
      diagAdd[i] += 1.0 / reg;
    }
  }
}

} // namespace

SapSolver::SapSolver()
{
  mDefaultOptions.maxIterations = 50;
  mDefaultOptions.absoluteTolerance = 1e-6;
  mDefaultOptions.relativeTolerance = 1e-4;
  // SAP uses a penalty/barrier method that produces approximate solutions.
  // Strict LCP complementarity validation would fail due to regularization.
  mDefaultOptions.validateSolution = false;
  mDefaultOptions.warmStart = true;
}

void SapSolver::setParameters(const Parameters& params)
{
  mParameters = params;
}

const SapSolver::Parameters& SapSolver::getParameters() const
{
  return mParameters;
}

LcpResult SapSolver::solve(
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

  const double reg = params->regularization;
  const double armijo = params->armijosParameter;
  const double backtrack = params->backtrackingFactor;
  const int maxLs = params->maxLineSearchIterations;

  Eigen::VectorXd loEff = lo;
  Eigen::VectorXd hiEff = hi;
  for (Eigen::Index i = 0; i < n; ++i) {
    if (findex[i] >= 0) {
      loEff[i] = -std::numeric_limits<double>::infinity();
      hiEff[i] = std::numeric_limits<double>::infinity();
    }
  }

  if (x.size() != n) {
    x = Eigen::VectorXd::Zero(n);
  } else if (!options.warmStart) {
    x.setZero();
  }

  for (Eigen::Index i = 0; i < n; ++i) {
    x[i] = std::clamp(x[i], loEff[i], hiEff[i]);
  }

  Eigen::VectorXd grad(n);
  Eigen::VectorXd diagAdd(n);
  Eigen::VectorXd dx(n);
  Eigen::VectorXd xNew(n);

  int iterationsUsed = 0;
  bool converged = false;

  for (int iter = 0; iter < maxIterations; ++iter) {
    ++iterationsUsed;

    for (Eigen::Index i = 0; i < n; ++i) {
      if (findex[i] >= 0) {
        const double fricLimit = std::abs(hi[i] * x[findex[i]]);
        loEff[i] = -fricLimit;
        hiEff[i] = fricLimit;
      }
    }

    // Cost: L(x) = 0.5*x'*A*x - b'*x + barrier(x)
    const double quadCost = 0.5 * x.dot(A * x) - b.dot(x);
    const double barrierCost = computeBarrierCost(x, loEff, hiEff, reg);
    const double cost = quadCost + barrierCost;

    // Gradient: grad = A*x - b + barrier_grad
    grad = A * x - b;
    computeBarrierGradient(x, loEff, hiEff, reg, grad);

    const double gradNorm = grad.lpNorm<Eigen::Infinity>();
    if (gradNorm < absTolerance) {
      converged = true;
      break;
    }

    // Hessian: H = A + barrier_hessian_diag
    diagAdd.setZero();
    addBarrierHessianDiagonal(x, loEff, hiEff, reg, diagAdd);

    Eigen::MatrixXd H = A;
    H.diagonal() += diagAdd;

    // Solve for Newton direction
    Eigen::LLT<Eigen::MatrixXd> llt(H);
    if (llt.info() != Eigen::Success) {
      H.diagonal().array() += 1e-6;
      llt.compute(H);
    }
    dx = llt.solve(-grad);

    // Armijo backtracking line search
    double alpha = 1.0;
    const double gradDotDx = grad.dot(dx);

    for (int ls = 0; ls < maxLs; ++ls) {
      xNew = x + alpha * dx;

      for (Eigen::Index i = 0; i < n; ++i) {
        if (findex[i] >= 0) {
          const double fricLimit = std::abs(hi[i] * xNew[findex[i]]);
          loEff[i] = -fricLimit;
          hiEff[i] = fricLimit;
        }
      }

      const double newQuadCost = 0.5 * xNew.dot(A * xNew) - b.dot(xNew);
      const double newBarrierCost = computeBarrierCost(xNew, loEff, hiEff, reg);
      const double newCost = newQuadCost + newBarrierCost;

      if (newCost <= cost + armijo * alpha * gradDotDx) {
        break;
      }
      alpha *= backtrack;
    }

    x = x + alpha * dx;
  }

  for (Eigen::Index i = 0; i < n; ++i) {
    if (findex[i] >= 0) {
      const double fricLimit = std::abs(hi[i] * x[findex[i]]);
      loEff[i] = -fricLimit;
      hiEff[i] = fricLimit;
    } else {
      loEff[i] = lo[i];
      hiEff[i] = hi[i];
    }
    x[i] = std::clamp(x[i], loEff[i], hiEff[i]);
  }

  Eigen::VectorXd wVec = A * x - b;
  result.iterations = iterationsUsed;

  Eigen::VectorXd loEffFinal;
  Eigen::VectorXd hiEffFinal;
  std::string boundsMessage;
  const bool boundsOk = detail::computeEffectiveBounds(
      lo, hi, findex, x, loEffFinal, hiEffFinal, &boundsMessage);
  if (!boundsOk) {
    result.status = LcpSolverStatus::InvalidProblem;
    result.message = boundsMessage;
    return result;
  }

  result.residual
      = detail::naturalResidualInfinityNorm(x, wVec, loEffFinal, hiEffFinal);
  result.complementarity = detail::complementarityInfinityNorm(
      x, wVec, loEffFinal, hiEffFinal, absTolerance);

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
        x, wVec, loEffFinal, hiEffFinal, validationTol, &validationMessage);
    result.validated = true;
    if (!valid) {
      result.status = LcpSolverStatus::NumericalError;
      result.message = validationMessage.empty() ? "Solution validation failed"
                                                 : validationMessage;
    }
  }

  return result;
}

std::string SapSolver::getName() const
{
  return "Sap";
}

std::string SapSolver::getCategory() const
{
  return "Other";
}

} // namespace dart::math
