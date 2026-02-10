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

#include "dart/math/lcp/newton/boxed_semi_smooth_newton_solver.hpp"

#include "dart/math/lcp/lcp_validation.hpp"

#include <Eigen/Cholesky>

#include <algorithm>

#include <cmath>

namespace dart::math {

namespace {

enum class ConstraintState
{
  AtLower,
  AtUpper,
  Interior
};

ConstraintState classifyPoint(
    double x, double grad, double lo, double hi, double tol)
{
  const bool atLo = std::isfinite(lo) && std::abs(x - lo) <= tol;
  const bool atHi = std::isfinite(hi) && std::abs(x - hi) <= tol;

  if (atLo && grad > tol) {
    return ConstraintState::AtLower;
  }
  if (atHi && grad < -tol) {
    return ConstraintState::AtUpper;
  }
  return ConstraintState::Interior;
}

double project(double val, double lo, double hi)
{
  if (std::isfinite(lo) && val < lo) {
    return lo;
  }
  if (std::isfinite(hi) && val > hi) {
    return hi;
  }
  return val;
}

Eigen::VectorXd projectVector(
    const Eigen::VectorXd& x,
    const Eigen::VectorXd& lo,
    const Eigen::VectorXd& hi)
{
  Eigen::VectorXd result(x.size());
  for (Eigen::Index i = 0; i < x.size(); ++i) {
    result[i] = project(x[i], lo[i], hi[i]);
  }
  return result;
}

Eigen::VectorXd computeNaturalResidual(
    const Eigen::VectorXd& x,
    const Eigen::VectorXd& w,
    const Eigen::VectorXd& lo,
    const Eigen::VectorXd& hi)
{
  const Eigen::VectorXd candidate = x - w;
  const Eigen::VectorXd projected = projectVector(candidate, lo, hi);
  return x - projected;
}

} // namespace

BoxedSemiSmoothNewtonSolver::BoxedSemiSmoothNewtonSolver()
{
  mDefaultOptions.maxIterations = 50;
  mDefaultOptions.absoluteTolerance = 1e-8;
  mDefaultOptions.relativeTolerance = 1e-6;
  mDefaultOptions.complementarityTolerance = 1e-8;
  mDefaultOptions.validateSolution = true;
  mDefaultOptions.warmStart = true;
}

void BoxedSemiSmoothNewtonSolver::setParameters(const Parameters& params)
{
  mParameters = params;
}

const BoxedSemiSmoothNewtonSolver::Parameters&
BoxedSemiSmoothNewtonSolver::getParameters() const
{
  return mParameters;
}

LcpResult BoxedSemiSmoothNewtonSolver::solve(
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
  const double absTol = (options.absoluteTolerance > 0)
                            ? options.absoluteTolerance
                            : mDefaultOptions.absoluteTolerance;
  const double compTol = (options.complementarityTolerance > 0)
                             ? options.complementarityTolerance
                             : mDefaultOptions.complementarityTolerance;

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
  x = projectVector(x, loEff, hiEff);

  Eigen::VectorXd w(n);
  Eigen::VectorXd H(n);
  Eigen::VectorXd dx(n);
  Eigen::MatrixXd J(n, n);

  int iterationsUsed = 0;
  bool converged = false;
  bool lineSearchFailed = false;

  for (int iter = 0; iter < maxIterations; ++iter) {
    ++iterationsUsed;

    for (Eigen::Index i = 0; i < n; ++i) {
      if (findex[i] >= 0) {
        const double fricLimit = std::abs(hi[i] * x[findex[i]]);
        loEff[i] = -fricLimit;
        hiEff[i] = fricLimit;
      }
    }

    w = A * x - b;
    H = computeNaturalResidual(x, w, loEff, hiEff);

    const double Hnorm = H.lpNorm<Eigen::Infinity>();
    if (Hnorm < absTol) {
      converged = true;
      break;
    }

    J.setZero();
    for (Eigen::Index i = 0; i < n; ++i) {
      const double candidate = x[i] - w[i];
      const ConstraintState state
          = classifyPoint(candidate, w[i], loEff[i], hiEff[i], absTol);

      switch (state) {
        case ConstraintState::AtLower:
        case ConstraintState::AtUpper:
          J(i, i) = 1.0;
          break;
        case ConstraintState::Interior:
          for (Eigen::Index j = 0; j < n; ++j) {
            J(i, j) = A(i, j);
          }
          break;
      }
    }

    J.diagonal().array() += params->jacobianRegularization;

    Eigen::LLT<Eigen::MatrixXd> llt(J.transpose() * J);
    if (llt.info() != Eigen::Success) {
      J.diagonal().array() += 1e-6;
      llt.compute(J.transpose() * J);
    }
    dx = llt.solve(-J.transpose() * H);

    if (!dx.allFinite()) {
      result.status = LcpSolverStatus::NumericalError;
      result.message = "Newton solve produced non-finite step";
      result.iterations = iterationsUsed;
      return result;
    }

    double alpha = 1.0;
    bool accepted = false;

    for (int ls = 0; ls < params->maxLineSearchSteps; ++ls) {
      Eigen::VectorXd xNew = projectVector(x + alpha * dx, loEff, hiEff);

      for (Eigen::Index i = 0; i < n; ++i) {
        if (findex[i] >= 0) {
          const double fricLimit = std::abs(hi[i] * xNew[findex[i]]);
          loEff[i] = -fricLimit;
          hiEff[i] = fricLimit;
          xNew[i] = project(xNew[i], loEff[i], hiEff[i]);
        }
      }

      const Eigen::VectorXd wNew = A * xNew - b;
      const Eigen::VectorXd HNew
          = computeNaturalResidual(xNew, wNew, loEff, hiEff);
      const double HnewNorm = HNew.lpNorm<Eigen::Infinity>();

      if (HnewNorm < Hnorm * (1.0 - params->sufficientDecrease * alpha)
          || HnewNorm < Hnorm) {
        x = xNew;
        accepted = true;
        break;
      }

      alpha *= params->stepReduction;
      if (alpha < params->minStep) {
        break;
      }
    }

    if (!accepted) {
      lineSearchFailed = true;
      break;
    }
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
    x[i] = project(x[i], loEff[i], hiEff[i]);
  }

  w = A * x - b;
  result.iterations = iterationsUsed;
  result.residual = detail::naturalResidualInfinityNorm(x, w, loEff, hiEff);
  result.complementarity
      = detail::complementarityInfinityNorm(x, w, loEff, hiEff, absTol);

  if (converged) {
    result.status = LcpSolverStatus::Success;
  } else if (lineSearchFailed) {
    result.status = LcpSolverStatus::Failed;
    result.message = "Line search failed";
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

std::string BoxedSemiSmoothNewtonSolver::getName() const
{
  return "BoxedSemiSmoothNewton";
}

std::string BoxedSemiSmoothNewtonSolver::getCategory() const
{
  return "Newton";
}

} // namespace dart::math
