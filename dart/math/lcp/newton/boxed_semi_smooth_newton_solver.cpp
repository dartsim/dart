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
#include "dart/math/lcp/projection/pgs_solver.hpp"

#include <Eigen/Cholesky>
#include <Eigen/LU>

#include <algorithm>
#include <ranges>

#include <cmath>

namespace dart::math {

namespace {

enum class ConstraintState
{
  AtLower,
  AtUpper,
  Interior
};

ConstraintState classifyProjectionBranch(
    double candidate, double lo, double hi, double tol)
{
  if (std::isfinite(lo) && candidate <= lo + tol) {
    return ConstraintState::AtLower;
  }
  if (std::isfinite(hi) && candidate >= hi - tol) {
    return ConstraintState::AtUpper;
  }
  return ConstraintState::Interior;
}

double frictionBoundDerivative(
    const Eigen::VectorXd& hi,
    const Eigen::VectorXi& findex,
    const Eigen::VectorXd& x,
    Eigen::Index i)
{
  const int ref = findex[i];
  if (ref < 0) {
    return 0.0;
  }

  const double scale = x[ref];
  if (scale > 0.0) {
    return std::abs(hi[i]);
  }
  if (scale < 0.0) {
    return -std::abs(hi[i]);
  }

  return 0.0;
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
  for (const auto i : std::views::iota(Eigen::Index{0}, x.size())) {
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

bool validateParameters(
    const BoxedSemiSmoothNewtonSolver::Parameters& params, std::string* message)
{
  if (params.maxLineSearchSteps <= 0) {
    if (message) {
      *message
          = "Boxed semi-smooth Newton max_line_search_steps must be positive";
    }
    return false;
  }
  if (!std::isfinite(params.stepReduction) || params.stepReduction <= 0.0
      || params.stepReduction >= 1.0) {
    if (message) {
      *message = "Boxed semi-smooth Newton step_reduction must be in (0, 1)";
    }
    return false;
  }
  if (!std::isfinite(params.sufficientDecrease)
      || params.sufficientDecrease < 0.0 || params.sufficientDecrease >= 1.0) {
    if (message) {
      *message
          = "Boxed semi-smooth Newton sufficient_decrease must be in [0, 1)";
    }
    return false;
  }
  if (!std::isfinite(params.minStep) || params.minStep <= 0.0) {
    if (message) {
      *message = "Boxed semi-smooth Newton min_step must be positive";
    }
    return false;
  }
  if (!std::isfinite(params.jacobianRegularization)
      || params.jacobianRegularization < 0.0) {
    if (message) {
      *message
          = "Boxed semi-smooth Newton jacobian_regularization must be "
            "non-negative";
    }
    return false;
  }
  if (params.maxPgsWarmStartIterations < 0) {
    if (message) {
      *message
          = "Boxed semi-smooth Newton max_pgs_warm_start_iterations must be "
            "non-negative";
    }
    return false;
  }
  if (!std::isfinite(params.pgsWarmStartRelaxation)
      || params.pgsWarmStartRelaxation <= 0.0
      || params.pgsWarmStartRelaxation > 2.0) {
    if (message) {
      *message
          = "Boxed semi-smooth Newton pgs_warm_start_relaxation must be in "
            "(0, 2]";
    }
    return false;
  }
  if (params.maxFrictionIndexExactSolveDimension < 0) {
    if (message) {
      *message
          = "Boxed semi-smooth Newton "
            "max_friction_index_exact_solve_dimension must be non-negative";
    }
    return false;
  }
  return true;
}

double computeNaturalResidualForProblem(
    const LcpProblem& problem, const Eigen::VectorXd& x)
{
  Eigen::VectorXd loEff;
  Eigen::VectorXd hiEff;
  std::string boundsMessage;
  if (!detail::computeEffectiveBounds(
          problem.lo,
          problem.hi,
          problem.findex,
          x,
          loEff,
          hiEff,
          &boundsMessage)) {
    return std::numeric_limits<double>::infinity();
  }

  const Eigen::VectorXd w = problem.A * x - problem.b;
  return detail::naturalResidualInfinityNorm(x, w, loEff, hiEff);
}

void runPgsWarmStart(
    const LcpProblem& problem,
    Eigen::VectorXd& x,
    const LcpOptions& options,
    const BoxedSemiSmoothNewtonSolver::Parameters& params)
{
  if (params.maxPgsWarmStartIterations <= 0) {
    return;
  }

  const double initialResidual = computeNaturalResidualForProblem(problem, x);
  Eigen::VectorXd candidate = x;
  LcpOptions pgsOptions = options;
  pgsOptions.maxIterations = params.maxPgsWarmStartIterations;
  pgsOptions.relaxation = params.pgsWarmStartRelaxation;
  pgsOptions.warmStart = true;
  pgsOptions.validateSolution = false;
  pgsOptions.customOptions = nullptr;

  PgsSolver pgs;
  pgs.solve(problem, candidate, pgsOptions);

  const double candidateResidual
      = computeNaturalResidualForProblem(problem, candidate);
  if (candidate.allFinite() && candidateResidual < initialResidual) {
    x = candidate;
  }
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
  std::string parameterMessage;
  if (!validateParameters(*params, &parameterMessage)) {
    result.status = LcpSolverStatus::InvalidProblem;
    result.message = parameterMessage;
    return result;
  }

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

  Eigen::VectorXd fastW;
  bool exactFastPath = false;
  if (!options.warmStart) {
    const double validationTolerance = std::max(absTol, compTol);
    if (problem.isStandardLcp(absTol)) {
      exactFastPath = detail::trySolveStrictInteriorStandardLcpLltFirst(
          problem, absTol, validationTolerance, x, &fastW);
    } else if (problem.isBoxedLcp()) {
      exactFastPath = detail::trySolveProjectedActiveSetBoxedLcp(
          problem, absTol, validationTolerance, x, &fastW);
    } else if (
        problem.hasFrictionIndex()
        && problem.size() <= params->maxFrictionIndexExactSolveDimension) {
      exactFastPath = detail::trySolveInteriorFrictionIndexLcp(
          problem, absTol, validationTolerance, x, &fastW);
    }
  }

  if (exactFastPath) {
    Eigen::VectorXd loEffFast;
    Eigen::VectorXd hiEffFast;
    std::string boundsMessage;
    if (!detail::computeEffectiveBounds(
            lo, hi, findex, x, loEffFast, hiEffFast, &boundsMessage)) {
      result.status = LcpSolverStatus::InvalidProblem;
      result.message = boundsMessage;
      return result;
    }

    result.status = LcpSolverStatus::Success;
    result.iterations = 0;
    result.residual
        = detail::naturalResidualInfinityNorm(x, fastW, loEffFast, hiEffFast);
    result.complementarity = detail::complementarityInfinityNorm(
        x, fastW, loEffFast, hiEffFast, compTol);
    result.validated = options.validateSolution;
    return result;
  }

  Eigen::VectorXd loEff = lo;
  Eigen::VectorXd hiEff = hi;
  for (const auto i : std::views::iota(Eigen::Index{0}, n)) {
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
  runPgsWarmStart(problem, x, options, *params);

  Eigen::VectorXd w(n);
  Eigen::VectorXd H(n);
  Eigen::VectorXd dx(n);
  Eigen::VectorXd xTrial(n);
  Eigen::VectorXd loTrial(n);
  Eigen::VectorXd hiTrial(n);
  Eigen::MatrixXd J(n, n);

  int iterationsUsed = 0;
  bool converged = false;
  bool lineSearchFailed = false;

  for ([[maybe_unused]] const auto iter : std::views::iota(0, maxIterations)) {
    ++iterationsUsed;

    for (const auto i : std::views::iota(Eigen::Index{0}, n)) {
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
    for (const auto i : std::views::iota(Eigen::Index{0}, n)) {
      const double candidate = x[i] - w[i];
      const ConstraintState state
          = classifyProjectionBranch(candidate, loEff[i], hiEff[i], absTol);

      switch (state) {
        case ConstraintState::AtLower:
          J(i, i) = 1.0;
          if (findex[i] >= 0) {
            J(i, findex[i]) += frictionBoundDerivative(hi, findex, x, i);
          }
          break;
        case ConstraintState::AtUpper:
          J(i, i) = 1.0;
          if (findex[i] >= 0) {
            J(i, findex[i]) -= frictionBoundDerivative(hi, findex, x, i);
          }
          break;
        case ConstraintState::Interior:
          for (const auto j : std::views::iota(Eigen::Index{0}, n)) {
            J(i, j) = A(i, j);
          }
          break;
      }
    }

    J.diagonal().array() += params->jacobianRegularization;

    dx = J.partialPivLu().solve(-H);

    if (!dx.allFinite()) {
      dx.setZero();
    }

    double acceptedResidual = std::numeric_limits<double>::infinity();
    auto acceptLineSearchStep = [&](const Eigen::VectorXd& step) {
      double alpha = 1.0;

      for (int ls = 0; ls < params->maxLineSearchSteps; ++ls) {
        xTrial = x + alpha * step;
        for (const auto i : std::views::iota(Eigen::Index{0}, n)) {
          xTrial[i] = project(xTrial[i], loEff[i], hiEff[i]);
        }

        loTrial = loEff;
        hiTrial = hiEff;
        for (const auto i : std::views::iota(Eigen::Index{0}, n)) {
          if (findex[i] >= 0) {
            const double fricLimit = std::abs(hi[i] * xTrial[findex[i]]);
            loTrial[i] = -fricLimit;
            hiTrial[i] = fricLimit;
            xTrial[i] = project(xTrial[i], loTrial[i], hiTrial[i]);
          }
        }

        const Eigen::VectorXd wNew = A * xTrial - b;
        const Eigen::VectorXd HNew
            = computeNaturalResidual(xTrial, wNew, loTrial, hiTrial);
        const double HnewNorm = HNew.lpNorm<Eigen::Infinity>();

        if (HnewNorm < Hnorm * (1.0 - params->sufficientDecrease * alpha)
            || HnewNorm < Hnorm) {
          x = xTrial;
          loEff = loTrial;
          hiEff = hiTrial;
          acceptedResidual = HnewNorm;
          return true;
        }

        alpha *= params->stepReduction;
        if (alpha < params->minStep) {
          break;
        }
      }

      return false;
    };

    bool accepted = acceptLineSearchStep(dx);

    if (!accepted) {
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
      accepted = acceptLineSearchStep(dx);
    }

    if (!accepted) {
      lineSearchFailed = true;
      break;
    }

    if (acceptedResidual <= absTol) {
      converged = true;
      break;
    }
  }

  for (const auto i : std::views::iota(Eigen::Index{0}, n)) {
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
