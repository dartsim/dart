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

#include "dart/math/lcp/pivoting/baraff_solver.hpp"

#include "dart/math/lcp/lcp_validation.hpp"
#include "dart/math/lcp/pivoting/dantzig_solver.hpp"

#include <Eigen/Dense>

#include <algorithm>
#include <iterator>
#include <limits>
#include <string>
#include <vector>

#include <cmath>

namespace dart::math {
namespace {

double matrixInfinityNorm(const Eigen::MatrixXd& A)
{
  if (A.size() == 0)
    return 0.0;

  return A.cwiseAbs().rowwise().sum().maxCoeff();
}

double vectorInfinityNorm(const Eigen::VectorXd& v)
{
  return v.size() > 0 ? v.cwiseAbs().maxCoeff() : 0.0;
}

enum class SetType
{
  Unprocessed,
  Active,
  Free
};

void RemoveIndex(std::vector<int>& indices, int idx)
{
  const auto it = std::ranges::find(indices, idx);
  if (it == indices.end())
    return;

  *it = indices.back();
  indices.pop_back();
}

void MoveIndex(int idx, std::vector<int>& from, std::vector<int>& to)
{
  RemoveIndex(from, idx);
  to.push_back(idx);
}

} // namespace

//==============================================================================
BaraffSolver::BaraffSolver()
{
  mDefaultOptions.maxIterations = std::numeric_limits<int>::max();
  mDefaultOptions.validateSolution = true;
  mDefaultOptions.warmStart = false;
}

//==============================================================================
LcpResult BaraffSolver::solve(
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

  const double absTol = (options.absoluteTolerance > 0)
                            ? options.absoluteTolerance
                            : mDefaultOptions.absoluteTolerance;
  const double relTol = (options.relativeTolerance > 0)
                            ? options.relativeTolerance
                            : mDefaultOptions.relativeTolerance;
  const double compTol = (options.complementarityTolerance > 0)
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
  if (maxIterations <= 0)
    maxIterations = std::numeric_limits<int>::max();

  if (x.size() != n || !options.warmStart || !x.allFinite()) {
    x = Eigen::VectorXd::Zero(n);
  } else {
    x = x.cwiseMax(0.0);
  }

  Eigen::VectorXd w = A * x - b;

  std::vector<int> activeSet;
  std::vector<int> freeSet;
  std::vector<int> unprocessed;
  activeSet.reserve(static_cast<std::size_t>(n));
  freeSet.reserve(static_cast<std::size_t>(n));
  unprocessed.reserve(static_cast<std::size_t>(n));

  if (options.warmStart) {
    for (int i = 0; i < n; ++i) {
      if (x[i] > absTol && std::abs(w[i]) <= absTol) {
        activeSet.push_back(i);
      } else if (x[i] <= absTol && w[i] > absTol) {
        freeSet.push_back(i);
      } else {
        unprocessed.push_back(i);
      }
    }
  } else {
    for (int i = 0; i < n; ++i)
      unprocessed.push_back(i);
  }

  int iterationsUsed = 0;
  bool hitLimit = false;
  bool numericalError = false;

  while (!unprocessed.empty()) {
    double minW = std::numeric_limits<double>::infinity();
    int entering = -1;
    for (const int idx : unprocessed) {
      if (w[idx] < minW) {
        minW = w[idx];
        entering = idx;
      }
    }

    if (entering < 0)
      break;

    if (minW >= -absTol) {
      for (const int idx : unprocessed) {
        freeSet.push_back(idx);
      }
      unprocessed.clear();
      break;
    }

    bool placed = false;
    while (!placed) {
      if (iterationsUsed >= maxIterations) {
        hitLimit = true;
        break;
      }
      ++iterationsUsed;

      Eigen::VectorXd deltaX = Eigen::VectorXd::Zero(n);
      deltaX[entering] = 1.0;

      if (!activeSet.empty()) {
        const auto m = std::ssize(activeSet);
        Eigen::MatrixXd A_aa(m, m);
        Eigen::VectorXd A_aj(m);
        for (int r = 0; r < m; ++r) {
          const int i = activeSet[r];
          A_aj[r] = A(i, entering);
          for (int c = 0; c < m; ++c) {
            A_aa(r, c) = A(i, activeSet[c]);
          }
        }

        Eigen::FullPivLU<Eigen::MatrixXd> lu(A_aa);
        if (!lu.isInvertible()) {
          numericalError = true;
          result.message = "Baraff solver: singular active-set matrix";
          break;
        }
        const Eigen::VectorXd deltaA = lu.solve(-A_aj);
        for (int r = 0; r < m; ++r)
          deltaX[activeSet[r]] = deltaA[r];
      }

      const Eigen::VectorXd deltaW = A * deltaX;
      if (!std::isfinite(deltaW[entering]) || deltaW[entering] <= 0.0) {
        numericalError = true;
        result.message = "Baraff solver: invalid search direction";
        break;
      }

      double alpha = -w[entering] / deltaW[entering];
      int blockingIdx = -1;
      SetType blockingType = SetType::Unprocessed;

      for (const int idx : activeSet) {
        if (deltaX[idx] < -absTol) {
          const double candidate = -x[idx] / deltaX[idx];
          if (candidate < alpha) {
            alpha = candidate;
            blockingIdx = idx;
            blockingType = SetType::Active;
          }
        }
      }

      for (const int idx : freeSet) {
        if (deltaW[idx] < -absTol) {
          const double candidate = -w[idx] / deltaW[idx];
          if (candidate < alpha) {
            alpha = candidate;
            blockingIdx = idx;
            blockingType = SetType::Free;
          }
        }
      }

      if (!std::isfinite(alpha) || alpha < 0.0) {
        numericalError = true;
        result.message = "Baraff solver: invalid step length";
        break;
      }

      x += alpha * deltaX;
      w += alpha * deltaW;

      if (blockingIdx < 0) {
        w[entering] = 0.0;
        RemoveIndex(unprocessed, entering);
        if (x[entering] > absTol) {
          activeSet.push_back(entering);
        } else {
          x[entering] = 0.0;
          freeSet.push_back(entering);
        }
        placed = true;
      } else if (blockingType == SetType::Active) {
        x[blockingIdx] = 0.0;
        MoveIndex(blockingIdx, activeSet, freeSet);
      } else if (blockingType == SetType::Free) {
        w[blockingIdx] = 0.0;
        MoveIndex(blockingIdx, freeSet, activeSet);
      }
    }

    if (hitLimit || numericalError)
      break;
  }

  result.iterations = iterationsUsed;

  if (numericalError) {
    result.status = LcpSolverStatus::NumericalError;
    return result;
  }

  w = A * x - b;
  Eigen::VectorXd loEff;
  Eigen::VectorXd hiEff;
  std::string boundsMessage;
  if (!detail::computeEffectiveBounds(
          lo, hi, findex, x, loEff, hiEff, &boundsMessage)) {
    result.status = LcpSolverStatus::InvalidProblem;
    result.message = boundsMessage;
    return result;
  }

  const double scale = std::max(
      1.0,
      std::max(
          vectorInfinityNorm(b),
          matrixInfinityNorm(A) * vectorInfinityNorm(x)));
  const double tol = std::max(absTol, relTol * scale);
  const double compTolScaled = std::max(compTol, relTol * scale);

  result.residual = detail::naturalResidualInfinityNorm(x, w, loEff, hiEff);
  result.complementarity
      = detail::complementarityInfinityNorm(x, w, loEff, hiEff, compTolScaled);

  const bool converged
      = result.residual <= tol && result.complementarity <= compTolScaled;
  if (converged)
    result.status = LcpSolverStatus::Success;
  else if (hitLimit)
    result.status = LcpSolverStatus::MaxIterations;
  else
    result.status = LcpSolverStatus::Failed;

  if (options.validateSolution && result.status == LcpSolverStatus::Success) {
    const double validationTol = std::max(tol, compTolScaled);
    std::string validationMessage;
    const bool feasible = detail::validateSolution(
        x, w, loEff, hiEff, validationTol, &validationMessage);
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
std::string BaraffSolver::getName() const
{
  return "Baraff";
}

//==============================================================================
std::string BaraffSolver::getCategory() const
{
  return "Pivoting";
}

} // namespace dart::math
