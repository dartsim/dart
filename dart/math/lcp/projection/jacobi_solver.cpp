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

#include "dart/math/lcp/projection/jacobi_solver.hpp"

#include "dart/math/lcp/lcp_validation.hpp"

#include <algorithm>
#include <atomic>
#include <barrier>
#include <iterator>
#include <limits>
#include <string>
#include <thread>
#include <vector>

#include <cmath>

namespace dart::math {
namespace {

constexpr int kMaxStrictInteriorFastPathSize = 96;
constexpr int kMaxFrictionIndexExactFastPathSize = 192;

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
JacobiSolver::JacobiSolver()
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
void JacobiSolver::setParameters(const Parameters& params)
{
  mParameters = params;
}

//==============================================================================
const JacobiSolver::Parameters& JacobiSolver::getParameters() const
{
  return mParameters;
}

//==============================================================================
LcpResult JacobiSolver::solve(
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
  const double relaxation = (options.relaxation > 0.0)
                                ? options.relaxation
                                : mDefaultOptions.relaxation;

  if (!std::isfinite(relaxation) || relaxation <= 0.0 || relaxation > 2.0) {
    result.status = LcpSolverStatus::InvalidProblem;
    result.message = "Relaxation parameter must be in (0, 2]";
    return result;
  }

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
  if (params->workerThreads <= 0) {
    result.status = LcpSolverStatus::InvalidProblem;
    result.message = "Worker thread count must be positive";
    return result;
  }

  Eigen::VectorXd fastW;
  bool exactFastPath = false;
  if (options.customOptions == nullptr && !options.warmStart) {
    const double validationTolerance = std::max(absTol, compTolOpt);
    if (n <= kMaxStrictInteriorFastPathSize && problem.isStandardLcp(absTol)) {
      exactFastPath = detail::trySolveStrictInteriorStandardLcpLltFirst(
          problem, absTol, validationTolerance, x, &fastW);
    } else if (
        n <= kMaxFrictionIndexExactFastPathSize && problem.hasFrictionIndex()) {
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
        x, fastW, loEffFast, hiEffFast, compTolOpt);
    result.validated = options.validateSolution;
    return result;
  }

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

  if (!updateMetrics(x)) {
    return result;
  }

  bool converged = (residual <= tol && complementarity <= compTol);
  int iterationsUsed = 0;

  Eigen::VectorXd xNew(n);

  auto updateRange = [&](const int begin, const int end) {
    for (int i = begin; i < end; ++i) {
      double value = x[i];
      const double diag = A(i, i);
      if (std::isfinite(diag) && std::abs(diag) >= params->epsilonForDivision) {
        const double step = x[i] - w[i] / diag;
        value = std::lerp(x[i], step, relaxation);
      }

      xNew[i] = detail::projectToBounds(value, loEff[i], hiEff[i]);
    }
  };

  const int requestedWorkers = std::max(1, params->workerThreads);
  const int workerCount = std::min<int>(requestedWorkers, static_cast<int>(n));

  if (workerCount > 1) {
    std::atomic<bool> stopWorkers{false};
    std::barrier iterationBarrier(workerCount + 1);
    std::vector<std::thread> workers;
    workers.reserve(static_cast<std::size_t>(workerCount));

    const int chunkSize = (static_cast<int>(n) + workerCount - 1) / workerCount;
    for (int worker = 0; worker < workerCount; ++worker) {
      const int begin = worker * chunkSize;
      const int end = std::min(static_cast<int>(n), begin + chunkSize);
      workers.emplace_back([&, begin, end]() {
        while (true) {
          iterationBarrier.arrive_and_wait();
          if (stopWorkers.load(std::memory_order_acquire)) {
            break;
          }
          updateRange(begin, end);
          iterationBarrier.arrive_and_wait();
        }
      });
    }

    for (int iter = 0; iter < maxIterations && !converged; ++iter) {
      iterationsUsed = iter + 1;

      iterationBarrier.arrive_and_wait();
      iterationBarrier.arrive_and_wait();
      x = xNew;

      if (!updateMetrics(x)) {
        stopWorkers.store(true, std::memory_order_release);
        iterationBarrier.arrive_and_wait();
        for (auto& worker : workers) {
          worker.join();
        }
        return result;
      }

      if (residual <= tol && complementarity <= compTol) {
        converged = true;
      }
    }

    stopWorkers.store(true, std::memory_order_release);
    iterationBarrier.arrive_and_wait();
    for (auto& worker : workers) {
      worker.join();
    }
  } else {
    for (int iter = 0; iter < maxIterations && !converged; ++iter) {
      iterationsUsed = iter + 1;

      updateRange(0, static_cast<int>(n));

      x = xNew;

      if (!updateMetrics(x)) {
        return result;
      }

      if (residual <= tol && complementarity <= compTol) {
        converged = true;
      }
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
std::string JacobiSolver::getName() const
{
  return "Jacobi";
}

//==============================================================================
std::string JacobiSolver::getCategory() const
{
  return "Projection";
}

} // namespace dart::math
