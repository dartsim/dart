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
 *     MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
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

#include "dart/math/lcp/projection/red_black_gauss_seidel_solver.hpp"

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

constexpr int kMaxStrictInteriorFastPathSize = 48;
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
RedBlackGaussSeidelSolver::RedBlackGaussSeidelSolver()
{
  mDefaultOptions.maxIterations = 100;
  mDefaultOptions.absoluteTolerance = 1e-6;
  mDefaultOptions.relativeTolerance = 1e-3;
  mDefaultOptions.complementarityTolerance = 1e-6;
  mDefaultOptions.relaxation = 1.0;
  mDefaultOptions.validateSolution = true;
  mDefaultOptions.warmStart = false;
}

//==============================================================================
void RedBlackGaussSeidelSolver::setParameters(const Parameters& params)
{
  mParameters = params;
}

//==============================================================================
const RedBlackGaussSeidelSolver::Parameters&
RedBlackGaussSeidelSolver::getParameters() const
{
  return mParameters;
}

//==============================================================================
LcpResult RedBlackGaussSeidelSolver::solve(
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
    } else if (n <= kMaxStrictInteriorFastPathSize && problem.isBoxedLcp()) {
      exactFastPath = detail::trySolveProjectedActiveSetBoxedLcp(
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

  std::vector<int> redIndices;
  std::vector<int> blackIndices;
  redIndices.reserve(static_cast<std::size_t>(n + 1) / 2);
  blackIndices.reserve(static_cast<std::size_t>(n) / 2);
  for (int i = 0; i < n; ++i) {
    if ((i % 2) == 0) {
      redIndices.push_back(i);
    } else {
      blackIndices.push_back(i);
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

  auto projectValue = [&](int i, const Eigen::VectorXd& xRef, double value) {
    if (findex[i] >= 0) {
      const int ref = findex[i];
      const double bound = std::abs(hi[i]) * std::abs(xRef[ref]);
      if (value > bound) {
        return bound;
      }
      if (value < -bound) {
        return -bound;
      }
      return value;
    }

    return detail::projectToBounds(value, lo[i], hi[i]);
  };

  auto updateIndex = [&](int i, const Eigen::VectorXd& xRead) {
    double value = xRead[i];
    const double diag = A(i, i);
    if (std::isfinite(diag) && std::abs(diag) >= params->epsilonForDivision) {
      const double residualEntry = A.row(i).dot(xRead) - b[i];
      const double step = xRead[i] - residualEntry / diag;
      value = std::lerp(xRead[i], step, relaxation);
    } else {
      value = 0.0;
    }

    x[i] = projectValue(i, xRead, value);
  };

  auto updateColorRange = [&](const std::vector<int>& indices,
                              const int begin,
                              const int end,
                              const Eigen::VectorXd& xRead) {
    for (int offset = begin; offset < end; ++offset) {
      updateIndex(indices[static_cast<std::size_t>(offset)], xRead);
    }
  };

  Eigen::VectorXd xOld;
  Eigen::VectorXd xMixed;

  const int largestColorSize
      = static_cast<int>(std::max(redIndices.size(), blackIndices.size()));
  const int requestedWorkers = std::max(1, params->workerThreads);
  const int workerCount = std::min(requestedWorkers, largestColorSize);

  if (workerCount > 1) {
    std::atomic<bool> stopWorkers{false};
    std::barrier phaseBarrier(workerCount + 1);
    std::vector<std::thread> workers;
    workers.reserve(static_cast<std::size_t>(workerCount));

    bool useRedIndices = true;

    for (int worker = 0; worker < workerCount; ++worker) {
      workers.emplace_back([&, worker]() {
        while (true) {
          phaseBarrier.arrive_and_wait();
          if (stopWorkers.load(std::memory_order_acquire)) {
            break;
          }

          const auto& indices = useRedIndices ? redIndices : blackIndices;
          const auto& xRead = useRedIndices ? xOld : xMixed;
          const int colorSize = static_cast<int>(indices.size());
          const int chunkSize = (colorSize + workerCount - 1) / workerCount;
          const int begin = worker * chunkSize;
          const int end = std::min(colorSize, begin + chunkSize);
          updateColorRange(indices, begin, end, xRead);
          phaseBarrier.arrive_and_wait();
        }
      });
    }

    auto runThreadedColor = [&](const bool redPhase) {
      useRedIndices = redPhase;
      phaseBarrier.arrive_and_wait();
      phaseBarrier.arrive_and_wait();
    };

    for (int iter = 0; iter < maxIterations && !converged; ++iter) {
      iterationsUsed = iter + 1;

      xOld = x;

      runThreadedColor(true);

      xMixed = xOld;
      for (const int i : redIndices) {
        xMixed[i] = x[i];
      }

      runThreadedColor(false);

      if (!updateMetrics()) {
        stopWorkers.store(true, std::memory_order_release);
        phaseBarrier.arrive_and_wait();
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
    phaseBarrier.arrive_and_wait();
    for (auto& worker : workers) {
      worker.join();
    }
  } else {
    for (int iter = 0; iter < maxIterations && !converged; ++iter) {
      iterationsUsed = iter + 1;

      xOld = x;

      updateColorRange(
          redIndices, 0, static_cast<int>(redIndices.size()), xOld);

      xMixed = xOld;
      for (const int i : redIndices) {
        xMixed[i] = x[i];
      }

      updateColorRange(
          blackIndices, 0, static_cast<int>(blackIndices.size()), xMixed);

      if (!updateMetrics()) {
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
std::string RedBlackGaussSeidelSolver::getName() const
{
  return "RedBlackGaussSeidel";
}

//==============================================================================
std::string RedBlackGaussSeidelSolver::getCategory() const
{
  return "Projection";
}

} // namespace dart::math
