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

#include "dart/math/lcp/pivoting/dantzig_solver.hpp"

#include "dart/math/lcp/lcp_validation.hpp"
#include "dart/math/lcp/pivoting/dantzig/common.hpp"
#include "dart/math/lcp/pivoting/dantzig/lcp.hpp"

#include <algorithm>
#include <iterator>
#include <limits>
#include <vector>

#include <cmath>

namespace dart::math {

//==============================================================================
void DantzigSolver::Scratch::clear() noexcept
{
  Adata.clear();
  xdata.clear();
  wdata.clear();
  bdata.clear();
  loData.clear();
  hiData.clear();
  findexData.clear();
  w.clear();
  loEff.clear();
  hiEff.clear();
  lcp.clear();
}

//==============================================================================
DantzigSolver::DantzigSolver()
{
  mDefaultOptions.warmStart = true;
}

//==============================================================================
LcpResult DantzigSolver::solve(
    const LcpProblem& problem, Eigen::VectorXd& x, const LcpOptions& options)
{
  Scratch scratch;
  return solve(problem, x, scratch, options);
}

//==============================================================================
LcpResult DantzigSolver::solve(
    const LcpProblem& problem,
    Eigen::VectorXd& x,
    Scratch& scratch,
    const LcpOptions& options)
{
  LcpResult result;

  const double absTol = (options.absoluteTolerance > 0)
                            ? options.absoluteTolerance
                            : mDefaultOptions.absoluteTolerance;
  const double compTol = (options.complementarityTolerance > 0)
                             ? options.complementarityTolerance
                             : mDefaultOptions.complementarityTolerance;

  Eigen::VectorXd fastW;
  bool exactFastPath = false;
  if (problem.size() > 0 && !options.warmStart
      && !scratch.usesProvidedAllocator) {
    const double validationTolerance = std::max(absTol, compTol);
    if (problem.isStandardLcp(absTol)) {
      exactFastPath = detail::trySolveStrictInteriorStandardLcpLltFirst(
          problem, absTol, validationTolerance, x, &fastW);
    } else if (problem.isBoxedLcp()) {
      exactFastPath = detail::trySolveProjectedActiveSetBoxedLcp(
          problem, absTol, validationTolerance, x, &fastW);
    } else if (problem.hasFrictionIndex()) {
      exactFastPath = detail::trySolveInteriorFrictionIndexLcp(
          problem, absTol, validationTolerance, x, &fastW);
    }
  }

  if (exactFastPath) {
    Eigen::VectorXd loEffFast;
    Eigen::VectorXd hiEffFast;
    std::string boundsMessage;
    if (!detail::computeEffectiveBounds(
            problem.lo,
            problem.hi,
            problem.findex,
            x,
            loEffFast,
            hiEffFast,
            &boundsMessage)) {
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

  return solve(
      problem.A,
      problem.b,
      problem.lo,
      problem.hi,
      problem.findex,
      x,
      scratch,
      options);
}

//==============================================================================
LcpResult DantzigSolver::solve(
    const Eigen::MatrixXd& A,
    const Eigen::VectorXd& b,
    const Eigen::VectorXd& lo,
    const Eigen::VectorXd& hi,
    const Eigen::VectorXi& findex,
    Eigen::VectorXd& x,
    Scratch& scratch,
    const LcpOptions& options)
{
  LcpResult result;

  std::string problemMessage;
  if (!detail::validateProblem(A, b, lo, hi, findex, &problemMessage)) {
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

  if (x.size() != n) {
    x = Eigen::VectorXd::Zero(n);
  }

  Eigen::Ref<Eigen::VectorXd> xRef(x);
  return solve(A, b, lo, hi, findex, xRef, scratch, options);
}

//==============================================================================
LcpResult DantzigSolver::solve(
    const Eigen::Ref<const Eigen::MatrixXd>& A,
    const Eigen::Ref<const Eigen::VectorXd>& b,
    const Eigen::Ref<const Eigen::VectorXd>& lo,
    const Eigen::Ref<const Eigen::VectorXd>& hi,
    const Eigen::Ref<const Eigen::VectorXi>& findex,
    Eigen::Ref<Eigen::VectorXd> x,
    Scratch& scratch,
    const LcpOptions& options)
{
  LcpResult result;

  std::string problemMessage;
  if (!detail::validateProblemView(A, b, lo, hi, findex, &problemMessage)) {
    result.status = LcpSolverStatus::InvalidProblem;
    result.message = problemMessage;
    return result;
  }

  const Eigen::Index n = b.size();
  if (x.size() != n) {
    result.status = LcpSolverStatus::InvalidProblem;
    result.message = "Solution vector dimensions inconsistent";
    return result;
  }

  if (n == 0) {
    result.status = LcpSolverStatus::Success;
    result.iterations = 0;
    result.residual = 0.0;
    result.complementarity = 0.0;
    result.validated = options.validateSolution;
    return result;
  }

  const int nSkip = padding(static_cast<int>(n));

  const auto vectorSize = static_cast<std::size_t>(n);
  const auto matrixSize = vectorSize * static_cast<std::size_t>(nSkip);
  scratch.Adata.assign(matrixSize, 0.0);
  scratch.xdata.resize(vectorSize);
  scratch.wdata.resize(vectorSize);
  scratch.bdata.resize(vectorSize);
  scratch.loData.resize(vectorSize);
  scratch.hiData.resize(vectorSize);
  scratch.findexData.resize(vectorSize);

  for (int i = 0; i < n; ++i) {
    const auto offset = static_cast<std::size_t>(i);
    scratch.bdata[offset] = b[i];
    scratch.loData[offset] = lo[i];
    scratch.hiData[offset] = hi[i];
    scratch.findexData[offset] = findex[i];
    scratch.wdata[offset] = 0.0;
    scratch.xdata[offset]
        = (options.warmStart && std::isfinite(x[i])) ? x[i] : 0.0;
    for (int j = 0; j < n; ++j) {
      scratch.Adata[static_cast<std::size_t>(i * nSkip + j)] = A(i, j);
    }
  }

  const bool success = SolveLCPWithScratch<double>(
      static_cast<int>(n),
      scratch.Adata.data(),
      scratch.xdata.data(),
      scratch.bdata.data(),
      scratch.wdata.data(),
      0,
      scratch.loData.data(),
      scratch.hiData.data(),
      scratch.findexData.data(),
      scratch.lcp,
      options.earlyTermination);

  const bool solveSucceeded = success;
  for (int i = 0; i < n; ++i) {
    x[i] = scratch.xdata[static_cast<std::size_t>(i)];
  }

  const double absTol = (options.absoluteTolerance > 0)
                            ? options.absoluteTolerance
                            : mDefaultOptions.absoluteTolerance;
  const double compTol = (options.complementarityTolerance > 0)
                             ? options.complementarityTolerance
                             : mDefaultOptions.complementarityTolerance;
  const double relTol = (options.relativeTolerance > 0)
                            ? options.relativeTolerance
                            : mDefaultOptions.relativeTolerance;
  const double validationTol = std::max(absTol, compTol);

  double matrixInfNorm = 0.0;
  for (int row = 0; row < n; ++row) {
    double rowSum = 0.0;
    for (int col = 0; col < n; ++col) {
      rowSum += std::abs(A(row, col));
    }
    matrixInfNorm = std::max(matrixInfNorm, rowSum);
  }
  const double bInfNorm = b.cwiseAbs().maxCoeff();

  struct ResidualTolerances
  {
    double residual;
    double complementarity;
  };

  auto resultTolerances = [&]() {
    const double xInfNorm = x.cwiseAbs().maxCoeff();
    const double scale
        = std::max(1.0, std::max(bInfNorm, matrixInfNorm * xInfNorm));
    return ResidualTolerances{
        std::max(absTol, relTol * scale), std::max(compTol, relTol * scale)};
  };

  // Residual/effective-bound scratch reused across the initial solve and any
  // friction-index refinement iterations; the allocator-backed buffers are
  // resized once here and mapped so no heap growth occurs on warmed solves.
  scratch.w.resize(vectorSize);
  scratch.loEff.resize(vectorSize);
  scratch.hiEff.resize(vectorSize);
  Eigen::Map<Eigen::VectorXd> wEval(scratch.w.data(), n);
  Eigen::Map<Eigen::VectorXd> loEff(scratch.loEff.data(), n);
  Eigen::Map<Eigen::VectorXd> hiEff(scratch.hiEff.data(), n);

  auto updateResiduals = [&](std::string* message = nullptr) -> bool {
    for (int row = 0; row < n; ++row) {
      double value = -b[row];
      for (int col = 0; col < n; ++col) {
        value += A(row, col) * x[col];
      }
      wEval[row] = value;
    }

    std::string boundsMessage;
    if (!detail::computeEffectiveBoundsInto(
            lo, hi, findex, x, loEff, hiEff, &boundsMessage)) {
      if (message) {
        *message = boundsMessage;
      }
      return false;
    }

    result.residual
        = detail::naturalResidualInfinityNormView(x, wEval, loEff, hiEff);
    result.complementarity = detail::complementarityInfinityNormView(
        x, wEval, loEff, hiEff, absTol);
    return true;
  };

  result.iterations = 1;
  std::string boundsMessage;
  const bool boundsOk = updateResiduals(&boundsMessage);
  if (!boundsOk) {
    result.status = LcpSolverStatus::InvalidProblem;
    result.message = boundsMessage;
    return result;
  }

  auto meetsResultTolerances = [&]() {
    const auto tolerances = resultTolerances();
    return result.residual <= tolerances.residual
           && result.complementarity <= tolerances.complementarity;
  };

  auto normalizedResultViolation = [&]() {
    const auto tolerances = resultTolerances();
    return std::max(
        result.residual / tolerances.residual,
        result.complementarity / tolerances.complementarity);
  };

  bool hasFrictionIndex = false;
  for (int i = 0; i < n; ++i) {
    if (findex[i] >= 0) {
      hasFrictionIndex = true;
      break;
    }
  }

  if (solveSucceeded && hasFrictionIndex && !meetsResultTolerances()) {
    constexpr int kMaxFrictionRefinementIterations = 8;
    for (int iteration = 0; iteration < kMaxFrictionRefinementIterations;
         ++iteration) {
      const Eigen::VectorXd previousX = x;
      const double previousResidual = result.residual;
      const double previousComplementarity = result.complementarity;

      for (int row = 0; row < n; ++row) {
        const auto offset = static_cast<std::size_t>(row);
        scratch.bdata[offset] = b[row];
        scratch.loData[offset] = scratch.loEff[row];
        scratch.hiData[offset] = scratch.hiEff[row];
        scratch.wdata[offset] = 0.0;
        scratch.xdata[offset] = x[row];
        for (int col = 0; col < n; ++col) {
          scratch.Adata[static_cast<std::size_t>(row * nSkip + col)]
              = A(row, col);
        }
      }

      const bool refinementSucceeded = SolveLCPWithScratch<double>(
          static_cast<int>(n),
          scratch.Adata.data(),
          scratch.xdata.data(),
          scratch.bdata.data(),
          scratch.wdata.data(),
          0,
          scratch.loData.data(),
          scratch.hiData.data(),
          nullptr,
          scratch.lcp,
          options.earlyTermination);

      for (int i = 0; i < n; ++i) {
        x[i] = scratch.xdata[static_cast<std::size_t>(i)];
      }
      ++result.iterations;

      std::string refinementMessage;
      if (!refinementSucceeded || !updateResiduals(&refinementMessage)) {
        x = previousX;
        result.residual = previousResidual;
        result.complementarity = previousComplementarity;
        updateResiduals();
        break;
      }

      const double currentViolation = normalizedResultViolation();
      if (!std::isfinite(currentViolation)
          || result.residual > previousResidual) {
        x = previousX;
        result.residual = previousResidual;
        result.complementarity = previousComplementarity;
        updateResiduals();
        break;
      }

      if (meetsResultTolerances()) {
        break;
      }
    }
  }

  result.status
      = solveSucceeded ? LcpSolverStatus::Success : LcpSolverStatus::Failed;

  if (result.status == LcpSolverStatus::Success && !meetsResultTolerances()) {
    result.status = LcpSolverStatus::NumericalError;
    result.message
        = "Dantzig solution violates LCP residual/complementarity tolerance";
  }

  if (options.validateSolution && result.status == LcpSolverStatus::Success) {
    std::string validationMessage;
    const bool feasible = detail::validateSolutionView(
        x, wEval, loEff, hiEff, validationTol, &validationMessage);
    result.validated = true;
    if (!feasible) {
      result.status = LcpSolverStatus::NumericalError;
      result.message = validationMessage;
    }
  }

  return result;
}

//==============================================================================
std::string DantzigSolver::getName() const
{
  return "Dantzig";
}

//==============================================================================
std::string DantzigSolver::getCategory() const
{
  return "Pivoting";
}

} // namespace dart::math
