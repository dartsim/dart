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

#include "dart/math/lcp/projection/subspace_minimization_solver.hpp"

#include "dart/math/lcp/lcp_validation.hpp"
#include "dart/math/lcp/projection/pgs_solver.hpp"

#include <Eigen/QR>

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

} // namespace

//==============================================================================
SubspaceMinimizationSolver::SubspaceMinimizationSolver()
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
void SubspaceMinimizationSolver::setParameters(const Parameters& params)
{
  mParameters = params;
}

//==============================================================================
const SubspaceMinimizationSolver::Parameters&
SubspaceMinimizationSolver::getParameters() const
{
  return mParameters;
}

//==============================================================================
LcpResult SubspaceMinimizationSolver::solve(
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

  if (x.size() != n || !options.warmStart || !x.allFinite())
    x = Eigen::VectorXd::Zero(n);

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

  const Parameters* params
      = options.customOptions
            ? static_cast<const Parameters*>(options.customOptions)
            : &mParameters;

  if (params->pgsIterations <= 0) {
    result.status = LcpSolverStatus::InvalidProblem;
    result.message = "Subspace minimization requires PGS iterations";
    return result;
  }

  PgsSolver pgs;
  LcpOptions pgsOptions = pgs.getDefaultOptions();
  pgsOptions.maxIterations = params->pgsIterations;
  pgsOptions.absoluteTolerance = absTol;
  pgsOptions.relativeTolerance = relTol;
  pgsOptions.complementarityTolerance = compTolOpt;
  pgsOptions.relaxation = options.relaxation;
  pgsOptions.warmStart = true;
  pgsOptions.validateSolution = false;
  pgsOptions.customOptions = nullptr;

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

  if (!updateMetrics())
    return result;

  bool converged = (residual <= tol && complementarity <= compTol);
  int iterationsUsed = 0;

  std::vector<int> freeIndices;
  std::vector<int> fixedIndices;
  freeIndices.reserve(static_cast<std::size_t>(n));
  fixedIndices.reserve(static_cast<std::size_t>(n));

  for (int iter = 0; iter < maxIterations && !converged; ++iter) {
    iterationsUsed = iter + 1;

    const auto pgsResult = pgs.solve(problem, x, pgsOptions);
    if (pgsResult.status == LcpSolverStatus::InvalidProblem
        || pgsResult.status == LcpSolverStatus::NumericalError) {
      return pgsResult;
    }

    if (!updateMetrics())
      return result;

    const double activeTol
        = (params->activeSetTolerance > 0.0) ? params->activeSetTolerance : tol;

    freeIndices.clear();
    fixedIndices.clear();

    for (int i = 0; i < n; ++i) {
      const bool hasLo = std::isfinite(loEff[i]);
      const bool hasUpper = std::isfinite(hiEff[i]);
      const bool atLo = hasLo && std::abs(x[i] - loEff[i]) <= activeTol;
      const bool atHi = hasUpper && std::abs(x[i] - hiEff[i]) <= activeTol;
      const bool interior = (!atLo && !atHi)
                            && (!hasLo || x[i] > loEff[i] + activeTol)
                            && (!hasUpper || x[i] < hiEff[i] - activeTol);
      if (interior)
        freeIndices.push_back(i);
      else
        fixedIndices.push_back(i);
    }

    if (!freeIndices.empty()) {
      const auto fSize = std::ssize(freeIndices);
      Eigen::MatrixXd A_ff(fSize, fSize);
      Eigen::VectorXd rhs(fSize);

      for (int r = 0; r < fSize; ++r) {
        const int i = freeIndices[r];
        double rhsVal = b[i];
        for (const int j : fixedIndices)
          rhsVal -= A(i, j) * x[j];
        rhs[r] = rhsVal;
        for (int c = 0; c < fSize; ++c)
          A_ff(r, c) = A(i, freeIndices[c]);
      }

      const Eigen::VectorXd xFree = A_ff.colPivHouseholderQr().solve(rhs);
      if (!xFree.allFinite()) {
        result.status = LcpSolverStatus::NumericalError;
        result.message = "Subspace solve produced non-finite values";
        return result;
      }

      for (int r = 0; r < fSize; ++r) {
        const int i = freeIndices[r];
        double value = xFree[r];
        if (std::isfinite(loEff[i]))
          value = std::max(value, loEff[i]);
        if (std::isfinite(hiEff[i]))
          value = std::min(value, hiEff[i]);
        x[i] = value;
      }
    }

    if (!updateMetrics())
      return result;

    if (residual <= tol && complementarity <= compTol)
      converged = true;
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
std::string SubspaceMinimizationSolver::getName() const
{
  return "SubspaceMinimization";
}

//==============================================================================
std::string SubspaceMinimizationSolver::getCategory() const
{
  return "Projection";
}

} // namespace dart::math
