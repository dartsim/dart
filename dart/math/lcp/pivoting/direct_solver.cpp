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

#include "dart/math/lcp/pivoting/direct_solver.hpp"

#include "dart/math/lcp/lcp_validation.hpp"
#include "dart/math/lcp/pivoting/dantzig_solver.hpp"

#include <Eigen/Dense>

#include <algorithm>
#include <iterator>
#include <limits>
#include <string>
#include <vector>

namespace dart::math {
namespace {

constexpr int kMaxDimension = 3;

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

} // namespace

//==============================================================================
DirectSolver::DirectSolver()
{
  mDefaultOptions.maxIterations = 0;
  mDefaultOptions.validateSolution = true;
}

//==============================================================================
LcpResult DirectSolver::solve(
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

  if (!isStandardLcp(lo, hi, findex, absTol)) {
    DantzigSolver fallback;
    return fallback.solve(problem, x, options);
  }

  if (n > kMaxDimension) {
    DantzigSolver fallback;
    return fallback.solve(problem, x, options);
  }

  const int totalMasks = 1 << static_cast<int>(n);
  bool found = false;
  int iterationsUsed = 0;
  Eigen::VectorXd candidate = Eigen::VectorXd::Zero(n);
  Eigen::VectorXd w;

  for (int mask = 0; mask < totalMasks; ++mask) {
    ++iterationsUsed;
    std::vector<int> active;
    active.reserve(static_cast<std::size_t>(n));
    for (int i = 0; i < n; ++i) {
      if (mask & (1 << i))
        active.push_back(i);
    }

    candidate.setZero();
    if (!active.empty()) {
      const auto m = std::ssize(active);
      Eigen::MatrixXd Aaa(m, m);
      Eigen::VectorXd bb(m);

      for (int r = 0; r < m; ++r) {
        const int i = active[r];
        bb[r] = b[i];
        for (int c = 0; c < m; ++c) {
          Aaa(r, c) = A(i, active[c]);
        }
      }

      Eigen::FullPivLU<Eigen::MatrixXd> lu(Aaa);
      if (!lu.isInvertible())
        continue;

      const Eigen::VectorXd xActive = lu.solve(bb);
      if (!xActive.allFinite())
        continue;

      for (int r = 0; r < m; ++r)
        candidate[active[r]] = xActive[r];
    }

    w = A * candidate - b;
    if ((candidate.array() < -absTol).any())
      continue;
    if ((w.array() < -absTol).any())
      continue;

    const double compError = (candidate.cwiseProduct(w)).cwiseAbs().maxCoeff();
    if (compError > std::max(absTol, compTol))
      continue;

    found = true;
    break;
  }

  result.iterations = iterationsUsed;

  if (!found) {
    result.status = LcpSolverStatus::Failed;
    result.message = "Direct solver did not find a feasible solution";
    return result;
  }

  x = candidate;

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
          w.cwiseAbs().maxCoeff(),
          A.cwiseAbs().rowwise().sum().maxCoeff() * x.cwiseAbs().maxCoeff()));
  const double tol = std::max(absTol, relTol * scale);
  const double compTolScaled = std::max(compTol, relTol * scale);

  result.residual = detail::naturalResidualInfinityNorm(x, w, loEff, hiEff);
  result.complementarity
      = detail::complementarityInfinityNorm(x, w, loEff, hiEff, compTolScaled);
  result.status = LcpSolverStatus::Success;

  if (options.validateSolution) {
    const double validationTol = std::max(tol, compTolScaled);
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
std::string DirectSolver::getName() const
{
  return "Direct";
}

//==============================================================================
std::string DirectSolver::getCategory() const
{
  return "Pivoting";
}

} // namespace dart::math
