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

#include "dart/math/lcp/other/staggering_solver.hpp"

#include "dart/math/lcp/lcp_validation.hpp"
#include "dart/math/lcp/pivoting/dantzig_solver.hpp"

#include <algorithm>
#include <iterator>
#include <limits>
#include <span>
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
StaggeringSolver::StaggeringSolver()
{
  mDefaultOptions.maxIterations = 50;
  mDefaultOptions.absoluteTolerance = 1e-6;
  mDefaultOptions.relativeTolerance = 1e-3;
  mDefaultOptions.complementarityTolerance = 1e-6;
  mDefaultOptions.relaxation = 1.0;
  mDefaultOptions.validateSolution = true;
  mDefaultOptions.warmStart = false;
}

//==============================================================================
LcpResult StaggeringSolver::solve(
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
  const double relaxation = (options.relaxation > 0.0)
                                ? options.relaxation
                                : mDefaultOptions.relaxation;

  if (!std::isfinite(relaxation) || relaxation <= 0.0 || relaxation > 2.0) {
    result.status = LcpSolverStatus::InvalidProblem;
    result.message = "Relaxation parameter must be in (0, 2]";
    return result;
  }

  std::vector<int> normalIndices;
  std::vector<int> frictionIndices;
  normalIndices.reserve(static_cast<std::size_t>(n));
  frictionIndices.reserve(static_cast<std::size_t>(n));

  for (int i = 0; i < n; ++i) {
    if (findex[i] >= 0)
      frictionIndices.push_back(i);
    else
      normalIndices.push_back(i);
  }

  if (normalIndices.empty() || frictionIndices.empty()) {
    DantzigSolver direct;
    LcpOptions directOptions = direct.getDefaultOptions();
    directOptions.maxIterations = options.maxIterations;
    directOptions.absoluteTolerance = absTol;
    directOptions.relativeTolerance = relTol;
    directOptions.complementarityTolerance = compTolOpt;
    directOptions.validateSolution = options.validateSolution;
    directOptions.warmStart = options.warmStart;
    directOptions.earlyTermination = options.earlyTermination;
    directOptions.customOptions = nullptr;
    return direct.solve(problem, x, directOptions);
  }

  const auto nNormal = std::ssize(normalIndices);
  const auto nFriction = std::ssize(frictionIndices);

  Eigen::MatrixXd A_nn(nNormal, nNormal);
  Eigen::MatrixXd A_nf(nNormal, nFriction);
  Eigen::MatrixXd A_fn(nFriction, nNormal);
  Eigen::MatrixXd A_ff(nFriction, nFriction);

  Eigen::VectorXd b_n(nNormal);
  Eigen::VectorXd b_f(nFriction);
  Eigen::VectorXd lo_n(nNormal);
  Eigen::VectorXd hi_n(nNormal);

  for (int r = 0; r < nNormal; ++r) {
    const int i = normalIndices[r];
    b_n[r] = b[i];
    lo_n[r] = lo[i];
    hi_n[r] = hi[i];
    for (int c = 0; c < nNormal; ++c)
      A_nn(r, c) = A(i, normalIndices[c]);
    for (int c = 0; c < nFriction; ++c)
      A_nf(r, c) = A(i, frictionIndices[c]);
  }

  for (int r = 0; r < nFriction; ++r) {
    const int i = frictionIndices[r];
    b_f[r] = b[i];
    for (int c = 0; c < nNormal; ++c)
      A_fn(r, c) = A(i, normalIndices[c]);
    for (int c = 0; c < nFriction; ++c)
      A_ff(r, c) = A(i, frictionIndices[c]);
  }

  Eigen::VectorXi findex_n = Eigen::VectorXi::Constant(nNormal, -1);
  Eigen::VectorXi findex_f = Eigen::VectorXi::Constant(nFriction, -1);

  auto gather = [&](std::span<const int> indices, Eigen::VectorXd& out) {
    out.resize(std::ssize(indices));
    for (int k = 0; k < std::ssize(indices); ++k)
      out[k] = x[indices[k]];
  };

  auto scatterRelaxedNormal
      = [&](std::span<const int> indices, const Eigen::VectorXd& values) {
          for (int k = 0; k < std::ssize(indices); ++k) {
            const int idx = indices[k];
            double updated = x[idx] + relaxation * (values[k] - x[idx]);
            if (std::isfinite(lo[idx]))
              updated = std::max(updated, lo[idx]);
            if (std::isfinite(hi[idx]))
              updated = std::min(updated, hi[idx]);
            x[idx] = updated;
          }
        };

  auto scatterRelaxedFriction = [&](std::span<const int> indices,
                                    const Eigen::VectorXd& values,
                                    const Eigen::VectorXd& loEff,
                                    const Eigen::VectorXd& hiEff) {
    for (int k = 0; k < std::ssize(indices); ++k) {
      const int idx = indices[k];
      double updated = x[idx] + relaxation * (values[k] - x[idx]);
      if (std::isfinite(loEff[idx]))
        updated = std::max(updated, loEff[idx]);
      if (std::isfinite(hiEff[idx]))
        updated = std::min(updated, hiEff[idx]);
      x[idx] = updated;
    }
  };

  DantzigSolver subSolver;
  LcpOptions subOptions = subSolver.getDefaultOptions();
  subOptions.maxIterations = options.maxIterations;
  subOptions.absoluteTolerance = absTol;
  subOptions.relativeTolerance = relTol;
  subOptions.complementarityTolerance = compTolOpt;
  subOptions.validateSolution = false;
  subOptions.warmStart = true;
  subOptions.earlyTermination = options.earlyTermination;
  subOptions.customOptions = nullptr;

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

  Eigen::VectorXd x_n;
  Eigen::VectorXd x_f;
  Eigen::VectorXd b_eff;

  for (int iter = 0; iter < maxIterations && !converged; ++iter) {
    iterationsUsed = iter + 1;

    gather(frictionIndices, x_f);
    b_eff = b_n - A_nf * x_f;

    gather(normalIndices, x_n);
    LcpProblem normalProblem(A_nn, b_eff, lo_n, hi_n, findex_n);
    const LcpResult normalResult
        = subSolver.solve(normalProblem, x_n, subOptions);
    if (normalResult.status == LcpSolverStatus::InvalidProblem
        || normalResult.status == LcpSolverStatus::NumericalError) {
      return normalResult;
    }
    scatterRelaxedNormal(normalIndices, x_n);

    if (!detail::computeEffectiveBounds(
            lo, hi, findex, x, loEff, hiEff, &problemMessage)) {
      result.status = LcpSolverStatus::InvalidProblem;
      result.message = problemMessage;
      return result;
    }

    Eigen::VectorXd lo_eff_f(nFriction);
    Eigen::VectorXd hi_eff_f(nFriction);
    for (int k = 0; k < nFriction; ++k) {
      const int idx = frictionIndices[k];
      lo_eff_f[k] = loEff[idx];
      hi_eff_f[k] = hiEff[idx];
    }

    gather(normalIndices, x_n);
    b_eff = b_f - A_fn * x_n;

    gather(frictionIndices, x_f);
    LcpProblem frictionProblem(A_ff, b_eff, lo_eff_f, hi_eff_f, findex_f);
    const LcpResult frictionResult
        = subSolver.solve(frictionProblem, x_f, subOptions);
    if (frictionResult.status == LcpSolverStatus::InvalidProblem
        || frictionResult.status == LcpSolverStatus::NumericalError) {
      return frictionResult;
    }
    scatterRelaxedFriction(frictionIndices, x_f, loEff, hiEff);

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
std::string StaggeringSolver::getName() const
{
  return "Staggering";
}

//==============================================================================
std::string StaggeringSolver::getCategory() const
{
  return "Other";
}

} // namespace dart::math
