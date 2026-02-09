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

#ifndef DART_TESTS_LCPTESTHARNESS_HPP_
#define DART_TESTS_LCPTESTHARNESS_HPP_

#include <dart/math/lcp/lcp_solver.hpp>
#include <dart/math/lcp/lcp_validation.hpp>

#include <Eigen/Core>

#include <algorithm>
#include <limits>
#include <sstream>
#include <string>

#include <cmath>

namespace dart::test {

struct LcpCheckResult
{
  bool finiteOk{false};
  bool boundsOk{false};
  bool residualOk{false};
  bool complementarityOk{false};
  bool validated{false};
  bool ok{false};

  double tol{std::numeric_limits<double>::quiet_NaN()};
  double compTol{std::numeric_limits<double>::quiet_NaN()};
  double residual{std::numeric_limits<double>::quiet_NaN()};
  double complementarity{std::numeric_limits<double>::quiet_NaN()};
  double boundViolation{std::numeric_limits<double>::quiet_NaN()};

  std::string message;
};

struct LcpSolveReport
{
  dart::math::LcpResult result;
  LcpCheckResult check;
};

inline double InfinityNorm(const Eigen::VectorXd& v)
{
  return v.size() > 0 ? v.cwiseAbs().maxCoeff() : 0.0;
}

inline double InfinityNorm(const Eigen::MatrixXd& m)
{
  if (m.size() == 0) {
    return 0.0;
  }

  return m.cwiseAbs().rowwise().sum().maxCoeff();
}

inline double ComputeBoundViolation(
    const Eigen::VectorXd& lo,
    const Eigen::VectorXd& hi,
    const Eigen::VectorXd& x)
{
  double violation = 0.0;
  for (Eigen::Index i = 0; i < x.size(); ++i) {
    if (std::isfinite(lo[i])) {
      violation = std::max(violation, lo[i] - x[i]);
    }
    if (std::isfinite(hi[i])) {
      violation = std::max(violation, x[i] - hi[i]);
    }
  }
  return std::max(0.0, violation);
}

inline LcpCheckResult CheckLcpSolution(
    const dart::math::LcpProblem& problem,
    const Eigen::VectorXd& x,
    const dart::math::LcpOptions& options)
{
  LcpCheckResult report;

  if (x.size() != problem.b.size()) {
    report.message = "Solution size does not match problem dimension";
    return report;
  }

  const Eigen::VectorXd w = problem.A * x - problem.b;
  report.finiteOk = x.allFinite() && w.allFinite();
  if (!report.finiteOk) {
    report.message = "Non-finite values in solution";
    return report;
  }

  Eigen::VectorXd loEff;
  Eigen::VectorXd hiEff;
  std::string boundsMessage;
  if (!dart::math::detail::computeEffectiveBounds(
          problem.lo,
          problem.hi,
          problem.findex,
          x,
          loEff,
          hiEff,
          &boundsMessage)) {
    report.message = boundsMessage;
    return report;
  }

  const double absTol
      = (options.absoluteTolerance > 0.0) ? options.absoluteTolerance : 1e-6;
  const double relTol
      = (options.relativeTolerance > 0.0) ? options.relativeTolerance : 1e-4;
  const double compTol = (options.complementarityTolerance > 0.0)
                             ? options.complementarityTolerance
                             : absTol;

  const double scale = std::max(
      1.0,
      std::max(
          InfinityNorm(problem.b), InfinityNorm(problem.A) * InfinityNorm(x)));
  report.tol = std::max(absTol, relTol * scale);
  report.compTol = std::max(compTol, relTol * scale);

  report.residual
      = dart::math::detail::naturalResidualInfinityNorm(x, w, loEff, hiEff);
  report.complementarity = dart::math::detail::complementarityInfinityNorm(
      x, w, loEff, hiEff, report.compTol);
  report.boundViolation = ComputeBoundViolation(loEff, hiEff, x);

  report.residualOk = report.residual <= report.tol;
  report.complementarityOk = report.complementarity <= report.compTol;
  report.boundsOk = report.boundViolation <= report.tol;

  std::string validationMessage;
  const double validationTol = std::max(report.tol, report.compTol);
  report.validated = dart::math::detail::validateSolution(
      x, w, loEff, hiEff, validationTol, &validationMessage);
  if (!report.validated && report.message.empty()) {
    report.message = validationMessage;
  }

  report.ok = report.finiteOk && report.boundsOk && report.residualOk
              && report.complementarityOk && report.validated;

  if (!report.ok && report.message.empty()) {
    report.message = "Solution violates comparison contract";
  }

  return report;
}

inline LcpSolveReport SolveAndCheck(
    dart::math::LcpSolver& solver,
    const dart::math::LcpProblem& problem,
    Eigen::VectorXd& x,
    const dart::math::LcpOptions& options)
{
  LcpSolveReport report;
  report.result = solver.solve(problem, x, options);
  report.check = CheckLcpSolution(problem, x, options);
  return report;
}

inline std::string DescribeReport(const LcpSolveReport& report)
{
  std::ostringstream out;
  out << "status=" << dart::math::toString(report.result.status)
      << " iters=" << report.result.iterations
      << " res=" << report.result.residual
      << " comp=" << report.result.complementarity
      << " tol=" << report.check.tol << " compTol=" << report.check.compTol
      << " boundViolation=" << report.check.boundViolation;
  if (!report.result.message.empty()) {
    out << " msg=" << report.result.message;
  }
  if (!report.check.message.empty()) {
    out << " check=" << report.check.message;
  }
  return out.str();
}

} // namespace dart::test

#endif // DART_TESTS_LCPTESTHARNESS_HPP_
