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

#include "dart/constraint/dantzig_boxed_lcp_solver.hpp"

#include "dart/common/diagnostics.hpp"
#include "dart/math/lcp/lcp_types.hpp"
#include "dart/math/lcp/lcp_utils.hpp"
#include "dart/math/lcp/pivoting/dantzig_solver.hpp"

#include <Eigen/Core>

DART_SUPPRESS_DEPRECATED_BEGIN

namespace dart {
namespace constraint {

namespace {

const std::string& dantzigBoxedLcpType()
{
  static const std::string type = "DantzigBoxedLcpSolver";
  return type;
}

} // namespace

//==============================================================================
const std::string& DantzigBoxedLcpSolver::getType() const
{
  return getStaticType();
}

//==============================================================================
const std::string& DantzigBoxedLcpSolver::getStaticType()
{
  return dantzigBoxedLcpType();
}

//==============================================================================
bool DantzigBoxedLcpSolver::solve(
    int n,
    double* A,
    double* x,
    double* b,
    int /*nub*/,
    double* lo,
    double* hi,
    int* findex,
    bool earlyTermination)
{
  if (n <= 0 || !A || !x || !b || !lo || !hi)
    return false;

  const int nSkip = math::padding(n);
  Eigen::MatrixXd Ablock(n, n);
  for (int r = 0; r < n; ++r) {
    const double* row = A + nSkip * r;
    for (int c = 0; c < n; ++c)
      Ablock(r, c) = row[c];
  }

  Eigen::VectorXd bVec(n);
  Eigen::VectorXd loVec(n);
  Eigen::VectorXd hiVec(n);
  Eigen::VectorXi findexVec = Eigen::VectorXi::Constant(n, -1);
  Eigen::VectorXd xVec(n);
  for (int i = 0; i < n; ++i) {
    bVec[i] = b[i];
    loVec[i] = lo[i];
    hiVec[i] = hi[i];
    if (findex)
      findexVec[i] = findex[i];
    xVec[i] = x[i];
  }

  math::LcpProblem problem(
      std::move(Ablock),
      std::move(bVec),
      std::move(loVec),
      std::move(hiVec),
      std::move(findexVec));

  math::DantzigSolver solver;
  math::LcpOptions options = solver.getDefaultOptions();
  options.earlyTermination = earlyTermination;
  options.validateSolution = false;

  const math::LcpResult result = solver.solve(problem, xVec, options);

  for (int i = 0; i < n; ++i)
    x[i] = xVec[i];

  return result.succeeded();
}

//==============================================================================
bool DantzigBoxedLcpSolver::canSolve(int n, const double* A)
{
  return n >= 0 && A != nullptr;
}

} // namespace constraint
} // namespace dart

DART_SUPPRESS_DEPRECATED_END
