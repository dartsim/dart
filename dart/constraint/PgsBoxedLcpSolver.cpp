/*
 * Copyright (c) 2011-2025, The DART development contributors
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

#include "dart/constraint/PgsBoxedLcpSolver.hpp"

#include <Eigen/Dense>

#include <cmath>

namespace {

constexpr double PGS_EPSILON = 1e-8;

} // namespace

namespace dart {
namespace constraint {

//==============================================================================
PgsBoxedLcpSolver::PgsBoxedLcpSolver()
{
  synchronizeDefaults();
}

//==============================================================================
PgsBoxedLcpSolver::Option::Option(
    int maxIteration,
    double deltaXTolerance,
    double relativeDeltaXTolerance,
    double epsilonForDivision,
    bool randomizeConstraintOrder)
  : mMaxIteration(maxIteration),
    mDeltaXThreshold(deltaXTolerance),
    mRelativeDeltaXTolerance(relativeDeltaXTolerance),
    mEpsilonForDivision(epsilonForDivision),
    mRandomizeConstraintOrder(randomizeConstraintOrder)
{
  // Do nothing
}

//==============================================================================
const std::string& PgsBoxedLcpSolver::getType() const
{
  return getStaticType();
}

//==============================================================================
const std::string& PgsBoxedLcpSolver::getStaticType()
{
  static const std::string type = "PgsBoxedLcpSolver";
  return type;
}

//==============================================================================
math::LcpResult PgsBoxedLcpSolver::solve(
    const Eigen::MatrixXd& A,
    const Eigen::VectorXd& b,
    const Eigen::VectorXd& lo,
    const Eigen::VectorXd& hi,
    const Eigen::VectorXi& findex,
    Eigen::VectorXd& x,
    const math::LcpOptions& options)
{
  math::LcpOptions appliedOptions = mDefaultOptions;

  if (options.maxIterations > 0)
    appliedOptions.maxIterations = options.maxIterations;
  if (options.absoluteTolerance > 0)
    appliedOptions.absoluteTolerance = options.absoluteTolerance;
  if (options.relativeTolerance > 0)
    appliedOptions.relativeTolerance = options.relativeTolerance;

  appliedOptions.complementarityTolerance
      = options.complementarityTolerance; // use caller tolerance verbatim
  appliedOptions.validateSolution = options.validateSolution;
  appliedOptions.relaxation = options.relaxation;
  appliedOptions.earlyTermination = options.earlyTermination;
  appliedOptions.warmStart = options.warmStart;
  appliedOptions.customOptions = options.customOptions;

  return mSolver.solve(A, b, lo, hi, findex, x, appliedOptions);
}

//==============================================================================
bool PgsBoxedLcpSolver::canSolve(const Eigen::MatrixXd& A)
{
  const int n = static_cast<int>(A.rows());

  // Return false if A has zero-diagonal or A is nonsymmetric matrix
  for (auto i = 0; i < n; ++i) {
    if (A(i, i) < PGS_EPSILON)
      return false;

    for (auto j = 0; j < n; ++j) {
      if (std::abs(A(i, j) - A(j, i)) > PGS_EPSILON)
        return false;
    }
  }

  return true;
}

//==============================================================================
void PgsBoxedLcpSolver::setOption(const PgsBoxedLcpSolver::Option& option)
{
  mOption = option;
  synchronizeDefaults();
}

//==============================================================================
const PgsBoxedLcpSolver::Option& PgsBoxedLcpSolver::getOption() const
{
  return mOption;
}

//==============================================================================
void PgsBoxedLcpSolver::synchronizeDefaults()
{
  math::LcpOptions defaults = mSolver.getDefaultOptions();
  defaults.maxIterations = mOption.mMaxIteration;
  defaults.absoluteTolerance = mOption.mDeltaXThreshold;
  defaults.relativeTolerance = mOption.mRelativeDeltaXTolerance;
  defaults.warmStart = true;
  mSolver.setDefaultOptions(defaults);
  mDefaultOptions = defaults;

  math::PGSSolver::Parameters params = mSolver.getParameters();
  params.epsilonForDivision = mOption.mEpsilonForDivision;
  params.randomizeConstraintOrder = mOption.mRandomizeConstraintOrder;
  mSolver.setParameters(params);
}

} // namespace constraint
} // namespace dart
