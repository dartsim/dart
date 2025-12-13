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
 *   * Redistributions in binary forms, with or
 *   modification, are permitted provided that the following
 *   disclaimer in the documentation and/or other materials provided
 *   with the distribution.
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

#include "dart/common/Diagnostics.hpp"

DART_SUPPRESS_DEPRECATED_BEGIN
#include "dart/constraint/PgsBoxedLcpSolver.hpp"
DART_SUPPRESS_DEPRECATED_END

#include "dart/math/lcp/projection/PgsSolver.hpp"

DART_SUPPRESS_DEPRECATED_BEGIN
namespace dart::constraint {

PgsBoxedLcpSolver::PgsBoxedLcpSolver()
  : mSolver(std::make_shared<math::PgsSolver>())
{
  syncOptions();
}

void PgsBoxedLcpSolver::setOption(const Option& option)
{
  mOption = option;
  syncOptions();
}

PgsBoxedLcpSolver::Option PgsBoxedLcpSolver::getOption() const
{
  return mOption;
}

const std::string& PgsBoxedLcpSolver::getType() const
{
  static const std::string type = "PgsBoxedLcpSolver";
  return type;
}

math::LcpSolverPtr PgsBoxedLcpSolver::getMathSolver() const
{
  return mSolver;
}

void PgsBoxedLcpSolver::syncOptions()
{
  auto solver = std::dynamic_pointer_cast<math::PgsSolver>(mSolver);
  if (!solver)
    return;

  math::LcpOptions opts = solver->getDefaultOptions();
  opts.maxIterations = mOption.mMaxIteration;
  opts.absoluteTolerance = mOption.mTolerance;
  opts.relativeTolerance = mOption.mTolerance * 10.0;
  solver->setDefaultOptions(opts);
}

} // namespace dart::constraint
DART_SUPPRESS_DEPRECATED_END

