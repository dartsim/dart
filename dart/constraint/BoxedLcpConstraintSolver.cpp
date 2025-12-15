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

#include "dart/constraint/BoxedLcpConstraintSolver.hpp"

#include "dart/common/Logging.hpp"
#include "dart/constraint/DantzigBoxedLcpSolver.hpp"
#include "dart/constraint/PgsBoxedLcpSolver.hpp"
#include "dart/math/lcp/pivoting/DantzigSolver.hpp"
#include "dart/math/lcp/projection/PgsSolver.hpp"

namespace dart {
namespace constraint {

namespace {

math::LcpSolverPtr createPrimarySolver(const BoxedLcpSolverPtr& boxed)
{
  if (!boxed)
    return nullptr;

  if (auto pgs = std::dynamic_pointer_cast<PgsBoxedLcpSolver>(boxed)) {
    auto solver = std::make_shared<math::PgsSolver>();

    math::LcpOptions options = solver->getDefaultOptions();
    options.maxIterations = pgs->getOption().mMaxIteration;
    options.absoluteTolerance = pgs->getOption().mDeltaXThreshold;
    options.relativeTolerance = pgs->getOption().mRelativeDeltaXTolerance;
    solver->setDefaultOptions(options);

    math::PgsSolver::Parameters params;
    params.epsilonForDivision = pgs->getOption().mEpsilonForDivision;
    params.randomizeConstraintOrder
        = pgs->getOption().mRandomizeConstraintOrder;
    solver->setParameters(params);
    return solver;
  }

  // Default to Dantzig.
  return std::make_shared<math::DantzigSolver>();
}

} // namespace

//==============================================================================
BoxedLcpConstraintSolver::BoxedLcpConstraintSolver() : ConstraintSolver()
{
  mBoxedLcpSolver = std::make_shared<DantzigBoxedLcpSolver>();
  mSecondaryBoxedLcpSolver = std::make_shared<PgsBoxedLcpSolver>();
}

//==============================================================================
BoxedLcpConstraintSolver::BoxedLcpConstraintSolver(
    BoxedLcpSolverPtr boxedLcpSolver)
  : BoxedLcpConstraintSolver(
      std::move(boxedLcpSolver), std::make_shared<PgsBoxedLcpSolver>())
{
  // Do nothing
}

//==============================================================================
BoxedLcpConstraintSolver::BoxedLcpConstraintSolver(
    BoxedLcpSolverPtr boxedLcpSolver, BoxedLcpSolverPtr secondaryBoxedLcpSolver)
  : ConstraintSolver()
{
  setBoxedLcpSolver(std::move(boxedLcpSolver));
  setSecondaryBoxedLcpSolver(std::move(secondaryBoxedLcpSolver));
}

//==============================================================================
void BoxedLcpConstraintSolver::setBoxedLcpSolver(BoxedLcpSolverPtr lcpSolver)
{
  if (!lcpSolver) {
    DART_WARN("nullptr for boxed LCP solver is not allowed.");
    return;
  }

  mBoxedLcpSolver = std::move(lcpSolver);

  setLcpSolver(createPrimarySolver(mBoxedLcpSolver));

  // Mirror gz-physics expectations: PGS is a standalone primary solver without
  // a fallback, while Dantzig uses PGS as a secondary solver by default.
  if (std::dynamic_pointer_cast<PgsBoxedLcpSolver>(mBoxedLcpSolver)) {
    mSecondaryBoxedLcpSolver.reset();
    setSecondaryLcpSolver(nullptr);
  } else if (!mSecondaryBoxedLcpSolver) {
    mSecondaryBoxedLcpSolver = std::make_shared<PgsBoxedLcpSolver>();
    setSecondaryLcpSolver(createPrimarySolver(mSecondaryBoxedLcpSolver));
  }

  if (auto dantzig
      = std::dynamic_pointer_cast<math::DantzigSolver>(mLcpSolver)) {
    math::LcpOptions options = dantzig->getDefaultOptions();
    options.earlyTermination = (mSecondaryLcpSolver != nullptr);
    dantzig->setDefaultOptions(options);
  }
}

//==============================================================================
ConstBoxedLcpSolverPtr BoxedLcpConstraintSolver::getBoxedLcpSolver() const
{
  return mBoxedLcpSolver;
}

//==============================================================================
void BoxedLcpConstraintSolver::setSecondaryBoxedLcpSolver(
    BoxedLcpSolverPtr lcpSolver)
{
  mSecondaryBoxedLcpSolver = std::move(lcpSolver);
  setSecondaryLcpSolver(createPrimarySolver(mSecondaryBoxedLcpSolver));

  if (auto dantzig
      = std::dynamic_pointer_cast<math::DantzigSolver>(mLcpSolver)) {
    math::LcpOptions options = dantzig->getDefaultOptions();
    options.earlyTermination = (mSecondaryLcpSolver != nullptr);
    dantzig->setDefaultOptions(options);
  }
}

//==============================================================================
ConstBoxedLcpSolverPtr BoxedLcpConstraintSolver::getSecondaryBoxedLcpSolver()
    const
{
  return mSecondaryBoxedLcpSolver;
}

} // namespace constraint
} // namespace dart
