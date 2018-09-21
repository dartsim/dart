/*
 * Copyright (c) 2011-2018, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/master/LICENSE
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

#include "dart/constraint/SequentialImpulseConstraintSolver.hpp"

#include <cassert>
#include "dart/common/Console.hpp"
#include "dart/constraint/ConstrainedGroup.hpp"
#include "dart/constraint/ConstraintBase.hpp"
#include "dart/external/odelcpsolver/lcp.h"

namespace dart {
namespace constraint {

//==============================================================================
SequentialImpulseConstraintSolver::SequentialImpulseConstraintSolver(
    double timeStep, std::size_t maxIteration, double squaredResidual)
  : ConstraintSolver(timeStep), mMaxIteration(maxIteration)
{
  // Do nothing
}

//==============================================================================
std::size_t SequentialImpulseConstraintSolver::getMaxIteration() const
{
  return mMaxIteration;
}

//==============================================================================
void SequentialImpulseConstraintSolver::setMaxIteration(
    std::size_t maxIteration)
{
  mMaxIteration = maxIteration;
}

//==============================================================================
void SequentialImpulseConstraintSolver::solveConstrainedGroup(
    ConstrainedGroup& group)
{
  prestep();

  for (auto i = 0u; i < mMaxIteration; ++i)
  {
    const double squaredResidual = solveSingleIteration(i, group);

    if (squaredResidual < mSquaredResidual)
      break;
  }
}

//==============================================================================
void SequentialImpulseConstraintSolver::prestep()
{
}

//==============================================================================
double SequentialImpulseConstraintSolver::solveSingleIteration(
    std::size_t iterationNumber, ConstrainedGroup& group)
{
  double squredResidual = 0;

  const std::size_t numConstraints = group.getNumConstraints();
  for (std::size_t i = 0; i < numConstraints; ++i)
  {
  }

  return squredResidual;
}

} // namespace constraint
} // namespace dart
