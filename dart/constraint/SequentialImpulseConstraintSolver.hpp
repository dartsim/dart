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

#ifndef DART_CONSTRAINT_SEQUENTIALIMPULSECONSTRAINTSOLVER_HPP_
#define DART_CONSTRAINT_SEQUENTIALIMPULSECONSTRAINTSOLVER_HPP_

#include "dart/constraint/ConstraintSolver.hpp"
#include "dart/constraint/SmartPointer.hpp"

namespace dart {
namespace constraint {

class SequentialImpulseConstraintSolver : public ConstraintSolver
{
public:
  /// Constructor
  SequentialImpulseConstraintSolver(
      double timeStep,
      std::size_t maxIteration = 30,
      double squaredResidual = 1e-3);

  std::size_t getMaxIteration() const;

  void setMaxIteration(std::size_t maxIteration);

protected:
  // Documentation inherited.
  void solveConstrainedGroup(ConstrainedGroup& group) override;

  void prestep();

  double solveSingleIteration(
      std::size_t iterationNumber, ConstrainedGroup& group);

  std::size_t mMaxIteration;

  double mSquaredResidual;
};

} // namespace constraint
} // namespace dart

#endif // DART_CONSTRAINT_SEQUENTIALIMPULSECONSTRAINTSOLVER_HPP_
