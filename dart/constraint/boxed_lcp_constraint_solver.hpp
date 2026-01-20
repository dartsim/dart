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

#ifndef DART_CONSTRAINT_BOXEDLCPCONSTRAINTSOLVER_HPP_
#define DART_CONSTRAINT_BOXEDLCPCONSTRAINTSOLVER_HPP_

#include <dart/constraint/constraint_solver.hpp>
#include <dart/constraint/fwd.hpp>

#include <dart/Export.hpp>

namespace dart {
namespace constraint {

/// Deprecated; will be removed in DART 8.0. Use dart::math::LcpSolver.
class DART_API DART_DEPRECATED("8.0") BoxedLcpConstraintSolver
  : public ConstraintSolver
{
public:
  BoxedLcpConstraintSolver();
  explicit BoxedLcpConstraintSolver(BoxedLcpSolverPtr boxedLcpSolver);
  BoxedLcpConstraintSolver(
      BoxedLcpSolverPtr boxedLcpSolver,
      BoxedLcpSolverPtr secondaryBoxedLcpSolver);

  void setBoxedLcpSolver(BoxedLcpSolverPtr lcpSolver);
  ConstBoxedLcpSolverPtr getBoxedLcpSolver() const;

  void setSecondaryBoxedLcpSolver(BoxedLcpSolverPtr lcpSolver);
  ConstBoxedLcpSolverPtr getSecondaryBoxedLcpSolver() const;

protected:
  void solveConstrainedGroup(ConstrainedGroup& group) override;

  BoxedLcpSolverPtr mBoxedLcpSolver;
  BoxedLcpSolverPtr mSecondaryBoxedLcpSolver;

private:
  void syncLcpSolversFromBoxedSolvers();
};

} // namespace constraint
} // namespace dart

#endif // DART_CONSTRAINT_BOXEDLCPCONSTRAINTSOLVER_HPP_
