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

#ifndef DART_SIMULATION_SOLVER_CLASSICRIGIDSOLVER_HPP_
#define DART_SIMULATION_SOLVER_CLASSICRIGIDSOLVER_HPP_

#include "dart/simulation/solver/Solver.hpp"

#include <memory>

namespace dart::simulation {

/// Rigid solver that wraps the classic Skeleton + ConstraintSolver pipeline.
///
/// Note: This solver is considered legacy and will eventually be deprecated and
/// removed in favor of the ECS-backed rigid solver.
class DART_API ClassicRigidSolver final : public Solver
{
public:
  ClassicRigidSolver();
  ~ClassicRigidSolver() override;

  std::optional<RigidSolverType> getRigidSolverType() const override;
  bool supportsConstraints() const override;
  bool supportsCollision() const override;
  bool supportsSkeletons() const override;

  void setConstraintSolver(
      constraint::UniqueConstraintSolverPtr solver) override;
  constraint::ConstraintSolver* getConstraintSolver() override;
  const constraint::ConstraintSolver* getConstraintSolver() const override;

  void setCollisionDetector(
      const collision::CollisionDetectorPtr& collisionDetector) override;
  void setCollisionDetector(CollisionDetectorType collisionDetector) override;
  collision::CollisionDetectorPtr getCollisionDetector() override;
  collision::ConstCollisionDetectorPtr getCollisionDetector() const override;
  const collision::CollisionResult& getLastCollisionResult() const override;
  bool checkCollision(
      const collision::CollisionOption& option,
      collision::CollisionResult* result) override;

  void handleSkeletonAdded(
      World& world, const dynamics::SkeletonPtr& skeleton) override;
  void handleSkeletonRemoved(
      World& world, const dynamics::SkeletonPtr& skeleton) override;

  void setTimeStep(double timeStep) override;
  void reset(World& world) override;
  void step(World& world, bool resetCommand) override;

private:
  std::unique_ptr<constraint::ConstraintSolver> mConstraintSolver;
};

} // namespace dart::simulation

#endif // DART_SIMULATION_SOLVER_CLASSICRIGIDSOLVER_HPP_
