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

#ifndef DART_SIMULATION_SOLVER_WORLDSOLVER_HPP_
#define DART_SIMULATION_SOLVER_WORLDSOLVER_HPP_

#include <dart/simulation/Fwd.hpp>

#include <dart/constraint/Fwd.hpp>

#include <dart/collision/CollisionOption.hpp>
#include <dart/collision/Fwd.hpp>

#include <dart/dynamics/Fwd.hpp>

#include <dart/common/Deprecated.hpp>
#include <dart/Export.hpp>

#include <optional>
#include <string>

namespace dart::simulation {

// Forward declaration to avoid circular include with World.
enum class CollisionDetectorType : int;

/// Identifies which rigid simulation backend a solver instance targets.
enum class RigidSolverType
{
  /// Legacy Skeleton-based constraint solver.
  ClassicSkeleton,
  /// Entity-component based rigid solver using entt::registry.
  EntityComponent,
};

/// Base class for simulation solvers that can be scheduled by World.
class DART_API WorldSolver
{
public:
  explicit WorldSolver(std::string name);
  virtual ~WorldSolver();

  /// Returns the human-friendly solver name.
  const std::string& getName() const;

  /// Returns which rigid backend this solver represents, if any.
  ///
  /// Returning std::nullopt indicates that this solver is not a rigid body
  /// solver and should not participate in rigid solver selection APIs.
  virtual std::optional<RigidSolverType> getRigidSolverType() const;

  /// Returns true if this solver advances rigid body state.
  bool isRigidSolver() const;

  /// Returns true if this solver owns a constraint solver backend.
  virtual bool supportsConstraints() const;

  /// Returns true if this solver manages collision detection.
  virtual bool supportsCollision() const;

  /// Called when the world's timestep changes.
  virtual void setTimeStep(double timeStep) = 0;

  /// Called when the world resets simulation state.
  virtual void reset(World& world);

  /// Advances this solver one step.
  virtual void step(World& world, bool resetCommand) = 0;

  /// Updates derived state without advancing simulation time.
  ///
  /// This hook is intended for solvers that cache or mirror state, so they can
  /// observe changes driven by another active solver.
  virtual void sync(World& world);

  /// Notifies the solver a skeleton has been added.
  virtual void handleSkeletonAdded(
      World& world, const dynamics::SkeletonPtr& skeleton);

  /// Notifies the solver a skeleton has been removed.
  virtual void handleSkeletonRemoved(
      World& world, const dynamics::SkeletonPtr& skeleton);

  // Legacy constraint APIs (optional)
  virtual void setConstraintSolver(
      constraint::UniqueConstraintSolverPtr solver);
  virtual constraint::ConstraintSolver* getConstraintSolver();
  virtual const constraint::ConstraintSolver* getConstraintSolver() const;

  // Legacy collision APIs (optional)
  DART_DEPRECATED(7.0)
  virtual void setCollisionDetector(
      const collision::CollisionDetectorPtr& collisionDetector);
  DART_DEPRECATED(7.0)
  virtual void setCollisionDetector(CollisionDetectorType collisionDetector);
  virtual collision::CollisionDetectorPtr getCollisionDetector();
  virtual collision::ConstCollisionDetectorPtr getCollisionDetector() const;
  virtual const collision::CollisionResult& getLastCollisionResult() const;
  virtual bool checkCollision(
      const collision::CollisionOption& option,
      collision::CollisionResult* result);

protected:
  std::string mName;
  double mTimeStep{0.0};
};

} // namespace dart::simulation

#endif // DART_SIMULATION_SOLVER_WORLDSOLVER_HPP_
