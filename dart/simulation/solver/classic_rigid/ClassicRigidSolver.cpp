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

#include "dart/simulation/solver/classic_rigid/ClassicRigidSolver.hpp"

#include "dart/collision/CollisionDetector.hpp"
#include "dart/collision/CollisionGroup.hpp"
#include "dart/common/Diagnostics.hpp"
#include "dart/common/Logging.hpp"
#include "dart/common/Profile.hpp"
#include "dart/constraint/BoxedLcpConstraintSolver.hpp"
#include "dart/dynamics/Skeleton.hpp"
#include "dart/simulation/World.hpp"

namespace dart::simulation {

ClassicRigidSolver::ClassicRigidSolver() : Solver("classic_rigid")
{
  DART_SUPPRESS_DEPRECATED_BEGIN
  mConstraintSolver = std::make_unique<constraint::BoxedLcpConstraintSolver>();
  DART_SUPPRESS_DEPRECATED_END
}

ClassicRigidSolver::~ClassicRigidSolver() = default;

std::optional<RigidSolverType> ClassicRigidSolver::getRigidSolverType() const
{
  return RigidSolverType::ClassicSkeleton;
}

bool ClassicRigidSolver::supportsConstraints() const
{
  return true;
}

bool ClassicRigidSolver::supportsCollision() const
{
  return true;
}

bool ClassicRigidSolver::supportsSkeletons() const
{
  return true;
}

void ClassicRigidSolver::setConstraintSolver(
    constraint::UniqueConstraintSolverPtr solver)
{
  if (!solver) {
    DART_WARN("nullptr for constraint solver is not allowed. Doing nothing.");
    return;
  }

  if (mConstraintSolver) {
    solver->setFromOtherConstraintSolver(*mConstraintSolver);
  }

  mConstraintSolver = std::move(solver);
  mConstraintSolver->setTimeStep(mTimeStep);
}

constraint::ConstraintSolver* ClassicRigidSolver::getConstraintSolver()
{
  return mConstraintSolver.get();
}

const constraint::ConstraintSolver* ClassicRigidSolver::getConstraintSolver()
    const
{
  return mConstraintSolver.get();
}

void ClassicRigidSolver::setCollisionDetector(
    const collision::CollisionDetectorPtr& collisionDetector)
{
  if (!collisionDetector) {
    DART_WARN(
        "Attempted to assign a null collision detector to classic solver.");
    return;
  }

  mConstraintSolver->setCollisionDetector(collisionDetector);
}

void ClassicRigidSolver::setCollisionDetector(
    CollisionDetectorType collisionDetector)
{
  DART_WARN(
      "ClassicRigidSolver::setCollisionDetector(CollisionDetectorType) should "
      "be routed through World for detector instantiation. Ignoring request "
      "for type {}.",
      static_cast<int>(collisionDetector));
}

collision::CollisionDetectorPtr ClassicRigidSolver::getCollisionDetector()
{
  return mConstraintSolver->getCollisionDetector();
}

collision::ConstCollisionDetectorPtr ClassicRigidSolver::getCollisionDetector()
    const
{
  return mConstraintSolver->getCollisionDetector();
}

const collision::CollisionResult& ClassicRigidSolver::getLastCollisionResult()
    const
{
  return mConstraintSolver->getLastCollisionResult();
}

bool ClassicRigidSolver::checkCollision(
    const collision::CollisionOption& option,
    collision::CollisionResult* result)
{
  return mConstraintSolver->getCollisionGroup()->collide(option, result);
}

void ClassicRigidSolver::handleSkeletonAdded(
    World&, const dynamics::SkeletonPtr& skeleton)
{
  mConstraintSolver->addSkeleton(skeleton);
}

void ClassicRigidSolver::handleSkeletonRemoved(
    World&, const dynamics::SkeletonPtr& skeleton)
{
  mConstraintSolver->removeSkeleton(skeleton);
}

void ClassicRigidSolver::setTimeStep(double timeStep)
{
  mTimeStep = timeStep;
  if (mConstraintSolver) {
    mConstraintSolver->setTimeStep(timeStep);
  }
}

void ClassicRigidSolver::reset(World&)
{
  if (mConstraintSolver) {
    mConstraintSolver->clearLastCollisionResult();
  }
}

void ClassicRigidSolver::step(World& world, bool resetCommand)
{
  // Integrate velocity for unconstrained skeletons
  {
    DART_PROFILE_SCOPED_N("ClassicRigidSolver::step - Integrate velocity");
    world.eachSkeleton([&](dynamics::Skeleton* skel) {
      if (!skel->isMobile()) {
        return true;
      }

      skel->computeForwardDynamics();
      skel->integrateVelocities(mTimeStep);
      return true;
    });
  }

  // Detect activated constraints and compute constraint impulses
  {
    DART_PROFILE_SCOPED_N("ClassicRigidSolver::step - Solve constraints");
    mConstraintSolver->solve();
  }

  // Compute velocity changes given constraint impulses
  world.eachSkeleton([&](dynamics::Skeleton* skel) {
    if (!skel->isMobile()) {
      return true;
    }

    if (skel->isImpulseApplied()) {
      skel->computeImpulseForwardDynamics();
      skel->setImpulseApplied(false);
    }

    skel->integratePositions(mTimeStep);

    if (resetCommand) {
      skel->clearInternalForces();
      skel->clearExternalForces();
      skel->resetCommands();
    }

    return true;
  });
}

} // namespace dart::simulation
