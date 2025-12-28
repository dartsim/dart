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

#include "dart/simulation/solver/Solver.hpp"

#include <dart/constraint/ConstraintSolver.hpp>

#include <dart/collision/CollisionResult.hpp>

#include <dart/common/Logging.hpp>

namespace dart::simulation {

namespace {

const collision::CollisionResult& emptyCollisionResult()
{
  static const collision::CollisionResult kEmptyResult;
  return kEmptyResult;
}

} // namespace

Solver::Solver(std::string name) : mName(std::move(name)) {}

Solver::~Solver() = default;

const std::string& Solver::getName() const
{
  return mName;
}

std::optional<RigidSolverType> Solver::getRigidSolverType() const
{
  return std::nullopt;
}

bool Solver::isRigidSolver() const
{
  return getRigidSolverType().has_value();
}

bool Solver::supportsConstraints() const
{
  return false;
}

bool Solver::supportsCollision() const
{
  return false;
}

bool Solver::supportsSkeletons() const
{
  return false;
}

void Solver::reset(World&)
{
  // Default no-op
}

void Solver::sync(World&)
{
  // Default no-op
}

void Solver::handleSkeletonAdded(
    World&, const dynamics::SkeletonPtr& /*skeleton*/)
{
}

void Solver::handleSkeletonRemoved(
    World&, const dynamics::SkeletonPtr& /*skeleton*/)
{
}

void Solver::handleEntityAdded(World&, EcsEntity /*entity*/) {}

void Solver::handleEntityRemoved(World&, EcsEntity /*entity*/) {}

void Solver::setConstraintSolver(
    constraint::UniqueConstraintSolverPtr /*solver*/)
{
  DART_WARN(
      "Requested to set a constraint solver on '{}', but this solver does not "
      "support constraint backends.",
      mName);
}

constraint::ConstraintSolver* Solver::getConstraintSolver()
{
  DART_WARN(
      "Requested constraint solver from '{}', but it does not manage one.",
      mName);
  return nullptr;
}

const constraint::ConstraintSolver* Solver::getConstraintSolver() const
{
  DART_WARN(
      "Requested constraint solver from '{}', but it does not manage one.",
      mName);
  return nullptr;
}

void Solver::setCollisionDetector(
    const collision::CollisionDetectorPtr& /*collisionDetector*/)
{
  DART_WARN(
      "Requested to set a collision detector on '{}', but this solver does not "
      "support collision handling.",
      mName);
}

void Solver::setCollisionDetector(CollisionDetectorType /*collisionDetector*/)
{
  DART_WARN(
      "Requested to set a collision detector on '{}', but this solver does not "
      "support collision handling.",
      mName);
}

collision::CollisionDetectorPtr Solver::getCollisionDetector()
{
  return nullptr;
}

collision::ConstCollisionDetectorPtr Solver::getCollisionDetector() const
{
  return nullptr;
}

const collision::CollisionResult& Solver::getLastCollisionResult() const
{
  return emptyCollisionResult();
}

bool Solver::checkCollision(
    const collision::CollisionOption& /*option*/,
    collision::CollisionResult* /*result*/)
{
  return false;
}

} // namespace dart::simulation
