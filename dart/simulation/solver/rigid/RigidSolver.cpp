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

#include "dart/simulation/solver/rigid/RigidSolver.hpp"

#include "dart/simulation/World.hpp"
#include "dart/simulation/detail/RigidSolverComponents.hpp"
#include "dart/simulation/detail/WorldEcsAccess.hpp"

namespace dart::simulation {

namespace {

std::size_t toFrameIndex(int frame)
{
  if (frame < 0) {
    return 0u;
  }
  return static_cast<std::size_t>(frame);
}

void updateRigidBodyState(
    const World& world, detail::comps::RigidBodyState& state)
{
  state.lastSyncedFrame = toFrameIndex(world.getSimFrames());
  state.lastSyncedTime = world.getTime();
}

} // namespace

RigidSolver::RigidSolver() : Solver("rigid") {}

std::optional<RigidSolverType> RigidSolver::getRigidSolverType() const
{
  return RigidSolverType::EntityComponent;
}

void RigidSolver::setTimeStep(double timeStep)
{
  mTimeStep = timeStep;
}

void RigidSolver::syncRigidBodyStates(World& world)
{
  auto& entityManager = detail::WorldEcsAccess::getEntityManager(world);
  auto view = entityManager.view<detail::comps::RigidBodyTag>();
  for (auto entity : view) {
    auto& state
        = entityManager.get_or_emplace<detail::comps::RigidBodyState>(entity);
    updateRigidBodyState(world, state);
  }
}

void RigidSolver::reset(World& world)
{
  syncRigidBodyStates(world);
}

void RigidSolver::step(World& world, bool)
{
  syncRigidBodyStates(world);
}

void RigidSolver::sync(World& world)
{
  syncRigidBodyStates(world);
}

void RigidSolver::handleEntityAdded(World& world, EcsEntity entity)
{
  auto& registry = detail::WorldEcsAccess::getEntityManager(world);
  const auto enttEntity = detail::WorldEcsAccess::toEntt(entity);
  if (enttEntity == entt::null || !registry.valid(enttEntity)) {
    return;
  }

  if (!registry.all_of<detail::comps::RigidBodyTag>(enttEntity)) {
    return;
  }

  auto& state
      = registry.get_or_emplace<detail::comps::RigidBodyState>(enttEntity);
  updateRigidBodyState(world, state);
}

void RigidSolver::handleEntityRemoved(World&, EcsEntity) {}

} // namespace dart::simulation
