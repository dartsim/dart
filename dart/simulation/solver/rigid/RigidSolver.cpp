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

#include "dart/dynamics/Skeleton.hpp"
#include "dart/simulation/comps/SkeletonComponents.hpp"

#include <algorithm>

namespace dart::simulation {

namespace {

void syncSkeletonState(
    comps::SkeletonState& state, const dynamics::Skeleton& skeleton)
{
  const auto dofs = static_cast<std::size_t>(skeleton.getNumDofs());
  state.positions.resize(dofs);
  state.velocities.resize(dofs);
  std::fill(state.positions.begin(), state.positions.end(), 0.0);
  std::fill(state.velocities.begin(), state.velocities.end(), 0.0);

  const auto& positions = skeleton.getPositions();
  const auto& velocities = skeleton.getVelocities();

  const auto positionsCount = static_cast<std::size_t>(positions.size());
  const auto velocitiesCount = static_cast<std::size_t>(velocities.size());

  const auto copyCount = std::min({dofs, positionsCount, velocitiesCount});
  std::copy_n(positions.data(), copyCount, state.positions.begin());
  std::copy_n(velocities.data(), copyCount, state.velocities.begin());
}

} // namespace

RigidSolver::RigidSolver(entt::registry& entityManager)
  : WorldSolver("rigid", RigidSolverType::EntityComponent),
    mEntityManager(&entityManager)
{
}

entt::registry& RigidSolver::getEntityManager()
{
  return *mEntityManager;
}

const entt::registry& RigidSolver::getEntityManager() const
{
  return *mEntityManager;
}

void RigidSolver::handleSkeletonAdded(
    World&, const dynamics::SkeletonPtr& skeleton)
{
  if (!mEntityManager || !skeleton)
    return;

  const auto* key = skeleton.get();
  if (mSkeletonEntities.find(key) != mSkeletonEntities.end())
    return;

  const auto entity = mEntityManager->create();
  mSkeletonEntities.emplace(key, entity);
  mEntityManager->emplace<comps::LegacySkeleton>(entity, skeleton);

  auto& state = mEntityManager->emplace<comps::SkeletonState>(entity);
  syncSkeletonState(state, *skeleton);
}

void RigidSolver::handleSkeletonRemoved(
    World&, const dynamics::SkeletonPtr& skeleton)
{
  if (!mEntityManager || !skeleton)
    return;

  const auto* key = skeleton.get();
  const auto it = mSkeletonEntities.find(key);
  if (it == mSkeletonEntities.end())
    return;

  mEntityManager->destroy(it->second);
  mSkeletonEntities.erase(it);
}

void RigidSolver::setTimeStep(double timeStep)
{
  mTimeStep = timeStep;
}

void RigidSolver::syncSkeletonStates()
{
  if (!mEntityManager)
    return;

  auto view
      = mEntityManager->view<comps::LegacySkeleton, comps::SkeletonState>();
  for (auto entity : view) {
    const auto& legacy = view.get<comps::LegacySkeleton>(entity);
    if (legacy.skeleton)
      syncSkeletonState(
          view.get<comps::SkeletonState>(entity), *legacy.skeleton);
  }
}

void RigidSolver::reset(World&)
{
  syncSkeletonStates();
}

void RigidSolver::step(World&, bool)
{
  syncSkeletonStates();
}

void RigidSolver::sync(World&)
{
  syncSkeletonStates();
}

} // namespace dart::simulation
