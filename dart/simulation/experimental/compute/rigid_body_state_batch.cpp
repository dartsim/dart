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

#include "dart/simulation/experimental/compute/rigid_body_state_batch.hpp"

#include "dart/simulation/experimental/common/exceptions.hpp"
#include "dart/simulation/experimental/comps/dynamics.hpp"
#include "dart/simulation/experimental/comps/rigid_body.hpp"
#include "dart/simulation/experimental/world.hpp"

#include <Eigen/Geometry>
#include <entt/entt.hpp>

namespace dart::simulation::experimental::compute {

//==============================================================================
RigidBodyStateBatch extractRigidBodyState(const World& world)
{
  const auto& registry = world.getRegistry();
  auto view
      = registry.view<comps::RigidBodyTag, comps::Transform, comps::Velocity>();

  RigidBodyStateBatch batch;
  batch.worldCount = 1;
  for (const auto entity : view) {
    ++batch.bodyCount;

    const auto& transform = view.get<comps::Transform>(entity);
    const auto& velocity = view.get<comps::Velocity>(entity);

    batch.position.push_back(transform.position.x());
    batch.position.push_back(transform.position.y());
    batch.position.push_back(transform.position.z());

    batch.orientation.push_back(transform.orientation.w());
    batch.orientation.push_back(transform.orientation.x());
    batch.orientation.push_back(transform.orientation.y());
    batch.orientation.push_back(transform.orientation.z());

    batch.linearVelocity.push_back(velocity.linear.x());
    batch.linearVelocity.push_back(velocity.linear.y());
    batch.linearVelocity.push_back(velocity.linear.z());

    batch.angularVelocity.push_back(velocity.angular.x());
    batch.angularVelocity.push_back(velocity.angular.y());
    batch.angularVelocity.push_back(velocity.angular.z());
  }

  return batch;
}

//==============================================================================
void applyRigidBodyState(World& world, const RigidBodyStateBatch& state)
{
  DART_EXPERIMENTAL_THROW_T_IF(
      state.worldCount != 1,
      InvalidArgumentException,
      "applyRigidBodyState supports single-world batches only, got worldCount "
      "{}",
      state.worldCount);

  // The fields are independently public, so validate their sizes against
  // bodyCount before indexing to avoid out-of-bounds reads on a malformed
  // batch.
  DART_EXPERIMENTAL_THROW_T_IF(
      state.position.size() != 3 * state.bodyCount
          || state.linearVelocity.size() != 3 * state.bodyCount
          || state.angularVelocity.size() != 3 * state.bodyCount
          || state.orientation.size() != 4 * state.bodyCount,
      InvalidArgumentException,
      "RigidBodyStateBatch arrays are inconsistent with bodyCount {}",
      state.bodyCount);

  auto& registry = world.getRegistry();
  auto view
      = registry.view<comps::RigidBodyTag, comps::Transform, comps::Velocity>();

  std::size_t index = 0;
  for (const auto entity : view) {
    DART_EXPERIMENTAL_THROW_T_IF(
        index >= state.bodyCount,
        InvalidArgumentException,
        "RigidBodyStateBatch has fewer bodies ({}) than the world",
        state.bodyCount);

    auto& transform = view.get<comps::Transform>(entity);
    auto& velocity = view.get<comps::Velocity>(entity);

    transform.position = Eigen::Vector3d(
        state.position[3 * index + 0],
        state.position[3 * index + 1],
        state.position[3 * index + 2]);
    transform.orientation = Eigen::Quaterniond(
        state.orientation[4 * index + 0],
        state.orientation[4 * index + 1],
        state.orientation[4 * index + 2],
        state.orientation[4 * index + 3]);
    velocity.linear = Eigen::Vector3d(
        state.linearVelocity[3 * index + 0],
        state.linearVelocity[3 * index + 1],
        state.linearVelocity[3 * index + 2]);
    velocity.angular = Eigen::Vector3d(
        state.angularVelocity[3 * index + 0],
        state.angularVelocity[3 * index + 1],
        state.angularVelocity[3 * index + 2]);

    ++index;
  }

  DART_EXPERIMENTAL_THROW_T_IF(
      index != state.bodyCount,
      InvalidArgumentException,
      "RigidBodyStateBatch body count ({}) does not match the world ({})",
      state.bodyCount,
      index);
}

//==============================================================================
RigidBodyStateBatch extractRigidBodyStateBatch(
    const std::vector<const World*>& worlds)
{
  RigidBodyStateBatch batch;
  batch.worldCount = worlds.size();
  if (worlds.empty()) {
    return batch;
  }

  for (std::size_t w = 0; w < worlds.size(); ++w) {
    DART_EXPERIMENTAL_THROW_T_IF(
        worlds[w] == nullptr,
        InvalidArgumentException,
        "extractRigidBodyStateBatch received a null world at index {}",
        w);

    const auto single = extractRigidBodyState(*worlds[w]);
    if (w == 0) {
      batch.bodyCount = single.bodyCount;
    } else {
      DART_EXPERIMENTAL_THROW_T_IF(
          single.bodyCount != batch.bodyCount,
          InvalidArgumentException,
          "Heterogeneous body counts: world 0 has {}, world {} has {}",
          batch.bodyCount,
          w,
          single.bodyCount);
    }

    batch.position.insert(
        batch.position.end(), single.position.begin(), single.position.end());
    batch.orientation.insert(
        batch.orientation.end(),
        single.orientation.begin(),
        single.orientation.end());
    batch.linearVelocity.insert(
        batch.linearVelocity.end(),
        single.linearVelocity.begin(),
        single.linearVelocity.end());
    batch.angularVelocity.insert(
        batch.angularVelocity.end(),
        single.angularVelocity.begin(),
        single.angularVelocity.end());
  }

  return batch;
}

//==============================================================================
void applyRigidBodyStateBatch(
    const std::vector<World*>& worlds, const RigidBodyStateBatch& state)
{
  DART_EXPERIMENTAL_THROW_T_IF(
      state.worldCount != worlds.size(),
      InvalidArgumentException,
      "RigidBodyStateBatch worldCount ({}) does not match the number of worlds "
      "({})",
      state.worldCount,
      worlds.size());

  const auto bodies = state.worldCount * state.bodyCount;
  DART_EXPERIMENTAL_THROW_T_IF(
      state.position.size() != 3 * bodies
          || state.linearVelocity.size() != 3 * bodies
          || state.angularVelocity.size() != 3 * bodies
          || state.orientation.size() != 4 * bodies,
      InvalidArgumentException,
      "RigidBodyStateBatch arrays are inconsistent with worldCount {} and "
      "bodyCount {}",
      state.worldCount,
      state.bodyCount);

  const auto bodyCount = state.bodyCount;
  for (std::size_t w = 0; w < worlds.size(); ++w) {
    DART_EXPERIMENTAL_THROW_T_IF(
        worlds[w] == nullptr,
        InvalidArgumentException,
        "applyRigidBodyStateBatch received a null world at index {}",
        w);

    const auto p0 = static_cast<std::ptrdiff_t>(3 * w * bodyCount);
    const auto p1 = static_cast<std::ptrdiff_t>(3 * (w + 1) * bodyCount);
    const auto o0 = static_cast<std::ptrdiff_t>(4 * w * bodyCount);
    const auto o1 = static_cast<std::ptrdiff_t>(4 * (w + 1) * bodyCount);

    RigidBodyStateBatch single;
    single.worldCount = 1;
    single.bodyCount = bodyCount;
    single.position.assign(
        state.position.begin() + p0, state.position.begin() + p1);
    single.orientation.assign(
        state.orientation.begin() + o0, state.orientation.begin() + o1);
    single.linearVelocity.assign(
        state.linearVelocity.begin() + p0, state.linearVelocity.begin() + p1);
    single.angularVelocity.assign(
        state.angularVelocity.begin() + p0, state.angularVelocity.begin() + p1);

    applyRigidBodyState(*worlds[w], single);
  }
}

} // namespace dart::simulation::experimental::compute
