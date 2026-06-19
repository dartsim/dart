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

#include "dart/simulation/common/exceptions.hpp"
#include "dart/simulation/comps/dynamics.hpp"
#include "dart/simulation/comps/frame_types.hpp"
#include "dart/simulation/comps/rigid_body.hpp"
#include "dart/simulation/compute/detail/stage_scratch.hpp"
#include "dart/simulation/compute/rigid_body_batch_ops.hpp"
#include "dart/simulation/compute/rigid_body_state_batch.hpp"
#include "dart/simulation/compute/world_step_stage.hpp"
#include "dart/simulation/detail/world_registry_access.hpp"
#include "dart/simulation/detail/world_storage.hpp"
#include "dart/simulation/world.hpp"

#include <dart/common/memory_manager.hpp>
#include <dart/common/stl_allocator.hpp>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <entt/entt.hpp>

#include <algorithm>
#include <memory>
#include <span>
#include <vector>

#include <cmath>

namespace dart::simulation::compute {

//==============================================================================
Eigen::Quaterniond normalizeOrIdentity(const Eigen::Quaterniond& orientation)
{
  const auto norm = orientation.norm();
  if (norm <= 0.0 || !std::isfinite(norm)) {
    return Eigen::Quaterniond::Identity();
  }

  auto normalized = orientation;
  normalized.coeffs() /= norm;
  return normalized;
}

//==============================================================================
Eigen::Isometry3d toIsometry(
    const comps::Transform& transform, const Eigen::Quaterniond& orientation)
{
  Eigen::Isometry3d localTransform = Eigen::Isometry3d::Identity();
  localTransform.linear() = orientation.toRotationMatrix();
  localTransform.translation() = transform.position;
  return localTransform;
}

//==============================================================================
Eigen::Isometry3d getLocalFrameTransform(
    const detail::WorldRegistry& registry, entt::entity entity)
{
  if (const auto* fixed
      = registry.try_get<comps::FixedFrameProperties>(entity)) {
    return fixed->localTransform;
  }

  if (const auto* free = registry.try_get<comps::FreeFrameProperties>(entity)) {
    return free->localTransform;
  }

  return Eigen::Isometry3d::Identity();
}

//==============================================================================
Eigen::Isometry3d computeFrameWorldTransform(
    const detail::WorldRegistry& registry, entt::entity entity)
{
  if (entity == entt::null) {
    return Eigen::Isometry3d::Identity();
  }

  const auto* frameState = registry.try_get<comps::FrameState>(entity);
  DART_SIMULATION_THROW_T_IF(
      !frameState,
      InvalidOperationException,
      "Frame entity is missing a FrameState component");

  const auto localTransform = getLocalFrameTransform(registry, entity);
  if (frameState->parentFrame == entt::null) {
    return localTransform;
  }

  return computeFrameWorldTransform(registry, frameState->parentFrame)
         * localTransform;
}

//==============================================================================
template <typename EntityVector>
bool containsEntity(const EntityVector& entities, entt::entity entity)
{
  return std::find(entities.begin(), entities.end(), entity) != entities.end();
}

//==============================================================================
template <typename EntityVector>
std::size_t findEntityIndex(const EntityVector& entities, entt::entity entity)
{
  for (std::size_t i = 0; i < entities.size(); ++i) {
    if (entities[i] == entity) {
      return i;
    }
  }

  return entities.size();
}

//==============================================================================
entt::entity findNearestRigidBodyAncestor(
    const detail::WorldRegistry& registry,
    entt::entity entity,
    const auto& rigidBodyEntities)
{
  while (entity != entt::null) {
    if (containsEntity(rigidBodyEntities, entity)) {
      return entity;
    }

    const auto* frameState = registry.try_get<comps::FrameState>(entity);
    DART_SIMULATION_THROW_T_IF(
        !frameState,
        InvalidOperationException,
        "Frame entity is missing a FrameState component");

    entity = frameState->parentFrame;
  }

  return entt::null;
}

//==============================================================================
bool hasRigidBodyFrameDependency(
    const detail::WorldRegistry& registry, const auto& rigidBodyEntities)
{
  for (const auto entity : rigidBodyEntities) {
    const auto& frameState = registry.get<comps::FrameState>(entity);
    if (findNearestRigidBodyAncestor(
            registry, frameState.parentFrame, rigidBodyEntities)
        != entt::null) {
      return true;
    }
  }

  return false;
}

//==============================================================================
void appendRigidBodyParentBeforeChild(
    const detail::WorldRegistry& registry,
    const auto& rigidBodyEntities,
    auto& visitState,
    auto& ordered,
    std::size_t index)
{
  if (visitState[index] == 2) {
    return;
  }

  DART_SIMULATION_THROW_T_IF(
      visitState[index] == 1,
      InvalidOperationException,
      "Rigid-body frame dependency graph contains a cycle");

  visitState[index] = 1;

  const auto entity = rigidBodyEntities[index];
  const auto& frameState = registry.get<comps::FrameState>(entity);
  const auto parentRigidBody = findNearestRigidBodyAncestor(
      registry, frameState.parentFrame, rigidBodyEntities);
  if (parentRigidBody != entt::null) {
    const auto parentIndex
        = findEntityIndex(rigidBodyEntities, parentRigidBody);
    DART_SIMULATION_THROW_T_IF(
        parentIndex == rigidBodyEntities.size(),
        InvalidOperationException,
        "Rigid-body ancestor is missing from the integration batch");

    appendRigidBodyParentBeforeChild(
        registry, rigidBodyEntities, visitState, ordered, parentIndex);
  }

  visitState[index] = 2;
  ordered.push_back(entity);
}

//==============================================================================
void orderRigidBodiesParentBeforeChild(
    const detail::WorldRegistry& registry,
    const auto& rigidBodyEntities,
    auto& ordered,
    auto& visitState)
{
  ordered.clear();
  ordered.reserve(rigidBodyEntities.size());

  visitState.assign(rigidBodyEntities.size(), 0);
  for (std::size_t i = 0; i < rigidBodyEntities.size(); ++i) {
    appendRigidBodyParentBeforeChild(
        registry, rigidBodyEntities, visitState, ordered, i);
  }
}

//==============================================================================
bool isPrescribedRigidBodyIntegrationBody(
    const detail::WorldRegistry& registry, entt::entity entity)
{
  return registry.all_of<comps::StaticBodyTag>(entity)
         || registry.all_of<comps::KinematicBodyTag>(entity);
}

namespace {

//==============================================================================
template <
    typename EntityAllocator = std::allocator<entt::entity>,
    typename ScalarAllocator = std::allocator<double>>
struct BasicRigidBodyForceBatch
{
  BasicRigidBodyForceBatch() = default;

  explicit BasicRigidBodyForceBatch(common::MemoryAllocator& allocator)
    : entities(EntityAllocator{allocator}),
      force(ScalarAllocator{allocator}),
      torque(ScalarAllocator{allocator})
  {
  }

  void clearAndReserve(std::size_t bodyCount)
  {
    entities.clear();
    force.clear();
    torque.clear();
    entities.reserve(bodyCount);
    force.reserve(3 * bodyCount);
    torque.reserve(3 * bodyCount);
  }

  std::vector<entt::entity, EntityAllocator> entities;
  std::vector<double, ScalarAllocator> force;
  std::vector<double, ScalarAllocator> torque;
};

using RigidBodyForceBatch = BasicRigidBodyForceBatch<>;
using AllocatorAwareRigidBodyForceBatch = BasicRigidBodyForceBatch<
    common::StlAllocator<entt::entity>,
    common::StlAllocator<double>>;

template <typename ScalarAllocator>
struct BasicRigidBodyStateScratchBatch
{
  using ScalarVector = std::vector<double, ScalarAllocator>;

  BasicRigidBodyStateScratchBatch() = default;

  explicit BasicRigidBodyStateScratchBatch(const ScalarAllocator& allocator)
    : position(allocator),
      orientation(allocator),
      linearVelocity(allocator),
      angularVelocity(allocator)
  {
  }

  std::size_t worldCount = 1;
  std::size_t bodyCount = 0;
  ScalarVector position;
  ScalarVector orientation;
  ScalarVector linearVelocity;
  ScalarVector angularVelocity;
};

using RigidBodyBatchScalarAllocator = common::StlAllocator<double>;
using AllocatorAwareRigidBodyStateBatch
    = BasicRigidBodyStateScratchBatch<RigidBodyBatchScalarAllocator>;

enum class RigidBodyForceAssemblyMode
{
  AllBodies,
  AdvanceableOnly,
};

//==============================================================================
template <typename ForceBatch>
void assembleRigidBodyForces(
    const World& world,
    bool includeGravity,
    ForceBatch& batch,
    RigidBodyForceAssemblyMode mode = RigidBodyForceAssemblyMode::AllBodies)
{
  const auto& registry = dart::simulation::detail::registryOf(world);
  const auto& model
      = dart::simulation::detail::ensureBakedWorldModelCurrent(world);
  batch.clearAndReserve(model.rigidBodyEntities.size());

  const Eigen::Vector3d gravity
      = includeGravity ? world.getGravity() : Eigen::Vector3d::Zero();
  for (const auto entity : model.rigidBodyEntities) {
    const bool prescribedBody
        = isPrescribedRigidBodyIntegrationBody(registry, entity);
    if (mode == RigidBodyForceAssemblyMode::AdvanceableOnly && prescribedBody) {
      continue;
    }

    batch.entities.push_back(entity);

    const auto& applied = registry.get<comps::Force>(entity);
    Eigen::Vector3d assembledForce = applied.force;
    Eigen::Vector3d assembledTorque = applied.torque;

    if (prescribedBody) {
      assembledForce.setZero();
      assembledTorque.setZero();
    } else if (includeGravity) {
      const auto& mass = registry.get<comps::MassProperties>(entity);
      if (mass.mass > 0.0 && std::isfinite(mass.mass)) {
        assembledForce += mass.mass * gravity;
      }
    }

    batch.force.push_back(assembledForce.x());
    batch.force.push_back(assembledForce.y());
    batch.force.push_back(assembledForce.z());
    batch.torque.push_back(assembledTorque.x());
    batch.torque.push_back(assembledTorque.y());
    batch.torque.push_back(assembledTorque.z());
  }
}

//==============================================================================
template <typename StateBatch>
void extractRigidBodyStateInto(const World& world, StateBatch& batch)
{
  const auto& registry = dart::simulation::detail::registryOf(world);
  const auto& model
      = dart::simulation::detail::ensureBakedWorldModelCurrent(world);

  batch.worldCount = 1;
  batch.bodyCount = model.rigidBodyEntities.size();
  batch.position.clear();
  batch.orientation.clear();
  batch.linearVelocity.clear();
  batch.angularVelocity.clear();

  const auto bodyCount = model.rigidBodyEntities.size();
  batch.position.reserve(3 * bodyCount);
  batch.orientation.reserve(4 * bodyCount);
  batch.linearVelocity.reserve(3 * bodyCount);
  batch.angularVelocity.reserve(3 * bodyCount);

  for (const auto entity : model.rigidBodyEntities) {
    const auto& transform = registry.get<comps::Transform>(entity);
    const auto& velocity = registry.get<comps::Velocity>(entity);

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
}

template <typename SourceStateBatch, typename TargetStateBatch>
void copyRigidBodyStateBatch(
    const SourceStateBatch& source, TargetStateBatch& target)
{
  target.worldCount = source.worldCount;
  target.bodyCount = source.bodyCount;
  target.position = source.position;
  target.orientation = source.orientation;
  target.linearVelocity = source.linearVelocity;
  target.angularVelocity = source.angularVelocity;
}

//==============================================================================
rigid_body_batch_ops::RigidBodyModelBatchView rigidBodyModelBatchView(
    const dart::simulation::detail::BakedWorldModel& model)
{
  return {
      1,
      model.rigidBodyEntities.size(),
      std::span<const double>{
          model.rigidBodyInverseMass.data(), model.rigidBodyInverseMass.size()},
      std::span<const double>{
          model.rigidBodyInertia.data(), model.rigidBodyInertia.size()}};
}

//==============================================================================
void restorePrescribedRigidBodyState(
    const detail::WorldRegistry& registry,
    std::span<const entt::entity> entities,
    const auto& source,
    auto& target)
{
  DART_SIMULATION_THROW_T_IF(
      source.bodyCount != entities.size()
          || target.bodyCount != entities.size(),
      InvalidOperationException,
      "Rigid-body batch state does not match the rigid-body entity count");

  for (std::size_t i = 0; i < entities.size(); ++i) {
    if (!isPrescribedRigidBodyIntegrationBody(registry, entities[i])) {
      continue;
    }

    std::copy_n(&source.position[3 * i], 3, &target.position[3 * i]);
    std::copy_n(
        &source.linearVelocity[3 * i], 3, &target.linearVelocity[3 * i]);
    std::copy_n(
        &source.angularVelocity[3 * i], 3, &target.angularVelocity[3 * i]);
    std::copy_n(&source.orientation[4 * i], 4, &target.orientation[4 * i]);
  }
}

//==============================================================================
void applyRigidBodyStateFromBatch(World& world, const auto& state)
{
  DART_SIMULATION_THROW_T_IF(
      state.worldCount != 1,
      InvalidArgumentException,
      "applyRigidBodyStateFromBatch supports single-world batches only, got "
      "worldCount {}",
      state.worldCount);

  DART_SIMULATION_THROW_T_IF(
      state.position.size() != 3 * state.bodyCount
          || state.linearVelocity.size() != 3 * state.bodyCount
          || state.angularVelocity.size() != 3 * state.bodyCount
          || state.orientation.size() != 4 * state.bodyCount,
      InvalidArgumentException,
      "Rigid-body state scratch arrays are inconsistent with bodyCount {}",
      state.bodyCount);

  auto& registry = dart::simulation::detail::registryOf(world);
  const auto& model
      = dart::simulation::detail::ensureBakedWorldModelCurrent(world);

  std::size_t index = 0;
  for (const auto entity : model.rigidBodyEntities) {
    DART_SIMULATION_THROW_T_IF(
        index >= state.bodyCount,
        InvalidArgumentException,
        "Rigid-body state scratch has fewer bodies ({}) than the world",
        state.bodyCount);

    auto& transform = registry.get<comps::Transform>(entity);
    auto& velocity = registry.get<comps::Velocity>(entity);

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

  DART_SIMULATION_THROW_T_IF(
      index != state.bodyCount,
      InvalidArgumentException,
      "Rigid-body state scratch body count ({}) does not match the world ({})",
      state.bodyCount,
      index);
}

} // namespace

//==============================================================================
struct BatchedRigidBodyIntegrationStage::Scratch
{
  Scratch() = default;

  explicit Scratch(common::MemoryAllocator& allocator)
    : forces(allocator),
      state(RigidBodyBatchScalarAllocator{allocator}),
      initialState(RigidBodyBatchScalarAllocator{allocator}),
      frameUpdateOrder(common::StlAllocator<entt::entity>{allocator}),
      visitState(common::StlAllocator<int>{allocator})
  {
  }

  AllocatorAwareRigidBodyForceBatch forces;
  AllocatorAwareRigidBodyStateBatch state;
  AllocatorAwareRigidBodyStateBatch initialState;
  std::vector<entt::entity, common::StlAllocator<entt::entity>>
      frameUpdateOrder;
  std::vector<int, common::StlAllocator<int>> visitState;
};

BatchedRigidBodyIntegrationStage::BatchedRigidBodyIntegrationStage()
  : BatchedRigidBodyIntegrationStage(nullptr)
{
}

//==============================================================================
BatchedRigidBodyIntegrationStage::BatchedRigidBodyIntegrationStage(
    common::MemoryManager* memoryManager)
  : m_memoryManager(memoryManager),
    m_scratch(createScratch(memoryManager), ScratchDeleter{memoryManager})
{
}

//==============================================================================
BatchedRigidBodyIntegrationStage::~BatchedRigidBodyIntegrationStage() = default;

//==============================================================================
void BatchedRigidBodyIntegrationStage::ScratchDeleter::operator()(
    Scratch* scratch) const noexcept
{
  stage_detail::destroyStageOwnedScratch(memoryManager, scratch);
}

//==============================================================================
BatchedRigidBodyIntegrationStage::Scratch*
BatchedRigidBodyIntegrationStage::createScratch(
    common::MemoryManager* memoryManager)
{
  if (memoryManager != nullptr) {
    return stage_detail::constructStageOwnedScratch<Scratch>(
        memoryManager, memoryManager->getFreeAllocator());
  }
  return stage_detail::constructStageOwnedScratch<Scratch>(nullptr);
}

//==============================================================================
std::string_view BatchedRigidBodyIntegrationStage::getName() const noexcept
{
  return "batched_rigid_body_integration";
}

//==============================================================================
ComputeStageMetadata BatchedRigidBodyIntegrationStage::getMetadata()
    const noexcept
{
  return {
      ComputeStageDomain::RigidBody,
      ComputeStageAcceleration::TaskParallel
          | ComputeStageAcceleration::DataParallel
          | ComputeStageAcceleration::Simd
          | ComputeStageAcceleration::DataLocality};
}

//==============================================================================
void BatchedRigidBodyIntegrationStage::execute(
    World& world, ComputeExecutor& /*executor*/)
{
  auto& registry = dart::simulation::detail::registryOf(world);
  if (m_scratch == nullptr) {
    m_scratch = ScratchPtr(
        createScratch(m_memoryManager), ScratchDeleter{m_memoryManager});
  }

  auto& scratch = *m_scratch;
  assembleRigidBodyForces(world, true, scratch.forces);
  const auto& forces = scratch.forces;
  const auto& entities = forces.entities;

  if (entities.empty()) {
    return;
  }

  extractRigidBodyStateInto(world, scratch.state);
  copyRigidBodyStateBatch(scratch.state, scratch.initialState);
  const auto timeStep = world.getTimeStep();
  const auto& model
      = dart::simulation::detail::ensureBakedWorldModelCurrent(world);

  rigid_body_batch_ops::integrateRigidBodyStateBatch(
      rigid_body_batch_ops::mutableStateBatchView(scratch.state),
      rigidBodyModelBatchView(model),
      forces.force,
      forces.torque,
      timeStep);

  restorePrescribedRigidBodyState(
      registry, entities, scratch.initialState, scratch.state);
  applyRigidBodyStateFromBatch(world, scratch.state);

  // Restore frame-cache consistency the same way the per-entity integrator
  // does, now that the world-space Transform has been written back. The SoA
  // integration itself is world-space and flat, but local transforms for
  // frame-coupled bodies must be written parent-before-child.
  orderRigidBodiesParentBeforeChild(
      registry, entities, scratch.frameUpdateOrder, scratch.visitState);
  for (const auto entity : scratch.frameUpdateOrder) {
    const auto& transform = registry.get<comps::Transform>(entity);
    auto& props = registry.get<comps::FreeFrameProperties>(entity);
    const auto worldTransform = toIsometry(transform, transform.orientation);
    const auto& frameState = registry.get<comps::FrameState>(entity);
    props.localTransform
        = computeFrameWorldTransform(registry, frameState.parentFrame).inverse()
          * worldTransform;

    auto& cache = registry.get<comps::FrameCache>(entity);
    cache.needTransformUpdate = true;
  }
}

} // namespace dart::simulation::compute
