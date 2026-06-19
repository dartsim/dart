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
#include "dart/simulation/compute/compute_executor.hpp"
#include "dart/simulation/compute/compute_graph.hpp"
#include "dart/simulation/compute/detail/stage_scratch.hpp"
#include "dart/simulation/compute/world_step_stage.hpp"
#include "dart/simulation/detail/entity_conversion.hpp"
#include "dart/simulation/detail/world_registry_access.hpp"
#include "dart/simulation/detail/world_storage.hpp"
#include "dart/simulation/world.hpp"

#include <dart/common/memory_manager.hpp>
#include <dart/common/stl_allocator.hpp>

#include <Eigen/Cholesky>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <entt/entt.hpp>

#include <algorithm>
#include <memory>
#include <string>
#include <vector>

#include <cmath>

namespace dart::simulation::compute {

struct RigidBodyIntegrationStage::Scratch
{
  Scratch() = default;

  explicit Scratch(common::MemoryAllocator& allocator)
    : entities(common::StlAllocator<entt::entity>{allocator}),
      orderedEntities(common::StlAllocator<entt::entity>{allocator}),
      visitState(common::StlAllocator<int>{allocator})
  {
  }

  std::vector<entt::entity, common::StlAllocator<entt::entity>> entities;
  std::vector<entt::entity, common::StlAllocator<entt::entity>> orderedEntities;
  std::vector<int, common::StlAllocator<int>> visitState;
};

namespace {

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
bool isFinite(const Eigen::Vector3d& value)
{
  return value.array().isFinite().all();
}

//==============================================================================
bool isFinite(const Eigen::Matrix3d& value)
{
  return value.array().isFinite().all();
}

//==============================================================================
void integrateAngularVelocity(
    comps::Velocity& velocity,
    const comps::MassProperties& mass,
    const comps::Force& force,
    const Eigen::Quaterniond& orientation,
    const double timeStep)
{
  if (!isFinite(force.torque) || !isFinite(mass.inertia)) {
    return;
  }

  const auto rotation = orientation.toRotationMatrix();
  const Eigen::Matrix3d worldInertia
      = rotation * mass.inertia * rotation.transpose();

  Eigen::LDLT<Eigen::Matrix3d> solver(worldInertia);
  if (solver.info() != Eigen::Success || !solver.isPositive()) {
    return;
  }

  const Eigen::Vector3d angularAcceleration = solver.solve(force.torque);
  if (solver.info() != Eigen::Success || !isFinite(angularAcceleration)) {
    return;
  }

  velocity.angular += angularAcceleration * timeStep;
}

//==============================================================================
// Update a body's velocity from an already-assembled force/torque buffer. Does
// not move the body or mutate the persistent force component.
void integrateRigidBodyVelocity(
    detail::WorldRegistry& registry,
    entt::entity entity,
    const Eigen::Vector3d& force,
    const Eigen::Vector3d& torque,
    const double timeStep)
{
  const auto& transform = registry.get<comps::Transform>(entity);
  auto& velocity = registry.get<comps::Velocity>(entity);
  const auto& mass = registry.get<comps::MassProperties>(entity);

  const auto orientation = normalizeOrIdentity(transform.orientation);

  if (mass.mass > 0.0 && std::isfinite(mass.mass)) {
    velocity.linear += (force / mass.mass) * timeStep;
  }

  comps::Force assembledForce;
  assembledForce.force = force;
  assembledForce.torque = torque;
  integrateAngularVelocity(
      velocity, mass, assembledForce, orientation, timeStep);
}

//==============================================================================
// Advance a body's pose from its current velocity and refresh its frame cache.
void integrateRigidBodyPosition(
    detail::WorldRegistry& registry, entt::entity entity, const double timeStep)
{
  auto& transform = registry.get<comps::Transform>(entity);
  const auto& velocity = registry.get<comps::Velocity>(entity);

  auto orientation = normalizeOrIdentity(transform.orientation);

  transform.position += velocity.linear * timeStep;

  const auto angularSpeed = velocity.angular.norm();
  if (angularSpeed > 0.0 && std::isfinite(angularSpeed)) {
    const auto rotation = Eigen::AngleAxisd(
        angularSpeed * timeStep, velocity.angular.normalized());
    orientation = rotation * orientation;
    orientation.normalize();
  }

  transform.orientation = normalizeOrIdentity(orientation);

  auto& props = registry.get<comps::FreeFrameProperties>(entity);
  const auto worldTransform = toIsometry(transform, transform.orientation);
  const auto& frameState = registry.get<comps::FrameState>(entity);
  props.localTransform
      = computeFrameWorldTransform(registry, frameState.parentFrame).inverse()
        * worldTransform;

  auto& cache = registry.get<comps::FrameCache>(entity);
  cache.needTransformUpdate = true;
}

//==============================================================================
bool isPrescribedRigidBodyIntegrationBody(
    const detail::WorldRegistry& registry, entt::entity entity)
{
  return registry.all_of<comps::StaticBodyTag>(entity)
         || registry.all_of<comps::KinematicBodyTag>(entity);
}

//==============================================================================
void integrateRigidBody(
    detail::WorldRegistry& registry,
    entt::entity entity,
    const Eigen::Vector3d& gravity,
    const double timeStep)
{
  if (isPrescribedRigidBodyIntegrationBody(registry, entity)) {
    return;
  }

  const auto& force = registry.get<comps::Force>(entity);
  Eigen::Vector3d assembledForce = force.force;
  const auto& mass = registry.get<comps::MassProperties>(entity);
  if (mass.mass > 0.0 && std::isfinite(mass.mass)) {
    assembledForce += mass.mass * gravity;
  }
  integrateRigidBodyVelocity(
      registry, entity, assembledForce, force.torque, timeStep);
  integrateRigidBodyPosition(registry, entity, timeStep);
}

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

} // namespace

//==============================================================================
//==============================================================================
RigidBodyIntegrationStage::RigidBodyIntegrationStage(std::size_t batchSize)
  : RigidBodyIntegrationStage(batchSize, nullptr)
{
}

//==============================================================================
RigidBodyIntegrationStage::RigidBodyIntegrationStage(
    common::MemoryManager* memoryManager)
  : RigidBodyIntegrationStage(64, memoryManager)
{
}

//==============================================================================
RigidBodyIntegrationStage::RigidBodyIntegrationStage(
    std::size_t batchSize, common::MemoryManager* memoryManager)
  : m_batchSize(std::max<std::size_t>(1, batchSize)),
    m_memoryManager(memoryManager),
    m_scratch(createScratch(memoryManager), ScratchDeleter{memoryManager})
{
}

//==============================================================================
RigidBodyIntegrationStage::~RigidBodyIntegrationStage() = default;

//==============================================================================
void RigidBodyIntegrationStage::ScratchDeleter::operator()(
    Scratch* scratch) const noexcept
{
  stage_detail::destroyStageOwnedScratch(memoryManager, scratch);
}

//==============================================================================
RigidBodyIntegrationStage::Scratch* RigidBodyIntegrationStage::createScratch(
    common::MemoryManager* memoryManager)
{
  if (memoryManager != nullptr) {
    return stage_detail::constructStageOwnedScratch<Scratch>(
        memoryManager, memoryManager->getFreeAllocator());
  }
  return stage_detail::constructStageOwnedScratch<Scratch>(nullptr);
}

//==============================================================================
std::string_view RigidBodyIntegrationStage::getName() const noexcept
{
  return "rigid_body_integration";
}

//==============================================================================
ComputeStageMetadata RigidBodyIntegrationStage::getMetadata() const noexcept
{
  return {
      ComputeStageDomain::RigidBody,
      ComputeStageAcceleration::TaskParallel
          | ComputeStageAcceleration::DataParallel
          | ComputeStageAcceleration::Simd
          | ComputeStageAcceleration::DataLocality};
}

//==============================================================================
void RigidBodyIntegrationStage::execute(World& world, ComputeExecutor& executor)
{
  auto& registry = dart::simulation::detail::registryOf(world);
  auto rigidBodyView = registry.view<
      comps::RigidBodyTag,
      comps::Transform,
      comps::Velocity,
      comps::MassProperties,
      comps::Force,
      comps::FrameState,
      comps::FreeFrameProperties,
      comps::FrameCache>();

  if (m_scratch == nullptr) {
    m_scratch = ScratchPtr(
        createScratch(m_memoryManager), ScratchDeleter{m_memoryManager});
  }
  auto& entities = m_scratch->entities;
  entities.clear();
  entities.reserve(rigidBodyView.size_hint());
  for (auto entity : rigidBodyView) {
    entities.push_back(entity);
  }

  auto graph = m_memoryManager != nullptr
                   ? ComputeGraph(m_memoryManager->getFreeAllocator())
                   : ComputeGraph();
  const auto gravity = world.getGravity();
  const auto timeStep = world.getTimeStep();

  auto* entityList = &entities;
  const bool serializeBatches = hasRigidBodyFrameDependency(registry, entities);
  if (serializeBatches) {
    orderRigidBodiesParentBeforeChild(
        registry, entities, m_scratch->orderedEntities, m_scratch->visitState);
    entityList = &m_scratch->orderedEntities;
  }

  ComputeNode* previousNode = nullptr;
  for (std::size_t begin = 0; begin < entityList->size();
       begin += m_batchSize) {
    const auto end = std::min(begin + m_batchSize, entityList->size());
    auto& node = graph.addNode(
        "rigid_body_batch_" + std::to_string(begin),
        [&registry, entityList, begin, end, gravity, timeStep]() {
          for (auto i = begin; i < end; ++i) {
            integrateRigidBody(
                registry,
                (*entityList)[static_cast<std::size_t>(i)],
                gravity,
                timeStep);
          }
        },
        getMetadata());
    if (serializeBatches && previousNode != nullptr) {
      graph.addDependency(*previousNode, node);
    }
    previousNode = &node;
  }

  executor.execute(graph);
}

//==============================================================================
std::size_t RigidBodyIntegrationStage::getBatchSize() const noexcept
{
  return m_batchSize;
}

struct RigidBodyVelocityStage::Scratch
{
  Scratch() = default;

  explicit Scratch(common::MemoryAllocator& allocator) : forces(allocator) {}

  AllocatorAwareRigidBodyForceBatch forces;
};

//==============================================================================
RigidBodyVelocityStage::RigidBodyVelocityStage()
  : RigidBodyVelocityStage(nullptr)
{
}

//==============================================================================
RigidBodyVelocityStage::RigidBodyVelocityStage(
    common::MemoryManager* memoryManager)
  : m_memoryManager(memoryManager),
    m_scratch(createScratch(memoryManager), ScratchDeleter{memoryManager})
{
}

//==============================================================================
RigidBodyVelocityStage::~RigidBodyVelocityStage() = default;

//==============================================================================
void RigidBodyVelocityStage::ScratchDeleter::operator()(
    Scratch* scratch) const noexcept
{
  stage_detail::destroyStageOwnedScratch(memoryManager, scratch);
}

//==============================================================================
RigidBodyVelocityStage::Scratch* RigidBodyVelocityStage::createScratch(
    common::MemoryManager* memoryManager)
{
  if (memoryManager != nullptr) {
    return stage_detail::constructStageOwnedScratch<Scratch>(
        memoryManager, memoryManager->getFreeAllocator());
  }
  return stage_detail::constructStageOwnedScratch<Scratch>(nullptr);
}

//==============================================================================
std::string_view RigidBodyVelocityStage::getName() const noexcept
{
  return "rigid_body_velocity";
}

//==============================================================================
ComputeStageMetadata RigidBodyVelocityStage::getMetadata() const noexcept
{
  return {
      ComputeStageDomain::RigidBody,
      ComputeStageAcceleration::TaskParallel
          | ComputeStageAcceleration::DataParallel};
}

//==============================================================================
void RigidBodyVelocityStage::prepare(World& world)
{
  if (m_scratch == nullptr) {
    m_scratch = ScratchPtr(
        createScratch(m_memoryManager), ScratchDeleter{m_memoryManager});
  }

  assembleRigidBodyForces(
      world,
      true,
      m_scratch->forces,
      RigidBodyForceAssemblyMode::AdvanceableOnly);
}

//==============================================================================
void RigidBodyVelocityStage::execute(
    World& world, ComputeExecutor& /*executor*/)
{
  auto& registry = dart::simulation::detail::registryOf(world);
  const auto timeStep = world.getTimeStep();
  if (m_scratch == nullptr) {
    m_scratch = ScratchPtr(
        createScratch(m_memoryManager), ScratchDeleter{m_memoryManager});
  }
  auto& forces = m_scratch->forces;
  assembleRigidBodyForces(
      world, true, forces, RigidBodyForceAssemblyMode::AdvanceableOnly);

  for (std::size_t i = 0; i < forces.entities.size(); ++i) {
    const auto entity = forces.entities[i];
    if (registry.all_of<comps::StaticBodyTag>(entity)
        || registry.all_of<comps::KinematicBodyTag>(entity)) {
      continue; // Static and kinematic bodies do not accelerate.
    }
    if (world.isDeactivationActiveForStep()
        && world.isDeactivationEntitySleeping(
            detail::fromRegistryEntity(entity))) {
      continue;
    }

    const Eigen::Vector3d force(
        forces.force[3 * i + 0],
        forces.force[3 * i + 1],
        forces.force[3 * i + 2]);
    const Eigen::Vector3d torque(
        forces.torque[3 * i + 0],
        forces.torque[3 * i + 1],
        forces.torque[3 * i + 2]);
    integrateRigidBodyVelocity(registry, entity, force, torque, timeStep);
  }
}

//==============================================================================
std::string_view RigidBodyPositionStage::getName() const noexcept
{
  return "rigid_body_position";
}

//==============================================================================
ComputeStageMetadata RigidBodyPositionStage::getMetadata() const noexcept
{
  return {
      ComputeStageDomain::RigidBody,
      ComputeStageAcceleration::TaskParallel
          | ComputeStageAcceleration::DataLocality};
}

//==============================================================================
void RigidBodyPositionStage::execute(
    World& world, ComputeExecutor& /*executor*/)
{
  auto& registry = dart::simulation::detail::registryOf(world);
  const auto timeStep = world.getTimeStep();

  auto view = registry.view<
      comps::RigidBodyTag,
      comps::Transform,
      comps::Velocity,
      comps::FreeFrameProperties,
      comps::FrameState,
      comps::FrameCache>();
  for (auto entity : view) {
    if (registry.all_of<comps::StaticBodyTag>(entity)
        || registry.all_of<comps::KinematicBodyTag>(entity)) {
      continue; // Static and kinematic bodies do not move under this stage.
    }
    if (world.isDeactivationActiveForStep()
        && world.isDeactivationEntitySleeping(
            detail::fromRegistryEntity(entity))) {
      continue;
    }
    integrateRigidBodyPosition(registry, entity, timeStep);
  }
}

} // namespace dart::simulation::compute
