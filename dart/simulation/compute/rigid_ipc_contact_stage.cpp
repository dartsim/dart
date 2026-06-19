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
#include "dart/simulation/comps/collision_geometry.hpp"
#include "dart/simulation/comps/deformable_body.hpp"
#include "dart/simulation/comps/dynamics.hpp"
#include "dart/simulation/comps/frame_types.hpp"
#include "dart/simulation/comps/joint.hpp"
#include "dart/simulation/comps/rigid_body.hpp"
#include "dart/simulation/compute/compute_executor.hpp"
#include "dart/simulation/compute/detail/stage_scratch.hpp"
#include "dart/simulation/compute/detail/world_step_stages.hpp"
#include "dart/simulation/detail/deformable_vbd/avbd_row_inventory.hpp"
#include "dart/simulation/detail/entity_conversion.hpp"
#include "dart/simulation/detail/newton_barrier/friction_kernel.hpp"
#include "dart/simulation/detail/newton_barrier/mixed_domain_coupling.hpp"
#include "dart/simulation/detail/newton_barrier/restitution_damping.hpp"
#include "dart/simulation/detail/rigid_avbd/rigid_world_contact.hpp"
#include "dart/simulation/detail/rigid_ipc/rigid_ipc_barrier.hpp"
#include "dart/simulation/detail/world_registry_access.hpp"
#include "dart/simulation/detail/world_storage.hpp"
#include "dart/simulation/world.hpp"
#include "dart/simulation/world_options.hpp"

#include <dart/common/memory_manager.hpp>
#include <dart/common/stl_allocator.hpp>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <entt/entt.hpp>

#include <algorithm>
#include <array>
#include <limits>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <vector>

#include <cmath>
#include <cstdint>

namespace dart::simulation::compute {

namespace dvbd = dart::simulation::detail::deformable_vbd;
namespace nb = dart::simulation::detail::newton_barrier;
namespace sxdetail = dart::simulation::detail;

struct RigidIpcBdf2StepState
{
  bool active = false;
  bool hasPreviousPosition = false;
  sxdetail::RigidIpcVector6d previousPosition
      = sxdetail::RigidIpcVector6d::Zero();
  sxdetail::RigidIpcVector6d currentPosition
      = sxdetail::RigidIpcVector6d::Zero();
  sxdetail::RigidIpcVector6d currentVelocity
      = sxdetail::RigidIpcVector6d::Zero();
};

struct RigidIpcBdf2HistoryEntry
{
  entt::entity entity = entt::null;
  bool hasPreviousPosition = false;
  sxdetail::RigidIpcVector6d previousPosition
      = sxdetail::RigidIpcVector6d::Zero();
  sxdetail::RigidIpcVector6d currentPosition
      = sxdetail::RigidIpcVector6d::Zero();
  sxdetail::RigidIpcVector6d currentVelocity
      = sxdetail::RigidIpcVector6d::Zero();
};

struct RigidIpcRuntimeBody
{
  RigidIpcRuntimeBody() = default;

  explicit RigidIpcRuntimeBody(common::MemoryAllocator& allocator)
    : surface(allocator)
  {
  }

  entt::entity entity = entt::null;
  bool kinematic = false;
  bool hasSupportedSurface = false;
  std::size_t surfaceIndex = std::numeric_limits<std::size_t>::max();
  sxdetail::RigidIpcPose initialPose;
  sxdetail::RigidIpcVector6d initialVelocity
      = sxdetail::RigidIpcVector6d::Zero();
  RigidIpcBdf2StepState bdf2StepState;
  sxdetail::RigidIpcBarrierSurface surface;
  sxdetail::RigidIpcBodyDynamicsTerm dynamicsTerm;
};

struct RigidIpcContactStage::Scratch
{
  Scratch() = default;

  explicit Scratch(common::MemoryAllocator& allocator)
    : payloadAllocator(&allocator),
      runtimeBodies(common::StlAllocator<RigidIpcRuntimeBody>{allocator}),
      solverBodies(common::StlAllocator<RigidIpcRuntimeBody>{allocator}),
      surfaces(
          common::StlAllocator<sxdetail::RigidIpcBarrierSurface>{allocator}),
      dynamicsTerms(
          common::StlAllocator<sxdetail::RigidIpcBodyDynamicsTerm>{allocator}),
      solveDynamicsTerms(
          common::StlAllocator<sxdetail::RigidIpcBodyDynamicsTerm>{allocator}),
      articulationConstraints(
          common::StlAllocator<sxdetail::RigidIpcArticulationConstraintInput>{
              allocator}),
      bdf2Histories(common::StlAllocator<RigidIpcBdf2HistoryEntry>{allocator}),
      mixedDomainSurfaces(
          common::StlAllocator<nb::MixedDomainSurface>{allocator}),
      tracedEntities(common::StlAllocator<entt::entity>{allocator}),
      blockedEntities(common::StlAllocator<entt::entity>{allocator}),
      writebackEntities(common::StlAllocator<entt::entity>{allocator}),
      orderedEntities(common::StlAllocator<entt::entity>{allocator}),
      visitState(common::StlAllocator<int>{allocator}),
      contactPowerSum(common::StlAllocator<double>{allocator}),
      sawNonStationaryContactBody(
          common::StlAllocator<std::uint8_t>{allocator}),
      stationaryContactBody(common::StlAllocator<std::uint8_t>{allocator}),
      solveResult(allocator),
      solveScratch(allocator)
  {
  }

  common::MemoryAllocator* payloadAllocator = nullptr;
  std::vector<RigidIpcRuntimeBody, common::StlAllocator<RigidIpcRuntimeBody>>
      runtimeBodies;
  std::vector<RigidIpcRuntimeBody, common::StlAllocator<RigidIpcRuntimeBody>>
      solverBodies;
  std::vector<
      sxdetail::RigidIpcBarrierSurface,
      common::StlAllocator<sxdetail::RigidIpcBarrierSurface>>
      surfaces;
  std::vector<
      sxdetail::RigidIpcBodyDynamicsTerm,
      common::StlAllocator<sxdetail::RigidIpcBodyDynamicsTerm>>
      dynamicsTerms;
  std::vector<
      sxdetail::RigidIpcBodyDynamicsTerm,
      common::StlAllocator<sxdetail::RigidIpcBodyDynamicsTerm>>
      solveDynamicsTerms;
  std::vector<
      sxdetail::RigidIpcArticulationConstraintInput,
      common::StlAllocator<sxdetail::RigidIpcArticulationConstraintInput>>
      articulationConstraints;
  std::vector<
      RigidIpcBdf2HistoryEntry,
      common::StlAllocator<RigidIpcBdf2HistoryEntry>>
      bdf2Histories;
  std::vector<
      nb::MixedDomainSurface,
      common::StlAllocator<nb::MixedDomainSurface>>
      mixedDomainSurfaces;
  std::vector<entt::entity, common::StlAllocator<entt::entity>> tracedEntities;
  std::vector<entt::entity, common::StlAllocator<entt::entity>> blockedEntities;
  std::vector<entt::entity, common::StlAllocator<entt::entity>>
      writebackEntities;
  std::vector<entt::entity, common::StlAllocator<entt::entity>> orderedEntities;
  std::vector<int, common::StlAllocator<int>> visitState;
  std::vector<double, common::StlAllocator<double>> contactPowerSum;
  std::vector<std::uint8_t, common::StlAllocator<std::uint8_t>>
      sawNonStationaryContactBody;
  std::vector<std::uint8_t, common::StlAllocator<std::uint8_t>>
      stationaryContactBody;
  sxdetail::RigidIpcProjectedNewtonSolveResult solveResult;
  sxdetail::RigidIpcProjectedNewtonSolveScratch solveScratch;
};

namespace {

constexpr double kDefaultRigidIpcContactStageActivationDistance = 1e-2;

[[nodiscard]] bool isValidRigidIpcTimeIntegration(
    const RigidIpcTimeIntegration integration) noexcept
{
  switch (integration) {
    case RigidIpcTimeIntegration::SemiImplicit:
    case RigidIpcTimeIntegration::Bdf2:
      return true;
  }
  return false;
}

[[nodiscard]] RigidIpcContactStageOptions
makeRigidIpcContactStageOptionsForMaxIterations(const std::size_t maxIterations)
{
  RigidIpcContactStageOptions options;
  options.maxIterations = maxIterations;
  return options;
}

[[nodiscard]] RigidIpcContactStageOptions sanitizeRigidIpcContactStageOptions(
    RigidIpcContactStageOptions options)
{
  if (!std::isfinite(options.activationDistance)
      || options.activationDistance <= 0.0) {
    options.activationDistance = kDefaultRigidIpcContactStageActivationDistance;
  }
  if (!std::isfinite(options.staticFrictionSpeedBound)
      || options.staticFrictionSpeedBound < 0.0) {
    options.staticFrictionSpeedBound = 1e-3;
  }
  if (!std::isfinite(options.frictionConvergenceTolerance)
      || options.frictionConvergenceTolerance < 0.0) {
    options.frictionConvergenceTolerance = 0.0;
  }
  if (!isValidRigidIpcTimeIntegration(options.timeIntegration)) {
    options.timeIntegration = RigidIpcTimeIntegration::SemiImplicit;
  }
  return options;
}

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
double frictionOf(const comps::ContactMaterial* material)
{
  if (material != nullptr) {
    return material->friction;
  }
  return 1.0;
}

//==============================================================================
double frictionOf(const detail::WorldRegistry& registry, entt::entity entity)
{
  return frictionOf(registry.try_get<comps::ContactMaterial>(entity));
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

Eigen::Vector3d rotationVectorFromQuaternion(
    const Eigen::Quaterniond& orientation)
{
  const Eigen::Quaterniond normalized = normalizeOrIdentity(orientation);
  const Eigen::AngleAxisd angleAxis(normalized);
  if (!std::isfinite(angleAxis.angle()) || angleAxis.angle() == 0.0
      || !angleAxis.axis().allFinite()) {
    return Eigen::Vector3d::Zero();
  }

  return angleAxis.angle() * angleAxis.axis();
}

//==============================================================================
Eigen::Quaterniond quaternionFromRotationVector(const Eigen::Vector3d& rotation)
{
  const Eigen::Matrix3d matrix = sxdetail::rigidIpcRotationVectorToMatrix(
      rotation.allFinite() ? rotation : Eigen::Vector3d::Zero());
  return normalizeOrIdentity(Eigen::Quaterniond(matrix));
}

//==============================================================================
void resetRigidIpcRuntimeBodyPreservingSurface(
    RigidIpcRuntimeBody& body) noexcept
{
  body.entity = entt::null;
  body.kinematic = false;
  body.hasSupportedSurface = false;
  body.surfaceIndex = std::numeric_limits<std::size_t>::max();
  body.initialPose = sxdetail::RigidIpcPose{};
  body.initialVelocity.setZero();
  body.bdf2StepState = RigidIpcBdf2StepState{};
  body.dynamicsTerm = sxdetail::RigidIpcBodyDynamicsTerm{};

  body.surface.body = 0u;
  body.surface.pose = sxdetail::RigidIpcPose{};
  body.surface.vertices.clear();
  body.surface.triangles.clear();
  body.surface.frictionCoefficient = 1.0;
  body.surface.dynamic = true;
  body.surface.kinematic = false;
  body.surface.kinematicStartPose = sxdetail::RigidIpcPose{};
}

//==============================================================================
void copyRigidIpcSurfacePreservingCapacity(
    const sxdetail::RigidIpcBarrierSurface& source,
    sxdetail::RigidIpcBarrierSurface& target)
{
  target.body = source.body;
  target.pose = source.pose;
  target.vertices.assign(source.vertices.begin(), source.vertices.end());
  target.triangles.assign(source.triangles.begin(), source.triangles.end());
  target.frictionCoefficient = source.frictionCoefficient;
  target.dynamic = source.dynamic;
  target.kinematic = source.kinematic;
  target.kinematicStartPose = source.kinematicStartPose;
}

//==============================================================================
void copyRigidIpcRuntimeBodyPreservingSurfaceCapacity(
    const RigidIpcRuntimeBody& source, RigidIpcRuntimeBody& target)
{
  target.entity = source.entity;
  target.kinematic = source.kinematic;
  target.hasSupportedSurface = source.hasSupportedSurface;
  target.surfaceIndex = source.surfaceIndex;
  target.initialPose = source.initialPose;
  target.initialVelocity = source.initialVelocity;
  target.bdf2StepState = source.bdf2StepState;
  copyRigidIpcSurfacePreservingCapacity(source.surface, target.surface);
  target.dynamicsTerm = source.dynamicsTerm;
}

//==============================================================================
template <typename RuntimeBodyVector>
void appendRigidIpcRuntimeBody(
    RuntimeBodyVector& bodies, common::MemoryAllocator* payloadAllocator)
{
  if (payloadAllocator != nullptr) {
    bodies.emplace_back(*payloadAllocator);
  } else {
    bodies.emplace_back();
  }
}

//==============================================================================
template <typename SurfaceVector>
void resizeRigidIpcSurfacesPreservingPayloadAllocator(
    SurfaceVector& surfaces,
    const std::size_t targetSize,
    common::MemoryAllocator* payloadAllocator)
{
  while (surfaces.size() < targetSize) {
    if (payloadAllocator != nullptr) {
      surfaces.emplace_back(*payloadAllocator);
    } else {
      surfaces.emplace_back();
    }
  }
  surfaces.resize(targetSize);
}

//==============================================================================
template <
    typename RuntimeBodyVector,
    typename SolverBodyVector,
    typename SurfaceVector,
    typename DynamicsTermVector>
void prepareRigidIpcSolverScratch(
    const RuntimeBodyVector& runtimeBodies,
    SolverBodyVector& solverBodies,
    SurfaceVector& surfaces,
    DynamicsTermVector& dynamicsTerms,
    common::MemoryAllocator* payloadAllocator = nullptr)
{
  std::size_t solverCount = 0u;
  for (const auto& body : runtimeBodies) {
    if (!body.hasSupportedSurface) {
      continue;
    }

    if (solverCount == solverBodies.size()) {
      appendRigidIpcRuntimeBody(solverBodies, payloadAllocator);
    }
    copyRigidIpcRuntimeBodyPreservingSurfaceCapacity(
        body, solverBodies[solverCount]);
    solverBodies[solverCount].surfaceIndex = solverCount;
    solverBodies[solverCount].surface.body = solverCount;
    ++solverCount;
  }

  solverBodies.resize(solverCount);
  resizeRigidIpcSurfacesPreservingPayloadAllocator(
      surfaces, solverCount, payloadAllocator);
  dynamicsTerms.resize(solverCount);
  for (std::size_t i = 0; i < solverCount; ++i) {
    copyRigidIpcSurfacePreservingCapacity(solverBodies[i].surface, surfaces[i]);
    surfaces[i].body = i;
    dynamicsTerms[i] = solverBodies[i].dynamicsTerm;
  }
}

//==============================================================================
sxdetail::RigidIpcPose toRigidIpcPose(const comps::Transform& transform)
{
  sxdetail::RigidIpcPose pose;
  pose.position = transform.position;
  pose.rotation = rotationVectorFromQuaternion(transform.orientation);
  return pose;
}

//==============================================================================
comps::Transform toTransform(const sxdetail::RigidIpcPose& pose)
{
  comps::Transform transform;
  transform.position = pose.position;
  transform.orientation = quaternionFromRotationVector(pose.rotation);
  return transform;
}

//==============================================================================
sxdetail::RigidIpcVector6d poseVector(const sxdetail::RigidIpcPose& pose)
{
  sxdetail::RigidIpcVector6d vector;
  vector.head<3>() = pose.position;
  vector.tail<3>() = pose.rotation;
  return vector;
}

//==============================================================================
sxdetail::RigidIpcPose poseFromVector(const sxdetail::RigidIpcVector6d& vector)
{
  sxdetail::RigidIpcPose pose;
  pose.position = vector.head<3>();
  pose.rotation = vector.tail<3>();
  return pose;
}

//==============================================================================
template <typename Bdf2HistoryVector>
RigidIpcBdf2HistoryEntry* findRigidIpcBdf2History(
    Bdf2HistoryVector& histories, const entt::entity entity)
{
  auto it = std::find_if(
      histories.begin(), histories.end(), [entity](const auto& history) {
        return history.entity == entity;
      });
  return it == histories.end() ? nullptr : std::addressof(*it);
}

//==============================================================================
template <typename Bdf2HistoryVector>
RigidIpcBdf2HistoryEntry& findOrCreateRigidIpcBdf2History(
    Bdf2HistoryVector& histories, const entt::entity entity)
{
  if (auto* history = findRigidIpcBdf2History(histories, entity);
      history != nullptr) {
    return *history;
  }

  RigidIpcBdf2HistoryEntry history;
  history.entity = entity;
  histories.push_back(history);
  return histories.back();
}

//==============================================================================
template <typename Bdf2HistoryVector, typename RuntimeBodyVector>
void pruneRigidIpcBdf2Histories(
    Bdf2HistoryVector& histories, const RuntimeBodyVector& bodies)
{
  histories.erase(
      std::remove_if(
          histories.begin(),
          histories.end(),
          [&bodies](const RigidIpcBdf2HistoryEntry& history) {
            return history.entity == entt::null
                   || std::none_of(
                       bodies.begin(), bodies.end(), [&](const auto& body) {
                         return body.entity == history.entity
                                && body.hasSupportedSurface
                                && body.surface.dynamic;
                       });
          }),
      histories.end());
}

//==============================================================================
// Advance a kinematic body's pose by its prescribed (world-frame) linear and
// angular velocity over one timestep: x_end = x + v*dt, R_end =
// exp(omega*dt)*R.
sxdetail::RigidIpcPose integrateRigidIpcKinematicPose(
    const World& world,
    const entt::entity entity,
    const sxdetail::RigidIpcPose& startPose)
{
  const auto& registry = dart::simulation::detail::registryOf(world);
  const auto& velocity = registry.get<comps::Velocity>(entity);
  double timeStep = world.getTimeStep();
  if (const auto* tag = registry.try_get<comps::KinematicBodyTag>(entity);
      tag != nullptr && tag->maxTime.has_value()
      && std::isfinite(*tag->maxTime)) {
    const double remainingTime = *tag->maxTime - world.getTime();
    if (!(remainingTime > 0.0)) {
      timeStep = 0.0;
    } else {
      timeStep = std::min(timeStep, remainingTime);
    }
  }

  sxdetail::RigidIpcPose endPose = startPose;
  if (!(timeStep > 0.0) || !std::isfinite(timeStep)) {
    return endPose;
  }
  endPose.position = startPose.position + velocity.linear * timeStep;

  const Eigen::Vector3d rotationDelta = velocity.angular * timeStep;
  const double angle = rotationDelta.norm();
  if (std::isfinite(angle) && angle > 1e-12) {
    const Eigen::Quaterniond startQuat
        = quaternionFromRotationVector(startPose.rotation);
    const Eigen::Quaterniond deltaQuat(
        Eigen::AngleAxisd(angle, rotationDelta / angle));
    endPose.rotation
        = rotationVectorFromQuaternion((deltaQuat * startQuat).normalized());
  }
  return endPose;
}

//==============================================================================
bool isPositiveFiniteVector(const Eigen::Vector3d& value)
{
  return value.allFinite() && (value.array() > 0.0).all();
}

//==============================================================================
bool isValidRigidIpcMeshTriangle(
    const Eigen::Vector3i& triangle, const std::size_t vertexCount)
{
  if (triangle.minCoeff() < 0) {
    return false;
  }

  const auto maxVertex = static_cast<std::size_t>(triangle.maxCoeff());
  if (maxVertex >= vertexCount) {
    return false;
  }

  return triangle.x() != triangle.y() && triangle.x() != triangle.z()
         && triangle.y() != triangle.z();
}

//==============================================================================
bool isValidRigidIpcMeshShape(const CollisionShape& shape)
{
  if (shape.vertices.empty() || shape.triangles.empty()) {
    return false;
  }

  for (const Eigen::Vector3d& vertex : shape.vertices) {
    if (!vertex.allFinite()) {
      return false;
    }
  }

  for (const Eigen::Vector3i& triangle : shape.triangles) {
    if (!isValidRigidIpcMeshTriangle(triangle, shape.vertices.size())) {
      return false;
    }
  }

  return true;
}

//==============================================================================
void copyBoxToRigidIpcSurface(
    const Eigen::Vector3d& halfExtents,
    sxdetail::RigidIpcBarrierSurface& surface)
{
  const double x = halfExtents.x();
  const double y = halfExtents.y();
  const double z = halfExtents.z();
  surface.vertices
      = {Eigen::Vector3d(-x, -y, -z),
         Eigen::Vector3d(x, -y, -z),
         Eigen::Vector3d(x, y, -z),
         Eigen::Vector3d(-x, y, -z),
         Eigen::Vector3d(-x, -y, z),
         Eigen::Vector3d(x, -y, z),
         Eigen::Vector3d(x, y, z),
         Eigen::Vector3d(-x, y, z)};
  surface.triangles
      = {Eigen::Vector3i(0, 2, 1),
         Eigen::Vector3i(0, 3, 2),
         Eigen::Vector3i(4, 5, 6),
         Eigen::Vector3i(4, 6, 7),
         Eigen::Vector3i(0, 1, 5),
         Eigen::Vector3i(0, 5, 4),
         Eigen::Vector3i(1, 2, 6),
         Eigen::Vector3i(1, 6, 5),
         Eigen::Vector3i(2, 3, 7),
         Eigen::Vector3i(2, 7, 6),
         Eigen::Vector3i(3, 0, 4),
         Eigen::Vector3i(3, 4, 7)};
}

//==============================================================================
void copySphereToRigidIpcSurface(
    const double radius, sxdetail::RigidIpcBarrierSurface& surface)
{
  constexpr std::size_t latitudeBands = 6;
  constexpr std::size_t longitudeSegments = 12;
  constexpr double pi = 3.141592653589793238462643383279502884;

  surface.vertices.clear();
  surface.triangles.clear();
  surface.vertices.reserve(2 + (latitudeBands - 1) * longitudeSegments);
  surface.triangles.reserve(2 * longitudeSegments * (latitudeBands - 1));

  surface.vertices.emplace_back(0.0, 0.0, radius);
  for (std::size_t latitude = 1; latitude < latitudeBands; ++latitude) {
    const double phi = pi * static_cast<double>(latitude)
                       / static_cast<double>(latitudeBands);
    const double z = radius * std::cos(phi);
    const double ringRadius = radius * std::sin(phi);
    for (std::size_t longitude = 0; longitude < longitudeSegments;
         ++longitude) {
      const double theta = 2.0 * pi * static_cast<double>(longitude)
                           / static_cast<double>(longitudeSegments);
      surface.vertices.emplace_back(
          ringRadius * std::cos(theta), ringRadius * std::sin(theta), z);
    }
  }
  const auto bottom = static_cast<int>(surface.vertices.size());
  surface.vertices.emplace_back(0.0, 0.0, -radius);

  const auto ringVertex
      = [](std::size_t latitude, std::size_t longitude) -> int {
    return static_cast<int>(
        1 + latitude * longitudeSegments + longitude % longitudeSegments);
  };

  for (std::size_t longitude = 0; longitude < longitudeSegments; ++longitude) {
    const int current = ringVertex(0, longitude);
    const int next = ringVertex(0, longitude + 1);
    surface.triangles.emplace_back(0, current, next);
  }

  for (std::size_t latitude = 0; latitude + 1 < latitudeBands - 1; ++latitude) {
    for (std::size_t longitude = 0; longitude < longitudeSegments;
         ++longitude) {
      const int upperCurrent = ringVertex(latitude, longitude);
      const int upperNext = ringVertex(latitude, longitude + 1);
      const int lowerCurrent = ringVertex(latitude + 1, longitude);
      const int lowerNext = ringVertex(latitude + 1, longitude + 1);
      surface.triangles.emplace_back(upperCurrent, lowerCurrent, upperNext);
      surface.triangles.emplace_back(upperNext, lowerCurrent, lowerNext);
    }
  }

  const std::size_t lastRing = latitudeBands - 2;
  for (std::size_t longitude = 0; longitude < longitudeSegments; ++longitude) {
    const int current = ringVertex(lastRing, longitude);
    const int next = ringVertex(lastRing, longitude + 1);
    surface.triangles.emplace_back(bottom, next, current);
  }
}

//==============================================================================
bool copyCollisionShapeToRigidIpcSurface(
    const CollisionShape& shape, sxdetail::RigidIpcBarrierSurface& surface)
{
  const auto applyLocalTransform = [&]() {
    if (!shape.localTransform.matrix().allFinite()) {
      return false;
    }
    for (Eigen::Vector3d& vertex : surface.vertices) {
      vertex = shape.localTransform * vertex;
    }
    return true;
  };

  switch (shape.type) {
    case CollisionShapeType::Mesh:
      if (!isValidRigidIpcMeshShape(shape)) {
        return false;
      }
      surface.vertices.assign(shape.vertices.begin(), shape.vertices.end());
      surface.triangles.assign(shape.triangles.begin(), shape.triangles.end());
      return applyLocalTransform();
    case CollisionShapeType::Box:
      if (!isPositiveFiniteVector(shape.halfExtents)) {
        return false;
      }
      copyBoxToRigidIpcSurface(shape.halfExtents, surface);
      return applyLocalTransform();
    case CollisionShapeType::Sphere:
      if (!(shape.radius > 0.0) || !std::isfinite(shape.radius)) {
        return false;
      }
      copySphereToRigidIpcSurface(shape.radius, surface);
      return applyLocalTransform();
    case CollisionShapeType::Capsule:
      // Capsule rigid-IPC surfaces are not supported yet; deformable-vs-capsule
      // contact uses the analytic capsule obstacle barrier.
      return false;
    case CollisionShapeType::Cylinder:
      // Cylinder rigid-IPC surfaces are not supported yet; triangulate before
      // enabling this path so the barrier mesh has explicit surface vertices.
      return false;
    case CollisionShapeType::Plane:
      return false;
  }

  return false;
}

//==============================================================================
RigidIpcSolveStatus toPublicRigidIpcSolveStatus(
    const sxdetail::RigidIpcProjectedNewtonSolveStatus status)
{
  switch (status) {
    case sxdetail::RigidIpcProjectedNewtonSolveStatus::NoDofs:
      return RigidIpcSolveStatus::NoDofs;
    case sxdetail::RigidIpcProjectedNewtonSolveStatus::Converged:
      return RigidIpcSolveStatus::Converged;
    case sxdetail::RigidIpcProjectedNewtonSolveStatus::MaxIterations:
      return RigidIpcSolveStatus::MaxIterations;
    case sxdetail::RigidIpcProjectedNewtonSolveStatus::LineSearchBlocked:
      return RigidIpcSolveStatus::LineSearchBlocked;
    case sxdetail::RigidIpcProjectedNewtonSolveStatus::FactorizationFailed:
      return RigidIpcSolveStatus::FactorizationFailed;
  }

  return RigidIpcSolveStatus::FactorizationFailed;
}

//==============================================================================
sxdetail::RigidIpcBodyDynamicsTerm makeRuntimeRigidIpcDynamicsTerm(
    const World& world,
    const entt::entity entity,
    const sxdetail::RigidIpcPose& pose)
{
  const auto& registry = dart::simulation::detail::registryOf(world);
  const bool isStatic = registry.all_of<comps::StaticBodyTag>(entity);
  const bool isKinematic = registry.all_of<comps::KinematicBodyTag>(entity);
  // Static and kinematic bodies both have prescribed motion: they contribute no
  // dynamics objective (no inertial target) and feel no gravity in the solve.
  const bool prescribedMotion = isStatic || isKinematic;
  const auto& velocity = registry.get<comps::Velocity>(entity);
  const auto& mass = registry.get<comps::MassProperties>(entity);
  const auto& force = registry.get<comps::Force>(entity);

  sxdetail::RigidIpcBodyDynamicsState state;
  state.active = !prescribedMotion;
  state.pose = pose;
  state.velocity.head<3>() = velocity.linear;
  state.velocity.tail<3>() = velocity.angular;
  state.mass = mass.mass;
  state.inertia = mass.inertia;
  state.generalizedForce.head<3>() = force.force;
  state.generalizedForce.tail<3>() = force.torque;
  if (!prescribedMotion && mass.mass > 0.0 && std::isfinite(mass.mass)) {
    state.generalizedForce.head<3>() += mass.mass * world.getGravity();
  }

  return sxdetail::makeRigidIpcBodyDynamicsTerm(state, world.getTimeStep());
}

//==============================================================================
sxdetail::RigidIpcBodyDynamicsTerm makeRuntimeRigidIpcBdf2DynamicsTerm(
    const World& world,
    const entt::entity entity,
    const sxdetail::RigidIpcPose& pose,
    RigidIpcBdf2HistoryEntry& history,
    RigidIpcBdf2StepState& stepState)
{
  sxdetail::RigidIpcBodyDynamicsTerm term;

  const auto& registry = dart::simulation::detail::registryOf(world);
  const bool isStatic = registry.all_of<comps::StaticBodyTag>(entity);
  const bool isKinematic = registry.all_of<comps::KinematicBodyTag>(entity);
  const bool prescribedMotion = isStatic || isKinematic;
  const auto& velocity = registry.get<comps::Velocity>(entity);
  const auto& mass = registry.get<comps::MassProperties>(entity);
  const auto& force = registry.get<comps::Force>(entity);

  sxdetail::RigidIpcVector6d currentPosition = poseVector(pose);
  sxdetail::RigidIpcVector6d currentVelocity
      = sxdetail::RigidIpcVector6d::Zero();
  currentVelocity.head<3>() = velocity.linear;
  currentVelocity.tail<3>() = velocity.angular;

  constexpr double kHistoryPoseTolerance = 1e-8;
  if ((history.currentPosition - currentPosition).norm()
      > kHistoryPoseTolerance) {
    history.hasPreviousPosition = false;
    history.previousPosition.setZero();
  }
  if (!history.currentPosition.allFinite()) {
    history.hasPreviousPosition = false;
  }

  nb::Bdf2StepHistory<6> bdf2History;
  bdf2History.active = !prescribedMotion;
  bdf2History.hasPreviousPosition
      = history.hasPreviousPosition && history.previousPosition.allFinite();
  bdf2History.previousPosition = history.previousPosition;
  bdf2History.currentPosition = currentPosition;
  bdf2History.currentVelocity = currentVelocity;

  const auto inertial = nb::makeBdf2InertialTerm<6>(
      bdf2History, /*mass=*/1.0, world.getTimeStep());
  if (!inertial.active || !std::isfinite(mass.mass) || mass.mass <= 0.0
      || !mass.inertia.allFinite()) {
    return term;
  }

  term.active = true;
  term.targetPose = poseFromVector(inertial.targetPosition);
  term.diagonalWeights.head<3>().setConstant(mass.mass * inertial.scalarWeight);
  term.diagonalWeights.tail<3>()
      = (mass.inertia.diagonal().array().max(0.0) * inertial.scalarWeight)
            .matrix();
  term.generalizedForce.head<3>() = force.force;
  term.generalizedForce.tail<3>() = force.torque;
  if (!prescribedMotion) {
    term.generalizedForce.head<3>() += mass.mass * world.getGravity();
  }

  stepState.active = true;
  stepState.hasPreviousPosition = bdf2History.hasPreviousPosition;
  stepState.previousPosition = bdf2History.previousPosition;
  stepState.currentPosition = bdf2History.currentPosition;
  stepState.currentVelocity = bdf2History.currentVelocity;
  return term;
}

//==============================================================================
template <typename Bdf2HistoryVector, typename RuntimeBodyVector>
void applyRigidIpcBdf2DynamicsTerms(
    const World& world,
    Bdf2HistoryVector& histories,
    RuntimeBodyVector& bodies,
    RigidIpcSolverStats& stats)
{
  pruneRigidIpcBdf2Histories(histories, bodies);

  for (RigidIpcRuntimeBody& body : bodies) {
    if (!body.hasSupportedSurface || !body.surface.dynamic) {
      continue;
    }

    RigidIpcBdf2HistoryEntry& history
        = findOrCreateRigidIpcBdf2History(histories, body.entity);
    history.entity = body.entity;
    body.bdf2StepState = RigidIpcBdf2StepState{};
    body.dynamicsTerm = makeRuntimeRigidIpcBdf2DynamicsTerm(
        world, body.entity, body.initialPose, history, body.bdf2StepState);
    if (!body.dynamicsTerm.active || !body.bdf2StepState.active) {
      continue;
    }
    if (body.bdf2StepState.hasPreviousPosition) {
      ++stats.bdf2SecondOrderDynamicsTerms;
    } else {
      ++stats.bdf2RestartedDynamicsTerms;
    }
  }
}

//==============================================================================
template <typename RuntimeBodyVector>
std::size_t collectRigidIpcRuntimeBodies(
    const World& world,
    RigidIpcSolverStats& stats,
    RuntimeBodyVector& bodies,
    common::MemoryAllocator* payloadAllocator = nullptr)
{
  const auto& registry = dart::simulation::detail::registryOf(world);
  auto view = registry.view<
      comps::RigidBodyTag,
      comps::Transform,
      comps::Velocity,
      comps::MassProperties,
      comps::Force,
      comps::FrameState,
      comps::FreeFrameProperties,
      comps::FrameCache>();

  bodies.reserve(view.size_hint());
  std::size_t outputCount = 0u;
  for (const auto entity : view) {
    const bool isStatic = registry.all_of<comps::StaticBodyTag>(entity);
    const bool isKinematic = registry.all_of<comps::KinematicBodyTag>(entity);
    const auto* geometry = registry.try_get<comps::CollisionGeometry>(entity);
    if (geometry == nullptr && !isKinematic) {
      continue;
    }

    ++stats.bodyCount;
    if (outputCount == bodies.size()) {
      appendRigidIpcRuntimeBody(bodies, payloadAllocator);
    }
    RigidIpcRuntimeBody& body = bodies[outputCount];
    resetRigidIpcRuntimeBodyPreservingSurface(body);

    const auto& transform = view.get<comps::Transform>(entity);
    const auto& velocity = view.get<comps::Velocity>(entity);

    body.entity = entity;
    body.kinematic = isKinematic;
    body.initialPose = toRigidIpcPose(transform);
    body.initialVelocity.head<3>() = velocity.linear;
    body.initialVelocity.tail<3>() = velocity.angular;
    body.surface.body = outputCount;
    body.surface.pose = body.initialPose;
    body.surface.dynamic = !isStatic && !isKinematic;
    body.surface.kinematic = isKinematic;
    body.surface.kinematicStartPose = body.initialPose;
    body.surface.frictionCoefficient = frictionOf(registry, entity);
    if (isKinematic) {
      // The obstacle advances under its prescribed velocity over the step; the
      // solver evaluates the barrier/dynamics at this end pose while the line
      // search and lagged friction use the start->end motion.
      body.surface.pose
          = integrateRigidIpcKinematicPose(world, entity, body.initialPose);
    }
    bool copiedSurface = false;
    if (geometry != nullptr) {
      for (const CollisionShape& shape : geometry->shapes) {
        if (copyCollisionShapeToRigidIpcSurface(shape, body.surface)) {
          copiedSurface = true;
          break;
        }
      }
    }
    if (!copiedSurface) {
      ++stats.skippedUnsupportedShapeCount;
      if (!isKinematic) {
        continue;
      }
    }

    if (copiedSurface) {
      body.hasSupportedSurface = true;
      body.surfaceIndex = stats.surfaceCount;
      body.dynamicsTerm
          = makeRuntimeRigidIpcDynamicsTerm(world, entity, body.initialPose);
      if (body.surface.dynamic) {
        ++stats.dynamicBodyCount;
      }
      ++stats.surfaceCount;
    }
    ++outputCount;
  }
  return outputCount;
}

//==============================================================================
template <typename RuntimeBodyVector>
bool appendRigidIpcDeformableSurface(
    entt::entity entity,
    const comps::DeformableNodeState& state,
    const comps::DeformableMeshTopology& topology,
    double frictionCoefficient,
    RigidIpcSolverStats& stats,
    RuntimeBodyVector& bodies,
    std::size_t& outputCount,
    common::MemoryAllocator* payloadAllocator)
{
  if (state.positions.empty() || topology.surfaceTriangles.empty()) {
    return false;
  }

  using TriangleAllocator = common::StlAllocator<Eigen::Vector3i>;
  std::vector<Eigen::Vector3i, TriangleAllocator> triangles{
      payloadAllocator != nullptr ? TriangleAllocator{*payloadAllocator}
                                  : TriangleAllocator{}};
  triangles.reserve(topology.surfaceTriangles.size());
  for (const auto& triangle : topology.surfaceTriangles) {
    if (triangle.nodeA >= state.positions.size()
        || triangle.nodeB >= state.positions.size()
        || triangle.nodeC >= state.positions.size()
        || triangle.nodeA == triangle.nodeB || triangle.nodeA == triangle.nodeC
        || triangle.nodeB == triangle.nodeC) {
      continue;
    }
    const Eigen::Vector3d& a = state.positions[triangle.nodeA];
    const Eigen::Vector3d& b = state.positions[triangle.nodeB];
    const Eigen::Vector3d& c = state.positions[triangle.nodeC];
    if (!a.allFinite() || !b.allFinite() || !c.allFinite()
        || (b - a).cross(c - a).squaredNorm() <= 0.0) {
      continue;
    }
    triangles.emplace_back(
        static_cast<int>(triangle.nodeA),
        static_cast<int>(triangle.nodeB),
        static_cast<int>(triangle.nodeC));
  }

  if (triangles.empty()) {
    return false;
  }

  if (outputCount == bodies.size()) {
    appendRigidIpcRuntimeBody(bodies, payloadAllocator);
  }
  RigidIpcRuntimeBody& body = bodies[outputCount];
  resetRigidIpcRuntimeBodyPreservingSurface(body);
  body.entity = entity;
  body.hasSupportedSurface = true;
  body.surfaceIndex = stats.surfaceCount;
  body.surface.body = outputCount;
  body.surface.pose = sxdetail::RigidIpcPose{};
  body.surface.vertices.assign(state.positions.begin(), state.positions.end());
  body.surface.triangles.assign(triangles.begin(), triangles.end());
  body.surface.dynamic = false;
  body.surface.kinematic = false;
  body.surface.frictionCoefficient = std::isfinite(frictionCoefficient)
                                         ? std::max(0.0, frictionCoefficient)
                                         : 0.0;
  ++stats.surfaceCount;
  ++stats.mixedDomainDeformableSurfaceCount;
  ++outputCount;
  return true;
}

//==============================================================================
template <typename RuntimeBodyVector>
void appendRigidIpcDeformableSurfaces(
    const World& world,
    RigidIpcSolverStats& stats,
    RuntimeBodyVector& bodies,
    std::size_t& outputCount,
    common::MemoryAllocator* payloadAllocator)
{
  const auto& registry = dart::simulation::detail::registryOf(world);
  auto view = registry.view<
      comps::DeformableBodyTag,
      comps::DeformableNodeState,
      comps::DeformableMeshTopology,
      comps::DeformableMaterial>();

  for (const auto entity : view) {
    const auto& state = view.get<comps::DeformableNodeState>(entity);
    const auto& topology = view.get<comps::DeformableMeshTopology>(entity);
    const auto& material = view.get<comps::DeformableMaterial>(entity);
    appendRigidIpcDeformableSurface(
        entity,
        state,
        topology,
        material.frictionCoefficient,
        stats,
        bodies,
        outputCount,
        payloadAllocator);
  }
  bodies.resize(outputCount);
}

//==============================================================================
void assignRigidIpcSurfaceWorldVertices(
    const sxdetail::RigidIpcBarrierSurface& surface,
    const sxdetail::RigidIpcPose& pose,
    auto& vertices)
{
  vertices.clear();
  vertices.reserve(surface.vertices.size());
  for (const Eigen::Vector3d& vertex : surface.vertices) {
    vertices.push_back(sxdetail::transformRigidIpcPoint(vertex, pose));
  }
}

//==============================================================================
template <typename RuntimeBodyVector, typename MixedDomainSurfaceVector>
void collectRigidIpcMixedDomainCandidateStats(
    const dart::simulation::detail::WorldRegistry& registry,
    const RuntimeBodyVector& bodies,
    const double activationDistance,
    RigidIpcSolverStats& stats,
    MixedDomainSurfaceVector& surfaces,
    common::MemoryAllocator* payloadAllocator)
{
  if (stats.mixedDomainDeformableSurfaceCount == 0u) {
    stats.mixedDomainSurfaceCount = 0u;
    stats.mixedDomainCandidateCount = 0u;
    stats.mixedDomainActiveBarrierCount = 0u;
    stats.mixedDomainBarrierValue = 0.0;
    return;
  }

  surfaces.reserve(bodies.size());
  std::size_t outputCount = 0u;
  for (std::size_t i = 0; i < bodies.size(); ++i) {
    const RigidIpcRuntimeBody& body = bodies[i];
    if (!body.hasSupportedSurface) {
      continue;
    }

    const bool deformableSurface
        = body.entity != entt::null
          && registry.all_of<comps::DeformableBodyTag>(body.entity);
    if (outputCount == surfaces.size()) {
      if (payloadAllocator != nullptr) {
        surfaces.emplace_back(*payloadAllocator);
      } else {
        surfaces.emplace_back();
      }
    }
    nb::MixedDomainSurface& surface = surfaces[outputCount];
    surface.startVertices.clear();
    surface.endVertices.clear();
    surface.edges.clear();
    surface.triangles.clear();
    surface.domain = deformableSurface ? nb::MixedDomainType::Deformable
                                       : nb::MixedDomainType::Rigid;
    surface.domainInstance = i;
    surface.active = true;
    surface.dynamic = body.surface.dynamic;
    surface.frictionCoefficient = body.surface.frictionCoefficient;
    assignRigidIpcSurfaceWorldVertices(
        body.surface,
        body.surface.kinematic ? body.surface.kinematicStartPose
                               : body.surface.pose,
        surface.startVertices);
    assignRigidIpcSurfaceWorldVertices(
        body.surface, body.surface.pose, surface.endVertices);
    surface.triangles.assign(
        body.surface.triangles.begin(), body.surface.triangles.end());
    ++outputCount;
  }
  surfaces.resize(outputCount);

  nb::MixedDomainCandidateOptions options;
  options.activationDistance = activationDistance;
  options.exactDistanceFilter = true;
  const std::span<const nb::MixedDomainSurface> surfaceSpan{
      surfaces.data(), surfaces.size()};
  const nb::MixedDomainCandidateSet candidateSet
      = nb::buildMixedDomainContactCandidates(
          surfaceSpan, options, payloadAllocator);
  const nb::MixedDomainBarrierDiagnostics diagnostics
      = nb::evaluateMixedDomainBarrierDiagnostics(
          surfaceSpan, candidateSet, activationDistance);
  stats.mixedDomainSurfaceCount = candidateSet.stats.surfaceCount;
  stats.mixedDomainCandidateCount = candidateSet.candidates.size();
  stats.mixedDomainActiveBarrierCount = diagnostics.activeBarrierCount;
  stats.mixedDomainBarrierValue = diagnostics.value;
}

//==============================================================================
template <typename RuntimeBodyVector>
std::size_t findRuntimeBodyIndex(
    const RuntimeBodyVector& bodies, const entt::entity entity)
{
  for (std::size_t i = 0; i < bodies.size(); ++i) {
    if (bodies[i].entity == entity) {
      return i;
    }
  }
  return bodies.size();
}

//==============================================================================
template <typename RuntimeBodyVector>
std::size_t findSolverBodyIndex(
    const RuntimeBodyVector& bodies, const entt::entity entity)
{
  for (std::size_t i = 0; i < bodies.size(); ++i) {
    if (bodies[i].entity == entity) {
      return i;
    }
  }
  return bodies.size();
}

//==============================================================================
template <typename ConstraintVector>
bool appendRigidIpcHingeAxisConstraint(
    const std::size_t bodyA,
    const std::size_t bodyB,
    const Eigen::Vector3d& parentLocalAxis,
    const Eigen::Quaterniond& targetParentToChild,
    ConstraintVector& constraints)
{
  if (!parentLocalAxis.allFinite() || parentLocalAxis.squaredNorm() <= 0.0
      || !targetParentToChild.coeffs().allFinite()
      || targetParentToChild.norm() == 0.0) {
    return false;
  }

  const Eigen::Vector3d localAxisA = parentLocalAxis.normalized();
  const Eigen::Vector3d localAxisB
      = targetParentToChild.conjugate() * localAxisA;
  if (!localAxisB.allFinite() || localAxisB.squaredNorm() <= 0.0) {
    return false;
  }

  sxdetail::RigidIpcArticulationConstraintInput hinge;
  hinge.type = sxdetail::RigidIpcArticulationConstraintType::HingeAxis;
  hinge.bodyA = bodyA;
  hinge.bodyB = bodyB;
  hinge.localAxisA = localAxisA;
  hinge.localAxisB = localAxisB.normalized();
  constraints.push_back(hinge);
  return true;
}

//==============================================================================
template <typename RuntimeBodyVector, typename ConstraintVector>
void collectRigidIpcArticulationConstraints(
    const World& world,
    const RuntimeBodyVector& solverBodies,
    ConstraintVector& constraints)
{
  const auto& registry = dart::simulation::detail::registryOf(world);
  constraints.clear();

  auto view
      = registry
            .view<comps::JointModel, dvbd::AvbdRigidWorldPointJointConfig>();
  for (const auto jointEntity : view) {
    const auto& joint = view.get<comps::JointModel>(jointEntity);
    const auto& jointState = registry.get<comps::JointState>(jointEntity);
    const auto& config
        = view.get<dvbd::AvbdRigidWorldPointJointConfig>(jointEntity);
    if (!config.enabled || jointState.broken || joint.breakForce > 0.0
        || joint.parentLink == entt::null || joint.childLink == entt::null
        || (joint.type != comps::JointType::Fixed
            && joint.type != comps::JointType::Revolute)
        || !config.localAnchorA.allFinite() || !config.localAnchorB.allFinite()
        || !config.angularAxes.allFinite()
        || !config.targetRelativeOrientation.coeffs().allFinite()
        || config.targetRelativeOrientation.norm() == 0.0) {
      continue;
    }

    const std::size_t bodyA
        = findSolverBodyIndex(solverBodies, joint.parentLink);
    const std::size_t bodyB
        = findSolverBodyIndex(solverBodies, joint.childLink);
    if (bodyA >= solverBodies.size() || bodyB >= solverBodies.size()) {
      continue;
    }

    sxdetail::RigidIpcArticulationConstraintInput point;
    point.type = sxdetail::RigidIpcArticulationConstraintType::PointConnection;
    point.bodyA = bodyA;
    point.bodyB = bodyB;
    point.localPointA = config.localAnchorA;
    point.localPointB = config.localAnchorB;
    constraints.push_back(point);

    if (joint.type != comps::JointType::Revolute) {
      const Eigen::Quaterniond targetParentToChild
          = config.targetRelativeOrientation.normalized();
      appendRigidIpcHingeAxisConstraint(
          bodyA,
          bodyB,
          config.angularAxes.col(0),
          targetParentToChild,
          constraints);
      appendRigidIpcHingeAxisConstraint(
          bodyA,
          bodyB,
          config.angularAxes.col(1),
          targetParentToChild,
          constraints);
      continue;
    }

    appendRigidIpcHingeAxisConstraint(
        bodyA,
        bodyB,
        config.angularAxes.col(2),
        config.targetRelativeOrientation.normalized(),
        constraints);
  }
}

//==============================================================================
template <typename EntityVector>
void clearKinematicBodyStepTraces(World& world, EntityVector& tracedEntities)
{
  auto& registry = dart::simulation::detail::registryOf(world);
  auto view = registry.view<comps::KinematicBodyStepTrace>();
  tracedEntities.clear();
  tracedEntities.reserve(view.size());
  for (const auto entity : view) {
    tracedEntities.push_back(entity);
  }
  for (const auto entity : tracedEntities) {
    registry.remove<comps::KinematicBodyStepTrace>(entity);
  }
}

//==============================================================================
void applyRigidIpcPoseToRuntimeBody(
    World& world,
    const RigidIpcRuntimeBody& body,
    const sxdetail::RigidIpcPose& pose,
    auto* bdf2Histories)
{
  auto& registry = dart::simulation::detail::registryOf(world);
  auto& transform = registry.get<comps::Transform>(body.entity);
  auto& velocity = registry.get<comps::Velocity>(body.entity);

  const double timeStep = world.getTimeStep();
  const sxdetail::RigidIpcVector6d acceptedPosition = poseVector(pose);
  sxdetail::RigidIpcVector6d acceptedVelocity
      = sxdetail::RigidIpcVector6d::Zero();
  transform.position = pose.position;
  transform.orientation = quaternionFromRotationVector(pose.rotation);
  if (timeStep > 0.0 && std::isfinite(timeStep)) {
    acceptedVelocity.head<3>()
        = (pose.position - body.initialPose.position) / timeStep;
    acceptedVelocity.tail<3>()
        = (pose.rotation - body.initialPose.rotation) / timeStep;
    if (bdf2Histories != nullptr && body.bdf2StepState.active) {
      nb::Bdf2StepHistory<6> history;
      history.active = true;
      history.hasPreviousPosition = body.bdf2StepState.hasPreviousPosition;
      history.previousPosition = body.bdf2StepState.previousPosition;
      history.currentPosition = body.bdf2StepState.currentPosition;
      history.currentVelocity = body.bdf2StepState.currentVelocity;
      const auto update
          = nb::makeBdf2VelocityUpdate<6>(history, acceptedPosition, timeStep);
      if (update.active && update.velocity.allFinite()) {
        acceptedVelocity = update.velocity;
      }
    }
    velocity.linear = acceptedVelocity.head<3>();
    velocity.angular = acceptedVelocity.tail<3>();
  }

  if (bdf2Histories != nullptr && body.bdf2StepState.active
      && acceptedPosition.allFinite() && acceptedVelocity.allFinite()) {
    RigidIpcBdf2HistoryEntry& history
        = findOrCreateRigidIpcBdf2History(*bdf2Histories, body.entity);
    history.entity = body.entity;
    history.hasPreviousPosition = true;
    history.previousPosition = body.bdf2StepState.currentPosition;
    history.currentPosition = acceptedPosition;
    history.currentVelocity = acceptedVelocity;
  }

  auto& props = registry.get<comps::FreeFrameProperties>(body.entity);
  const auto worldTransform = toIsometry(transform, transform.orientation);
  const auto& frameState = registry.get<comps::FrameState>(body.entity);
  props.localTransform
      = computeFrameWorldTransform(registry, frameState.parentFrame).inverse()
        * worldTransform;

  auto& cache = registry.get<comps::FrameCache>(body.entity);
  cache.needTransformUpdate = true;
}

//==============================================================================
// Advance a kinematic obstacle to its prescribed end-of-step pose. Unlike the
// dynamic write-back this preserves the body's velocity (the prescribed
// kinematic velocity), so the obstacle keeps moving at a constant rate.
void applyKinematicRuntimeBody(World& world, const RigidIpcRuntimeBody& body)
{
  auto& registry = dart::simulation::detail::registryOf(world);
  auto& transform = registry.get<comps::Transform>(body.entity);

  comps::KinematicBodyStepTrace trace;
  trace.frame = world.getFrame();
  trace.startTransform = toTransform(body.initialPose);
  trace.endTransform = toTransform(body.surface.pose);
  registry.emplace_or_replace<comps::KinematicBodyStepTrace>(
      body.entity, std::move(trace));

  transform.position = body.surface.pose.position;
  transform.orientation
      = quaternionFromRotationVector(body.surface.pose.rotation);

  auto& props = registry.get<comps::FreeFrameProperties>(body.entity);
  const auto worldTransform = toIsometry(transform, transform.orientation);
  const auto& frameState = registry.get<comps::FrameState>(body.entity);
  props.localTransform
      = computeFrameWorldTransform(registry, frameState.parentFrame).inverse()
        * worldTransform;

  auto& cache = registry.get<comps::FrameCache>(body.entity);
  cache.needTransformUpdate = true;
}

//==============================================================================
void applyRigidIpcRuntimeResult(
    World& world,
    const auto& bodies,
    const sxdetail::RigidIpcProjectedNewtonSolveResult& result,
    auto* bdf2Histories,
    auto& writebackEntities,
    auto& orderedEntities,
    auto& visitState)
{
  auto& registry = dart::simulation::detail::registryOf(world);
  writebackEntities.clear();
  writebackEntities.reserve(bodies.size());
  for (const auto& body : bodies) {
    if (body.surface.dynamic || body.kinematic) {
      writebackEntities.push_back(body.entity);
    }
  }

  orderRigidBodiesParentBeforeChild(
      registry, writebackEntities, orderedEntities, visitState);
  for (const auto entity : orderedEntities) {
    const std::size_t bodyIndex = findRuntimeBodyIndex(bodies, entity);
    if (bodyIndex >= bodies.size()) {
      continue;
    }
    const auto& body = bodies[bodyIndex];
    if (body.kinematic) {
      applyKinematicRuntimeBody(world, body);
    } else if (body.surface.dynamic && body.hasSupportedSurface) {
      if (body.surfaceIndex >= result.surfaces.size()) {
        continue;
      }
      applyRigidIpcPoseToRuntimeBody(
          world, body, result.surfaces[body.surfaceIndex].pose, bdf2Histories);
    }
  }
}

//==============================================================================
void applyRigidIpcKinematicRuntimeBodies(
    World& world,
    const auto& bodies,
    const auto& blockedKinematicEntities,
    auto& writebackEntities,
    auto& orderedEntities,
    auto& visitState)
{
  auto& registry = dart::simulation::detail::registryOf(world);
  writebackEntities.clear();
  writebackEntities.reserve(bodies.size());
  for (const auto& body : bodies) {
    if (body.kinematic
        && std::find(
               blockedKinematicEntities.begin(),
               blockedKinematicEntities.end(),
               body.entity)
               == blockedKinematicEntities.end()) {
      writebackEntities.push_back(body.entity);
    }
  }

  orderRigidBodiesParentBeforeChild(
      registry, writebackEntities, orderedEntities, visitState);
  for (const auto entity : orderedEntities) {
    const std::size_t bodyIndex = findRuntimeBodyIndex(bodies, entity);
    if (bodyIndex >= bodies.size()) {
      continue;
    }
    applyKinematicRuntimeBody(world, bodies[bodyIndex]);
  }
}

//==============================================================================
void blockRejectedRigidIpcKinematicBody(
    const auto& solverBodies,
    const std::size_t solverBodyIndex,
    auto& blockedEntities)
{
  if (solverBodyIndex >= solverBodies.size()
      || !solverBodies[solverBodyIndex].kinematic) {
    return;
  }

  const entt::entity entity = solverBodies[solverBodyIndex].entity;
  if (std::find(blockedEntities.begin(), blockedEntities.end(), entity)
      == blockedEntities.end()) {
    blockedEntities.push_back(entity);
  }
}

//==============================================================================
void blockedKinematicEntitiesAfterRejectedRigidIpcSolve(
    const auto& solverBodies,
    const sxdetail::RigidIpcProjectedNewtonSolveResult& result,
    auto& blockedEntities)
{
  blockedEntities.clear();
  const auto blockBodyPair = [&](const std::size_t bodyA,
                                 const std::size_t bodyB) {
    blockRejectedRigidIpcKinematicBody(solverBodies, bodyA, blockedEntities);
    blockRejectedRigidIpcKinematicBody(solverBodies, bodyB, blockedEntities);
  };

  for (const auto& constraint : result.assembly.activeConstraints) {
    blockBodyPair(constraint.bodyA, constraint.bodyB);
  }
  for (const auto& constraint : result.assembly.activeFrictionConstraints) {
    blockBodyPair(constraint.bodyA, constraint.bodyB);
  }
  for (const auto& constraint : result.assembly.activeArticulationConstraints) {
    blockBodyPair(constraint.bodyA, constraint.bodyB);
  }
  if (result.lineSearch.limited || !result.lineSearch.allowsPositiveStep()) {
    blockBodyPair(result.lineSearch.bodyA, result.lineSearch.bodyB);
  }
}

//==============================================================================
void applyRigidIpcKinematicRuntimeBodiesAfterRejectedSolve(
    World& world,
    const auto& runtimeBodies,
    const auto& solverBodies,
    const sxdetail::RigidIpcProjectedNewtonSolveResult& result,
    auto& blockedEntities,
    auto& writebackEntities,
    auto& orderedEntities,
    auto& visitState)
{
  // Rejected dynamic solve results are discarded, but kinematic bodies that did
  // not participate in active IPC rows or the limiting CCD pair still have an
  // independent prescribed motion. Block only the involved supported surfaces;
  // unsupported kinematic bodies never entered the solve and remain
  // advanceable.
  blockedKinematicEntitiesAfterRejectedRigidIpcSolve(
      solverBodies, result, blockedEntities);
  applyRigidIpcKinematicRuntimeBodies(
      world,
      runtimeBodies,
      blockedEntities,
      writebackEntities,
      orderedEntities,
      visitState);
}

//==============================================================================
bool canApplyRestingContactNoOp(
    const auto& bodies,
    const sxdetail::RigidIpcProjectedNewtonSolveResult& result,
    auto& contactPowerSum,
    auto& sawNonStationaryContactBody,
    auto& stationaryContactBody)
{
  if (!result.failed
      || result.status
             != sxdetail::RigidIpcProjectedNewtonSolveStatus::LineSearchBlocked
      || result.stats.acceptedSteps != 0u
      || result.stats.lineSearchZeroStepCount == 0u
      || result.assembly.activeConstraints.empty()) {
    return false;
  }

  constexpr double kStationaryVelocityTolerance = 1e-10;
  constexpr double kContactPowerTolerance = 1e-12;
  bool sawDynamicContactBody = false;
  contactPowerSum.assign(bodies.size(), 0.0);
  sawNonStationaryContactBody.assign(bodies.size(), std::uint8_t{0});
  stationaryContactBody.assign(bodies.size(), std::uint8_t{0});
  const auto accumulateContactPower = [&](const auto& constraint) {
    const std::array<std::size_t, 2> bodyIndices{
        constraint.bodyA, constraint.bodyB};
    for (std::size_t localBody = 0; localBody < bodyIndices.size();
         ++localBody) {
      const std::size_t bodyIndex = bodyIndices[localBody];
      if (bodyIndex >= bodies.size() || !bodies[bodyIndex].surface.dynamic) {
        continue;
      }

      sawDynamicContactBody = true;
      const auto& velocity = bodies[bodyIndex].initialVelocity;
      if (!velocity.allFinite()) {
        return false;
      }
      if (velocity.norm() <= kStationaryVelocityTolerance) {
        stationaryContactBody[bodyIndex] = std::uint8_t{1};
        continue;
      }
      sawNonStationaryContactBody[bodyIndex] = std::uint8_t{1};

      const auto gradient = constraint.reduced.gradient.template segment<6>(
          static_cast<Eigen::Index>(6 * localBody));
      if (!gradient.allFinite()) {
        return false;
      }
      const double barrierPower = gradient.dot(velocity);
      if (!std::isfinite(barrierPower)) {
        return false;
      }
      contactPowerSum[bodyIndex] += barrierPower;
    }
    return true;
  };

  for (const auto& constraint : result.assembly.activeConstraints) {
    if (!accumulateContactPower(constraint)) {
      return false;
    }
  }
  for (const auto& constraint : result.assembly.activeFrictionConstraints) {
    if (!accumulateContactPower(constraint)) {
      return false;
    }
  }

  for (std::size_t bodyIndex = 0; bodyIndex < bodies.size(); ++bodyIndex) {
    if (stationaryContactBody[bodyIndex] == 0u
        && sawNonStationaryContactBody[bodyIndex] == 0u) {
      continue;
    }
    if (sawNonStationaryContactBody[bodyIndex] == 0u) {
      continue;
    }
    if (std::abs(contactPowerSum[bodyIndex]) <= kContactPowerTolerance) {
      return false;
    }
  }

  return sawDynamicContactBody;
}

} // namespace

//==============================================================================
RigidIpcContactStage::RigidIpcContactStage(const std::size_t maxIterations)
  : RigidIpcContactStage(
        makeRigidIpcContactStageOptionsForMaxIterations(maxIterations))
{
}

//==============================================================================
RigidIpcContactStage::RigidIpcContactStage(
    const std::size_t maxIterations, common::MemoryManager* memoryManager)
  : RigidIpcContactStage(
        makeRigidIpcContactStageOptionsForMaxIterations(maxIterations),
        memoryManager)
{
}

//==============================================================================
RigidIpcContactStage::RigidIpcContactStage(RigidIpcContactStageOptions options)
  : RigidIpcContactStage(options, nullptr)
{
}

//==============================================================================
RigidIpcContactStage::RigidIpcContactStage(
    RigidIpcContactStageOptions options, common::MemoryManager* memoryManager)
  : m_options(sanitizeRigidIpcContactStageOptions(options)),
    m_memoryManager(memoryManager),
    m_scratch(
        memoryManager != nullptr
            ? stage_detail::constructStageOwnedScratch<Scratch>(
                  memoryManager, memoryManager->getFreeAllocator())
            : stage_detail::constructStageOwnedScratch<Scratch>(nullptr),
        ScratchDeleter{memoryManager})
{
}

//==============================================================================
RigidIpcContactStage::~RigidIpcContactStage() = default;

//==============================================================================
void RigidIpcContactStage::ScratchDeleter::operator()(
    Scratch* scratch) const noexcept
{
  stage_detail::destroyStageOwnedScratch(memoryManager, scratch);
}

//==============================================================================
std::string_view RigidIpcContactStage::getName() const noexcept
{
  return "rigid_ipc_contact";
}

//==============================================================================
ComputeStageMetadata RigidIpcContactStage::getMetadata() const noexcept
{
  return {
      ComputeStageDomain::Constraint,
      ComputeStageAcceleration::TaskParallel
          | ComputeStageAcceleration::DataLocality,
      {{"rigid_body.transform", ComputeAccessMode::ReadWrite},
       {"rigid_body.velocity", ComputeAccessMode::ReadWrite},
       {"rigid_body.mass", ComputeAccessMode::Read},
       {"rigid_body.force", ComputeAccessMode::Read},
       {"rigid_body.joint", ComputeAccessMode::Read},
       {"rigid_body.kinematic_step_trace", ComputeAccessMode::ReadWrite},
       {"deformable_body.state", ComputeAccessMode::Read},
       {"deformable_body.model", ComputeAccessMode::Read},
       {"deformable_body.topology", ComputeAccessMode::Read},
       {"collision_geometry", ComputeAccessMode::Read}}};
}

//==============================================================================
namespace {

// Build the projected-Newton solve options for a rigid-IPC step: the adaptive
// barrier-stiffness inputs (world AABB diagonal over all collision surfaces and
// the average dynamic-body mass), barrier/friction settings, and the moved-in
// dynamics terms. Shared by execute() (the real solve) and prepare() (the
// bake-time warm solve) so a baked step warms exactly the buffers the real
// solve allocates, taking the identical adaptive-kappa configuration rather
// than a divergent fixed-stiffness assemble-only prewarm.
template <
    typename SolverBodies,
    typename SolveDynamicsTerms,
    typename DynamicsTerms,
    typename ArticulationConstraints>
sxdetail::RigidIpcProjectedNewtonSolveOptions buildRigidIpcSolveOptions(
    World& world,
    const SolverBodies& solverBodies,
    SolveDynamicsTerms& solveDynamicsTerms,
    const DynamicsTerms& dynamicsTerms,
    const ArticulationConstraints& articulationConstraints,
    const RigidIpcContactStageOptions& stageOptions,
    common::MemoryManager* memoryManager,
    double adaptiveBarrierStiffnessLowerBound)
{
  solveDynamicsTerms.assign(dynamicsTerms.begin(), dynamicsTerms.end());

  const double timeStep = world.getTimeStep();
  Eigen::Vector3d aabbMin
      = Eigen::Vector3d::Constant(std::numeric_limits<double>::infinity());
  Eigen::Vector3d aabbMax
      = Eigen::Vector3d::Constant(-std::numeric_limits<double>::infinity());
  double massSum = 0.0;
  std::size_t massCount = 0;
  for (const auto& body : solverBodies) {
    const Eigen::Matrix3d rotation
        = sxdetail::rigidIpcRotationVectorToMatrix(body.surface.pose.rotation);
    for (const Eigen::Vector3d& localVertex : body.surface.vertices) {
      const Eigen::Vector3d worldVertex
          = rotation * localVertex + body.surface.pose.position;
      aabbMin = aabbMin.cwiseMin(worldVertex);
      aabbMax = aabbMax.cwiseMax(worldVertex);
    }
    if (body.surface.dynamic && body.dynamicsTerm.active && timeStep > 0.0
        && std::isfinite(timeStep)) {
      const double mass
          = body.dynamicsTerm.diagonalWeights[0] * timeStep * timeStep;
      if (std::isfinite(mass) && mass > 0.0) {
        massSum += mass;
        ++massCount;
      }
    }
  }
  double bboxDiagonal = 1.0;
  if (aabbMin.allFinite() && aabbMax.allFinite()
      && (aabbMax.array() >= aabbMin.array()).all()) {
    const double diagonal = (aabbMax - aabbMin).norm();
    if (std::isfinite(diagonal) && diagonal > 0.0) {
      bboxDiagonal = diagonal;
    }
  }
  const double averageMass
      = massCount > 0u ? massSum / static_cast<double>(massCount) : 1.0;

  sxdetail::RigidIpcProjectedNewtonSolveOptions options
      = memoryManager != nullptr
            ? sxdetail::
                  RigidIpcProjectedNewtonSolveOptions{memoryManager
                                                          ->getFreeAllocator()}
            : sxdetail::RigidIpcProjectedNewtonSolveOptions{};
  options.barrier.squaredActivationDistance
      = stageOptions.activationDistance * stageOptions.activationDistance;
  options.barrier.stiffness = std::max(1.0, adaptiveBarrierStiffnessLowerBound);
  options.lineSearch.maxIterations = 256;
  options.adaptiveStiffness.enabled = true;
  options.adaptiveStiffness.averageMass = averageMass;
  options.adaptiveStiffness.bboxDiagonal = bboxDiagonal;
  options.friction.coefficient = 1.0;
  options.friction.staticFrictionDisplacement
      = std::max(0.0, stageOptions.staticFrictionSpeedBound * timeStep);
  options.dynamicsTerms = std::move(solveDynamicsTerms);
  options.articulationConstraints = articulationConstraints;
  options.maxIterations = stageOptions.maxIterations;
  options.frictionIterations = stageOptions.frictionIterations;
  options.frictionConvergenceTolerance
      = stageOptions.frictionConvergenceTolerance;
  options.useLineSearch = true;
  options.newton.gradientTolerance = 1e-10;
  options.newton.relativeGradientTolerance = 1e-6;
  options.stepTolerance = 1e-12;
  return options;
}

} // namespace

void RigidIpcContactStage::prepare(World& world)
{
  auto& registry = dart::simulation::detail::registryOf(world);
  auto bodyView = registry.view<comps::RigidBodyTag>();
  const auto bodyCount = bodyView.size();
  auto deformableView = registry.view<comps::DeformableBodyTag>();
  const auto mixedDomainSurfaceCount = bodyCount + deformableView.size();
  auto kinematicView = registry.view<comps::KinematicBodyTag>();
  const auto kinematicCount = kinematicView.size();

  m_scratch->runtimeBodies.reserve(mixedDomainSurfaceCount);
  m_scratch->solverBodies.reserve(mixedDomainSurfaceCount);
  m_scratch->surfaces.reserve(mixedDomainSurfaceCount);
  m_scratch->dynamicsTerms.reserve(mixedDomainSurfaceCount);
  m_scratch->solveDynamicsTerms.reserve(mixedDomainSurfaceCount);
  m_scratch->articulationConstraints.reserve(bodyCount);
  m_scratch->bdf2Histories.reserve(bodyCount);
  m_scratch->mixedDomainSurfaces.reserve(mixedDomainSurfaceCount);
  m_scratch->tracedEntities.reserve(kinematicCount);
  m_scratch->blockedEntities.reserve(kinematicCount);
  m_scratch->writebackEntities.reserve(mixedDomainSurfaceCount);
  m_scratch->orderedEntities.reserve(mixedDomainSurfaceCount);
  m_scratch->visitState.reserve(mixedDomainSurfaceCount);
  m_scratch->contactPowerSum.reserve(mixedDomainSurfaceCount);
  m_scratch->sawNonStationaryContactBody.reserve(mixedDomainSurfaceCount);
  m_scratch->stationaryContactBody.reserve(mixedDomainSurfaceCount);
  m_scratch->solveResult.surfaces.reserve(mixedDomainSurfaceCount);
  m_scratch->solveScratch.laggedSurfaces.reserve(mixedDomainSurfaceCount);
  m_scratch->solveScratch.lineSearchStartSurfaces.reserve(
      mixedDomainSurfaceCount);
  m_scratch->solveScratch.candidateSurfaces.reserve(mixedDomainSurfaceCount);
  m_scratch->solveScratch.acceptedSurfaces.reserve(mixedDomainSurfaceCount);
  m_scratch->solveScratch.bestDecreasingSurfaces.reserve(
      mixedDomainSurfaceCount);

  RigidIpcSolverStats warmupStats;
  std::size_t warmupBodyCount = collectRigidIpcRuntimeBodies(
      world,
      warmupStats,
      m_scratch->runtimeBodies,
      m_scratch->payloadAllocator);
  appendRigidIpcDeformableSurfaces(
      world,
      warmupStats,
      m_scratch->runtimeBodies,
      warmupBodyCount,
      m_scratch->payloadAllocator);
  collectRigidIpcMixedDomainCandidateStats(
      registry,
      m_scratch->runtimeBodies,
      m_options.activationDistance,
      warmupStats,
      m_scratch->mixedDomainSurfaces,
      m_scratch->payloadAllocator);
  prepareRigidIpcSolverScratch(
      m_scratch->runtimeBodies,
      m_scratch->solverBodies,
      m_scratch->surfaces,
      m_scratch->dynamicsTerms,
      m_scratch->payloadAllocator);
  collectRigidIpcArticulationConstraints(
      world, m_scratch->solverBodies, m_scratch->articulationConstraints);
  const auto prewarmSolveSurfaces = [&](auto& solveSurfaces) {
    solveSurfaces.assign(
        m_scratch->surfaces.begin(), m_scratch->surfaces.end());
  };
  prewarmSolveSurfaces(m_scratch->solveResult.surfaces);
  prewarmSolveSurfaces(m_scratch->solveScratch.laggedSurfaces);
  prewarmSolveSurfaces(m_scratch->solveScratch.lineSearchStartSurfaces);
  prewarmSolveSurfaces(m_scratch->solveScratch.candidateSurfaces);
  prewarmSolveSurfaces(m_scratch->solveScratch.acceptedSurfaces);
  prewarmSolveSurfaces(m_scratch->solveScratch.bestDecreasingSurfaces);
  std::size_t dynamicDofCount = 0u;
  for (const auto& surface : m_scratch->surfaces) {
    if (surface.dynamic) {
      dynamicDofCount += 6u;
    }
  }
  m_scratch->solveResult.lastStep.delta.resize(
      static_cast<Eigen::Index>(dynamicDofCount));
  m_scratch->solveScratch.step.delta.resize(
      static_cast<Eigen::Index>(dynamicDofCount));
  auto& assembly = m_scratch->solveResult.assembly;
  assembly.bodyDofOffsets.reserve(m_scratch->surfaces.size());
  assembly.activeArticulationConstraints.reserve(
      m_scratch->articulationConstraints.size());

  // Warm the projected-Newton solve through the same options + solve path
  // execute() uses, so a baked step reuses every buffer the real solve
  // allocates (barrier assembly, line search, reduced factorization) instead of
  // growing the world allocator on the first warmed step. The result is
  // discarded and no body moves -- the solve only fills scratch, and it takes
  // the identical adaptive-kappa configuration the step will, so the warmed
  // active-pair set matches and nothing is reallocated on the first step.
  sxdetail::RigidIpcProjectedNewtonSolveOptions warmOptions
      = buildRigidIpcSolveOptions(
          world,
          m_scratch->solverBodies,
          m_scratch->solveDynamicsTerms,
          m_scratch->dynamicsTerms,
          m_scratch->articulationConstraints,
          m_options,
          m_memoryManager,
          world.getRigidIpcAdaptiveBarrierStiffnessLowerBound());
  sxdetail::solveRigidIpcProjectedNewtonBarrierSystem(
      m_scratch->surfaces,
      warmOptions,
      m_scratch->solveResult,
      m_scratch->solveScratch);
  sxdetail::reserveRigidIpcProjectedNewtonSolveScratchForSameShape(
      m_scratch->surfaces, m_scratch->solveResult, m_scratch->solveScratch);
  m_scratch->solveDynamicsTerms = std::move(warmOptions.dynamicsTerms);
}

//==============================================================================
void RigidIpcContactStage::execute(World& world, ComputeExecutor& /*executor*/)
{
  m_lastStats.reset();
  auto& scratch = *m_scratch;
  clearKinematicBodyStepTraces(world, scratch.tracedEntities);

  std::size_t runtimeBodyCount = collectRigidIpcRuntimeBodies(
      world, m_lastStats, scratch.runtimeBodies, scratch.payloadAllocator);
  appendRigidIpcDeformableSurfaces(
      world,
      m_lastStats,
      scratch.runtimeBodies,
      runtimeBodyCount,
      scratch.payloadAllocator);
  collectRigidIpcMixedDomainCandidateStats(
      dart::simulation::detail::registryOf(world),
      scratch.runtimeBodies,
      m_options.activationDistance,
      m_lastStats,
      scratch.mixedDomainSurfaces,
      scratch.payloadAllocator);
  if (m_options.timeIntegration == RigidIpcTimeIntegration::Bdf2) {
    applyRigidIpcBdf2DynamicsTerms(
        world, scratch.bdf2Histories, scratch.runtimeBodies, m_lastStats);
  } else {
    scratch.bdf2Histories.clear();
  }
  const auto& runtimeBodies = scratch.runtimeBodies;

  if (runtimeBodies.empty()) {
    return;
  }
  if (m_lastStats.dynamicBodyCount == 0u) {
    // Kinematic-only scenes have no solve acceptance gate, so advance
    // prescribed bodies directly.
    scratch.blockedEntities.clear();
    applyRigidIpcKinematicRuntimeBodies(
        world,
        runtimeBodies,
        scratch.blockedEntities,
        scratch.writebackEntities,
        scratch.orderedEntities,
        scratch.visitState);
    return;
  }

  prepareRigidIpcSolverScratch(
      runtimeBodies,
      scratch.solverBodies,
      scratch.surfaces,
      scratch.dynamicsTerms,
      scratch.payloadAllocator);
  auto& solverBodies = scratch.solverBodies;
  auto& surfaces = scratch.surfaces;
  auto& dynamicsTerms = scratch.dynamicsTerms;
  auto& solveDynamicsTerms = scratch.solveDynamicsTerms;
  auto& articulationConstraints = scratch.articulationConstraints;
  collectRigidIpcArticulationConstraints(
      world, solverBodies, articulationConstraints);

  // The conservative line search runs a curved ACCD per candidate primitive
  // pair; resting contacts that settle into compression routinely exhaust the
  // ACCD budget (returning Indeterminate -> a zero step -> LineSearchBlocked),
  // so options.lineSearch.maxIterations is raised inside
  // buildRigidIpcSolveOptions. The apply policy below writes back a
  // not-fully-converged result when it made progress, relying on every accepted
  // Newton step having passed the conservative line-search feasibility check,
  // so options.useLineSearch is pinned there as well.
  sxdetail::RigidIpcProjectedNewtonSolveOptions options
      = buildRigidIpcSolveOptions(
          world,
          solverBodies,
          solveDynamicsTerms,
          dynamicsTerms,
          articulationConstraints,
          m_options,
          m_memoryManager,
          world.getRigidIpcAdaptiveBarrierStiffnessLowerBound());

  sxdetail::RigidIpcProjectedNewtonSolveResult& result = scratch.solveResult;
  // With one supported dynamic surface and no articulation rows, there are no
  // IPC contact pairs. The objective is the diagonal inertial quadratic, so the
  // exact minimizer is the target pose plus force divided by diagonal weight.
  // Taking that path directly avoids constructing solver-private Eigen dynamic
  // matrices for a contact-free step.
  if (solverBodies.size() == 1u && surfaces.size() == 1u
      && options.dynamicsTerms.size() == 1u
      && options.articulationConstraints.empty() && surfaces[0].dynamic
      && options.dynamicsTerms[0].active && options.maxIterations > 0u) {
    const auto& term = options.dynamicsTerms[0];
    sxdetail::RigidIpcVector6d acceptedPosition = poseVector(term.targetPose);
    bool diagonalSolveValid = true;
    for (Eigen::Index dof = 0; dof < acceptedPosition.size(); ++dof) {
      const double weight = term.diagonalWeights[dof];
      const double force = term.generalizedForce[dof];
      if (!std::isfinite(weight) || weight <= 1e-15 || !std::isfinite(force)) {
        diagonalSolveValid = false;
        break;
      }
      acceptedPosition[dof] += force / weight;
    }

    if (diagonalSolveValid && acceptedPosition.allFinite()) {
      const sxdetail::RigidIpcVector6d startPosition
          = poseVector(surfaces[0].pose);
      const double stepNorm = (acceptedPosition - startPosition).norm();

      m_lastStats.status = RigidIpcSolveStatus::Converged;
      m_lastStats.activeDynamicsTerms = 1u;
      m_lastStats.solverIterations = stepNorm > 0.0 ? 1u : 0u;
      m_lastStats.acceptedSteps = stepNorm > 0.0 ? 1u : 0u;
      m_lastStats.sufficientDecreaseChecks = m_lastStats.acceptedSteps;
      m_lastStats.initialGradientNorm
          = (term.diagonalWeights.cwiseProduct(
                 startPosition - poseVector(term.targetPose))
             - term.generalizedForce)
                .norm();
      m_lastStats.finalGradientNorm = 0.0;
      m_lastStats.lastStepNorm = stepNorm;
      m_lastStats.barrierStiffness = options.barrier.stiffness;
      m_lastStats.converged = true;
      m_lastStats.resultApplied = true;
      world.resetRigidIpcAdaptiveBarrierStiffnessLowerBound();

      solveDynamicsTerms = std::move(options.dynamicsTerms);
      applyRigidIpcPoseToRuntimeBody(
          world,
          solverBodies[0],
          poseFromVector(acceptedPosition),
          m_options.timeIntegration == RigidIpcTimeIntegration::Bdf2
              ? std::addressof(scratch.bdf2Histories)
              : nullptr);
      return;
    }
  }

  sxdetail::solveRigidIpcProjectedNewtonBarrierSystem(
      surfaces, options, result, scratch.solveScratch);
  solveDynamicsTerms = std::move(options.dynamicsTerms);

  const bool lineSearchBlocked
      = result.status
        == sxdetail::RigidIpcProjectedNewtonSolveStatus::LineSearchBlocked;
  const bool hasKinematicRuntimeBody = std::any_of(
      solverBodies.begin(), solverBodies.end(), [](const auto& body) {
        return body.kinematic;
      });
  const bool restingContactBlocked = !hasKinematicRuntimeBody
                                     && lineSearchBlocked
                                     && canApplyRestingContactNoOp(
                                         solverBodies,
                                         result,
                                         scratch.contactPowerSum,
                                         scratch.sawNonStationaryContactBody,
                                         scratch.stationaryContactBody);

  m_lastStats.status = toPublicRigidIpcSolveStatus(result.status);
  m_lastStats.activeConstraints = result.assembly.activeConstraints.size();
  m_lastStats.activeFrictionConstraints
      = result.assembly.activeFrictionConstraints.size();
  m_lastStats.activeArticulationConstraints
      = result.assembly.activeArticulationConstraints.size();
  m_lastStats.activeDynamicsTerms = result.assembly.activeDynamicsTerms;
  m_lastStats.solverIterations = result.stats.iterations;
  m_lastStats.frictionIterations = result.stats.frictionIterations;
  m_lastStats.acceptedSteps = result.stats.acceptedSteps;
  m_lastStats.lineSearchLimitedSteps = result.stats.lineSearchLimitedSteps;
  m_lastStats.initialValue = result.stats.initialValue;
  m_lastStats.finalValue = result.stats.finalValue;
  m_lastStats.initialGradientNorm = result.stats.initialGradientNorm;
  m_lastStats.finalGradientNorm = result.stats.finalGradientNorm;
  m_lastStats.initialEqualityResidualNorm
      = result.stats.initialEqualityResidualNorm;
  m_lastStats.finalEqualityResidualNorm
      = result.stats.finalEqualityResidualNorm;
  m_lastStats.finalMomentumBalance = result.stats.finalMomentumBalance;
  m_lastStats.lastStepNorm = result.stats.lastStepNorm;
  m_lastStats.barrierStiffness = result.stats.barrierStiffness;
  m_lastStats.barrierStiffnessIncreases
      = result.stats.barrierStiffnessIncreases;
  m_lastStats.lastLineSearchStepBound = result.lineSearch.stepBound;
  m_lastStats.lastLineSearchIndeterminate = result.lineSearch.indeterminate;
  m_lastStats.lineSearchPointPointChecks
      = result.stats.lineSearchPointPointChecks;
  m_lastStats.lineSearchPointEdgeChecks
      = result.stats.lineSearchPointEdgeChecks;
  m_lastStats.lineSearchEdgeEdgeChecks = result.stats.lineSearchEdgeEdgeChecks;
  m_lastStats.lineSearchPointTriangleChecks
      = result.stats.lineSearchPointTriangleChecks;
  m_lastStats.lineSearchHits = result.stats.lineSearchHits;
  m_lastStats.lineSearchMisses = result.stats.lineSearchMisses;
  m_lastStats.lineSearchIndeterminateCount
      = result.stats.lineSearchIndeterminateCount;
  m_lastStats.lineSearchZeroStepCount = result.stats.lineSearchZeroStepCount;
  m_lastStats.sufficientDecreaseChecks = result.stats.sufficientDecreaseChecks;
  m_lastStats.sufficientDecreaseBacktracks
      = result.stats.sufficientDecreaseBacktracks;
  m_lastStats.converged = result.converged;
  m_lastStats.failed = result.failed && !restingContactBlocked;

  if (result.assembly.activeConstraints.empty()) {
    world.resetRigidIpcAdaptiveBarrierStiffnessLowerBound();
  } else if (
      std::isfinite(result.stats.barrierStiffness)
      && result.stats.barrierStiffness > 0.0) {
    world.setRigidIpcAdaptiveBarrierStiffnessLowerBound(
        result.stats.barrierStiffness);
  }

  // A failed solve (conservative line search blocked the step, or the
  // factorization failed) normally never writes back: applying it could
  // penetrate. The exception is an exact resting-contact plateau where the
  // line search blocked before any accepted step and the active dynamic contact
  // bodies were either stationary or constrained by contact-gradient work.
  // Writing back the unchanged current pose is safe in that narrow case (it
  // introduces no new motion) and avoids reporting a persistent failure for a
  // static dense contact. Pure tangential or friction-only motion must be
  // preserved by leaving the runtime state untouched.
  if (result.failed && !restingContactBlocked) {
    applyRigidIpcKinematicRuntimeBodiesAfterRejectedSolve(
        world,
        runtimeBodies,
        solverBodies,
        result,
        scratch.blockedEntities,
        scratch.writebackEntities,
        scratch.orderedEntities,
        scratch.visitState);
    return;
  }
  // Otherwise apply the last intersection-free iterate the bounded solve
  // reached. Every accepted Newton step is a descent step bounded by the
  // conservative line-search feasibility check, so even a not-fully-converged
  // result (e.g. a stiff friction-limited contact that plateaus before the
  // gradient tolerance) is a valid, penetration-free forward step that the next
  // substep re-solves from -- like the reference IPC, which advances with the
  // optimizer's feasible iterate rather than discarding a non-converged step.
  // The internal solve applies sufficient-decrease backtracking when the
  // assembled objective admits it, and otherwise stops at the last feasible
  // decreasing iterate. A solve that made no progress at all (e.g. a zero
  // iteration budget) is still skipped.
  if (!result.converged && !result.madeProgress() && !restingContactBlocked) {
    m_lastStats.nonConvergedResultSkipped = true;
    applyRigidIpcKinematicRuntimeBodiesAfterRejectedSolve(
        world,
        runtimeBodies,
        solverBodies,
        result,
        scratch.blockedEntities,
        scratch.writebackEntities,
        scratch.orderedEntities,
        scratch.visitState);
    return;
  }

  m_lastStats.resultApplied = true;
  applyRigidIpcRuntimeResult(
      world,
      runtimeBodies,
      result,
      m_options.timeIntegration == RigidIpcTimeIntegration::Bdf2
          ? std::addressof(scratch.bdf2Histories)
          : nullptr,
      scratch.writebackEntities,
      scratch.orderedEntities,
      scratch.visitState);
}

//==============================================================================
std::size_t RigidIpcContactStage::getMaxIterations() const noexcept
{
  return m_options.maxIterations;
}

//==============================================================================
double RigidIpcContactStage::getActivationDistance() const noexcept
{
  return m_options.activationDistance;
}

//==============================================================================
std::size_t RigidIpcContactStage::getFrictionIterations() const noexcept
{
  return m_options.frictionIterations;
}

//==============================================================================
double RigidIpcContactStage::getStaticFrictionSpeedBound() const noexcept
{
  return m_options.staticFrictionSpeedBound;
}

//==============================================================================
double RigidIpcContactStage::getFrictionConvergenceTolerance() const noexcept
{
  return m_options.frictionConvergenceTolerance;
}

//==============================================================================
RigidIpcContactStageOptions RigidIpcContactStage::getOptions() const noexcept
{
  return m_options;
}

//==============================================================================
const RigidIpcSolverStats& RigidIpcContactStage::getLastStats() const noexcept
{
  return m_lastStats;
}

} // namespace dart::simulation::compute
