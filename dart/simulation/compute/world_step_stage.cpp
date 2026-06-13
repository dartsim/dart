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

#include "dart/simulation/compute/world_step_stage.hpp"

#include "dart/simulation/body/rigid_body.hpp"
#include "dart/simulation/common/exceptions.hpp"
#include "dart/simulation/comps/collision_geometry.hpp"
#include "dart/simulation/comps/contact_material.hpp"
#include "dart/simulation/comps/deformable_body.hpp"
#include "dart/simulation/comps/dynamics.hpp"
#include "dart/simulation/comps/frame_types.hpp"
#include "dart/simulation/comps/joint.hpp"
#include "dart/simulation/comps/rigid_body.hpp"
#include "dart/simulation/compute/compute_executor.hpp"
#include "dart/simulation/compute/compute_graph.hpp"
#include "dart/simulation/compute/detail/deformable_avbd_replay_state.hpp"
#include "dart/simulation/compute/rigid_body_constraint.hpp"
#include "dart/simulation/compute/rigid_body_state_batch.hpp"
#include "dart/simulation/compute/world_kinematics_graph.hpp"
#include "dart/simulation/detail/boxed_lcp_contact.hpp"
#include "dart/simulation/detail/deformable_contact/barrier_kernel.hpp"
#include "dart/simulation/detail/deformable_contact/candidate_set.hpp"
#include "dart/simulation/detail/deformable_contact/continuous_collision_step.hpp"
#include "dart/simulation/detail/deformable_contact/tangent_stencil.hpp"
#include "dart/simulation/detail/deformable_elasticity/fem_tet_element.hpp"
#include "dart/simulation/detail/deformable_vbd/attachment_kernel.hpp"
#include "dart/simulation/detail/deformable_vbd/avbd_row_inventory.hpp"
#include "dart/simulation/detail/deformable_vbd/block_descent.hpp"
#include "dart/simulation/detail/deformable_vbd/parallel_block_descent.hpp"
#include "dart/simulation/detail/deformable_vbd/rigid_world_contact.hpp"
#include "dart/simulation/detail/entity_conversion.hpp"
#include "dart/simulation/detail/newton_barrier/friction_kernel.hpp"
#include "dart/simulation/detail/newton_barrier/mixed_domain_coupling.hpp"
#include "dart/simulation/detail/newton_barrier/projected_newton.hpp"
#include "dart/simulation/detail/newton_barrier/psd_backend.hpp"
#include "dart/simulation/detail/newton_barrier/restitution_damping.hpp"
#include "dart/simulation/detail/rigid_ipc_barrier.hpp"
#include "dart/simulation/detail/world_registry_access.hpp"
#include "dart/simulation/world.hpp"
#include "dart/simulation/world_options.hpp"

#include <dart/config.hpp>

#include <dart/math/lcp/lcp_types.hpp>
#include <dart/math/lcp/pivoting/dantzig_solver.hpp>

#include <Eigen/Cholesky>
#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>
#include <Eigen/IterativeLinearSolvers>
#include <Eigen/SparseCholesky>
#include <Eigen/SparseCore>
#include <entt/entt.hpp>

#include <algorithm>
#include <array>
#include <limits>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <tuple>
#include <vector>

#include <cmath>
#include <cstdint>

#if DART_BUILD_PROFILE
  #include <chrono>
#endif

namespace dart::simulation::compute {

namespace dc = dart::simulation::detail::deformable_contact;
namespace dvbd = dart::simulation::detail::deformable_vbd;
namespace fem = dart::simulation::detail::deformable_elasticity;
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
  std::vector<RigidIpcRuntimeBody> runtimeBodies;
  std::vector<RigidIpcRuntimeBody> solverBodies;
  std::vector<sxdetail::RigidIpcBarrierSurface> surfaces;
  std::vector<sxdetail::RigidIpcBodyDynamicsTerm> dynamicsTerms;
  std::vector<sxdetail::RigidIpcBodyDynamicsTerm> solveDynamicsTerms;
  std::vector<sxdetail::RigidIpcArticulationConstraintInput>
      articulationConstraints;
  std::vector<RigidIpcBdf2HistoryEntry> bdf2Histories;
  std::vector<nb::MixedDomainSurface> mixedDomainSurfaces;
  std::vector<entt::entity> tracedEntities;
  std::vector<entt::entity> blockedEntities;
  std::vector<entt::entity> writebackEntities;
  std::vector<entt::entity> orderedEntities;
  std::vector<int> visitState;
  std::vector<double> contactPowerSum;
  std::vector<std::uint8_t> sawNonStationaryContactBody;
  std::vector<std::uint8_t> stationaryContactBody;
  sxdetail::RigidIpcProjectedNewtonSolveResult solveResult;
  sxdetail::RigidIpcProjectedNewtonSolveScratch solveScratch;
};

struct RigidBodyNode
{
  entt::entity entity;
  ComputeNode* node;
};

struct RigidBodyIntegrationStage::Scratch
{
  std::vector<entt::entity> entities;
  std::vector<RigidBodyNode> nodes;
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
std::optional<comps::Transform> collisionShapeWorldTransform(
    const comps::Transform& bodyTransform, const CollisionShape& shape)
{
  if (!bodyTransform.position.allFinite()
      || !bodyTransform.orientation.coeffs().allFinite()
      || !shape.localTransform.matrix().allFinite()) {
    return std::nullopt;
  }

  const Eigen::Quaterniond bodyOrientation
      = normalizeOrIdentity(bodyTransform.orientation);
  const Eigen::Quaterniond shapeOrientation(shape.localTransform.linear());

  comps::Transform result;
  result.position = bodyTransform.position
                    + bodyOrientation.toRotationMatrix()
                          * shape.localTransform.translation();
  result.orientation = bodyOrientation * shapeOrientation.normalized();
  result.orientation.normalize();
  return result;
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
bool containsEntity(
    const std::vector<entt::entity>& entities, entt::entity entity)
{
  return std::find(entities.begin(), entities.end(), entity) != entities.end();
}

//==============================================================================
std::size_t findEntityIndex(
    const std::vector<entt::entity>& entities, entt::entity entity)
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
    const std::vector<entt::entity>& rigidBodyEntities)
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
    const detail::WorldRegistry& registry,
    const std::vector<entt::entity>& rigidBodyEntities)
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
    const std::vector<entt::entity>& rigidBodyEntities,
    std::vector<int>& visitState,
    std::vector<entt::entity>& ordered,
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
    const std::vector<entt::entity>& rigidBodyEntities,
    std::vector<entt::entity>& ordered,
    std::vector<int>& visitState)
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
ComputeNode* findRigidBodyNode(
    const std::vector<RigidBodyNode>& nodes, entt::entity entity)
{
  const auto it = std::find_if(
      nodes.begin(), nodes.end(), [entity](const RigidBodyNode& entry) {
        return entry.entity == entity;
      });
  if (it == nodes.end()) {
    return nullptr;
  }

  return it->node;
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

//==============================================================================
const comps::RigidAvbdContactConfig* enabledRigidAvbdContactConfig(
    const detail::WorldRegistry& registry, entt::entity entity)
{
  const auto* config = registry.try_get<comps::RigidAvbdContactConfig>(entity);
  if (config == nullptr || !config->enabled) {
    return nullptr;
  }
  return config;
}

//==============================================================================
bool mayHaveRigidAvbdContactConfigs(const detail::WorldRegistry& registry)
{
  const auto* configStorage = registry.storage<comps::RigidAvbdContactConfig>();
  return configStorage != nullptr && configStorage->size() != 0u;
}

//==============================================================================
std::optional<comps::RigidAvbdContactConfig> rigidAvbdContactStageConfig(
    const detail::WorldRegistry& registry, std::span<const Contact> contacts)
{
  std::optional<comps::RigidAvbdContactConfig> merged;
  for (const Contact& contact : contacts) {
    const auto* configA = enabledRigidAvbdContactConfig(
        registry, detail::toRegistryEntity(contact.bodyA.getEntity()));
    const auto* configB = enabledRigidAvbdContactConfig(
        registry, detail::toRegistryEntity(contact.bodyB.getEntity()));
    if (configA == nullptr && configB == nullptr) {
      return std::nullopt;
    }

    const auto absorb = [&](const comps::RigidAvbdContactConfig& config) {
      if (!merged.has_value()) {
        merged = config;
        return;
      }

      merged->startStiffness
          = std::max(merged->startStiffness, config.startStiffness);
      merged->alpha = std::max(merged->alpha, config.alpha);
      merged->beta = std::max(merged->beta, config.beta);
      merged->gamma = std::max(merged->gamma, config.gamma);
      merged->maxStiffness
          = std::min(merged->maxStiffness, config.maxStiffness);
    };

    if (configA != nullptr) {
      absorb(*configA);
    }
    if (configB != nullptr) {
      absorb(*configB);
    }
  }

  return merged;
}

} // namespace

//==============================================================================
ComputeStageMetadata WorldStepStage::getMetadata() const noexcept
{
  return {};
}

//==============================================================================
WorldStepPipeline& WorldStepPipeline::addStage(WorldStepStage& stage)
{
  if (m_stageCount >= m_stages.size()) {
    if (m_overflowStages.empty()) {
      m_overflowStages.reserve(m_stages.size());
    }

    m_overflowStages.push_back(&stage);
    ++m_stageCount;
    return *this;
  }

  m_stages[m_stageCount] = &stage;
  ++m_stageCount;
  return *this;
}

//==============================================================================
void WorldStepPipeline::clear() noexcept
{
  m_stageCount = 0;
  m_overflowStages.clear();
}

//==============================================================================
std::size_t WorldStepPipeline::getStageCount() const noexcept
{
  return m_stageCount;
}

//==============================================================================
bool WorldStepPipeline::isEmpty() const noexcept
{
  return m_stageCount == 0;
}

//==============================================================================
WorldStepStage& WorldStepPipeline::getStage(std::size_t index) const
{
  DART_SIMULATION_THROW_T_IF(
      index >= m_stageCount,
      OutOfRangeException,
      "World step pipeline stage index {} is out of range",
      index);

  if (index < m_stages.size()) {
    return *m_stages[index];
  }

  return *m_overflowStages[index - m_stages.size()];
}

//==============================================================================
void WorldStepPipeline::execute(World& world, ComputeExecutor& executor)
{
  for (std::size_t i = 0; i < m_stageCount; ++i) {
    getStage(i).execute(world, executor);
  }
}

#if DART_BUILD_PROFILE
namespace {

/// Executor adapter used only by World step profiling. It preserves the stage's
/// normal executor injection while capturing any nested compute graph profiles
/// the stage runs.
class WorldStepProfilingExecutor final : public ComputeExecutor
{
public:
  WorldStepProfilingExecutor(
      ComputeExecutor& delegate, WorldStepStageProfile& stageProfile)
    : m_delegate(delegate), m_stageProfile(stageProfile)
  {
  }

  void execute(const ComputeGraph& graph) override
  {
    m_stageProfile.graphProfiles.push_back(m_delegate.executeProfiled(graph));
  }

  [[nodiscard]] ComputeExecutionProfile executeProfiled(
      const ComputeGraph& graph) override
  {
    auto profile = m_delegate.executeProfiled(graph);
    m_stageProfile.graphProfiles.push_back(profile);
    return profile;
  }

  [[nodiscard]] std::size_t getWorkerCount() const override
  {
    return m_delegate.getWorkerCount();
  }

private:
  ComputeExecutor& m_delegate;
  WorldStepStageProfile& m_stageProfile;
};

} // namespace
#endif

//==============================================================================
WorldStepProfile WorldStepPipeline::executeProfiled(
    World& world, ComputeExecutor& executor)
{
  WorldStepProfile profile;

#if DART_BUILD_PROFILE
  profile.stepCount = 1;
  profile.stages.reserve(m_stageCount);

  const auto stepStart = std::chrono::steady_clock::now();
  for (std::size_t i = 0; i < m_stageCount; ++i) {
    auto& stage = getStage(i);
    const auto metadata = stage.getMetadata();

    auto& entry = profile.stages.emplace_back();
    entry.name = stage.getName();
    entry.domain = metadata.domain;
    entry.acceleration = metadata.acceleration;

    const bool stageCanUseGpuBackend
        = hasAcceleration(metadata.acceleration, ComputeStageAcceleration::Gpu);
    const bool acceleratedBackendBefore
        = stageCanUseGpuBackend && isDeformablePsdAccelerated();

    WorldStepProfilingExecutor profilingExecutor(executor, entry);
    const auto stageStart = std::chrono::steady_clock::now();
    stage.execute(world, profilingExecutor);
    const auto stageEnd = std::chrono::steady_clock::now();

    entry.duration = stageEnd - stageStart;
    entry.acceleratedBackendEnabled
        = acceleratedBackendBefore
          || (stageCanUseGpuBackend && isDeformablePsdAccelerated());
  }
  profile.wallTime = std::chrono::steady_clock::now() - stepStart;
#else
  execute(world, executor);
#endif

  return profile;
}

//==============================================================================
std::string_view KinematicsStage::getName() const noexcept
{
  return "kinematics";
}

//==============================================================================
KinematicsStage::KinematicsStage() = default;

//==============================================================================
KinematicsStage::~KinematicsStage() = default;

//==============================================================================
ComputeStageMetadata KinematicsStage::getMetadata() const noexcept
{
  return {
      ComputeStageDomain::Kinematics,
      ComputeStageAcceleration::TaskParallel
          | ComputeStageAcceleration::DataLocality};
}

//==============================================================================
void KinematicsStage::prepare(World& world)
{
  if (m_cachedWorld != &world || !m_cachedGraph) {
    m_cachedGraph = std::make_unique<WorldKinematicsGraph>(world);
    m_cachedWorld = &world;
    return;
  }

  if (!m_cachedGraph->isTopologyCurrent()) {
    m_cachedGraph->rebuild();
  }
}

//==============================================================================
void KinematicsStage::execute(World& world, ComputeExecutor& executor)
{
  if (!m_cachedGraph || m_cachedWorld != &world || !world.isSimulationMode()
      || !m_cachedGraph->isTopologyCurrent()) {
    prepare(world);
  }
  m_cachedGraph->execute(executor);
}

//==============================================================================
RigidBodyIntegrationStage::RigidBodyIntegrationStage(std::size_t batchSize)
  : m_batchSize(std::max<std::size_t>(1, batchSize)),
    m_scratch(std::make_unique<Scratch>())
{
}

//==============================================================================
RigidBodyIntegrationStage::~RigidBodyIntegrationStage() = default;

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
    m_scratch = std::make_unique<Scratch>();
  }
  auto& entities = m_scratch->entities;
  entities.clear();
  entities.reserve(rigidBodyView.size_hint());
  for (auto entity : rigidBodyView) {
    entities.push_back(entity);
  }

  ComputeGraph graph;
  const auto gravity = world.getGravity();
  const auto timeStep = world.getTimeStep();
  if (hasRigidBodyFrameDependency(registry, entities)) {
    auto& nodes = m_scratch->nodes;
    nodes.clear();
    nodes.reserve(entities.size());

    for (const auto entity : entities) {
      auto& node = graph.addNode(
          "rigid_body_entity_" + std::to_string(entt::to_integral(entity)),
          [&registry, entity, gravity, timeStep]() {
            integrateRigidBody(registry, entity, gravity, timeStep);
          },
          getMetadata());
      nodes.push_back({entity, &node});
    }

    for (const auto& entry : nodes) {
      const auto& frameState = registry.get<comps::FrameState>(entry.entity);
      const auto parentRigidBody = findNearestRigidBodyAncestor(
          registry, frameState.parentFrame, entities);
      if (parentRigidBody == entt::null) {
        continue;
      }

      auto* parentNode = findRigidBodyNode(nodes, parentRigidBody);
      DART_SIMULATION_THROW_T_IF(
          !parentNode,
          InvalidOperationException,
          "Rigid-body ancestor is missing an integration node");

      graph.addDependency(*parentNode, *entry.node);
    }

    executor.execute(graph);
    return;
  }

  const auto* entityList = &entities;
  for (std::size_t begin = 0; begin < entities.size(); begin += m_batchSize) {
    const auto end = std::min(begin + m_batchSize, entities.size());
    graph.addNode(
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
  }

  executor.execute(graph);
}

//==============================================================================
std::size_t RigidBodyIntegrationStage::getBatchSize() const noexcept
{
  return m_batchSize;
}

//==============================================================================
/// Transient cached topology for the experimental Vertex Block Descent inner
/// solver: the spring and tetrahedron element lists plus the vertex-graph
/// coloring (over the union of springs and tets) and the per-element incident
/// adjacencies, rebuilt only when the element topology changes. Defined here
/// (not in comps) so the heavy kernel headers stay out of the component
/// headers; it is stored per body and never serialized.
struct DeformableVbdScratch
{
  template <typename Row>
  struct AvbdFrictionWarmStartRecord
  {
    dvbd::AvbdScalarRowKey key;
    Row row;
  };

  std::vector<dvbd::SpringElement> springs;
  std::vector<dvbd::TetMeshElement> tets;
  dvbd::VertexColoring coloring;
  dvbd::SpringAdjacency springAdjacency;
  dvbd::TetAdjacency tetAdjacency;
  // Per-vertex static ground-contact planes, rebuilt each step from the barrier
  // set at the warm-start position (lagged); a zero stiffness marks "no ground
  // under this vertex".
  std::vector<dvbd::ContactPlane> contactPlanes;
  // Stable static-contact feature IDs parallel to `contactPlanes`, used by the
  // AVBD row inventory so a vertex contact against ground, sphere, or box
  // features warm-starts only against the same static feature.
  std::vector<std::uint64_t> contactObjectIds;
  std::vector<std::uint64_t> contactFeatureIds;
  std::vector<dvbd::AvbdScalarRowDescriptor> avbdContactDescriptors;
  dvbd::AvbdScalarRowInventory avbdContactInventory;
  std::vector<dvbd::AvbdHalfSpaceContactRow> avbdContactRows;
  std::vector<dvbd::AvbdScalarRowDescriptor> avbdFrictionDescriptors;
  dvbd::AvbdScalarRowInventory avbdFrictionInventory;
  std::vector<dvbd::AvbdHalfSpaceFrictionRow> avbdFrictionRows;
  std::vector<AvbdFrictionWarmStartRecord<dvbd::AvbdHalfSpaceFrictionRow>>
      previousAvbdFrictionWarmStarts;
  std::vector<dvbd::AvbdScalarRowDescriptor> avbdSelfContactDescriptors;
  dvbd::AvbdScalarRowInventory avbdSelfContactInventory;
  std::vector<dvbd::AvbdSelfContactNormalRow> avbdSelfContactRows;
  std::vector<dvbd::AvbdScalarRowDescriptor> avbdSelfContactFrictionDescriptors;
  dvbd::AvbdScalarRowInventory avbdSelfContactFrictionInventory;
  std::vector<dvbd::AvbdSelfContactFrictionRow> avbdSelfContactFrictionRows;
  std::vector<AvbdFrictionWarmStartRecord<dvbd::AvbdSelfContactFrictionRow>>
      previousAvbdSelfContactFrictionWarmStarts;
  std::vector<dvbd::AvbdScalarRowDescriptor> avbdAttachmentDescriptors;
  dvbd::AvbdScalarRowInventory avbdAttachmentInventory;
  std::vector<dvbd::AvbdPointAttachmentRow> avbdAttachmentRows;
  std::vector<dvbd::AvbdScalarRowDescriptor> avbdSpringDescriptors;
  dvbd::AvbdScalarRowInventory avbdSpringInventory;
  std::vector<dvbd::AvbdSpringFiniteStiffnessRow> avbdSpringRows;
  std::vector<dvbd::AvbdScalarRowDescriptor> avbdTetDescriptors;
  dvbd::AvbdScalarRowInventory avbdTetInventory;
  std::vector<dvbd::AvbdTetMaterialFiniteStiffnessRow> avbdTetRows;
  std::vector<std::uint8_t> avbdSolveFixed;
  // Self-contact candidate set + per-vertex incident lists, rebuilt each step
  // (lagged) from the body's swept start-to-warm-start surface motion.
  dc::ContactCandidateSet selfContactCandidates;
  dc::detail::ContactCandidateSweepScratch selfContactSweepScratch;
  dvbd::SelfContactAdjacency selfContactAdjacency;
  std::size_t cachedNodeCount = 0;
  std::size_t cachedEdgeCount = 0;
  std::size_t cachedTetCount = 0;
  bool initialized = false;
};

//==============================================================================
namespace {

void clearDeformableAvbdWarmStartRows(DeformableVbdScratch& scratch)
{
  scratch.avbdContactDescriptors.clear();
  scratch.avbdContactInventory.records().clear();
  scratch.avbdContactRows.clear();
  scratch.avbdFrictionDescriptors.clear();
  scratch.avbdFrictionInventory.records().clear();
  scratch.avbdFrictionRows.clear();
  scratch.previousAvbdFrictionWarmStarts.clear();
  scratch.avbdSelfContactDescriptors.clear();
  scratch.avbdSelfContactInventory.records().clear();
  scratch.avbdSelfContactRows.clear();
  scratch.avbdSelfContactFrictionDescriptors.clear();
  scratch.avbdSelfContactFrictionInventory.records().clear();
  scratch.avbdSelfContactFrictionRows.clear();
  scratch.previousAvbdSelfContactFrictionWarmStarts.clear();
  scratch.avbdAttachmentDescriptors.clear();
  scratch.avbdAttachmentInventory.records().clear();
  scratch.avbdAttachmentRows.clear();
  scratch.avbdSpringDescriptors.clear();
  scratch.avbdSpringInventory.records().clear();
  scratch.avbdSpringRows.clear();
  scratch.avbdTetDescriptors.clear();
  scratch.avbdTetInventory.records().clear();
  scratch.avbdTetRows.clear();
  scratch.avbdSolveFixed.clear();
}

//==============================================================================
template <typename Row>
void rebuildAvbdFrictionWarmStartLookup(
    std::vector<DeformableVbdScratch::AvbdFrictionWarmStartRecord<Row>>& lookup,
    const std::vector<dvbd::AvbdScalarRowDescriptor>& descriptors,
    const std::vector<Row>& rows)
{
  lookup.clear();
  const std::size_t rowCount = std::min(descriptors.size(), rows.size());
  lookup.reserve(rowCount);
  for (std::size_t i = 0; i < rowCount; ++i) {
    lookup.push_back({descriptors[i].key, rows[i]});
  }
  std::sort(lookup.begin(), lookup.end(), [](const auto& lhs, const auto& rhs) {
    return lhs.key < rhs.key;
  });
}

//==============================================================================
template <typename Row>
[[nodiscard]] const Row* findAvbdFrictionWarmStartRow(
    const std::vector<DeformableVbdScratch::AvbdFrictionWarmStartRecord<Row>>&
        lookup,
    const dvbd::AvbdScalarRowKey& key)
{
  const auto match = std::lower_bound(
      lookup.begin(),
      lookup.end(),
      key,
      [](const auto& record, const dvbd::AvbdScalarRowKey& value) {
        return record.key < value;
      });
  if (match != lookup.end() && match->key == key) {
    return &match->row;
  }
  return nullptr;
}

} // namespace

//==============================================================================
std::vector<avbd_replay::DeformableAvbdWarmStartReplayState>
avbd_replay::captureDeformableAvbdWarmStartReplayState(
    const detail::WorldRegistry& registry)
{
  std::vector<avbd_replay::DeformableAvbdWarmStartReplayState> states;

  auto view = registry.view<DeformableVbdScratch>();
  for (auto entity : view) {
    const auto& scratch = view.get<DeformableVbdScratch>(entity);
    states.push_back(
        avbd_replay::DeformableAvbdWarmStartReplayState{
            entity,
            scratch.avbdContactInventory.records(),
            scratch.avbdFrictionInventory.records(),
            scratch.avbdSelfContactInventory.records(),
            scratch.avbdSelfContactFrictionInventory.records(),
            scratch.avbdAttachmentInventory.records(),
            scratch.avbdSpringInventory.records(),
            scratch.avbdTetInventory.records()});
  }

  std::ranges::sort(states, [](const auto& lhs, const auto& rhs) {
    return static_cast<std::uint32_t>(lhs.entity)
           < static_cast<std::uint32_t>(rhs.entity);
  });
  return states;
}

//==============================================================================
void avbd_replay::restoreDeformableAvbdWarmStartReplayState(
    detail::WorldRegistry& registry,
    std::span<const avbd_replay::DeformableAvbdWarmStartReplayState>
        replayStates)
{
  for (const auto& state : replayStates) {
    DART_SIMULATION_THROW_T_IF(
        !registry.valid(state.entity)
            || !registry.all_of<comps::DeformableBodyTag>(state.entity),
        InvalidOperationException,
        "Cannot restore replay frame: deformable AVBD entity layout changed");
  }

  for (auto entity : registry.view<DeformableVbdScratch>()) {
    clearDeformableAvbdWarmStartRows(
        registry.get<DeformableVbdScratch>(entity));
  }

  for (const auto& state : replayStates) {
    auto& scratch = registry.get_or_emplace<DeformableVbdScratch>(state.entity);
    clearDeformableAvbdWarmStartRows(scratch);
    scratch.avbdContactInventory.records() = state.contactRows;
    scratch.avbdFrictionInventory.records() = state.frictionRows;
    scratch.avbdSelfContactInventory.records() = state.selfContactRows;
    scratch.avbdSelfContactFrictionInventory.records()
        = state.selfContactFrictionRows;
    scratch.avbdAttachmentInventory.records() = state.attachmentRows;
    scratch.avbdSpringInventory.records() = state.springRows;
    scratch.avbdTetInventory.records() = state.tetRows;
  }
}

namespace {

//==============================================================================
struct RigidBodyForceBatch
{
  void clearAndReserve(std::size_t bodyCount)
  {
    entities.clear();
    force.clear();
    torque.clear();
    entities.reserve(bodyCount);
    force.reserve(3 * bodyCount);
    torque.reserve(3 * bodyCount);
  }

  std::vector<entt::entity> entities;
  std::vector<double> force;
  std::vector<double> torque;
};

enum class RigidBodyForceAssemblyMode
{
  AllBodies,
  AdvanceableOnly,
};

//==============================================================================
void assembleRigidBodyForces(
    const World& world,
    bool includeGravity,
    RigidBodyForceBatch& batch,
    RigidBodyForceAssemblyMode mode = RigidBodyForceAssemblyMode::AllBodies)
{
  const auto& registry = dart::simulation::detail::registryOf(world);
  auto view
      = registry.view<comps::RigidBodyTag, comps::Transform, comps::Velocity>();
  batch.clearAndReserve(view.size_hint());

  const Eigen::Vector3d gravity
      = includeGravity ? world.getGravity() : Eigen::Vector3d::Zero();
  for (const auto entity : view) {
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
void extractRigidBodyStateInto(const World& world, RigidBodyStateBatch& batch)
{
  const auto& registry = dart::simulation::detail::registryOf(world);
  auto view
      = registry.view<comps::RigidBodyTag, comps::Transform, comps::Velocity>();

  batch.worldCount = 1;
  batch.bodyCount = 0;
  batch.position.clear();
  batch.orientation.clear();
  batch.linearVelocity.clear();
  batch.angularVelocity.clear();

  const auto bodyCount = view.size_hint();
  batch.position.reserve(3 * bodyCount);
  batch.orientation.reserve(4 * bodyCount);
  batch.linearVelocity.reserve(3 * bodyCount);
  batch.angularVelocity.reserve(3 * bodyCount);

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
}

//==============================================================================
void extractRigidBodyModelBatchInto(
    const World& world, RigidBodyModelBatch& model)
{
  const auto& registry = dart::simulation::detail::registryOf(world);
  auto view
      = registry.view<comps::RigidBodyTag, comps::Transform, comps::Velocity>();

  model.worldCount = 1;
  model.bodyCount = 0;
  model.inverseMass.clear();
  model.inertia.clear();

  const auto bodyCount = view.size_hint();
  model.inverseMass.reserve(bodyCount);
  model.inertia.reserve(9 * bodyCount);

  for (const auto entity : view) {
    ++model.bodyCount;

    const auto& mass = registry.get<comps::MassProperties>(entity);
    const double inverse
        = (mass.mass > 0.0 && std::isfinite(mass.mass)) ? 1.0 / mass.mass : 0.0;
    model.inverseMass.push_back(inverse);

    for (int row = 0; row < 3; ++row) {
      for (int col = 0; col < 3; ++col) {
        model.inertia.push_back(mass.inertia(row, col));
      }
    }
  }
}

//==============================================================================
void copyRigidBodyStateBatch(
    const RigidBodyStateBatch& source, RigidBodyStateBatch& target)
{
  target.worldCount = source.worldCount;
  target.bodyCount = source.bodyCount;
  target.position = source.position;
  target.orientation = source.orientation;
  target.linearVelocity = source.linearVelocity;
  target.angularVelocity = source.angularVelocity;
}

//==============================================================================
void restorePrescribedRigidBodyState(
    const detail::WorldRegistry& registry,
    const std::vector<entt::entity>& entities,
    const RigidBodyStateBatch& source,
    RigidBodyStateBatch& target)
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
double inverseMass(const comps::MassProperties& mass)
{
  return (mass.mass > 0.0 && std::isfinite(mass.mass)) ? 1.0 / mass.mass : 0.0;
}

//==============================================================================
Eigen::Matrix3d inverseWorldInertia(
    const comps::MassProperties& mass, const comps::Transform& transform)
{
  if (!(mass.mass > 0.0) || !std::isfinite(mass.mass)) {
    return Eigen::Matrix3d::Zero();
  }

  const Eigen::Matrix3d rotation
      = normalizeOrIdentity(transform.orientation).toRotationMatrix();
  const Eigen::Matrix3d worldInertia
      = rotation * mass.inertia * rotation.transpose();
  Eigen::LDLT<Eigen::Matrix3d> solver(worldInertia);
  if (solver.info() != Eigen::Success || !solver.isPositive()) {
    return Eigen::Matrix3d::Zero();
  }
  return solver.solve(Eigen::Matrix3d::Identity());
}

//==============================================================================
bool needsContactInverseInertia(
    const Eigen::Vector3d& normalArmCross, const double friction)
{
  return friction > 0.0 || normalArmCross.x() != 0.0
         || normalArmCross.y() != 0.0 || normalArmCross.z() != 0.0;
}

//==============================================================================
double restitutionOf(const detail::WorldRegistry& registry, entt::entity entity)
{
  if (const auto* material = registry.try_get<comps::ContactMaterial>(entity)) {
    return material->restitution;
  }
  return 0.0;
}

//==============================================================================
double frictionOf(const detail::WorldRegistry& registry, entt::entity entity)
{
  if (const auto* material = registry.try_get<comps::ContactMaterial>(entity)) {
    return material->friction;
  }
  return 1.0;
}

//==============================================================================
bool hasPrescribedRigidBodyContactResponse(
    const detail::WorldRegistry& registry, entt::entity entity)
{
  return registry.all_of<comps::StaticBodyTag>(entity)
         || registry.all_of<comps::KinematicBodyTag>(entity);
}

//==============================================================================
bool shouldSkipRigidBodyContactQuery(const detail::WorldRegistry& registry)
{
  const auto collisionGeometryView = registry.view<comps::CollisionGeometry>();
  for (const entt::entity entity : collisionGeometryView) {
    if (hasPrescribedRigidBodyContactResponse(registry, entity)) {
      continue;
    }

    if (registry.all_of<comps::RigidBodyTag>(entity)
        || registry.all_of<comps::Link>(entity)) {
      return false;
    }
  }

  return true;
}

//==============================================================================
// Positional correction (projection) for rigid-body normal contacts: removes
// residual penetration beyond a small allowance without injecting velocity, so
// resting stacks do not sink. Shared by the sequential-impulse and boxed-LCP
// contact paths; the sequential path inlines an equivalent loop over its
// precomputed constraints, while the LCP path drives this from the raw
// contacts. Only rigid-body/rigid-body pairs with at least one dynamic body
// are corrected.
void resolveRigidBodyContactPositions(
    detail::WorldRegistry& registry,
    const std::vector<Contact>& contacts,
    double /*timeStep*/)
{
  constexpr double allowance = 1e-4;
  constexpr double correctionFactor = 0.2;
  for (const auto& contact : contacts) {
    const auto entityA = detail::toRegistryEntity(contact.bodyA.getEntity());
    const auto entityB = detail::toRegistryEntity(contact.bodyB.getEntity());
    if (!registry.all_of<comps::RigidBodyTag>(entityA)
        || !registry.all_of<comps::RigidBodyTag>(entityB)) {
      continue;
    }

    const double penetration = contact.depth - allowance;
    if (penetration <= 0.0) {
      continue;
    }

    const bool staticA
        = hasPrescribedRigidBodyContactResponse(registry, entityA);
    const bool staticB
        = hasPrescribedRigidBodyContactResponse(registry, entityB);
    const double invMassA
        = staticA ? 0.0
                  : inverseMass(registry.get<comps::MassProperties>(entityA));
    const double invMassB
        = staticB ? 0.0
                  : inverseMass(registry.get<comps::MassProperties>(entityB));
    const double totalInverseMass = invMassA + invMassB;
    if (totalInverseMass <= 0.0) {
      continue;
    }

    const Eigen::Vector3d correction
        = (correctionFactor * penetration / totalInverseMass) * contact.normal;
    registry.get<comps::Transform>(entityA).position -= invMassA * correction;
    registry.get<comps::Transform>(entityB).position += invMassB * correction;
  }
}

//==============================================================================
struct StaticGroundBarrier
{
  enum class Shape
  {
    Box,
    Sphere,
  };

  Shape shape = Shape::Box;
  Eigen::Vector3d center = Eigen::Vector3d::Zero();
  Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();
  Eigen::Vector3d halfExtents = Eigen::Vector3d::Zero();
  double radius = 0.0;
  double top = 0.0;
};

//==============================================================================
// A static sphere obstacle that exerts a full radial clamped-log barrier force
// on nearby deformable nodes (unlike the vertical-only ground barrier above),
// so a deformable settles smoothly at ~d_hat against any side of the sphere.
struct SphereObstacleBarrier
{
  Eigen::Vector3d center = Eigen::Vector3d::Zero();
  double radius = 0.0;
};

// A static (oriented) box opted in as a deformable obstacle exerts a
// clamped-log barrier on nearby deformable nodes along the outward surface
// normal -- the box analogue of the sphere obstacle barrier. Outside the box,
// the closest point on the box surface is found by clamping the node into the
// box's local frame, which handles face/edge/corner contact uniformly. Inside
// the box, the nearest exit face supplies the outward normal and penetration
// depth.
struct BoxObstacleBarrier
{
  Eigen::Vector3d center = Eigen::Vector3d::Zero();
  Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();
  Eigen::Vector3d halfExtents = Eigen::Vector3d::Zero();
};

// A static capsule (a segment swept by a radius -- a rod/wire) opted in as a
// deformable obstacle exerts a clamped-log barrier on nearby deformable nodes
// along the outward radial normal. The closest point on the capsule surface is
// found from the point-to-segment closest point, so the distance is
// |node - closestOnSegment| - radius. This is the codimensional (1D) obstacle
// analogue of the sphere (0D point) and box obstacles.
struct CapsuleObstacleBarrier
{
  Eigen::Vector3d pointA = Eigen::Vector3d::Zero();
  Eigen::Vector3d pointB = Eigen::Vector3d::Zero();
  double radius = 0.0;
};

//==============================================================================
// A lagged self-contact friction contact: the four stencil nodes, the lagged
// normal-force magnitude, and the tangent projection (2x12) that maps the
// stacked four-node displacement to tangential relative motion. Computed once
// per outer iteration (standard IPC lagging) at the current iterate.
struct SelfContactFrictionContact
{
  std::array<std::size_t, 4> nodes{};
  double normalForce = 0.0;
  dc::Matrix2x12d projection = dc::Matrix2x12d::Zero();
};

//==============================================================================
struct ProjectedNewtonMatrixFreeBlock3
{
  std::size_t rowNode = 0;
  std::size_t colNode = 0;
  Eigen::Matrix3d block = Eigen::Matrix3d::Zero();
};

//==============================================================================
struct DeformableContactSolverScratch
{
  std::vector<DeformableSurfaceTriangle> surfaceTriangles;
  std::vector<std::uint8_t> surfaceContactPointMask;
  dc::ContactCandidateSet candidates;
  // Self-contact barrier active set, assembled once per outer solver iteration
  // at the current positions (within the barrier activation distance d_hat).
  dc::ContactCandidateSet barrierCandidates;
  dc::detail::ContactCandidateSweepScratch sweepScratch;
  std::vector<dc::SurfaceEdge> interBodyCurrentEdges;
  std::vector<dc::detail::SweepItem> interBodyCurrentPointItems;
  std::vector<dc::detail::SweepItem> interBodyObstaclePointItems;
  std::vector<dc::detail::SweepItem> interBodyCurrentTriangleItems;
  std::vector<dc::detail::SweepItem> interBodyObstacleTriangleItems;
  std::vector<dc::detail::SweepItem> interBodyCurrentEdgeItems;
  std::vector<dc::detail::SweepItem> interBodyObstacleEdgeItems;
  std::vector<std::size_t> interBodySweepLinks;

  // Persistent projected-Newton sparse Cholesky. The fill-reducing symbolic
  // factorization is reused across iterations and steps whenever the assembled
  // Hessian sparsity pattern (cached column/row index arrays) is unchanged, so
  // only the numeric factorization repeats. Behavior-preserving: a structure
  // mismatch (or a failed factorization) re-runs analyzePattern.
  Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>> newtonSolver;
  std::vector<int> newtonPatternOuter;
  std::vector<int> newtonPatternInner;
  bool newtonPatternValid = false;

  // Cached per-tetrahedron FEM rest shapes (inverse rest edge matrix + rest
  // volume). The rest configuration never changes, so these are computed once
  // (when the cached count first matches the body's tetrahedron count) and
  // reused every step instead of re-inverting each tet's rest edges per step.
  std::vector<fem::TetRestShape> femRestShapes;

  Eigen::VectorXd projectedNewtonRhs;
  Eigen::VectorXd projectedNewtonSolution;
  Eigen::SparseMatrix<double> projectedNewtonHessian;
  std::vector<Eigen::Triplet<double>> projectedNewtonTriplets;
  std::vector<double> projectedNewtonEdgeBlocks;
  std::vector<std::array<std::size_t, 2>> projectedNewtonEdgeBlockNodes;
  std::vector<double> projectedNewtonTetBlocks;
  std::vector<std::array<std::size_t, 4>> projectedNewtonTetBlockNodes;
  std::vector<double> projectedNewtonBarrierBlocks;
  std::vector<std::array<std::size_t, 4>> projectedNewtonBarrierBlockNodes;
  std::vector<ProjectedNewtonMatrixFreeBlock3> projectedNewtonMatrixFreeBlocks;
  std::vector<Eigen::Matrix3d> projectedNewtonMatrixFreeDiagonalBlocks;
  std::vector<Eigen::Matrix3d> projectedNewtonMatrixFreeInverseDiagonalBlocks;
  Eigen::VectorXd projectedNewtonMatrixFreeResidual;
  Eigen::VectorXd projectedNewtonMatrixFreePreconditionedResidual;
  Eigen::VectorXd projectedNewtonMatrixFreeDirection;
  Eigen::VectorXd projectedNewtonMatrixFreeHessianDirection;

  std::vector<double> groundFrictionNormalForce;
  std::vector<Eigen::Vector3d> groundFrictionNormalDirection;
  std::vector<SelfContactFrictionContact> selfContactFrictionContacts;
};

//==============================================================================
struct ProjectedNewtonMatrixFreeHessian
{
  ProjectedNewtonMatrixFreeHessian(
      std::vector<ProjectedNewtonMatrixFreeBlock3>& blockStorage,
      std::vector<Eigen::Matrix3d>& diagonalStorage,
      std::vector<Eigen::Matrix3d>& inverseDiagonalStorage)
    : blocks(blockStorage),
      diagonalBlocks(diagonalStorage),
      inverseDiagonalBlocks(inverseDiagonalStorage)
  {
  }

  void reset(std::size_t nodes)
  {
    nodeCount = nodes;
    blocks.clear();
    diagonalBlocks.resize(nodeCount);
    for (auto& block : diagonalBlocks) {
      block.setZero();
    }
  }

  void addBlock3(
      std::size_t rowNode, std::size_t colNode, const Eigen::Matrix3d& block)
  {
    blocks.push_back({rowNode, colNode, block});
    if (rowNode == colNode) {
      diagonalBlocks[rowNode] += block;
    }
  }

  void multiplyInto(const Eigen::VectorXd& x, Eigen::VectorXd& y) const
  {
    y.resize(static_cast<Eigen::Index>(3 * nodeCount));
    y.setZero();
    for (const auto& entry : blocks) {
      y.segment<3>(static_cast<Eigen::Index>(3 * entry.rowNode))
          += entry.block
             * x.segment<3>(static_cast<Eigen::Index>(3 * entry.colNode));
    }
  }

  [[nodiscard]] bool factorBlockJacobi()
  {
    inverseDiagonalBlocks.resize(nodeCount);
    for (std::size_t i = 0; i < nodeCount; ++i) {
      Eigen::LLT<Eigen::Matrix3d> llt(diagonalBlocks[i]);
      if (llt.info() != Eigen::Success) {
        return false;
      }
      inverseDiagonalBlocks[i] = llt.solve(Eigen::Matrix3d::Identity()).eval();
      if (!inverseDiagonalBlocks[i].allFinite()) {
        return false;
      }
    }
    return true;
  }

  void applyPreconditionerInto(
      const Eigen::VectorXd& residual, Eigen::VectorXd& z) const
  {
    z.resize(static_cast<Eigen::Index>(3 * nodeCount));
    for (std::size_t i = 0; i < nodeCount; ++i) {
      z.segment<3>(static_cast<Eigen::Index>(3 * i))
          = inverseDiagonalBlocks[i]
            * residual.segment<3>(static_cast<Eigen::Index>(3 * i));
    }
  }

  std::size_t nodeCount = 0;
  std::vector<ProjectedNewtonMatrixFreeBlock3>& blocks;
  std::vector<Eigen::Matrix3d>& diagonalBlocks;
  std::vector<Eigen::Matrix3d>& inverseDiagonalBlocks;
};

//==============================================================================
bool solveMatrixFreeConjugateGradient(
    ProjectedNewtonMatrixFreeHessian& hessian,
    const Eigen::VectorXd& rhs,
    Eigen::VectorXd& solution,
    Eigen::VectorXd& residual,
    Eigen::VectorXd& z,
    Eigen::VectorXd& direction,
    Eigen::VectorXd& hessianDirection,
    std::size_t& iterations,
    double& relativeResidual)
{
  constexpr double kTolerance = 1e-8;
  const Eigen::Index maxIterations = 2 * rhs.size();
  solution.resize(rhs.size());
  solution.setZero();
  iterations = 0;
  relativeResidual = 0.0;

  if (!hessian.factorBlockJacobi()) {
    return false;
  }

  const double rhsNorm = rhs.norm();
  if (rhsNorm == 0.0) {
    return true;
  }

  residual.resize(rhs.size());
  residual = rhs;
  hessian.applyPreconditionerInto(residual, z);
  direction.resize(rhs.size());
  direction = z;
  double rz = residual.dot(z);
  if (!std::isfinite(rz) || rz <= 0.0 || !direction.allFinite()) {
    return false;
  }

  for (Eigen::Index iter = 0; iter < maxIterations; ++iter) {
    hessian.multiplyInto(direction, hessianDirection);
    const double curvature = direction.dot(hessianDirection);
    if (!std::isfinite(curvature) || curvature <= 0.0) {
      return false;
    }

    const double alpha = rz / curvature;
    solution += alpha * direction;
    residual -= alpha * hessianDirection;
    if (!solution.allFinite() || !residual.allFinite()) {
      return false;
    }

    relativeResidual = residual.norm() / rhsNorm;
    iterations = static_cast<std::size_t>(iter + 1);
    if (relativeResidual <= kTolerance) {
      return true;
    }

    hessian.applyPreconditionerInto(residual, z);
    const double nextRz = residual.dot(z);
    if (!std::isfinite(nextRz) || nextRz <= 0.0 || !z.allFinite()) {
      return false;
    }
    const double beta = nextRz / rz;
    direction *= beta;
    direction += z;
    if (!direction.allFinite()) {
      return false;
    }
    rz = nextRz;
  }

  return false;
}

//==============================================================================
struct SurfaceContactSnapshot
{
  entt::entity entity = entt::null;
  std::vector<Eigen::Vector3d> positions;
  std::vector<DeformableSurfaceTriangle> surfaceTriangles;
  std::vector<std::uint8_t> surfaceContactPointMask;
  std::vector<dc::SurfaceEdge> surfaceEdges;
};

//==============================================================================
SurfaceContactSnapshot& nextSurfaceContactSnapshot(
    std::vector<SurfaceContactSnapshot>& snapshots, std::size_t& count)
{
  if (count == snapshots.size()) {
    snapshots.emplace_back();
  }
  return snapshots[count++];
}

//==============================================================================
std::span<const SurfaceContactSnapshot> activeSurfaceContactSnapshots(
    const std::vector<SurfaceContactSnapshot>& snapshots, std::size_t count)
{
  return std::span<const SurfaceContactSnapshot>(snapshots.data(), count);
}

//==============================================================================
struct InterBodySurfaceContactResult
{
  bool hit = false;
  bool indeterminate = false;
  double stepBound = 1.0;
  std::size_t pointTriangleCandidateCount = 0;
  std::size_t edgeEdgeCandidateCount = 0;
  dc::ContinuousCollisionStepStats stats;
};

//==============================================================================
double surfaceContactMinSeparation()
{
  return 1e-4;
}

//==============================================================================
double surfaceContactTolerance()
{
  return 1e-6;
}

//==============================================================================
dc::ContactCandidateOptions makeSurfaceContactCandidateOptions()
{
  dc::ContactCandidateOptions options;
  options.activationDistance
      = surfaceContactMinSeparation() + surfaceContactTolerance();
  options.exactDistanceFilter = true;
  options.excludeIncidentPointTriangles = true;
  options.excludeAdjacentEdges = true;
  return options;
}

//==============================================================================
dc::ContinuousCollisionStepOptions makeSurfaceContactCcdOptions()
{
  dc::ContinuousCollisionStepOptions options;
  options.minSeparation = surfaceContactMinSeparation();
  options.tolerance = surfaceContactTolerance();
  options.maxIterations = 128;
  return options;
}

//==============================================================================
void copySurfaceContactTopology(
    std::span<const comps::DeformableSurfaceTriangle> source,
    std::size_t nodeCount,
    bool restrictPointsToReferencedSurfaceNodes,
    std::vector<DeformableSurfaceTriangle>& surfaceTriangles,
    std::vector<std::uint8_t>& surfaceContactPointMask)
{
  surfaceTriangles.clear();
  surfaceTriangles.reserve(source.size());
  if (restrictPointsToReferencedSurfaceNodes) {
    surfaceContactPointMask.assign(nodeCount, 0u);
  } else {
    surfaceContactPointMask.clear();
  }

  for (const auto& triangle : source) {
    surfaceTriangles.push_back(
        DeformableSurfaceTriangle{
            triangle.nodeA, triangle.nodeB, triangle.nodeC});
    if (restrictPointsToReferencedSurfaceNodes) {
      surfaceContactPointMask[triangle.nodeA] = 1u;
      surfaceContactPointMask[triangle.nodeB] = 1u;
      surfaceContactPointMask[triangle.nodeC] = 1u;
    }
  }
}

//==============================================================================
void syncSurfaceContactTopology(
    std::span<const comps::DeformableSurfaceTriangle> source,
    std::size_t nodeCount,
    bool restrictPointsToReferencedSurfaceNodes,
    DeformableContactSolverScratch& scratch)
{
  copySurfaceContactTopology(
      source,
      nodeCount,
      restrictPointsToReferencedSurfaceNodes,
      scratch.surfaceTriangles,
      scratch.surfaceContactPointMask);
}

//==============================================================================
void filterSurfaceContactPointCandidates(
    dc::ContactCandidateSet& candidates,
    std::span<const std::uint8_t> pointMask)
{
  if (pointMask.empty()) {
    return;
  }

  auto& pointTriangleCandidates = candidates.pointTriangleCandidates;
  pointTriangleCandidates.erase(
      std::remove_if(
          pointTriangleCandidates.begin(),
          pointTriangleCandidates.end(),
          [&](const dc::PointTriangleCandidate& candidate) {
            return candidate.point >= pointMask.size()
                   || pointMask[candidate.point] == 0u;
          }),
      pointTriangleCandidates.end());
  candidates.stats.pointTriangleCandidateCount = pointTriangleCandidates.size();
}

//==============================================================================
bool surfaceContactPointAllowed(
    const std::size_t point, std::span<const std::uint8_t> pointMask)
{
  return pointMask.empty()
         || (point < pointMask.size() && pointMask[point] != 0u);
}

//==============================================================================
void considerInterBodyContactResult(
    InterBodySurfaceContactResult& aggregate,
    const dc::ContinuousCollisionStepResult& candidate)
{
  nb::accumulateLineSearchStats(aggregate.stats, candidate.stats);
  aggregate.indeterminate = aggregate.indeterminate || candidate.indeterminate;
  if (candidate.indeterminate) {
    aggregate.stepBound = 0.0;
    return;
  }

  if (!candidate.hit) {
    return;
  }

  if (!aggregate.hit || candidate.stepBound < aggregate.stepBound) {
    aggregate.hit = true;
    aggregate.stepBound = candidate.stepBound;
  }
}

//==============================================================================
void buildPointSweepItems(
    std::span<const Eigen::Vector3d> positionsStart,
    std::span<const Eigen::Vector3d> positionsEnd,
    std::span<const std::uint8_t> pointMask,
    double margin,
    std::vector<dc::detail::SweepItem>& items)
{
  items.clear();
  items.reserve(positionsStart.size());
  for (std::size_t point = 0; point < positionsStart.size(); ++point) {
    if (!surfaceContactPointAllowed(point, pointMask)) {
      continue;
    }
    items.push_back(
        dc::detail::SweepItem{
            point,
            dc::detail::makeSweptPointAabb(
                positionsStart[point], positionsEnd[point], margin)});
  }
}

//==============================================================================
void buildTriangleSweepItems(
    std::span<const Eigen::Vector3d> positionsStart,
    std::span<const Eigen::Vector3d> positionsEnd,
    std::span<const DeformableSurfaceTriangle> triangles,
    double margin,
    std::vector<dc::detail::SweepItem>& items)
{
  items.clear();
  items.reserve(triangles.size());
  for (std::size_t triangle = 0; triangle < triangles.size(); ++triangle) {
    const auto& t = triangles[triangle];
    items.push_back(
        dc::detail::SweepItem{
            triangle,
            dc::detail::makeSweptTriangleAabb(
                positionsStart[t.nodeA],
                positionsEnd[t.nodeA],
                positionsStart[t.nodeB],
                positionsEnd[t.nodeB],
                positionsStart[t.nodeC],
                positionsEnd[t.nodeC],
                margin)});
  }
}

//==============================================================================
void buildEdgeSweepItems(
    std::span<const Eigen::Vector3d> positionsStart,
    std::span<const Eigen::Vector3d> positionsEnd,
    std::span<const dc::SurfaceEdge> edges,
    double margin,
    std::vector<dc::detail::SweepItem>& items)
{
  items.clear();
  items.reserve(edges.size());
  for (std::size_t edge = 0; edge < edges.size(); ++edge) {
    const auto& e = edges[edge];
    items.push_back(
        dc::detail::SweepItem{
            edge,
            dc::detail::makeSweptSegmentAabb(
                positionsStart[e.nodeA],
                positionsEnd[e.nodeA],
                positionsStart[e.nodeB],
                positionsEnd[e.nodeB],
                margin)});
  }
}

//==============================================================================
InterBodySurfaceContactResult interBodySurfaceContactStepBound(
    std::span<const Eigen::Vector3d> currentStart,
    std::span<const Eigen::Vector3d> currentEnd,
    std::span<const DeformableSurfaceTriangle> currentTriangles,
    std::span<const std::uint8_t> currentPointMask,
    std::span<const dc::SurfaceEdge> currentEdges,
    const SurfaceContactSnapshot& obstacle,
    const dc::ContactCandidateOptions& candidateOptions,
    const dc::ContinuousCollisionStepOptions& ccdOptions,
    DeformableContactSolverScratch& scratch)
{
  InterBodySurfaceContactResult aggregate;
  const double margin
      = 0.5 * dc::detail::nonnegativeActivationDistance(candidateOptions);

  buildPointSweepItems(
      currentStart,
      currentEnd,
      currentPointMask,
      margin,
      scratch.interBodyCurrentPointItems);
  buildPointSweepItems(
      obstacle.positions,
      obstacle.positions,
      obstacle.surfaceContactPointMask,
      margin,
      scratch.interBodyObstaclePointItems);
  buildTriangleSweepItems(
      currentStart,
      currentEnd,
      currentTriangles,
      margin,
      scratch.interBodyCurrentTriangleItems);
  buildTriangleSweepItems(
      obstacle.positions,
      obstacle.positions,
      obstacle.surfaceTriangles,
      margin,
      scratch.interBodyObstacleTriangleItems);

  dc::detail::visitSweepPairs(
      scratch.interBodyCurrentPointItems,
      scratch.interBodyObstacleTriangleItems,
      [&](const std::size_t point, const std::size_t triangleIndex) {
        ++aggregate.pointTriangleCandidateCount;
        const auto& triangle = obstacle.surfaceTriangles[triangleIndex];
        const auto result = dc::pointTriangleStepBound(
            currentStart[point],
            currentEnd[point],
            obstacle.positions[triangle.nodeA],
            obstacle.positions[triangle.nodeA],
            obstacle.positions[triangle.nodeB],
            obstacle.positions[triangle.nodeB],
            obstacle.positions[triangle.nodeC],
            obstacle.positions[triangle.nodeC],
            ccdOptions);
        considerInterBodyContactResult(aggregate, result);
      },
      scratch.interBodySweepLinks);

  dc::detail::visitSweepPairs(
      scratch.interBodyObstaclePointItems,
      scratch.interBodyCurrentTriangleItems,
      [&](const std::size_t point, const std::size_t triangleIndex) {
        ++aggregate.pointTriangleCandidateCount;
        const auto& triangle = currentTriangles[triangleIndex];
        const auto result = dc::pointTriangleStepBound(
            obstacle.positions[point],
            obstacle.positions[point],
            currentStart[triangle.nodeA],
            currentEnd[triangle.nodeA],
            currentStart[triangle.nodeB],
            currentEnd[triangle.nodeB],
            currentStart[triangle.nodeC],
            currentEnd[triangle.nodeC],
            ccdOptions);
        considerInterBodyContactResult(aggregate, result);
      },
      scratch.interBodySweepLinks);

  buildEdgeSweepItems(
      currentStart,
      currentEnd,
      currentEdges,
      margin,
      scratch.interBodyCurrentEdgeItems);
  buildEdgeSweepItems(
      obstacle.positions,
      obstacle.positions,
      obstacle.surfaceEdges,
      margin,
      scratch.interBodyObstacleEdgeItems);
  dc::detail::visitSweepPairs(
      scratch.interBodyCurrentEdgeItems,
      scratch.interBodyObstacleEdgeItems,
      [&](const std::size_t currentEdge, const std::size_t obstacleEdge) {
        ++aggregate.edgeEdgeCandidateCount;
        const auto& a = currentEdges[currentEdge];
        const auto& b = obstacle.surfaceEdges[obstacleEdge];
        const auto result = dc::edgeEdgeStepBound(
            currentStart[a.nodeA],
            currentEnd[a.nodeA],
            currentStart[a.nodeB],
            currentEnd[a.nodeB],
            obstacle.positions[b.nodeA],
            obstacle.positions[b.nodeA],
            obstacle.positions[b.nodeB],
            obstacle.positions[b.nodeB],
            ccdOptions);
        considerInterBodyContactResult(aggregate, result);
      },
      scratch.interBodySweepLinks);

  return aggregate;
}

//==============================================================================
void fillStaticBoxSurfaceCcdSnapshot(
    SurfaceContactSnapshot& snapshot,
    entt::entity entity,
    const Eigen::Vector3d& halfExtents,
    const comps::Transform& transform)
{
  snapshot.entity = entity;
  snapshot.positions.clear();
  snapshot.positions.reserve(8);
  snapshot.surfaceTriangles.clear();
  snapshot.surfaceContactPointMask.clear();
  snapshot.surfaceEdges.clear();

  const Eigen::Matrix3d rotation
      = normalizeOrIdentity(transform.orientation).toRotationMatrix();
  const std::array<Eigen::Vector3d, 8> localVertices{
      Eigen::Vector3d(-halfExtents.x(), -halfExtents.y(), -halfExtents.z()),
      Eigen::Vector3d(halfExtents.x(), -halfExtents.y(), -halfExtents.z()),
      Eigen::Vector3d(halfExtents.x(), halfExtents.y(), -halfExtents.z()),
      Eigen::Vector3d(-halfExtents.x(), halfExtents.y(), -halfExtents.z()),
      Eigen::Vector3d(-halfExtents.x(), -halfExtents.y(), halfExtents.z()),
      Eigen::Vector3d(halfExtents.x(), -halfExtents.y(), halfExtents.z()),
      Eigen::Vector3d(halfExtents.x(), halfExtents.y(), halfExtents.z()),
      Eigen::Vector3d(-halfExtents.x(), halfExtents.y(), halfExtents.z())};
  for (const auto& local : localVertices) {
    snapshot.positions.push_back(transform.position + rotation * local);
  }

  constexpr std::array<std::array<std::size_t, 3>, 12> kBoxTriangles{{
      {{0, 3, 1}},
      {{1, 3, 2}},
      {{4, 5, 7}},
      {{5, 6, 7}},
      {{0, 1, 4}},
      {{1, 5, 4}},
      {{2, 3, 6}},
      {{3, 7, 6}},
      {{0, 4, 3}},
      {{3, 4, 7}},
      {{1, 2, 5}},
      {{2, 6, 5}},
  }};
  snapshot.surfaceTriangles.reserve(kBoxTriangles.size());
  for (const auto& triangle : kBoxTriangles) {
    snapshot.surfaceTriangles.push_back(
        DeformableSurfaceTriangle{triangle[0], triangle[1], triangle[2]});
  }

  constexpr std::array<std::array<std::size_t, 2>, 12> kBoxEdges{{
      {{0, 1}},
      {{1, 2}},
      {{2, 3}},
      {{0, 3}},
      {{4, 5}},
      {{5, 6}},
      {{6, 7}},
      {{4, 7}},
      {{0, 4}},
      {{1, 5}},
      {{2, 6}},
      {{3, 7}},
  }};
  snapshot.surfaceEdges.reserve(kBoxEdges.size());
  for (const auto& edge : kBoxEdges) {
    snapshot.surfaceEdges.push_back(
        dc::detail::makeSurfaceEdge(edge[0], edge[1]));
  }
}

//==============================================================================
// A static sphere has no flat faces to triangulate, so it is tessellated into a
// UV-sphere polyhedron that CIRCUMSCRIBES the true sphere: the vertices sit at
// a slightly larger radius so every flat face stays outside the analytic
// surface. That keeps the conservative no-penetration guarantee (a deformable
// is stopped at the polyhedron, never inside the real sphere), and the existing
// point-triangle / edge-edge CCD reducers consume the snapshot unchanged. The
// outward inflation is the modest over-conservatism this introduces (smaller
// with finer tessellation), analogous to the box supersampling's thin-corner
// over-coverage.
void fillStaticSphereSurfaceCcdSnapshot(
    SurfaceContactSnapshot& snapshot,
    entt::entity entity,
    double radius,
    const comps::Transform& transform)
{
  constexpr int kLongitude = 16; // segments around the equator
  constexpr int kLatitude = 8;   // bands from pole to pole
  constexpr double kPi = 3.14159265358979323846;
  constexpr std::size_t kNorthPole = 0;

  const double dTheta = 2.0 * kPi / static_cast<double>(kLongitude);
  const double dPhi = kPi / static_cast<double>(kLatitude);
  // Inflate the vertex radius so the flat faces (chords) circumscribe the
  // sphere: a cell's circumscribed cap has angular radius ~0.5 * the cell
  // diagonal, and a face at vertex radius r sits at r*cos(that) from the
  // center, so r = radius / cos(angularRadius) puts every face at >= radius. A
  // small safety factor absorbs rounding.
  const double angularRadius = 0.5 * std::sqrt(dTheta * dTheta + dPhi * dPhi);
  const double r = radius * (1.001 / std::cos(angularRadius));

  const Eigen::Matrix3d rotation
      = normalizeOrIdentity(transform.orientation).toRotationMatrix();
  const auto worldPoint = [&](const Eigen::Vector3d& local) {
    return Eigen::Vector3d(transform.position + rotation * local);
  };

  snapshot.entity = entity;
  snapshot.positions.clear();
  snapshot.surfaceTriangles.clear();
  snapshot.surfaceContactPointMask.clear();
  snapshot.surfaceEdges.clear();

  // Vertices: north pole, (kLatitude - 1) interior rings of kLongitude vertices
  // each, then the south pole.
  snapshot.positions.push_back(worldPoint(Eigen::Vector3d(0.0, 0.0, r)));
  for (int i = 1; i < kLatitude; ++i) {
    const double phi = dPhi * static_cast<double>(i);
    const double z = r * std::cos(phi);
    const double ringRadius = r * std::sin(phi);
    for (int j = 0; j < kLongitude; ++j) {
      const double theta = dTheta * static_cast<double>(j);
      snapshot.positions.push_back(worldPoint(
          Eigen::Vector3d(
              ringRadius * std::cos(theta), ringRadius * std::sin(theta), z)));
    }
  }
  const auto southPole = snapshot.positions.size();
  snapshot.positions.push_back(worldPoint(Eigen::Vector3d(0.0, 0.0, -r)));

  // Vertex index for column j (wrapping) of interior ring in [1, kLatitude -
  // 1].
  const auto ringVertex = [&](int ring, int j) {
    const int col = ((j % kLongitude) + kLongitude) % kLongitude;
    return static_cast<std::size_t>(1 + (ring - 1) * kLongitude + col);
  };

  for (int j = 0; j < kLongitude; ++j) {
    snapshot.surfaceTriangles.push_back(
        DeformableSurfaceTriangle{
            kNorthPole, ringVertex(1, j), ringVertex(1, j + 1)});
  }
  for (int i = 1; i < kLatitude - 1; ++i) {
    for (int j = 0; j < kLongitude; ++j) {
      const auto a = ringVertex(i, j);
      const auto b = ringVertex(i, j + 1);
      const auto c = ringVertex(i + 1, j);
      const auto d = ringVertex(i + 1, j + 1);
      snapshot.surfaceTriangles.push_back(DeformableSurfaceTriangle{a, c, b});
      snapshot.surfaceTriangles.push_back(DeformableSurfaceTriangle{b, c, d});
    }
  }
  for (int j = 0; j < kLongitude; ++j) {
    snapshot.surfaceTriangles.push_back(
        DeformableSurfaceTriangle{
            southPole,
            ringVertex(kLatitude - 1, j + 1),
            ringVertex(kLatitude - 1, j)});
  }

  // Unlike a box (whose triangulation diagonals are not physical edges), every
  // sphere tessellation edge lies on the surface, so the full unique edge set
  // is the correct edge-edge CCD input.
  dc::buildUniqueSurfaceEdges(snapshot.surfaceTriangles, snapshot.surfaceEdges);
}

//==============================================================================
bool isCurrentPoseRigidSurfaceCcdObstacle(
    const detail::WorldRegistry& registry, const entt::entity entity)
{
  return registry.all_of<comps::StaticBodyTag>(entity)
         || registry.all_of<comps::KinematicBodyTag>(entity);
}

//==============================================================================
bool hasCurrentKinematicStepTrace(const World& world, const entt::entity entity)
{
  const auto& registry = dart::simulation::detail::registryOf(world);
  const auto* trace = registry.try_get<comps::KinematicBodyStepTrace>(entity);
  return trace != nullptr && trace->frame == world.getFrame();
}

//==============================================================================
void collectStaticRigidSurfaceCcdObstaclesInto(
    const World& world,
    DeformableSolverStats& stats,
    std::vector<SurfaceContactSnapshot>& snapshots,
    std::size_t& snapshotCount)
{
  ++stats.staticRigidSurfaceCcdSnapshotBuilds;
  snapshotCount = 0;

  const auto& registry = dart::simulation::detail::registryOf(world);
  // Barrier-only obstacles keep their contact barrier but are excluded from the
  // CCD limiter, so a deformable can slide tangentially against them (and be
  // decelerated by friction) instead of having its whole step over-limited.
  auto view = registry.view<
      comps::RigidBodyTag,
      comps::DeformableSurfaceCcdObstacleTag,
      comps::CollisionGeometry,
      comps::Transform>(entt::exclude<comps::DeformableObstacleNoCcdTag>);

  for (const auto entity : view) {
    if (!isCurrentPoseRigidSurfaceCcdObstacle(registry, entity)) {
      continue;
    }
    const auto& geometry = view.get<comps::CollisionGeometry>(entity);
    const auto* shape = geometry.getPrimaryShape();
    if (shape == nullptr) {
      continue;
    }
    // Current-frame kinematic boxes move to the swept moving collector; other
    // supported shapes stay as current-pose snapshots until they gain swept
    // snapshot support.
    if (hasCurrentKinematicStepTrace(world, entity)
        && shape->type == CollisionShapeType::Box) {
      continue;
    }
    const auto& transform = view.get<comps::Transform>(entity);
    const auto shapeTransform = collisionShapeWorldTransform(transform, *shape);
    if (!shapeTransform.has_value()) {
      continue;
    }

    if (shape->type == CollisionShapeType::Box) {
      if (!shape->halfExtents.allFinite()
          || (shape->halfExtents.array() <= 0.0).any()) {
        continue;
      }
      auto& snapshot = nextSurfaceContactSnapshot(snapshots, snapshotCount);
      fillStaticBoxSurfaceCcdSnapshot(
          snapshot, entity, shape->halfExtents, *shapeTransform);
      ++stats.staticRigidSurfaceCcdBoxCount;
      stats.staticRigidSurfaceCcdTriangleCount
          += snapshot.surfaceTriangles.size();
      stats.staticRigidSurfaceCcdEdgeCount += snapshot.surfaceEdges.size();
    } else if (shape->type == CollisionShapeType::Sphere) {
      if (!std::isfinite(shape->radius) || shape->radius <= 0.0) {
        continue;
      }
      auto& snapshot = nextSurfaceContactSnapshot(snapshots, snapshotCount);
      fillStaticSphereSurfaceCcdSnapshot(
          snapshot, entity, shape->radius, *shapeTransform);
      ++stats.staticRigidSurfaceCcdSphereCount;
      stats.staticRigidSurfaceCcdTriangleCount
          += snapshot.surfaceTriangles.size();
      stats.staticRigidSurfaceCcdEdgeCount += snapshot.surfaceEdges.size();
    } else {
      continue;
    }
  }
}

//==============================================================================
// Predict a free rigid body's end-of-step transform from its current pose and
// velocity WITHOUT mutating any component. Mirrors integrateRigidBodyPosition
// exactly so the deformable stage limits against the SAME motion that
// RigidBodyPositionStage will later apply at stage 5.
comps::Transform predictRigidBodyEndTransform(
    const comps::Transform& transform,
    const comps::Velocity& velocity,
    const double timeStep)
{
  comps::Transform end = transform;
  end.position = transform.position + velocity.linear * timeStep;

  auto orientation = normalizeOrIdentity(transform.orientation);
  const double angularSpeed = velocity.angular.norm();
  if (angularSpeed > 0.0 && std::isfinite(angularSpeed)) {
    const auto rotation = Eigen::AngleAxisd(
        angularSpeed * timeStep, velocity.angular.normalized());
    orientation = rotation * orientation;
    orientation.normalize();
  }
  end.orientation = normalizeOrIdentity(orientation);
  return end;
}

//==============================================================================
comps::Transform predictKinematicRigidSurfaceCcdEndTransform(
    const World& world,
    const detail::WorldRegistry& registry,
    const entt::entity entity,
    const comps::Transform& transform,
    const comps::Velocity& velocity)
{
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
  return predictRigidBodyEndTransform(transform, velocity, timeStep);
}

//==============================================================================
// Number of intermediate static box poses that tile a moving obstacle's swept
// motion. Below the cap, consecutive sample centers are one box min-half-extent
// apart so the sampled boxes overlap with margin. The count is capped so a
// pathologically fast obstacle cannot explode the work; past the cap the
// residual spacing grows, and the caller restores overlap by inflating the
// sampled boxes (see collectMovingRigidSurfaceCcdObstacles), so the swept
// corridor stays covered rather than developing tunnelable gaps.
std::size_t movingRigidSurfaceCcdSampleCount(
    const Eigen::Vector3d& halfExtents,
    const comps::Transform& startTransform,
    const comps::Transform& endTransform,
    const Eigen::Vector3d& angularVelocity,
    const double timeStep)
{
  constexpr std::size_t kMaxSamples = 64;
  const double minHalfExtent = halfExtents.minCoeff();
  if (!(minHalfExtent > 0.0)) {
    return 2;
  }

  const double linearMotion
      = (endTransform.position - startTransform.position).norm();
  const double radius = halfExtents.norm();
  const double angularMotion = radius * angularVelocity.norm() * timeStep;
  const double motion = linearMotion + angularMotion;
  if (!std::isfinite(motion) || motion <= 0.0) {
    return 2;
  }

  const auto segments
      = static_cast<std::size_t>(std::ceil(motion / minHalfExtent));
  const std::size_t samples = std::min(kMaxSamples, segments + 1);
  return std::max<std::size_t>(samples, 2);
}

//==============================================================================
// Collect conservative static box snapshots that tile each MOVING rigid box
// obstacle's swept motion over the step. Each snapshot is an ordinary static
// pose, so the existing static-style limiter and
// interBodySurfaceContactStepBound handle them unchanged. Free-rigid obstacle
// end poses are predicted from velocity because deformable dynamics runs before
// RigidBodyPositionStage. Kinematic obstacle end poses come from the current
// step trace written by rigid IPC, so deformables are limited against the
// realized start->current motion instead of a future frame.
//
// This treats the swept volume as a static blocker for the step (timing-
// agnostic): it is more conservative than a timing-aware sweep for fast
// obstacles. At IPC-scale time steps the per-step motion is small, so few
// samples closely approximate the true contact surface. Consecutive sample
// boxes are kept overlapping so the deformable cannot tunnel between them: when
// the sample count hits its cap and the natural spacing would open gaps, the
// sampled boxes are isotropically inflated by half the residual spacing, which
// keeps the real obstacle inside each sample and the corridor covered (at the
// cost of extra over-conservatism). Coverage of the swept hull is exact along
// the box min axis for axis-aligned motion; for diagonal motion a bounded,
// half-extent-scale thin-corner under-coverage remains, inherent to all box
// supersampling.
void collectMovingRigidSurfaceCcdObstaclesInto(
    const World& world,
    const double timeStep,
    DeformableSolverStats& stats,
    std::vector<SurfaceContactSnapshot>& snapshots,
    std::size_t& snapshotCount,
    const bool primeKinematicWithoutCurrentTrace = false)
{
  ++stats.movingRigidSurfaceCcdSnapshotBuilds;
  snapshotCount = 0;

  const auto& registry = dart::simulation::detail::registryOf(world);
  // Moving obstacles are free rigid bodies integrated by RigidBodyPositionStage
  // or kinematic bodies already advanced by the rigid IPC stage. Static bodies
  // stay in collectStaticRigidSurfaceCcdObstacles. Kinematic bodies only enter
  // this moving set when rigid IPC left a current-frame trace; otherwise they
  // remain current-pose obstacles.
  auto view = registry.view<
      comps::RigidBodyTag,
      comps::DeformableSurfaceCcdObstacleTag,
      comps::CollisionGeometry,
      comps::Transform,
      comps::Velocity>(entt::exclude<comps::StaticBodyTag>);

  for (const auto entity : view) {
    const auto& geometry = view.get<comps::CollisionGeometry>(entity);
    const auto* shape = geometry.getPrimaryShape();
    if (shape == nullptr) {
      continue;
    }
    const auto& transform = view.get<comps::Transform>(entity);
    const auto& velocity = view.get<comps::Velocity>(entity);
    const bool isKinematic = registry.all_of<comps::KinematicBodyTag>(entity);
    const auto* trace = registry.try_get<comps::KinematicBodyStepTrace>(entity);
    if (shape->type != CollisionShapeType::Box
        || !shape->halfExtents.allFinite()
        || (shape->halfExtents.array() <= 0.0).any()
        || !velocity.linear.allFinite() || !velocity.angular.allFinite()) {
      continue;
    }
    comps::Transform startTransform = transform;
    comps::Transform endTransform;
    if (isKinematic) {
      if (trace != nullptr && trace->frame == world.getFrame()) {
        startTransform = trace->startTransform;
        endTransform = trace->endTransform;
      } else if (primeKinematicWithoutCurrentTrace) {
        endTransform = predictKinematicRigidSurfaceCcdEndTransform(
            world, registry, entity, transform, velocity);
      } else {
        continue;
      }
    } else {
      endTransform
          = predictRigidBodyEndTransform(transform, velocity, timeStep);
    }
    const auto startShapeTransform
        = collisionShapeWorldTransform(startTransform, *shape);
    if (!startShapeTransform.has_value()) {
      continue;
    }

    const auto endShapeTransform
        = collisionShapeWorldTransform(endTransform, *shape);
    if (!endShapeTransform.has_value()) {
      continue;
    }
    const auto& halfExtents = shape->halfExtents;
    const std::size_t samples = movingRigidSurfaceCcdSampleCount(
        halfExtents,
        *startShapeTransform,
        *endShapeTransform,
        velocity.angular,
        timeStep);

    // Keep consecutive sample boxes overlapping. The natural (uncapped) spacing
    // is one box min-half-extent, so the boxes overlap with margin; once the
    // sample count is capped the residual spacing can exceed the box min
    // dimension, so inflate each sampled box isotropically by half the residual
    // spacing to bridge the gap. inflation is 0 in the common (uncapped) case.
    const double minHalfExtent = halfExtents.minCoeff();
    const double linearMotion
        = (endShapeTransform->position - startShapeTransform->position).norm();
    const double angularMotion
        = halfExtents.norm() * velocity.angular.norm() * timeStep;
    const double motion = linearMotion + angularMotion;
    const double spacing = (samples > 1 && std::isfinite(motion))
                               ? motion / static_cast<double>(samples - 1)
                               : 0.0;
    const double inflation = std::max(0.0, 0.5 * spacing - minHalfExtent);
    const Eigen::Vector3d sampleHalfExtents
        = halfExtents + Eigen::Vector3d::Constant(inflation);
    if (inflation > 0.0) {
      ++stats.movingRigidSurfaceCcdInflatedBoxCount;
    }

    const Eigen::Quaterniond startOrientation
        = normalizeOrIdentity(startShapeTransform->orientation);
    const Eigen::Quaterniond endOrientation
        = normalizeOrIdentity(endShapeTransform->orientation);

    for (std::size_t k = 0; k < samples; ++k) {
      // samples is always >= 2, so the denominator is never zero.
      const double fraction
          = static_cast<double>(k) / static_cast<double>(samples - 1);
      comps::Transform sampleTransform;
      sampleTransform.position = startShapeTransform->position
                                 + fraction
                                       * (endShapeTransform->position
                                          - startShapeTransform->position);
      sampleTransform.orientation
          = startOrientation.slerp(fraction, endOrientation);

      auto& snapshot = nextSurfaceContactSnapshot(snapshots, snapshotCount);
      fillStaticBoxSurfaceCcdSnapshot(
          snapshot, entity, sampleHalfExtents, sampleTransform);
      stats.movingRigidSurfaceCcdTriangleCount
          += snapshot.surfaceTriangles.size();
      stats.movingRigidSurfaceCcdEdgeCount += snapshot.surfaceEdges.size();
      ++stats.movingRigidSurfaceCcdSampleCount;
    }
    ++stats.movingRigidSurfaceCcdBoxCount;
  }
}

//==============================================================================
// A static-ground contact sample under a deformable node: the supporting
// surface height (`top`, world z directly below/above the node) and the
// geometric outward surface normal there (unit, upward hemisphere). The barrier
// itself is a vertical height-field penalty (force along +z), but friction
// resolves its tangent plane against this true normal, so a sphere or tilted
// box gives a tilted tangent plane rather than a hardcoded xy plane.
struct StaticGroundContact
{
  double top = 0.0;
  Eigen::Vector3d normal = Eigen::Vector3d::UnitZ();
};

//==============================================================================
// Vertical ray-march of the +z line through the node's (x, y) against a
// (possibly rotated) box, returning the exit height and that exit face's
// outward world normal. The exit face is whichever slab bound the +z ray leaves
// last; its local outward normal points along sign(direction) of the binding
// axis (toward the upper hemisphere). For an axis-aligned box this is the +z
// top face with normal +z, recovering the flat-ground case exactly.
std::optional<StaticGroundContact> boxContactAt(
    const StaticGroundBarrier& barrier, const Eigen::Vector3d& position)
{
  constexpr double tolerance = 1e-12;
  const Eigen::Vector3d worldAtZero(position.x(), position.y(), 0.0);
  const Eigen::Vector3d localAtZero
      = barrier.rotation.transpose() * (worldAtZero - barrier.center);
  const Eigen::Vector3d localVertical
      = barrier.rotation.transpose() * Eigen::Vector3d::UnitZ();

  double minZ = -std::numeric_limits<double>::infinity();
  double maxZ = std::numeric_limits<double>::infinity();
  int bindingAxis = -1;
  double bindingSign = 1.0;
  for (int axis = 0; axis < 3; ++axis) {
    const double extent = barrier.halfExtents[axis];
    const double origin = localAtZero[axis];
    const double direction = localVertical[axis];
    if (std::abs(direction) <= tolerance) {
      if (origin < -extent - tolerance || origin > extent + tolerance) {
        return std::nullopt;
      }
      continue;
    }

    double intervalMin = (-extent - origin) / direction;
    double intervalMax = (extent - origin) / direction;
    if (intervalMin > intervalMax) {
      std::swap(intervalMin, intervalMax);
    }

    minZ = std::max(minZ, intervalMin);
    if (intervalMax < maxZ) {
      maxZ = intervalMax;
      bindingAxis = axis;
      // The exit face on this axis is the one the +z ray leaves through; its
      // local outward normal points along the sign of the ray direction.
      bindingSign = (direction > 0.0) ? 1.0 : -1.0;
    }
    if (minZ > maxZ + tolerance) {
      return std::nullopt;
    }
  }

  if (!std::isfinite(maxZ) || bindingAxis < 0) {
    return std::nullopt;
  }

  Eigen::Vector3d localNormal = Eigen::Vector3d::Zero();
  localNormal[bindingAxis] = bindingSign;
  Eigen::Vector3d worldNormal = barrier.rotation * localNormal;
  if (worldNormal.z() < 0.0) {
    worldNormal = -worldNormal;
  }
  const double norm = worldNormal.norm();

  StaticGroundContact contact;
  contact.top = maxZ;
  contact.normal = (norm > tolerance)
                       ? Eigen::Vector3d(worldNormal / norm)
                       : Eigen::Vector3d(Eigen::Vector3d::UnitZ());
  return contact;
}

//==============================================================================
void collectStaticGroundBarriersInto(
    const World& world, std::vector<StaticGroundBarrier>& barriers)
{
  barriers.clear();

  const auto& registry = dart::simulation::detail::registryOf(world);
  auto view = registry.view<
      comps::RigidBodyTag,
      comps::StaticBodyTag,
      comps::DeformableGroundBarrierTag,
      comps::CollisionGeometry,
      comps::Transform>();

  for (const auto entity : view) {
    const auto& geometry = view.get<comps::CollisionGeometry>(entity);
    const auto* shape = geometry.getPrimaryShape();
    if (shape == nullptr) {
      continue;
    }
    const auto& transform = view.get<comps::Transform>(entity);
    const auto shapeTransform = collisionShapeWorldTransform(transform, *shape);
    if (!shapeTransform.has_value()) {
      continue;
    }

    switch (shape->type) {
      case CollisionShapeType::Sphere: {
        const double radius = shape->radius;
        if (!(radius > 0.0) || !std::isfinite(radius)) {
          break;
        }

        StaticGroundBarrier barrier;
        barrier.shape = StaticGroundBarrier::Shape::Sphere;
        barrier.center = shapeTransform->position;
        barrier.radius = radius;
        barrier.top = shapeTransform->position.z() + radius;
        barriers.push_back(barrier);
        break;
      }
      case CollisionShapeType::Box: {
        if (!shape->halfExtents.allFinite()
            || (shape->halfExtents.array() <= 0.0).any()) {
          break;
        }

        StaticGroundBarrier barrier;
        barrier.shape = StaticGroundBarrier::Shape::Box;
        barrier.center = shapeTransform->position;
        barrier.rotation = normalizeOrIdentity(shapeTransform->orientation)
                               .toRotationMatrix();
        barrier.halfExtents = shape->halfExtents;
        barriers.push_back(barrier);
        break;
      }
      case CollisionShapeType::Capsule:
      case CollisionShapeType::Cylinder:
      case CollisionShapeType::Plane:
        break;
      case CollisionShapeType::Mesh:
        break;
    }
  }
}

//==============================================================================
// Static rigid SPHERE bodies opted in as deformable surface-CCD obstacles also
// exert a full radial barrier force on nearby deformable nodes. They are
// collected by center + radius (the smooth radial barrier needs no
// tessellation). Boxes opted in as surface-CCD obstacles are skipped here --
// their barrier force is a later increment -- and the surface CCD limiter
// remains the conservative no-penetration gate.
void collectSphereObstacleBarriersInto(
    const World& world, std::vector<SphereObstacleBarrier>& obstacles)
{
  obstacles.clear();

  const auto& registry = dart::simulation::detail::registryOf(world);
  auto view = registry.view<
      comps::RigidBodyTag,
      comps::DeformableSurfaceCcdObstacleTag,
      comps::CollisionGeometry,
      comps::Transform>();

  for (const auto entity : view) {
    if (!isCurrentPoseRigidSurfaceCcdObstacle(registry, entity)) {
      continue;
    }

    const auto& geometry = view.get<comps::CollisionGeometry>(entity);
    const auto* shape = geometry.getPrimaryShape();
    if (shape == nullptr) {
      continue;
    }
    const auto& transform = view.get<comps::Transform>(entity);
    const auto shapeTransform = collisionShapeWorldTransform(transform, *shape);
    if (shape->type != CollisionShapeType::Sphere || !(shape->radius > 0.0)
        || !std::isfinite(shape->radius) || !shapeTransform.has_value()) {
      continue;
    }
    obstacles.push_back(
        SphereObstacleBarrier{shapeTransform->position, shape->radius});
  }
}

//==============================================================================
void collectBoxObstacleBarriersInto(
    const World& world, std::vector<BoxObstacleBarrier>& obstacles)
{
  obstacles.clear();

  const auto& registry = dart::simulation::detail::registryOf(world);
  auto view = registry.view<
      comps::RigidBodyTag,
      comps::DeformableSurfaceCcdObstacleTag,
      comps::CollisionGeometry,
      comps::Transform>();

  for (const auto entity : view) {
    if (!isCurrentPoseRigidSurfaceCcdObstacle(registry, entity)) {
      continue;
    }

    const auto& geometry = view.get<comps::CollisionGeometry>(entity);
    const auto* shape = geometry.getPrimaryShape();
    if (shape == nullptr) {
      continue;
    }
    const auto& transform = view.get<comps::Transform>(entity);
    const auto shapeTransform = collisionShapeWorldTransform(transform, *shape);
    if (shape->type != CollisionShapeType::Box
        || !shape->halfExtents.allFinite()
        || (shape->halfExtents.array() <= 0.0).any()
        || !shapeTransform.has_value()) {
      continue;
    }
    BoxObstacleBarrier obstacle;
    obstacle.center = shapeTransform->position;
    obstacle.rotation
        = normalizeOrIdentity(shapeTransform->orientation).toRotationMatrix();
    obstacle.halfExtents = shape->halfExtents;
    obstacles.push_back(obstacle);
  }
}

//==============================================================================
void collectCapsuleObstacleBarriersInto(
    const World& world, std::vector<CapsuleObstacleBarrier>& obstacles)
{
  obstacles.clear();

  const auto& registry = dart::simulation::detail::registryOf(world);
  auto view = registry.view<
      comps::RigidBodyTag,
      comps::DeformableSurfaceCcdObstacleTag,
      comps::CollisionGeometry,
      comps::Transform>();

  for (const auto entity : view) {
    if (!isCurrentPoseRigidSurfaceCcdObstacle(registry, entity)) {
      continue;
    }

    const auto& geometry = view.get<comps::CollisionGeometry>(entity);
    const auto* shape = geometry.getPrimaryShape();
    if (shape == nullptr) {
      continue;
    }
    const auto& transform = view.get<comps::Transform>(entity);
    const auto shapeTransform = collisionShapeWorldTransform(transform, *shape);
    const double radius = shape->radius;
    const double halfHeight = shape->halfExtents.z();
    if (shape->type != CollisionShapeType::Capsule || !(radius > 0.0)
        || !std::isfinite(radius) || !(halfHeight > 0.0)
        || !std::isfinite(halfHeight) || !shapeTransform.has_value()) {
      continue;
    }
    // The capsule axis is the body z axis; map its two segment endpoints into
    // the world frame.
    const Eigen::Vector3d axis
        = normalizeOrIdentity(shapeTransform->orientation)
              .toRotationMatrix()
              .col(2);
    CapsuleObstacleBarrier obstacle;
    obstacle.pointA = shapeTransform->position - halfHeight * axis;
    obstacle.pointB = shapeTransform->position + halfHeight * axis;
    obstacle.radius = radius;
    obstacles.push_back(obstacle);
  }
}

//==============================================================================
// The supporting static-ground contact under a node: the highest barrier
// surface across all barriers at the node's (x, y), together with that
// surface's geometric normal. Mirrors the max-top selection of the legacy
// height query but also carries the normal for friction's tangent basis.
std::optional<StaticGroundContact> staticGroundContactAt(
    const Eigen::Vector3d& position,
    const std::vector<StaticGroundBarrier>& barriers)
{
  std::optional<StaticGroundContact> best;
  for (const auto& barrier : barriers) {
    std::optional<StaticGroundContact> candidate;
    switch (barrier.shape) {
      case StaticGroundBarrier::Shape::Box:
        candidate = boxContactAt(barrier, position);
        break;
      case StaticGroundBarrier::Shape::Sphere: {
        const Eigen::Vector2d offset
            = position.head<2>() - barrier.center.head<2>();
        const double radiusSquared = barrier.radius * barrier.radius;
        const double planarDistanceSquared = offset.squaredNorm();
        if (planarDistanceSquared <= radiusSquared) {
          const double height
              = std::sqrt(radiusSquared - planarDistanceSquared);
          StaticGroundContact contact;
          contact.top = barrier.center.z() + height;
          // The contact point sits on the sphere directly above the node's
          // (x, y); its outward normal is radial, tilting away from +z toward
          // the rim. |(offset.x, offset.y, height)| == radius by construction.
          const Eigen::Vector3d radial(offset.x(), offset.y(), height);
          const double norm = radial.norm();
          contact.normal = (norm > 1e-12)
                               ? Eigen::Vector3d(radial / norm)
                               : Eigen::Vector3d(Eigen::Vector3d::UnitZ());
          candidate = contact;
        }
        break;
      }
    }

    if (candidate.has_value() && std::isfinite(candidate->top)) {
      if (!best.has_value() || candidate->top > best->top) {
        best = candidate;
      }
    }
  }

  return best;
}

//==============================================================================
std::optional<double> staticGroundTopAt(
    const Eigen::Vector3d& position,
    const std::vector<StaticGroundBarrier>& barriers)
{
  const auto contact = staticGroundContactAt(position, barriers);
  if (!contact.has_value()) {
    return std::nullopt;
  }
  return contact->top;
}

//==============================================================================
double minimumStaticGroundHeight(double groundTop)
{
  constexpr double clearance = 1e-4;
  return groundTop + clearance;
}

//==============================================================================
double staticGroundBarrierCcdClearanceTolerance()
{
  return 1e-12;
}

//==============================================================================
double cross2d(const Eigen::Vector2d& lhs, const Eigen::Vector2d& rhs)
{
  return lhs.x() * rhs.y() - lhs.y() * rhs.x();
}

//==============================================================================
struct TimeInterval
{
  double begin{0.0};
  double end{1.0};
};

//==============================================================================
struct ProjectedBoxFootprint
{
  std::array<Eigen::Vector2d, 16> points{};
  std::size_t size = 0;

  void pushBack(const Eigen::Vector2d& point)
  {
    points[size++] = point;
  }

  void popBack()
  {
    --size;
  }

  [[nodiscard]] Eigen::Vector2d& back()
  {
    return points[size - 1];
  }

  [[nodiscard]] const Eigen::Vector2d& back() const
  {
    return points[size - 1];
  }

  [[nodiscard]] std::span<const Eigen::Vector2d> span() const
  {
    return std::span<const Eigen::Vector2d>(points.data(), size);
  }
};

//==============================================================================
ProjectedBoxFootprint projectedBoxFootprint(const StaticGroundBarrier& barrier)
{
  ProjectedBoxFootprint points;
  for (const double xSign : {-1.0, 1.0}) {
    for (const double ySign : {-1.0, 1.0}) {
      for (const double zSign : {-1.0, 1.0}) {
        const Eigen::Vector3d local(
            xSign * barrier.halfExtents.x(),
            ySign * barrier.halfExtents.y(),
            zSign * barrier.halfExtents.z());
        const Eigen::Vector3d world = barrier.center + barrier.rotation * local;
        points.pushBack(world.head<2>());
      }
    }
  }

  constexpr double tolerance = 1e-12;
  std::sort(
      points.points.begin(),
      points.points.begin() + static_cast<std::ptrdiff_t>(points.size),
      [](const Eigen::Vector2d& lhs, const Eigen::Vector2d& rhs) {
        return std::tie(lhs.x(), lhs.y()) < std::tie(rhs.x(), rhs.y());
      });
  const auto uniqueEnd = std::unique(
      points.points.begin(),
      points.points.begin() + static_cast<std::ptrdiff_t>(points.size),
      [](const Eigen::Vector2d& lhs, const Eigen::Vector2d& rhs) {
        return (lhs - rhs).squaredNorm() <= tolerance * tolerance;
      });
  points.size = static_cast<std::size_t>(
      std::distance(points.points.begin(), uniqueEnd));

  if (points.size <= 2) {
    return points;
  }

  ProjectedBoxFootprint hull;
  for (std::size_t i = 0; i < points.size; ++i) {
    const auto& point = points.points[i];
    while (hull.size >= 2
           && cross2d(
                  hull.back() - hull.points[hull.size - 2], point - hull.back())
                  <= tolerance) {
      hull.popBack();
    }
    hull.pushBack(point);
  }

  const auto lowerSize = hull.size;
  for (std::size_t i = points.size - 2; i < points.size; --i) {
    const auto& point = points.points[i];
    while (hull.size > lowerSize
           && cross2d(
                  hull.back() - hull.points[hull.size - 2], point - hull.back())
                  <= tolerance) {
      hull.popBack();
    }
    hull.pushBack(point);
    if (i == 0) {
      break;
    }
  }
  if (hull.size > 0) {
    hull.popBack();
  }

  return hull;
}

//==============================================================================
std::optional<TimeInterval> clipSegmentToConvexFootprint(
    const Eigen::Vector2d& start,
    const Eigen::Vector2d& end,
    std::span<const Eigen::Vector2d> footprint)
{
  if (footprint.size() < 3) {
    return std::nullopt;
  }

  constexpr double tolerance = 1e-12;
  const Eigen::Vector2d direction = end - start;
  double intervalBegin = 0.0;
  double intervalEnd = 1.0;

  for (std::size_t i = 0; i < footprint.size(); ++i) {
    const Eigen::Vector2d& a = footprint[i];
    const Eigen::Vector2d& b = footprint[(i + 1) % footprint.size()];
    const Eigen::Vector2d edge = b - a;
    const double valueAtStart = cross2d(edge, start - a);
    const double slope = cross2d(edge, direction);

    if (std::abs(slope) <= tolerance) {
      if (valueAtStart < -tolerance) {
        return std::nullopt;
      }
      continue;
    }

    const double boundaryT = (-tolerance - valueAtStart) / slope;
    if (slope > 0.0) {
      intervalBegin = std::max(intervalBegin, boundaryT);
    } else {
      intervalEnd = std::min(intervalEnd, boundaryT);
    }

    if (intervalBegin > intervalEnd) {
      return std::nullopt;
    }
  }

  return TimeInterval{
      std::clamp(intervalBegin, 0.0, 1.0), std::clamp(intervalEnd, 0.0, 1.0)};
}

//==============================================================================
std::optional<TimeInterval> sphereFootprintInterval(
    const StaticGroundBarrier& barrier,
    const Eigen::Vector3d& start,
    const Eigen::Vector3d& end)
{
  const Eigen::Vector2d origin = start.head<2>() - barrier.center.head<2>();
  const Eigen::Vector2d direction = end.head<2>() - start.head<2>();
  const double a = direction.squaredNorm();
  const double b = 2.0 * origin.dot(direction);
  const double c = origin.squaredNorm() - barrier.radius * barrier.radius;
  constexpr double tolerance = 1e-14;

  if (a <= tolerance) {
    if (c <= tolerance) {
      return TimeInterval{0.0, 1.0};
    }
    return std::nullopt;
  }

  const double discriminant = b * b - 4.0 * a * c;
  if (discriminant < -tolerance) {
    return std::nullopt;
  }

  const double root = std::sqrt(std::max(0.0, discriminant));
  double intervalBegin = (-b - root) / (2.0 * a);
  double intervalEnd = (-b + root) / (2.0 * a);
  if (intervalBegin > intervalEnd) {
    std::swap(intervalBegin, intervalEnd);
  }

  intervalBegin = std::max(intervalBegin, 0.0);
  intervalEnd = std::min(intervalEnd, 1.0);
  if (intervalBegin > intervalEnd) {
    return std::nullopt;
  }

  return TimeInterval{intervalBegin, intervalEnd};
}

//==============================================================================
std::optional<TimeInterval> staticGroundBarrierFootprintInterval(
    const StaticGroundBarrier& barrier,
    const Eigen::Vector3d& start,
    const Eigen::Vector3d& end)
{
  switch (barrier.shape) {
    case StaticGroundBarrier::Shape::Box: {
      const auto footprint = projectedBoxFootprint(barrier);
      return clipSegmentToConvexFootprint(
          start.head<2>(), end.head<2>(), footprint.span());
    }
    case StaticGroundBarrier::Shape::Sphere:
      return sphereFootprintInterval(barrier, start, end);
  }

  return std::nullopt;
}

//==============================================================================
std::optional<double> staticGroundClearanceAt(
    const Eigen::Vector3d& position,
    const std::vector<StaticGroundBarrier>& barriers,
    DeformableSolverStats& stats)
{
  ++stats.staticGroundBarrierCcdSampleChecks;
  const auto groundTop = staticGroundTopAt(position, barriers);
  if (!groundTop.has_value()) {
    return std::nullopt;
  }
  return position.z() - minimumStaticGroundHeight(*groundTop);
}

//==============================================================================
Eigen::Vector3d interpolatePoint(
    const Eigen::Vector3d& start, const Eigen::Vector3d& end, double t)
{
  return start + t * (end - start);
}

//==============================================================================
bool isStaticGroundBarrierCcdHit(
    const Eigen::Vector3d& start,
    const Eigen::Vector3d& end,
    double t,
    const std::vector<StaticGroundBarrier>& barriers,
    DeformableSolverStats& stats)
{
  const auto clearance = staticGroundClearanceAt(
      interpolatePoint(start, end, t), barriers, stats);
  return clearance.has_value()
         && *clearance < -staticGroundBarrierCcdClearanceTolerance();
}

//==============================================================================
std::optional<double> verticalStaticGroundBarrierStepBound(
    const Eigen::Vector3d& start,
    const Eigen::Vector3d& end,
    const std::vector<StaticGroundBarrier>& barriers,
    DeformableSolverStats& stats)
{
  const auto startClearance = staticGroundClearanceAt(start, barriers, stats);
  const auto endClearance = staticGroundClearanceAt(end, barriers, stats);
  const double tolerance = staticGroundBarrierCcdClearanceTolerance();
  if (startClearance.has_value() && *startClearance < -tolerance) {
    return 0.0;
  }
  if (!endClearance.has_value() || *endClearance >= -tolerance) {
    return std::nullopt;
  }
  if (!startClearance.has_value()) {
    return std::nullopt;
  }

  const double denominator = *startClearance - *endClearance;
  if (!(denominator > 0.0) || !std::isfinite(denominator)) {
    return std::nullopt;
  }

  return std::clamp(*startClearance / denominator, 0.0, 1.0);
}

//==============================================================================
std::optional<double> firstStaticGroundBarrierHitInInterval(
    const Eigen::Vector3d& start,
    const Eigen::Vector3d& end,
    const TimeInterval& interval,
    const std::vector<StaticGroundBarrier>& barriers,
    DeformableSolverStats& stats)
{
  const double tolerance = staticGroundBarrierCcdClearanceTolerance();
  if (isStaticGroundBarrierCcdHit(
          start, end, interval.begin, barriers, stats)) {
    return interval.begin;
  }
  if (interval.end <= interval.begin) {
    return std::nullopt;
  }

  constexpr int minimizationIterations = 48;
  double lo = interval.begin;
  double hi = interval.end;
  const auto clearanceOrInfinity = [&](double t) {
    const auto clearance = staticGroundClearanceAt(
        interpolatePoint(start, end, t), barriers, stats);
    return clearance.has_value() ? *clearance
                                 : std::numeric_limits<double>::infinity();
  };

  for (int iteration = 0; iteration < minimizationIterations; ++iteration) {
    const double third = (hi - lo) / 3.0;
    const double midA = lo + third;
    const double midB = hi - third;
    if (clearanceOrInfinity(midA) < clearanceOrInfinity(midB)) {
      hi = midB;
    } else {
      lo = midA;
    }
  }

  double hitT = 0.5 * (lo + hi);
  const auto minClearance = staticGroundClearanceAt(
      interpolatePoint(start, end, hitT), barriers, stats);
  if (!minClearance.has_value() || *minClearance >= -tolerance) {
    return std::nullopt;
  }

  constexpr int bisectionIterations = 32;
  lo = interval.begin;
  hi = hitT;
  for (int iteration = 0; iteration < bisectionIterations; ++iteration) {
    const double mid = 0.5 * (lo + hi);
    if (isStaticGroundBarrierCcdHit(start, end, mid, barriers, stats)) {
      hi = mid;
    } else {
      lo = mid;
    }
  }

  return std::clamp(hi, 0.0, 1.0);
}

//==============================================================================
std::optional<double> continuousStaticGroundBarrierStepBound(
    const Eigen::Vector3d& start,
    const Eigen::Vector3d& end,
    const std::vector<StaticGroundBarrier>& barriers,
    DeformableSolverStats& stats)
{
  if (isStaticGroundBarrierCcdHit(start, end, 0.0, barriers, stats)) {
    return 0.0;
  }

  std::optional<double> stepBound;
  for (const auto& barrier : barriers) {
    const auto interval
        = staticGroundBarrierFootprintInterval(barrier, start, end);
    if (!interval.has_value()) {
      continue;
    }

    const auto hit = firstStaticGroundBarrierHitInInterval(
        start, end, *interval, barriers, stats);
    if (hit.has_value()) {
      stepBound = stepBound.has_value() ? std::min(*stepBound, *hit) : hit;
    }
  }

  return stepBound;
}

//==============================================================================
std::optional<double> staticGroundBarrierStepBound(
    const Eigen::Vector3d& start,
    const Eigen::Vector3d& end,
    const std::vector<StaticGroundBarrier>& barriers,
    DeformableSolverStats& stats)
{
  constexpr double planarTolerance = 1e-14;
  if ((end.head<2>() - start.head<2>()).squaredNorm()
      <= planarTolerance * planarTolerance) {
    return verticalStaticGroundBarrierStepBound(start, end, barriers, stats);
  }
  return continuousStaticGroundBarrierStepBound(start, end, barriers, stats);
}

//==============================================================================
double staticGroundBarrierActivationDistance()
{
  return 2e-2;
}

//==============================================================================
// The default (fixed) clamped-log barrier stiffness kappa shared by the ground,
// obstacle, and self-contact barriers.
constexpr double kDefaultBarrierStiffness = 25.0;

//==============================================================================
// IPC-style adaptive barrier stiffness (kappa) for a body, opted in per body.
//
// The clamped-log barrier's curvature contribution to the Hessian scales like
// kappa / d_hat^2, while a node's inertial stiffness is mass / dt^2. Balancing
// the two (so the barrier resolves contact in few iterations regardless of the
// mass/stiffness ratio) gives kappa ~ (mass / dt^2) * d_hat^2. For a unit nodal
// mass at dt = 1/250 and d_hat = 2e-2 this evaluates to exactly the historical
// fixed kappa = 25, so the fixed value is just this balance at unit mass; the
// adaptive form generalizes it, stiffening the barrier for heavier/faster
// bodies. The result is floored at the fixed default (never softer, so contact
// robustness never regresses) and capped to avoid an ill-conditioned Hessian.
double adaptiveBarrierStiffness(
    double maxNodalMass, double timeStep, double activationDistance)
{
  if (!(maxNodalMass > 0.0) || !(timeStep > 0.0) || !std::isfinite(maxNodalMass)
      || !std::isfinite(timeStep)) {
    return kDefaultBarrierStiffness;
  }
  constexpr double kMaxBarrierStiffness = 1.0e6;
  const double balanced = (maxNodalMass / (timeStep * timeStep))
                          * activationDistance * activationDistance;
  return std::clamp(balanced, kDefaultBarrierStiffness, kMaxBarrierStiffness);
}

//==============================================================================
bool satisfiesStaticGroundBarrier(
    const std::vector<Eigen::Vector3d>& positions,
    const std::vector<std::uint8_t>& fixed,
    const std::vector<StaticGroundBarrier>& barriers)
{
  if (barriers.empty()) {
    return true;
  }

  for (std::size_t i = 0; i < positions.size(); ++i) {
    const auto groundTop = staticGroundTopAt(positions[i], barriers);
    if (fixed[i] == 0u && groundTop.has_value()
        && positions[i].z() < minimumStaticGroundHeight(*groundTop)) {
      return false;
    }
  }
  return true;
}

//==============================================================================
void makeInitialPositionsFeasible(
    std::vector<Eigen::Vector3d>& positions,
    const std::vector<std::uint8_t>& fixed,
    const std::vector<StaticGroundBarrier>& barriers,
    DeformableSolverStats* stats)
{
  if (barriers.empty()) {
    return;
  }

  for (std::size_t i = 0; i < positions.size(); ++i) {
    if (fixed[i] == 0u) {
      const auto groundTop = staticGroundTopAt(positions[i], barriers);
      if (groundTop.has_value()) {
        const double previousZ = positions[i].z();
        positions[i].z()
            = std::max(positions[i].z(), minimumStaticGroundHeight(*groundTop));
        if (stats != nullptr && positions[i].z() != previousZ) {
          ++stats->initialProjectionCount;
        }
      }
    }
  }
}

//==============================================================================
double addStaticGroundBarrierEnergy(
    const std::vector<Eigen::Vector3d>& positions,
    const std::vector<std::uint8_t>& fixed,
    const std::vector<StaticGroundBarrier>& barriers,
    std::vector<Eigen::Vector3d>* gradient,
    double barrierStiffness = kDefaultBarrierStiffness)
{
  if (barriers.empty()) {
    return 0.0;
  }

  const double activationDistance = staticGroundBarrierActivationDistance();
  const double barrierScale = barrierStiffness;

  double energy = 0.0;
  for (std::size_t i = 0; i < positions.size(); ++i) {
    if (fixed[i] != 0u) {
      continue;
    }

    const auto groundTop = staticGroundTopAt(positions[i], barriers);
    if (!groundTop.has_value()) {
      continue;
    }

    const double distance = positions[i].z() - *groundTop;
    if (distance <= 0.0 || !std::isfinite(distance)) {
      return std::numeric_limits<double>::infinity();
    }
    if (distance >= activationDistance) {
      continue;
    }

    const double normalizedDistance = distance / activationDistance;
    const double distanceOffset = distance - activationDistance;
    energy += -barrierScale * distanceOffset * distanceOffset
              * std::log(normalizedDistance);

    if (gradient != nullptr) {
      const double derivative
          = -barrierScale
            * (2.0 * distanceOffset * std::log(normalizedDistance)
               + distanceOffset * distanceOffset / distance);
      (*gradient)[i].z() += derivative;
    }
  }

  return energy;
}

//==============================================================================
// Full radial clamped-log barrier for static sphere obstacles: each free node
// within the activation band of a sphere's surface is pushed out along the
// outward radial normal, so a deformable settles smoothly at ~d_hat against any
// side of the sphere (a true 3D contact force, unlike the vertical-only ground
// barrier). Energy + gradient only -- the projected-Newton Hessian, box and
// codimensional obstacles are later increments; the line search on the
// barrier-inclusive energy keeps nodes outside because the clamped-log energy
// diverges as the distance approaches zero.
double addSphereObstacleBarrierEnergy(
    const std::vector<Eigen::Vector3d>& positions,
    const std::vector<std::uint8_t>& fixed,
    const std::vector<SphereObstacleBarrier>& obstacles,
    std::vector<Eigen::Vector3d>* gradient,
    double barrierStiffness = kDefaultBarrierStiffness)
{
  if (obstacles.empty()) {
    return 0.0;
  }

  const double activationDistance = staticGroundBarrierActivationDistance();
  const double barrierScale = barrierStiffness;

  double energy = 0.0;
  for (std::size_t i = 0; i < positions.size(); ++i) {
    if (fixed[i] != 0u) {
      continue;
    }

    for (const auto& obstacle : obstacles) {
      const Eigen::Vector3d offset = positions[i] - obstacle.center;
      const double centerDistance = offset.norm();
      const double distance = centerDistance - obstacle.radius;
      if (distance <= 0.0 || !std::isfinite(distance)) {
        return std::numeric_limits<double>::infinity();
      }
      if (distance >= activationDistance || centerDistance <= 0.0) {
        continue;
      }

      const double normalizedDistance = distance / activationDistance;
      const double distanceOffset = distance - activationDistance;
      energy += -barrierScale * distanceOffset * distanceOffset
                * std::log(normalizedDistance);

      if (gradient != nullptr) {
        const double derivative
            = -barrierScale
              * (2.0 * distanceOffset * std::log(normalizedDistance)
                 + distanceOffset * distanceOffset / distance);
        // dDistance/dx is the outward radial unit normal.
        (*gradient)[i] += derivative * (offset / centerDistance);
      }
    }
  }

  return energy;
}

//==============================================================================
// Signed closest-surface distance from a node to an oriented box obstacle, with
// the outward world-frame surface normal set for outside, on-surface, and
// inside nodes. Outside the box, the node is clamped into the box's local
// frame; |local - clamp(local)| is the distance to the surface, uniform across
// face, edge, and corner contact. Inside the box, the nearest exit face
// provides the outward normal and the returned distance is negative penetration
// depth.
double boxObstacleSurfaceDistance(
    const Eigen::Vector3d& position,
    const BoxObstacleBarrier& obstacle,
    Eigen::Vector3d& outwardNormal)
{
  const Eigen::Vector3d local
      = obstacle.rotation.transpose() * (position - obstacle.center);
  const Eigen::Vector3d clamped
      = local.cwiseMax(-obstacle.halfExtents).cwiseMin(obstacle.halfExtents);
  const Eigen::Vector3d delta = local - clamped;
  const double distance = delta.norm();
  if (distance > 0.0 && std::isfinite(distance)) {
    outwardNormal = obstacle.rotation * (delta / distance);
    return distance;
  }

  Eigen::Index nearestAxis = 0;
  double nearestMargin = std::numeric_limits<double>::infinity();
  for (Eigen::Index axis = 0; axis < 3; ++axis) {
    const double margin = obstacle.halfExtents[axis] - std::abs(local[axis]);
    if (margin < nearestMargin) {
      nearestMargin = margin;
      nearestAxis = axis;
    }
  }
  Eigen::Vector3d localNormal = Eigen::Vector3d::Zero();
  localNormal[nearestAxis] = local[nearestAxis] >= 0.0 ? 1.0 : -1.0;
  outwardNormal = obstacle.rotation * localNormal;
  return -nearestMargin;
}

//==============================================================================
// Closest-surface distance from a node to a capsule obstacle, with the outward
// radial surface normal set when positive. The distance is the point-to-segment
// axis distance minus the radius; the normal points from the closest axis point
// to the node.
double capsuleObstacleSurfaceDistance(
    const Eigen::Vector3d& position,
    const CapsuleObstacleBarrier& obstacle,
    Eigen::Vector3d& outwardNormal)
{
  const Eigen::Vector3d axis = obstacle.pointB - obstacle.pointA;
  const double axisLengthSq = axis.squaredNorm();
  double t = 0.0;
  if (axisLengthSq > 0.0) {
    t = (position - obstacle.pointA).dot(axis) / axisLengthSq;
    t = std::clamp(t, 0.0, 1.0);
  }
  const Eigen::Vector3d closest = obstacle.pointA + t * axis;
  const Eigen::Vector3d delta = position - closest;
  const double axisDistance = delta.norm();
  if (axisDistance > 0.0 && std::isfinite(axisDistance)) {
    outwardNormal = delta / axisDistance;
  }
  return axisDistance - obstacle.radius;
}

//==============================================================================
// Adds the clamped-log barrier energy/gradient pushing free deformable nodes
// out of the activation band of a capsule obstacle, along the outward radial
// normal. The capsule analogue of addBoxObstacleBarrierEnergy.
double addCapsuleObstacleBarrierEnergy(
    const std::vector<Eigen::Vector3d>& positions,
    const std::vector<std::uint8_t>& fixed,
    const std::vector<CapsuleObstacleBarrier>& obstacles,
    std::vector<Eigen::Vector3d>* gradient,
    double barrierStiffness = kDefaultBarrierStiffness)
{
  if (obstacles.empty()) {
    return 0.0;
  }

  const double activationDistance = staticGroundBarrierActivationDistance();
  const double barrierScale = barrierStiffness;

  double energy = 0.0;
  for (std::size_t i = 0; i < positions.size(); ++i) {
    if (fixed[i] != 0u) {
      continue;
    }

    for (const auto& obstacle : obstacles) {
      Eigen::Vector3d normal;
      const double distance
          = capsuleObstacleSurfaceDistance(positions[i], obstacle, normal);
      if (distance <= 0.0 || !std::isfinite(distance)) {
        // On or inside the capsule: a penetration the barrier forbids.
        return std::numeric_limits<double>::infinity();
      }
      if (distance >= activationDistance) {
        continue;
      }

      const double normalizedDistance = distance / activationDistance;
      const double distanceOffset = distance - activationDistance;
      energy += -barrierScale * distanceOffset * distanceOffset
                * std::log(normalizedDistance);

      if (gradient != nullptr) {
        const double derivative
            = -barrierScale
              * (2.0 * distanceOffset * std::log(normalizedDistance)
                 + distanceOffset * distanceOffset / distance);
        (*gradient)[i] += derivative * normal;
      }
    }
  }

  return energy;
}

//==============================================================================
// Adds the clamped-log barrier energy/gradient pushing free deformable nodes
// out of the activation band of an oriented box obstacle, along the outward
// surface normal. The box analogue of addSphereObstacleBarrierEnergy.
double addBoxObstacleBarrierEnergy(
    const std::vector<Eigen::Vector3d>& positions,
    const std::vector<std::uint8_t>& fixed,
    const std::vector<BoxObstacleBarrier>& obstacles,
    std::vector<Eigen::Vector3d>* gradient,
    double barrierStiffness = kDefaultBarrierStiffness)
{
  if (obstacles.empty()) {
    return 0.0;
  }

  const double activationDistance = staticGroundBarrierActivationDistance();
  const double barrierScale = barrierStiffness;

  double energy = 0.0;
  for (std::size_t i = 0; i < positions.size(); ++i) {
    if (fixed[i] != 0u) {
      continue;
    }

    for (const auto& obstacle : obstacles) {
      Eigen::Vector3d normal;
      const double distance
          = boxObstacleSurfaceDistance(positions[i], obstacle, normal);
      if (distance <= 0.0 || !std::isfinite(distance)) {
        // On or inside the box: a penetration the barrier forbids.
        return std::numeric_limits<double>::infinity();
      }
      if (distance >= activationDistance) {
        continue;
      }

      const double normalizedDistance = distance / activationDistance;
      const double distanceOffset = distance - activationDistance;
      energy += -barrierScale * distanceOffset * distanceOffset
                * std::log(normalizedDistance);

      if (gradient != nullptr) {
        const double derivative
            = -barrierScale
              * (2.0 * distanceOffset * std::log(normalizedDistance)
                 + distanceOffset * distanceOffset / distance);
        (*gradient)[i] += derivative * normal;
      }
    }
  }

  return energy;
}

//==============================================================================
// Tangential speed below which static-ground friction smoothly vanishes (the
// IPC mollifier velocity threshold epsv). Scaled by the time step to a
// displacement radius.
double staticGroundFrictionVelocityThreshold()
{
  return 1e-3;
}

//==============================================================================
// Compute the lagged static-ground normal force magnitude per node at the
// current iterate: the upward barrier force |dB/dz| for nodes inside the
// activation band, zero otherwise. Friction lags this across the inner line
// search (standard IPC), so it is evaluated once per outer iteration.
void computeStaticGroundNormalForces(
    const std::vector<Eigen::Vector3d>& positions,
    const std::vector<std::uint8_t>& fixed,
    const std::vector<StaticGroundBarrier>& barriers,
    std::vector<double>& normalForce,
    std::vector<Eigen::Vector3d>& normalDirection)
{
  normalForce.assign(positions.size(), 0.0);
  normalDirection.assign(positions.size(), Eigen::Vector3d::UnitZ());
  if (barriers.empty()) {
    return;
  }

  const double activationDistance = staticGroundBarrierActivationDistance();
  // The lagged friction normal-force estimate uses the base barrier stiffness
  // (adaptive kappa scales the contact barriers, not this approximate force).
  const double barrierScale = kDefaultBarrierStiffness;
  for (std::size_t i = 0; i < positions.size(); ++i) {
    if (fixed[i] != 0u) {
      continue;
    }
    const auto contact = staticGroundContactAt(positions[i], barriers);
    if (!contact.has_value()) {
      continue;
    }
    const double distance = positions[i].z() - contact->top;
    if (distance <= 0.0 || distance >= activationDistance
        || !std::isfinite(distance)) {
      continue;
    }
    const double distanceOffset = distance - activationDistance;
    const double normalizedDistance = distance / activationDistance;
    const double derivative
        = -barrierScale
          * (2.0 * distanceOffset * std::log(normalizedDistance)
             + distanceOffset * distanceOffset / distance);
    // derivative = dB/dz < 0 (repulsive); the upward normal force is its
    // magnitude. The barrier acts vertically (height field); the geometric
    // surface normal only shapes friction's tangent plane below.
    normalForce[i] = -derivative;
    normalDirection[i] = contact->normal;
  }
}

//==============================================================================
// Merges the capsule obstacle barrier's per-node radial normal force and
// direction into the friction normal-force arrays (populated first by
// computeStaticGroundNormalForces). The dominant (largest-force) contact per
// node wins, so a node resting on a capsule rod gets the rod's radial normal
// for its friction tangent plane. The capsule obstacle is barrier-only (no
// surface CCD), so tangential sliding is unconstrained and friction is
// effective.
void addCapsuleObstacleNormalForces(
    const std::vector<Eigen::Vector3d>& positions,
    const std::vector<std::uint8_t>& fixed,
    const std::vector<CapsuleObstacleBarrier>& obstacles,
    std::vector<double>& normalForce,
    std::vector<Eigen::Vector3d>& normalDirection)
{
  if (obstacles.empty()) {
    return;
  }
  const double activationDistance = staticGroundBarrierActivationDistance();
  const double barrierScale = kDefaultBarrierStiffness;
  for (std::size_t i = 0; i < positions.size(); ++i) {
    if (fixed[i] != 0u) {
      continue;
    }
    for (const auto& obstacle : obstacles) {
      Eigen::Vector3d normal;
      const double distance
          = capsuleObstacleSurfaceDistance(positions[i], obstacle, normal);
      if (distance <= 0.0 || distance >= activationDistance
          || !std::isfinite(distance)) {
        continue;
      }
      const double distanceOffset = distance - activationDistance;
      const double normalizedDistance = distance / activationDistance;
      const double derivative
          = -barrierScale
            * (2.0 * distanceOffset * std::log(normalizedDistance)
               + distanceOffset * distanceOffset / distance);
      const double force = -derivative;
      if (force > normalForce[i]) {
        normalForce[i] = force;
        normalDirection[i] = normal;
      }
    }
  }
}

//==============================================================================
// Merges the sphere obstacle barrier's per-node radial normal force/direction
// into the friction normal-force arrays (dominant contact per node wins). Used
// for friction against barrier-only sphere obstacles.
void addSphereObstacleNormalForces(
    const std::vector<Eigen::Vector3d>& positions,
    const std::vector<std::uint8_t>& fixed,
    const std::vector<SphereObstacleBarrier>& obstacles,
    std::vector<double>& normalForce,
    std::vector<Eigen::Vector3d>& normalDirection)
{
  if (obstacles.empty()) {
    return;
  }
  const double activationDistance = staticGroundBarrierActivationDistance();
  const double barrierScale = kDefaultBarrierStiffness;
  for (std::size_t i = 0; i < positions.size(); ++i) {
    if (fixed[i] != 0u) {
      continue;
    }
    for (const auto& obstacle : obstacles) {
      const Eigen::Vector3d offset = positions[i] - obstacle.center;
      const double centerDistance = offset.norm();
      const double distance = centerDistance - obstacle.radius;
      if (distance <= 0.0 || distance >= activationDistance
          || centerDistance <= 0.0 || !std::isfinite(distance)) {
        continue;
      }
      const double distanceOffset = distance - activationDistance;
      const double normalizedDistance = distance / activationDistance;
      const double derivative
          = -barrierScale
            * (2.0 * distanceOffset * std::log(normalizedDistance)
               + distanceOffset * distanceOffset / distance);
      const double force = -derivative;
      if (force > normalForce[i]) {
        normalForce[i] = force;
        normalDirection[i] = offset / centerDistance;
      }
    }
  }
}

//==============================================================================
// Merges the box obstacle barrier's per-node surface normal force/direction
// into the friction normal-force arrays (dominant contact per node wins). Used
// for friction against barrier-only box obstacles.
void addBoxObstacleNormalForces(
    const std::vector<Eigen::Vector3d>& positions,
    const std::vector<std::uint8_t>& fixed,
    const std::vector<BoxObstacleBarrier>& obstacles,
    std::vector<double>& normalForce,
    std::vector<Eigen::Vector3d>& normalDirection)
{
  if (obstacles.empty()) {
    return;
  }
  const double activationDistance = staticGroundBarrierActivationDistance();
  const double barrierScale = kDefaultBarrierStiffness;
  for (std::size_t i = 0; i < positions.size(); ++i) {
    if (fixed[i] != 0u) {
      continue;
    }
    for (const auto& obstacle : obstacles) {
      Eigen::Vector3d normal;
      const double distance
          = boxObstacleSurfaceDistance(positions[i], obstacle, normal);
      if (distance <= 0.0 || distance >= activationDistance
          || !std::isfinite(distance)) {
        continue;
      }
      const double distanceOffset = distance - activationDistance;
      const double normalizedDistance = distance / activationDistance;
      const double derivative
          = -barrierScale
            * (2.0 * distanceOffset * std::log(normalizedDistance)
               + distanceOffset * distanceOffset / distance);
      const double force = -derivative;
      if (force > normalForce[i]) {
        normalForce[i] = force;
        normalDirection[i] = normal;
      }
    }
  }
}

//==============================================================================
// Lagged smoothed Coulomb friction inputs for static-ground contact. The
// per-node normal force and the step-start positions are fixed across the inner
// line search; the tangential displacement uses the candidate positions.
struct GroundFrictionInputs
{
  double coefficient = 0.0; // mu
  double epsilon = 0.0;     // epsv * timeStep (mollifier displacement radius)
  const std::vector<Eigen::Vector3d>* stepStartPositions = nullptr;
  const std::vector<double>* laggedNormalForce = nullptr;
  // Per-node geometric ground normal at the lagged contact (unit, upward). When
  // null the tangent plane defaults to xy (flat ground), preserving the legacy
  // behavior exactly.
  const std::vector<Eigen::Vector3d>* laggedNormalDirection = nullptr;
};

//==============================================================================
// IPC smoothed-friction mollifier (Li et al. 2020): f0 is the friction
// potential profile and f1 = f0' the force profile. Both are C1 with f1 -> 1
// (kinetic) for tangential displacement beyond the threshold and a smooth ramp
// to zero below it.
double frictionF0(double y, double epsilon)
{
  if (y >= epsilon) {
    return y;
  }
  return y * y / epsilon - y * y * y / (3.0 * epsilon * epsilon)
         + epsilon / 3.0;
}

double frictionF1(double y, double epsilon)
{
  if (y >= epsilon) {
    return 1.0;
  }
  return 2.0 * y / epsilon - y * y / (epsilon * epsilon);
}

//==============================================================================
// Add the lagged smoothed Coulomb friction energy/gradient for static-ground
// contact. The tangent plane is the plane orthogonal to the lagged geometric
// ground normal n: the friction opposes the node's step displacement projected
// into that plane, u_T = (I - n n^T) (x - x_start). For flat/box-top ground
// n = +z and u_T is the xy displacement, recovering the legacy behavior; a
// sphere or tilted box gives a tilted tangent plane. The force magnitude
// saturates at mu * normalForce (kinetic) and ramps smoothly to zero at rest,
// so there is no division by zero.
double addGroundFrictionEnergy(
    const std::vector<Eigen::Vector3d>& positions,
    const std::vector<std::uint8_t>& fixed,
    const GroundFrictionInputs& friction,
    std::vector<Eigen::Vector3d>* gradient)
{
  if (friction.coefficient <= 0.0 || friction.epsilon <= 0.0
      || friction.stepStartPositions == nullptr
      || friction.laggedNormalForce == nullptr) {
    return 0.0;
  }

  const auto& start = *friction.stepStartPositions;
  const auto& normalForce = *friction.laggedNormalForce;
  const auto* normalDirection = friction.laggedNormalDirection;
  double energy = 0.0;
  for (std::size_t i = 0; i < positions.size(); ++i) {
    if (fixed[i] != 0u || normalForce[i] <= 0.0) {
      continue;
    }
    const Eigen::Vector3d n = (normalDirection != nullptr)
                                  ? (*normalDirection)[i]
                                  : Eigen::Vector3d::UnitZ();
    const Eigen::Vector3d u = positions[i] - start[i];
    const Eigen::Vector3d tangent = u - n.dot(u) * n;
    const double y = tangent.norm();
    const double scale = friction.coefficient * normalForce[i];
    energy += scale * frictionF0(y, friction.epsilon);
    if (gradient != nullptr) {
      // grad = mu * normalForce * f1(y) * u_T / ||u_T||, which lies in the
      // tangent plane. As y -> 0, f1(y)/y -> 2/epsilon, so the force vanishes
      // smoothly at rest.
      constexpr double tiny = 1e-12;
      const double ratio = (y > tiny) ? frictionF1(y, friction.epsilon) / y
                                      : 2.0 / friction.epsilon;
      (*gradient)[i] += scale * ratio * tangent;
    }
  }
  return energy;
}

//==============================================================================
// Activation distance d_hat for the self-contact barrier. The barrier is active
// only when a contact pair's distance is below this value.
double selfContactBarrierActivationDistance()
{
  return 2e-2;
}

// Fixed barrier stiffness (kappa). Adaptive stiffness is a later slice.
double selfContactBarrierStiffness()
{
  return 1e5;
}

// Inputs for the IPC clamped-log self-contact barrier energy term. Null/zero
// fields disable the term, preserving the contact-free objective exactly.
struct SelfContactBarrierInputs
{
  const dc::ContactCandidateSet* candidates = nullptr;
  const std::vector<DeformableSurfaceTriangle>* triangles = nullptr;
  double squaredActivationDistance = 0.0;
  double stiffness = 0.0;
};

// Adds the IPC clamped-log barrier energy/gradient over the active self-contact
// candidate set (point-triangle and edge-edge). The barrier kernel already
// returns position-space derivatives (a 12-vector over the four primitive
// points), so each contact's gradient is scattered into its four nodes. This
// produces smooth repulsive contact forces; the CCD limiters remain the hard
// no-penetration guarantee.
double addSelfContactBarrierEnergy(
    const std::vector<Eigen::Vector3d>& positions,
    const std::vector<std::uint8_t>& fixed,
    const SelfContactBarrierInputs& inputs,
    std::size_t* activeContacts,
    std::vector<Eigen::Vector3d>* gradient)
{
  if (inputs.candidates == nullptr || inputs.triangles == nullptr
      || inputs.stiffness <= 0.0 || !(inputs.squaredActivationDistance > 0.0)) {
    return 0.0;
  }

  const auto& candidates = *inputs.candidates;
  const auto& triangles = *inputs.triangles;
  double energy = 0.0;

  const auto scatter = [&](const dc::PrimitiveBarrierResult& result,
                           const std::array<std::size_t, 4>& nodes) {
    energy += result.value;
    if (activeContacts != nullptr) {
      ++(*activeContacts);
    }
    if (gradient == nullptr) {
      return;
    }
    for (int k = 0; k < 4; ++k) {
      if (fixed[nodes[k]] != 0u) {
        continue;
      }
      (*gradient)[nodes[k]] += result.gradient.segment<3>(3 * k);
    }
  };

  for (const auto& candidate : candidates.pointTriangleCandidates) {
    const auto& triangle = triangles[candidate.triangle];
    const auto result = dc::pointTriangleBarrier(
        positions[candidate.point],
        positions[triangle.nodeA],
        positions[triangle.nodeB],
        positions[triangle.nodeC],
        inputs.squaredActivationDistance,
        inputs.stiffness);
    if (!result.active) {
      continue;
    }
    scatter(
        result,
        {candidate.point, triangle.nodeA, triangle.nodeB, triangle.nodeC});
  }

  // Edge-edge uses the plain (non-mollified) barrier. The IPC edge-edge
  // mollifier that smooths nearly-parallel configurations is intentionally
  // deferred: it pairs with the projected-Newton slice that also needs the
  // mollified Hessian. The plain barrier is finite and safe here; the CCD
  // limiter remains the hard no-penetration gate.
  for (const auto& candidate : candidates.edgeEdgeCandidates) {
    const auto& edgeA = candidates.surfaceEdges[candidate.edgeA];
    const auto& edgeB = candidates.surfaceEdges[candidate.edgeB];
    const auto result = dc::edgeEdgeBarrier(
        positions[edgeA.nodeA],
        positions[edgeA.nodeB],
        positions[edgeB.nodeA],
        positions[edgeB.nodeB],
        inputs.squaredActivationDistance,
        inputs.stiffness);
    if (!result.active) {
      continue;
    }
    scatter(result, {edgeA.nodeA, edgeA.nodeB, edgeB.nodeA, edgeB.nodeB});
  }

  return energy;
}

struct SelfContactFrictionInputs
{
  double coefficient = 0.0; // mu
  double epsilon = 0.0;     // epsv * timeStep
  const std::vector<Eigen::Vector3d>* stepStartPositions = nullptr;
  const std::vector<SelfContactFrictionContact>* contacts = nullptr;
};

// Assemble the lagged self-contact friction set from the active point-triangle
// and edge-edge barrier candidates at the current iterate. The lagged normal
// force is the barrier force magnitude on the primitive's first feature (the
// point node, or the net force on the first edge), and the tangent projection
// comes from the matching point-triangle / edge-edge tangent stencil. The
// downstream friction energy/gradient/Hessian are generic over the four-node
// stencil, so both contact types share them.
void buildSelfContactFrictionContacts(
    const std::vector<Eigen::Vector3d>& positions,
    const SelfContactBarrierInputs& barrier,
    std::vector<SelfContactFrictionContact>& contacts)
{
  contacts.clear();
  if (barrier.candidates == nullptr || barrier.triangles == nullptr
      || barrier.stiffness <= 0.0
      || !(barrier.squaredActivationDistance > 0.0)) {
    return;
  }

  const auto& candidates = *barrier.candidates;
  const auto& triangles = *barrier.triangles;
  for (const auto& candidate : candidates.pointTriangleCandidates) {
    const auto& triangle = triangles[candidate.triangle];
    const auto& p = positions[candidate.point];
    const auto& a = positions[triangle.nodeA];
    const auto& b = positions[triangle.nodeB];
    const auto& c = positions[triangle.nodeC];
    const auto result = dc::pointTriangleBarrier(
        p, a, b, c, barrier.squaredActivationDistance, barrier.stiffness);
    if (!result.active) {
      continue;
    }
    SelfContactFrictionContact contact;
    contact.nodes
        = {candidate.point, triangle.nodeA, triangle.nodeB, triangle.nodeC};
    contact.normalForce = result.gradient.template head<3>().norm();
    contact.projection = dc::pointTriangleTangentStencil(p, a, b, c).projection;
    contacts.push_back(contact);
  }

  for (const auto& candidate : candidates.edgeEdgeCandidates) {
    const auto& edgeA = candidates.surfaceEdges[candidate.edgeA];
    const auto& edgeB = candidates.surfaceEdges[candidate.edgeB];
    const auto& a0 = positions[edgeA.nodeA];
    const auto& a1 = positions[edgeA.nodeB];
    const auto& b0 = positions[edgeB.nodeA];
    const auto& b1 = positions[edgeB.nodeB];
    const auto result = dc::edgeEdgeBarrier(
        a0, a1, b0, b1, barrier.squaredActivationDistance, barrier.stiffness);
    if (!result.active) {
      continue;
    }
    SelfContactFrictionContact contact;
    contact.nodes = {edgeA.nodeA, edgeA.nodeB, edgeB.nodeA, edgeB.nodeB};
    // Lagged normal force = net barrier force on edge A (its two endpoints).
    contact.normalForce = (result.gradient.template head<3>()
                           + result.gradient.template segment<3>(3))
                              .norm();
    contact.projection = dc::edgeEdgeTangentStencil(a0, a1, b0, b1).projection;
    contacts.push_back(contact);
  }
}

// Add lagged smoothed Coulomb self-contact friction energy/gradient over the
// active point-triangle contacts. The tangential relative displacement is
// projection * (stacked four-node displacement over the step); the IPC f0/f1
// mollifier gives a C1 force opposing it (saturating at mu * normalForce). The
// lagged friction Hessian is a later increment, like the ground-friction path's
// first cut; the line search on this energy still ensures descent.
double addSelfContactFrictionEnergy(
    const std::vector<Eigen::Vector3d>& positions,
    const std::vector<std::uint8_t>& fixed,
    const SelfContactFrictionInputs& friction,
    std::vector<Eigen::Vector3d>* gradient)
{
  if (friction.coefficient <= 0.0 || friction.epsilon <= 0.0
      || friction.stepStartPositions == nullptr
      || friction.contacts == nullptr) {
    return 0.0;
  }

  const auto& start = *friction.stepStartPositions;
  double energy = 0.0;
  for (const auto& contact : *friction.contacts) {
    if (contact.normalForce <= 0.0) {
      continue;
    }
    Eigen::Matrix<double, 12, 1> displacement;
    for (int k = 0; k < 4; ++k) {
      displacement.segment<3>(3 * k)
          = positions[contact.nodes[k]] - start[contact.nodes[k]];
    }
    const Eigen::Vector2d tangent = contact.projection * displacement;
    const double y = tangent.norm();
    const double scale = friction.coefficient * contact.normalForce;
    energy += scale * frictionF0(y, friction.epsilon);
    if (gradient != nullptr) {
      constexpr double tiny = 1e-12;
      const double ratio = (y > tiny) ? frictionF1(y, friction.epsilon) / y
                                      : 2.0 / friction.epsilon;
      const Eigen::Matrix<double, 12, 1> g
          = scale * ratio * (contact.projection.transpose() * tangent);
      for (int k = 0; k < 4; ++k) {
        if (fixed[contact.nodes[k]] != 0u) {
          continue;
        }
        (*gradient)[contact.nodes[k]] += g.segment<3>(3 * k);
      }
    }
  }
  return energy;
}

//==============================================================================
// Friction diagnostics at the converged iterate, over both static-ground and
// self-contact friction. Accumulates the IPC Coulomb dissipation
// mu * lambda * f1(y) * y (force times tangential slip; equal to mu * lambda *
// y in the kinetic regime and ramped smoothly to zero at rest by the mollifier)
// and the count of contacts carrying a nonzero lagged normal force. Evaluated
// once per step outside the line-search hot path, mirroring the slip measures
// the friction energy uses: u_T = (I - n n^T)(x - x_start) for ground contact
// and projection * (stacked four-node displacement) for self-contact.
void accumulateFrictionDiagnostics(
    const std::vector<Eigen::Vector3d>& positions,
    const std::vector<Eigen::Vector3d>& stepStart,
    const std::vector<std::uint8_t>& fixed,
    const double frictionCoefficient,
    const double epsilon,
    const std::vector<double>& groundNormalForce,
    const std::vector<Eigen::Vector3d>& groundNormalDirection,
    const std::vector<SelfContactFrictionContact>& selfContacts,
    double& dissipation,
    std::size_t& activeContacts)
{
  if (frictionCoefficient <= 0.0 || epsilon <= 0.0) {
    return;
  }

  for (std::size_t i = 0; i < groundNormalForce.size() && i < positions.size();
       ++i) {
    if (fixed[i] != 0u || groundNormalForce[i] <= 0.0) {
      continue;
    }
    const Eigen::Vector3d n = (i < groundNormalDirection.size())
                                  ? groundNormalDirection[i]
                                  : Eigen::Vector3d::UnitZ();
    const Eigen::Vector3d u = positions[i] - stepStart[i];
    const double y = (u - n.dot(u) * n).norm();
    const auto contribution = nb::frictionWorkContribution(
        y, frictionCoefficient * groundNormalForce[i], epsilon);
    if (contribution.active) {
      dissipation += contribution.work;
      ++activeContacts;
    }
  }

  for (const auto& contact : selfContacts) {
    if (contact.normalForce <= 0.0) {
      continue;
    }
    Eigen::Matrix<double, 12, 1> displacement;
    for (int k = 0; k < 4; ++k) {
      displacement.segment<3>(3 * k)
          = positions[contact.nodes[k]] - stepStart[contact.nodes[k]];
    }
    const double y = (contact.projection * displacement).norm();
    const auto contribution = nb::frictionWorkContribution(
        y, frictionCoefficient * contact.normalForce, epsilon);
    if (contribution.active) {
      dissipation += contribution.work;
      ++activeContacts;
    }
  }
}

//==============================================================================
// Closest-approach diagnostic over the active self-contact barrier set at the
// converged iterate. Each candidate's point-triangle / edge-edge squared
// distance is recomputed at the terminal positions; candidates within the
// activation band (squared distance < d_hat^2) form the active set, and the
// smallest distance among them is the IPC intersection-free "minimum distance"
// statistic. Returns the active-set size and writes the closest distance (0
// when the set is empty). Read once after the outer loop, not on the
// line-search hot path.
std::size_t accumulateContactDistanceDiagnostics(
    const std::vector<Eigen::Vector3d>& positions,
    const SelfContactBarrierInputs& barrier,
    double& outMinDistance)
{
  outMinDistance = 0.0;
  if (barrier.candidates == nullptr || barrier.triangles == nullptr
      || !(barrier.squaredActivationDistance > 0.0)) {
    return 0;
  }

  const auto& candidates = *barrier.candidates;
  const auto& triangles = *barrier.triangles;
  double minSquared = std::numeric_limits<double>::infinity();
  std::size_t activeContacts = 0;

  for (const auto& candidate : candidates.pointTriangleCandidates) {
    const auto& triangle = triangles[candidate.triangle];
    const double squaredDistance = dc::pointTriangleSquaredDistance(
                                       positions[candidate.point],
                                       positions[triangle.nodeA],
                                       positions[triangle.nodeB],
                                       positions[triangle.nodeC])
                                       .squaredDistance;
    if (squaredDistance >= barrier.squaredActivationDistance) {
      continue;
    }
    ++activeContacts;
    minSquared = std::min(minSquared, squaredDistance);
  }

  for (const auto& candidate : candidates.edgeEdgeCandidates) {
    const auto& edgeA = candidates.surfaceEdges[candidate.edgeA];
    const auto& edgeB = candidates.surfaceEdges[candidate.edgeB];
    const double squaredDistance = dc::edgeEdgeSquaredDistance(
                                       positions[edgeA.nodeA],
                                       positions[edgeA.nodeB],
                                       positions[edgeB.nodeA],
                                       positions[edgeB.nodeB])
                                       .squaredDistance;
    if (squaredDistance >= barrier.squaredActivationDistance) {
      continue;
    }
    ++activeContacts;
    minSquared = std::min(minSquared, squaredDistance);
  }

  if (activeContacts > 0 && std::isfinite(minSquared)) {
    outMinDistance = std::sqrt(std::max(0.0, minSquared));
  }
  return activeContacts;
}

//==============================================================================
// Stable neo-Hookean FEM elasticity inputs. When non-null (opt-in via
// DeformableMaterial.useFiniteElementElasticity), each tetrahedron contributes
// a volumetric strain energy/gradient/Hessian instead of the mass-spring edge
// model. Null preserves the spring objective exactly.
struct FemElasticityInputs
{
  const std::vector<comps::DeformableTetrahedron>* tetrahedra = nullptr;
  const std::vector<fem::TetRestShape>* restShapes = nullptr;
  fem::LameParameters lame;
  // Selects the isotropic material: false (default) is the inversion-robust
  // stable neo-Hookean kernel; true is fixed-corotational (the IPC paper's
  // other material), opt-in via
  // DeformableMaterial.useFixedCorotationalElasticity.
  bool fixedCorotational = false;
};

// Dispatches one tetrahedron to the configured FEM material kernel. Both
// kernels share the TetElementResult shape, so the rest of the assembly is
// identical.
inline fem::TetElementResult evaluateFemTetElement(
    const FemElasticityInputs& inputs,
    const std::vector<Eigen::Vector3d>& positions,
    const comps::DeformableTetrahedron& tet,
    const fem::TetRestShape& rest,
    const bool computeHessian)
{
  if (inputs.fixedCorotational) {
    return fem::evaluateFixedCorotationalTet(
        positions[tet.nodeA],
        positions[tet.nodeB],
        positions[tet.nodeC],
        positions[tet.nodeD],
        rest,
        inputs.lame,
        computeHessian);
  }
  return fem::evaluateStableNeoHookeanTet(
      positions[tet.nodeA],
      positions[tet.nodeB],
      positions[tet.nodeC],
      positions[tet.nodeD],
      rest,
      inputs.lame,
      computeHessian);
}

// Adds the stable neo-Hookean strain energy/gradient over every valid
// tetrahedron, scattering each element's 12-vector force gradient into its four
// nodes (fixed nodes receive no gradient, like the spring term).
double addFemElasticityEnergy(
    const std::vector<Eigen::Vector3d>& positions,
    const std::vector<std::uint8_t>& fixed,
    const FemElasticityInputs& inputs,
    std::vector<Eigen::Vector3d>* gradient)
{
  if (inputs.tetrahedra == nullptr || inputs.restShapes == nullptr) {
    return 0.0;
  }
  const auto& tets = *inputs.tetrahedra;
  const auto& rests = *inputs.restShapes;
  const std::size_t count = std::min(tets.size(), rests.size());
  double energy = 0.0;
  for (std::size_t t = 0; t < count; ++t) {
    const auto& tet = tets[t];
    const fem::TetElementResult element = evaluateFemTetElement(
        inputs, positions, tet, rests[t], /*computeHessian=*/false);
    if (!element.valid) {
      continue;
    }
    energy += element.energy;
    if (gradient != nullptr) {
      const std::array<std::size_t, 4> nodes
          = {tet.nodeA, tet.nodeB, tet.nodeC, tet.nodeD};
      for (int i = 0; i < 4; ++i) {
        if (fixed[nodes[i]] == 0u) {
          (*gradient)[nodes[i]] += element.gradient.segment<3>(3 * i);
        }
      }
    }
  }
  return energy;
}

double evaluateDeformableObjective(
    const comps::DeformableNodeState& state,
    const comps::DeformableSpringModel& model,
    const std::vector<Eigen::Vector3d>& positions,
    const std::vector<Eigen::Vector3d>& inertialTargets,
    const std::vector<std::uint8_t>& fixed,
    const std::vector<StaticGroundBarrier>& barriers,
    const std::vector<SphereObstacleBarrier>& sphereObstacles,
    const std::vector<BoxObstacleBarrier>& boxObstacles,
    const std::vector<CapsuleObstacleBarrier>& capsuleObstacles,
    double timeStep,
    std::vector<Eigen::Vector3d>* gradient,
    const SelfContactBarrierInputs* contactBarrier = nullptr,
    std::size_t* barrierActiveContacts = nullptr,
    const GroundFrictionInputs* groundFriction = nullptr,
    const SelfContactFrictionInputs* selfContactFriction = nullptr,
    const FemElasticityInputs* femElasticity = nullptr,
    double barrierStiffness = kDefaultBarrierStiffness)
{
  if (gradient != nullptr) {
    if (gradient->size() != positions.size()) {
      gradient->resize(positions.size());
    }
    std::fill(gradient->begin(), gradient->end(), Eigen::Vector3d::Zero());
  }

  double energy = 0.0;
  const double invDt2 = 1.0 / (timeStep * timeStep);
  for (std::size_t i = 0; i < positions.size(); ++i) {
    if (fixed[i] != 0u) {
      continue;
    }

    const Eigen::Vector3d delta = positions[i] - inertialTargets[i];
    energy += 0.5 * state.masses[i] * invDt2 * delta.squaredNorm();
    if (gradient != nullptr) {
      (*gradient)[i] += state.masses[i] * invDt2 * delta;
    }
  }

  constexpr double minLength = 1e-12;
  for (const auto& edge : model.edges) {
    const Eigen::Vector3d delta = positions[edge.nodeB] - positions[edge.nodeA];
    const double length = delta.norm();
    if (length <= minLength || !std::isfinite(length)) {
      continue;
    }

    const Eigen::Vector3d direction = delta / length;
    const double stretch = length - edge.restLength;
    energy += 0.5 * model.stiffness * stretch * stretch;
    if (gradient != nullptr) {
      const Eigen::Vector3d springGradient
          = model.stiffness * stretch * direction;
      if (fixed[edge.nodeA] == 0u) {
        (*gradient)[edge.nodeA] -= springGradient;
      }
      if (fixed[edge.nodeB] == 0u) {
        (*gradient)[edge.nodeB] += springGradient;
      }
    }
  }

  if (femElasticity != nullptr) {
    energy
        += addFemElasticityEnergy(positions, fixed, *femElasticity, gradient);
  }
  energy += addStaticGroundBarrierEnergy(
      positions, fixed, barriers, gradient, barrierStiffness);
  energy += addSphereObstacleBarrierEnergy(
      positions, fixed, sphereObstacles, gradient, barrierStiffness);
  energy += addBoxObstacleBarrierEnergy(
      positions, fixed, boxObstacles, gradient, barrierStiffness);
  energy += addCapsuleObstacleBarrierEnergy(
      positions, fixed, capsuleObstacles, gradient, barrierStiffness);
  if (contactBarrier != nullptr) {
    energy += addSelfContactBarrierEnergy(
        positions, fixed, *contactBarrier, barrierActiveContacts, gradient);
  }
  if (groundFriction != nullptr) {
    energy
        += addGroundFrictionEnergy(positions, fixed, *groundFriction, gradient);
  }
  if (selfContactFriction != nullptr) {
    energy += addSelfContactFrictionEnergy(
        positions, fixed, *selfContactFriction, gradient);
  }
  return energy;
}

//==============================================================================
double gradientNormSquared(
    const std::vector<Eigen::Vector3d>& gradient,
    const std::vector<std::uint8_t>& fixed)
{
  double normSquared = 0.0;
  for (std::size_t i = 0; i < gradient.size(); ++i) {
    if (fixed[i] == 0u) {
      normSquared += gradient[i].squaredNorm();
    }
  }
  return normSquared;
}

//==============================================================================
double buildLineSearchCandidate(
    const std::vector<Eigen::Vector3d>& current,
    const std::vector<Eigen::Vector3d>& direction,
    const std::vector<Eigen::Vector3d>& gradient,
    const std::vector<std::uint8_t>& fixed,
    double step,
    std::vector<Eigen::Vector3d>& candidate)
{
  double directionalDerivative = 0.0;
  for (std::size_t i = 0; i < current.size(); ++i) {
    candidate[i] = current[i];
    if (fixed[i] == 0u) {
      candidate[i] += step * direction[i];
      directionalDerivative += gradient[i].dot(candidate[i] - current[i]);
    }
  }
  return directionalDerivative;
}

//==============================================================================
bool applySurfaceContactCcdLimit(
    const std::vector<Eigen::Vector3d>& current,
    const std::vector<Eigen::Vector3d>& direction,
    const std::vector<Eigen::Vector3d>& gradient,
    const std::vector<std::uint8_t>& fixed,
    DeformableContactSolverScratch& contactScratch,
    DeformableSolverStats& stats,
    double& step,
    std::vector<Eigen::Vector3d>& candidate,
    double& directionalDerivative)
{
  if (contactScratch.surfaceTriangles.empty()) {
    return true;
  }

  ++stats.surfaceContactCandidateBuilds;
  dc::buildMotionAwareContactCandidatesSweep(
      current,
      candidate,
      contactScratch.surfaceTriangles,
      makeSurfaceContactCandidateOptions(),
      contactScratch.candidates,
      contactScratch.sweepScratch);
  filterSurfaceContactPointCandidates(
      contactScratch.candidates, contactScratch.surfaceContactPointMask);
  stats.surfaceContactPointTriangleCandidates
      += contactScratch.candidates.pointTriangleCandidates.size();
  stats.surfaceContactEdgeEdgeCandidates
      += contactScratch.candidates.edgeEdgeCandidates.size();

  const auto result = dc::contactCandidateStepBound(
      current,
      candidate,
      contactScratch.surfaceTriangles,
      contactScratch.candidates,
      makeSurfaceContactCcdOptions());
  stats.surfaceContactCcdPointTriangleChecks
      += result.stats.pointTriangleChecks;
  stats.surfaceContactCcdEdgeEdgeChecks += result.stats.edgeEdgeChecks;
  stats.surfaceContactCcdHits += result.stats.hits;
  stats.surfaceContactCcdMisses += result.stats.misses;
  stats.surfaceContactCcdIndeterminateCount += result.stats.indeterminate;
  stats.surfaceContactCcdZeroStepCount += result.stats.zeroStepCount;

  if (!result.allowsPositiveStep()) {
    return false;
  }

  if (!result.hit) {
    return true;
  }

  const double safeFraction
      = nb::makeInteriorLineSearchStepScale(result.stepBound);
  if (safeFraction <= 0.0) {
    return false;
  }

  step *= safeFraction;
  ++stats.surfaceContactCcdLimitedSteps;
  directionalDerivative = buildLineSearchCandidate(
      current, direction, gradient, fixed, step, candidate);
  return true;
}

//==============================================================================
bool applyInterBodySurfaceContactCcdLimit(
    entt::entity entity,
    std::span<const SurfaceContactSnapshot> surfaceSnapshots,
    const std::vector<Eigen::Vector3d>& current,
    const std::vector<Eigen::Vector3d>& direction,
    const std::vector<Eigen::Vector3d>& gradient,
    const std::vector<std::uint8_t>& fixed,
    DeformableContactSolverScratch& contactScratch,
    DeformableSolverStats& stats,
    double& step,
    std::vector<Eigen::Vector3d>& candidate,
    double& directionalDerivative)
{
  if (contactScratch.surfaceTriangles.empty()) {
    return true;
  }

  bool hasObstacleSurface = false;
  for (const auto& snapshot : surfaceSnapshots) {
    if (snapshot.entity != entity && !snapshot.surfaceTriangles.empty()) {
      hasObstacleSurface = true;
      break;
    }
  }
  if (!hasObstacleSurface) {
    return true;
  }

  dc::buildUniqueSurfaceEdges(
      contactScratch.surfaceTriangles, contactScratch.interBodyCurrentEdges);

  ++stats.interBodySurfaceContactCandidateBuilds;
  bool hit = false;
  bool indeterminate = false;
  double stepBound = 1.0;

  for (const auto& snapshot : surfaceSnapshots) {
    if (snapshot.entity == entity || snapshot.surfaceTriangles.empty()) {
      continue;
    }

    const auto result = interBodySurfaceContactStepBound(
        current,
        candidate,
        contactScratch.surfaceTriangles,
        contactScratch.surfaceContactPointMask,
        contactScratch.interBodyCurrentEdges,
        snapshot,
        makeSurfaceContactCandidateOptions(),
        makeSurfaceContactCcdOptions(),
        contactScratch);

    stats.interBodySurfaceContactPointTriangleCandidates
        += result.pointTriangleCandidateCount;
    stats.interBodySurfaceContactEdgeEdgeCandidates
        += result.edgeEdgeCandidateCount;
    stats.interBodySurfaceContactCcdPointTriangleChecks
        += result.stats.pointTriangleChecks;
    stats.interBodySurfaceContactCcdEdgeEdgeChecks
        += result.stats.edgeEdgeChecks;
    stats.interBodySurfaceContactCcdHits += result.stats.hits;
    stats.interBodySurfaceContactCcdMisses += result.stats.misses;
    stats.interBodySurfaceContactCcdIndeterminateCount
        += result.stats.indeterminate;
    stats.interBodySurfaceContactCcdZeroStepCount += result.stats.zeroStepCount;

    indeterminate = indeterminate || result.indeterminate;
    if (result.indeterminate) {
      stepBound = 0.0;
    }
    if (result.hit && (!hit || result.stepBound < stepBound)) {
      hit = true;
      stepBound = result.stepBound;
    }
  }

  if (indeterminate) {
    return false;
  }

  if (!hit) {
    return true;
  }

  const double safeFraction = nb::makeInteriorLineSearchStepScale(stepBound);
  if (safeFraction <= 0.0) {
    return false;
  }

  step *= safeFraction;
  ++stats.interBodySurfaceContactCcdLimitedSteps;
  directionalDerivative = buildLineSearchCandidate(
      current, direction, gradient, fixed, step, candidate);
  return true;
}

//==============================================================================
// Apply the self-surface CCD limiter to a fully assembled candidate
// displacement. VBD uses this after its block solve so fast same-body surface
// crossings keep the same no-tunneling limit as the default line-search path.
bool applySurfaceContactCcdCandidateLimit(
    const std::vector<Eigen::Vector3d>& current,
    const std::vector<std::uint8_t>& fixed,
    DeformableContactSolverScratch& contactScratch,
    DeformableSolverStats& stats,
    comps::DeformableSolverScratch& scratch)
{
  if (contactScratch.surfaceTriangles.empty()) {
    return true;
  }

  const std::size_t nodeCount = current.size();
  scratch.direction.resize(nodeCount);
  scratch.gradient.assign(nodeCount, Eigen::Vector3d::Zero());
  scratch.candidate = scratch.next;
  for (std::size_t i = 0; i < nodeCount; ++i) {
    scratch.direction[i] = scratch.candidate[i] - current[i];
  }

  double step = 1.0;
  double directionalDerivative = 0.0;
  const bool accepted = applySurfaceContactCcdLimit(
      current,
      scratch.direction,
      scratch.gradient,
      fixed,
      contactScratch,
      stats,
      step,
      scratch.candidate,
      directionalDerivative);

  if (accepted) {
    scratch.next = scratch.candidate;
    return true;
  }

  scratch.next = current;
  return false;
}

//==============================================================================
// Apply the inter-body deformable-surface CCD limiter to a fully assembled
// candidate displacement. VBD uses this after its block solve so opt-in
// surface bodies keep the same no-tunneling limit the default line-search path
// applies between deformable bodies.
bool applyInterBodySurfaceContactCcdCandidateLimit(
    entt::entity entity,
    std::span<const SurfaceContactSnapshot> surfaceSnapshots,
    const std::vector<Eigen::Vector3d>& current,
    const std::vector<std::uint8_t>& fixed,
    DeformableContactSolverScratch& contactScratch,
    DeformableSolverStats& stats,
    comps::DeformableSolverScratch& scratch)
{
  if (surfaceSnapshots.empty()) {
    return true;
  }

  const std::size_t nodeCount = current.size();
  scratch.direction.resize(nodeCount);
  scratch.gradient.assign(nodeCount, Eigen::Vector3d::Zero());
  scratch.candidate = scratch.next;
  for (std::size_t i = 0; i < nodeCount; ++i) {
    scratch.direction[i] = scratch.candidate[i] - current[i];
  }

  double step = 1.0;
  double directionalDerivative = 0.0;
  const bool accepted = applyInterBodySurfaceContactCcdLimit(
      entity,
      surfaceSnapshots,
      current,
      scratch.direction,
      scratch.gradient,
      fixed,
      contactScratch,
      stats,
      step,
      scratch.candidate,
      directionalDerivative);

  if (accepted) {
    scratch.next = scratch.candidate;
    return true;
  }

  scratch.next = current;
  return false;
}

//==============================================================================
bool applyStaticRigidSurfaceCcdLimit(
    std::span<const SurfaceContactSnapshot> rigidSurfaceSnapshots,
    const std::vector<Eigen::Vector3d>& current,
    const std::vector<Eigen::Vector3d>& direction,
    const std::vector<Eigen::Vector3d>& gradient,
    const std::vector<std::uint8_t>& fixed,
    DeformableContactSolverScratch& contactScratch,
    DeformableSolverStats& stats,
    double& step,
    std::vector<Eigen::Vector3d>& candidate,
    double& directionalDerivative)
{
  if (rigidSurfaceSnapshots.empty()) {
    return true;
  }

  if (contactScratch.surfaceTriangles.empty()) {
    contactScratch.interBodyCurrentEdges.clear();
  } else {
    dc::buildUniqueSurfaceEdges(
        contactScratch.surfaceTriangles, contactScratch.interBodyCurrentEdges);
  }

  ++stats.staticRigidSurfaceCcdCandidateBuilds;
  bool hit = false;
  bool indeterminate = false;
  double stepBound = 1.0;

  for (const auto& snapshot : rigidSurfaceSnapshots) {
    if (snapshot.surfaceTriangles.empty()) {
      continue;
    }

    const auto result = interBodySurfaceContactStepBound(
        current,
        candidate,
        contactScratch.surfaceTriangles,
        contactScratch.surfaceContactPointMask,
        contactScratch.interBodyCurrentEdges,
        snapshot,
        makeSurfaceContactCandidateOptions(),
        makeSurfaceContactCcdOptions(),
        contactScratch);

    stats.staticRigidSurfaceCcdPointTriangleCandidates
        += result.pointTriangleCandidateCount;
    stats.staticRigidSurfaceCcdEdgeEdgeCandidates
        += result.edgeEdgeCandidateCount;
    stats.staticRigidSurfaceCcdPointTriangleChecks
        += result.stats.pointTriangleChecks;
    stats.staticRigidSurfaceCcdEdgeEdgeChecks += result.stats.edgeEdgeChecks;
    stats.staticRigidSurfaceCcdHits += result.stats.hits;
    stats.staticRigidSurfaceCcdMisses += result.stats.misses;
    stats.staticRigidSurfaceCcdIndeterminateCount += result.stats.indeterminate;
    stats.staticRigidSurfaceCcdZeroStepCount += result.stats.zeroStepCount;

    indeterminate = indeterminate || result.indeterminate;
    if (result.indeterminate) {
      stepBound = 0.0;
    }
    if (result.hit && (!hit || result.stepBound < stepBound)) {
      hit = true;
      stepBound = result.stepBound;
    }
  }

  if (indeterminate) {
    return false;
  }

  if (!hit) {
    return true;
  }

  const double safeFraction = nb::makeInteriorLineSearchStepScale(stepBound);
  if (safeFraction <= 0.0) {
    ++stats.staticRigidSurfaceCcdZeroStepCount;
    return false;
  }

  step *= safeFraction;
  ++stats.staticRigidSurfaceCcdLimitedSteps;
  directionalDerivative = buildLineSearchCandidate(
      current, direction, gradient, fixed, step, candidate);
  return true;
}

//==============================================================================
// Apply the static rigid-surface CCD limiter to a fully assembled candidate
// displacement, such as the World VBD path's post-solve iterate. This reuses
// the default solver's step-bound machinery without coupling VBD to line-search
// gradients.
bool applyStaticRigidSurfaceCcdCandidateLimit(
    std::span<const SurfaceContactSnapshot> rigidSurfaceSnapshots,
    const std::vector<Eigen::Vector3d>& current,
    const std::vector<std::uint8_t>& fixed,
    DeformableContactSolverScratch& contactScratch,
    DeformableSolverStats& stats,
    comps::DeformableSolverScratch& scratch)
{
  if (rigidSurfaceSnapshots.empty()) {
    return true;
  }

  const std::size_t nodeCount = current.size();
  scratch.direction.resize(nodeCount);
  scratch.gradient.assign(nodeCount, Eigen::Vector3d::Zero());
  scratch.candidate = scratch.next;
  for (std::size_t i = 0; i < nodeCount; ++i) {
    scratch.direction[i] = scratch.candidate[i] - current[i];
  }

  double step = 1.0;
  double directionalDerivative = 0.0;
  const bool accepted = applyStaticRigidSurfaceCcdLimit(
      rigidSurfaceSnapshots,
      current,
      scratch.direction,
      scratch.gradient,
      fixed,
      contactScratch,
      stats,
      step,
      scratch.candidate,
      directionalDerivative);

  if (accepted) {
    scratch.next = scratch.candidate;
    return true;
  }

  scratch.next = current;
  return false;
}

//==============================================================================
// Conservative CCD limiter against MOVING rigid box obstacles. The snapshots
// are static poses sampled along the obstacle's predicted swept motion, so this
// reuses exactly the static-obstacle step-bound path; only the stat family
// differs. Limiting against every sampled pose keeps the deformable out of the
// obstacle's swept corridor.
bool applyMovingRigidSurfaceCcdLimit(
    std::span<const SurfaceContactSnapshot> movingRigidSurfaceSnapshots,
    const std::vector<Eigen::Vector3d>& current,
    const std::vector<Eigen::Vector3d>& direction,
    const std::vector<Eigen::Vector3d>& gradient,
    const std::vector<std::uint8_t>& fixed,
    DeformableContactSolverScratch& contactScratch,
    DeformableSolverStats& stats,
    double& step,
    std::vector<Eigen::Vector3d>& candidate,
    double& directionalDerivative)
{
  if (movingRigidSurfaceSnapshots.empty()) {
    return true;
  }

  if (contactScratch.surfaceTriangles.empty()) {
    contactScratch.interBodyCurrentEdges.clear();
  } else {
    dc::buildUniqueSurfaceEdges(
        contactScratch.surfaceTriangles, contactScratch.interBodyCurrentEdges);
  }

  ++stats.movingRigidSurfaceCcdCandidateBuilds;
  bool hit = false;
  bool indeterminate = false;
  double stepBound = 1.0;

  for (const auto& snapshot : movingRigidSurfaceSnapshots) {
    if (snapshot.surfaceTriangles.empty()) {
      continue;
    }

    const auto result = interBodySurfaceContactStepBound(
        current,
        candidate,
        contactScratch.surfaceTriangles,
        contactScratch.surfaceContactPointMask,
        contactScratch.interBodyCurrentEdges,
        snapshot,
        makeSurfaceContactCandidateOptions(),
        makeSurfaceContactCcdOptions(),
        contactScratch);

    stats.movingRigidSurfaceCcdPointTriangleCandidates
        += result.pointTriangleCandidateCount;
    stats.movingRigidSurfaceCcdEdgeEdgeCandidates
        += result.edgeEdgeCandidateCount;
    stats.movingRigidSurfaceCcdPointTriangleChecks
        += result.stats.pointTriangleChecks;
    stats.movingRigidSurfaceCcdEdgeEdgeChecks += result.stats.edgeEdgeChecks;
    stats.movingRigidSurfaceCcdHits += result.stats.hits;
    stats.movingRigidSurfaceCcdMisses += result.stats.misses;
    stats.movingRigidSurfaceCcdIndeterminateCount += result.stats.indeterminate;
    stats.movingRigidSurfaceCcdZeroStepCount += result.stats.zeroStepCount;

    indeterminate = indeterminate || result.indeterminate;
    if (result.indeterminate) {
      stepBound = 0.0;
    }
    if (result.hit && (!hit || result.stepBound < stepBound)) {
      hit = true;
      stepBound = result.stepBound;
    }
  }

  if (indeterminate) {
    return false;
  }

  if (!hit) {
    return true;
  }

  const double safeFraction = nb::makeInteriorLineSearchStepScale(stepBound);
  if (safeFraction <= 0.0) {
    ++stats.movingRigidSurfaceCcdZeroStepCount;
    return false;
  }

  step *= safeFraction;
  ++stats.movingRigidSurfaceCcdLimitedSteps;
  directionalDerivative = buildLineSearchCandidate(
      current, direction, gradient, fixed, step, candidate);
  return true;
}

//==============================================================================
bool applyStaticGroundBarrierCcdLimit(
    const std::vector<Eigen::Vector3d>& current,
    const std::vector<Eigen::Vector3d>& direction,
    const std::vector<Eigen::Vector3d>& gradient,
    const std::vector<std::uint8_t>& fixed,
    const std::vector<StaticGroundBarrier>& barriers,
    DeformableSolverStats& stats,
    double& step,
    std::vector<Eigen::Vector3d>& candidate,
    double& directionalDerivative)
{
  if (barriers.empty()) {
    return true;
  }

  bool hit = false;
  double stepBound = 1.0;
  for (std::size_t node = 0; node < current.size(); ++node) {
    if (fixed[node] != 0u) {
      continue;
    }

    ++stats.staticGroundBarrierCcdNodeChecks;
    const auto nodeStepBound = staticGroundBarrierStepBound(
        current[node], candidate[node], barriers, stats);
    if (!nodeStepBound.has_value()) {
      continue;
    }

    ++stats.staticGroundBarrierCcdHits;
    if (*nodeStepBound <= 0.0) {
      ++stats.staticGroundBarrierCcdZeroStepCount;
      return false;
    }
    hit = true;
    stepBound = std::min(stepBound, *nodeStepBound);
  }

  if (!hit) {
    return true;
  }

  const double safeFraction = nb::makeInteriorLineSearchStepScale(stepBound);
  if (safeFraction <= 0.0) {
    ++stats.staticGroundBarrierCcdZeroStepCount;
    return false;
  }

  step *= safeFraction;
  ++stats.staticGroundBarrierCcdLimitedSteps;
  directionalDerivative = buildLineSearchCandidate(
      current, direction, gradient, fixed, step, candidate);
  return true;
}

//==============================================================================
bool isBoundaryActiveAtStepStart(double time, double start, double end)
{
  // Contact-free scene controls intentionally use step-start sampling. Later
  // force-work slices can add partial-step integration when needed.
  return time >= start && time < end;
}

//==============================================================================
double elapsedBoundaryTime(
    double time, double timeStep, double start, double end)
{
  const double nextTime = std::min(time + timeStep, end);
  return std::max(0.0, nextTime - start);
}

//==============================================================================
Eigen::Vector3d boundaryVelocity(
    const comps::DeformableDirichletBoundary& boundary,
    const Eigen::Vector3d& referencePosition)
{
  return boundary.linearVelocity
         + boundary.angularVelocity.cross(referencePosition - boundary.center);
}

//==============================================================================
void reserveDeformableSolverScratch(
    const comps::DeformableNodeState& state,
    comps::DeformableSolverScratch& scratch)
{
  const auto nodeCount = state.positions.size();
  scratch.inertialTargets.reserve(nodeCount);
  scratch.next.reserve(nodeCount);
  scratch.gradient.reserve(nodeCount);
  scratch.direction.reserve(nodeCount);
  scratch.candidate.reserve(nodeCount);
  scratch.previousStepPositions.reserve(nodeCount);
  scratch.externalAccelerations.reserve(nodeCount);
  scratch.activeFixed.reserve(nodeCount);
  scratch.activeDirichlet.reserve(nodeCount);
  scratch.countedDirichlet.reserve(nodeCount);
  scratch.countedNeumann.reserve(nodeCount);
}

//==============================================================================
void reserveSurfaceContactCandidateScratch(
    std::size_t nodeCount, DeformableContactSolverScratch& scratch)
{
  const std::size_t triangleCount = scratch.surfaceTriangles.size();
  const std::size_t edgeCapacity = 3 * triangleCount;
  const std::size_t pointTriangleCapacity = 8 * (nodeCount + triangleCount);
  const std::size_t edgeEdgeCapacity = 12 * edgeCapacity;
  // Motion-aware late-activation sweeps can overlap several static grid
  // bands in one step, so reserve the swept broad-phase envelope rather than
  // only the current-pose contact band.
  const std::size_t sweptPointTriangleCapacity
      = 32 * (nodeCount + triangleCount);
  const std::size_t sweptEdgeEdgeCapacity = 48 * edgeCapacity;

  scratch.candidates.surfaceEdges.reserve(edgeCapacity);
  scratch.candidates.pointTriangleCandidates.reserve(
      sweptPointTriangleCapacity);
  scratch.candidates.edgeEdgeCandidates.reserve(sweptEdgeEdgeCapacity);
  scratch.barrierCandidates.surfaceEdges.reserve(edgeCapacity);
  scratch.barrierCandidates.pointTriangleCandidates.reserve(
      pointTriangleCapacity);
  scratch.barrierCandidates.edgeEdgeCandidates.reserve(edgeEdgeCapacity);
  scratch.sweepScratch.pointItems.reserve(nodeCount);
  scratch.sweepScratch.triangleItems.reserve(triangleCount);
  scratch.sweepScratch.edgeItems.reserve(edgeCapacity);
  scratch.sweepScratch.sweepLinks.reserve(
      std::max(nodeCount, std::max(triangleCount, edgeCapacity)));
}

//==============================================================================
void reserveVbdSelfContactCandidateScratch(
    std::size_t nodeCount,
    std::size_t triangleCount,
    DeformableVbdScratch& scratch)
{
  const std::size_t edgeCapacity = 3 * triangleCount;
  const std::size_t pointTriangleCapacity = 4 * (nodeCount + triangleCount);
  const std::size_t edgeEdgeCapacity = 6 * edgeCapacity;

  scratch.selfContactCandidates.surfaceEdges.reserve(edgeCapacity);
  scratch.selfContactCandidates.pointTriangleCandidates.reserve(
      pointTriangleCapacity);
  scratch.selfContactCandidates.edgeEdgeCandidates.reserve(edgeEdgeCapacity);
  scratch.selfContactSweepScratch.pointItems.reserve(nodeCount);
  scratch.selfContactSweepScratch.triangleItems.reserve(triangleCount);
  scratch.selfContactSweepScratch.edgeItems.reserve(edgeCapacity);
  scratch.selfContactSweepScratch.sweepLinks.reserve(
      std::max(nodeCount, std::max(triangleCount, edgeCapacity)));
}

//==============================================================================
void reserveDeformableFrictionScratch(
    std::size_t nodeCount, DeformableContactSolverScratch& scratch)
{
  scratch.groundFrictionNormalForce.reserve(nodeCount);
  scratch.groundFrictionNormalDirection.reserve(nodeCount);
  scratch.selfContactFrictionContacts.reserve(
      scratch.barrierCandidates.pointTriangleCandidates.capacity()
      + scratch.barrierCandidates.edgeEdgeCandidates.capacity());
}

//==============================================================================
void reserveProjectedNewtonScratch(
    std::size_t nodeCount,
    const comps::DeformableSpringModel& model,
    const comps::DeformableMeshTopology& topology,
    DeformableContactSolverScratch& scratch)
{
  const auto dim = static_cast<Eigen::Index>(3 * nodeCount);
  scratch.projectedNewtonRhs.resize(dim);
  scratch.projectedNewtonSolution.resize(dim);

  // Reserve DART-owned barrier buffers for the baked candidate capacity, not
  // only the contacts active at bake.
  const std::size_t barrierCandidateCount
      = scratch.barrierCandidates.pointTriangleCandidates.capacity()
        + scratch.barrierCandidates.edgeEdgeCandidates.capacity();
  const std::size_t tripletEstimate = 3 * nodeCount + 36 * model.edges.size()
                                      + 144 * topology.tetrahedra.size()
                                      + 144 * barrierCandidateCount
                                      + 36 * nodeCount;
  const std::size_t matrixFreeBlockEstimate
      = 4 * nodeCount + 4 * model.edges.size() + 16 * topology.tetrahedra.size()
        + 16 * barrierCandidateCount;
  scratch.projectedNewtonTriplets.reserve(tripletEstimate);
  scratch.projectedNewtonHessian.resize(dim, dim);
  scratch.projectedNewtonHessian.reserve(
      static_cast<Eigen::Index>(tripletEstimate));
  scratch.newtonPatternOuter.reserve(static_cast<std::size_t>(dim + 1));
  scratch.newtonPatternInner.reserve(tripletEstimate);
  scratch.projectedNewtonEdgeBlocks.reserve(36 * model.edges.size());
  scratch.projectedNewtonEdgeBlockNodes.reserve(model.edges.size());
  scratch.projectedNewtonTetBlocks.reserve(144 * topology.tetrahedra.size());
  scratch.projectedNewtonTetBlockNodes.reserve(topology.tetrahedra.size());
  scratch.projectedNewtonBarrierBlocks.reserve(144 * barrierCandidateCount);
  scratch.projectedNewtonBarrierBlockNodes.reserve(barrierCandidateCount);
  scratch.projectedNewtonMatrixFreeBlocks.reserve(matrixFreeBlockEstimate);
  scratch.projectedNewtonMatrixFreeDiagonalBlocks.reserve(nodeCount);
  scratch.projectedNewtonMatrixFreeDiagonalBlocks.resize(nodeCount);
  scratch.projectedNewtonMatrixFreeInverseDiagonalBlocks.reserve(nodeCount);
  scratch.projectedNewtonMatrixFreeInverseDiagonalBlocks.resize(nodeCount);
  scratch.projectedNewtonMatrixFreeResidual.resize(dim);
  scratch.projectedNewtonMatrixFreePreconditionedResidual.resize(dim);
  scratch.projectedNewtonMatrixFreeDirection.resize(dim);
  scratch.projectedNewtonMatrixFreeHessianDirection.resize(dim);
}

//==============================================================================
void syncFemRestShapeScratch(
    std::size_t nodeCount,
    const comps::DeformableMeshTopology& topology,
    const comps::DeformableMaterial& material,
    DeformableContactSolverScratch& scratch)
{
  if (!material.useFiniteElementElasticity || topology.tetrahedra.empty()
      || topology.restPositions.size() != nodeCount
      || scratch.femRestShapes.size() == topology.tetrahedra.size()) {
    return;
  }

  scratch.femRestShapes.clear();
  scratch.femRestShapes.reserve(topology.tetrahedra.size());
  for (const auto& tet : topology.tetrahedra) {
    scratch.femRestShapes.push_back(
        fem::makeTetRestShape(
            topology.restPositions[tet.nodeA],
            topology.restPositions[tet.nodeB],
            topology.restPositions[tet.nodeC],
            topology.restPositions[tet.nodeD]));
  }
}

//==============================================================================
void prepareDeformableBoundaryConditions(
    comps::DeformableNodeState& state,
    const comps::DeformableBoundaryConditions* boundaryConditions,
    double time,
    double timeStep,
    comps::DeformableSolverScratch& scratch,
    DeformableSolverStats& stats)
{
  const auto nodeCount = state.positions.size();
  reserveDeformableSolverScratch(state, scratch);
  scratch.previousStepPositions = state.positions;
  scratch.externalAccelerations.assign(nodeCount, Eigen::Vector3d::Zero());
  scratch.activeFixed = state.fixed;
  scratch.activeDirichlet.assign(nodeCount, 0u);

  if (boundaryConditions == nullptr) {
    return;
  }

  scratch.countedDirichlet.assign(nodeCount, 0u);
  for (const auto& boundary : boundaryConditions->dirichlet) {
    if (!isBoundaryActiveAtStepStart(
            time, boundary.startTime, boundary.endTime)) {
      continue;
    }

    DART_SIMULATION_THROW_T_IF(
        boundary.nodes.size() != boundary.referencePositions.size(),
        InvalidArgumentException,
        "Serialized deformable Dirichlet boundary has mismatched node and "
        "reference-position counts");
    const double elapsed = elapsedBoundaryTime(
        time, timeStep, boundary.startTime, boundary.endTime);
    for (std::size_t i = 0; i < boundary.nodes.size(); ++i) {
      const auto node = boundary.nodes[i];
      DART_SIMULATION_THROW_T_IF(
          node >= nodeCount,
          InvalidArgumentException,
          "Serialized deformable Dirichlet boundary references out-of-range "
          "node {}",
          node);

      const auto velocity
          = boundaryVelocity(boundary, boundary.referencePositions[i]);
      state.positions[node]
          = boundary.referencePositions[i] + elapsed * velocity;
      state.velocities[node] = velocity;
      scratch.activeFixed[node] = 1u;
      scratch.activeDirichlet[node] = 1u;
      if (scratch.countedDirichlet[node] == 0u) {
        scratch.countedDirichlet[node] = 1u;
        ++stats.activeDirichletNodeCount;
      }
    }
  }

  scratch.countedNeumann.assign(nodeCount, 0u);
  for (const auto& boundary : boundaryConditions->neumann) {
    if (!isBoundaryActiveAtStepStart(
            time, boundary.startTime, boundary.endTime)) {
      continue;
    }

    for (const auto node : boundary.nodes) {
      DART_SIMULATION_THROW_T_IF(
          node >= nodeCount,
          InvalidArgumentException,
          "Serialized deformable Neumann boundary references out-of-range "
          "node {}",
          node);
      if (scratch.activeFixed[node] != 0u) {
        continue;
      }

      scratch.externalAccelerations[node] += boundary.acceleration;
      if (scratch.countedNeumann[node] == 0u) {
        scratch.countedNeumann[node] = 1u;
        ++stats.activeNeumannNodeCount;
      }
    }
  }
}

inline constexpr std::uint64_t kAvbdStaticGroundObjectId = 1;
inline constexpr std::uint64_t kAvbdStaticSphereObjectId = 2;
inline constexpr std::uint64_t kAvbdStaticBoxObjectId = 3;
inline constexpr std::uint32_t kAvbdSelfContactPointTriangleRow = 0;
inline constexpr std::uint32_t kAvbdSelfContactEdgeEdgeRow = 1;

//==============================================================================
void syncVbdTopologyScratch(
    std::size_t nodeCount,
    const comps::DeformableSpringModel& model,
    const comps::DeformableMeshTopology& topology,
    DeformableVbdScratch& vbdScratch)
{
  if (vbdScratch.initialized && vbdScratch.cachedNodeCount == nodeCount
      && vbdScratch.cachedEdgeCount == model.edges.size()
      && vbdScratch.cachedTetCount == topology.tetrahedra.size()) {
    return;
  }

  vbdScratch.springs.clear();
  vbdScratch.springs.reserve(model.edges.size());
  for (const auto& edge : model.edges) {
    vbdScratch.springs.push_back(
        {static_cast<std::uint32_t>(edge.nodeA),
         static_cast<std::uint32_t>(edge.nodeB),
         edge.restLength});
  }

  vbdScratch.tets.clear();
  vbdScratch.tets.reserve(topology.tetrahedra.size());
  for (const auto& tet : topology.tetrahedra) {
    const std::array<std::uint32_t, 4> vertices
        = {static_cast<std::uint32_t>(tet.nodeA),
           static_cast<std::uint32_t>(tet.nodeB),
           static_cast<std::uint32_t>(tet.nodeC),
           static_cast<std::uint32_t>(tet.nodeD)};
    const dvbd::TetRestShape rest = dvbd::makeTetRestShape(
        {topology.restPositions[tet.nodeA],
         topology.restPositions[tet.nodeB],
         topology.restPositions[tet.nodeC],
         topology.restPositions[tet.nodeD]});
    vbdScratch.tets.push_back({vertices, rest});
  }

  vbdScratch.coloring
      = dvbd::colorDeformable(nodeCount, vbdScratch.springs, vbdScratch.tets);
  vbdScratch.springAdjacency
      = dvbd::SpringAdjacency::build(nodeCount, vbdScratch.springs);
  vbdScratch.tetAdjacency
      = dvbd::TetAdjacency::build(nodeCount, vbdScratch.tets);
  vbdScratch.cachedNodeCount = nodeCount;
  vbdScratch.cachedEdgeCount = model.edges.size();
  vbdScratch.cachedTetCount = topology.tetrahedra.size();
  vbdScratch.initialized = true;
}

//==============================================================================
void primeVbdStaticContactScratch(
    std::size_t nodeCount,
    const std::vector<StaticGroundBarrier>& barriers,
    const std::vector<SphereObstacleBarrier>& sphereObstacles,
    const std::vector<BoxObstacleBarrier>& boxObstacles,
    const comps::DeformableVbdConfig& config,
    DeformableVbdScratch& vbdScratch)
{
  const bool anyStaticContact
      = !barriers.empty() || !sphereObstacles.empty() || !boxObstacles.empty();
  if (config.contactStiffness <= 0.0 || !anyStaticContact) {
    return;
  }

  vbdScratch.contactPlanes.assign(nodeCount, dvbd::ContactPlane{});
  vbdScratch.contactObjectIds.assign(nodeCount, 0);
  vbdScratch.contactFeatureIds.assign(nodeCount, 0);
}

//==============================================================================
/// Solve one implicit-Euler step for a deformable body with the
/// Vertex Block Descent inner solver: graph-colored Gauss-Seidel block
/// coordinate descent on the same variational objective the default solver
/// minimizes, warm-started at the inertial target. Handles both distance
/// springs and volumetric tetrahedra, with the World path routing tet
/// elasticity through the shared FEM kernels so the body's material choice is
/// honored. Fills `scratch.next`; the caller's write-back updates
/// positions/velocities. With `config.contactStiffness > 0` it also resolves
/// static ground/obstacle half-space contact (penalty + optional Coulomb
/// friction) plus lagged surface self-contact normal/friction penalties. Moving
/// rigid-surface CCD and static capsule obstacle barriers still fall back to
/// the default solver.
void runVbdDeformableSolve(
    entt::entity entity,
    const comps::DeformableNodeState& state,
    const comps::DeformableSpringModel& model,
    const comps::DeformableMeshTopology& topology,
    comps::DeformableSolverScratch& scratch,
    DeformableVbdScratch& vbdScratch,
    double timeStep,
    double youngsModulus,
    double poissonRatio,
    bool useFixedCorotationalTets,
    const std::vector<StaticGroundBarrier>& barriers,
    const std::vector<SphereObstacleBarrier>& sphereObstacles,
    const std::vector<BoxObstacleBarrier>& boxObstacles,
    const std::vector<DeformableSurfaceTriangle>& surfaceTriangles,
    std::span<const std::uint8_t> surfaceContactPointMask,
    double frictionCoeff,
    const comps::DeformableVbdConfig& config,
    DeformableSolverStats& stats)
{
  const std::size_t nodeCount = scratch.next.size();

  syncVbdTopologyScratch(nodeCount, model, topology, vbdScratch);

  for (std::size_t i = 0; i < nodeCount; ++i) {
    if (scratch.activeFixed[i] == 0u) {
      scratch.next[i] = scratch.inertialTargets[i];
    }
  }

  // Build the per-vertex static-contact planes from the barrier + obstacle set
  // at the warm-start position (lagged for the step). Each free vertex gets the
  // single most-binding (smallest-gap) linearized half-space among the z-up
  // ground barriers and the static sphere/box obstacles it is near: a z-up
  // plane at the ground top, or the tangent plane at the closest sphere/box
  // surface point along the outward surface normal. The half-space penalty (and
  // Coulomb friction) act only on penetration, so an armed-but-separated plane
  // is a no-op; vertices off every constraint get a zero-stiffness (inactive)
  // plane and fall freely. One plane per vertex keeps the driver contract
  // unchanged; a vertex pressed into ground and an obstacle at once resolves to
  // the nearer constraint and recovers over steps.
  const std::vector<dvbd::ContactPlane>* contactPlanes = nullptr;
  bool hasActiveContactPlanes = false;
  const bool anyStaticContact
      = !barriers.empty() || !sphereObstacles.empty() || !boxObstacles.empty();
  if (config.contactStiffness > 0.0 && anyStaticContact) {
    const double band = staticGroundBarrierActivationDistance();
    vbdScratch.contactPlanes.assign(nodeCount, dvbd::ContactPlane{});
    vbdScratch.contactObjectIds.assign(nodeCount, 0);
    vbdScratch.contactFeatureIds.assign(nodeCount, 0);
    for (std::size_t i = 0; i < nodeCount; ++i) {
      dvbd::ContactPlane& plane = vbdScratch.contactPlanes[i];
      plane.normal = Eigen::Vector3d::UnitZ();
      plane.offset = 0.0;
      plane.stiffness = 0.0;
      if (scratch.activeFixed[i] != 0u) {
        continue;
      }
      const Eigen::Vector3d& position = scratch.next[i];
      double bestGap = std::numeric_limits<double>::infinity();
      Eigen::Vector3d bestNormal = Eigen::Vector3d::UnitZ();
      double bestOffset = 0.0;
      std::uint64_t bestObjectId = 0;
      std::uint64_t bestFeatureId = 0;
      bool found = false;

      const auto groundTop = staticGroundTopAt(position, barriers);
      if (groundTop.has_value()) {
        const double gap = position.z() - *groundTop;
        if (gap < band && gap < bestGap) {
          bestGap = gap;
          bestNormal = Eigen::Vector3d::UnitZ();
          bestOffset = *groundTop;
          bestObjectId = kAvbdStaticGroundObjectId;
          bestFeatureId = 0;
          found = true;
        }
      }

      for (std::size_t s = 0; s < sphereObstacles.size(); ++s) {
        const SphereObstacleBarrier& sphere = sphereObstacles[s];
        const Eigen::Vector3d offset = position - sphere.center;
        const double centerDistance = offset.norm();
        if (!std::isfinite(centerDistance)) {
          continue;
        }
        Eigen::Vector3d normal = Eigen::Vector3d::UnitZ();
        if (centerDistance > 0.0) {
          normal = offset / centerDistance;
        }
        const double surfaceDistance = centerDistance - sphere.radius;
        if (surfaceDistance >= band) {
          continue;
        }
        const double planeOffset = normal.dot(sphere.center) + sphere.radius;
        const double gap = normal.dot(position) - planeOffset;
        if (gap < bestGap) {
          bestGap = gap;
          bestNormal = normal;
          bestOffset = planeOffset;
          bestObjectId = kAvbdStaticSphereObjectId;
          bestFeatureId = static_cast<std::uint64_t>(s);
          found = true;
        }
      }

      for (std::size_t b = 0; b < boxObstacles.size(); ++b) {
        const BoxObstacleBarrier& box = boxObstacles[b];
        Eigen::Vector3d normal;
        const double surfaceDistance
            = boxObstacleSurfaceDistance(position, box, normal);
        if (!std::isfinite(surfaceDistance) || surfaceDistance >= band) {
          continue;
        }
        const Eigen::Vector3d surfacePoint
            = position - surfaceDistance * normal;
        const double planeOffset = normal.dot(surfacePoint);
        const double gap = normal.dot(position) - planeOffset;
        if (gap < bestGap) {
          const Eigen::Vector3d localPosition
              = box.rotation.transpose() * (position - box.center);
          const std::uint64_t featureCode
              = dvbd::avbdBoxContactFeatureCode(localPosition, box.halfExtents);
          bestGap = gap;
          bestNormal = normal;
          bestOffset = planeOffset;
          bestObjectId = kAvbdStaticBoxObjectId;
          bestFeatureId = dvbd::packAvbdBoxContactFeatureId(
              static_cast<std::uint64_t>(b), featureCode);
          found = true;
        }
      }

      if (found) {
        plane.normal = bestNormal;
        plane.offset = bestOffset;
        plane.stiffness = config.contactStiffness;
        vbdScratch.contactObjectIds[i] = bestObjectId;
        vbdScratch.contactFeatureIds[i] = bestFeatureId;
        hasActiveContactPlanes = true;
      }
    }
    contactPlanes = &vbdScratch.contactPlanes;
  }

  const dvbd::LameParameters lame
      = dvbd::lameFromYoungPoisson(youngsModulus, poissonRatio);

  dvbd::BlockDescentOptions options;
  options.iterations = config.iterations;
  options.clampSpringHessian = true;
  options.convergenceDisplacement = config.convergenceDisplacement;
  options.useChebyshev = config.useChebyshev;
  options.chebyshevRho = config.chebyshevRho;
  options.rayleighDamping = config.rayleighDamping;
  // Route tetrahedra through the shared FEM elasticity kernels so a VBD body
  // applies the same hyperelastic material (Stable Neo-Hookean or
  // fixed-corotational) the default solver would, instead of the VBD-local
  // Stable Neo-Hookean copy.
  options.useFemTetKernel = true;
  options.useFixedCorotationalTets = useFixedCorotationalTets;
  // Build the lagged self-contact candidate set + per-vertex incident lists
  // from the body's swept start-to-warm-start surface motion. The IPC
  // clamped-log point-triangle / edge-edge barrier then enters each free
  // vertex's block during the colored sweeps, so a VBD surface resists folding
  // onto itself. Self-contact uses its own IPC barrier stiffness, independent
  // of the static ground/obstacle penalty stiffness; convex bodies that do not
  // fold produce no candidates and pay nothing. The post-solve self CCD limit
  // below remains the no-tunneling backstop for contacts that cross and land
  // outside the barrier band.
  const dvbd::SelfContactAdjacency* selfContact = nullptr;
  if (surfaceTriangles.size() >= 2) {
    const double dHat = selfContactBarrierActivationDistance();
    // Find candidates across the full barrier activation band (not just the
    // tight CCD min-separation the default solver uses), so VBD's lagged
    // penalty barrier engages and decelerates an approaching surface before it
    // can cross
    // -- VBD has no CCD line-search backstop of its own.
    dc::ContactCandidateOptions candidateOptions;
    candidateOptions.activationDistance = dHat;
    candidateOptions.exactDistanceFilter = true;
    candidateOptions.excludeIncidentPointTriangles = true;
    candidateOptions.excludeAdjacentEdges = true;
    dc::buildMotionAwareContactCandidatesSweep(
        state.positions,
        scratch.next,
        surfaceTriangles,
        candidateOptions,
        vbdScratch.selfContactCandidates,
        vbdScratch.selfContactSweepScratch);
    filterSurfaceContactPointCandidates(
        vbdScratch.selfContactCandidates, surfaceContactPointMask);
    vbdScratch.selfContactAdjacency.rebuild(
        nodeCount,
        vbdScratch.selfContactCandidates,
        surfaceTriangles,
        dHat * dHat,
        selfContactBarrierStiffness());
    if (!vbdScratch.selfContactCandidates.pointTriangleCandidates.empty()
        || !vbdScratch.selfContactCandidates.edgeEdgeCandidates.empty()) {
      selfContact = &vbdScratch.selfContactAdjacency;
    }
  }

  bool hasFixedNodes = false;
  for (std::size_t i = 0; i < nodeCount; ++i) {
    if (scratch.activeFixed[i] != 0u) {
      hasFixedNodes = true;
      break;
    }
  }
  const bool hasRequestedAvbdContactNormalRows
      = config.useAvbdContactNormalRows && hasActiveContactPlanes;
  const bool hasRequestedAvbdAttachmentRows
      = config.useAvbdAttachmentRows && hasFixedNodes;
  const bool hasRequestedAvbdFiniteStiffnessRows
      = config.useAvbdFiniteStiffnessRows && !vbdScratch.springs.empty();
  const bool useAvbdFrictionRows = config.useAvbdContactNormalRows
                                   && hasActiveContactPlanes
                                   && frictionCoeff > 0.0;
  const bool useAvbdSelfContactRows
      = config.useAvbdSelfContactNormalRows && selfContact != nullptr;
  const bool hasRequestedAvbdMassSpringRows
      = hasRequestedAvbdContactNormalRows || useAvbdSelfContactRows
        || hasRequestedAvbdAttachmentRows
        || hasRequestedAvbdFiniteStiffnessRows;
  const bool useAvbdSelfContactFrictionRows
      = useAvbdSelfContactRows && frictionCoeff > 0.0;
  const bool hasUnsupportedAvbdFrictionSource
      = frictionCoeff > 0.0
        && ((hasActiveContactPlanes && !useAvbdFrictionRows)
            || (selfContact != nullptr && !useAvbdSelfContactFrictionRows));
  const bool canUseAvbdMassSpringRows
      = hasRequestedAvbdMassSpringRows && vbdScratch.tets.empty()
        && !hasUnsupportedAvbdFrictionSource
        && (selfContact == nullptr || useAvbdSelfContactRows)
        && config.workerThreads <= 1 && !options.useChebyshev
        && options.rayleighDamping <= 0.0;
  const bool canUseAvbdTetMaterialRows
      = config.useAvbdFiniteStiffnessRows && !config.useAvbdContactNormalRows
        && !config.useAvbdAttachmentRows && contactPlanes == nullptr
        && vbdScratch.springs.empty() && !vbdScratch.tets.empty()
        && !hasUnsupportedAvbdFrictionSource && config.workerThreads <= 1
        && !options.useChebyshev && options.rayleighDamping <= 0.0;

  dvbd::BlockDescentStats result;
  const auto projectAvbdSelfContactFrictionWarmStarts =
      [](dvbd::AvbdScalarRowInventory& inventory,
         std::vector<dvbd::AvbdSelfContactFrictionRow>& rows,
         const std::vector<DeformableVbdScratch::AvbdFrictionWarmStartRecord<
             dvbd::AvbdSelfContactFrictionRow>>& previousRows) {
        for (std::size_t i = 0; i + 1 < inventory.size() && i + 1 < rows.size();
             i += 2) {
          dvbd::AvbdScalarRowRecord& firstRecord = inventory[i];
          dvbd::AvbdScalarRowRecord& secondRecord = inventory[i + 1];
          dvbd::AvbdSelfContactFrictionRow& firstRow = rows[i];
          dvbd::AvbdSelfContactFrictionRow& secondRow = rows[i + 1];
          auto expectedSecondKey = firstRecord.descriptor.key;
          expectedSecondKey.axis = 1;
          if (firstRecord.descriptor.key.axis != 0
              || secondRecord.descriptor.key != expectedSecondKey
              || firstRow.axis != 0 || secondRow.axis != 1
              || !dvbd::avbdSelfContactSameFrictionPrimitive(
                  firstRow, secondRow)) {
            continue;
          }

          const auto* previousFirst = findAvbdFrictionWarmStartRow(
              previousRows, firstRecord.descriptor.key);
          const auto* previousSecond = findAvbdFrictionWarmStartRow(
              previousRows, secondRecord.descriptor.key);
          if (previousFirst == nullptr || previousSecond == nullptr) {
            continue;
          }

          const Eigen::Vector2d projected
              = dvbd::projectAvbdSelfContactFrictionDualToTangentPair(
                  firstRow.state.lambda,
                  secondRow.state.lambda,
                  *previousFirst,
                  *previousSecond,
                  firstRow,
                  secondRow);
          firstRow.state.lambda
              = dvbd::clampAvbdRowForce(projected.x(), firstRow.bounds);
          secondRow.state.lambda
              = dvbd::clampAvbdRowForce(projected.y(), secondRow.bounds);
        }
      };
  if (canUseAvbdMassSpringRows) {
    const auto bodyId = static_cast<std::uint64_t>(entt::to_integral(entity));
    vbdScratch.avbdTetDescriptors.clear();
    vbdScratch.avbdTetRows.clear();
    vbdScratch.avbdTetInventory.records().clear();

    vbdScratch.avbdSelfContactDescriptors.clear();
    if (useAvbdSelfContactRows) {
      vbdScratch.avbdSelfContactDescriptors.reserve(
          vbdScratch.selfContactCandidates.pointTriangleCandidates.size()
          + vbdScratch.selfContactCandidates.edgeEdgeCandidates.size());
      for (const dc::PointTriangleCandidate& candidate :
           vbdScratch.selfContactCandidates.pointTriangleCandidates) {
        dvbd::AvbdScalarRowDescriptor descriptor;
        descriptor.key.role = dvbd::AvbdScalarRowRole::SelfContactNormal;
        descriptor.key.objectA = bodyId;
        descriptor.key.objectB = bodyId;
        descriptor.key.featureA = static_cast<std::uint64_t>(candidate.point);
        descriptor.key.featureB
            = static_cast<std::uint64_t>(candidate.triangle);
        descriptor.key.row = kAvbdSelfContactPointTriangleRow;
        descriptor.key.axis = 0;
        descriptor.kind = dvbd::AvbdScalarRowKind::HardConstraint;
        descriptor.bounds = dvbd::avbdContactNormalBounds();
        descriptor.startStiffness = selfContactBarrierStiffness();
        descriptor.maxStiffness = config.avbdMaxStiffness;
        vbdScratch.avbdSelfContactDescriptors.push_back(descriptor);
      }
      for (const dc::EdgeEdgeCandidate& candidate :
           vbdScratch.selfContactCandidates.edgeEdgeCandidates) {
        dvbd::AvbdScalarRowDescriptor descriptor;
        descriptor.key.role = dvbd::AvbdScalarRowRole::SelfContactNormal;
        descriptor.key.objectA = bodyId;
        descriptor.key.objectB = bodyId;
        descriptor.key.featureA = static_cast<std::uint64_t>(candidate.edgeA);
        descriptor.key.featureB = static_cast<std::uint64_t>(candidate.edgeB);
        descriptor.key.row = kAvbdSelfContactEdgeEdgeRow;
        descriptor.key.axis = 0;
        descriptor.kind = dvbd::AvbdScalarRowKind::HardConstraint;
        descriptor.bounds = dvbd::avbdContactNormalBounds();
        descriptor.startStiffness = selfContactBarrierStiffness();
        descriptor.maxStiffness = config.avbdMaxStiffness;
        vbdScratch.avbdSelfContactDescriptors.push_back(descriptor);
      }
    }

    vbdScratch.avbdContactDescriptors.clear();
    if (config.useAvbdContactNormalRows && contactPlanes != nullptr) {
      for (std::size_t i = 0; i < nodeCount; ++i) {
        const dvbd::ContactPlane& plane = (*contactPlanes)[i];
        if (plane.stiffness <= 0.0) {
          continue;
        }
        dvbd::AvbdScalarRowDescriptor descriptor;
        descriptor.key.role = dvbd::AvbdScalarRowRole::ContactNormal;
        descriptor.key.objectA = bodyId;
        descriptor.key.objectB = vbdScratch.contactObjectIds[i];
        descriptor.key.featureA = static_cast<std::uint64_t>(i);
        descriptor.key.featureB = vbdScratch.contactFeatureIds[i];
        descriptor.key.row = 0;
        descriptor.key.axis = 0;
        descriptor.kind = dvbd::AvbdScalarRowKind::HardConstraint;
        descriptor.bounds = dvbd::avbdContactNormalBounds();
        descriptor.startStiffness = plane.stiffness;
        descriptor.maxStiffness = config.avbdMaxStiffness;
        vbdScratch.avbdContactDescriptors.push_back(descriptor);
      }
    }

    vbdScratch.avbdSolveFixed = scratch.activeFixed;
    vbdScratch.avbdAttachmentDescriptors.clear();
    if (config.useAvbdAttachmentRows) {
      for (std::size_t i = 0; i < nodeCount; ++i) {
        if (scratch.activeFixed[i] == 0u) {
          continue;
        }

        vbdScratch.avbdSolveFixed[i] = 0u;
        scratch.next[i] = scratch.inertialTargets[i];

        for (std::uint8_t axis = 0; axis < 3; ++axis) {
          dvbd::AvbdScalarRowDescriptor descriptor;
          descriptor.key.role = dvbd::AvbdScalarRowRole::Attachment;
          descriptor.key.objectA = bodyId;
          descriptor.key.featureA = static_cast<std::uint64_t>(i);
          descriptor.key.row = 0;
          descriptor.key.axis = axis;
          descriptor.kind = dvbd::AvbdScalarRowKind::HardConstraint;
          descriptor.startStiffness = config.avbdAttachmentStiffness;
          descriptor.maxStiffness = config.avbdMaxStiffness;
          vbdScratch.avbdAttachmentDescriptors.push_back(descriptor);
        }
      }
    }

    dvbd::AvbdRowWarmStartOptions warmStartOptions;
    warmStartOptions.alpha = config.avbdAlpha;
    warmStartOptions.gamma = config.avbdGamma;
    warmStartOptions.maxStiffness = config.avbdMaxStiffness;
    vbdScratch.avbdSelfContactInventory.reserve(
        vbdScratch.avbdSelfContactDescriptors.size());
    vbdScratch.avbdSelfContactInventory.syncActiveRows(
        vbdScratch.avbdSelfContactDescriptors, warmStartOptions);
    vbdScratch.avbdContactInventory.reserve(
        vbdScratch.avbdContactDescriptors.size());
    vbdScratch.avbdContactInventory.syncActiveRows(
        vbdScratch.avbdContactDescriptors, warmStartOptions);
    vbdScratch.avbdAttachmentInventory.reserve(
        vbdScratch.avbdAttachmentDescriptors.size());
    vbdScratch.avbdAttachmentInventory.syncActiveRows(
        vbdScratch.avbdAttachmentDescriptors, warmStartOptions);

    const auto assignSelfContactPrimitive
        = [&](const dvbd::AvbdScalarRowDescriptor& descriptor, auto& row) {
            if (descriptor.key.row == kAvbdSelfContactPointTriangleRow) {
              const auto point
                  = static_cast<std::size_t>(descriptor.key.featureA);
              const auto triangleIndex
                  = static_cast<std::size_t>(descriptor.key.featureB);
              const DeformableSurfaceTriangle& triangle
                  = surfaceTriangles[triangleIndex];
              row.nodes
                  = {static_cast<std::uint32_t>(point),
                     static_cast<std::uint32_t>(triangle.nodeA),
                     static_cast<std::uint32_t>(triangle.nodeB),
                     static_cast<std::uint32_t>(triangle.nodeC)};
              row.isEdgeEdge = false;
            } else {
              const auto edgeAIndex
                  = static_cast<std::size_t>(descriptor.key.featureA);
              const auto edgeBIndex
                  = static_cast<std::size_t>(descriptor.key.featureB);
              const dc::SurfaceEdge& edgeA
                  = vbdScratch.selfContactCandidates.surfaceEdges[edgeAIndex];
              const dc::SurfaceEdge& edgeB
                  = vbdScratch.selfContactCandidates.surfaceEdges[edgeBIndex];
              row.nodes
                  = {static_cast<std::uint32_t>(edgeA.nodeA),
                     static_cast<std::uint32_t>(edgeA.nodeB),
                     static_cast<std::uint32_t>(edgeB.nodeA),
                     static_cast<std::uint32_t>(edgeB.nodeB)};
              row.isEdgeEdge = true;
            }
          };

    vbdScratch.avbdSelfContactRows.clear();
    vbdScratch.avbdSelfContactRows.reserve(
        vbdScratch.avbdSelfContactInventory.size());
    for (const dvbd::AvbdScalarRowRecord& record :
         vbdScratch.avbdSelfContactInventory.records()) {
      dvbd::AvbdSelfContactNormalRow row;
      row.state = record.state;
      row.squaredActivationDistance
          = vbdScratch.selfContactAdjacency.squaredActivationDistance;
      row.bounds = record.descriptor.bounds;
      assignSelfContactPrimitive(record.descriptor, row);
      row.previousConstraintValue
          = dvbd::avbdSelfContactNormalConstraintValue(row, state.positions);
      vbdScratch.avbdSelfContactRows.push_back(row);
    }

    rebuildAvbdFrictionWarmStartLookup(
        vbdScratch.previousAvbdSelfContactFrictionWarmStarts,
        vbdScratch.avbdSelfContactFrictionDescriptors,
        vbdScratch.avbdSelfContactFrictionRows);
    vbdScratch.avbdSelfContactFrictionDescriptors.clear();
    if (useAvbdSelfContactFrictionRows) {
      vbdScratch.avbdSelfContactFrictionDescriptors.reserve(
          2 * vbdScratch.avbdSelfContactRows.size());
      for (std::size_t i = 0; i < vbdScratch.avbdSelfContactRows.size(); ++i) {
        const dvbd::AvbdScalarRowRecord& normalRecord
            = vbdScratch.avbdSelfContactInventory[i];
        const dvbd::AvbdSelfContactNormalRow& normalRow
            = vbdScratch.avbdSelfContactRows[i];
        const double laggedNormalForce = std::max(0.0, normalRow.state.lambda);
        const double forceLimit = frictionCoeff * laggedNormalForce;
        for (std::uint8_t axis = 0; axis < 2; ++axis) {
          dvbd::AvbdScalarRowDescriptor descriptor;
          descriptor.key.role = dvbd::AvbdScalarRowRole::FrictionTangent;
          descriptor.key.objectA = normalRecord.descriptor.key.objectA;
          descriptor.key.objectB = normalRecord.descriptor.key.objectB;
          descriptor.key.featureA = normalRecord.descriptor.key.featureA;
          descriptor.key.featureB = normalRecord.descriptor.key.featureB;
          descriptor.key.row = normalRecord.descriptor.key.row;
          descriptor.key.axis = axis;
          descriptor.kind = dvbd::AvbdScalarRowKind::HardConstraint;
          descriptor.bounds = dvbd::avbdFrictionTangentBounds(forceLimit);
          descriptor.startStiffness = selfContactBarrierStiffness();
          descriptor.maxStiffness = config.avbdMaxStiffness;
          vbdScratch.avbdSelfContactFrictionDescriptors.push_back(descriptor);
        }
      }
    }
    vbdScratch.avbdSelfContactFrictionInventory.reserve(
        vbdScratch.avbdSelfContactFrictionDescriptors.size());
    vbdScratch.avbdSelfContactFrictionInventory.syncActiveRows(
        vbdScratch.avbdSelfContactFrictionDescriptors, warmStartOptions);

    vbdScratch.avbdSelfContactFrictionRows.clear();
    vbdScratch.avbdSelfContactFrictionRows.reserve(
        vbdScratch.avbdSelfContactFrictionInventory.size());
    for (const dvbd::AvbdScalarRowRecord& record :
         vbdScratch.avbdSelfContactFrictionInventory.records()) {
      dvbd::AvbdSelfContactFrictionRow row;
      row.state = record.state;
      row.axis = record.descriptor.key.axis;
      row.bounds = record.descriptor.bounds;
      assignSelfContactPrimitive(record.descriptor, row);
      for (std::uint8_t i = 0; i < 4; ++i) {
        row.stepStartPositions[i] = state.positions[row.nodes[i]];
      }
      row.previousConstraintValue
          = dvbd::avbdSelfContactFrictionConstraintValue(row, state.positions);
      vbdScratch.avbdSelfContactFrictionRows.push_back(row);
    }
    projectAvbdSelfContactFrictionWarmStarts(
        vbdScratch.avbdSelfContactFrictionInventory,
        vbdScratch.avbdSelfContactFrictionRows,
        vbdScratch.previousAvbdSelfContactFrictionWarmStarts);

    vbdScratch.avbdContactRows.clear();
    vbdScratch.avbdContactRows.reserve(vbdScratch.avbdContactInventory.size());
    for (const dvbd::AvbdScalarRowRecord& record :
         vbdScratch.avbdContactInventory.records()) {
      const auto vertex
          = static_cast<std::uint32_t>(record.descriptor.key.featureA);
      const dvbd::ContactPlane& plane = (*contactPlanes)[vertex];
      vbdScratch.avbdContactRows.push_back(
          dvbd::AvbdHalfSpaceContactRow{
              vertex,
              plane,
              record.state,
              dvbd::avbdHalfSpaceContactConstraintValue(
                  state.positions[vertex], plane),
              record.descriptor.bounds});
    }

    rebuildAvbdFrictionWarmStartLookup(
        vbdScratch.previousAvbdFrictionWarmStarts,
        vbdScratch.avbdFrictionDescriptors,
        vbdScratch.avbdFrictionRows);
    vbdScratch.avbdFrictionDescriptors.clear();
    if (useAvbdFrictionRows) {
      vbdScratch.avbdFrictionDescriptors.reserve(
          2 * vbdScratch.avbdContactRows.size());
      for (std::size_t i = 0; i < vbdScratch.avbdContactRows.size(); ++i) {
        const dvbd::AvbdHalfSpaceContactRow& contactRow
            = vbdScratch.avbdContactRows[i];
        const std::uint32_t vertex = contactRow.vertex;
        const double laggedNormalForce = std::max(0.0, contactRow.state.lambda);
        const double forceLimit = frictionCoeff * laggedNormalForce;
        for (std::uint8_t axis = 0; axis < 2; ++axis) {
          dvbd::AvbdScalarRowDescriptor descriptor;
          descriptor.key.role = dvbd::AvbdScalarRowRole::FrictionTangent;
          descriptor.key.objectA = bodyId;
          descriptor.key.objectB = vbdScratch.contactObjectIds[vertex];
          descriptor.key.featureA = static_cast<std::uint64_t>(vertex);
          descriptor.key.featureB = vbdScratch.contactFeatureIds[vertex];
          descriptor.key.row = 0;
          descriptor.key.axis = axis;
          descriptor.kind = dvbd::AvbdScalarRowKind::HardConstraint;
          descriptor.bounds = dvbd::avbdFrictionTangentBounds(forceLimit);
          descriptor.startStiffness = contactRow.plane.stiffness;
          descriptor.maxStiffness = config.avbdMaxStiffness;
          vbdScratch.avbdFrictionDescriptors.push_back(descriptor);
        }
      }
    }
    vbdScratch.avbdFrictionInventory.reserve(
        vbdScratch.avbdFrictionDescriptors.size());
    vbdScratch.avbdFrictionInventory.syncActiveRows(
        vbdScratch.avbdFrictionDescriptors, warmStartOptions);
    for (std::size_t i = 0; i + 1 < vbdScratch.avbdFrictionInventory.size();
         i += 2) {
      dvbd::AvbdScalarRowRecord& firstRecord
          = vbdScratch.avbdFrictionInventory[i];
      dvbd::AvbdScalarRowRecord& secondRecord
          = vbdScratch.avbdFrictionInventory[i + 1];
      auto expectedSecondKey = firstRecord.descriptor.key;
      expectedSecondKey.axis = 1;
      if (firstRecord.descriptor.key.axis != 0
          || secondRecord.descriptor.key != expectedSecondKey) {
        continue;
      }
      const auto* previousFirst = findAvbdFrictionWarmStartRow(
          vbdScratch.previousAvbdFrictionWarmStarts,
          firstRecord.descriptor.key);
      const auto* previousSecond = findAvbdFrictionWarmStartRow(
          vbdScratch.previousAvbdFrictionWarmStarts,
          secondRecord.descriptor.key);
      if (previousFirst == nullptr || previousSecond == nullptr) {
        continue;
      }

      const auto vertex
          = static_cast<std::uint32_t>(firstRecord.descriptor.key.featureA);
      const dvbd::ContactPlane& plane = (*contactPlanes)[vertex];
      const dc::Matrix3x2d basis
          = dc::detail::fallbackBasisFromNormal(plane.normal);
      const Eigen::Vector2d projected
          = dvbd::projectAvbdFrictionDualToTangentPair(
              firstRecord.state.lambda,
              secondRecord.state.lambda,
              previousFirst->axis,
              previousSecond->axis,
              basis.col(0),
              basis.col(1));
      firstRecord.state.lambda = dvbd::clampAvbdRowForce(
          projected.x(), firstRecord.descriptor.bounds);
      secondRecord.state.lambda = dvbd::clampAvbdRowForce(
          projected.y(), secondRecord.descriptor.bounds);
    }

    vbdScratch.avbdFrictionRows.clear();
    vbdScratch.avbdFrictionRows.reserve(
        vbdScratch.avbdFrictionInventory.size());
    for (const dvbd::AvbdScalarRowRecord& record :
         vbdScratch.avbdFrictionInventory.records()) {
      const auto vertex
          = static_cast<std::uint32_t>(record.descriptor.key.featureA);
      const std::uint8_t axisId = record.descriptor.key.axis;
      const dvbd::ContactPlane& plane = (*contactPlanes)[vertex];
      const dc::Matrix3x2d basis
          = dc::detail::fallbackBasisFromNormal(plane.normal);
      const Eigen::Vector3d axis = basis.col(axisId < 2 ? axisId : 0);
      vbdScratch.avbdFrictionRows.push_back(
          dvbd::AvbdHalfSpaceFrictionRow{
              vertex,
              state.positions[vertex],
              axis,
              record.state,
              dvbd::avbdHalfSpaceFrictionConstraintValue(
                  state.positions[vertex], state.positions[vertex], axis),
              record.descriptor.bounds});
    }

    const bool hasRestTargets = topology.restPositions.size() == nodeCount;
    const bool hasDirichletMask = scratch.activeDirichlet.size() == nodeCount;
    vbdScratch.avbdAttachmentRows.clear();
    vbdScratch.avbdAttachmentRows.reserve(
        vbdScratch.avbdAttachmentInventory.size());
    for (const dvbd::AvbdScalarRowRecord& record :
         vbdScratch.avbdAttachmentInventory.records()) {
      const auto vertex
          = static_cast<std::uint32_t>(record.descriptor.key.featureA);
      const std::uint8_t axisId = record.descriptor.key.axis;
      const Eigen::Vector3d axis = dvbd::canonicalAvbdAttachmentAxis(axisId);
      const bool isScriptedDirichlet
          = hasDirichletMask && scratch.activeDirichlet[vertex] != 0u;
      const Eigen::Vector3d target = (state.fixed[vertex] != 0u
                                      && !isScriptedDirichlet && hasRestTargets)
                                         ? topology.restPositions[vertex]
                                         : state.positions[vertex];
      vbdScratch.avbdAttachmentRows.push_back(
          dvbd::AvbdPointAttachmentRow{
              vertex,
              target,
              axis,
              record.state,
              dvbd::avbdPointAttachmentConstraintValue(
                  scratch.previousStepPositions[vertex], target, axis),
              record.descriptor.bounds});
    }

    vbdScratch.avbdSpringDescriptors.clear();
    if (config.useAvbdFiniteStiffnessRows) {
      vbdScratch.avbdSpringDescriptors.reserve(vbdScratch.springs.size());
      for (std::size_t i = 0; i < vbdScratch.springs.size(); ++i) {
        dvbd::AvbdScalarRowDescriptor descriptor;
        descriptor.key.role = dvbd::AvbdScalarRowRole::DeformableSpring;
        descriptor.key.objectA = bodyId;
        descriptor.key.featureA = static_cast<std::uint64_t>(i);
        descriptor.kind = dvbd::AvbdScalarRowKind::FiniteStiffness;
        descriptor.startStiffness = config.avbdFiniteStiffnessStart;
        descriptor.materialStiffness = model.stiffness;
        descriptor.maxStiffness = config.avbdMaxStiffness;
        vbdScratch.avbdSpringDescriptors.push_back(descriptor);
      }
    }
    vbdScratch.avbdSpringInventory.reserve(
        vbdScratch.avbdSpringDescriptors.size());
    vbdScratch.avbdSpringInventory.syncActiveRows(
        vbdScratch.avbdSpringDescriptors, warmStartOptions);

    vbdScratch.avbdSpringRows.clear();
    vbdScratch.avbdSpringRows.reserve(vbdScratch.avbdSpringInventory.size());
    for (const dvbd::AvbdScalarRowRecord& record :
         vbdScratch.avbdSpringInventory.records()) {
      vbdScratch.avbdSpringRows.push_back(
          dvbd::AvbdSpringFiniteStiffnessRow{
              static_cast<std::uint32_t>(record.descriptor.key.featureA),
              record.state,
              dvbd::maxAvbdDescriptorStiffness(
                  record.descriptor, warmStartOptions)});
    }

    dvbd::AvbdHalfSpaceContactOptions contactOptions;
    contactOptions.alpha = config.avbdAlpha;
    contactOptions.beta = config.avbdBeta;
    contactOptions.maxStiffness = config.avbdMaxStiffness;
    dvbd::AvbdPointAttachmentOptions attachmentOptions;
    attachmentOptions.alpha = config.avbdAlpha;
    attachmentOptions.beta = config.avbdBeta;
    attachmentOptions.maxStiffness = config.avbdMaxStiffness;
    dvbd::AvbdSpringFiniteStiffnessOptions springOptions;
    springOptions.beta = config.avbdBeta;
    springOptions.maxStiffness = config.avbdMaxStiffness;
    dvbd::AvbdHalfSpaceFrictionOptions frictionOptions;
    frictionOptions.alpha = config.avbdAlpha;
    frictionOptions.beta = config.avbdBeta;
    frictionOptions.maxStiffness = config.avbdMaxStiffness;
    dvbd::AvbdSelfContactNormalOptions selfContactOptions;
    selfContactOptions.alpha = config.avbdAlpha;
    selfContactOptions.beta = config.avbdBeta;
    selfContactOptions.maxStiffness = config.avbdMaxStiffness;
    dvbd::AvbdSelfContactFrictionOptions selfContactFrictionOptions;
    selfContactFrictionOptions.alpha = config.avbdAlpha;
    selfContactFrictionOptions.beta = config.avbdBeta;
    selfContactFrictionOptions.maxStiffness = config.avbdMaxStiffness;
    result = dvbd::blockDescentMassSpringAvbdRows(
        scratch.next,
        state.masses,
        vbdScratch.avbdSolveFixed,
        scratch.inertialTargets,
        vbdScratch.springs,
        model.stiffness,
        timeStep,
        vbdScratch.avbdContactRows,
        vbdScratch.avbdAttachmentRows,
        vbdScratch.avbdSpringRows,
        vbdScratch.coloring,
        vbdScratch.springAdjacency,
        options,
        contactOptions,
        attachmentOptions,
        springOptions,
        &vbdScratch.avbdFrictionRows,
        &frictionOptions,
        &vbdScratch.avbdSelfContactRows,
        useAvbdSelfContactRows ? selfContact : nullptr,
        &selfContactOptions,
        useAvbdSelfContactFrictionRows ? &vbdScratch.avbdSelfContactFrictionRows
                                       : nullptr,
        useAvbdSelfContactFrictionRows ? &selfContactFrictionOptions : nullptr);

    for (std::size_t i = 0; i < vbdScratch.avbdContactRows.size(); ++i) {
      vbdScratch.avbdContactInventory[i].state
          = vbdScratch.avbdContactRows[i].state;
    }
    for (std::size_t i = 0; i < vbdScratch.avbdAttachmentRows.size(); ++i) {
      vbdScratch.avbdAttachmentInventory[i].state
          = vbdScratch.avbdAttachmentRows[i].state;
    }
    for (std::size_t i = 0; i < vbdScratch.avbdSpringRows.size(); ++i) {
      vbdScratch.avbdSpringInventory[i].state
          = vbdScratch.avbdSpringRows[i].state;
    }
    for (std::size_t i = 0; i < vbdScratch.avbdFrictionRows.size(); ++i) {
      vbdScratch.avbdFrictionInventory[i].state
          = vbdScratch.avbdFrictionRows[i].state;
    }
    for (std::size_t i = 0; i < vbdScratch.avbdSelfContactRows.size(); ++i) {
      vbdScratch.avbdSelfContactInventory[i].state
          = vbdScratch.avbdSelfContactRows[i].state;
    }
    for (std::size_t i = 0; i < vbdScratch.avbdSelfContactFrictionRows.size();
         ++i) {
      vbdScratch.avbdSelfContactFrictionInventory[i].state
          = vbdScratch.avbdSelfContactFrictionRows[i].state;
    }
    stats.vbdAvbdContactNormalRows += vbdScratch.avbdContactRows.size();
    stats.vbdAvbdSelfContactNormalRows += vbdScratch.avbdSelfContactRows.size();
    stats.vbdAvbdFrictionTangentRows
        += vbdScratch.avbdFrictionRows.size()
           + vbdScratch.avbdSelfContactFrictionRows.size();
    stats.vbdAvbdAttachmentRows += vbdScratch.avbdAttachmentRows.size();
    stats.vbdAvbdFiniteStiffnessRows += vbdScratch.avbdSpringRows.size();
  } else if (canUseAvbdTetMaterialRows) {
    const auto bodyId = static_cast<std::uint64_t>(entt::to_integral(entity));

    vbdScratch.avbdContactDescriptors.clear();
    vbdScratch.avbdContactRows.clear();
    vbdScratch.avbdContactInventory.records().clear();
    vbdScratch.avbdFrictionDescriptors.clear();
    vbdScratch.avbdFrictionRows.clear();
    vbdScratch.avbdFrictionInventory.records().clear();
    vbdScratch.avbdSelfContactDescriptors.clear();
    vbdScratch.avbdSelfContactRows.clear();
    rebuildAvbdFrictionWarmStartLookup(
        vbdScratch.previousAvbdSelfContactFrictionWarmStarts,
        vbdScratch.avbdSelfContactFrictionDescriptors,
        vbdScratch.avbdSelfContactFrictionRows);
    vbdScratch.avbdSelfContactFrictionDescriptors.clear();
    vbdScratch.avbdSelfContactFrictionRows.clear();
    vbdScratch.avbdAttachmentDescriptors.clear();
    vbdScratch.avbdAttachmentRows.clear();
    vbdScratch.avbdAttachmentInventory.records().clear();
    vbdScratch.avbdSpringDescriptors.clear();
    vbdScratch.avbdSpringRows.clear();
    vbdScratch.avbdSpringInventory.records().clear();
    vbdScratch.avbdSolveFixed.clear();

    dvbd::AvbdRowWarmStartOptions warmStartOptions;
    warmStartOptions.alpha = config.avbdAlpha;
    warmStartOptions.gamma = config.avbdGamma;
    warmStartOptions.maxStiffness = std::min(1.0, config.avbdMaxStiffness);

    vbdScratch.avbdTetDescriptors.clear();
    vbdScratch.avbdTetDescriptors.reserve(vbdScratch.tets.size());
    for (std::size_t i = 0; i < vbdScratch.tets.size(); ++i) {
      dvbd::AvbdScalarRowDescriptor descriptor;
      descriptor.key.role = dvbd::AvbdScalarRowRole::DeformableTet;
      descriptor.key.objectA = bodyId;
      descriptor.key.featureA = static_cast<std::uint64_t>(i);
      descriptor.kind = dvbd::AvbdScalarRowKind::FiniteStiffness;
      descriptor.startStiffness = config.avbdFiniteStiffnessStart;
      descriptor.materialStiffness = 1.0;
      descriptor.maxStiffness = 1.0;
      vbdScratch.avbdTetDescriptors.push_back(descriptor);
    }
    vbdScratch.avbdTetInventory.reserve(vbdScratch.avbdTetDescriptors.size());
    vbdScratch.avbdTetInventory.syncActiveRows(
        vbdScratch.avbdTetDescriptors, warmStartOptions);

    vbdScratch.avbdTetRows.clear();
    vbdScratch.avbdTetRows.reserve(vbdScratch.avbdTetInventory.size());
    for (const dvbd::AvbdScalarRowRecord& record :
         vbdScratch.avbdTetInventory.records()) {
      vbdScratch.avbdTetRows.push_back(
          dvbd::AvbdTetMaterialFiniteStiffnessRow{
              static_cast<std::uint32_t>(record.descriptor.key.featureA),
              record.state,
              dvbd::maxAvbdDescriptorStiffness(
                  record.descriptor, warmStartOptions)});
    }

    dvbd::AvbdRowWarmStartOptions hardRowWarmStartOptions;
    hardRowWarmStartOptions.alpha = config.avbdAlpha;
    hardRowWarmStartOptions.gamma = config.avbdGamma;
    hardRowWarmStartOptions.maxStiffness = config.avbdMaxStiffness;

    vbdScratch.avbdSelfContactDescriptors.clear();
    if (useAvbdSelfContactRows) {
      vbdScratch.avbdSelfContactDescriptors.reserve(
          vbdScratch.selfContactCandidates.pointTriangleCandidates.size()
          + vbdScratch.selfContactCandidates.edgeEdgeCandidates.size());
      for (const dc::PointTriangleCandidate& candidate :
           vbdScratch.selfContactCandidates.pointTriangleCandidates) {
        dvbd::AvbdScalarRowDescriptor descriptor;
        descriptor.key.role = dvbd::AvbdScalarRowRole::SelfContactNormal;
        descriptor.key.objectA = bodyId;
        descriptor.key.objectB = bodyId;
        descriptor.key.featureA = static_cast<std::uint64_t>(candidate.point);
        descriptor.key.featureB
            = static_cast<std::uint64_t>(candidate.triangle);
        descriptor.key.row = kAvbdSelfContactPointTriangleRow;
        descriptor.key.axis = 0;
        descriptor.kind = dvbd::AvbdScalarRowKind::HardConstraint;
        descriptor.bounds = dvbd::avbdContactNormalBounds();
        descriptor.startStiffness = selfContactBarrierStiffness();
        descriptor.maxStiffness = config.avbdMaxStiffness;
        vbdScratch.avbdSelfContactDescriptors.push_back(descriptor);
      }
      for (const dc::EdgeEdgeCandidate& candidate :
           vbdScratch.selfContactCandidates.edgeEdgeCandidates) {
        dvbd::AvbdScalarRowDescriptor descriptor;
        descriptor.key.role = dvbd::AvbdScalarRowRole::SelfContactNormal;
        descriptor.key.objectA = bodyId;
        descriptor.key.objectB = bodyId;
        descriptor.key.featureA = static_cast<std::uint64_t>(candidate.edgeA);
        descriptor.key.featureB = static_cast<std::uint64_t>(candidate.edgeB);
        descriptor.key.row = kAvbdSelfContactEdgeEdgeRow;
        descriptor.key.axis = 0;
        descriptor.kind = dvbd::AvbdScalarRowKind::HardConstraint;
        descriptor.bounds = dvbd::avbdContactNormalBounds();
        descriptor.startStiffness = selfContactBarrierStiffness();
        descriptor.maxStiffness = config.avbdMaxStiffness;
        vbdScratch.avbdSelfContactDescriptors.push_back(descriptor);
      }
    }
    vbdScratch.avbdSelfContactInventory.reserve(
        vbdScratch.avbdSelfContactDescriptors.size());
    vbdScratch.avbdSelfContactInventory.syncActiveRows(
        vbdScratch.avbdSelfContactDescriptors, hardRowWarmStartOptions);

    const auto assignSelfContactPrimitive
        = [&](const dvbd::AvbdScalarRowDescriptor& descriptor, auto& row) {
            if (descriptor.key.row == kAvbdSelfContactPointTriangleRow) {
              const auto point
                  = static_cast<std::size_t>(descriptor.key.featureA);
              const auto triangleIndex
                  = static_cast<std::size_t>(descriptor.key.featureB);
              const DeformableSurfaceTriangle& triangle
                  = surfaceTriangles[triangleIndex];
              row.nodes
                  = {static_cast<std::uint32_t>(point),
                     static_cast<std::uint32_t>(triangle.nodeA),
                     static_cast<std::uint32_t>(triangle.nodeB),
                     static_cast<std::uint32_t>(triangle.nodeC)};
              row.isEdgeEdge = false;
            } else {
              const auto edgeAIndex
                  = static_cast<std::size_t>(descriptor.key.featureA);
              const auto edgeBIndex
                  = static_cast<std::size_t>(descriptor.key.featureB);
              const dc::SurfaceEdge& edgeA
                  = vbdScratch.selfContactCandidates.surfaceEdges[edgeAIndex];
              const dc::SurfaceEdge& edgeB
                  = vbdScratch.selfContactCandidates.surfaceEdges[edgeBIndex];
              row.nodes
                  = {static_cast<std::uint32_t>(edgeA.nodeA),
                     static_cast<std::uint32_t>(edgeA.nodeB),
                     static_cast<std::uint32_t>(edgeB.nodeA),
                     static_cast<std::uint32_t>(edgeB.nodeB)};
              row.isEdgeEdge = true;
            }
          };

    vbdScratch.avbdSelfContactRows.clear();
    vbdScratch.avbdSelfContactRows.reserve(
        vbdScratch.avbdSelfContactInventory.size());
    for (const dvbd::AvbdScalarRowRecord& record :
         vbdScratch.avbdSelfContactInventory.records()) {
      dvbd::AvbdSelfContactNormalRow row;
      row.state = record.state;
      row.squaredActivationDistance
          = vbdScratch.selfContactAdjacency.squaredActivationDistance;
      row.bounds = record.descriptor.bounds;
      assignSelfContactPrimitive(record.descriptor, row);
      row.previousConstraintValue
          = dvbd::avbdSelfContactNormalConstraintValue(row, state.positions);
      vbdScratch.avbdSelfContactRows.push_back(row);
    }

    vbdScratch.avbdSelfContactFrictionDescriptors.clear();
    if (useAvbdSelfContactFrictionRows) {
      vbdScratch.avbdSelfContactFrictionDescriptors.reserve(
          2 * vbdScratch.avbdSelfContactRows.size());
      for (std::size_t i = 0; i < vbdScratch.avbdSelfContactRows.size(); ++i) {
        const dvbd::AvbdScalarRowRecord& normalRecord
            = vbdScratch.avbdSelfContactInventory[i];
        const dvbd::AvbdSelfContactNormalRow& normalRow
            = vbdScratch.avbdSelfContactRows[i];
        const double laggedNormalForce = std::max(0.0, normalRow.state.lambda);
        const double forceLimit = frictionCoeff * laggedNormalForce;
        for (std::uint8_t axis = 0; axis < 2; ++axis) {
          dvbd::AvbdScalarRowDescriptor descriptor;
          descriptor.key.role = dvbd::AvbdScalarRowRole::FrictionTangent;
          descriptor.key.objectA = normalRecord.descriptor.key.objectA;
          descriptor.key.objectB = normalRecord.descriptor.key.objectB;
          descriptor.key.featureA = normalRecord.descriptor.key.featureA;
          descriptor.key.featureB = normalRecord.descriptor.key.featureB;
          descriptor.key.row = normalRecord.descriptor.key.row;
          descriptor.key.axis = axis;
          descriptor.kind = dvbd::AvbdScalarRowKind::HardConstraint;
          descriptor.bounds = dvbd::avbdFrictionTangentBounds(forceLimit);
          descriptor.startStiffness = selfContactBarrierStiffness();
          descriptor.maxStiffness = config.avbdMaxStiffness;
          vbdScratch.avbdSelfContactFrictionDescriptors.push_back(descriptor);
        }
      }
    }
    vbdScratch.avbdSelfContactFrictionInventory.reserve(
        vbdScratch.avbdSelfContactFrictionDescriptors.size());
    vbdScratch.avbdSelfContactFrictionInventory.syncActiveRows(
        vbdScratch.avbdSelfContactFrictionDescriptors, hardRowWarmStartOptions);

    vbdScratch.avbdSelfContactFrictionRows.clear();
    vbdScratch.avbdSelfContactFrictionRows.reserve(
        vbdScratch.avbdSelfContactFrictionInventory.size());
    for (const dvbd::AvbdScalarRowRecord& record :
         vbdScratch.avbdSelfContactFrictionInventory.records()) {
      dvbd::AvbdSelfContactFrictionRow row;
      row.state = record.state;
      row.axis = record.descriptor.key.axis;
      row.bounds = record.descriptor.bounds;
      assignSelfContactPrimitive(record.descriptor, row);
      for (std::uint8_t i = 0; i < 4; ++i) {
        row.stepStartPositions[i] = state.positions[row.nodes[i]];
      }
      row.previousConstraintValue
          = dvbd::avbdSelfContactFrictionConstraintValue(row, state.positions);
      vbdScratch.avbdSelfContactFrictionRows.push_back(row);
    }
    projectAvbdSelfContactFrictionWarmStarts(
        vbdScratch.avbdSelfContactFrictionInventory,
        vbdScratch.avbdSelfContactFrictionRows,
        vbdScratch.previousAvbdSelfContactFrictionWarmStarts);

    dvbd::AvbdTetMaterialFiniteStiffnessOptions tetOptions;
    tetOptions.beta = config.avbdBeta;
    tetOptions.maxStiffness = std::min(1.0, config.avbdMaxStiffness);
    dvbd::AvbdSelfContactNormalOptions selfContactOptions;
    selfContactOptions.alpha = config.avbdAlpha;
    selfContactOptions.beta = config.avbdBeta;
    selfContactOptions.maxStiffness = config.avbdMaxStiffness;
    dvbd::AvbdSelfContactFrictionOptions selfContactFrictionOptions;
    selfContactFrictionOptions.alpha = config.avbdAlpha;
    selfContactFrictionOptions.beta = config.avbdBeta;
    selfContactFrictionOptions.maxStiffness = config.avbdMaxStiffness;
    result = dvbd::blockDescentTetMeshAvbdFiniteStiffness(
        scratch.next,
        state.masses,
        scratch.activeFixed,
        scratch.inertialTargets,
        vbdScratch.tets,
        lame.mu,
        lame.lambda,
        timeStep,
        vbdScratch.avbdTetRows,
        vbdScratch.coloring,
        vbdScratch.tetAdjacency,
        options,
        tetOptions,
        selfContact,
        useAvbdSelfContactRows ? &vbdScratch.avbdSelfContactRows : nullptr,
        useAvbdSelfContactRows ? &selfContactOptions : nullptr,
        useAvbdSelfContactFrictionRows ? &vbdScratch.avbdSelfContactFrictionRows
                                       : nullptr,
        useAvbdSelfContactFrictionRows ? &selfContactFrictionOptions : nullptr);

    for (std::size_t i = 0; i < vbdScratch.avbdTetRows.size(); ++i) {
      vbdScratch.avbdTetInventory[i].state = vbdScratch.avbdTetRows[i].state;
    }
    for (std::size_t i = 0; i < vbdScratch.avbdSelfContactRows.size(); ++i) {
      vbdScratch.avbdSelfContactInventory[i].state
          = vbdScratch.avbdSelfContactRows[i].state;
    }
    for (std::size_t i = 0; i < vbdScratch.avbdSelfContactFrictionRows.size();
         ++i) {
      vbdScratch.avbdSelfContactFrictionInventory[i].state
          = vbdScratch.avbdSelfContactFrictionRows[i].state;
    }
    stats.vbdAvbdSelfContactNormalRows += vbdScratch.avbdSelfContactRows.size();
    stats.vbdAvbdFrictionTangentRows
        += vbdScratch.avbdSelfContactFrictionRows.size();
    stats.vbdAvbdFiniteStiffnessRows += vbdScratch.avbdTetRows.size();
    stats.vbdAvbdFiniteStiffnessTetRows += vbdScratch.avbdTetRows.size();
  } else {
    vbdScratch.avbdContactDescriptors.clear();
    vbdScratch.avbdContactRows.clear();
    vbdScratch.avbdContactInventory.records().clear();
    vbdScratch.avbdFrictionDescriptors.clear();
    vbdScratch.avbdFrictionRows.clear();
    vbdScratch.avbdFrictionInventory.records().clear();
    vbdScratch.avbdSelfContactDescriptors.clear();
    vbdScratch.avbdSelfContactRows.clear();
    vbdScratch.avbdSelfContactInventory.records().clear();
    vbdScratch.avbdSelfContactFrictionDescriptors.clear();
    vbdScratch.avbdSelfContactFrictionRows.clear();
    vbdScratch.avbdSelfContactFrictionInventory.records().clear();
    vbdScratch.avbdAttachmentDescriptors.clear();
    vbdScratch.avbdAttachmentRows.clear();
    vbdScratch.avbdAttachmentInventory.records().clear();
    vbdScratch.avbdSpringDescriptors.clear();
    vbdScratch.avbdSpringRows.clear();
    vbdScratch.avbdSpringInventory.records().clear();
    vbdScratch.avbdTetDescriptors.clear();
    vbdScratch.avbdTetRows.clear();
    vbdScratch.avbdTetInventory.records().clear();
    vbdScratch.avbdSolveFixed.clear();

    // state.positions holds x^t for this step (the write-back to the live state
    // happens after the solve), so it is the Rayleigh displacement reference.
    // parallelBlockDescentDeformable falls back to the full-featured serial
    // driver when workerThreads <= 1.
    result = dvbd::parallelBlockDescentDeformable(
        scratch.next,
        state.masses,
        scratch.activeFixed,
        scratch.inertialTargets,
        vbdScratch.springs,
        model.stiffness,
        vbdScratch.springAdjacency,
        vbdScratch.tets,
        lame.mu,
        lame.lambda,
        vbdScratch.tetAdjacency,
        timeStep,
        vbdScratch.coloring,
        options,
        config.workerThreads,
        &state.positions,
        contactPlanes,
        frictionCoeff,
        selfContact);
  }

  ++stats.vbdBodyCount;
  stats.vbdSweeps += result.iterations;
  stats.vbdVertexUpdates += result.vertexUpdates;
  stats.vbdResidualNormSquared = result.finalResidualNormSquared;
}

//==============================================================================
// Assemble the sparse per-step Hessian (inertia + spring + self-contact barrier
// + static ground barrier) with per-element PSD projection and solve
// H d = -gradient for the projected-Newton search direction via a sparse
// Cholesky (LDL^T) factorization. Returns false so the caller falls back to
// mass-scaled steepest descent when the body is too large for the solve or the
// factorization is not positive definite. The per-element barrier/spring
// eigen-decompositions and the triplet assembly are data-parallel GPU
// candidates.
bool computeProjectedNewtonDirection(
    const comps::DeformableNodeState& state,
    const comps::DeformableSpringModel& model,
    const std::vector<Eigen::Vector3d>& positions,
    const std::vector<std::uint8_t>& fixed,
    const std::vector<StaticGroundBarrier>& barriers,
    const std::vector<SphereObstacleBarrier>& sphereObstacles,
    const std::vector<BoxObstacleBarrier>& boxObstacles,
    const std::vector<CapsuleObstacleBarrier>& capsuleObstacles,
    const SelfContactBarrierInputs* contactBarrier,
    const GroundFrictionInputs* groundFriction,
    const SelfContactFrictionInputs* selfContactFriction,
    const FemElasticityInputs* femElasticity,
    const double timeStep,
    const std::vector<Eigen::Vector3d>& gradient,
    std::vector<Eigen::Vector3d>& direction,
    DeformableContactSolverScratch& solverCache,
    DeformableSolverStats& stats,
    double barrierStiffness = kDefaultBarrierStiffness,
    bool useIterativeSolver = false,
    bool useMatrixFreeSolver = false)
{
  const std::size_t nodeCount = positions.size();
  // The sparse direct (Cholesky) solve is fastest for small/medium meshes but
  // its fill-in grows super-linearly, so it is capped. Above the direct cap (or
  // when iterative solving is opted in) a sparse IC-CG solve is used instead.
  // The explicit matrix-free opt-in goes one step further and bypasses sparse
  // Hessian assembly, using local block Hessian-vector products. The hard cap
  // is a runaway bound only.
  constexpr std::size_t kMaxDirectNodes = 20000;
  constexpr std::size_t kMaxIterativeNodes = 1000000;
  if (nodeCount == 0 || nodeCount > kMaxIterativeNodes) {
    return false;
  }
  const bool solveIteratively = useIterativeSolver || useMatrixFreeSolver
                                || nodeCount > kMaxDirectNodes;
  const bool solveMatrixFree = useMatrixFreeSolver;

  const auto dim = static_cast<Eigen::Index>(3 * nodeCount);
  const double invDt2 = 1.0 / (timeStep * timeStep);

  // Right-hand side: -gradient on free DOFs, zero on pinned (fixed) DOFs.
  Eigen::VectorXd& rhs = solverCache.projectedNewtonRhs;
  rhs.resize(dim);
  rhs.setZero();
  for (std::size_t i = 0; i < nodeCount; ++i) {
    if (fixed[i] != 0u) {
      continue;
    }
    for (int k = 0; k < 3; ++k) {
      rhs(static_cast<Eigen::Index>(3 * i + k)) = -gradient[i][k];
    }
  }

  // Triplet assembly. Each fixed DOF is pinned with a lone unit diagonal and a
  // zero right-hand side; only the free-free entries of each element block are
  // emitted (a principal submatrix of a PSD block stays PSD), which decouples
  // pinned DOFs from the free system exactly as zeroing a dense row/column
  // would. Eigen sums duplicate triplets, giving standard additive assembly.
  // The reserve below is a best-effort capacity hint only (it over-counts edges
  // touching fixed nodes and omits ground-barrier diagonals); setFromTriplets
  // sizes the matrix exactly from the actual triplet list regardless.
  std::size_t tripletEstimate = 3 * nodeCount + 36 * model.edges.size();
  if (contactBarrier != nullptr && contactBarrier->candidates != nullptr) {
    tripletEstimate
        += 144
           * (contactBarrier->candidates->pointTriangleCandidates.size()
              + contactBarrier->candidates->edgeEdgeCandidates.size());
  }
  std::vector<Eigen::Triplet<double>>& triplets
      = solverCache.projectedNewtonTriplets;
  triplets.clear();
  ProjectedNewtonMatrixFreeHessian matrixFreeHessian(
      solverCache.projectedNewtonMatrixFreeBlocks,
      solverCache.projectedNewtonMatrixFreeDiagonalBlocks,
      solverCache.projectedNewtonMatrixFreeInverseDiagonalBlocks);
  matrixFreeHessian.reset(solveMatrixFree ? nodeCount : 0);
  if (!solveMatrixFree) {
    triplets.reserve(tripletEstimate);
  } else {
    const std::size_t activeContactCount
        = contactBarrier != nullptr && contactBarrier->candidates != nullptr
              ? contactBarrier->candidates->pointTriangleCandidates.size()
                    + contactBarrier->candidates->edgeEdgeCandidates.size()
              : 0u;
    const std::size_t matrixFreeBlockEstimate
        = 4 * nodeCount + 4 * model.edges.size()
          + (femElasticity != nullptr && femElasticity->tetrahedra != nullptr
                 ? 16 * femElasticity->tetrahedra->size()
                 : 0u)
          + 16 * activeContactCount;
    matrixFreeHessian.blocks.reserve(matrixFreeBlockEstimate);
  }

  const auto isFree = [&](std::size_t node) {
    return fixed[node] == 0u;
  };
  const auto addBlock3 = [&](std::size_t nodeRow,
                             std::size_t nodeCol,
                             const Eigen::Matrix3d& block) {
    if (!isFree(nodeRow) || !isFree(nodeCol)) {
      return;
    }
    if (solveMatrixFree) {
      matrixFreeHessian.addBlock3(nodeRow, nodeCol, block);
      return;
    }
    for (int r = 0; r < 3; ++r) {
      for (int c = 0; c < 3; ++c) {
        triplets.emplace_back(
            static_cast<Eigen::Index>(3 * nodeRow + r),
            static_cast<Eigen::Index>(3 * nodeCol + c),
            block(r, c));
      }
    }
  };
  const auto addDiagonalBlock3 = [&](std::size_t node, double diagonal) {
    if (solveMatrixFree) {
      matrixFreeHessian.addBlock3(
          node, node, diagonal * Eigen::Matrix3d::Identity());
      return;
    }
    for (int r = 0; r < 3; ++r) {
      triplets.emplace_back(
          static_cast<Eigen::Index>(3 * node + r),
          static_cast<Eigen::Index>(3 * node + r),
          diagonal);
    }
  };

  // Inertia: block diagonal, positive definite for free nodes. Fixed DOFs get
  // a unit diagonal so the global matrix stays positive definite.
  for (std::size_t i = 0; i < nodeCount; ++i) {
    const double diagonal = isFree(i) ? state.masses[i] * invDt2 : 1.0;
    addDiagonalBlock3(i, diagonal);
  }

  // Spring stretch Hessian per edge, PSD-projected over its 6x6 block. The
  // per-element projections are collected into one packed batch and projected
  // through the pluggable PSD backend (CPU by default, optional GPU offload),
  // then scattered. Batching is what lets a data-parallel backend amortize the
  // per-block eigensolves; the CPU backend is bit-identical to the previous
  // inline per-block projection.
  constexpr double minLength = 1e-12;
  constexpr std::size_t kEdgeBlockEntries = 36; // 6x6
  std::vector<double>& edgeBlocks = solverCache.projectedNewtonEdgeBlocks;
  std::vector<std::array<std::size_t, 2>>& edgeBlockNodes
      = solverCache.projectedNewtonEdgeBlockNodes;
  edgeBlocks.clear();
  edgeBlockNodes.clear();
  edgeBlocks.reserve(kEdgeBlockEntries * model.edges.size());
  edgeBlockNodes.reserve(model.edges.size());
  for (const auto& edge : model.edges) {
    const Eigen::Vector3d delta = positions[edge.nodeB] - positions[edge.nodeA];
    const double length = delta.norm();
    if (length <= minLength || !std::isfinite(length)) {
      continue;
    }
    const Eigen::Vector3d dir = delta / length;
    const Eigen::Matrix3d projection = dir * dir.transpose();
    const double scale = std::max(0.0, 1.0 - edge.restLength / length);
    const Eigen::Matrix3d block
        = model.stiffness
          * (projection + scale * (Eigen::Matrix3d::Identity() - projection));
    Eigen::Matrix<double, 6, 6, Eigen::RowMajor> edgeHessian;
    edgeHessian.block<3, 3>(0, 0) = block;
    edgeHessian.block<3, 3>(3, 3) = block;
    edgeHessian.block<3, 3>(0, 3) = -block;
    edgeHessian.block<3, 3>(3, 0) = -block;
    const std::size_t offset = edgeBlocks.size();
    edgeBlocks.resize(offset + kEdgeBlockEntries);
    Eigen::Map<Eigen::Matrix<double, 6, 6, Eigen::RowMajor>>(
        edgeBlocks.data() + offset) = edgeHessian;
    edgeBlockNodes.push_back({edge.nodeA, edge.nodeB});
  }
  nb::projectSymmetricBlocksToPsd(edgeBlocks.data(), 6, edgeBlockNodes.size());
  for (std::size_t b = 0; b < edgeBlockNodes.size(); ++b) {
    const Eigen::Map<const Eigen::Matrix<double, 6, 6, Eigen::RowMajor>>
        edgeHessian(edgeBlocks.data() + b * kEdgeBlockEntries);
    const auto& nodes = edgeBlockNodes[b];
    for (int bi = 0; bi < 2; ++bi) {
      for (int bj = 0; bj < 2; ++bj) {
        addBlock3(
            nodes[bi], nodes[bj], edgeHessian.block<3, 3>(3 * bi, 3 * bj));
      }
    }
  }

  // Stable neo-Hookean FEM elasticity Hessian per tetrahedron, PSD-projected
  // over its 12x12 block through the same batched seam as the spring and
  // barrier blocks. Null femElasticity (the default mass-spring path) skips
  // this entirely, so spring bodies assemble exactly as before.
  if (femElasticity != nullptr && femElasticity->tetrahedra != nullptr
      && femElasticity->restShapes != nullptr) {
    const auto& tets = *femElasticity->tetrahedra;
    const auto& rests = *femElasticity->restShapes;
    const std::size_t tetCount = std::min(tets.size(), rests.size());
    constexpr std::size_t kTetBlockEntries = 144; // 12x12
    std::vector<double>& tetBlocks = solverCache.projectedNewtonTetBlocks;
    std::vector<std::array<std::size_t, 4>>& tetBlockNodes
        = solverCache.projectedNewtonTetBlockNodes;
    tetBlocks.clear();
    tetBlockNodes.clear();
    tetBlocks.reserve(kTetBlockEntries * tetCount);
    tetBlockNodes.reserve(tetCount);
    for (std::size_t t = 0; t < tetCount; ++t) {
      const auto& tet = tets[t];
      const fem::TetElementResult element = evaluateFemTetElement(
          *femElasticity, positions, tet, rests[t], /*computeHessian=*/true);
      if (!element.valid) {
        continue;
      }
      const std::size_t offset = tetBlocks.size();
      tetBlocks.resize(offset + kTetBlockEntries);
      Eigen::Map<Eigen::Matrix<double, 12, 12, Eigen::RowMajor>>(
          tetBlocks.data() + offset) = element.hessian;
      tetBlockNodes.push_back({tet.nodeA, tet.nodeB, tet.nodeC, tet.nodeD});
    }
    nb::projectSymmetricBlocksToPsd(tetBlocks.data(), 12, tetBlockNodes.size());
    for (std::size_t b = 0; b < tetBlockNodes.size(); ++b) {
      const Eigen::Map<const Eigen::Matrix<double, 12, 12, Eigen::RowMajor>>
          projected(tetBlocks.data() + b * kTetBlockEntries);
      const auto& nodes = tetBlockNodes[b];
      for (int bi = 0; bi < 4; ++bi) {
        for (int bj = 0; bj < 4; ++bj) {
          addBlock3(
              nodes[bi], nodes[bj], projected.block<3, 3>(3 * bi, 3 * bj));
        }
      }
    }
  }

  // Self-contact barrier Hessian per active contact, PSD-projected over 12x12.
  // Active point-triangle and edge-edge blocks are collected into one packed
  // batch, projected through the pluggable PSD backend (CPU by default,
  // optional GPU offload), then scattered -- the same batching seam the spring
  // blocks use. The CPU backend is bit-identical to the previous inline
  // per-block projection.
  if (contactBarrier != nullptr && contactBarrier->candidates != nullptr
      && contactBarrier->triangles != nullptr && contactBarrier->stiffness > 0.0
      && contactBarrier->squaredActivationDistance > 0.0) {
    const auto& candidates = *contactBarrier->candidates;
    const auto& triangles = *contactBarrier->triangles;
    const double sqAct = contactBarrier->squaredActivationDistance;
    const double kappa = contactBarrier->stiffness;
    constexpr std::size_t kBarrierBlockEntries = 144; // 12x12
    std::vector<double>& barrierBlocks
        = solverCache.projectedNewtonBarrierBlocks;
    std::vector<std::array<std::size_t, 4>>& barrierBlockNodes
        = solverCache.projectedNewtonBarrierBlockNodes;
    barrierBlocks.clear();
    barrierBlockNodes.clear();
    const auto collect12 = [&](const dc::Matrix12d& blockHessian,
                               const std::array<std::size_t, 4>& nodes) {
      const std::size_t offset = barrierBlocks.size();
      barrierBlocks.resize(offset + kBarrierBlockEntries);
      Eigen::Map<Eigen::Matrix<double, 12, 12, Eigen::RowMajor>>(
          barrierBlocks.data() + offset) = blockHessian;
      barrierBlockNodes.push_back(nodes);
    };
    for (const auto& candidate : candidates.pointTriangleCandidates) {
      const auto& triangle = triangles[candidate.triangle];
      const auto result = dc::pointTriangleBarrier(
          positions[candidate.point],
          positions[triangle.nodeA],
          positions[triangle.nodeB],
          positions[triangle.nodeC],
          sqAct,
          kappa);
      if (!result.active) {
        continue;
      }
      collect12(
          result.hessian,
          {candidate.point, triangle.nodeA, triangle.nodeB, triangle.nodeC});
    }
    for (const auto& candidate : candidates.edgeEdgeCandidates) {
      const auto& edgeA = candidates.surfaceEdges[candidate.edgeA];
      const auto& edgeB = candidates.surfaceEdges[candidate.edgeB];
      const auto result = dc::edgeEdgeBarrier(
          positions[edgeA.nodeA],
          positions[edgeA.nodeB],
          positions[edgeB.nodeA],
          positions[edgeB.nodeB],
          sqAct,
          kappa);
      if (!result.active) {
        continue;
      }
      collect12(
          result.hessian, {edgeA.nodeA, edgeA.nodeB, edgeB.nodeA, edgeB.nodeB});
    }
    nb::projectSymmetricBlocksToPsd(
        barrierBlocks.data(), 12, barrierBlockNodes.size());
    for (std::size_t b = 0; b < barrierBlockNodes.size(); ++b) {
      const Eigen::Map<const Eigen::Matrix<double, 12, 12, Eigen::RowMajor>>
          projected(barrierBlocks.data() + b * kBarrierBlockEntries);
      const auto& nodes = barrierBlockNodes[b];
      for (int bi = 0; bi < 4; ++bi) {
        for (int bj = 0; bj < 4; ++bj) {
          addBlock3(
              nodes[bi], nodes[bj], projected.block<3, 3>(3 * bi, 3 * bj));
        }
      }
    }
  }

  // Static ground barrier Hessian (vertical scalar per active free node).
  if (!barriers.empty()) {
    const double activationDistance = staticGroundBarrierActivationDistance();
    const double barrierScale = barrierStiffness;
    for (std::size_t i = 0; i < nodeCount; ++i) {
      if (!isFree(i)) {
        continue;
      }
      const auto groundTop = staticGroundTopAt(positions[i], barriers);
      if (!groundTop.has_value()) {
        continue;
      }
      const double d = positions[i].z() - *groundTop;
      if (d <= 0.0 || d >= activationDistance || !std::isfinite(d)) {
        continue;
      }
      const double offset = d - activationDistance;
      const double logRatio = std::log(d / activationDistance);
      const double second
          = -barrierScale
            * (2.0 * logRatio + 4.0 * offset / d - (offset * offset) / (d * d));
      Eigen::Matrix3d block = Eigen::Matrix3d::Zero();
      block(2, 2) = std::max(0.0, second);
      addBlock3(i, i, block);
    }
  }

  // Static sphere obstacle barrier Hessian. The full per-node barrier Hessian
  // is B''(d) n n^T + (B'(d)/|x-c|)(I - n n^T) with n the outward radial
  // normal; its tangential eigenvalues B'(d)/|x-c| are non-positive (the
  // barrier force pulls the node radially out), so the PSD projection keeps
  // only the rank-1 radial term max(0, B''(d)) n n^T -- the sphere analogue of
  // the vertical ground barrier curvature, now along the radial normal.
  if (!sphereObstacles.empty()) {
    const double activationDistance = staticGroundBarrierActivationDistance();
    const double barrierScale = barrierStiffness;
    for (std::size_t i = 0; i < nodeCount; ++i) {
      if (!isFree(i)) {
        continue;
      }
      for (const auto& obstacle : sphereObstacles) {
        const Eigen::Vector3d offset = positions[i] - obstacle.center;
        const double centerDistance = offset.norm();
        const double d = centerDistance - obstacle.radius;
        if (d <= 0.0 || d >= activationDistance || centerDistance <= 0.0
            || !std::isfinite(d)) {
          continue;
        }
        const double distanceOffset = d - activationDistance;
        const double logRatio = std::log(d / activationDistance);
        const double second = -barrierScale
                              * (2.0 * logRatio + 4.0 * distanceOffset / d
                                 - (distanceOffset * distanceOffset) / (d * d));
        const double curvature = std::max(0.0, second);
        if (curvature <= 0.0) {
          continue;
        }
        const Eigen::Vector3d normal = offset / centerDistance;
        addBlock3(i, i, curvature * (normal * normal.transpose()));
      }
    }
  }

  // Static box obstacle barrier Hessian: the rank-1 radial curvature
  // max(0, B''(d)) n n^T along the outward box-surface normal, the box analogue
  // of the sphere obstacle barrier Hessian above (the tangential eigenvalues
  // are again non-positive and drop out under PSD projection).
  if (!boxObstacles.empty()) {
    const double activationDistance = staticGroundBarrierActivationDistance();
    const double barrierScale = barrierStiffness;
    for (std::size_t i = 0; i < nodeCount; ++i) {
      if (!isFree(i)) {
        continue;
      }
      for (const auto& obstacle : boxObstacles) {
        Eigen::Vector3d normal;
        const double d
            = boxObstacleSurfaceDistance(positions[i], obstacle, normal);
        if (d <= 0.0 || d >= activationDistance || !std::isfinite(d)) {
          continue;
        }
        const double distanceOffset = d - activationDistance;
        const double logRatio = std::log(d / activationDistance);
        const double second = -barrierScale
                              * (2.0 * logRatio + 4.0 * distanceOffset / d
                                 - (distanceOffset * distanceOffset) / (d * d));
        const double curvature = std::max(0.0, second);
        if (curvature <= 0.0) {
          continue;
        }
        addBlock3(i, i, curvature * (normal * normal.transpose()));
      }
    }
  }

  // Capsule obstacle barrier Hessian: the same rank-1 radial block as the
  // sphere/box obstacle barriers (curvature * n n^T along the outward radial
  // normal; the tangential eigenvalues are non-positive and drop out under PSD
  // projection).
  if (!capsuleObstacles.empty()) {
    const double activationDistance = staticGroundBarrierActivationDistance();
    const double barrierScale = barrierStiffness;
    for (std::size_t i = 0; i < nodeCount; ++i) {
      if (!isFree(i)) {
        continue;
      }
      for (const auto& obstacle : capsuleObstacles) {
        Eigen::Vector3d normal;
        const double d
            = capsuleObstacleSurfaceDistance(positions[i], obstacle, normal);
        if (d <= 0.0 || d >= activationDistance || !std::isfinite(d)) {
          continue;
        }
        const double distanceOffset = d - activationDistance;
        const double logRatio = std::log(d / activationDistance);
        const double second = -barrierScale
                              * (2.0 * logRatio + 4.0 * distanceOffset / d
                                 - (distanceOffset * distanceOffset) / (d * d));
        const double curvature = std::max(0.0, second);
        if (curvature <= 0.0) {
          continue;
        }
        addBlock3(i, i, curvature * (normal * normal.transpose()));
      }
    }
  }

  // Lagged ground-friction Hessian: a 3x3 tangent-plane block per node with an
  // active friction normal force. With P = I - n n^T the tangent projector onto
  // the plane orthogonal to the lagged ground normal n, T = u_T/||u_T|| the
  // unit slip direction (in that plane), the block is scale *
  // [ (f1/||u_T||) (P - T T^T) + f1' T T^T ]. Both coefficients are
  // non-negative and (P - T T^T), T T^T are PSD with ranges inside the tangent
  // plane, so the block is positive semidefinite by construction (no projection
  // needed) and -> scale * (2/eps) P isotropically as ||u_T|| -> 0. For flat
  // ground n = +z, P = diag(1, 1, 0) and this reduces to the xy 2x2 block.
  if (groundFriction != nullptr && groundFriction->coefficient > 0.0
      && groundFriction->epsilon > 0.0
      && groundFriction->stepStartPositions != nullptr
      && groundFriction->laggedNormalForce != nullptr) {
    const auto& start = *groundFriction->stepStartPositions;
    const auto& normalForce = *groundFriction->laggedNormalForce;
    const auto* normalDirection = groundFriction->laggedNormalDirection;
    const double epsilon = groundFriction->epsilon;
    for (std::size_t i = 0; i < nodeCount; ++i) {
      if (!isFree(i) || normalForce[i] <= 0.0) {
        continue;
      }
      const Eigen::Vector3d n = (normalDirection != nullptr)
                                    ? (*normalDirection)[i]
                                    : Eigen::Vector3d::UnitZ();
      const Eigen::Matrix3d projector
          = Eigen::Matrix3d::Identity() - n * n.transpose();
      const Eigen::Vector3d u = positions[i] - start[i];
      const Eigen::Vector3d tangent = u - n.dot(u) * n;
      const double y = tangent.norm();
      const double scale = groundFriction->coefficient * normalForce[i];
      constexpr double tiny = 1e-12;
      Eigen::Matrix3d block;
      if (y > tiny) {
        const double f1OverY = frictionF1(y, epsilon) / y;
        const double f1Prime
            = (y < epsilon) ? (2.0 / epsilon - 2.0 * y / (epsilon * epsilon))
                            : 0.0;
        const Eigen::Vector3d t = tangent / y;
        const Eigen::Matrix3d tt = t * t.transpose();
        block = scale * (f1OverY * (projector - tt) + f1Prime * tt);
      } else {
        block = scale * (2.0 / epsilon) * projector;
      }
      addBlock3(i, i, block);
    }
  }

  // Lagged self-contact friction Hessian (point-triangle): a PSD 12x12 block
  // per active contact, projection^T * H_2x2 * projection, scattered to the
  // four stencil nodes (free-free only). Mirrors the ground-friction tangent
  // Hessian through the contact's tangent projection; non-negative coefficients
  // keep H_2x2 positive semidefinite, so the assembled block is PSD by
  // construction.
  if (selfContactFriction != nullptr && selfContactFriction->coefficient > 0.0
      && selfContactFriction->epsilon > 0.0
      && selfContactFriction->stepStartPositions != nullptr
      && selfContactFriction->contacts != nullptr) {
    const auto& start = *selfContactFriction->stepStartPositions;
    const double epsilon = selfContactFriction->epsilon;
    for (const auto& contact : *selfContactFriction->contacts) {
      if (contact.normalForce <= 0.0) {
        continue;
      }
      Eigen::Matrix<double, 12, 1> displacement;
      for (int k = 0; k < 4; ++k) {
        displacement.segment<3>(3 * k)
            = positions[contact.nodes[k]] - start[contact.nodes[k]];
      }
      const Eigen::Vector2d tangent = contact.projection * displacement;
      const double y = tangent.norm();
      const double scale
          = selfContactFriction->coefficient * contact.normalForce;
      constexpr double tiny = 1e-12;
      double f1OverY = 2.0 / epsilon;
      double f1Prime = 2.0 / epsilon;
      double tx = 0.0;
      double ty = 0.0;
      if (y > tiny) {
        f1OverY = frictionF1(y, epsilon) / y;
        f1Prime = (y < epsilon)
                      ? (2.0 / epsilon - 2.0 * y / (epsilon * epsilon))
                      : 0.0;
        tx = tangent.x() / y;
        ty = tangent.y() / y;
      }
      Eigen::Matrix2d tangentHessian;
      tangentHessian(0, 0) = f1OverY * (1.0 - tx * tx) + f1Prime * tx * tx;
      tangentHessian(0, 1) = (f1Prime - f1OverY) * tx * ty;
      tangentHessian(1, 0) = tangentHessian(0, 1);
      tangentHessian(1, 1) = f1OverY * (1.0 - ty * ty) + f1Prime * ty * ty;
      const Eigen::Matrix<double, 12, 12> block
          = scale * contact.projection.transpose() * tangentHessian
            * contact.projection;
      for (int bi = 0; bi < 4; ++bi) {
        for (int bj = 0; bj < 4; ++bj) {
          addBlock3(
              contact.nodes[bi],
              contact.nodes[bj],
              block.block<3, 3>(3 * bi, 3 * bj));
        }
      }
    }
  }

  Eigen::SparseMatrix<double>& hessian = solverCache.projectedNewtonHessian;
  if (!solveMatrixFree) {
    hessian.resize(dim, dim);
    hessian.setZero();
    hessian.reserve(static_cast<Eigen::Index>(triplets.size()));
    hessian.setFromTriplets(triplets.begin(), triplets.end());
    hessian.makeCompressed();
    const auto hessianNonZeros = static_cast<std::size_t>(hessian.nonZeros());
    const auto hessianCols = static_cast<std::size_t>(hessian.cols());
    using SparseStorageIndex = Eigen::SparseMatrix<double>::StorageIndex;
    const std::size_t hessianStorageBytes
        = hessianNonZeros * (sizeof(double) + sizeof(SparseStorageIndex))
          + (hessianCols + 1u) * sizeof(SparseStorageIndex);
    stats.projectedNewtonHessianNonZeros
        = std::max(stats.projectedNewtonHessianNonZeros, hessianNonZeros);
    stats.projectedNewtonHessianStorageBytes = std::max(
        stats.projectedNewtonHessianStorageBytes, hessianStorageBytes);
  }

  Eigen::VectorXd& solution = solverCache.projectedNewtonSolution;
  if (solveMatrixFree) {
    std::size_t cgIterations = 0;
    double cgError = 0.0;
    if (!solveMatrixFreeConjugateGradient(
            matrixFreeHessian,
            rhs,
            solution,
            solverCache.projectedNewtonMatrixFreeResidual,
            solverCache.projectedNewtonMatrixFreePreconditionedResidual,
            solverCache.projectedNewtonMatrixFreeDirection,
            solverCache.projectedNewtonMatrixFreeHessianDirection,
            cgIterations,
            cgError)
        || !solution.allFinite()) {
      solverCache.newtonPatternValid = false;
      return false;
    }
    ++stats.projectedNewtonIterativeSolves;
    ++stats.projectedNewtonMatrixFreeSolves;
    stats.projectedNewtonIterativeIterations += cgIterations;
    if (std::isfinite(cgError)) {
      stats.projectedNewtonIterativeMaxError
          = std::max(stats.projectedNewtonIterativeMaxError, cgError);
    }
    solverCache.newtonPatternValid = false;
  } else if (solveIteratively) {
    // Iterative path: an incomplete-Cholesky preconditioned conjugate-gradient
    // solve. The inertia term (m/dt^2 on every free DOF) plus the PSD-projected
    // spring/barrier blocks make the Hessian symmetric positive definite, so CG
    // is guaranteed to converge; it only ever factorizes *incompletely* (a
    // sparse approximate Cholesky that drops fill), so time and memory stay
    // near O(nnz) and the solve scales to meshes well past the direct cap. The
    // incomplete-Cholesky preconditioner is far stronger than a diagonal
    // (Jacobi) one on the ill-conditioned Hessians that stiff barrier contact
    // produces -- it collapses the CG iteration count there, so the iterative
    // path converges within the cap (and thus matches the direct solve) on
    // contact scenes where plain Jacobi-CG would stall and fall back. Reading
    // only the lower triangle matches the direct solver's symmetric assumption.
    // A non-converged or non-finite solve (including an incomplete-Cholesky
    // breakdown that Eigen's diagonal shifting cannot repair) falls back to
    // mass-scaled steepest descent below, exactly as the direct path does on an
    // indefinite factorization.
    Eigen::ConjugateGradient<
        Eigen::SparseMatrix<double>,
        Eigen::Lower,
        Eigen::IncompleteCholesky<double>>
        cg;
    cg.setTolerance(1e-8);
    cg.setMaxIterations(static_cast<Eigen::Index>(2) * dim);
    cg.compute(hessian);
    if (cg.info() != Eigen::Success) {
      solverCache.newtonPatternValid = false;
      return false;
    }
    solution = cg.solve(rhs);
    if (cg.info() != Eigen::Success || !solution.allFinite()) {
      solverCache.newtonPatternValid = false;
      return false;
    }
    ++stats.projectedNewtonIterativeSolves;
    stats.projectedNewtonIterativeIterations
        += static_cast<std::size_t>(std::max<Eigen::Index>(0, cg.iterations()));
    if (std::isfinite(cg.error())) {
      stats.projectedNewtonIterativeMaxError
          = std::max(stats.projectedNewtonIterativeMaxError, cg.error());
    }
    // The cached direct-solver symbolic pattern was not refreshed this step, so
    // invalidate it: a later step that drops back to the direct path must
    // re-analyze rather than trust a stale ordering.
    solverCache.newtonPatternValid = false;
  } else {
    // Reuse the fill-reducing symbolic factorization when the sparsity pattern
    // is unchanged from the last analyzed matrix (same column/row index
    // arrays); the expensive ordering then runs once and only the numeric
    // factorization repeats. This is behavior-preserving (analyzePattern +
    // factorize gives the same result as compute()); any structural mismatch
    // re-analyzes, so it is safe by construction.
    auto& ldlt = solverCache.newtonSolver;
    const auto cols = hessian.cols();
    const auto nnz = hessian.nonZeros();
    const int* outer = hessian.outerIndexPtr();
    const int* inner = hessian.innerIndexPtr();
    bool patternMatches
        = solverCache.newtonPatternValid
          && static_cast<Eigen::Index>(solverCache.newtonPatternOuter.size())
                 == cols + 1
          && static_cast<Eigen::Index>(solverCache.newtonPatternInner.size())
                 == nnz;
    if (patternMatches) {
      for (Eigen::Index i = 0; i <= cols && patternMatches; ++i) {
        patternMatches = solverCache.newtonPatternOuter[i] == outer[i];
      }
      for (Eigen::Index i = 0; i < nnz && patternMatches; ++i) {
        patternMatches = solverCache.newtonPatternInner[i] == inner[i];
      }
    }

    if (patternMatches) {
      ldlt.factorize(hessian);
      ++stats.projectedNewtonNumericFactorizations;
    } else {
      ldlt.analyzePattern(hessian);
      ldlt.factorize(hessian);
      solverCache.newtonPatternOuter.assign(outer, outer + cols + 1);
      solverCache.newtonPatternInner.assign(inner, inner + nnz);
      solverCache.newtonPatternValid = true;
      ++stats.projectedNewtonSymbolicFactorizations;
      ++stats.projectedNewtonNumericFactorizations;
    }

    // The inertia term (m/dt^2 on every free DOF, with positive node masses
    // enforced at construction) keeps the Hessian positive definite once the
    // spring/barrier blocks are PSD-projected, so no diagonal regularization is
    // needed (it would perturb the otherwise-exact solve). SimplicialLDLT's
    // info() only flags a structural failure / exact-zero pivot, so the
    // strictly-positive-D check is the actual positive-definiteness test (a
    // negative pivot would otherwise factorize silently). This has the same
    // intent as the dense LDLT isPositive() guard the rest of the module uses;
    // the two verdicts can only differ for near-singular matrices, which the
    // m/dt^2 inertia floor precludes here. Either failure falls back to
    // steepest descent below.
    if (ldlt.info() != Eigen::Success
        || (ldlt.vectorD().array() <= 0.0).any()) {
      // Force a fresh analysis next time after a failed/indefinite
      // factorization.
      solverCache.newtonPatternValid = false;
      return false;
    }
    solution = ldlt.solve(rhs);
    if (ldlt.info() != Eigen::Success || !solution.allFinite()) {
      solverCache.newtonPatternValid = false;
      return false;
    }
  }

  direction.resize(nodeCount);
  for (std::size_t i = 0; i < nodeCount; ++i) {
    if (fixed[i] != 0u) {
      direction[i].setZero();
    } else {
      direction[i] = solution.segment<3>(static_cast<Eigen::Index>(3 * i));
    }
  }
  return true;
}

//==============================================================================
void advanceDeformableBody(
    entt::entity entity,
    comps::DeformableNodeState& state,
    const comps::DeformableSpringModel& model,
    const comps::DeformableMeshTopology& topology,
    comps::DeformableSolverScratch& scratch,
    DeformableContactSolverScratch& contactScratch,
    DeformableVbdScratch& vbdScratch,
    const comps::DeformableVbdConfig* vbdConfig,
    std::span<const SurfaceContactSnapshot> surfaceSnapshots,
    std::span<const SurfaceContactSnapshot> rigidSurfaceSnapshots,
    std::span<const SurfaceContactSnapshot> movingRigidSurfaceSnapshots,
    const Eigen::Vector3d& gravity,
    double timeStep,
    const std::vector<StaticGroundBarrier>& barriers,
    const std::vector<SphereObstacleBarrier>& sphereObstacles,
    const std::vector<BoxObstacleBarrier>& boxObstacles,
    const std::vector<CapsuleObstacleBarrier>& capsuleObstacles,
    const comps::DeformableMaterial& material,
    DeformableSolverStats& stats)
{
  const auto nodeCount = state.positions.size();
  if (nodeCount == 0) {
    return;
  }

  const double frictionCoefficient = material.frictionCoefficient;

  // Opt-in stable neo-Hookean FEM elasticity. Each tetrahedron's rest shape is
  // computed once and cached in the per-entity scratch (the rest configuration
  // never changes), then reused every step; the inputs are passed by pointer
  // into the objective and projected-Newton assembly, replacing the mass-spring
  // edge model for this body. Null (the default) leaves the spring path
  // byte-identical.
  FemElasticityInputs femElasticity;
  const FemElasticityInputs* femElasticityPtr = nullptr;
  if (material.useFiniteElementElasticity && !topology.tetrahedra.empty()
      && topology.restPositions.size() == nodeCount) {
    syncFemRestShapeScratch(nodeCount, topology, material, contactScratch);
    femElasticity.tetrahedra = &topology.tetrahedra;
    femElasticity.restShapes = &contactScratch.femRestShapes;
    femElasticity.lame
        = fem::lameParameters(material.youngsModulus, material.poissonRatio);
    femElasticity.fixedCorotational = material.useFixedCorotationalElasticity;
    femElasticityPtr = &femElasticity;
  }

  // Per-step barrier stiffness (kappa). Opt-in adaptive scaling balances the
  // barrier against the heaviest free node's inertial stiffness; off it is the
  // fixed default, so every existing scene is byte-identical.
  double barrierStiffness = kDefaultBarrierStiffness;
  if (material.useAdaptiveBarrierStiffness) {
    double maxNodalMass = 0.0;
    for (std::size_t i = 0; i < nodeCount; ++i) {
      maxNodalMass = std::max(maxNodalMass, state.masses[i]);
    }
    barrierStiffness = adaptiveBarrierStiffness(
        maxNodalMass, timeStep, staticGroundBarrierActivationDistance());
  }

  // Opt this body into the iterative (conjugate-gradient) Newton linear solve.
  // Large meshes above the direct-solve node cap always take the iterative path
  // regardless of this flag; the flag forces it for any size so callers can
  // trade direct-solve speed for the factorization-free memory profile.
  const bool useIterativeSolver = material.useIterativeLinearSolver;
  const bool useMatrixFreeSolver = material.useMatrixFreeLinearSolver;

  stats.nodeCount += nodeCount;
  stats.edgeCount += model.edges.size();
  syncSurfaceContactTopology(
      topology.surfaceTriangles,
      nodeCount,
      !topology.tetrahedra.empty(),
      contactScratch);
  reserveDeformableFrictionScratch(nodeCount, contactScratch);

  scratch.inertialTargets.resize(nodeCount);
  scratch.next.resize(nodeCount);
  scratch.gradient.resize(nodeCount);
  scratch.direction.resize(nodeCount);
  scratch.candidate.resize(nodeCount);
  if (scratch.activeFixed.size() != nodeCount) {
    scratch.activeFixed = state.fixed;
  }
  if (scratch.externalAccelerations.size() != nodeCount) {
    scratch.externalAccelerations.assign(nodeCount, Eigen::Vector3d::Zero());
  }
  if (scratch.previousStepPositions.size() != nodeCount) {
    scratch.previousStepPositions = state.positions;
  }

  const double dampingScale = 1.0 / (1.0 + model.damping * timeStep);
  const Eigen::Vector3d gravityStep = gravity * timeStep * timeStep;
  for (std::size_t i = 0; i < nodeCount; ++i) {
    scratch.inertialTargets[i] = state.positions[i];
    scratch.next[i] = state.positions[i];
    if (scratch.activeFixed[i] == 0u) {
      scratch.inertialTargets[i]
          += timeStep * dampingScale * state.velocities[i] + gravityStep
             + timeStep * timeStep * scratch.externalAccelerations[i];
    }
  }

  makeInitialPositionsFeasible(
      scratch.next, scratch.activeFixed, barriers, &stats);

  // VBD handles static ground barriers and static sphere/box obstacles itself
  // (lagged per-vertex half-space penalty contact + Coulomb friction) when
  // contactStiffness > 0. Static rigid-surface CCD obstacles are always sphere
  // or box bodies (collectStaticRigidSurfaceCcdObstacles skips other shapes),
  // i.e. exactly the obstacles VBD handles via those barriers, so they no
  // longer force the body onto the default solver; after the VBD solve, the
  // shared self-surface and static rigid-surface CCD limiters still clip fast
  // crossings before write-back. The VBD candidate is also clipped by the
  // shared inter-body deformable-surface CCD limiter. VBD still cannot honor
  // *moving* rigid-surface CCD. Surface triangles no longer disqualify VBD:
  // runVbdDeformableSolve builds a lagged VT/EE self-contact candidate set and
  // adds the normal barrier blocks during the colored sweeps. A body with
  // static contacts but no VBD contact stiffness falls back to the default
  // solver so it still rests on / collides with them. The default-solver fast
  // path below keeps the stricter surface check so non-VBD bodies still get
  // self-contact.
  const bool movingRigidSurfaceFree = movingRigidSurfaceSnapshots.empty();
  const bool anyStaticContact = !barriers.empty() || !sphereObstacles.empty()
                                || !boxObstacles.empty()
                                || !rigidSurfaceSnapshots.empty();
  const bool vbdHandlesStaticContacts
      = !anyStaticContact
        || (vbdConfig != nullptr && vbdConfig->contactStiffness > 0.0);
  // VBD does not yet handle static capsule-rod obstacles, so a body near one
  // falls back to the default solver (which does).
  const bool contactFree = movingRigidSurfaceFree && !anyStaticContact
                           && capsuleObstacles.empty()
                           && contactScratch.surfaceTriangles.empty();
  if (vbdConfig != nullptr && vbdConfig->enabled && movingRigidSurfaceFree
      && capsuleObstacles.empty() && vbdHandlesStaticContacts) {
    runVbdDeformableSolve(
        entity,
        state,
        model,
        topology,
        scratch,
        vbdScratch,
        timeStep,
        material.youngsModulus,
        material.poissonRatio,
        material.useFiniteElementElasticity
            && material.useFixedCorotationalElasticity,
        barriers,
        sphereObstacles,
        boxObstacles,
        contactScratch.surfaceTriangles,
        contactScratch.surfaceContactPointMask,
        frictionCoefficient,
        *vbdConfig,
        stats);
    applySurfaceContactCcdCandidateLimit(
        state.positions, scratch.activeFixed, contactScratch, stats, scratch);
    applyInterBodySurfaceContactCcdCandidateLimit(
        entity,
        surfaceSnapshots,
        state.positions,
        scratch.activeFixed,
        contactScratch,
        stats,
        scratch);
    applyStaticRigidSurfaceCcdCandidateLimit(
        rigidSurfaceSnapshots,
        state.positions,
        scratch.activeFixed,
        contactScratch,
        stats,
        scratch);
  } else if (
      model.edges.empty() && contactFree && femElasticityPtr == nullptr) {
    for (std::size_t i = 0; i < nodeCount; ++i) {
      if (scratch.activeFixed[i] == 0u) {
        scratch.next[i] = scratch.inertialTargets[i];
      }
    }
  } else {
    constexpr std::size_t maxIterations = 64;
    constexpr std::size_t maxLineSearchIterations = 16;
    constexpr double gradientTolerance = 1e-9;
    constexpr double armijo = nb::kDefaultSufficientDecreaseFactor;
    constexpr double minStep = 1e-12;
    const double backtrackingScale
        = nb::sanitizeBacktrackingScale(nb::kDefaultBacktrackingScale);

    double lastGradSquared = 0.0;
    double lastAcceptedStepInfinityNorm = 0.0;
    // Whether the solve left the loop via an early break (converged, stalled,
    // or non-finite energy) rather than exhausting the iteration cap. It gates
    // the terminal-residual recompute below.
    bool brokeEarly = false;
    auto& groundFrictionNormalForce = contactScratch.groundFrictionNormalForce;
    auto& groundFrictionNormalDirection
        = contactScratch.groundFrictionNormalDirection;
    auto& selfContactFrictionContacts
        = contactScratch.selfContactFrictionContacts;
    const double frictionEpsilon
        = staticGroundFrictionVelocityThreshold() * timeStep;
    for (std::size_t iteration = 0; iteration < maxIterations; ++iteration) {
      ++stats.solverIterations;
      ++stats.objectiveEvaluations;

      // Assemble the self-contact barrier active set at the current positions,
      // held fixed for this outer iteration (standard IPC). Skip when the body
      // has no surface mesh (point-mass bodies have no self-contact).
      SelfContactBarrierInputs contactBarrier;
      if (!contactScratch.surfaceTriangles.empty()) {
        const double dHat = selfContactBarrierActivationDistance();
        dc::ContactCandidateOptions barrierOptions;
        barrierOptions.activationDistance = dHat;
        barrierOptions.exactDistanceFilter = true;
        barrierOptions.excludeIncidentPointTriangles = true;
        barrierOptions.excludeAdjacentEdges = true;
        dc::buildContactCandidatesSweep(
            scratch.next,
            contactScratch.surfaceTriangles,
            barrierOptions,
            contactScratch.barrierCandidates,
            contactScratch.sweepScratch);
        // Restrict point-triangle barrier candidates to surface-referenced
        // nodes (same mask the CCD path uses), so volumetric interior nodes do
        // not receive spurious barrier forces against their own shell.
        filterSurfaceContactPointCandidates(
            contactScratch.barrierCandidates,
            contactScratch.surfaceContactPointMask);
        ++stats.selfContactBarrierCandidateBuilds;
        contactBarrier.candidates = &contactScratch.barrierCandidates;
        contactBarrier.triangles = &contactScratch.surfaceTriangles;
        contactBarrier.squaredActivationDistance = dHat * dHat;
        contactBarrier.stiffness = selfContactBarrierStiffness();
      }

      // Lagged smoothed Coulomb friction for static-ground contact: the normal
      // force is sampled once per outer iteration (held fixed through the inner
      // line search), opposing each contacting node's tangential displacement
      // over the step. With mu == 0 or no ground contact this is a no-op.
      GroundFrictionInputs groundFriction;
      if (frictionCoefficient > 0.0
          && (!barriers.empty() || !sphereObstacles.empty()
              || !boxObstacles.empty() || !capsuleObstacles.empty())) {
        computeStaticGroundNormalForces(
            scratch.next,
            scratch.activeFixed,
            barriers,
            groundFrictionNormalForce,
            groundFrictionNormalDirection);
        addSphereObstacleNormalForces(
            scratch.next,
            scratch.activeFixed,
            sphereObstacles,
            groundFrictionNormalForce,
            groundFrictionNormalDirection);
        addBoxObstacleNormalForces(
            scratch.next,
            scratch.activeFixed,
            boxObstacles,
            groundFrictionNormalForce,
            groundFrictionNormalDirection);
        addCapsuleObstacleNormalForces(
            scratch.next,
            scratch.activeFixed,
            capsuleObstacles,
            groundFrictionNormalForce,
            groundFrictionNormalDirection);
        groundFriction.coefficient = frictionCoefficient;
        groundFriction.epsilon = frictionEpsilon;
        groundFriction.stepStartPositions = &scratch.previousStepPositions;
        groundFriction.laggedNormalForce = &groundFrictionNormalForce;
        groundFriction.laggedNormalDirection = &groundFrictionNormalDirection;
      }

      // Lagged smoothed self-contact friction over the active point-triangle
      // barrier set, sampled once per outer iteration. No-op without friction
      // or a surface mesh.
      SelfContactFrictionInputs selfContactFriction;
      if (frictionCoefficient > 0.0 && contactBarrier.candidates != nullptr) {
        buildSelfContactFrictionContacts(
            scratch.next, contactBarrier, selfContactFrictionContacts);
        selfContactFriction.coefficient = frictionCoefficient;
        selfContactFriction.epsilon = frictionEpsilon;
        selfContactFriction.stepStartPositions = &scratch.previousStepPositions;
        selfContactFriction.contacts = &selfContactFrictionContacts;
      }

      const double energy = evaluateDeformableObjective(
          state,
          model,
          scratch.next,
          scratch.inertialTargets,
          scratch.activeFixed,
          barriers,
          sphereObstacles,
          boxObstacles,
          capsuleObstacles,
          timeStep,
          &scratch.gradient,
          &contactBarrier,
          &stats.selfContactBarrierActiveContacts,
          &groundFriction,
          &selfContactFriction,
          femElasticityPtr,
          barrierStiffness);
      if (!std::isfinite(energy)) {
        brokeEarly = true;
        break;
      }

      const double gradSquared
          = gradientNormSquared(scratch.gradient, scratch.activeFixed);
      lastGradSquared = gradSquared;
      if (nb::projectedNewtonSquaredResidualConverged(
              gradSquared, gradientTolerance)) {
        brokeEarly = true;
        break;
      }

      // Mass-scaled steepest-descent direction: the graceful fallback used when
      // the dense Newton solve is skipped/fails, or when the Newton line search
      // cannot make progress.
      const auto fillSteepestDescentDirection = [&]() {
        for (std::size_t i = 0; i < nodeCount; ++i) {
          if (scratch.activeFixed[i] == 0u) {
            scratch.direction[i] = -(timeStep * timeStep / state.masses[i])
                                   * scratch.gradient[i];
          } else {
            scratch.direction[i].setZero();
          }
        }
      };

      // Armijo backtracking line search along scratch.direction, enforcing
      // every CCD limiter and the static-ground barrier. Commits the accepted
      // step into scratch.next and returns true on success.
      const auto runLineSearch = [&]() -> bool {
        double step = 1.0;
        for (std::size_t ls = 0; ls < maxLineSearchIterations; ++ls) {
          ++stats.lineSearchTrials;
          double directionalDerivative = buildLineSearchCandidate(
              scratch.next,
              scratch.direction,
              scratch.gradient,
              scratch.activeFixed,
              step,
              scratch.candidate);

          if (applySurfaceContactCcdLimit(
                  scratch.next,
                  scratch.direction,
                  scratch.gradient,
                  scratch.activeFixed,
                  contactScratch,
                  stats,
                  step,
                  scratch.candidate,
                  directionalDerivative)
              && applyInterBodySurfaceContactCcdLimit(
                  entity,
                  surfaceSnapshots,
                  scratch.next,
                  scratch.direction,
                  scratch.gradient,
                  scratch.activeFixed,
                  contactScratch,
                  stats,
                  step,
                  scratch.candidate,
                  directionalDerivative)
              && applyStaticRigidSurfaceCcdLimit(
                  rigidSurfaceSnapshots,
                  scratch.next,
                  scratch.direction,
                  scratch.gradient,
                  scratch.activeFixed,
                  contactScratch,
                  stats,
                  step,
                  scratch.candidate,
                  directionalDerivative)
              && applyMovingRigidSurfaceCcdLimit(
                  movingRigidSurfaceSnapshots,
                  scratch.next,
                  scratch.direction,
                  scratch.gradient,
                  scratch.activeFixed,
                  contactScratch,
                  stats,
                  step,
                  scratch.candidate,
                  directionalDerivative)
              && applyStaticGroundBarrierCcdLimit(
                  scratch.next,
                  scratch.direction,
                  scratch.gradient,
                  scratch.activeFixed,
                  barriers,
                  stats,
                  step,
                  scratch.candidate,
                  directionalDerivative)
              && directionalDerivative < -1e-24
              && satisfiesStaticGroundBarrier(
                  scratch.candidate, scratch.activeFixed, barriers)) {
            ++stats.objectiveEvaluations;
            const double candidateEnergy = evaluateDeformableObjective(
                state,
                model,
                scratch.candidate,
                scratch.inertialTargets,
                scratch.activeFixed,
                barriers,
                sphereObstacles,
                boxObstacles,
                capsuleObstacles,
                timeStep,
                nullptr,
                &contactBarrier,
                nullptr,
                &groundFriction,
                &selfContactFriction,
                femElasticityPtr,
                barrierStiffness);
            if (nb::satisfiesSufficientDecrease(
                    energy, candidateEnergy, directionalDerivative, armijo)) {
              // Record the accepted step's infinity norm (the actual per-node
              // position change). It is the converged-ness measure for stiff
              // barrier problems: it shrinks to ~0 at equilibrium even while
              // the raw gradient norm stays large because the barrier Hessian
              // is near-singular. scratch.candidate holds the new positions and
              // scratch.next the prior iterate (the swap follows).
              double stepInfinityNorm = 0.0;
              for (std::size_t i = 0; i < nodeCount; ++i) {
                if (scratch.activeFixed[i] != 0u) {
                  continue;
                }
                stepInfinityNorm = std::max(
                    stepInfinityNorm,
                    (scratch.candidate[i] - scratch.next[i])
                        .cwiseAbs()
                        .maxCoeff());
              }
              lastAcceptedStepInfinityNorm = stepInfinityNorm;
              std::swap(scratch.next, scratch.candidate);
              ++stats.acceptedLineSearchSteps;
              return true;
            }
          }

          ++stats.rejectedLineSearchCandidates;
          step *= backtrackingScale;
          if (step < minStep) {
            break;
          }
        }
        return false;
      };

      // Projected-Newton search direction (sparse PD Hessian solve); falls back
      // to mass-scaled steepest descent if the sparse solve is skipped or
      // fails. Newton lets the stiff barrier term converge cleanly.
      const bool newtonDirection = computeProjectedNewtonDirection(
          state,
          model,
          scratch.next,
          scratch.activeFixed,
          barriers,
          sphereObstacles,
          boxObstacles,
          capsuleObstacles,
          &contactBarrier,
          &groundFriction,
          &selfContactFriction,
          femElasticityPtr,
          timeStep,
          scratch.gradient,
          scratch.direction,
          contactScratch,
          stats,
          barrierStiffness,
          useIterativeSolver,
          useMatrixFreeSolver);
      if (newtonDirection) {
        ++stats.projectedNewtonSteps;
      } else {
        ++stats.projectedNewtonFallbacks;
        fillSteepestDescentDirection();
      }

      bool accepted = runLineSearch();
      if (!accepted && newtonDirection) {
        // The Newton direction was finite but the line search could not make
        // progress (an ill-conditioned barrier Hessian can round the
        // directional derivative to ~0). Degrade gracefully to mass-scaled
        // steepest descent within this iteration instead of stalling the solve.
        ++stats.projectedNewtonFallbacks;
        fillSteepestDescentDirection();
        accepted = runLineSearch();
      }

      if (!accepted) {
        brokeEarly = true;
        break;
      }
    }
    // If the solve ran to the iteration cap, the final accepted line-search
    // step changed scratch.next *after* lastGradSquared was recorded at the top
    // of that iteration, so the stored residual is the pre-final-step value.
    // Recompute the gradient at the terminal iterate -- rebuilding the
    // self-contact barrier active set and the lagged ground / self-contact
    // friction inputs there so the residual matches the in-loop objective --
    // and use it, so the convergence diagnostic reports the residual at solve
    // termination.
    if (!brokeEarly) {
      SelfContactBarrierInputs terminalBarrier;
      if (!contactScratch.surfaceTriangles.empty()) {
        const double dHat = selfContactBarrierActivationDistance();
        dc::ContactCandidateOptions barrierOptions;
        barrierOptions.activationDistance = dHat;
        barrierOptions.exactDistanceFilter = true;
        barrierOptions.excludeIncidentPointTriangles = true;
        barrierOptions.excludeAdjacentEdges = true;
        dc::buildContactCandidatesSweep(
            scratch.next,
            contactScratch.surfaceTriangles,
            barrierOptions,
            contactScratch.barrierCandidates,
            contactScratch.sweepScratch);
        filterSurfaceContactPointCandidates(
            contactScratch.barrierCandidates,
            contactScratch.surfaceContactPointMask);
        terminalBarrier.candidates = &contactScratch.barrierCandidates;
        terminalBarrier.triangles = &contactScratch.surfaceTriangles;
        terminalBarrier.squaredActivationDistance = dHat * dHat;
        terminalBarrier.stiffness = selfContactBarrierStiffness();
      }
      GroundFrictionInputs terminalGroundFriction;
      if (frictionCoefficient > 0.0
          && (!barriers.empty() || !sphereObstacles.empty()
              || !boxObstacles.empty() || !capsuleObstacles.empty())) {
        computeStaticGroundNormalForces(
            scratch.next,
            scratch.activeFixed,
            barriers,
            groundFrictionNormalForce,
            groundFrictionNormalDirection);
        addSphereObstacleNormalForces(
            scratch.next,
            scratch.activeFixed,
            sphereObstacles,
            groundFrictionNormalForce,
            groundFrictionNormalDirection);
        addBoxObstacleNormalForces(
            scratch.next,
            scratch.activeFixed,
            boxObstacles,
            groundFrictionNormalForce,
            groundFrictionNormalDirection);
        addCapsuleObstacleNormalForces(
            scratch.next,
            scratch.activeFixed,
            capsuleObstacles,
            groundFrictionNormalForce,
            groundFrictionNormalDirection);
        terminalGroundFriction.coefficient = frictionCoefficient;
        terminalGroundFriction.epsilon = frictionEpsilon;
        terminalGroundFriction.stepStartPositions
            = &scratch.previousStepPositions;
        terminalGroundFriction.laggedNormalForce = &groundFrictionNormalForce;
        terminalGroundFriction.laggedNormalDirection
            = &groundFrictionNormalDirection;
      }
      SelfContactFrictionInputs terminalSelfContactFriction;
      if (frictionCoefficient > 0.0 && terminalBarrier.candidates != nullptr) {
        buildSelfContactFrictionContacts(
            scratch.next, terminalBarrier, selfContactFrictionContacts);
        terminalSelfContactFriction.coefficient = frictionCoefficient;
        terminalSelfContactFriction.epsilon = frictionEpsilon;
        terminalSelfContactFriction.stepStartPositions
            = &scratch.previousStepPositions;
        terminalSelfContactFriction.contacts = &selfContactFrictionContacts;
      }
      const double terminalEnergy = evaluateDeformableObjective(
          state,
          model,
          scratch.next,
          scratch.inertialTargets,
          scratch.activeFixed,
          barriers,
          sphereObstacles,
          boxObstacles,
          capsuleObstacles,
          timeStep,
          &scratch.gradient,
          &terminalBarrier,
          nullptr,
          &terminalGroundFriction,
          &terminalSelfContactFriction,
          femElasticityPtr,
          barrierStiffness);
      if (std::isfinite(terminalEnergy)) {
        lastGradSquared
            = gradientNormSquared(scratch.gradient, scratch.activeFixed);
      }
    }
    // Record the worst-case solve residual across the step's bodies (the
    // gradient norm at termination), a convergence diagnostic for the
    // benchmark statistics.
    stats.finalGradientResidualNorm = std::max(
        stats.finalGradientResidualNorm,
        nb::projectedNewtonResidualNormFromSquared(lastGradSquared));
    // Converged-ness measure that stays meaningful for stiff barrier problems:
    // the worst-case last accepted step (infinity norm) across the step's
    // bodies. It tends to zero at equilibrium even when the gradient residual
    // stays large because the barrier Hessian is near-singular, so it is the
    // honest companion to finalGradientResidualNorm.
    stats.finalStepInfinityNorm
        = std::max(stats.finalStepInfinityNorm, lastAcceptedStepInfinityNorm);

    // Friction dissipation/active-contact diagnostics at the converged iterate,
    // using the final lagged ground normals and self-contact friction set.
    accumulateFrictionDiagnostics(
        scratch.next,
        scratch.previousStepPositions,
        scratch.activeFixed,
        frictionCoefficient,
        frictionEpsilon,
        groundFrictionNormalForce,
        groundFrictionNormalDirection,
        selfContactFrictionContacts,
        stats.frictionDissipation,
        stats.activeFrictionContacts);

    // Closest-approach (minimum) distance over the active self-contact barrier
    // set at the converged iterate -- the IPC intersection-free statistic. The
    // barrier candidate buffer holds the terminal active set: rebuilt at
    // scratch.next above when the solve hit the iteration cap, otherwise the
    // last in-loop set at the converged iterate (the final accepted step was
    // sub-tolerance, so the iterate barely moved), mirroring how the friction
    // diagnostics reuse their lagged sets.
    if (!contactScratch.surfaceTriangles.empty()) {
      const double dHat = selfContactBarrierActivationDistance();
      SelfContactBarrierInputs diagnosticBarrier;
      diagnosticBarrier.candidates = &contactScratch.barrierCandidates;
      diagnosticBarrier.triangles = &contactScratch.surfaceTriangles;
      diagnosticBarrier.squaredActivationDistance = dHat * dHat;
      diagnosticBarrier.stiffness = selfContactBarrierStiffness();
      double bodyMinContactDistance = 0.0;
      const std::size_t bodyActiveContacts
          = accumulateContactDistanceDiagnostics(
              scratch.next, diagnosticBarrier, bodyMinContactDistance);
      if (bodyActiveContacts > 0) {
        // Fold across bodies: sum the active counts, take the minimum closest
        // approach. Gate the min on the running count (not on a zero-distance
        // sentinel) so the first contacting body always seeds it.
        if (stats.convergedActiveContactCount == 0) {
          stats.minActiveContactDistance = bodyMinContactDistance;
        } else {
          stats.minActiveContactDistance = std::min(
              stats.minActiveContactDistance, bodyMinContactDistance);
        }
        stats.convergedActiveContactCount += bodyActiveContacts;
      }
    }
  }

  for (std::size_t i = 0; i < nodeCount; ++i) {
    const Eigen::Vector3d previous = scratch.previousStepPositions[i];
    state.previousPositions[i] = previous;
    if (scratch.activeFixed[i] != 0u) {
      state.positions[i] = scratch.next[i];
      state.velocities[i] = (state.positions[i] - previous) / timeStep;
      continue;
    }
    state.positions[i] = scratch.next[i];
    state.velocities[i] = (state.positions[i] - previous) / timeStep;
  }
}

//==============================================================================
//==============================================================================
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
void prepareRigidIpcSolverScratch(
    const std::vector<RigidIpcRuntimeBody>& runtimeBodies,
    std::vector<RigidIpcRuntimeBody>& solverBodies,
    std::vector<sxdetail::RigidIpcBarrierSurface>& surfaces,
    std::vector<sxdetail::RigidIpcBodyDynamicsTerm>& dynamicsTerms)
{
  std::size_t solverCount = 0u;
  for (const auto& body : runtimeBodies) {
    if (!body.hasSupportedSurface) {
      continue;
    }

    if (solverCount == solverBodies.size()) {
      solverBodies.emplace_back();
    }
    copyRigidIpcRuntimeBodyPreservingSurfaceCapacity(
        body, solverBodies[solverCount]);
    solverBodies[solverCount].surfaceIndex = solverCount;
    solverBodies[solverCount].surface.body = solverCount;
    ++solverCount;
  }

  solverBodies.resize(solverCount);
  surfaces.resize(solverCount);
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
RigidIpcBdf2HistoryEntry* findRigidIpcBdf2History(
    std::vector<RigidIpcBdf2HistoryEntry>& histories, const entt::entity entity)
{
  auto it = std::find_if(
      histories.begin(), histories.end(), [entity](const auto& history) {
        return history.entity == entity;
      });
  return it == histories.end() ? nullptr : std::addressof(*it);
}

//==============================================================================
RigidIpcBdf2HistoryEntry& findOrCreateRigidIpcBdf2History(
    std::vector<RigidIpcBdf2HistoryEntry>& histories, const entt::entity entity)
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
void pruneRigidIpcBdf2Histories(
    std::vector<RigidIpcBdf2HistoryEntry>& histories,
    const std::vector<RigidIpcRuntimeBody>& bodies)
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
      surface.vertices = shape.vertices;
      surface.triangles = shape.triangles;
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
void applyRigidIpcBdf2DynamicsTerms(
    const World& world,
    std::vector<RigidIpcBdf2HistoryEntry>& histories,
    std::vector<RigidIpcRuntimeBody>& bodies,
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
void collectRigidIpcRuntimeBodies(
    const World& world,
    RigidIpcSolverStats& stats,
    std::vector<RigidIpcRuntimeBody>& bodies)
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
      bodies.emplace_back();
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
  bodies.resize(outputCount);
}

//==============================================================================
bool appendRigidIpcDeformableSurface(
    entt::entity entity,
    const comps::DeformableNodeState& state,
    const comps::DeformableMeshTopology& topology,
    double frictionCoefficient,
    RigidIpcSolverStats& stats,
    std::vector<RigidIpcRuntimeBody>& bodies)
{
  if (state.positions.empty() || topology.surfaceTriangles.empty()) {
    return false;
  }

  std::vector<Eigen::Vector3i> triangles;
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

  bodies.emplace_back();
  RigidIpcRuntimeBody& body = bodies.back();
  resetRigidIpcRuntimeBodyPreservingSurface(body);
  body.entity = entity;
  body.hasSupportedSurface = true;
  body.surfaceIndex = stats.surfaceCount;
  body.surface.body = bodies.size() - 1u;
  body.surface.pose = sxdetail::RigidIpcPose{};
  body.surface.vertices = state.positions;
  body.surface.triangles = std::move(triangles);
  body.surface.dynamic = false;
  body.surface.kinematic = false;
  body.surface.frictionCoefficient = std::isfinite(frictionCoefficient)
                                         ? std::max(0.0, frictionCoefficient)
                                         : 0.0;
  ++stats.surfaceCount;
  ++stats.mixedDomainDeformableSurfaceCount;
  return true;
}

//==============================================================================
void appendRigidIpcDeformableSurfaces(
    const World& world,
    RigidIpcSolverStats& stats,
    std::vector<RigidIpcRuntimeBody>& bodies)
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
        entity, state, topology, material.frictionCoefficient, stats, bodies);
  }
}

//==============================================================================
std::vector<Eigen::Vector3d> rigidIpcSurfaceWorldVertices(
    const sxdetail::RigidIpcBarrierSurface& surface,
    const sxdetail::RigidIpcPose& pose)
{
  std::vector<Eigen::Vector3d> vertices;
  vertices.reserve(surface.vertices.size());
  for (const Eigen::Vector3d& vertex : surface.vertices) {
    vertices.push_back(sxdetail::transformRigidIpcPoint(vertex, pose));
  }
  return vertices;
}

//==============================================================================
void collectRigidIpcMixedDomainCandidateStats(
    const dart::simulation::detail::WorldRegistry& registry,
    const std::vector<RigidIpcRuntimeBody>& bodies,
    const double activationDistance,
    RigidIpcSolverStats& stats,
    std::vector<nb::MixedDomainSurface>& surfaces)
{
  if (stats.mixedDomainDeformableSurfaceCount == 0u) {
    stats.mixedDomainSurfaceCount = 0u;
    stats.mixedDomainCandidateCount = 0u;
    stats.mixedDomainActiveBarrierCount = 0u;
    stats.mixedDomainBarrierValue = 0.0;
    return;
  }

  surfaces.clear();
  surfaces.reserve(bodies.size());
  for (std::size_t i = 0; i < bodies.size(); ++i) {
    const RigidIpcRuntimeBody& body = bodies[i];
    if (!body.hasSupportedSurface) {
      continue;
    }

    const bool deformableSurface
        = body.entity != entt::null
          && registry.all_of<comps::DeformableBodyTag>(body.entity);
    nb::MixedDomainSurface surface;
    surface.domain = deformableSurface ? nb::MixedDomainType::Deformable
                                       : nb::MixedDomainType::Rigid;
    surface.domainInstance = i;
    surface.active = true;
    surface.dynamic = body.surface.dynamic;
    surface.frictionCoefficient = body.surface.frictionCoefficient;
    surface.startVertices = rigidIpcSurfaceWorldVertices(
        body.surface,
        body.surface.kinematic ? body.surface.kinematicStartPose
                               : body.surface.pose);
    surface.endVertices
        = rigidIpcSurfaceWorldVertices(body.surface, body.surface.pose);
    surface.triangles = body.surface.triangles;
    surfaces.push_back(std::move(surface));
  }

  nb::MixedDomainCandidateOptions options;
  options.activationDistance = activationDistance;
  options.exactDistanceFilter = true;
  const nb::MixedDomainCandidateSet candidateSet
      = nb::buildMixedDomainContactCandidates(surfaces, options);
  const nb::MixedDomainBarrierDiagnostics diagnostics
      = nb::evaluateMixedDomainBarrierDiagnostics(
          surfaces, candidateSet, activationDistance);
  stats.mixedDomainSurfaceCount = candidateSet.stats.surfaceCount;
  stats.mixedDomainCandidateCount = candidateSet.candidates.size();
  stats.mixedDomainActiveBarrierCount = diagnostics.activeBarrierCount;
  stats.mixedDomainBarrierValue = diagnostics.value;
}

//==============================================================================
std::size_t findRuntimeBodyIndex(
    const std::vector<RigidIpcRuntimeBody>& bodies, const entt::entity entity)
{
  for (std::size_t i = 0; i < bodies.size(); ++i) {
    if (bodies[i].entity == entity) {
      return i;
    }
  }
  return bodies.size();
}

//==============================================================================
std::size_t findSolverBodyIndex(
    const std::vector<RigidIpcRuntimeBody>& bodies, const entt::entity entity)
{
  for (std::size_t i = 0; i < bodies.size(); ++i) {
    if (bodies[i].entity == entity) {
      return i;
    }
  }
  return bodies.size();
}

//==============================================================================
bool appendRigidIpcHingeAxisConstraint(
    const std::size_t bodyA,
    const std::size_t bodyB,
    const Eigen::Vector3d& parentLocalAxis,
    const Eigen::Quaterniond& targetParentToChild,
    std::vector<sxdetail::RigidIpcArticulationConstraintInput>& constraints)
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
void collectRigidIpcArticulationConstraints(
    const World& world,
    const std::vector<RigidIpcRuntimeBody>& solverBodies,
    std::vector<sxdetail::RigidIpcArticulationConstraintInput>& constraints)
{
  const auto& registry = dart::simulation::detail::registryOf(world);
  constraints.clear();

  auto view
      = registry.view<comps::Joint, dvbd::AvbdRigidWorldPointJointConfig>();
  for (const auto jointEntity : view) {
    const auto& joint = view.get<comps::Joint>(jointEntity);
    const auto& config
        = view.get<dvbd::AvbdRigidWorldPointJointConfig>(jointEntity);
    if (!config.enabled || joint.broken || joint.breakForce > 0.0
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
void clearKinematicBodyStepTraces(
    World& world, std::vector<entt::entity>& tracedEntities)
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
    std::vector<RigidIpcBdf2HistoryEntry>* bdf2Histories)
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
    const std::vector<RigidIpcRuntimeBody>& bodies,
    const sxdetail::RigidIpcProjectedNewtonSolveResult& result,
    std::vector<RigidIpcBdf2HistoryEntry>* bdf2Histories,
    std::vector<entt::entity>& writebackEntities,
    std::vector<entt::entity>& orderedEntities,
    std::vector<int>& visitState)
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
    const std::vector<RigidIpcRuntimeBody>& bodies,
    const std::vector<entt::entity>& blockedKinematicEntities,
    std::vector<entt::entity>& writebackEntities,
    std::vector<entt::entity>& orderedEntities,
    std::vector<int>& visitState)
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
    const std::vector<RigidIpcRuntimeBody>& solverBodies,
    const std::size_t solverBodyIndex,
    std::vector<entt::entity>& blockedEntities)
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
    const std::vector<RigidIpcRuntimeBody>& solverBodies,
    const sxdetail::RigidIpcProjectedNewtonSolveResult& result,
    std::vector<entt::entity>& blockedEntities)
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
    const std::vector<RigidIpcRuntimeBody>& runtimeBodies,
    const std::vector<RigidIpcRuntimeBody>& solverBodies,
    const sxdetail::RigidIpcProjectedNewtonSolveResult& result,
    std::vector<entt::entity>& blockedEntities,
    std::vector<entt::entity>& writebackEntities,
    std::vector<entt::entity>& orderedEntities,
    std::vector<int>& visitState)
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
    const std::vector<RigidIpcRuntimeBody>& bodies,
    const sxdetail::RigidIpcProjectedNewtonSolveResult& result,
    std::vector<double>& contactPowerSum,
    std::vector<std::uint8_t>& sawNonStationaryContactBody,
    std::vector<std::uint8_t>& stationaryContactBody)
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
struct DeformableDynamicsStage::Scratch
{
  std::vector<StaticGroundBarrier> barriers;
  std::vector<SphereObstacleBarrier> sphereObstacles;
  std::vector<BoxObstacleBarrier> boxObstacles;
  std::vector<CapsuleObstacleBarrier> capsuleObstacles;
  std::vector<SurfaceContactSnapshot> surfaceSnapshots;
  std::vector<SurfaceContactSnapshot> rigidSurfaceSnapshots;
  std::vector<SurfaceContactSnapshot> movingRigidSurfaceSnapshots;
  std::size_t surfaceSnapshotCount = 0;
  std::size_t rigidSurfaceSnapshotCount = 0;
  std::size_t movingRigidSurfaceSnapshotCount = 0;
};

//==============================================================================
struct RigidBodyVelocityStage::Scratch
{
  RigidBodyForceBatch forces;
};

//==============================================================================
struct BatchedRigidBodyIntegrationStage::Scratch
{
  RigidBodyForceBatch forces;
  RigidBodyStateBatch state;
  RigidBodyStateBatch initialState;
  RigidBodyModelBatch model;
  std::vector<entt::entity> frameUpdateOrder;
  std::vector<int> visitState;
};

//==============================================================================
RigidBodyVelocityStage::RigidBodyVelocityStage()
  : m_scratch(std::make_unique<Scratch>())
{
}

//==============================================================================
RigidBodyVelocityStage::~RigidBodyVelocityStage() = default;

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
    m_scratch = std::make_unique<Scratch>();
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
    m_scratch = std::make_unique<Scratch>();
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
    integrateRigidBodyPosition(registry, entity, timeStep);
  }
}

//==============================================================================
struct RigidBodyContactStage::AvbdScratch
{
  void clear()
  {
    dvbd::clearAvbdRigidWorldContactSnapshot(snapshot);
    pointJoints.clear();
    distanceSprings.clear();
    buildScratch.rowCounters.clear();
    buildScratch.contactRowOrder.clear();
    normalInventory.records().clear();
    frictionInventory.records().clear();
    jointLinearInventory.records().clear();
    jointAngularInventory.records().clear();
    motorInventory.records().clear();
    distanceSpringInventory.records().clear();
    solveScratch.clear();
  }

  void reserve(
      std::size_t bodyCapacity,
      std::size_t contactCapacity,
      std::size_t jointCapacity,
      std::size_t distanceSpringCapacity)
  {
    dvbd::reserveAvbdRigidWorldContactSnapshot(
        snapshot,
        bodyCapacity,
        contactCapacity,
        jointCapacity,
        jointCapacity,
        distanceSpringCapacity);
    pointJoints.reserve(jointCapacity);
    distanceSprings.reserve(distanceSpringCapacity);
    buildScratch.rowCounters.reserve(
        std::max(
            contactCapacity, std::max(jointCapacity, distanceSpringCapacity)));
    buildScratch.contactRowOrder.reserve(contactCapacity);
    dvbd::reserveAvbdRigidWorldContactSolveScratch(
        solveScratch,
        contactCapacity,
        jointCapacity,
        jointCapacity,
        bodyCapacity,
        distanceSpringCapacity);
    normalInventory.reserve(contactCapacity);
    frictionInventory.reserve(2u * contactCapacity);
    jointLinearInventory.reserve(3u * jointCapacity);
    jointAngularInventory.reserve(3u * jointCapacity);
    motorInventory.reserve(jointCapacity);
    distanceSpringInventory.reserve(distanceSpringCapacity);
  }

  dvbd::AvbdRigidWorldContactSnapshot snapshot;
  std::vector<dvbd::AvbdRigidWorldPointJointInput> pointJoints;
  std::vector<dvbd::AvbdRigidWorldDistanceSpringInput> distanceSprings;
  dvbd::AvbdRigidWorldContactBuildScratch buildScratch;
  dvbd::AvbdRigidWorldContactSolveScratch solveScratch;
  dvbd::AvbdScalarRowInventory normalInventory;
  dvbd::AvbdScalarRowInventory frictionInventory;
  dvbd::AvbdScalarRowInventory jointLinearInventory;
  dvbd::AvbdScalarRowInventory jointAngularInventory;
  dvbd::AvbdScalarRowInventory motorInventory;
  dvbd::AvbdScalarRowInventory distanceSpringInventory;
};

//==============================================================================
struct RigidBodyContactStage::ContactScratch
{
  struct NormalConstraint
  {
    entt::entity bodyA;
    entt::entity bodyB;
    comps::Transform* transformA;
    comps::Transform* transformB;
    comps::Velocity* velocityA;
    comps::Velocity* velocityB;
    Eigen::Vector3d normal;
    Eigen::Vector3d armA;
    Eigen::Vector3d armB;
    bool staticA;
    bool staticB;
    double invMassA;
    double invMassB;
    Eigen::Matrix3d invInertiaA;
    Eigen::Matrix3d invInertiaB;
    double effectiveMass;
    double inverseEffectiveMass;
    double depth;
    double restitutionVelocity;
    double normalImpulse;
    Eigen::Vector3d normalArmCrossA;
    Eigen::Vector3d normalArmCrossB;
    Eigen::Vector3d normalLinearDeltaA;
    Eigen::Vector3d normalLinearDeltaB;
    Eigen::Vector3d normalAngularDeltaA;
    Eigen::Vector3d normalAngularDeltaB;
    Eigen::Vector3d tangent1;
    Eigen::Vector3d tangent2;
    double tangentMass1;
    double tangentMass2;
    double tangentImpulse1;
    double tangentImpulse2;
    double friction;
  };

  std::vector<NormalConstraint> constraints;
};

//==============================================================================
RigidBodyContactStage::RigidBodyContactStage(std::size_t iterations)
  : m_iterations(std::max<std::size_t>(1, iterations)),
    m_avbdScratch(std::make_unique<AvbdScratch>()),
    m_contactScratch(std::make_unique<ContactScratch>())
{
}

//==============================================================================
RigidBodyContactStage::~RigidBodyContactStage() = default;

//==============================================================================
std::string_view RigidBodyContactStage::getName() const noexcept
{
  return "rigid_body_contact";
}

//==============================================================================
ComputeStageMetadata RigidBodyContactStage::getMetadata() const noexcept
{
  return {
      ComputeStageDomain::Constraint,
      ComputeStageAcceleration::TaskParallel
          | ComputeStageAcceleration::DataLocality};
}

//==============================================================================
void RigidBodyContactStage::prepare(World& world)
{
  if (m_contactScratch == nullptr) {
    m_contactScratch = std::make_unique<ContactScratch>();
  }
  auto& constraints = m_contactScratch->constraints;
  constraints.clear();

  const auto& registry = dart::simulation::detail::registryOf(world);
  const bool skipContactQuery = shouldSkipRigidBodyContactQuery(registry);
  std::size_t contactCapacity = 0u;
  if (!skipContactQuery) {
    std::size_t collisionShapeCount = 0;
    const auto geometryView = registry.view<comps::CollisionGeometry>();
    for (const entt::entity entity : geometryView) {
      collisionShapeCount
          += geometryView.get<comps::CollisionGeometry>(entity).shapes.size();
    }
    constexpr std::size_t kContactConstraintCapacityPerShape = 4u;
    const std::size_t contactConstraintCapacity
        = collisionShapeCount <= std::numeric_limits<std::size_t>::max()
                                     / kContactConstraintCapacityPerShape
              ? collisionShapeCount * kContactConstraintCapacityPerShape
              : std::numeric_limits<std::size_t>::max();
    contactCapacity = contactConstraintCapacity;
    constraints.reserve(contactCapacity);
    world.prepareCollisionQueryCache(
        CollisionQueryOptions{}, /*includeShapeContactDetails=*/false);
  }

  const bool mayHavePointJointConfigs
      = dvbd::mayHaveAvbdRigidWorldPointJointConfigs(registry);
  const std::size_t jointCapacity
      = mayHavePointJointConfigs
            ? registry
                  .view<comps::Joint, dvbd::AvbdRigidWorldPointJointConfig>()
                  .size_hint()
            : 0u;
  const auto* distanceSpringStorage
      = registry.storage<dvbd::AvbdRigidWorldDistanceSpringConfig>();
  const std::size_t distanceSpringCapacity
      = distanceSpringStorage != nullptr ? distanceSpringStorage->size() : 0u;
  const std::size_t avbdContactCapacity
      = contactCapacity != 0u && mayHaveRigidAvbdContactConfigs(registry)
            ? contactCapacity
            : 0u;
  if (avbdContactCapacity == 0u && jointCapacity == 0u
      && distanceSpringCapacity == 0u) {
    if (m_avbdScratch != nullptr) {
      m_avbdScratch->clear();
    }
    return;
  }

  if (m_avbdScratch == nullptr) {
    m_avbdScratch = std::make_unique<AvbdScratch>();
  }
  const std::size_t maxSize = std::numeric_limits<std::size_t>::max();
  const std::size_t rowEndpointCapacity
      = avbdContactCapacity <= maxSize - jointCapacity
                && avbdContactCapacity + jointCapacity
                       <= maxSize - distanceSpringCapacity
            ? avbdContactCapacity + jointCapacity + distanceSpringCapacity
            : maxSize;
  const std::size_t bodyCapacity = rowEndpointCapacity <= maxSize / 2u
                                       ? 2u * rowEndpointCapacity
                                       : maxSize;
  m_avbdScratch->reserve(
      bodyCapacity, avbdContactCapacity, jointCapacity, distanceSpringCapacity);
}

//==============================================================================
void RigidBodyContactStage::execute(World& world, ComputeExecutor& /*executor*/)
{
  auto& registry = dart::simulation::detail::registryOf(world);

  const auto projectAvbdRigidPointJoints = [&]() {
    const bool hasPointJointConfigs
        = dvbd::mayHaveAvbdRigidWorldPointJointConfigs(registry);
    const bool hasDistanceSpringConfigs
        = dvbd::mayHaveAvbdRigidWorldDistanceSpringConfigs(registry);
    if (!hasPointJointConfigs && !hasDistanceSpringConfigs) {
      return false;
    }

    if (m_avbdScratch == nullptr) {
      m_avbdScratch = std::make_unique<AvbdScratch>();
    }
    auto& scratch = *m_avbdScratch;
    if (hasPointJointConfigs) {
      dvbd::extractAvbdRigidWorldPointJointInputsInto(
          registry, scratch.pointJoints, /*includeWorldAnchors=*/false);
    } else {
      scratch.pointJoints.clear();
    }
    if (hasDistanceSpringConfigs) {
      dvbd::extractAvbdRigidWorldDistanceSpringInputsInto(
          registry, scratch.distanceSprings, /*includeWorldAnchors=*/false);
    } else {
      scratch.distanceSprings.clear();
    }
    if (scratch.pointJoints.empty() && scratch.distanceSprings.empty()) {
      return false;
    }

    dvbd::clearAvbdRigidWorldContactSnapshot(scratch.snapshot);
    const std::size_t appendedJoints
        = scratch.pointJoints.empty() ? 0u
                                      : dvbd::appendAvbdRigidWorldPointJoints(
                                            registry,
                                            scratch.pointJoints,
                                            scratch.snapshot,
                                            scratch.buildScratch);
    const std::size_t appendedDistanceSprings
        = scratch.distanceSprings.empty()
              ? 0u
              : dvbd::appendAvbdRigidWorldDistanceSprings(
                    registry,
                    scratch.distanceSprings,
                    scratch.snapshot,
                    scratch.buildScratch);
    if (appendedJoints == 0u && appendedDistanceSprings == 0u) {
      return false;
    }

    const double timeStep = world.getTimeStep();
    dvbd::predictAvbdRigidWorldContactInertialTargets(
        registry, scratch.snapshot, timeStep);

    dvbd::AvbdRigidWorldContactSolveOptions solveOptions;
    solveOptions.descent.iterations = m_iterations;
    solveOptions.descent.regularization = 1e-12;
    const dvbd::AvbdRigidWorldContactSolveResult solveResult
        = dvbd::solveAvbdRigidWorldContactSnapshot(
            scratch.snapshot,
            scratch.normalInventory,
            scratch.frictionInventory,
            scratch.jointLinearInventory,
            scratch.jointAngularInventory,
            scratch.motorInventory,
            scratch.distanceSpringInventory,
            timeStep,
            scratch.solveScratch,
            solveOptions);
    (void)dvbd::markAvbdRigidWorldFracturedPointJoints(
        registry, scratch.snapshot, solveResult.fracturedJointIndices);
    if (solveResult.jointLinearRows == 0u && solveResult.jointAngularRows == 0u
        && solveResult.motorRows == 0u
        && solveResult.distanceSpringRows == 0u) {
      return false;
    }

    const dvbd::AvbdRigidWorldContactApplyResult projection
        = dvbd::applyAvbdRigidWorldContactVelocityProjection(
            registry, scratch.snapshot, timeStep);
    return projection.bodies != 0u;
  };

  if (shouldSkipRigidBodyContactQuery(registry)) {
    if (projectAvbdRigidPointJoints()) {
      return;
    }

    if (m_avbdScratch != nullptr) {
      m_avbdScratch->clear();
    }
    return;
  }

  const bool mayUseAvbdContactDetails
      = mayHaveRigidAvbdContactConfigs(registry);
  const auto& contacts = world.queryContacts(
      CollisionQueryOptions{},
      /*includeShapeContactDetails=*/mayUseAvbdContactDetails);
  if (contacts.empty()) {
    if (projectAvbdRigidPointJoints()) {
      return;
    }

    if (m_avbdScratch != nullptr) {
      m_avbdScratch->clear();
    }
    return;
  }

  // Internal AVBD rigid contact path (PLAN-104 AVBD): when every active contact
  // has at least one body carrying the private opt-in config, assemble the
  // contact manifold into 6-DOF point-pair rows, solve against the
  // velocity-predicted inertial target, then project the solved displacement
  // back into velocities. The standard rigid position stage still advances
  // poses, so default pipeline ordering is unchanged. Unsupported envelopes
  // fall through to the sequential-impulse path below.
  if (const auto avbdConfig = rigidAvbdContactStageConfig(registry, contacts)) {
    if (m_avbdScratch == nullptr) {
      m_avbdScratch = std::make_unique<AvbdScratch>();
    }
    auto& scratch = *m_avbdScratch;

    dvbd::AvbdRigidWorldContactOptions contactOptions;
    contactOptions.startStiffness = std::max(0.0, avbdConfig->startStiffness);
    contactOptions.maxStiffness
        = std::max(contactOptions.startStiffness, avbdConfig->maxStiffness);
    dvbd::buildAvbdRigidWorldContactSnapshot(
        registry,
        contacts,
        scratch.snapshot,
        scratch.buildScratch,
        contactOptions);
    std::size_t appendedJoints = 0u;
    std::size_t appendedDistanceSprings = 0u;
    if (dvbd::mayHaveAvbdRigidWorldPointJointConfigs(registry)) {
      dvbd::extractAvbdRigidWorldPointJointInputsInto(
          registry, scratch.pointJoints, /*includeWorldAnchors=*/false);
      if (!scratch.pointJoints.empty()) {
        appendedJoints = dvbd::appendAvbdRigidWorldPointJoints(
            registry,
            scratch.pointJoints,
            scratch.snapshot,
            scratch.buildScratch);
      }
    } else {
      scratch.pointJoints.clear();
    }
    if (dvbd::mayHaveAvbdRigidWorldDistanceSpringConfigs(registry)) {
      dvbd::extractAvbdRigidWorldDistanceSpringInputsInto(
          registry, scratch.distanceSprings, /*includeWorldAnchors=*/false);
      if (!scratch.distanceSprings.empty()) {
        appendedDistanceSprings = dvbd::appendAvbdRigidWorldDistanceSprings(
            registry,
            scratch.distanceSprings,
            scratch.snapshot,
            scratch.buildScratch);
      }
    } else {
      scratch.distanceSprings.clear();
    }

    if (scratch.snapshot.contacts.size() == contacts.size()
        && (!scratch.snapshot.contacts.empty() || appendedJoints != 0u
            || appendedDistanceSprings != 0u)) {
      const double timeStep = world.getTimeStep();
      dvbd::predictAvbdRigidWorldContactInertialTargets(
          registry, scratch.snapshot, timeStep);

      dvbd::AvbdRigidWorldContactSolveOptions solveOptions;
      solveOptions.warmStart.alpha = avbdConfig->alpha;
      solveOptions.warmStart.gamma = avbdConfig->gamma;
      solveOptions.warmStart.maxStiffness = contactOptions.maxStiffness;
      solveOptions.row.alpha = avbdConfig->alpha;
      solveOptions.row.beta = avbdConfig->beta;
      solveOptions.row.maxStiffness = contactOptions.maxStiffness;
      solveOptions.friction.alpha = avbdConfig->alpha;
      solveOptions.friction.beta = avbdConfig->beta;
      solveOptions.friction.maxStiffness = contactOptions.maxStiffness;
      solveOptions.distanceSpring.beta = avbdConfig->beta;
      solveOptions.distanceSpring.maxStiffness = contactOptions.maxStiffness;
      solveOptions.descent.iterations = m_iterations;
      solveOptions.descent.regularization = 1e-12;

      const dvbd::AvbdRigidWorldContactSolveResult solveResult
          = dvbd::solveAvbdRigidWorldContactSnapshot(
              scratch.snapshot,
              scratch.normalInventory,
              scratch.frictionInventory,
              scratch.jointLinearInventory,
              scratch.jointAngularInventory,
              scratch.motorInventory,
              scratch.distanceSpringInventory,
              timeStep,
              scratch.solveScratch,
              solveOptions);
      (void)dvbd::markAvbdRigidWorldFracturedPointJoints(
          registry, scratch.snapshot, solveResult.fracturedJointIndices);
      if (solveResult.normalRows != 0u || solveResult.frictionRows != 0u
          || solveResult.jointLinearRows != 0u
          || solveResult.jointAngularRows != 0u || solveResult.motorRows != 0u
          || solveResult.distanceSpringRows != 0u) {
        const dvbd::AvbdRigidWorldContactApplyResult projection
            = dvbd::applyAvbdRigidWorldContactVelocityProjection(
                registry, scratch.snapshot, timeStep);
        if (projection.bodies != 0u) {
          return;
        }
      }
    }

    m_avbdScratch->clear();
  }

  if (projectAvbdRigidPointJoints()) {
    // Keep the AVBD joint projection active while ordinary contacts continue
    // through the selected non-AVBD contact solver below.
  } else if (m_avbdScratch != nullptr) {
    m_avbdScratch->clear();
  }

  // Opt-in boxed-LCP path (PLAN-080 WS4): assemble and solve the Coulomb
  // normal+tangent Delassus system with the pivoting Dantzig solver, applying
  // the resulting impulses to body velocities. The default SequentialImpulse
  // path below is unchanged.
  if (world.getContactSolverMethod() == ContactSolverMethod::BoxedLcp) {
    (void)detail::solveBoxedLcpContacts(
        registry, contacts, world.getTimeStep());
    resolveRigidBodyContactPositions(registry, contacts, world.getTimeStep());
    return;
  }

  const auto contactPointVelocity = [](const comps::Velocity& velocity,
                                       const Eigen::Vector3d& arm,
                                       bool isStatic) -> Eigen::Vector3d {
    if (isStatic) {
      return Eigen::Vector3d::Zero();
    }
    return velocity.linear + velocity.angular.cross(arm);
  };
  const auto normalPointVelocity = [](const comps::Velocity& velocity,
                                      const Eigen::Vector3d& normal,
                                      const Eigen::Vector3d& normalArmCross,
                                      bool isStatic) -> double {
    if (isStatic) {
      return 0.0;
    }
    return velocity.linear.dot(normal) + velocity.angular.dot(normalArmCross);
  };

  if (m_contactScratch == nullptr) {
    m_contactScratch = std::make_unique<ContactScratch>();
  }
  auto& constraints = m_contactScratch->constraints;
  constraints.clear();
  constraints.reserve(contacts.size());
  bool hasFrictionConstraints = false;
  for (const auto& contact : contacts) {
    const auto entityA = detail::toRegistryEntity(contact.bodyA.getEntity());
    const auto entityB = detail::toRegistryEntity(contact.bodyB.getEntity());

    // This sequential-impulse solver handles rigid-body pairs only; contacts
    // involving multibody links are resolved by the articulated contact solve.
    if (!registry.all_of<comps::RigidBodyTag>(entityA)
        || !registry.all_of<comps::RigidBodyTag>(entityB)) {
      continue;
    }

    auto& transformA = registry.get<comps::Transform>(entityA);
    auto& transformB = registry.get<comps::Transform>(entityB);
    const auto& massA = registry.get<comps::MassProperties>(entityA);
    const auto& massB = registry.get<comps::MassProperties>(entityB);

    const bool staticA
        = hasPrescribedRigidBodyContactResponse(registry, entityA);
    const bool staticB
        = hasPrescribedRigidBodyContactResponse(registry, entityB);

    ContactScratch::NormalConstraint constraint;
    constraint.bodyA = entityA;
    constraint.bodyB = entityB;
    constraint.transformA = &transformA;
    constraint.transformB = &transformB;
    constraint.normal = contact.normal;
    constraint.depth = contact.depth;
    constraint.armA = contact.point - transformA.position;
    constraint.armB = contact.point - transformB.position;
    constraint.staticA = staticA;
    constraint.staticB = staticB;
    constraint.invMassA = staticA ? 0.0 : inverseMass(massA);
    constraint.invMassB = staticB ? 0.0 : inverseMass(massB);
    constraint.friction = std::sqrt(
        frictionOf(registry, entityA) * frictionOf(registry, entityB));

    constraint.normalArmCrossA = constraint.armA.cross(constraint.normal);
    constraint.normalArmCrossB = constraint.armB.cross(constraint.normal);
    const bool needsInverseInertiaA
        = !staticA
          && needsContactInverseInertia(
              constraint.normalArmCrossA, constraint.friction);
    const bool needsInverseInertiaB
        = !staticB
          && needsContactInverseInertia(
              constraint.normalArmCrossB, constraint.friction);
    constraint.invInertiaA = needsInverseInertiaA
                                 ? inverseWorldInertia(massA, transformA)
                                 : Eigen::Matrix3d::Zero();
    constraint.invInertiaB = needsInverseInertiaB
                                 ? inverseWorldInertia(massB, transformB)
                                 : Eigen::Matrix3d::Zero();
    constraint.normalLinearDeltaA = -constraint.invMassA * constraint.normal;
    constraint.normalLinearDeltaB = constraint.invMassB * constraint.normal;
    constraint.normalAngularDeltaA
        = -constraint.invInertiaA * constraint.normalArmCrossA;
    constraint.normalAngularDeltaB
        = constraint.invInertiaB * constraint.normalArmCrossB;
    const double angular
        = constraint.normalArmCrossA.dot(
              constraint.invInertiaA * constraint.normalArmCrossA)
          + constraint.normalArmCrossB.dot(
              constraint.invInertiaB * constraint.normalArmCrossB);
    constraint.effectiveMass
        = constraint.invMassA + constraint.invMassB + angular;
    if (constraint.effectiveMass <= 0.0) {
      continue; // Both bodies are static.
    }
    constraint.inverseEffectiveMass = 1.0 / constraint.effectiveMass;

    // Restitution target from the pre-solve approach velocity (the impact
    // speed). Combine the two materials by taking the larger bounce.
    const double restitution = std::max(
        restitutionOf(registry, entityA), restitutionOf(registry, entityB));
    auto& velocityA = registry.get<comps::Velocity>(entityA);
    auto& velocityB = registry.get<comps::Velocity>(entityB);
    constraint.velocityA = &velocityA;
    constraint.velocityB = &velocityB;
    const double initialApproach = normalPointVelocity(
                                       velocityB,
                                       constraint.normal,
                                       constraint.normalArmCrossB,
                                       constraint.staticB)
                                   - normalPointVelocity(
                                       velocityA,
                                       constraint.normal,
                                       constraint.normalArmCrossA,
                                       constraint.staticA);
    constexpr double restitutionThreshold = 1e-3;
    constraint.restitutionVelocity
        = (restitution > 0.0 && initialApproach < -restitutionThreshold)
              ? -restitution * initialApproach
              : 0.0;
    constraint.normalImpulse = 0.0;

    constraint.tangentImpulse1 = 0.0;
    constraint.tangentImpulse2 = 0.0;
    if (constraint.friction > 0.0) {
      // Two tangent directions spanning the contact plane, plus their effective
      // masses, for a friction-pyramid (box) Coulomb model.
      constraint.tangent1 = constraint.normal.unitOrthogonal();
      constraint.tangent2 = constraint.normal.cross(constraint.tangent1);
      const auto tangentMass = [&](const Eigen::Vector3d& tangent) {
        const Eigen::Vector3d crossTangentA = constraint.armA.cross(tangent);
        const Eigen::Vector3d crossTangentB = constraint.armB.cross(tangent);
        return constraint.invMassA + constraint.invMassB
               + tangent.dot((constraint.invInertiaA * crossTangentA)
                                 .cross(constraint.armA))
               + tangent.dot((constraint.invInertiaB * crossTangentB)
                                 .cross(constraint.armB));
      };
      constraint.tangentMass1 = tangentMass(constraint.tangent1);
      constraint.tangentMass2 = tangentMass(constraint.tangent2);
      hasFrictionConstraints = true;
    } else {
      constraint.tangent1 = Eigen::Vector3d::Zero();
      constraint.tangent2 = Eigen::Vector3d::Zero();
      constraint.tangentMass1 = 0.0;
      constraint.tangentMass2 = 0.0;
    }

    constraints.push_back(constraint);
  }

  // Sequential impulses (Gauss-Seidel) drive each contact's normal approach
  // velocity to its restitution target. The accumulated normal impulse is
  // clamped non-negative so contacts only push, never pull.
  const auto solveNormalImpulse =
      [&](ContactScratch::NormalConstraint& constraint) {
        auto& velocityA = *constraint.velocityA;
        auto& velocityB = *constraint.velocityB;

        const double approach = normalPointVelocity(
                                    velocityB,
                                    constraint.normal,
                                    constraint.normalArmCrossB,
                                    constraint.staticB)
                                - normalPointVelocity(
                                    velocityA,
                                    constraint.normal,
                                    constraint.normalArmCrossA,
                                    constraint.staticA);

        double lambda = -(approach - constraint.restitutionVelocity)
                        * constraint.inverseEffectiveMass;
        const double clamped = std::max(constraint.normalImpulse + lambda, 0.0);
        lambda = clamped - constraint.normalImpulse;
        constraint.normalImpulse = clamped;

        velocityA.linear += lambda * constraint.normalLinearDeltaA;
        velocityA.angular += lambda * constraint.normalAngularDeltaA;
        velocityB.linear += lambda * constraint.normalLinearDeltaB;
        velocityB.angular += lambda * constraint.normalAngularDeltaB;
      };

  if (!hasFrictionConstraints) {
    for (std::size_t iteration = 0; iteration < m_iterations; ++iteration) {
      for (auto& constraint : constraints) {
        solveNormalImpulse(constraint);
      }
    }
  } else {
    for (std::size_t iteration = 0; iteration < m_iterations; ++iteration) {
      for (auto& constraint : constraints) {
        solveNormalImpulse(constraint);

        // Coulomb friction along each tangent, clamped to the friction pyramid
        // bounded by the accumulated normal impulse.
        if (constraint.friction > 0.0) {
          auto& velocityA = *constraint.velocityA;
          auto& velocityB = *constraint.velocityB;
          const double frictionLimit
              = constraint.friction * constraint.normalImpulse;
          const auto solveFriction = [&](const Eigen::Vector3d& tangent,
                                         double tangentMass,
                                         double& tangentImpulse) {
            if (tangentMass <= 0.0 || frictionLimit <= 0.0) {
              return;
            }
            const Eigen::Vector3d tangentVelocity
                = contactPointVelocity(
                      velocityB, constraint.armB, constraint.staticB)
                  - contactPointVelocity(
                      velocityA, constraint.armA, constraint.staticA);
            double tangentLambda = -tangentVelocity.dot(tangent) / tangentMass;
            const double clampedTangent = std::clamp(
                tangentImpulse + tangentLambda, -frictionLimit, frictionLimit);
            tangentLambda = clampedTangent - tangentImpulse;
            tangentImpulse = clampedTangent;

            const Eigen::Vector3d tangentImpulseVector
                = tangentLambda * tangent;
            velocityB.linear += constraint.invMassB * tangentImpulseVector;
            velocityB.angular += constraint.invInertiaB
                                 * constraint.armB.cross(tangentImpulseVector);
            velocityA.linear -= constraint.invMassA * tangentImpulseVector;
            velocityA.angular -= constraint.invInertiaA
                                 * constraint.armA.cross(tangentImpulseVector);
          };
          solveFriction(
              constraint.tangent1,
              constraint.tangentMass1,
              constraint.tangentImpulse1);
          solveFriction(
              constraint.tangent2,
              constraint.tangentMass2,
              constraint.tangentImpulse2);
        }
      }
    }
  }

  // Positional correction (projection) removes residual penetration beyond a
  // small allowance without injecting velocity, so resting stacks do not sink.
  constexpr double allowance = 1e-4;
  constexpr double correctionFactor = 0.2;
  for (const auto& constraint : constraints) {
    const double penetration = constraint.depth - allowance;
    if (penetration <= 0.0) {
      continue;
    }

    const double totalInverseMass = constraint.invMassA + constraint.invMassB;
    if (totalInverseMass <= 0.0) {
      continue;
    }

    const Eigen::Vector3d correction
        = (correctionFactor * penetration / totalInverseMass)
          * constraint.normal;
    constraint.transformA->position -= constraint.invMassA * correction;
    constraint.transformB->position += constraint.invMassB * correction;
  }
}

//==============================================================================
std::size_t RigidBodyContactStage::getIterations() const noexcept
{
  return m_iterations;
}

//==============================================================================
RigidIpcContactStage::RigidIpcContactStage(const std::size_t maxIterations)
  : RigidIpcContactStage(
        makeRigidIpcContactStageOptionsForMaxIterations(maxIterations))
{
}

//==============================================================================
RigidIpcContactStage::RigidIpcContactStage(RigidIpcContactStageOptions options)
  : m_options(sanitizeRigidIpcContactStageOptions(options)),
    m_scratch(std::make_unique<Scratch>())
{
}

//==============================================================================
RigidIpcContactStage::~RigidIpcContactStage() = default;

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
void RigidIpcContactStage::prepare(World& world)
{
  auto& registry = dart::simulation::detail::registryOf(world);
  auto bodyView = registry.view<comps::RigidBodyTag>();
  const auto bodyCount = bodyView.size();
  auto kinematicView = registry.view<comps::KinematicBodyTag>();
  const auto kinematicCount = kinematicView.size();

  m_scratch->runtimeBodies.reserve(bodyCount);
  m_scratch->solverBodies.reserve(bodyCount);
  m_scratch->surfaces.reserve(bodyCount);
  m_scratch->dynamicsTerms.reserve(bodyCount);
  m_scratch->solveDynamicsTerms.reserve(bodyCount);
  m_scratch->articulationConstraints.reserve(bodyCount);
  m_scratch->tracedEntities.reserve(kinematicCount);
  m_scratch->blockedEntities.reserve(kinematicCount);
  m_scratch->writebackEntities.reserve(bodyCount);
  m_scratch->orderedEntities.reserve(bodyCount);
  m_scratch->visitState.reserve(bodyCount);
  m_scratch->contactPowerSum.reserve(bodyCount);
  m_scratch->sawNonStationaryContactBody.reserve(bodyCount);
  m_scratch->stationaryContactBody.reserve(bodyCount);
  m_scratch->solveResult.surfaces.reserve(bodyCount);
  m_scratch->solveScratch.laggedSurfaces.reserve(bodyCount);
  m_scratch->solveScratch.lineSearchStartSurfaces.reserve(bodyCount);
  m_scratch->solveScratch.candidateSurfaces.reserve(bodyCount);
  m_scratch->solveScratch.acceptedSurfaces.reserve(bodyCount);
  m_scratch->solveScratch.bestDecreasingSurfaces.reserve(bodyCount);

  RigidIpcSolverStats warmupStats;
  collectRigidIpcRuntimeBodies(world, warmupStats, m_scratch->runtimeBodies);
  prepareRigidIpcSolverScratch(
      m_scratch->runtimeBodies,
      m_scratch->solverBodies,
      m_scratch->surfaces,
      m_scratch->dynamicsTerms);
}

//==============================================================================
void RigidIpcContactStage::execute(World& world, ComputeExecutor& executor)
{
  m_lastStats.reset();
  auto& scratch = *m_scratch;
  clearKinematicBodyStepTraces(world, scratch.tracedEntities);

  collectRigidIpcRuntimeBodies(world, m_lastStats, scratch.runtimeBodies);
  appendRigidIpcDeformableSurfaces(world, m_lastStats, scratch.runtimeBodies);
  collectRigidIpcMixedDomainCandidateStats(
      dart::simulation::detail::registryOf(world),
      scratch.runtimeBodies,
      m_options.activationDistance,
      m_lastStats,
      scratch.mixedDomainSurfaces);
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
      scratch.dynamicsTerms);
  auto& solverBodies = scratch.solverBodies;
  auto& surfaces = scratch.surfaces;
  auto& dynamicsTerms = scratch.dynamicsTerms;
  auto& solveDynamicsTerms = scratch.solveDynamicsTerms;
  auto& articulationConstraints = scratch.articulationConstraints;
  solveDynamicsTerms.assign(dynamicsTerms.begin(), dynamicsTerms.end());
  collectRigidIpcArticulationConstraints(
      world, solverBodies, articulationConstraints);

  // Adaptive barrier-stiffness inputs: the world AABB diagonal over all
  // collision surfaces and the average dynamic-body mass. These drive the IPC
  // adaptive-kappa scheme so the barrier is stiff enough to hold bodies at a
  // gap > 0 under gravity instead of letting them creep into penetration.
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

  sxdetail::RigidIpcProjectedNewtonSolveOptions options;
  options.barrier.squaredActivationDistance
      = m_options.activationDistance * m_options.activationDistance;
  options.barrier.stiffness
      = std::max(1.0, world.getRigidIpcAdaptiveBarrierStiffnessLowerBound());
  // The conservative line search runs a curved ACCD per candidate primitive
  // pair. If the ACCD exhausts its iteration budget on a tight, slowly
  // converging pair it returns Indeterminate, which the line search treats as a
  // zero step -- so the whole solve reports LineSearchBlocked and is skipped.
  // Dense resting contacts (stacks, piles, arches) routinely produce such pairs
  // once they settle into compression, freezing the solve even though the
  // contacts are advanceable. Give the ACCD a larger budget so those pairs
  // resolve to a real Hit/Miss instead. This only increases CCD accuracy -- a
  // real contact still limits the step -- so it cannot weaken the
  // intersection-free guarantee; well-separated scenes converge well under the
  // cap and are unaffected.
  options.lineSearch.maxIterations = 256;
  options.adaptiveStiffness.enabled = true;
  options.adaptiveStiffness.averageMass = averageMass;
  options.adaptiveStiffness.bboxDiagonal = bboxDiagonal;
  options.friction.coefficient = 1.0;
  options.friction.staticFrictionDisplacement
      = std::max(0.0, m_options.staticFrictionSpeedBound * world.getTimeStep());
  options.dynamicsTerms = std::move(solveDynamicsTerms);
  options.articulationConstraints = articulationConstraints;
  options.maxIterations = m_options.maxIterations;
  options.frictionIterations = m_options.frictionIterations;
  options.frictionConvergenceTolerance = m_options.frictionConvergenceTolerance;
  // The apply policy below writes back a not-fully-converged result when it
  // made progress, relying on every accepted Newton step having passed the
  // conservative line-search feasibility check. That guarantee only holds with
  // the line search enabled, so pin it explicitly (it is the default, but the
  // anti-tunneling invariant must not silently depend on that default).
  options.useLineSearch = true;
  options.newton.gradientTolerance = 1e-10;
  // The adaptive barrier is stiff (kappa can reach ~1e6), so the absolute
  // gradient tolerance is unreachable at a resting contact. A relative floor
  // lets a near-stationary contact converge instead of being skipped as
  // non-converged (which would otherwise freeze the body at near-rest).
  options.newton.relativeGradientTolerance = 1e-6;
  options.stepTolerance = 1e-12;

  sxdetail::RigidIpcProjectedNewtonSolveResult& result = scratch.solveResult;
  ComputeGraph graph;
  graph.addNode(
      "rigid_ipc_projected_newton_solve",
      [&]() {
        sxdetail::solveRigidIpcProjectedNewtonBarrierSystem(
            surfaces, options, result, scratch.solveScratch);
      },
      getMetadata());
  executor.execute(graph);
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

//==============================================================================
DeformableDynamicsStage::DeformableDynamicsStage()
  : m_scratch(std::make_unique<Scratch>())
{
}

//==============================================================================
DeformableDynamicsStage::~DeformableDynamicsStage() = default;

//==============================================================================
std::string_view DeformableDynamicsStage::getName() const noexcept
{
  return "deformable_dynamics";
}

//==============================================================================
ComputeStageMetadata DeformableDynamicsStage::getMetadata() const noexcept
{
  return {
      ComputeStageDomain::DeformableBody,
      ComputeStageAcceleration::TaskParallel
          | ComputeStageAcceleration::DataLocality
          | ComputeStageAcceleration::Gpu,
      {{"deformable_body.state", ComputeAccessMode::ReadWrite},
       {"deformable_body.model", ComputeAccessMode::Read},
       {"deformable_body.topology", ComputeAccessMode::Read},
       {"deformable_body.boundary_conditions", ComputeAccessMode::Read},
       {"rigid_body.kinematic_step_trace", ComputeAccessMode::Read},
       {"static_collision_geometry", ComputeAccessMode::Read}}};
}

namespace {

//==============================================================================
void primeSurfaceContactCandidateScratch(
    const comps::DeformableNodeState& state,
    DeformableContactSolverScratch& contactScratch)
{
  if (contactScratch.surfaceTriangles.empty()) {
    return;
  }

  reserveSurfaceContactCandidateScratch(state.positions.size(), contactScratch);

  dc::buildMotionAwareContactCandidatesSweep(
      state.positions,
      state.positions,
      contactScratch.surfaceTriangles,
      makeSurfaceContactCandidateOptions(),
      contactScratch.candidates,
      contactScratch.sweepScratch);
  filterSurfaceContactPointCandidates(
      contactScratch.candidates, contactScratch.surfaceContactPointMask);

  dc::ContactCandidateOptions barrierOptions;
  barrierOptions.activationDistance = selfContactBarrierActivationDistance();
  barrierOptions.exactDistanceFilter = true;
  barrierOptions.excludeIncidentPointTriangles = true;
  barrierOptions.excludeAdjacentEdges = true;
  dc::buildContactCandidatesSweep(
      state.positions,
      contactScratch.surfaceTriangles,
      barrierOptions,
      contactScratch.barrierCandidates,
      contactScratch.sweepScratch);
  filterSurfaceContactPointCandidates(
      contactScratch.barrierCandidates, contactScratch.surfaceContactPointMask);
}

//==============================================================================
void collectDeformableSurfaceSnapshotsInto(
    const sxdetail::WorldRegistry& registry,
    std::vector<SurfaceContactSnapshot>& snapshots,
    std::size_t& snapshotCount)
{
  auto view = registry.view<
      comps::DeformableBodyTag,
      comps::DeformableNodeState,
      comps::DeformableMeshTopology>();

  snapshotCount = 0;
  for (const auto entity : view) {
    const auto& state = view.get<comps::DeformableNodeState>(entity);
    const auto& topology = view.get<comps::DeformableMeshTopology>(entity);
    if (topology.surfaceTriangles.empty()) {
      continue;
    }

    auto& snapshot = nextSurfaceContactSnapshot(snapshots, snapshotCount);
    snapshot.entity = entity;
    snapshot.positions = state.positions;
    copySurfaceContactTopology(
        topology.surfaceTriangles,
        state.positions.size(),
        !topology.tetrahedra.empty(),
        snapshot.surfaceTriangles,
        snapshot.surfaceContactPointMask);
    dc::buildUniqueSurfaceEdges(
        snapshot.surfaceTriangles, snapshot.surfaceEdges);
  }
}

//==============================================================================
void primeSurfaceContactSnapshotSweepScratch(
    std::span<const Eigen::Vector3d> positions,
    std::span<const SurfaceContactSnapshot> snapshots,
    DeformableContactSolverScratch& contactScratch)
{
  const auto candidateOptions = makeSurfaceContactCandidateOptions();
  const auto ccdOptions = makeSurfaceContactCcdOptions();
  for (const auto& snapshot : snapshots) {
    if (snapshot.surfaceTriangles.empty()) {
      continue;
    }

    (void)interBodySurfaceContactStepBound(
        positions,
        positions,
        contactScratch.surfaceTriangles,
        contactScratch.surfaceContactPointMask,
        contactScratch.interBodyCurrentEdges,
        snapshot,
        candidateOptions,
        ccdOptions,
        contactScratch);
  }
}

//==============================================================================
void primeInterBodySurfaceContactScratch(
    entt::entity entity,
    const comps::DeformableNodeState& state,
    std::span<const SurfaceContactSnapshot> surfaceSnapshots,
    std::span<const SurfaceContactSnapshot> rigidSurfaceSnapshots,
    std::span<const SurfaceContactSnapshot> movingRigidSurfaceSnapshots,
    DeformableContactSolverScratch& contactScratch)
{
  if (contactScratch.surfaceTriangles.empty()) {
    contactScratch.interBodyCurrentEdges.clear();
  } else {
    dc::buildUniqueSurfaceEdges(
        contactScratch.surfaceTriangles, contactScratch.interBodyCurrentEdges);

    const auto candidateOptions = makeSurfaceContactCandidateOptions();
    const auto ccdOptions = makeSurfaceContactCcdOptions();
    for (const auto& snapshot : surfaceSnapshots) {
      if (snapshot.entity == entity || snapshot.surfaceTriangles.empty()) {
        continue;
      }

      (void)interBodySurfaceContactStepBound(
          state.positions,
          state.positions,
          contactScratch.surfaceTriangles,
          contactScratch.surfaceContactPointMask,
          contactScratch.interBodyCurrentEdges,
          snapshot,
          candidateOptions,
          ccdOptions,
          contactScratch);
    }
  }

  primeSurfaceContactSnapshotSweepScratch(
      state.positions, rigidSurfaceSnapshots, contactScratch);
  primeSurfaceContactSnapshotSweepScratch(
      state.positions, movingRigidSurfaceSnapshots, contactScratch);
}

} // namespace

//==============================================================================
void DeformableDynamicsStage::prepare(World& world)
{
  auto& registry = dart::simulation::detail::registryOf(world);
  auto view = registry.view<
      comps::DeformableBodyTag,
      comps::DeformableNodeState,
      comps::DeformableMeshTopology>();

  for (const auto entity : view) {
    const auto& state = view.get<comps::DeformableNodeState>(entity);
    const auto& topology = view.get<comps::DeformableMeshTopology>(entity);
    auto& solverScratch
        = registry.get_or_emplace<comps::DeformableSolverScratch>(entity);
    reserveDeformableSolverScratch(state, solverScratch);
    solverScratch.previousStepPositions = state.positions;
    solverScratch.externalAccelerations.assign(
        state.positions.size(), Eigen::Vector3d::Zero());
    solverScratch.activeFixed = state.fixed;
    solverScratch.activeDirichlet.assign(state.positions.size(), 0u);

    auto& contactScratch
        = registry.get_or_emplace<DeformableContactSolverScratch>(entity);
    syncSurfaceContactTopology(
        topology.surfaceTriangles,
        state.positions.size(),
        !topology.tetrahedra.empty(),
        contactScratch);
    primeSurfaceContactCandidateScratch(state, contactScratch);
    reserveDeformableFrictionScratch(state.positions.size(), contactScratch);
    if (const auto* model
        = registry.try_get<comps::DeformableSpringModel>(entity)) {
      reserveProjectedNewtonScratch(
          state.positions.size(), *model, topology, contactScratch);
    }
    if (const auto* material
        = registry.try_get<comps::DeformableMaterial>(entity)) {
      syncFemRestShapeScratch(
          state.positions.size(), topology, *material, contactScratch);
    }
    (void)registry.get_or_emplace<DeformableVbdScratch>(entity);
  }

  auto& scratch = *m_scratch;
  collectStaticGroundBarriersInto(world, scratch.barriers);
  collectSphereObstacleBarriersInto(world, scratch.sphereObstacles);
  collectBoxObstacleBarriersInto(world, scratch.boxObstacles);
  collectCapsuleObstacleBarriersInto(world, scratch.capsuleObstacles);
  DeformableSolverStats stats;
  collectStaticRigidSurfaceCcdObstaclesInto(
      world,
      stats,
      scratch.rigidSurfaceSnapshots,
      scratch.rigidSurfaceSnapshotCount);
  collectMovingRigidSurfaceCcdObstaclesInto(
      world,
      world.getTimeStep(),
      stats,
      scratch.movingRigidSurfaceSnapshots,
      scratch.movingRigidSurfaceSnapshotCount,
      true);
  collectDeformableSurfaceSnapshotsInto(
      registry, scratch.surfaceSnapshots, scratch.surfaceSnapshotCount);

  const auto surfaceSnapshots = activeSurfaceContactSnapshots(
      scratch.surfaceSnapshots, scratch.surfaceSnapshotCount);
  const auto rigidSurfaceSnapshots = activeSurfaceContactSnapshots(
      scratch.rigidSurfaceSnapshots, scratch.rigidSurfaceSnapshotCount);
  const auto movingRigidSurfaceSnapshots = activeSurfaceContactSnapshots(
      scratch.movingRigidSurfaceSnapshots,
      scratch.movingRigidSurfaceSnapshotCount);
  for (const auto entity : view) {
    const auto& state = view.get<comps::DeformableNodeState>(entity);
    const auto& topology = view.get<comps::DeformableMeshTopology>(entity);
    auto& contactScratch
        = registry.get_or_emplace<DeformableContactSolverScratch>(entity);
    primeInterBodySurfaceContactScratch(
        entity,
        state,
        surfaceSnapshots,
        rigidSurfaceSnapshots,
        movingRigidSurfaceSnapshots,
        contactScratch);
    auto* vbdConfig = registry.try_get<comps::DeformableVbdConfig>(entity);
    const auto* model = registry.try_get<comps::DeformableSpringModel>(entity);
    if (vbdConfig != nullptr && vbdConfig->enabled && model != nullptr
        && scratch.capsuleObstacles.empty()
        && movingRigidSurfaceSnapshots.empty()) {
      auto& vbdScratch = registry.get_or_emplace<DeformableVbdScratch>(entity);
      syncVbdTopologyScratch(
          state.positions.size(), *model, topology, vbdScratch);
      primeVbdStaticContactScratch(
          state.positions.size(),
          scratch.barriers,
          scratch.sphereObstacles,
          scratch.boxObstacles,
          *vbdConfig,
          vbdScratch);
      if (vbdConfig->useAvbdContactNormalRows
          && vbdConfig->contactStiffness > 0.0) {
        const std::size_t contactRowCapacity = state.positions.size();
        vbdScratch.avbdSolveFixed.reserve(state.positions.size());
        vbdScratch.avbdContactDescriptors.reserve(contactRowCapacity);
        vbdScratch.avbdContactInventory.reserve(contactRowCapacity);
        vbdScratch.avbdContactRows.reserve(contactRowCapacity);
        const std::size_t frictionRowCapacity = 2 * contactRowCapacity;
        vbdScratch.avbdFrictionDescriptors.reserve(frictionRowCapacity);
        vbdScratch.avbdFrictionInventory.reserve(frictionRowCapacity);
        vbdScratch.avbdFrictionRows.reserve(frictionRowCapacity);
        vbdScratch.previousAvbdFrictionWarmStarts.reserve(frictionRowCapacity);
      }
      if (vbdConfig->useAvbdSelfContactNormalRows
          && contactScratch.surfaceTriangles.size() >= 2) {
        vbdScratch.avbdSolveFixed.reserve(state.positions.size());
        reserveVbdSelfContactCandidateScratch(
            state.positions.size(),
            contactScratch.surfaceTriangles.size(),
            vbdScratch);

        const double dHat = selfContactBarrierActivationDistance();
        dc::ContactCandidateOptions candidateOptions;
        candidateOptions.activationDistance = dHat;
        candidateOptions.exactDistanceFilter = true;
        candidateOptions.excludeIncidentPointTriangles = true;
        candidateOptions.excludeAdjacentEdges = true;
        dc::buildMotionAwareContactCandidatesSweep(
            state.positions,
            state.positions,
            contactScratch.surfaceTriangles,
            candidateOptions,
            vbdScratch.selfContactCandidates,
            vbdScratch.selfContactSweepScratch);
        filterSurfaceContactPointCandidates(
            vbdScratch.selfContactCandidates,
            contactScratch.surfaceContactPointMask);
        vbdScratch.selfContactAdjacency.rebuild(
            state.positions.size(),
            vbdScratch.selfContactCandidates,
            contactScratch.surfaceTriangles,
            dHat * dHat,
            selfContactBarrierStiffness());

        const std::size_t selfContactRowCapacity
            = vbdScratch.selfContactCandidates.pointTriangleCandidates
                  .capacity()
              + vbdScratch.selfContactCandidates.edgeEdgeCandidates.capacity();
        vbdScratch.avbdSelfContactDescriptors.reserve(selfContactRowCapacity);
        vbdScratch.avbdSelfContactInventory.reserve(selfContactRowCapacity);
        vbdScratch.avbdSelfContactRows.reserve(selfContactRowCapacity);
        const std::size_t frictionRowCapacity = 2 * selfContactRowCapacity;
        vbdScratch.avbdSelfContactFrictionDescriptors.reserve(
            frictionRowCapacity);
        vbdScratch.avbdSelfContactFrictionInventory.reserve(
            frictionRowCapacity);
        vbdScratch.avbdSelfContactFrictionRows.reserve(frictionRowCapacity);
        vbdScratch.previousAvbdSelfContactFrictionWarmStarts.reserve(
            frictionRowCapacity);
      }
    }
  }
}

//==============================================================================
void DeformableDynamicsStage::execute(
    World& world, ComputeExecutor& /*executor*/)
{
  m_lastStats.reset();

  auto& registry = dart::simulation::detail::registryOf(world);
  auto view = registry.view<
      comps::DeformableBodyTag,
      comps::DeformableNodeState,
      comps::DeformableSpringModel,
      comps::DeformableMeshTopology,
      comps::DeformableMaterial>();
  if (view.begin() == view.end()) {
    return;
  }

  auto& stageScratch = *m_scratch;
  collectStaticGroundBarriersInto(world, stageScratch.barriers);
  collectSphereObstacleBarriersInto(world, stageScratch.sphereObstacles);
  collectBoxObstacleBarriersInto(world, stageScratch.boxObstacles);
  collectCapsuleObstacleBarriersInto(world, stageScratch.capsuleObstacles);
  collectStaticRigidSurfaceCcdObstaclesInto(
      world,
      m_lastStats,
      stageScratch.rigidSurfaceSnapshots,
      stageScratch.rigidSurfaceSnapshotCount);
  const auto timeStep = world.getTimeStep();
  collectMovingRigidSurfaceCcdObstaclesInto(
      world,
      timeStep,
      m_lastStats,
      stageScratch.movingRigidSurfaceSnapshots,
      stageScratch.movingRigidSurfaceSnapshotCount);
  const auto gravity = world.getGravity();
  const auto& barriers = stageScratch.barriers;
  const auto& sphereObstacles = stageScratch.sphereObstacles;
  const auto& boxObstacles = stageScratch.boxObstacles;
  const auto& capsuleObstacles = stageScratch.capsuleObstacles;
  const auto rigidSurfaceSnapshots = activeSurfaceContactSnapshots(
      stageScratch.rigidSurfaceSnapshots,
      stageScratch.rigidSurfaceSnapshotCount);
  const auto movingRigidSurfaceSnapshots = activeSurfaceContactSnapshots(
      stageScratch.movingRigidSurfaceSnapshots,
      stageScratch.movingRigidSurfaceSnapshotCount);
  m_lastStats.staticGroundBarrierCount = barriers.size();

  for (const auto entity : view) {
    auto& state = view.get<comps::DeformableNodeState>(entity);
    auto& scratch
        = registry.get_or_emplace<comps::DeformableSolverScratch>(entity);
    const auto* boundaryConditions
        = registry.try_get<comps::DeformableBoundaryConditions>(entity);
    ++m_lastStats.bodyCount;
    prepareDeformableBoundaryConditions(
        state,
        boundaryConditions,
        world.getTime(),
        timeStep,
        scratch,
        m_lastStats);
  }

  collectDeformableSurfaceSnapshotsInto(
      registry,
      stageScratch.surfaceSnapshots,
      stageScratch.surfaceSnapshotCount);
  const auto surfaceSnapshots = activeSurfaceContactSnapshots(
      stageScratch.surfaceSnapshots, stageScratch.surfaceSnapshotCount);

  for (const auto entity : view) {
    auto& state = view.get<comps::DeformableNodeState>(entity);
    const auto& model = view.get<comps::DeformableSpringModel>(entity);
    const auto& topology = view.get<comps::DeformableMeshTopology>(entity);
    const auto& material = view.get<comps::DeformableMaterial>(entity);
    auto& scratch
        = registry.get_or_emplace<comps::DeformableSolverScratch>(entity);
    auto& contactScratch
        = registry.get_or_emplace<DeformableContactSolverScratch>(entity);
    auto& vbdScratch = registry.get_or_emplace<DeformableVbdScratch>(entity);
    const auto* vbdConfig
        = registry.try_get<comps::DeformableVbdConfig>(entity);
    advanceDeformableBody(
        entity,
        state,
        model,
        topology,
        scratch,
        contactScratch,
        vbdScratch,
        vbdConfig,
        surfaceSnapshots,
        rigidSurfaceSnapshots,
        movingRigidSurfaceSnapshots,
        gravity,
        timeStep,
        barriers,
        sphereObstacles,
        boxObstacles,
        capsuleObstacles,
        material,
        m_lastStats);
  }
}

//==============================================================================
const DeformableSolverStats& DeformableDynamicsStage::getLastStats()
    const noexcept
{
  return m_lastStats;
}

//==============================================================================
void reserveDeformableDynamicsRegistryStorage(
    detail::WorldRegistry& registry, std::size_t deformableBodyCount)
{
  auto& contactScratchStorage
      = registry.storage<DeformableContactSolverScratch>();
  auto& vbdScratchStorage = registry.storage<DeformableVbdScratch>();
  contactScratchStorage.reserve(deformableBodyCount);
  vbdScratchStorage.reserve(deformableBodyCount);

  if (deformableBodyCount == 0u) {
    return;
  }

  auto view = registry.view<comps::DeformableBodyTag>();
  for (auto entity : view) {
    if (!registry.all_of<DeformableContactSolverScratch>(entity)) {
      registry.emplace<DeformableContactSolverScratch>(entity);
      registry.remove<DeformableContactSolverScratch>(entity);
    }
    if (!registry.all_of<DeformableVbdScratch>(entity)) {
      registry.emplace<DeformableVbdScratch>(entity);
      registry.remove<DeformableVbdScratch>(entity);
    }
  }
}

//==============================================================================
BatchedRigidBodyIntegrationStage::BatchedRigidBodyIntegrationStage()
  : m_scratch(std::make_unique<Scratch>())
{
}

//==============================================================================
BatchedRigidBodyIntegrationStage::~BatchedRigidBodyIntegrationStage() = default;

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
    m_scratch = std::make_unique<Scratch>();
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
  extractRigidBodyModelBatchInto(world, scratch.model);
  const auto timeStep = world.getTimeStep();

  integrateRigidBodyStateBatch(
      scratch.state, scratch.model, forces.force, forces.torque, timeStep);

  restorePrescribedRigidBodyState(
      registry, entities, scratch.initialState, scratch.state);
  applyRigidBodyState(world, scratch.state);

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
