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

#include "dart/simulation/experimental/compute/world_step_stage.hpp"

#include "dart/simulation/experimental/common/exceptions.hpp"
#include "dart/simulation/experimental/comps/dynamics.hpp"
#include "dart/simulation/experimental/comps/frame_types.hpp"
#include "dart/simulation/experimental/comps/rigid_body.hpp"
#include "dart/simulation/experimental/compute/compute_executor.hpp"
#include "dart/simulation/experimental/compute/compute_graph.hpp"
#include "dart/simulation/experimental/compute/rigid_body_state_batch.hpp"
#include "dart/simulation/experimental/compute/world_kinematics_graph.hpp"
#include "dart/simulation/experimental/world.hpp"

#include <Eigen/Cholesky>
#include <Eigen/Geometry>
#include <entt/entt.hpp>

#include <algorithm>
#include <memory>
#include <string>
#include <vector>

#include <cmath>

namespace dart::simulation::experimental::compute {

namespace {

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
    const entt::registry& registry, entt::entity entity)
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
    const entt::registry& registry, entt::entity entity)
{
  if (entity == entt::null) {
    return Eigen::Isometry3d::Identity();
  }

  const auto* frameState = registry.try_get<comps::FrameState>(entity);
  DART_EXPERIMENTAL_THROW_T_IF(
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
entt::entity findNearestRigidBodyAncestor(
    const entt::registry& registry,
    entt::entity entity,
    const std::vector<entt::entity>& rigidBodyEntities)
{
  while (entity != entt::null) {
    if (containsEntity(rigidBodyEntities, entity)) {
      return entity;
    }

    const auto* frameState = registry.try_get<comps::FrameState>(entity);
    DART_EXPERIMENTAL_THROW_T_IF(
        !frameState,
        InvalidOperationException,
        "Frame entity is missing a FrameState component");

    entity = frameState->parentFrame;
  }

  return entt::null;
}

//==============================================================================
bool hasRigidBodyFrameDependency(
    const entt::registry& registry,
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
struct RigidBodyNode
{
  entt::entity entity;
  ComputeNode* node;
};

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
void integrateRigidBody(
    entt::registry& registry, entt::entity entity, const double timeStep)
{
  auto& transform = registry.get<comps::Transform>(entity);
  auto& velocity = registry.get<comps::Velocity>(entity);
  const auto& mass = registry.get<comps::MassProperties>(entity);
  const auto& force = registry.get<comps::Force>(entity);

  auto orientation = normalizeOrIdentity(transform.orientation);

  if (mass.mass > 0.0 && std::isfinite(mass.mass)) {
    velocity.linear += (force.force / mass.mass) * timeStep;
  }

  integrateAngularVelocity(velocity, mass, force, orientation, timeStep);

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

} // namespace

//==============================================================================
ComputeStageMetadata WorldStepStage::getMetadata() const noexcept
{
  return {};
}

//==============================================================================
WorldStepPipeline& WorldStepPipeline::addStage(WorldStepStage& stage)
{
  m_stages.push_back(&stage);
  return *this;
}

//==============================================================================
void WorldStepPipeline::clear() noexcept
{
  m_stages.clear();
}

//==============================================================================
std::size_t WorldStepPipeline::getStageCount() const noexcept
{
  return m_stages.size();
}

//==============================================================================
bool WorldStepPipeline::isEmpty() const noexcept
{
  return m_stages.empty();
}

//==============================================================================
WorldStepStage& WorldStepPipeline::getStage(std::size_t index) const
{
  DART_EXPERIMENTAL_THROW_T_IF(
      index >= m_stages.size(),
      OutOfRangeException,
      "World step pipeline stage index {} is out of range",
      index);

  return *m_stages[index];
}

//==============================================================================
void WorldStepPipeline::execute(World& world, ComputeExecutor& executor)
{
  for (auto* stage : m_stages) {
    stage->execute(world, executor);
  }
}

//==============================================================================
std::string_view KinematicsStage::getName() const noexcept
{
  return "kinematics";
}

//==============================================================================
ComputeStageMetadata KinematicsStage::getMetadata() const noexcept
{
  return {
      ComputeStageDomain::Kinematics,
      ComputeStageAcceleration::TaskParallel
          | ComputeStageAcceleration::DataLocality};
}

//==============================================================================
void KinematicsStage::execute(World& world, ComputeExecutor& executor)
{
  WorldKinematicsGraph graph(world);
  graph.execute(executor);
}

//==============================================================================
RigidBodyIntegrationStage::RigidBodyIntegrationStage(std::size_t batchSize)
  : m_batchSize(std::max<std::size_t>(1, batchSize))
{
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
  auto& registry = world.getRegistry();
  auto rigidBodyView = registry.view<
      comps::RigidBodyTag,
      comps::Transform,
      comps::Velocity,
      comps::MassProperties,
      comps::Force,
      comps::FrameState,
      comps::FreeFrameProperties,
      comps::FrameCache>();

  auto entities = std::make_shared<std::vector<entt::entity>>();
  entities->reserve(rigidBodyView.size_hint());
  for (auto entity : rigidBodyView) {
    entities->push_back(entity);
  }

  ComputeGraph graph;
  const auto timeStep = world.getTimeStep();
  if (hasRigidBodyFrameDependency(registry, *entities)) {
    std::vector<RigidBodyNode> nodes;
    nodes.reserve(entities->size());

    for (const auto entity : *entities) {
      auto& node = graph.addNode(
          "rigid_body_entity_" + std::to_string(entt::to_integral(entity)),
          [&registry, entity, timeStep]() {
            integrateRigidBody(registry, entity, timeStep);
          },
          getMetadata());
      nodes.push_back({entity, &node});
    }

    for (const auto& entry : nodes) {
      const auto& frameState = registry.get<comps::FrameState>(entry.entity);
      const auto parentRigidBody = findNearestRigidBodyAncestor(
          registry, frameState.parentFrame, *entities);
      if (parentRigidBody == entt::null) {
        continue;
      }

      auto* parentNode = findRigidBodyNode(nodes, parentRigidBody);
      DART_EXPERIMENTAL_THROW_T_IF(
          !parentNode,
          InvalidOperationException,
          "Rigid-body ancestor is missing an integration node");

      graph.addDependency(*parentNode, *entry.node);
    }

    executor.execute(graph);
    return;
  }

  for (std::size_t begin = 0; begin < entities->size(); begin += m_batchSize) {
    const auto end = std::min(begin + m_batchSize, entities->size());
    graph.addNode(
        "rigid_body_batch_" + std::to_string(begin),
        [&registry, entities, begin, end, timeStep]() {
          for (auto i = begin; i < end; ++i) {
            integrateRigidBody(
                registry, (*entities)[static_cast<std::size_t>(i)], timeStep);
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
    World& world, ComputeExecutor& executor)
{
  auto& registry = world.getRegistry();

  // Gather entities in the same view order used by the state/model extractors
  // so the force array and the frame-update loop stay aligned body-for-body.
  auto view
      = registry.view<comps::RigidBodyTag, comps::Transform, comps::Velocity>();
  std::vector<entt::entity> entities;
  entities.reserve(view.size_hint());
  for (const auto entity : view) {
    entities.push_back(entity);
  }

  if (entities.empty()) {
    return;
  }

  // Frame-coupled rigid bodies require parent-before-child ordering for the
  // local-transform bookkeeping; the SoA path integrates in flat order, so
  // defer those cases to the per-entity stage.
  if (hasRigidBodyFrameDependency(registry, entities)) {
    RigidBodyIntegrationStage fallback;
    fallback.execute(world, executor);
    return;
  }

  std::vector<double> force;
  std::vector<double> torque;
  force.reserve(3 * entities.size());
  torque.reserve(3 * entities.size());
  for (const auto entity : entities) {
    const auto& f = registry.get<comps::Force>(entity);
    force.push_back(f.force.x());
    force.push_back(f.force.y());
    force.push_back(f.force.z());
    torque.push_back(f.torque.x());
    torque.push_back(f.torque.y());
    torque.push_back(f.torque.z());
  }

  auto state = extractRigidBodyState(world);
  const auto model = extractRigidBodyModelBatch(world);
  const auto timeStep = world.getTimeStep();

  ComputeGraph graph;
  graph.addNode(
      "soa_rigid_body_integration",
      [&state, &model, &force, &torque, timeStep]() {
        integrateRigidBodyStateBatch(state, model, force, torque, timeStep);
      },
      getMetadata());
  executor.execute(graph);

  applyRigidBodyState(world, state);

  // Restore frame-cache consistency the same way the per-entity integrator
  // does, now that the world-space Transform has been written back. With no
  // rigid-body-to-rigid-body parenting, view order is a valid update order.
  for (const auto entity : entities) {
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

} // namespace dart::simulation::experimental::compute
