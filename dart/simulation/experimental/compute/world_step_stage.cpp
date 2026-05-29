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

#include "dart/simulation/experimental/body/contact.hpp"
#include "dart/simulation/experimental/body/rigid_body.hpp"
#include "dart/simulation/experimental/common/exceptions.hpp"
#include "dart/simulation/experimental/comps/collision_geometry.hpp"
#include "dart/simulation/experimental/comps/contact_material.hpp"
#include "dart/simulation/experimental/comps/deformable_body.hpp"
#include "dart/simulation/experimental/comps/dynamics.hpp"
#include "dart/simulation/experimental/comps/frame_types.hpp"
#include "dart/simulation/experimental/comps/rigid_body.hpp"
#include "dart/simulation/experimental/compute/compute_executor.hpp"
#include "dart/simulation/experimental/compute/compute_graph.hpp"
#include "dart/simulation/experimental/compute/rigid_body_state_batch.hpp"
#include "dart/simulation/experimental/compute/world_kinematics_graph.hpp"
#include "dart/simulation/experimental/detail/deformable_contact/barrier_kernel.hpp"
#include "dart/simulation/experimental/detail/deformable_contact/candidate_set.hpp"
#include "dart/simulation/experimental/detail/deformable_contact/continuous_collision_step.hpp"
#include "dart/simulation/experimental/detail/deformable_contact/tangent_stencil.hpp"
#include "dart/simulation/experimental/world.hpp"

#include <Eigen/Cholesky>
#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>
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

namespace dart::simulation::experimental::compute {

namespace dc = dart::simulation::experimental::detail::deformable_contact;

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
void appendRigidBodyParentBeforeChild(
    const entt::registry& registry,
    const std::vector<entt::entity>& rigidBodyEntities,
    std::vector<int>& visitState,
    std::vector<entt::entity>& ordered,
    std::size_t index)
{
  if (visitState[index] == 2) {
    return;
  }

  DART_EXPERIMENTAL_THROW_T_IF(
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
    DART_EXPERIMENTAL_THROW_T_IF(
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
std::vector<entt::entity> orderRigidBodiesParentBeforeChild(
    const entt::registry& registry,
    const std::vector<entt::entity>& rigidBodyEntities)
{
  std::vector<entt::entity> ordered;
  ordered.reserve(rigidBodyEntities.size());

  std::vector<int> visitState(rigidBodyEntities.size(), 0);
  for (std::size_t i = 0; i < rigidBodyEntities.size(); ++i) {
    appendRigidBodyParentBeforeChild(
        registry, rigidBodyEntities, visitState, ordered, i);
  }

  return ordered;
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
// Update a body's velocity from an already-assembled force/torque buffer. Does
// not move the body or mutate the persistent force component.
void integrateRigidBodyVelocity(
    entt::registry& registry,
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
    entt::registry& registry, entt::entity entity, const double timeStep)
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
void integrateRigidBody(
    entt::registry& registry,
    entt::entity entity,
    const Eigen::Vector3d& gravity,
    const double timeStep)
{
  if (registry.all_of<comps::StaticBodyTag>(entity)) {
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
  const auto gravity = world.getGravity();
  const auto timeStep = world.getTimeStep();
  if (hasRigidBodyFrameDependency(registry, *entities)) {
    std::vector<RigidBodyNode> nodes;
    nodes.reserve(entities->size());

    for (const auto entity : *entities) {
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
        [&registry, entities, begin, end, gravity, timeStep]() {
          for (auto i = begin; i < end; ++i) {
            integrateRigidBody(
                registry,
                (*entities)[static_cast<std::size_t>(i)],
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

namespace {

//==============================================================================
struct RigidBodyForceBatch
{
  std::vector<entt::entity> entities;
  std::vector<double> force;
  std::vector<double> torque;
};

//==============================================================================
RigidBodyForceBatch assembleRigidBodyForces(
    const World& world, bool includeGravity)
{
  const auto& registry = world.getRegistry();
  auto view
      = registry.view<comps::RigidBodyTag, comps::Transform, comps::Velocity>();

  RigidBodyForceBatch batch;
  batch.entities.reserve(view.size_hint());
  batch.force.reserve(3 * view.size_hint());
  batch.torque.reserve(3 * view.size_hint());

  const Eigen::Vector3d gravity
      = includeGravity ? world.getGravity() : Eigen::Vector3d::Zero();
  for (const auto entity : view) {
    batch.entities.push_back(entity);

    const auto& applied = registry.get<comps::Force>(entity);
    Eigen::Vector3d assembledForce = applied.force;
    Eigen::Vector3d assembledTorque = applied.torque;

    if (registry.all_of<comps::StaticBodyTag>(entity)) {
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

  return batch;
}

//==============================================================================
void restoreStaticRigidBodyState(
    const entt::registry& registry,
    const std::vector<entt::entity>& entities,
    const RigidBodyStateBatch& source,
    RigidBodyStateBatch& target)
{
  DART_EXPERIMENTAL_THROW_T_IF(
      source.bodyCount != entities.size()
          || target.bodyCount != entities.size(),
      InvalidOperationException,
      "Rigid-body batch state does not match the rigid-body entity count");

  for (std::size_t i = 0; i < entities.size(); ++i) {
    if (!registry.all_of<comps::StaticBodyTag>(entities[i])) {
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
double restitutionOf(const entt::registry& registry, entt::entity entity)
{
  if (const auto* material = registry.try_get<comps::ContactMaterial>(entity)) {
    return material->restitution;
  }
  return 0.0;
}

//==============================================================================
double frictionOf(const entt::registry& registry, entt::entity entity)
{
  if (const auto* material = registry.try_get<comps::ContactMaterial>(entity)) {
    return material->friction;
  }
  return 1.0;
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

  // Persistent projected-Newton sparse Cholesky. The fill-reducing symbolic
  // factorization is reused across iterations and steps whenever the assembled
  // Hessian sparsity pattern (cached column/row index arrays) is unchanged, so
  // only the numeric factorization repeats. Behavior-preserving: a structure
  // mismatch (or a failed factorization) re-runs analyzePattern.
  Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>> newtonSolver;
  std::vector<int> newtonPatternOuter;
  std::vector<int> newtonPatternInner;
  bool newtonPatternValid = false;
};

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
void accumulateCcdStats(
    dc::ContinuousCollisionStepStats& total,
    const dc::ContinuousCollisionStepStats& addend)
{
  total.pointTriangleChecks += addend.pointTriangleChecks;
  total.edgeEdgeChecks += addend.edgeEdgeChecks;
  total.hits += addend.hits;
  total.misses += addend.misses;
  total.indeterminate += addend.indeterminate;
  total.zeroStepCount += addend.zeroStepCount;
}

//==============================================================================
void considerInterBodyContactResult(
    InterBodySurfaceContactResult& aggregate,
    const dc::ContinuousCollisionStepResult& candidate)
{
  accumulateCcdStats(aggregate.stats, candidate.stats);
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
      });

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
      });

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
      });

  return aggregate;
}

//==============================================================================
SurfaceContactSnapshot makeStaticBoxSurfaceCcdSnapshot(
    entt::entity entity,
    const Eigen::Vector3d& halfExtents,
    const comps::Transform& transform)
{
  SurfaceContactSnapshot snapshot;
  snapshot.entity = entity;
  snapshot.positions.reserve(8);

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

  return snapshot;
}

//==============================================================================
std::vector<SurfaceContactSnapshot> collectStaticRigidSurfaceCcdObstacles(
    const World& world, DeformableSolverStats& stats)
{
  ++stats.staticRigidSurfaceCcdSnapshotBuilds;

  const auto& registry = world.getRegistry();
  auto view = registry.view<
      comps::RigidBodyTag,
      comps::StaticBodyTag,
      comps::DeformableSurfaceCcdObstacleTag,
      comps::CollisionGeometry,
      comps::Transform>();

  std::vector<SurfaceContactSnapshot> snapshots;
  for (const auto entity : view) {
    const auto& geometry = view.get<comps::CollisionGeometry>(entity);
    const auto& transform = view.get<comps::Transform>(entity);
    if (geometry.shape.type != CollisionShapeType::Box
        || !geometry.shape.halfExtents.allFinite()
        || (geometry.shape.halfExtents.array() <= 0.0).any()
        || !transform.position.allFinite()) {
      continue;
    }

    auto snapshot = makeStaticBoxSurfaceCcdSnapshot(
        entity, geometry.shape.halfExtents, transform);
    stats.staticRigidSurfaceCcdTriangleCount
        += snapshot.surfaceTriangles.size();
    stats.staticRigidSurfaceCcdEdgeCount += snapshot.surfaceEdges.size();
    snapshots.push_back(std::move(snapshot));
    ++stats.staticRigidSurfaceCcdBoxCount;
  }

  return snapshots;
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
// interBodySurfaceContactStepBound handle them unchanged. The obstacle end pose
// is predicted from velocity, so the deformable is limited against where the
// obstacle will be even though the deformable stage runs before
// RigidBodyPositionStage.
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
std::vector<SurfaceContactSnapshot> collectMovingRigidSurfaceCcdObstacles(
    const World& world, const double timeStep, DeformableSolverStats& stats)
{
  ++stats.movingRigidSurfaceCcdSnapshotBuilds;

  const auto& registry = world.getRegistry();
  // Moving obstacles are free (non-static) rigid bodies integrated by
  // RigidBodyPositionStage. entt::exclude<StaticBodyTag> keeps this set
  // disjoint from collectStaticRigidSurfaceCcdObstacles so no body is limited
  // or counted twice. comps::Velocity is required to predict the end pose.
  auto view = registry.view<
      comps::RigidBodyTag,
      comps::DeformableSurfaceCcdObstacleTag,
      comps::CollisionGeometry,
      comps::Transform,
      comps::Velocity>(entt::exclude<comps::StaticBodyTag>);

  std::vector<SurfaceContactSnapshot> snapshots;
  for (const auto entity : view) {
    const auto& geometry = view.get<comps::CollisionGeometry>(entity);
    const auto& transform = view.get<comps::Transform>(entity);
    const auto& velocity = view.get<comps::Velocity>(entity);
    if (geometry.shape.type != CollisionShapeType::Box
        || !geometry.shape.halfExtents.allFinite()
        || (geometry.shape.halfExtents.array() <= 0.0).any()
        || !transform.position.allFinite() || !velocity.linear.allFinite()
        || !velocity.angular.allFinite()) {
      continue;
    }

    const auto endTransform
        = predictRigidBodyEndTransform(transform, velocity, timeStep);
    const auto& halfExtents = geometry.shape.halfExtents;
    const std::size_t samples = movingRigidSurfaceCcdSampleCount(
        halfExtents, transform, endTransform, velocity.angular, timeStep);

    // Keep consecutive sample boxes overlapping. The natural (uncapped) spacing
    // is one box min-half-extent, so the boxes overlap with margin; once the
    // sample count is capped the residual spacing can exceed the box min
    // dimension, so inflate each sampled box isotropically by half the residual
    // spacing to bridge the gap. inflation is 0 in the common (uncapped) case.
    const double minHalfExtent = halfExtents.minCoeff();
    const double linearMotion
        = (endTransform.position - transform.position).norm();
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
        = normalizeOrIdentity(transform.orientation);
    const Eigen::Quaterniond endOrientation
        = normalizeOrIdentity(endTransform.orientation);

    for (std::size_t k = 0; k < samples; ++k) {
      // samples is always >= 2, so the denominator is never zero.
      const double fraction
          = static_cast<double>(k) / static_cast<double>(samples - 1);
      comps::Transform sampleTransform;
      sampleTransform.position
          = transform.position
            + fraction * (endTransform.position - transform.position);
      sampleTransform.orientation
          = startOrientation.slerp(fraction, endOrientation);

      auto snapshot = makeStaticBoxSurfaceCcdSnapshot(
          entity, sampleHalfExtents, sampleTransform);
      stats.movingRigidSurfaceCcdTriangleCount
          += snapshot.surfaceTriangles.size();
      stats.movingRigidSurfaceCcdEdgeCount += snapshot.surfaceEdges.size();
      ++stats.movingRigidSurfaceCcdSampleCount;
      snapshots.push_back(std::move(snapshot));
    }
    ++stats.movingRigidSurfaceCcdBoxCount;
  }

  return snapshots;
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
std::vector<StaticGroundBarrier> collectStaticGroundBarriers(const World& world)
{
  const auto& registry = world.getRegistry();
  auto view = registry.view<
      comps::RigidBodyTag,
      comps::StaticBodyTag,
      comps::DeformableGroundBarrierTag,
      comps::CollisionGeometry,
      comps::Transform>();

  std::vector<StaticGroundBarrier> barriers;
  for (const auto entity : view) {
    const auto& geometry = view.get<comps::CollisionGeometry>(entity);
    const auto& transform = view.get<comps::Transform>(entity);

    switch (geometry.shape.type) {
      case CollisionShapeType::Sphere: {
        const double radius = geometry.shape.radius;
        if (!(radius > 0.0) || !std::isfinite(radius)
            || !transform.position.allFinite()) {
          break;
        }

        StaticGroundBarrier barrier;
        barrier.shape = StaticGroundBarrier::Shape::Sphere;
        barrier.center = transform.position;
        barrier.radius = radius;
        barrier.top = transform.position.z() + radius;
        barriers.push_back(barrier);
        break;
      }
      case CollisionShapeType::Box: {
        if (!geometry.shape.halfExtents.allFinite()
            || (geometry.shape.halfExtents.array() <= 0.0).any()
            || !transform.position.allFinite()) {
          break;
        }

        StaticGroundBarrier barrier;
        barrier.shape = StaticGroundBarrier::Shape::Box;
        barrier.center = transform.position;
        barrier.rotation
            = normalizeOrIdentity(transform.orientation).toRotationMatrix();
        barrier.halfExtents = geometry.shape.halfExtents;
        barriers.push_back(barrier);
        break;
      }
    }
  }

  return barriers;
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
std::vector<Eigen::Vector2d> projectedBoxFootprint(
    const StaticGroundBarrier& barrier)
{
  std::vector<Eigen::Vector2d> points;
  points.reserve(8);
  for (const double xSign : {-1.0, 1.0}) {
    for (const double ySign : {-1.0, 1.0}) {
      for (const double zSign : {-1.0, 1.0}) {
        const Eigen::Vector3d local(
            xSign * barrier.halfExtents.x(),
            ySign * barrier.halfExtents.y(),
            zSign * barrier.halfExtents.z());
        const Eigen::Vector3d world = barrier.center + barrier.rotation * local;
        points.push_back(world.head<2>());
      }
    }
  }

  constexpr double tolerance = 1e-12;
  std::sort(
      points.begin(),
      points.end(),
      [](const Eigen::Vector2d& lhs, const Eigen::Vector2d& rhs) {
        return std::tie(lhs.x(), lhs.y()) < std::tie(rhs.x(), rhs.y());
      });
  points.erase(
      std::unique(
          points.begin(),
          points.end(),
          [](const Eigen::Vector2d& lhs, const Eigen::Vector2d& rhs) {
            return (lhs - rhs).squaredNorm() <= tolerance * tolerance;
          }),
      points.end());

  if (points.size() <= 2) {
    return points;
  }

  std::vector<Eigen::Vector2d> hull;
  hull.reserve(points.size() * 2);
  for (const auto& point : points) {
    while (hull.size() >= 2
           && cross2d(hull.back() - hull[hull.size() - 2], point - hull.back())
                  <= tolerance) {
      hull.pop_back();
    }
    hull.push_back(point);
  }

  const auto lowerSize = hull.size();
  for (auto it = points.rbegin() + 1; it != points.rend(); ++it) {
    while (hull.size() > lowerSize
           && cross2d(hull.back() - hull[hull.size() - 2], *it - hull.back())
                  <= tolerance) {
      hull.pop_back();
    }
    hull.push_back(*it);
  }
  if (!hull.empty()) {
    hull.pop_back();
  }

  return hull;
}

//==============================================================================
std::optional<TimeInterval> clipSegmentToConvexFootprint(
    const Eigen::Vector2d& start,
    const Eigen::Vector2d& end,
    const std::vector<Eigen::Vector2d>& footprint)
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
    case StaticGroundBarrier::Shape::Box:
      return clipSegmentToConvexFootprint(
          start.head<2>(), end.head<2>(), projectedBoxFootprint(barrier));
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
    std::vector<Eigen::Vector3d>* gradient)
{
  if (barriers.empty()) {
    return 0.0;
  }

  const double activationDistance = staticGroundBarrierActivationDistance();
  constexpr double barrierScale = 25.0;

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
  constexpr double barrierScale = 25.0;
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
    dissipation += frictionCoefficient * groundNormalForce[i]
                   * frictionF1(y, epsilon) * y;
    ++activeContacts;
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
    dissipation += frictionCoefficient * contact.normalForce
                   * frictionF1(y, epsilon) * y;
    ++activeContacts;
  }
}

//==============================================================================
double evaluateDeformableObjective(
    const comps::DeformableNodeState& state,
    const comps::DeformableSpringModel& model,
    const std::vector<Eigen::Vector3d>& positions,
    const std::vector<Eigen::Vector3d>& inertialTargets,
    const std::vector<std::uint8_t>& fixed,
    const std::vector<StaticGroundBarrier>& barriers,
    double timeStep,
    std::vector<Eigen::Vector3d>* gradient,
    const SelfContactBarrierInputs* contactBarrier = nullptr,
    std::size_t* barrierActiveContacts = nullptr,
    const GroundFrictionInputs* groundFriction = nullptr,
    const SelfContactFrictionInputs* selfContactFriction = nullptr)
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

  energy += addStaticGroundBarrierEnergy(positions, fixed, barriers, gradient);
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

  if (result.indeterminate) {
    return false;
  }

  if (!result.hit) {
    return true;
  }

  const double stepBound = std::clamp(result.stepBound, 0.0, 1.0);
  if (stepBound <= 0.0) {
    return false;
  }

  const double safeFraction = std::nextafter(stepBound, 0.0);
  if (safeFraction <= 0.0 || !std::isfinite(safeFraction)) {
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

  stepBound = std::clamp(stepBound, 0.0, 1.0);
  if (stepBound <= 0.0) {
    return false;
  }

  const double safeFraction = std::nextafter(stepBound, 0.0);
  if (safeFraction <= 0.0 || !std::isfinite(safeFraction)) {
    return false;
  }

  step *= safeFraction;
  ++stats.interBodySurfaceContactCcdLimitedSteps;
  directionalDerivative = buildLineSearchCandidate(
      current, direction, gradient, fixed, step, candidate);
  return true;
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

  stepBound = std::clamp(stepBound, 0.0, 1.0);
  if (stepBound <= 0.0) {
    ++stats.staticRigidSurfaceCcdZeroStepCount;
    return false;
  }

  const double safeFraction = std::nextafter(stepBound, 0.0);
  if (safeFraction <= 0.0 || !std::isfinite(safeFraction)) {
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

  stepBound = std::clamp(stepBound, 0.0, 1.0);
  if (stepBound <= 0.0) {
    ++stats.movingRigidSurfaceCcdZeroStepCount;
    return false;
  }

  const double safeFraction = std::nextafter(stepBound, 0.0);
  if (safeFraction <= 0.0 || !std::isfinite(safeFraction)) {
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

  stepBound = std::clamp(stepBound, 0.0, 1.0);
  if (stepBound <= 0.0) {
    ++stats.staticGroundBarrierCcdZeroStepCount;
    return false;
  }

  const double safeFraction = std::nextafter(stepBound, 0.0);
  if (safeFraction <= 0.0 || !std::isfinite(safeFraction)) {
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
void prepareDeformableBoundaryConditions(
    comps::DeformableNodeState& state,
    const comps::DeformableBoundaryConditions* boundaryConditions,
    double time,
    double timeStep,
    comps::DeformableSolverScratch& scratch,
    DeformableSolverStats& stats)
{
  const auto nodeCount = state.positions.size();
  scratch.previousStepPositions = state.positions;
  scratch.externalAccelerations.assign(nodeCount, Eigen::Vector3d::Zero());
  scratch.activeFixed = state.fixed;

  if (boundaryConditions == nullptr) {
    return;
  }

  std::vector<std::uint8_t> countedDirichlet(nodeCount, 0u);
  for (const auto& boundary : boundaryConditions->dirichlet) {
    if (!isBoundaryActiveAtStepStart(
            time, boundary.startTime, boundary.endTime)) {
      continue;
    }

    DART_EXPERIMENTAL_THROW_T_IF(
        boundary.nodes.size() != boundary.referencePositions.size(),
        InvalidArgumentException,
        "Serialized deformable Dirichlet boundary has mismatched node and "
        "reference-position counts");
    const double elapsed = elapsedBoundaryTime(
        time, timeStep, boundary.startTime, boundary.endTime);
    for (std::size_t i = 0; i < boundary.nodes.size(); ++i) {
      const auto node = boundary.nodes[i];
      DART_EXPERIMENTAL_THROW_T_IF(
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
      if (countedDirichlet[node] == 0u) {
        countedDirichlet[node] = 1u;
        ++stats.activeDirichletNodeCount;
      }
    }
  }

  std::vector<std::uint8_t> countedNeumann(nodeCount, 0u);
  for (const auto& boundary : boundaryConditions->neumann) {
    if (!isBoundaryActiveAtStepStart(
            time, boundary.startTime, boundary.endTime)) {
      continue;
    }

    for (const auto node : boundary.nodes) {
      DART_EXPERIMENTAL_THROW_T_IF(
          node >= nodeCount,
          InvalidArgumentException,
          "Serialized deformable Neumann boundary references out-of-range "
          "node {}",
          node);
      if (scratch.activeFixed[node] != 0u) {
        continue;
      }

      scratch.externalAccelerations[node] += boundary.acceleration;
      if (countedNeumann[node] == 0u) {
        countedNeumann[node] = 1u;
        ++stats.activeNeumannNodeCount;
      }
    }
  }
}

//==============================================================================
// Project a symmetric matrix onto the nearest positive-semidefinite matrix by
// clamping negative eigenvalues to zero (per-element PSD projection, as in
// IPC).
template <typename Mat>
Mat projectSymmetricToPsd(const Mat& matrix)
{
  const Mat symmetric = 0.5 * (matrix + matrix.transpose());
  Eigen::SelfAdjointEigenSolver<Mat> solver(symmetric);
  if (solver.info() != Eigen::Success) {
    // Eigen-decomposition failed: drop this element's curvature rather than
    // scatter a possibly-indefinite block. Returning zero is the conservative
    // IPC choice and keeps the assembled Hessian positive definite (the
    // inertia diagonal still supplies the free-DOF curvature).
    return Mat::Zero();
  }
  auto eigenvalues = solver.eigenvalues();
  for (Eigen::Index i = 0; i < eigenvalues.size(); ++i) {
    eigenvalues[i] = std::max(0.0, eigenvalues[i]);
  }
  return solver.eigenvectors() * eigenvalues.asDiagonal()
         * solver.eigenvectors().transpose();
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
    const SelfContactBarrierInputs* contactBarrier,
    const GroundFrictionInputs* groundFriction,
    const SelfContactFrictionInputs* selfContactFriction,
    const double timeStep,
    const std::vector<Eigen::Vector3d>& gradient,
    std::vector<Eigen::Vector3d>& direction,
    DeformableContactSolverScratch& solverCache,
    DeformableSolverStats& stats)
{
  const std::size_t nodeCount = positions.size();
  // Sparse Cholesky scales to large meshes; the cap is a safety bound against
  // pathological fill-in (a matrix-free CG / GPU solve is a later slice).
  constexpr std::size_t kMaxSparseNodes = 20000;
  if (nodeCount == 0 || nodeCount > kMaxSparseNodes) {
    return false;
  }

  const auto dim = static_cast<Eigen::Index>(3 * nodeCount);
  const double invDt2 = 1.0 / (timeStep * timeStep);

  // Right-hand side: -gradient on free DOFs, zero on pinned (fixed) DOFs.
  Eigen::VectorXd rhs = Eigen::VectorXd::Zero(dim);
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
  std::vector<Eigen::Triplet<double>> triplets;
  triplets.reserve(tripletEstimate);

  const auto isFree = [&](std::size_t node) {
    return fixed[node] == 0u;
  };
  const auto addBlock3 = [&](std::size_t nodeRow,
                             std::size_t nodeCol,
                             const Eigen::Matrix3d& block) {
    if (!isFree(nodeRow) || !isFree(nodeCol)) {
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

  // Inertia: block diagonal, positive definite for free nodes. Fixed DOFs get
  // a unit diagonal so the global matrix stays positive definite.
  for (std::size_t i = 0; i < nodeCount; ++i) {
    const double diagonal = isFree(i) ? state.masses[i] * invDt2 : 1.0;
    for (int k = 0; k < 3; ++k) {
      triplets.emplace_back(
          static_cast<Eigen::Index>(3 * i + k),
          static_cast<Eigen::Index>(3 * i + k),
          diagonal);
    }
  }

  // Spring stretch Hessian per edge, PSD-projected over its 6x6 block.
  constexpr double minLength = 1e-12;
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
    Eigen::Matrix<double, 6, 6> edgeHessian;
    edgeHessian.block<3, 3>(0, 0) = block;
    edgeHessian.block<3, 3>(3, 3) = block;
    edgeHessian.block<3, 3>(0, 3) = -block;
    edgeHessian.block<3, 3>(3, 0) = -block;
    edgeHessian = projectSymmetricToPsd(edgeHessian);
    const std::array<std::size_t, 2> nodes{edge.nodeA, edge.nodeB};
    for (int bi = 0; bi < 2; ++bi) {
      for (int bj = 0; bj < 2; ++bj) {
        addBlock3(
            nodes[bi], nodes[bj], edgeHessian.block<3, 3>(3 * bi, 3 * bj));
      }
    }
  }

  // Self-contact barrier Hessian per active contact, PSD-projected over 12x12.
  if (contactBarrier != nullptr && contactBarrier->candidates != nullptr
      && contactBarrier->triangles != nullptr && contactBarrier->stiffness > 0.0
      && contactBarrier->squaredActivationDistance > 0.0) {
    const auto& candidates = *contactBarrier->candidates;
    const auto& triangles = *contactBarrier->triangles;
    const double sqAct = contactBarrier->squaredActivationDistance;
    const double kappa = contactBarrier->stiffness;
    const auto scatter12 = [&](const dc::Matrix12d& blockHessian,
                               const std::array<std::size_t, 4>& nodes) {
      const dc::Matrix12d projected = projectSymmetricToPsd(blockHessian);
      for (int bi = 0; bi < 4; ++bi) {
        for (int bj = 0; bj < 4; ++bj) {
          addBlock3(
              nodes[bi], nodes[bj], projected.block<3, 3>(3 * bi, 3 * bj));
        }
      }
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
      scatter12(
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
      scatter12(
          result.hessian, {edgeA.nodeA, edgeA.nodeB, edgeB.nodeA, edgeB.nodeB});
    }
  }

  // Static ground barrier Hessian (vertical scalar per active free node).
  if (!barriers.empty()) {
    const double activationDistance = staticGroundBarrierActivationDistance();
    constexpr double barrierScale = 25.0;
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
      triplets.emplace_back(
          static_cast<Eigen::Index>(3 * i + 2),
          static_cast<Eigen::Index>(3 * i + 2),
          std::max(0.0, second));
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

  Eigen::SparseMatrix<double> hessian(dim, dim);
  hessian.setFromTriplets(triplets.begin(), triplets.end());
  hessian.makeCompressed();

  // Reuse the fill-reducing symbolic factorization when the sparsity pattern is
  // unchanged from the last analyzed matrix (same column/row index arrays); the
  // expensive ordering then runs once and only the numeric factorization
  // repeats. This is behavior-preserving (analyzePattern + factorize gives the
  // same result as compute()); any structural mismatch re-analyzes, so it is
  // safe by construction.
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
  // m/dt^2 inertia floor precludes here. Either failure falls back to steepest
  // descent below.
  if (ldlt.info() != Eigen::Success || (ldlt.vectorD().array() <= 0.0).any()) {
    // Force a fresh analysis next time after a failed/indefinite factorization.
    solverCache.newtonPatternValid = false;
    return false;
  }
  const Eigen::VectorXd solution = ldlt.solve(rhs);
  if (ldlt.info() != Eigen::Success || !solution.allFinite()) {
    solverCache.newtonPatternValid = false;
    return false;
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
    std::span<const SurfaceContactSnapshot> surfaceSnapshots,
    std::span<const SurfaceContactSnapshot> rigidSurfaceSnapshots,
    std::span<const SurfaceContactSnapshot> movingRigidSurfaceSnapshots,
    const Eigen::Vector3d& gravity,
    double timeStep,
    const std::vector<StaticGroundBarrier>& barriers,
    double frictionCoefficient,
    DeformableSolverStats& stats)
{
  const auto nodeCount = state.positions.size();
  if (nodeCount == 0) {
    return;
  }

  stats.nodeCount += nodeCount;
  stats.edgeCount += model.edges.size();
  syncSurfaceContactTopology(
      topology.surfaceTriangles,
      nodeCount,
      !topology.tetrahedra.empty(),
      contactScratch);

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

  if (model.edges.empty() && barriers.empty() && rigidSurfaceSnapshots.empty()
      && movingRigidSurfaceSnapshots.empty()
      && contactScratch.surfaceTriangles.empty()) {
    for (std::size_t i = 0; i < nodeCount; ++i) {
      if (scratch.activeFixed[i] == 0u) {
        scratch.next[i] = scratch.inertialTargets[i];
      }
    }
  } else {
    constexpr std::size_t maxIterations = 64;
    constexpr std::size_t maxLineSearchIterations = 16;
    constexpr double gradientToleranceSquared = 1e-18;
    constexpr double armijo = 1e-4;
    constexpr double minStep = 1e-12;

    double lastGradSquared = 0.0;
    std::vector<double> groundFrictionNormalForce;
    std::vector<Eigen::Vector3d> groundFrictionNormalDirection;
    std::vector<SelfContactFrictionContact> selfContactFrictionContacts;
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
      if (frictionCoefficient > 0.0 && !barriers.empty()) {
        computeStaticGroundNormalForces(
            scratch.next,
            scratch.activeFixed,
            barriers,
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
          timeStep,
          &scratch.gradient,
          &contactBarrier,
          &stats.selfContactBarrierActiveContacts,
          &groundFriction,
          &selfContactFriction);
      if (!std::isfinite(energy)) {
        break;
      }

      const double gradSquared
          = gradientNormSquared(scratch.gradient, scratch.activeFixed);
      lastGradSquared = gradSquared;
      if (gradSquared <= gradientToleranceSquared) {
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
                timeStep,
                nullptr,
                &contactBarrier,
                nullptr,
                &groundFriction,
                &selfContactFriction);
            if (std::isfinite(candidateEnergy)
                && candidateEnergy <= energy + armijo * directionalDerivative) {
              std::swap(scratch.next, scratch.candidate);
              ++stats.acceptedLineSearchSteps;
              return true;
            }
          }

          ++stats.rejectedLineSearchCandidates;
          step *= 0.5;
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
          &contactBarrier,
          &groundFriction,
          &selfContactFriction,
          timeStep,
          scratch.gradient,
          scratch.direction,
          contactScratch,
          stats);
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
        break;
      }
    }
    // Record the worst-case solve residual across the step's bodies (the
    // gradient norm at termination), a convergence diagnostic for the
    // benchmark statistics.
    stats.finalGradientResidualNorm
        = std::max(stats.finalGradientResidualNorm, std::sqrt(lastGradSquared));

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

} // namespace

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
void RigidBodyVelocityStage::execute(
    World& world, ComputeExecutor& /*executor*/)
{
  auto& registry = world.getRegistry();
  const auto timeStep = world.getTimeStep();
  const auto forces = assembleRigidBodyForces(world, true);

  for (std::size_t i = 0; i < forces.entities.size(); ++i) {
    const auto entity = forces.entities[i];
    if (registry.all_of<comps::StaticBodyTag>(entity)) {
      continue; // Static bodies do not accelerate.
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
  auto& registry = world.getRegistry();
  const auto timeStep = world.getTimeStep();

  auto view = registry.view<
      comps::RigidBodyTag,
      comps::Transform,
      comps::Velocity,
      comps::FreeFrameProperties,
      comps::FrameState,
      comps::FrameCache>();
  for (auto entity : view) {
    if (registry.all_of<comps::StaticBodyTag>(entity)) {
      continue; // Static bodies do not move.
    }
    integrateRigidBodyPosition(registry, entity, timeStep);
  }
}

//==============================================================================
RigidBodyContactStage::RigidBodyContactStage(std::size_t iterations)
  : m_iterations(std::max<std::size_t>(1, iterations))
{
}

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
void RigidBodyContactStage::execute(World& world, ComputeExecutor& /*executor*/)
{
  const auto contacts = world.collide();
  if (contacts.empty()) {
    return;
  }

  auto& registry = world.getRegistry();

  // Precompute per-contact constants for a sequential-impulse normal solve.
  struct NormalConstraint
  {
    entt::entity bodyA;
    entt::entity bodyB;
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
    double depth;
    double restitutionVelocity;
    double normalImpulse;
    Eigen::Vector3d tangent1;
    Eigen::Vector3d tangent2;
    double tangentMass1;
    double tangentMass2;
    double tangentImpulse1;
    double tangentImpulse2;
    double friction;
  };

  const auto contactPointVelocity = [](const comps::Velocity& velocity,
                                       const Eigen::Vector3d& arm,
                                       bool isStatic) -> Eigen::Vector3d {
    if (isStatic) {
      return Eigen::Vector3d::Zero();
    }
    return velocity.linear + velocity.angular.cross(arm);
  };

  std::vector<NormalConstraint> constraints;
  constraints.reserve(contacts.size());
  for (const auto& contact : contacts) {
    const auto entityA = contact.bodyA.getEntity();
    const auto entityB = contact.bodyB.getEntity();

    // This sequential-impulse solver handles rigid-body pairs only; contacts
    // involving multibody links are resolved by the articulated contact solve.
    if (!registry.all_of<comps::RigidBodyTag>(entityA)
        || !registry.all_of<comps::RigidBodyTag>(entityB)) {
      continue;
    }

    const auto& transformA = registry.get<comps::Transform>(entityA);
    const auto& transformB = registry.get<comps::Transform>(entityB);
    const auto& massA = registry.get<comps::MassProperties>(entityA);
    const auto& massB = registry.get<comps::MassProperties>(entityB);

    const bool staticA = registry.all_of<comps::StaticBodyTag>(entityA);
    const bool staticB = registry.all_of<comps::StaticBodyTag>(entityB);

    NormalConstraint constraint;
    constraint.bodyA = entityA;
    constraint.bodyB = entityB;
    constraint.normal = contact.normal;
    constraint.depth = contact.depth;
    constraint.armA = contact.point - transformA.position;
    constraint.armB = contact.point - transformB.position;
    constraint.staticA = staticA;
    constraint.staticB = staticB;
    constraint.invMassA = staticA ? 0.0 : inverseMass(massA);
    constraint.invMassB = staticB ? 0.0 : inverseMass(massB);
    constraint.invInertiaA = staticA ? Eigen::Matrix3d::Zero()
                                     : inverseWorldInertia(massA, transformA);
    constraint.invInertiaB = staticB ? Eigen::Matrix3d::Zero()
                                     : inverseWorldInertia(massB, transformB);

    const Eigen::Vector3d crossA = constraint.armA.cross(constraint.normal);
    const Eigen::Vector3d crossB = constraint.armB.cross(constraint.normal);
    const double angular
        = constraint.normal.dot(
              (constraint.invInertiaA * crossA).cross(constraint.armA))
          + constraint.normal.dot(
              (constraint.invInertiaB * crossB).cross(constraint.armB));
    constraint.effectiveMass
        = constraint.invMassA + constraint.invMassB + angular;
    if (constraint.effectiveMass <= 0.0) {
      continue; // Both bodies are static.
    }

    // Restitution target from the pre-solve approach velocity (the impact
    // speed). Combine the two materials by taking the larger bounce.
    const double restitution = std::max(
        restitutionOf(registry, entityA), restitutionOf(registry, entityB));
    const auto& velocityA = registry.get<comps::Velocity>(entityA);
    const auto& velocityB = registry.get<comps::Velocity>(entityB);
    const double initialApproach
        = (contactPointVelocity(velocityB, constraint.armB, constraint.staticB)
           - contactPointVelocity(
               velocityA, constraint.armA, constraint.staticA))
              .dot(constraint.normal);
    constexpr double restitutionThreshold = 1e-3;
    constraint.restitutionVelocity
        = (restitution > 0.0 && initialApproach < -restitutionThreshold)
              ? -restitution * initialApproach
              : 0.0;
    constraint.normalImpulse = 0.0;

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
    constraint.tangentImpulse1 = 0.0;
    constraint.tangentImpulse2 = 0.0;
    constraint.friction = std::sqrt(
        frictionOf(registry, entityA) * frictionOf(registry, entityB));

    constraints.push_back(constraint);
  }

  // Sequential impulses (Gauss-Seidel) drive each contact's normal approach
  // velocity to its restitution target. The accumulated normal impulse is
  // clamped non-negative so contacts only push, never pull.
  for (std::size_t iteration = 0; iteration < m_iterations; ++iteration) {
    for (auto& constraint : constraints) {
      auto& velocityA = registry.get<comps::Velocity>(constraint.bodyA);
      auto& velocityB = registry.get<comps::Velocity>(constraint.bodyB);

      const Eigen::Vector3d pointVelocityA = contactPointVelocity(
          velocityA, constraint.armA, constraint.staticA);
      const Eigen::Vector3d pointVelocityB = contactPointVelocity(
          velocityB, constraint.armB, constraint.staticB);
      const double approach
          = (pointVelocityB - pointVelocityA).dot(constraint.normal);

      double lambda = -(approach - constraint.restitutionVelocity)
                      / constraint.effectiveMass;
      const double clamped = std::max(constraint.normalImpulse + lambda, 0.0);
      lambda = clamped - constraint.normalImpulse;
      constraint.normalImpulse = clamped;

      const Eigen::Vector3d impulse = lambda * constraint.normal;
      velocityB.linear += constraint.invMassB * impulse;
      velocityB.angular
          += constraint.invInertiaB * constraint.armB.cross(impulse);
      velocityA.linear -= constraint.invMassA * impulse;
      velocityA.angular
          -= constraint.invInertiaA * constraint.armA.cross(impulse);

      // Coulomb friction along each tangent, clamped to the friction pyramid
      // bounded by the accumulated normal impulse.
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

        const Eigen::Vector3d tangentImpulseVector = tangentLambda * tangent;
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
    registry.get<comps::Transform>(constraint.bodyA).position
        -= constraint.invMassA * correction;
    registry.get<comps::Transform>(constraint.bodyB).position
        += constraint.invMassB * correction;
  }
}

//==============================================================================
std::size_t RigidBodyContactStage::getIterations() const noexcept
{
  return m_iterations;
}

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
          | ComputeStageAcceleration::DataLocality,
      {{"deformable_body.state", ComputeAccessMode::ReadWrite},
       {"deformable_body.model", ComputeAccessMode::Read},
       {"deformable_body.topology", ComputeAccessMode::Read},
       {"deformable_body.boundary_conditions", ComputeAccessMode::Read},
       {"static_collision_geometry", ComputeAccessMode::Read}}};
}

//==============================================================================
void DeformableDynamicsStage::execute(
    World& world, ComputeExecutor& /*executor*/)
{
  m_lastStats.reset();

  auto& registry = world.getRegistry();
  auto view = registry.view<
      comps::DeformableBodyTag,
      comps::DeformableNodeState,
      comps::DeformableSpringModel,
      comps::DeformableMeshTopology,
      comps::DeformableMaterial>();
  if (view.begin() == view.end()) {
    return;
  }

  const auto barriers = collectStaticGroundBarriers(world);
  const auto rigidSurfaceSnapshots
      = collectStaticRigidSurfaceCcdObstacles(world, m_lastStats);
  const auto timeStep = world.getTimeStep();
  const auto movingRigidSurfaceSnapshots
      = collectMovingRigidSurfaceCcdObstacles(world, timeStep, m_lastStats);
  const auto gravity = world.getGravity();
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

  std::vector<SurfaceContactSnapshot> surfaceSnapshots;
  for (const auto entity : view) {
    const auto& state = view.get<comps::DeformableNodeState>(entity);
    const auto& topology = view.get<comps::DeformableMeshTopology>(entity);
    if (topology.surfaceTriangles.empty()) {
      continue;
    }

    SurfaceContactSnapshot snapshot;
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
    surfaceSnapshots.push_back(std::move(snapshot));
  }

  for (const auto entity : view) {
    auto& state = view.get<comps::DeformableNodeState>(entity);
    const auto& model = view.get<comps::DeformableSpringModel>(entity);
    const auto& topology = view.get<comps::DeformableMeshTopology>(entity);
    const auto& material = view.get<comps::DeformableMaterial>(entity);
    auto& scratch
        = registry.get_or_emplace<comps::DeformableSolverScratch>(entity);
    auto& contactScratch
        = registry.get_or_emplace<DeformableContactSolverScratch>(entity);
    advanceDeformableBody(
        entity,
        state,
        model,
        topology,
        scratch,
        contactScratch,
        surfaceSnapshots,
        rigidSurfaceSnapshots,
        movingRigidSurfaceSnapshots,
        gravity,
        timeStep,
        barriers,
        material.frictionCoefficient,
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
  const auto forces = assembleRigidBodyForces(world, true);
  const auto& entities = forces.entities;

  if (entities.empty()) {
    return;
  }

  auto state = extractRigidBodyState(world);
  const auto initialState = state;
  const auto model = extractRigidBodyModelBatch(world);
  const auto timeStep = world.getTimeStep();

  ComputeGraph graph;
  graph.addNode(
      "soa_rigid_body_integration",
      [&state, &model, &forces, timeStep]() {
        integrateRigidBodyStateBatch(
            state, model, forces.force, forces.torque, timeStep);
      },
      getMetadata());
  executor.execute(graph);

  restoreStaticRigidBodyState(registry, entities, initialState, state);
  applyRigidBodyState(world, state);

  // Restore frame-cache consistency the same way the per-entity integrator
  // does, now that the world-space Transform has been written back. The SoA
  // integration itself is world-space and flat, but local transforms for
  // frame-coupled bodies must be written parent-before-child.
  const auto frameUpdateOrder
      = orderRigidBodiesParentBeforeChild(registry, entities);
  for (const auto entity : frameUpdateOrder) {
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
