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
#include "dart/simulation/experimental/compute/deformable_psd_backend.hpp"
#include "dart/simulation/experimental/compute/rigid_body_state_batch.hpp"
#include "dart/simulation/experimental/compute/world_kinematics_graph.hpp"
#include "dart/simulation/experimental/detail/boxed_lcp_contact.hpp"
#include "dart/simulation/experimental/detail/deformable_contact/barrier_kernel.hpp"
#include "dart/simulation/experimental/detail/deformable_contact/candidate_set.hpp"
#include "dart/simulation/experimental/detail/deformable_contact/continuous_collision_step.hpp"
#include "dart/simulation/experimental/detail/deformable_contact/tangent_stencil.hpp"
#include "dart/simulation/experimental/detail/deformable_elasticity/fem_tet_element.hpp"
#include "dart/simulation/experimental/detail/deformable_vbd/block_descent.hpp"
#include "dart/simulation/experimental/detail/deformable_vbd/parallel_block_descent.hpp"
#include "dart/simulation/experimental/detail/rigid_ipc_barrier.hpp"
#include "dart/simulation/experimental/world.hpp"
#include "dart/simulation/experimental/world_options.hpp"

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

namespace dart::simulation::experimental::compute {

namespace dc = dart::simulation::experimental::detail::deformable_contact;
namespace dvbd = dart::simulation::experimental::detail::deformable_vbd;
namespace fem = dart::simulation::experimental::detail::deformable_elasticity;
namespace sxdetail = dart::simulation::experimental::detail;

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
// Positional correction (projection) for rigid-body normal contacts: removes
// residual penetration beyond a small allowance without injecting velocity, so
// resting stacks do not sink. Shared by the sequential-impulse and boxed-LCP
// contact paths; the sequential path inlines an equivalent loop over its
// precomputed constraints, while the LCP path drives this from the raw
// contacts. Only rigid-body/rigid-body pairs with at least one dynamic body
// are corrected.
void resolveRigidBodyContactPositions(
    entt::registry& registry,
    const std::vector<Contact>& contacts,
    double /*timeStep*/)
{
  constexpr double allowance = 1e-4;
  constexpr double correctionFactor = 0.2;
  for (const auto& contact : contacts) {
    const auto entityA = contact.bodyA.getEntity();
    const auto entityB = contact.bodyB.getEntity();
    if (!registry.all_of<comps::RigidBodyTag>(entityA)
        || !registry.all_of<comps::RigidBodyTag>(entityB)) {
      continue;
    }

    const double penetration = contact.depth - allowance;
    if (penetration <= 0.0) {
      continue;
    }

    const bool staticA = registry.all_of<comps::StaticBodyTag>(entityA);
    const bool staticB = registry.all_of<comps::StaticBodyTag>(entityB);
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

  // Cached per-tetrahedron FEM rest shapes (inverse rest edge matrix + rest
  // volume). The rest configuration never changes, so these are computed once
  // (when the cached count first matches the body's tetrahedron count) and
  // reused every step instead of re-inverting each tet's rest edges per step.
  std::vector<fem::TetRestShape> femRestShapes;
};

//==============================================================================
struct ProjectedNewtonMatrixFreeHessian
{
  struct Block3
  {
    std::size_t rowNode = 0;
    std::size_t colNode = 0;
    Eigen::Matrix3d block = Eigen::Matrix3d::Zero();
  };

  explicit ProjectedNewtonMatrixFreeHessian(std::size_t nodes)
    : nodeCount(nodes), diagonalBlocks(nodes, Eigen::Matrix3d::Zero())
  {
  }

  void addBlock3(
      std::size_t rowNode, std::size_t colNode, const Eigen::Matrix3d& block)
  {
    blocks.push_back({rowNode, colNode, block});
    if (rowNode == colNode) {
      diagonalBlocks[rowNode] += block;
    }
  }

  [[nodiscard]] Eigen::VectorXd multiply(const Eigen::VectorXd& x) const
  {
    Eigen::VectorXd y
        = Eigen::VectorXd::Zero(static_cast<Eigen::Index>(3 * nodeCount));
    for (const auto& entry : blocks) {
      y.segment<3>(static_cast<Eigen::Index>(3 * entry.rowNode))
          += entry.block
             * x.segment<3>(static_cast<Eigen::Index>(3 * entry.colNode));
    }
    return y;
  }

  [[nodiscard]] bool factorBlockJacobi()
  {
    inverseDiagonalBlocks.assign(nodeCount, Eigen::Matrix3d::Zero());
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

  [[nodiscard]] Eigen::VectorXd applyPreconditioner(
      const Eigen::VectorXd& residual) const
  {
    Eigen::VectorXd z
        = Eigen::VectorXd::Zero(static_cast<Eigen::Index>(3 * nodeCount));
    for (std::size_t i = 0; i < nodeCount; ++i) {
      z.segment<3>(static_cast<Eigen::Index>(3 * i))
          = inverseDiagonalBlocks[i]
            * residual.segment<3>(static_cast<Eigen::Index>(3 * i));
    }
    return z;
  }

  std::size_t nodeCount = 0;
  std::vector<Block3> blocks;
  std::vector<Eigen::Matrix3d> diagonalBlocks;
  std::vector<Eigen::Matrix3d> inverseDiagonalBlocks;
};

//==============================================================================
bool solveMatrixFreeConjugateGradient(
    ProjectedNewtonMatrixFreeHessian& hessian,
    const Eigen::VectorXd& rhs,
    Eigen::VectorXd& solution,
    std::size_t& iterations,
    double& relativeResidual)
{
  constexpr double kTolerance = 1e-8;
  const Eigen::Index maxIterations = 2 * rhs.size();
  solution = Eigen::VectorXd::Zero(rhs.size());
  iterations = 0;
  relativeResidual = 0.0;

  if (!hessian.factorBlockJacobi()) {
    return false;
  }

  const double rhsNorm = rhs.norm();
  if (rhsNorm == 0.0) {
    return true;
  }

  Eigen::VectorXd residual = rhs;
  Eigen::VectorXd z = hessian.applyPreconditioner(residual);
  Eigen::VectorXd direction = z;
  double rz = residual.dot(z);
  if (!std::isfinite(rz) || rz <= 0.0 || !direction.allFinite()) {
    return false;
  }

  for (Eigen::Index iter = 0; iter < maxIterations; ++iter) {
    const Eigen::VectorXd hessianDirection = hessian.multiply(direction);
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

    z = hessian.applyPreconditioner(residual);
    const double nextRz = residual.dot(z);
    if (!std::isfinite(nextRz) || nextRz <= 0.0 || !z.allFinite()) {
      return false;
    }
    const double beta = nextRz / rz;
    direction = z + beta * direction;
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
// A static sphere has no flat faces to triangulate, so it is tessellated into a
// UV-sphere polyhedron that CIRCUMSCRIBES the true sphere: the vertices sit at
// a slightly larger radius so every flat face stays outside the analytic
// surface. That keeps the conservative no-penetration guarantee (a deformable
// is stopped at the polyhedron, never inside the real sphere), and the existing
// point-triangle / edge-edge CCD reducers consume the snapshot unchanged. The
// outward inflation is the modest over-conservatism this introduces (smaller
// with finer tessellation), analogous to the box supersampling's thin-corner
// over-coverage.
SurfaceContactSnapshot makeStaticSphereSurfaceCcdSnapshot(
    entt::entity entity, double radius, const comps::Transform& transform)
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

  SurfaceContactSnapshot snapshot;
  snapshot.entity = entity;

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

  return snapshot;
}

//==============================================================================
std::vector<SurfaceContactSnapshot> collectStaticRigidSurfaceCcdObstacles(
    const World& world, DeformableSolverStats& stats)
{
  ++stats.staticRigidSurfaceCcdSnapshotBuilds;

  const auto& registry = world.getRegistry();
  // Barrier-only obstacles keep their contact barrier but are excluded from the
  // CCD limiter, so a deformable can slide tangentially against them (and be
  // decelerated by friction) instead of having its whole step over-limited.
  auto view = registry.view<
      comps::RigidBodyTag,
      comps::StaticBodyTag,
      comps::DeformableSurfaceCcdObstacleTag,
      comps::CollisionGeometry,
      comps::Transform>(entt::exclude<comps::DeformableObstacleNoCcdTag>);

  std::vector<SurfaceContactSnapshot> snapshots;
  for (const auto entity : view) {
    const auto& geometry = view.get<comps::CollisionGeometry>(entity);
    const auto& transform = view.get<comps::Transform>(entity);
    if (!transform.position.allFinite()) {
      continue;
    }

    SurfaceContactSnapshot snapshot;
    if (geometry.shape.type == CollisionShapeType::Box) {
      if (!geometry.shape.halfExtents.allFinite()
          || (geometry.shape.halfExtents.array() <= 0.0).any()) {
        continue;
      }
      snapshot = makeStaticBoxSurfaceCcdSnapshot(
          entity, geometry.shape.halfExtents, transform);
      ++stats.staticRigidSurfaceCcdBoxCount;
    } else if (geometry.shape.type == CollisionShapeType::Sphere) {
      if (!std::isfinite(geometry.shape.radius)
          || geometry.shape.radius <= 0.0) {
        continue;
      }
      snapshot = makeStaticSphereSurfaceCcdSnapshot(
          entity, geometry.shape.radius, transform);
      ++stats.staticRigidSurfaceCcdSphereCount;
    } else {
      continue;
    }

    stats.staticRigidSurfaceCcdTriangleCount
        += snapshot.surfaceTriangles.size();
    stats.staticRigidSurfaceCcdEdgeCount += snapshot.surfaceEdges.size();
    snapshots.push_back(std::move(snapshot));
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
      case CollisionShapeType::Capsule:
      case CollisionShapeType::Mesh:
        break;
    }
  }

  return barriers;
}

//==============================================================================
// Static rigid SPHERE bodies opted in as deformable surface-CCD obstacles also
// exert a full radial barrier force on nearby deformable nodes. They are
// collected by center + radius (the smooth radial barrier needs no
// tessellation). Boxes opted in as surface-CCD obstacles are skipped here --
// their barrier force is a later increment -- and the surface CCD limiter
// remains the conservative no-penetration gate.
std::vector<SphereObstacleBarrier> collectSphereObstacleBarriers(
    const World& world)
{
  const auto& registry = world.getRegistry();
  auto view = registry.view<
      comps::RigidBodyTag,
      comps::StaticBodyTag,
      comps::DeformableSurfaceCcdObstacleTag,
      comps::CollisionGeometry,
      comps::Transform>();

  std::vector<SphereObstacleBarrier> obstacles;
  for (const auto entity : view) {
    const auto& geometry = view.get<comps::CollisionGeometry>(entity);
    const auto& transform = view.get<comps::Transform>(entity);
    if (geometry.shape.type != CollisionShapeType::Sphere
        || !(geometry.shape.radius > 0.0)
        || !std::isfinite(geometry.shape.radius)
        || !transform.position.allFinite()) {
      continue;
    }
    obstacles.push_back(
        SphereObstacleBarrier{transform.position, geometry.shape.radius});
  }
  return obstacles;
}

//==============================================================================
std::vector<BoxObstacleBarrier> collectBoxObstacleBarriers(const World& world)
{
  const auto& registry = world.getRegistry();
  auto view = registry.view<
      comps::RigidBodyTag,
      comps::StaticBodyTag,
      comps::DeformableSurfaceCcdObstacleTag,
      comps::CollisionGeometry,
      comps::Transform>();

  std::vector<BoxObstacleBarrier> obstacles;
  for (const auto entity : view) {
    const auto& geometry = view.get<comps::CollisionGeometry>(entity);
    const auto& transform = view.get<comps::Transform>(entity);
    if (geometry.shape.type != CollisionShapeType::Box
        || !geometry.shape.halfExtents.allFinite()
        || (geometry.shape.halfExtents.array() <= 0.0).any()
        || !transform.position.allFinite()
        || !transform.orientation.coeffs().allFinite()) {
      continue;
    }
    BoxObstacleBarrier obstacle;
    obstacle.center = transform.position;
    obstacle.rotation = transform.orientation.normalized().toRotationMatrix();
    obstacle.halfExtents = geometry.shape.halfExtents;
    obstacles.push_back(obstacle);
  }
  return obstacles;
}

//==============================================================================
std::vector<CapsuleObstacleBarrier> collectCapsuleObstacleBarriers(
    const World& world)
{
  const auto& registry = world.getRegistry();
  auto view = registry.view<
      comps::RigidBodyTag,
      comps::StaticBodyTag,
      comps::DeformableSurfaceCcdObstacleTag,
      comps::CollisionGeometry,
      comps::Transform>();

  std::vector<CapsuleObstacleBarrier> obstacles;
  for (const auto entity : view) {
    const auto& geometry = view.get<comps::CollisionGeometry>(entity);
    const auto& transform = view.get<comps::Transform>(entity);
    const double radius = geometry.shape.radius;
    const double halfHeight = geometry.shape.halfExtents.z();
    if (geometry.shape.type != CollisionShapeType::Capsule || !(radius > 0.0)
        || !std::isfinite(radius) || !(halfHeight > 0.0)
        || !std::isfinite(halfHeight) || !transform.position.allFinite()
        || !transform.orientation.coeffs().allFinite()) {
      continue;
    }
    // The capsule axis is the body z axis; map its two segment endpoints into
    // the world frame.
    const Eigen::Vector3d axis
        = transform.orientation.normalized().toRotationMatrix().col(2);
    CapsuleObstacleBarrier obstacle;
    obstacle.pointA = transform.position - halfHeight * axis;
    obstacle.pointB = transform.position + halfHeight * axis;
    obstacle.radius = radius;
    obstacles.push_back(obstacle);
  }
  return obstacles;
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
/// Transient cached topology for the experimental Vertex Block Descent inner
/// solver: the spring and tetrahedron element lists plus the vertex-graph
/// coloring (over the union of springs and tets) and the per-element incident
/// adjacencies, rebuilt only when the element topology changes. Defined here
/// (not in comps) so the heavy kernel headers stay out of the component
/// headers; it is stored per body and never serialized.
struct DeformableVbdScratch
{
  std::vector<dvbd::SpringElement> springs;
  std::vector<dvbd::TetMeshElement> tets;
  dvbd::VertexColoring coloring;
  dvbd::SpringAdjacency springAdjacency;
  dvbd::TetAdjacency tetAdjacency;
  // Per-vertex static ground-contact planes, rebuilt each step from the barrier
  // set at the warm-start position (lagged); a zero stiffness marks "no ground
  // under this vertex".
  std::vector<dvbd::ContactPlane> contactPlanes;
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
/// Solve one implicit-Euler step for a deformable body with the
/// Vertex Block Descent inner solver: graph-colored Gauss-Seidel block
/// coordinate descent on the same variational objective the default solver
/// minimizes, warm-started at the inertial target. Handles both distance
/// springs and volumetric tetrahedra, with the World path routing tet
/// elasticity through the shared FEM kernels so the body's material choice is
/// honored. Fills `scratch.next`; the caller's write-back updates
/// positions/velocities. With `config.contactStiffness > 0` it also resolves
/// static ground/obstacle half-space contact (penalty + optional Coulomb
/// friction) and lagged surface self-contact normal penalties. Moving
/// rigid-surface CCD and static capsule obstacle barriers still fall back to
/// the default solver.
void runVbdDeformableSolve(
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

  if (!vbdScratch.initialized || vbdScratch.cachedNodeCount != nodeCount
      || vbdScratch.cachedEdgeCount != model.edges.size()
      || vbdScratch.cachedTetCount != topology.tetrahedra.size()) {
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
  const bool anyStaticContact
      = !barriers.empty() || !sphereObstacles.empty() || !boxObstacles.empty();
  if (config.contactStiffness > 0.0 && anyStaticContact) {
    const double band = staticGroundBarrierActivationDistance();
    vbdScratch.contactPlanes.assign(nodeCount, dvbd::ContactPlane{});
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
      bool found = false;

      const auto groundTop = staticGroundTopAt(position, barriers);
      if (groundTop.has_value()) {
        const double gap = position.z() - *groundTop;
        if (gap < bestGap) {
          bestGap = gap;
          bestNormal = Eigen::Vector3d::UnitZ();
          bestOffset = *groundTop;
          found = true;
        }
      }

      for (const SphereObstacleBarrier& sphere : sphereObstacles) {
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
          found = true;
        }
      }

      for (const BoxObstacleBarrier& box : boxObstacles) {
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
          bestGap = gap;
          bestNormal = normal;
          bestOffset = planeOffset;
          found = true;
        }
      }

      if (found) {
        plane.normal = bestNormal;
        plane.offset = bestOffset;
        plane.stiffness = config.contactStiffness;
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
    vbdScratch.selfContactAdjacency = dvbd::SelfContactAdjacency::build(
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

  // state.positions holds x^t for this step (the write-back to the live state
  // happens after the solve), so it is the Rayleigh displacement reference.
  // parallelBlockDescentDeformable falls back to the full-featured serial
  // driver when workerThreads <= 1.
  const dvbd::BlockDescentStats result = dvbd::parallelBlockDescentDeformable(
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
  ProjectedNewtonMatrixFreeHessian matrixFreeHessian(
      solveMatrixFree ? nodeCount : 0);
  if (!solveMatrixFree) {
    triplets.reserve(tripletEstimate);
  } else {
    matrixFreeHessian.blocks.reserve(tripletEstimate / 9u);
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
  std::vector<double> edgeBlocks;
  std::vector<std::array<std::size_t, 2>> edgeBlockNodes;
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
  projectSymmetricBlocksToPsd(edgeBlocks.data(), 6, edgeBlockNodes.size());
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
    std::vector<double> tetBlocks;
    std::vector<std::array<std::size_t, 4>> tetBlockNodes;
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
    projectSymmetricBlocksToPsd(tetBlocks.data(), 12, tetBlockNodes.size());
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
    std::vector<double> barrierBlocks;
    std::vector<std::array<std::size_t, 4>> barrierBlockNodes;
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
    projectSymmetricBlocksToPsd(
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

  Eigen::SparseMatrix<double> hessian;
  if (!solveMatrixFree) {
    hessian.resize(dim, dim);
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

  Eigen::VectorXd solution;
  if (solveMatrixFree) {
    std::size_t cgIterations = 0;
    double cgError = 0.0;
    if (!solveMatrixFreeConjugateGradient(
            matrixFreeHessian, rhs, solution, cgIterations, cgError)
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
    if (contactScratch.femRestShapes.size() != topology.tetrahedra.size()) {
      contactScratch.femRestShapes.clear();
      contactScratch.femRestShapes.reserve(topology.tetrahedra.size());
      for (const auto& tet : topology.tetrahedra) {
        contactScratch.femRestShapes.push_back(
            fem::makeTetRestShape(
                topology.restPositions[tet.nodeA],
                topology.restPositions[tet.nodeB],
                topology.restPositions[tet.nodeC],
                topology.restPositions[tet.nodeD]));
      }
    }
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
    constexpr double gradientToleranceSquared = 1e-18;
    constexpr double armijo = 1e-4;
    constexpr double minStep = 1e-12;

    double lastGradSquared = 0.0;
    double lastAcceptedStepInfinityNorm = 0.0;
    // Whether the solve left the loop via an early break (converged, stalled,
    // or non-finite energy) rather than exhausting the iteration cap. It gates
    // the terminal-residual recompute below.
    bool brokeEarly = false;
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
      if (gradSquared <= gradientToleranceSquared) {
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
            if (std::isfinite(candidateEnergy)
                && candidateEnergy <= energy + armijo * directionalDerivative) {
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
    stats.finalGradientResidualNorm
        = std::max(stats.finalGradientResidualNorm, std::sqrt(lastGradSquared));
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
struct RigidIpcRuntimeBody
{
  entt::entity entity = entt::null;
  sxdetail::RigidIpcPose initialPose;
  sxdetail::RigidIpcBarrierSurface surface;
  sxdetail::RigidIpcBodyDynamicsTerm dynamicsTerm;
};

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
sxdetail::RigidIpcPose toRigidIpcPose(const comps::Transform& transform)
{
  sxdetail::RigidIpcPose pose;
  pose.position = transform.position;
  pose.rotation = rotationVectorFromQuaternion(transform.orientation);
  return pose;
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
  switch (shape.type) {
    case CollisionShapeType::Mesh:
      if (!isValidRigidIpcMeshShape(shape)) {
        return false;
      }
      surface.vertices = shape.vertices;
      surface.triangles = shape.triangles;
      return true;
    case CollisionShapeType::Box:
      if (!isPositiveFiniteVector(shape.halfExtents)) {
        return false;
      }
      copyBoxToRigidIpcSurface(shape.halfExtents, surface);
      return true;
    case CollisionShapeType::Sphere:
      if (!(shape.radius > 0.0) || !std::isfinite(shape.radius)) {
        return false;
      }
      copySphereToRigidIpcSurface(shape.radius, surface);
      return true;
    case CollisionShapeType::Capsule:
      // Capsule rigid-IPC surfaces are not supported yet; deformable-vs-capsule
      // contact uses the analytic capsule obstacle barrier.
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
  const auto& registry = world.getRegistry();
  const bool isStatic = registry.all_of<comps::StaticBodyTag>(entity);
  const auto& velocity = registry.get<comps::Velocity>(entity);
  const auto& mass = registry.get<comps::MassProperties>(entity);
  const auto& force = registry.get<comps::Force>(entity);

  sxdetail::RigidIpcBodyDynamicsState state;
  state.active = !isStatic;
  state.pose = pose;
  state.velocity.head<3>() = velocity.linear;
  state.velocity.tail<3>() = velocity.angular;
  state.mass = mass.mass;
  state.inertia = mass.inertia;
  state.generalizedForce.head<3>() = force.force;
  state.generalizedForce.tail<3>() = force.torque;
  if (!isStatic && mass.mass > 0.0 && std::isfinite(mass.mass)) {
    state.generalizedForce.head<3>() += mass.mass * world.getGravity();
  }

  return sxdetail::makeRigidIpcBodyDynamicsTerm(state, world.getTimeStep());
}

//==============================================================================
std::vector<RigidIpcRuntimeBody> collectRigidIpcRuntimeBodies(
    const World& world, RigidIpcSolverStats& stats)
{
  const auto& registry = world.getRegistry();
  auto view = registry.view<
      comps::RigidBodyTag,
      comps::CollisionGeometry,
      comps::Transform,
      comps::Velocity,
      comps::MassProperties,
      comps::Force,
      comps::FrameState,
      comps::FreeFrameProperties,
      comps::FrameCache>();

  std::vector<RigidIpcRuntimeBody> bodies;
  bodies.reserve(view.size_hint());
  for (const auto entity : view) {
    ++stats.bodyCount;

    const auto& geometry = view.get<comps::CollisionGeometry>(entity);
    const auto& transform = view.get<comps::Transform>(entity);

    RigidIpcRuntimeBody body;
    body.entity = entity;
    body.initialPose = toRigidIpcPose(transform);
    body.surface.body = bodies.size();
    body.surface.pose = body.initialPose;
    body.surface.dynamic = !registry.all_of<comps::StaticBodyTag>(entity);
    body.surface.frictionCoefficient = frictionOf(registry, entity);
    if (!copyCollisionShapeToRigidIpcSurface(geometry.shape, body.surface)) {
      ++stats.skippedUnsupportedShapeCount;
      continue;
    }

    body.dynamicsTerm
        = makeRuntimeRigidIpcDynamicsTerm(world, entity, body.initialPose);
    if (body.surface.dynamic) {
      ++stats.dynamicBodyCount;
    }
    ++stats.surfaceCount;
    bodies.push_back(std::move(body));
  }

  return bodies;
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
void applyRigidIpcPoseToRuntimeBody(
    World& world,
    const RigidIpcRuntimeBody& body,
    const sxdetail::RigidIpcPose& pose)
{
  auto& registry = world.getRegistry();
  auto& transform = registry.get<comps::Transform>(body.entity);
  auto& velocity = registry.get<comps::Velocity>(body.entity);

  const double timeStep = world.getTimeStep();
  transform.position = pose.position;
  transform.orientation = quaternionFromRotationVector(pose.rotation);
  if (timeStep > 0.0 && std::isfinite(timeStep)) {
    velocity.linear = (pose.position - body.initialPose.position) / timeStep;
    velocity.angular = (pose.rotation - body.initialPose.rotation) / timeStep;
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
void applyRigidIpcRuntimeResult(
    World& world,
    const std::vector<RigidIpcRuntimeBody>& bodies,
    const sxdetail::RigidIpcProjectedNewtonSolveResult& result)
{
  auto& registry = world.getRegistry();
  std::vector<entt::entity> dynamicEntities;
  dynamicEntities.reserve(bodies.size());
  for (const auto& body : bodies) {
    if (body.surface.dynamic) {
      dynamicEntities.push_back(body.entity);
    }
  }

  const auto orderedEntities
      = orderRigidBodiesParentBeforeChild(registry, dynamicEntities);
  for (const auto entity : orderedEntities) {
    const std::size_t bodyIndex = findRuntimeBodyIndex(bodies, entity);
    if (bodyIndex >= bodies.size() || bodyIndex >= result.surfaces.size()) {
      continue;
    }
    applyRigidIpcPoseToRuntimeBody(
        world, bodies[bodyIndex], result.surfaces[bodyIndex].pose);
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

  // Opt-in boxed-LCP path (PLAN-080 WS4): assemble and solve the frictionless
  // normal Delassus system with the pivoting Dantzig solver, applying the
  // resulting impulses to body velocities. The default SequentialImpulse path
  // below is unchanged.
  if (world.getContactSolverMethod() == ContactSolverMethod::BoxedLcp) {
    (void)detail::solveBoxedLcpContacts(
        registry, contacts, world.getTimeStep());
    resolveRigidBodyContactPositions(registry, contacts, world.getTimeStep());
    return;
  }

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
RigidIpcContactStage::RigidIpcContactStage(const std::size_t maxIterations)
  : m_maxIterations(maxIterations)
{
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
       {"collision_geometry", ComputeAccessMode::Read}}};
}

//==============================================================================
void RigidIpcContactStage::execute(World& world, ComputeExecutor& executor)
{
  m_lastStats.reset();

  const auto runtimeBodies = collectRigidIpcRuntimeBodies(world, m_lastStats);
  if (runtimeBodies.empty() || m_lastStats.dynamicBodyCount == 0u) {
    return;
  }

  std::vector<sxdetail::RigidIpcBarrierSurface> surfaces;
  std::vector<sxdetail::RigidIpcBodyDynamicsTerm> dynamicsTerms;
  surfaces.reserve(runtimeBodies.size());
  dynamicsTerms.reserve(runtimeBodies.size());
  for (const auto& body : runtimeBodies) {
    surfaces.push_back(body.surface);
    dynamicsTerms.push_back(body.dynamicsTerm);
  }

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
  for (const auto& body : runtimeBodies) {
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
  constexpr double activationDistance = 1e-2;
  options.barrier.squaredActivationDistance
      = activationDistance * activationDistance;
  options.barrier.stiffness = 1.0;
  options.adaptiveStiffness.enabled = true;
  options.adaptiveStiffness.averageMass = averageMass;
  options.adaptiveStiffness.bboxDiagonal = bboxDiagonal;
  options.friction.coefficient = 1.0;
  options.friction.staticFrictionDisplacement
      = std::max(0.0, 1e-3 * world.getTimeStep());
  options.dynamicsTerms = std::move(dynamicsTerms);
  options.maxIterations = m_maxIterations;
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

  sxdetail::RigidIpcProjectedNewtonSolveResult result;
  ComputeGraph graph;
  graph.addNode(
      "rigid_ipc_projected_newton_solve",
      [&]() {
        result = sxdetail::solveRigidIpcProjectedNewtonBarrierSystem(
            surfaces, options);
      },
      getMetadata());
  executor.execute(graph);

  m_lastStats.status = toPublicRigidIpcSolveStatus(result.status);
  m_lastStats.activeConstraints = result.assembly.activeConstraints.size();
  m_lastStats.activeFrictionConstraints
      = result.assembly.activeFrictionConstraints.size();
  m_lastStats.activeDynamicsTerms = result.assembly.activeDynamicsTerms;
  m_lastStats.solverIterations = result.stats.iterations;
  m_lastStats.frictionIterations = result.stats.frictionIterations;
  m_lastStats.acceptedSteps = result.stats.acceptedSteps;
  m_lastStats.lineSearchLimitedSteps = result.stats.lineSearchLimitedSteps;
  m_lastStats.initialValue = result.stats.initialValue;
  m_lastStats.finalValue = result.stats.finalValue;
  m_lastStats.initialGradientNorm = result.stats.initialGradientNorm;
  m_lastStats.finalGradientNorm = result.stats.finalGradientNorm;
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
  m_lastStats.converged = result.converged;
  m_lastStats.failed = result.failed;

  // A failed solve (conservative line search blocked the step, or the
  // factorization failed) never writes back: applying it could penetrate.
  if (result.failed) {
    return;
  }
  // Otherwise apply the last intersection-free iterate the bounded solve
  // reached. Every accepted Newton step is a descent step bounded by the
  // conservative line-search feasibility check, so even a not-fully-converged
  // result (e.g. a stiff friction-limited contact that plateaus before the
  // gradient tolerance) is a valid, penetration-free forward step that the next
  // substep re-solves from -- like the reference IPC, which advances with the
  // optimizer's feasible iterate rather than discarding a non-converged step.
  // (Unlike the reference this iterate is not Armijo energy-minimized; the
  // sufficient-decrease guarantee is a documented follow-up.) A solve that made
  // no progress at all (e.g. a zero iteration budget) is still skipped.
  if (!result.converged && !result.madeProgress()) {
    m_lastStats.nonConvergedResultSkipped = true;
    return;
  }

  m_lastStats.resultApplied = true;
  applyRigidIpcRuntimeResult(world, runtimeBodies, result);
}

//==============================================================================
std::size_t RigidIpcContactStage::getMaxIterations() const noexcept
{
  return m_maxIterations;
}

//==============================================================================
const RigidIpcSolverStats& RigidIpcContactStage::getLastStats() const noexcept
{
  return m_lastStats;
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
  const auto sphereObstacles = collectSphereObstacleBarriers(world);
  const auto boxObstacles = collectBoxObstacleBarriers(world);
  const auto capsuleObstacles = collectCapsuleObstacleBarriers(world);
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
