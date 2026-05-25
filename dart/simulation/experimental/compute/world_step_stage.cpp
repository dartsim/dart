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
#include "dart/simulation/experimental/comps/contact_material.hpp"
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
    entt::registry& registry, entt::entity entity, const double timeStep)
{
  const auto& force = registry.get<comps::Force>(entity);
  integrateRigidBodyVelocity(
      registry, entity, force.force, force.torque, timeStep);
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
  const auto forces = assembleRigidBodyForces(world, false);
  const auto& entities = forces.entities;

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

  auto state = extractRigidBodyState(world);
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
