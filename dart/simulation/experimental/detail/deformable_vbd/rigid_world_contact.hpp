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
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS AND
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 */

#pragma once

#include <dart/simulation/experimental/body/contact.hpp>
#include <dart/simulation/experimental/comps/collision_geometry.hpp>
#include <dart/simulation/experimental/comps/contact_material.hpp>
#include <dart/simulation/experimental/comps/dynamics.hpp>
#include <dart/simulation/experimental/comps/frame_types.hpp>
#include <dart/simulation/experimental/comps/joint.hpp>
#include <dart/simulation/experimental/comps/rigid_body.hpp>
#include <dart/simulation/experimental/detail/deformable_vbd/rigid_block_kernel.hpp>

#include <entt/entt.hpp>

#include <algorithm>
#include <iterator>
#include <limits>
#include <map>
#include <span>
#include <utility>
#include <vector>

#include <cmath>
#include <cstddef>
#include <cstdint>

namespace dart::simulation::experimental::detail::deformable_vbd {

struct AvbdRigidWorldContactOptions
{
  double startStiffness = 1.0;
  double maxStiffness = std::numeric_limits<double>::infinity();
};

struct AvbdRigidWorldPointJointInput
{
  entt::entity bodyA = entt::null;
  entt::entity bodyB = entt::null;
  Eigen::Vector3d anchorA = Eigen::Vector3d::Zero();
  Eigen::Vector3d anchorB = Eigen::Vector3d::Zero();
  Eigen::Quaterniond targetRelativeOrientation = Eigen::Quaterniond::Identity();
  double startStiffness = 1.0;
  double maxStiffness = std::numeric_limits<double>::infinity();
};

struct AvbdRigidWorldPointJointConfig
{
  bool enabled = true;
  Eigen::Vector3d localAnchorA = Eigen::Vector3d::Zero();
  Eigen::Vector3d localAnchorB = Eigen::Vector3d::Zero();
  Eigen::Quaterniond targetRelativeOrientation = Eigen::Quaterniond::Identity();
  double startStiffness = 1.0;
  double maxStiffness = std::numeric_limits<double>::infinity();
};

struct AvbdRigidWorldContactSnapshot
{
  std::vector<entt::entity> entities;
  std::vector<AvbdRigidBodyState> states;
  std::vector<AvbdRigidBodyState> inertialTargets;
  std::vector<double> masses;
  std::vector<Eigen::Matrix3d> bodyInertias;
  std::vector<std::uint8_t> fixed;
  std::vector<AvbdRigidContactManifoldPoint> contacts;
  std::vector<AvbdRigidPointJoint> joints;
};

struct AvbdRigidWorldContactSolveOptions
{
  AvbdRowWarmStartOptions warmStart;
  AvbdRigidPointAttachmentOptions row;
  AvbdRigidPointPairFrictionOptions friction;
  AvbdRigidBlockDescentOptions descent;
};

struct AvbdRigidWorldContactSolveResult
{
  AvbdRigidBlockDescentStats stats;
  std::size_t normalRows = 0;
  std::size_t frictionRows = 0;
  std::size_t jointLinearRows = 0;
  std::size_t jointAngularRows = 0;
};

struct AvbdRigidWorldContactApplyResult
{
  std::size_t bodies = 0;
};

struct AvbdRigidWorldContactStepOptions
{
  AvbdRigidWorldContactOptions contact;
  AvbdRigidWorldContactSolveOptions solve;
  bool useVelocityInertialTargets = false;
};

struct AvbdRigidWorldContactStepResult
{
  std::size_t bodies = 0;
  std::size_t contacts = 0;
  std::size_t joints = 0;
  AvbdRigidWorldContactSolveResult solve;
  AvbdRigidWorldContactApplyResult apply;
};

//==============================================================================
inline std::uint64_t avbdRigidWorldContactObjectId(entt::entity entity) noexcept
{
  return static_cast<std::uint64_t>(entt::to_integral(entity)) + 1u;
}

//==============================================================================
inline AvbdContactEndpointId avbdRigidWorldBodyEndpointId(
    entt::entity entity) noexcept
{
  return AvbdContactEndpointId{
      avbdRigidWorldContactObjectId(entity),
      packAvbdContactFeatureId(AvbdContactFeatureKind::Body, 0)};
}

//==============================================================================
inline Eigen::Isometry3d avbdRigidWorldContactToIsometry(
    const AvbdRigidBodyState& state)
{
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.linear()
      = normalizeAvbdRigidOrientation(state.orientation).toRotationMatrix();
  transform.translation() = state.position;
  return transform;
}

//==============================================================================
inline Eigen::Isometry3d avbdRigidWorldContactFrameLocalTransform(
    const entt::registry& registry, entt::entity entity)
{
  if (const auto* props
      = registry.try_get<comps::FreeFrameProperties>(entity)) {
    return props->localTransform;
  }

  if (const auto* props
      = registry.try_get<comps::FixedFrameProperties>(entity)) {
    return props->localTransform;
  }

  return Eigen::Isometry3d::Identity();
}

//==============================================================================
inline Eigen::Isometry3d avbdRigidWorldContactFrameWorldTransform(
    const entt::registry& registry, entt::entity entity)
{
  if (entity == entt::null || !registry.valid(entity)) {
    return Eigen::Isometry3d::Identity();
  }

  if (const auto* transform = registry.try_get<comps::Transform>(entity)) {
    return avbdRigidWorldContactToIsometry(
        AvbdRigidBodyState{transform->position, transform->orientation});
  }

  if (const auto* cache = registry.try_get<comps::FrameCache>(entity);
      cache != nullptr && !cache->needTransformUpdate) {
    return cache->worldTransform;
  }

  const auto* frameState = registry.try_get<comps::FrameState>(entity);
  const Eigen::Isometry3d parentWorld
      = frameState != nullptr ? avbdRigidWorldContactFrameWorldTransform(
                                    registry, frameState->parentFrame)
                              : Eigen::Isometry3d::Identity();
  return parentWorld
         * avbdRigidWorldContactFrameLocalTransform(registry, entity);
}

//==============================================================================
inline void avbdRigidWorldContactMarkFrameSubtreeDirty(
    entt::registry& registry, entt::entity root)
{
  if (root == entt::null || !registry.valid(root)) {
    return;
  }

  std::vector<entt::entity> stack;
  stack.push_back(root);
  const auto frameStateView = registry.view<comps::FrameState>();

  while (!stack.empty()) {
    const entt::entity entity = stack.back();
    stack.pop_back();
    if (!registry.valid(entity)) {
      continue;
    }

    if (auto* cache = registry.try_get<comps::FrameCache>(entity)) {
      cache->needTransformUpdate = true;
    }

    for (const entt::entity child : frameStateView) {
      if (frameStateView.get<comps::FrameState>(child).parentFrame == entity) {
        stack.push_back(child);
      }
    }
  }
}

namespace detail {

//==============================================================================
inline double avbdRigidWorldContactFriction(
    const entt::registry& registry, entt::entity entity)
{
  const auto* material = registry.try_get<comps::ContactMaterial>(entity);
  if (material == nullptr || !std::isfinite(material->friction)) {
    return 1.0;
  }
  return std::max(0.0, material->friction);
}

//==============================================================================
inline Eigen::Vector3d avbdRigidWorldContactLocalPoint(
    const entt::registry& registry,
    entt::entity entity,
    const Eigen::Vector3d& point)
{
  const auto* transform = registry.try_get<comps::Transform>(entity);
  if (transform == nullptr) {
    return point;
  }

  return normalizeAvbdRigidOrientation(transform->orientation).conjugate()
         * (point - transform->position);
}

//==============================================================================
inline AvbdContactEndpointId avbdRigidWorldContactEndpointId(
    const entt::registry& registry,
    entt::entity entity,
    const Eigen::Vector3d& point)
{
  AvbdContactEndpointId endpoint = avbdRigidWorldBodyEndpointId(entity);

  const auto* geometry = registry.try_get<comps::CollisionGeometry>(entity);
  const CollisionShape* shape
      = geometry == nullptr ? nullptr : geometry->getPrimaryShape();
  if (shape == nullptr) {
    return endpoint;
  }

  const Eigen::Vector3d localPoint
      = avbdRigidWorldContactLocalPoint(registry, entity, point);
  if (!localPoint.allFinite()) {
    return endpoint;
  }

  if (shape->type == CollisionShapeType::Box && shape->halfExtents.allFinite()
      && (shape->halfExtents.array() > 0.0).all()) {
    const std::uint64_t featureCode
        = avbdBoxContactFeatureCode(localPoint, shape->halfExtents);
    endpoint.feature = packAvbdContactFeatureId(
        avbdBoxContactFeatureKind(featureCode),
        packAvbdBoxContactFeatureId(/*boxIndex=*/0, featureCode));
  } else if (
      shape->type == CollisionShapeType::Cylinder
      && std::isfinite(shape->radius) && shape->radius > 0.0
      && shape->halfExtents.allFinite() && shape->halfExtents.z() > 0.0) {
    const std::uint64_t featureCode = avbdCylinderContactFeatureCode(
        localPoint, shape->radius, shape->halfExtents.z());
    endpoint.feature = packAvbdContactFeatureId(
        avbdCylinderContactFeatureKind(featureCode),
        packAvbdCylinderContactFeatureId(/*cylinderIndex=*/0, featureCode));
  } else if (
      shape->type == CollisionShapeType::Capsule && std::isfinite(shape->radius)
      && shape->radius > 0.0 && shape->halfExtents.allFinite()
      && shape->halfExtents.z() > 0.0) {
    const std::uint64_t featureCode
        = avbdCapsuleContactFeatureCode(localPoint, shape->halfExtents.z());
    endpoint.feature = packAvbdContactFeatureId(
        avbdCapsuleContactFeatureKind(featureCode),
        packAvbdCapsuleContactFeatureId(/*capsuleIndex=*/0, featureCode));
  }
  return endpoint;
}

//==============================================================================
inline std::uint32_t findAvbdRigidWorldBodyIndex(
    const AvbdRigidWorldContactSnapshot& snapshot, entt::entity entity)
{
  const auto it
      = std::find(snapshot.entities.begin(), snapshot.entities.end(), entity);
  if (it == snapshot.entities.end()) {
    return std::numeric_limits<std::uint32_t>::max();
  }
  return static_cast<std::uint32_t>(
      std::distance(snapshot.entities.begin(), it));
}

//==============================================================================
inline std::uint32_t ensureAvbdRigidWorldBodyIndex(
    const entt::registry& registry,
    AvbdRigidWorldContactSnapshot& snapshot,
    entt::entity entity)
{
  const std::uint32_t existing = findAvbdRigidWorldBodyIndex(snapshot, entity);
  if (existing != std::numeric_limits<std::uint32_t>::max()) {
    return existing;
  }

  if (!registry.all_of<
          comps::RigidBodyTag,
          comps::Transform,
          comps::MassProperties>(entity)) {
    return std::numeric_limits<std::uint32_t>::max();
  }

  const auto& transform = registry.get<comps::Transform>(entity);
  const auto& mass = registry.get<comps::MassProperties>(entity);

  snapshot.entities.push_back(entity);
  snapshot.states.push_back(
      AvbdRigidBodyState{
          transform.position,
          normalizeAvbdRigidOrientation(transform.orientation)});
  snapshot.masses.push_back(mass.mass);
  snapshot.bodyInertias.push_back(mass.inertia);
  snapshot.fixed.push_back(
      registry.all_of<comps::StaticBodyTag>(entity) ? 1u : 0u);
  return static_cast<std::uint32_t>(snapshot.entities.size() - 1u);
}

} // namespace detail

//==============================================================================
inline AvbdRigidWorldContactSnapshot buildAvbdRigidWorldContactSnapshot(
    const entt::registry& registry,
    std::span<const Contact> contacts,
    const AvbdRigidWorldContactOptions& options = {})
{
  AvbdRigidWorldContactSnapshot snapshot;
  snapshot.contacts.reserve(contacts.size());
  std::map<
      std::pair<AvbdContactEndpointId, AvbdContactEndpointId>,
      std::uint32_t>
      rowCounters;

  for (std::size_t contactIndex = 0; contactIndex < contacts.size();
       ++contactIndex) {
    const Contact& contact = contacts[contactIndex];
    const entt::entity entityA = contact.bodyA.getEntity();
    const entt::entity entityB = contact.bodyB.getEntity();
    if (entityA == entt::null || entityB == entt::null || entityA == entityB
        || contact.depth <= 0.0 || !std::isfinite(contact.depth)
        || !contact.point.allFinite() || !contact.normal.allFinite()
        || contact.normal.squaredNorm() <= 0.0) {
      continue;
    }

    if (!registry.all_of<comps::RigidBodyTag>(entityA)
        || !registry.all_of<comps::RigidBodyTag>(entityB)) {
      continue;
    }

    const bool staticA = registry.all_of<comps::StaticBodyTag>(entityA);
    const bool staticB = registry.all_of<comps::StaticBodyTag>(entityB);
    if (staticA && staticB) {
      continue;
    }

    const std::uint32_t bodyA
        = detail::ensureAvbdRigidWorldBodyIndex(registry, snapshot, entityA);
    const std::uint32_t bodyB
        = detail::ensureAvbdRigidWorldBodyIndex(registry, snapshot, entityB);
    if (bodyA == std::numeric_limits<std::uint32_t>::max()
        || bodyB == std::numeric_limits<std::uint32_t>::max()) {
      continue;
    }

    AvbdRigidContactManifoldPoint manifoldPoint;
    manifoldPoint.bodyA = bodyA;
    manifoldPoint.bodyB = bodyB;
    manifoldPoint.endpointA = detail::avbdRigidWorldContactEndpointId(
        registry, entityA, contact.point);
    manifoldPoint.endpointB = detail::avbdRigidWorldContactEndpointId(
        registry, entityB, contact.point);
    manifoldPoint.point = contact.point;
    manifoldPoint.normalFromAtoB = contact.normal;
    manifoldPoint.depth = contact.depth;
    manifoldPoint.frictionCoefficient = std::sqrt(
        detail::avbdRigidWorldContactFriction(registry, entityA)
        * detail::avbdRigidWorldContactFriction(registry, entityB));
    manifoldPoint.startStiffness = options.startStiffness;
    manifoldPoint.maxStiffness = options.maxStiffness;
    const auto rowKey = canonicalizeAvbdContactEndpoints(
        manifoldPoint.endpointA, manifoldPoint.endpointB);
    std::uint32_t& nextRow = rowCounters[rowKey];
    manifoldPoint.row = nextRow;
    if (nextRow < std::numeric_limits<std::uint32_t>::max()) {
      ++nextRow;
    }
    snapshot.contacts.push_back(manifoldPoint);
  }

  return snapshot;
}

//==============================================================================
inline std::size_t appendAvbdRigidWorldPointJoints(
    const entt::registry& registry,
    std::span<const AvbdRigidWorldPointJointInput> inputs,
    AvbdRigidWorldContactSnapshot& snapshot)
{
  std::map<
      std::pair<AvbdContactEndpointId, AvbdContactEndpointId>,
      std::uint32_t>
      rowCounters;
  for (const AvbdRigidPointJoint& joint : snapshot.joints) {
    const auto rowKey
        = canonicalizeAvbdContactEndpoints(joint.endpointA, joint.endpointB);
    std::uint32_t& nextRow = rowCounters[rowKey];
    if (joint.row < std::numeric_limits<std::uint32_t>::max()) {
      nextRow = std::max(nextRow, joint.row + 1u);
    } else {
      nextRow = std::numeric_limits<std::uint32_t>::max();
    }
  }

  std::size_t appended = 0;
  for (const AvbdRigidWorldPointJointInput& input : inputs) {
    if (input.bodyA == entt::null || input.bodyB == entt::null
        || input.bodyA == input.bodyB || !input.anchorA.allFinite()
        || !input.anchorB.allFinite()) {
      continue;
    }

    const std::uint32_t bodyA = detail::ensureAvbdRigidWorldBodyIndex(
        registry, snapshot, input.bodyA);
    const std::uint32_t bodyB = detail::ensureAvbdRigidWorldBodyIndex(
        registry, snapshot, input.bodyB);
    if (bodyA == std::numeric_limits<std::uint32_t>::max()
        || bodyB == std::numeric_limits<std::uint32_t>::max()) {
      continue;
    }

    AvbdRigidPointJoint joint;
    joint.bodyA = bodyA;
    joint.bodyB = bodyB;
    joint.endpointA = avbdRigidWorldBodyEndpointId(input.bodyA);
    joint.endpointB = avbdRigidWorldBodyEndpointId(input.bodyB);
    joint.localPointA
        = avbdRigidBodyLocalPoint(snapshot.states[bodyA], input.anchorA);
    joint.localPointB
        = avbdRigidBodyLocalPoint(snapshot.states[bodyB], input.anchorB);
    joint.targetRelativeOrientation
        = normalizeAvbdRigidOrientation(input.targetRelativeOrientation);
    joint.startStiffness = std::max(0.0, input.startStiffness);
    joint.maxStiffness = std::max(joint.startStiffness, input.maxStiffness);

    const auto rowKey
        = canonicalizeAvbdContactEndpoints(joint.endpointA, joint.endpointB);
    std::uint32_t& nextRow = rowCounters[rowKey];
    joint.row = nextRow;
    if (nextRow < std::numeric_limits<std::uint32_t>::max()) {
      ++nextRow;
    }

    snapshot.joints.push_back(joint);
    ++appended;
  }

  return appended;
}

//==============================================================================
inline void predictAvbdRigidWorldContactInertialTargets(
    const entt::registry& registry,
    AvbdRigidWorldContactSnapshot& snapshot,
    double timeStep)
{
  snapshot.inertialTargets = snapshot.states;
  if (timeStep <= 0.0 || !std::isfinite(timeStep)) {
    return;
  }

  const std::size_t bodyCount = std::min(
      {snapshot.entities.size(),
       snapshot.inertialTargets.size(),
       snapshot.fixed.size()});
  for (std::size_t body = 0; body < bodyCount; ++body) {
    if (snapshot.fixed[body] != 0u) {
      continue;
    }

    const entt::entity entity = snapshot.entities[body];
    if (entity == entt::null || !registry.valid(entity)) {
      continue;
    }

    const auto* velocity = registry.try_get<comps::Velocity>(entity);
    if (velocity == nullptr) {
      continue;
    }

    AvbdRigidBodyState& target = snapshot.inertialTargets[body];
    target.position += velocity->linear * timeStep;

    const double angularSpeed = velocity->angular.norm();
    if (angularSpeed > 0.0 && std::isfinite(angularSpeed)) {
      target.orientation = normalizeAvbdRigidOrientation(
          avbdRigidOrientationDelta(velocity->angular * timeStep)
          * target.orientation);
    }
  }
}

//==============================================================================
inline AvbdRigidWorldContactSolveResult solveAvbdRigidWorldContactSnapshot(
    AvbdRigidWorldContactSnapshot& snapshot,
    AvbdScalarRowInventory& normalInventory,
    AvbdScalarRowInventory& frictionInventory,
    AvbdScalarRowInventory& jointLinearInventory,
    AvbdScalarRowInventory& jointAngularInventory,
    double timeStep,
    const AvbdRigidWorldContactSolveOptions& options = {})
{
  AvbdRigidWorldContactSolveResult result;
  if (snapshot.states.empty()) {
    return result;
  }

  std::vector<AvbdRigidBodyPointPairRow> normalRows;
  std::vector<AvbdRigidBodyPointPairFrictionRows> frictionRows;
  buildAvbdRigidContactManifoldRows(
      snapshot.states,
      snapshot.contacts,
      normalInventory,
      frictionInventory,
      normalRows,
      frictionRows,
      options.warmStart);
  result.normalRows = normalRows.size();
  result.frictionRows = 2u * frictionRows.size();

  std::vector<AvbdRigidBodyPointPairRow> jointLinearRows;
  std::vector<AvbdRigidBodyAngularPairRow> jointAngularRows;
  buildAvbdRigidPointJointConstraintRows(
      snapshot.states,
      snapshot.joints,
      jointLinearInventory,
      jointAngularInventory,
      jointLinearRows,
      jointAngularRows,
      options.warmStart);
  result.jointLinearRows = jointLinearRows.size();
  result.jointAngularRows = jointAngularRows.size();

  std::vector<AvbdRigidBodyPointPairRow> pointPairRows;
  pointPairRows.reserve(normalRows.size() + jointLinearRows.size());
  pointPairRows.insert(
      pointPairRows.end(), normalRows.begin(), normalRows.end());
  pointPairRows.insert(
      pointPairRows.end(), jointLinearRows.begin(), jointLinearRows.end());

  std::vector<AvbdRigidBodyPointAttachmentRow> attachmentRows;
  const std::vector<AvbdRigidBodyState> inertialTargets
      = snapshot.inertialTargets.size() == snapshot.states.size()
            ? snapshot.inertialTargets
            : snapshot.states;
  result.stats = blockDescentRigidBodiesAvbdRows(
      snapshot.states,
      snapshot.masses,
      snapshot.bodyInertias,
      snapshot.fixed,
      inertialTargets,
      timeStep,
      attachmentRows,
      pointPairRows,
      jointAngularRows,
      frictionRows,
      options.descent,
      options.row,
      options.friction);

  for (std::size_t i = 0; i < normalRows.size() && i < normalInventory.size();
       ++i) {
    normalInventory[i].state = pointPairRows[i].row.state;
  }
  const std::size_t jointLinearOffset = normalRows.size();
  for (std::size_t i = 0;
       i < jointLinearRows.size() && i < jointLinearInventory.size();
       ++i) {
    jointLinearInventory[i].state
        = pointPairRows[jointLinearOffset + i].row.state;
  }
  for (std::size_t i = 0;
       i < jointAngularRows.size() && i < jointAngularInventory.size();
       ++i) {
    jointAngularInventory[i].state = jointAngularRows[i].row.state;
  }
  for (std::size_t i = 0; i < frictionRows.size(); ++i) {
    const std::size_t first = 2u * i;
    const std::size_t second = first + 1u;
    if (second >= frictionInventory.size()) {
      break;
    }
    frictionInventory[first].state = frictionRows[i].first.state;
    frictionInventory[second].state = frictionRows[i].second.state;
  }

  return result;
}

//==============================================================================
inline AvbdRigidWorldContactSolveResult solveAvbdRigidWorldContactSnapshot(
    AvbdRigidWorldContactSnapshot& snapshot,
    AvbdScalarRowInventory& normalInventory,
    AvbdScalarRowInventory& frictionInventory,
    double timeStep,
    const AvbdRigidWorldContactSolveOptions& options = {})
{
  AvbdScalarRowInventory jointLinearInventory;
  AvbdScalarRowInventory jointAngularInventory;
  return solveAvbdRigidWorldContactSnapshot(
      snapshot,
      normalInventory,
      frictionInventory,
      jointLinearInventory,
      jointAngularInventory,
      timeStep,
      options);
}

//==============================================================================
inline AvbdRigidWorldContactApplyResult
applyAvbdRigidWorldContactVelocityProjection(
    entt::registry& registry,
    const AvbdRigidWorldContactSnapshot& snapshot,
    double timeStep)
{
  AvbdRigidWorldContactApplyResult result;
  if (timeStep <= 0.0 || !std::isfinite(timeStep)) {
    return result;
  }

  const std::size_t bodyCount = std::min(
      {snapshot.entities.size(),
       snapshot.states.size(),
       snapshot.fixed.size()});
  for (std::size_t body = 0; body < bodyCount; ++body) {
    if (snapshot.fixed[body] != 0u) {
      continue;
    }

    const entt::entity entity = snapshot.entities[body];
    if (entity == entt::null || !registry.valid(entity)) {
      continue;
    }

    const auto* transform = registry.try_get<comps::Transform>(entity);
    auto* velocity = registry.try_get<comps::Velocity>(entity);
    if (transform == nullptr || velocity == nullptr) {
      continue;
    }

    const AvbdRigidBodyState& state = snapshot.states[body];
    if (!state.position.allFinite()) {
      continue;
    }

    const Eigen::Quaterniond oldOrientation
        = normalizeAvbdRigidOrientation(transform->orientation);
    const Eigen::Quaterniond newOrientation
        = normalizeAvbdRigidOrientation(state.orientation);
    velocity->linear = (state.position - transform->position) / timeStep;
    velocity->angular
        = avbdRigidRotationVector(newOrientation * oldOrientation.conjugate())
          / timeStep;
    ++result.bodies;
  }

  return result;
}

//==============================================================================
inline AvbdRigidWorldContactApplyResult applyAvbdRigidWorldContactSnapshot(
    entt::registry& registry,
    const AvbdRigidWorldContactSnapshot& snapshot,
    double timeStep)
{
  AvbdRigidWorldContactApplyResult result;
  const std::size_t bodyCount = std::min(
      {snapshot.entities.size(),
       snapshot.states.size(),
       snapshot.fixed.size()});
  const bool updateVelocity = timeStep > 0.0 && std::isfinite(timeStep);

  for (std::size_t body = 0; body < bodyCount; ++body) {
    if (snapshot.fixed[body] != 0u) {
      continue;
    }

    const entt::entity entity = snapshot.entities[body];
    if (entity == entt::null || !registry.valid(entity)) {
      continue;
    }

    auto* transform = registry.try_get<comps::Transform>(entity);
    auto* velocity = registry.try_get<comps::Velocity>(entity);
    if (transform == nullptr || velocity == nullptr) {
      continue;
    }

    const AvbdRigidBodyState& state = snapshot.states[body];
    if (!state.position.allFinite()) {
      continue;
    }

    const Eigen::Quaterniond oldOrientation
        = normalizeAvbdRigidOrientation(transform->orientation);
    const Eigen::Vector3d oldPosition = transform->position;
    const Eigen::Quaterniond newOrientation
        = normalizeAvbdRigidOrientation(state.orientation);

    transform->position = state.position;
    transform->orientation = newOrientation;
    if (updateVelocity) {
      velocity->linear = (state.position - oldPosition) / timeStep;
      velocity->angular
          = avbdRigidRotationVector(newOrientation * oldOrientation.conjugate())
            / timeStep;
    }

    if (auto* props = registry.try_get<comps::FreeFrameProperties>(entity)) {
      const auto* frameState = registry.try_get<comps::FrameState>(entity);
      const Eigen::Isometry3d parentWorld
          = frameState != nullptr ? avbdRigidWorldContactFrameWorldTransform(
                                        registry, frameState->parentFrame)
                                  : Eigen::Isometry3d::Identity();
      props->localTransform
          = parentWorld.inverse()
            * avbdRigidWorldContactToIsometry(
                AvbdRigidBodyState{
                    transform->position, transform->orientation});
    }

    avbdRigidWorldContactMarkFrameSubtreeDirty(registry, entity);
    ++result.bodies;
  }

  return result;
}

//==============================================================================
inline AvbdRigidWorldContactStepResult runAvbdRigidWorldContactStep(
    entt::registry& registry,
    std::span<const Contact> contacts,
    std::span<const AvbdRigidWorldPointJointInput> joints,
    AvbdScalarRowInventory& normalInventory,
    AvbdScalarRowInventory& frictionInventory,
    AvbdScalarRowInventory& jointLinearInventory,
    AvbdScalarRowInventory& jointAngularInventory,
    double timeStep,
    const AvbdRigidWorldContactStepOptions& options = {})
{
  AvbdRigidWorldContactStepResult result;
  AvbdRigidWorldContactSnapshot snapshot
      = buildAvbdRigidWorldContactSnapshot(registry, contacts, options.contact);
  result.joints = appendAvbdRigidWorldPointJoints(registry, joints, snapshot);
  result.bodies = snapshot.states.size();
  result.contacts = snapshot.contacts.size();
  if (snapshot.states.empty()
      || (snapshot.contacts.empty() && snapshot.joints.empty())) {
    return result;
  }
  if (options.useVelocityInertialTargets) {
    predictAvbdRigidWorldContactInertialTargets(registry, snapshot, timeStep);
  }

  result.solve = solveAvbdRigidWorldContactSnapshot(
      snapshot,
      normalInventory,
      frictionInventory,
      jointLinearInventory,
      jointAngularInventory,
      timeStep,
      options.solve);
  if (result.solve.normalRows == 0u && result.solve.frictionRows == 0u
      && result.solve.jointLinearRows == 0u
      && result.solve.jointAngularRows == 0u) {
    return result;
  }

  result.apply
      = applyAvbdRigidWorldContactSnapshot(registry, snapshot, timeStep);
  return result;
}

//==============================================================================
inline std::vector<AvbdRigidWorldPointJointInput>
extractAvbdRigidWorldPointJointInputs(const entt::registry& registry)
{
  std::vector<AvbdRigidWorldPointJointInput> inputs;
  const auto view
      = registry.view<comps::Joint, AvbdRigidWorldPointJointConfig>();
  inputs.reserve(view.size_hint());

  for (const entt::entity entity : view) {
    const auto& joint = view.get<comps::Joint>(entity);
    const auto& config = view.get<AvbdRigidWorldPointJointConfig>(entity);
    if (!config.enabled || joint.type != comps::JointType::Fixed) {
      continue;
    }

    if (joint.parentLink == entt::null || joint.childLink == entt::null
        || joint.parentLink == joint.childLink) {
      continue;
    }

    if (!registry.all_of<
            comps::RigidBodyTag,
            comps::Transform,
            comps::MassProperties>(joint.parentLink)
        || !registry.all_of<
            comps::RigidBodyTag,
            comps::Transform,
            comps::MassProperties>(joint.childLink)) {
      continue;
    }

    if (!config.localAnchorA.allFinite() || !config.localAnchorB.allFinite()
        || !config.targetRelativeOrientation.coeffs().allFinite()) {
      continue;
    }

    const auto& transformA = registry.get<comps::Transform>(joint.parentLink);
    const auto& transformB = registry.get<comps::Transform>(joint.childLink);
    const Eigen::Isometry3d worldA = avbdRigidWorldContactToIsometry(
        AvbdRigidBodyState{transformA.position, transformA.orientation});
    const Eigen::Isometry3d worldB = avbdRigidWorldContactToIsometry(
        AvbdRigidBodyState{transformB.position, transformB.orientation});

    AvbdRigidWorldPointJointInput input;
    input.bodyA = joint.parentLink;
    input.bodyB = joint.childLink;
    input.anchorA = worldA * config.localAnchorA;
    input.anchorB = worldB * config.localAnchorB;
    input.targetRelativeOrientation
        = normalizeAvbdRigidOrientation(config.targetRelativeOrientation);
    input.startStiffness = config.startStiffness;
    input.maxStiffness = config.maxStiffness;
    inputs.push_back(input);
  }

  return inputs;
}

//==============================================================================
inline AvbdRigidWorldContactStepResult runAvbdRigidWorldContactStep(
    entt::registry& registry,
    std::span<const Contact> contacts,
    AvbdScalarRowInventory& normalInventory,
    AvbdScalarRowInventory& frictionInventory,
    AvbdScalarRowInventory& jointLinearInventory,
    AvbdScalarRowInventory& jointAngularInventory,
    double timeStep,
    const AvbdRigidWorldContactStepOptions& options = {})
{
  const std::vector<AvbdRigidWorldPointJointInput> joints
      = extractAvbdRigidWorldPointJointInputs(registry);
  return runAvbdRigidWorldContactStep(
      registry,
      contacts,
      std::span<const AvbdRigidWorldPointJointInput>(
          joints.data(), joints.size()),
      normalInventory,
      frictionInventory,
      jointLinearInventory,
      jointAngularInventory,
      timeStep,
      options);
}

//==============================================================================
inline AvbdRigidWorldContactStepResult runAvbdRigidWorldContactStep(
    entt::registry& registry,
    std::span<const Contact> contacts,
    AvbdScalarRowInventory& normalInventory,
    AvbdScalarRowInventory& frictionInventory,
    double timeStep,
    const AvbdRigidWorldContactStepOptions& options = {})
{
  AvbdScalarRowInventory jointLinearInventory;
  AvbdScalarRowInventory jointAngularInventory;
  return runAvbdRigidWorldContactStep(
      registry,
      contacts,
      std::span<const AvbdRigidWorldPointJointInput>(),
      normalInventory,
      frictionInventory,
      jointLinearInventory,
      jointAngularInventory,
      timeStep,
      options);
}

} // namespace dart::simulation::experimental::detail::deformable_vbd
