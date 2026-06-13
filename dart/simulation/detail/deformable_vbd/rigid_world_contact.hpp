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

#include <dart/simulation/body/contact.hpp>
#include <dart/simulation/comps/collision_geometry.hpp>
#include <dart/simulation/comps/contact_material.hpp>
#include <dart/simulation/comps/dynamics.hpp>
#include <dart/simulation/comps/frame_types.hpp>
#include <dart/simulation/comps/joint.hpp>
#include <dart/simulation/comps/link.hpp>
#include <dart/simulation/comps/rigid_body.hpp>
#include <dart/simulation/detail/deformable_vbd/rigid_block_kernel.hpp>
#include <dart/simulation/detail/entity_conversion.hpp>
#include <dart/simulation/detail/world_registry_types.hpp>

#include <dart/common/memory_allocator.hpp>
#include <dart/common/stl_allocator.hpp>

#include <entt/entt.hpp>

#include <algorithm>
#include <functional>
#include <iterator>
#include <limits>
#include <span>
#include <type_traits>
#include <unordered_map>
#include <utility>
#include <vector>

#include <cmath>
#include <cstddef>
#include <cstdint>

namespace dart::simulation::detail::deformable_vbd {

struct AvbdRigidWorldContactOptions
{
  double startStiffness = 1.0;
  double maxStiffness = std::numeric_limits<double>::infinity();
};

enum class AvbdRigidWorldEndpointKind
{
  Unsupported,
  FreeRigidBody,
  MultibodyLink,
};

struct AvbdRigidWorldEndpoint
{
  entt::entity entity = entt::null;
  AvbdRigidWorldEndpointKind kind = AvbdRigidWorldEndpointKind::Unsupported;
  bool canProjectAsRigidBody = false;
};

struct AvbdRigidWorldProjectableBodyView
{
  const comps::Transform* transform = nullptr;
  const comps::MassProperties* mass = nullptr;
  bool isStatic = false;

  explicit operator bool() const noexcept
  {
    return transform != nullptr && mass != nullptr;
  }
};

struct AvbdRigidWorldPointJointInput
{
  entt::entity joint = entt::null;
  entt::entity bodyA = entt::null;
  entt::entity bodyB = entt::null;
  AvbdRigidWorldProjectableBodyView bodyAView;
  AvbdRigidWorldProjectableBodyView bodyBView;
  Eigen::Vector3d anchorA = Eigen::Vector3d::Zero();
  Eigen::Vector3d anchorB = Eigen::Vector3d::Zero();
  bool anchorsAreLocal = false;
  Eigen::Quaterniond targetRelativeOrientation = Eigen::Quaterniond::Identity();
  Eigen::Matrix3d linearAxes = Eigen::Matrix3d::Identity();
  Eigen::Matrix3d angularAxes = Eigen::Matrix3d::Identity();
  std::uint8_t linearAxisMask = kAvbdRigidJointAllAxesMask;
  std::uint8_t angularAxisMask = kAvbdRigidJointAllAxesMask;
  bool useLinearMotor = false;
  bool useAngularMotor = false;
  double motorTargetSpeed = 0.0;
  double motorMaxForce = std::numeric_limits<double>::infinity();
  double motorMaxTorque = std::numeric_limits<double>::infinity();
  double startStiffness = 1.0;
  double linearMaterialStiffness = std::numeric_limits<double>::infinity();
  double angularMaterialStiffness = std::numeric_limits<double>::infinity();
  double maxStiffness = std::numeric_limits<double>::infinity();
  double fractureThreshold = 0.0;
};

struct AvbdRigidWorldDistanceSpringInput
{
  entt::entity spring = entt::null;
  entt::entity bodyA = entt::null;
  entt::entity bodyB = entt::null;
  AvbdRigidWorldProjectableBodyView bodyAView;
  AvbdRigidWorldProjectableBodyView bodyBView;
  Eigen::Vector3d anchorA = Eigen::Vector3d::Zero();
  Eigen::Vector3d anchorB = Eigen::Vector3d::Zero();
  bool anchorsAreLocal = false;
  double restLength = 0.0;
  double startStiffness = 1.0;
  double materialStiffness = std::numeric_limits<double>::infinity();
  double maxStiffness = std::numeric_limits<double>::infinity();
};

struct AvbdRigidWorldPointJointConfig
{
  bool enabled = true;
  Eigen::Vector3d localAnchorA = Eigen::Vector3d::Zero();
  Eigen::Vector3d localAnchorB = Eigen::Vector3d::Zero();
  Eigen::Quaterniond targetRelativeOrientation = Eigen::Quaterniond::Identity();
  Eigen::Matrix3d linearAxes = Eigen::Matrix3d::Identity();
  Eigen::Matrix3d angularAxes = Eigen::Matrix3d::Identity();
  std::uint8_t linearAxisMask = kAvbdRigidJointAllAxesMask;
  std::uint8_t angularAxisMask = kAvbdRigidJointAllAxesMask;
  double startStiffness = 1.0;
  double linearMaterialStiffness = std::numeric_limits<double>::infinity();
  double angularMaterialStiffness = std::numeric_limits<double>::infinity();
  double maxStiffness = std::numeric_limits<double>::infinity();
};

struct AvbdRigidWorldDistanceSpringConfig
{
  bool enabled = true;
  entt::entity bodyA = entt::null;
  entt::entity bodyB = entt::null;
  Eigen::Vector3d localAnchorA = Eigen::Vector3d::Zero();
  Eigen::Vector3d localAnchorB = Eigen::Vector3d::Zero();
  double restLength = 0.0;
  double startStiffness = 1.0;
  double materialStiffness = std::numeric_limits<double>::infinity();
  double maxStiffness = std::numeric_limits<double>::infinity();
};

struct AvbdRigidWorldContactSnapshot
{
  using EntityAllocator = ::dart::common::StlAllocator<entt::entity>;
  using EntityVector = std::vector<entt::entity, EntityAllocator>;
  using EntityBodyIndexPair = std::pair<const entt::entity, std::uint32_t>;
  using EntityBodyIndexAllocator
      = ::dart::common::StlAllocator<EntityBodyIndexPair>;
  using EntityBodyIndexMap = std::unordered_map<
      entt::entity,
      std::uint32_t,
      std::hash<entt::entity>,
      std::equal_to<entt::entity>,
      EntityBodyIndexAllocator>;
  using BodyStateAllocator = ::dart::common::StlAllocator<AvbdRigidBodyState>;
  using DoubleAllocator = ::dart::common::StlAllocator<double>;
  using MatrixAllocator = ::dart::common::StlAllocator<Eigen::Matrix3d>;
  using ByteAllocator = ::dart::common::StlAllocator<std::uint8_t>;
  using ContactAllocator
      = ::dart::common::StlAllocator<AvbdRigidContactManifoldPoint>;
  using JointAllocator = ::dart::common::StlAllocator<AvbdRigidPointJoint>;
  using LinearMotorAllocator
      = ::dart::common::StlAllocator<AvbdRigidLinearMotor>;
  using MotorAllocator = ::dart::common::StlAllocator<AvbdRigidAngularMotor>;
  using DistanceSpringRow = AvbdRigidBodyPointPairDistanceSpringRow;
  using DistanceSpringAllocator
      = ::dart::common::StlAllocator<DistanceSpringRow>;
  using BodyStateVector = std::vector<AvbdRigidBodyState, BodyStateAllocator>;
  using DoubleVector = std::vector<double, DoubleAllocator>;
  using MatrixVector = std::vector<Eigen::Matrix3d, MatrixAllocator>;
  using ByteVector = std::vector<std::uint8_t, ByteAllocator>;
  using ContactVector
      = std::vector<AvbdRigidContactManifoldPoint, ContactAllocator>;
  using JointVector = std::vector<AvbdRigidPointJoint, JointAllocator>;
  using LinearMotorVector
      = std::vector<AvbdRigidLinearMotor, LinearMotorAllocator>;
  using MotorVector = std::vector<AvbdRigidAngularMotor, MotorAllocator>;
  using DistanceSpringVector
      = std::vector<DistanceSpringRow, DistanceSpringAllocator>;

  AvbdRigidWorldContactSnapshot() = default;

  explicit AvbdRigidWorldContactSnapshot(
      ::dart::common::MemoryAllocator& allocator)
    : entities(EntityAllocator{allocator}),
      entityBodyIndices(
          0u,
          std::hash<entt::entity>{},
          std::equal_to<entt::entity>{},
          EntityBodyIndexAllocator{allocator}),
      states(BodyStateAllocator{allocator}),
      inertialTargets(BodyStateAllocator{allocator}),
      masses(DoubleAllocator{allocator}),
      bodyInertias(MatrixAllocator{allocator}),
      fixed(ByteAllocator{allocator}),
      contacts(ContactAllocator{allocator}),
      joints(JointAllocator{allocator}),
      jointEntities(EntityAllocator{allocator}),
      linearMotors(LinearMotorAllocator{allocator}),
      motors(MotorAllocator{allocator}),
      distanceSprings(DistanceSpringAllocator{allocator}),
      distanceSpringEntities(EntityAllocator{allocator}),
      frameDirtyStack(EntityAllocator{allocator})
  {
  }

  template <
      typename Allocator,
      typename
      = std::enable_if_t<std::is_constructible_v<EntityAllocator, Allocator>>>
  explicit AvbdRigidWorldContactSnapshot(const Allocator& allocator)
    : entities(EntityAllocator{allocator}),
      entityBodyIndices(
          0u,
          std::hash<entt::entity>{},
          std::equal_to<entt::entity>{},
          EntityBodyIndexAllocator{allocator}),
      states(BodyStateAllocator{allocator}),
      inertialTargets(BodyStateAllocator{allocator}),
      masses(DoubleAllocator{allocator}),
      bodyInertias(MatrixAllocator{allocator}),
      fixed(ByteAllocator{allocator}),
      contacts(ContactAllocator{allocator}),
      joints(JointAllocator{allocator}),
      jointEntities(EntityAllocator{allocator}),
      linearMotors(LinearMotorAllocator{allocator}),
      motors(MotorAllocator{allocator}),
      distanceSprings(DistanceSpringAllocator{allocator}),
      distanceSpringEntities(EntityAllocator{allocator}),
      frameDirtyStack(EntityAllocator{allocator})
  {
  }

  EntityVector entities;
  EntityBodyIndexMap entityBodyIndices;
  BodyStateVector states;
  BodyStateVector inertialTargets;
  DoubleVector masses;
  MatrixVector bodyInertias;
  ByteVector fixed;
  ContactVector contacts;
  JointVector joints;
  EntityVector jointEntities;
  LinearMotorVector linearMotors;
  MotorVector motors;
  DistanceSpringVector distanceSprings;
  EntityVector distanceSpringEntities;
  EntityVector frameDirtyStack;
};

//==============================================================================
inline void clearAvbdRigidWorldContactSnapshot(
    AvbdRigidWorldContactSnapshot& snapshot)
{
  snapshot.entities.clear();
  snapshot.entityBodyIndices.clear();
  snapshot.states.clear();
  snapshot.inertialTargets.clear();
  snapshot.masses.clear();
  snapshot.bodyInertias.clear();
  snapshot.fixed.clear();
  snapshot.contacts.clear();
  snapshot.joints.clear();
  snapshot.jointEntities.clear();
  snapshot.linearMotors.clear();
  snapshot.motors.clear();
  snapshot.distanceSprings.clear();
  snapshot.distanceSpringEntities.clear();
  snapshot.frameDirtyStack.clear();
}

struct AvbdRigidWorldContactSolveOptions
{
  AvbdRowWarmStartOptions warmStart;
  AvbdRigidPointAttachmentOptions row;
  AvbdRigidPointPairFrictionOptions friction;
  AvbdRigidPointPairDistanceSpringOptions distanceSpring;
  AvbdRigidBlockDescentOptions descent;
};

struct AvbdRigidWorldContactSolveResult
{
  using SizeAllocator = ::dart::common::StlAllocator<std::size_t>;
  using SizeVector = std::vector<std::size_t, SizeAllocator>;

  AvbdRigidWorldContactSolveResult() = default;

  explicit AvbdRigidWorldContactSolveResult(SizeAllocator allocator)
    : fracturedJointIndices(allocator)
  {
  }

  AvbdRigidBlockDescentStats stats;
  std::size_t normalRows = 0;
  std::size_t frictionRows = 0;
  std::size_t jointLinearRows = 0;
  std::size_t jointAngularRows = 0;
  std::size_t motorRows = 0;
  std::size_t distanceSpringRows = 0;
  std::size_t fracturedJoints = 0;
  SizeVector fracturedJointIndices;
};

struct AvbdRigidWorldContactApplyResult
{
  std::size_t bodies = 0;
};

using AvbdRigidWorldRowCounterKey
    = std::pair<AvbdContactEndpointId, AvbdContactEndpointId>;

struct AvbdRigidWorldContactRowOrder
{
  AvbdRigidWorldRowCounterKey rowKey;
  Eigen::Vector3d localPoint = Eigen::Vector3d::Zero();
  std::size_t contact = 0u;
};

struct AvbdRigidWorldRowCounter
{
  AvbdRigidWorldRowCounterKey key;
  std::uint32_t nextRow = 0;
};

struct AvbdRigidWorldContactBuildScratch
{
  using RowCounterAllocator
      = ::dart::common::StlAllocator<AvbdRigidWorldRowCounter>;
  using RowOrderAllocator
      = ::dart::common::StlAllocator<AvbdRigidWorldContactRowOrder>;
  using RowCounterVector
      = std::vector<AvbdRigidWorldRowCounter, RowCounterAllocator>;
  using RowOrderVector
      = std::vector<AvbdRigidWorldContactRowOrder, RowOrderAllocator>;

  AvbdRigidWorldContactBuildScratch() = default;

  explicit AvbdRigidWorldContactBuildScratch(
      ::dart::common::MemoryAllocator& allocator)
    : rowCounters(RowCounterAllocator{allocator}),
      contactRowOrder(RowOrderAllocator{allocator})
  {
  }

  explicit AvbdRigidWorldContactBuildScratch(
      const AvbdRigidWorldContactSnapshot::ContactAllocator& allocator)
    : rowCounters(RowCounterAllocator{allocator}),
      contactRowOrder(RowOrderAllocator{allocator})
  {
  }

  template <
      typename Allocator,
      typename = std::enable_if_t<
          std::is_constructible_v<RowCounterAllocator, Allocator>>>
  explicit AvbdRigidWorldContactBuildScratch(const Allocator& allocator)
    : rowCounters(RowCounterAllocator{allocator}),
      contactRowOrder(RowOrderAllocator{allocator})
  {
  }

  RowCounterVector rowCounters;
  RowOrderVector contactRowOrder;
};

struct AvbdRigidWorldContactSolveScratch
{
  using PointPairAllocator
      = ::dart::common::StlAllocator<AvbdRigidBodyPointPairRow>;
  using FrictionPairAllocator
      = ::dart::common::StlAllocator<AvbdRigidBodyPointPairFrictionRows>;
  using AngularPairAllocator
      = ::dart::common::StlAllocator<AvbdRigidBodyAngularPairRow>;
  using DistanceSpringAllocator
      = ::dart::common::StlAllocator<AvbdRigidBodyPointPairDistanceSpringRow>;
  using AttachmentAllocator
      = ::dart::common::StlAllocator<AvbdRigidBodyPointAttachmentRow>;
  using BodyStateAllocator = ::dart::common::StlAllocator<AvbdRigidBodyState>;
  using PointPairVector
      = std::vector<AvbdRigidBodyPointPairRow, PointPairAllocator>;
  using FrictionPairVector
      = std::vector<AvbdRigidBodyPointPairFrictionRows, FrictionPairAllocator>;
  using AngularPairVector
      = std::vector<AvbdRigidBodyAngularPairRow, AngularPairAllocator>;
  using DistanceSpringVector = std::
      vector<AvbdRigidBodyPointPairDistanceSpringRow, DistanceSpringAllocator>;
  using AttachmentVector
      = std::vector<AvbdRigidBodyPointAttachmentRow, AttachmentAllocator>;
  using BodyStateVector = std::vector<AvbdRigidBodyState, BodyStateAllocator>;

  AvbdRigidWorldContactSolveScratch() = default;

  explicit AvbdRigidWorldContactSolveScratch(
      ::dart::common::MemoryAllocator& allocator)
    : contactRows(allocator),
      jointLinearRowsScratch(allocator),
      jointAngularRowsScratch(allocator),
      motorRowsScratch(allocator),
      distanceSpringRowsScratch(allocator),
      normalRows(PointPairAllocator{allocator}),
      frictionRows(FrictionPairAllocator{allocator}),
      jointLinearRows(PointPairAllocator{allocator}),
      jointAngularRows(AngularPairAllocator{allocator}),
      linearMotorRows(PointPairAllocator{allocator}),
      motorRows(AngularPairAllocator{allocator}),
      distanceSpringRows(DistanceSpringAllocator{allocator}),
      pointPairRows(PointPairAllocator{allocator}),
      angularRows(AngularPairAllocator{allocator}),
      attachmentRows(AttachmentAllocator{allocator}),
      fallbackInertialTargets(BodyStateAllocator{allocator}),
      rowIndexScratch(allocator)
  {
  }

  template <
      typename Allocator,
      typename = std::enable_if_t<
          std::is_constructible_v<PointPairAllocator, Allocator>>>
  explicit AvbdRigidWorldContactSolveScratch(const Allocator& allocator)
    : contactRows(allocator),
      jointLinearRowsScratch(allocator),
      jointAngularRowsScratch(allocator),
      motorRowsScratch(allocator),
      distanceSpringRowsScratch(allocator),
      normalRows(PointPairAllocator{allocator}),
      frictionRows(FrictionPairAllocator{allocator}),
      jointLinearRows(PointPairAllocator{allocator}),
      jointAngularRows(AngularPairAllocator{allocator}),
      linearMotorRows(PointPairAllocator{allocator}),
      motorRows(AngularPairAllocator{allocator}),
      distanceSpringRows(DistanceSpringAllocator{allocator}),
      pointPairRows(PointPairAllocator{allocator}),
      angularRows(AngularPairAllocator{allocator}),
      attachmentRows(AttachmentAllocator{allocator}),
      fallbackInertialTargets(BodyStateAllocator{allocator}),
      rowIndexScratch(allocator)
  {
  }

  void clear()
  {
    normalRows.clear();
    frictionRows.clear();
    jointLinearRows.clear();
    jointAngularRows.clear();
    linearMotorRows.clear();
    motorRows.clear();
    distanceSpringRows.clear();
    motorRowsScratch.clear();
    distanceSpringRowsScratch.clear();
    pointPairRows.clear();
    angularRows.clear();
    attachmentRows.clear();
    fallbackInertialTargets.clear();
    // Keep row-index layout scratch warm across frames; blockDescent rebuilds
    // any family whose body layout changed and clears absent families.
  }

  AvbdRigidContactManifoldRowScratch contactRows;
  AvbdRigidPointJointRowScratch jointLinearRowsScratch;
  AvbdRigidPointJointRowScratch jointAngularRowsScratch;
  AvbdRigidMotorRowScratch motorRowsScratch;
  AvbdRigidDistanceSpringRowScratch distanceSpringRowsScratch;
  PointPairVector normalRows;
  FrictionPairVector frictionRows;
  PointPairVector jointLinearRows;
  AngularPairVector jointAngularRows;
  PointPairVector linearMotorRows;
  AngularPairVector motorRows;
  DistanceSpringVector distanceSpringRows;
  PointPairVector pointPairRows;
  AngularPairVector angularRows;
  AttachmentVector attachmentRows;
  BodyStateVector fallbackInertialTargets;
  AvbdRigidBodyRowIndexScratch rowIndexScratch;
};

inline void reserveAvbdRigidWorldContactSnapshot(
    AvbdRigidWorldContactSnapshot& snapshot,
    std::size_t bodyCapacity,
    std::size_t contactCapacity,
    std::size_t jointCapacity,
    std::size_t motorCapacity,
    std::size_t distanceSpringCapacity = 0)
{
  snapshot.entities.reserve(bodyCapacity);
  if (bodyCapacity > detail::kAvbdRigidSmallRowStackCapacity
      && bodyCapacity > snapshot.entityBodyIndices.bucket_count()) {
    snapshot.entityBodyIndices.reserve(bodyCapacity);
  }
  snapshot.states.reserve(bodyCapacity);
  snapshot.inertialTargets.reserve(bodyCapacity);
  snapshot.masses.reserve(bodyCapacity);
  snapshot.bodyInertias.reserve(bodyCapacity);
  snapshot.fixed.reserve(bodyCapacity);
  snapshot.frameDirtyStack.reserve(bodyCapacity);
  snapshot.contacts.reserve(contactCapacity);
  snapshot.joints.reserve(jointCapacity);
  snapshot.jointEntities.reserve(jointCapacity);
  snapshot.linearMotors.reserve(motorCapacity);
  snapshot.motors.reserve(motorCapacity);
  snapshot.distanceSprings.reserve(distanceSpringCapacity);
  snapshot.distanceSpringEntities.reserve(distanceSpringCapacity);
}

inline void reserveAvbdRigidWorldContactSolveScratch(
    AvbdRigidWorldContactSolveScratch& scratch,
    std::size_t contactCapacity,
    std::size_t jointCapacity,
    std::size_t motorCapacity,
    std::size_t bodyCapacity = 0,
    std::size_t distanceSpringCapacity = 0)
{
  scratch.contactRows.activeContacts.reserve(contactCapacity);
  scratch.contactRows.normalDescriptors.reserve(contactCapacity);
  scratch.contactRows.frictionDescriptors.reserve(2u * contactCapacity);
  scratch.contactRows.previousFrictionDirections.reserve(2u * contactCapacity);
  scratch.jointLinearRowsScratch.activeRows.reserve(3u * jointCapacity);
  scratch.jointLinearRowsScratch.descriptors.reserve(3u * jointCapacity);
  scratch.jointAngularRowsScratch.activeRows.reserve(3u * jointCapacity);
  scratch.jointAngularRowsScratch.descriptors.reserve(3u * jointCapacity);
  scratch.motorRowsScratch.activeLinearRows.reserve(motorCapacity);
  scratch.motorRowsScratch.activeAngularRows.reserve(motorCapacity);
  scratch.distanceSpringRowsScratch.activeRows.reserve(distanceSpringCapacity);
  scratch.normalRows.reserve(contactCapacity);
  scratch.frictionRows.reserve(contactCapacity);
  scratch.jointLinearRows.reserve(3u * jointCapacity);
  scratch.jointAngularRows.reserve(3u * jointCapacity);
  scratch.linearMotorRows.reserve(motorCapacity);
  scratch.motorRows.reserve(motorCapacity);
  scratch.distanceSpringRows.reserve(distanceSpringCapacity);
  const std::size_t pointPairRowCapacity
      = contactCapacity + 3u * jointCapacity + motorCapacity;
  const std::size_t angularPairRowCapacity = 3u * jointCapacity + motorCapacity;
  scratch.pointPairRows.reserve(pointPairRowCapacity);
  scratch.angularRows.reserve(angularPairRowCapacity);
  scratch.fallbackInertialTargets.reserve(bodyCapacity);
  scratch.rowIndexScratch.reserve(
      bodyCapacity,
      /*attachmentRowCapacity=*/0u,
      pointPairRowCapacity,
      distanceSpringCapacity,
      angularPairRowCapacity,
      contactCapacity);
}

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
  std::size_t motors = 0;
  std::size_t distanceSprings = 0;
  std::size_t fracturedJoints = 0;
  AvbdRigidWorldContactSolveResult solve;
  AvbdRigidWorldContactApplyResult apply;
};

//==============================================================================
inline std::uint64_t avbdRigidWorldContactObjectId(entt::entity entity) noexcept
{
  return static_cast<std::uint64_t>(entt::to_integral(entity)) + 1u;
}

//==============================================================================
inline std::size_t avbdRigidWorldActiveJointAxisCount(std::uint8_t mask)
{
  std::size_t count = 0;
  for (std::uint8_t axis = 0; axis < 3u; ++axis) {
    if (detail::avbdRigidJointAxisEnabled(mask, axis)) {
      ++count;
    }
  }
  return count;
}

//==============================================================================
inline double avbdRigidWorldJointRowLambdaSquared(double lambda)
{
  if (!std::isfinite(lambda)) {
    return std::numeric_limits<double>::infinity();
  }
  return lambda * lambda;
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
inline AvbdRigidWorldProjectableBodyView avbdRigidWorldProjectableBody(
    const ::dart::simulation::detail::WorldRegistry& registry,
    entt::entity entity)
{
  if (entity == entt::null || !registry.valid(entity)
      || !registry.all_of<comps::RigidBodyTag>(entity)) {
    return {};
  }

  const auto* transform = registry.try_get<comps::Transform>(entity);
  const auto* mass = registry.try_get<comps::MassProperties>(entity);
  if (transform == nullptr || mass == nullptr) {
    return {};
  }

  return AvbdRigidWorldProjectableBodyView{
      transform,
      mass,
      registry.all_of<comps::StaticBodyTag>(entity)
          || registry.all_of<comps::KinematicBodyTag>(entity)};
}

//==============================================================================
inline AvbdRigidWorldProjectableBodyView avbdRigidWorldCachedProjectableBody(
    const ::dart::simulation::detail::WorldRegistry& registry,
    entt::entity entity,
    const AvbdRigidWorldProjectableBodyView& cached)
{
  if (cached) {
    return cached;
  }
  return avbdRigidWorldProjectableBody(registry, entity);
}

//==============================================================================
inline const comps::Transform* avbdRigidWorldProjectableBodyTransform(
    const ::dart::simulation::detail::WorldRegistry& registry,
    entt::entity entity)
{
  const AvbdRigidWorldProjectableBodyView body
      = avbdRigidWorldProjectableBody(registry, entity);
  if (!body) {
    return nullptr;
  }
  return body.transform;
}

//==============================================================================
inline AvbdRigidWorldEndpoint classifyAvbdRigidWorldEndpoint(
    const ::dart::simulation::detail::WorldRegistry& registry,
    entt::entity entity)
{
  AvbdRigidWorldEndpoint endpoint;
  endpoint.entity = entity;

  if (entity == entt::null || !registry.valid(entity)) {
    return endpoint;
  }

  if (avbdRigidWorldProjectableBodyTransform(registry, entity) != nullptr) {
    endpoint.kind = AvbdRigidWorldEndpointKind::FreeRigidBody;
    endpoint.canProjectAsRigidBody = true;
    return endpoint;
  }

  if (registry.all_of<comps::Link>(entity)) {
    endpoint.kind = AvbdRigidWorldEndpointKind::MultibodyLink;
    return endpoint;
  }

  return endpoint;
}

//==============================================================================
inline bool canProjectAvbdRigidWorldEndpointAsRigidBody(
    const ::dart::simulation::detail::WorldRegistry& registry,
    entt::entity entity)
{
  return avbdRigidWorldProjectableBodyTransform(registry, entity) != nullptr;
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
inline bool isAvbdRigidWorldPointJointType(comps::JointType type)
{
  return type == comps::JointType::Fixed || type == comps::JointType::Revolute
         || type == comps::JointType::Prismatic
         || type == comps::JointType::Spherical;
}

//==============================================================================
inline bool isAvbdRigidWorldPointJointFacade(
    const ::dart::simulation::detail::WorldRegistry& registry,
    entt::entity jointEntity,
    const comps::Joint& joint)
{
  if (!isAvbdRigidWorldPointJointType(joint.type)
      || joint.parentLink == joint.childLink) {
    return false;
  }

  const bool worldA = joint.parentLink == entt::null;
  const bool worldB = joint.childLink == entt::null;
  if (worldA && worldB) {
    return false;
  }

  const auto* childLink = registry.try_get<comps::Link>(joint.childLink);
  if (childLink != nullptr && childLink->parentJoint == jointEntity) {
    return false;
  }

  const auto endpointSupported = [&](entt::entity entity, bool worldEndpoint) {
    if (worldEndpoint) {
      return true;
    }
    const AvbdRigidWorldEndpoint endpoint
        = classifyAvbdRigidWorldEndpoint(registry, entity);
    return endpoint.kind == AvbdRigidWorldEndpointKind::FreeRigidBody
           || endpoint.kind == AvbdRigidWorldEndpointKind::MultibodyLink;
  };

  return endpointSupported(joint.parentLink, worldA)
         && endpointSupported(joint.childLink, worldB);
}

//==============================================================================
inline bool computeAvbdRigidWorldPointJointDefaultStiffness(
    const ::dart::simulation::detail::WorldRegistry& registry,
    const comps::Joint& joint,
    double& startStiffnessOut,
    double& maxStiffnessOut)
{
  const bool worldA = joint.parentLink == entt::null;
  const bool worldB = joint.childLink == entt::null;
  if (worldA && worldB) {
    return false;
  }

  const AvbdRigidWorldEndpoint endpointA
      = worldA
            ? AvbdRigidWorldEndpoint{joint.parentLink, AvbdRigidWorldEndpointKind::Unsupported, false}
            : classifyAvbdRigidWorldEndpoint(registry, joint.parentLink);
  const AvbdRigidWorldEndpoint endpointB
      = worldB
            ? AvbdRigidWorldEndpoint{joint.childLink, AvbdRigidWorldEndpointKind::Unsupported, false}
            : classifyAvbdRigidWorldEndpoint(registry, joint.childLink);
  if ((!worldA && endpointA.kind == AvbdRigidWorldEndpointKind::Unsupported)
      || (!worldB
          && endpointB.kind == AvbdRigidWorldEndpointKind::Unsupported)) {
    return false;
  }

  const bool hasMultibodyEndpoint
      = (!worldA && endpointA.kind == AvbdRigidWorldEndpointKind::MultibodyLink)
        || (!worldB
            && endpointB.kind == AvbdRigidWorldEndpointKind::MultibodyLink);
  const auto* configA
      = worldA
            ? nullptr
            : registry.try_get<comps::RigidAvbdContactConfig>(joint.parentLink);
  const auto* configB
      = worldB
            ? nullptr
            : registry.try_get<comps::RigidAvbdContactConfig>(joint.childLink);
  double startStiffness = 0.0;
  double maxStiffness = std::numeric_limits<double>::infinity();
  bool hasEnabledConfig = false;
  const auto absorb = [&](const comps::RigidAvbdContactConfig& config) {
    if (!config.enabled) {
      return;
    }
    hasEnabledConfig = true;
    startStiffness = std::max(startStiffness, config.startStiffness);
    maxStiffness = std::min(maxStiffness, config.maxStiffness);
  };
  if (configA != nullptr) {
    absorb(*configA);
  }
  if (configB != nullptr) {
    absorb(*configB);
  }
  if (!hasEnabledConfig) {
    if (hasMultibodyEndpoint) {
      startStiffness = std::numeric_limits<double>::infinity();
      maxStiffness = std::numeric_limits<double>::infinity();
    } else {
      const comps::RigidAvbdContactConfig defaultConfig;
      startStiffness = defaultConfig.startStiffness;
      maxStiffness = defaultConfig.maxStiffness;
    }
  }
  if (maxStiffness < startStiffness) {
    maxStiffness = startStiffness;
  }

  startStiffnessOut = startStiffness;
  maxStiffnessOut = maxStiffness;
  return true;
}

//==============================================================================
inline double avbdRigidWorldSymmetricEffortLimit(const comps::Joint& joint)
{
  if (joint.limits.effortLower.size() < 1 || joint.limits.effortUpper.size() < 1
      || std::isnan(joint.limits.effortLower[0])
      || std::isnan(joint.limits.effortUpper[0])) {
    return std::numeric_limits<double>::infinity();
  }

  const double lower = joint.limits.effortLower[0];
  const double upper = joint.limits.effortUpper[0];
  if (lower > 0.0 || upper < 0.0 || lower > upper) {
    return 0.0;
  }

  const double lowerMagnitude = std::isfinite(lower)
                                    ? std::max(0.0, -lower)
                                    : std::numeric_limits<double>::infinity();
  const double upperMagnitude = std::isfinite(upper)
                                    ? std::max(0.0, upper)
                                    : std::numeric_limits<double>::infinity();
  return std::min(lowerMagnitude, upperMagnitude);
}

//==============================================================================
inline bool configureAvbdRigidWorldPointJointFromCurrentPose(
    ::dart::simulation::detail::WorldRegistry& registry,
    entt::entity jointEntity,
    double startStiffness = 1.0,
    double maxStiffness = std::numeric_limits<double>::infinity())
{
  if (!registry.valid(jointEntity) || std::isnan(startStiffness)
      || startStiffness < 0.0 || std::isnan(maxStiffness)
      || maxStiffness < startStiffness) {
    return false;
  }

  auto* joint = registry.try_get<comps::Joint>(jointEntity);
  if (joint == nullptr || !isAvbdRigidWorldPointJointType(joint->type)
      || joint->parentLink == joint->childLink) {
    return false;
  }

  const bool worldA = joint->parentLink == entt::null;
  const bool worldB = joint->childLink == entt::null;
  if (worldA && worldB) {
    return false;
  }
  const auto* childLink = registry.try_get<comps::Link>(joint->childLink);
  if (childLink != nullptr && childLink->parentJoint == jointEntity) {
    return false;
  }

  const auto endpointTransform = [&](entt::entity entity,
                                     bool worldEndpoint,
                                     Eigen::Isometry3d& transformOut) -> bool {
    transformOut = Eigen::Isometry3d::Identity();
    if (worldEndpoint) {
      return true;
    }

    const AvbdRigidWorldEndpoint endpoint
        = classifyAvbdRigidWorldEndpoint(registry, entity);
    if (endpoint.kind != AvbdRigidWorldEndpointKind::FreeRigidBody
        && endpoint.kind != AvbdRigidWorldEndpointKind::MultibodyLink) {
      return false;
    }

    if (const auto* transform = registry.try_get<comps::Transform>(entity)) {
      if (!transform->position.allFinite()
          || !transform->orientation.coeffs().allFinite()) {
        return false;
      }
      transformOut = avbdRigidWorldContactToIsometry(
          AvbdRigidBodyState{transform->position, transform->orientation});
      return transformOut.matrix().allFinite();
    }

    const auto* cache = registry.try_get<comps::FrameCache>(entity);
    if (cache != nullptr && !cache->needTransformUpdate
        && cache->worldTransform.matrix().allFinite()) {
      transformOut = cache->worldTransform;
      return true;
    }

    if (const auto* link = registry.try_get<comps::Link>(entity)) {
      if (!link->worldTransform.matrix().allFinite()) {
        return false;
      }
      transformOut = link->worldTransform;
      return true;
    }

    return false;
  };

  Eigen::Isometry3d worldAFromEndpoint = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d worldBFromEndpoint = Eigen::Isometry3d::Identity();
  if (!endpointTransform(joint->parentLink, worldA, worldAFromEndpoint)
      || !endpointTransform(joint->childLink, worldB, worldBFromEndpoint)) {
    return false;
  }

  const double effectiveStartStiffness = joint->hasAvbdStiffnessState
                                             ? joint->avbdStartStiffness
                                             : startStiffness;
  const double effectiveMaxStiffness
      = joint->hasAvbdStiffnessState ? joint->avbdMaxStiffness : maxStiffness;
  if (std::isnan(effectiveStartStiffness) || effectiveStartStiffness < 0.0
      || std::isnan(joint->avbdLinearStiffness)
      || joint->avbdLinearStiffness < 0.0
      || std::isnan(joint->avbdAngularStiffness)
      || joint->avbdAngularStiffness < 0.0 || std::isnan(effectiveMaxStiffness)
      || effectiveMaxStiffness < effectiveStartStiffness) {
    return false;
  }

  Eigen::Vector3d localAnchorA = Eigen::Vector3d::Zero();
  Eigen::Vector3d localAnchorB = Eigen::Vector3d::Zero();
  Eigen::Quaterniond targetRelativeOrientation = Eigen::Quaterniond::Identity();
  if (joint->hasRigidBodyFixedJointAnchors) {
    if (!joint->rigidBodyFixedJointLocalAnchorParent.allFinite()
        || !joint->rigidBodyFixedJointLocalAnchorChild.allFinite()
        || !joint->rigidBodyFixedJointTargetRelativeOrientation.coeffs()
                .allFinite()
        || joint->rigidBodyFixedJointTargetRelativeOrientation.norm() == 0.0) {
      return false;
    }
    localAnchorA = joint->rigidBodyFixedJointLocalAnchorParent;
    localAnchorB = joint->rigidBodyFixedJointLocalAnchorChild;
    targetRelativeOrientation = normalizeAvbdRigidOrientation(
        joint->rigidBodyFixedJointTargetRelativeOrientation);
  } else {
    const Eigen::Quaterniond orientationA = normalizeAvbdRigidOrientation(
        Eigen::Quaterniond(worldAFromEndpoint.linear()));
    const Eigen::Quaterniond orientationB = normalizeAvbdRigidOrientation(
        Eigen::Quaterniond(worldBFromEndpoint.linear()));
    const Eigen::Vector3d worldAnchor = !worldB
                                            ? worldBFromEndpoint.translation()
                                            : worldAFromEndpoint.translation();
    localAnchorA = worldAFromEndpoint.inverse() * worldAnchor;
    localAnchorB = worldBFromEndpoint.inverse() * worldAnchor;
    targetRelativeOrientation = normalizeAvbdRigidOrientation(
        orientationA.conjugate() * orientationB);

    joint->hasRigidBodyFixedJointAnchors = true;
    joint->rigidBodyFixedJointLocalAnchorParent = localAnchorA;
    joint->rigidBodyFixedJointLocalAnchorChild = localAnchorB;
    joint->rigidBodyFixedJointTargetRelativeOrientation
        = targetRelativeOrientation;
  }

  Eigen::Matrix3d linearAxes = Eigen::Matrix3d::Identity();
  Eigen::Matrix3d angularAxes = Eigen::Matrix3d::Identity();
  std::uint8_t linearAxisMask = kAvbdRigidJointAllAxesMask;
  std::uint8_t angularAxisMask = kAvbdRigidJointAllAxesMask;
  if (joint->type == comps::JointType::Fixed) {
    // Defaults already represent a fixed joint.
  } else if (joint->type == comps::JointType::Spherical) {
    angularAxisMask = 0u;
  } else {
    if (!joint->axis.allFinite() || joint->axis.squaredNorm() <= 0.0) {
      return false;
    }

    const Eigen::Matrix3d jointAxes
        = avbdRigidJointAxesFromFreeAxis(joint->axis);
    if (joint->type == comps::JointType::Revolute) {
      angularAxes = jointAxes;
      angularAxisMask = avbdRigidJointAllButAxisMask(/*freeAxis=*/2u);
    } else {
      linearAxes = jointAxes;
      angularAxes = jointAxes;
      linearAxisMask = avbdRigidJointAllButAxisMask(/*freeAxis=*/2u);
    }
  }

  auto& config = registry.emplace_or_replace<AvbdRigidWorldPointJointConfig>(
      jointEntity);
  config.enabled = true;
  config.localAnchorA = localAnchorA;
  config.localAnchorB = localAnchorB;
  config.targetRelativeOrientation = targetRelativeOrientation;
  config.linearAxes = linearAxes;
  config.angularAxes = angularAxes;
  config.linearAxisMask = linearAxisMask;
  config.angularAxisMask = angularAxisMask;
  config.startStiffness = effectiveStartStiffness;
  config.linearMaterialStiffness = joint->avbdLinearStiffness;
  config.angularMaterialStiffness = joint->avbdAngularStiffness;
  config.maxStiffness = effectiveMaxStiffness;

  joint->hasAvbdStiffnessState = true;
  joint->avbdStartStiffness = effectiveStartStiffness;
  joint->avbdMaxStiffness = effectiveMaxStiffness;
  return true;
}

//==============================================================================
inline bool configureAvbdRigidWorldFixedJointFromCurrentPose(
    ::dart::simulation::detail::WorldRegistry& registry,
    entt::entity jointEntity,
    double startStiffness = 1.0,
    double maxStiffness = std::numeric_limits<double>::infinity())
{
  const auto* joint = registry.try_get<comps::Joint>(jointEntity);
  if (joint == nullptr || joint->type != comps::JointType::Fixed) {
    return false;
  }

  return configureAvbdRigidWorldPointJointFromCurrentPose(
      registry, jointEntity, startStiffness, maxStiffness);
}

//==============================================================================
inline std::size_t configureAvbdRigidWorldPointJointsFromCurrentPoses(
    ::dart::simulation::detail::WorldRegistry& registry)
{
  std::size_t configured = 0;
  const auto view = registry.view<comps::Joint>(
      entt::exclude<AvbdRigidWorldPointJointConfig>);

  for (const entt::entity entity : view) {
    const auto& joint = view.get<comps::Joint>(entity);
    if (!isAvbdRigidWorldPointJointType(joint.type)
        || joint.parentLink == joint.childLink) {
      continue;
    }

    const bool worldA = joint.parentLink == entt::null;
    const bool worldB = joint.childLink == entt::null;
    if (worldA && worldB) {
      continue;
    }
    const auto* childLink = registry.try_get<comps::Link>(joint.childLink);
    if (childLink != nullptr && childLink->parentJoint == entity) {
      continue;
    }
    double startStiffness = 0.0;
    double maxStiffness = std::numeric_limits<double>::infinity();
    if (!computeAvbdRigidWorldPointJointDefaultStiffness(
            registry, joint, startStiffness, maxStiffness)) {
      continue;
    }

    if (configureAvbdRigidWorldPointJointFromCurrentPose(
            registry, entity, startStiffness, maxStiffness)) {
      ++configured;
    }
  }

  return configured;
}

//==============================================================================
inline Eigen::Isometry3d avbdRigidWorldContactFrameLocalTransform(
    const ::dart::simulation::detail::WorldRegistry& registry,
    entt::entity entity)
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
    const ::dart::simulation::detail::WorldRegistry& registry,
    entt::entity entity)
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
    ::dart::simulation::detail::WorldRegistry& registry,
    entt::entity root,
    AvbdRigidWorldContactSnapshot::EntityVector& stack)
{
  stack.clear();
  if (root == entt::null || !registry.valid(root)) {
    return;
  }

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

// All shapes of a compound body share one contact-feature id namespace, so each
// shape is given a large disjoint per-shape block. The low portion of a block
// keeps the fixed primitive feature ranges from contact_kernel.hpp; mesh
// triangle/edge/vertex features use the remaining per-shape space.
inline constexpr std::uint64_t kAvbdRigidWorldShapeFeatureStride
    = std::uint64_t{1} << 40;
inline constexpr std::uint64_t kAvbdRigidWorldPlaneContactFeatureIdOffset
    = kAvbdCapsuleContactFeatureIdOffset + kAvbdCapsuleContactFeatureCodeCount;
inline constexpr std::uint64_t kAvbdRigidWorldMeshContactFeatureIdOffset
    = kAvbdRigidWorldPlaneContactFeatureIdOffset + 1u;
inline constexpr std::uint64_t kAvbdRigidWorldMeshIndexBits = 20;
inline constexpr std::uint64_t kAvbdRigidWorldMeshIndexMask
    = (std::uint64_t{1} << kAvbdRigidWorldMeshIndexBits) - 1u;

namespace detail {

inline double avbdRigidWorldContactFriction(
    const ::dart::simulation::detail::WorldRegistry& registry,
    entt::entity entity)
{
  const auto* material = registry.try_get<comps::ContactMaterial>(entity);
  if (material == nullptr || !std::isfinite(material->friction)) {
    return 1.0;
  }
  return std::max(0.0, material->friction);
}

//==============================================================================
inline Eigen::Vector3d avbdRigidWorldContactLocalPoint(
    const ::dart::simulation::detail::WorldRegistry& registry,
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
inline bool avbdRigidWorldShapeContainsLocalPoint(
    const CollisionShape& shape, const Eigen::Vector3d& localPoint)
{
  constexpr double kContainmentTolerance = 1e-9;
  if (!localPoint.allFinite()) {
    return false;
  }

  switch (shape.type) {
    case CollisionShapeType::Sphere:
      return std::isfinite(shape.radius) && shape.radius > 0.0
             && localPoint.norm() <= shape.radius + kContainmentTolerance;
    case CollisionShapeType::Box:
      return shape.halfExtents.allFinite()
             && (shape.halfExtents.array() > 0.0).all()
             && (localPoint.cwiseAbs().array()
                 <= shape.halfExtents.array() + kContainmentTolerance)
                    .all();
    case CollisionShapeType::Capsule: {
      if (!std::isfinite(shape.radius) || shape.radius <= 0.0
          || !shape.halfExtents.allFinite() || shape.halfExtents.z() <= 0.0) {
        return false;
      }
      const double axial = std::clamp(
          localPoint.z(), -shape.halfExtents.z(), shape.halfExtents.z());
      const Eigen::Vector3d axisPoint(0.0, 0.0, axial);
      return (localPoint - axisPoint).norm()
             <= shape.radius + kContainmentTolerance;
    }
    case CollisionShapeType::Cylinder:
      return std::isfinite(shape.radius) && shape.radius > 0.0
             && shape.halfExtents.allFinite() && shape.halfExtents.z() > 0.0
             && localPoint.head<2>().norm()
                    <= shape.radius + kContainmentTolerance
             && std::abs(localPoint.z())
                    <= shape.halfExtents.z() + kContainmentTolerance;
    case CollisionShapeType::Plane: {
      if (!shape.normal.allFinite() || shape.normal.squaredNorm() <= 0.0
          || !std::isfinite(shape.offset)) {
        return false;
      }
      const Eigen::Vector3d normal = shape.normal.normalized();
      return std::abs(normal.dot(localPoint) - shape.offset)
             <= kContainmentTolerance;
    }
    case CollisionShapeType::Mesh:
      for (const Eigen::Vector3i& triangle : shape.triangles) {
        if ((triangle.array() < 0).any()
            || triangle.x() >= static_cast<Eigen::Index>(shape.vertices.size())
            || triangle.y() >= static_cast<Eigen::Index>(shape.vertices.size())
            || triangle.z()
                   >= static_cast<Eigen::Index>(shape.vertices.size())) {
          continue;
        }

        const Eigen::Vector3d& a = shape.vertices[triangle.x()];
        const Eigen::Vector3d& b = shape.vertices[triangle.y()];
        const Eigen::Vector3d& c = shape.vertices[triangle.z()];
        if (!a.allFinite() || !b.allFinite() || !c.allFinite()) {
          continue;
        }

        const Eigen::Vector3d edge0 = b - a;
        const Eigen::Vector3d edge1 = c - a;
        const Eigen::Vector3d normal = edge0.cross(edge1);
        const double normalNorm = normal.norm();
        if (normalNorm <= kContainmentTolerance) {
          continue;
        }
        if (std::abs(normal.dot(localPoint - a)) / normalNorm
            > kContainmentTolerance) {
          continue;
        }

        const Eigen::Vector3d pointOffset = localPoint - a;
        const double d00 = edge0.dot(edge0);
        const double d01 = edge0.dot(edge1);
        const double d11 = edge1.dot(edge1);
        const double d20 = pointOffset.dot(edge0);
        const double d21 = pointOffset.dot(edge1);
        const double denom = d00 * d11 - d01 * d01;
        if (std::abs(denom) <= kContainmentTolerance) {
          continue;
        }

        const double v = (d11 * d20 - d01 * d21) / denom;
        const double w = (d00 * d21 - d01 * d20) / denom;
        const double u = 1.0 - v - w;
        if (u >= -kContainmentTolerance && v >= -kContainmentTolerance
            && w >= -kContainmentTolerance && u <= 1.0 + kContainmentTolerance
            && v <= 1.0 + kContainmentTolerance
            && w <= 1.0 + kContainmentTolerance) {
          return true;
        }
      }
      return false;
  }

  return false;
}

//==============================================================================
inline std::size_t inferAvbdRigidWorldContactShapeIndex(
    const std::vector<CollisionShape>& shapes,
    const Eigen::Vector3d& bodyLocalPoint)
{
  std::size_t match = Contact::UnknownShapeIndex;
  for (std::size_t i = 0; i < shapes.size(); ++i) {
    const CollisionShape& shape = shapes[i];
    if (!shape.localTransform.matrix().allFinite()) {
      continue;
    }

    const Eigen::Vector3d shapeLocalPoint
        = shape.localTransform.inverse() * bodyLocalPoint;
    if (!avbdRigidWorldShapeContainsLocalPoint(shape, shapeLocalPoint)) {
      continue;
    }

    if (match != Contact::UnknownShapeIndex) {
      return Contact::UnknownShapeIndex;
    }
    match = i;
  }
  return match;
}

//==============================================================================
inline bool avbdRigidWorldShapeFeatureBlock(
    std::size_t shapeIndex, std::uint64_t& shapeBlock)
{
  constexpr std::uint64_t kMaxShapeIndex
      = kAvbdContactFeatureIndexMask / kAvbdRigidWorldShapeFeatureStride;
  if (shapeIndex > kMaxShapeIndex) {
    return false;
  }
  shapeBlock = static_cast<std::uint64_t>(shapeIndex)
               * kAvbdRigidWorldShapeFeatureStride;
  return true;
}

//==============================================================================
struct AvbdRigidWorldMeshContactFeature
{
  AvbdContactFeatureKind kind = AvbdContactFeatureKind::Body;
  std::uint64_t localIndex = 0;
  bool valid = false;
};

//==============================================================================
inline bool avbdRigidWorldPackMeshEdgeFeature(
    int first, int second, std::uint64_t& localIndex)
{
  if (first < 0 || second < 0) {
    return false;
  }

  const std::uint64_t a = static_cast<std::uint64_t>(std::min(first, second));
  const std::uint64_t b = static_cast<std::uint64_t>(std::max(first, second));
  if (a > kAvbdRigidWorldMeshIndexMask || b > kAvbdRigidWorldMeshIndexMask) {
    return false;
  }

  localIndex = (a << kAvbdRigidWorldMeshIndexBits) | b;
  return true;
}

//==============================================================================
inline AvbdRigidWorldMeshContactFeature avbdRigidWorldMeshContactFeature(
    const CollisionShape& shape, const Eigen::Vector3d& localPoint)
{
  constexpr double kMeshFeatureTolerance = 1e-9;
  AvbdRigidWorldMeshContactFeature feature;
  if (shape.type != CollisionShapeType::Mesh || !localPoint.allFinite()) {
    return feature;
  }

  for (std::size_t triangleIndex = 0; triangleIndex < shape.triangles.size();
       ++triangleIndex) {
    const Eigen::Vector3i& triangle = shape.triangles[triangleIndex];
    if ((triangle.array() < 0).any()
        || triangle.x() >= static_cast<Eigen::Index>(shape.vertices.size())
        || triangle.y() >= static_cast<Eigen::Index>(shape.vertices.size())
        || triangle.z() >= static_cast<Eigen::Index>(shape.vertices.size())) {
      continue;
    }

    const Eigen::Vector3d& a = shape.vertices[triangle.x()];
    const Eigen::Vector3d& b = shape.vertices[triangle.y()];
    const Eigen::Vector3d& c = shape.vertices[triangle.z()];
    if (!a.allFinite() || !b.allFinite() || !c.allFinite()) {
      continue;
    }

    const Eigen::Vector3d edge0 = b - a;
    const Eigen::Vector3d edge1 = c - a;
    const Eigen::Vector3d normal = edge0.cross(edge1);
    const double normalNorm = normal.norm();
    if (normalNorm <= kMeshFeatureTolerance) {
      continue;
    }
    if (std::abs(normal.dot(localPoint - a)) / normalNorm
        > kMeshFeatureTolerance) {
      continue;
    }

    const Eigen::Vector3d pointOffset = localPoint - a;
    const double d00 = edge0.dot(edge0);
    const double d01 = edge0.dot(edge1);
    const double d11 = edge1.dot(edge1);
    const double d20 = pointOffset.dot(edge0);
    const double d21 = pointOffset.dot(edge1);
    const double denom = d00 * d11 - d01 * d01;
    if (std::abs(denom) <= kMeshFeatureTolerance) {
      continue;
    }

    const double v = (d11 * d20 - d01 * d21) / denom;
    const double w = (d00 * d21 - d01 * d20) / denom;
    const double u = 1.0 - v - w;
    if (u < -kMeshFeatureTolerance || v < -kMeshFeatureTolerance
        || w < -kMeshFeatureTolerance || u > 1.0 + kMeshFeatureTolerance
        || v > 1.0 + kMeshFeatureTolerance || w > 1.0 + kMeshFeatureTolerance) {
      continue;
    }

    const bool nearU = std::abs(u) <= kMeshFeatureTolerance;
    const bool nearV = std::abs(v) <= kMeshFeatureTolerance;
    const bool nearW = std::abs(w) <= kMeshFeatureTolerance;
    const int nearCount = static_cast<int>(nearU) + static_cast<int>(nearV)
                          + static_cast<int>(nearW);
    if (nearCount >= 2) {
      int vertex = triangle.x();
      if (!nearV) {
        vertex = triangle.y();
      } else if (!nearW) {
        vertex = triangle.z();
      }
      if (vertex >= 0
          && static_cast<std::uint64_t>(vertex)
                 <= kAvbdRigidWorldMeshIndexMask) {
        feature.kind = AvbdContactFeatureKind::Vertex;
        feature.localIndex = static_cast<std::uint64_t>(vertex);
        feature.valid = true;
        return feature;
      }
      return feature;
    }

    if (nearCount == 1) {
      int first = triangle.x();
      int second = triangle.y();
      if (nearU) {
        first = triangle.y();
        second = triangle.z();
      } else if (nearV) {
        first = triangle.z();
        second = triangle.x();
      }

      std::uint64_t edgeIndex = 0;
      if (avbdRigidWorldPackMeshEdgeFeature(first, second, edgeIndex)) {
        feature.kind = AvbdContactFeatureKind::Edge;
        feature.localIndex = edgeIndex;
        feature.valid = true;
        return feature;
      }
      return feature;
    }

    feature.kind = AvbdContactFeatureKind::Face;
    feature.localIndex = static_cast<std::uint64_t>(triangleIndex);
    feature.valid = true;
    return feature;
  }

  return feature;
}

//==============================================================================
inline AvbdContactEndpointId avbdRigidWorldContactEndpointId(
    const ::dart::simulation::detail::WorldRegistry& registry,
    entt::entity entity,
    std::size_t shapeIndex,
    const Eigen::Vector3d& point,
    const Eigen::Vector3d& shapeLocalPointHint)
{
  AvbdContactEndpointId endpoint = avbdRigidWorldBodyEndpointId(entity);

  const auto* geometry = registry.try_get<comps::CollisionGeometry>(entity);
  if (geometry == nullptr) {
    return endpoint;
  }

  const Eigen::Vector3d bodyLocalPoint
      = avbdRigidWorldContactLocalPoint(registry, entity, point);
  if (!bodyLocalPoint.allFinite()) {
    return endpoint;
  }

  const bool hasExplicitShapeIndex = shapeIndex != Contact::UnknownShapeIndex;
  if (shapeIndex == Contact::UnknownShapeIndex) {
    if (geometry->shapes.size() == 1u) {
      shapeIndex = 0u;
    } else {
      shapeIndex = inferAvbdRigidWorldContactShapeIndex(
          geometry->shapes, bodyLocalPoint);
      if (shapeIndex == Contact::UnknownShapeIndex) {
        return endpoint;
      }
    }
  }

  if (shapeIndex >= geometry->shapes.size()) {
    return endpoint;
  }

  const CollisionShape& shape = geometry->shapes[shapeIndex];
  if (!shape.localTransform.matrix().allFinite()) {
    return endpoint;
  }
  const Eigen::Vector3d shapeLocalPoint
      = hasExplicitShapeIndex && shapeLocalPointHint.allFinite()
            ? shapeLocalPointHint
            : shape.localTransform.inverse() * bodyLocalPoint;
  if (!shapeLocalPoint.allFinite()) {
    return endpoint;
  }

  // Offset every shape's feature id into its own disjoint block so distinct
  // shapes of one compound body never share a warm-start row. Default to a
  // shape-scoped body feature so shape types without a specialized face/edge
  // code (sphere) are still distinguished per shape; the per-type
  // packers below are called with local index 0 — their box/cylinder/capsule
  // offsets keep the shape types disjoint within a block — and refine it for
  // box/cylinder/capsule/plane/mesh.
  std::uint64_t shapeBlock = 0;
  if (!avbdRigidWorldShapeFeatureBlock(shapeIndex, shapeBlock)) {
    return endpoint;
  }
  endpoint.feature
      = packAvbdContactFeatureId(AvbdContactFeatureKind::Body, shapeBlock);
  if (shape.type == CollisionShapeType::Box && shape.halfExtents.allFinite()
      && (shape.halfExtents.array() > 0.0).all()) {
    const std::uint64_t featureCode
        = avbdBoxContactFeatureCode(shapeLocalPoint, shape.halfExtents);
    endpoint.feature = packAvbdContactFeatureId(
        avbdBoxContactFeatureKind(featureCode),
        shapeBlock + packAvbdBoxContactFeatureId(0, featureCode));
  } else if (
      shape.type == CollisionShapeType::Cylinder && std::isfinite(shape.radius)
      && shape.radius > 0.0 && shape.halfExtents.allFinite()
      && shape.halfExtents.z() > 0.0) {
    const std::uint64_t featureCode = avbdCylinderContactFeatureCode(
        shapeLocalPoint, shape.radius, shape.halfExtents.z());
    endpoint.feature = packAvbdContactFeatureId(
        avbdCylinderContactFeatureKind(featureCode),
        shapeBlock + packAvbdCylinderContactFeatureId(0, featureCode));
  } else if (
      shape.type == CollisionShapeType::Capsule && std::isfinite(shape.radius)
      && shape.radius > 0.0 && shape.halfExtents.allFinite()
      && shape.halfExtents.z() > 0.0) {
    const std::uint64_t featureCode
        = avbdCapsuleContactFeatureCode(shapeLocalPoint, shape.halfExtents.z());
    endpoint.feature = packAvbdContactFeatureId(
        avbdCapsuleContactFeatureKind(featureCode),
        shapeBlock + packAvbdCapsuleContactFeatureId(0, featureCode));
  } else if (
      shape.type == CollisionShapeType::Plane && shape.normal.allFinite()
      && shape.normal.squaredNorm() > 0.0 && std::isfinite(shape.offset)) {
    endpoint.feature = packAvbdContactFeatureId(
        AvbdContactFeatureKind::Face,
        shapeBlock + kAvbdRigidWorldPlaneContactFeatureIdOffset);
  } else if (shape.type == CollisionShapeType::Mesh) {
    const AvbdRigidWorldMeshContactFeature feature
        = avbdRigidWorldMeshContactFeature(shape, shapeLocalPoint);
    const std::uint64_t maxMeshLocalIndex
        = kAvbdRigidWorldShapeFeatureStride
          - kAvbdRigidWorldMeshContactFeatureIdOffset - 1u;
    if (feature.valid && feature.localIndex <= maxMeshLocalIndex) {
      endpoint.feature = packAvbdContactFeatureId(
          feature.kind,
          shapeBlock + kAvbdRigidWorldMeshContactFeatureIdOffset
              + feature.localIndex);
    }
  }
  return endpoint;
}

//==============================================================================
inline std::uint32_t findAvbdRigidWorldBodyIndex(
    const AvbdRigidWorldContactSnapshot& snapshot, entt::entity entity)
{
  if (!snapshot.entityBodyIndices.empty()) {
    const auto found = snapshot.entityBodyIndices.find(entity);
    if (found != snapshot.entityBodyIndices.end()) {
      const std::uint32_t index = found->second;
      if (index < snapshot.entities.size()
          && snapshot.entities[index] == entity) {
        return index;
      }
    }
  }

  const auto it
      = std::find(snapshot.entities.begin(), snapshot.entities.end(), entity);
  if (it == snapshot.entities.end()) {
    return std::numeric_limits<std::uint32_t>::max();
  }
  return static_cast<std::uint32_t>(
      std::distance(snapshot.entities.begin(), it));
}

//==============================================================================
inline void reserveAvbdRigidWorldBodyIndexMap(
    AvbdRigidWorldContactSnapshot& snapshot, std::size_t bodyCapacity)
{
  if (bodyCapacity > kAvbdRigidSmallRowStackCapacity) {
    snapshot.entityBodyIndices.reserve(bodyCapacity);
  }
}

//==============================================================================
inline void recordAvbdRigidWorldBodyIndex(
    AvbdRigidWorldContactSnapshot& snapshot,
    entt::entity entity,
    std::uint32_t index)
{
  if (!snapshot.entityBodyIndices.empty()
      || snapshot.entityBodyIndices.bucket_count()
             > kAvbdRigidSmallRowStackCapacity) {
    snapshot.entityBodyIndices[entity] = index;
    return;
  }

  if (snapshot.entities.size() <= kAvbdRigidSmallRowStackCapacity) {
    return;
  }

  snapshot.entityBodyIndices.reserve(snapshot.entities.size());
  for (std::size_t body = 0; body < snapshot.entities.size(); ++body) {
    snapshot.entityBodyIndices[snapshot.entities[body]]
        = static_cast<std::uint32_t>(body);
  }
}

//==============================================================================
inline std::uint32_t ensureAvbdRigidWorldBodyIndex(
    AvbdRigidWorldContactSnapshot& snapshot,
    entt::entity entity,
    const AvbdRigidWorldProjectableBodyView& body)
{
  const std::uint32_t existing = findAvbdRigidWorldBodyIndex(snapshot, entity);
  if (existing != std::numeric_limits<std::uint32_t>::max()) {
    recordAvbdRigidWorldBodyIndex(snapshot, entity, existing);
    return existing;
  }

  if (!body) {
    return std::numeric_limits<std::uint32_t>::max();
  }

  snapshot.entities.push_back(entity);
  snapshot.states.push_back(
      AvbdRigidBodyState{
          body.transform->position,
          normalizeAvbdRigidOrientation(body.transform->orientation)});
  snapshot.masses.push_back(body.mass->mass);
  snapshot.bodyInertias.push_back(body.mass->inertia);
  snapshot.fixed.push_back(body.isStatic ? 1u : 0u);
  const auto index = static_cast<std::uint32_t>(snapshot.entities.size() - 1u);
  recordAvbdRigidWorldBodyIndex(snapshot, entity, index);
  return index;
}

//==============================================================================
inline std::uint32_t ensureAvbdRigidWorldBodyIndex(
    const ::dart::simulation::detail::WorldRegistry& registry,
    AvbdRigidWorldContactSnapshot& snapshot,
    entt::entity entity)
{
  const AvbdRigidWorldProjectableBodyView body
      = avbdRigidWorldProjectableBody(registry, entity);
  if (!body) {
    return std::numeric_limits<std::uint32_t>::max();
  }
  return ensureAvbdRigidWorldBodyIndex(snapshot, entity, body);
}

//==============================================================================
inline bool avbdRigidWorldContactPointLess(
    const Eigen::Vector3d& lhs, const Eigen::Vector3d& rhs)
{
  for (Eigen::Index axis = 0; axis < 3; ++axis) {
    if (lhs[axis] < rhs[axis]) {
      return true;
    }
    if (rhs[axis] < lhs[axis]) {
      return false;
    }
  }
  return false;
}

//==============================================================================
inline Eigen::Vector3d avbdRigidWorldContactCanonicalLocalPoint(
    const AvbdRigidWorldContactSnapshot& snapshot,
    const AvbdRigidContactManifoldPoint& contact)
{
  const auto rowKey
      = canonicalizeAvbdContactEndpoints(contact.endpointA, contact.endpointB);
  if (rowKey.first == contact.endpointA
      && contact.bodyA < snapshot.states.size()) {
    return avbdRigidBodyLocalPoint(
        snapshot.states[contact.bodyA], contact.point);
  }
  if (rowKey.first == contact.endpointB
      && contact.bodyB < snapshot.states.size()) {
    return avbdRigidBodyLocalPoint(
        snapshot.states[contact.bodyB], contact.point);
  }
  return contact.point;
}

//==============================================================================
using AvbdRigidWorldEndpointPairKey
    = std::pair<AvbdContactEndpointId, AvbdContactEndpointId>;

struct AvbdRigidWorldEndpointPairKeyHash
{
  std::size_t operator()(
      const AvbdRigidWorldEndpointPairKey& key) const noexcept
  {
    std::size_t seed = 0u;
    const auto combine = [&seed](std::uint64_t value) {
      seed ^= std::hash<std::uint64_t>{}(value) + 0x9e3779b97f4a7c15ull
              + (seed << 6u) + (seed >> 2u);
    };
    combine(key.first.object);
    combine(key.first.feature);
    combine(key.second.object);
    combine(key.second.feature);
    return seed;
  }
};

using AvbdRigidWorldRowCounterMap = std::unordered_map<
    AvbdRigidWorldEndpointPairKey,
    std::uint32_t,
    AvbdRigidWorldEndpointPairKeyHash>;

//==============================================================================
template <typename RowOrderVector, typename RowCounterVector>
inline void assignAvbdRigidWorldContactRows(
    AvbdRigidWorldContactSnapshot& snapshot,
    RowOrderVector& rows,
    RowCounterVector& rowCounters)
{
  rows.clear();
  rows.reserve(snapshot.contacts.size());
  for (std::size_t contact = 0; contact < snapshot.contacts.size(); ++contact) {
    const AvbdRigidContactManifoldPoint& manifoldPoint
        = snapshot.contacts[contact];
    rows.push_back(
        AvbdRigidWorldContactRowOrder{
            canonicalizeAvbdContactEndpoints(
                manifoldPoint.endpointA, manifoldPoint.endpointB),
            avbdRigidWorldContactCanonicalLocalPoint(snapshot, manifoldPoint),
            contact});
  }

  std::sort(
      rows.begin(),
      rows.end(),
      [](const AvbdRigidWorldContactRowOrder& lhs,
         const AvbdRigidWorldContactRowOrder& rhs) {
        if (lhs.rowKey < rhs.rowKey) {
          return true;
        }
        if (rhs.rowKey < lhs.rowKey) {
          return false;
        }
        if (avbdRigidWorldContactPointLess(lhs.localPoint, rhs.localPoint)) {
          return true;
        }
        if (avbdRigidWorldContactPointLess(rhs.localPoint, lhs.localPoint)) {
          return false;
        }
        return lhs.contact < rhs.contact;
      });

  rowCounters.clear();
  rowCounters.reserve(rows.size());
  for (const AvbdRigidWorldContactRowOrder& row : rows) {
    const auto found = std::lower_bound(
        rowCounters.begin(),
        rowCounters.end(),
        row.rowKey,
        [](const AvbdRigidWorldRowCounter& counter,
           const AvbdRigidWorldRowCounterKey& value) {
          return counter.key < value;
        });
    AvbdRigidWorldRowCounter& counter
        = found != rowCounters.end() && found->key == row.rowKey
              ? *found
              : *rowCounters.insert(
                    found, AvbdRigidWorldRowCounter{row.rowKey, 0u});
    snapshot.contacts[row.contact].row = counter.nextRow;
    if (counter.nextRow < std::numeric_limits<std::uint32_t>::max()) {
      ++counter.nextRow;
    }
  }
}

//==============================================================================
inline void assignAvbdRigidWorldContactRows(
    AvbdRigidWorldContactSnapshot& snapshot)
{
  AvbdRigidWorldContactBuildScratch scratch(snapshot.contacts.get_allocator());
  assignAvbdRigidWorldContactRows(
      snapshot, scratch.contactRowOrder, scratch.rowCounters);
}

} // namespace detail

inline void resetAvbdRigidWorldRowCounters(
    AvbdRigidWorldContactBuildScratch& scratch, std::size_t capacity)
{
  scratch.rowCounters.clear();
  scratch.rowCounters.reserve(capacity);
}

//==============================================================================
template <typename RowCounterVector>
inline AvbdRigidWorldRowCounter& ensureAvbdRigidWorldRowCounter(
    RowCounterVector& rowCounters, const AvbdRigidWorldRowCounterKey& key)
{
  const auto it = std::lower_bound(
      rowCounters.begin(),
      rowCounters.end(),
      key,
      [](const AvbdRigidWorldRowCounter& counter,
         const AvbdRigidWorldRowCounterKey& value) {
        return counter.key < value;
      });
  if (it != rowCounters.end() && it->key == key) {
    return *it;
  }
  return *rowCounters.insert(it, AvbdRigidWorldRowCounter{key, 0u});
}

//==============================================================================
template <typename RowCounterVector>
inline std::uint32_t claimNextAvbdRigidWorldRow(
    RowCounterVector& rowCounters, const AvbdRigidWorldRowCounterKey& key)
{
  AvbdRigidWorldRowCounter& counter
      = ensureAvbdRigidWorldRowCounter(rowCounters, key);
  const std::uint32_t row = counter.nextRow;
  if (counter.nextRow < std::numeric_limits<std::uint32_t>::max()) {
    ++counter.nextRow;
  }
  return row;
}

//==============================================================================
inline void buildAvbdRigidWorldContactSnapshot(
    const ::dart::simulation::detail::WorldRegistry& registry,
    std::span<const Contact> contacts,
    AvbdRigidWorldContactSnapshot& snapshot,
    AvbdRigidWorldContactBuildScratch& scratch,
    const AvbdRigidWorldContactOptions& options = {})
{
  clearAvbdRigidWorldContactSnapshot(snapshot);
  reserveAvbdRigidWorldContactSnapshot(
      snapshot,
      /*bodyCapacity=*/2u * contacts.size(),
      /*contactCapacity=*/contacts.size(),
      /*jointCapacity=*/0u,
      /*motorCapacity=*/0u,
      /*distanceSpringCapacity=*/0u);
  resetAvbdRigidWorldRowCounters(scratch, contacts.size());

  for (std::size_t contactIndex = 0; contactIndex < contacts.size();
       ++contactIndex) {
    const Contact& contact = contacts[contactIndex];
    const entt::entity entityA = toRegistryEntity(contact.bodyA.getEntity());
    const entt::entity entityB = toRegistryEntity(contact.bodyB.getEntity());
    if (entityA == entt::null || entityB == entt::null || entityA == entityB
        || contact.depth <= 0.0 || !std::isfinite(contact.depth)
        || !contact.point.allFinite() || !contact.normal.allFinite()
        || contact.normal.squaredNorm() <= 0.0) {
      continue;
    }

    const AvbdRigidWorldProjectableBodyView bodyAView
        = avbdRigidWorldProjectableBody(registry, entityA);
    const AvbdRigidWorldProjectableBodyView bodyBView
        = avbdRigidWorldProjectableBody(registry, entityB);
    if (!bodyAView || !bodyBView) {
      continue;
    }

    if (bodyAView.isStatic && bodyBView.isStatic) {
      continue;
    }

    const std::uint32_t bodyA
        = detail::ensureAvbdRigidWorldBodyIndex(snapshot, entityA, bodyAView);
    const std::uint32_t bodyB
        = detail::ensureAvbdRigidWorldBodyIndex(snapshot, entityB, bodyBView);
    if (bodyA == std::numeric_limits<std::uint32_t>::max()
        || bodyB == std::numeric_limits<std::uint32_t>::max()) {
      continue;
    }

    AvbdRigidContactManifoldPoint manifoldPoint;
    manifoldPoint.bodyA = bodyA;
    manifoldPoint.bodyB = bodyB;
    manifoldPoint.endpointA = detail::avbdRigidWorldContactEndpointId(
        registry,
        entityA,
        contact.shapeIndexA,
        contact.point,
        contact.localPointA);
    manifoldPoint.endpointB = detail::avbdRigidWorldContactEndpointId(
        registry,
        entityB,
        contact.shapeIndexB,
        contact.point,
        contact.localPointB);
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
    manifoldPoint.row = claimNextAvbdRigidWorldRow(scratch.rowCounters, rowKey);
    snapshot.contacts.push_back(manifoldPoint);
  }
  detail::assignAvbdRigidWorldContactRows(
      snapshot, scratch.contactRowOrder, scratch.rowCounters);
}

//==============================================================================
inline AvbdRigidWorldContactSnapshot buildAvbdRigidWorldContactSnapshot(
    const ::dart::simulation::detail::WorldRegistry& registry,
    std::span<const Contact> contacts,
    const AvbdRigidWorldContactOptions& options = {})
{
  AvbdRigidWorldContactSnapshot snapshot;
  AvbdRigidWorldContactBuildScratch scratch;
  buildAvbdRigidWorldContactSnapshot(
      registry, contacts, snapshot, scratch, options);
  return snapshot;
}

//==============================================================================
inline void buildAvbdRigidWorldContactSnapshotInto(
    const ::dart::simulation::detail::WorldRegistry& registry,
    std::span<const Contact> contacts,
    AvbdRigidWorldContactSnapshot& snapshot,
    const AvbdRigidWorldContactOptions& options = {})
{
  AvbdRigidWorldContactBuildScratch scratch(snapshot.contacts.get_allocator());
  buildAvbdRigidWorldContactSnapshot(
      registry, contacts, snapshot, scratch, options);
}

//==============================================================================
inline std::size_t appendAvbdRigidWorldPointJoints(
    const ::dart::simulation::detail::WorldRegistry& registry,
    std::span<const AvbdRigidWorldPointJointInput> inputs,
    AvbdRigidWorldContactSnapshot& snapshot,
    AvbdRigidWorldContactBuildScratch& scratch)
{
  if (!inputs.empty()) {
    const std::size_t maxNewBodies = 2u * inputs.size();
    reserveAvbdRigidWorldContactSnapshot(
        snapshot,
        snapshot.entities.size() + maxNewBodies,
        snapshot.contacts.size(),
        snapshot.joints.size() + inputs.size(),
        snapshot.linearMotors.size() + snapshot.motors.size() + inputs.size(),
        snapshot.distanceSprings.size());
  }
  resetAvbdRigidWorldRowCounters(
      scratch, snapshot.joints.size() + inputs.size());
  for (const AvbdRigidPointJoint& joint : snapshot.joints) {
    const auto rowKey
        = canonicalizeAvbdContactEndpoints(joint.endpointA, joint.endpointB);
    AvbdRigidWorldRowCounter& counter
        = ensureAvbdRigidWorldRowCounter(scratch.rowCounters, rowKey);
    if (joint.row < std::numeric_limits<std::uint32_t>::max()) {
      counter.nextRow = std::max(counter.nextRow, joint.row + 1u);
    } else {
      counter.nextRow = std::numeric_limits<std::uint32_t>::max();
    }
  }

  std::size_t appended = 0;
  for (const AvbdRigidWorldPointJointInput& input : inputs) {
    if (input.bodyA == entt::null || input.bodyB == entt::null
        || input.bodyA == input.bodyB || !input.anchorA.allFinite()
        || !input.anchorB.allFinite() || std::isnan(input.startStiffness)
        || input.startStiffness < 0.0
        || std::isnan(input.linearMaterialStiffness)
        || input.linearMaterialStiffness < 0.0
        || std::isnan(input.angularMaterialStiffness)
        || input.angularMaterialStiffness < 0.0
        || std::isnan(input.maxStiffness)
        || input.maxStiffness < input.startStiffness) {
      continue;
    }

    const AvbdRigidWorldProjectableBodyView bodyAView
        = avbdRigidWorldCachedProjectableBody(
            registry, input.bodyA, input.bodyAView);
    const AvbdRigidWorldProjectableBodyView bodyBView
        = avbdRigidWorldCachedProjectableBody(
            registry, input.bodyB, input.bodyBView);
    if (!bodyAView || !bodyBView
        || (bodyAView.isStatic && bodyBView.isStatic)) {
      continue;
    }

    const std::uint32_t bodyA = detail::ensureAvbdRigidWorldBodyIndex(
        snapshot, input.bodyA, bodyAView);
    const std::uint32_t bodyB = detail::ensureAvbdRigidWorldBodyIndex(
        snapshot, input.bodyB, bodyBView);
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
        = input.anchorsAreLocal
              ? input.anchorA
              : avbdRigidBodyLocalPoint(snapshot.states[bodyA], input.anchorA);
    joint.localPointB
        = input.anchorsAreLocal
              ? input.anchorB
              : avbdRigidBodyLocalPoint(snapshot.states[bodyB], input.anchorB);
    joint.targetRelativeOrientation
        = normalizeAvbdRigidOrientation(input.targetRelativeOrientation);
    joint.linearAxes = input.linearAxes;
    joint.angularAxes = input.angularAxes;
    joint.linearAxisMask = input.linearAxisMask;
    joint.angularAxisMask = input.angularAxisMask;
    joint.startStiffness = std::max(0.0, input.startStiffness);
    joint.linearMaterialStiffness = input.linearMaterialStiffness;
    joint.angularMaterialStiffness = input.angularMaterialStiffness;
    joint.maxStiffness = std::max(joint.startStiffness, input.maxStiffness);
    joint.fractureThreshold = input.fractureThreshold;

    const auto rowKey
        = canonicalizeAvbdContactEndpoints(joint.endpointA, joint.endpointB);
    joint.row = claimNextAvbdRigidWorldRow(scratch.rowCounters, rowKey);

    if (input.useAngularMotor && input.angularAxes.allFinite()
        && std::isfinite(input.motorTargetSpeed) && input.motorMaxTorque > 0.0
        && !std::isnan(input.motorMaxTorque)) {
      const Eigen::Quaterniond orientationA
          = normalizeAvbdRigidOrientation(snapshot.states[bodyA].orientation);
      const Eigen::Quaterniond orientationB
          = normalizeAvbdRigidOrientation(snapshot.states[bodyB].orientation);
      snapshot.motors.push_back(makeAvbdRigidAngularMotor(
          joint.bodyA,
          joint.bodyB,
          joint.endpointA,
          joint.endpointB,
          orientationA.conjugate() * orientationB,
          input.angularAxes.col(2),
          input.motorTargetSpeed,
          input.motorMaxTorque,
          joint.startStiffness,
          joint.maxStiffness,
          joint.row));
    }
    if (input.useLinearMotor && input.linearAxes.allFinite()
        && std::isfinite(input.motorTargetSpeed) && input.motorMaxForce > 0.0
        && !std::isnan(input.motorMaxForce)) {
      snapshot.linearMotors.push_back(makeAvbdRigidLinearMotor(
          joint.bodyA,
          joint.bodyB,
          joint.endpointA,
          joint.endpointB,
          joint.localPointA,
          joint.localPointB,
          input.linearAxes.col(2),
          input.motorTargetSpeed,
          input.motorMaxForce,
          joint.startStiffness,
          joint.maxStiffness,
          joint.row));
    }

    snapshot.joints.push_back(joint);
    snapshot.jointEntities.push_back(input.joint);
    ++appended;
  }

  return appended;
}

//==============================================================================
inline std::size_t appendAvbdRigidWorldDistanceSprings(
    const ::dart::simulation::detail::WorldRegistry& registry,
    std::span<const AvbdRigidWorldDistanceSpringInput> inputs,
    AvbdRigidWorldContactSnapshot& snapshot,
    AvbdRigidWorldContactBuildScratch& scratch)
{
  if (!inputs.empty()) {
    const std::size_t maxNewBodies = 2u * inputs.size();
    reserveAvbdRigidWorldContactSnapshot(
        snapshot,
        snapshot.entities.size() + maxNewBodies,
        snapshot.contacts.size(),
        snapshot.joints.size(),
        snapshot.linearMotors.size() + snapshot.motors.size(),
        snapshot.distanceSprings.size() + inputs.size());
  }
  resetAvbdRigidWorldRowCounters(
      scratch, snapshot.distanceSprings.size() + inputs.size());
  for (const AvbdRigidBodyPointPairDistanceSpringRow& spring :
       snapshot.distanceSprings) {
    const auto rowKey
        = canonicalizeAvbdContactEndpoints(spring.endpointA, spring.endpointB);
    AvbdRigidWorldRowCounter& counter
        = ensureAvbdRigidWorldRowCounter(scratch.rowCounters, rowKey);
    if (spring.rowIndex < std::numeric_limits<std::uint32_t>::max()) {
      counter.nextRow = std::max(counter.nextRow, spring.rowIndex + 1u);
    } else {
      counter.nextRow = std::numeric_limits<std::uint32_t>::max();
    }
  }

  std::size_t appended = 0;
  for (const AvbdRigidWorldDistanceSpringInput& input : inputs) {
    if (input.bodyA == entt::null || input.bodyB == entt::null
        || input.bodyA == input.bodyB || !input.anchorA.allFinite()
        || !input.anchorB.allFinite() || !std::isfinite(input.restLength)
        || input.restLength < 0.0 || std::isnan(input.startStiffness)
        || input.startStiffness < 0.0 || std::isnan(input.materialStiffness)
        || input.materialStiffness < 0.0 || std::isnan(input.maxStiffness)
        || input.maxStiffness < input.startStiffness) {
      continue;
    }

    const AvbdRigidWorldProjectableBodyView bodyAView
        = avbdRigidWorldCachedProjectableBody(
            registry, input.bodyA, input.bodyAView);
    const AvbdRigidWorldProjectableBodyView bodyBView
        = avbdRigidWorldCachedProjectableBody(
            registry, input.bodyB, input.bodyBView);
    if (!bodyAView || !bodyBView
        || (bodyAView.isStatic && bodyBView.isStatic)) {
      continue;
    }

    const std::uint32_t bodyA = detail::ensureAvbdRigidWorldBodyIndex(
        snapshot, input.bodyA, bodyAView);
    const std::uint32_t bodyB = detail::ensureAvbdRigidWorldBodyIndex(
        snapshot, input.bodyB, bodyBView);
    if (bodyA == std::numeric_limits<std::uint32_t>::max()
        || bodyB == std::numeric_limits<std::uint32_t>::max()) {
      continue;
    }

    AvbdRigidBodyPointPairDistanceSpringRow spring;
    spring.bodyA = bodyA;
    spring.bodyB = bodyB;
    spring.endpointA = avbdRigidWorldBodyEndpointId(input.bodyA);
    spring.endpointB = avbdRigidWorldBodyEndpointId(input.bodyB);
    spring.row.localPointA
        = input.anchorsAreLocal
              ? input.anchorA
              : avbdRigidBodyLocalPoint(snapshot.states[bodyA], input.anchorA);
    spring.row.localPointB
        = input.anchorsAreLocal
              ? input.anchorB
              : avbdRigidBodyLocalPoint(snapshot.states[bodyB], input.anchorB);
    spring.row.restLength = input.restLength;
    spring.row.materialStiffness = input.materialStiffness;
    spring.startStiffness = std::max(0.0, input.startStiffness);
    spring.maxStiffness = std::max(spring.startStiffness, input.maxStiffness);

    const auto rowKey
        = canonicalizeAvbdContactEndpoints(spring.endpointA, spring.endpointB);
    spring.rowIndex = claimNextAvbdRigidWorldRow(scratch.rowCounters, rowKey);

    snapshot.distanceSprings.push_back(spring);
    snapshot.distanceSpringEntities.push_back(input.spring);
    ++appended;
  }

  return appended;
}

//==============================================================================
inline std::size_t appendAvbdRigidWorldPointJoints(
    const ::dart::simulation::detail::WorldRegistry& registry,
    std::span<const AvbdRigidWorldPointJointInput> inputs,
    AvbdRigidWorldContactSnapshot& snapshot)
{
  AvbdRigidWorldContactBuildScratch scratch(snapshot.contacts.get_allocator());
  return appendAvbdRigidWorldPointJoints(registry, inputs, snapshot, scratch);
}

//==============================================================================
inline std::size_t appendAvbdRigidWorldDistanceSprings(
    const ::dart::simulation::detail::WorldRegistry& registry,
    std::span<const AvbdRigidWorldDistanceSpringInput> inputs,
    AvbdRigidWorldContactSnapshot& snapshot)
{
  AvbdRigidWorldContactBuildScratch scratch(snapshot.contacts.get_allocator());
  return appendAvbdRigidWorldDistanceSprings(
      registry, inputs, snapshot, scratch);
}

//==============================================================================
inline void predictAvbdRigidWorldContactInertialTargets(
    const ::dart::simulation::detail::WorldRegistry& registry,
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
    AvbdScalarRowInventory& motorInventory,
    AvbdScalarRowInventory& distanceSpringInventory,
    double timeStep,
    AvbdRigidWorldContactSolveScratch& scratch,
    const AvbdRigidWorldContactSolveOptions& options = {})
{
  AvbdRigidWorldContactSolveResult result{
      AvbdRigidWorldContactSolveResult::SizeAllocator{
          scratch.normalRows.get_allocator()}};
  if (snapshot.states.empty()) {
    return result;
  }

  scratch.clear();
  auto& normalRows = scratch.normalRows;
  auto& frictionRows = scratch.frictionRows;
  buildAvbdRigidContactManifoldRows(
      std::span<const AvbdRigidBodyState>{
          snapshot.states.data(), snapshot.states.size()},
      std::span<const AvbdRigidContactManifoldPoint>{
          snapshot.contacts.data(), snapshot.contacts.size()},
      normalInventory,
      frictionInventory,
      normalRows,
      frictionRows,
      scratch.contactRows,
      options.warmStart);
  result.normalRows = normalRows.size();
  result.frictionRows = 2u * frictionRows.size();

  auto& jointLinearRows = scratch.jointLinearRows;
  auto& jointAngularRows = scratch.jointAngularRows;
  buildAvbdRigidPointJointConstraintRows(
      std::span<const AvbdRigidBodyState>{
          snapshot.states.data(), snapshot.states.size()},
      std::span<const AvbdRigidPointJoint>{
          snapshot.joints.data(), snapshot.joints.size()},
      jointLinearInventory,
      jointAngularInventory,
      jointLinearRows,
      jointAngularRows,
      scratch.jointLinearRowsScratch,
      scratch.jointAngularRowsScratch,
      options.warmStart);
  result.jointLinearRows = jointLinearRows.size();
  result.jointAngularRows = jointAngularRows.size();

  auto& linearMotorRows = scratch.linearMotorRows;
  auto& motorRows = scratch.motorRows;
  buildAvbdRigidMotorRows(
      std::span<const AvbdRigidBodyState>{
          snapshot.states.data(), snapshot.states.size()},
      std::span<const AvbdRigidLinearMotor>{
          snapshot.linearMotors.data(), snapshot.linearMotors.size()},
      std::span<const AvbdRigidAngularMotor>{
          snapshot.motors.data(), snapshot.motors.size()},
      motorInventory,
      linearMotorRows,
      motorRows,
      timeStep,
      scratch.motorRowsScratch,
      options.warmStart);
  result.motorRows = linearMotorRows.size() + motorRows.size();

  auto& distanceSpringRows = scratch.distanceSpringRows;
  buildAvbdRigidDistanceSpringRows(
      snapshot.states,
      snapshot.distanceSprings,
      distanceSpringInventory,
      distanceSpringRows,
      scratch.distanceSpringRowsScratch,
      options.warmStart);
  result.distanceSpringRows = distanceSpringRows.size();

  const std::size_t normalRowCount = normalRows.size();
  const std::size_t jointLinearRowCount = jointLinearRows.size();
  const std::size_t jointAngularRowCount = jointAngularRows.size();
  const std::size_t linearMotorRowCount = linearMotorRows.size();
  const std::size_t angularMotorRowCount = motorRows.size();
  const std::size_t distanceSpringRowCount = distanceSpringRows.size();

  auto& pointPairRows = scratch.pointPairRows;
  auto* solvePointPairRows = &pointPairRows;
  std::size_t normalOffset = 0u;
  std::size_t jointLinearOffset = normalRowCount;
  std::size_t linearMotorOffset = normalRowCount + jointLinearRowCount;
  const std::size_t pointPairFamilyCount
      = static_cast<std::size_t>(normalRowCount != 0u)
        + static_cast<std::size_t>(jointLinearRowCount != 0u)
        + static_cast<std::size_t>(linearMotorRowCount != 0u);
  if (pointPairFamilyCount == 1u) {
    if (normalRowCount != 0u) {
      solvePointPairRows = &normalRows;
      normalOffset = 0u;
    } else if (jointLinearRowCount != 0u) {
      solvePointPairRows = &jointLinearRows;
      jointLinearOffset = 0u;
    } else {
      solvePointPairRows = &linearMotorRows;
      linearMotorOffset = 0u;
    }
  } else {
    pointPairRows.clear();
    pointPairRows.reserve(
        normalRowCount + jointLinearRowCount + linearMotorRowCount);
    pointPairRows.insert(
        pointPairRows.end(), normalRows.begin(), normalRows.end());
    pointPairRows.insert(
        pointPairRows.end(), jointLinearRows.begin(), jointLinearRows.end());
    pointPairRows.insert(
        pointPairRows.end(), linearMotorRows.begin(), linearMotorRows.end());
  }

  auto& angularRows = scratch.angularRows;
  auto* solveAngularRows = &angularRows;
  std::size_t angularMotorOffset = jointAngularRowCount;
  const std::size_t angularFamilyCount
      = static_cast<std::size_t>(jointAngularRowCount != 0u)
        + static_cast<std::size_t>(angularMotorRowCount != 0u);
  if (angularFamilyCount == 1u) {
    if (jointAngularRowCount != 0u) {
      solveAngularRows = &jointAngularRows;
    } else {
      solveAngularRows = &motorRows;
      angularMotorOffset = 0u;
    }
  } else {
    angularRows.clear();
    angularRows.reserve(jointAngularRowCount + angularMotorRowCount);
    angularRows.insert(
        angularRows.end(), jointAngularRows.begin(), jointAngularRows.end());
    angularRows.insert(angularRows.end(), motorRows.begin(), motorRows.end());
  }

  auto& attachmentRows = scratch.attachmentRows;
  attachmentRows.clear();
  auto& fallbackInertialTargets = scratch.fallbackInertialTargets;
  std::span<const AvbdRigidBodyState> inertialTargets;
  if (snapshot.inertialTargets.size() == snapshot.states.size()) {
    inertialTargets = std::span<const AvbdRigidBodyState>{
        snapshot.inertialTargets.data(), snapshot.inertialTargets.size()};
  } else {
    fallbackInertialTargets.assign(
        snapshot.states.begin(), snapshot.states.end());
    inertialTargets = std::span<const AvbdRigidBodyState>{
        fallbackInertialTargets.data(), fallbackInertialTargets.size()};
  }
  result.stats = blockDescentRigidBodiesAvbdRows(
      std::span<AvbdRigidBodyState>{
          snapshot.states.data(), snapshot.states.size()},
      std::span<const double>{snapshot.masses.data(), snapshot.masses.size()},
      std::span<const Eigen::Matrix3d>{
          snapshot.bodyInertias.data(), snapshot.bodyInertias.size()},
      std::span<const std::uint8_t>{
          snapshot.fixed.data(), snapshot.fixed.size()},
      inertialTargets,
      timeStep,
      attachmentRows,
      *solvePointPairRows,
      *solveAngularRows,
      frictionRows,
      options.descent,
      options.row,
      options.friction,
      &scratch.rowIndexScratch,
      distanceSpringRows,
      options.distanceSpring);

  for (std::size_t i = 0; i < normalRowCount && i < normalInventory.size();
       ++i) {
    normalInventory[i].state
        = (*solvePointPairRows)[normalOffset + i].row.state;
  }
  for (std::size_t i = 0;
       i < jointLinearRowCount && i < jointLinearInventory.size();
       ++i) {
    jointLinearInventory[i].state
        = (*solvePointPairRows)[jointLinearOffset + i].row.state;
  }
  for (std::size_t i = 0;
       i < jointAngularRowCount && i < jointAngularInventory.size();
       ++i) {
    jointAngularInventory[i].state = (*solveAngularRows)[i].row.state;
  }
  std::size_t motorRecordIndex = 0;
  for (std::size_t i = 0;
       i < linearMotorRowCount && motorRecordIndex < motorInventory.size();
       ++i, ++motorRecordIndex) {
    motorInventory[motorRecordIndex].state
        = (*solvePointPairRows)[linearMotorOffset + i].row.state;
  }
  for (std::size_t i = 0;
       i < angularMotorRowCount && motorRecordIndex < motorInventory.size();
       ++i, ++motorRecordIndex) {
    motorInventory[motorRecordIndex].state
        = (*solveAngularRows)[angularMotorOffset + i].row.state;
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
  for (std::size_t i = 0;
       i < distanceSpringRowCount && i < distanceSpringInventory.size();
       ++i) {
    distanceSpringInventory[i].state = distanceSpringRows[i].row.state;
  }

  std::size_t linearCursor = 0;
  std::size_t angularCursor = 0;
  for (std::size_t jointIndex = 0; jointIndex < snapshot.joints.size();
       ++jointIndex) {
    const AvbdRigidPointJoint& joint = snapshot.joints[jointIndex];
    if (!detail::isValidAvbdRigidPointJoint(joint, snapshot.states.size())) {
      continue;
    }

    const std::size_t linearRows
        = avbdRigidWorldActiveJointAxisCount(joint.linearAxisMask);
    const std::size_t angularRowsForJoint
        = avbdRigidWorldActiveJointAxisCount(joint.angularAxisMask);
    double lambdaSquared = 0.0;
    for (std::size_t i = 0;
         i < linearRows && linearCursor + i < jointLinearRowCount;
         ++i) {
      lambdaSquared += avbdRigidWorldJointRowLambdaSquared(
          (*solvePointPairRows)[jointLinearOffset + linearCursor + i]
              .row.state.lambda);
    }
    for (std::size_t i = 0;
         i < angularRowsForJoint && angularCursor + i < jointAngularRowCount;
         ++i) {
      lambdaSquared += avbdRigidWorldJointRowLambdaSquared(
          (*solveAngularRows)[angularCursor + i].row.state.lambda);
    }
    if (joint.fractureThreshold > 0.0 && std::isfinite(joint.fractureThreshold)
        && std::sqrt(lambdaSquared) >= joint.fractureThreshold) {
      result.fracturedJointIndices.push_back(jointIndex);
    }

    linearCursor += linearRows;
    angularCursor += angularRowsForJoint;
  }
  result.fracturedJoints = result.fracturedJointIndices.size();

  return result;
}

//==============================================================================
inline AvbdRigidWorldContactSolveResult solveAvbdRigidWorldContactSnapshot(
    AvbdRigidWorldContactSnapshot& snapshot,
    AvbdScalarRowInventory& normalInventory,
    AvbdScalarRowInventory& frictionInventory,
    AvbdScalarRowInventory& jointLinearInventory,
    AvbdScalarRowInventory& jointAngularInventory,
    AvbdScalarRowInventory& motorInventory,
    AvbdScalarRowInventory& distanceSpringInventory,
    AvbdRigidWorldContactSolveScratch& scratch,
    double timeStep,
    const AvbdRigidWorldContactSolveOptions& options = {})
{
  return solveAvbdRigidWorldContactSnapshot(
      snapshot,
      normalInventory,
      frictionInventory,
      jointLinearInventory,
      jointAngularInventory,
      motorInventory,
      distanceSpringInventory,
      timeStep,
      scratch,
      options);
}

//==============================================================================
inline AvbdRigidWorldContactSolveResult solveAvbdRigidWorldContactSnapshot(
    AvbdRigidWorldContactSnapshot& snapshot,
    AvbdScalarRowInventory& normalInventory,
    AvbdScalarRowInventory& frictionInventory,
    AvbdScalarRowInventory& jointLinearInventory,
    AvbdScalarRowInventory& jointAngularInventory,
    AvbdScalarRowInventory& motorInventory,
    double timeStep,
    AvbdRigidWorldContactSolveScratch& scratch,
    const AvbdRigidWorldContactSolveOptions& options = {})
{
  AvbdScalarRowInventory distanceSpringInventory(
      normalInventory.records().get_allocator());
  return solveAvbdRigidWorldContactSnapshot(
      snapshot,
      normalInventory,
      frictionInventory,
      jointLinearInventory,
      jointAngularInventory,
      motorInventory,
      distanceSpringInventory,
      timeStep,
      scratch,
      options);
}

//==============================================================================
inline AvbdRigidWorldContactSolveResult solveAvbdRigidWorldContactSnapshot(
    AvbdRigidWorldContactSnapshot& snapshot,
    AvbdScalarRowInventory& normalInventory,
    AvbdScalarRowInventory& frictionInventory,
    AvbdScalarRowInventory& jointLinearInventory,
    AvbdScalarRowInventory& jointAngularInventory,
    AvbdScalarRowInventory& motorInventory,
    double timeStep,
    const AvbdRigidWorldContactSolveOptions& options = {})
{
  const auto allocator = normalInventory.records().get_allocator();
  AvbdRigidWorldContactSolveScratch scratch(allocator);
  AvbdScalarRowInventory distanceSpringInventory(allocator);
  return solveAvbdRigidWorldContactSnapshot(
      snapshot,
      normalInventory,
      frictionInventory,
      jointLinearInventory,
      jointAngularInventory,
      motorInventory,
      distanceSpringInventory,
      timeStep,
      scratch,
      options);
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
  AvbdScalarRowInventory motorInventory(
      normalInventory.records().get_allocator());
  return solveAvbdRigidWorldContactSnapshot(
      snapshot,
      normalInventory,
      frictionInventory,
      jointLinearInventory,
      jointAngularInventory,
      motorInventory,
      timeStep,
      options);
}

//==============================================================================
inline AvbdRigidWorldContactSolveResult solveAvbdRigidWorldContactSnapshot(
    AvbdRigidWorldContactSnapshot& snapshot,
    AvbdScalarRowInventory& normalInventory,
    AvbdScalarRowInventory& frictionInventory,
    double timeStep,
    const AvbdRigidWorldContactSolveOptions& options = {})
{
  const auto allocator = normalInventory.records().get_allocator();
  AvbdScalarRowInventory jointLinearInventory(allocator);
  AvbdScalarRowInventory jointAngularInventory(allocator);
  AvbdScalarRowInventory motorInventory(allocator);
  return solveAvbdRigidWorldContactSnapshot(
      snapshot,
      normalInventory,
      frictionInventory,
      jointLinearInventory,
      jointAngularInventory,
      motorInventory,
      timeStep,
      options);
}

//==============================================================================
inline AvbdRigidWorldContactApplyResult
applyAvbdRigidWorldContactVelocityProjection(
    ::dart::simulation::detail::WorldRegistry& registry,
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
inline std::size_t markAvbdRigidWorldFracturedPointJoints(
    ::dart::simulation::detail::WorldRegistry& registry,
    const AvbdRigidWorldContactSnapshot& snapshot,
    std::span<const std::size_t> fracturedJointIndices)
{
  std::size_t marked = 0;
  for (const std::size_t jointIndex : fracturedJointIndices) {
    if (jointIndex >= snapshot.jointEntities.size()) {
      continue;
    }

    const entt::entity jointEntity = snapshot.jointEntities[jointIndex];
    if (jointEntity == entt::null
        || !registry.all_of<comps::Joint>(jointEntity)) {
      continue;
    }

    auto& joint = registry.get<comps::Joint>(jointEntity);
    if (!joint.broken) {
      joint.broken = true;
      ++marked;
    }
  }
  return marked;
}

//==============================================================================
inline AvbdRigidWorldContactApplyResult
applyAvbdRigidWorldContactSnapshotWithStack(
    ::dart::simulation::detail::WorldRegistry& registry,
    const AvbdRigidWorldContactSnapshot& snapshot,
    AvbdRigidWorldContactSnapshot::EntityVector& frameDirtyStack,
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

    avbdRigidWorldContactMarkFrameSubtreeDirty(
        registry, entity, frameDirtyStack);
    ++result.bodies;
  }

  return result;
}

//==============================================================================
inline AvbdRigidWorldContactApplyResult applyAvbdRigidWorldContactSnapshot(
    ::dart::simulation::detail::WorldRegistry& registry,
    AvbdRigidWorldContactSnapshot& snapshot,
    double timeStep)
{
  return applyAvbdRigidWorldContactSnapshotWithStack(
      registry, snapshot, snapshot.frameDirtyStack, timeStep);
}

//==============================================================================
inline AvbdRigidWorldContactApplyResult applyAvbdRigidWorldContactSnapshot(
    ::dart::simulation::detail::WorldRegistry& registry,
    const AvbdRigidWorldContactSnapshot& snapshot,
    double timeStep)
{
  AvbdRigidWorldContactSnapshot::EntityVector frameDirtyStack(
      snapshot.entities.get_allocator());
  return applyAvbdRigidWorldContactSnapshotWithStack(
      registry, snapshot, frameDirtyStack, timeStep);
}

//==============================================================================
inline AvbdRigidWorldContactStepResult runAvbdRigidWorldContactStep(
    ::dart::simulation::detail::WorldRegistry& registry,
    std::span<const Contact> contacts,
    std::span<const AvbdRigidWorldPointJointInput> joints,
    std::span<const AvbdRigidWorldDistanceSpringInput> distanceSprings,
    AvbdScalarRowInventory& normalInventory,
    AvbdScalarRowInventory& frictionInventory,
    AvbdScalarRowInventory& jointLinearInventory,
    AvbdScalarRowInventory& jointAngularInventory,
    AvbdScalarRowInventory& motorInventory,
    AvbdScalarRowInventory& distanceSpringInventory,
    double timeStep,
    const AvbdRigidWorldContactStepOptions& options = {})
{
  AvbdRigidWorldContactStepResult result;
  const auto allocator = normalInventory.records().get_allocator();
  AvbdRigidWorldContactSnapshot snapshot(allocator);
  AvbdRigidWorldContactBuildScratch buildScratch(
      snapshot.contacts.get_allocator());
  buildAvbdRigidWorldContactSnapshot(
      registry, contacts, snapshot, buildScratch, options.contact);
  result.joints = appendAvbdRigidWorldPointJoints(registry, joints, snapshot);
  result.distanceSprings = appendAvbdRigidWorldDistanceSprings(
      registry, distanceSprings, snapshot);
  result.motors = snapshot.linearMotors.size() + snapshot.motors.size();
  result.bodies = snapshot.states.size();
  result.contacts = snapshot.contacts.size();
  if (snapshot.states.empty()
      || (snapshot.contacts.empty() && snapshot.joints.empty()
          && snapshot.linearMotors.empty() && snapshot.motors.empty()
          && snapshot.distanceSprings.empty())) {
    return result;
  }
  if (options.useVelocityInertialTargets) {
    predictAvbdRigidWorldContactInertialTargets(registry, snapshot, timeStep);
  }

  AvbdRigidWorldContactSolveScratch scratch(allocator);
  result.solve = solveAvbdRigidWorldContactSnapshot(
      snapshot,
      normalInventory,
      frictionInventory,
      jointLinearInventory,
      jointAngularInventory,
      motorInventory,
      distanceSpringInventory,
      timeStep,
      scratch,
      options.solve);
  result.fracturedJoints = markAvbdRigidWorldFracturedPointJoints(
      registry, snapshot, result.solve.fracturedJointIndices);
  if (result.solve.normalRows == 0u && result.solve.frictionRows == 0u
      && result.solve.jointLinearRows == 0u
      && result.solve.jointAngularRows == 0u && result.solve.motorRows == 0u
      && result.solve.distanceSpringRows == 0u) {
    return result;
  }

  result.apply
      = applyAvbdRigidWorldContactSnapshot(registry, snapshot, timeStep);
  return result;
}

//==============================================================================
inline AvbdRigidWorldContactStepResult runAvbdRigidWorldContactStep(
    ::dart::simulation::detail::WorldRegistry& registry,
    std::span<const Contact> contacts,
    std::span<const AvbdRigidWorldPointJointInput> joints,
    AvbdScalarRowInventory& normalInventory,
    AvbdScalarRowInventory& frictionInventory,
    AvbdScalarRowInventory& jointLinearInventory,
    AvbdScalarRowInventory& jointAngularInventory,
    AvbdScalarRowInventory& motorInventory,
    double timeStep,
    const AvbdRigidWorldContactStepOptions& options = {})
{
  AvbdScalarRowInventory distanceSpringInventory(
      normalInventory.records().get_allocator());
  return runAvbdRigidWorldContactStep(
      registry,
      contacts,
      joints,
      std::span<const AvbdRigidWorldDistanceSpringInput>{},
      normalInventory,
      frictionInventory,
      jointLinearInventory,
      jointAngularInventory,
      motorInventory,
      distanceSpringInventory,
      timeStep,
      options);
}

//==============================================================================
inline AvbdRigidWorldContactStepResult runAvbdRigidWorldContactStep(
    ::dart::simulation::detail::WorldRegistry& registry,
    std::span<const Contact> contacts,
    std::span<const AvbdRigidWorldPointJointInput> joints,
    AvbdScalarRowInventory& normalInventory,
    AvbdScalarRowInventory& frictionInventory,
    AvbdScalarRowInventory& jointLinearInventory,
    AvbdScalarRowInventory& jointAngularInventory,
    double timeStep,
    const AvbdRigidWorldContactStepOptions& options = {})
{
  AvbdScalarRowInventory motorInventory(
      normalInventory.records().get_allocator());
  return runAvbdRigidWorldContactStep(
      registry,
      contacts,
      joints,
      normalInventory,
      frictionInventory,
      jointLinearInventory,
      jointAngularInventory,
      motorInventory,
      timeStep,
      options);
}

//==============================================================================
template <typename InputVector>
inline void extractAvbdRigidWorldPointJointInputsInto(
    const ::dart::simulation::detail::WorldRegistry& registry,
    InputVector& inputs,
    bool includeWorldAnchors = true)
{
  const auto view
      = registry.view<comps::Joint, AvbdRigidWorldPointJointConfig>();
  inputs.clear();
  inputs.reserve(view.size_hint());

  for (const entt::entity entity : view) {
    const auto& joint = view.get<comps::Joint>(entity);
    const auto& config = view.get<AvbdRigidWorldPointJointConfig>(entity);
    if (!config.enabled || joint.broken
        || !isAvbdRigidWorldPointJointType(joint.type)) {
      continue;
    }

    if (joint.parentLink == entt::null || joint.childLink == entt::null
        || joint.parentLink == joint.childLink) {
      continue;
    }

    const AvbdRigidWorldProjectableBodyView bodyA
        = avbdRigidWorldProjectableBody(registry, joint.parentLink);
    const AvbdRigidWorldProjectableBodyView bodyB
        = avbdRigidWorldProjectableBody(registry, joint.childLink);
    if (!bodyA || !bodyB || (bodyA.isStatic && bodyB.isStatic)) {
      continue;
    }
    const auto* transformA = bodyA.transform;
    const auto* transformB = bodyB.transform;

    if (!config.localAnchorA.allFinite() || !config.localAnchorB.allFinite()
        || !config.targetRelativeOrientation.coeffs().allFinite()
        || !config.linearAxes.allFinite() || !config.angularAxes.allFinite()
        || std::isnan(config.startStiffness) || config.startStiffness < 0.0
        || std::isnan(config.linearMaterialStiffness)
        || config.linearMaterialStiffness < 0.0
        || std::isnan(config.angularMaterialStiffness)
        || config.angularMaterialStiffness < 0.0
        || std::isnan(config.maxStiffness)
        || config.maxStiffness < config.startStiffness) {
      continue;
    }

    if (!transformA->position.allFinite()
        || !transformA->orientation.coeffs().allFinite()
        || !transformB->position.allFinite()
        || !transformB->orientation.coeffs().allFinite()) {
      continue;
    }

    const bool hasAngularVelocityMotor
        = joint.type == comps::JointType::Revolute
          && joint.actuatorType == comps::ActuatorType::Velocity
          && joint.commandVelocity.size() == 1
          && joint.commandVelocity.allFinite();
    const bool hasLinearVelocityMotor
        = joint.type == comps::JointType::Prismatic
          && joint.actuatorType == comps::ActuatorType::Velocity
          && joint.commandVelocity.size() == 1
          && joint.commandVelocity.allFinite();
    const bool useStableFullLinearBasis
        = !includeWorldAnchors
          && config.linearAxisMask == kAvbdRigidJointAllAxesMask
          && config.angularAxisMask == 0u && !hasAngularVelocityMotor
          && !hasLinearVelocityMotor
          && config.linearAxes.isApprox(Eigen::Matrix3d::Identity(), 0.0);
    Eigen::Quaterniond orientationA = Eigen::Quaterniond::Identity();
    Eigen::Matrix3d parentRotation = Eigen::Matrix3d::Identity();
    if (!useStableFullLinearBasis || includeWorldAnchors) {
      orientationA = normalizeAvbdRigidOrientation(transformA->orientation);
      parentRotation = orientationA.toRotationMatrix();
    }

    AvbdRigidWorldPointJointInput input;
    input.joint = entity;
    input.bodyA = joint.parentLink;
    input.bodyB = joint.childLink;
    input.bodyAView = bodyA;
    input.bodyBView = bodyB;
    if (includeWorldAnchors) {
      const Eigen::Quaterniond orientationB
          = normalizeAvbdRigidOrientation(transformB->orientation);
      input.anchorA = transformA->position + orientationA * config.localAnchorA;
      input.anchorB = transformB->position + orientationB * config.localAnchorB;
    } else {
      input.anchorA = config.localAnchorA;
      input.anchorB = config.localAnchorB;
      input.anchorsAreLocal = true;
    }
    input.targetRelativeOrientation
        = normalizeAvbdRigidOrientation(config.targetRelativeOrientation);
    input.linearAxes = useStableFullLinearBasis
                           ? config.linearAxes
                           : parentRotation * config.linearAxes;
    input.angularAxes = useStableFullLinearBasis
                            ? config.angularAxes
                            : parentRotation * config.angularAxes;
    input.linearAxisMask = config.linearAxisMask;
    input.angularAxisMask = config.angularAxisMask;
    if (hasAngularVelocityMotor) {
      const double maxTorque = avbdRigidWorldSymmetricEffortLimit(joint);
      if (maxTorque > 0.0 && !std::isnan(maxTorque)) {
        input.useAngularMotor = true;
        input.motorTargetSpeed = joint.commandVelocity[0];
        input.motorMaxTorque = maxTorque;
      }
    }
    if (hasLinearVelocityMotor) {
      const double maxForce = avbdRigidWorldSymmetricEffortLimit(joint);
      if (maxForce > 0.0 && !std::isnan(maxForce)) {
        input.useLinearMotor = true;
        input.motorTargetSpeed = joint.commandVelocity[0];
        input.motorMaxForce = maxForce;
      }
    }
    input.startStiffness = config.startStiffness;
    input.linearMaterialStiffness = config.linearMaterialStiffness;
    input.angularMaterialStiffness = config.angularMaterialStiffness;
    input.maxStiffness = config.maxStiffness;
    input.fractureThreshold = joint.breakForce;
    inputs.push_back(input);
  }
}

//==============================================================================
inline void extractAvbdRigidWorldPointJointInputs(
    const ::dart::simulation::detail::WorldRegistry& registry,
    std::vector<AvbdRigidWorldPointJointInput>& inputs)
{
  extractAvbdRigidWorldPointJointInputsInto(registry, inputs);
}

//==============================================================================
inline bool hasAvbdRigidWorldPointJointConfigs(
    const ::dart::simulation::detail::WorldRegistry& registry)
{
  const auto view
      = registry.view<comps::Joint, AvbdRigidWorldPointJointConfig>();
  return view.begin() != view.end();
}

//==============================================================================
template <typename InputVector>
inline void extractAvbdRigidWorldDistanceSpringInputsInto(
    const ::dart::simulation::detail::WorldRegistry& registry,
    InputVector& inputs,
    bool includeWorldAnchors = true)
{
  inputs.clear();
  const auto view = registry.view<AvbdRigidWorldDistanceSpringConfig>();

  for (const entt::entity entity : view) {
    const auto& config = view.get<AvbdRigidWorldDistanceSpringConfig>(entity);
    if (!config.enabled || config.bodyA == entt::null
        || config.bodyB == entt::null || config.bodyA == config.bodyB) {
      continue;
    }

    const AvbdRigidWorldProjectableBodyView bodyA
        = avbdRigidWorldProjectableBody(registry, config.bodyA);
    const AvbdRigidWorldProjectableBodyView bodyB
        = avbdRigidWorldProjectableBody(registry, config.bodyB);
    if (!bodyA || !bodyB || (bodyA.isStatic && bodyB.isStatic)) {
      continue;
    }
    const auto* transformA = bodyA.transform;
    const auto* transformB = bodyB.transform;

    if (!config.localAnchorA.allFinite() || !config.localAnchorB.allFinite()
        || !std::isfinite(config.restLength) || config.restLength < 0.0
        || std::isnan(config.startStiffness) || config.startStiffness < 0.0
        || std::isnan(config.materialStiffness)
        || config.materialStiffness < 0.0 || std::isnan(config.maxStiffness)
        || config.maxStiffness < config.startStiffness) {
      continue;
    }

    if (!transformA->position.allFinite()
        || !transformA->orientation.coeffs().allFinite()
        || !transformB->position.allFinite()
        || !transformB->orientation.coeffs().allFinite()) {
      continue;
    }

    AvbdRigidWorldDistanceSpringInput input;
    input.spring = entity;
    input.bodyA = config.bodyA;
    input.bodyB = config.bodyB;
    input.bodyAView = bodyA;
    input.bodyBView = bodyB;
    if (includeWorldAnchors) {
      const Eigen::Quaterniond orientationA
          = normalizeAvbdRigidOrientation(transformA->orientation);
      const Eigen::Quaterniond orientationB
          = normalizeAvbdRigidOrientation(transformB->orientation);
      input.anchorA = transformA->position + orientationA * config.localAnchorA;
      input.anchorB = transformB->position + orientationB * config.localAnchorB;
    } else {
      input.anchorA = config.localAnchorA;
      input.anchorB = config.localAnchorB;
      input.anchorsAreLocal = true;
    }
    input.restLength = config.restLength;
    input.startStiffness = config.startStiffness;
    input.materialStiffness = config.materialStiffness;
    input.maxStiffness = config.maxStiffness;
    inputs.push_back(input);
  }
}

//==============================================================================
inline bool hasAvbdRigidWorldDistanceSpringConfigs(
    const ::dart::simulation::detail::WorldRegistry& registry)
{
  const auto view = registry.view<AvbdRigidWorldDistanceSpringConfig>();
  return view.begin() != view.end();
}

//==============================================================================
inline bool hasAvbdRigidWorldPairConstraintConfigs(
    const ::dart::simulation::detail::WorldRegistry& registry)
{
  return hasAvbdRigidWorldPointJointConfigs(registry)
         || hasAvbdRigidWorldDistanceSpringConfigs(registry);
}

//==============================================================================
inline std::vector<AvbdRigidWorldPointJointInput>
extractAvbdRigidWorldPointJointInputs(
    const ::dart::simulation::detail::WorldRegistry& registry)
{
  std::vector<AvbdRigidWorldPointJointInput> inputs;
  extractAvbdRigidWorldPointJointInputsInto(registry, inputs);
  return inputs;
}

//==============================================================================
inline std::vector<AvbdRigidWorldDistanceSpringInput>
extractAvbdRigidWorldDistanceSpringInputs(
    const ::dart::simulation::detail::WorldRegistry& registry)
{
  std::vector<AvbdRigidWorldDistanceSpringInput> inputs;
  extractAvbdRigidWorldDistanceSpringInputsInto(registry, inputs);
  return inputs;
}

//==============================================================================
inline AvbdRigidWorldContactStepResult runAvbdRigidWorldContactStep(
    ::dart::simulation::detail::WorldRegistry& registry,
    std::span<const Contact> contacts,
    AvbdScalarRowInventory& normalInventory,
    AvbdScalarRowInventory& frictionInventory,
    AvbdScalarRowInventory& jointLinearInventory,
    AvbdScalarRowInventory& jointAngularInventory,
    AvbdScalarRowInventory& motorInventory,
    double timeStep,
    const AvbdRigidWorldContactStepOptions& options = {})
{
  using PointJointInputAllocator
      = ::dart::common::StlAllocator<AvbdRigidWorldPointJointInput>;
  using DistanceSpringInputAllocator
      = ::dart::common::StlAllocator<AvbdRigidWorldDistanceSpringInput>;
  using PointJointInputVector
      = std::vector<AvbdRigidWorldPointJointInput, PointJointInputAllocator>;
  using DistanceSpringInputVector = std::
      vector<AvbdRigidWorldDistanceSpringInput, DistanceSpringInputAllocator>;

  const auto allocator = normalInventory.records().get_allocator();
  PointJointInputVector joints(PointJointInputAllocator{allocator});
  extractAvbdRigidWorldPointJointInputsInto(
      registry, joints, /*includeWorldAnchors=*/false);
  DistanceSpringInputVector distanceSprings(
      DistanceSpringInputAllocator{allocator});
  extractAvbdRigidWorldDistanceSpringInputsInto(
      registry, distanceSprings, /*includeWorldAnchors=*/false);
  AvbdScalarRowInventory distanceSpringInventory(allocator);
  return runAvbdRigidWorldContactStep(
      registry,
      contacts,
      std::span<const AvbdRigidWorldPointJointInput>(
          joints.data(), joints.size()),
      std::span<const AvbdRigidWorldDistanceSpringInput>(
          distanceSprings.data(), distanceSprings.size()),
      normalInventory,
      frictionInventory,
      jointLinearInventory,
      jointAngularInventory,
      motorInventory,
      distanceSpringInventory,
      timeStep,
      options);
}

//==============================================================================
inline AvbdRigidWorldContactStepResult runAvbdRigidWorldContactStep(
    ::dart::simulation::detail::WorldRegistry& registry,
    std::span<const Contact> contacts,
    AvbdScalarRowInventory& normalInventory,
    AvbdScalarRowInventory& frictionInventory,
    AvbdScalarRowInventory& jointLinearInventory,
    AvbdScalarRowInventory& jointAngularInventory,
    double timeStep,
    const AvbdRigidWorldContactStepOptions& options = {})
{
  AvbdScalarRowInventory motorInventory(
      normalInventory.records().get_allocator());
  return runAvbdRigidWorldContactStep(
      registry,
      contacts,
      normalInventory,
      frictionInventory,
      jointLinearInventory,
      jointAngularInventory,
      motorInventory,
      timeStep,
      options);
}

//==============================================================================
inline AvbdRigidWorldContactStepResult runAvbdRigidWorldContactStep(
    ::dart::simulation::detail::WorldRegistry& registry,
    std::span<const Contact> contacts,
    AvbdScalarRowInventory& normalInventory,
    AvbdScalarRowInventory& frictionInventory,
    double timeStep,
    const AvbdRigidWorldContactStepOptions& options = {})
{
  const auto allocator = normalInventory.records().get_allocator();
  AvbdScalarRowInventory jointLinearInventory(allocator);
  AvbdScalarRowInventory jointAngularInventory(allocator);
  AvbdScalarRowInventory motorInventory(allocator);
  return runAvbdRigidWorldContactStep(
      registry,
      contacts,
      normalInventory,
      frictionInventory,
      jointLinearInventory,
      jointAngularInventory,
      motorInventory,
      timeStep,
      options);
}

} // namespace dart::simulation::detail::deformable_vbd
