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

#pragma once

#include <dart/simulation/comps/dynamics.hpp>
#include <dart/simulation/comps/frame_types.hpp>
#include <dart/simulation/comps/joint.hpp>
#include <dart/simulation/comps/rigid_body.hpp>
#include <dart/simulation/detail/world_registry_types.hpp>

#include <Eigen/Dense>
#include <entt/entt.hpp>

#include <algorithm>
#include <limits>
#include <optional>
#include <type_traits>
#include <utility>

#include <cmath>
#include <cstdint>

namespace dart::simulation::detail {

inline constexpr std::uint8_t kRigidPairConstraintAllAxesMask = 0x7u;

/// Solver-neutral geometric description of a public rigid-body pair
/// constraint.
///
/// Material stiffness and persistent multipliers intentionally do not live
/// here. Solver families enrich this geometry with their own policy/state.
struct RigidPairConstraintGeometry
{
  bool enabled = true;
  Eigen::Vector3d localAnchorA = Eigen::Vector3d::Zero();
  Eigen::Vector3d localAnchorB = Eigen::Vector3d::Zero();
  Eigen::Quaterniond targetRelativeOrientation = Eigen::Quaterniond::Identity();
  Eigen::Matrix3d linearAxes = Eigen::Matrix3d::Identity();
  Eigen::Matrix3d angularAxes = Eigen::Matrix3d::Identity();
  std::uint8_t linearAxisMask = kRigidPairConstraintAllAxesMask;
  std::uint8_t angularAxisMask = kRigidPairConstraintAllAxesMask;
};

/// Read-only endpoint data shared by rigid pair-constraint solver families.
struct RigidPairConstraintBodyView
{
  const comps::Transform* transform = nullptr;
  const comps::MassProperties* mass = nullptr;
  bool isStatic = false;

  explicit operator bool() const noexcept
  {
    return transform != nullptr && mass != nullptr;
  }
};

/// Solver-neutral runtime input for a public rigid-body pair constraint.
///
/// The input includes public geometry, actuation, and fracture policy. AVBD
/// stiffness and warm-start state are deliberately owned by AVBD adapters.
struct RigidPairConstraintInput
{
  entt::entity joint = entt::null;
  entt::entity bodyA = entt::null;
  entt::entity bodyB = entt::null;
  RigidPairConstraintBodyView bodyAView;
  RigidPairConstraintBodyView bodyBView;
  Eigen::Vector3d anchorA = Eigen::Vector3d::Zero();
  Eigen::Vector3d anchorB = Eigen::Vector3d::Zero();
  bool anchorsAreLocal = false;
  Eigen::Quaterniond targetRelativeOrientation = Eigen::Quaterniond::Identity();
  Eigen::Matrix3d linearAxes = Eigen::Matrix3d::Identity();
  Eigen::Matrix3d angularAxes = Eigen::Matrix3d::Identity();
  std::uint8_t linearAxisMask = kRigidPairConstraintAllAxesMask;
  std::uint8_t angularAxisMask = kRigidPairConstraintAllAxesMask;
  bool useLinearMotor = false;
  bool useAngularMotor = false;
  double motorTargetSpeed = 0.0;
  double motorMaxForce = std::numeric_limits<double>::infinity();
  double motorMaxTorque = std::numeric_limits<double>::infinity();
  double fractureThreshold = 0.0;
};

//==============================================================================
inline Eigen::Quaterniond normalizeRigidPairConstraintOrientation(
    const Eigen::Quaterniond& orientation)
{
  const double squaredNorm = orientation.squaredNorm();
  if (!std::isfinite(squaredNorm) || squaredNorm <= 0.0) {
    return Eigen::Quaterniond::Identity();
  }
  if (squaredNorm == 1.0) {
    return orientation;
  }

  Eigen::Quaterniond normalized = orientation;
  normalized.coeffs() /= std::sqrt(squaredNorm);
  return normalized;
}

inline constexpr double kRigidPairConstraintPi
    = 3.141592653589793238462643383279502884;

//==============================================================================
inline Eigen::Vector3d rigidPairConstraintRotationVectorFromNormalized(
    const Eigen::Quaterniond& q)
{
  const Eigen::Vector3d vector = q.vec();
  const double vectorNorm = vector.norm();
  if (vectorNorm <= 0.0 || !std::isfinite(vectorNorm)) {
    return Eigen::Vector3d::Zero();
  }

  double angle = 2.0 * std::atan2(vectorNorm, q.w());
  if (angle > kRigidPairConstraintPi) {
    angle -= 2.0 * kRigidPairConstraintPi;
  } else if (angle < -kRigidPairConstraintPi) {
    angle += 2.0 * kRigidPairConstraintPi;
  }
  return (angle / vectorNorm) * vector;
}

//==============================================================================
/// Shortest-arc rotation vector taking `targetOrientation` to `orientation`.
inline Eigen::Vector3d rigidPairConstraintOrientationError(
    const Eigen::Quaterniond& orientation,
    const Eigen::Quaterniond& targetOrientation)
{
  const Eigen::Quaterniond current
      = normalizeRigidPairConstraintOrientation(orientation);
  const Eigen::Quaterniond target
      = normalizeRigidPairConstraintOrientation(targetOrientation);
  return rigidPairConstraintRotationVectorFromNormalized(
      current * target.conjugate());
}

//==============================================================================
inline bool isRigidPairConstraintType(comps::JointType type)
{
  return type == comps::JointType::Fixed || type == comps::JointType::Revolute
         || type == comps::JointType::Prismatic
         || type == comps::JointType::Spherical;
}

//==============================================================================
inline std::uint8_t rigidPairConstraintAxisBit(std::uint8_t axis)
{
  return axis < 3u ? static_cast<std::uint8_t>(1u << axis) : 0u;
}

//==============================================================================
inline std::uint8_t rigidPairConstraintAllButAxisMask(std::uint8_t freeAxis)
{
  return static_cast<std::uint8_t>(
      kRigidPairConstraintAllAxesMask & ~rigidPairConstraintAxisBit(freeAxis));
}

//==============================================================================
inline Eigen::Matrix3d rigidPairConstraintAxesFromFreeAxis(
    const Eigen::Vector3d& freeAxis,
    const Eigen::Vector3d& fallback = Eigen::Vector3d::UnitZ())
{
  Eigen::Vector3d axis = freeAxis;
  const double axisNorm = axis.norm();
  if (!axis.allFinite() || axisNorm <= 0.0) {
    axis = fallback;
  } else {
    axis /= axisNorm;
  }

  const Eigen::Vector3d first = axis.unitOrthogonal();
  Eigen::Vector3d second = axis.cross(first);
  const double secondNorm = second.norm();
  if (!second.allFinite() || secondNorm <= 0.0) {
    second = axis.cross(Eigen::Vector3d::UnitX());
    if (second.squaredNorm() <= 0.0) {
      second = axis.cross(Eigen::Vector3d::UnitY());
    }
    second.normalize();
  } else {
    second /= secondNorm;
  }

  Eigen::Matrix3d axes;
  axes.col(0) = first;
  axes.col(1) = second;
  axes.col(2) = axis;
  return axes;
}

//==============================================================================
inline RigidPairConstraintBodyView rigidPairConstraintBody(
    const WorldRegistry& registry, entt::entity entity)
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

  return RigidPairConstraintBodyView{
      transform,
      mass,
      registry.all_of<comps::StaticBodyTag>(entity)
          || registry.all_of<comps::KinematicBodyTag>(entity)};
}

//==============================================================================
inline double rigidPairConstraintSymmetricEffortLimit(
    const comps::JointModel& joint)
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
inline std::optional<RigidPairConstraintGeometry>
rigidPairConstraintGeometryFromJointModel(const comps::JointModel& joint)
{
  if (!joint.hasRigidBodyPairConstraintGeometry
      || !isRigidPairConstraintType(joint.type)) {
    return std::nullopt;
  }

  RigidPairConstraintGeometry geometry;
  geometry.localAnchorA = joint.rigidBodyPairConstraintLocalAnchorParent;
  geometry.localAnchorB = joint.rigidBodyPairConstraintLocalAnchorChild;
  geometry.targetRelativeOrientation
      = joint.rigidBodyPairConstraintTargetRelativeOrientation;

  if (joint.type == comps::JointType::Spherical) {
    geometry.angularAxisMask = 0u;
  } else if (
      joint.type == comps::JointType::Revolute
      || joint.type == comps::JointType::Prismatic) {
    if (!joint.axis.allFinite() || joint.axis.squaredNorm() <= 0.0) {
      return std::nullopt;
    }
    const Eigen::Matrix3d axes
        = rigidPairConstraintAxesFromFreeAxis(joint.axis);
    if (joint.type == comps::JointType::Revolute) {
      geometry.angularAxes = axes;
      geometry.angularAxisMask
          = rigidPairConstraintAllButAxisMask(/*freeAxis=*/2u);
    } else {
      geometry.linearAxes = axes;
      geometry.angularAxes = axes;
      geometry.linearAxisMask
          = rigidPairConstraintAllButAxisMask(/*freeAxis=*/2u);
    }
  }
  return geometry;
}

/// Per-entity eligibility decision shared by row extraction and expected-row
/// counting so both surfaces always skip exactly the same joints, including
/// degenerate or non-finite state.
struct RigidPairConstraintEligibility
{
  RigidPairConstraintGeometry geometry;
  RigidPairConstraintBodyView bodyA;
  RigidPairConstraintBodyView bodyB;
};

template <typename GeometryProvider>
inline std::optional<RigidPairConstraintEligibility>
eligibleRigidPairConstraint(
    const WorldRegistry& registry,
    entt::entity entity,
    const comps::JointModel& jointModel,
    const comps::JointState& jointState,
    GeometryProvider&& geometryProvider)
{
  if (jointState.broken || !isRigidPairConstraintType(jointModel.type)) {
    return std::nullopt;
  }

  std::optional<RigidPairConstraintGeometry> geometryResult
      = geometryProvider(entity, jointModel);
  if (!geometryResult.has_value() || !geometryResult->enabled) {
    return std::nullopt;
  }

  if (jointModel.parentLink == entt::null || jointModel.childLink == entt::null
      || jointModel.parentLink == jointModel.childLink) {
    return std::nullopt;
  }

  const RigidPairConstraintBodyView bodyA
      = rigidPairConstraintBody(registry, jointModel.parentLink);
  const RigidPairConstraintBodyView bodyB
      = rigidPairConstraintBody(registry, jointModel.childLink);
  if (!bodyA || !bodyB || (bodyA.isStatic && bodyB.isStatic)) {
    return std::nullopt;
  }

  const RigidPairConstraintGeometry& geometry = *geometryResult;
  if (!geometry.localAnchorA.allFinite() || !geometry.localAnchorB.allFinite()
      || !geometry.targetRelativeOrientation.coeffs().allFinite()
      || geometry.targetRelativeOrientation.squaredNorm() <= 0.0
      || !geometry.linearAxes.allFinite()
      || !geometry.angularAxes.allFinite()) {
    return std::nullopt;
  }
  if (!bodyA.transform->position.allFinite()
      || !bodyA.transform->orientation.coeffs().allFinite()
      || !bodyB.transform->position.allFinite()
      || !bodyB.transform->orientation.coeffs().allFinite()) {
    return std::nullopt;
  }

  return RigidPairConstraintEligibility{
      std::move(*geometryResult), bodyA, bodyB};
}

/// Shared extraction implementation. Geometry providers may select canonical
/// JointModel geometry or a family-private compatibility geometry. Enrichers
/// add solver-owned policy without moving it into the shared input.
///
/// Row order follows the iteration order of the
/// `JointModel`/`JointState`/`JointActuation` view. Gauss-Seidel style
/// consumers are order-sensitive, so changing the view composition or its
/// driving pool changes solver trajectories.
template <
    typename InputVector,
    typename GeometryProvider,
    typename InputEnricher>
inline void extractRigidPairConstraintInputsInto(
    const WorldRegistry& registry,
    InputVector& inputs,
    bool includeWorldAnchors,
    GeometryProvider&& geometryProvider,
    InputEnricher&& enrichInput)
{
  using Input = typename InputVector::value_type;
  static_assert(
      std::is_same_v<Input, RigidPairConstraintInput>
      || std::is_base_of_v<RigidPairConstraintInput, Input>);

  const auto view = registry.view<
      comps::JointModel,
      comps::JointState,
      comps::JointActuation>();
  inputs.clear();
  inputs.reserve(view.size_hint());

  for (const entt::entity entity : view) {
    const auto& jointModel = view.get<comps::JointModel>(entity);
    const auto& jointState = view.get<comps::JointState>(entity);
    const auto& jointActuation = view.get<comps::JointActuation>(entity);

    const std::optional<RigidPairConstraintEligibility> eligibility
        = eligibleRigidPairConstraint(
            registry, entity, jointModel, jointState, geometryProvider);
    if (!eligibility.has_value()) {
      continue;
    }
    const RigidPairConstraintGeometry& geometry = eligibility->geometry;
    const RigidPairConstraintBodyView& bodyA = eligibility->bodyA;
    const RigidPairConstraintBodyView& bodyB = eligibility->bodyB;
    const auto* transformA = bodyA.transform;
    const auto* transformB = bodyB.transform;

    const bool hasAngularVelocityMotor
        = jointModel.type == comps::JointType::Revolute
          && jointActuation.actuatorType == comps::ActuatorType::Velocity
          && jointActuation.commandVelocity.size() == 1
          && jointActuation.commandVelocity.allFinite();
    const bool hasLinearVelocityMotor
        = jointModel.type == comps::JointType::Prismatic
          && jointActuation.actuatorType == comps::ActuatorType::Velocity
          && jointActuation.commandVelocity.size() == 1
          && jointActuation.commandVelocity.allFinite();
    const bool useStableFullLinearBasis
        = !includeWorldAnchors
          && geometry.linearAxisMask == kRigidPairConstraintAllAxesMask
          && geometry.angularAxisMask == 0u && !hasAngularVelocityMotor
          && !hasLinearVelocityMotor
          && geometry.linearAxes.isApprox(Eigen::Matrix3d::Identity(), 0.0);

    Eigen::Quaterniond orientationA = Eigen::Quaterniond::Identity();
    Eigen::Matrix3d parentRotation = Eigen::Matrix3d::Identity();
    if (!useStableFullLinearBasis) {
      orientationA
          = normalizeRigidPairConstraintOrientation(transformA->orientation);
      parentRotation = orientationA.toRotationMatrix();
    }

    Input input;
    RigidPairConstraintInput& sharedInput = input;
    sharedInput.joint = entity;
    sharedInput.bodyA = jointModel.parentLink;
    sharedInput.bodyB = jointModel.childLink;
    sharedInput.bodyAView = bodyA;
    sharedInput.bodyBView = bodyB;
    if (includeWorldAnchors) {
      const Eigen::Quaterniond orientationB
          = normalizeRigidPairConstraintOrientation(transformB->orientation);
      sharedInput.anchorA
          = transformA->position + orientationA * geometry.localAnchorA;
      sharedInput.anchorB
          = transformB->position + orientationB * geometry.localAnchorB;
    } else {
      sharedInput.anchorA = geometry.localAnchorA;
      sharedInput.anchorB = geometry.localAnchorB;
      sharedInput.anchorsAreLocal = true;
    }
    sharedInput.targetRelativeOrientation
        = normalizeRigidPairConstraintOrientation(
            geometry.targetRelativeOrientation);
    sharedInput.linearAxes = useStableFullLinearBasis
                                 ? geometry.linearAxes
                                 : parentRotation * geometry.linearAxes;
    sharedInput.angularAxes = useStableFullLinearBasis
                                  ? geometry.angularAxes
                                  : parentRotation * geometry.angularAxes;
    sharedInput.linearAxisMask = geometry.linearAxisMask;
    sharedInput.angularAxisMask = geometry.angularAxisMask;
    if (hasAngularVelocityMotor) {
      const double maxTorque
          = rigidPairConstraintSymmetricEffortLimit(jointModel);
      if (maxTorque > 0.0 && !std::isnan(maxTorque)) {
        sharedInput.useAngularMotor = true;
        sharedInput.motorTargetSpeed = jointActuation.commandVelocity[0];
        sharedInput.motorMaxTorque = maxTorque;
      }
    }
    if (hasLinearVelocityMotor) {
      const double maxForce
          = rigidPairConstraintSymmetricEffortLimit(jointModel);
      if (maxForce > 0.0 && !std::isnan(maxForce)) {
        sharedInput.useLinearMotor = true;
        sharedInput.motorTargetSpeed = jointActuation.commandVelocity[0];
        sharedInput.motorMaxForce = maxForce;
      }
    }
    sharedInput.fractureThreshold = jointModel.breakForce;
    if (enrichInput(entity, input)) {
      inputs.push_back(std::move(input));
    }
  }
}

template <typename InputVector>
inline void extractRigidPairConstraintInputsInto(
    const WorldRegistry& registry,
    InputVector& inputs,
    bool includeWorldAnchors = true)
{
  const auto geometryProvider
      = [](entt::entity, const comps::JointModel& joint) {
          return rigidPairConstraintGeometryFromJointModel(joint);
        };
  const auto accept = [](entt::entity, auto&) {
    return true;
  };
  extractRigidPairConstraintInputsInto(
      registry, inputs, includeWorldAnchors, geometryProvider, accept);
}

} // namespace dart::simulation::detail
