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
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS OR
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

#include <dart/simulation/detail/deformable_vbd/avbd_constraint.hpp>
#include <dart/simulation/detail/deformable_vbd/contact_kernel.hpp>

#include <Eigen/Cholesky>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <algorithm>
#include <array>
#include <limits>
#include <span>
#include <utility>
#include <vector>

#include <cmath>
#include <cstddef>
#include <cstdint>

namespace dart::simulation::detail::deformable_vbd {

using Vector6d = Eigen::Matrix<double, 6, 1>;
using Matrix3x6d = Eigen::Matrix<double, 3, 6>;
using Matrix6d = Eigen::Matrix<double, 6, 6>;

inline constexpr double kAvbdRigidPi = 3.141592653589793238462643383279502884;
inline constexpr std::uint8_t kAvbdRigidJointAllAxesMask = 0x7u;
inline constexpr double kAvbdRigidMinDistanceSpringLength = 1e-12;

//==============================================================================
inline Eigen::Quaterniond normalizeAvbdRigidOrientation(
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

//==============================================================================
inline Eigen::Vector3d avbdRigidRotationVector(
    const Eigen::Quaterniond& orientation)
{
  const Eigen::Quaterniond q = normalizeAvbdRigidOrientation(orientation);
  const Eigen::Vector3d vector = q.vec();
  const double vectorNorm = vector.norm();
  if (vectorNorm <= 0.0 || !std::isfinite(vectorNorm)) {
    return Eigen::Vector3d::Zero();
  }

  double angle = 2.0 * std::atan2(vectorNorm, q.w());
  if (angle > kAvbdRigidPi) {
    angle -= 2.0 * kAvbdRigidPi;
  } else if (angle < -kAvbdRigidPi) {
    angle += 2.0 * kAvbdRigidPi;
  }
  return (angle / vectorNorm) * vector;
}

//==============================================================================
inline Eigen::Quaterniond avbdRigidOrientationDelta(
    const Eigen::Vector3d& angularStep)
{
  const double angle = angularStep.norm();
  if (angle <= 0.0 || !std::isfinite(angle)) {
    return Eigen::Quaterniond::Identity();
  }
  return Eigen::Quaterniond(Eigen::AngleAxisd(angle, angularStep / angle));
}

/// Internal 6-DOF rigid-body block accumulator for AVBD row solves.
///
/// The first three coordinates are world-frame translation. The last three are
/// a world-frame angular tangent vector applied by left-multiplying the current
/// orientation with an exponential-map quaternion.
struct AvbdRigidBodyBlock
{
  Vector6d force = Vector6d::Zero();
  Matrix6d hessian = Matrix6d::Zero();

  void reset() noexcept
  {
    force.setZero();
    hessian.setZero();
  }
};

struct AvbdRigidBodyState
{
  Eigen::Vector3d position = Eigen::Vector3d::Zero();
  Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();
};

struct AvbdRigidPointAttachmentRow
{
  Eigen::Vector3d localPoint = Eigen::Vector3d::Zero();
  Eigen::Vector3d target = Eigen::Vector3d::Zero();
  Eigen::Vector3d axis = Eigen::Vector3d::UnitX();
  AvbdScalarRowState state;
  double previousConstraintValue = 0.0;
  AvbdScalarRowBounds bounds{
      -std::numeric_limits<double>::infinity(),
      std::numeric_limits<double>::infinity()};
};

struct AvbdRigidPointPairRow
{
  Eigen::Vector3d localPointA = Eigen::Vector3d::Zero();
  Eigen::Vector3d localPointB = Eigen::Vector3d::Zero();
  Eigen::Vector3d axis = Eigen::Vector3d::UnitX();
  double offset = 0.0;
  AvbdScalarRowState state;
  double materialStiffness = std::numeric_limits<double>::infinity();
  double previousConstraintValue = 0.0;
  AvbdScalarRowBounds bounds{
      -std::numeric_limits<double>::infinity(),
      std::numeric_limits<double>::infinity()};
};

struct AvbdRigidPointPairDistanceSpringRow
{
  Eigen::Vector3d localPointA = Eigen::Vector3d::Zero();
  Eigen::Vector3d localPointB = Eigen::Vector3d::Zero();
  double restLength = 0.0;
  AvbdScalarRowState state;
  double materialStiffness = std::numeric_limits<double>::infinity();
};

struct AvbdRigidAngularPairRow
{
  Eigen::Quaterniond targetRelativeOrientation = Eigen::Quaterniond::Identity();
  Eigen::Vector3d axis = Eigen::Vector3d::UnitX();
  double offset = 0.0;
  AvbdScalarRowState state;
  double materialStiffness = std::numeric_limits<double>::infinity();
  double previousConstraintValue = 0.0;
  AvbdScalarRowBounds bounds{
      -std::numeric_limits<double>::infinity(),
      std::numeric_limits<double>::infinity()};
};

struct AvbdRigidBodyPointAttachmentRow
{
  std::uint32_t body = 0;
  AvbdRigidPointAttachmentRow row;
};

struct AvbdRigidBodyPointPairRow
{
  std::uint32_t bodyA = 0;
  std::uint32_t bodyB = 0;
  AvbdRigidPointPairRow row;
};

struct AvbdRigidBodyPointPairDistanceSpringRow
{
  std::uint32_t bodyA = 0;
  std::uint32_t bodyB = 0;
  AvbdContactEndpointId endpointA;
  AvbdContactEndpointId endpointB;
  std::uint32_t rowIndex = 0;
  AvbdRigidPointPairDistanceSpringRow row;
  double startStiffness = 1.0;
  double maxStiffness = std::numeric_limits<double>::infinity();
};

struct AvbdRigidBodyAngularPairRow
{
  std::uint32_t bodyA = 0;
  std::uint32_t bodyB = 0;
  AvbdRigidAngularPairRow row;
};

struct AvbdRigidBodyPointPairFrictionRows
{
  std::uint32_t bodyA = 0;
  std::uint32_t bodyB = 0;
  AvbdRigidPointPairRow first;
  AvbdRigidPointPairRow second;
};

struct AvbdRigidPointJoint
{
  std::uint32_t bodyA = 0;
  std::uint32_t bodyB = 0;
  AvbdContactEndpointId endpointA;
  AvbdContactEndpointId endpointB;
  Eigen::Vector3d localPointA = Eigen::Vector3d::Zero();
  Eigen::Vector3d localPointB = Eigen::Vector3d::Zero();
  Eigen::Quaterniond targetRelativeOrientation = Eigen::Quaterniond::Identity();
  Eigen::Matrix3d linearAxes = Eigen::Matrix3d::Identity();
  Eigen::Matrix3d angularAxes = Eigen::Matrix3d::Identity();
  std::uint8_t linearAxisMask = kAvbdRigidJointAllAxesMask;
  std::uint8_t angularAxisMask = kAvbdRigidJointAllAxesMask;
  double startStiffness = 1.0;
  double linearMaterialStiffness = std::numeric_limits<double>::infinity();
  double angularMaterialStiffness = std::numeric_limits<double>::infinity();
  double maxStiffness = std::numeric_limits<double>::infinity();
  double fractureThreshold = 0.0;
  std::uint32_t row = 0;
};

struct AvbdRigidAngularMotor
{
  std::uint32_t bodyA = 0;
  std::uint32_t bodyB = 0;
  AvbdContactEndpointId endpointA;
  AvbdContactEndpointId endpointB;
  Eigen::Quaterniond targetRelativeOrientation = Eigen::Quaterniond::Identity();
  Eigen::Vector3d axis = Eigen::Vector3d::UnitZ();
  double targetSpeed = 0.0;
  double maxTorque = std::numeric_limits<double>::infinity();
  double startStiffness = 1.0;
  double maxStiffness = std::numeric_limits<double>::infinity();
  std::uint32_t row = 0;
};

struct AvbdRigidLinearMotor
{
  std::uint32_t bodyA = 0;
  std::uint32_t bodyB = 0;
  AvbdContactEndpointId endpointA;
  AvbdContactEndpointId endpointB;
  Eigen::Vector3d localPointA = Eigen::Vector3d::Zero();
  Eigen::Vector3d localPointB = Eigen::Vector3d::Zero();
  Eigen::Vector3d axis = Eigen::Vector3d::UnitX();
  double targetSpeed = 0.0;
  double maxForce = std::numeric_limits<double>::infinity();
  double startStiffness = 1.0;
  double maxStiffness = std::numeric_limits<double>::infinity();
  std::uint32_t row = 0;
};

struct AvbdRigidContactManifoldPoint
{
  std::uint32_t bodyA = 0;
  std::uint32_t bodyB = 0;
  AvbdContactEndpointId endpointA;
  AvbdContactEndpointId endpointB;
  Eigen::Vector3d point = Eigen::Vector3d::Zero();
  Eigen::Vector3d normalFromAtoB = Eigen::Vector3d::UnitX();
  double depth = 0.0;
  double frictionCoefficient = 0.0;
  double startStiffness = 1.0;
  double maxStiffness = std::numeric_limits<double>::infinity();
  std::uint32_t row = 0;
};

struct AvbdRigidPointAttachmentOptions
{
  double alpha = 0.0;
  double beta = 1.0;
  double maxStiffness = std::numeric_limits<double>::infinity();
};

struct AvbdRigidPointPairFrictionOptions
{
  double alpha = 0.0;
  double beta = 1.0;
  double maxStiffness = std::numeric_limits<double>::infinity();
  double staticFrictionTolerance = 1e-12;
};

struct AvbdRigidPointPairDistanceSpringOptions
{
  double beta = 1.0;
  double maxStiffness = std::numeric_limits<double>::infinity();
};

struct AvbdRigidBlockDescentOptions
{
  std::size_t iterations = 20;
  double regularization = 0.0;
  double convergenceDisplacement = 0.0;
};

struct AvbdRigidBlockDescentStats
{
  std::size_t iterations = 0;
  std::size_t bodyUpdates = 0;
};

struct AvbdRigidBodyRowIndexScratch
{
  void clear()
  {
    attachmentRowOffsets.clear();
    attachmentRowIndices.clear();
    attachmentRowCursor.clear();
    attachmentRowBodyKeys.clear();
    attachmentRowBodyCount = 0u;
    pointPairRowOffsets.clear();
    pointPairRowIndices.clear();
    pointPairRowCursor.clear();
    pointPairRowBodyKeys.clear();
    pointPairRowBodyCount = 0u;
    distanceSpringRowOffsets.clear();
    distanceSpringRowIndices.clear();
    distanceSpringRowCursor.clear();
    distanceSpringRowBodyKeys.clear();
    distanceSpringRowBodyCount = 0u;
    angularPairRowOffsets.clear();
    angularPairRowIndices.clear();
    angularPairRowCursor.clear();
    angularPairRowBodyKeys.clear();
    angularPairRowBodyCount = 0u;
    frictionPairRowOffsets.clear();
    frictionPairRowIndices.clear();
    frictionPairRowCursor.clear();
    frictionPairRowBodyKeys.clear();
    frictionPairRowBodyCount = 0u;
  }

  void reserve(
      std::size_t bodyCapacity,
      std::size_t attachmentRowCapacity,
      std::size_t pointPairRowCapacity,
      std::size_t distanceSpringRowCapacity,
      std::size_t angularPairRowCapacity,
      std::size_t frictionPairRowCapacity)
  {
    const std::size_t offsetCapacity
        = bodyCapacity < std::numeric_limits<std::size_t>::max()
              ? bodyCapacity + 1u
              : bodyCapacity;
    const auto reserveFamily = [offsetCapacity](
                                   std::vector<std::size_t>& offsets,
                                   std::vector<std::size_t>& indices,
                                   std::vector<std::size_t>& cursor,
                                   std::vector<std::uint64_t>& keys,
                                   std::size_t rowCapacity,
                                   std::size_t indexCapacity) {
      offsets.reserve(offsetCapacity);
      indices.reserve(indexCapacity);
      cursor.reserve(offsetCapacity);
      keys.reserve(rowCapacity);
    };

    reserveFamily(
        attachmentRowOffsets,
        attachmentRowIndices,
        attachmentRowCursor,
        attachmentRowBodyKeys,
        attachmentRowCapacity,
        attachmentRowCapacity);
    reserveFamily(
        pointPairRowOffsets,
        pointPairRowIndices,
        pointPairRowCursor,
        pointPairRowBodyKeys,
        pointPairRowCapacity,
        2u * pointPairRowCapacity);
    reserveFamily(
        distanceSpringRowOffsets,
        distanceSpringRowIndices,
        distanceSpringRowCursor,
        distanceSpringRowBodyKeys,
        distanceSpringRowCapacity,
        2u * distanceSpringRowCapacity);
    reserveFamily(
        angularPairRowOffsets,
        angularPairRowIndices,
        angularPairRowCursor,
        angularPairRowBodyKeys,
        angularPairRowCapacity,
        2u * angularPairRowCapacity);
    reserveFamily(
        frictionPairRowOffsets,
        frictionPairRowIndices,
        frictionPairRowCursor,
        frictionPairRowBodyKeys,
        frictionPairRowCapacity,
        2u * frictionPairRowCapacity);
  }

  std::vector<std::size_t> attachmentRowOffsets;
  std::vector<std::size_t> attachmentRowIndices;
  std::vector<std::size_t> attachmentRowCursor;
  std::vector<std::uint64_t> attachmentRowBodyKeys;
  std::size_t attachmentRowBodyCount = 0u;
  std::vector<std::size_t> pointPairRowOffsets;
  std::vector<std::size_t> pointPairRowIndices;
  std::vector<std::size_t> pointPairRowCursor;
  std::vector<std::uint64_t> pointPairRowBodyKeys;
  std::size_t pointPairRowBodyCount = 0u;
  std::vector<std::size_t> distanceSpringRowOffsets;
  std::vector<std::size_t> distanceSpringRowIndices;
  std::vector<std::size_t> distanceSpringRowCursor;
  std::vector<std::uint64_t> distanceSpringRowBodyKeys;
  std::size_t distanceSpringRowBodyCount = 0u;
  std::vector<std::size_t> angularPairRowOffsets;
  std::vector<std::size_t> angularPairRowIndices;
  std::vector<std::size_t> angularPairRowCursor;
  std::vector<std::uint64_t> angularPairRowBodyKeys;
  std::size_t angularPairRowBodyCount = 0u;
  std::vector<std::size_t> frictionPairRowOffsets;
  std::vector<std::size_t> frictionPairRowIndices;
  std::vector<std::size_t> frictionPairRowCursor;
  std::vector<std::uint64_t> frictionPairRowBodyKeys;
  std::size_t frictionPairRowBodyCount = 0u;
};

//==============================================================================
inline Matrix6d avbdRigidBodyMassMatrix(
    double mass,
    const Eigen::Matrix3d& bodyInertia,
    const Eigen::Quaterniond& orientation)
{
  Matrix6d massMatrix = Matrix6d::Zero();
  massMatrix.topLeftCorner<3, 3>().diagonal().array() = mass;

  const Eigen::Matrix3d rotation
      = normalizeAvbdRigidOrientation(orientation).toRotationMatrix();
  massMatrix.bottomRightCorner<3, 3>().noalias()
      = rotation * bodyInertia * rotation.transpose();
  return massMatrix;
}

//==============================================================================
inline Eigen::Vector3d avbdRigidBodyOrientationError(
    const Eigen::Quaterniond& orientation,
    const Eigen::Quaterniond& targetOrientation)
{
  const Eigen::Quaterniond current = normalizeAvbdRigidOrientation(orientation);
  const Eigen::Quaterniond target
      = normalizeAvbdRigidOrientation(targetOrientation);
  return avbdRigidRotationVector(current * target.conjugate());
}

//==============================================================================
inline void addAvbdRigidBodyInertiaTerm(
    AvbdRigidBodyBlock& block,
    double mass,
    const Eigen::Matrix3d& bodyInertia,
    double timeStep,
    const AvbdRigidBodyState& state,
    const AvbdRigidBodyState& inertialTarget)
{
  const double invDt2 = 1.0 / (timeStep * timeStep);
  const Matrix6d massMatrix
      = avbdRigidBodyMassMatrix(mass, bodyInertia, state.orientation);
  const Vector6d error
      = (Vector6d() << state.position - inertialTarget.position,
         avbdRigidBodyOrientationError(
             state.orientation, inertialTarget.orientation))
            .finished();

  block.force.noalias() -= invDt2 * massMatrix * error;
  block.hessian.noalias() += invDt2 * massMatrix;
}

//==============================================================================
inline void addAvbdRigidBodyInertiaTermLowerTriangle(
    AvbdRigidBodyBlock& block,
    double mass,
    const Eigen::Matrix3d& bodyInertia,
    double timeStep,
    const AvbdRigidBodyState& state,
    const AvbdRigidBodyState& inertialTarget)
{
  const double invDt2 = 1.0 / (timeStep * timeStep);
  const Eigen::Quaterniond orientation
      = normalizeAvbdRigidOrientation(state.orientation);
  const Eigen::Matrix3d rotation = orientation.toRotationMatrix();
  const Eigen::Matrix3d worldInertia
      = rotation * bodyInertia * rotation.transpose();
  const Eigen::Vector3d orientationError = avbdRigidBodyOrientationError(
      state.orientation, inertialTarget.orientation);

  block.force.head<3>().noalias()
      -= invDt2 * mass * (state.position - inertialTarget.position);
  block.force.tail<3>().noalias() -= invDt2 * worldInertia * orientationError;

  block.hessian.topLeftCorner<3, 3>().diagonal().array() += invDt2 * mass;
  const Eigen::Matrix3d scaledWorldInertia = invDt2 * worldInertia;
  block.hessian.bottomRightCorner<3, 3>()
      .template triangularView<Eigen::Lower>() += scaledWorldInertia;
}

//==============================================================================
inline void addAvbdRigidBlockHessianRankOneLowerTriangle(
    AvbdRigidBodyBlock& block, const Vector6d& direction, double scale)
{
  block.hessian.template selfadjointView<Eigen::Lower>().rankUpdate(
      direction, scale);
}

//==============================================================================
inline Eigen::Matrix3d avbdRigidSkewMatrix(const Eigen::Vector3d& value)
{
  Eigen::Matrix3d result;
  result << 0.0, -value.z(), value.y(), value.z(), 0.0, -value.x(), -value.y(),
      value.x(), 0.0;
  return result;
}

//==============================================================================
inline Eigen::Vector3d avbdRigidBodyWorldPoint(
    const AvbdRigidBodyState& state, const Eigen::Vector3d& localPoint)
{
  if (localPoint.x() == 0.0 && localPoint.y() == 0.0 && localPoint.z() == 0.0) {
    return state.position;
  }

  return state.position
         + normalizeAvbdRigidOrientation(state.orientation) * localPoint;
}

//==============================================================================
inline bool avbdRigidWorldPointIsBodyOrigin(
    const AvbdRigidBodyState& state, const Eigen::Vector3d& worldPoint)
{
  return worldPoint.x() == state.position.x()
         && worldPoint.y() == state.position.y()
         && worldPoint.z() == state.position.z();
}

//==============================================================================
inline Vector6d avbdRigidWorldPointDirection(
    const AvbdRigidBodyState& state,
    const Eigen::Vector3d& worldPoint,
    const Eigen::Vector3d& axis)
{
  Vector6d direction = Vector6d::Zero();
  direction.head<3>() = axis;
  if (!avbdRigidWorldPointIsBodyOrigin(state, worldPoint)) {
    direction.tail<3>() = (worldPoint - state.position).cross(axis);
  }
  return direction;
}

//==============================================================================
inline Eigen::Vector3d normalizedAvbdRigidPointPairAxis(
    const Eigen::Vector3d& axis,
    const Eigen::Vector3d& fallback = Eigen::Vector3d::UnitX())
{
  const double norm = axis.norm();
  if (axis.allFinite() && norm > 0.0) {
    return axis / norm;
  }
  return fallback;
}

//==============================================================================
inline std::uint8_t avbdRigidJointAxisBit(std::uint8_t axis)
{
  return axis < 3u ? static_cast<std::uint8_t>(1u << axis) : 0u;
}

//==============================================================================
inline std::uint8_t avbdRigidJointAllButAxisMask(std::uint8_t freeAxis)
{
  return static_cast<std::uint8_t>(
      kAvbdRigidJointAllAxesMask & ~avbdRigidJointAxisBit(freeAxis));
}

//==============================================================================
inline Eigen::Matrix3d avbdRigidJointAxesFromFreeAxis(
    const Eigen::Vector3d& freeAxis,
    const Eigen::Vector3d& fallback = Eigen::Vector3d::UnitZ())
{
  const Eigen::Vector3d axis
      = normalizedAvbdRigidPointPairAxis(freeAxis, fallback);
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
inline Eigen::Vector3d avbdRigidPointPairRelativePosition(
    const AvbdRigidBodyState& stateA,
    const AvbdRigidBodyState& stateB,
    const AvbdRigidPointPairRow& row)
{
  return avbdRigidBodyWorldPoint(stateB, row.localPointB)
         - avbdRigidBodyWorldPoint(stateA, row.localPointA);
}

//==============================================================================
inline Eigen::Vector3d avbdRigidPointPairDistanceSpringRelativePosition(
    const AvbdRigidBodyState& stateA,
    const AvbdRigidBodyState& stateB,
    const AvbdRigidPointPairDistanceSpringRow& row)
{
  return avbdRigidBodyWorldPoint(stateB, row.localPointB)
         - avbdRigidBodyWorldPoint(stateA, row.localPointA);
}

//==============================================================================
inline Eigen::Quaterniond avbdRigidAngularPairTargetOrientationB(
    const AvbdRigidBodyState& stateA, const AvbdRigidAngularPairRow& row)
{
  return normalizeAvbdRigidOrientation(
      normalizeAvbdRigidOrientation(stateA.orientation)
      * row.targetRelativeOrientation);
}

//==============================================================================
inline Eigen::Vector3d avbdRigidBodyLocalPoint(
    const AvbdRigidBodyState& state, const Eigen::Vector3d& worldPoint)
{
  return normalizeAvbdRigidOrientation(state.orientation).conjugate()
         * (worldPoint - state.position);
}

//==============================================================================
inline AvbdRigidPointPairRow makeAvbdRigidContactNormalRow(
    const Eigen::Vector3d& localPointA,
    const Eigen::Vector3d& localPointB,
    const Eigen::Vector3d& normalOnA,
    double targetDistance,
    AvbdScalarRowState state,
    double previousConstraintValue = 0.0)
{
  AvbdRigidPointPairRow row;
  row.localPointA = localPointA;
  row.localPointB = localPointB;
  row.axis = normalizedAvbdRigidPointPairAxis(normalOnA);
  row.offset = targetDistance;
  row.state = state;
  row.previousConstraintValue = previousConstraintValue;
  row.bounds = avbdContactNormalBounds();
  return row;
}

//==============================================================================
inline Eigen::Matrix<double, 3, 2> avbdRigidContactTangentBasis(
    const Eigen::Vector3d& normalFromAtoB)
{
  Eigen::Vector3d normal = normalFromAtoB;
  const double norm = normal.norm();
  if (!normal.allFinite() || norm <= 0.0) {
    normal = Eigen::Vector3d::UnitX();
  } else {
    normal /= norm;
  }

  Eigen::Matrix<double, 3, 2> basis;
  basis.col(0) = normal.unitOrthogonal();
  basis.col(1) = normal.cross(basis.col(0)).normalized();
  return basis;
}

//==============================================================================
inline bool isValidAvbdRigidContactFrictionDirection(
    const Eigen::Vector3d& direction)
{
  return direction.allFinite() && direction.squaredNorm() > 0.0;
}

//==============================================================================
inline AvbdRigidPointPairRow makeAvbdRigidContactFrictionTangentRow(
    const Eigen::Vector3d& localPointA,
    const Eigen::Vector3d& localPointB,
    const Eigen::Vector3d& tangentOnA,
    const Eigen::Vector3d& stepStartRelativePosition,
    double forceLimit,
    AvbdScalarRowState state,
    double previousConstraintValue = 0.0)
{
  AvbdRigidPointPairRow row;
  row.localPointA = localPointA;
  row.localPointB = localPointB;
  row.axis
      = normalizedAvbdRigidPointPairAxis(tangentOnA, Eigen::Vector3d::UnitY());
  row.offset = -row.axis.dot(stepStartRelativePosition);
  row.state = state;
  row.previousConstraintValue = previousConstraintValue;
  row.bounds = avbdFrictionTangentBounds(forceLimit);
  return row;
}

//==============================================================================
inline AvbdRigidAngularPairRow makeAvbdRigidJointAngularRow(
    const Eigen::Quaterniond& targetRelativeOrientation,
    const Eigen::Vector3d& axis,
    AvbdScalarRowState state,
    double previousConstraintValue = 0.0)
{
  AvbdRigidAngularPairRow row;
  row.targetRelativeOrientation
      = normalizeAvbdRigidOrientation(targetRelativeOrientation);
  row.axis = normalizedAvbdRigidPointPairAxis(axis);
  row.state = state;
  row.previousConstraintValue = previousConstraintValue;
  return row;
}

//==============================================================================
inline AvbdRigidAngularPairRow makeAvbdRigidAngularMotorRow(
    const Eigen::Quaterniond& targetRelativeOrientation,
    const Eigen::Vector3d& axis,
    double targetSpeed,
    double timeStep,
    AvbdScalarRowState state)
{
  AvbdRigidAngularPairRow row = makeAvbdRigidJointAngularRow(
      targetRelativeOrientation, axis, state, /*previousConstraintValue=*/0.0);
  row.offset = -targetSpeed * timeStep;
  return row;
}

//==============================================================================
inline AvbdRigidPointPairRow makeAvbdRigidLinearMotorRow(
    const Eigen::Vector3d& localPointA,
    const Eigen::Vector3d& localPointB,
    const Eigen::Vector3d& axis,
    const Eigen::Vector3d& stepStartRelativePosition,
    double targetSpeed,
    double timeStep,
    AvbdScalarRowState state)
{
  AvbdRigidPointPairRow row;
  row.localPointA = localPointA;
  row.localPointB = localPointB;
  row.axis = normalizedAvbdRigidPointPairAxis(axis, Eigen::Vector3d::UnitX());
  row.offset
      = -row.axis.dot(stepStartRelativePosition) - targetSpeed * timeStep;
  row.state = state;
  return row;
}

//==============================================================================
inline AvbdRigidAngularMotor makeAvbdRigidAngularMotor(
    std::uint32_t bodyA,
    std::uint32_t bodyB,
    AvbdContactEndpointId endpointA,
    AvbdContactEndpointId endpointB,
    const Eigen::Quaterniond& targetRelativeOrientation,
    const Eigen::Vector3d& axis,
    double targetSpeed,
    double maxTorque,
    double startStiffness = 1.0,
    double maxStiffness = std::numeric_limits<double>::infinity(),
    std::uint32_t row = 0)
{
  AvbdRigidAngularMotor motor;
  motor.bodyA = bodyA;
  motor.bodyB = bodyB;
  motor.endpointA = endpointA;
  motor.endpointB = endpointB;
  motor.targetRelativeOrientation
      = normalizeAvbdRigidOrientation(targetRelativeOrientation);
  motor.axis = normalizedAvbdRigidPointPairAxis(axis, Eigen::Vector3d::UnitZ());
  motor.targetSpeed = targetSpeed;
  motor.maxTorque = std::max(0.0, maxTorque);
  motor.startStiffness = std::max(0.0, startStiffness);
  motor.maxStiffness = std::max(motor.startStiffness, maxStiffness);
  motor.row = row;
  return motor;
}

//==============================================================================
inline AvbdRigidLinearMotor makeAvbdRigidLinearMotor(
    std::uint32_t bodyA,
    std::uint32_t bodyB,
    AvbdContactEndpointId endpointA,
    AvbdContactEndpointId endpointB,
    const Eigen::Vector3d& localPointA,
    const Eigen::Vector3d& localPointB,
    const Eigen::Vector3d& axis,
    double targetSpeed,
    double maxForce,
    double startStiffness = 1.0,
    double maxStiffness = std::numeric_limits<double>::infinity(),
    std::uint32_t row = 0)
{
  AvbdRigidLinearMotor motor;
  motor.bodyA = bodyA;
  motor.bodyB = bodyB;
  motor.endpointA = endpointA;
  motor.endpointB = endpointB;
  motor.localPointA = localPointA;
  motor.localPointB = localPointB;
  motor.axis = normalizedAvbdRigidPointPairAxis(axis, Eigen::Vector3d::UnitX());
  motor.targetSpeed = targetSpeed;
  motor.maxForce = std::max(0.0, maxForce);
  motor.startStiffness = std::max(0.0, startStiffness);
  motor.maxStiffness = std::max(motor.startStiffness, maxStiffness);
  motor.row = row;
  return motor;
}

//==============================================================================
inline AvbdRigidPointJoint makeAvbdRigidRevolutePointJoint(
    std::uint32_t bodyA,
    std::uint32_t bodyB,
    AvbdContactEndpointId endpointA,
    AvbdContactEndpointId endpointB,
    const Eigen::Vector3d& localPointA,
    const Eigen::Vector3d& localPointB,
    const Eigen::Quaterniond& targetRelativeOrientation,
    const Eigen::Vector3d& hingeAxisWorld,
    double startStiffness = 1.0,
    double maxStiffness = std::numeric_limits<double>::infinity(),
    std::uint32_t row = 0)
{
  AvbdRigidPointJoint joint;
  joint.bodyA = bodyA;
  joint.bodyB = bodyB;
  joint.endpointA = endpointA;
  joint.endpointB = endpointB;
  joint.localPointA = localPointA;
  joint.localPointB = localPointB;
  joint.targetRelativeOrientation
      = normalizeAvbdRigidOrientation(targetRelativeOrientation);
  joint.angularAxes = avbdRigidJointAxesFromFreeAxis(hingeAxisWorld);
  joint.linearAxisMask = kAvbdRigidJointAllAxesMask;
  joint.angularAxisMask = avbdRigidJointAllButAxisMask(/*freeAxis=*/2u);
  joint.startStiffness = std::max(0.0, startStiffness);
  joint.maxStiffness = std::max(joint.startStiffness, maxStiffness);
  joint.row = row;
  return joint;
}

//==============================================================================
inline AvbdRigidPointJoint makeAvbdRigidPrismaticPointJoint(
    std::uint32_t bodyA,
    std::uint32_t bodyB,
    AvbdContactEndpointId endpointA,
    AvbdContactEndpointId endpointB,
    const Eigen::Vector3d& localPointA,
    const Eigen::Vector3d& localPointB,
    const Eigen::Quaterniond& targetRelativeOrientation,
    const Eigen::Vector3d& translationAxisWorld,
    double startStiffness = 1.0,
    double maxStiffness = std::numeric_limits<double>::infinity(),
    std::uint32_t row = 0)
{
  AvbdRigidPointJoint joint;
  joint.bodyA = bodyA;
  joint.bodyB = bodyB;
  joint.endpointA = endpointA;
  joint.endpointB = endpointB;
  joint.localPointA = localPointA;
  joint.localPointB = localPointB;
  joint.targetRelativeOrientation
      = normalizeAvbdRigidOrientation(targetRelativeOrientation);
  joint.linearAxes = avbdRigidJointAxesFromFreeAxis(translationAxisWorld);
  joint.angularAxes = joint.linearAxes;
  joint.linearAxisMask = avbdRigidJointAllButAxisMask(/*freeAxis=*/2u);
  joint.angularAxisMask = kAvbdRigidJointAllAxesMask;
  joint.startStiffness = std::max(0.0, startStiffness);
  joint.maxStiffness = std::max(joint.startStiffness, maxStiffness);
  joint.row = row;
  return joint;
}

namespace detail {

inline constexpr std::size_t kAvbdRigidSmallRowStackCapacity = 16u;

//==============================================================================
inline bool avbdRigidVectorExactEqual(
    const Eigen::Vector3d& a, const Eigen::Vector3d& b)
{
  return (a.array() == b.array()).all();
}

//==============================================================================
inline bool avbdRigidQuaternionExactEqual(
    const Eigen::Quaterniond& a, const Eigen::Quaterniond& b)
{
  return (a.coeffs().array() == b.coeffs().array()).all();
}

//==============================================================================
inline bool avbdRigidJointAxisEnabled(std::uint8_t mask, std::uint8_t axis)
{
  return axis < 3u && (mask & avbdRigidJointAxisBit(axis)) != 0u;
}

//==============================================================================
inline bool hasValidActiveAvbdRigidJointAxes(
    const Eigen::Matrix3d& axes, std::uint8_t mask)
{
  for (std::uint8_t axis = 0; axis < 3u; ++axis) {
    if (!avbdRigidJointAxisEnabled(mask, axis)) {
      continue;
    }

    const Eigen::Vector3d column = axes.col(axis);
    if (!column.allFinite() || column.squaredNorm() <= 0.0) {
      return false;
    }
  }
  return true;
}

//==============================================================================
inline bool isValidAvbdRigidContactManifoldPoint(
    const AvbdRigidContactManifoldPoint& contact, std::size_t bodyCount)
{
  return contact.bodyA < bodyCount && contact.bodyB < bodyCount
         && contact.bodyA != contact.bodyB && contact.point.allFinite()
         && contact.normalFromAtoB.allFinite()
         && contact.normalFromAtoB.squaredNorm() > 0.0 && contact.depth > 0.0
         && std::isfinite(contact.depth);
}

//==============================================================================
inline bool isValidAvbdRigidPointJoint(
    const AvbdRigidPointJoint& joint, std::size_t bodyCount)
{
  return joint.bodyA < bodyCount && joint.bodyB < bodyCount
         && joint.bodyA != joint.bodyB && joint.localPointA.allFinite()
         && joint.localPointB.allFinite()
         && joint.targetRelativeOrientation.coeffs().allFinite()
         && joint.targetRelativeOrientation.norm() > 0.0
         && hasValidActiveAvbdRigidJointAxes(
             joint.linearAxes, joint.linearAxisMask)
         && hasValidActiveAvbdRigidJointAxes(
             joint.angularAxes, joint.angularAxisMask)
         && !std::isnan(joint.startStiffness) && joint.startStiffness >= 0.0
         && !std::isnan(joint.linearMaterialStiffness)
         && joint.linearMaterialStiffness >= 0.0
         && !std::isnan(joint.angularMaterialStiffness)
         && joint.angularMaterialStiffness >= 0.0
         && !std::isnan(joint.maxStiffness)
         && joint.maxStiffness >= joint.startStiffness;
}

//==============================================================================
inline bool isValidAvbdRigidAngularMotor(
    const AvbdRigidAngularMotor& motor, std::size_t bodyCount, double timeStep)
{
  return motor.bodyA < bodyCount && motor.bodyB < bodyCount
         && motor.bodyA != motor.bodyB
         && motor.targetRelativeOrientation.coeffs().allFinite()
         && motor.targetRelativeOrientation.norm() > 0.0
         && motor.axis.allFinite() && motor.axis.squaredNorm() > 0.0
         && std::isfinite(motor.targetSpeed) && timeStep > 0.0
         && std::isfinite(timeStep) && motor.maxTorque > 0.0
         && !std::isnan(motor.maxTorque);
}

//==============================================================================
inline bool isValidAvbdRigidLinearMotor(
    const AvbdRigidLinearMotor& motor, std::size_t bodyCount, double timeStep)
{
  return motor.bodyA < bodyCount && motor.bodyB < bodyCount
         && motor.bodyA != motor.bodyB && motor.localPointA.allFinite()
         && motor.localPointB.allFinite() && motor.axis.allFinite()
         && motor.axis.squaredNorm() > 0.0 && std::isfinite(motor.targetSpeed)
         && timeStep > 0.0 && std::isfinite(timeStep) && motor.maxForce > 0.0
         && !std::isnan(motor.maxForce);
}

//==============================================================================
inline bool isValidAvbdRigidDistanceSpring(
    const AvbdRigidBodyPointPairDistanceSpringRow& spring,
    std::size_t bodyCount)
{
  return spring.bodyA < bodyCount && spring.bodyB < bodyCount
         && spring.bodyA != spring.bodyB && spring.row.localPointA.allFinite()
         && spring.row.localPointB.allFinite()
         && std::isfinite(spring.row.restLength) && spring.row.restLength >= 0.0
         && !std::isnan(spring.startStiffness) && spring.startStiffness >= 0.0
         && !std::isnan(spring.row.materialStiffness)
         && spring.row.materialStiffness >= 0.0
         && !std::isnan(spring.maxStiffness)
         && spring.maxStiffness >= spring.startStiffness;
}

//==============================================================================
inline AvbdScalarRowDescriptor makeAvbdRigidJointLinearRowDescriptor(
    AvbdContactEndpointId first,
    AvbdContactEndpointId second,
    double startStiffness,
    double materialStiffness,
    double maxStiffness,
    std::uint32_t row = 0,
    std::uint8_t axis = 0)
{
  AvbdScalarRowDescriptor descriptor;
  descriptor.key = makeAvbdEndpointPairRowKey(
      AvbdScalarRowRole::JointLinear, first, second, row, axis);
  descriptor.kind = std::isfinite(materialStiffness)
                        ? AvbdScalarRowKind::FiniteStiffness
                        : AvbdScalarRowKind::HardConstraint;
  descriptor.bounds
      = {-std::numeric_limits<double>::infinity(),
         std::numeric_limits<double>::infinity()};
  descriptor.startStiffness = startStiffness;
  descriptor.materialStiffness = materialStiffness;
  descriptor.maxStiffness = maxStiffness;
  return descriptor;
}

//==============================================================================
inline AvbdScalarRowDescriptor makeAvbdRigidJointAngularRowDescriptor(
    AvbdContactEndpointId first,
    AvbdContactEndpointId second,
    double startStiffness,
    double materialStiffness,
    double maxStiffness,
    std::uint32_t row = 0,
    std::uint8_t axis = 0)
{
  AvbdScalarRowDescriptor descriptor;
  descriptor.key = makeAvbdEndpointPairRowKey(
      AvbdScalarRowRole::JointAngular, first, second, row, axis);
  descriptor.kind = std::isfinite(materialStiffness)
                        ? AvbdScalarRowKind::FiniteStiffness
                        : AvbdScalarRowKind::HardConstraint;
  descriptor.bounds
      = {-std::numeric_limits<double>::infinity(),
         std::numeric_limits<double>::infinity()};
  descriptor.startStiffness = startStiffness;
  descriptor.materialStiffness = materialStiffness;
  descriptor.maxStiffness = maxStiffness;
  return descriptor;
}

//==============================================================================
inline AvbdScalarRowDescriptor makeAvbdRigidDistanceSpringRowDescriptor(
    AvbdContactEndpointId first,
    AvbdContactEndpointId second,
    double startStiffness,
    double materialStiffness,
    double maxStiffness,
    std::uint32_t row = 0)
{
  AvbdScalarRowDescriptor descriptor;
  descriptor.key = makeAvbdEndpointPairRowKey(
      AvbdScalarRowRole::RigidDistanceSpring, first, second, row, /*axis=*/0);
  descriptor.kind = AvbdScalarRowKind::FiniteStiffness;
  descriptor.startStiffness = startStiffness;
  descriptor.materialStiffness = materialStiffness;
  descriptor.maxStiffness = maxStiffness;
  return descriptor;
}

//==============================================================================
inline AvbdScalarRowDescriptor makeAvbdRigidAngularMotorRowDescriptor(
    AvbdContactEndpointId first,
    AvbdContactEndpointId second,
    double maxTorque,
    double startStiffness,
    double maxStiffness,
    std::uint32_t row = 0)
{
  AvbdScalarRowDescriptor descriptor;
  descriptor.key = makeAvbdEndpointPairRowKey(
      AvbdScalarRowRole::Motor, first, second, row, /*axis=*/0);
  descriptor.kind = AvbdScalarRowKind::HardConstraint;
  descriptor.bounds = {-maxTorque, maxTorque};
  descriptor.startStiffness = startStiffness;
  descriptor.maxStiffness = maxStiffness;
  return descriptor;
}

//==============================================================================
inline AvbdScalarRowDescriptor makeAvbdRigidLinearMotorRowDescriptor(
    AvbdContactEndpointId first,
    AvbdContactEndpointId second,
    double maxForce,
    double startStiffness,
    double maxStiffness,
    std::uint32_t row = 0)
{
  AvbdScalarRowDescriptor descriptor;
  descriptor.key = makeAvbdEndpointPairRowKey(
      AvbdScalarRowRole::Motor, first, second, row, /*axis=*/0);
  descriptor.kind = AvbdScalarRowKind::HardConstraint;
  descriptor.bounds = {-maxForce, maxForce};
  descriptor.startStiffness = startStiffness;
  descriptor.maxStiffness = maxStiffness;
  return descriptor;
}

} // namespace detail

//==============================================================================
inline double avbdRigidPointAttachmentConstraintValueAtWorldPoint(
    const Eigen::Vector3d& worldPoint, const AvbdRigidPointAttachmentRow& row)
{
  return row.axis.dot(row.target - worldPoint);
}

//==============================================================================
inline double avbdRigidPointAttachmentConstraintValue(
    const AvbdRigidBodyState& state, const AvbdRigidPointAttachmentRow& row)
{
  return avbdRigidPointAttachmentConstraintValueAtWorldPoint(
      avbdRigidBodyWorldPoint(state, row.localPoint), row);
}

//==============================================================================
inline Vector6d avbdRigidPointAttachmentDirection(
    const AvbdRigidBodyState& state, const AvbdRigidPointAttachmentRow& row)
{
  const Eigen::Vector3d worldPoint
      = avbdRigidBodyWorldPoint(state, row.localPoint);
  return avbdRigidWorldPointDirection(state, worldPoint, row.axis);
}

//==============================================================================
inline double avbdRigidPointPairConstraintValue(
    const AvbdRigidBodyState& stateA,
    const AvbdRigidBodyState& stateB,
    const AvbdRigidPointPairRow& row)
{
  return row.offset
         + row.axis.dot(
             avbdRigidPointPairRelativePosition(stateA, stateB, row));
}

//==============================================================================
inline double avbdRigidPointPairConstraintValueAtWorldPoints(
    const AvbdRigidPointPairRow& row,
    const Eigen::Vector3d& worldPointA,
    const Eigen::Vector3d& worldPointB)
{
  return row.offset + row.axis.dot(worldPointB - worldPointA);
}

//==============================================================================
inline Eigen::Vector2d avbdRigidPointPairConstraintValuesAtWorldPoints(
    const AvbdRigidPointPairRow& first,
    const AvbdRigidPointPairRow& second,
    const Eigen::Vector3d& firstWorldPointA,
    const Eigen::Vector3d& firstWorldPointB,
    const Eigen::Vector3d& secondWorldPointA,
    const Eigen::Vector3d& secondWorldPointB,
    double alpha)
{
  return Eigen::Vector2d(
      regularizeAvbdConstraintValue(
          avbdRigidPointPairConstraintValueAtWorldPoints(
              first, firstWorldPointA, firstWorldPointB),
          first.previousConstraintValue,
          alpha),
      regularizeAvbdConstraintValue(
          avbdRigidPointPairConstraintValueAtWorldPoints(
              second, secondWorldPointA, secondWorldPointB),
          second.previousConstraintValue,
          alpha));
}

//==============================================================================
inline bool avbdRigidPointPairRowsShareLocalPoints(
    const AvbdRigidPointPairRow& first, const AvbdRigidPointPairRow& second)
{
  return detail::avbdRigidVectorExactEqual(
             first.localPointA, second.localPointA)
         && detail::avbdRigidVectorExactEqual(
             first.localPointB, second.localPointB);
}

//==============================================================================
inline double avbdRigidPointPairDistanceSpringConstraintValue(
    const AvbdRigidBodyState& stateA,
    const AvbdRigidBodyState& stateB,
    const AvbdRigidPointPairDistanceSpringRow& row)
{
  const double length
      = avbdRigidPointPairDistanceSpringRelativePosition(stateA, stateB, row)
            .norm();
  if (!std::isfinite(length)) {
    return 0.0;
  }
  return length - row.restLength;
}

//==============================================================================
inline double avbdRigidAngularPairConstraintValue(
    const AvbdRigidBodyState& stateA,
    const AvbdRigidBodyState& stateB,
    const AvbdRigidAngularPairRow& row)
{
  return row.offset
         + row.axis.dot(avbdRigidBodyOrientationError(
             stateB.orientation,
             avbdRigidAngularPairTargetOrientationB(stateA, row)));
}

//==============================================================================
inline Eigen::Vector2d avbdRigidPointPairConstraintValues(
    const AvbdRigidBodyState& stateA,
    const AvbdRigidBodyState& stateB,
    const AvbdRigidPointPairRow& first,
    const AvbdRigidPointPairRow& second,
    double alpha)
{
  const Eigen::Vector3d firstWorldPointA
      = avbdRigidBodyWorldPoint(stateA, first.localPointA);
  const Eigen::Vector3d firstWorldPointB
      = avbdRigidBodyWorldPoint(stateB, first.localPointB);
  const bool sharedAnchors
      = avbdRigidPointPairRowsShareLocalPoints(first, second);
  const Eigen::Vector3d secondWorldPointA
      = sharedAnchors ? firstWorldPointA
                      : avbdRigidBodyWorldPoint(stateA, second.localPointA);
  const Eigen::Vector3d secondWorldPointB
      = sharedAnchors ? firstWorldPointB
                      : avbdRigidBodyWorldPoint(stateB, second.localPointB);
  return avbdRigidPointPairConstraintValuesAtWorldPoints(
      first,
      second,
      firstWorldPointA,
      firstWorldPointB,
      secondWorldPointA,
      secondWorldPointB,
      alpha);
}

//==============================================================================
inline double avbdRigidPointPairFrictionForceLimit(
    const AvbdRigidPointPairRow& row)
{
  const double lowerLimit = row.bounds.lower < 0.0 ? -row.bounds.lower : 0.0;
  const double upperLimit = row.bounds.upper > 0.0 ? row.bounds.upper : 0.0;
  return std::max(0.0, std::min(lowerLimit, upperLimit));
}

//==============================================================================
inline double avbdRigidPointPairFrictionPairForceLimit(
    const AvbdRigidPointPairRow& first, const AvbdRigidPointPairRow& second)
{
  return std::min(
      avbdRigidPointPairFrictionForceLimit(first),
      avbdRigidPointPairFrictionForceLimit(second));
}

//==============================================================================
inline bool avbdRigidScalarRowExceedsFractureThreshold(
    const AvbdScalarRowState& state, double fractureThreshold)
{
  return fractureThreshold > 0.0 && std::isfinite(fractureThreshold)
         && std::abs(state.lambda) >= fractureThreshold;
}

//==============================================================================
inline double avbdRigidAngularPairLambdaNorm(
    std::span<const AvbdRigidBodyAngularPairRow> rows)
{
  double squaredNorm = 0.0;
  for (const AvbdRigidBodyAngularPairRow& row : rows) {
    if (!std::isfinite(row.row.state.lambda)) {
      return std::numeric_limits<double>::infinity();
    }
    squaredNorm += row.row.state.lambda * row.row.state.lambda;
  }
  return std::sqrt(squaredNorm);
}

//==============================================================================
inline bool avbdRigidAngularPairRowsExceedFractureThreshold(
    std::span<const AvbdRigidBodyAngularPairRow> rows, double fractureThreshold)
{
  return fractureThreshold > 0.0 && std::isfinite(fractureThreshold)
         && avbdRigidAngularPairLambdaNorm(rows) >= fractureThreshold;
}

//==============================================================================
inline void resetAvbdRigidAngularPairRowsAfterFracture(
    std::span<AvbdRigidBodyAngularPairRow> rows)
{
  for (AvbdRigidBodyAngularPairRow& row : rows) {
    row.row.state.lambda = 0.0;
    row.row.state.stiffness = 0.0;
    row.row.previousConstraintValue = 0.0;
  }
}

//==============================================================================
inline bool avbdRigidPointPairFrictionPreviousDualInsideCone(
    const AvbdRigidPointPairRow& first,
    const AvbdRigidPointPairRow& second,
    double staticFrictionTolerance = 1e-12)
{
  const double limit = avbdRigidPointPairFrictionPairForceLimit(first, second);
  if (!std::isfinite(limit)) {
    return true;
  }

  const double tolerance = std::max(0.0, staticFrictionTolerance);
  const double previousNorm
      = std::hypot(first.state.lambda, second.state.lambda);
  return previousNorm < std::max(0.0, limit - tolerance);
}

//==============================================================================
inline Eigen::Vector2d
avbdRigidPointPairFrictionTangentPairForceFromConstraintValues(
    const Eigen::Vector2d& constraintValues,
    const AvbdRigidPointPairRow& first,
    const AvbdRigidPointPairRow& second,
    const AvbdRigidPointPairFrictionOptions& options,
    bool* clamped = nullptr)
{
  if (clamped != nullptr) {
    *clamped = false;
  }

  const double limit = avbdRigidPointPairFrictionPairForceLimit(first, second);
  if (limit <= 0.0) {
    if (clamped != nullptr) {
      *clamped = true;
    }
    return Eigen::Vector2d::Zero();
  }

  const bool staticMode = avbdRigidPointPairFrictionPreviousDualInsideCone(
      first, second, options.staticFrictionTolerance);
  if (!staticMode && std::isfinite(limit)) {
    const double tangentError = constraintValues.norm();
    if (tangentError > std::max(1e-12, options.staticFrictionTolerance)) {
      if (clamped != nullptr) {
        *clamped = true;
      }
      return (limit / tangentError) * constraintValues;
    }
  }

  Eigen::Vector2d force(
      first.state.stiffness * constraintValues.x() + first.state.lambda,
      second.state.stiffness * constraintValues.y() + second.state.lambda);
  if (std::isfinite(limit)) {
    const double norm = force.norm();
    if (norm > limit && norm > 0.0) {
      if (clamped != nullptr) {
        *clamped = true;
      }
      force *= limit / norm;
    }
  }
  return force;
}

//==============================================================================
inline Eigen::Vector2d avbdRigidPointPairFrictionTangentPairForce(
    const AvbdRigidBodyState& stateA,
    const AvbdRigidBodyState& stateB,
    const AvbdRigidPointPairRow& first,
    const AvbdRigidPointPairRow& second,
    const AvbdRigidPointPairFrictionOptions& options,
    bool* clamped = nullptr)
{
  return avbdRigidPointPairFrictionTangentPairForceFromConstraintValues(
      avbdRigidPointPairConstraintValues(
          stateA, stateB, first, second, options.alpha),
      first,
      second,
      options,
      clamped);
}

//==============================================================================
inline Vector6d avbdRigidPointPairDirectionA(
    const AvbdRigidBodyState& stateA, const AvbdRigidPointPairRow& row)
{
  const Eigen::Vector3d worldPoint
      = avbdRigidBodyWorldPoint(stateA, row.localPointA);
  return avbdRigidWorldPointDirection(stateA, worldPoint, row.axis);
}

//==============================================================================
inline Vector6d avbdRigidPointPairDirectionB(
    const AvbdRigidBodyState& stateB, const AvbdRigidPointPairRow& row)
{
  const Eigen::Vector3d worldPoint
      = avbdRigidBodyWorldPoint(stateB, row.localPointB);
  return avbdRigidWorldPointDirection(stateB, worldPoint, -row.axis);
}

//==============================================================================
inline Eigen::Vector3d avbdRigidPointPairDistanceSpringAxis(
    const AvbdRigidBodyState& stateA,
    const AvbdRigidBodyState& stateB,
    const AvbdRigidPointPairDistanceSpringRow& row)
{
  const Eigen::Vector3d relative
      = avbdRigidPointPairDistanceSpringRelativePosition(stateA, stateB, row);
  const double length = relative.norm();
  if (!relative.allFinite() || length <= kAvbdRigidMinDistanceSpringLength) {
    return Eigen::Vector3d::Zero();
  }
  return relative / length;
}

//==============================================================================
inline Vector6d avbdRigidPointPairDistanceSpringDirectionA(
    const AvbdRigidBodyState& stateA,
    const AvbdRigidBodyState& stateB,
    const AvbdRigidPointPairDistanceSpringRow& row)
{
  const Eigen::Vector3d axis
      = avbdRigidPointPairDistanceSpringAxis(stateA, stateB, row);
  const Eigen::Vector3d worldPoint
      = avbdRigidBodyWorldPoint(stateA, row.localPointA);
  return avbdRigidWorldPointDirection(stateA, worldPoint, axis);
}

//==============================================================================
inline Vector6d avbdRigidPointPairDistanceSpringDirectionB(
    const AvbdRigidBodyState& stateA,
    const AvbdRigidBodyState& stateB,
    const AvbdRigidPointPairDistanceSpringRow& row)
{
  const Eigen::Vector3d axis
      = avbdRigidPointPairDistanceSpringAxis(stateA, stateB, row);
  const Eigen::Vector3d worldPoint
      = avbdRigidBodyWorldPoint(stateB, row.localPointB);
  return avbdRigidWorldPointDirection(stateB, worldPoint, -axis);
}

//==============================================================================
inline Matrix3x6d avbdRigidWorldPointJacobianAtWorldPoint(
    const AvbdRigidBodyState& state, const Eigen::Vector3d& worldPoint)
{
  const Eigen::Vector3d arm = worldPoint - state.position;

  Matrix3x6d jacobian = Matrix3x6d::Zero();
  jacobian.leftCols<3>().setIdentity();
  jacobian.rightCols<3>() = -avbdRigidSkewMatrix(arm);
  return jacobian;
}

//==============================================================================
inline Matrix3x6d avbdRigidWorldPointJacobian(
    const AvbdRigidBodyState& state, const Eigen::Vector3d& localPoint)
{
  return avbdRigidWorldPointJacobianAtWorldPoint(
      state, avbdRigidBodyWorldPoint(state, localPoint));
}

//==============================================================================
inline Vector6d avbdRigidDistanceSpringDirectionAtWorldPoint(
    const AvbdRigidBodyState& state,
    const Eigen::Vector3d& worldPoint,
    const Eigen::Vector3d& axis)
{
  return avbdRigidWorldPointDirection(state, worldPoint, axis);
}

//==============================================================================
inline void addAvbdRigidDistanceSpringHessianAtWorldPoint(
    AvbdRigidBodyBlock& block,
    const AvbdRigidBodyState& state,
    const Eigen::Vector3d& worldPoint,
    const Eigen::Vector3d& axis,
    double length,
    double restLength,
    double stiffness,
    bool clampToPsd)
{
  if (!axis.allFinite() || length <= kAvbdRigidMinDistanceSpringLength
      || !std::isfinite(stiffness)) {
    return;
  }

  double transverse = 1.0 - restLength / length;
  if (clampToPsd) {
    transverse = std::max(0.0, transverse);
  }

  const Eigen::Matrix3d nnT = axis * axis.transpose();
  const Eigen::Matrix3d pointHessian
      = stiffness * (nnT + transverse * (Eigen::Matrix3d::Identity() - nnT));
  if (avbdRigidWorldPointIsBodyOrigin(state, worldPoint)) {
    block.hessian.topLeftCorner<3, 3>().noalias() += pointHessian;
    return;
  }

  const Matrix3x6d jacobian
      = avbdRigidWorldPointJacobianAtWorldPoint(state, worldPoint);
  block.hessian.noalias() += jacobian.transpose() * pointHessian * jacobian;
}

//==============================================================================
inline void addAvbdRigidDistanceSpringHessian(
    AvbdRigidBodyBlock& block,
    const AvbdRigidBodyState& state,
    const Eigen::Vector3d& localPoint,
    const Eigen::Vector3d& axis,
    double length,
    double restLength,
    double stiffness,
    bool clampToPsd)
{
  addAvbdRigidDistanceSpringHessianAtWorldPoint(
      block,
      state,
      avbdRigidBodyWorldPoint(state, localPoint),
      axis,
      length,
      restLength,
      stiffness,
      clampToPsd);
}

//==============================================================================
inline Vector6d avbdRigidAngularPairDirectionA(
    const AvbdRigidAngularPairRow& row)
{
  Vector6d direction = Vector6d::Zero();
  direction.tail<3>() = row.axis;
  return direction;
}

//==============================================================================
inline Vector6d avbdRigidAngularPairDirectionB(
    const AvbdRigidAngularPairRow& row)
{
  Vector6d direction = Vector6d::Zero();
  direction.tail<3>() = -row.axis;
  return direction;
}

//==============================================================================
inline double addAvbdRigidPointAttachment(
    AvbdRigidBodyBlock& block,
    const AvbdRigidBodyState& state,
    const AvbdRigidPointAttachmentRow& row,
    double alpha)
{
  const Eigen::Vector3d worldPoint
      = avbdRigidBodyWorldPoint(state, row.localPoint);
  const double constraintValue = regularizeAvbdConstraintValue(
      avbdRigidPointAttachmentConstraintValueAtWorldPoint(worldPoint, row),
      row.previousConstraintValue,
      alpha);
  const double forceMagnitude
      = computeAvbdHardConstraintForce(row.state, constraintValue, row.bounds);
  const Vector6d direction
      = avbdRigidWorldPointDirection(state, worldPoint, row.axis);
  block.force.noalias() += forceMagnitude * direction;
  block.hessian.noalias()
      += row.state.stiffness * (direction * direction.transpose());
  return forceMagnitude;
}

//==============================================================================
inline bool avbdRigidRowUsesFiniteMaterial(double materialStiffness) noexcept
{
  return std::isfinite(materialStiffness);
}

//==============================================================================
inline double avbdRigidScalarRowForce(
    const AvbdScalarRowState& state,
    double constraintValue,
    const AvbdScalarRowBounds& bounds,
    double materialStiffness)
{
  if (avbdRigidRowUsesFiniteMaterial(materialStiffness)) {
    return state.stiffness * constraintValue;
  }

  return computeAvbdHardConstraintForce(state, constraintValue, bounds);
}

//==============================================================================
inline double addAvbdRigidPointPair(
    AvbdRigidBodyBlock& blockA,
    AvbdRigidBodyBlock& blockB,
    const AvbdRigidBodyState& stateA,
    const AvbdRigidBodyState& stateB,
    const AvbdRigidPointPairRow& row,
    double alpha)
{
  const Eigen::Vector3d worldPointA
      = avbdRigidBodyWorldPoint(stateA, row.localPointA);
  const Eigen::Vector3d worldPointB
      = avbdRigidBodyWorldPoint(stateB, row.localPointB);
  const double rawConstraintValue
      = row.offset + row.axis.dot(worldPointB - worldPointA);
  const double constraintValue
      = avbdRigidRowUsesFiniteMaterial(row.materialStiffness)
            ? rawConstraintValue
            : regularizeAvbdConstraintValue(
                  rawConstraintValue, row.previousConstraintValue, alpha);
  const double forceMagnitude = avbdRigidScalarRowForce(
      row.state, constraintValue, row.bounds, row.materialStiffness);
  const Vector6d firstDirection
      = avbdRigidWorldPointDirection(stateA, worldPointA, row.axis);
  const Vector6d secondDirection
      = avbdRigidWorldPointDirection(stateB, worldPointB, -row.axis);

  // AVBD solves per-body blocks; coupling is carried by the shared scalar dual.
  blockA.force.noalias() += forceMagnitude * firstDirection;
  blockB.force.noalias() += forceMagnitude * secondDirection;
  blockA.hessian.noalias()
      += row.state.stiffness * (firstDirection * firstDirection.transpose());
  blockB.hessian.noalias()
      += row.state.stiffness * (secondDirection * secondDirection.transpose());
  return forceMagnitude;
}

//==============================================================================
inline double addAvbdRigidPointPairDistanceSpring(
    AvbdRigidBodyBlock& blockA,
    AvbdRigidBodyBlock& blockB,
    const AvbdRigidBodyState& stateA,
    const AvbdRigidBodyState& stateB,
    const AvbdRigidPointPairDistanceSpringRow& row,
    bool clampToPsd = true)
{
  const Eigen::Vector3d worldPointA
      = avbdRigidBodyWorldPoint(stateA, row.localPointA);
  const Eigen::Vector3d worldPointB
      = avbdRigidBodyWorldPoint(stateB, row.localPointB);
  const Eigen::Vector3d relative = worldPointB - worldPointA;
  const double length = relative.norm();
  if (!relative.allFinite() || length <= kAvbdRigidMinDistanceSpringLength) {
    return 0.0;
  }

  const Eigen::Vector3d axis = relative / length;
  const double constraintValue = length - row.restLength;
  const double forceMagnitude = row.state.stiffness * constraintValue;
  const Vector6d firstDirection
      = avbdRigidDistanceSpringDirectionAtWorldPoint(stateA, worldPointA, axis);
  const Vector6d secondDirection = avbdRigidDistanceSpringDirectionAtWorldPoint(
      stateB, worldPointB, -axis);

  blockA.force.noalias() += forceMagnitude * firstDirection;
  blockB.force.noalias() += forceMagnitude * secondDirection;
  addAvbdRigidDistanceSpringHessianAtWorldPoint(
      blockA,
      stateA,
      worldPointA,
      axis,
      length,
      row.restLength,
      row.state.stiffness,
      clampToPsd);
  addAvbdRigidDistanceSpringHessianAtWorldPoint(
      blockB,
      stateB,
      worldPointB,
      axis,
      length,
      row.restLength,
      row.state.stiffness,
      clampToPsd);
  return forceMagnitude;
}

//==============================================================================
inline double addAvbdRigidAngularPair(
    AvbdRigidBodyBlock& blockA,
    AvbdRigidBodyBlock& blockB,
    const AvbdRigidBodyState& stateA,
    const AvbdRigidBodyState& stateB,
    const AvbdRigidAngularPairRow& row,
    double alpha)
{
  const double rawConstraintValue
      = avbdRigidAngularPairConstraintValue(stateA, stateB, row);
  const double constraintValue
      = avbdRigidRowUsesFiniteMaterial(row.materialStiffness)
            ? rawConstraintValue
            : regularizeAvbdConstraintValue(
                  rawConstraintValue, row.previousConstraintValue, alpha);
  const double forceMagnitude = avbdRigidScalarRowForce(
      row.state, constraintValue, row.bounds, row.materialStiffness);
  const Vector6d firstDirection = avbdRigidAngularPairDirectionA(row);
  const Vector6d secondDirection = avbdRigidAngularPairDirectionB(row);

  blockA.force.noalias() += forceMagnitude * firstDirection;
  blockB.force.noalias() += forceMagnitude * secondDirection;
  blockA.hessian.noalias()
      += row.state.stiffness * (firstDirection * firstDirection.transpose());
  blockB.hessian.noalias()
      += row.state.stiffness * (secondDirection * secondDirection.transpose());
  return forceMagnitude;
}

//==============================================================================
inline Eigen::Vector2d addAvbdRigidPointPairFrictionTangentPair(
    AvbdRigidBodyBlock& blockA,
    AvbdRigidBodyBlock& blockB,
    const AvbdRigidBodyState& stateA,
    const AvbdRigidBodyState& stateB,
    const AvbdRigidPointPairRow& first,
    const AvbdRigidPointPairRow& second,
    const AvbdRigidPointPairFrictionOptions& options)
{
  const Eigen::Vector3d firstWorldPointA
      = avbdRigidBodyWorldPoint(stateA, first.localPointA);
  const Eigen::Vector3d firstWorldPointB
      = avbdRigidBodyWorldPoint(stateB, first.localPointB);
  const bool sharedAnchors
      = avbdRigidPointPairRowsShareLocalPoints(first, second);
  const Eigen::Vector3d secondWorldPointA
      = sharedAnchors ? firstWorldPointA
                      : avbdRigidBodyWorldPoint(stateA, second.localPointA);
  const Eigen::Vector3d secondWorldPointB
      = sharedAnchors ? firstWorldPointB
                      : avbdRigidBodyWorldPoint(stateB, second.localPointB);
  const Eigen::Vector2d constraintValues
      = avbdRigidPointPairConstraintValuesAtWorldPoints(
          first,
          second,
          firstWorldPointA,
          firstWorldPointB,
          secondWorldPointA,
          secondWorldPointB,
          options.alpha);
  const Eigen::Vector2d force
      = avbdRigidPointPairFrictionTangentPairForceFromConstraintValues(
          constraintValues, first, second, options);
  const Vector6d firstDirectionA
      = avbdRigidWorldPointDirection(stateA, firstWorldPointA, first.axis);
  const Vector6d firstDirectionB
      = avbdRigidWorldPointDirection(stateB, firstWorldPointB, -first.axis);
  const Vector6d secondDirectionA
      = avbdRigidWorldPointDirection(stateA, secondWorldPointA, second.axis);
  const Vector6d secondDirectionB
      = avbdRigidWorldPointDirection(stateB, secondWorldPointB, -second.axis);

  blockA.force.noalias()
      += force.x() * firstDirectionA + force.y() * secondDirectionA;
  blockB.force.noalias()
      += force.x() * firstDirectionB + force.y() * secondDirectionB;
  blockA.hessian.noalias() += first.state.stiffness
                              * (firstDirectionA * firstDirectionA.transpose());
  blockA.hessian.noalias()
      += second.state.stiffness
         * (secondDirectionA * secondDirectionA.transpose());
  blockB.hessian.noalias() += first.state.stiffness
                              * (firstDirectionB * firstDirectionB.transpose());
  blockB.hessian.noalias()
      += second.state.stiffness
         * (secondDirectionB * secondDirectionB.transpose());
  return force;
}

//==============================================================================
inline AvbdScalarRowState updateAvbdRigidPointAttachmentRow(
    AvbdScalarRowState state,
    const AvbdRigidBodyState& rigidState,
    const AvbdRigidPointAttachmentRow& row,
    const AvbdRigidPointAttachmentOptions& options)
{
  const double constraintValue = regularizeAvbdConstraintValue(
      avbdRigidPointAttachmentConstraintValue(rigidState, row),
      row.previousConstraintValue,
      options.alpha);
  return updateAvbdHardConstraintRow(
      state, constraintValue, options.beta, row.bounds, options.maxStiffness);
}

//==============================================================================
inline AvbdScalarRowState updateAvbdRigidPointPairRow(
    AvbdScalarRowState state,
    const AvbdRigidBodyState& stateA,
    const AvbdRigidBodyState& stateB,
    const AvbdRigidPointPairRow& row,
    const AvbdRigidPointAttachmentOptions& options)
{
  if (avbdRigidRowUsesFiniteMaterial(row.materialStiffness)) {
    state.lambda = 0.0;
    const double maxStiffness
        = std::min(row.materialStiffness, options.maxStiffness);
    if (options.beta >= 0.0 && state.stiffness >= maxStiffness) {
      state.stiffness = maxStiffness;
      return state;
    }

    const double rawConstraintValue
        = avbdRigidPointPairConstraintValue(stateA, stateB, row);
    state.stiffness = updateAvbdFiniteStiffness(
        state.stiffness, rawConstraintValue, options.beta, maxStiffness);
    return state;
  }

  const double rawConstraintValue
      = avbdRigidPointPairConstraintValue(stateA, stateB, row);
  const double constraintValue = regularizeAvbdConstraintValue(
      rawConstraintValue, row.previousConstraintValue, options.alpha);
  return updateAvbdHardConstraintRow(
      state, constraintValue, options.beta, row.bounds, options.maxStiffness);
}

//==============================================================================
inline AvbdScalarRowState updateAvbdRigidPointPairDistanceSpringRow(
    AvbdScalarRowState state,
    const AvbdRigidBodyState& stateA,
    const AvbdRigidBodyState& stateB,
    const AvbdRigidPointPairDistanceSpringRow& row,
    const AvbdRigidPointPairDistanceSpringOptions& options)
{
  state.lambda = 0.0;
  const double maxStiffness
      = std::min(row.materialStiffness, options.maxStiffness);
  if (options.beta >= 0.0 && state.stiffness >= maxStiffness) {
    state.stiffness = maxStiffness;
    return state;
  }

  const double constraintValue
      = avbdRigidPointPairDistanceSpringConstraintValue(stateA, stateB, row);
  state.stiffness = updateAvbdFiniteStiffness(
      state.stiffness, constraintValue, options.beta, maxStiffness);
  return state;
}

//==============================================================================
inline AvbdScalarRowState updateAvbdRigidAngularPairRow(
    AvbdScalarRowState state,
    const AvbdRigidBodyState& stateA,
    const AvbdRigidBodyState& stateB,
    const AvbdRigidAngularPairRow& row,
    const AvbdRigidPointAttachmentOptions& options)
{
  if (avbdRigidRowUsesFiniteMaterial(row.materialStiffness)) {
    state.lambda = 0.0;
    const double maxStiffness
        = std::min(row.materialStiffness, options.maxStiffness);
    if (options.beta >= 0.0 && state.stiffness >= maxStiffness) {
      state.stiffness = maxStiffness;
      return state;
    }

    const double rawConstraintValue
        = avbdRigidAngularPairConstraintValue(stateA, stateB, row);
    state.stiffness = updateAvbdFiniteStiffness(
        state.stiffness, rawConstraintValue, options.beta, maxStiffness);
    return state;
  }

  const double rawConstraintValue
      = avbdRigidAngularPairConstraintValue(stateA, stateB, row);
  const double constraintValue = regularizeAvbdConstraintValue(
      rawConstraintValue, row.previousConstraintValue, options.alpha);
  return updateAvbdHardConstraintRow(
      state, constraintValue, options.beta, row.bounds, options.maxStiffness);
}

//==============================================================================
inline void updateAvbdRigidPointPairFrictionTangentPair(
    AvbdRigidPointPairRow& first,
    AvbdRigidPointPairRow& second,
    const AvbdRigidBodyState& stateA,
    const AvbdRigidBodyState& stateB,
    const AvbdRigidPointPairFrictionOptions& options)
{
  bool clamped = false;
  const Eigen::Vector3d firstWorldPointA
      = avbdRigidBodyWorldPoint(stateA, first.localPointA);
  const Eigen::Vector3d firstWorldPointB
      = avbdRigidBodyWorldPoint(stateB, first.localPointB);
  const bool sharedAnchors
      = avbdRigidPointPairRowsShareLocalPoints(first, second);
  const Eigen::Vector3d secondWorldPointA
      = sharedAnchors ? firstWorldPointA
                      : avbdRigidBodyWorldPoint(stateA, second.localPointA);
  const Eigen::Vector3d secondWorldPointB
      = sharedAnchors ? firstWorldPointB
                      : avbdRigidBodyWorldPoint(stateB, second.localPointB);
  const Eigen::Vector2d constraintValues
      = avbdRigidPointPairConstraintValuesAtWorldPoints(
          first,
          second,
          firstWorldPointA,
          firstWorldPointB,
          secondWorldPointA,
          secondWorldPointB,
          options.alpha);
  const Eigen::Vector2d force
      = avbdRigidPointPairFrictionTangentPairForceFromConstraintValues(
          constraintValues, first, second, options, &clamped);

  first.state.lambda = force.x();
  second.state.lambda = force.y();
  if (!clamped) {
    first.state.stiffness = std::min(
        options.maxStiffness,
        first.state.stiffness + options.beta * std::abs(constraintValues.x()));
    second.state.stiffness = std::min(
        options.maxStiffness,
        second.state.stiffness + options.beta * std::abs(constraintValues.y()));
  }
}

//==============================================================================
inline Vector6d solveAvbdRigidBodyBlock(
    const AvbdRigidBodyBlock& block, double regularization = 0.0)
{
  Matrix6d hessian = block.hessian;
  if (regularization > 0.0) {
    hessian.diagonal().array() += regularization;
  }

  if (!hessian.allFinite() || !block.force.allFinite()) {
    return Vector6d::Zero();
  }

  Eigen::LDLT<Matrix6d> ldlt(hessian);
  if (ldlt.info() != Eigen::Success || !ldlt.isPositive()) {
    return Vector6d::Zero();
  }

  const Vector6d delta = ldlt.solve(block.force);
  if (!delta.allFinite()) {
    return Vector6d::Zero();
  }
  return delta;
}

//==============================================================================
inline void applyAvbdRigidBodyStep(
    AvbdRigidBodyState& state, const Vector6d& step)
{
  state.position += step.head<3>();
  state.orientation = normalizeAvbdRigidOrientation(
      avbdRigidOrientationDelta(step.tail<3>()) * state.orientation);
}

//==============================================================================
struct AvbdRigidContactManifoldRowScratch
{
  std::vector<AvbdRigidContactManifoldPoint> activeContacts;
  std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> contactLocalPoints;
  std::vector<AvbdScalarRowDescriptor> normalDescriptors;
  std::vector<AvbdScalarRowDescriptor> frictionDescriptors;
  std::vector<std::pair<AvbdScalarRowKey, Eigen::Vector3d>>
      previousFrictionDirections;
};

struct AvbdRigidPointJointActiveAxis
{
  const AvbdRigidPointJoint* joint = nullptr;
  std::uint8_t axis = 0;
};

struct AvbdRigidPointJointRowScratch
{
  std::vector<AvbdRigidPointJointActiveAxis> activeRows;
  std::vector<AvbdScalarRowDescriptor> descriptors;
};

struct AvbdRigidAngularMotorRowScratch
{
  std::vector<const AvbdRigidAngularMotor*> activeRows;
  std::vector<AvbdScalarRowDescriptor> descriptors;
};

struct AvbdRigidMotorRowScratch
{
  std::vector<const AvbdRigidLinearMotor*> activeLinearRows;
  std::vector<const AvbdRigidAngularMotor*> activeAngularRows;
};

struct AvbdRigidDistanceSpringRowScratch
{
  std::vector<const AvbdRigidBodyPointPairDistanceSpringRow*> activeRows;
};

//==============================================================================
inline void buildAvbdRigidContactManifoldRows(
    const std::vector<AvbdRigidBodyState>& states,
    std::span<const AvbdRigidContactManifoldPoint> contacts,
    AvbdScalarRowInventory& normalInventory,
    AvbdScalarRowInventory& frictionInventory,
    std::vector<AvbdRigidBodyPointPairRow>& normalRows,
    std::vector<AvbdRigidBodyPointPairFrictionRows>& frictionRows,
    AvbdRigidContactManifoldRowScratch& scratch,
    const AvbdRowWarmStartOptions& warmStartOptions = {})
{
  const auto appendActiveRows =
      [&](std::span<const AvbdRigidContactManifoldPoint> activeContacts,
          std::span<const AvbdScalarRowDescriptor> normalDescriptors,
          std::span<std::pair<Eigen::Vector3d, Eigen::Vector3d>>
              contactLocalPoints,
          std::span<AvbdScalarRowDescriptor> frictionDescriptors,
          std::span<std::pair<AvbdScalarRowKey, Eigen::Vector3d>>
              previousFrictionDirectionStorage) {
        normalInventory.reserve(normalDescriptors.size());
        normalInventory.syncActiveRows(normalDescriptors, warmStartOptions);

        for (std::size_t i = 0; i < activeContacts.size(); ++i) {
          const AvbdRigidContactManifoldPoint& contact = activeContacts[i];
          contactLocalPoints[i]
              = {avbdRigidBodyLocalPoint(states[contact.bodyA], contact.point),
                 avbdRigidBodyLocalPoint(states[contact.bodyB], contact.point)};
        }

        const auto frictionForceLimit = [&](std::size_t i) {
          const AvbdRigidContactManifoldPoint& contact = activeContacts[i];
          double laggedNormalForce
              = i < normalInventory.size()
                    ? std::max(0.0, normalInventory[i].state.lambda)
                    : 0.0;
          if (laggedNormalForce <= 0.0) {
            // Moving manifolds can keep penetrating after their row identity
            // changes; keep Coulomb rows bounded by the active normal penalty.
            laggedNormalForce
                = std::max(0.0, contact.startStiffness * contact.depth);
          }
          return std::max(0.0, contact.frictionCoefficient) * laggedNormalForce;
        };

        bool hasPositiveFrictionLimit = false;
        for (std::size_t i = 0; i < activeContacts.size(); ++i) {
          hasPositiveFrictionLimit
              = hasPositiveFrictionLimit || frictionForceLimit(i) > 0.0;
        }

        std::size_t frictionDescriptorCount = 0u;
        if (hasPositiveFrictionLimit) {
          for (std::size_t i = 0; i < activeContacts.size(); ++i) {
            const AvbdRigidContactManifoldPoint& contact = activeContacts[i];
            const double forceLimit = frictionForceLimit(i);
            frictionDescriptors[frictionDescriptorCount++]
                = makeAvbdContactFrictionRowDescriptor(
                    contact.endpointA,
                    contact.endpointB,
                    /*axis=*/0,
                    forceLimit,
                    contact.startStiffness,
                    contact.maxStiffness,
                    contact.row);
            frictionDescriptors[frictionDescriptorCount++]
                = makeAvbdContactFrictionRowDescriptor(
                    contact.endpointA,
                    contact.endpointB,
                    /*axis=*/1,
                    forceLimit,
                    contact.startStiffness,
                    contact.maxStiffness,
                    contact.row);
          }
        }
        const std::span<const AvbdScalarRowDescriptor>
            activeFrictionDescriptors{
                frictionDescriptors.data(), frictionDescriptorCount};

        normalRows.clear();
        normalRows.reserve(normalInventory.size());
        for (std::size_t i = 0;
             i < activeContacts.size() && i < normalInventory.size();
             ++i) {
          const AvbdRigidContactManifoldPoint& contact = activeContacts[i];
          const AvbdScalarRowRecord& record = normalInventory[i];
          const auto& localPoints = contactLocalPoints[i];
          AvbdRigidBodyPointPairRow indexedRow;
          indexedRow.bodyA = contact.bodyA;
          indexedRow.bodyB = contact.bodyB;
          indexedRow.row = makeAvbdRigidContactNormalRow(
              localPoints.first,
              localPoints.second,
              -contact.normalFromAtoB,
              contact.depth,
              record.state,
              contact.depth);
          normalRows.push_back(indexedRow);
        }

        if (activeFrictionDescriptors.empty()) {
          frictionInventory.syncActiveRows(
              activeFrictionDescriptors, warmStartOptions);
          frictionRows.clear();
          return;
        }

        std::span<std::pair<AvbdScalarRowKey, Eigen::Vector3d>>
            previousFrictionDirections;
        if (!previousFrictionDirectionStorage.empty()) {
          std::size_t previousFrictionDirectionCount = 0u;
          for (const AvbdScalarRowDescriptor& descriptor :
               activeFrictionDescriptors) {
            for (const AvbdScalarRowRecord& record :
                 frictionInventory.records()) {
              if (record.descriptor.key != descriptor.key
                  || !isValidAvbdRigidContactFrictionDirection(
                      record.direction)) {
                continue;
              }

              previousFrictionDirectionStorage[previousFrictionDirectionCount++]
                  = {record.descriptor.key, record.direction};
              break;
            }
          }
          previousFrictionDirections
              = {previousFrictionDirectionStorage.data(),
                 previousFrictionDirectionCount};
        } else {
          auto& previousFrictionDirectionScratch
              = scratch.previousFrictionDirections;
          previousFrictionDirectionScratch.clear();
          previousFrictionDirectionScratch.reserve(
              frictionInventory.records().size());
          for (const AvbdScalarRowRecord& record :
               frictionInventory.records()) {
            if (isValidAvbdRigidContactFrictionDirection(record.direction)) {
              previousFrictionDirectionScratch.emplace_back(
                  record.descriptor.key, record.direction);
            }
          }
          previousFrictionDirections = previousFrictionDirectionScratch;
        }
        std::sort(
            previousFrictionDirections.begin(),
            previousFrictionDirections.end(),
            [](const auto& lhs, const auto& rhs) {
              return lhs.first < rhs.first;
            });
        const auto findPreviousFrictionDirection
            = [&previousFrictionDirections](
                  const AvbdScalarRowKey& key) -> const Eigen::Vector3d* {
          const auto found = std::lower_bound(
              previousFrictionDirections.begin(),
              previousFrictionDirections.end(),
              key,
              [](const auto& value, const AvbdScalarRowKey& target) {
                return value.first < target;
              });
          if (found == previousFrictionDirections.end()
              || found->first != key) {
            return nullptr;
          }
          return &found->second;
        };

        frictionInventory.reserve(frictionDescriptorCount);
        frictionInventory.syncActiveRows(
            activeFrictionDescriptors, warmStartOptions);

        frictionRows.clear();
        frictionRows.reserve(activeContacts.size());
        for (std::size_t contactIndex = 0; contactIndex < activeContacts.size();
             ++contactIndex) {
          const std::size_t firstRecordIndex = 2 * contactIndex;
          const std::size_t secondRecordIndex = firstRecordIndex + 1;
          if (secondRecordIndex >= frictionInventory.size()) {
            break;
          }

          const AvbdRigidContactManifoldPoint& contact
              = activeContacts[contactIndex];
          AvbdScalarRowRecord& firstRecord
              = frictionInventory[firstRecordIndex];
          AvbdScalarRowRecord& secondRecord
              = frictionInventory[secondRecordIndex];
          const auto& localPoints = contactLocalPoints[contactIndex];
          const Eigen::Vector3d stepStartRelativePosition
              = Eigen::Vector3d::Zero();
          const Eigen::Matrix<double, 3, 2> basis
              = avbdRigidContactTangentBasis(contact.normalFromAtoB);
          if (const Eigen::Vector3d* previousFirst
              = findPreviousFrictionDirection(firstRecord.descriptor.key)) {
            if (const Eigen::Vector3d* previousSecond
                = findPreviousFrictionDirection(secondRecord.descriptor.key)) {
              const Eigen::Vector2d projected
                  = projectAvbdFrictionDualToTangentPair(
                      firstRecord.state.lambda,
                      secondRecord.state.lambda,
                      *previousFirst,
                      *previousSecond,
                      basis.col(0),
                      basis.col(1));
              firstRecord.state.lambda = clampAvbdRowForce(
                  projected.x(), firstRecord.descriptor.bounds);
              secondRecord.state.lambda = clampAvbdRowForce(
                  projected.y(), secondRecord.descriptor.bounds);
            }
          }
          firstRecord.direction = basis.col(0);
          secondRecord.direction = basis.col(1);
          const auto forceLimitFromBounds = [](AvbdScalarRowBounds bounds) {
            const double lowerLimit = bounds.lower < 0.0 ? -bounds.lower : 0.0;
            const double upperLimit = bounds.upper > 0.0 ? bounds.upper : 0.0;
            return std::max(0.0, std::min(lowerLimit, upperLimit));
          };

          AvbdRigidBodyPointPairFrictionRows indexedRows;
          indexedRows.bodyA = contact.bodyA;
          indexedRows.bodyB = contact.bodyB;
          indexedRows.first = makeAvbdRigidContactFrictionTangentRow(
              localPoints.first,
              localPoints.second,
              basis.col(0),
              stepStartRelativePosition,
              forceLimitFromBounds(firstRecord.descriptor.bounds),
              firstRecord.state,
              0.0);
          indexedRows.first.bounds = firstRecord.descriptor.bounds;
          indexedRows.second = makeAvbdRigidContactFrictionTangentRow(
              localPoints.first,
              localPoints.second,
              basis.col(1),
              stepStartRelativePosition,
              forceLimitFromBounds(secondRecord.descriptor.bounds),
              secondRecord.state,
              0.0);
          indexedRows.second.bounds = secondRecord.descriptor.bounds;
          frictionRows.push_back(indexedRows);
        }
      };

  if (contacts.size() <= detail::kAvbdRigidSmallRowStackCapacity) {
    std::array<
        AvbdRigidContactManifoldPoint,
        detail::kAvbdRigidSmallRowStackCapacity>
        activeContacts;
    std::array<AvbdScalarRowDescriptor, detail::kAvbdRigidSmallRowStackCapacity>
        normalDescriptors;
    std::array<
        std::pair<Eigen::Vector3d, Eigen::Vector3d>,
        detail::kAvbdRigidSmallRowStackCapacity>
        contactLocalPoints;
    std::array<
        AvbdScalarRowDescriptor,
        2u * detail::kAvbdRigidSmallRowStackCapacity>
        frictionDescriptors;
    std::array<
        std::pair<AvbdScalarRowKey, Eigen::Vector3d>,
        2u * detail::kAvbdRigidSmallRowStackCapacity>
        previousFrictionDirections;
    std::size_t activeContactCount = 0u;
    for (const AvbdRigidContactManifoldPoint& contact : contacts) {
      if (!detail::isValidAvbdRigidContactManifoldPoint(
              contact, states.size())) {
        continue;
      }

      activeContacts[activeContactCount] = contact;
      normalDescriptors[activeContactCount]
          = makeAvbdContactNormalRowDescriptor(
              contact.endpointA,
              contact.endpointB,
              contact.startStiffness,
              contact.maxStiffness,
              contact.row);
      ++activeContactCount;
    }

    appendActiveRows(
        std::span<const AvbdRigidContactManifoldPoint>{
            activeContacts.data(), activeContactCount},
        std::span<const AvbdScalarRowDescriptor>{
            normalDescriptors.data(), activeContactCount},
        std::span<std::pair<Eigen::Vector3d, Eigen::Vector3d>>{
            contactLocalPoints.data(), activeContactCount},
        std::span<AvbdScalarRowDescriptor>{
            frictionDescriptors.data(), 2u * activeContactCount},
        std::span<std::pair<AvbdScalarRowKey, Eigen::Vector3d>>{
            previousFrictionDirections.data(),
            previousFrictionDirections.size()});
    return;
  }

  auto& activeContacts = scratch.activeContacts;
  activeContacts.clear();
  activeContacts.reserve(contacts.size());
  auto& normalDescriptors = scratch.normalDescriptors;
  normalDescriptors.clear();
  normalDescriptors.reserve(contacts.size());
  for (const AvbdRigidContactManifoldPoint& contact : contacts) {
    if (!detail::isValidAvbdRigidContactManifoldPoint(contact, states.size())) {
      continue;
    }

    activeContacts.push_back(contact);
    normalDescriptors.push_back(makeAvbdContactNormalRowDescriptor(
        contact.endpointA,
        contact.endpointB,
        contact.startStiffness,
        contact.maxStiffness,
        contact.row));
  }

  auto& contactLocalPoints = scratch.contactLocalPoints;
  contactLocalPoints.resize(activeContacts.size());
  auto& frictionDescriptors = scratch.frictionDescriptors;
  frictionDescriptors.resize(2u * activeContacts.size());
  appendActiveRows(
      activeContacts,
      normalDescriptors,
      contactLocalPoints,
      frictionDescriptors,
      {});
}

//==============================================================================
inline void buildAvbdRigidContactManifoldRows(
    const std::vector<AvbdRigidBodyState>& states,
    std::span<const AvbdRigidContactManifoldPoint> contacts,
    AvbdScalarRowInventory& normalInventory,
    AvbdScalarRowInventory& frictionInventory,
    std::vector<AvbdRigidBodyPointPairRow>& normalRows,
    std::vector<AvbdRigidBodyPointPairFrictionRows>& frictionRows,
    const AvbdRowWarmStartOptions& warmStartOptions = {})
{
  AvbdRigidContactManifoldRowScratch scratch;
  buildAvbdRigidContactManifoldRows(
      states,
      contacts,
      normalInventory,
      frictionInventory,
      normalRows,
      frictionRows,
      scratch,
      warmStartOptions);
}

//==============================================================================
inline void buildAvbdRigidPointJointRows(
    const std::vector<AvbdRigidBodyState>& states,
    std::span<const AvbdRigidPointJoint> joints,
    AvbdScalarRowInventory& linearInventory,
    std::vector<AvbdRigidBodyPointPairRow>& linearRows,
    AvbdRigidPointJointRowScratch& scratch,
    const AvbdRowWarmStartOptions& warmStartOptions = {})
{
  const auto appendLinearRows = [&](const auto& activeRows,
                                    std::size_t activeRowCount) {
    linearRows.clear();
    linearRows.reserve(linearInventory.size());
    for (std::size_t recordIndex = 0; recordIndex < activeRowCount;
         ++recordIndex) {
      if (recordIndex >= linearInventory.size()) {
        return;
      }

      const AvbdRigidPointJoint& joint = *activeRows[recordIndex].joint;
      const std::uint8_t axis = activeRows[recordIndex].axis;
      const AvbdScalarRowRecord& record = linearInventory[recordIndex];
      AvbdRigidBodyPointPairRow indexedRow;
      indexedRow.bodyA = joint.bodyA;
      indexedRow.bodyB = joint.bodyB;
      indexedRow.row.localPointA = joint.localPointA;
      indexedRow.row.localPointB = joint.localPointB;
      indexedRow.row.axis = normalizedAvbdRigidPointPairAxis(
          joint.linearAxes.col(axis), Eigen::Vector3d::Unit(axis));
      indexedRow.row.state = record.state;
      indexedRow.row.materialStiffness = record.descriptor.materialStiffness;
      indexedRow.row.bounds = record.descriptor.bounds;
      if (!avbdRigidRowUsesFiniteMaterial(indexedRow.row.materialStiffness)) {
        indexedRow.row.previousConstraintValue
            = avbdRigidPointPairConstraintValue(
                states[joint.bodyA], states[joint.bodyB], indexedRow.row);
      }
      linearRows.push_back(indexedRow);
    }
  };

  const std::size_t maxActiveRows = 3 * joints.size();
  if (maxActiveRows <= detail::kAvbdRigidSmallRowStackCapacity) {
    std::array<
        AvbdRigidPointJointActiveAxis,
        detail::kAvbdRigidSmallRowStackCapacity>
        activeRows;
    std::array<AvbdScalarRowDescriptor, detail::kAvbdRigidSmallRowStackCapacity>
        descriptors;
    std::size_t activeRowCount = 0u;
    for (const AvbdRigidPointJoint& joint : joints) {
      if (!detail::isValidAvbdRigidPointJoint(joint, states.size())) {
        continue;
      }

      for (std::uint8_t axis = 0; axis < 3u; ++axis) {
        if (!detail::avbdRigidJointAxisEnabled(joint.linearAxisMask, axis)) {
          continue;
        }

        activeRows[activeRowCount] = AvbdRigidPointJointActiveAxis{
            &joint,
            axis,
        };
        descriptors[activeRowCount]
            = detail::makeAvbdRigidJointLinearRowDescriptor(
                joint.endpointA,
                joint.endpointB,
                joint.startStiffness,
                joint.linearMaterialStiffness,
                joint.maxStiffness,
                joint.row,
                axis);
        ++activeRowCount;
      }
    }

    linearInventory.syncActiveRows(
        std::span<const AvbdScalarRowDescriptor>{
            descriptors.data(), activeRowCount},
        warmStartOptions);
    appendLinearRows(activeRows, activeRowCount);
    return;
  }

  auto& activeRows = scratch.activeRows;
  activeRows.clear();
  activeRows.reserve(maxActiveRows);
  auto& descriptors = scratch.descriptors;
  descriptors.clear();
  descriptors.reserve(maxActiveRows);
  for (const AvbdRigidPointJoint& joint : joints) {
    if (!detail::isValidAvbdRigidPointJoint(joint, states.size())) {
      continue;
    }

    for (std::uint8_t axis = 0; axis < 3u; ++axis) {
      if (!detail::avbdRigidJointAxisEnabled(joint.linearAxisMask, axis)) {
        continue;
      }

      activeRows.push_back(AvbdRigidPointJointActiveAxis{&joint, axis});
      descriptors.push_back(
          detail::makeAvbdRigidJointLinearRowDescriptor(
              joint.endpointA,
              joint.endpointB,
              joint.startStiffness,
              joint.linearMaterialStiffness,
              joint.maxStiffness,
              joint.row,
              axis));
    }
  }

  linearInventory.reserve(descriptors.size());
  linearInventory.syncActiveRows(descriptors, warmStartOptions);
  appendLinearRows(activeRows, activeRows.size());
}

//==============================================================================
inline void buildAvbdRigidPointJointRows(
    const std::vector<AvbdRigidBodyState>& states,
    std::span<const AvbdRigidPointJoint> joints,
    AvbdScalarRowInventory& linearInventory,
    std::vector<AvbdRigidBodyPointPairRow>& linearRows,
    const AvbdRowWarmStartOptions& warmStartOptions = {})
{
  AvbdRigidPointJointRowScratch scratch;
  buildAvbdRigidPointJointRows(
      states, joints, linearInventory, linearRows, scratch, warmStartOptions);
}

//==============================================================================
inline void buildAvbdRigidPointJointAngularRows(
    const std::vector<AvbdRigidBodyState>& states,
    std::span<const AvbdRigidPointJoint> joints,
    AvbdScalarRowInventory& angularInventory,
    std::vector<AvbdRigidBodyAngularPairRow>& angularRows,
    AvbdRigidPointJointRowScratch& scratch,
    const AvbdRowWarmStartOptions& warmStartOptions = {})
{
  const auto appendAngularRows = [&](const auto& activeRows,
                                     std::size_t activeRowCount) {
    angularRows.clear();
    angularRows.reserve(angularInventory.size());
    for (std::size_t recordIndex = 0; recordIndex < activeRowCount;
         ++recordIndex) {
      if (recordIndex >= angularInventory.size()) {
        return;
      }

      const AvbdRigidPointJoint& joint = *activeRows[recordIndex].joint;
      const std::uint8_t axis = activeRows[recordIndex].axis;
      const AvbdScalarRowRecord& record = angularInventory[recordIndex];
      AvbdRigidBodyAngularPairRow indexedRow;
      indexedRow.bodyA = joint.bodyA;
      indexedRow.bodyB = joint.bodyB;
      indexedRow.row = makeAvbdRigidJointAngularRow(
          joint.targetRelativeOrientation,
          joint.angularAxes.col(axis),
          record.state);
      indexedRow.row.materialStiffness = record.descriptor.materialStiffness;
      indexedRow.row.bounds = record.descriptor.bounds;
      if (!avbdRigidRowUsesFiniteMaterial(indexedRow.row.materialStiffness)) {
        indexedRow.row.previousConstraintValue
            = avbdRigidAngularPairConstraintValue(
                states[joint.bodyA], states[joint.bodyB], indexedRow.row);
      }
      angularRows.push_back(indexedRow);
    }
  };

  const std::size_t maxActiveRows = 3 * joints.size();
  if (maxActiveRows <= detail::kAvbdRigidSmallRowStackCapacity) {
    std::array<
        AvbdRigidPointJointActiveAxis,
        detail::kAvbdRigidSmallRowStackCapacity>
        activeRows;
    std::array<AvbdScalarRowDescriptor, detail::kAvbdRigidSmallRowStackCapacity>
        descriptors;
    std::size_t activeRowCount = 0u;
    for (const AvbdRigidPointJoint& joint : joints) {
      if (!detail::isValidAvbdRigidPointJoint(joint, states.size())) {
        continue;
      }

      for (std::uint8_t axis = 0; axis < 3u; ++axis) {
        if (!detail::avbdRigidJointAxisEnabled(joint.angularAxisMask, axis)) {
          continue;
        }

        activeRows[activeRowCount] = AvbdRigidPointJointActiveAxis{
            &joint,
            axis,
        };
        descriptors[activeRowCount]
            = detail::makeAvbdRigidJointAngularRowDescriptor(
                joint.endpointA,
                joint.endpointB,
                joint.startStiffness,
                joint.angularMaterialStiffness,
                joint.maxStiffness,
                joint.row,
                axis);
        ++activeRowCount;
      }
    }

    angularInventory.syncActiveRows(
        std::span<const AvbdScalarRowDescriptor>{
            descriptors.data(), activeRowCount},
        warmStartOptions);
    appendAngularRows(activeRows, activeRowCount);
    return;
  }

  auto& activeRows = scratch.activeRows;
  activeRows.clear();
  activeRows.reserve(maxActiveRows);
  auto& descriptors = scratch.descriptors;
  descriptors.clear();
  descriptors.reserve(maxActiveRows);
  for (const AvbdRigidPointJoint& joint : joints) {
    if (!detail::isValidAvbdRigidPointJoint(joint, states.size())) {
      continue;
    }

    for (std::uint8_t axis = 0; axis < 3u; ++axis) {
      if (!detail::avbdRigidJointAxisEnabled(joint.angularAxisMask, axis)) {
        continue;
      }

      activeRows.push_back(AvbdRigidPointJointActiveAxis{&joint, axis});
      descriptors.push_back(
          detail::makeAvbdRigidJointAngularRowDescriptor(
              joint.endpointA,
              joint.endpointB,
              joint.startStiffness,
              joint.angularMaterialStiffness,
              joint.maxStiffness,
              joint.row,
              axis));
    }
  }

  angularInventory.reserve(descriptors.size());
  angularInventory.syncActiveRows(descriptors, warmStartOptions);
  appendAngularRows(activeRows, activeRows.size());
}

//==============================================================================
inline void buildAvbdRigidPointJointAngularRows(
    const std::vector<AvbdRigidBodyState>& states,
    std::span<const AvbdRigidPointJoint> joints,
    AvbdScalarRowInventory& angularInventory,
    std::vector<AvbdRigidBodyAngularPairRow>& angularRows,
    const AvbdRowWarmStartOptions& warmStartOptions = {})
{
  AvbdRigidPointJointRowScratch scratch;
  buildAvbdRigidPointJointAngularRows(
      states, joints, angularInventory, angularRows, scratch, warmStartOptions);
}

//==============================================================================
inline void buildAvbdRigidAngularMotorRows(
    const std::vector<AvbdRigidBodyState>& states,
    std::span<const AvbdRigidAngularMotor> motors,
    AvbdScalarRowInventory& motorInventory,
    std::vector<AvbdRigidBodyAngularPairRow>& motorRows,
    double timeStep,
    AvbdRigidAngularMotorRowScratch& scratch,
    const AvbdRowWarmStartOptions& warmStartOptions = {})
{
  const auto appendMotorRows
      = [&](const auto& activeRows, std::size_t activeRowCount) {
          motorRows.clear();
          motorRows.reserve(motorInventory.size());
          for (std::size_t recordIndex = 0; recordIndex < activeRowCount;
               ++recordIndex) {
            if (recordIndex >= motorInventory.size()) {
              return;
            }

            const AvbdRigidAngularMotor& motor = *activeRows[recordIndex];
            const AvbdScalarRowRecord& record = motorInventory[recordIndex];
            AvbdRigidBodyAngularPairRow indexedRow;
            indexedRow.bodyA = motor.bodyA;
            indexedRow.bodyB = motor.bodyB;
            indexedRow.row = makeAvbdRigidAngularMotorRow(
                motor.targetRelativeOrientation,
                motor.axis,
                motor.targetSpeed,
                timeStep,
                record.state);
            indexedRow.row.bounds = record.descriptor.bounds;
            motorRows.push_back(indexedRow);
          }
        };

  if (motors.size() <= detail::kAvbdRigidSmallRowStackCapacity) {
    std::array<
        const AvbdRigidAngularMotor*,
        detail::kAvbdRigidSmallRowStackCapacity>
        activeRows;
    std::array<AvbdScalarRowDescriptor, detail::kAvbdRigidSmallRowStackCapacity>
        descriptors;
    std::size_t activeRowCount = 0u;
    for (const AvbdRigidAngularMotor& motor : motors) {
      if (!detail::isValidAvbdRigidAngularMotor(
              motor, states.size(), timeStep)) {
        continue;
      }

      activeRows[activeRowCount] = &motor;
      descriptors[activeRowCount]
          = detail::makeAvbdRigidAngularMotorRowDescriptor(
              motor.endpointA,
              motor.endpointB,
              motor.maxTorque,
              motor.startStiffness,
              motor.maxStiffness,
              motor.row);
      ++activeRowCount;
    }

    motorInventory.syncActiveRows(
        std::span<const AvbdScalarRowDescriptor>{
            descriptors.data(), activeRowCount},
        warmStartOptions);
    appendMotorRows(activeRows, activeRowCount);
    return;
  }

  auto& activeRows = scratch.activeRows;
  activeRows.clear();
  activeRows.reserve(motors.size());
  auto& descriptors = scratch.descriptors;
  descriptors.clear();
  descriptors.reserve(motors.size());
  for (const AvbdRigidAngularMotor& motor : motors) {
    if (!detail::isValidAvbdRigidAngularMotor(motor, states.size(), timeStep)) {
      continue;
    }

    activeRows.push_back(&motor);
    descriptors.push_back(
        detail::makeAvbdRigidAngularMotorRowDescriptor(
            motor.endpointA,
            motor.endpointB,
            motor.maxTorque,
            motor.startStiffness,
            motor.maxStiffness,
            motor.row));
  }

  motorInventory.reserve(descriptors.size());
  motorInventory.syncActiveRows(descriptors, warmStartOptions);
  appendMotorRows(activeRows, activeRows.size());
}

//==============================================================================
inline void buildAvbdRigidAngularMotorRows(
    const std::vector<AvbdRigidBodyState>& states,
    std::span<const AvbdRigidAngularMotor> motors,
    AvbdScalarRowInventory& motorInventory,
    std::vector<AvbdRigidBodyAngularPairRow>& motorRows,
    double timeStep,
    const AvbdRowWarmStartOptions& warmStartOptions = {})
{
  AvbdRigidAngularMotorRowScratch scratch;
  buildAvbdRigidAngularMotorRows(
      states,
      motors,
      motorInventory,
      motorRows,
      timeStep,
      scratch,
      warmStartOptions);
}

//==============================================================================
inline void buildAvbdRigidMotorRows(
    const std::vector<AvbdRigidBodyState>& states,
    std::span<const AvbdRigidLinearMotor> linearMotors,
    std::span<const AvbdRigidAngularMotor> angularMotors,
    AvbdScalarRowInventory& motorInventory,
    std::vector<AvbdRigidBodyPointPairRow>& linearMotorRows,
    std::vector<AvbdRigidBodyAngularPairRow>& angularMotorRows,
    double timeStep,
    AvbdRigidMotorRowScratch& scratch,
    const AvbdRowWarmStartOptions& warmStartOptions = {})
{
  const auto appendMotorRows = [&](const auto& activeLinearRows,
                                   std::size_t activeLinearCount,
                                   const auto& activeAngularRows,
                                   std::size_t activeAngularCount) {
    linearMotorRows.clear();
    linearMotorRows.reserve(activeLinearCount);
    std::size_t recordIndex = 0;
    for (std::size_t motorIndex = 0; motorIndex < activeLinearCount;
         ++motorIndex) {
      if (recordIndex >= motorInventory.size()) {
        return;
      }

      const AvbdRigidLinearMotor& motor = *activeLinearRows[motorIndex];
      const AvbdScalarRowRecord& record = motorInventory[recordIndex++];
      AvbdRigidBodyPointPairRow indexedRow;
      indexedRow.bodyA = motor.bodyA;
      indexedRow.bodyB = motor.bodyB;
      const Eigen::Vector3d stepStartRelativePosition
          = avbdRigidBodyWorldPoint(states[motor.bodyB], motor.localPointB)
            - avbdRigidBodyWorldPoint(states[motor.bodyA], motor.localPointA);
      indexedRow.row = makeAvbdRigidLinearMotorRow(
          motor.localPointA,
          motor.localPointB,
          motor.axis,
          stepStartRelativePosition,
          motor.targetSpeed,
          timeStep,
          record.state);
      indexedRow.row.bounds = record.descriptor.bounds;
      linearMotorRows.push_back(indexedRow);
    }

    angularMotorRows.clear();
    angularMotorRows.reserve(activeAngularCount);
    for (std::size_t motorIndex = 0; motorIndex < activeAngularCount;
         ++motorIndex) {
      if (recordIndex >= motorInventory.size()) {
        return;
      }

      const AvbdRigidAngularMotor& motor = *activeAngularRows[motorIndex];
      const AvbdScalarRowRecord& record = motorInventory[recordIndex++];
      AvbdRigidBodyAngularPairRow indexedRow;
      indexedRow.bodyA = motor.bodyA;
      indexedRow.bodyB = motor.bodyB;
      indexedRow.row = makeAvbdRigidAngularMotorRow(
          motor.targetRelativeOrientation,
          motor.axis,
          motor.targetSpeed,
          timeStep,
          record.state);
      indexedRow.row.bounds = record.descriptor.bounds;
      angularMotorRows.push_back(indexedRow);
    }
  };

  const std::size_t maxActiveRows = linearMotors.size() + angularMotors.size();
  if (maxActiveRows <= detail::kAvbdRigidSmallRowStackCapacity) {
    std::array<
        const AvbdRigidLinearMotor*,
        detail::kAvbdRigidSmallRowStackCapacity>
        activeLinearRows;
    std::array<
        const AvbdRigidAngularMotor*,
        detail::kAvbdRigidSmallRowStackCapacity>
        activeAngularRows;
    std::array<AvbdScalarRowDescriptor, detail::kAvbdRigidSmallRowStackCapacity>
        descriptors;
    std::size_t activeLinearCount = 0u;
    std::size_t activeAngularCount = 0u;
    std::size_t descriptorCount = 0u;
    for (const AvbdRigidLinearMotor& motor : linearMotors) {
      if (!detail::isValidAvbdRigidLinearMotor(
              motor, states.size(), timeStep)) {
        continue;
      }

      activeLinearRows[activeLinearCount++] = &motor;
      descriptors[descriptorCount++]
          = detail::makeAvbdRigidLinearMotorRowDescriptor(
              motor.endpointA,
              motor.endpointB,
              motor.maxForce,
              motor.startStiffness,
              motor.maxStiffness,
              motor.row);
    }
    for (const AvbdRigidAngularMotor& motor : angularMotors) {
      if (!detail::isValidAvbdRigidAngularMotor(
              motor, states.size(), timeStep)) {
        continue;
      }

      activeAngularRows[activeAngularCount++] = &motor;
      descriptors[descriptorCount++]
          = detail::makeAvbdRigidAngularMotorRowDescriptor(
              motor.endpointA,
              motor.endpointB,
              motor.maxTorque,
              motor.startStiffness,
              motor.maxStiffness,
              motor.row);
    }

    motorInventory.syncActiveRows(
        std::span<const AvbdScalarRowDescriptor>{
            descriptors.data(), descriptorCount},
        warmStartOptions);
    appendMotorRows(
        activeLinearRows,
        activeLinearCount,
        activeAngularRows,
        activeAngularCount);
    return;
  }

  auto& activeLinearRows = scratch.activeLinearRows;
  activeLinearRows.clear();
  activeLinearRows.reserve(linearMotors.size());
  auto& activeAngularRows = scratch.activeAngularRows;
  activeAngularRows.clear();
  activeAngularRows.reserve(angularMotors.size());
  for (const AvbdRigidLinearMotor& motor : linearMotors) {
    if (!detail::isValidAvbdRigidLinearMotor(motor, states.size(), timeStep)) {
      continue;
    }

    activeLinearRows.push_back(&motor);
  }
  for (const AvbdRigidAngularMotor& motor : angularMotors) {
    if (!detail::isValidAvbdRigidAngularMotor(motor, states.size(), timeStep)) {
      continue;
    }

    activeAngularRows.push_back(&motor);
  }

  motorInventory.syncActiveRowsByIndex(
      activeLinearRows.size() + activeAngularRows.size(),
      [&](std::size_t index) {
        if (index < activeLinearRows.size()) {
          const AvbdRigidLinearMotor& motor = *activeLinearRows[index];
          return makeAvbdEndpointPairRowKey(
              AvbdScalarRowRole::Motor,
              motor.endpointA,
              motor.endpointB,
              motor.row,
              /*axis=*/0);
        }

        const AvbdRigidAngularMotor& motor
            = *activeAngularRows[index - activeLinearRows.size()];
        return makeAvbdEndpointPairRowKey(
            AvbdScalarRowRole::Motor,
            motor.endpointA,
            motor.endpointB,
            motor.row,
            /*axis=*/0);
      },
      [&](std::size_t index) {
        if (index < activeLinearRows.size()) {
          const AvbdRigidLinearMotor& motor = *activeLinearRows[index];
          return detail::makeAvbdRigidLinearMotorRowDescriptor(
              motor.endpointA,
              motor.endpointB,
              motor.maxForce,
              motor.startStiffness,
              motor.maxStiffness,
              motor.row);
        }

        const AvbdRigidAngularMotor& motor
            = *activeAngularRows[index - activeLinearRows.size()];
        return detail::makeAvbdRigidAngularMotorRowDescriptor(
            motor.endpointA,
            motor.endpointB,
            motor.maxTorque,
            motor.startStiffness,
            motor.maxStiffness,
            motor.row);
      },
      warmStartOptions);
  appendMotorRows(
      activeLinearRows,
      activeLinearRows.size(),
      activeAngularRows,
      activeAngularRows.size());
}

//==============================================================================
inline void buildAvbdRigidMotorRows(
    const std::vector<AvbdRigidBodyState>& states,
    std::span<const AvbdRigidLinearMotor> linearMotors,
    std::span<const AvbdRigidAngularMotor> angularMotors,
    AvbdScalarRowInventory& motorInventory,
    std::vector<AvbdRigidBodyPointPairRow>& linearMotorRows,
    std::vector<AvbdRigidBodyAngularPairRow>& angularMotorRows,
    double timeStep,
    const AvbdRowWarmStartOptions& warmStartOptions = {})
{
  AvbdRigidMotorRowScratch scratch;
  buildAvbdRigidMotorRows(
      states,
      linearMotors,
      angularMotors,
      motorInventory,
      linearMotorRows,
      angularMotorRows,
      timeStep,
      scratch,
      warmStartOptions);
}

//==============================================================================
inline void buildAvbdRigidPointJointConstraintRows(
    const std::vector<AvbdRigidBodyState>& states,
    std::span<const AvbdRigidPointJoint> joints,
    AvbdScalarRowInventory& linearInventory,
    AvbdScalarRowInventory& angularInventory,
    std::vector<AvbdRigidBodyPointPairRow>& linearRows,
    std::vector<AvbdRigidBodyAngularPairRow>& angularRows,
    AvbdRigidPointJointRowScratch& linearScratch,
    AvbdRigidPointJointRowScratch& angularScratch,
    const AvbdRowWarmStartOptions& warmStartOptions = {})
{
  buildAvbdRigidPointJointRows(
      states,
      joints,
      linearInventory,
      linearRows,
      linearScratch,
      warmStartOptions);
  buildAvbdRigidPointJointAngularRows(
      states,
      joints,
      angularInventory,
      angularRows,
      angularScratch,
      warmStartOptions);
}

//==============================================================================
inline void buildAvbdRigidPointJointConstraintRows(
    const std::vector<AvbdRigidBodyState>& states,
    std::span<const AvbdRigidPointJoint> joints,
    AvbdScalarRowInventory& linearInventory,
    AvbdScalarRowInventory& angularInventory,
    std::vector<AvbdRigidBodyPointPairRow>& linearRows,
    std::vector<AvbdRigidBodyAngularPairRow>& angularRows,
    const AvbdRowWarmStartOptions& warmStartOptions = {})
{
  AvbdRigidPointJointRowScratch linearScratch;
  AvbdRigidPointJointRowScratch angularScratch;
  buildAvbdRigidPointJointConstraintRows(
      states,
      joints,
      linearInventory,
      angularInventory,
      linearRows,
      angularRows,
      linearScratch,
      angularScratch,
      warmStartOptions);
}

//==============================================================================
inline void buildAvbdRigidDistanceSpringRows(
    const std::vector<AvbdRigidBodyState>& states,
    std::span<const AvbdRigidBodyPointPairDistanceSpringRow> springs,
    AvbdScalarRowInventory& springInventory,
    std::vector<AvbdRigidBodyPointPairDistanceSpringRow>& springRows,
    AvbdRigidDistanceSpringRowScratch& scratch,
    const AvbdRowWarmStartOptions& warmStartOptions = {})
{
  const auto appendSpringRows = [&](const auto& activeRows,
                                    std::size_t activeRowCount) {
    springRows.clear();
    springRows.reserve(springInventory.size());
    for (std::size_t recordIndex = 0; recordIndex < activeRowCount;
         ++recordIndex) {
      if (recordIndex >= springInventory.size()) {
        return;
      }

      AvbdRigidBodyPointPairDistanceSpringRow indexedRow
          = *activeRows[recordIndex];
      const AvbdScalarRowRecord& record = springInventory[recordIndex];
      indexedRow.row.state = record.state;
      indexedRow.row.materialStiffness = record.descriptor.materialStiffness;
      springRows.push_back(indexedRow);
    }
  };

  if (springs.size() <= detail::kAvbdRigidSmallRowStackCapacity) {
    std::array<
        const AvbdRigidBodyPointPairDistanceSpringRow*,
        detail::kAvbdRigidSmallRowStackCapacity>
        activeRows;
    std::array<AvbdScalarRowDescriptor, detail::kAvbdRigidSmallRowStackCapacity>
        descriptors;
    std::size_t activeRowCount = 0u;
    for (const AvbdRigidBodyPointPairDistanceSpringRow& spring : springs) {
      if (!detail::isValidAvbdRigidDistanceSpring(spring, states.size())) {
        continue;
      }

      activeRows[activeRowCount] = &spring;
      descriptors[activeRowCount]
          = detail::makeAvbdRigidDistanceSpringRowDescriptor(
              spring.endpointA,
              spring.endpointB,
              spring.startStiffness,
              spring.row.materialStiffness,
              spring.maxStiffness,
              spring.rowIndex);
      ++activeRowCount;
    }

    springInventory.syncActiveRows(
        std::span<const AvbdScalarRowDescriptor>{
            descriptors.data(), activeRowCount},
        warmStartOptions);
    appendSpringRows(activeRows, activeRowCount);
    return;
  }

  auto& activeRows = scratch.activeRows;
  activeRows.clear();
  activeRows.reserve(springs.size());
  for (const AvbdRigidBodyPointPairDistanceSpringRow& spring : springs) {
    if (!detail::isValidAvbdRigidDistanceSpring(spring, states.size())) {
      continue;
    }

    activeRows.push_back(&spring);
  }

  springInventory.syncActiveRowsByIndex(
      activeRows.size(),
      [&](std::size_t index) {
        const AvbdRigidBodyPointPairDistanceSpringRow& spring
            = *activeRows[index];
        return makeAvbdEndpointPairRowKey(
            AvbdScalarRowRole::RigidDistanceSpring,
            spring.endpointA,
            spring.endpointB,
            spring.rowIndex,
            /*axis=*/0);
      },
      [&](std::size_t index) {
        const AvbdRigidBodyPointPairDistanceSpringRow& spring
            = *activeRows[index];
        return detail::makeAvbdRigidDistanceSpringRowDescriptor(
            spring.endpointA,
            spring.endpointB,
            spring.startStiffness,
            spring.row.materialStiffness,
            spring.maxStiffness,
            spring.rowIndex);
      },
      warmStartOptions);
  appendSpringRows(activeRows, activeRows.size());
}

//==============================================================================
inline void buildAvbdRigidDistanceSpringRows(
    const std::vector<AvbdRigidBodyState>& states,
    std::span<const AvbdRigidBodyPointPairDistanceSpringRow> springs,
    AvbdScalarRowInventory& springInventory,
    std::vector<AvbdRigidBodyPointPairDistanceSpringRow>& springRows,
    const AvbdRowWarmStartOptions& warmStartOptions = {})
{
  AvbdRigidDistanceSpringRowScratch scratch;
  buildAvbdRigidDistanceSpringRows(
      states, springs, springInventory, springRows, scratch, warmStartOptions);
}

//==============================================================================
inline std::uint64_t avbdRigidBodyRowKey(std::uint32_t body) noexcept
{
  return static_cast<std::uint64_t>(body);
}

//==============================================================================
inline std::uint64_t avbdRigidBodyPairRowKey(
    std::uint32_t bodyA, std::uint32_t bodyB) noexcept
{
  return (static_cast<std::uint64_t>(bodyA) << 32u)
         | static_cast<std::uint64_t>(bodyB);
}

//==============================================================================
template <typename Rows, typename KeyFn>
inline bool avbdRigidRowIndexLayoutMatches(
    const Rows& rows,
    std::size_t bodyCount,
    std::size_t cachedBodyCount,
    const std::vector<std::uint64_t>& cachedKeys,
    const std::vector<std::size_t>& offsets,
    KeyFn keyOf)
{
  if (cachedBodyCount != bodyCount || offsets.size() != bodyCount + 1u
      || cachedKeys.size() != rows.size()) {
    return false;
  }

  for (std::size_t rowIndex = 0; rowIndex < rows.size(); ++rowIndex) {
    if (cachedKeys[rowIndex] != keyOf(rows[rowIndex])) {
      return false;
    }
  }
  return true;
}

//==============================================================================
template <typename Rows, typename KeyFn>
inline void avbdRigidRefreshRowIndexLayoutKeys(
    const Rows& rows,
    std::size_t bodyCount,
    std::size_t& cachedBodyCount,
    std::vector<std::uint64_t>& cachedKeys,
    KeyFn keyOf)
{
  cachedBodyCount = bodyCount;
  cachedKeys.clear();
  cachedKeys.reserve(rows.size());
  for (std::size_t rowIndex = 0; rowIndex < rows.size(); ++rowIndex) {
    cachedKeys.push_back(keyOf(rows[rowIndex]));
  }
}

//==============================================================================
inline AvbdRigidBlockDescentStats blockDescentRigidBodiesAvbdRows(
    std::vector<AvbdRigidBodyState>& states,
    const std::vector<double>& masses,
    const std::vector<Eigen::Matrix3d>& bodyInertias,
    const std::vector<std::uint8_t>& fixed,
    std::span<const AvbdRigidBodyState> inertialTargets,
    double timeStep,
    std::vector<AvbdRigidBodyPointAttachmentRow>& attachmentRows,
    std::vector<AvbdRigidBodyPointPairRow>& pointPairRows,
    std::vector<AvbdRigidBodyAngularPairRow>& angularPairRows,
    std::vector<AvbdRigidBodyPointPairFrictionRows>& frictionPairRows,
    const AvbdRigidBlockDescentOptions& options,
    const AvbdRigidPointAttachmentOptions& rowOptions,
    const AvbdRigidPointPairFrictionOptions& frictionOptions,
    AvbdRigidBodyRowIndexScratch* rowIndexScratch = nullptr,
    std::span<AvbdRigidBodyPointPairDistanceSpringRow> distanceSpringRows = {},
    const AvbdRigidPointPairDistanceSpringOptions& distanceSpringOptions = {})
{
  AvbdRigidBlockDescentStats stats;
  const std::size_t bodyCount = states.size();
  if (bodyCount == 0 || masses.size() != bodyCount
      || bodyInertias.size() != bodyCount || fixed.size() != bodyCount
      || inertialTargets.size() != bodyCount || timeStep <= 0.0) {
    return stats;
  }

  const auto validBody = [bodyCount](std::uint32_t body) {
    return body < bodyCount;
  };

  AvbdRigidBodyRowIndexScratch localRowIndexScratch;
  AvbdRigidBodyRowIndexScratch& rowIndexData
      = rowIndexScratch != nullptr ? *rowIndexScratch : localRowIndexScratch;

  auto& attachmentRowOffsets = rowIndexData.attachmentRowOffsets;
  auto& attachmentRowIndices = rowIndexData.attachmentRowIndices;
  auto& attachmentRowCursor = rowIndexData.attachmentRowCursor;
  const auto attachmentRowKeyOf
      = [](const AvbdRigidBodyPointAttachmentRow& row) {
          return avbdRigidBodyRowKey(row.body);
        };
  const auto clearAttachmentRowLayout = [&]() {
    attachmentRowOffsets.clear();
    attachmentRowIndices.clear();
    attachmentRowCursor.clear();
    rowIndexData.attachmentRowBodyKeys.clear();
    rowIndexData.attachmentRowBodyCount = 0u;
  };
  if (attachmentRows.empty()) {
    clearAttachmentRowLayout();
  } else if (!avbdRigidRowIndexLayoutMatches(
                 attachmentRows,
                 bodyCount,
                 rowIndexData.attachmentRowBodyCount,
                 rowIndexData.attachmentRowBodyKeys,
                 attachmentRowOffsets,
                 attachmentRowKeyOf)) {
    attachmentRowOffsets.assign(bodyCount + 1u, 0u);
    for (std::size_t rowIndex = 0; rowIndex < attachmentRows.size();
         ++rowIndex) {
      const AvbdRigidBodyPointAttachmentRow& indexedRow
          = attachmentRows[rowIndex];
      if (validBody(indexedRow.body)) {
        ++attachmentRowOffsets[indexedRow.body + 1u];
      }
    }
    for (std::size_t body = 1; body < attachmentRowOffsets.size(); ++body) {
      attachmentRowOffsets[body] += attachmentRowOffsets[body - 1u];
    }
    attachmentRowIndices.resize(attachmentRowOffsets.back());
    attachmentRowCursor.assign(
        attachmentRowOffsets.begin(), attachmentRowOffsets.end());
    for (std::size_t rowIndex = 0; rowIndex < attachmentRows.size();
         ++rowIndex) {
      const AvbdRigidBodyPointAttachmentRow& indexedRow
          = attachmentRows[rowIndex];
      if (validBody(indexedRow.body)) {
        attachmentRowIndices[attachmentRowCursor[indexedRow.body]++] = rowIndex;
      }
    }
    avbdRigidRefreshRowIndexLayoutKeys(
        attachmentRows,
        bodyCount,
        rowIndexData.attachmentRowBodyCount,
        rowIndexData.attachmentRowBodyKeys,
        attachmentRowKeyOf);
  }

  auto& pointPairRowOffsets = rowIndexData.pointPairRowOffsets;
  auto& pointPairRowIndices = rowIndexData.pointPairRowIndices;
  auto& pointPairRowCursor = rowIndexData.pointPairRowCursor;
  const auto pointPairRowKeyOf = [](const AvbdRigidBodyPointPairRow& row) {
    return avbdRigidBodyPairRowKey(row.bodyA, row.bodyB);
  };
  const auto clearPointPairRowLayout = [&]() {
    pointPairRowOffsets.clear();
    pointPairRowIndices.clear();
    pointPairRowCursor.clear();
    rowIndexData.pointPairRowBodyKeys.clear();
    rowIndexData.pointPairRowBodyCount = 0u;
  };
  if (pointPairRows.empty()) {
    clearPointPairRowLayout();
  } else if (!avbdRigidRowIndexLayoutMatches(
                 pointPairRows,
                 bodyCount,
                 rowIndexData.pointPairRowBodyCount,
                 rowIndexData.pointPairRowBodyKeys,
                 pointPairRowOffsets,
                 pointPairRowKeyOf)) {
    pointPairRowOffsets.assign(bodyCount + 1u, 0u);
    for (std::size_t rowIndex = 0; rowIndex < pointPairRows.size();
         ++rowIndex) {
      const AvbdRigidBodyPointPairRow& indexedRow = pointPairRows[rowIndex];
      if (validBody(indexedRow.bodyA) && validBody(indexedRow.bodyB)) {
        ++pointPairRowOffsets[indexedRow.bodyA + 1u];
        if (indexedRow.bodyB != indexedRow.bodyA) {
          ++pointPairRowOffsets[indexedRow.bodyB + 1u];
        }
      }
    }
    for (std::size_t body = 1; body < pointPairRowOffsets.size(); ++body) {
      pointPairRowOffsets[body] += pointPairRowOffsets[body - 1u];
    }
    pointPairRowIndices.resize(pointPairRowOffsets.back());
    pointPairRowCursor.assign(
        pointPairRowOffsets.begin(), pointPairRowOffsets.end());
    for (std::size_t rowIndex = 0; rowIndex < pointPairRows.size();
         ++rowIndex) {
      const AvbdRigidBodyPointPairRow& indexedRow = pointPairRows[rowIndex];
      if (validBody(indexedRow.bodyA) && validBody(indexedRow.bodyB)) {
        pointPairRowIndices[pointPairRowCursor[indexedRow.bodyA]++] = rowIndex;
        if (indexedRow.bodyB != indexedRow.bodyA) {
          pointPairRowIndices[pointPairRowCursor[indexedRow.bodyB]++]
              = rowIndex;
        }
      }
    }
    avbdRigidRefreshRowIndexLayoutKeys(
        pointPairRows,
        bodyCount,
        rowIndexData.pointPairRowBodyCount,
        rowIndexData.pointPairRowBodyKeys,
        pointPairRowKeyOf);
  }

  auto& distanceSpringRowOffsets = rowIndexData.distanceSpringRowOffsets;
  auto& distanceSpringRowIndices = rowIndexData.distanceSpringRowIndices;
  auto& distanceSpringRowCursor = rowIndexData.distanceSpringRowCursor;
  const auto distanceSpringRowKeyOf
      = [](const AvbdRigidBodyPointPairDistanceSpringRow& row) {
          return avbdRigidBodyPairRowKey(row.bodyA, row.bodyB);
        };
  const auto clearDistanceSpringRowLayout = [&]() {
    distanceSpringRowOffsets.clear();
    distanceSpringRowIndices.clear();
    distanceSpringRowCursor.clear();
    rowIndexData.distanceSpringRowBodyKeys.clear();
    rowIndexData.distanceSpringRowBodyCount = 0u;
  };
  if (distanceSpringRows.empty()) {
    clearDistanceSpringRowLayout();
  } else if (!avbdRigidRowIndexLayoutMatches(
                 distanceSpringRows,
                 bodyCount,
                 rowIndexData.distanceSpringRowBodyCount,
                 rowIndexData.distanceSpringRowBodyKeys,
                 distanceSpringRowOffsets,
                 distanceSpringRowKeyOf)) {
    distanceSpringRowOffsets.assign(bodyCount + 1u, 0u);
    for (std::size_t rowIndex = 0; rowIndex < distanceSpringRows.size();
         ++rowIndex) {
      const AvbdRigidBodyPointPairDistanceSpringRow& indexedRow
          = distanceSpringRows[rowIndex];
      if (validBody(indexedRow.bodyA) && validBody(indexedRow.bodyB)
          && indexedRow.bodyB != indexedRow.bodyA) {
        ++distanceSpringRowOffsets[indexedRow.bodyA + 1u];
        ++distanceSpringRowOffsets[indexedRow.bodyB + 1u];
      }
    }
    for (std::size_t body = 1; body < distanceSpringRowOffsets.size(); ++body) {
      distanceSpringRowOffsets[body] += distanceSpringRowOffsets[body - 1u];
    }
    distanceSpringRowIndices.resize(distanceSpringRowOffsets.back());
    distanceSpringRowCursor.assign(
        distanceSpringRowOffsets.begin(), distanceSpringRowOffsets.end());
    for (std::size_t rowIndex = 0; rowIndex < distanceSpringRows.size();
         ++rowIndex) {
      const AvbdRigidBodyPointPairDistanceSpringRow& indexedRow
          = distanceSpringRows[rowIndex];
      if (validBody(indexedRow.bodyA) && validBody(indexedRow.bodyB)
          && indexedRow.bodyB != indexedRow.bodyA) {
        distanceSpringRowIndices[distanceSpringRowCursor[indexedRow.bodyA]++]
            = rowIndex;
        distanceSpringRowIndices[distanceSpringRowCursor[indexedRow.bodyB]++]
            = rowIndex;
      }
    }
    avbdRigidRefreshRowIndexLayoutKeys(
        distanceSpringRows,
        bodyCount,
        rowIndexData.distanceSpringRowBodyCount,
        rowIndexData.distanceSpringRowBodyKeys,
        distanceSpringRowKeyOf);
  }

  auto& angularPairRowOffsets = rowIndexData.angularPairRowOffsets;
  auto& angularPairRowIndices = rowIndexData.angularPairRowIndices;
  auto& angularPairRowCursor = rowIndexData.angularPairRowCursor;
  const auto angularPairRowKeyOf = [](const AvbdRigidBodyAngularPairRow& row) {
    return avbdRigidBodyPairRowKey(row.bodyA, row.bodyB);
  };
  const auto clearAngularPairRowLayout = [&]() {
    angularPairRowOffsets.clear();
    angularPairRowIndices.clear();
    angularPairRowCursor.clear();
    rowIndexData.angularPairRowBodyKeys.clear();
    rowIndexData.angularPairRowBodyCount = 0u;
  };
  if (angularPairRows.empty()) {
    clearAngularPairRowLayout();
  } else if (!avbdRigidRowIndexLayoutMatches(
                 angularPairRows,
                 bodyCount,
                 rowIndexData.angularPairRowBodyCount,
                 rowIndexData.angularPairRowBodyKeys,
                 angularPairRowOffsets,
                 angularPairRowKeyOf)) {
    angularPairRowOffsets.assign(bodyCount + 1u, 0u);
    for (std::size_t rowIndex = 0; rowIndex < angularPairRows.size();
         ++rowIndex) {
      const AvbdRigidBodyAngularPairRow& indexedRow = angularPairRows[rowIndex];
      if (validBody(indexedRow.bodyA) && validBody(indexedRow.bodyB)) {
        ++angularPairRowOffsets[indexedRow.bodyA + 1u];
        if (indexedRow.bodyB != indexedRow.bodyA) {
          ++angularPairRowOffsets[indexedRow.bodyB + 1u];
        }
      }
    }
    for (std::size_t body = 1; body < angularPairRowOffsets.size(); ++body) {
      angularPairRowOffsets[body] += angularPairRowOffsets[body - 1u];
    }
    angularPairRowIndices.resize(angularPairRowOffsets.back());
    angularPairRowCursor.assign(
        angularPairRowOffsets.begin(), angularPairRowOffsets.end());
    for (std::size_t rowIndex = 0; rowIndex < angularPairRows.size();
         ++rowIndex) {
      const AvbdRigidBodyAngularPairRow& indexedRow = angularPairRows[rowIndex];
      if (validBody(indexedRow.bodyA) && validBody(indexedRow.bodyB)) {
        angularPairRowIndices[angularPairRowCursor[indexedRow.bodyA]++]
            = rowIndex;
        if (indexedRow.bodyB != indexedRow.bodyA) {
          angularPairRowIndices[angularPairRowCursor[indexedRow.bodyB]++]
              = rowIndex;
        }
      }
    }
    avbdRigidRefreshRowIndexLayoutKeys(
        angularPairRows,
        bodyCount,
        rowIndexData.angularPairRowBodyCount,
        rowIndexData.angularPairRowBodyKeys,
        angularPairRowKeyOf);
  }

  auto& frictionPairRowOffsets = rowIndexData.frictionPairRowOffsets;
  auto& frictionPairRowIndices = rowIndexData.frictionPairRowIndices;
  auto& frictionPairRowCursor = rowIndexData.frictionPairRowCursor;
  const auto frictionPairRowKeyOf
      = [](const AvbdRigidBodyPointPairFrictionRows& row) {
          return avbdRigidBodyPairRowKey(row.bodyA, row.bodyB);
        };
  const auto clearFrictionPairRowLayout = [&]() {
    frictionPairRowOffsets.clear();
    frictionPairRowIndices.clear();
    frictionPairRowCursor.clear();
    rowIndexData.frictionPairRowBodyKeys.clear();
    rowIndexData.frictionPairRowBodyCount = 0u;
  };
  if (frictionPairRows.empty()) {
    clearFrictionPairRowLayout();
  } else if (!avbdRigidRowIndexLayoutMatches(
                 frictionPairRows,
                 bodyCount,
                 rowIndexData.frictionPairRowBodyCount,
                 rowIndexData.frictionPairRowBodyKeys,
                 frictionPairRowOffsets,
                 frictionPairRowKeyOf)) {
    frictionPairRowOffsets.assign(bodyCount + 1u, 0u);
    for (std::size_t rowIndex = 0; rowIndex < frictionPairRows.size();
         ++rowIndex) {
      const AvbdRigidBodyPointPairFrictionRows& indexedRows
          = frictionPairRows[rowIndex];
      if (validBody(indexedRows.bodyA) && validBody(indexedRows.bodyB)) {
        ++frictionPairRowOffsets[indexedRows.bodyA + 1u];
        if (indexedRows.bodyB != indexedRows.bodyA) {
          ++frictionPairRowOffsets[indexedRows.bodyB + 1u];
        }
      }
    }
    for (std::size_t body = 1; body < frictionPairRowOffsets.size(); ++body) {
      frictionPairRowOffsets[body] += frictionPairRowOffsets[body - 1u];
    }
    frictionPairRowIndices.resize(frictionPairRowOffsets.back());
    frictionPairRowCursor.assign(
        frictionPairRowOffsets.begin(), frictionPairRowOffsets.end());
    for (std::size_t rowIndex = 0; rowIndex < frictionPairRows.size();
         ++rowIndex) {
      const AvbdRigidBodyPointPairFrictionRows& indexedRows
          = frictionPairRows[rowIndex];
      if (validBody(indexedRows.bodyA) && validBody(indexedRows.bodyB)) {
        frictionPairRowIndices[frictionPairRowCursor[indexedRows.bodyA]++]
            = rowIndex;
        if (indexedRows.bodyB != indexedRows.bodyA) {
          frictionPairRowIndices[frictionPairRowCursor[indexedRows.bodyB]++]
              = rowIndex;
        }
      }
    }
    avbdRigidRefreshRowIndexLayoutKeys(
        frictionPairRows,
        bodyCount,
        rowIndexData.frictionPairRowBodyCount,
        rowIndexData.frictionPairRowBodyKeys,
        frictionPairRowKeyOf);
  }

  struct PointPairWorldPointCache
  {
    bool valid = false;
    std::uint32_t bodyA = std::numeric_limits<std::uint32_t>::max();
    std::uint32_t bodyB = std::numeric_limits<std::uint32_t>::max();
    Eigen::Vector3d localPointA = Eigen::Vector3d::Zero();
    Eigen::Vector3d localPointB = Eigen::Vector3d::Zero();
    Eigen::Vector3d worldPointA = Eigen::Vector3d::Zero();
    Eigen::Vector3d worldPointB = Eigen::Vector3d::Zero();
  };

  const auto pointPairWorldPoints =
      [&](const AvbdRigidBodyPointPairRow& indexedRow,
          PointPairWorldPointCache& cache) -> const PointPairWorldPointCache& {
    const AvbdRigidPointPairRow& row = indexedRow.row;
    if (!cache.valid || cache.bodyA != indexedRow.bodyA
        || cache.bodyB != indexedRow.bodyB
        || !detail::avbdRigidVectorExactEqual(
            cache.localPointA, row.localPointA)
        || !detail::avbdRigidVectorExactEqual(
            cache.localPointB, row.localPointB)) {
      cache.valid = true;
      cache.bodyA = indexedRow.bodyA;
      cache.bodyB = indexedRow.bodyB;
      cache.localPointA = row.localPointA;
      cache.localPointB = row.localPointB;
      cache.worldPointA
          = avbdRigidBodyWorldPoint(states[indexedRow.bodyA], row.localPointA);
      cache.worldPointB
          = avbdRigidBodyWorldPoint(states[indexedRow.bodyB], row.localPointB);
    }
    return cache;
  };

  const auto addPointPairToBlock =
      [&](AvbdRigidBodyBlock& block,
          std::uint32_t body,
          const AvbdRigidBodyPointPairRow& indexedRow,
          PointPairWorldPointCache& cache) {
        const AvbdRigidPointPairRow& row = indexedRow.row;
        const AvbdRigidBodyState& stateA = states[indexedRow.bodyA];
        const AvbdRigidBodyState& stateB = states[indexedRow.bodyB];
        const PointPairWorldPointCache& points
            = pointPairWorldPoints(indexedRow, cache);
        const Eigen::Vector3d& worldPointA = points.worldPointA;
        const Eigen::Vector3d& worldPointB = points.worldPointB;
        const double rawConstraintValue
            = row.offset + row.axis.dot(worldPointB - worldPointA);
        const double constraintValue
            = avbdRigidRowUsesFiniteMaterial(row.materialStiffness)
                  ? rawConstraintValue
                  : regularizeAvbdConstraintValue(
                        rawConstraintValue,
                        row.previousConstraintValue,
                        rowOptions.alpha);
        const double forceMagnitude = avbdRigidScalarRowForce(
            row.state, constraintValue, row.bounds, row.materialStiffness);

        if (indexedRow.bodyA == body) {
          const Vector6d direction
              = avbdRigidWorldPointDirection(stateA, worldPointA, row.axis);
          block.force.noalias() += forceMagnitude * direction;
          addAvbdRigidBlockHessianRankOneLowerTriangle(
              block, direction, row.state.stiffness);
        }
        if (indexedRow.bodyB == body && indexedRow.bodyB != indexedRow.bodyA) {
          const Vector6d direction
              = avbdRigidWorldPointDirection(stateB, worldPointB, -row.axis);
          block.force.noalias() += forceMagnitude * direction;
          addAvbdRigidBlockHessianRankOneLowerTriangle(
              block, direction, row.state.stiffness);
        }
      };

  const auto addDistanceSpringToBlock
      = [&](AvbdRigidBodyBlock& block,
            std::uint32_t body,
            const AvbdRigidBodyPointPairDistanceSpringRow& indexedRow) {
          const AvbdRigidPointPairDistanceSpringRow& row = indexedRow.row;
          const AvbdRigidBodyState& stateA = states[indexedRow.bodyA];
          const AvbdRigidBodyState& stateB = states[indexedRow.bodyB];
          const Eigen::Vector3d worldPointA
              = avbdRigidBodyWorldPoint(stateA, row.localPointA);
          const Eigen::Vector3d worldPointB
              = avbdRigidBodyWorldPoint(stateB, row.localPointB);
          const Eigen::Vector3d relative = worldPointB - worldPointA;
          const double length = relative.norm();
          if (!relative.allFinite()
              || length <= kAvbdRigidMinDistanceSpringLength) {
            return;
          }

          const Eigen::Vector3d axis = relative / length;
          const double forceMagnitude
              = row.state.stiffness * (length - row.restLength);
          if (indexedRow.bodyA == body) {
            const Vector6d direction
                = avbdRigidDistanceSpringDirectionAtWorldPoint(
                    stateA, worldPointA, axis);
            block.force.noalias() += forceMagnitude * direction;
            addAvbdRigidDistanceSpringHessianAtWorldPoint(
                block,
                stateA,
                worldPointA,
                axis,
                length,
                row.restLength,
                row.state.stiffness,
                /*clampToPsd=*/true);
          }
          if (indexedRow.bodyB == body) {
            const Vector6d direction
                = avbdRigidDistanceSpringDirectionAtWorldPoint(
                    stateB, worldPointB, -axis);
            block.force.noalias() += forceMagnitude * direction;
            addAvbdRigidDistanceSpringHessianAtWorldPoint(
                block,
                stateB,
                worldPointB,
                axis,
                length,
                row.restLength,
                row.state.stiffness,
                /*clampToPsd=*/true);
          }
        };

  struct AngularPairConstraintCache
  {
    bool valid = false;
    std::uint32_t bodyA = std::numeric_limits<std::uint32_t>::max();
    std::uint32_t bodyB = std::numeric_limits<std::uint32_t>::max();
    Eigen::Quaterniond targetRelativeOrientation
        = Eigen::Quaterniond::Identity();
    Eigen::Vector3d orientationError = Eigen::Vector3d::Zero();
  };

  const auto angularPairOrientationError
      = [&](const AvbdRigidBodyAngularPairRow& indexedRow,
            AngularPairConstraintCache& cache) -> const Eigen::Vector3d& {
    const AvbdRigidAngularPairRow& row = indexedRow.row;
    if (!cache.valid || cache.bodyA != indexedRow.bodyA
        || cache.bodyB != indexedRow.bodyB
        || !detail::avbdRigidQuaternionExactEqual(
            cache.targetRelativeOrientation, row.targetRelativeOrientation)) {
      cache.valid = true;
      cache.bodyA = indexedRow.bodyA;
      cache.bodyB = indexedRow.bodyB;
      cache.targetRelativeOrientation = row.targetRelativeOrientation;
      cache.orientationError = avbdRigidBodyOrientationError(
          states[indexedRow.bodyB].orientation,
          avbdRigidAngularPairTargetOrientationB(
              states[indexedRow.bodyA], row));
    }
    return cache.orientationError;
  };

  const auto addAngularPairToBlock =
      [&](AvbdRigidBodyBlock& block,
          std::uint32_t body,
          const AvbdRigidBodyAngularPairRow& indexedRow,
          AngularPairConstraintCache& cache) {
        const AvbdRigidAngularPairRow& row = indexedRow.row;
        const double rawConstraintValue
            = row.offset
              + row.axis.dot(angularPairOrientationError(indexedRow, cache));
        const double constraintValue
            = avbdRigidRowUsesFiniteMaterial(row.materialStiffness)
                  ? rawConstraintValue
                  : regularizeAvbdConstraintValue(
                        rawConstraintValue,
                        row.previousConstraintValue,
                        rowOptions.alpha);
        const double forceMagnitude = avbdRigidScalarRowForce(
            row.state, constraintValue, row.bounds, row.materialStiffness);

        if (indexedRow.bodyA == body) {
          const Vector6d direction = avbdRigidAngularPairDirectionA(row);
          block.force.noalias() += forceMagnitude * direction;
          addAvbdRigidBlockHessianRankOneLowerTriangle(
              block, direction, row.state.stiffness);
        }
        if (indexedRow.bodyB == body && indexedRow.bodyB != indexedRow.bodyA) {
          const Vector6d direction = avbdRigidAngularPairDirectionB(row);
          block.force.noalias() += forceMagnitude * direction;
          addAvbdRigidBlockHessianRankOneLowerTriangle(
              block, direction, row.state.stiffness);
        }
      };

  const auto addFrictionPairToBlock
      = [&](AvbdRigidBodyBlock& block,
            std::uint32_t body,
            const AvbdRigidBodyPointPairFrictionRows& indexedRows) {
          const AvbdRigidBodyState& stateA = states[indexedRows.bodyA];
          const AvbdRigidBodyState& stateB = states[indexedRows.bodyB];
          const Eigen::Vector3d firstWorldPointA
              = avbdRigidBodyWorldPoint(stateA, indexedRows.first.localPointA);
          const Eigen::Vector3d firstWorldPointB
              = avbdRigidBodyWorldPoint(stateB, indexedRows.first.localPointB);
          const bool sharedAnchors = avbdRigidPointPairRowsShareLocalPoints(
              indexedRows.first, indexedRows.second);
          const Eigen::Vector3d secondWorldPointA
              = sharedAnchors ? firstWorldPointA
                              : avbdRigidBodyWorldPoint(
                                    stateA, indexedRows.second.localPointA);
          const Eigen::Vector3d secondWorldPointB
              = sharedAnchors ? firstWorldPointB
                              : avbdRigidBodyWorldPoint(
                                    stateB, indexedRows.second.localPointB);
          const Eigen::Vector2d constraintValues
              = avbdRigidPointPairConstraintValuesAtWorldPoints(
                  indexedRows.first,
                  indexedRows.second,
                  firstWorldPointA,
                  firstWorldPointB,
                  secondWorldPointA,
                  secondWorldPointB,
                  frictionOptions.alpha);
          const Eigen::Vector2d force
              = avbdRigidPointPairFrictionTangentPairForceFromConstraintValues(
                  constraintValues,
                  indexedRows.first,
                  indexedRows.second,
                  frictionOptions);
          if (indexedRows.bodyA == body) {
            const Vector6d firstDirection = avbdRigidWorldPointDirection(
                stateA, firstWorldPointA, indexedRows.first.axis);
            const Vector6d secondDirection = avbdRigidWorldPointDirection(
                stateA, secondWorldPointA, indexedRows.second.axis);
            block.force.noalias()
                += force.x() * firstDirection + force.y() * secondDirection;
            addAvbdRigidBlockHessianRankOneLowerTriangle(
                block, firstDirection, indexedRows.first.state.stiffness);
            addAvbdRigidBlockHessianRankOneLowerTriangle(
                block, secondDirection, indexedRows.second.state.stiffness);
          }
          if (indexedRows.bodyB == body
              && indexedRows.bodyB != indexedRows.bodyA) {
            const Vector6d firstDirection = avbdRigidWorldPointDirection(
                stateB, firstWorldPointB, -indexedRows.first.axis);
            const Vector6d secondDirection = avbdRigidWorldPointDirection(
                stateB, secondWorldPointB, -indexedRows.second.axis);
            block.force.noalias()
                += force.x() * firstDirection + force.y() * secondDirection;
            addAvbdRigidBlockHessianRankOneLowerTriangle(
                block, firstDirection, indexedRows.first.state.stiffness);
            addAvbdRigidBlockHessianRankOneLowerTriangle(
                block, secondDirection, indexedRows.second.state.stiffness);
          }
        };

  const auto assemble = [&](std::uint32_t body) {
    AvbdRigidBodyBlock block;
    PointPairWorldPointCache pointPairCache;
    AngularPairConstraintCache angularPairCache;
    addAvbdRigidBodyInertiaTermLowerTriangle(
        block,
        masses[body],
        bodyInertias[body],
        timeStep,
        states[body],
        inertialTargets[body]);

    if (!attachmentRows.empty()) {
      for (std::size_t cursor = attachmentRowOffsets[body];
           cursor < attachmentRowOffsets[body + 1u];
           ++cursor) {
        const AvbdRigidBodyPointAttachmentRow& indexedRow
            = attachmentRows[attachmentRowIndices[cursor]];
        addAvbdRigidPointAttachment(
            block, states[body], indexedRow.row, rowOptions.alpha);
      }
    }
    if (!pointPairRows.empty()) {
      for (std::size_t cursor = pointPairRowOffsets[body];
           cursor < pointPairRowOffsets[body + 1u];
           ++cursor) {
        const AvbdRigidBodyPointPairRow& indexedRow
            = pointPairRows[pointPairRowIndices[cursor]];
        addPointPairToBlock(block, body, indexedRow, pointPairCache);
      }
    }
    if (!distanceSpringRows.empty()) {
      for (std::size_t cursor = distanceSpringRowOffsets[body];
           cursor < distanceSpringRowOffsets[body + 1u];
           ++cursor) {
        const AvbdRigidBodyPointPairDistanceSpringRow& indexedRow
            = distanceSpringRows[distanceSpringRowIndices[cursor]];
        addDistanceSpringToBlock(block, body, indexedRow);
      }
    }
    if (!angularPairRows.empty()) {
      for (std::size_t cursor = angularPairRowOffsets[body];
           cursor < angularPairRowOffsets[body + 1u];
           ++cursor) {
        const AvbdRigidBodyAngularPairRow& indexedRow
            = angularPairRows[angularPairRowIndices[cursor]];
        addAngularPairToBlock(block, body, indexedRow, angularPairCache);
      }
    }
    if (!frictionPairRows.empty()) {
      for (std::size_t cursor = frictionPairRowOffsets[body];
           cursor < frictionPairRowOffsets[body + 1u];
           ++cursor) {
        const AvbdRigidBodyPointPairFrictionRows& indexedRows
            = frictionPairRows[frictionPairRowIndices[cursor]];
        addFrictionPairToBlock(block, body, indexedRows);
      }
    }
    return block;
  };

  const double convergenceSquared
      = options.convergenceDisplacement * options.convergenceDisplacement;
  for (std::size_t iteration = 0; iteration < options.iterations; ++iteration) {
    ++stats.iterations;
    double maxStepSquared = 0.0;
    for (std::uint32_t body = 0; body < bodyCount; ++body) {
      if (fixed[body] != 0u) {
        continue;
      }

      const AvbdRigidBodyBlock block = assemble(body);
      const Vector6d step
          = solveAvbdRigidBodyBlock(block, options.regularization);
      applyAvbdRigidBodyStep(states[body], step);
      maxStepSquared = std::max(maxStepSquared, step.squaredNorm());
      ++stats.bodyUpdates;
    }

    for (AvbdRigidBodyPointAttachmentRow& indexedRow : attachmentRows) {
      if (validBody(indexedRow.body)) {
        indexedRow.row.state = updateAvbdRigidPointAttachmentRow(
            indexedRow.row.state,
            states[indexedRow.body],
            indexedRow.row,
            rowOptions);
      }
    }
    PointPairWorldPointCache pointPairUpdateCache;
    for (AvbdRigidBodyPointPairRow& indexedRow : pointPairRows) {
      if (validBody(indexedRow.bodyA) && validBody(indexedRow.bodyB)) {
        if (avbdRigidRowUsesFiniteMaterial(indexedRow.row.materialStiffness)) {
          indexedRow.row.state.lambda = 0.0;
          const double maxStiffness = std::min(
              indexedRow.row.materialStiffness, rowOptions.maxStiffness);
          if (rowOptions.beta >= 0.0
              && indexedRow.row.state.stiffness >= maxStiffness) {
            indexedRow.row.state.stiffness = maxStiffness;
            continue;
          }
        }

        const PointPairWorldPointCache& points
            = pointPairWorldPoints(indexedRow, pointPairUpdateCache);
        const double rawConstraintValue
            = indexedRow.row.offset
              + indexedRow.row.axis.dot(
                  points.worldPointB - points.worldPointA);
        if (avbdRigidRowUsesFiniteMaterial(indexedRow.row.materialStiffness)) {
          const double maxStiffness = std::min(
              indexedRow.row.materialStiffness, rowOptions.maxStiffness);
          indexedRow.row.state.stiffness = updateAvbdFiniteStiffness(
              indexedRow.row.state.stiffness,
              rawConstraintValue,
              rowOptions.beta,
              maxStiffness);
        } else {
          const double constraintValue = regularizeAvbdConstraintValue(
              rawConstraintValue,
              indexedRow.row.previousConstraintValue,
              rowOptions.alpha);
          indexedRow.row.state = updateAvbdHardConstraintRow(
              indexedRow.row.state,
              constraintValue,
              rowOptions.beta,
              indexedRow.row.bounds,
              rowOptions.maxStiffness);
        }
      }
    }
    for (AvbdRigidBodyPointPairDistanceSpringRow& indexedRow :
         distanceSpringRows) {
      if (validBody(indexedRow.bodyA) && validBody(indexedRow.bodyB)
          && indexedRow.bodyB != indexedRow.bodyA) {
        indexedRow.row.state = updateAvbdRigidPointPairDistanceSpringRow(
            indexedRow.row.state,
            states[indexedRow.bodyA],
            states[indexedRow.bodyB],
            indexedRow.row,
            distanceSpringOptions);
      }
    }
    AngularPairConstraintCache angularPairUpdateCache;
    for (AvbdRigidBodyAngularPairRow& indexedRow : angularPairRows) {
      if (validBody(indexedRow.bodyA) && validBody(indexedRow.bodyB)) {
        if (avbdRigidRowUsesFiniteMaterial(indexedRow.row.materialStiffness)) {
          indexedRow.row.state.lambda = 0.0;
          const double maxStiffness = std::min(
              indexedRow.row.materialStiffness, rowOptions.maxStiffness);
          if (rowOptions.beta >= 0.0
              && indexedRow.row.state.stiffness >= maxStiffness) {
            indexedRow.row.state.stiffness = maxStiffness;
            continue;
          }
        }

        const double rawConstraintValue
            = indexedRow.row.offset
              + indexedRow.row.axis.dot(angularPairOrientationError(
                  indexedRow, angularPairUpdateCache));
        if (avbdRigidRowUsesFiniteMaterial(indexedRow.row.materialStiffness)) {
          indexedRow.row.state.stiffness = updateAvbdFiniteStiffness(
              indexedRow.row.state.stiffness,
              rawConstraintValue,
              rowOptions.beta,
              std::min(
                  indexedRow.row.materialStiffness, rowOptions.maxStiffness));
        } else {
          const double constraintValue = regularizeAvbdConstraintValue(
              rawConstraintValue,
              indexedRow.row.previousConstraintValue,
              rowOptions.alpha);
          indexedRow.row.state = updateAvbdHardConstraintRow(
              indexedRow.row.state,
              constraintValue,
              rowOptions.beta,
              indexedRow.row.bounds,
              rowOptions.maxStiffness);
        }
      }
    }
    for (AvbdRigidBodyPointPairFrictionRows& indexedRows : frictionPairRows) {
      if (validBody(indexedRows.bodyA) && validBody(indexedRows.bodyB)) {
        updateAvbdRigidPointPairFrictionTangentPair(
            indexedRows.first,
            indexedRows.second,
            states[indexedRows.bodyA],
            states[indexedRows.bodyB],
            frictionOptions);
      }
    }

    if (convergenceSquared > 0.0 && maxStepSquared <= convergenceSquared) {
      break;
    }
  }

  return stats;
}

} // namespace dart::simulation::detail::deformable_vbd
