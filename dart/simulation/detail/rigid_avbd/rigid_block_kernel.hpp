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
#include <dart/simulation/detail/deformable_vbd/parallel_row_update.hpp>
#include <dart/simulation/detail/deformable_vbd/quasi_newton_hessian.hpp>

#include <dart/common/memory_allocator.hpp>
#include <dart/common/stl_allocator.hpp>

#include <Eigen/Cholesky>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <algorithm>
#include <array>
#include <limits>
#include <span>
#include <type_traits>
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
inline Eigen::Vector3d avbdRigidRotationVectorFromNormalized(
    const Eigen::Quaterniond& q)
{
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
inline Eigen::Vector3d avbdRigidRotationVector(
    const Eigen::Quaterniond& orientation)
{
  const Eigen::Quaterniond q = normalizeAvbdRigidOrientation(orientation);
  return avbdRigidRotationVectorFromNormalized(q);
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

/// Step-start first-order model for an AVBD rigid contact scalar row.
///
/// Contact directions are the negative constraint gradients used by the block
/// solve. Their translational components are exactly `axis` and `-axis`, so
/// only the angular components need separate storage. The two orientations and
/// projected body-origin separation define the world-frame tangent coordinates
/// `x - x_t` without duplicating both complete body states in every row.
struct AvbdRigidPointPairTaylorLinearization
{
  Eigen::Quaterniond stepStartOrientationA = Eigen::Quaterniond::Identity();
  Eigen::Quaterniond stepStartOrientationB = Eigen::Quaterniond::Identity();
  Eigen::Vector3d angularDirectionA = Eigen::Vector3d::Zero();
  Eigen::Vector3d angularDirectionB = Eigen::Vector3d::Zero();
  double stepStartAxisPositionDifference = 0.0;
  double stepStartConstraintValue = 0.0;
  bool valid = false;
};

/// Curvature model for a world-point scalar row.
///
/// General joints and motors recompute their nonlinear world points during the
/// solve and use the AVBD Section 3.5 quasi-Newton geometric term. Contact rows
/// cache C(x_t) and their Jacobian directions at the beginning of the step and
/// use the paper's first-order Taylor contact energy; its second-order term is
/// intentionally discarded.
enum class AvbdRigidPointCurvatureModel : std::uint8_t
{
  QuasiNewton,
  TaylorLinearized,
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
  AvbdRigidPointCurvatureModel curvatureModel
      = AvbdRigidPointCurvatureModel::QuasiNewton;
  AvbdRigidPointPairTaylorLinearization taylorLinearization;
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

struct AvbdRigidPointAttachmentOptions
{
  double alpha = 0.0;
  double beta = 1.0;
  double maxStiffness = std::numeric_limits<double>::infinity();
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
  bool hasSolveOptionsOverride = false;
  AvbdRigidPointAttachmentOptions solveOptionsOverride;
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
  std::size_t normalRowIndex = std::numeric_limits<std::size_t>::max();
  double frictionCoefficient = 0.0;
  // Scalar dual/stiffness state remains in the row inventory while the
  // axis-independent static-friction anchor is owned once per contact pair by
  // `AvbdRigidContactManifoldRowScratch`. The builder finalizes both owner
  // buffers before publishing these pointers; they remain valid until either
  // inventory or the contact scratch is mutated by a later build/clear.
  AvbdScalarRowRecord* persistentFirstRecord = nullptr;
  AvbdScalarRowRecord* persistentSecondRecord = nullptr;
  AvbdContactTangentAnchorState* persistentAnchor = nullptr;
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

inline constexpr double kAvbdRigidStaticFrictionTolerance = 1e-5;

struct AvbdRigidPointPairFrictionOptions
{
  double alpha = 0.0;
  double beta = 1.0;
  double maxStiffness = std::numeric_limits<double>::infinity();
  // Match the 3D AVBD reference implementation's tangential-displacement
  // threshold for retaining a static-friction contact anchor.
  double staticFrictionTolerance = kAvbdRigidStaticFrictionTolerance;
  bool fixedPenalty = false;
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

//==============================================================================
inline const AvbdRigidPointAttachmentOptions& avbdRigidPointPairSolveOptions(
    const AvbdRigidBodyPointPairRow& row,
    const AvbdRigidPointAttachmentOptions& fallback) noexcept
{
  return row.hasSolveOptionsOverride ? row.solveOptionsOverride : fallback;
}

struct AvbdRigidBodyRowIndexScratch
{
  using SizeAllocator = ::dart::common::StlAllocator<std::size_t>;
  using KeyAllocator = ::dart::common::StlAllocator<std::uint64_t>;
  using SizeVector = std::vector<std::size_t, SizeAllocator>;
  using KeyVector = std::vector<std::uint64_t, KeyAllocator>;

  AvbdRigidBodyRowIndexScratch() = default;

  explicit AvbdRigidBodyRowIndexScratch(
      ::dart::common::MemoryAllocator& allocator)
    : attachmentRowOffsets(SizeAllocator{allocator}),
      attachmentRowIndices(SizeAllocator{allocator}),
      attachmentRowCursor(SizeAllocator{allocator}),
      attachmentRowBodyKeys(KeyAllocator{allocator}),
      pointPairRowOffsets(SizeAllocator{allocator}),
      pointPairRowIndices(SizeAllocator{allocator}),
      pointPairRowCursor(SizeAllocator{allocator}),
      pointPairRowBodyKeys(KeyAllocator{allocator}),
      distanceSpringRowOffsets(SizeAllocator{allocator}),
      distanceSpringRowIndices(SizeAllocator{allocator}),
      distanceSpringRowCursor(SizeAllocator{allocator}),
      distanceSpringRowBodyKeys(KeyAllocator{allocator}),
      angularPairRowOffsets(SizeAllocator{allocator}),
      angularPairRowIndices(SizeAllocator{allocator}),
      angularPairRowCursor(SizeAllocator{allocator}),
      angularPairRowBodyKeys(KeyAllocator{allocator}),
      frictionPairRowOffsets(SizeAllocator{allocator}),
      frictionPairRowIndices(SizeAllocator{allocator}),
      frictionPairRowCursor(SizeAllocator{allocator}),
      frictionPairRowBodyKeys(KeyAllocator{allocator})
  {
  }

  template <
      typename Allocator,
      typename
      = std::enable_if_t<std::is_constructible_v<SizeAllocator, Allocator>>>
  explicit AvbdRigidBodyRowIndexScratch(const Allocator& allocator)
    : attachmentRowOffsets(SizeAllocator{allocator}),
      attachmentRowIndices(SizeAllocator{allocator}),
      attachmentRowCursor(SizeAllocator{allocator}),
      attachmentRowBodyKeys(KeyAllocator{allocator}),
      pointPairRowOffsets(SizeAllocator{allocator}),
      pointPairRowIndices(SizeAllocator{allocator}),
      pointPairRowCursor(SizeAllocator{allocator}),
      pointPairRowBodyKeys(KeyAllocator{allocator}),
      distanceSpringRowOffsets(SizeAllocator{allocator}),
      distanceSpringRowIndices(SizeAllocator{allocator}),
      distanceSpringRowCursor(SizeAllocator{allocator}),
      distanceSpringRowBodyKeys(KeyAllocator{allocator}),
      angularPairRowOffsets(SizeAllocator{allocator}),
      angularPairRowIndices(SizeAllocator{allocator}),
      angularPairRowCursor(SizeAllocator{allocator}),
      angularPairRowBodyKeys(KeyAllocator{allocator}),
      frictionPairRowOffsets(SizeAllocator{allocator}),
      frictionPairRowIndices(SizeAllocator{allocator}),
      frictionPairRowCursor(SizeAllocator{allocator}),
      frictionPairRowBodyKeys(KeyAllocator{allocator})
  {
  }

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
                                   auto& offsets,
                                   auto& indices,
                                   auto& cursor,
                                   auto& keys,
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

  SizeVector attachmentRowOffsets;
  SizeVector attachmentRowIndices;
  SizeVector attachmentRowCursor;
  KeyVector attachmentRowBodyKeys;
  std::size_t attachmentRowBodyCount = 0u;
  SizeVector pointPairRowOffsets;
  SizeVector pointPairRowIndices;
  SizeVector pointPairRowCursor;
  KeyVector pointPairRowBodyKeys;
  std::size_t pointPairRowBodyCount = 0u;
  SizeVector distanceSpringRowOffsets;
  SizeVector distanceSpringRowIndices;
  SizeVector distanceSpringRowCursor;
  KeyVector distanceSpringRowBodyKeys;
  std::size_t distanceSpringRowBodyCount = 0u;
  SizeVector angularPairRowOffsets;
  SizeVector angularPairRowIndices;
  SizeVector angularPairRowCursor;
  KeyVector angularPairRowBodyKeys;
  std::size_t angularPairRowBodyCount = 0u;
  SizeVector frictionPairRowOffsets;
  SizeVector frictionPairRowIndices;
  SizeVector frictionPairRowCursor;
  KeyVector frictionPairRowBodyKeys;
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
inline Eigen::Vector3d avbdRigidBodyOrientationErrorFromNormalized(
    const Eigen::Quaterniond& orientation,
    const Eigen::Quaterniond& targetOrientation)
{
  return avbdRigidRotationVectorFromNormalized(
      orientation * targetOrientation.conjugate());
}

//==============================================================================
inline Eigen::Vector3d avbdRigidBodyOrientationError(
    const Eigen::Quaterniond& orientation,
    const Eigen::Quaterniond& targetOrientation)
{
  const Eigen::Quaterniond current = normalizeAvbdRigidOrientation(orientation);
  const Eigen::Quaterniond target
      = normalizeAvbdRigidOrientation(targetOrientation);
  return avbdRigidBodyOrientationErrorFromNormalized(current, target);
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
  const Eigen::Quaterniond targetOrientation
      = normalizeAvbdRigidOrientation(inertialTarget.orientation);
  const Eigen::Vector3d orientationError
      = avbdRigidBodyOrientationErrorFromNormalized(
          orientation, targetOrientation);

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
inline void initializeAvbdRigidPointPairTaylorLinearization(
    AvbdRigidPointPairRow& row,
    const AvbdRigidBodyState& stepStartStateA,
    const AvbdRigidBodyState& stepStartStateB)
{
  AvbdRigidPointPairTaylorLinearization& linearization
      = row.taylorLinearization;
  if (row.curvatureModel != AvbdRigidPointCurvatureModel::TaylorLinearized) {
    linearization = AvbdRigidPointPairTaylorLinearization{};
    return;
  }

  const Eigen::Vector3d worldPointA
      = avbdRigidBodyWorldPoint(stepStartStateA, row.localPointA);
  const Eigen::Vector3d worldPointB
      = avbdRigidBodyWorldPoint(stepStartStateB, row.localPointB);
  linearization.stepStartOrientationA
      = normalizeAvbdRigidOrientation(stepStartStateA.orientation);
  linearization.stepStartOrientationB
      = normalizeAvbdRigidOrientation(stepStartStateB.orientation);
  linearization.angularDirectionA
      = (worldPointA - stepStartStateA.position).cross(row.axis);
  linearization.angularDirectionB
      = (worldPointB - stepStartStateB.position).cross(-row.axis);
  linearization.stepStartAxisPositionDifference
      = row.axis.dot(stepStartStateB.position - stepStartStateA.position);
  linearization.stepStartConstraintValue
      = row.offset + row.axis.dot(worldPointB - worldPointA);
  linearization.valid = true;

  // Equation 18 subtracts alpha times the unregularized step-start value.
  // Keeping this intercept with the cached Taylor model is especially
  // important for a persistent static-friction anchor, whose C(x_t) need not
  // be zero after the bodies have moved tangentially during the prior frame.
  row.previousConstraintValue = linearization.stepStartConstraintValue;
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
  row.curvatureModel = AvbdRigidPointCurvatureModel::TaylorLinearized;
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
inline AvbdContactTangentAnchorKey makeAvbdContactTangentAnchorKey(
    AvbdContactEndpointId first,
    AvbdContactEndpointId second,
    std::uint32_t row = 0)
{
  const auto endpoints = canonicalizeAvbdContactEndpoints(first, second);
  AvbdContactTangentAnchorKey key;
  key.objectA = endpoints.first.object;
  key.objectB = endpoints.second.object;
  key.featureA = endpoints.first.feature;
  key.featureB = endpoints.second.feature;
  key.row = row;
  return key;
}

//==============================================================================
inline AvbdContactTangentAnchorKey makeAvbdContactTangentAnchorKey(
    const AvbdScalarRowKey& key)
{
  AvbdContactTangentAnchorKey anchorKey;
  anchorKey.objectA = key.objectA;
  anchorKey.objectB = key.objectB;
  anchorKey.featureA = key.featureA;
  anchorKey.featureB = key.featureB;
  anchorKey.row = key.row;
  return anchorKey;
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
  row.curvatureModel = AvbdRigidPointCurvatureModel::TaylorLinearized;
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
      AvbdScalarRowRole::MotorAngular, first, second, row, /*axis=*/0);
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
      AvbdScalarRowRole::MotorLinear, first, second, row, /*axis=*/0);
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
  const AvbdRigidPointPairTaylorLinearization& linearization
      = row.taylorLinearization;
  if (row.curvatureModel == AvbdRigidPointCurvatureModel::TaylorLinearized
      && linearization.valid) {
    const double translationDelta
        = row.axis.dot(stateB.position - stateA.position)
          - linearization.stepStartAxisPositionDifference;
    const Eigen::Vector3d angularDeltaA = avbdRigidBodyOrientationError(
        stateA.orientation, linearization.stepStartOrientationA);
    const Eigen::Vector3d angularDeltaB = avbdRigidBodyOrientationError(
        stateB.orientation, linearization.stepStartOrientationB);
    return linearization.stepStartConstraintValue + translationDelta
           - linearization.angularDirectionA.dot(angularDeltaA)
           - linearization.angularDirectionB.dot(angularDeltaB);
  }

  return row.offset
         + row.axis.dot(
             avbdRigidPointPairRelativePosition(stateA, stateB, row));
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
  return Eigen::Vector2d(
      regularizeAvbdConstraintValue(
          avbdRigidPointPairConstraintValue(stateA, stateB, first),
          first.previousConstraintValue,
          alpha),
      regularizeAvbdConstraintValue(
          avbdRigidPointPairConstraintValue(stateA, stateB, second),
          second.previousConstraintValue,
          alpha));
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
inline bool avbdRigidPointPairFrictionPreviousDualInsideConeForLimit(
    const AvbdRigidPointPairRow& first,
    const AvbdRigidPointPairRow& second,
    double forceLimit)
{
  const double limit = std::max(0.0, forceLimit);
  if (!std::isfinite(limit)) {
    return true;
  }

  const double previousNorm
      = std::hypot(first.state.lambda, second.state.lambda);
  return previousNorm <= limit;
}

//==============================================================================
inline bool avbdRigidPointPairFrictionPreviousDualInsideCone(
    const AvbdRigidPointPairRow& first, const AvbdRigidPointPairRow& second)
{
  return avbdRigidPointPairFrictionPreviousDualInsideConeForLimit(
      first, second, avbdRigidPointPairFrictionPairForceLimit(first, second));
}

//==============================================================================
inline Eigen::Vector2d
avbdRigidPointPairFrictionTangentPairForceFromConstraintValuesAndLimit(
    const Eigen::Vector2d& constraintValues,
    const AvbdRigidPointPairRow& first,
    const AvbdRigidPointPairRow& second,
    double forceLimit,
    bool* clamped = nullptr)
{
  if (clamped != nullptr) {
    *clamped = false;
  }

  const double limit = std::isnan(forceLimit) ? 0.0 : std::max(0.0, forceLimit);

  // Equation 13 first forms the augmented-Lagrangian trial force k*C+lambda.
  // Coulomb friction then projects that trial to the live cone. In particular,
  // a previous dual on the cone can return to its interior; choosing a slip
  // direction from C alone would discard lambda and can reverse the force.
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
inline Eigen::Vector2d
avbdRigidPointPairFrictionTangentPairForceFromConstraintValues(
    const Eigen::Vector2d& constraintValues,
    const AvbdRigidPointPairRow& first,
    const AvbdRigidPointPairRow& second,
    bool* clamped = nullptr)
{
  return avbdRigidPointPairFrictionTangentPairForceFromConstraintValuesAndLimit(
      constraintValues,
      first,
      second,
      avbdRigidPointPairFrictionPairForceLimit(first, second),
      clamped);
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
      clamped);
}

//==============================================================================
inline Vector6d avbdRigidPointPairDirectionA(
    const AvbdRigidBodyState& stateA, const AvbdRigidPointPairRow& row)
{
  if (row.curvatureModel == AvbdRigidPointCurvatureModel::TaylorLinearized
      && row.taylorLinearization.valid) {
    Vector6d direction = Vector6d::Zero();
    direction.head<3>() = row.axis;
    direction.tail<3>() = row.taylorLinearization.angularDirectionA;
    return direction;
  }

  const Eigen::Vector3d worldPoint
      = avbdRigidBodyWorldPoint(stateA, row.localPointA);
  return avbdRigidWorldPointDirection(stateA, worldPoint, row.axis);
}

//==============================================================================
inline Vector6d avbdRigidPointPairDirectionB(
    const AvbdRigidBodyState& stateB, const AvbdRigidPointPairRow& row)
{
  if (row.curvatureModel == AvbdRigidPointCurvatureModel::TaylorLinearized
      && row.taylorLinearization.valid) {
    Vector6d direction = Vector6d::Zero();
    direction.head<3>() = -row.axis;
    direction.tail<3>() = row.taylorLinearization.angularDirectionB;
    return direction;
  }

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
/// Return the force-scaled rotational curvature of a rigid world point.
///
/// With arm r and the actual world force w applied at the point, the negative
/// Jacobian of the generalized torque r x w under DART's left exponential-map
/// update is (r dot w) I - r w^T. This is the rigid-body form of the AVBD
/// geometric-stiffness term before Section 3.5 diagonal lumping.
inline Eigen::Matrix3d avbdRigidWorldPointGeometricStiffness(
    const AvbdRigidBodyState& state,
    const Eigen::Vector3d& worldPoint,
    const Eigen::Vector3d& worldForce)
{
  const Eigen::Vector3d arm = worldPoint - state.position;
  return arm.dot(worldForce) * Eigen::Matrix3d::Identity()
         - arm * worldForce.transpose();
}

//==============================================================================
inline void addAvbdRigidWorldPointQuasiNewtonGeometricDiagonal(
    AvbdRigidBodyBlock& block,
    const AvbdRigidBodyState& state,
    const Eigen::Vector3d& worldPoint,
    const Eigen::Vector3d& worldForce,
    AvbdRigidPointCurvatureModel curvatureModel)
{
  if (curvatureModel == AvbdRigidPointCurvatureModel::TaylorLinearized) {
    return;
  }

  const Eigen::Vector3d arm = worldPoint - state.position;
  if (avbdRigidWorldPointIsBodyOrigin(state, worldPoint)) {
    return;
  }

  const double armForceDot = arm.dot(worldForce);
  const double armSquaredNorm = arm.squaredNorm();
  Eigen::Vector3d diagonal;
  for (Eigen::Index column = 0; column < 3; ++column) {
    const double squaredNorm
        = armForceDot * armForceDot
          - 2.0 * armForceDot * arm[column] * worldForce[column]
          + armSquaredNorm * worldForce[column] * worldForce[column];
    diagonal[column] = std::sqrt(std::max(squaredNorm, 0.0));
  }
  block.hessian.diagonal().tail<3>() += diagonal;
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
    double stiffness)
{
  if (!axis.allFinite() || length <= kAvbdRigidMinDistanceSpringLength
      || !std::isfinite(stiffness)) {
    return;
  }

  const double forceMagnitude = stiffness * (length - restLength);
  const Vector6d direction
      = avbdRigidWorldPointDirection(state, worldPoint, axis);
  block.hessian.noalias() += stiffness * (direction * direction.transpose());

  if (avbdRigidWorldPointIsBodyOrigin(state, worldPoint)) {
    block.hessian.diagonal().head<3>()
        += avbdQuasiNewtonProjectedDistanceDiagonal(
            axis, forceMagnitude / length);
    return;
  }

  const Eigen::Matrix3d constraintHessian
      = (Eigen::Matrix3d::Identity() - axis * axis.transpose()) / length;
  const Matrix3x6d jacobian
      = avbdRigidWorldPointJacobianAtWorldPoint(state, worldPoint);
  Matrix6d geometricStiffness
      = forceMagnitude * jacobian.transpose() * constraintHessian * jacobian;
  geometricStiffness.bottomRightCorner<3, 3>()
      += avbdRigidWorldPointGeometricStiffness(
          state, worldPoint, forceMagnitude * axis);
  block.hessian.diagonal()
      += avbdQuasiNewtonGeometricDiagonal(geometricStiffness);
}

//==============================================================================
inline void addAvbdRigidDistanceSpringHessian(
    AvbdRigidBodyBlock& block,
    const AvbdRigidBodyState& state,
    const Eigen::Vector3d& localPoint,
    const Eigen::Vector3d& axis,
    double length,
    double restLength,
    double stiffness)
{
  addAvbdRigidDistanceSpringHessianAtWorldPoint(
      block,
      state,
      avbdRigidBodyWorldPoint(state, localPoint),
      axis,
      length,
      restLength,
      stiffness);
}

//==============================================================================
inline Eigen::Matrix3d avbdRigidSo3LeftJacobianInverse(
    const Eigen::Vector3d& rotationVector)
{
  const double angleSquared = rotationVector.squaredNorm();
  if (!std::isfinite(angleSquared)) {
    return Eigen::Matrix3d::Identity();
  }

  const Eigen::Matrix3d skew = avbdRigidSkewMatrix(rotationVector);
  double quadraticCoefficient = 1.0 / 12.0;
  if (angleSquared > 1e-12) {
    const double angle = std::sqrt(angleSquared);
    const double halfAngle = 0.5 * angle;
    quadraticCoefficient
        = 1.0 / angleSquared
          - std::cos(halfAngle) / (2.0 * angle * std::sin(halfAngle));
  } else {
    // 1/theta^2 - cot(theta/2)/(2 theta)
    // = 1/12 + theta^2/720 + O(theta^4).
    quadraticCoefficient += angleSquared / 720.0;
  }

  return Eigen::Matrix3d::Identity() - 0.5 * skew
         + quadraticCoefficient * (skew * skew);
}

//==============================================================================
inline Vector6d avbdRigidAngularPairDirectionA(
    const Eigen::Vector3d& orientationError, const AvbdRigidAngularPairRow& row)
{
  Vector6d direction = Vector6d::Zero();
  // orientationError = Log(R_B (R_A R_target)^T). A world-frame left
  // perturbation of A is a right perturbation of that relative rotation. The
  // AVBD direction is the negative constraint gradient.
  direction.tail<3>()
      = avbdRigidSo3LeftJacobianInverse(orientationError) * row.axis;
  return direction;
}

//==============================================================================
inline Vector6d avbdRigidAngularPairDirectionB(
    const Eigen::Vector3d& orientationError, const AvbdRigidAngularPairRow& row)
{
  Vector6d direction = Vector6d::Zero();
  // A world-frame left perturbation of B is a left perturbation of the
  // relative rotation. J_l(phi)^-T = J_l(-phi)^-1.
  direction.tail<3>()
      = -avbdRigidSo3LeftJacobianInverse(-orientationError) * row.axis;
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
  addAvbdRigidWorldPointQuasiNewtonGeometricDiagonal(
      block,
      state,
      worldPoint,
      forceMagnitude * row.axis,
      AvbdRigidPointCurvatureModel::QuasiNewton);
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
    return clampAvbdRowForce(state.stiffness * constraintValue, bounds);
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
  const double rawConstraintValue
      = avbdRigidPointPairConstraintValue(stateA, stateB, row);
  const double constraintValue
      = avbdRigidRowUsesFiniteMaterial(row.materialStiffness)
            ? rawConstraintValue
            : regularizeAvbdConstraintValue(
                  rawConstraintValue, row.previousConstraintValue, alpha);
  const double forceMagnitude = avbdRigidScalarRowForce(
      row.state, constraintValue, row.bounds, row.materialStiffness);
  const Vector6d firstDirection = avbdRigidPointPairDirectionA(stateA, row);
  const Vector6d secondDirection = avbdRigidPointPairDirectionB(stateB, row);

  // AVBD solves per-body blocks; coupling is carried by the shared scalar dual.
  blockA.force.noalias() += forceMagnitude * firstDirection;
  blockB.force.noalias() += forceMagnitude * secondDirection;
  blockA.hessian.noalias()
      += row.state.stiffness * (firstDirection * firstDirection.transpose());
  blockB.hessian.noalias()
      += row.state.stiffness * (secondDirection * secondDirection.transpose());
  if (row.curvatureModel == AvbdRigidPointCurvatureModel::QuasiNewton) {
    const Eigen::Vector3d worldPointA
        = avbdRigidBodyWorldPoint(stateA, row.localPointA);
    const Eigen::Vector3d worldPointB
        = avbdRigidBodyWorldPoint(stateB, row.localPointB);
    addAvbdRigidWorldPointQuasiNewtonGeometricDiagonal(
        blockA,
        stateA,
        worldPointA,
        forceMagnitude * row.axis,
        row.curvatureModel);
    addAvbdRigidWorldPointQuasiNewtonGeometricDiagonal(
        blockB,
        stateB,
        worldPointB,
        -forceMagnitude * row.axis,
        row.curvatureModel);
  }
  return forceMagnitude;
}

//==============================================================================
inline double addAvbdRigidPointPairDistanceSpring(
    AvbdRigidBodyBlock& blockA,
    AvbdRigidBodyBlock& blockB,
    const AvbdRigidBodyState& stateA,
    const AvbdRigidBodyState& stateB,
    const AvbdRigidPointPairDistanceSpringRow& row)
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
      row.state.stiffness);
  addAvbdRigidDistanceSpringHessianAtWorldPoint(
      blockB,
      stateB,
      worldPointB,
      -axis,
      length,
      row.restLength,
      row.state.stiffness);
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
  const Eigen::Vector3d orientationError = avbdRigidBodyOrientationError(
      stateB.orientation, avbdRigidAngularPairTargetOrientationB(stateA, row));
  const double rawConstraintValue = row.offset + row.axis.dot(orientationError);
  const double constraintValue
      = avbdRigidRowUsesFiniteMaterial(row.materialStiffness)
            ? rawConstraintValue
            : regularizeAvbdConstraintValue(
                  rawConstraintValue, row.previousConstraintValue, alpha);
  const double forceMagnitude = avbdRigidScalarRowForce(
      row.state, constraintValue, row.bounds, row.materialStiffness);
  const Vector6d firstDirection
      = avbdRigidAngularPairDirectionA(orientationError, row);
  const Vector6d secondDirection
      = avbdRigidAngularPairDirectionB(orientationError, row);

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
  const Eigen::Vector2d constraintValues = avbdRigidPointPairConstraintValues(
      stateA, stateB, first, second, options.alpha);
  const Eigen::Vector2d force
      = avbdRigidPointPairFrictionTangentPairForceFromConstraintValues(
          constraintValues, first, second);
  const Vector6d firstDirectionA = avbdRigidPointPairDirectionA(stateA, first);
  const Vector6d firstDirectionB = avbdRigidPointPairDirectionB(stateB, first);
  const Vector6d secondDirectionA
      = avbdRigidPointPairDirectionA(stateA, second);
  const Vector6d secondDirectionB
      = avbdRigidPointPairDirectionB(stateB, second);

  blockA.force.noalias()
      += force.x() * firstDirectionA + force.y() * secondDirectionA;
  blockB.force.noalias()
      += force.x() * firstDirectionB + force.y() * secondDirectionB;
  // The AVBD friction force is cone-projected, but its quasi-Newton penalty
  // Hessian is deliberately assembled without differentiating that clamp.
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
  if (first.curvatureModel == AvbdRigidPointCurvatureModel::QuasiNewton
      || second.curvatureModel == AvbdRigidPointCurvatureModel::QuasiNewton) {
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
    addAvbdRigidWorldPointQuasiNewtonGeometricDiagonal(
        blockA,
        stateA,
        firstWorldPointA,
        force.x() * first.axis,
        first.curvatureModel);
    addAvbdRigidWorldPointQuasiNewtonGeometricDiagonal(
        blockA,
        stateA,
        secondWorldPointA,
        force.y() * second.axis,
        second.curvatureModel);
    addAvbdRigidWorldPointQuasiNewtonGeometricDiagonal(
        blockB,
        stateB,
        firstWorldPointB,
        -force.x() * first.axis,
        first.curvatureModel);
    addAvbdRigidWorldPointQuasiNewtonGeometricDiagonal(
        blockB,
        stateB,
        secondWorldPointB,
        -force.y() * second.axis,
        second.curvatureModel);
  }
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
inline bool updateAvbdRigidPointPairFrictionTangentPairForLimit(
    AvbdRigidPointPairRow& first,
    AvbdRigidPointPairRow& second,
    const AvbdRigidBodyState& stateA,
    const AvbdRigidBodyState& stateB,
    const AvbdRigidPointPairFrictionOptions& options,
    double forceLimit)
{
  const double limit = std::isnan(forceLimit) ? 0.0 : std::max(0.0, forceLimit);
  first.bounds = avbdFrictionTangentBounds(limit);
  second.bounds = first.bounds;
  if (options.fixedPenalty) {
    // Fixed-penalty VBD recomputes k*C from the current configuration on the
    // next block sweep. It has no friction dual and no stiffness evolution.
    first.state.lambda = 0.0;
    second.state.lambda = 0.0;
    return false;
  }

  bool clamped = false;
  const Eigen::Vector2d constraintValues = avbdRigidPointPairConstraintValues(
      stateA, stateB, first, second, options.alpha);
  const Eigen::Vector2d force
      = avbdRigidPointPairFrictionTangentPairForceFromConstraintValuesAndLimit(
          constraintValues, first, second, limit, &clamped);

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
  // The reference implementation grows stiffness whenever the trial was
  // accepted, but retains a static anchor only when the accepted solution also
  // has negligible tangential displacement. These are distinct decisions.
  return !clamped
         && constraintValues.norm()
                < std::max(0.0, options.staticFrictionTolerance);
}

//==============================================================================
inline void updateAvbdRigidPointPairFrictionTangentPair(
    AvbdRigidPointPairRow& first,
    AvbdRigidPointPairRow& second,
    const AvbdRigidBodyState& stateA,
    const AvbdRigidBodyState& stateB,
    const AvbdRigidPointPairFrictionOptions& options)
{
  (void)updateAvbdRigidPointPairFrictionTangentPairForLimit(
      first,
      second,
      stateA,
      stateB,
      options,
      avbdRigidPointPairFrictionPairForceLimit(first, second));
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
struct AvbdRigidPreviousFrictionState
{
  AvbdScalarRowKey key;
  Eigen::Vector3d direction = Eigen::Vector3d::Zero();
};

//==============================================================================
struct AvbdRigidContactManifoldRowScratch
{
  using ContactAllocator
      = ::dart::common::StlAllocator<AvbdRigidContactManifoldPoint>;
  using ContactLocalPointAllocator = ::dart::common::StlAllocator<
      std::pair<Eigen::Vector3d, Eigen::Vector3d>>;
  using DescriptorAllocator
      = ::dart::common::StlAllocator<AvbdScalarRowDescriptor>;
  using FrictionDirectionAllocator
      = ::dart::common::StlAllocator<AvbdRigidPreviousFrictionState>;
  using ContactIdentityAllocator
      = ::dart::common::StlAllocator<AvbdRigidContactIdentityState>;
  using ContactAnchorAllocator
      = ::dart::common::StlAllocator<AvbdContactTangentAnchorState>;

  AvbdRigidContactManifoldRowScratch() = default;

  /// Number of manifold builds that discarded every contact row's continuation
  /// because a multi-point group's identity could not be proven (a rank moved
  /// by more than a tenth of the group's point separation, a point was
  /// replaced, or a local point was non-finite). Diagnostic only: the
  /// cold-start itself is the fail-closed behaviour, this makes it auditable.
  std::size_t contactIdentityColdStarts = 0;

  explicit AvbdRigidContactManifoldRowScratch(
      ::dart::common::MemoryAllocator& allocator)
    : activeContacts(ContactAllocator{allocator}),
      contactLocalPoints(ContactLocalPointAllocator{allocator}),
      normalDescriptors(DescriptorAllocator{allocator}),
      frictionDescriptors(DescriptorAllocator{allocator}),
      previousFrictionDirections(FrictionDirectionAllocator{allocator}),
      contactIdentities(ContactIdentityAllocator{allocator}),
      previousContactIdentities(ContactIdentityAllocator{allocator}),
      contactTangentAnchors(ContactAnchorAllocator{allocator}),
      previousContactTangentAnchors(ContactAnchorAllocator{allocator})
  {
  }

  template <
      typename Allocator,
      typename
      = std::enable_if_t<std::is_constructible_v<ContactAllocator, Allocator>>>
  explicit AvbdRigidContactManifoldRowScratch(const Allocator& allocator)
    : activeContacts(ContactAllocator{allocator}),
      contactLocalPoints(ContactLocalPointAllocator{allocator}),
      normalDescriptors(DescriptorAllocator{allocator}),
      frictionDescriptors(DescriptorAllocator{allocator}),
      previousFrictionDirections(FrictionDirectionAllocator{allocator}),
      contactIdentities(ContactIdentityAllocator{allocator}),
      previousContactIdentities(ContactIdentityAllocator{allocator}),
      contactTangentAnchors(ContactAnchorAllocator{allocator}),
      previousContactTangentAnchors(ContactAnchorAllocator{allocator})
  {
  }

  void clearContinuationState() noexcept
  {
    contactIdentities.clear();
    previousContactIdentities.clear();
    contactTangentAnchors.clear();
    previousContactTangentAnchors.clear();
    previousFrictionDirections.clear();
  }

  std::vector<AvbdRigidContactManifoldPoint, ContactAllocator> activeContacts;
  std::vector<
      std::pair<Eigen::Vector3d, Eigen::Vector3d>,
      ContactLocalPointAllocator>
      contactLocalPoints;
  std::vector<AvbdScalarRowDescriptor, DescriptorAllocator> normalDescriptors;
  std::vector<AvbdScalarRowDescriptor, DescriptorAllocator> frictionDescriptors;
  std::vector<AvbdRigidPreviousFrictionState, FrictionDirectionAllocator>
      previousFrictionDirections;
  std::vector<AvbdRigidContactIdentityState, ContactIdentityAllocator>
      contactIdentities;
  std::vector<AvbdRigidContactIdentityState, ContactIdentityAllocator>
      previousContactIdentities;
  std::vector<AvbdContactTangentAnchorState, ContactAnchorAllocator>
      contactTangentAnchors;
  std::vector<AvbdContactTangentAnchorState, ContactAnchorAllocator>
      previousContactTangentAnchors;
};

struct AvbdRigidPointJointActiveAxis
{
  const AvbdRigidPointJoint* joint = nullptr;
  std::uint8_t axis = 0;
};

struct AvbdRigidPointJointRowScratch
{
  using ActiveAxisAllocator
      = ::dart::common::StlAllocator<AvbdRigidPointJointActiveAxis>;
  using DescriptorAllocator
      = ::dart::common::StlAllocator<AvbdScalarRowDescriptor>;

  AvbdRigidPointJointRowScratch() = default;

  explicit AvbdRigidPointJointRowScratch(
      ::dart::common::MemoryAllocator& allocator)
    : activeRows(ActiveAxisAllocator{allocator}),
      descriptors(DescriptorAllocator{allocator})
  {
  }

  template <
      typename Allocator,
      typename = std::enable_if_t<
          std::is_constructible_v<ActiveAxisAllocator, Allocator>>>
  explicit AvbdRigidPointJointRowScratch(const Allocator& allocator)
    : activeRows(ActiveAxisAllocator{allocator}),
      descriptors(DescriptorAllocator{allocator})
  {
  }

  std::vector<AvbdRigidPointJointActiveAxis, ActiveAxisAllocator> activeRows;
  std::vector<AvbdScalarRowDescriptor, DescriptorAllocator> descriptors;
};

struct AvbdRigidAngularMotorRowScratch
{
  using MotorAllocator
      = ::dart::common::StlAllocator<const AvbdRigidAngularMotor*>;
  using DescriptorAllocator
      = ::dart::common::StlAllocator<AvbdScalarRowDescriptor>;

  AvbdRigidAngularMotorRowScratch() = default;

  explicit AvbdRigidAngularMotorRowScratch(
      ::dart::common::MemoryAllocator& allocator)
    : activeRows(MotorAllocator{allocator}),
      descriptors(DescriptorAllocator{allocator})
  {
  }

  template <
      typename Allocator,
      typename
      = std::enable_if_t<std::is_constructible_v<MotorAllocator, Allocator>>>
  explicit AvbdRigidAngularMotorRowScratch(const Allocator& allocator)
    : activeRows(MotorAllocator{allocator}),
      descriptors(DescriptorAllocator{allocator})
  {
  }

  std::vector<const AvbdRigidAngularMotor*, MotorAllocator> activeRows;
  std::vector<AvbdScalarRowDescriptor, DescriptorAllocator> descriptors;
};

struct AvbdRigidMotorRowScratch
{
  using LinearMotorPointerAllocator
      = ::dart::common::StlAllocator<const AvbdRigidLinearMotor*>;
  using AngularMotorPointerAllocator
      = ::dart::common::StlAllocator<const AvbdRigidAngularMotor*>;

  AvbdRigidMotorRowScratch() = default;

  explicit AvbdRigidMotorRowScratch(::dart::common::MemoryAllocator& allocator)
    : activeLinearRows(LinearMotorPointerAllocator{allocator}),
      activeAngularRows(AngularMotorPointerAllocator{allocator})
  {
  }

  template <
      typename Allocator,
      typename = std::enable_if_t<
          std::is_constructible_v<LinearMotorPointerAllocator, Allocator>>>
  explicit AvbdRigidMotorRowScratch(const Allocator& allocator)
    : activeLinearRows(LinearMotorPointerAllocator{allocator}),
      activeAngularRows(AngularMotorPointerAllocator{allocator})
  {
  }

  void clear()
  {
    activeLinearRows.clear();
    activeAngularRows.clear();
  }

  std::vector<const AvbdRigidLinearMotor*, LinearMotorPointerAllocator>
      activeLinearRows;
  std::vector<const AvbdRigidAngularMotor*, AngularMotorPointerAllocator>
      activeAngularRows;
};

struct AvbdRigidDistanceSpringRowScratch
{
  using DistanceSpringPointerAllocator = ::dart::common::StlAllocator<
      const AvbdRigidBodyPointPairDistanceSpringRow*>;

  AvbdRigidDistanceSpringRowScratch() = default;

  explicit AvbdRigidDistanceSpringRowScratch(
      ::dart::common::MemoryAllocator& allocator)
    : activeRows(DistanceSpringPointerAllocator{allocator})
  {
  }

  template <
      typename Allocator,
      typename = std::enable_if_t<
          std::is_constructible_v<DistanceSpringPointerAllocator, Allocator>>>
  explicit AvbdRigidDistanceSpringRowScratch(const Allocator& allocator)
    : activeRows(DistanceSpringPointerAllocator{allocator})
  {
  }

  void clear()
  {
    activeRows.clear();
  }

  std::vector<
      const AvbdRigidBodyPointPairDistanceSpringRow*,
      DistanceSpringPointerAllocator>
      activeRows;
};

//==============================================================================
template <typename NormalRowVector, typename FrictionRowVector>
inline void buildAvbdRigidContactManifoldRows(
    std::span<const AvbdRigidBodyState> states,
    std::span<const AvbdRigidContactManifoldPoint> contacts,
    AvbdScalarRowInventory& normalInventory,
    AvbdScalarRowInventory& frictionInventory,
    NormalRowVector& normalRows,
    FrictionRowVector& frictionRows,
    AvbdRigidContactManifoldRowScratch& scratch,
    const AvbdRowWarmStartOptions& warmStartOptions = {})
{
  const auto appendActiveRows =
      [&](std::span<const AvbdRigidContactManifoldPoint> activeContacts,
          std::span<const AvbdScalarRowDescriptor> normalDescriptors,
          std::span<std::pair<Eigen::Vector3d, Eigen::Vector3d>>
              contactLocalPoints,
          std::span<AvbdScalarRowDescriptor> frictionDescriptors,
          std::span<AvbdRigidPreviousFrictionState>
              previousFrictionDirectionStorage) {
        for (std::size_t i = 0; i < activeContacts.size(); ++i) {
          const AvbdRigidContactManifoldPoint& contact = activeContacts[i];
          contactLocalPoints[i]
              = {avbdRigidBodyLocalPoint(states[contact.bodyA], contact.point),
                 avbdRigidBodyLocalPoint(states[contact.bodyB], contact.point)};
        }

        // A multi-point spatial rank is deterministic only while the same set
        // of material contact points survives. If a lower-ranked point vanishes
        // or a replacement appears, an unchanged scalar key can otherwise
        // inherit another point's normal lambda and sticking anchor. Prove each
        // multi-point feature group's correspondence against canonical local
        // points before reusing either inventory; a singleton has no rank peer
        // and may keep its anchor while its detected point moves.
        const auto groupTuple = [](const AvbdScalarRowKey& key) {
          return std::tuple{
              key.objectA, key.objectB, key.featureA, key.featureB};
        };
        const auto canonicalDetectedAnchor = [](const auto& contact,
                                                const auto& localPoints) {
          AvbdContactTangentAnchorState anchor;
          anchor.key = makeAvbdContactTangentAnchorKey(
              contact.endpointA, contact.endpointB, contact.row);
          const bool endpointsReversed = contact.endpointB < contact.endpointA;
          anchor.localPointA
              = endpointsReversed ? localPoints.second : localPoints.first;
          anchor.localPointB
              = endpointsReversed ? localPoints.first : localPoints.second;
          anchor.valid = true;
          return anchor;
        };
        const auto exactPoint
            = [](const Eigen::Vector3d& lhs, const Eigen::Vector3d& rhs) {
                return (lhs.array() == rhs.array()).all();
              };
        // Joint distance of a contact identity's two canonical local points.
        const auto identityPointDistance
            = [](const auto& lhs, const auto& rhs) {
                return std::sqrt(
                    (lhs.localPointA - rhs.localPointA).squaredNorm()
                    + (lhs.localPointB - rhs.localPointB).squaredNorm());
              };
        // Smallest joint distance between two distinct identities of a group.
        const auto groupPointSeparation
            = [&](const auto groupBegin, std::size_t groupSize) {
                double separation = std::numeric_limits<double>::infinity();
                for (std::size_t i = 0u; i < groupSize; ++i) {
                  for (std::size_t j = i + 1u; j < groupSize; ++j) {
                    separation = std::min(
                        separation,
                        identityPointDistance(groupBegin[i], groupBegin[j]));
                  }
                }
                return separation;
              };
        const auto pointLess
            = [](const Eigen::Vector3d& lhs, const Eigen::Vector3d& rhs) {
                for (Eigen::Index axis = 0; axis < 3; ++axis) {
                  if (lhs[axis] < rhs[axis]) {
                    return true;
                  }
                  if (rhs[axis] < lhs[axis]) {
                    return false;
                  }
                }
                return false;
              };

        const std::size_t previousNormalCount
            = scratch.contactIdentities.size();
        const std::size_t currentNormalCount = activeContacts.size();
        const std::size_t identityCount
            = previousNormalCount + currentNormalCount;
        auto& identityOrder = scratch.previousContactIdentities;
        identityOrder.clear();
        const std::size_t reusableIdentityCapacity
            = currentNormalCount <= std::numeric_limits<std::size_t>::max() / 2u
                  ? 2u * currentNormalCount
                  : identityCount;
        identityOrder.reserve(
            std::max(identityCount, reusableIdentityCapacity));
        identityOrder.insert(
            identityOrder.end(),
            scratch.contactIdentities.begin(),
            scratch.contactIdentities.end());

        auto& currentIdentityState = scratch.contactIdentities;
        currentIdentityState.resize(currentNormalCount);
        identityOrder.resize(identityCount);
        for (std::size_t i = 0u; i < currentNormalCount; ++i) {
          const AvbdContactTangentAnchorState detected
              = canonicalDetectedAnchor(
                  activeContacts[i], contactLocalPoints[i]);
          currentIdentityState[i] = AvbdRigidContactIdentityState{
              normalDescriptors[i].key,
              detected.localPointA,
              detected.localPointB};
          identityOrder[previousNormalCount + i] = currentIdentityState[i];
        }
        auto previousNormalIdentities
            = std::span<AvbdRigidContactIdentityState>{identityOrder}.first(
                previousNormalCount);
        auto currentNormalIdentities
            = std::span<AvbdRigidContactIdentityState>{identityOrder}.subspan(
                previousNormalCount, currentNormalCount);

        const auto identityPointLess = [&](const auto& lhs, const auto& rhs) {
          const auto lhsGroup = groupTuple(lhs.key);
          const auto rhsGroup = groupTuple(rhs.key);
          if (lhsGroup != rhsGroup) {
            return lhsGroup < rhsGroup;
          }
          if (pointLess(lhs.localPointA, rhs.localPointA)) {
            return true;
          }
          if (pointLess(rhs.localPointA, lhs.localPointA)) {
            return false;
          }
          if (pointLess(lhs.localPointB, rhs.localPointB)) {
            return true;
          }
          if (pointLess(rhs.localPointB, lhs.localPointB)) {
            return false;
          }
          return lhs.key < rhs.key;
        };
        std::sort(
            previousNormalIdentities.begin(),
            previousNormalIdentities.end(),
            identityPointLess);
        std::sort(
            currentNormalIdentities.begin(),
            currentNormalIdentities.end(),
            identityPointLess);

        bool ambiguousContactIdentity = false;
        // Equal canonical points have no stable discriminator: the collision
        // query's transient order is the final row-order tie breaker, so two
        // duplicates may exchange distinct normals/depths between frames.
        // Sort once and reject adjacent duplicates instead of comparing the
        // entire global contact envelope quadratically.
        const auto hasAdjacentDuplicate = [&](const auto identities) {
          for (std::size_t i = 1u; i < identities.size(); ++i) {
            const auto& first = identities[i - 1u];
            const auto& second = identities[i];
            if (groupTuple(first.key) == groupTuple(second.key)
                && exactPoint(first.localPointA, second.localPointA)
                && exactPoint(first.localPointB, second.localPointB)) {
              return true;
            }
          }
          return false;
        };
        ambiguousContactIdentity
            = hasAdjacentDuplicate(currentNormalIdentities)
              || hasAdjacentDuplicate(previousNormalIdentities);

        // A lone feature-pair contact has no spatial-rank peer with which it
        // can alias, so its detector point may move while the persistent
        // static-friction anchor stays attached. Multi-point groups must match
        // cardinality and every canonical material point exactly; otherwise a
        // rank can silently inherit another point's dual and anchor.
        for (std::size_t currentIndex = 0u;
             !ambiguousContactIdentity
             && currentIndex < currentNormalIdentities.size();) {
          const auto group
              = groupTuple(currentNormalIdentities[currentIndex].key);
          const auto currentEnd = std::upper_bound(
              currentNormalIdentities.begin() + currentIndex,
              currentNormalIdentities.end(),
              group,
              [&](const auto& target, const auto& value) {
                return target < groupTuple(value.key);
              });
          const auto previousBegin = std::lower_bound(
              previousNormalIdentities.begin(),
              previousNormalIdentities.end(),
              group,
              [&](const auto& value, const auto& target) {
                return groupTuple(value.key) < target;
              });
          const auto previousEnd = std::upper_bound(
              previousBegin,
              previousNormalIdentities.end(),
              group,
              [&](const auto& target, const auto& value) {
                return target < groupTuple(value.key);
              });
          const std::size_t currentGroupSize = static_cast<std::size_t>(
              currentEnd - (currentNormalIdentities.begin() + currentIndex));
          const std::size_t previousGroupSize
              = static_cast<std::size_t>(previousEnd - previousBegin);
          if (previousGroupSize != 0u) {
            if (previousGroupSize != currentGroupSize) {
              ambiguousContactIdentity = true;
              break;
            }
            const auto currentBegin
                = currentNormalIdentities.begin() + currentIndex;
            if (currentGroupSize == 1u) {
              ambiguousContactIdentity
                  = !(previousBegin->key == currentBegin->key);
            } else {
              // Same-rank points must stay within a tenth of the group's
              // smallest point separation of their previous rank-mate. Settling
              // and sliding move every material point by a small fraction of
              // that separation per step, whereas a vanished point with a
              // replacement moves at least one rank by a large fraction of it
              // and must cold-start. The comparison is written so that a
              // non-finite tolerance (an all-NaN group leaves the separation
              // at infinity) or a non-finite point distance also cold-starts:
              // a poisoned identity never inherits continuation.
              const double tolerance
                  = 0.1 * groupPointSeparation(currentBegin, currentGroupSize);
              for (std::size_t offset = 0u; offset < currentGroupSize;
                   ++offset) {
                const auto& previous = previousBegin[offset];
                const auto& current = currentBegin[offset];
                if (!(previous.key == current.key) || !std::isfinite(tolerance)
                    || !(
                        identityPointDistance(previous, current)
                        <= tolerance)) {
                  ambiguousContactIdentity = true;
                  break;
                }
              }
            }
          }
          currentIndex += currentGroupSize;
        }
        if (ambiguousContactIdentity) {
          ++scratch.contactIdentityColdStarts;
          normalInventory.records().clear();
          frictionInventory.records().clear();
        }

        normalInventory.reserve(normalDescriptors.size());
        normalInventory.syncActiveRows(normalDescriptors, warmStartOptions);

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
        const auto hasFriction = [&](std::size_t i) {
          const double coefficient = activeContacts[i].frictionCoefficient;
          return std::isfinite(coefficient) && coefficient > 0.0;
        };

        std::size_t frictionDescriptorCount = 0u;
        for (std::size_t i = 0; i < activeContacts.size(); ++i) {
          const AvbdRigidContactManifoldPoint& contact = activeContacts[i];
          const double forceLimit = frictionForceLimit(i);
          if (!hasFriction(i)) {
            continue;
          }
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
        const std::span<const AvbdScalarRowDescriptor>
            activeFrictionDescriptors{
                frictionDescriptors.data(), frictionDescriptorCount};

        auto& previousContactAnchors = scratch.previousContactTangentAnchors;
        previousContactAnchors.clear();
        previousContactAnchors.reserve(
            std::max(
                scratch.contactTangentAnchors.size(), activeContacts.size()));
        previousContactAnchors.insert(
            previousContactAnchors.end(),
            scratch.contactTangentAnchors.begin(),
            scratch.contactTangentAnchors.end());
        std::sort(
            previousContactAnchors.begin(),
            previousContactAnchors.end(),
            [](const auto& lhs, const auto& rhs) { return lhs.key < rhs.key; });
        if (ambiguousContactIdentity) {
          previousContactAnchors.clear();
        }
        auto& contactAnchors = scratch.contactTangentAnchors;
        contactAnchors.clear();
        contactAnchors.reserve(frictionDescriptorCount / 2u);

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
          initializeAvbdRigidPointPairTaylorLinearization(
              indexedRow.row,
              states[indexedRow.bodyA],
              states[indexedRow.bodyB]);
          normalRows.push_back(indexedRow);
        }

        if (activeFrictionDescriptors.empty()) {
          frictionInventory.syncActiveRows(
              activeFrictionDescriptors, warmStartOptions);
          frictionRows.clear();
          return;
        }

        std::span<AvbdRigidPreviousFrictionState> previousFrictionDirections;
        const std::size_t previousFrictionCount = frictionInventory.size();
        const auto capturePreviousFrictionState
            = [](const AvbdScalarRowRecord& record) {
                AvbdRigidPreviousFrictionState previous;
                previous.key = record.descriptor.key;
                previous.direction = record.direction;
                return previous;
              };
        if (!previousFrictionDirectionStorage.empty()
            && previousFrictionCount
                   <= previousFrictionDirectionStorage.size()) {
          for (std::size_t i = 0u; i < previousFrictionCount; ++i) {
            previousFrictionDirectionStorage[i]
                = capturePreviousFrictionState(frictionInventory[i]);
          }
          previousFrictionDirections = {
              previousFrictionDirectionStorage.data(), previousFrictionCount};
        } else {
          auto& previousFrictionDirectionScratch
              = scratch.previousFrictionDirections;
          previousFrictionDirectionScratch.clear();
          previousFrictionDirectionScratch.reserve(
              std::max(
                  previousFrictionCount, activeFrictionDescriptors.size()));
          for (const AvbdScalarRowRecord& record :
               frictionInventory.records()) {
            previousFrictionDirectionScratch.push_back(
                capturePreviousFrictionState(record));
          }
          previousFrictionDirections = previousFrictionDirectionScratch;
        }
        std::sort(
            previousFrictionDirections.begin(),
            previousFrictionDirections.end(),
            [](const auto& lhs, const auto& rhs) { return lhs.key < rhs.key; });
        const auto findPreviousFrictionState
            = [&previousFrictionDirections](const AvbdScalarRowKey& key)
            -> const AvbdRigidPreviousFrictionState* {
          const auto found = std::lower_bound(
              previousFrictionDirections.begin(),
              previousFrictionDirections.end(),
              key,
              [](const auto& value, const auto& target) {
                return value.key < target;
              });
          return found != previousFrictionDirections.end() && found->key == key
                     ? &*found
                     : nullptr;
        };
        const auto findPreviousContactAnchor
            = [&previousContactAnchors](
                  const AvbdContactTangentAnchorKey& anchorKey)
            -> const AvbdContactTangentAnchorState* {
          const auto found = std::lower_bound(
              previousContactAnchors.begin(),
              previousContactAnchors.end(),
              anchorKey,
              [](const auto& value, const auto& target) {
                return value.key < target;
              });
          return found != previousContactAnchors.end() && found->valid
                         && found->localPointA.allFinite()
                         && found->localPointB.allFinite()
                         && found->key == anchorKey
                     ? &*found
                     : nullptr;
        };

        frictionInventory.reserve(frictionDescriptorCount);
        frictionInventory.syncActiveRows(
            activeFrictionDescriptors, warmStartOptions);

        // The inventory is now at its final size and contactAnchors reserved
        // one slot per friction pair above. No capacity-changing operation on
        // either owner occurs after their addresses escape into frictionRows.

        frictionRows.clear();
        frictionRows.reserve(activeContacts.size());
        std::size_t firstRecordIndex = 0u;
        for (std::size_t contactIndex = 0; contactIndex < activeContacts.size();
             ++contactIndex) {
          if (!hasFriction(contactIndex)) {
            continue;
          }
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
          const auto& detectedLocalPoints = contactLocalPoints[contactIndex];
          const Eigen::Vector3d stepStartRelativePosition
              = Eigen::Vector3d::Zero();
          const Eigen::Matrix<double, 3, 2> basis
              = avbdRigidContactTangentBasis(contact.normalFromAtoB);
          const bool endpointsReversed = contact.endpointB < contact.endpointA;
          // Scalar tangent lambda is the force on transient endpoint A. Store
          // directions as force on the canonical first endpoint instead, so
          // warm-start projection keeps the same physical force if collision
          // detection reverses A/B (and therefore also reverses the normal).
          const double canonicalDirectionSign = endpointsReversed ? -1.0 : 1.0;
          const Eigen::Vector3d canonicalFirstDirection
              = canonicalDirectionSign * basis.col(0);
          const Eigen::Vector3d canonicalSecondDirection
              = canonicalDirectionSign * basis.col(1);
          const auto forceLimitFromBounds = [](AvbdScalarRowBounds bounds) {
            const double lowerLimit = bounds.lower < 0.0 ? -bounds.lower : 0.0;
            const double upperLimit = bounds.upper > 0.0 ? bounds.upper : 0.0;
            return std::max(0.0, std::min(lowerLimit, upperLimit));
          };
          const double forceLimit = std::min(
              forceLimitFromBounds(firstRecord.descriptor.bounds),
              forceLimitFromBounds(secondRecord.descriptor.bounds));
          const AvbdRigidPreviousFrictionState* previousFirst
              = findPreviousFrictionState(firstRecord.descriptor.key);
          const AvbdRigidPreviousFrictionState* previousSecond
              = findPreviousFrictionState(secondRecord.descriptor.key);
          const AvbdContactTangentAnchorState* previousAnchor
              = findPreviousContactAnchor(
                  makeAvbdContactTangentAnchorKey(firstRecord.descriptor.key));
          bool previousDualInsideCone = false;
          if (previousFirst != nullptr && previousSecond != nullptr
              && isValidAvbdRigidContactFrictionDirection(
                  previousFirst->direction)
              && isValidAvbdRigidContactFrictionDirection(
                  previousSecond->direction)) {
            const Eigen::Vector2d projected
                = projectAvbdFrictionDualToTangentPair(
                    firstRecord.state.lambda,
                    secondRecord.state.lambda,
                    previousFirst->direction,
                    previousSecond->direction,
                    canonicalFirstDirection,
                    canonicalSecondDirection);
            if (projected.allFinite()) {
              const double projectedNorm = projected.norm();
              previousDualInsideCone
                  = !std::isfinite(forceLimit) || projectedNorm <= forceLimit;
              const double radialScale
                  = !previousDualInsideCone && projectedNorm > 0.0
                        ? forceLimit / projectedNorm
                        : 1.0;
              firstRecord.state.lambda = clampAvbdRowForce(
                  radialScale * projected.x(), firstRecord.descriptor.bounds);
              secondRecord.state.lambda = clampAvbdRowForce(
                  radialScale * projected.y(), secondRecord.descriptor.bounds);
            } else {
              firstRecord.state.lambda = 0.0;
              secondRecord.state.lambda = 0.0;
            }
          } else {
            firstRecord.state.lambda = 0.0;
            secondRecord.state.lambda = 0.0;
          }
          firstRecord.direction = canonicalFirstDirection;
          secondRecord.direction = canonicalSecondDirection;

          const bool reusePreviousAnchor = previousAnchor != nullptr
                                           && previousAnchor->sticking
                                           && previousDualInsideCone;
          Eigen::Vector3d localPointA = detectedLocalPoints.first;
          Eigen::Vector3d localPointB = detectedLocalPoints.second;
          if (reusePreviousAnchor) {
            localPointA = endpointsReversed ? previousAnchor->localPointB
                                            : previousAnchor->localPointA;
            localPointB = endpointsReversed ? previousAnchor->localPointA
                                            : previousAnchor->localPointB;
          }

          AvbdContactTangentAnchorState contactAnchor;
          contactAnchor.key = makeAvbdContactTangentAnchorKey(
              contact.endpointA, contact.endpointB, contact.row);
          contactAnchor.localPointA
              = endpointsReversed ? localPointB : localPointA;
          contactAnchor.localPointB
              = endpointsReversed ? localPointA : localPointB;
          contactAnchor.valid = true;
          // New and previously sliding contacts reset to the detected feature
          // points. Only a solve can promote that fresh anchor to sticking.
          contactAnchor.sticking = reusePreviousAnchor;
          contactAnchors.push_back(contactAnchor);

          AvbdRigidBodyPointPairFrictionRows indexedRows;
          indexedRows.bodyA = contact.bodyA;
          indexedRows.bodyB = contact.bodyB;
          indexedRows.normalRowIndex = contactIndex;
          indexedRows.frictionCoefficient = contact.frictionCoefficient;
          indexedRows.persistentFirstRecord = &firstRecord;
          indexedRows.persistentSecondRecord = &secondRecord;
          indexedRows.persistentAnchor = &contactAnchors.back();
          indexedRows.first = makeAvbdRigidContactFrictionTangentRow(
              localPointA,
              localPointB,
              basis.col(0),
              stepStartRelativePosition,
              forceLimitFromBounds(firstRecord.descriptor.bounds),
              firstRecord.state,
              0.0);
          indexedRows.first.bounds = firstRecord.descriptor.bounds;
          initializeAvbdRigidPointPairTaylorLinearization(
              indexedRows.first,
              states[indexedRows.bodyA],
              states[indexedRows.bodyB]);
          indexedRows.second = makeAvbdRigidContactFrictionTangentRow(
              localPointA,
              localPointB,
              basis.col(1),
              stepStartRelativePosition,
              forceLimitFromBounds(secondRecord.descriptor.bounds),
              secondRecord.state,
              0.0);
          indexedRows.second.bounds = secondRecord.descriptor.bounds;
          initializeAvbdRigidPointPairTaylorLinearization(
              indexedRows.second,
              states[indexedRows.bodyA],
              states[indexedRows.bodyB]);
          frictionRows.push_back(indexedRows);
          firstRecordIndex += 2u;
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
        AvbdRigidPreviousFrictionState,
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
        std::span<AvbdRigidPreviousFrictionState>{
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
template <typename LinearRowVector>
inline void buildAvbdRigidPointJointRows(
    std::span<const AvbdRigidBodyState> states,
    std::span<const AvbdRigidPointJoint> joints,
    AvbdScalarRowInventory& linearInventory,
    LinearRowVector& linearRows,
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
template <typename LinearRowVector>
inline void buildAvbdRigidPointJointRows(
    std::span<const AvbdRigidBodyState> states,
    std::span<const AvbdRigidPointJoint> joints,
    AvbdScalarRowInventory& linearInventory,
    LinearRowVector& linearRows,
    const AvbdRowWarmStartOptions& warmStartOptions = {})
{
  AvbdRigidPointJointRowScratch scratch;
  buildAvbdRigidPointJointRows(
      states, joints, linearInventory, linearRows, scratch, warmStartOptions);
}

//==============================================================================
template <typename AngularRowVector>
inline void buildAvbdRigidPointJointAngularRows(
    std::span<const AvbdRigidBodyState> states,
    std::span<const AvbdRigidPointJoint> joints,
    AvbdScalarRowInventory& angularInventory,
    AngularRowVector& angularRows,
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
template <typename AngularRowVector>
inline void buildAvbdRigidPointJointAngularRows(
    std::span<const AvbdRigidBodyState> states,
    std::span<const AvbdRigidPointJoint> joints,
    AvbdScalarRowInventory& angularInventory,
    AngularRowVector& angularRows,
    const AvbdRowWarmStartOptions& warmStartOptions = {})
{
  AvbdRigidPointJointRowScratch scratch;
  buildAvbdRigidPointJointAngularRows(
      states, joints, angularInventory, angularRows, scratch, warmStartOptions);
}

//==============================================================================
template <typename MotorRowVector>
inline void buildAvbdRigidAngularMotorRows(
    std::span<const AvbdRigidBodyState> states,
    std::span<const AvbdRigidAngularMotor> motors,
    AvbdScalarRowInventory& motorInventory,
    MotorRowVector& motorRows,
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
template <typename MotorRowVector>
inline void buildAvbdRigidAngularMotorRows(
    std::span<const AvbdRigidBodyState> states,
    std::span<const AvbdRigidAngularMotor> motors,
    AvbdScalarRowInventory& motorInventory,
    MotorRowVector& motorRows,
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
template <typename LinearMotorRowVector, typename AngularMotorRowVector>
inline void buildAvbdRigidMotorRows(
    std::span<const AvbdRigidBodyState> states,
    std::span<const AvbdRigidLinearMotor> linearMotors,
    std::span<const AvbdRigidAngularMotor> angularMotors,
    AvbdScalarRowInventory& motorInventory,
    LinearMotorRowVector& linearMotorRows,
    AngularMotorRowVector& angularMotorRows,
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
              AvbdScalarRowRole::MotorLinear,
              motor.endpointA,
              motor.endpointB,
              motor.row,
              /*axis=*/0);
        }

        const AvbdRigidAngularMotor& motor
            = *activeAngularRows[index - activeLinearRows.size()];
        return makeAvbdEndpointPairRowKey(
            AvbdScalarRowRole::MotorAngular,
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
template <typename LinearMotorRowVector, typename AngularMotorRowVector>
inline void buildAvbdRigidMotorRows(
    std::span<const AvbdRigidBodyState> states,
    std::span<const AvbdRigidLinearMotor> linearMotors,
    std::span<const AvbdRigidAngularMotor> angularMotors,
    AvbdScalarRowInventory& motorInventory,
    LinearMotorRowVector& linearMotorRows,
    AngularMotorRowVector& angularMotorRows,
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
template <typename LinearRowVector, typename AngularRowVector>
inline void buildAvbdRigidPointJointConstraintRows(
    std::span<const AvbdRigidBodyState> states,
    std::span<const AvbdRigidPointJoint> joints,
    AvbdScalarRowInventory& linearInventory,
    AvbdScalarRowInventory& angularInventory,
    LinearRowVector& linearRows,
    AngularRowVector& angularRows,
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
template <typename LinearRowVector, typename AngularRowVector>
inline void buildAvbdRigidPointJointConstraintRows(
    std::span<const AvbdRigidBodyState> states,
    std::span<const AvbdRigidPointJoint> joints,
    AvbdScalarRowInventory& linearInventory,
    AvbdScalarRowInventory& angularInventory,
    LinearRowVector& linearRows,
    AngularRowVector& angularRows,
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
template <typename SpringRowVector>
inline void buildAvbdRigidDistanceSpringRows(
    std::span<const AvbdRigidBodyState> states,
    std::span<const AvbdRigidBodyPointPairDistanceSpringRow> springs,
    AvbdScalarRowInventory& springInventory,
    SpringRowVector& springRows,
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
template <typename SpringRowVector>
inline void buildAvbdRigidDistanceSpringRows(
    std::span<const AvbdRigidBodyState> states,
    std::span<const AvbdRigidBodyPointPairDistanceSpringRow> springs,
    AvbdScalarRowInventory& springInventory,
    SpringRowVector& springRows,
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
template <
    typename Rows,
    typename KeyFn,
    typename KeyVector,
    typename OffsetVector>
inline bool avbdRigidRowIndexLayoutMatches(
    const Rows& rows,
    std::size_t bodyCount,
    std::size_t cachedBodyCount,
    const KeyVector& cachedKeys,
    const OffsetVector& offsets,
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
template <typename Rows, typename KeyFn, typename KeyVector>
inline void avbdRigidRefreshRowIndexLayoutKeys(
    const Rows& rows,
    std::size_t bodyCount,
    std::size_t& cachedBodyCount,
    KeyVector& cachedKeys,
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
/// Run the rigid AVBD primal block sweep and persistent row-state update.
///
/// Body blocks retain deterministic serial order. When `rowUpdateExecutor` is
/// provided, independent attachment, point-pair, distance, angular, and
/// complete adjacent friction-pair updates use deterministic contiguous
/// ranges; incomplete friction layouts preserve the serial fallback.
template <
    typename AttachmentRowVector,
    typename PointPairRowVector,
    typename AngularPairRowVector,
    typename FrictionPairRowVector>
inline AvbdRigidBlockDescentStats blockDescentRigidBodiesAvbdRows(
    std::span<AvbdRigidBodyState> states,
    std::span<const double> masses,
    std::span<const Eigen::Matrix3d> bodyInertias,
    std::span<const std::uint8_t> fixed,
    std::span<const AvbdRigidBodyState> inertialTargets,
    double timeStep,
    AttachmentRowVector& attachmentRows,
    PointPairRowVector& pointPairRows,
    AngularPairRowVector& angularPairRows,
    FrictionPairRowVector& frictionPairRows,
    const AvbdRigidBlockDescentOptions& options,
    const AvbdRigidPointAttachmentOptions& rowOptions,
    const AvbdRigidPointPairFrictionOptions& frictionOptions,
    AvbdRigidBodyRowIndexScratch* rowIndexScratch = nullptr,
    std::span<AvbdRigidBodyPointPairDistanceSpringRow> distanceSpringRows = {},
    const AvbdRigidPointPairDistanceSpringOptions& distanceSpringOptions = {},
    compute::ComputeExecutor* rowUpdateExecutor = nullptr)
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

  // `states` is x_t on entry. Freeze every contact row's first-order model
  // before the first Gauss-Seidel body update mutates any pose. Reinitializing
  // here also makes manually assembled contact rows obey the same contract as
  // rows produced by the world-contact builder.
  for (AvbdRigidBodyPointPairRow& indexedRow : pointPairRows) {
    if (validBody(indexedRow.bodyA) && validBody(indexedRow.bodyB)
        && indexedRow.row.curvatureModel
               == AvbdRigidPointCurvatureModel::TaylorLinearized) {
      initializeAvbdRigidPointPairTaylorLinearization(
          indexedRow.row, states[indexedRow.bodyA], states[indexedRow.bodyB]);
    }
  }
  for (AvbdRigidBodyPointPairFrictionRows& indexedRows : frictionPairRows) {
    // Start the step with an empty Coulomb cone for every row that can derive
    // one from its normal row. `liveFrictionForceLimit` then raises it to the
    // largest normal force this contact transmits during the step and never
    // lowers it again. Rows without a resolvable normal row keep the bounds
    // their caller supplied.
    if (indexedRows.normalRowIndex < pointPairRows.size()
        && std::isfinite(indexedRows.frictionCoefficient)
        && indexedRows.frictionCoefficient > 0.0) {
      const AvbdRigidBodyPointPairRow& indexedNormal
          = pointPairRows[indexedRows.normalRowIndex];
      if (validBody(indexedNormal.bodyA) && validBody(indexedNormal.bodyB)
          && indexedNormal.bodyA == indexedRows.bodyA
          && indexedNormal.bodyB == indexedRows.bodyB) {
        indexedRows.first.bounds = avbdFrictionTangentBounds(0.0);
        indexedRows.second.bounds = indexedRows.first.bounds;
      }
    }
    if (validBody(indexedRows.bodyA) && validBody(indexedRows.bodyB)) {
      if (indexedRows.first.curvatureModel
          == AvbdRigidPointCurvatureModel::TaylorLinearized) {
        initializeAvbdRigidPointPairTaylorLinearization(
            indexedRows.first,
            states[indexedRows.bodyA],
            states[indexedRows.bodyB]);
      }
      if (indexedRows.second.curvatureModel
          == AvbdRigidPointCurvatureModel::TaylorLinearized) {
        initializeAvbdRigidPointPairTaylorLinearization(
            indexedRows.second,
            states[indexedRows.bodyA],
            states[indexedRows.bodyB]);
      }
    }
  }

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
        const AvbdRigidPointAttachmentOptions& effectiveRowOptions
            = avbdRigidPointPairSolveOptions(indexedRow, rowOptions);
        const AvbdRigidBodyState& stateA = states[indexedRow.bodyA];
        const AvbdRigidBodyState& stateB = states[indexedRow.bodyB];
        const double rawConstraintValue
            = avbdRigidPointPairConstraintValue(stateA, stateB, row);
        const double constraintValue
            = avbdRigidRowUsesFiniteMaterial(row.materialStiffness)
                  ? rawConstraintValue
                  : regularizeAvbdConstraintValue(
                        rawConstraintValue,
                        row.previousConstraintValue,
                        effectiveRowOptions.alpha);
        const double forceMagnitude = avbdRigidScalarRowForce(
            row.state, constraintValue, row.bounds, row.materialStiffness);

        if (indexedRow.bodyA == body) {
          const Vector6d direction = avbdRigidPointPairDirectionA(stateA, row);
          block.force.noalias() += forceMagnitude * direction;
          addAvbdRigidBlockHessianRankOneLowerTriangle(
              block, direction, row.state.stiffness);
          if (row.curvatureModel == AvbdRigidPointCurvatureModel::QuasiNewton) {
            const PointPairWorldPointCache& points
                = pointPairWorldPoints(indexedRow, cache);
            addAvbdRigidWorldPointQuasiNewtonGeometricDiagonal(
                block,
                stateA,
                points.worldPointA,
                forceMagnitude * row.axis,
                row.curvatureModel);
          }
        }
        if (indexedRow.bodyB == body && indexedRow.bodyB != indexedRow.bodyA) {
          const Vector6d direction = avbdRigidPointPairDirectionB(stateB, row);
          block.force.noalias() += forceMagnitude * direction;
          addAvbdRigidBlockHessianRankOneLowerTriangle(
              block, direction, row.state.stiffness);
          if (row.curvatureModel == AvbdRigidPointCurvatureModel::QuasiNewton) {
            const PointPairWorldPointCache& points
                = pointPairWorldPoints(indexedRow, cache);
            addAvbdRigidWorldPointQuasiNewtonGeometricDiagonal(
                block,
                stateB,
                points.worldPointB,
                -forceMagnitude * row.axis,
                row.curvatureModel);
          }
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
                row.state.stiffness);
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
                -axis,
                length,
                row.restLength,
                row.state.stiffness);
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
          const Vector6d direction = avbdRigidAngularPairDirectionA(
              angularPairOrientationError(indexedRow, cache), row);
          block.force.noalias() += forceMagnitude * direction;
          addAvbdRigidBlockHessianRankOneLowerTriangle(
              block, direction, row.state.stiffness);
        }
        if (indexedRow.bodyB == body && indexedRow.bodyB != indexedRow.bodyA) {
          const Vector6d direction = avbdRigidAngularPairDirectionB(
              angularPairOrientationError(indexedRow, cache), row);
          block.force.noalias() += forceMagnitude * direction;
          addAvbdRigidBlockHessianRankOneLowerTriangle(
              block, direction, row.state.stiffness);
        }
      };

  // Coulomb limit for one tangent pair. The pair's `bounds` carry the cone for
  // the current step: the descent prologue clears them for every row whose
  // normal row is resolvable, and this helper only ever raises them.
  //
  // Two properties matter. First, the bound comes from the normal row's
  // accepted dual, not from its penalty trial `k C + lambda`: on the first
  // sweep of a step that trial still carries the whole step-start penetration
  // and overstates the transmitted normal force by orders of magnitude.
  // Second, the cone may not shrink inside a step. Coulomb bounds the friction
  // impulse by mu times the normal impulse, whereas the instantaneous dual
  // decays to zero once the regularized normal constraint is met. A cone that
  // followed it down would silence a row that has already spent a tangential
  // impulse under a wider cone -- typically the warm-started dual applied at
  // `C = 0` -- and strand that displacement, because the augmented-Lagrangian
  // trial that would undo it is exactly what the closing cone clamps away.
  const auto liveFrictionForceLimit
      = [&](AvbdRigidBodyPointPairFrictionRows& indexedRows) {
          const auto stepLimit = [&]() {
            return avbdRigidPointPairFrictionPairForceLimit(
                indexedRows.first, indexedRows.second);
          };
          if (indexedRows.normalRowIndex >= pointPairRows.size()
              || !std::isfinite(indexedRows.frictionCoefficient)
              || indexedRows.frictionCoefficient <= 0.0) {
            return stepLimit();
          }

          const AvbdRigidBodyPointPairRow& indexedNormal
              = pointPairRows[indexedRows.normalRowIndex];
          if (!validBody(indexedNormal.bodyA) || !validBody(indexedNormal.bodyB)
              || indexedNormal.bodyA != indexedRows.bodyA
              || indexedNormal.bodyB != indexedRows.bodyB) {
            return stepLimit();
          }

          const AvbdRigidPointPairRow& normal = indexedNormal.row;
          double normalForce = 0.0;
          if (avbdRigidRowUsesFiniteMaterial(normal.materialStiffness)) {
            // A finite-material normal row carries no dual, so its penalty
            // force is the force it transmits.
            normalForce = avbdRigidScalarRowForce(
                normal.state,
                avbdRigidPointPairConstraintValue(
                    states[indexedNormal.bodyA],
                    states[indexedNormal.bodyB],
                    normal),
                normal.bounds,
                normal.materialStiffness);
          } else {
            normalForce = normal.state.lambda;
          }

          if (std::isnan(normalForce)) {
            // A NaN normal force marks a poisoned row whose own force is
            // already non-finite. The cone stays at its step high-water mark
            // because bounds never decrease within a sweep (the Hessian was
            // assembled against them); it is not reopened or widened here.
            return stepLimit();
          }
          const double liveLimit
              = indexedRows.frictionCoefficient * std::max(0.0, normalForce);
          if (liveLimit > stepLimit()) {
            indexedRows.first.bounds = avbdFrictionTangentBounds(liveLimit);
            indexedRows.second.bounds = indexedRows.first.bounds;
          }
          return stepLimit();
        };

  const auto addFrictionPairToBlock = [&](AvbdRigidBodyBlock& block,
                                          std::uint32_t body,
                                          AvbdRigidBodyPointPairFrictionRows&
                                              indexedRows) {
    const AvbdRigidBodyState& stateA = states[indexedRows.bodyA];
    const AvbdRigidBodyState& stateB = states[indexedRows.bodyB];
    const Eigen::Vector2d constraintValues = avbdRigidPointPairConstraintValues(
        stateA,
        stateB,
        indexedRows.first,
        indexedRows.second,
        frictionOptions.alpha);
    const double forceLimit = liveFrictionForceLimit(indexedRows);
    const Eigen::Vector2d force
        = avbdRigidPointPairFrictionTangentPairForceFromConstraintValuesAndLimit(
            constraintValues,
            indexedRows.first,
            indexedRows.second,
            forceLimit);
    if (indexedRows.bodyA == body) {
      const Vector6d firstDirection
          = avbdRigidPointPairDirectionA(stateA, indexedRows.first);
      const Vector6d secondDirection
          = avbdRigidPointPairDirectionA(stateA, indexedRows.second);
      block.force.noalias()
          += force.x() * firstDirection + force.y() * secondDirection;
      // Match the AVBD paper's simple friction approximation: project the
      // force to the Coulomb cone, but retain the unclamped penalty Hessian.
      addAvbdRigidBlockHessianRankOneLowerTriangle(
          block, firstDirection, indexedRows.first.state.stiffness);
      addAvbdRigidBlockHessianRankOneLowerTriangle(
          block, secondDirection, indexedRows.second.state.stiffness);
      if (indexedRows.first.curvatureModel
              == AvbdRigidPointCurvatureModel::QuasiNewton
          || indexedRows.second.curvatureModel
                 == AvbdRigidPointCurvatureModel::QuasiNewton) {
        const Eigen::Vector3d firstWorldPointA
            = avbdRigidBodyWorldPoint(stateA, indexedRows.first.localPointA);
        const Eigen::Vector3d secondWorldPointA
            = avbdRigidPointPairRowsShareLocalPoints(
                  indexedRows.first, indexedRows.second)
                  ? firstWorldPointA
                  : avbdRigidBodyWorldPoint(
                        stateA, indexedRows.second.localPointA);
        addAvbdRigidWorldPointQuasiNewtonGeometricDiagonal(
            block,
            stateA,
            firstWorldPointA,
            force.x() * indexedRows.first.axis,
            indexedRows.first.curvatureModel);
        addAvbdRigidWorldPointQuasiNewtonGeometricDiagonal(
            block,
            stateA,
            secondWorldPointA,
            force.y() * indexedRows.second.axis,
            indexedRows.second.curvatureModel);
      }
    }
    if (indexedRows.bodyB == body && indexedRows.bodyB != indexedRows.bodyA) {
      const Vector6d firstDirection
          = avbdRigidPointPairDirectionB(stateB, indexedRows.first);
      const Vector6d secondDirection
          = avbdRigidPointPairDirectionB(stateB, indexedRows.second);
      block.force.noalias()
          += force.x() * firstDirection + force.y() * secondDirection;
      addAvbdRigidBlockHessianRankOneLowerTriangle(
          block, firstDirection, indexedRows.first.state.stiffness);
      addAvbdRigidBlockHessianRankOneLowerTriangle(
          block, secondDirection, indexedRows.second.state.stiffness);
      if (indexedRows.first.curvatureModel
              == AvbdRigidPointCurvatureModel::QuasiNewton
          || indexedRows.second.curvatureModel
                 == AvbdRigidPointCurvatureModel::QuasiNewton) {
        const Eigen::Vector3d firstWorldPointB
            = avbdRigidBodyWorldPoint(stateB, indexedRows.first.localPointB);
        const Eigen::Vector3d secondWorldPointB
            = avbdRigidPointPairRowsShareLocalPoints(
                  indexedRows.first, indexedRows.second)
                  ? firstWorldPointB
                  : avbdRigidBodyWorldPoint(
                        stateB, indexedRows.second.localPointB);
        addAvbdRigidWorldPointQuasiNewtonGeometricDiagonal(
            block,
            stateB,
            firstWorldPointB,
            -force.x() * indexedRows.first.axis,
            indexedRows.first.curvatureModel);
        addAvbdRigidWorldPointQuasiNewtonGeometricDiagonal(
            block,
            stateB,
            secondWorldPointB,
            -force.y() * indexedRows.second.axis,
            indexedRows.second.curvatureModel);
      }
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
        AvbdRigidBodyPointPairFrictionRows& indexedRows
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

    forEachAvbdRowUpdateRange(
        rowUpdateExecutor,
        attachmentRows.size(),
        [&](std::size_t begin, std::size_t end) {
          for (std::size_t i = begin; i < end; ++i) {
            AvbdRigidBodyPointAttachmentRow& indexedRow = attachmentRows[i];
            if (validBody(indexedRow.body)) {
              indexedRow.row.state = updateAvbdRigidPointAttachmentRow(
                  indexedRow.row.state,
                  states[indexedRow.body],
                  indexedRow.row,
                  rowOptions);
            }
          }
        });
    forEachAvbdRowUpdateRange(
        rowUpdateExecutor,
        pointPairRows.size(),
        [&](std::size_t begin, std::size_t end) {
          for (std::size_t i = begin; i < end; ++i) {
            AvbdRigidBodyPointPairRow& indexedRow = pointPairRows[i];
            if (!validBody(indexedRow.bodyA) || !validBody(indexedRow.bodyB)) {
              continue;
            }
            const AvbdRigidPointAttachmentOptions& effectiveRowOptions
                = avbdRigidPointPairSolveOptions(indexedRow, rowOptions);
            if (avbdRigidRowUsesFiniteMaterial(
                    indexedRow.row.materialStiffness)) {
              indexedRow.row.state.lambda = 0.0;
              const double maxStiffness = std::min(
                  indexedRow.row.materialStiffness,
                  effectiveRowOptions.maxStiffness);
              if (effectiveRowOptions.beta >= 0.0
                  && indexedRow.row.state.stiffness >= maxStiffness) {
                indexedRow.row.state.stiffness = maxStiffness;
                continue;
              }
            }

            const double rawConstraintValue = avbdRigidPointPairConstraintValue(
                states[indexedRow.bodyA],
                states[indexedRow.bodyB],
                indexedRow.row);
            if (avbdRigidRowUsesFiniteMaterial(
                    indexedRow.row.materialStiffness)) {
              const double maxStiffness = std::min(
                  indexedRow.row.materialStiffness,
                  effectiveRowOptions.maxStiffness);
              indexedRow.row.state.stiffness = updateAvbdFiniteStiffness(
                  indexedRow.row.state.stiffness,
                  rawConstraintValue,
                  effectiveRowOptions.beta,
                  maxStiffness);
            } else {
              const double constraintValue = regularizeAvbdConstraintValue(
                  rawConstraintValue,
                  indexedRow.row.previousConstraintValue,
                  effectiveRowOptions.alpha);
              indexedRow.row.state = updateAvbdHardConstraintRow(
                  indexedRow.row.state,
                  constraintValue,
                  effectiveRowOptions.beta,
                  indexedRow.row.bounds,
                  effectiveRowOptions.maxStiffness);
            }
          }
        });
    forEachAvbdRowUpdateRange(
        rowUpdateExecutor,
        distanceSpringRows.size(),
        [&](std::size_t begin, std::size_t end) {
          for (std::size_t i = begin; i < end; ++i) {
            AvbdRigidBodyPointPairDistanceSpringRow& indexedRow
                = distanceSpringRows[i];
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
        });
    forEachAvbdRowUpdateRange(
        rowUpdateExecutor,
        angularPairRows.size(),
        [&](std::size_t begin, std::size_t end) {
          AngularPairConstraintCache angularPairUpdateCache;
          for (std::size_t i = begin; i < end; ++i) {
            AvbdRigidBodyAngularPairRow& indexedRow = angularPairRows[i];
            if (!validBody(indexedRow.bodyA) || !validBody(indexedRow.bodyB)) {
              continue;
            }
            if (avbdRigidRowUsesFiniteMaterial(
                    indexedRow.row.materialStiffness)) {
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
            if (avbdRigidRowUsesFiniteMaterial(
                    indexedRow.row.materialStiffness)) {
              indexedRow.row.state.stiffness = updateAvbdFiniteStiffness(
                  indexedRow.row.state.stiffness,
                  rawConstraintValue,
                  rowOptions.beta,
                  std::min(
                      indexedRow.row.materialStiffness,
                      rowOptions.maxStiffness));
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
        });
    forEachAvbdRowUpdateRange(
        rowUpdateExecutor,
        frictionPairRows.size(),
        [&](std::size_t begin, std::size_t end) {
          for (std::size_t i = begin; i < end; ++i) {
            AvbdRigidBodyPointPairFrictionRows& indexedRows
                = frictionPairRows[i];
            if (validBody(indexedRows.bodyA) && validBody(indexedRows.bodyB)) {
              const double forceLimit = liveFrictionForceLimit(indexedRows);
              const bool sticking
                  = updateAvbdRigidPointPairFrictionTangentPairForLimit(
                      indexedRows.first,
                      indexedRows.second,
                      states[indexedRows.bodyA],
                      states[indexedRows.bodyB],
                      frictionOptions,
                      forceLimit);
              if (indexedRows.persistentFirstRecord != nullptr) {
                indexedRows.persistentFirstRecord->state
                    = indexedRows.first.state;
                indexedRows.persistentFirstRecord->descriptor.bounds
                    = indexedRows.first.bounds;
              }
              if (indexedRows.persistentSecondRecord != nullptr) {
                indexedRows.persistentSecondRecord->state
                    = indexedRows.second.state;
                indexedRows.persistentSecondRecord->descriptor.bounds
                    = indexedRows.second.bounds;
              }
              if (indexedRows.persistentAnchor != nullptr) {
                indexedRows.persistentAnchor->sticking = sticking;
              }
            }
          }
        });

    if (convergenceSquared > 0.0 && maxStepSquared <= convergenceSquared) {
      break;
    }
  }

  return stats;
}

} // namespace dart::simulation::detail::deformable_vbd
