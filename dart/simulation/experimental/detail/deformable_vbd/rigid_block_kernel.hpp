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

#include <dart/simulation/experimental/detail/deformable_vbd/avbd_constraint.hpp>
#include <dart/simulation/experimental/detail/deformable_vbd/contact_kernel.hpp>

#include <Eigen/Cholesky>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <limits>

#include <cmath>

namespace dart::simulation::experimental::detail::deformable_vbd {

using Vector6d = Eigen::Matrix<double, 6, 1>;
using Matrix6d = Eigen::Matrix<double, 6, 6>;

inline constexpr double kAvbdRigidPi = 3.141592653589793238462643383279502884;

//==============================================================================
inline Eigen::Quaterniond normalizeAvbdRigidOrientation(
    const Eigen::Quaterniond& orientation)
{
  const double norm = orientation.norm();
  if (std::isfinite(norm) && norm > 0.0) {
    Eigen::Quaterniond normalized = orientation;
    normalized.coeffs() /= norm;
    return normalized;
  }
  return Eigen::Quaterniond::Identity();
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
inline Eigen::Vector3d avbdRigidBodyWorldPoint(
    const AvbdRigidBodyState& state, const Eigen::Vector3d& localPoint)
{
  return state.position
         + normalizeAvbdRigidOrientation(state.orientation) * localPoint;
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
inline Eigen::Vector3d avbdRigidPointPairRelativePosition(
    const AvbdRigidBodyState& stateA,
    const AvbdRigidBodyState& stateB,
    const AvbdRigidPointPairRow& row)
{
  return avbdRigidBodyWorldPoint(stateB, row.localPointB)
         - avbdRigidBodyWorldPoint(stateA, row.localPointA);
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
inline double avbdRigidPointAttachmentConstraintValue(
    const AvbdRigidBodyState& state, const AvbdRigidPointAttachmentRow& row)
{
  return row.axis.dot(
      row.target - avbdRigidBodyWorldPoint(state, row.localPoint));
}

//==============================================================================
inline Vector6d avbdRigidPointAttachmentDirection(
    const AvbdRigidBodyState& state, const AvbdRigidPointAttachmentRow& row)
{
  const Eigen::Vector3d arm
      = avbdRigidBodyWorldPoint(state, row.localPoint) - state.position;

  Vector6d direction = Vector6d::Zero();
  direction.head<3>() = row.axis;
  direction.tail<3>() = arm.cross(row.axis);
  return direction;
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
inline Vector6d avbdRigidPointPairDirectionA(
    const AvbdRigidBodyState& stateA, const AvbdRigidPointPairRow& row)
{
  const Eigen::Vector3d arm
      = avbdRigidBodyWorldPoint(stateA, row.localPointA) - stateA.position;

  Vector6d direction = Vector6d::Zero();
  direction.head<3>() = row.axis;
  direction.tail<3>() = arm.cross(row.axis);
  return direction;
}

//==============================================================================
inline Vector6d avbdRigidPointPairDirectionB(
    const AvbdRigidBodyState& stateB, const AvbdRigidPointPairRow& row)
{
  const Eigen::Vector3d arm
      = avbdRigidBodyWorldPoint(stateB, row.localPointB) - stateB.position;

  Vector6d direction = Vector6d::Zero();
  direction.head<3>() = -row.axis;
  direction.tail<3>() = arm.cross(-row.axis);
  return direction;
}

//==============================================================================
inline double addAvbdRigidPointAttachment(
    AvbdRigidBodyBlock& block,
    const AvbdRigidBodyState& state,
    const AvbdRigidPointAttachmentRow& row,
    double alpha)
{
  const double constraintValue = regularizeAvbdConstraintValue(
      avbdRigidPointAttachmentConstraintValue(state, row),
      row.previousConstraintValue,
      alpha);
  const double forceMagnitude
      = computeAvbdHardConstraintForce(row.state, constraintValue, row.bounds);
  const Vector6d direction = avbdRigidPointAttachmentDirection(state, row);
  block.force.noalias() += forceMagnitude * direction;
  block.hessian.noalias()
      += row.state.stiffness * (direction * direction.transpose());
  return forceMagnitude;
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
  const double constraintValue = regularizeAvbdConstraintValue(
      avbdRigidPointPairConstraintValue(stateA, stateB, row),
      row.previousConstraintValue,
      alpha);
  const double forceMagnitude
      = computeAvbdHardConstraintForce(row.state, constraintValue, row.bounds);
  const Vector6d firstDirection = avbdRigidPointPairDirectionA(stateA, row);
  const Vector6d secondDirection = avbdRigidPointPairDirectionB(stateB, row);

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
  const double constraintValue = regularizeAvbdConstraintValue(
      avbdRigidPointPairConstraintValue(stateA, stateB, row),
      row.previousConstraintValue,
      options.alpha);
  return updateAvbdHardConstraintRow(
      state, constraintValue, options.beta, row.bounds, options.maxStiffness);
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

} // namespace dart::simulation::experimental::detail::deformable_vbd
