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
 *     copyright notice and this list of conditions in the documentation
 *     and/or other materials provided with the distribution.
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

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <algorithm>
#include <limits>

#include <cmath>

namespace dart::simulation::detail::newton_barrier {

namespace detail {

//==============================================================================
[[nodiscard]] inline Eigen::Vector3d normalizedOr(
    const Eigen::Vector3d& vector, const Eigen::Vector3d& fallback)
{
  const double norm = vector.norm();
  if (std::isfinite(norm) && norm > 0.0) {
    return vector / norm;
  }
  return fallback.normalized();
}

//==============================================================================
[[nodiscard]] inline Eigen::Vector3d perpendicularUnit(
    const Eigen::Vector3d& axis)
{
  const Eigen::Vector3d candidate = std::abs(axis.x()) < 0.9
                                        ? Eigen::Vector3d::UnitX()
                                        : Eigen::Vector3d::UnitY();
  return (candidate - axis * axis.dot(candidate)).normalized();
}

//==============================================================================
[[nodiscard]] inline Eigen::Vector3d projectedUnitOr(
    const Eigen::Vector3d& vector, const Eigen::Vector3d& normal)
{
  const Eigen::Vector3d projected = vector - normal * normal.dot(vector);
  const double norm = projected.norm();
  if (std::isfinite(norm) && norm > 0.0) {
    return projected / norm;
  }
  return perpendicularUnit(normal);
}

} // namespace detail

struct PointConnectionConstraintResult
{
  Eigen::Vector3d residual = Eigen::Vector3d::Zero();
  Eigen::Matrix<double, 3, 6> jacobian = Eigen::Matrix<double, 3, 6>::Zero();
  double squaredNorm = 0.0;
};

struct HingeAxisConstraintResult
{
  Eigen::Vector3d residual = Eigen::Vector3d::Zero();
  double squaredNorm = 0.0;
};

struct SlidingConstraintResult
{
  Eigen::Vector3d residual = Eigen::Vector3d::Zero();
  Eigen::Matrix3d jacobian = Eigen::Matrix3d::Zero();
  double coordinate = 0.0;
  double squaredNorm = 0.0;
};

struct RelativeSlidingConstraintResult
{
  Eigen::Vector3d residual = Eigen::Vector3d::Zero();
  Eigen::Matrix<double, 3, 6> jacobian = Eigen::Matrix<double, 3, 6>::Zero();
  double coordinate = 0.0;
  double squaredNorm = 0.0;
};

struct DistanceConstraintResult
{
  double residual = 0.0;
  double distance = 0.0;
  Eigen::Matrix<double, 1, 6> jacobian = Eigen::Matrix<double, 1, 6>::Zero();
};

struct ScalarRangeMargins
{
  double lower = 0.0;
  double upper = 0.0;
  bool feasible = false;
};

struct ScalarRangeBarrierResult
{
  double value = 0.0;
  double firstDerivative = 0.0;
  double secondDerivative = 0.0;
  ScalarRangeMargins margins;
  bool activeLower = false;
  bool activeUpper = false;
  bool active = false;
};

struct BoundedDistanceBarrierResult
{
  double value = 0.0;
  Eigen::Matrix<double, 6, 1> gradient = Eigen::Matrix<double, 6, 1>::Zero();
  Eigen::Matrix<double, 6, 6> hessian = Eigen::Matrix<double, 6, 6>::Zero();
  double distance = 0.0;
  ScalarRangeMargins margins;
  bool active = false;
};

struct SlidingRangeBarrierResult
{
  double value = 0.0;
  Eigen::Vector3d gradient = Eigen::Vector3d::Zero();
  Eigen::Matrix3d hessian = Eigen::Matrix3d::Zero();
  double coordinate = 0.0;
  ScalarRangeMargins margins;
  bool active = false;
};

struct RotationRangeBarrierResult
{
  double value = 0.0;
  double firstDerivative = 0.0;
  double secondDerivative = 0.0;
  double angle = 0.0;
  ScalarRangeMargins margins;
  bool active = false;
};

struct ConeTwistCoordinates
{
  double bendAngle = 0.0;
  double twistAngle = 0.0;
};

struct FixedPointConstraintResult
{
  Eigen::Vector3d residual = Eigen::Vector3d::Zero();
  Eigen::Matrix3d jacobian = Eigen::Matrix3d::Zero();
  double squaredNorm = 0.0;
};

//==============================================================================
[[nodiscard]] inline PointConnectionConstraintResult pointConnectionConstraint(
    const Eigen::Vector3d& pointA, const Eigen::Vector3d& pointB)
{
  PointConnectionConstraintResult result;
  result.residual = pointA - pointB;
  result.jacobian.leftCols<3>().setIdentity();
  result.jacobian.rightCols<3>() = -Eigen::Matrix3d::Identity();
  result.squaredNorm = result.residual.squaredNorm();
  return result;
}

//==============================================================================
[[nodiscard]] inline FixedPointConstraintResult fixedPointConstraint(
    const Eigen::Vector3d& point, const Eigen::Vector3d& target)
{
  FixedPointConstraintResult result;
  result.residual = point - target;
  result.jacobian.setIdentity();
  result.squaredNorm = result.residual.squaredNorm();
  return result;
}

//==============================================================================
[[nodiscard]] inline DistanceConstraintResult distanceConstraint(
    const Eigen::Vector3d& pointA,
    const Eigen::Vector3d& pointB,
    const double targetDistance)
{
  const Eigen::Vector3d offset = pointA - pointB;
  const double distance = offset.norm();

  DistanceConstraintResult result;
  result.distance = distance;
  result.residual = distance - targetDistance;
  if (std::isfinite(distance) && distance > 0.0) {
    const Eigen::Vector3d direction = offset / distance;
    result.jacobian.leftCols<3>() = direction.transpose();
    result.jacobian.rightCols<3>() = -direction.transpose();
  }
  return result;
}

//==============================================================================
[[nodiscard]] inline ScalarRangeMargins scalarRangeMargins(
    const double coordinate, const double lower, const double upper)
{
  ScalarRangeMargins margins;
  margins.lower = coordinate - lower;
  margins.upper = upper - coordinate;
  margins.feasible = std::isfinite(margins.lower)
                     && std::isfinite(margins.upper) && margins.lower > 0.0
                     && margins.upper > 0.0;
  return margins;
}

//==============================================================================
[[nodiscard]] inline bool rangeBarrierFeasible(
    const ScalarRangeMargins& margins)
{
  return margins.feasible;
}

namespace detail {

struct MarginBarrierDerivatives
{
  double value = 0.0;
  double firstDerivative = 0.0;
  double secondDerivative = 0.0;
  bool active = false;
};

constexpr double kRangeBarrierMarginFloorScale = 1e-16;

//==============================================================================
[[nodiscard]] inline double safeRangeBarrierMargin(
    const double margin, const double activationDistance)
{
  const double activeInteriorLimit = std::nextafter(activationDistance, 0.0);
  if (!(activeInteriorLimit > 0.0)) {
    return std::max(margin, activationDistance);
  }

  const double floor = std::min(
      std::max(
          std::numeric_limits<double>::denorm_min(),
          kRangeBarrierMarginFloorScale * activationDistance),
      activeInteriorLimit);
  return std::max(margin, floor);
}

//==============================================================================
[[nodiscard]] inline MarginBarrierDerivatives marginBarrier(
    const double margin,
    const double activationDistance,
    const double stiffness)
{
  MarginBarrierDerivatives result;
  if (!std::isfinite(margin) || !std::isfinite(activationDistance)
      || activationDistance <= 0.0 || !std::isfinite(stiffness)
      || stiffness <= 0.0 || margin >= activationDistance) {
    return result;
  }

  const double safeMargin = safeRangeBarrierMargin(margin, activationDistance);
  if (!(safeMargin > 0.0) || safeMargin >= activationDistance) {
    result.active = true;
    return result;
  }

  const double offset = safeMargin - activationDistance;
  const double logRatio = std::log(safeMargin / activationDistance);
  const double offsetSquared = offset * offset;

  result.value = stiffness * (-offsetSquared * logRatio);
  result.firstDerivative
      = stiffness * (-2.0 * offset * logRatio - offsetSquared / safeMargin);
  result.secondDerivative = stiffness
                            * (-2.0 * logRatio - 4.0 * offset / safeMargin
                               + offsetSquared / (safeMargin * safeMargin));
  result.active = true;
  return result;
}

} // namespace detail

//==============================================================================
[[nodiscard]] inline ScalarRangeBarrierResult scalarRangeBarrier(
    const double coordinate,
    const double lower,
    const double upper,
    const double activationDistance,
    const double stiffness = 1.0)
{
  const ScalarRangeMargins margins
      = scalarRangeMargins(coordinate, lower, upper);
  const auto lowerBarrier
      = detail::marginBarrier(margins.lower, activationDistance, stiffness);
  const auto upperBarrier
      = detail::marginBarrier(margins.upper, activationDistance, stiffness);

  ScalarRangeBarrierResult result;
  result.value = lowerBarrier.value + upperBarrier.value;
  result.firstDerivative
      = lowerBarrier.firstDerivative - upperBarrier.firstDerivative;
  result.secondDerivative
      = lowerBarrier.secondDerivative + upperBarrier.secondDerivative;
  result.margins = margins;
  result.activeLower = lowerBarrier.active;
  result.activeUpper = upperBarrier.active;
  result.active = result.activeLower || result.activeUpper;
  return result;
}

//==============================================================================
[[nodiscard]] inline BoundedDistanceBarrierResult boundedDistanceBarrier(
    const Eigen::Vector3d& pointA,
    const Eigen::Vector3d& pointB,
    const double lowerDistance,
    const double upperDistance,
    const double activationDistance,
    const double stiffness = 1.0)
{
  const auto distance
      = distanceConstraint(pointA, pointB, /*targetDistance=*/0.0);
  const auto barrier = scalarRangeBarrier(
      distance.distance,
      lowerDistance,
      upperDistance,
      activationDistance,
      stiffness);

  BoundedDistanceBarrierResult result;
  result.value = barrier.value;
  result.distance = distance.distance;
  result.margins = barrier.margins;
  result.active = barrier.active;
  result.gradient = barrier.firstDerivative * distance.jacobian.transpose();
  result.hessian = std::max(0.0, barrier.secondDerivative)
                   * (distance.jacobian.transpose() * distance.jacobian);
  return result;
}

//==============================================================================
[[nodiscard]] inline SlidingConstraintResult slidingConstraint(
    const Eigen::Vector3d& point,
    const Eigen::Vector3d& lineOrigin,
    const Eigen::Vector3d& lineAxis)
{
  const Eigen::Vector3d axis
      = detail::normalizedOr(lineAxis, Eigen::Vector3d::UnitX());
  const Eigen::Vector3d offset = point - lineOrigin;
  const Eigen::Matrix3d projector
      = Eigen::Matrix3d::Identity() - axis * axis.transpose();

  SlidingConstraintResult result;
  result.residual = projector * offset;
  result.jacobian = projector;
  result.coordinate = axis.dot(offset);
  result.squaredNorm = result.residual.squaredNorm();
  return result;
}

//==============================================================================
[[nodiscard]] inline RelativeSlidingConstraintResult relativeSlidingConstraint(
    const Eigen::Vector3d& pointA,
    const Eigen::Vector3d& pointB,
    const Eigen::Vector3d& slideAxis)
{
  const Eigen::Vector3d axis
      = detail::normalizedOr(slideAxis, Eigen::Vector3d::UnitX());
  const Eigen::Vector3d offset = pointA - pointB;
  const Eigen::Matrix3d projector
      = Eigen::Matrix3d::Identity() - axis * axis.transpose();

  RelativeSlidingConstraintResult result;
  result.residual = projector * offset;
  result.jacobian.leftCols<3>() = projector;
  result.jacobian.rightCols<3>() = -projector;
  result.coordinate = axis.dot(offset);
  result.squaredNorm = result.residual.squaredNorm();
  return result;
}

//==============================================================================
[[nodiscard]] inline SlidingRangeBarrierResult slidingRangeBarrier(
    const Eigen::Vector3d& point,
    const Eigen::Vector3d& lineOrigin,
    const Eigen::Vector3d& lineAxis,
    const double lower,
    const double upper,
    const double activationDistance,
    const double stiffness = 1.0)
{
  const Eigen::Vector3d axis
      = detail::normalizedOr(lineAxis, Eigen::Vector3d::UnitX());
  const double coordinate = axis.dot(point - lineOrigin);
  const auto barrier = scalarRangeBarrier(
      coordinate, lower, upper, activationDistance, stiffness);

  SlidingRangeBarrierResult result;
  result.value = barrier.value;
  result.gradient = barrier.firstDerivative * axis;
  result.hessian
      = std::max(0.0, barrier.secondDerivative) * (axis * axis.transpose());
  result.coordinate = coordinate;
  result.margins = barrier.margins;
  result.active = barrier.active;
  return result;
}

//==============================================================================
[[nodiscard]] inline HingeAxisConstraintResult hingeAxisConstraint(
    const Eigen::Vector3d& axisA, const Eigen::Vector3d& axisB)
{
  const Eigen::Vector3d unitA
      = detail::normalizedOr(axisA, Eigen::Vector3d::UnitZ());
  const Eigen::Vector3d unitB = detail::normalizedOr(axisB, unitA);

  HingeAxisConstraintResult result;
  result.residual = unitA.cross(unitB);
  result.squaredNorm = result.residual.squaredNorm();
  return result;
}

//==============================================================================
[[nodiscard]] inline ConeTwistCoordinates coneTwistCoordinates(
    const Eigen::Vector3d& axisA,
    const Eigen::Vector3d& referenceA,
    const Eigen::Vector3d& axisB,
    const Eigen::Vector3d& referenceB)
{
  const Eigen::Vector3d unitA
      = detail::normalizedOr(axisA, Eigen::Vector3d::UnitZ());
  const Eigen::Vector3d unitB = detail::normalizedOr(axisB, unitA);
  const Eigen::Vector3d refA = detail::projectedUnitOr(referenceA, unitA);
  const Eigen::Vector3d refB = detail::projectedUnitOr(referenceB, unitB);

  ConeTwistCoordinates coordinates;
  coordinates.bendAngle = std::atan2(
      unitA.cross(unitB).norm(), std::clamp(unitA.dot(unitB), -1.0, 1.0));

  const Eigen::Quaterniond alignBToA
      = Eigen::Quaterniond::FromTwoVectors(unitB, unitA);
  const Eigen::Vector3d alignedRefB
      = detail::projectedUnitOr(alignBToA * refB, unitA);
  coordinates.twistAngle = std::atan2(
      unitA.dot(refA.cross(alignedRefB)),
      std::clamp(refA.dot(alignedRefB), -1.0, 1.0));
  return coordinates;
}

//==============================================================================
[[nodiscard]] inline RotationRangeBarrierResult rotationRangeBarrier(
    const double angle,
    const double lower,
    const double upper,
    const double activationDistance,
    const double stiffness = 1.0)
{
  const auto barrier
      = scalarRangeBarrier(angle, lower, upper, activationDistance, stiffness);

  RotationRangeBarrierResult result;
  result.value = barrier.value;
  result.firstDerivative = barrier.firstDerivative;
  result.secondDerivative = std::max(0.0, barrier.secondDerivative);
  result.angle = angle;
  result.margins = barrier.margins;
  result.active = barrier.active;
  return result;
}

//==============================================================================
[[nodiscard]] inline RotationRangeBarrierResult coneTwistRotationRangeBarrier(
    const Eigen::Vector3d& axisA,
    const Eigen::Vector3d& referenceA,
    const Eigen::Vector3d& axisB,
    const Eigen::Vector3d& referenceB,
    const double lower,
    const double upper,
    const double activationDistance,
    const double stiffness = 1.0)
{
  const auto coordinates
      = coneTwistCoordinates(axisA, referenceA, axisB, referenceB);
  return rotationRangeBarrier(
      coordinates.twistAngle, lower, upper, activationDistance, stiffness);
}

} // namespace dart::simulation::detail::newton_barrier
