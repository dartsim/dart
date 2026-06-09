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

} // namespace dart::simulation::detail::newton_barrier
