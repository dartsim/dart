/*
 * Copyright (c) 2011, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/main/LICENSE
 *
 * This file is provided under the following "BSD-style" License:
 *   Redistribution and use in source and binary form, with or
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

#include <dart/simulation/experimental/detail/deformable_contact/primitive_distance.hpp>

#include <Eigen/Core>

#include <algorithm>
#include <limits>

#include <cmath>

namespace dart::simulation::experimental::detail::deformable_contact {

struct BarrierScalarDerivatives
{
  double value = 0.0;
  double firstDerivative = 0.0;
  double secondDerivative = 0.0;
  double squaredDistance = 0.0;
  double safeSquaredDistance = 0.0;
  double squaredActivationDistance = 0.0;
  bool active = false;
};

struct PrimitiveBarrierResult
{
  double value = 0.0;
  Vector12d gradient = Vector12d::Zero();
  Matrix12d hessian = Matrix12d::Zero();
  double squaredDistance = 0.0;
  double safeSquaredDistance = 0.0;
  double squaredActivationDistance = 0.0;
  double mollifier = 1.0;
  double mollifierThreshold = 0.0;
  bool active = false;
};

namespace detail {

constexpr double kBarrierDistanceFloorScale = 1e-16;

//==============================================================================
inline double nonnegativeFiniteStiffness(const double stiffness)
{
  if (!std::isfinite(stiffness)) {
    return 0.0;
  }
  return std::max(0.0, stiffness);
}

//==============================================================================
inline double safeSquaredBarrierDistance(
    const double squaredDistance, const double squaredActivationDistance)
{
  const double activeInteriorLimit
      = std::nextafter(squaredActivationDistance, 0.0);
  if (!(activeInteriorLimit > 0.0)) {
    return std::max(squaredDistance, squaredActivationDistance);
  }

  const double floor = std::min(
      std::max(
          std::numeric_limits<double>::denorm_min(),
          kBarrierDistanceFloorScale * squaredActivationDistance),
      activeInteriorLimit);
  return std::max(squaredDistance, floor);
}

//==============================================================================
inline PrimitiveBarrierResult makeInactivePrimitiveBarrier(
    const double squaredDistance,
    const double safeSquaredDistance,
    const double squaredActivationDistance)
{
  PrimitiveBarrierResult result;
  result.squaredDistance = squaredDistance;
  result.safeSquaredDistance = safeSquaredDistance;
  result.squaredActivationDistance = squaredActivationDistance;
  return result;
}

} // namespace detail

//==============================================================================
inline BarrierScalarDerivatives c2ClampedLogBarrier(
    const double squaredDistance, const double squaredActivationDistance)
{
  BarrierScalarDerivatives result;
  result.squaredDistance = squaredDistance;
  result.squaredActivationDistance = squaredActivationDistance;

  if (!std::isfinite(squaredActivationDistance)
      || squaredActivationDistance <= 0.0 || !std::isfinite(squaredDistance)) {
    result.safeSquaredDistance = squaredDistance;
    return result;
  }

  if (squaredDistance >= squaredActivationDistance) {
    result.safeSquaredDistance = squaredDistance;
    return result;
  }

  const double d = detail::safeSquaredBarrierDistance(
      squaredDistance, squaredActivationDistance);
  const double dHat = squaredActivationDistance;
  const double offset = d - dHat;
  const double logRatio = std::log(d / dHat);
  const double offsetSquared = offset * offset;

  result.value = -offsetSquared * logRatio;
  result.firstDerivative = -2.0 * offset * logRatio - offsetSquared / d;
  result.secondDerivative
      = -2.0 * logRatio - 4.0 * offset / d + offsetSquared / (d * d);
  result.safeSquaredDistance = d;
  result.active = true;
  return result;
}

//==============================================================================
inline PrimitiveBarrierResult pointPointBarrier(
    const Eigen::Vector3d& a,
    const Eigen::Vector3d& b,
    const double squaredActivationDistance,
    const double stiffness = 1.0)
{
  const double squaredDistance = pointPointSquaredDistance(a, b);
  const auto barrier
      = c2ClampedLogBarrier(squaredDistance, squaredActivationDistance);

  PrimitiveBarrierResult result = detail::makeInactivePrimitiveBarrier(
      squaredDistance, barrier.safeSquaredDistance, squaredActivationDistance);
  result.active = barrier.active;

  const double kappa = detail::nonnegativeFiniteStiffness(stiffness);
  if (!barrier.active || kappa <= 0.0) {
    return result;
  }

  Vector12d distanceGradient = Vector12d::Zero();
  distanceGradient.head<6>() = pointPointSquaredDistanceGradient(a, b);
  Matrix12d distanceHessian = Matrix12d::Zero();
  distanceHessian.topLeftCorner<6, 6>() = pointPointSquaredDistanceHessian();

  result.value = kappa * barrier.value;
  result.gradient = kappa * barrier.firstDerivative * distanceGradient;
  result.hessian = kappa
                   * (barrier.secondDerivative
                          * (distanceGradient * distanceGradient.transpose())
                      + barrier.firstDerivative * distanceHessian);
  return result;
}

//==============================================================================
inline PrimitiveBarrierResult pointEdgeBarrier(
    const Eigen::Vector3d& p,
    const Eigen::Vector3d& a,
    const Eigen::Vector3d& b,
    const double squaredActivationDistance,
    const double stiffness = 1.0)
{
  const auto distance = pointEdgeSquaredDistance(p, a, b);
  const auto barrier = c2ClampedLogBarrier(
      distance.squaredDistance, squaredActivationDistance);

  PrimitiveBarrierResult result = detail::makeInactivePrimitiveBarrier(
      distance.squaredDistance,
      barrier.safeSquaredDistance,
      squaredActivationDistance);
  result.active = barrier.active;

  const double kappa = detail::nonnegativeFiniteStiffness(stiffness);
  if (!barrier.active || kappa <= 0.0) {
    return result;
  }

  Vector12d distanceGradient = Vector12d::Zero();
  distanceGradient.head<9>() = pointEdgeSquaredDistanceGradient(p, a, b);
  Matrix12d distanceHessian = Matrix12d::Zero();
  distanceHessian.topLeftCorner<9, 9>()
      = pointEdgeSquaredDistanceHessian(p, a, b);

  result.value = kappa * barrier.value;
  result.gradient = kappa * barrier.firstDerivative * distanceGradient;
  result.hessian = kappa
                   * (barrier.secondDerivative
                          * (distanceGradient * distanceGradient.transpose())
                      + barrier.firstDerivative * distanceHessian);
  return result;
}

//==============================================================================
inline PrimitiveBarrierResult pointTriangleBarrier(
    const Eigen::Vector3d& p,
    const Eigen::Vector3d& a,
    const Eigen::Vector3d& b,
    const Eigen::Vector3d& c,
    const double squaredActivationDistance,
    const double stiffness = 1.0)
{
  const auto distance = pointTriangleSquaredDistance(p, a, b, c);
  const auto barrier = c2ClampedLogBarrier(
      distance.squaredDistance, squaredActivationDistance);

  PrimitiveBarrierResult result = detail::makeInactivePrimitiveBarrier(
      distance.squaredDistance,
      barrier.safeSquaredDistance,
      squaredActivationDistance);
  result.active = barrier.active;

  const double kappa = detail::nonnegativeFiniteStiffness(stiffness);
  if (!barrier.active || kappa <= 0.0) {
    return result;
  }

  const Vector12d distanceGradient
      = pointTriangleSquaredDistanceGradient(p, a, b, c);
  const Matrix12d distanceHessian
      = pointTriangleSquaredDistanceHessian(p, a, b, c);

  result.value = kappa * barrier.value;
  result.gradient = kappa * barrier.firstDerivative * distanceGradient;
  result.hessian = kappa
                   * (barrier.secondDerivative
                          * (distanceGradient * distanceGradient.transpose())
                      + barrier.firstDerivative * distanceHessian);
  return result;
}

//==============================================================================
inline PrimitiveBarrierResult edgeEdgeBarrier(
    const Eigen::Vector3d& a,
    const Eigen::Vector3d& b,
    const Eigen::Vector3d& c,
    const Eigen::Vector3d& d,
    const double squaredActivationDistance,
    const double stiffness = 1.0)
{
  const auto distance = edgeEdgeSquaredDistance(a, b, c, d);
  const auto barrier = c2ClampedLogBarrier(
      distance.squaredDistance, squaredActivationDistance);

  PrimitiveBarrierResult result = detail::makeInactivePrimitiveBarrier(
      distance.squaredDistance,
      barrier.safeSquaredDistance,
      squaredActivationDistance);
  result.active = barrier.active;

  const double kappa = detail::nonnegativeFiniteStiffness(stiffness);
  if (!barrier.active || kappa <= 0.0) {
    return result;
  }

  const Vector12d distanceGradient
      = edgeEdgeSquaredDistanceGradient(a, b, c, d);
  const Matrix12d distanceHessian = edgeEdgeSquaredDistanceHessian(a, b, c, d);

  result.value = kappa * barrier.value;
  result.gradient = kappa * barrier.firstDerivative * distanceGradient;
  result.hessian = kappa
                   * (barrier.secondDerivative
                          * (distanceGradient * distanceGradient.transpose())
                      + barrier.firstDerivative * distanceHessian);
  return result;
}

//==============================================================================
inline PrimitiveBarrierResult mollifiedEdgeEdgeBarrier(
    const Eigen::Vector3d& a,
    const Eigen::Vector3d& b,
    const Eigen::Vector3d& c,
    const Eigen::Vector3d& d,
    const double squaredActivationDistance,
    const double mollifierThreshold,
    const double stiffness = 1.0)
{
  const auto distance = edgeEdgeSquaredDistance(a, b, c, d);
  const auto barrier = c2ClampedLogBarrier(
      distance.squaredDistance, squaredActivationDistance);

  PrimitiveBarrierResult result = detail::makeInactivePrimitiveBarrier(
      distance.squaredDistance,
      barrier.safeSquaredDistance,
      squaredActivationDistance);
  result.mollifierThreshold = mollifierThreshold;
  result.mollifier = edgeEdgeMollifier(a, b, c, d, mollifierThreshold);
  result.active = barrier.active;

  const double kappa = detail::nonnegativeFiniteStiffness(stiffness);
  if (!barrier.active || kappa <= 0.0) {
    return result;
  }

  const Vector12d distanceGradient
      = edgeEdgeSquaredDistanceGradient(a, b, c, d);
  const Matrix12d distanceHessian = edgeEdgeSquaredDistanceHessian(a, b, c, d);
  const Vector12d mollifierGradient
      = edgeEdgeMollifierGradient(a, b, c, d, mollifierThreshold);
  const Matrix12d mollifierHessian
      = edgeEdgeMollifierHessian(a, b, c, d, mollifierThreshold);

  result.value = kappa * result.mollifier * barrier.value;
  result.gradient
      = kappa
        * (result.mollifier * barrier.firstDerivative * distanceGradient
           + barrier.value * mollifierGradient);
  result.hessian
      = kappa
        * (result.mollifier * barrier.secondDerivative
               * (distanceGradient * distanceGradient.transpose())
           + result.mollifier * barrier.firstDerivative * distanceHessian
           + barrier.firstDerivative
                 * (distanceGradient * mollifierGradient.transpose()
                    + mollifierGradient * distanceGradient.transpose())
           + barrier.value * mollifierHessian);
  return result;
}

} // namespace dart::simulation::experimental::detail::deformable_contact
