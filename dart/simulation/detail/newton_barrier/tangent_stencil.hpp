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

#include <dart/simulation/detail/newton_barrier/primitive_distance.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <algorithm>
#include <array>
#include <limits>

#include <cmath>

namespace dart::simulation::detail::newton_barrier {

using Matrix3x2d = Eigen::Matrix<double, 3, 2>;
using Matrix2x6d = Eigen::Matrix<double, 2, 6>;
using Matrix2x9d = Eigen::Matrix<double, 2, 9>;
using Matrix2x12d = Eigen::Matrix<double, 2, 12>;

struct PointTriangleTangentStencil
{
  /// Columns span the tangent plane for ordered stencil [p, t0, t1, t2].
  Matrix3x2d basis = Matrix3x2d::Zero();
  /// Upstream-style unconstrained closest coordinates (beta1, beta2).
  Eigen::Vector2d coordinates = Eigen::Vector2d::Zero();
  /// Maps stacked vertex displacements [dp, dt0, dt1, dt2] to tangent motion.
  Matrix2x12d projection = Matrix2x12d::Zero();
  /// Equal to projection.transpose() * projection.
  Matrix12d metric = Matrix12d::Zero();
  /// True only when DART's deterministic finite fallback replaces the upstream
  /// nondegenerate tangent-basis formula.
  bool usedFallbackBasis = false;
};

struct EdgeEdgeTangentStencil
{
  /// Columns span the tangent plane for ordered stencil [ea0, ea1, eb0, eb1].
  Matrix3x2d basis = Matrix3x2d::Zero();
  /// Upstream-style unconstrained closest coordinates (gamma1, gamma2).
  Eigen::Vector2d coordinates = Eigen::Vector2d::Zero();
  /// Maps stacked vertex displacements [dea0, dea1, deb0, deb1] to tangent
  /// motion.
  Matrix2x12d projection = Matrix2x12d::Zero();
  /// Equal to projection.transpose() * projection.
  Matrix12d metric = Matrix12d::Zero();
  /// True only when DART's deterministic finite fallback replaces the upstream
  /// nondegenerate tangent-basis formula.
  bool usedFallbackBasis = false;
};

struct PointEdgeTangentStencil
{
  /// Columns span the tangent plane for ordered stencil [p, e0, e1].
  Matrix3x2d basis = Matrix3x2d::Zero();
  /// Upstream-style unconstrained point-edge closest coordinate eta.
  double coordinate = 0.0;
  /// Maps stacked vertex displacements [dp, de0, de1] to tangent motion.
  Matrix2x9d projection = Matrix2x9d::Zero();
  /// Equal to projection.transpose() * projection.
  Matrix9d metric = Matrix9d::Zero();
  /// True only when DART's deterministic finite fallback replaces the upstream
  /// nondegenerate tangent-basis formula.
  bool usedFallbackBasis = false;
};

struct PointPointTangentStencil
{
  /// Columns span the tangent plane for ordered stencil [p0, p1].
  Matrix3x2d basis = Matrix3x2d::Zero();
  /// Maps stacked vertex displacements [dp0, dp1] to tangent motion.
  Matrix2x6d projection = Matrix2x6d::Zero();
  /// Equal to projection.transpose() * projection.
  Matrix6d metric = Matrix6d::Zero();
  /// True only when DART's deterministic finite fallback replaces the upstream
  /// nondegenerate tangent-basis formula.
  bool usedFallbackBasis = false;
};

namespace detail {

constexpr double kTangentBasisEpsilon
    = 64.0 * std::numeric_limits<double>::epsilon();

//==============================================================================
inline bool isFinite(const Eigen::Vector3d& value)
{
  return std::isfinite(value.x()) && std::isfinite(value.y())
         && std::isfinite(value.z());
}

//==============================================================================
inline bool normalized(
    const Eigen::Vector3d& value, Eigen::Vector3d& normalizedValue)
{
  if (!isFinite(value)) {
    normalizedValue.setZero();
    return false;
  }

  const double squaredNorm = value.squaredNorm();
  if (!(squaredNorm > kTangentBasisEpsilon)) {
    normalizedValue.setZero();
    return false;
  }

  normalizedValue = value / std::sqrt(squaredNorm);
  return true;
}

//==============================================================================
inline Matrix3x2d fallbackBasisFromNormal(const Eigen::Vector3d& normal)
{
  Eigen::Vector3d unitNormal;
  if (!normalized(normal, unitNormal)) {
    unitNormal = Eigen::Vector3d::UnitZ();
  }

  Matrix3x2d basis;
  basis.col(0) = unitNormal.unitOrthogonal();
  basis.col(1) = unitNormal.cross(basis.col(0)).normalized();
  return basis;
}

//==============================================================================
inline Matrix3x2d basisFromFirstTangentAndSecondHint(
    const Eigen::Vector3d& firstTangent,
    const Eigen::Vector3d& secondHint,
    const Eigen::Vector3d& fallbackNormal,
    bool& usedFallbackBasis)
{
  usedFallbackBasis = false;

  Eigen::Vector3d tangent0;
  Eigen::Vector3d tangent1;
  if (normalized(firstTangent, tangent0)) {
    const Eigen::Vector3d orthogonalHint
        = secondHint - secondHint.dot(tangent0) * tangent0;
    if (normalized(orthogonalHint, tangent1)) {
      Matrix3x2d basis;
      basis.col(0) = tangent0;
      basis.col(1) = tangent1;
      return basis;
    }
  }

  usedFallbackBasis = true;
  return fallbackBasisFromNormal(fallbackNormal);
}

//==============================================================================
inline Eigen::Vector2d pointTriangleCoordinates(
    const Eigen::Vector3d& p,
    const Eigen::Vector3d& a,
    const Eigen::Vector3d& b,
    const Eigen::Vector3d& c)
{
  const Eigen::Vector3d ab = b - a;
  const Eigen::Vector3d ac = c - a;
  Eigen::Matrix2d matrix;
  matrix << ab.dot(ab), ab.dot(ac), ab.dot(ac), ac.dot(ac);

  const double determinant = matrix.determinant();
  if (std::isfinite(determinant)
      && std::abs(determinant) > kTangentBasisEpsilon) {
    Eigen::Vector2d rhs;
    rhs << (p - a).dot(ab), (p - a).dot(ac);
    return matrix.ldlt().solve(rhs);
  }

  const auto distance = pointTriangleSquaredDistance(p, a, b, c);
  return Eigen::Vector2d(distance.barycentric.y(), distance.barycentric.z());
}

//==============================================================================
inline Eigen::Vector2d edgeEdgeCoordinates(
    const Eigen::Vector3d& a,
    const Eigen::Vector3d& b,
    const Eigen::Vector3d& c,
    const Eigen::Vector3d& d)
{
  const Eigen::Vector3d e20 = a - c;
  const Eigen::Vector3d e01 = b - a;
  const Eigen::Vector3d e23 = d - c;

  Eigen::Matrix2d matrix;
  matrix << e01.dot(e01), -e23.dot(e01), -e23.dot(e01), e23.dot(e23);

  const double determinant = matrix.determinant();
  if (std::isfinite(determinant)
      && std::abs(determinant) > kTangentBasisEpsilon) {
    Eigen::Vector2d rhs;
    rhs << -e20.dot(e01), e20.dot(e23);
    return matrix.ldlt().solve(rhs);
  }

  const auto distance = edgeEdgeSquaredDistance(a, b, c, d);
  return Eigen::Vector2d(distance.edgeACoordinate, distance.edgeBCoordinate);
}

//==============================================================================
inline double pointEdgeCoordinate(
    const Eigen::Vector3d& p,
    const Eigen::Vector3d& a,
    const Eigen::Vector3d& b)
{
  const Eigen::Vector3d ab = b - a;
  const double squaredLength = ab.squaredNorm();
  if (std::isfinite(squaredLength) && squaredLength > kTangentBasisEpsilon) {
    return (p - a).dot(ab) / squaredLength;
  }

  return 0.0;
}

//==============================================================================
template <int Columns>
inline Eigen::Matrix<double, 2, Columns> projectionFromCoefficients(
    const Matrix3x2d& basis,
    const std::array<double, Columns / 3>& coefficients)
{
  Eigen::Matrix<double, 2, Columns> projection
      = Eigen::Matrix<double, 2, Columns>::Zero();
  for (int block = 0; block < static_cast<int>(coefficients.size()); ++block) {
    projection.template block<2, 3>(0, 3 * block)
        = coefficients[block] * basis.transpose();
  }
  return projection;
}

} // namespace detail

//==============================================================================
/// Internal tangent stencil matching the nondegenerate point-triangle formulas
/// used by upstream ipc-sim/IPC friction stencils.
///
/// Degenerate inputs return finite DART-owned fallback bases instead of
/// normalizing zero vectors. This helper does not compute friction energy,
/// gradients, Hessians, or solver caches.
inline PointTriangleTangentStencil pointTriangleTangentStencil(
    const Eigen::Vector3d& p,
    const Eigen::Vector3d& a,
    const Eigen::Vector3d& b,
    const Eigen::Vector3d& c)
{
  PointTriangleTangentStencil result;

  const Eigen::Vector3d ab = b - a;
  const Eigen::Vector3d ac = c - a;
  const Eigen::Vector3d triangleNormal = ab.cross(ac);
  result.basis = detail::basisFromFirstTangentAndSecondHint(
      ab, triangleNormal.cross(ab), p - a, result.usedFallbackBasis);
  result.coordinates = detail::pointTriangleCoordinates(p, a, b, c);

  const double beta1 = result.coordinates.x();
  const double beta2 = result.coordinates.y();
  result.projection = detail::projectionFromCoefficients<12>(
      result.basis, {1.0, -1.0 + beta1 + beta2, -beta1, -beta2});
  result.metric = result.projection.transpose() * result.projection;
  return result;
}

//==============================================================================
/// Internal tangent stencil matching the nondegenerate edge-edge formulas used
/// by upstream ipc-sim/IPC friction stencils.
///
/// Degenerate or parallel edges return finite DART-owned fallback bases instead
/// of normalizing zero vectors.
inline EdgeEdgeTangentStencil edgeEdgeTangentStencil(
    const Eigen::Vector3d& a,
    const Eigen::Vector3d& b,
    const Eigen::Vector3d& c,
    const Eigen::Vector3d& d)
{
  EdgeEdgeTangentStencil result;

  const Eigen::Vector3d ab = b - a;
  const Eigen::Vector3d cd = d - c;
  const Eigen::Vector3d ac = c - a;
  const Eigen::Vector3d normal = ab.cross(cd);

  Eigen::Vector3d abUnit;
  Eigen::Vector3d fallbackHint = ac;
  if (detail::normalized(ab, abUnit)) {
    fallbackHint = ac - ac.dot(abUnit) * abUnit;
  }

  result.basis = detail::basisFromFirstTangentAndSecondHint(
      ab, normal.cross(ab), fallbackHint, result.usedFallbackBasis);
  result.coordinates = detail::edgeEdgeCoordinates(a, b, c, d);

  const double gamma1 = result.coordinates.x();
  const double gamma2 = result.coordinates.y();
  result.projection = detail::projectionFromCoefficients<12>(
      result.basis, {1.0 - gamma1, gamma1, gamma2 - 1.0, -gamma2});
  result.metric = result.projection.transpose() * result.projection;
  return result;
}

//==============================================================================
/// Internal tangent stencil matching the nondegenerate point-edge formulas used
/// by upstream ipc-sim/IPC friction stencils.
///
/// Degenerate edges return finite DART-owned fallback bases instead of
/// normalizing zero vectors.
inline PointEdgeTangentStencil pointEdgeTangentStencil(
    const Eigen::Vector3d& p,
    const Eigen::Vector3d& a,
    const Eigen::Vector3d& b)
{
  PointEdgeTangentStencil result;

  const Eigen::Vector3d ab = b - a;
  result.basis = detail::basisFromFirstTangentAndSecondHint(
      ab, ab.cross(p - a), p - a, result.usedFallbackBasis);
  result.coordinate = detail::pointEdgeCoordinate(p, a, b);

  const double eta = result.coordinate;
  result.projection = detail::projectionFromCoefficients<9>(
      result.basis, {1.0, eta - 1.0, -eta});
  result.metric = result.projection.transpose() * result.projection;
  return result;
}

//==============================================================================
/// Internal tangent stencil matching the nondegenerate point-point formulas
/// used by upstream ipc-sim/IPC friction stencils.
///
/// Coincident points return finite DART-owned fallback bases instead of
/// normalizing zero vectors.
inline PointPointTangentStencil pointPointTangentStencil(
    const Eigen::Vector3d& a, const Eigen::Vector3d& b)
{
  PointPointTangentStencil result;

  const Eigen::Vector3d ab = b - a;
  Eigen::Vector3d xCross = Eigen::Vector3d::UnitX().cross(ab);
  Eigen::Vector3d yCross = Eigen::Vector3d::UnitY().cross(ab);
  const Eigen::Vector3d firstTangent
      = std::abs(ab.y()) > std::abs(ab.x()) ? xCross : yCross;
  result.basis = detail::basisFromFirstTangentAndSecondHint(
      firstTangent, ab.cross(firstTangent), ab, result.usedFallbackBasis);

  result.projection
      = detail::projectionFromCoefficients<6>(result.basis, {1.0, -1.0});
  result.metric = result.projection.transpose() * result.projection;
  return result;
}

//==============================================================================
template <typename Projection, typename Displacement>
inline Eigen::Vector2d projectTangentialDisplacement(
    const Projection& projection, const Displacement& displacement)
{
  return projection * displacement;
}

//==============================================================================
template <typename Projection>
inline auto liftTangentialDisplacement(
    const Projection& projection, const Eigen::Vector2d& displacement)
{
  return projection.transpose() * displacement;
}

} // namespace dart::simulation::detail::newton_barrier
