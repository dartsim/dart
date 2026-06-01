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
#include <dart/simulation/experimental/detail/deformable_vbd/avbd_row_inventory.hpp>
#include <dart/simulation/experimental/detail/deformable_vbd/vertex_block_kernel.hpp>

#include <Eigen/Core>

#include <algorithm>
#include <array>
#include <limits>
#include <tuple>
#include <utility>

#include <cmath>
#include <cstdint>

namespace dart::simulation::experimental::detail::deformable_vbd {

/// A static half-space contact plane `{x : normal . x >= offset}` (the free
/// side). A vertex penetrates when `normal . x < offset`.
struct ContactPlane
{
  Eigen::Vector3d normal = Eigen::Vector3d::UnitY();
  double offset = 0.0;
  double stiffness = 0.0;
};

inline constexpr std::uint64_t kAvbdBoxContactFeatureCodeCount = 27;
inline constexpr std::uint8_t kAvbdContactFeatureKindShift = 56;
inline constexpr std::uint64_t kAvbdContactFeatureIndexMask
    = (std::uint64_t{1} << kAvbdContactFeatureKindShift) - 1u;

enum class AvbdContactFeatureKind : std::uint8_t
{
  Unknown = 0,
  Vertex,
  Edge,
  Face,
  Body,
  Manifold,
};

struct AvbdContactEndpointId
{
  std::uint64_t object = 0;
  std::uint64_t feature = 0;
};

//==============================================================================
inline std::uint64_t packAvbdContactFeatureId(
    AvbdContactFeatureKind kind, std::uint64_t localIndex)
{
  return (static_cast<std::uint64_t>(kind) << kAvbdContactFeatureKindShift)
         | (localIndex & kAvbdContactFeatureIndexMask);
}

//==============================================================================
inline AvbdContactFeatureKind avbdContactFeatureKind(std::uint64_t featureId)
{
  return static_cast<AvbdContactFeatureKind>(
      featureId >> kAvbdContactFeatureKindShift);
}

//==============================================================================
inline std::uint64_t avbdContactFeatureLocalIndex(std::uint64_t featureId)
{
  return featureId & kAvbdContactFeatureIndexMask;
}

//==============================================================================
inline auto avbdContactEndpointTuple(const AvbdContactEndpointId& endpoint)
{
  return std::tuple{endpoint.object, endpoint.feature};
}

//==============================================================================
inline bool operator<(
    const AvbdContactEndpointId& lhs, const AvbdContactEndpointId& rhs)
{
  return avbdContactEndpointTuple(lhs) < avbdContactEndpointTuple(rhs);
}

//==============================================================================
inline bool operator==(
    const AvbdContactEndpointId& lhs, const AvbdContactEndpointId& rhs)
{
  return avbdContactEndpointTuple(lhs) == avbdContactEndpointTuple(rhs);
}

//==============================================================================
inline std::pair<AvbdContactEndpointId, AvbdContactEndpointId>
canonicalizeAvbdContactEndpoints(
    AvbdContactEndpointId first, AvbdContactEndpointId second)
{
  if (second < first) {
    return {second, first};
  }
  return {first, second};
}

//==============================================================================
inline AvbdScalarRowKey makeAvbdEndpointPairRowKey(
    AvbdScalarRowRole role,
    AvbdContactEndpointId first,
    AvbdContactEndpointId second,
    std::uint32_t row = 0,
    std::uint8_t axis = 0)
{
  const auto endpoints = canonicalizeAvbdContactEndpoints(first, second);

  AvbdScalarRowKey key;
  key.role = role;
  key.objectA = endpoints.first.object;
  key.objectB = endpoints.second.object;
  key.featureA = endpoints.first.feature;
  key.featureB = endpoints.second.feature;
  key.row = row;
  key.axis = axis;
  return key;
}

//==============================================================================
inline AvbdScalarRowKey makeAvbdContactManifoldRowKey(
    AvbdScalarRowRole role,
    AvbdContactEndpointId first,
    AvbdContactEndpointId second,
    std::uint32_t row = 0,
    std::uint8_t axis = 0)
{
  return makeAvbdEndpointPairRowKey(role, first, second, row, axis);
}

//==============================================================================
/// Encode the closest box surface feature for AVBD static-obstacle contact row
/// keys. The ternary per-axis code distinguishes the six faces, twelve edges,
/// and eight corners; a point just inside a face maps to that face so rows warm
/// start across small penetrations but reset when contact moves to another box
/// feature.
inline std::uint64_t avbdBoxContactFeatureCode(
    const Eigen::Vector3d& localPosition, const Eigen::Vector3d& halfExtents)
{
  std::array<int, 3> status{1, 1, 1};
  bool outside = false;
  for (int axis = 0; axis < 3; ++axis) {
    if (localPosition[axis] < -halfExtents[axis]) {
      status[axis] = 0;
      outside = true;
    } else if (localPosition[axis] > halfExtents[axis]) {
      status[axis] = 2;
      outside = true;
    }
  }

  if (!outside) {
    int nearestAxis = 0;
    double nearestMargin = std::numeric_limits<double>::infinity();
    for (int axis = 0; axis < 3; ++axis) {
      const double margin = halfExtents[axis] - std::abs(localPosition[axis]);
      if (margin < nearestMargin) {
        nearestMargin = margin;
        nearestAxis = axis;
      }
    }
    status[nearestAxis] = localPosition[nearestAxis] >= 0.0 ? 2 : 0;
  }

  return static_cast<std::uint64_t>(status[0] + 3 * status[1] + 9 * status[2]);
}

//==============================================================================
inline std::uint64_t packAvbdBoxContactFeatureId(
    std::uint64_t boxIndex, std::uint64_t featureCode)
{
  return boxIndex * kAvbdBoxContactFeatureCodeCount
         + (featureCode % kAvbdBoxContactFeatureCodeCount);
}

//==============================================================================
inline AvbdContactFeatureKind avbdBoxContactFeatureKind(
    std::uint64_t featureCode)
{
  std::uint64_t code = featureCode % kAvbdBoxContactFeatureCodeCount;
  std::uint8_t boundaryAxes = 0;
  for (int axis = 0; axis < 3; ++axis) {
    if (code % 3u != 1u) {
      ++boundaryAxes;
    }
    code /= 3u;
  }

  if (boundaryAxes == 1u) {
    return AvbdContactFeatureKind::Face;
  }
  if (boundaryAxes == 2u) {
    return AvbdContactFeatureKind::Edge;
  }
  if (boundaryAxes == 3u) {
    return AvbdContactFeatureKind::Vertex;
  }
  return AvbdContactFeatureKind::Body;
}

/// One active AVBD half-space normal row for a deformable vertex. Row
/// generation and persistence live outside this narrow kernel; this struct is
/// the CPU block-stamping slice for an already-active contact candidate.
struct AvbdHalfSpaceContactRow
{
  std::uint32_t vertex = 0;
  ContactPlane plane;
  AvbdScalarRowState state;
  double previousConstraintValue = 0.0;
  AvbdScalarRowBounds bounds{0.0, std::numeric_limits<double>::infinity()};
};

/// One active AVBD friction-tangent row for a deformable vertex against a
/// half-space contact. The row constrains tangential displacement from the
/// previous step along one tangent axis and clamps its force to a lagged
/// Coulomb limit supplied through `bounds`.
struct AvbdHalfSpaceFrictionRow
{
  std::uint32_t vertex = 0;
  Eigen::Vector3d stepStartPosition = Eigen::Vector3d::Zero();
  Eigen::Vector3d axis = Eigen::Vector3d::UnitX();
  AvbdScalarRowState state;
  double previousConstraintValue = 0.0;
  AvbdScalarRowBounds bounds;
};

/// Per-sweep AVBD normal-contact update parameters.
struct AvbdHalfSpaceContactOptions
{
  double alpha = 0.0;
  double beta = 1.0;
  double maxStiffness = std::numeric_limits<double>::infinity();
};

/// Per-sweep AVBD friction-tangent update parameters.
struct AvbdHalfSpaceFrictionOptions
{
  double alpha = 0.0;
  double beta = 1.0;
  double maxStiffness = std::numeric_limits<double>::infinity();
  double staticFrictionTolerance = 1e-12;
};

//==============================================================================
inline AvbdScalarRowBounds avbdContactNormalBounds()
{
  AvbdScalarRowBounds bounds;
  bounds.lower = 0.0;
  bounds.upper = std::numeric_limits<double>::infinity();
  return bounds;
}

//==============================================================================
inline AvbdScalarRowBounds avbdFrictionTangentBounds(double forceLimit)
{
  const double limit = std::max(0.0, forceLimit);
  AvbdScalarRowBounds bounds;
  bounds.lower = -limit;
  bounds.upper = limit;
  return bounds;
}

//==============================================================================
inline AvbdScalarRowDescriptor makeAvbdContactNormalRowDescriptor(
    AvbdContactEndpointId first,
    AvbdContactEndpointId second,
    double startStiffness,
    double maxStiffness = std::numeric_limits<double>::infinity(),
    std::uint32_t row = 0)
{
  AvbdScalarRowDescriptor descriptor;
  descriptor.key = makeAvbdContactManifoldRowKey(
      AvbdScalarRowRole::ContactNormal, first, second, row, /*axis=*/0);
  descriptor.kind = AvbdScalarRowKind::HardConstraint;
  descriptor.bounds = avbdContactNormalBounds();
  descriptor.startStiffness = startStiffness;
  descriptor.maxStiffness = maxStiffness;
  return descriptor;
}

//==============================================================================
inline AvbdScalarRowDescriptor makeAvbdContactFrictionRowDescriptor(
    AvbdContactEndpointId first,
    AvbdContactEndpointId second,
    std::uint8_t axis,
    double forceLimit,
    double startStiffness,
    double maxStiffness = std::numeric_limits<double>::infinity(),
    std::uint32_t row = 0)
{
  AvbdScalarRowDescriptor descriptor;
  descriptor.key = makeAvbdContactManifoldRowKey(
      AvbdScalarRowRole::FrictionTangent, first, second, row, axis);
  descriptor.kind = AvbdScalarRowKind::HardConstraint;
  descriptor.bounds = avbdFrictionTangentBounds(forceLimit);
  descriptor.startStiffness = startStiffness;
  descriptor.maxStiffness = maxStiffness;
  return descriptor;
}

//==============================================================================
inline double avbdFrictionTangentForceLimit(const AvbdHalfSpaceFrictionRow& row)
{
  const double lowerLimit = row.bounds.lower < 0.0 ? -row.bounds.lower : 0.0;
  const double upperLimit = row.bounds.upper > 0.0 ? row.bounds.upper : 0.0;
  return std::max(0.0, std::min(lowerLimit, upperLimit));
}

//==============================================================================
inline double avbdFrictionTangentPairForceLimit(
    const AvbdHalfSpaceFrictionRow& first,
    const AvbdHalfSpaceFrictionRow& second)
{
  return std::min(
      avbdFrictionTangentForceLimit(first),
      avbdFrictionTangentForceLimit(second));
}

//==============================================================================
inline Eigen::Vector2d projectAvbdFrictionDualToTangentPair(
    double previousFirstLambda,
    double previousSecondLambda,
    const Eigen::Vector3d& previousFirstAxis,
    const Eigen::Vector3d& previousSecondAxis,
    const Eigen::Vector3d& currentFirstAxis,
    const Eigen::Vector3d& currentSecondAxis)
{
  const Eigen::Vector3d worldDual = previousFirstLambda * previousFirstAxis
                                    + previousSecondLambda * previousSecondAxis;
  return Eigen::Vector2d(
      worldDual.dot(currentFirstAxis), worldDual.dot(currentSecondAxis));
}

//==============================================================================
inline double avbdHalfSpaceContactConstraintValue(
    const Eigen::Vector3d& position, const ContactPlane& plane)
{
  return plane.offset - plane.normal.dot(position);
}

//==============================================================================
inline double avbdHalfSpaceFrictionConstraintValue(
    const Eigen::Vector3d& position,
    const Eigen::Vector3d& stepStartPosition,
    const Eigen::Vector3d& axis)
{
  return axis.dot(stepStartPosition - position);
}

//==============================================================================
inline Eigen::Vector2d avbdHalfSpaceFrictionConstraintValues(
    const Eigen::Vector3d& position,
    const AvbdHalfSpaceFrictionRow& first,
    const AvbdHalfSpaceFrictionRow& second,
    double alpha)
{
  return Eigen::Vector2d(
      regularizeAvbdConstraintValue(
          avbdHalfSpaceFrictionConstraintValue(
              position, first.stepStartPosition, first.axis),
          first.previousConstraintValue,
          alpha),
      regularizeAvbdConstraintValue(
          avbdHalfSpaceFrictionConstraintValue(
              position, second.stepStartPosition, second.axis),
          second.previousConstraintValue,
          alpha));
}

//==============================================================================
inline bool avbdFrictionPreviousDualInsideCone(
    const AvbdHalfSpaceFrictionRow& first,
    const AvbdHalfSpaceFrictionRow& second,
    double staticFrictionTolerance = 1e-12)
{
  const double limit = avbdFrictionTangentPairForceLimit(first, second);
  if (!std::isfinite(limit)) {
    return true;
  }

  const double tolerance = std::max(0.0, staticFrictionTolerance);
  const double previousNorm
      = std::hypot(first.state.lambda, second.state.lambda);
  return previousNorm < std::max(0.0, limit - tolerance);
}

//==============================================================================
inline Eigen::Vector2d avbdHalfSpaceFrictionTangentPairForce(
    const Eigen::Vector3d& position,
    const AvbdHalfSpaceFrictionRow& first,
    const AvbdHalfSpaceFrictionRow& second,
    const AvbdHalfSpaceFrictionOptions& options,
    bool* clamped = nullptr)
{
  if (clamped != nullptr) {
    *clamped = false;
  }

  const double limit = avbdFrictionTangentPairForceLimit(first, second);
  if (limit <= 0.0) {
    if (clamped != nullptr) {
      *clamped = true;
    }
    return Eigen::Vector2d::Zero();
  }

  const Eigen::Vector2d constraintValues
      = avbdHalfSpaceFrictionConstraintValues(
          position, first, second, options.alpha);
  const bool staticMode = avbdFrictionPreviousDualInsideCone(
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
/// Add the VBD penalty-contact term for a vertex against a static half-space to
/// `block`. The contact energy is `E_c = (k_c / 2) d^2` with penetration depth
/// `d = max(0, offset - normal . x)`, so contact is active only while the
/// vertex is below the plane:
///   f += k_c d normal,   H += k_c normal normal^T.
/// The Hessian contribution is the rank-1 positive-semidefinite `k_c n n^T`,
/// keeping the per-vertex block positive-definite when combined with inertia.
/// `normal` is assumed unit length.
inline void addHalfSpacePenaltyContact(
    VertexBlock& block,
    const Eigen::Vector3d& position,
    const ContactPlane& plane)
{
  const double gap = plane.normal.dot(position) - plane.offset;
  if (gap >= 0.0 || plane.stiffness <= 0.0) {
    return;
  }
  const double penetration = -gap;
  block.force.noalias() += plane.stiffness * penetration * plane.normal;
  block.hessian.noalias()
      += plane.stiffness * (plane.normal * plane.normal.transpose());
}

//==============================================================================
/// Stamp an active AVBD half-space normal row into a VBD vertex block. The
/// scalar constraint is the penetration value `offset - normal . x`, so a
/// positive row force pushes the vertex along the half-space normal. The caller
/// is responsible for providing only active/persistent contact rows; unlike the
/// penalty helper above, an AVBD row keeps its stiffness block while the row is
/// active even if the current force clamps to zero.
inline double addAvbdHalfSpaceContactNormal(
    VertexBlock& block,
    const Eigen::Vector3d& position,
    const ContactPlane& plane,
    const AvbdScalarRowState& row,
    double previousConstraintValue,
    double alpha,
    AvbdScalarRowBounds bounds = avbdContactNormalBounds())
{
  const double constraintValue = regularizeAvbdConstraintValue(
      avbdHalfSpaceContactConstraintValue(position, plane),
      previousConstraintValue,
      alpha);
  const double forceMagnitude
      = computeAvbdHardConstraintForce(row, constraintValue, bounds);
  block.force.noalias() += forceMagnitude * plane.normal;
  block.hessian.noalias()
      += row.stiffness * (plane.normal * plane.normal.transpose());
  return forceMagnitude;
}

//==============================================================================
inline AvbdScalarRowState updateAvbdHalfSpaceContactNormalRow(
    AvbdScalarRowState row,
    const Eigen::Vector3d& position,
    const ContactPlane& plane,
    const AvbdHalfSpaceContactOptions& options,
    double previousConstraintValue,
    AvbdScalarRowBounds bounds = avbdContactNormalBounds())
{
  const double constraintValue = regularizeAvbdConstraintValue(
      avbdHalfSpaceContactConstraintValue(position, plane),
      previousConstraintValue,
      options.alpha);
  return updateAvbdHardConstraintRow(
      row, constraintValue, options.beta, bounds, options.maxStiffness);
}

//==============================================================================
/// Stamp one active AVBD half-space friction row into a VBD vertex block. The
/// scalar row is the negative tangent displacement since the start of the step:
/// `axis . (x_t - x)`, so a positive row force pushes along `axis` and a
/// negative row force pushes against tangential motion along `axis`. Bounds
/// should be the lagged Coulomb interval `[-mu lambda_n, mu lambda_n]`.
inline double addAvbdHalfSpaceFrictionTangent(
    VertexBlock& block,
    const Eigen::Vector3d& position,
    const AvbdHalfSpaceFrictionRow& row,
    double alpha)
{
  const double constraintValue = regularizeAvbdConstraintValue(
      avbdHalfSpaceFrictionConstraintValue(
          position, row.stepStartPosition, row.axis),
      row.previousConstraintValue,
      alpha);
  const double forceMagnitude
      = computeAvbdHardConstraintForce(row.state, constraintValue, row.bounds);
  block.force.noalias() += forceMagnitude * row.axis;
  block.hessian.noalias()
      += row.state.stiffness * (row.axis * row.axis.transpose());
  return forceMagnitude;
}

//==============================================================================
/// Stamp the two tangent rows for one contact as one Coulomb-cone pair. If the
/// previous tangential dual lies inside the cone, the pair acts as static
/// friction and constrains tangential displacement toward zero. Once the lagged
/// dual reaches the cone edge, the pair switches to dynamic friction and keeps
/// the force on the circular Coulomb bound opposite the current slip.
inline Eigen::Vector2d addAvbdHalfSpaceFrictionTangentPair(
    VertexBlock& block,
    const Eigen::Vector3d& position,
    const AvbdHalfSpaceFrictionRow& first,
    const AvbdHalfSpaceFrictionRow& second,
    const AvbdHalfSpaceFrictionOptions& options)
{
  const Eigen::Vector2d force
      = avbdHalfSpaceFrictionTangentPairForce(position, first, second, options);
  block.force.noalias() += force.x() * first.axis + force.y() * second.axis;
  block.hessian.noalias()
      += first.state.stiffness * (first.axis * first.axis.transpose());
  block.hessian.noalias()
      += second.state.stiffness * (second.axis * second.axis.transpose());
  return force;
}

//==============================================================================
inline AvbdScalarRowState updateAvbdHalfSpaceFrictionTangentRow(
    AvbdScalarRowState state,
    const Eigen::Vector3d& position,
    const AvbdHalfSpaceFrictionRow& row,
    const AvbdHalfSpaceFrictionOptions& options)
{
  const double constraintValue = regularizeAvbdConstraintValue(
      avbdHalfSpaceFrictionConstraintValue(
          position, row.stepStartPosition, row.axis),
      row.previousConstraintValue,
      options.alpha);
  return updateAvbdHardConstraintRow(
      state, constraintValue, options.beta, row.bounds, options.maxStiffness);
}

//==============================================================================
inline void updateAvbdHalfSpaceFrictionTangentPair(
    AvbdHalfSpaceFrictionRow& first,
    AvbdHalfSpaceFrictionRow& second,
    const Eigen::Vector3d& position,
    const AvbdHalfSpaceFrictionOptions& options)
{
  bool clamped = false;
  const Eigen::Vector2d force = avbdHalfSpaceFrictionTangentPairForce(
      position, first, second, options, &clamped);
  const Eigen::Vector2d constraintValues
      = avbdHalfSpaceFrictionConstraintValues(
          position, first, second, options.alpha);

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
/// Add semi-implicit Coulomb friction for a vertex in contact with a static
/// half-space to `block`. Friction resists the tangential displacement since
/// the step start, `u = (I - n n^T)(x - x^t)`, as a tangential penalty
/// `-k_c u` (sticking) clamped to the Coulomb limit `mu * lambda` with
/// `lambda = k_c * penetration` the lagged normal-force magnitude (sliding):
///   sticking (|k_c u| <= mu lambda):  f -= k_c u,        H += k_c (I - n n^T)
///   sliding  (otherwise):             f -= mu lambda u/|u|,
///                                     H += (mu lambda / |u|)(I - n n^T)
/// Both Hessian contributions are positive-semidefinite (a scaled tangential
/// projector). Inactive when the vertex is above the plane. `normal` is unit.
inline void addHalfSpaceFriction(
    VertexBlock& block,
    const Eigen::Vector3d& position,
    const Eigen::Vector3d& stepStartPosition,
    const ContactPlane& plane,
    double frictionCoeff)
{
  const double gap = plane.normal.dot(position) - plane.offset;
  if (gap >= 0.0 || plane.stiffness <= 0.0 || frictionCoeff <= 0.0) {
    return;
  }
  const double normalForce = plane.stiffness * (-gap);
  const Eigen::Matrix3d tangent
      = Eigen::Matrix3d::Identity() - plane.normal * plane.normal.transpose();
  const Eigen::Vector3d delta = position - stepStartPosition;
  const Eigen::Vector3d u = tangent * delta;
  const double uNorm = u.norm();
  if (uNorm <= 1e-12) {
    block.hessian.noalias() += plane.stiffness * tangent;
    return;
  }
  const double coulomb = frictionCoeff * normalForce;
  if (plane.stiffness * uNorm <= coulomb) {
    block.force.noalias() -= plane.stiffness * u;
    block.hessian.noalias() += plane.stiffness * tangent;
  } else {
    block.force.noalias() -= (coulomb / uNorm) * u;
    block.hessian.noalias() += (coulomb / uNorm) * tangent;
  }
}

} // namespace dart::simulation::experimental::detail::deformable_vbd
