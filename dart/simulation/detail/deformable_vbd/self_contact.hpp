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

#include <dart/simulation/body/deformable_body_options.hpp>
#include <dart/simulation/detail/deformable_contact/barrier_kernel.hpp>
#include <dart/simulation/detail/deformable_contact/candidate_set.hpp>
#include <dart/simulation/detail/deformable_contact/tangent_stencil.hpp>
#include <dart/simulation/detail/deformable_vbd/avbd_constraint.hpp>
#include <dart/simulation/detail/deformable_vbd/vertex_block_kernel.hpp>

#include <dart/common/stl_allocator.hpp>

#include <Eigen/Core>

#include <algorithm>
#include <array>
#include <limits>
#include <span>
#include <vector>

#include <cmath>
#include <cstddef>
#include <cstdint>

namespace dart::simulation::detail::deformable_vbd {

namespace contact = ::dart::simulation::detail::deformable_contact;

/// One incident self-contact constraint for a single vertex: the four nodes of
/// the point-triangle or edge-edge primitive pair, this vertex's local index
/// (0..3) within that stencil, which barrier (VT vs EE) to evaluate, and the
/// primitive constraint index in point-triangle-then-edge-edge candidate order.
/// This is enough to recompute the vertex's barrier block during a
/// Gauss-Seidel sweep and to stamp a shared AVBD scalar row for the primitive.
struct SelfContactEntry
{
  std::array<std::uint32_t, 4> nodes{0, 0, 0, 0};
  std::uint8_t localVertex = 0;
  bool isEdgeEdge = false;
  std::uint32_t constraint = 0;
};

/// Per-vertex incident self-contact constraints, built once per step (lagged)
/// from a surface contact-candidate set, plus the IPC clamped-log barrier
/// parameters. Each point-triangle candidate contributes to its point and the
/// three triangle nodes; each edge-edge candidate to its four edge nodes.
struct SelfContactAdjacency
{
  using IncidentVector = std::
      vector<SelfContactEntry, ::dart::common::StlAllocator<SelfContactEntry>>;
  using IncidentVectorAllocator = ::dart::common::StlAllocator<IncidentVector>;
  using IncidentRows = std::vector<IncidentVector, IncidentVectorAllocator>;

  SelfContactAdjacency()
    : SelfContactAdjacency(::dart::common::MemoryAllocator::GetDefault())
  {
    // Intentionally empty.
  }

  explicit SelfContactAdjacency(::dart::common::MemoryAllocator& allocator)
    : incident(IncidentVectorAllocator{allocator}), m_allocator(&allocator)
  {
    // Intentionally empty.
  }

  IncidentRows incident;
  double squaredActivationDistance = 0.0;
  double stiffness = 0.0;

  [[nodiscard]] bool active() const
  {
    return stiffness > 0.0 && squaredActivationDistance > 0.0
           && !incident.empty();
  }

  void reserve(std::size_t vertexCount, std::size_t candidateCapacity)
  {
    resizeIncidentRows(vertexCount);
    const std::size_t entryCapacity
        = vertexCount == 0
              ? 0
              : (4 * candidateCapacity + vertexCount - 1) / vertexCount;
    for (auto& entries : incident) {
      entries.reserve(entryCapacity);
    }
  }

  void rebuild(
      std::size_t vertexCount,
      const contact::ContactCandidateSet& candidates,
      std::span<const DeformableSurfaceTriangle> triangles,
      double squaredActivationDistance,
      double stiffness)
  {
    this->squaredActivationDistance = squaredActivationDistance;
    this->stiffness = stiffness;
    resizeIncidentRows(vertexCount);
    for (auto& entries : incident) {
      entries.clear();
    }

    const auto scatter = [&](const std::array<std::uint32_t, 4>& nodes,
                             bool isEdgeEdge,
                             std::uint32_t constraint) {
      for (std::uint8_t k = 0; k < 4; ++k) {
        if (nodes[k] < vertexCount) {
          incident[nodes[k]].push_back(
              SelfContactEntry{nodes, k, isEdgeEdge, constraint});
        }
      }
    };

    std::uint32_t constraint = 0;
    for (const auto& candidate : candidates.pointTriangleCandidates) {
      const auto& triangle = triangles[candidate.triangle];
      scatter(
          {static_cast<std::uint32_t>(candidate.point),
           static_cast<std::uint32_t>(triangle.nodeA),
           static_cast<std::uint32_t>(triangle.nodeB),
           static_cast<std::uint32_t>(triangle.nodeC)},
          /*isEdgeEdge=*/false,
          constraint++);
    }
    for (const auto& candidate : candidates.edgeEdgeCandidates) {
      const auto& edgeA = candidates.surfaceEdges[candidate.edgeA];
      const auto& edgeB = candidates.surfaceEdges[candidate.edgeB];
      scatter(
          {static_cast<std::uint32_t>(edgeA.nodeA),
           static_cast<std::uint32_t>(edgeA.nodeB),
           static_cast<std::uint32_t>(edgeB.nodeA),
           static_cast<std::uint32_t>(edgeB.nodeB)},
          /*isEdgeEdge=*/true,
          constraint++);
    }
  }

  static SelfContactAdjacency build(
      std::size_t vertexCount,
      const contact::ContactCandidateSet& candidates,
      const std::vector<DeformableSurfaceTriangle>& triangles,
      double squaredActivationDistance,
      double stiffness,
      ::dart::common::MemoryAllocator& allocator
      = ::dart::common::MemoryAllocator::GetDefault())
  {
    SelfContactAdjacency adjacency(allocator);
    adjacency.rebuild(
        vertexCount,
        candidates,
        triangles,
        squaredActivationDistance,
        stiffness);
    return adjacency;
  }

private:
  void resizeIncidentRows(std::size_t vertexCount)
  {
    if (vertexCount < incident.size()) {
      incident.resize(vertexCount);
      return;
    }
    incident.reserve(vertexCount);
    while (incident.size() < vertexCount) {
      incident.emplace_back(
          ::dart::common::StlAllocator<SelfContactEntry>{*m_allocator});
    }
  }

  ::dart::common::MemoryAllocator* m_allocator;
};

/// One active AVBD self-contact normal row for a point-triangle or edge-edge
/// primitive pair. The row stores one scalar state for the full primitive; VBD
/// block assembly passes `localVertex` to stamp the matching 3x1 sub-block for
/// whichever stencil vertex is being solved.
struct AvbdSelfContactNormalRow
{
  std::array<std::uint32_t, 4> nodes{0, 0, 0, 0};
  bool isEdgeEdge = false;
  AvbdScalarRowState state;
  double squaredActivationDistance = 0.0;
  double previousConstraintValue = 0.0;
  AvbdScalarRowBounds bounds{0.0, std::numeric_limits<double>::infinity()};
};

/// Per-sweep AVBD self-contact normal update parameters.
struct AvbdSelfContactNormalOptions
{
  double alpha = 0.0;
  double beta = 1.0;
  double maxStiffness = std::numeric_limits<double>::infinity();
};

/// One active AVBD self-contact friction tangent row for a point-triangle or
/// edge-edge primitive pair. Rows are generated from the lagged primitive
/// stencil. Two adjacent rows with axes 0 and 1 form one Coulomb-cone pair.
struct AvbdSelfContactFrictionRow
{
  std::array<std::uint32_t, 4> nodes{0, 0, 0, 0};
  std::array<Eigen::Vector3d, 4> stepStartPositions{
      Eigen::Vector3d::Zero(),
      Eigen::Vector3d::Zero(),
      Eigen::Vector3d::Zero(),
      Eigen::Vector3d::Zero()};
  bool isEdgeEdge = false;
  std::uint8_t axis = 0;
  AvbdScalarRowState state;
  double previousConstraintValue = 0.0;
  AvbdScalarRowBounds bounds;
};

/// Per-sweep AVBD self-contact friction update parameters.
struct AvbdSelfContactFrictionOptions
{
  double alpha = 0.0;
  double beta = 1.0;
  double maxStiffness = std::numeric_limits<double>::infinity();
  double staticFrictionTolerance = 1e-12;
};

//==============================================================================
inline std::uint8_t avbdSelfContactLocalVertex(
    const AvbdSelfContactFrictionRow& row, std::uint32_t vertex)
{
  for (std::uint8_t i = 0; i < 4; ++i) {
    if (row.nodes[i] == vertex) {
      return i;
    }
  }
  return 4u;
}

//==============================================================================
inline bool avbdSelfContactSameFrictionPrimitive(
    const AvbdSelfContactFrictionRow& first,
    const AvbdSelfContactFrictionRow& second)
{
  return first.nodes == second.nodes && first.isEdgeEdge == second.isEdgeEdge;
}

//==============================================================================
inline double avbdSelfContactFrictionForceLimit(
    const AvbdSelfContactFrictionRow& row)
{
  const double lowerLimit = row.bounds.lower < 0.0 ? -row.bounds.lower : 0.0;
  const double upperLimit = row.bounds.upper > 0.0 ? row.bounds.upper : 0.0;
  return std::max(0.0, std::min(lowerLimit, upperLimit));
}

//==============================================================================
inline double avbdSelfContactFrictionPairForceLimit(
    const AvbdSelfContactFrictionRow& first,
    const AvbdSelfContactFrictionRow& second)
{
  return std::min(
      avbdSelfContactFrictionForceLimit(first),
      avbdSelfContactFrictionForceLimit(second));
}

//==============================================================================
inline contact::Matrix2x12d avbdSelfContactFrictionProjection(
    const AvbdSelfContactFrictionRow& row)
{
  return row.isEdgeEdge ? contact::edgeEdgeTangentStencil(
                              row.stepStartPositions[0],
                              row.stepStartPositions[1],
                              row.stepStartPositions[2],
                              row.stepStartPositions[3])
                              .projection
                        : contact::pointTriangleTangentStencil(
                              row.stepStartPositions[0],
                              row.stepStartPositions[1],
                              row.stepStartPositions[2],
                              row.stepStartPositions[3])
                              .projection;
}

//==============================================================================
inline contact::Vector12d avbdSelfContactFrictionDisplacement(
    const AvbdSelfContactFrictionRow& row,
    std::span<const Eigen::Vector3d> positions)
{
  contact::Vector12d displacement = contact::Vector12d::Zero();
  for (std::uint8_t i = 0; i < 4; ++i) {
    const std::uint32_t node = row.nodes[i];
    if (node >= positions.size()) {
      displacement.setZero();
      return displacement;
    }
    displacement.segment<3>(3 * static_cast<int>(i))
        = positions[node] - row.stepStartPositions[i];
  }
  return displacement;
}

//==============================================================================
inline double avbdSelfContactFrictionConstraintValue(
    const AvbdSelfContactFrictionRow& row,
    std::span<const Eigen::Vector3d> positions)
{
  const std::uint8_t axis = row.axis < 2u ? row.axis : 0u;
  const contact::Matrix2x12d projection
      = avbdSelfContactFrictionProjection(row);
  return -projection.row(axis).dot(
      avbdSelfContactFrictionDisplacement(row, positions));
}

//==============================================================================
inline Eigen::Vector2d avbdSelfContactFrictionConstraintValues(
    const AvbdSelfContactFrictionRow& first,
    const AvbdSelfContactFrictionRow& second,
    std::span<const Eigen::Vector3d> positions,
    double alpha)
{
  return Eigen::Vector2d(
      regularizeAvbdConstraintValue(
          avbdSelfContactFrictionConstraintValue(first, positions),
          first.previousConstraintValue,
          alpha),
      regularizeAvbdConstraintValue(
          avbdSelfContactFrictionConstraintValue(second, positions),
          second.previousConstraintValue,
          alpha));
}

//==============================================================================
inline Eigen::Vector3d avbdSelfContactFrictionLocalDirection(
    const AvbdSelfContactFrictionRow& row, std::uint8_t localVertex)
{
  if (localVertex >= 4u) {
    return Eigen::Vector3d::Zero();
  }
  const std::uint8_t axis = row.axis < 2u ? row.axis : 0u;
  const contact::Matrix2x12d projection
      = avbdSelfContactFrictionProjection(row);
  return projection.block<1, 3>(axis, 3 * static_cast<int>(localVertex))
      .transpose();
}

//==============================================================================
inline contact::Vector12d avbdSelfContactFrictionGeneralizedDirection(
    const AvbdSelfContactFrictionRow& row)
{
  const std::uint8_t axis = row.axis < 2u ? row.axis : 0u;
  const contact::Matrix2x12d projection
      = avbdSelfContactFrictionProjection(row);
  return projection.row(axis).transpose();
}

//==============================================================================
inline Eigen::Vector2d projectAvbdSelfContactFrictionDualToTangentPair(
    double previousFirstLambda,
    double previousSecondLambda,
    const AvbdSelfContactFrictionRow& previousFirst,
    const AvbdSelfContactFrictionRow& previousSecond,
    const AvbdSelfContactFrictionRow& currentFirst,
    const AvbdSelfContactFrictionRow& currentSecond)
{
  const contact::Vector12d previousDual
      = previousFirstLambda
            * avbdSelfContactFrictionGeneralizedDirection(previousFirst)
        + previousSecondLambda
              * avbdSelfContactFrictionGeneralizedDirection(previousSecond);

  const contact::Vector12d currentFirstDirection
      = avbdSelfContactFrictionGeneralizedDirection(currentFirst);
  const contact::Vector12d currentSecondDirection
      = avbdSelfContactFrictionGeneralizedDirection(currentSecond);

  const double gram00 = currentFirstDirection.squaredNorm();
  const double gram01 = currentFirstDirection.dot(currentSecondDirection);
  const double gram11 = currentSecondDirection.squaredNorm();
  const double rhs0 = currentFirstDirection.dot(previousDual);
  const double rhs1 = currentSecondDirection.dot(previousDual);
  const double determinant = gram00 * gram11 - gram01 * gram01;
  const double scale = std::max({1.0, std::abs(gram00), std::abs(gram11)});
  if (std::isfinite(determinant)
      && std::abs(determinant)
             > 64.0 * std::numeric_limits<double>::epsilon() * scale * scale) {
    return Eigen::Vector2d(
        (gram11 * rhs0 - gram01 * rhs1) / determinant,
        (gram00 * rhs1 - gram01 * rhs0) / determinant);
  }

  Eigen::Vector2d projected = Eigen::Vector2d::Zero();
  if (std::isfinite(gram00) && gram00 > 0.0) {
    projected.x() = rhs0 / gram00;
  }
  if (std::isfinite(gram11) && gram11 > 0.0) {
    projected.y() = rhs1 / gram11;
  }
  return projected;
}

//==============================================================================
inline bool avbdSelfContactFrictionPreviousDualInsideCone(
    const AvbdSelfContactFrictionRow& first,
    const AvbdSelfContactFrictionRow& second,
    double staticFrictionTolerance = 1e-12)
{
  const double limit = avbdSelfContactFrictionPairForceLimit(first, second);
  if (!std::isfinite(limit)) {
    return true;
  }

  const double tolerance = std::max(0.0, staticFrictionTolerance);
  const double previousNorm
      = std::hypot(first.state.lambda, second.state.lambda);
  return previousNorm < std::max(0.0, limit - tolerance);
}

//==============================================================================
inline Eigen::Vector2d avbdSelfContactFrictionTangentPairForce(
    const AvbdSelfContactFrictionRow& first,
    const AvbdSelfContactFrictionRow& second,
    std::span<const Eigen::Vector3d> positions,
    const AvbdSelfContactFrictionOptions& options,
    bool* clamped = nullptr)
{
  if (clamped != nullptr) {
    *clamped = false;
  }

  const double limit = avbdSelfContactFrictionPairForceLimit(first, second);
  if (limit <= 0.0) {
    if (clamped != nullptr) {
      *clamped = true;
    }
    return Eigen::Vector2d::Zero();
  }

  const Eigen::Vector2d constraintValues
      = avbdSelfContactFrictionConstraintValues(
          first, second, positions, options.alpha);
  const bool staticMode = avbdSelfContactFrictionPreviousDualInsideCone(
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
inline contact::PrimitiveBarrierResult evaluateAvbdSelfContactPrimitive(
    const AvbdSelfContactNormalRow& row,
    std::span<const Eigen::Vector3d> positions)
{
  if (!(row.squaredActivationDistance > 0.0)) {
    return {};
  }

  const auto validNode = [&](std::uint32_t node) {
    return node < positions.size();
  };
  const auto& n = row.nodes;
  if (!validNode(n[0]) || !validNode(n[1]) || !validNode(n[2])
      || !validNode(n[3])) {
    return {};
  }

  constexpr double kUnitBarrierStiffness = 1.0;
  return row.isEdgeEdge ? contact::edgeEdgeBarrier(
                              positions[n[0]],
                              positions[n[1]],
                              positions[n[2]],
                              positions[n[3]],
                              row.squaredActivationDistance,
                              kUnitBarrierStiffness)
                        : contact::pointTriangleBarrier(
                              positions[n[0]],
                              positions[n[1]],
                              positions[n[2]],
                              positions[n[3]],
                              row.squaredActivationDistance,
                              kUnitBarrierStiffness);
}

//==============================================================================
inline double avbdSelfContactNormalConstraintValue(
    const contact::PrimitiveBarrierResult& primitive)
{
  if (!primitive.active || !(primitive.squaredActivationDistance > 0.0)
      || !std::isfinite(primitive.safeSquaredDistance)
      || !std::isfinite(primitive.squaredActivationDistance)) {
    return 0.0;
  }

  const double activationDistance
      = std::sqrt(primitive.squaredActivationDistance);
  const double safeDistance
      = std::sqrt(std::max(0.0, primitive.safeSquaredDistance));
  if (!std::isfinite(activationDistance) || !std::isfinite(safeDistance)) {
    return 0.0;
  }
  return activationDistance - safeDistance;
}

//==============================================================================
inline double avbdSelfContactNormalConstraintValue(
    const AvbdSelfContactNormalRow& row,
    std::span<const Eigen::Vector3d> positions)
{
  return avbdSelfContactNormalConstraintValue(
      evaluateAvbdSelfContactPrimitive(row, positions));
}

//==============================================================================
inline Eigen::Vector3d avbdSelfContactLocalNormal(
    const contact::PrimitiveBarrierResult& primitive, std::uint8_t localVertex)
{
  if (!primitive.active || localVertex >= 4u) {
    return Eigen::Vector3d::Zero();
  }

  Eigen::Vector3d normal
      = -primitive.gradient.segment<3>(3 * static_cast<int>(localVertex));
  const double normalNorm = normal.norm();
  if (!(normalNorm > 0.0) || !std::isfinite(normalNorm)) {
    return Eigen::Vector3d::Zero();
  }
  normal /= normalNorm;
  return normal;
}

//==============================================================================
/// Stamp one active AVBD self-contact normal row into a VBD vertex block. The
/// scalar constraint is the activation-band depth `d_hat - d`, and the normal
/// axis is taken from the IPC barrier force direction for this local primitive
/// vertex. The row keeps a rank-1 positive-semidefinite Hessian block while the
/// primitive is active, matching the other hard AVBD normal rows.
inline double addAvbdSelfContactNormal(
    VertexBlock& block,
    std::span<const Eigen::Vector3d> positions,
    const AvbdSelfContactNormalRow& row,
    std::uint8_t localVertex,
    double alpha)
{
  const contact::PrimitiveBarrierResult primitive
      = evaluateAvbdSelfContactPrimitive(row, positions);
  if (!primitive.active) {
    return 0.0;
  }

  const Eigen::Vector3d normal
      = avbdSelfContactLocalNormal(primitive, localVertex);
  if (normal.squaredNorm() == 0.0) {
    return 0.0;
  }

  const double constraintValue = regularizeAvbdConstraintValue(
      avbdSelfContactNormalConstraintValue(primitive),
      row.previousConstraintValue,
      alpha);
  const double forceMagnitude
      = computeAvbdHardConstraintForce(row.state, constraintValue, row.bounds);
  block.force.noalias() += forceMagnitude * normal;
  block.hessian.noalias()
      += row.state.stiffness * (normal * normal.transpose());
  return forceMagnitude;
}

//==============================================================================
inline AvbdScalarRowState updateAvbdSelfContactNormalRow(
    AvbdScalarRowState state,
    std::span<const Eigen::Vector3d> positions,
    const AvbdSelfContactNormalRow& row,
    const AvbdSelfContactNormalOptions& options)
{
  const contact::PrimitiveBarrierResult primitive
      = evaluateAvbdSelfContactPrimitive(row, positions);
  if (!primitive.active) {
    return state;
  }

  const double constraintValue = regularizeAvbdConstraintValue(
      avbdSelfContactNormalConstraintValue(primitive),
      row.previousConstraintValue,
      options.alpha);
  return updateAvbdHardConstraintRow(
      state, constraintValue, options.beta, row.bounds, options.maxStiffness);
}

//==============================================================================
/// Stamp one active AVBD self-contact friction tangent row into a VBD vertex
/// block. The row constrains the lagged tangent-stencil displacement from the
/// start of the step, so the force opposes relative tangential slip across the
/// primitive pair.
inline double addAvbdSelfContactFrictionTangent(
    VertexBlock& block,
    std::span<const Eigen::Vector3d> positions,
    const AvbdSelfContactFrictionRow& row,
    std::uint8_t localVertex,
    double alpha)
{
  const Eigen::Vector3d direction
      = avbdSelfContactFrictionLocalDirection(row, localVertex);
  if (direction.squaredNorm() == 0.0) {
    return 0.0;
  }

  const double constraintValue = regularizeAvbdConstraintValue(
      avbdSelfContactFrictionConstraintValue(row, positions),
      row.previousConstraintValue,
      alpha);
  const double forceMagnitude
      = computeAvbdHardConstraintForce(row.state, constraintValue, row.bounds);
  block.force.noalias() += forceMagnitude * direction;
  block.hessian.noalias()
      += row.state.stiffness * (direction * direction.transpose());
  return forceMagnitude;
}

//==============================================================================
/// Stamp the two self-contact tangent rows for one primitive as one
/// Coulomb-cone pair. If the previous tangential dual lies inside the cone,
/// the pair acts as static friction. Once the lagged dual reaches the cone
/// edge, the pair switches to dynamic friction and keeps the force on the
/// circular Coulomb bound opposite the current slip.
inline Eigen::Vector2d addAvbdSelfContactFrictionTangentPair(
    VertexBlock& block,
    std::span<const Eigen::Vector3d> positions,
    const AvbdSelfContactFrictionRow& first,
    const AvbdSelfContactFrictionRow& second,
    std::uint8_t localVertex,
    const AvbdSelfContactFrictionOptions& options)
{
  const Eigen::Vector2d force = avbdSelfContactFrictionTangentPairForce(
      first, second, positions, options);
  const Eigen::Vector3d direction0
      = avbdSelfContactFrictionLocalDirection(first, localVertex);
  const Eigen::Vector3d direction1
      = avbdSelfContactFrictionLocalDirection(second, localVertex);
  block.force.noalias() += force.x() * direction0 + force.y() * direction1;
  block.hessian.noalias()
      += first.state.stiffness * (direction0 * direction0.transpose());
  block.hessian.noalias()
      += second.state.stiffness * (direction1 * direction1.transpose());
  return force;
}

//==============================================================================
inline AvbdScalarRowState updateAvbdSelfContactFrictionTangentRow(
    AvbdScalarRowState state,
    std::span<const Eigen::Vector3d> positions,
    const AvbdSelfContactFrictionRow& row,
    const AvbdSelfContactFrictionOptions& options)
{
  const double constraintValue = regularizeAvbdConstraintValue(
      avbdSelfContactFrictionConstraintValue(row, positions),
      row.previousConstraintValue,
      options.alpha);
  return updateAvbdHardConstraintRow(
      state, constraintValue, options.beta, row.bounds, options.maxStiffness);
}

//==============================================================================
inline void updateAvbdSelfContactFrictionTangentPair(
    AvbdSelfContactFrictionRow& first,
    AvbdSelfContactFrictionRow& second,
    std::span<const Eigen::Vector3d> positions,
    const AvbdSelfContactFrictionOptions& options)
{
  bool clamped = false;
  const Eigen::Vector2d force = avbdSelfContactFrictionTangentPairForce(
      first, second, positions, options, &clamped);
  const Eigen::Vector2d constraintValues
      = avbdSelfContactFrictionConstraintValues(
          first, second, positions, options.alpha);

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

/// Add the IPC clamped-log self-contact barrier force and 3x3 Hessian block for
/// one vertex's incident constraints to its VertexBlock. The other stencil
/// nodes are read at their current positions (Gauss-Seidel), and only the
/// vertex's own 3x1 gradient sub-block and 3x3 diagonal Hessian block are
/// extracted from each primitive's 12-vector / 12x12 barrier result -- the same
/// per-vertex reduction VBD applies to tetrahedral elements. Inactive
/// (out-of-band) contacts add nothing, so a body that is not folding onto
/// itself is a no-op.
inline void addSelfContactTerms(
    VertexBlock& block,
    std::uint32_t vertex,
    const SelfContactAdjacency& selfContact,
    std::span<const Eigen::Vector3d> positions)
{
  if (!selfContact.active() || vertex >= selfContact.incident.size()) {
    return;
  }
  for (const SelfContactEntry& entry : selfContact.incident[vertex]) {
    const auto& n = entry.nodes;
    const contact::PrimitiveBarrierResult result
        = entry.isEdgeEdge ? contact::edgeEdgeBarrier(
                                 positions[n[0]],
                                 positions[n[1]],
                                 positions[n[2]],
                                 positions[n[3]],
                                 selfContact.squaredActivationDistance,
                                 selfContact.stiffness)
                           : contact::pointTriangleBarrier(
                                 positions[n[0]],
                                 positions[n[1]],
                                 positions[n[2]],
                                 positions[n[3]],
                                 selfContact.squaredActivationDistance,
                                 selfContact.stiffness);
    if (!result.active) {
      continue;
    }
    const int base = 3 * static_cast<int>(entry.localVertex);
    block.force -= result.gradient.segment<3>(base);
    block.hessian += result.hessian.block<3, 3>(base, base);
  }
}

} // namespace dart::simulation::detail::deformable_vbd
