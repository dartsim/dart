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

#include <dart/simulation/experimental/body/deformable_body_options.hpp>
#include <dart/simulation/experimental/detail/deformable_contact/barrier_kernel.hpp>
#include <dart/simulation/experimental/detail/deformable_contact/candidate_set.hpp>
#include <dart/simulation/experimental/detail/deformable_vbd/avbd_constraint.hpp>
#include <dart/simulation/experimental/detail/deformable_vbd/vertex_block_kernel.hpp>

#include <Eigen/Core>

#include <algorithm>
#include <array>
#include <limits>
#include <vector>

#include <cmath>
#include <cstddef>
#include <cstdint>

namespace dart::simulation::experimental::detail::deformable_vbd {

namespace contact
    = ::dart::simulation::experimental::detail::deformable_contact;

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
  std::vector<std::vector<SelfContactEntry>> incident;
  double squaredActivationDistance = 0.0;
  double stiffness = 0.0;

  [[nodiscard]] bool active() const
  {
    return stiffness > 0.0 && squaredActivationDistance > 0.0
           && !incident.empty();
  }

  static SelfContactAdjacency build(
      std::size_t vertexCount,
      const contact::ContactCandidateSet& candidates,
      const std::vector<DeformableSurfaceTriangle>& triangles,
      double squaredActivationDistance,
      double stiffness)
  {
    SelfContactAdjacency adjacency;
    adjacency.squaredActivationDistance = squaredActivationDistance;
    adjacency.stiffness = stiffness;
    adjacency.incident.resize(vertexCount);

    const auto scatter = [&](const std::array<std::uint32_t, 4>& nodes,
                             bool isEdgeEdge,
                             std::uint32_t constraint) {
      for (std::uint8_t k = 0; k < 4; ++k) {
        if (nodes[k] < vertexCount) {
          adjacency.incident[nodes[k]].push_back(
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
    return adjacency;
  }
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

//==============================================================================
inline contact::PrimitiveBarrierResult evaluateAvbdSelfContactPrimitive(
    const AvbdSelfContactNormalRow& row,
    const std::vector<Eigen::Vector3d>& positions)
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
    const std::vector<Eigen::Vector3d>& positions)
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
    const std::vector<Eigen::Vector3d>& positions,
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
    const std::vector<Eigen::Vector3d>& positions,
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
    const std::vector<Eigen::Vector3d>& positions)
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

} // namespace dart::simulation::experimental::detail::deformable_vbd
