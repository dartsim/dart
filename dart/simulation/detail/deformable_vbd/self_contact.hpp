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
#include <dart/simulation/detail/deformable_vbd/quasi_newton_hessian.hpp>
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

//==============================================================================
/// The one indexing contract for self-contact primitives.
///
/// A candidate is admitted only when every index it dereferences resolves, and
/// admitted candidates are numbered densely in candidate-set order: all
/// admitted point-triangle candidates first, then all admitted edge-edge
/// candidates. `SelfContactAdjacency::rebuild` stamps those numbers into
/// `SelfContactEntry::constraint`, so any AVBD row array addressed through that
/// field -- the World self-contact normal rows above all -- must admit and skip
/// exactly the same candidates, or `constraint` will index the wrong row.
[[nodiscard]] inline bool isSelfContactPointTriangleCandidateInRange(
    const contact::PointTriangleCandidate& candidate,
    std::size_t triangleCount) noexcept
{
  return candidate.triangle < triangleCount;
}

//==============================================================================
/// Edge-edge half of the contract documented on
/// `isSelfContactPointTriangleCandidateInRange`.
[[nodiscard]] inline bool isSelfContactEdgeEdgeCandidateInRange(
    const contact::EdgeEdgeCandidate& candidate,
    std::size_t surfaceEdgeCount) noexcept
{
  return candidate.edgeA < surfaceEdgeCount
         && candidate.edgeB < surfaceEdgeCount;
}

//==============================================================================
/// Number of self-contact constraints the contract above admits from
/// `candidates`. `SelfContactAdjacency::rebuild` numbers its entries
/// `0 .. selfContactConstraintCount() - 1`, so this is exactly the length any
/// row array addressed through `SelfContactEntry::constraint` must have.
[[nodiscard]] inline std::size_t selfContactConstraintCount(
    const contact::ContactCandidateSet& candidates, std::size_t triangleCount)
{
  std::size_t count = 0;
  for (const auto& candidate : candidates.pointTriangleCandidates) {
    if (isSelfContactPointTriangleCandidateInRange(candidate, triangleCount)) {
      ++count;
    }
  }
  for (const auto& candidate : candidates.edgeEdgeCandidates) {
    if (isSelfContactEdgeEdgeCandidateInRange(
            candidate, candidates.surfaceEdges.size())) {
      ++count;
    }
  }
  return count;
}

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
  using EntryVector = std::
      vector<SelfContactEntry, ::dart::common::StlAllocator<SelfContactEntry>>;
  using CountVector
      = std::vector<std::size_t, ::dart::common::StlAllocator<std::size_t>>;

  SelfContactAdjacency()
    : SelfContactAdjacency(::dart::common::MemoryAllocator::GetDefault())
  {
    // Intentionally empty.
  }

  explicit SelfContactAdjacency(::dart::common::MemoryAllocator& allocator)
    : m_entries(::dart::common::StlAllocator<SelfContactEntry>{allocator}),
      m_incidentCounts(::dart::common::StlAllocator<std::size_t>{allocator}),
      m_offsets(::dart::common::StlAllocator<std::size_t>{allocator}),
      m_writeOffsets(::dart::common::StlAllocator<std::size_t>{allocator})
  {
    // Intentionally empty.
  }

  double squaredActivationDistance = 0.0;
  double stiffness = 0.0;

  [[nodiscard]] bool active() const
  {
    return stiffness > 0.0 && squaredActivationDistance > 0.0
           && m_vertexCount != 0u;
  }

  [[nodiscard]] std::size_t vertexCount() const
  {
    return m_vertexCount;
  }

  [[nodiscard]] std::span<const SelfContactEntry> entriesFor(
      std::size_t vertex) const
  {
    if (vertex >= m_vertexCount) {
      return {};
    }
    const std::size_t begin = m_offsets[vertex];
    const std::size_t count = m_offsets[vertex + 1u] - begin;
    if (count == 0u) {
      return {};
    }
    return {m_entries.data() + begin, count};
  }

  [[nodiscard]] const EntryVector& entries() const
  {
    return m_entries;
  }

  void clear()
  {
    m_entries.clear();
    m_incidentCounts.clear();
    m_offsets.clear();
    m_writeOffsets.clear();
    m_vertexCount = 0u;
    squaredActivationDistance = 0.0;
    stiffness = 0.0;
  }

  void reserve(std::size_t vertexCount, std::size_t candidateCapacity)
  {
    DART_SIMULATION_THROW_T_IF(
        candidateCapacity
            > std::numeric_limits<std::size_t>::max() / std::size_t{4},
        InvalidOperationException,
        "Self-contact adjacency capacity overflows size_t");
    m_entries.reserve(4u * candidateCapacity);
    m_incidentCounts.reserve(vertexCount);
    DART_SIMULATION_THROW_T_IF(
        vertexCount == std::numeric_limits<std::size_t>::max(),
        InvalidOperationException,
        "Self-contact adjacency vertex count overflows size_t");
    m_offsets.reserve(vertexCount + 1u);
    m_writeOffsets.reserve(vertexCount);
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
    m_vertexCount = vertexCount;

    // Count and scatter into one CSR buffer. World preparation reserves this
    // buffer from the resolved combined candidate cap, so a later active-set
    // change cannot allocate one vector per vertex during the warmed solve.
    m_incidentCounts.assign(vertexCount, 0u);
    const auto countNodes = [&](const std::array<std::uint32_t, 4>& nodes) {
      for (const std::uint32_t node : nodes) {
        if (node < vertexCount) {
          ++m_incidentCounts[node];
        }
      }
    };
    for (const auto& candidate : candidates.pointTriangleCandidates) {
      if (!isSelfContactPointTriangleCandidateInRange(
              candidate, triangles.size())) {
        continue;
      }
      const auto& triangle = triangles[candidate.triangle];
      countNodes(
          {static_cast<std::uint32_t>(candidate.point),
           static_cast<std::uint32_t>(triangle.nodeA),
           static_cast<std::uint32_t>(triangle.nodeB),
           static_cast<std::uint32_t>(triangle.nodeC)});
    }
    for (const auto& candidate : candidates.edgeEdgeCandidates) {
      if (!isSelfContactEdgeEdgeCandidateInRange(
              candidate, candidates.surfaceEdges.size())) {
        continue;
      }
      const auto& edgeA = candidates.surfaceEdges[candidate.edgeA];
      const auto& edgeB = candidates.surfaceEdges[candidate.edgeB];
      countNodes(
          {static_cast<std::uint32_t>(edgeA.nodeA),
           static_cast<std::uint32_t>(edgeA.nodeB),
           static_cast<std::uint32_t>(edgeB.nodeA),
           static_cast<std::uint32_t>(edgeB.nodeB)});
    }
    m_offsets.resize(vertexCount + 1u);
    m_offsets[0] = 0u;
    for (std::size_t vertex = 0; vertex < vertexCount; ++vertex) {
      DART_SIMULATION_THROW_T_IF(
          m_incidentCounts[vertex]
              > std::numeric_limits<std::size_t>::max() - m_offsets[vertex],
          InvalidOperationException,
          "Self-contact adjacency entry count overflows size_t");
      m_offsets[vertex + 1u] = m_offsets[vertex] + m_incidentCounts[vertex];
    }
    m_entries.resize(m_offsets.back());
    m_writeOffsets.assign(m_offsets.begin(), m_offsets.end() - 1);

    const auto scatter = [&](const std::array<std::uint32_t, 4>& nodes,
                             bool isEdgeEdge,
                             std::uint32_t constraint) {
      for (std::uint8_t k = 0; k < 4; ++k) {
        if (nodes[k] < vertexCount) {
          m_entries[m_writeOffsets[nodes[k]]++]
              = SelfContactEntry{nodes, k, isEdgeEdge, constraint};
        }
      }
    };

    std::uint32_t constraint = 0;
    for (const auto& candidate : candidates.pointTriangleCandidates) {
      if (!isSelfContactPointTriangleCandidateInRange(
              candidate, triangles.size())) {
        continue;
      }
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
      if (!isSelfContactEdgeEdgeCandidateInRange(
              candidate, candidates.surfaceEdges.size())) {
        continue;
      }
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
    DART_SIMULATION_THROW_T_IF(
        candidates.pointTriangleCandidates.size()
            > std::numeric_limits<std::size_t>::max()
                  - candidates.edgeEdgeCandidates.size(),
        InvalidOperationException,
        "Self-contact candidate count overflows size_t");
    adjacency.reserve(
        vertexCount,
        candidates.pointTriangleCandidates.size()
            + candidates.edgeEdgeCandidates.size());
    adjacency.rebuild(
        vertexCount,
        candidates,
        triangles,
        squaredActivationDistance,
        stiffness);
    return adjacency;
  }

private:
  EntryVector m_entries;
  CountVector m_incidentCounts;
  CountVector m_offsets;
  CountVector m_writeOffsets;
  std::size_t m_vertexCount = 0u;
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
/// edge-edge primitive pair. Rows use the current step's lagged primitive
/// stencil plus a persistent sticking displacement transported from the prior
/// stencil. Two adjacent rows with axes 0 and 1 form one live Coulomb-cone
/// pair coupled to its owning normal row.
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
  /// Relative tangential motion accumulated while the primitive pair sticks,
  /// transported from the prior tangent stencil into this row's world-space
  /// tangent plane when the contact is rebuilt.
  Eigen::Vector3d accumulatedTangentialDisplacement = Eigen::Vector3d::Zero();
  std::size_t normalRow = std::numeric_limits<std::size_t>::max();
  double frictionCoefficient = 0.0;
  bool sticking = false;
  /// True only while the owning normal primitive is inside the active band
  /// but has no safe differential (for example, at the barrier safety floor).
  /// The complete normal/tangent continuation is then preserved and the row
  /// emits no primal or dual contribution until the primitive is valid again.
  bool differentialSuspended = false;
};

/// Per-sweep AVBD self-contact friction update parameters.
struct AvbdSelfContactFrictionOptions
{
  double alpha = 0.0;
  double beta = 1.0;
  double maxStiffness = std::numeric_limits<double>::infinity();
  /// Tangential-displacement threshold for retaining a sticking anchor. This
  /// matches the maintained 3D AVBD reference implementation.
  double staticFrictionTolerance = 1e-5;
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
inline contact::Matrix3x2d avbdSelfContactFrictionBasis(
    const AvbdSelfContactFrictionRow& row)
{
  return row.isEdgeEdge ? contact::edgeEdgeTangentStencil(
                              row.stepStartPositions[0],
                              row.stepStartPositions[1],
                              row.stepStartPositions[2],
                              row.stepStartPositions[3])
                              .basis
                        : contact::pointTriangleTangentStencil(
                              row.stepStartPositions[0],
                              row.stepStartPositions[1],
                              row.stepStartPositions[2],
                              row.stepStartPositions[3])
                              .basis;
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
  const contact::Matrix3x2d basis = avbdSelfContactFrictionBasis(row);
  const contact::Matrix2x12d projection
      = avbdSelfContactFrictionProjection(row);
  const Eigen::Vector2d incrementalCoordinates
      = projection * avbdSelfContactFrictionDisplacement(row, positions);
  const Eigen::Vector3d totalDisplacement
      = row.accumulatedTangentialDisplacement + basis * incrementalCoordinates;
  return -basis.col(axis).dot(totalDisplacement);
}

//==============================================================================
inline Eigen::Vector3d avbdSelfContactFrictionTotalTangentialDisplacement(
    const AvbdSelfContactFrictionRow& row,
    std::span<const Eigen::Vector3d> positions)
{
  const contact::Matrix3x2d basis = avbdSelfContactFrictionBasis(row);
  return row.accumulatedTangentialDisplacement
         + basis
               * (avbdSelfContactFrictionProjection(row)
                  * avbdSelfContactFrictionDisplacement(row, positions));
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
    const AvbdSelfContactFrictionRow& second)
{
  const double limit = avbdSelfContactFrictionPairForceLimit(first, second);
  if (!std::isfinite(limit)) {
    return true;
  }

  const double previousNorm
      = std::hypot(first.state.lambda, second.state.lambda);
  return previousNorm <= limit;
}

//==============================================================================
inline void setAvbdSelfContactFrictionTangentPairForceLimit(
    AvbdSelfContactFrictionRow& first,
    AvbdSelfContactFrictionRow& second,
    double forceLimit)
{
  const double limit
      = std::isfinite(forceLimit) && forceLimit > 0.0 ? forceLimit : 0.0;
  first.bounds = avbdFrictionTangentBounds(limit);
  second.bounds = first.bounds;

  if (!(limit > 0.0)) {
    // With no admissible normal load there is no tangent constraint to retain.
    // Cold-clear the material-point continuation while preserving the learned
    // stiffness for a later, genuinely active contact.
    first.state.lambda = 0.0;
    second.state.lambda = 0.0;
    first.sticking = false;
    second.sticking = false;
    first.accumulatedTangentialDisplacement.setZero();
    second.accumulatedTangentialDisplacement.setZero();
    return;
  }

  const double norm = std::hypot(first.state.lambda, second.state.lambda);
  if (norm > limit && norm > 0.0) {
    const double scale = limit / norm;
    first.state.lambda *= scale;
    second.state.lambda *= scale;
    first.sticking = false;
    second.sticking = false;
  }
}

//==============================================================================
inline Eigen::Vector2d avbdSelfContactFrictionTangentPairForce(
    const AvbdSelfContactFrictionRow& first,
    const AvbdSelfContactFrictionRow& second,
    std::span<const Eigen::Vector3d> positions,
    const AvbdSelfContactFrictionOptions& options,
    bool* clamped = nullptr,
    bool* valid = nullptr)
{
  if (clamped != nullptr) {
    *clamped = false;
  }
  if (valid != nullptr) {
    *valid = true;
  }

  const double limit = avbdSelfContactFrictionPairForceLimit(first, second);
  if (!(limit > 0.0)) {
    return Eigen::Vector2d::Zero();
  }
  if (!std::isfinite(limit)) {
    if (valid != nullptr) {
      *valid = false;
    }
    return Eigen::Vector2d::Zero();
  }

  const Eigen::Vector2d constraintValues
      = avbdSelfContactFrictionConstraintValues(
          first, second, positions, options.alpha);
  // Equation 13 first forms the augmented-Lagrangian trial force k*C+lambda.
  // Project that trial to the live Coulomb cone only when it exceeds the cone;
  // a prior dual on the boundary may return to the interior instead of
  // discarding lambda and choosing a direction from C alone.
  Eigen::Vector2d force(
      first.state.stiffness * constraintValues.x() + first.state.lambda,
      second.state.stiffness * constraintValues.y() + second.state.lambda);
  if (!force.allFinite()) {
    if (valid != nullptr) {
      *valid = false;
    }
    return Eigen::Vector2d::Zero();
  }
  const double norm = force.norm();
  if (!std::isfinite(norm)) {
    if (valid != nullptr) {
      *valid = false;
    }
    return Eigen::Vector2d::Zero();
  }
  if (norm > limit && norm > 0.0) {
    if (clamped != nullptr) {
      *clamped = true;
    }
    force *= limit / norm;
  }
  return force;
}

//==============================================================================
struct AvbdSelfContactPrimitiveResult
{
  contact::Vector12d distanceGradient = contact::Vector12d::Zero();
  contact::Matrix12d constraintHessian = contact::Matrix12d::Zero();
  double squaredDistance = 0.0;
  double safeSquaredDistance = 0.0;
  double squaredActivationDistance = 0.0;
  bool active = false;
  /// False when the distance is outside the active band, invalid, or below
  /// the barrier safety floor.  In the under-floor case the clamped barrier
  /// value remains finite, but differentiating it as if it were the raw
  /// distance would be mathematically inconsistent, so AVBD emits no row.
  bool differentialValid = false;
  bool clampedToSafetyFloor = false;
};

//==============================================================================
inline AvbdSelfContactPrimitiveResult evaluateAvbdSelfContactPrimitive(
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

  AvbdSelfContactPrimitiveResult result;
  result.squaredActivationDistance = row.squaredActivationDistance;
  result.squaredDistance = row.isEdgeEdge
                               ? contact::edgeEdgeSquaredDistance(
                                     positions[n[0]],
                                     positions[n[1]],
                                     positions[n[2]],
                                     positions[n[3]])
                                     .squaredDistance
                               : contact::pointTriangleSquaredDistance(
                                     positions[n[0]],
                                     positions[n[1]],
                                     positions[n[2]],
                                     positions[n[3]])
                                     .squaredDistance;
  const auto barrier = contact::c2ClampedLogBarrier(
      result.squaredDistance, row.squaredActivationDistance);
  result.safeSquaredDistance = barrier.safeSquaredDistance;
  result.active = barrier.active;
  if (!result.active || !(result.safeSquaredDistance > 0.0)) {
    return result;
  }
  const double safetyFloor = contact::detail::safeSquaredBarrierDistance(
      0.0, row.squaredActivationDistance);
  // Constructing a primitive at sqrt(safetyFloor) may round its re-squared
  // distance one representable value above the stored floor. Treat that
  // boundary value as clamped too; no exact differential exists at the max()
  // transition, and the one-ULP fail-closed band is deterministic.
  const double inclusiveSafetyFloor
      = std::nextafter(safetyFloor, std::numeric_limits<double>::infinity());
  if (result.squaredDistance <= inclusiveSafetyFloor) {
    result.active = false;
    result.clampedToSafetyFloor = true;
    return result;
  }

  const contact::Vector12d squaredDistanceGradient
      = row.isEdgeEdge ? contact::edgeEdgeSquaredDistanceGradient(
                             positions[n[0]],
                             positions[n[1]],
                             positions[n[2]],
                             positions[n[3]])
                       : contact::pointTriangleSquaredDistanceGradient(
                             positions[n[0]],
                             positions[n[1]],
                             positions[n[2]],
                             positions[n[3]]);
  const contact::Matrix12d squaredDistanceHessian
      = row.isEdgeEdge ? contact::edgeEdgeSquaredDistanceHessian(
                             positions[n[0]],
                             positions[n[1]],
                             positions[n[2]],
                             positions[n[3]])
                       : contact::pointTriangleSquaredDistanceHessian(
                             positions[n[0]],
                             positions[n[1]],
                             positions[n[2]],
                             positions[n[3]]);
  const double distance = std::sqrt(result.safeSquaredDistance);
  const double inverseTwoDistance = 0.5 / distance;
  result.distanceGradient = inverseTwoDistance * squaredDistanceGradient;
  const contact::Matrix12d distanceHessian
      = inverseTwoDistance * squaredDistanceHessian
        - (0.25 / (distance * distance * distance))
              * (squaredDistanceGradient * squaredDistanceGradient.transpose());
  result.constraintHessian = -distanceHessian;
  if (!result.distanceGradient.allFinite()
      || !result.constraintHessian.allFinite()) {
    result.active = false;
    result.distanceGradient.setZero();
    result.constraintHessian.setZero();
    return result;
  }
  result.differentialValid = true;
  return result;
}

//==============================================================================
inline double avbdSelfContactNormalConstraintValue(
    const AvbdSelfContactPrimitiveResult& primitive)
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
inline contact::Vector12d avbdSelfContactGeneralizedNormalDirection(
    const AvbdSelfContactPrimitiveResult& primitive)
{
  if (!primitive.active || !primitive.distanceGradient.allFinite()) {
    return contact::Vector12d::Zero();
  }
  return primitive.distanceGradient;
}

//==============================================================================
inline Eigen::Vector3d avbdSelfContactLocalNormal(
    const AvbdSelfContactPrimitiveResult& primitive, std::uint8_t localVertex)
{
  if (localVertex >= 4u) {
    return Eigen::Vector3d::Zero();
  }
  return avbdSelfContactGeneralizedNormalDirection(primitive).segment<3>(
      3 * static_cast<int>(localVertex));
}

//==============================================================================
/// Stamp one active AVBD self-contact normal row into a VBD vertex block. The
/// scalar constraint is the activation-band depth `d_hat - d`, and each local
/// direction is the corresponding block of `grad(d)`, including the shared
/// primitive's barycentric/segment weight. The local Hessian combines the
/// rank-one penalty block with AVBD Section 3.5's column-norm diagonal for the
/// force-scaled geometric constraint Hessian.
inline double addAvbdSelfContactNormal(
    VertexBlock& block,
    std::span<const Eigen::Vector3d> positions,
    const AvbdSelfContactNormalRow& row,
    std::uint8_t localVertex,
    double alpha)
{
  const AvbdSelfContactPrimitiveResult primitive
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
  const Eigen::Matrix3d geometricStiffness
      = forceMagnitude
        * primitive.constraintHessian.block<3, 3>(
            3 * static_cast<int>(localVertex),
            3 * static_cast<int>(localVertex));
  block.hessian.diagonal()
      += avbdQuasiNewtonGeometricDiagonal(geometricStiffness);
  return forceMagnitude;
}

//==============================================================================
inline AvbdScalarRowState updateAvbdSelfContactNormalRow(
    AvbdScalarRowState state,
    std::span<const Eigen::Vector3d> positions,
    const AvbdSelfContactNormalRow& row,
    const AvbdSelfContactNormalOptions& options)
{
  const AvbdSelfContactPrimitiveResult primitive
      = evaluateAvbdSelfContactPrimitive(row, positions);
  double currentConstraintValue = 0.0;
  if (primitive.active) {
    currentConstraintValue = avbdSelfContactNormalConstraintValue(primitive);
  } else {
    // Primal barrier derivatives remain an inactive no-op outside d_hat, but
    // the fixed candidate row still has a well-defined separating scalar gap.
    // Apply Eq. 13 to that negative C so a positive normal dual decays/clamps
    // instead of surviving separation and being re-injected on re-entry.
    // Under-floor primitives deliberately remain fail-closed: their clamped
    // scalar has no matching exact differential, so preserve the row state.
    if (primitive.clampedToSafetyFloor
        || !(primitive.squaredActivationDistance > 0.0)
        || !std::isfinite(primitive.squaredActivationDistance)
        || !std::isfinite(primitive.squaredDistance)
        || primitive.squaredDistance < primitive.squaredActivationDistance) {
      return state;
    }
    currentConstraintValue = std::sqrt(primitive.squaredActivationDistance)
                             - std::sqrt(primitive.squaredDistance);
  }

  const double constraintValue = regularizeAvbdConstraintValue(
      currentConstraintValue, row.previousConstraintValue, options.alpha);
  return updateAvbdHardConstraintRow(
      state, constraintValue, options.beta, row.bounds, options.maxStiffness);
}

//==============================================================================
inline double avbdSelfContactNormalTrialForce(
    const AvbdSelfContactNormalRow& row,
    std::span<const Eigen::Vector3d> positions,
    double alpha)
{
  const AvbdSelfContactPrimitiveResult primitive
      = evaluateAvbdSelfContactPrimitive(row, positions);
  if (!primitive.active) {
    return 0.0;
  }

  const double constraintValue = regularizeAvbdConstraintValue(
      avbdSelfContactNormalConstraintValue(primitive),
      row.previousConstraintValue,
      alpha);
  return computeAvbdHardConstraintForce(row.state, constraintValue, row.bounds);
}

//==============================================================================
/// Stamp one active AVBD self-contact friction tangent row into a VBD vertex
/// block. The row constrains persistent accumulated tangent motion plus the
/// current lagged-stencil displacement, so sticking does not reset and creep
/// at every timestep. A zero live cone is an inactive row and contributes
/// neither force nor Hessian.
inline double addAvbdSelfContactFrictionTangent(
    VertexBlock& block,
    std::span<const Eigen::Vector3d> positions,
    const AvbdSelfContactFrictionRow& row,
    std::uint8_t localVertex,
    double alpha)
{
  if (row.differentialSuspended) {
    return 0.0;
  }
  if (!(avbdSelfContactFrictionForceLimit(row) > 0.0)) {
    return 0.0;
  }
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
/// Coulomb-cone pair. The augmented-Lagrangian trial is projected radially only
/// when it exceeds the live circular Coulomb bound; equality is accepted. A
/// zero live cone is an inactive row and contributes neither force nor Hessian.
inline Eigen::Vector2d addAvbdSelfContactFrictionTangentPair(
    VertexBlock& block,
    std::span<const Eigen::Vector3d> positions,
    const AvbdSelfContactFrictionRow& first,
    const AvbdSelfContactFrictionRow& second,
    std::uint8_t localVertex,
    const AvbdSelfContactFrictionOptions& options)
{
  if (first.differentialSuspended || second.differentialSuspended) {
    return Eigen::Vector2d::Zero();
  }
  if (!(avbdSelfContactFrictionPairForceLimit(first, second) > 0.0)) {
    return Eigen::Vector2d::Zero();
  }
  bool valid = false;
  const Eigen::Vector2d force = avbdSelfContactFrictionTangentPairForce(
      first, second, positions, options, nullptr, &valid);
  if (!valid) {
    return Eigen::Vector2d::Zero();
  }
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
  if (row.differentialSuspended) {
    return state;
  }
  if (!(avbdSelfContactFrictionForceLimit(row) > 0.0)) {
    state.lambda = 0.0;
    return state;
  }
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
  if (first.differentialSuspended || second.differentialSuspended) {
    return;
  }
  if (!(avbdSelfContactFrictionPairForceLimit(first, second) > 0.0)) {
    first.state.lambda = 0.0;
    second.state.lambda = 0.0;
    first.sticking = false;
    second.sticking = false;
    return;
  }
  bool clamped = false;
  bool valid = false;
  const Eigen::Vector2d force = avbdSelfContactFrictionTangentPairForce(
      first, second, positions, options, &clamped, &valid);
  if (!valid) {
    first.state.lambda = 0.0;
    second.state.lambda = 0.0;
    first.sticking = false;
    second.sticking = false;
    first.accumulatedTangentialDisplacement.setZero();
    second.accumulatedTangentialDisplacement.setZero();
    return;
  }
  const Eigen::Vector2d constraintValues
      = avbdSelfContactFrictionConstraintValues(
          first, second, positions, options.alpha);

  first.state.lambda = force.x();
  second.state.lambda = force.y();
  const bool sticking = !clamped
                        && constraintValues.norm()
                               < std::max(0.0, options.staticFrictionTolerance);
  first.sticking = sticking;
  second.sticking = sticking;
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
  if (!selfContact.active() || vertex >= selfContact.vertexCount()) {
    return;
  }
  for (const SelfContactEntry& entry : selfContact.entriesFor(vertex)) {
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
