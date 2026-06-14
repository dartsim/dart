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

#include <dart/simulation/detail/deformable_elasticity/fem_tet_element.hpp>
#include <dart/simulation/detail/deformable_vbd/acceleration.hpp>
#include <dart/simulation/detail/deformable_vbd/attachment_kernel.hpp>
#include <dart/simulation/detail/deformable_vbd/contact_kernel.hpp>
#include <dart/simulation/detail/deformable_vbd/finite_stiffness_kernel.hpp>
#include <dart/simulation/detail/deformable_vbd/neo_hookean.hpp>
#include <dart/simulation/detail/deformable_vbd/self_contact.hpp>
#include <dart/simulation/detail/deformable_vbd/vertex_block_kernel.hpp>
#include <dart/simulation/detail/deformable_vbd/vertex_coloring.hpp>

#include <dart/common/stl_allocator.hpp>

#include <Eigen/Core>

#include <algorithm>
#include <array>
#include <span>
#include <utility>
#include <vector>

#include <cstddef>
#include <cstdint>

namespace dart::simulation::detail::deformable_vbd {

/// Shorthand for the shared FEM tetrahedral elasticity kernels that VBD can
/// optionally route through (see BlockDescentOptions::useFemTetKernel).
namespace elasticity = ::dart::simulation::detail::deformable_elasticity;

/// One distance-spring element for the mass-spring block-descent driver.
struct SpringElement
{
  std::uint32_t a = 0;
  std::uint32_t b = 0;
  double restLength = 0.0;
};

/// Per-vertex incident-spring lists, built once from a spring list and reused
/// across solves. `incidentSprings[v]` holds the indices into the spring array
/// of every spring touching vertex `v`.
struct SpringAdjacency
{
  using IncidentSpringVector
      = std::vector<std::uint32_t, ::dart::common::StlAllocator<std::uint32_t>>;
  using IncidentSpringVectorAllocator
      = ::dart::common::StlAllocator<IncidentSpringVector>;
  using IncidentSpringRows
      = std::vector<IncidentSpringVector, IncidentSpringVectorAllocator>;

  SpringAdjacency()
    : SpringAdjacency(::dart::common::MemoryAllocator::GetDefault())
  {
    // Intentionally empty.
  }

  explicit SpringAdjacency(::dart::common::MemoryAllocator& allocator)
    : incidentSprings(IncidentSpringVectorAllocator{allocator}),
      m_allocator(&allocator)
  {
    // Intentionally empty.
  }

  IncidentSpringRows incidentSprings;

  static SpringAdjacency build(
      std::size_t vertexCount,
      std::span<const SpringElement> springs,
      ::dart::common::MemoryAllocator& allocator
      = ::dart::common::MemoryAllocator::GetDefault())
  {
    SpringAdjacency adjacency(allocator);
    adjacency.resize(vertexCount);
    for (std::uint32_t s = 0; s < springs.size(); ++s) {
      const SpringElement& spring = springs[s];
      if (spring.a < vertexCount) {
        adjacency.incidentSprings[spring.a].push_back(s);
      }
      if (spring.b < vertexCount && spring.b != spring.a) {
        adjacency.incidentSprings[spring.b].push_back(s);
      }
    }
    return adjacency;
  }

private:
  void resize(std::size_t vertexCount)
  {
    if (vertexCount < incidentSprings.size()) {
      incidentSprings.resize(vertexCount);
      return;
    }
    incidentSprings.reserve(vertexCount);
    while (incidentSprings.size() < vertexCount) {
      incidentSprings.emplace_back(
          ::dart::common::StlAllocator<std::uint32_t>{*m_allocator});
    }
  }

  ::dart::common::MemoryAllocator* m_allocator;
};

/// Build the vertex-graph coloring induced by a spring list.
inline VertexColoring colorSprings(
    std::size_t vertexCount,
    std::span<const SpringElement> springs,
    ::dart::common::MemoryAllocator& allocator
    = ::dart::common::MemoryAllocator::GetDefault())
{
  VertexAdjacency adjacency(vertexCount, allocator);
  for (const SpringElement& spring : springs) {
    adjacency.addEdge(spring.a, spring.b);
  }
  return greedyColorVertices(adjacency);
}

/// Tuning for the block-descent sweep.
struct BlockDescentOptions
{
  std::size_t iterations = 20;    ///< Max VBD sweeps over all colors per call.
  double regularization = 0.0;    ///< Diagonal damping added to each block.
  bool clampSpringHessian = true; ///< PSD-project the spring Hessian blocks.
  /// Stop early once the largest per-vertex update in a sweep falls below this
  /// length (0 disables early termination and runs the full sweep budget).
  double convergenceDisplacement = 0.0;
  /// Chebyshev semi-iterative over-relaxation across sweeps (honored by
  /// blockDescentDeformable). Accelerates convergence without changing the
  /// fixed point when `chebyshevRho` matches the actual Gauss-Seidel spectral
  /// radius.
  bool useChebyshev = false;
  /// Estimated spectral radius in (0, 1) for the Chebyshev recurrence. It
  /// should approximate the body's actual Gauss-Seidel convergence rate:
  /// setting it well above the true rate over-relaxes and can diverge within a
  /// step, so it is most useful for stiff, slowly-converging bodies and should
  /// be left conservative (or Chebyshev disabled) otherwise.
  double chebyshevRho = 0.95;
  /// Stiffness-proportional (Rayleigh) damping coefficient k_d, honored by
  /// blockDescentDeformable when step-start positions are supplied (0 = off).
  /// Adds (k_d/h) H_elastic to each block and opposes the per-step
  /// displacement.
  double rayleighDamping = 0.0;
  /// Route tetrahedral elasticity through the shared `deformable_elasticity`
  /// FEM kernels (so a VBD body honors the same hyperelastic material the
  /// default solver would apply) instead of the VBD-local Stable Neo-Hookean
  /// kernel. Off by default to keep the standalone drivers byte-identical; the
  /// World VBD path turns it on.
  bool useFemTetKernel = false;
  /// When `useFemTetKernel` is set, evaluate tetrahedra with the
  /// fixed-corotational FEM material (honoring
  /// `DeformableMaterial::useFixedCorotationalElasticity`) rather than Stable
  /// Neo-Hookean. Ignored when `useFemTetKernel` is false.
  bool useFixedCorotationalTets = false;
};

/// Outcome of a block-descent solve.
struct BlockDescentStats
{
  std::size_t iterations = 0;
  std::size_t vertexUpdates = 0;
  /// Sum of squared per-vertex residual force over free vertices after the last
  /// sweep. Approaches zero as the solve converges to the objective's
  /// minimizer.
  double finalResidualNormSquared = 0.0;
};

namespace detail {

//==============================================================================
/// Assemble the inertia + incident-spring block for one free vertex at its
/// current position.
inline VertexBlock assembleVertexBlock(
    std::uint32_t vertex,
    const std::vector<Eigen::Vector3d>& positions,
    std::span<const double> masses,
    const std::vector<Eigen::Vector3d>& inertialTargets,
    std::span<const SpringElement> springs,
    const SpringAdjacency& adjacency,
    double springStiffness,
    double timeStep,
    bool clampSpringHessian)
{
  VertexBlock block;
  addInertiaTerm(
      block,
      masses[vertex],
      timeStep,
      positions[vertex],
      inertialTargets[vertex]);
  for (const std::uint32_t springIndex : adjacency.incidentSprings[vertex]) {
    const SpringElement& spring = springs[springIndex];
    const std::uint32_t other = (spring.a == vertex) ? spring.b : spring.a;
    addSpringTerm(
        block,
        springStiffness,
        spring.restLength,
        positions[vertex],
        positions[other],
        clampSpringHessian);
  }
  return block;
}

} // namespace detail

//==============================================================================
/// Run graph-colored Gauss-Seidel block coordinate descent on a single
/// mass-spring body, minimizing the variational implicit-Euler objective
///   G(x) = sum_i (m_i / (2 h^2)) ||x_i - y_i||^2 + sum_e (k/2)(l_e - L_e)^2.
///
/// Colors are swept sequentially; within a color, the vertices share no spring,
/// so updating them one at a time is numerically identical to the parallel
/// Jacobi update VBD specifies (a same-color vertex never appears in another
/// same-color vertex's block). Fixed vertices are held as Dirichlet
/// constraints.
///
/// `positions` is updated in place. `coloring` and `adjacency` must be built
/// for the same `springs`/vertex count (see colorSprings /
/// SpringAdjacency::build).
inline BlockDescentStats blockDescentMassSpring(
    std::vector<Eigen::Vector3d>& positions,
    std::span<const double> masses,
    const std::vector<std::uint8_t>& fixed,
    const std::vector<Eigen::Vector3d>& inertialTargets,
    std::span<const SpringElement> springs,
    double springStiffness,
    double timeStep,
    const VertexColoring& coloring,
    const SpringAdjacency& adjacency,
    const BlockDescentOptions& options)
{
  BlockDescentStats stats;
  const std::size_t vertexCount = positions.size();

  const double convergenceSquared
      = options.convergenceDisplacement * options.convergenceDisplacement;
  for (std::size_t iteration = 0; iteration < options.iterations; ++iteration) {
    ++stats.iterations;
    double maxDeltaSquared = 0.0;
    for (const auto& group : coloring.groups) {
      for (const std::uint32_t vertex : group) {
        if (vertex >= vertexCount || fixed[vertex] != 0u) {
          continue;
        }
        const VertexBlock block = detail::assembleVertexBlock(
            vertex,
            positions,
            masses,
            inertialTargets,
            springs,
            adjacency,
            springStiffness,
            timeStep,
            options.clampSpringHessian);
        const Eigen::Vector3d delta
            = solveVertexBlock(block, options.regularization);
        positions[vertex] += delta;
        maxDeltaSquared = std::max(maxDeltaSquared, delta.squaredNorm());
        ++stats.vertexUpdates;
      }
    }
    if (convergenceSquared > 0.0 && maxDeltaSquared <= convergenceSquared) {
      break;
    }
  }

  // Residual: recompute the full per-vertex force at the final positions.
  double residualNormSquared = 0.0;
  for (std::uint32_t vertex = 0; vertex < vertexCount; ++vertex) {
    if (fixed[vertex] != 0u) {
      continue;
    }
    const VertexBlock block = detail::assembleVertexBlock(
        vertex,
        positions,
        masses,
        inertialTargets,
        springs,
        adjacency,
        springStiffness,
        timeStep,
        options.clampSpringHessian);
    residualNormSquared += block.force.squaredNorm();
  }
  stats.finalResidualNormSquared = residualNormSquared;
  return stats;
}

//==============================================================================
/// Evaluate the variational implicit-Euler objective G(x) for a mass-spring
/// body. Provided so tests and benchmarks can verify monotone energy decrease
/// and convergence without depending on the solver-internal objective.
inline double massSpringObjective(
    const std::vector<Eigen::Vector3d>& positions,
    std::span<const double> masses,
    const std::vector<std::uint8_t>& fixed,
    const std::vector<Eigen::Vector3d>& inertialTargets,
    std::span<const SpringElement> springs,
    double springStiffness,
    double timeStep)
{
  double energy = 0.0;
  const double invDt2 = 1.0 / (timeStep * timeStep);
  for (std::size_t i = 0; i < positions.size(); ++i) {
    if (fixed[i] != 0u) {
      continue;
    }
    const Eigen::Vector3d delta = positions[i] - inertialTargets[i];
    energy += 0.5 * masses[i] * invDt2 * delta.squaredNorm();
  }
  for (const SpringElement& spring : springs) {
    const double length = (positions[spring.b] - positions[spring.a]).norm();
    const double stretch = length - spring.restLength;
    energy += 0.5 * springStiffness * stretch * stretch;
  }
  return energy;
}

//==============================================================================
/// Find the finite-stiffness row for `springIndex`. World-generated rows are
/// stored in spring order, but the fallback scan keeps the standalone driver
/// robust for tests that pass sparse or reordered row arrays.
template <typename SpringRows>
inline const AvbdSpringFiniteStiffnessRow* findAvbdSpringFiniteStiffnessRow(
    const SpringRows& rows, std::uint32_t springIndex)
{
  if (springIndex < rows.size() && rows[springIndex].spring == springIndex) {
    return &rows[springIndex];
  }

  const auto it
      = std::find_if(rows.begin(), rows.end(), [springIndex](const auto& row) {
          return row.spring == springIndex;
        });
  return it == rows.end() ? nullptr : &(*it);
}

//==============================================================================
/// Find the finite-stiffness material row for `tetIndex`. Rows generated from a
/// tetrahedral mesh are normally stored in tet order, but tests and future row
/// generation can pass sparse or reordered arrays.
template <typename TetRows>
inline const AvbdTetMaterialFiniteStiffnessRow*
findAvbdTetMaterialFiniteStiffnessRow(
    const TetRows& rows, std::uint32_t tetIndex)
{
  if (tetIndex < rows.size() && rows[tetIndex].tet == tetIndex) {
    return &rows[tetIndex];
  }

  const auto it
      = std::find_if(rows.begin(), rows.end(), [tetIndex](const auto& row) {
          return row.tet == tetIndex;
        });
  return it == rows.end() ? nullptr : &(*it);
}

//==============================================================================
/// Mass-spring block descent with AVBD finite-stiffness spring rows. Each
/// spring uses its row's current effective stiffness during the primal sweep,
/// then the row stiffness is increased toward the material stiffness after the
/// sweep based on the observed spring constraint error. These rows
/// intentionally carry no dual value.
template <typename SpringRows = std::vector<AvbdSpringFiniteStiffnessRow>>
inline BlockDescentStats blockDescentMassSpringAvbdFiniteStiffness(
    std::vector<Eigen::Vector3d>& positions,
    std::span<const double> masses,
    const std::vector<std::uint8_t>& fixed,
    const std::vector<Eigen::Vector3d>& inertialTargets,
    std::span<const SpringElement> springs,
    double fallbackSpringStiffness,
    double timeStep,
    SpringRows& springRows,
    const VertexColoring& coloring,
    const SpringAdjacency& adjacency,
    const BlockDescentOptions& options,
    const AvbdSpringFiniteStiffnessOptions& avbdOptions)
{
  BlockDescentStats stats;
  const std::size_t vertexCount = positions.size();
  const auto assemble = [&](std::uint32_t vertex) {
    VertexBlock block;
    addInertiaTerm(
        block,
        masses[vertex],
        timeStep,
        positions[vertex],
        inertialTargets[vertex]);
    for (const std::uint32_t springIndex : adjacency.incidentSprings[vertex]) {
      const SpringElement& spring = springs[springIndex];
      const std::uint32_t other = (spring.a == vertex) ? spring.b : spring.a;
      const AvbdSpringFiniteStiffnessRow* row
          = findAvbdSpringFiniteStiffnessRow(springRows, springIndex);
      if (row != nullptr) {
        addAvbdSpringFiniteStiffness(
            block,
            positions[vertex],
            positions[other],
            spring.restLength,
            *row,
            options.clampSpringHessian);
      } else {
        addSpringTerm(
            block,
            fallbackSpringStiffness,
            spring.restLength,
            positions[vertex],
            positions[other],
            options.clampSpringHessian);
      }
    }
    return block;
  };

  const double convergenceSquared
      = options.convergenceDisplacement * options.convergenceDisplacement;
  for (std::size_t iteration = 0; iteration < options.iterations; ++iteration) {
    ++stats.iterations;
    double maxDeltaSquared = 0.0;
    for (const auto& group : coloring.groups) {
      for (const std::uint32_t vertex : group) {
        if (vertex >= vertexCount || fixed[vertex] != 0u) {
          continue;
        }
        const VertexBlock block = assemble(vertex);
        const Eigen::Vector3d delta
            = solveVertexBlock(block, options.regularization);
        positions[vertex] += delta;
        maxDeltaSquared = std::max(maxDeltaSquared, delta.squaredNorm());
        ++stats.vertexUpdates;
      }
    }

    for (AvbdSpringFiniteStiffnessRow& row : springRows) {
      if (row.spring >= springs.size()) {
        continue;
      }
      const SpringElement& spring = springs[row.spring];
      if (spring.a >= vertexCount || spring.b >= vertexCount) {
        continue;
      }
      if (fixed[spring.a] != 0u && fixed[spring.b] != 0u) {
        continue;
      }
      const double constraintValue = avbdSpringConstraintValue(
          positions[spring.a], positions[spring.b], spring.restLength);
      row.state = updateAvbdSpringFiniteStiffnessRow(
          row.state, constraintValue, row, avbdOptions);
    }
    if (convergenceSquared > 0.0 && maxDeltaSquared <= convergenceSquared) {
      break;
    }
  }

  double residualNormSquared = 0.0;
  for (std::uint32_t vertex = 0; vertex < vertexCount; ++vertex) {
    if (fixed[vertex] != 0u) {
      continue;
    }
    residualNormSquared += assemble(vertex).force.squaredNorm();
  }
  stats.finalResidualNormSquared = residualNormSquared;
  return stats;
}

//==============================================================================
/// Mass-spring block descent with the currently wired AVBD deformable row
/// families combined in one serial solve. This keeps the first World-level AVBD
/// integration honest: contact-normal hard rows, scalar hard attachments, and
/// finite-stiffness spring rows all contribute to the same primal vertex block,
/// then each row family updates its persistent dual/stiffness state after every
/// sweep. The optional friction-tangent rows provide the first bounded row
/// family in the same serial driver. Optional self-contact normal rows share
/// one scalar row per point-triangle / edge-edge primitive and stamp each
/// incident local vertex through the lagged self-contact adjacency. Friction
/// tangent rows generated as adjacent pairs use the lagged tangential dual to
/// switch between static and dynamic Coulomb modes. Full contact-manifold
/// friction persistence, broader self-contact friction envelopes, tetrahedral
/// row mixing, and parallel dual scheduling remain later slices.
template <
    typename PositionVector,
    typename FixedMask,
    typename ContactRows = std::vector<AvbdHalfSpaceContactRow>,
    typename AttachmentRows = std::vector<AvbdPointAttachmentRow>,
    typename SpringRows = std::vector<AvbdSpringFiniteStiffnessRow>,
    typename FrictionRows = std::vector<AvbdHalfSpaceFrictionRow>,
    typename SelfContactRows = std::vector<AvbdSelfContactNormalRow>,
    typename SelfContactFrictionRows = std::vector<AvbdSelfContactFrictionRow>>
inline BlockDescentStats blockDescentMassSpringAvbdRows(
    PositionVector& positions,
    std::span<const double> masses,
    const FixedMask& fixed,
    std::span<const Eigen::Vector3d> inertialTargets,
    std::span<const SpringElement> springs,
    double fallbackSpringStiffness,
    double timeStep,
    ContactRows& contactRows,
    AttachmentRows& attachmentRows,
    SpringRows& springRows,
    const VertexColoring& coloring,
    const SpringAdjacency& adjacency,
    const BlockDescentOptions& options,
    const AvbdHalfSpaceContactOptions& contactOptions,
    const AvbdPointAttachmentOptions& attachmentOptions,
    const AvbdSpringFiniteStiffnessOptions& springOptions,
    FrictionRows* frictionRows = nullptr,
    const AvbdHalfSpaceFrictionOptions* frictionOptions = nullptr,
    SelfContactRows* selfContactRows = nullptr,
    const SelfContactAdjacency* selfContact = nullptr,
    const AvbdSelfContactNormalOptions* selfContactOptions = nullptr,
    SelfContactFrictionRows* selfContactFrictionRows = nullptr,
    const AvbdSelfContactFrictionOptions* selfContactFrictionOptions = nullptr)
{
  BlockDescentStats stats;
  const std::size_t vertexCount = positions.size();
  const auto hasFreeSelfContactFrictionVertex
      = [&](const AvbdSelfContactFrictionRow& row) {
          for (const std::uint32_t node : row.nodes) {
            if (node < vertexCount && fixed[node] == 0u) {
              return true;
            }
          }
          return false;
        };
  const auto assemble = [&](std::uint32_t vertex) {
    VertexBlock block;
    addInertiaTerm(
        block,
        masses[vertex],
        timeStep,
        positions[vertex],
        inertialTargets[vertex]);
    for (const std::uint32_t springIndex : adjacency.incidentSprings[vertex]) {
      const SpringElement& spring = springs[springIndex];
      const std::uint32_t other = (spring.a == vertex) ? spring.b : spring.a;
      const AvbdSpringFiniteStiffnessRow* row
          = findAvbdSpringFiniteStiffnessRow(springRows, springIndex);
      if (row != nullptr) {
        addAvbdSpringFiniteStiffness(
            block,
            positions[vertex],
            positions[other],
            spring.restLength,
            *row,
            options.clampSpringHessian);
      } else {
        addSpringTerm(
            block,
            fallbackSpringStiffness,
            spring.restLength,
            positions[vertex],
            positions[other],
            options.clampSpringHessian);
      }
    }
    if (selfContactRows != nullptr && selfContact != nullptr
        && selfContactOptions != nullptr
        && vertex < selfContact->incident.size()) {
      for (const SelfContactEntry& entry : selfContact->incident[vertex]) {
        if (entry.constraint >= selfContactRows->size()) {
          continue;
        }
        addAvbdSelfContactNormal(
            block,
            positions,
            (*selfContactRows)[entry.constraint],
            entry.localVertex,
            selfContactOptions->alpha);
      }
    }
    if (selfContactFrictionRows != nullptr
        && selfContactFrictionOptions != nullptr) {
      for (std::size_t i = 0; i < selfContactFrictionRows->size();) {
        const AvbdSelfContactFrictionRow& row = (*selfContactFrictionRows)[i];
        const std::uint8_t localVertex
            = avbdSelfContactLocalVertex(row, vertex);
        if (localVertex < 4u) {
          if (i + 1 < selfContactFrictionRows->size()
              && avbdSelfContactSameFrictionPrimitive(
                  row, (*selfContactFrictionRows)[i + 1])) {
            addAvbdSelfContactFrictionTangentPair(
                block,
                positions,
                row,
                (*selfContactFrictionRows)[i + 1],
                localVertex,
                *selfContactFrictionOptions);
            i += 2;
            continue;
          }
          addAvbdSelfContactFrictionTangent(
              block,
              positions,
              row,
              localVertex,
              selfContactFrictionOptions->alpha);
        }
        ++i;
      }
    }
    for (const AvbdHalfSpaceContactRow& row : contactRows) {
      if (row.vertex == vertex) {
        addAvbdHalfSpaceContactNormal(
            block,
            positions[vertex],
            row.plane,
            row.state,
            row.previousConstraintValue,
            contactOptions.alpha,
            row.bounds);
      }
    }
    for (const AvbdPointAttachmentRow& row : attachmentRows) {
      if (row.vertex == vertex) {
        addAvbdPointAttachment(
            block, positions[vertex], row, attachmentOptions.alpha);
      }
    }
    if (frictionRows != nullptr && frictionOptions != nullptr) {
      for (std::size_t i = 0; i < frictionRows->size();) {
        const AvbdHalfSpaceFrictionRow& row = (*frictionRows)[i];
        if (row.vertex == vertex) {
          if (i + 1 < frictionRows->size()
              && (*frictionRows)[i + 1].vertex == vertex) {
            addAvbdHalfSpaceFrictionTangentPair(
                block,
                positions[vertex],
                row,
                (*frictionRows)[i + 1],
                *frictionOptions);
            i += 2;
            continue;
          }
          addAvbdHalfSpaceFrictionTangent(
              block, positions[vertex], row, frictionOptions->alpha);
        }
        ++i;
      }
    }
    return block;
  };

  const double convergenceSquared
      = options.convergenceDisplacement * options.convergenceDisplacement;
  for (std::size_t iteration = 0; iteration < options.iterations; ++iteration) {
    ++stats.iterations;
    double maxDeltaSquared = 0.0;
    for (const auto& group : coloring.groups) {
      for (const std::uint32_t vertex : group) {
        if (vertex >= vertexCount || fixed[vertex] != 0u) {
          continue;
        }
        const VertexBlock block = assemble(vertex);
        const Eigen::Vector3d delta
            = solveVertexBlock(block, options.regularization);
        positions[vertex] += delta;
        maxDeltaSquared = std::max(maxDeltaSquared, delta.squaredNorm());
        ++stats.vertexUpdates;
      }
    }

    for (AvbdHalfSpaceContactRow& row : contactRows) {
      if (row.vertex >= vertexCount || fixed[row.vertex] != 0u) {
        continue;
      }
      row.state = updateAvbdHalfSpaceContactNormalRow(
          row.state,
          positions[row.vertex],
          row.plane,
          contactOptions,
          row.previousConstraintValue,
          row.bounds);
    }
    for (AvbdPointAttachmentRow& row : attachmentRows) {
      if (row.vertex >= vertexCount || fixed[row.vertex] != 0u) {
        continue;
      }
      row.state = updateAvbdPointAttachmentRow(
          row.state, positions[row.vertex], row, attachmentOptions);
    }
    for (AvbdSpringFiniteStiffnessRow& row : springRows) {
      if (row.spring >= springs.size()) {
        continue;
      }
      const SpringElement& spring = springs[row.spring];
      if (spring.a >= vertexCount || spring.b >= vertexCount) {
        continue;
      }
      if (fixed[spring.a] != 0u && fixed[spring.b] != 0u) {
        continue;
      }
      const double constraintValue = avbdSpringConstraintValue(
          positions[spring.a], positions[spring.b], spring.restLength);
      row.state = updateAvbdSpringFiniteStiffnessRow(
          row.state, constraintValue, row, springOptions);
    }
    if (frictionRows != nullptr && frictionOptions != nullptr) {
      for (std::size_t i = 0; i < frictionRows->size();) {
        AvbdHalfSpaceFrictionRow& row = (*frictionRows)[i];
        if (row.vertex >= vertexCount || fixed[row.vertex] != 0u) {
          ++i;
          continue;
        }
        if (i + 1 < frictionRows->size()
            && (*frictionRows)[i + 1].vertex == row.vertex
            && (*frictionRows)[i + 1].vertex < vertexCount
            && fixed[(*frictionRows)[i + 1].vertex] == 0u) {
          updateAvbdHalfSpaceFrictionTangentPair(
              row,
              (*frictionRows)[i + 1],
              positions[row.vertex],
              *frictionOptions);
          i += 2;
          continue;
        }
        row.state = updateAvbdHalfSpaceFrictionTangentRow(
            row.state, positions[row.vertex], row, *frictionOptions);
        ++i;
      }
    }
    if (selfContactRows != nullptr && selfContactOptions != nullptr) {
      for (AvbdSelfContactNormalRow& row : *selfContactRows) {
        bool hasFreeVertex = false;
        for (const std::uint32_t node : row.nodes) {
          if (node < vertexCount && fixed[node] == 0u) {
            hasFreeVertex = true;
            break;
          }
        }
        if (!hasFreeVertex) {
          continue;
        }
        row.state = updateAvbdSelfContactNormalRow(
            row.state, positions, row, *selfContactOptions);
      }
    }
    if (selfContactFrictionRows != nullptr
        && selfContactFrictionOptions != nullptr) {
      for (std::size_t i = 0; i < selfContactFrictionRows->size();) {
        AvbdSelfContactFrictionRow& row = (*selfContactFrictionRows)[i];
        if (!hasFreeSelfContactFrictionVertex(row)) {
          ++i;
          continue;
        }
        if (i + 1 < selfContactFrictionRows->size()
            && avbdSelfContactSameFrictionPrimitive(
                row, (*selfContactFrictionRows)[i + 1])
            && hasFreeSelfContactFrictionVertex(
                (*selfContactFrictionRows)[i + 1])) {
          updateAvbdSelfContactFrictionTangentPair(
              row,
              (*selfContactFrictionRows)[i + 1],
              positions,
              *selfContactFrictionOptions);
          i += 2;
          continue;
        }
        row.state = updateAvbdSelfContactFrictionTangentRow(
            row.state, positions, row, *selfContactFrictionOptions);
        ++i;
      }
    }
    if (convergenceSquared > 0.0 && maxDeltaSquared <= convergenceSquared) {
      break;
    }
  }

  double residualNormSquared = 0.0;
  for (std::uint32_t vertex = 0; vertex < vertexCount; ++vertex) {
    if (fixed[vertex] != 0u) {
      continue;
    }
    residualNormSquared += assemble(vertex).force.squaredNorm();
  }
  stats.finalResidualNormSquared = residualNormSquared;
  return stats;
}

//==============================================================================
/// One tetrahedral element for the Neo-Hookean block-descent driver, with its
/// rest shape precomputed once via makeTetRestShape.
struct TetMeshElement
{
  std::array<std::uint32_t, 4> vertices{0, 0, 0, 0};
  TetRestShape rest;
};

/// Per-vertex incident-tetrahedron lists. `incidentTets[v]` holds, for each
/// tetrahedron touching vertex `v`, the tet index and the vertex's local index
/// (0..3) within that tet.
struct TetAdjacency
{
  using IncidentTet = std::pair<std::uint32_t, std::uint8_t>;
  using IncidentTetVector
      = std::vector<IncidentTet, ::dart::common::StlAllocator<IncidentTet>>;
  using IncidentTetVectorAllocator
      = ::dart::common::StlAllocator<IncidentTetVector>;
  using IncidentTetRows
      = std::vector<IncidentTetVector, IncidentTetVectorAllocator>;

  TetAdjacency() : TetAdjacency(::dart::common::MemoryAllocator::GetDefault())
  {
    // Intentionally empty.
  }

  explicit TetAdjacency(::dart::common::MemoryAllocator& allocator)
    : incidentTets(IncidentTetVectorAllocator{allocator}),
      m_allocator(&allocator)
  {
    // Intentionally empty.
  }

  IncidentTetRows incidentTets;

  static TetAdjacency build(
      std::size_t vertexCount,
      std::span<const TetMeshElement> tets,
      ::dart::common::MemoryAllocator& allocator
      = ::dart::common::MemoryAllocator::GetDefault())
  {
    TetAdjacency adjacency(allocator);
    adjacency.resize(vertexCount);
    for (std::uint32_t t = 0; t < tets.size(); ++t) {
      for (std::uint8_t local = 0; local < 4; ++local) {
        const std::uint32_t vertex = tets[t].vertices[local];
        if (vertex < vertexCount) {
          adjacency.incidentTets[vertex].emplace_back(t, local);
        }
      }
    }
    return adjacency;
  }

private:
  void resize(std::size_t vertexCount)
  {
    if (vertexCount < incidentTets.size()) {
      incidentTets.resize(vertexCount);
      return;
    }
    incidentTets.reserve(vertexCount);
    while (incidentTets.size() < vertexCount) {
      incidentTets.emplace_back(
          ::dart::common::StlAllocator<IncidentTet>{*m_allocator});
    }
  }

  ::dart::common::MemoryAllocator* m_allocator;
};

/// Build the vertex-graph coloring induced by a tetrahedral mesh (each tet is a
/// 4-vertex clique in the vertex graph).
inline VertexColoring colorTetMesh(
    std::size_t vertexCount,
    std::span<const TetMeshElement> tets,
    ::dart::common::MemoryAllocator& allocator
    = ::dart::common::MemoryAllocator::GetDefault())
{
  VertexAdjacency adjacency(vertexCount, allocator);
  for (const TetMeshElement& tet : tets) {
    adjacency.addTetrahedron(
        tet.vertices[0], tet.vertices[1], tet.vertices[2], tet.vertices[3]);
  }
  return greedyColorVertices(adjacency);
}

namespace detail {

//==============================================================================
/// Per-vertex tetrahedral elasticity term routed through the shared FEM kernels
/// (Stable Neo-Hookean or fixed-corotational). Extracts the local vertex's 3x1
/// force (the negated gradient sub-block) and 3x3 diagonal Hessian block from
/// the element's 12x1 gradient / 12x12 Hessian, so a VBD body reproduces the
/// same hyperelastic material the default solver applies. Degenerate
/// (non-positive rest-volume) tetrahedra contribute nothing.
inline void addFemTetTerm(
    VertexBlock& block,
    std::uint8_t localVertex,
    const TetRestShape& rest,
    const std::array<Eigen::Vector3d, 4>& positions,
    double mu,
    double lambda,
    bool useFixedCorotational)
{
  if (!(rest.restVolume > 0.0)) {
    return;
  }
  elasticity::TetRestShape femRest;
  femRest.inverseRestEdges = rest.restShapeInverse;
  femRest.restVolume = rest.restVolume;
  femRest.valid = true;
  const elasticity::LameParameters lame{mu, lambda};
  const elasticity::TetElementResult result
      = useFixedCorotational ? elasticity::evaluateFixedCorotationalTet(
                                   positions[0],
                                   positions[1],
                                   positions[2],
                                   positions[3],
                                   femRest,
                                   lame)
                             : elasticity::evaluateStableNeoHookeanTet(
                                   positions[0],
                                   positions[1],
                                   positions[2],
                                   positions[3],
                                   femRest,
                                   lame);
  const int base = 3 * static_cast<int>(localVertex);
  block.force -= result.gradient.segment<3>(base);
  block.hessian += result.hessian.block<3, 3>(base, base);
}

//==============================================================================
/// Assemble the inertia + incident-tetrahedron Neo-Hookean block for one free
/// vertex at the current positions.
inline VertexBlock assembleTetVertexBlock(
    std::uint32_t vertex,
    const std::vector<Eigen::Vector3d>& positions,
    std::span<const double> masses,
    const std::vector<Eigen::Vector3d>& inertialTargets,
    std::span<const TetMeshElement> tets,
    const TetAdjacency& adjacency,
    double mu,
    double lambda,
    double timeStep,
    bool useFemTetKernel = false,
    bool useFixedCorotationalTets = false)
{
  VertexBlock block;
  addInertiaTerm(
      block,
      masses[vertex],
      timeStep,
      positions[vertex],
      inertialTargets[vertex]);
  for (const auto& [tetIndex, localVertex] : adjacency.incidentTets[vertex]) {
    const TetMeshElement& tet = tets[tetIndex];
    const std::array<Eigen::Vector3d, 4> tetPositions
        = {positions[tet.vertices[0]],
           positions[tet.vertices[1]],
           positions[tet.vertices[2]],
           positions[tet.vertices[3]]};
    if (useFemTetKernel) {
      addFemTetTerm(
          block,
          localVertex,
          tet.rest,
          tetPositions,
          mu,
          lambda,
          useFixedCorotationalTets);
    } else {
      addNeoHookeanTetTerm(
          block, localVertex, tet.rest, tetPositions, mu, lambda);
    }
  }
  return block;
}

} // namespace detail

//==============================================================================
/// Run graph-colored Gauss-Seidel block coordinate descent on a single
/// tetrahedral Stable Neo-Hookean body, minimizing
///   G(x) = sum_i (m_i / (2 h^2)) ||x_i - y_i||^2 + sum_t A_t Psi(F_t(x)).
///
/// Same-color vertices share no tetrahedron, so the serial within-color sweep
/// is numerically identical to the parallel Jacobi update. `positions` is
/// updated in place. `coloring` and `adjacency` must be built for `tets` (see
/// colorTetMesh / TetAdjacency::build).
inline BlockDescentStats blockDescentTetMesh(
    std::vector<Eigen::Vector3d>& positions,
    std::span<const double> masses,
    const std::vector<std::uint8_t>& fixed,
    const std::vector<Eigen::Vector3d>& inertialTargets,
    std::span<const TetMeshElement> tets,
    double mu,
    double lambda,
    double timeStep,
    const VertexColoring& coloring,
    const TetAdjacency& adjacency,
    const BlockDescentOptions& options)
{
  BlockDescentStats stats;
  const std::size_t vertexCount = positions.size();

  const double convergenceSquared
      = options.convergenceDisplacement * options.convergenceDisplacement;
  for (std::size_t iteration = 0; iteration < options.iterations; ++iteration) {
    ++stats.iterations;
    double maxDeltaSquared = 0.0;
    for (const auto& group : coloring.groups) {
      for (const std::uint32_t vertex : group) {
        if (vertex >= vertexCount || fixed[vertex] != 0u) {
          continue;
        }
        const VertexBlock block = detail::assembleTetVertexBlock(
            vertex,
            positions,
            masses,
            inertialTargets,
            tets,
            adjacency,
            mu,
            lambda,
            timeStep,
            options.useFemTetKernel,
            options.useFixedCorotationalTets);
        const Eigen::Vector3d delta
            = solveVertexBlock(block, options.regularization);
        positions[vertex] += delta;
        maxDeltaSquared = std::max(maxDeltaSquared, delta.squaredNorm());
        ++stats.vertexUpdates;
      }
    }
    if (convergenceSquared > 0.0 && maxDeltaSquared <= convergenceSquared) {
      break;
    }
  }

  double residualNormSquared = 0.0;
  for (std::uint32_t vertex = 0; vertex < vertexCount; ++vertex) {
    if (fixed[vertex] != 0u) {
      continue;
    }
    const VertexBlock block = detail::assembleTetVertexBlock(
        vertex,
        positions,
        masses,
        inertialTargets,
        tets,
        adjacency,
        mu,
        lambda,
        timeStep,
        options.useFemTetKernel,
        options.useFixedCorotationalTets);
    residualNormSquared += block.force.squaredNorm();
  }
  stats.finalResidualNormSquared = residualNormSquared;
  return stats;
}

//==============================================================================
/// Tetrahedral block descent with AVBD finite-stiffness material rows. Each
/// tet uses its row's current effective stiffness as a scale on the body Lamé
/// parameters during the primal sweep, then the scale ramps toward 1.0 from the
/// observed deformation-gradient error after each sweep. Optional AVBD
/// self-contact rows let pure-tet scenes share the same hard normal/friction
/// row path as the mass-spring envelope; when they are absent, the existing
/// lagged VBD self-contact penalty remains available.
template <
    typename PositionVector,
    typename FixedMask,
    typename TetRows = std::vector<AvbdTetMaterialFiniteStiffnessRow>,
    typename SelfContactRows = std::vector<AvbdSelfContactNormalRow>,
    typename SelfContactFrictionRows = std::vector<AvbdSelfContactFrictionRow>>
inline BlockDescentStats blockDescentTetMeshAvbdFiniteStiffness(
    PositionVector& positions,
    std::span<const double> masses,
    const FixedMask& fixed,
    std::span<const Eigen::Vector3d> inertialTargets,
    std::span<const TetMeshElement> tets,
    double mu,
    double lambda,
    double timeStep,
    TetRows& tetRows,
    const VertexColoring& coloring,
    const TetAdjacency& adjacency,
    const BlockDescentOptions& options,
    const AvbdTetMaterialFiniteStiffnessOptions& avbdOptions,
    const SelfContactAdjacency* selfContact = nullptr,
    SelfContactRows* selfContactRows = nullptr,
    const AvbdSelfContactNormalOptions* selfContactOptions = nullptr,
    SelfContactFrictionRows* selfContactFrictionRows = nullptr,
    const AvbdSelfContactFrictionOptions* selfContactFrictionOptions = nullptr)
{
  BlockDescentStats stats;
  const std::size_t vertexCount = positions.size();
  const auto hasFreeSelfContactFrictionVertex
      = [&](const AvbdSelfContactFrictionRow& row) {
          for (const std::uint32_t node : row.nodes) {
            if (node < vertexCount && fixed[node] == 0u) {
              return true;
            }
          }
          return false;
        };
  const auto assemble = [&](std::uint32_t vertex) {
    VertexBlock block;
    addInertiaTerm(
        block,
        masses[vertex],
        timeStep,
        positions[vertex],
        inertialTargets[vertex]);
    for (const auto& [tetIndex, localVertex] : adjacency.incidentTets[vertex]) {
      const TetMeshElement& tet = tets[tetIndex];
      const std::array<Eigen::Vector3d, 4> tetPositions
          = {positions[tet.vertices[0]],
             positions[tet.vertices[1]],
             positions[tet.vertices[2]],
             positions[tet.vertices[3]]};
      double effectiveMu = mu;
      double effectiveLambda = lambda;
      const AvbdTetMaterialFiniteStiffnessRow* row
          = findAvbdTetMaterialFiniteStiffnessRow(tetRows, tetIndex);
      if (row != nullptr) {
        const LameParameters scaled = avbdScaledTetMaterial(mu, lambda, *row);
        effectiveMu = scaled.mu;
        effectiveLambda = scaled.lambda;
      }
      if (!(effectiveMu > 0.0) || !(effectiveLambda > 0.0)) {
        continue;
      }
      if (options.useFemTetKernel) {
        detail::addFemTetTerm(
            block,
            localVertex,
            tet.rest,
            tetPositions,
            effectiveMu,
            effectiveLambda,
            options.useFixedCorotationalTets);
      } else {
        addNeoHookeanTetTerm(
            block,
            localVertex,
            tet.rest,
            tetPositions,
            effectiveMu,
            effectiveLambda);
      }
    }
    if (selfContactRows != nullptr && selfContact != nullptr
        && selfContactOptions != nullptr
        && vertex < selfContact->incident.size()) {
      for (const SelfContactEntry& entry : selfContact->incident[vertex]) {
        if (entry.constraint >= selfContactRows->size()) {
          continue;
        }
        addAvbdSelfContactNormal(
            block,
            positions,
            (*selfContactRows)[entry.constraint],
            entry.localVertex,
            selfContactOptions->alpha);
      }
    } else if (selfContact != nullptr) {
      addSelfContactTerms(block, vertex, *selfContact, positions);
    }
    if (selfContactFrictionRows != nullptr
        && selfContactFrictionOptions != nullptr) {
      for (std::size_t i = 0; i < selfContactFrictionRows->size();) {
        const AvbdSelfContactFrictionRow& row = (*selfContactFrictionRows)[i];
        const std::uint8_t localVertex
            = avbdSelfContactLocalVertex(row, vertex);
        if (localVertex < 4u) {
          if (i + 1 < selfContactFrictionRows->size()
              && avbdSelfContactSameFrictionPrimitive(
                  row, (*selfContactFrictionRows)[i + 1])) {
            addAvbdSelfContactFrictionTangentPair(
                block,
                positions,
                row,
                (*selfContactFrictionRows)[i + 1],
                localVertex,
                *selfContactFrictionOptions);
            i += 2;
            continue;
          }
          addAvbdSelfContactFrictionTangent(
              block,
              positions,
              row,
              localVertex,
              selfContactFrictionOptions->alpha);
        }
        ++i;
      }
    }
    return block;
  };

  const double convergenceSquared
      = options.convergenceDisplacement * options.convergenceDisplacement;
  for (std::size_t iteration = 0; iteration < options.iterations; ++iteration) {
    ++stats.iterations;
    double maxDeltaSquared = 0.0;
    for (const auto& group : coloring.groups) {
      for (const std::uint32_t vertex : group) {
        if (vertex >= vertexCount || fixed[vertex] != 0u) {
          continue;
        }
        const VertexBlock block = assemble(vertex);
        const Eigen::Vector3d delta
            = solveVertexBlock(block, options.regularization);
        positions[vertex] += delta;
        maxDeltaSquared = std::max(maxDeltaSquared, delta.squaredNorm());
        ++stats.vertexUpdates;
      }
    }

    for (AvbdTetMaterialFiniteStiffnessRow& row : tetRows) {
      if (row.tet >= tets.size()) {
        continue;
      }
      const TetMeshElement& tet = tets[row.tet];
      const std::array<std::uint32_t, 4>& vertices = tet.vertices;
      if (vertices[0] >= vertexCount || vertices[1] >= vertexCount
          || vertices[2] >= vertexCount || vertices[3] >= vertexCount) {
        continue;
      }
      if (fixed[vertices[0]] != 0u && fixed[vertices[1]] != 0u
          && fixed[vertices[2]] != 0u && fixed[vertices[3]] != 0u) {
        continue;
      }
      const std::array<Eigen::Vector3d, 4> tetPositions
          = {positions[vertices[0]],
             positions[vertices[1]],
             positions[vertices[2]],
             positions[vertices[3]]};
      const double constraintValue
          = avbdTetMaterialConstraintValue(tet.rest, tetPositions);
      row.state = updateAvbdTetMaterialFiniteStiffnessRow(
          row.state, constraintValue, row, avbdOptions);
    }
    if (selfContactRows != nullptr && selfContactOptions != nullptr) {
      for (AvbdSelfContactNormalRow& row : *selfContactRows) {
        bool hasFreeVertex = false;
        for (const std::uint32_t node : row.nodes) {
          if (node < vertexCount && fixed[node] == 0u) {
            hasFreeVertex = true;
            break;
          }
        }
        if (!hasFreeVertex) {
          continue;
        }
        row.state = updateAvbdSelfContactNormalRow(
            row.state, positions, row, *selfContactOptions);
      }
    }
    if (selfContactFrictionRows != nullptr
        && selfContactFrictionOptions != nullptr) {
      for (std::size_t i = 0; i < selfContactFrictionRows->size();) {
        AvbdSelfContactFrictionRow& row = (*selfContactFrictionRows)[i];
        if (!hasFreeSelfContactFrictionVertex(row)) {
          ++i;
          continue;
        }
        if (i + 1 < selfContactFrictionRows->size()
            && avbdSelfContactSameFrictionPrimitive(
                row, (*selfContactFrictionRows)[i + 1])
            && hasFreeSelfContactFrictionVertex(
                (*selfContactFrictionRows)[i + 1])) {
          updateAvbdSelfContactFrictionTangentPair(
              row,
              (*selfContactFrictionRows)[i + 1],
              positions,
              *selfContactFrictionOptions);
          i += 2;
          continue;
        }
        row.state = updateAvbdSelfContactFrictionTangentRow(
            row.state, positions, row, *selfContactFrictionOptions);
        ++i;
      }
    }
    if (convergenceSquared > 0.0 && maxDeltaSquared <= convergenceSquared) {
      break;
    }
  }

  double residualNormSquared = 0.0;
  for (std::uint32_t vertex = 0; vertex < vertexCount; ++vertex) {
    if (fixed[vertex] != 0u) {
      continue;
    }
    residualNormSquared += assemble(vertex).force.squaredNorm();
  }
  stats.finalResidualNormSquared = residualNormSquared;
  return stats;
}

//==============================================================================
/// Evaluate the variational implicit-Euler objective for a tetrahedral
/// Neo-Hookean body. Provided for convergence/energy-decrease verification.
inline double tetMeshObjective(
    const std::vector<Eigen::Vector3d>& positions,
    std::span<const double> masses,
    const std::vector<std::uint8_t>& fixed,
    const std::vector<Eigen::Vector3d>& inertialTargets,
    std::span<const TetMeshElement> tets,
    double mu,
    double lambda,
    double timeStep)
{
  double energy = 0.0;
  const double invDt2 = 1.0 / (timeStep * timeStep);
  for (std::size_t i = 0; i < positions.size(); ++i) {
    if (fixed[i] != 0u) {
      continue;
    }
    const Eigen::Vector3d delta = positions[i] - inertialTargets[i];
    energy += 0.5 * masses[i] * invDt2 * delta.squaredNorm();
  }
  for (const TetMeshElement& tet : tets) {
    const std::array<Eigen::Vector3d, 4> tetPositions
        = {positions[tet.vertices[0]],
           positions[tet.vertices[1]],
           positions[tet.vertices[2]],
           positions[tet.vertices[3]]};
    const Eigen::Matrix3d F
        = deformationGradient(tet.rest.restShapeInverse, tetPositions);
    energy
        += tet.rest.restVolume * stableNeoHookeanEnergyDensity(F, mu, lambda);
  }
  return energy;
}

//==============================================================================
/// Build the vertex-graph coloring induced by both a spring list and a tet mesh
/// (springs are edges, tets are 4-vertex cliques). Vertices that share neither
/// a spring nor a tetrahedron may take the same color, so the within-color
/// sweep stays numerically identical to the parallel Jacobi update for a body
/// that mixes distance springs and volumetric Neo-Hookean tetrahedra.
inline VertexColoring colorDeformable(
    std::size_t vertexCount,
    std::span<const SpringElement> springs,
    std::span<const TetMeshElement> tets,
    ::dart::common::MemoryAllocator& allocator
    = ::dart::common::MemoryAllocator::GetDefault())
{
  VertexAdjacency adjacency(vertexCount, allocator);
  for (const SpringElement& spring : springs) {
    adjacency.addEdge(spring.a, spring.b);
  }
  for (const TetMeshElement& tet : tets) {
    adjacency.addTetrahedron(
        tet.vertices[0], tet.vertices[1], tet.vertices[2], tet.vertices[3]);
  }
  return greedyColorVertices(adjacency);
}

namespace detail {

//==============================================================================
/// Assemble the inertia + incident-spring + incident-tetrahedron block for one
/// free vertex at its current position. Combines the mass-spring and tet
/// assemblers so a single body can carry both distance springs and volumetric
/// Stable Neo-Hookean tetrahedra.
inline VertexBlock assembleDeformableVertexBlock(
    std::uint32_t vertex,
    std::span<const Eigen::Vector3d> positions,
    std::span<const double> masses,
    std::span<const Eigen::Vector3d> inertialTargets,
    std::span<const SpringElement> springs,
    const SpringAdjacency& springAdjacency,
    double springStiffness,
    bool clampSpringHessian,
    std::span<const TetMeshElement> tets,
    const TetAdjacency& tetAdjacency,
    double mu,
    double lambda,
    double timeStep,
    bool useFemTetKernel = false,
    bool useFixedCorotationalTets = false,
    const SelfContactAdjacency* selfContact = nullptr)
{
  VertexBlock block;
  addInertiaTerm(
      block,
      masses[vertex],
      timeStep,
      positions[vertex],
      inertialTargets[vertex]);
  if (vertex < springAdjacency.incidentSprings.size()) {
    for (const std::uint32_t springIndex :
         springAdjacency.incidentSprings[vertex]) {
      const SpringElement& spring = springs[springIndex];
      const std::uint32_t other = (spring.a == vertex) ? spring.b : spring.a;
      addSpringTerm(
          block,
          springStiffness,
          spring.restLength,
          positions[vertex],
          positions[other],
          clampSpringHessian);
    }
  }
  if (vertex < tetAdjacency.incidentTets.size()) {
    for (const auto& [tetIndex, localVertex] :
         tetAdjacency.incidentTets[vertex]) {
      const TetMeshElement& tet = tets[tetIndex];
      const std::array<Eigen::Vector3d, 4> tetPositions
          = {positions[tet.vertices[0]],
             positions[tet.vertices[1]],
             positions[tet.vertices[2]],
             positions[tet.vertices[3]]};
      if (useFemTetKernel) {
        addFemTetTerm(
            block,
            localVertex,
            tet.rest,
            tetPositions,
            mu,
            lambda,
            useFixedCorotationalTets);
      } else {
        addNeoHookeanTetTerm(
            block, localVertex, tet.rest, tetPositions, mu, lambda);
      }
    }
  }
  if (selfContact != nullptr) {
    addSelfContactTerms(block, vertex, *selfContact, positions);
  }
  return block;
}

} // namespace detail

//==============================================================================
/// Run graph-colored Gauss-Seidel block coordinate descent on a single
/// deformable body that mixes distance springs and Stable Neo-Hookean
/// tetrahedra, minimizing
///   G(x) = sum_i (m_i/2h^2)||x_i - y_i||^2
///        + sum_e (k/2)(l_e - L_e)^2 + sum_t A_t Psi(F_t(x)).
///
/// `coloring` must be built for the union of `springs` and `tets` (see
/// colorDeformable); `springAdjacency`/`tetAdjacency` are the matching
/// incident-element lists. With no tets this reduces to blockDescentMassSpring;
/// with no springs it reduces to blockDescentTetMesh. `positions` is updated in
/// place.
///
/// `options.useChebyshev` over-relaxes each sweep with Chebyshev acceleration
/// (faster convergence, same fixed point). `options.rayleighDamping` adds
/// stiffness-proportional damping opposing the per-step displacement; it
/// requires `stepStartPositions` (the positions x^t at the start of the step)
/// and is a no-op when that is null or the coefficient is zero.
template <
    typename PositionVector,
    typename FixedMask,
    typename ChebyshevTwoStepsBackVector = std::vector<Eigen::Vector3d>,
    typename ChebyshevBeforeSweepVector = std::vector<Eigen::Vector3d>>
inline BlockDescentStats blockDescentDeformable(
    PositionVector& positions,
    std::span<const double> masses,
    const FixedMask& fixed,
    std::span<const Eigen::Vector3d> inertialTargets,
    std::span<const SpringElement> springs,
    double springStiffness,
    const SpringAdjacency& springAdjacency,
    std::span<const TetMeshElement> tets,
    double mu,
    double lambda,
    const TetAdjacency& tetAdjacency,
    double timeStep,
    const VertexColoring& coloring,
    const BlockDescentOptions& options,
    std::span<const Eigen::Vector3d> stepStartPositions = {},
    std::span<const ContactPlane> contactPlanes = {},
    double contactFriction = 0.0,
    const SelfContactAdjacency* selfContact = nullptr,
    ChebyshevTwoStepsBackVector* chebyshevTwoStepsBackScratch = nullptr,
    ChebyshevBeforeSweepVector* chebyshevBeforeSweepScratch = nullptr)
{
  BlockDescentStats stats;
  const std::size_t vertexCount = positions.size();
  const double invDt2 = 1.0 / (timeStep * timeStep);
  const bool useRayleigh
      = options.rayleighDamping > 0.0 && !stepStartPositions.empty();

  const auto assemble = [&](std::uint32_t vertex) {
    const SelfContactAdjacency* blockSelfContact
        = useRayleigh ? nullptr : selfContact;
    VertexBlock block = detail::assembleDeformableVertexBlock(
        vertex,
        positions,
        masses,
        inertialTargets,
        springs,
        springAdjacency,
        springStiffness,
        options.clampSpringHessian,
        tets,
        tetAdjacency,
        mu,
        lambda,
        timeStep,
        options.useFemTetKernel,
        options.useFixedCorotationalTets,
        blockSelfContact);
    if (useRayleigh) {
      // The elastic Hessian is the full block Hessian minus the (m/h^2) I
      // inertia term that addInertiaTerm placed on the diagonal. Contact
      // barriers are not elastic material stiffness, so add them after the
      // Rayleigh term.
      Eigen::Matrix3d elasticHessian = block.hessian;
      elasticHessian.diagonal().array() -= masses[vertex] * invDt2;
      addRayleighDamping(
          block,
          elasticHessian,
          positions[vertex] - stepStartPositions[vertex],
          options.rayleighDamping,
          timeStep);
      if (selfContact != nullptr) {
        addSelfContactTerms(block, vertex, *selfContact, positions);
      }
    }
    if (!contactPlanes.empty() && vertex < contactPlanes.size()) {
      const ContactPlane& plane = contactPlanes[vertex];
      addHalfSpacePenaltyContact(block, positions[vertex], plane);
      if (contactFriction > 0.0 && !stepStartPositions.empty()) {
        addHalfSpaceFriction(
            block,
            positions[vertex],
            stepStartPositions[vertex],
            plane,
            contactFriction);
      }
    }
    return block;
  };

  const double convergenceSquared
      = options.convergenceDisplacement * options.convergenceDisplacement;
  ChebyshevTwoStepsBackVector localTwoStepsBack;
  ChebyshevBeforeSweepVector localBeforeSweep;
  ChebyshevTwoStepsBackVector& twoStepsBack
      = chebyshevTwoStepsBackScratch != nullptr ? *chebyshevTwoStepsBackScratch
                                                : localTwoStepsBack;
  ChebyshevBeforeSweepVector& beforeSweep
      = chebyshevBeforeSweepScratch != nullptr ? *chebyshevBeforeSweepScratch
                                               : localBeforeSweep;
  if (options.useChebyshev) {
    twoStepsBack.assign(positions.begin(), positions.end());
  }
  double omega = 1.0;
  for (std::size_t iteration = 0; iteration < options.iterations; ++iteration) {
    ++stats.iterations;
    if (options.useChebyshev) {
      beforeSweep.assign(positions.begin(), positions.end());
    }
    double maxDeltaSquared = 0.0;
    for (const auto& group : coloring.groups) {
      for (const std::uint32_t vertex : group) {
        if (vertex >= vertexCount || fixed[vertex] != 0u) {
          continue;
        }
        const VertexBlock block = assemble(vertex);
        const Eigen::Vector3d delta
            = solveVertexBlock(block, options.regularization);
        positions[vertex] += delta;
        maxDeltaSquared = std::max(maxDeltaSquared, delta.squaredNorm());
        ++stats.vertexUpdates;
      }
    }
    if (options.useChebyshev) {
      omega = chebyshevOmega(iteration + 1, options.chebyshevRho, omega);
      if (omega > 1.0) {
        for (std::size_t i = 0; i < vertexCount; ++i) {
          if (fixed[i] == 0u) {
            positions[i] = applyChebyshev(omega, positions[i], twoStepsBack[i]);
          }
        }
      }
      twoStepsBack.assign(beforeSweep.begin(), beforeSweep.end());
    }
    if (convergenceSquared > 0.0 && maxDeltaSquared <= convergenceSquared) {
      break;
    }
  }

  double residualNormSquared = 0.0;
  for (std::uint32_t vertex = 0; vertex < vertexCount; ++vertex) {
    if (fixed[vertex] != 0u) {
      continue;
    }
    residualNormSquared += assemble(vertex).force.squaredNorm();
  }
  stats.finalResidualNormSquared = residualNormSquared;
  return stats;
}

//==============================================================================
/// Evaluate the combined springs + tetrahedra variational implicit-Euler
/// objective G(x) for convergence/energy-decrease verification.
inline double deformableObjective(
    const std::vector<Eigen::Vector3d>& positions,
    std::span<const double> masses,
    const std::vector<std::uint8_t>& fixed,
    const std::vector<Eigen::Vector3d>& inertialTargets,
    std::span<const SpringElement> springs,
    double springStiffness,
    std::span<const TetMeshElement> tets,
    double mu,
    double lambda,
    double timeStep)
{
  // The inertia term is shared, so sum the two element objectives and subtract
  // one copy of the inertia (which tetMeshObjective and massSpringObjective
  // each include).
  double energy = 0.0;
  const double invDt2 = 1.0 / (timeStep * timeStep);
  for (std::size_t i = 0; i < positions.size(); ++i) {
    if (fixed[i] != 0u) {
      continue;
    }
    const Eigen::Vector3d delta = positions[i] - inertialTargets[i];
    energy += 0.5 * masses[i] * invDt2 * delta.squaredNorm();
  }
  for (const SpringElement& spring : springs) {
    const double length = (positions[spring.b] - positions[spring.a]).norm();
    const double stretch = length - spring.restLength;
    energy += 0.5 * springStiffness * stretch * stretch;
  }
  for (const TetMeshElement& tet : tets) {
    const std::array<Eigen::Vector3d, 4> tetPositions
        = {positions[tet.vertices[0]],
           positions[tet.vertices[1]],
           positions[tet.vertices[2]],
           positions[tet.vertices[3]]};
    const Eigen::Matrix3d F
        = deformationGradient(tet.rest.restShapeInverse, tetPositions);
    energy
        += tet.rest.restVolume * stableNeoHookeanEnergyDensity(F, mu, lambda);
  }
  return energy;
}

//==============================================================================
/// Mass-spring block descent with static half-space penalty contact. Identical
/// to blockDescentMassSpring, but each vertex block additionally accumulates
/// the VBD penalty-contact term for every plane it penetrates, so a body rests
/// on (rather than tunnels through) ground/obstacle half-spaces. `positions` is
/// updated in place.
inline BlockDescentStats blockDescentMassSpringGround(
    std::vector<Eigen::Vector3d>& positions,
    std::span<const double> masses,
    const std::vector<std::uint8_t>& fixed,
    const std::vector<Eigen::Vector3d>& inertialTargets,
    std::span<const SpringElement> springs,
    double springStiffness,
    double timeStep,
    const std::vector<ContactPlane>& planes,
    const VertexColoring& coloring,
    const SpringAdjacency& adjacency,
    const BlockDescentOptions& options)
{
  BlockDescentStats stats;
  const std::size_t vertexCount = positions.size();
  const auto assemble = [&](std::uint32_t vertex) {
    VertexBlock block = detail::assembleVertexBlock(
        vertex,
        positions,
        masses,
        inertialTargets,
        springs,
        adjacency,
        springStiffness,
        timeStep,
        options.clampSpringHessian);
    for (const ContactPlane& plane : planes) {
      addHalfSpacePenaltyContact(block, positions[vertex], plane);
    }
    return block;
  };

  for (std::size_t iteration = 0; iteration < options.iterations; ++iteration) {
    ++stats.iterations;
    for (const auto& group : coloring.groups) {
      for (const std::uint32_t vertex : group) {
        if (vertex >= vertexCount || fixed[vertex] != 0u) {
          continue;
        }
        const VertexBlock block = assemble(vertex);
        positions[vertex] += solveVertexBlock(block, options.regularization);
        ++stats.vertexUpdates;
      }
    }
  }

  double residualNormSquared = 0.0;
  for (std::uint32_t vertex = 0; vertex < vertexCount; ++vertex) {
    if (fixed[vertex] != 0u) {
      continue;
    }
    residualNormSquared += assemble(vertex).force.squaredNorm();
  }
  stats.finalResidualNormSquared = residualNormSquared;
  return stats;
}

//==============================================================================
/// Mass-spring block descent with active AVBD half-space normal rows. This is
/// the first CPU AVBD contact-normal kernel slice: the primal sweep stamps the
/// bounded hard-row force/Hessian into each affected vertex block, then the
/// dual pass updates the row lambda/stiffness after every sweep. Row
/// generation, persistent row-ID mapping, friction, and multi-contact manifolds
/// are layered on top of this primitive.
template <typename ContactRows = std::vector<AvbdHalfSpaceContactRow>>
inline BlockDescentStats blockDescentMassSpringAvbdGround(
    std::vector<Eigen::Vector3d>& positions,
    std::span<const double> masses,
    const std::vector<std::uint8_t>& fixed,
    const std::vector<Eigen::Vector3d>& inertialTargets,
    std::span<const SpringElement> springs,
    double springStiffness,
    double timeStep,
    ContactRows& contactRows,
    const VertexColoring& coloring,
    const SpringAdjacency& adjacency,
    const BlockDescentOptions& options,
    const AvbdHalfSpaceContactOptions& avbdOptions)
{
  BlockDescentStats stats;
  const std::size_t vertexCount = positions.size();
  const auto assemble = [&](std::uint32_t vertex) {
    VertexBlock block = detail::assembleVertexBlock(
        vertex,
        positions,
        masses,
        inertialTargets,
        springs,
        adjacency,
        springStiffness,
        timeStep,
        options.clampSpringHessian);
    for (const AvbdHalfSpaceContactRow& row : contactRows) {
      if (row.vertex == vertex) {
        addAvbdHalfSpaceContactNormal(
            block,
            positions[vertex],
            row.plane,
            row.state,
            row.previousConstraintValue,
            avbdOptions.alpha,
            row.bounds);
      }
    }
    return block;
  };

  const double convergenceSquared
      = options.convergenceDisplacement * options.convergenceDisplacement;
  for (std::size_t iteration = 0; iteration < options.iterations; ++iteration) {
    ++stats.iterations;
    double maxDeltaSquared = 0.0;
    for (const auto& group : coloring.groups) {
      for (const std::uint32_t vertex : group) {
        if (vertex >= vertexCount || fixed[vertex] != 0u) {
          continue;
        }
        const VertexBlock block = assemble(vertex);
        const Eigen::Vector3d delta
            = solveVertexBlock(block, options.regularization);
        positions[vertex] += delta;
        maxDeltaSquared = std::max(maxDeltaSquared, delta.squaredNorm());
        ++stats.vertexUpdates;
      }
    }
    for (AvbdHalfSpaceContactRow& row : contactRows) {
      if (row.vertex >= vertexCount || fixed[row.vertex] != 0u) {
        continue;
      }
      row.state = updateAvbdHalfSpaceContactNormalRow(
          row.state,
          positions[row.vertex],
          row.plane,
          avbdOptions,
          row.previousConstraintValue,
          row.bounds);
    }
    if (convergenceSquared > 0.0 && maxDeltaSquared <= convergenceSquared) {
      break;
    }
  }

  double residualNormSquared = 0.0;
  for (std::uint32_t vertex = 0; vertex < vertexCount; ++vertex) {
    if (fixed[vertex] != 0u) {
      continue;
    }
    residualNormSquared += assemble(vertex).force.squaredNorm();
  }
  stats.finalResidualNormSquared = residualNormSquared;
  return stats;
}

//==============================================================================
/// Mass-spring block descent with active AVBD hard point-attachment rows. This
/// is the first non-contact hard-row kernel slice: a full 3D attachment is
/// represented by scalar rows, usually one per world axis, so row identity,
/// warm starting, force bounds, and stiffness growth remain per scalar row.
template <typename AttachmentRows = std::vector<AvbdPointAttachmentRow>>
inline BlockDescentStats blockDescentMassSpringAvbdAttachments(
    std::vector<Eigen::Vector3d>& positions,
    std::span<const double> masses,
    const std::vector<std::uint8_t>& fixed,
    const std::vector<Eigen::Vector3d>& inertialTargets,
    std::span<const SpringElement> springs,
    double springStiffness,
    double timeStep,
    AttachmentRows& attachmentRows,
    const VertexColoring& coloring,
    const SpringAdjacency& adjacency,
    const BlockDescentOptions& options,
    const AvbdPointAttachmentOptions& avbdOptions)
{
  BlockDescentStats stats;
  const std::size_t vertexCount = positions.size();
  const auto assemble = [&](std::uint32_t vertex) {
    VertexBlock block = detail::assembleVertexBlock(
        vertex,
        positions,
        masses,
        inertialTargets,
        springs,
        adjacency,
        springStiffness,
        timeStep,
        options.clampSpringHessian);
    for (const AvbdPointAttachmentRow& row : attachmentRows) {
      if (row.vertex == vertex) {
        addAvbdPointAttachment(
            block, positions[vertex], row, avbdOptions.alpha);
      }
    }
    return block;
  };

  const double convergenceSquared
      = options.convergenceDisplacement * options.convergenceDisplacement;
  for (std::size_t iteration = 0; iteration < options.iterations; ++iteration) {
    ++stats.iterations;
    double maxDeltaSquared = 0.0;
    for (const auto& group : coloring.groups) {
      for (const std::uint32_t vertex : group) {
        if (vertex >= vertexCount || fixed[vertex] != 0u) {
          continue;
        }
        const VertexBlock block = assemble(vertex);
        const Eigen::Vector3d delta
            = solveVertexBlock(block, options.regularization);
        positions[vertex] += delta;
        maxDeltaSquared = std::max(maxDeltaSquared, delta.squaredNorm());
        ++stats.vertexUpdates;
      }
    }
    for (AvbdPointAttachmentRow& row : attachmentRows) {
      if (row.vertex >= vertexCount || fixed[row.vertex] != 0u) {
        continue;
      }
      row.state = updateAvbdPointAttachmentRow(
          row.state, positions[row.vertex], row, avbdOptions);
    }
    if (convergenceSquared > 0.0 && maxDeltaSquared <= convergenceSquared) {
      break;
    }
  }

  double residualNormSquared = 0.0;
  for (std::uint32_t vertex = 0; vertex < vertexCount; ++vertex) {
    if (fixed[vertex] != 0u) {
      continue;
    }
    residualNormSquared += assemble(vertex).force.squaredNorm();
  }
  stats.finalResidualNormSquared = residualNormSquared;
  return stats;
}

//==============================================================================
/// Mass-spring block descent with half-space penalty contact and semi-implicit
/// Coulomb friction. Like blockDescentMassSpringGround, but each contacting
/// vertex also accumulates the friction term resisting its tangential
/// displacement from `stepStartPositions` (the positions at the start of the
/// step). `positions` is updated in place.
inline BlockDescentStats blockDescentMassSpringGroundFriction(
    std::vector<Eigen::Vector3d>& positions,
    std::span<const double> masses,
    const std::vector<std::uint8_t>& fixed,
    const std::vector<Eigen::Vector3d>& inertialTargets,
    const std::vector<Eigen::Vector3d>& stepStartPositions,
    std::span<const SpringElement> springs,
    double springStiffness,
    double timeStep,
    const std::vector<ContactPlane>& planes,
    double frictionCoeff,
    const VertexColoring& coloring,
    const SpringAdjacency& adjacency,
    const BlockDescentOptions& options)
{
  BlockDescentStats stats;
  const std::size_t vertexCount = positions.size();
  const auto assemble = [&](std::uint32_t vertex) {
    VertexBlock block = detail::assembleVertexBlock(
        vertex,
        positions,
        masses,
        inertialTargets,
        springs,
        adjacency,
        springStiffness,
        timeStep,
        options.clampSpringHessian);
    for (const ContactPlane& plane : planes) {
      addHalfSpacePenaltyContact(block, positions[vertex], plane);
      addHalfSpaceFriction(
          block,
          positions[vertex],
          stepStartPositions[vertex],
          plane,
          frictionCoeff);
    }
    return block;
  };

  for (std::size_t iteration = 0; iteration < options.iterations; ++iteration) {
    ++stats.iterations;
    for (const auto& group : coloring.groups) {
      for (const std::uint32_t vertex : group) {
        if (vertex >= vertexCount || fixed[vertex] != 0u) {
          continue;
        }
        const VertexBlock block = assemble(vertex);
        positions[vertex] += solveVertexBlock(block, options.regularization);
        ++stats.vertexUpdates;
      }
    }
  }

  double residualNormSquared = 0.0;
  for (std::uint32_t vertex = 0; vertex < vertexCount; ++vertex) {
    if (fixed[vertex] != 0u) {
      continue;
    }
    residualNormSquared += assemble(vertex).force.squaredNorm();
  }
  stats.finalResidualNormSquared = residualNormSquared;
  return stats;
}

} // namespace dart::simulation::detail::deformable_vbd
