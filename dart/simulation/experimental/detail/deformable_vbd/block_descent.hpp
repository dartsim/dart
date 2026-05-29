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

#include <dart/simulation/experimental/detail/deformable_vbd/contact_kernel.hpp>
#include <dart/simulation/experimental/detail/deformable_vbd/neo_hookean.hpp>
#include <dart/simulation/experimental/detail/deformable_vbd/vertex_block_kernel.hpp>
#include <dart/simulation/experimental/detail/deformable_vbd/vertex_coloring.hpp>

#include <Eigen/Core>

#include <algorithm>
#include <array>
#include <utility>
#include <vector>

#include <cstddef>
#include <cstdint>

namespace dart::simulation::experimental::detail::deformable_vbd {

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
  std::vector<std::vector<std::uint32_t>> incidentSprings;

  static SpringAdjacency build(
      std::size_t vertexCount, const std::vector<SpringElement>& springs)
  {
    SpringAdjacency adjacency;
    adjacency.incidentSprings.resize(vertexCount);
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
};

/// Build the vertex-graph coloring induced by a spring list.
inline VertexColoring colorSprings(
    std::size_t vertexCount, const std::vector<SpringElement>& springs)
{
  VertexAdjacency adjacency(vertexCount);
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
    const std::vector<double>& masses,
    const std::vector<Eigen::Vector3d>& inertialTargets,
    const std::vector<SpringElement>& springs,
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
    const std::vector<double>& masses,
    const std::vector<std::uint8_t>& fixed,
    const std::vector<Eigen::Vector3d>& inertialTargets,
    const std::vector<SpringElement>& springs,
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
    const std::vector<double>& masses,
    const std::vector<std::uint8_t>& fixed,
    const std::vector<Eigen::Vector3d>& inertialTargets,
    const std::vector<SpringElement>& springs,
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
  std::vector<std::vector<std::pair<std::uint32_t, std::uint8_t>>> incidentTets;

  static TetAdjacency build(
      std::size_t vertexCount, const std::vector<TetMeshElement>& tets)
  {
    TetAdjacency adjacency;
    adjacency.incidentTets.resize(vertexCount);
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
};

/// Build the vertex-graph coloring induced by a tetrahedral mesh (each tet is a
/// 4-vertex clique in the vertex graph).
inline VertexColoring colorTetMesh(
    std::size_t vertexCount, const std::vector<TetMeshElement>& tets)
{
  VertexAdjacency adjacency(vertexCount);
  for (const TetMeshElement& tet : tets) {
    adjacency.addTetrahedron(
        tet.vertices[0], tet.vertices[1], tet.vertices[2], tet.vertices[3]);
  }
  return greedyColorVertices(adjacency);
}

namespace detail {

//==============================================================================
/// Assemble the inertia + incident-tetrahedron Neo-Hookean block for one free
/// vertex at the current positions.
inline VertexBlock assembleTetVertexBlock(
    std::uint32_t vertex,
    const std::vector<Eigen::Vector3d>& positions,
    const std::vector<double>& masses,
    const std::vector<Eigen::Vector3d>& inertialTargets,
    const std::vector<TetMeshElement>& tets,
    const TetAdjacency& adjacency,
    double mu,
    double lambda,
    double timeStep)
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
    addNeoHookeanTetTerm(
        block, localVertex, tet.rest, tetPositions, mu, lambda);
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
    const std::vector<double>& masses,
    const std::vector<std::uint8_t>& fixed,
    const std::vector<Eigen::Vector3d>& inertialTargets,
    const std::vector<TetMeshElement>& tets,
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
            timeStep);
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
        timeStep);
    residualNormSquared += block.force.squaredNorm();
  }
  stats.finalResidualNormSquared = residualNormSquared;
  return stats;
}

//==============================================================================
/// Evaluate the variational implicit-Euler objective for a tetrahedral
/// Neo-Hookean body. Provided for convergence/energy-decrease verification.
inline double tetMeshObjective(
    const std::vector<Eigen::Vector3d>& positions,
    const std::vector<double>& masses,
    const std::vector<std::uint8_t>& fixed,
    const std::vector<Eigen::Vector3d>& inertialTargets,
    const std::vector<TetMeshElement>& tets,
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
/// Mass-spring block descent with static half-space penalty contact. Identical
/// to blockDescentMassSpring, but each vertex block additionally accumulates
/// the VBD penalty-contact term for every plane it penetrates, so a body rests
/// on (rather than tunnels through) ground/obstacle half-spaces. `positions` is
/// updated in place.
inline BlockDescentStats blockDescentMassSpringGround(
    std::vector<Eigen::Vector3d>& positions,
    const std::vector<double>& masses,
    const std::vector<std::uint8_t>& fixed,
    const std::vector<Eigen::Vector3d>& inertialTargets,
    const std::vector<SpringElement>& springs,
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
/// Mass-spring block descent with half-space penalty contact and semi-implicit
/// Coulomb friction. Like blockDescentMassSpringGround, but each contacting
/// vertex also accumulates the friction term resisting its tangential
/// displacement from `stepStartPositions` (the positions at the start of the
/// step). `positions` is updated in place.
inline BlockDescentStats blockDescentMassSpringGroundFriction(
    std::vector<Eigen::Vector3d>& positions,
    const std::vector<double>& masses,
    const std::vector<std::uint8_t>& fixed,
    const std::vector<Eigen::Vector3d>& inertialTargets,
    const std::vector<Eigen::Vector3d>& stepStartPositions,
    const std::vector<SpringElement>& springs,
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

} // namespace dart::simulation::experimental::detail::deformable_vbd
