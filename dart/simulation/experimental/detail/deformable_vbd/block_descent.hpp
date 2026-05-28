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

#include <dart/simulation/experimental/detail/deformable_vbd/vertex_block_kernel.hpp>
#include <dart/simulation/experimental/detail/deformable_vbd/vertex_coloring.hpp>

#include <Eigen/Core>

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
  std::size_t iterations = 20;    ///< VBD sweeps over all colors per call.
  double regularization = 0.0;    ///< Diagonal damping added to each block.
  bool clampSpringHessian = true; ///< PSD-project the spring Hessian blocks.
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

  for (std::size_t iteration = 0; iteration < options.iterations; ++iteration) {
    ++stats.iterations;
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
        ++stats.vertexUpdates;
      }
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

} // namespace dart::simulation::experimental::detail::deformable_vbd
