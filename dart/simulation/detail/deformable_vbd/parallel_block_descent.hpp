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

#include <dart/simulation/detail/deformable_vbd/block_descent.hpp>

#include <Eigen/Core>

#include <algorithm>
#include <barrier>
#include <span>
#include <thread>
#include <vector>

#include <cstddef>
#include <cstdint>

namespace dart::simulation::detail::deformable_vbd {

//==============================================================================
/// Multithreaded graph-colored Gauss-Seidel mass-spring block descent.
///
/// This is the CPU realization of VBD's parallelism: within a color, vertices
/// share no spring, so a fixed pool of worker threads each updates a disjoint
/// contiguous slice of that color's vertices with no data races (each writes
/// only its own vertices and reads only other-colored neighbors). A
/// `std::barrier` synchronizes the threads between colors so the Gauss-Seidel
/// color order is preserved. The result is therefore identical to the serial
/// `blockDescentMassSpring` for the same iteration count.
///
/// `threadCount <= 1` falls back to the serial driver. Early termination is not
/// applied here (it would need a cross-thread reduction), so the full
/// `options.iterations` budget runs.
inline BlockDescentStats parallelBlockDescentMassSpring(
    std::vector<Eigen::Vector3d>& positions,
    std::span<const double> masses,
    const std::vector<std::uint8_t>& fixed,
    const std::vector<Eigen::Vector3d>& inertialTargets,
    std::span<const SpringElement> springs,
    double springStiffness,
    double timeStep,
    const VertexColoring& coloring,
    const SpringAdjacency& adjacency,
    const BlockDescentOptions& options,
    unsigned int threadCount)
{
  if (threadCount <= 1) {
    BlockDescentOptions serialOptions = options;
    serialOptions.convergenceDisplacement = 0.0;
    return blockDescentMassSpring(
        positions,
        masses,
        fixed,
        inertialTargets,
        springs,
        springStiffness,
        timeStep,
        coloring,
        adjacency,
        serialOptions);
  }

  const std::size_t vertexCount = positions.size();
  std::barrier sync(static_cast<std::ptrdiff_t>(threadCount));

  const auto worker = [&](unsigned int threadId) {
    for (std::size_t iteration = 0; iteration < options.iterations;
         ++iteration) {
      for (const auto& group : coloring.groups) {
        const std::size_t groupSize = group.size();
        const std::size_t chunk = (groupSize + threadCount - 1) / threadCount;
        const std::size_t begin = std::min(groupSize, threadId * chunk);
        const std::size_t end = std::min(groupSize, begin + chunk);
        for (std::size_t k = begin; k < end; ++k) {
          const std::uint32_t vertex = group[k];
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
          positions[vertex] += solveVertexBlock(block, options.regularization);
        }
        sync.arrive_and_wait();
      }
    }
  };

  std::vector<std::thread> threads;
  threads.reserve(threadCount);
  for (unsigned int t = 0; t < threadCount; ++t) {
    threads.emplace_back(worker, t);
  }
  for (auto& thread : threads) {
    thread.join();
  }

  BlockDescentStats stats;
  stats.iterations = options.iterations;
  double residualNormSquared = 0.0;
  for (std::uint32_t vertex = 0; vertex < vertexCount; ++vertex) {
    if (fixed[vertex] != 0u) {
      continue;
    }
    ++stats.vertexUpdates;
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
  stats.vertexUpdates *= options.iterations;
  stats.finalResidualNormSquared = residualNormSquared;
  return stats;
}

//==============================================================================
/// Multithreaded graph-colored Gauss-Seidel block descent for a body that mixes
/// distance springs and Stable Neo-Hookean tetrahedra (the parallel counterpart
/// of blockDescentDeformable). Same-color vertices share neither a spring nor a
/// tetrahedron, so a fixed worker pool updates each color's disjoint vertex
/// slices race-free with a `std::barrier` between colors, giving a result
/// identical to the serial driver for the same iteration count.
///
/// Optional Rayleigh damping is honored via `stepStartPositions`. Chebyshev
/// over-relaxation and residual early termination are NOT applied on the
/// multithreaded path (they need cross-thread reductions / a global
/// extrapolation); `threadCount <= 1` falls back to the full-featured serial
/// blockDescentDeformable, which does honor them. Active self-contact also
/// falls back to the serial driver because the lagged VT/EE contact stencils
/// are not part of the cached spring/tet coloring.
template <
    typename PositionVector,
    typename FixedMask,
    typename ChebyshevTwoStepsBackVector = std::vector<Eigen::Vector3d>,
    typename ChebyshevBeforeSweepVector = std::vector<Eigen::Vector3d>>
inline BlockDescentStats parallelBlockDescentDeformable(
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
    unsigned int threadCount,
    std::span<const Eigen::Vector3d> stepStartPositions = {},
    std::span<const ContactPlane> contactPlanes = {},
    double contactFriction = 0.0,
    const SelfContactAdjacency* selfContact = nullptr,
    ChebyshevTwoStepsBackVector* chebyshevTwoStepsBackScratch = nullptr,
    ChebyshevBeforeSweepVector* chebyshevBeforeSweepScratch = nullptr)
{
  if (threadCount <= 1 || (selfContact != nullptr && selfContact->active())) {
    return blockDescentDeformable(
        positions,
        masses,
        fixed,
        inertialTargets,
        springs,
        springStiffness,
        springAdjacency,
        tets,
        mu,
        lambda,
        tetAdjacency,
        timeStep,
        coloring,
        options,
        stepStartPositions,
        contactPlanes,
        contactFriction,
        selfContact,
        chebyshevTwoStepsBackScratch,
        chebyshevBeforeSweepScratch);
  }

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

  std::barrier sync(static_cast<std::ptrdiff_t>(threadCount));
  const auto worker = [&](unsigned int threadId) {
    for (std::size_t iteration = 0; iteration < options.iterations;
         ++iteration) {
      for (const auto& group : coloring.groups) {
        const std::size_t groupSize = group.size();
        const std::size_t chunk = (groupSize + threadCount - 1) / threadCount;
        const std::size_t begin = std::min(groupSize, threadId * chunk);
        const std::size_t end = std::min(groupSize, begin + chunk);
        for (std::size_t k = begin; k < end; ++k) {
          const std::uint32_t vertex = group[k];
          if (vertex >= vertexCount || fixed[vertex] != 0u) {
            continue;
          }
          positions[vertex]
              += solveVertexBlock(assemble(vertex), options.regularization);
        }
        sync.arrive_and_wait();
      }
    }
  };

  std::vector<std::thread> threads;
  threads.reserve(threadCount);
  for (unsigned int t = 0; t < threadCount; ++t) {
    threads.emplace_back(worker, t);
  }
  for (auto& thread : threads) {
    thread.join();
  }

  BlockDescentStats stats;
  stats.iterations = options.iterations;
  double residualNormSquared = 0.0;
  for (std::uint32_t vertex = 0; vertex < vertexCount; ++vertex) {
    if (fixed[vertex] != 0u) {
      continue;
    }
    ++stats.vertexUpdates;
    residualNormSquared += assemble(vertex).force.squaredNorm();
  }
  stats.vertexUpdates *= options.iterations;
  stats.finalResidualNormSquared = residualNormSquared;
  return stats;
}

} // namespace dart::simulation::detail::deformable_vbd
