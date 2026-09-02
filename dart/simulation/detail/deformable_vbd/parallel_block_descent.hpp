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

#include <dart/simulation/compute/compute_executor.hpp>
#include <dart/simulation/detail/deformable_vbd/block_descent.hpp>

#include <Eigen/Core>

#include <algorithm>
#include <span>
#include <vector>

#include <cstddef>
#include <cstdint>

namespace dart::simulation::detail::deformable_vbd {

//==============================================================================
/// Executor-parallel graph-colored Gauss-Seidel mass-spring block descent.
///
/// This is the CPU realization of VBD's parallelism: within a color, vertices
/// share no spring, so the injected executor updates deterministic contiguous
/// slices of each color with no data races. `parallelFor()` blocks at the end
/// of each color, preserving Gauss-Seidel color order. The result is therefore
/// identical to the serial `blockDescentMassSpring` for the same iteration
/// count.
///
/// A single-worker executor falls back to the serial driver. The parallel path
/// reduces per-sweep displacement in vertex-index order after all color
/// barriers, so early termination is deterministic across worker counts.
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
    compute::ComputeExecutor& executor)
{
  const std::size_t workerCount = executor.getWorkerCount();
  if (workerCount <= 1u) {
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
        options);
  }

  BlockDescentStats stats;
  const std::size_t vertexCount = positions.size();
  const double convergenceSquared
      = options.convergenceDisplacement * options.convergenceDisplacement;
  std::vector<Eigen::Vector3d> beforeSweep;
  for (std::size_t iteration = 0; iteration < options.iterations; ++iteration) {
    ++stats.iterations;
    if (convergenceSquared > 0.0) {
      beforeSweep.assign(positions.begin(), positions.end());
    }
    for (const auto& group : coloring.groups) {
      const std::size_t groupSize = group.size();
      const std::size_t chunkSize
          = (groupSize + workerCount - 1u) / workerCount;
      executor.parallelFor(
          groupSize, chunkSize, [&](std::size_t begin, std::size_t end) {
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
              positions[vertex]
                  += solveVertexBlock(block, options.regularization);
            }
          });
    }
    if (convergenceSquared > 0.0) {
      double maxDeltaSquared = 0.0;
      for (std::size_t vertex = 0; vertex < vertexCount; ++vertex) {
        if (fixed[vertex] == 0u) {
          maxDeltaSquared = std::max(
              maxDeltaSquared,
              (positions[vertex] - beforeSweep[vertex]).squaredNorm());
        }
      }
      if (maxDeltaSquared <= convergenceSquared) {
        break;
      }
    }
  }

  std::size_t freeVertexCount = 0u;
  double residualNormSquared = 0.0;
  for (std::uint32_t vertex = 0; vertex < vertexCount; ++vertex) {
    if (fixed[vertex] != 0u) {
      continue;
    }
    ++freeVertexCount;
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
  stats.vertexUpdates = freeVertexCount * stats.iterations;
  stats.finalResidualNormSquared = residualNormSquared;
  return stats;
}

//==============================================================================
/// Executor-parallel graph-colored Gauss-Seidel block descent for a body that
/// mixes distance springs and Stable Neo-Hookean tetrahedra (the parallel
/// counterpart of blockDescentDeformable). Same-color vertices share neither a
/// spring nor a tetrahedron, so the injected executor updates deterministic
/// contiguous slices of each color race-free, giving a result identical to the
/// serial driver for the same iteration count.
///
/// Optional Rayleigh damping is honored via `stepStartPositions`. Chebyshev
/// extrapolation is parallel per vertex, and convergence is reduced in
/// vertex-index order after extrapolation, making both controls deterministic
/// across worker counts. Active self-contact falls back to the serial driver
/// because the lagged VT/EE contact stencils are not part of the cached
/// spring/tet coloring.
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
    compute::ComputeExecutor& executor,
    std::span<const Eigen::Vector3d> stepStartPositions = {},
    std::span<const ContactPlane> contactPlanes = {},
    double contactFriction = 0.0,
    const SelfContactAdjacency* selfContact = nullptr,
    ChebyshevTwoStepsBackVector* chebyshevTwoStepsBackScratch = nullptr,
    ChebyshevBeforeSweepVector* chebyshevBeforeSweepScratch = nullptr)
{
  const std::size_t workerCount = executor.getWorkerCount();
  if (workerCount <= 1u || (selfContact != nullptr && selfContact->active())) {
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

  BlockDescentStats stats;
  double omega = 1.0;
  for (std::size_t iteration = 0; iteration < options.iterations; ++iteration) {
    ++stats.iterations;
    if (options.useChebyshev || convergenceSquared > 0.0) {
      beforeSweep.assign(positions.begin(), positions.end());
    }
    for (const auto& group : coloring.groups) {
      const std::size_t groupSize = group.size();
      const std::size_t chunkSize
          = (groupSize + workerCount - 1u) / workerCount;
      executor.parallelFor(
          groupSize, chunkSize, [&](std::size_t begin, std::size_t end) {
            for (std::size_t k = begin; k < end; ++k) {
              const std::uint32_t vertex = group[k];
              if (vertex >= vertexCount || fixed[vertex] != 0u) {
                continue;
              }
              positions[vertex]
                  += solveVertexBlock(assemble(vertex), options.regularization);
            }
          });
    }
    if (options.useChebyshev) {
      omega = chebyshevOmega(iteration + 1, options.chebyshevRho, omega);
      if (omega > 1.0 && vertexCount > 0u) {
        const std::size_t chunkSize
            = (vertexCount + workerCount - 1u) / workerCount;
        executor.parallelFor(
            vertexCount, chunkSize, [&](std::size_t begin, std::size_t end) {
              for (std::size_t vertex = begin; vertex < end; ++vertex) {
                if (fixed[vertex] == 0u) {
                  positions[vertex] = applyChebyshev(
                      omega, positions[vertex], twoStepsBack[vertex]);
                }
              }
            });
      }
      twoStepsBack.assign(beforeSweep.begin(), beforeSweep.end());
    }
    if (convergenceSquared > 0.0) {
      double maxDeltaSquared = 0.0;
      for (std::size_t vertex = 0; vertex < vertexCount; ++vertex) {
        if (fixed[vertex] == 0u) {
          maxDeltaSquared = std::max(
              maxDeltaSquared,
              (positions[vertex] - beforeSweep[vertex]).squaredNorm());
        }
      }
      if (maxDeltaSquared <= convergenceSquared) {
        break;
      }
    }
  }

  std::size_t freeVertexCount = 0u;
  double residualNormSquared = 0.0;
  for (std::uint32_t vertex = 0; vertex < vertexCount; ++vertex) {
    if (fixed[vertex] != 0u) {
      continue;
    }
    ++freeVertexCount;
    residualNormSquared += assemble(vertex).force.squaredNorm();
  }
  stats.vertexUpdates = freeVertexCount * stats.iterations;
  stats.finalResidualNormSquared = residualNormSquared;
  return stats;
}

} // namespace dart::simulation::detail::deformable_vbd
