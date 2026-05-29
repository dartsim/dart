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
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
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

#include <vector>

#include <cstddef>
#include <cstdint>

namespace dart::simulation::experimental::compute::cuda {

/// Return whether the CUDA runtime currently exposes at least one device.
/// Defined in the experimental CUDA target (shared with the rigid-body path);
/// this build-tree-only `.cuh` keeps CUDA names out of installed headers.
[[nodiscard]] bool isCudaRuntimeAvailable() noexcept;

/// A flattened single-body mass-spring Vertex Block Descent problem laid out
/// for the GPU: structure-of-arrays node state, a spring list, per-vertex
/// incident-spring CSR, and a vertex-graph coloring in CSR form.
///
/// Same-color vertices share no spring, so one CUDA kernel launch per color
/// updates every vertex of that color in parallel with no data races; colors
/// are launched in order on a single stream (Gauss-Seidel across colors). This
/// is the GPU shape the VBD paper uses (graph-colored parallel block descent).
struct VbdCudaMassSpringProblem
{
  std::size_t nodeCount = 0;
  std::vector<double> positions;       ///< 3*nodeCount, updated in place.
  std::vector<double> inertialTargets; ///< 3*nodeCount.
  std::vector<double> masses;          ///< nodeCount.
  std::vector<std::uint8_t> fixed;     ///< nodeCount.

  std::vector<std::uint32_t> springA; ///< springCount.
  std::vector<std::uint32_t> springB; ///< springCount.
  std::vector<double> springRest;     ///< springCount.

  std::vector<std::uint32_t> incidentOffsets; ///< nodeCount + 1 CSR offsets.
  std::vector<std::uint32_t> incidentSprings; ///< flattened incident springs.

  std::vector<std::uint32_t> colorOffsets;  ///< colorCount + 1 CSR offsets.
  std::vector<std::uint32_t> colorVertices; ///< vertices grouped by color.

  double stiffness = 0.0;
  double timeStep = 0.0;
  std::size_t iterations = 0;
};

/// Run `iterations` graph-colored Gauss-Seidel VBD sweeps for the problem on
/// the GPU. State is uploaded once, the per-color kernels run on a single
/// ordered stream for every sweep, and the final positions are copied back into
/// `problem.positions`. Throws on a CUDA error.
void vbdStepMassSpringCuda(VbdCudaMassSpringProblem& problem);

} // namespace dart::simulation::experimental::compute::cuda
