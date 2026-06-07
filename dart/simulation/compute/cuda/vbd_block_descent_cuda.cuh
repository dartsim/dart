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

#include <dart/simulation/compute/cuda/cuda_runtime.cuh>

#include <vector>

#include <cstddef>
#include <cstdint>

namespace dart::simulation::compute::cuda {

// isCudaRuntimeAvailable() is declared once in cuda_runtime.cuh (included
// above).

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

/// A device-resident multi-step mass-spring VBD rollout: the full step loop
/// (inertial-target prediction, colored block-descent sweeps, velocity update)
/// runs on the GPU for `stepCount` implicit-Euler steps with a single
/// host-to-device upload and a single device-to-host download. This is the GPU
/// performance shape that avoids per-step transfer, matching how the reference
/// keeps state resident across steps.
struct VbdCudaRolloutProblem
{
  std::size_t nodeCount = 0;
  std::vector<double> positions;   ///< 3*nodeCount, updated in place.
  std::vector<double> velocities;  ///< 3*nodeCount, updated in place.
  std::vector<double> masses;      ///< nodeCount.
  std::vector<std::uint8_t> fixed; ///< nodeCount.

  std::vector<std::uint32_t> springA;
  std::vector<std::uint32_t> springB;
  std::vector<double> springRest;

  std::vector<std::uint32_t> incidentOffsets;
  std::vector<std::uint32_t> incidentSprings;

  std::vector<std::uint32_t> colorOffsets;
  std::vector<std::uint32_t> colorVertices;

  double gravity[3] = {0.0, 0.0, 0.0};
  double stiffness = 0.0;
  double timeStep = 0.0;
  std::size_t iterations = 0; ///< Sweeps per step.
  std::size_t stepCount = 0;  ///< Implicit-Euler steps in the rollout.
  /// Capture one step's kernel launch sequence into a CUDA graph and replay it
  /// for every step, amortizing per-launch overhead. The per-step launch shape
  /// is identical across steps, so a single captured graph replays exactly.
  bool useCudaGraph = false;
  /// Run the rollout in single precision (mixed precision): host state stays
  /// double, but the device arrays and arithmetic use float. On GPUs with a low
  /// double-throughput ratio this is markedly faster, at float accuracy.
  bool useSinglePrecision = false;
};

/// Run a device-resident VBD mass-spring rollout. Uploads once, runs the full
/// per-step pipeline on the GPU for `stepCount` steps, and downloads the final
/// positions and velocities. Throws on a CUDA error.
void vbdRolloutMassSpringCuda(VbdCudaRolloutProblem& problem);

/// A flattened tetrahedral Stable Neo-Hookean VBD problem for the GPU: SoA node
/// state, per-tet topology and rest data, per-vertex incident-tet CSR, and the
/// vertex-graph coloring. `tetRestShapeInverse` stores each tet's `Dm^{-1}` as
/// 9 row-major doubles.
struct VbdCudaTetProblem
{
  std::size_t nodeCount = 0;
  std::vector<double> positions;       ///< 3*nodeCount, updated in place.
  std::vector<double> inertialTargets; ///< 3*nodeCount.
  std::vector<double> masses;          ///< nodeCount.
  std::vector<std::uint8_t> fixed;     ///< nodeCount.

  std::vector<std::uint32_t> tetVertices;  ///< 4*tetCount.
  std::vector<double> tetRestShapeInverse; ///< 9*tetCount, row-major Dm^-1.
  std::vector<double> tetRestVolume;       ///< tetCount.

  std::vector<std::uint32_t> incidentTetOffsets; ///< nodeCount + 1 CSR.
  std::vector<std::uint32_t> incidentTetIndex;   ///< flattened tet indices.
  std::vector<std::uint8_t> incidentLocalVertex; ///< flattened local 0..3.

  std::vector<std::uint32_t> colorOffsets;
  std::vector<std::uint32_t> colorVertices;

  double mu = 0.0;
  double lambda = 0.0;
  double timeStep = 0.0;
  std::size_t iterations = 0;
};

/// Run `iterations` graph-colored Gauss-Seidel Stable Neo-Hookean VBD sweeps
/// for a tetrahedral body on the GPU, copying the final positions back. Throws
/// on a CUDA error.
void vbdStepTetMeshCuda(VbdCudaTetProblem& problem);

/// A device-resident multi-step tetrahedral Stable Neo-Hookean VBD rollout: the
/// full step loop (inertial-target prediction, colored Neo-Hookean sweeps,
/// velocity update) runs on the GPU for `stepCount` implicit-Euler steps with a
/// single host-to-device upload and a single device-to-host download. This is
/// the volumetric counterpart of vbdRolloutMassSpringCuda.
struct VbdCudaTetRolloutProblem
{
  std::size_t nodeCount = 0;
  std::vector<double> positions;   ///< 3*nodeCount, updated in place.
  std::vector<double> velocities;  ///< 3*nodeCount, updated in place.
  std::vector<double> masses;      ///< nodeCount.
  std::vector<std::uint8_t> fixed; ///< nodeCount.

  std::vector<std::uint32_t> tetVertices;  ///< 4*tetCount.
  std::vector<double> tetRestShapeInverse; ///< 9*tetCount, row-major Dm^-1.
  std::vector<double> tetRestVolume;       ///< tetCount.

  std::vector<std::uint32_t> incidentTetOffsets; ///< nodeCount + 1 CSR.
  std::vector<std::uint32_t> incidentTetIndex;   ///< flattened tet indices.
  std::vector<std::uint8_t> incidentLocalVertex; ///< flattened local 0..3.

  std::vector<std::uint32_t> colorOffsets;
  std::vector<std::uint32_t> colorVertices;

  double gravity[3] = {0.0, 0.0, 0.0};
  double mu = 0.0;
  double lambda = 0.0;
  double timeStep = 0.0;
  std::size_t iterations = 0; ///< Sweeps per step.
  std::size_t stepCount = 0;  ///< Implicit-Euler steps in the rollout.
  /// Capture one step into a CUDA graph and replay it for every step (see
  /// VbdCudaRolloutProblem::useCudaGraph).
  bool useCudaGraph = false;
  /// Run the rollout in single precision (mixed precision); see
  /// VbdCudaRolloutProblem::useSinglePrecision.
  bool useSinglePrecision = false;
};

/// Run a device-resident tetrahedral VBD rollout. Uploads once, runs the full
/// per-step pipeline on the GPU for `stepCount` steps, and downloads the final
/// positions and velocities. Throws on a CUDA error.
void vbdRolloutTetMeshCuda(VbdCudaTetRolloutProblem& problem);

} // namespace dart::simulation::compute::cuda
