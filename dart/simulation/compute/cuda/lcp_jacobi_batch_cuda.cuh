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

namespace dart::simulation::compute::cuda {

/// Host packet for a homogeneous dense boxed-LCP batch solved by CUDA LCP
/// kernels.
///
/// All problems have @ref problemSize rows. Matrices are row-major and grouped
/// by problem: `A[p * n * n + row * n + col]`. Vectors are grouped by problem:
/// `b[p * n + row]`, `lo[...]`, `hi[...]`, `findex[...]`, and `x[...]`.
///
/// This is a DART 7 experimental CUDA execution packet, not a general-purpose
/// replacement for @c dart::math::LcpSolver yet. It covers standard, boxed, and
/// friction-index moving bounds for fixed-iteration projected Jacobi,
/// red-black Gauss-Seidel, and PGS batch kernels.
struct LcpBatchCudaProblem
{
  std::size_t problemSize{0};
  std::size_t problemCount{0};
  std::size_t iterations{100};
  double relaxation{1.0};
  double epsilonForDivision{1e-9};

  std::vector<double> A;
  std::vector<double> b;
  std::vector<double> lo;
  std::vector<double> hi;
  std::vector<int> findex;
  std::vector<double> x;
};

using LcpJacobiBatchCudaProblem = LcpBatchCudaProblem;
using LcpPgsBatchCudaProblem = LcpBatchCudaProblem;
using LcpRedBlackGaussSeidelBatchCudaProblem = LcpBatchCudaProblem;

/// Solve a homogeneous dense standard/boxed/friction-index LCP batch using CUDA
/// Jacobi.
///
/// The packet is validated before any CUDA runtime access. The implementation
/// copies one batch packet to the device, launches one row-parallel Jacobi
/// update kernel per iteration, and copies the final @ref x vector back.
void solveBoxedLcpJacobiBatchCuda(LcpBatchCudaProblem& problem);

/// Solve a homogeneous dense standard/boxed/friction-index LCP batch using CUDA
/// red-black Gauss-Seidel.
///
/// The packet is validated before any CUDA runtime access. The implementation
/// copies one batch packet to the device, then launches one row-parallel red
/// pass and one row-parallel black pass per iteration. Red rows read the
/// previous iterate; black rows read red updates and previous black values.
void solveBoxedLcpRedBlackGaussSeidelBatchCuda(LcpBatchCudaProblem& problem);

/// Solve a homogeneous dense standard/boxed/friction-index LCP batch using CUDA
/// PGS.
///
/// The packet is validated before any CUDA runtime access. The implementation
/// copies one batch packet to the device, launches one problem-parallel PGS
/// kernel that performs @ref iterations sequential sweeps per problem, and
/// copies the final @ref x vector back.
void solveBoxedLcpPgsBatchCuda(LcpBatchCudaProblem& problem);

/// Solve a grouped dense boxed-LCP batch using CUDA Jacobi.
///
/// Each entry is one homogeneous CUDA packet. This is the host-side bridge for
/// variable-size batches: callers group problems by row count, then each group
/// executes on the same CUDA Jacobi kernel used by @ref
/// solveBoxedLcpJacobiBatchCuda.
void solveBoxedLcpJacobiGroupedBatchCuda(
    std::vector<LcpBatchCudaProblem>& problemGroups);

/// Solve a grouped dense boxed-LCP batch using CUDA red-black Gauss-Seidel.
///
/// Each entry is one homogeneous CUDA packet. This covers variable-size batches
/// by executing one CUDA red-black Gauss-Seidel packet per row-count group.
void solveBoxedLcpRedBlackGaussSeidelGroupedBatchCuda(
    std::vector<LcpBatchCudaProblem>& problemGroups);

/// Solve a grouped dense boxed-LCP batch using CUDA PGS.
///
/// Each entry is one homogeneous CUDA packet. This covers variable-size batches
/// by executing one CUDA PGS packet per row-count group.
void solveBoxedLcpPgsGroupedBatchCuda(
    std::vector<LcpBatchCudaProblem>& problemGroups);

} // namespace dart::simulation::compute::cuda
