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
 *     copyright notice, this list of conditions and the disclaimer in
 *     the documentation and/or other materials provided with the
 *     distribution.
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

namespace dart::simulation::compute::cuda {

constexpr std::size_t kNewtonAssemblySolveDofsPerBody = 6;
constexpr std::size_t kNewtonAssemblySolveBlockEntries
    = kNewtonAssemblySolveDofsPerBody * kNewtonAssemblySolveDofsPerBody;

struct NewtonAssemblySolveRowInput
{
  std::uint32_t bodyIndex = 0;
  double hessianDiagonal[kNewtonAssemblySolveDofsPerBody] = {};
  double gradient[kNewtonAssemblySolveDofsPerBody] = {};
};

struct NewtonOffDiagonalAssemblyRowInput
{
  std::uint32_t pairIndex = 0;
  double hessianBlock[kNewtonAssemblySolveBlockEntries] = {};
};

struct NewtonEqualityReductionEntry
{
  std::uint32_t fullDofIndex = 0;
  std::uint32_t reducedDofIndex = 0;
  double basisValue = 0.0;
};

struct NewtonAssemblySolveTiming
{
  double setupNs = 0.0;
  double hostToDeviceNs = 0.0;
  double assemblyKernelNs = 0.0;
  double reductionKernelNs = 0.0;
  double solveKernelNs = 0.0;
  double expansionKernelNs = 0.0;
  double deviceToHostNs = 0.0;
};

struct NewtonAssemblySolveResult
{
  std::vector<double> assembledDiagonal;
  std::vector<double> assembledGradient;
  std::vector<double> step;
  std::vector<double> residual;
  std::size_t bodyCount = 0;
  std::size_t rowCount = 0;
  std::size_t activeDofCount = 0;
  double maxDiagonal = 0.0;
  double maxGradientAbs = 0.0;
  double stepNorm = 0.0;
  double residualNorm = 0.0;
  NewtonAssemblySolveTiming timing;
};

struct NewtonOffDiagonalAssemblyResult
{
  std::vector<double> assembledBlocks;
  std::size_t pairCount = 0;
  std::size_t rowCount = 0;
  std::size_t activeBlockCount = 0;
  double maxBlockAbs = 0.0;
  NewtonAssemblySolveTiming timing;
};

struct NewtonEqualityReducedSolveResult
{
  std::vector<double> assembledDiagonal;
  std::vector<double> assembledGradient;
  std::vector<double> reducedDiagonal;
  std::vector<double> reducedGradient;
  std::vector<double> reducedStep;
  std::vector<double> reducedResidual;
  std::vector<double> fullStep;
  std::size_t bodyCount = 0;
  std::size_t rowCount = 0;
  std::size_t fullDofCount = 0;
  std::size_t reductionEntryCount = 0;
  std::size_t reducedDofCount = 0;
  std::size_t activeReducedDofCount = 0;
  double maxDiagonal = 0.0;
  double maxGradientAbs = 0.0;
  double stepNorm = 0.0;
  double residualNorm = 0.0;
  NewtonAssemblySolveTiming timing;
};

/// Evaluate a private reduced Newton assembly/solve packet on CUDA.
///
/// The packet scatters diagonal 6-DOF row contributions into per-body
/// generalized diagonals and gradients, then solves the independent
/// regularized diagonal Newton step. It intentionally does not cover
/// off-diagonal sparse blocks, equality reduction, global sparse factorization,
/// or a public GPU solver backend.
void evaluateNewtonAssemblySolveCuda(
    const std::vector<NewtonAssemblySolveRowInput>& rows,
    std::size_t bodyCount,
    double regularization,
    NewtonAssemblySolveResult& result);

/// Evaluate a private reduced off-diagonal sparse-block assembly packet.
///
/// The packet scatters 6x6 off-diagonal Hessian contributions into
/// preallocated sparse pair slots. It intentionally does not cover sparse graph
/// construction, equality reduction, global sparse factorization, runtime scene
/// assembly, or a public GPU solver backend.
void evaluateNewtonOffDiagonalAssemblyCuda(
    const std::vector<NewtonOffDiagonalAssemblyRowInput>& rows,
    std::size_t pairCount,
    NewtonOffDiagonalAssemblyResult& result);

/// Evaluate a private reduced equality-projected diagonal solve packet.
///
/// The packet assembles full-space diagonal rows, projects them through a
/// sparse reduced-coordinate basis, solves the reduced regularized diagonal
/// Newton step, and expands the step back to full coordinates. It intentionally
/// does not cover global sparse factorization, nonlinear equality constraints,
/// runtime scene assembly, or a public GPU solver backend.
void evaluateNewtonEqualityReducedSolveCuda(
    const std::vector<NewtonAssemblySolveRowInput>& rows,
    std::size_t bodyCount,
    const std::vector<NewtonEqualityReductionEntry>& reductionEntries,
    std::size_t reducedDofCount,
    double regularization,
    NewtonEqualityReducedSolveResult& result);

} // namespace dart::simulation::compute::cuda
