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
constexpr std::size_t kNewtonDirectSparseSolveMaxDofs = 48;

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

struct NewtonSparseBlockEntry
{
  std::uint32_t rowBodyIndex = 0;
  std::uint32_t columnBodyIndex = 0;
  double hessianBlock[kNewtonAssemblySolveBlockEntries] = {};
};

struct NewtonEqualityReductionEntry
{
  std::uint32_t fullDofIndex = 0;
  std::uint32_t reducedDofIndex = 0;
  double basisValue = 0.0;
};

struct NewtonSceneNodeInput
{
  double position[3] = {};
  double velocity[3] = {};
  double mass = 0.0;
};

struct NewtonSceneSurfaceTriangleInput
{
  std::uint32_t nodeA = 0;
  std::uint32_t nodeB = 0;
  std::uint32_t nodeC = 0;
};

struct NewtonSceneDistanceEqualityConstraintInput
{
  std::uint32_t nodeA = 0;
  std::uint32_t nodeB = 0;
  double restLength = 0.0;
  double stiffness = 0.0;
  double damping = 0.0;
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

struct NewtonSceneSparseGraphAssemblyTiming
{
  double setupNs = 0.0;
  double hostToDeviceNs = 0.0;
  double incidenceKernelNs = 0.0;
  double diagonalKernelNs = 0.0;
  double sparseBlockKernelNs = 0.0;
  double deviceToHostNs = 0.0;
};

struct NewtonSceneNonlinearEqualityAssemblyTiming
{
  double setupNs = 0.0;
  double hostToDeviceNs = 0.0;
  double constraintKernelNs = 0.0;
  double deviceToHostNs = 0.0;
};

struct NewtonSceneNonlinearEqualitySolveTiming
{
  double setupNs = 0.0;
  double hostToDeviceNs = 0.0;
  double solveKernelNs = 0.0;
  double postResidualKernelNs = 0.0;
  double deviceToHostNs = 0.0;
};

struct NewtonSceneNonlinearEqualityConvergenceTiming
{
  double setupNs = 0.0;
  double hostToDeviceNs = 0.0;
  double iterationKernelNs = 0.0;
  double residualKernelNs = 0.0;
  double convergenceReadbackNs = 0.0;
  double finalAssemblyKernelNs = 0.0;
  double deviceToHostNs = 0.0;
};

struct NewtonSparseResidualTiming
{
  double setupNs = 0.0;
  double hostToDeviceNs = 0.0;
  double assemblyKernelNs = 0.0;
  double gradientSeedNs = 0.0;
  double diagonalKernelNs = 0.0;
  double offDiagonalKernelNs = 0.0;
  double deviceToHostNs = 0.0;
};

struct NewtonSparseJacobiSolveTiming
{
  double setupNs = 0.0;
  double hostToDeviceNs = 0.0;
  double assemblyKernelNs = 0.0;
  double iterationKernelNs = 0.0;
  double finalResidualKernelNs = 0.0;
  double deviceToHostNs = 0.0;
};

struct NewtonSparseCgSolveTiming
{
  double setupNs = 0.0;
  double hostToDeviceNs = 0.0;
  double assemblyKernelNs = 0.0;
  double iterationKernelNs = 0.0;
  double finalResidualKernelNs = 0.0;
  double deviceToHostNs = 0.0;
};

struct NewtonDirectSparseSolveTiming
{
  double setupNs = 0.0;
  double hostToDeviceNs = 0.0;
  double assemblyKernelNs = 0.0;
  double denseMatrixKernelNs = 0.0;
  double factorSolveKernelNs = 0.0;
  double finalResidualKernelNs = 0.0;
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

struct NewtonSceneSparseGraphAssemblyResult
{
  std::vector<NewtonAssemblySolveRowInput> rows;
  std::vector<NewtonSparseBlockEntry> blocks;
  std::size_t nodeCount = 0;
  std::size_t triangleCount = 0;
  std::size_t rowCount = 0;
  std::size_t bodyCount = 0;
  std::size_t dofCount = 0;
  std::size_t blockCount = 0;
  std::size_t blockEntryCount = 0;
  double maxDiagonal = 0.0;
  double maxGradientAbs = 0.0;
  double maxBlockAbs = 0.0;
  NewtonSceneSparseGraphAssemblyTiming timing;
};

struct NewtonSceneNonlinearEqualityAssemblyResult
{
  std::vector<NewtonAssemblySolveRowInput> rows;
  std::vector<NewtonSparseBlockEntry> blocks;
  std::vector<double> residuals;
  std::size_t nodeCount = 0;
  std::size_t constraintCount = 0;
  std::size_t rowCount = 0;
  std::size_t bodyCount = 0;
  std::size_t dofCount = 0;
  std::size_t blockCount = 0;
  std::size_t blockEntryCount = 0;
  double maxConstraintResidualAbs = 0.0;
  double maxDiagonal = 0.0;
  double maxGradientAbs = 0.0;
  double maxBlockAbs = 0.0;
  NewtonSceneNonlinearEqualityAssemblyTiming timing;
};

struct NewtonSceneNonlinearEqualitySolveResult
{
  std::vector<NewtonAssemblySolveRowInput> rows;
  std::vector<NewtonSparseBlockEntry> blocks;
  std::vector<double> residuals;
  std::vector<double> step;
  std::vector<double> postSolveLinearizedResiduals;
  std::size_t nodeCount = 0;
  std::size_t constraintCount = 0;
  std::size_t rowCount = 0;
  std::size_t bodyCount = 0;
  std::size_t dofCount = 0;
  std::size_t blockCount = 0;
  std::size_t blockEntryCount = 0;
  std::size_t activeDofCount = 0;
  double regularization = 0.0;
  double maxConstraintResidualAbs = 0.0;
  double maxPostSolveLinearizedResidualAbs = 0.0;
  double maxDiagonal = 0.0;
  double maxGradientAbs = 0.0;
  double maxBlockAbs = 0.0;
  double stepNorm = 0.0;
  NewtonSceneNonlinearEqualitySolveTiming timing;
};

struct NewtonSceneNonlinearEqualityConvergenceResult
{
  std::vector<NewtonAssemblySolveRowInput> rows;
  std::vector<NewtonSparseBlockEntry> blocks;
  std::vector<double> initialResiduals;
  std::vector<double> finalResiduals;
  std::vector<double> step;
  std::size_t nodeCount = 0;
  std::size_t constraintCount = 0;
  std::size_t rowCount = 0;
  std::size_t bodyCount = 0;
  std::size_t dofCount = 0;
  std::size_t blockCount = 0;
  std::size_t blockEntryCount = 0;
  std::size_t maxIterationCount = 0;
  std::size_t completedIterationCount = 0;
  std::size_t activeDofCount = 0;
  double regularization = 0.0;
  double residualTolerance = 0.0;
  double initialMaxConstraintResidualAbs = 0.0;
  double finalMaxConstraintResidualAbs = 0.0;
  double maxDiagonal = 0.0;
  double maxGradientAbs = 0.0;
  double maxBlockAbs = 0.0;
  double stepNorm = 0.0;
  bool converged = false;
  NewtonSceneNonlinearEqualityConvergenceTiming timing;
};

struct NewtonSparseResidualResult
{
  std::vector<double> assembledDiagonal;
  std::vector<double> assembledGradient;
  std::vector<double> residual;
  std::size_t bodyCount = 0;
  std::size_t rowCount = 0;
  std::size_t dofCount = 0;
  std::size_t blockCount = 0;
  std::size_t activeDofCount = 0;
  double maxDiagonal = 0.0;
  double maxGradientAbs = 0.0;
  double residualNorm = 0.0;
  double maxResidualAbs = 0.0;
  NewtonSparseResidualTiming timing;
};

struct NewtonSparseJacobiSolveResult
{
  std::vector<double> assembledDiagonal;
  std::vector<double> assembledGradient;
  std::vector<double> step;
  std::vector<double> residual;
  std::size_t bodyCount = 0;
  std::size_t rowCount = 0;
  std::size_t dofCount = 0;
  std::size_t blockCount = 0;
  std::size_t iterationCount = 0;
  std::size_t activeDofCount = 0;
  double maxDiagonal = 0.0;
  double maxGradientAbs = 0.0;
  double stepNorm = 0.0;
  double residualNorm = 0.0;
  double maxResidualAbs = 0.0;
  NewtonSparseJacobiSolveTiming timing;
};

struct NewtonSparseCgSolveResult
{
  std::vector<double> assembledDiagonal;
  std::vector<double> assembledGradient;
  std::vector<double> step;
  std::vector<double> residual;
  std::size_t bodyCount = 0;
  std::size_t rowCount = 0;
  std::size_t dofCount = 0;
  std::size_t blockCount = 0;
  std::size_t maxIterationCount = 0;
  std::size_t completedIterationCount = 0;
  std::size_t activeDofCount = 0;
  double residualTolerance = 0.0;
  double initialResidualNorm = 0.0;
  double maxDiagonal = 0.0;
  double maxGradientAbs = 0.0;
  double stepNorm = 0.0;
  double residualNorm = 0.0;
  double maxResidualAbs = 0.0;
  bool converged = false;
  NewtonSparseCgSolveTiming timing;
};

struct NewtonDirectSparseSolveResult
{
  std::vector<double> assembledDiagonal;
  std::vector<double> assembledGradient;
  std::vector<double> factorMatrixLower;
  std::vector<double> step;
  std::vector<double> residual;
  std::size_t bodyCount = 0;
  std::size_t rowCount = 0;
  std::size_t dofCount = 0;
  std::size_t blockCount = 0;
  std::size_t activeDofCount = 0;
  double regularization = 0.0;
  double minimumFactorPivot = 0.0;
  double maxDiagonal = 0.0;
  double maxGradientAbs = 0.0;
  double stepNorm = 0.0;
  double residualNorm = 0.0;
  double maxResidualAbs = 0.0;
  NewtonDirectSparseSolveTiming timing;
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

/// Evaluate a private reduced scene sparse graph construction packet.
///
/// The packet consumes scene-owned node state and surface triangles, counts
/// triangle incidence, emits per-node diagonal Newton rows, and emits one 6x6
/// sparse block for each oriented surface edge. It intentionally does not cover
/// production sparse graph deduplication, global sparse factorization,
/// nonlinear equality constraints, runtime World::step integration, or a
/// public GPU solver backend.
void evaluateNewtonSceneSparseGraphAssemblyCuda(
    const std::vector<NewtonSceneNodeInput>& nodes,
    const std::vector<NewtonSceneSurfaceTriangleInput>& triangles,
    NewtonSceneSparseGraphAssemblyResult& result);

/// Evaluate a private reduced scene nonlinear equality assembly packet.
///
/// The packet consumes scene-owned node state and reduced distance-equality
/// constraints, linearizes each constraint at the current state, emits two
/// local Newton rows and one 6x6 sparse block per constraint. It intentionally
/// does not cover nonlinear constraint solving, production constraint graph
/// ownership, global sparse factorization, runtime World::step integration, or
/// a public GPU solver backend.
void evaluateNewtonSceneNonlinearEqualityAssemblyCuda(
    const std::vector<NewtonSceneNodeInput>& nodes,
    const std::vector<NewtonSceneDistanceEqualityConstraintInput>& constraints,
    NewtonSceneNonlinearEqualityAssemblyResult& result);

/// Evaluate a private reduced scene nonlinear equality solve packet.
///
/// The packet linearizes scene-owned distance-equality constraints, accumulates
/// one deterministic Jacobi-style correction step in scene-node generalized
/// coordinates, and reports post-step linearized residuals. It intentionally
/// does not cover production constraint graph ownership, direct or global
/// sparse factorization, nonlinear solve convergence policy, runtime
/// World::step integration, or a public GPU solver backend.
void evaluateNewtonSceneNonlinearEqualitySolveCuda(
    const std::vector<NewtonSceneNodeInput>& nodes,
    const std::vector<NewtonSceneDistanceEqualityConstraintInput>& constraints,
    double regularization,
    NewtonSceneNonlinearEqualitySolveResult& result);

/// Evaluate a private reduced scene nonlinear equality convergence packet.
///
/// The packet repeats the scene-owned distance-equality Jacobi-style
/// correction on a mutable reduced node state for a capped iteration count,
/// reports initial/final nonlinear residuals, and stops early when the reduced
/// residual tolerance is reached. It intentionally does not cover production
/// constraint graph ownership, direct or global sparse factorization,
/// production nonlinear solve policy, runtime World::step integration, or a
/// public GPU solver backend.
void evaluateNewtonSceneNonlinearEqualityConvergenceCuda(
    const std::vector<NewtonSceneNodeInput>& nodes,
    const std::vector<NewtonSceneDistanceEqualityConstraintInput>& constraints,
    double regularization,
    std::size_t maxIterationCount,
    double residualTolerance,
    NewtonSceneNonlinearEqualityConvergenceResult& result);

/// Evaluate a private sparse block residual packet.
///
/// The packet assembles full-space diagonal rows, applies symmetric 6x6
/// off-diagonal body blocks to a supplied step, and returns the regularized
/// sparse residual `H * step + gradient`. It intentionally does not cover
/// sparse graph construction, factorization, nonlinear equality constraints,
/// runtime scene assembly, or a public GPU solver backend.
void evaluateNewtonSparseResidualCuda(
    const std::vector<NewtonAssemblySolveRowInput>& rows,
    std::size_t bodyCount,
    const std::vector<NewtonSparseBlockEntry>& blocks,
    const std::vector<double>& step,
    double regularization,
    NewtonSparseResidualResult& result);

/// Evaluate a private fixed-iteration sparse Jacobi solve packet.
///
/// The packet assembles full-space diagonal rows and applies symmetric 6x6
/// sparse blocks while running a deterministic diagonal-preconditioned
/// residual update. It intentionally does not cover sparse graph construction,
/// direct factorization, nonlinear equality constraints, runtime scene
/// assembly, convergence policy, or a public GPU solver backend.
void evaluateNewtonSparseJacobiSolveCuda(
    const std::vector<NewtonAssemblySolveRowInput>& rows,
    std::size_t bodyCount,
    const std::vector<NewtonSparseBlockEntry>& blocks,
    std::size_t iterationCount,
    double regularization,
    NewtonSparseJacobiSolveResult& result);

/// Evaluate a private capped-iteration sparse CG solve packet.
///
/// The packet assembles full-space diagonal rows and applies symmetric 6x6
/// sparse blocks while running a deterministic capped Conjugate Gradient solve
/// for the reduced sparse system. It intentionally does not cover sparse graph
/// construction, direct factorization, nonlinear equality constraints, runtime
/// scene assembly, convergence policy for production solves, or a public GPU
/// solver backend.
void evaluateNewtonSparseCgSolveCuda(
    const std::vector<NewtonAssemblySolveRowInput>& rows,
    std::size_t bodyCount,
    const std::vector<NewtonSparseBlockEntry>& blocks,
    std::size_t maxIterationCount,
    double residualTolerance,
    double regularization,
    NewtonSparseCgSolveResult& result);

/// Evaluate a private bounded direct sparse solve packet.
///
/// The packet assembles full-space diagonal rows and symmetric 6x6 sparse
/// blocks into a small dense global matrix, runs an in-kernel Cholesky solve,
/// and checks the solved step with the sparse residual kernels. It
/// intentionally does not cover production sparse factorization, unbounded
/// systems, nonlinear equality constraints, runtime scene assembly, or a
/// public GPU solver backend.
void evaluateNewtonDirectSparseSolveCuda(
    const std::vector<NewtonAssemblySolveRowInput>& rows,
    std::size_t bodyCount,
    const std::vector<NewtonSparseBlockEntry>& blocks,
    double regularization,
    NewtonDirectSparseSolveResult& result);

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
