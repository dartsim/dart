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

#include <dart/simulation/body/deformable_body.hpp>
#include <dart/simulation/body/deformable_body_options.hpp>
#include <dart/simulation/world.hpp>

#include <Eigen/Core>
#include <benchmark/benchmark.h>
#include <dart/simulation/compute/cuda/cuda_runtime.cuh>
#include <dart/simulation/compute/cuda/newton_assembly_solve_cuda.cuh>

#include <algorithm>
#include <utility>
#include <vector>

#include <cmath>
#include <cstdint>

namespace cuda = dart::simulation::compute::cuda;
namespace sx = dart::simulation;

namespace {

constexpr double kRegularization = 0.25;
constexpr std::size_t kSparseJacobiIterations = 16;
constexpr std::size_t kSparseCgMaxIterations = 32;
constexpr double kSparseCgResidualTolerance = 1e-12;
constexpr std::size_t kDirectSparseBodyCount = 4;
constexpr std::size_t kSceneRuntimeDirectSparseSelectedNodeCount = 8;
constexpr double kSceneRuntimeAssemblyTimeStep = 1.0;
constexpr double kSceneNonlinearEqualitySolveRegularization = 0.05;
constexpr std::size_t kSceneNonlinearEqualityConvergenceMaxIterations = 8;
constexpr double kSceneNonlinearEqualityConvergenceResidualTolerance = 1e-5;
constexpr double kSceneNonlinearEqualityConvergenceStepScale = 0.25;

static_assert(
    kDirectSparseBodyCount * cuda::kNewtonAssemblySolveDofsPerBody
    <= cuda::kNewtonDirectSparseSolveMaxDofs);
static_assert(
    kSceneRuntimeDirectSparseSelectedNodeCount
        * cuda::kNewtonAssemblySolveDofsPerBody
    <= cuda::kNewtonDirectSparseSolveMaxDofs);

struct CpuAssemblySolveResult
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
};

struct AssemblySolveFixture
{
  std::vector<cuda::NewtonAssemblySolveRowInput> rows;
  std::size_t bodyCount = 0;
  CpuAssemblySolveResult cpu;
};

struct SceneRuntimeAssemblySolveFixture
{
  std::vector<cuda::NewtonAssemblySolveRowInput> rows;
  std::size_t bodyCount = 0;
  std::size_t sceneBodyCount = 0;
  std::size_t sceneNodeCount = 0;
  std::size_t sceneTriangleCount = 0;
  CpuAssemblySolveResult cpu;
};

struct CpuOffDiagonalAssemblyResult
{
  std::vector<double> assembledBlocks;
  std::size_t pairCount = 0;
  std::size_t rowCount = 0;
  std::size_t activeBlockCount = 0;
  double maxBlockAbs = 0.0;
};

struct OffDiagonalAssemblyFixture
{
  std::vector<cuda::NewtonOffDiagonalAssemblyRowInput> rows;
  std::size_t pairCount = 0;
  CpuOffDiagonalAssemblyResult cpu;
};

struct SceneRuntimeOffDiagonalAssemblyFixture : OffDiagonalAssemblyFixture
{
  std::size_t sceneBodyCount = 0;
  std::size_t sceneNodeCount = 0;
  std::size_t sceneTriangleCount = 0;
  std::size_t sceneEdgePairCount = 0;
};

struct CpuSceneSparseGraphAssemblyResult
{
  std::vector<cuda::NewtonAssemblySolveRowInput> rows;
  std::vector<cuda::NewtonSparseBlockEntry> blocks;
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
};

struct CpuSceneSparseGraphUniqueAssemblyResult
{
  std::vector<cuda::NewtonAssemblySolveRowInput> rows;
  std::vector<cuda::NewtonSparseBlockEntry> blocks;
  std::size_t nodeCount = 0;
  std::size_t triangleCount = 0;
  std::size_t edgeSlotCount = 0;
  std::size_t uniqueEdgeCount = 0;
  std::size_t duplicateEdgeSlotCount = 0;
  std::size_t rowCount = 0;
  std::size_t bodyCount = 0;
  std::size_t dofCount = 0;
  std::size_t blockCount = 0;
  std::size_t blockEntryCount = 0;
  double maxDiagonal = 0.0;
  double maxGradientAbs = 0.0;
  double maxBlockAbs = 0.0;
};

struct SceneRuntimeSparseGraphAssemblyFixture
{
  std::vector<cuda::NewtonSceneNodeInput> nodes;
  std::vector<cuda::NewtonSceneSurfaceTriangleInput> triangles;
  std::size_t sceneBodyCount = 0;
  CpuSceneSparseGraphAssemblyResult cpu;
};

struct SceneRuntimeSparseGraphUniqueAssemblyFixture
{
  std::vector<cuda::NewtonSceneNodeInput> nodes;
  std::vector<cuda::NewtonSceneSurfaceTriangleInput> triangles;
  std::size_t sceneBodyCount = 0;
  CpuSceneSparseGraphUniqueAssemblyResult cpu;
};

struct CpuSceneNonlinearEqualityAssemblyResult
{
  std::vector<cuda::NewtonAssemblySolveRowInput> rows;
  std::vector<cuda::NewtonSparseBlockEntry> blocks;
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
};

struct SceneRuntimeNonlinearEqualityAssemblyFixture
{
  std::vector<cuda::NewtonSceneNodeInput> nodes;
  std::vector<cuda::NewtonSceneDistanceEqualityConstraintInput> constraints;
  std::size_t sceneBodyCount = 0;
  std::size_t sceneTriangleCount = 0;
  std::size_t sceneEdgePairCount = 0;
  CpuSceneNonlinearEqualityAssemblyResult cpu;
};

struct CpuSceneNonlinearEqualitySolveResult
{
  std::vector<cuda::NewtonAssemblySolveRowInput> rows;
  std::vector<cuda::NewtonSparseBlockEntry> blocks;
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
  double maxConstraintResidualAbs = 0.0;
  double maxPostSolveLinearizedResidualAbs = 0.0;
  double maxDiagonal = 0.0;
  double maxGradientAbs = 0.0;
  double maxBlockAbs = 0.0;
  double stepNorm = 0.0;
};

struct CpuSceneNonlinearEqualityConvergenceResult
{
  std::vector<cuda::NewtonAssemblySolveRowInput> rows;
  std::vector<cuda::NewtonSparseBlockEntry> blocks;
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
  double initialMaxConstraintResidualAbs = 0.0;
  double finalMaxConstraintResidualAbs = 0.0;
  double maxDiagonal = 0.0;
  double maxGradientAbs = 0.0;
  double maxBlockAbs = 0.0;
  double stepNorm = 0.0;
  bool converged = false;
};

struct SceneRuntimeNonlinearEqualitySolveFixture
{
  std::vector<cuda::NewtonSceneNodeInput> nodes;
  std::vector<cuda::NewtonSceneDistanceEqualityConstraintInput> constraints;
  std::size_t sceneBodyCount = 0;
  std::size_t sceneTriangleCount = 0;
  std::size_t sceneEdgePairCount = 0;
  CpuSceneNonlinearEqualitySolveResult cpu;
};

struct SceneRuntimeNonlinearEqualityConvergenceFixture
{
  std::vector<cuda::NewtonSceneNodeInput> nodes;
  std::vector<cuda::NewtonSceneDistanceEqualityConstraintInput> constraints;
  std::size_t sceneBodyCount = 0;
  std::size_t sceneTriangleCount = 0;
  std::size_t sceneEdgePairCount = 0;
  CpuSceneNonlinearEqualityConvergenceResult cpu;
};

struct CpuSparseResidualResult
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
};

struct SparseResidualFixture
{
  std::vector<cuda::NewtonAssemblySolveRowInput> rows;
  std::vector<cuda::NewtonSparseBlockEntry> blocks;
  std::vector<double> step;
  std::size_t bodyCount = 0;
  CpuSparseResidualResult cpu;
};

struct SceneRuntimeSparseResidualFixture : SparseResidualFixture
{
  std::size_t sceneBodyCount = 0;
  std::size_t sceneNodeCount = 0;
  std::size_t sceneTriangleCount = 0;
  std::size_t sceneEdgePairCount = 0;
};

struct CpuSparseJacobiSolveResult
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
};

struct SparseJacobiSolveFixture
{
  std::vector<cuda::NewtonAssemblySolveRowInput> rows;
  std::vector<cuda::NewtonSparseBlockEntry> blocks;
  std::size_t bodyCount = 0;
  CpuSparseJacobiSolveResult cpu;
};

struct SceneRuntimeSparseJacobiSolveFixture : SparseJacobiSolveFixture
{
  std::size_t sceneBodyCount = 0;
  std::size_t sceneNodeCount = 0;
  std::size_t sceneTriangleCount = 0;
  std::size_t sceneEdgePairCount = 0;
};

struct CpuSparseCgSolveResult
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
};

struct SparseCgSolveFixture
{
  std::vector<cuda::NewtonAssemblySolveRowInput> rows;
  std::vector<cuda::NewtonSparseBlockEntry> blocks;
  std::size_t bodyCount = 0;
  CpuSparseCgSolveResult cpu;
};

struct SceneRuntimeSparseCgSolveFixture : SparseCgSolveFixture
{
  std::size_t sceneBodyCount = 0;
  std::size_t sceneNodeCount = 0;
  std::size_t sceneTriangleCount = 0;
  std::size_t sceneEdgePairCount = 0;
};

struct CpuDirectSparseSolveResult
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
  double minimumFactorPivot = 0.0;
  double maxDiagonal = 0.0;
  double maxGradientAbs = 0.0;
  double stepNorm = 0.0;
  double residualNorm = 0.0;
  double maxResidualAbs = 0.0;
  bool factorized = false;
};

struct DirectSparseSolveFixture
{
  std::vector<cuda::NewtonAssemblySolveRowInput> rows;
  std::vector<cuda::NewtonSparseBlockEntry> blocks;
  std::size_t bodyCount = 0;
  CpuDirectSparseSolveResult cpu;
};

struct SceneRuntimeDirectSparseSolveFixture : DirectSparseSolveFixture
{
  std::size_t sceneBodyCount = 0;
  std::size_t sceneNodeCount = 0;
  std::size_t sceneTriangleCount = 0;
  std::size_t selectedSceneNodeCount = 0;
  std::size_t selectedSceneEdgePairCount = 0;
};

struct CpuEqualityReducedSolveResult
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
};

struct EqualityReducedSolveFixture
{
  std::vector<cuda::NewtonAssemblySolveRowInput> rows;
  std::vector<cuda::NewtonEqualityReductionEntry> entries;
  std::size_t bodyCount = 0;
  std::size_t reducedDofCount = 0;
  CpuEqualityReducedSolveResult cpu;
};

struct SceneRuntimeEqualityReducedSolveFixture : EqualityReducedSolveFixture
{
  std::size_t sceneBodyCount = 0;
  std::size_t sceneNodeCount = 0;
  std::size_t sceneTriangleCount = 0;
};

cuda::NewtonAssemblySolveRowInput makeRow(
    const int rowIndex, const std::size_t bodyCount)
{
  cuda::NewtonAssemblySolveRowInput row;
  row.bodyIndex = static_cast<std::uint32_t>(
      static_cast<std::size_t>(rowIndex) % bodyCount);
  for (std::size_t dof = 0; dof < cuda::kNewtonAssemblySolveDofsPerBody;
       ++dof) {
    const auto signedBucket = static_cast<int>((rowIndex + 3 * dof) % 23) - 11;
    row.hessianDiagonal[dof]
        = 0.5 + 0.01 * static_cast<double>((rowIndex + dof) % 17)
          + 0.05 * static_cast<double>(dof + 1);
    row.gradient[dof] = 0.025 * static_cast<double>(signedBucket);
  }
  return row;
}

cuda::NewtonOffDiagonalAssemblyRowInput makeOffDiagonalRow(
    const int rowIndex, const std::size_t pairCount)
{
  cuda::NewtonOffDiagonalAssemblyRowInput row;
  row.pairIndex = static_cast<std::uint32_t>(
      static_cast<std::size_t>(rowIndex) % pairCount);
  for (std::size_t entry = 0; entry < cuda::kNewtonAssemblySolveBlockEntries;
       ++entry) {
    const auto signedBucket
        = static_cast<int>((rowIndex + 7 * entry) % 31) - 15;
    row.hessianBlock[entry] = 0.0025 * static_cast<double>(signedBucket)
                              + 0.0001 * static_cast<double>((entry % 6) + 1);
  }
  return row;
}

cuda::NewtonOffDiagonalAssemblyRowInput makeSceneRuntimeOffDiagonalRow(
    const sx::DeformableBody& body,
    const std::size_t pairIndex,
    const std::size_t nodeA,
    const std::size_t nodeB)
{
  cuda::NewtonOffDiagonalAssemblyRowInput row;
  row.pairIndex = static_cast<std::uint32_t>(pairIndex);

  const Eigen::Vector3d delta
      = body.getPosition(nodeB) - body.getPosition(nodeA);
  const Eigen::Vector3d velocityDelta
      = body.getVelocity(nodeB) - body.getVelocity(nodeA);
  const double length = std::max(delta.norm(), 1e-9);
  const double massScale = 0.5 * (body.getMass(nodeA) + body.getMass(nodeB));

  for (std::size_t localRow = 0;
       localRow < cuda::kNewtonAssemblySolveDofsPerBody;
       ++localRow) {
    const std::size_t rowAxis = localRow % 3u;
    for (std::size_t localColumn = 0;
         localColumn < cuda::kNewtonAssemblySolveDofsPerBody;
         ++localColumn) {
      const std::size_t columnAxis = localColumn % 3u;
      const double axisCoupling
          = delta[rowAxis] * delta[columnAxis] / (length * length);
      const double velocityCoupling
          = velocityDelta[rowAxis] * velocityDelta[columnAxis];
      const double diagonalBias = rowAxis == columnAxis ? 0.001 : 0.0001;
      row.hessianBlock
          [localRow * cuda::kNewtonAssemblySolveDofsPerBody + localColumn]
          = diagonalBias * massScale + 0.00025 * axisCoupling
            + 0.00005 * velocityCoupling
            + 0.00001 * static_cast<double>(localRow + localColumn + 1u);
    }
  }

  return row;
}

cuda::NewtonSparseBlockEntry makeSparseBlock(
    const int blockIndex, const std::size_t bodyCount)
{
  cuda::NewtonSparseBlockEntry block;
  const std::size_t rowBody = static_cast<std::size_t>(blockIndex) % bodyCount;
  block.rowBodyIndex = static_cast<std::uint32_t>(rowBody);
  block.columnBodyIndex = static_cast<std::uint32_t>((rowBody + 1) % bodyCount);
  for (std::size_t entry = 0; entry < cuda::kNewtonAssemblySolveBlockEntries;
       ++entry) {
    const auto signedBucket
        = static_cast<int>((blockIndex + 5 * entry) % 37) - 18;
    block.hessianBlock[entry]
        = 0.0015 * static_cast<double>(signedBucket)
          + 0.00005 * static_cast<double>((entry % 6) + 1);
  }
  return block;
}

cuda::NewtonSparseBlockEntry makeSceneRuntimeSparseBlock(
    const sx::DeformableBody& body,
    const std::size_t nodeA,
    const std::size_t nodeB)
{
  cuda::NewtonSparseBlockEntry block;
  block.rowBodyIndex = static_cast<std::uint32_t>(nodeA);
  block.columnBodyIndex = static_cast<std::uint32_t>(nodeB);

  const auto row = makeSceneRuntimeOffDiagonalRow(body, 0u, nodeA, nodeB);
  for (std::size_t entry = 0; entry < cuda::kNewtonAssemblySolveBlockEntries;
       ++entry) {
    block.hessianBlock[entry] = row.hessianBlock[entry];
  }
  return block;
}

cuda::NewtonSceneNodeInput makeSceneRuntimeNodeInput(
    const sx::DeformableBody& body, const std::size_t node)
{
  cuda::NewtonSceneNodeInput input;
  const Eigen::Vector3d position = body.getPosition(node);
  const Eigen::Vector3d velocity = body.getVelocity(node);
  for (std::size_t axis = 0; axis < 3u; ++axis) {
    input.position[axis] = position[axis];
    input.velocity[axis] = velocity[axis];
  }
  input.mass = body.getMass(node);
  return input;
}

cuda::NewtonSceneSurfaceTriangleInput makeSceneRuntimeTriangleInput(
    const sx::DeformableBody& body, const std::size_t triangle)
{
  const auto surfaceTriangle = body.getSurfaceTriangle(triangle);
  return {
      static_cast<std::uint32_t>(surfaceTriangle.nodeA),
      static_cast<std::uint32_t>(surfaceTriangle.nodeB),
      static_cast<std::uint32_t>(surfaceTriangle.nodeC),
  };
}

cuda::NewtonSceneDistanceEqualityConstraintInput
makeSceneRuntimeDistanceEqualityConstraint(
    const std::vector<cuda::NewtonSceneNodeInput>& nodes,
    const std::size_t nodeA,
    const std::size_t nodeB,
    const std::size_t constraintIndex)
{
  double lengthSquared = 0.0;
  for (std::size_t axis = 0; axis < 3u; ++axis) {
    const double delta
        = nodes[nodeB].position[axis] - nodes[nodeA].position[axis];
    lengthSquared += delta * delta;
  }
  const double length = std::max(std::sqrt(lengthSquared), 1e-9);

  cuda::NewtonSceneDistanceEqualityConstraintInput constraint;
  constraint.nodeA = static_cast<std::uint32_t>(nodeA);
  constraint.nodeB = static_cast<std::uint32_t>(nodeB);
  constraint.restLength
      = length * (0.985 + 0.001 * static_cast<double>(constraintIndex % 5u));
  constraint.stiffness
      = 1.25 + 0.05 * static_cast<double>(constraintIndex % 7u);
  constraint.damping = 0.10 + 0.02 * static_cast<double>(constraintIndex % 3u);
  return constraint;
}

cuda::NewtonAssemblySolveRowInput makeSceneRuntimeAssemblyRow(
    const cuda::NewtonSceneNodeInput& node,
    const std::size_t nodeIndex,
    const std::size_t incidentTriangleCount)
{
  cuda::NewtonAssemblySolveRowInput row;
  row.bodyIndex = static_cast<std::uint32_t>(nodeIndex);

  const double radius = std::sqrt(
      node.position[0] * node.position[0] + node.position[1] * node.position[1]
      + node.position[2] * node.position[2]);
  const double incidentScale = static_cast<double>(incidentTriangleCount);

  for (std::size_t dof = 0; dof < cuda::kNewtonAssemblySolveDofsPerBody;
       ++dof) {
    const std::size_t axis = dof % 3u;
    const double axisPosition = node.position[axis];
    const double axisVelocity = node.velocity[axis];
    if (dof < 3u) {
      row.hessianDiagonal[dof]
          = node.mass + 0.20 * static_cast<double>(axis + 1u)
            + 0.05 * incidentScale + 0.01 * std::abs(axisPosition);
      row.gradient[dof] = 0.08 * axisVelocity + 0.03 * axisPosition
                          + 0.01 * static_cast<double>(axis + 1u);
    } else {
      row.hessianDiagonal[dof] = 0.50 + 0.10 * node.mass
                                 + 0.05 * static_cast<double>(axis + 1u)
                                 + 0.025 * incidentScale + 0.005 * radius;
      row.gradient[dof] = 0.015 * static_cast<double>(axis + 1u) * axisPosition
                          - 0.02 * axisVelocity;
    }
  }

  return row;
}

cuda::NewtonSparseBlockEntry makeSceneRuntimeSparseBlock(
    const std::vector<cuda::NewtonSceneNodeInput>& nodes,
    const std::size_t nodeA,
    const std::size_t nodeB)
{
  cuda::NewtonSparseBlockEntry block;
  block.rowBodyIndex = static_cast<std::uint32_t>(nodeA);
  block.columnBodyIndex = static_cast<std::uint32_t>(nodeB);

  double delta[3] = {};
  double velocityDelta[3] = {};
  double lengthSquared = 0.0;
  for (std::size_t axis = 0; axis < 3u; ++axis) {
    delta[axis] = nodes[nodeB].position[axis] - nodes[nodeA].position[axis];
    velocityDelta[axis]
        = nodes[nodeB].velocity[axis] - nodes[nodeA].velocity[axis];
    lengthSquared += delta[axis] * delta[axis];
  }
  const double length = std::max(std::sqrt(lengthSquared), 1e-9);
  const double massScale = 0.5 * (nodes[nodeA].mass + nodes[nodeB].mass);

  for (std::size_t localRow = 0;
       localRow < cuda::kNewtonAssemblySolveDofsPerBody;
       ++localRow) {
    const std::size_t rowAxis = localRow % 3u;
    for (std::size_t localColumn = 0;
         localColumn < cuda::kNewtonAssemblySolveDofsPerBody;
         ++localColumn) {
      const std::size_t columnAxis = localColumn % 3u;
      const double axisCoupling
          = delta[rowAxis] * delta[columnAxis] / (length * length);
      const double velocityCoupling
          = velocityDelta[rowAxis] * velocityDelta[columnAxis];
      const double diagonalBias = rowAxis == columnAxis ? 0.001 : 0.0001;
      const std::size_t entry
          = localRow * cuda::kNewtonAssemblySolveDofsPerBody + localColumn;
      block.hessianBlock[entry]
          = diagonalBias * massScale + 0.00025 * axisCoupling
            + 0.00005 * velocityCoupling
            + 0.00001 * static_cast<double>(localRow + localColumn + 1u);
    }
  }

  return block;
}

void makeSceneRuntimeDistanceEqualityBasis(
    const cuda::NewtonSceneNodeInput& node,
    const double normal[3],
    const double translationalSign,
    double basis[cuda::kNewtonAssemblySolveDofsPerBody])
{
  for (std::size_t axis = 0; axis < 3u; ++axis) {
    basis[axis] = translationalSign * normal[axis];
  }

  const double rotationalBasis[3] = {
      node.position[1] * normal[2] - node.position[2] * normal[1],
      node.position[2] * normal[0] - node.position[0] * normal[2],
      node.position[0] * normal[1] - node.position[1] * normal[0],
  };
  for (std::size_t axis = 0; axis < 3u; ++axis) {
    basis[axis + 3u] = 0.10 * translationalSign * rotationalBasis[axis];
  }
}

void appendSceneRuntimeDistanceEqualityContribution(
    const std::vector<cuda::NewtonSceneNodeInput>& nodes,
    const cuda::NewtonSceneDistanceEqualityConstraintInput& constraint,
    CpuSceneNonlinearEqualityAssemblyResult& result)
{
  const auto& nodeA = nodes[constraint.nodeA];
  const auto& nodeB = nodes[constraint.nodeB];

  double delta[3] = {};
  double velocityDelta[3] = {};
  double lengthSquared = 0.0;
  for (std::size_t axis = 0; axis < 3u; ++axis) {
    delta[axis] = nodeB.position[axis] - nodeA.position[axis];
    velocityDelta[axis] = nodeB.velocity[axis] - nodeA.velocity[axis];
    lengthSquared += delta[axis] * delta[axis];
  }

  const double length = std::max(std::sqrt(lengthSquared), 1e-9);
  double normal[3] = {};
  double normalVelocity = 0.0;
  for (std::size_t axis = 0; axis < 3u; ++axis) {
    normal[axis] = delta[axis] / length;
    normalVelocity += normal[axis] * velocityDelta[axis];
  }

  const double residual = length - constraint.restLength;
  const double scalarGradient
      = constraint.stiffness * residual + constraint.damping * normalVelocity;
  result.residuals.push_back(residual);

  double basisA[cuda::kNewtonAssemblySolveDofsPerBody] = {};
  double basisB[cuda::kNewtonAssemblySolveDofsPerBody] = {};
  makeSceneRuntimeDistanceEqualityBasis(nodeA, normal, -1.0, basisA);
  makeSceneRuntimeDistanceEqualityBasis(nodeB, normal, 1.0, basisB);

  cuda::NewtonAssemblySolveRowInput rowA;
  cuda::NewtonAssemblySolveRowInput rowB;
  rowA.bodyIndex = constraint.nodeA;
  rowB.bodyIndex = constraint.nodeB;
  cuda::NewtonSparseBlockEntry block;
  block.rowBodyIndex = constraint.nodeA;
  block.columnBodyIndex = constraint.nodeB;

  for (std::size_t dof = 0; dof < cuda::kNewtonAssemblySolveDofsPerBody;
       ++dof) {
    const double dampingBias
        = 0.001 * constraint.damping * static_cast<double>(dof + 1u);
    rowA.hessianDiagonal[dof]
        = constraint.stiffness * basisA[dof] * basisA[dof] + dampingBias;
    rowB.hessianDiagonal[dof]
        = constraint.stiffness * basisB[dof] * basisB[dof] + dampingBias;
    rowA.gradient[dof] = scalarGradient * basisA[dof];
    rowB.gradient[dof] = scalarGradient * basisB[dof];
  }

  for (std::size_t localRow = 0;
       localRow < cuda::kNewtonAssemblySolveDofsPerBody;
       ++localRow) {
    for (std::size_t localColumn = 0;
         localColumn < cuda::kNewtonAssemblySolveDofsPerBody;
         ++localColumn) {
      const std::size_t entry
          = localRow * cuda::kNewtonAssemblySolveDofsPerBody + localColumn;
      block.hessianBlock[entry]
          = constraint.stiffness * basisA[localRow] * basisB[localColumn];
    }
  }

  result.rows.push_back(rowA);
  result.rows.push_back(rowB);
  result.blocks.push_back(block);
}

std::vector<double> makeSparseStep(const std::size_t dofCount)
{
  std::vector<double> step(dofCount, 0.0);
  for (std::size_t dof = 0; dof < dofCount; ++dof) {
    const auto signedBucket = static_cast<int>((dof * 7) % 29) - 14;
    step[dof] = 0.015 * static_cast<double>(signedBucket);
  }
  return step;
}

std::vector<double> makeSceneRuntimeSparseStep(const sx::DeformableBody& body)
{
  std::vector<double> step(
      body.getNodeCount() * cuda::kNewtonAssemblySolveDofsPerBody, 0.0);
  for (std::size_t node = 0; node < body.getNodeCount(); ++node) {
    const Eigen::Vector3d position = body.getPosition(node);
    const Eigen::Vector3d velocity = body.getVelocity(node);
    const std::size_t offset = node * cuda::kNewtonAssemblySolveDofsPerBody;
    for (std::size_t axis = 0; axis < 3u; ++axis) {
      step[offset + axis] = 0.012 * velocity[axis] + 0.004 * position[axis];
      step[offset + 3u + axis] = 0.003 * position[axis] - 0.002 * velocity[axis]
                                 + 0.0005 * static_cast<double>(axis + 1u);
    }
  }
  return step;
}

cuda::NewtonAssemblySolveRowInput makeSceneRuntimeAssemblyRow(
    const sx::DeformableBody& body,
    std::size_t nodeIndex,
    std::size_t incidentTriangleCount);

void evaluateCpuSceneSparseGraphAssembly(
    const std::vector<cuda::NewtonSceneNodeInput>& nodes,
    const std::vector<cuda::NewtonSceneSurfaceTriangleInput>& triangles,
    CpuSceneSparseGraphAssemblyResult& result)
{
  result = CpuSceneSparseGraphAssemblyResult{};
  result.nodeCount = nodes.size();
  result.triangleCount = triangles.size();
  result.rowCount = nodes.size();
  result.bodyCount = nodes.size();
  result.dofCount = nodes.size() * cuda::kNewtonAssemblySolveDofsPerBody;
  result.blockCount = 3u * triangles.size();
  result.blockEntryCount
      = result.blockCount * cuda::kNewtonAssemblySolveBlockEntries;

  std::vector<std::size_t> incidentTriangleCounts(nodes.size(), 0u);
  for (const auto& triangle : triangles) {
    ++incidentTriangleCounts[triangle.nodeA];
    ++incidentTriangleCounts[triangle.nodeB];
    ++incidentTriangleCounts[triangle.nodeC];
  }

  result.rows.reserve(nodes.size());
  for (std::size_t node = 0; node < nodes.size(); ++node) {
    result.rows.push_back(makeSceneRuntimeAssemblyRow(
        nodes[node], node, incidentTriangleCounts[node]));
  }

  result.blocks.reserve(result.blockCount);
  for (const auto& triangle : triangles) {
    result.blocks.push_back(
        makeSceneRuntimeSparseBlock(nodes, triangle.nodeA, triangle.nodeB));
    result.blocks.push_back(
        makeSceneRuntimeSparseBlock(nodes, triangle.nodeB, triangle.nodeC));
    result.blocks.push_back(
        makeSceneRuntimeSparseBlock(nodes, triangle.nodeC, triangle.nodeA));
  }

  for (const auto& row : result.rows) {
    for (std::size_t dof = 0; dof < cuda::kNewtonAssemblySolveDofsPerBody;
         ++dof) {
      result.maxDiagonal
          = std::max(result.maxDiagonal, row.hessianDiagonal[dof]);
      result.maxGradientAbs
          = std::max(result.maxGradientAbs, std::abs(row.gradient[dof]));
    }
  }
  for (const auto& block : result.blocks) {
    for (std::size_t entry = 0; entry < cuda::kNewtonAssemblySolveBlockEntries;
         ++entry) {
      result.maxBlockAbs
          = std::max(result.maxBlockAbs, std::abs(block.hessianBlock[entry]));
    }
  }
}

void evaluateCpuSceneSparseGraphUniqueAssembly(
    const std::vector<cuda::NewtonSceneNodeInput>& nodes,
    const std::vector<cuda::NewtonSceneSurfaceTriangleInput>& triangles,
    CpuSceneSparseGraphUniqueAssemblyResult& result)
{
  result = CpuSceneSparseGraphUniqueAssemblyResult{};
  result.nodeCount = nodes.size();
  result.triangleCount = triangles.size();
  result.edgeSlotCount = 3u * triangles.size();
  result.rowCount = nodes.size();
  result.bodyCount = nodes.size();
  result.dofCount = nodes.size() * cuda::kNewtonAssemblySolveDofsPerBody;

  std::vector<std::size_t> incidentTriangleCounts(nodes.size(), 0u);
  std::vector<std::pair<std::uint32_t, std::uint32_t>> edges;
  edges.reserve(result.edgeSlotCount);
  const auto appendEdge = [&edges](std::uint32_t nodeA, std::uint32_t nodeB) {
    if (nodeB < nodeA) {
      std::swap(nodeA, nodeB);
    }
    edges.emplace_back(nodeA, nodeB);
  };
  for (const auto& triangle : triangles) {
    ++incidentTriangleCounts[triangle.nodeA];
    ++incidentTriangleCounts[triangle.nodeB];
    ++incidentTriangleCounts[triangle.nodeC];
    appendEdge(triangle.nodeA, triangle.nodeB);
    appendEdge(triangle.nodeB, triangle.nodeC);
    appendEdge(triangle.nodeC, triangle.nodeA);
  }
  std::sort(edges.begin(), edges.end());
  edges.erase(std::unique(edges.begin(), edges.end()), edges.end());
  result.uniqueEdgeCount = edges.size();
  result.duplicateEdgeSlotCount = result.edgeSlotCount - result.uniqueEdgeCount;
  result.blockCount = result.uniqueEdgeCount;
  result.blockEntryCount
      = result.blockCount * cuda::kNewtonAssemblySolveBlockEntries;

  result.rows.reserve(nodes.size());
  for (std::size_t node = 0; node < nodes.size(); ++node) {
    result.rows.push_back(makeSceneRuntimeAssemblyRow(
        nodes[node], node, incidentTriangleCounts[node]));
  }

  result.blocks.reserve(result.blockCount);
  for (const auto& [nodeA, nodeB] : edges) {
    result.blocks.push_back(makeSceneRuntimeSparseBlock(nodes, nodeA, nodeB));
  }

  for (const auto& row : result.rows) {
    for (std::size_t dof = 0; dof < cuda::kNewtonAssemblySolveDofsPerBody;
         ++dof) {
      result.maxDiagonal
          = std::max(result.maxDiagonal, row.hessianDiagonal[dof]);
      result.maxGradientAbs
          = std::max(result.maxGradientAbs, std::abs(row.gradient[dof]));
    }
  }
  for (const auto& block : result.blocks) {
    for (std::size_t entry = 0; entry < cuda::kNewtonAssemblySolveBlockEntries;
         ++entry) {
      result.maxBlockAbs
          = std::max(result.maxBlockAbs, std::abs(block.hessianBlock[entry]));
    }
  }
}

void evaluateCpuSceneNonlinearEqualityAssembly(
    const std::vector<cuda::NewtonSceneNodeInput>& nodes,
    const std::vector<cuda::NewtonSceneDistanceEqualityConstraintInput>&
        constraints,
    CpuSceneNonlinearEqualityAssemblyResult& result)
{
  result = CpuSceneNonlinearEqualityAssemblyResult{};
  result.nodeCount = nodes.size();
  result.constraintCount = constraints.size();
  result.rowCount = 2u * constraints.size();
  result.bodyCount = nodes.size();
  result.dofCount = nodes.size() * cuda::kNewtonAssemblySolveDofsPerBody;
  result.blockCount = constraints.size();
  result.blockEntryCount
      = result.blockCount * cuda::kNewtonAssemblySolveBlockEntries;
  result.rows.reserve(result.rowCount);
  result.blocks.reserve(result.blockCount);
  result.residuals.reserve(result.constraintCount);

  for (const auto& constraint : constraints) {
    appendSceneRuntimeDistanceEqualityContribution(nodes, constraint, result);
  }

  for (const auto residual : result.residuals) {
    result.maxConstraintResidualAbs
        = std::max(result.maxConstraintResidualAbs, std::abs(residual));
  }
  for (const auto& row : result.rows) {
    for (std::size_t dof = 0; dof < cuda::kNewtonAssemblySolveDofsPerBody;
         ++dof) {
      result.maxDiagonal
          = std::max(result.maxDiagonal, row.hessianDiagonal[dof]);
      result.maxGradientAbs
          = std::max(result.maxGradientAbs, std::abs(row.gradient[dof]));
    }
  }
  for (const auto& block : result.blocks) {
    for (std::size_t entry = 0; entry < cuda::kNewtonAssemblySolveBlockEntries;
         ++entry) {
      result.maxBlockAbs
          = std::max(result.maxBlockAbs, std::abs(block.hessianBlock[entry]));
    }
  }
}

void evaluateCpuSceneNonlinearEqualitySolve(
    const std::vector<cuda::NewtonSceneNodeInput>& nodes,
    const std::vector<cuda::NewtonSceneDistanceEqualityConstraintInput>&
        constraints,
    const double regularization,
    CpuSceneNonlinearEqualitySolveResult& result)
{
  CpuSceneNonlinearEqualityAssemblyResult assembly;
  evaluateCpuSceneNonlinearEqualityAssembly(nodes, constraints, assembly);

  result = CpuSceneNonlinearEqualitySolveResult{};
  result.rows = std::move(assembly.rows);
  result.blocks = std::move(assembly.blocks);
  result.residuals = std::move(assembly.residuals);
  result.nodeCount = assembly.nodeCount;
  result.constraintCount = assembly.constraintCount;
  result.rowCount = assembly.rowCount;
  result.bodyCount = assembly.bodyCount;
  result.dofCount = assembly.dofCount;
  result.blockCount = assembly.blockCount;
  result.blockEntryCount = assembly.blockEntryCount;
  result.maxConstraintResidualAbs = assembly.maxConstraintResidualAbs;
  result.maxDiagonal = assembly.maxDiagonal;
  result.maxGradientAbs = assembly.maxGradientAbs;
  result.maxBlockAbs = assembly.maxBlockAbs;
  result.step.assign(result.dofCount, 0.0);
  result.postSolveLinearizedResiduals.assign(result.constraintCount, 0.0);

  for (std::size_t constraintIndex = 0; constraintIndex < constraints.size();
       ++constraintIndex) {
    const auto& constraint = constraints[constraintIndex];
    const auto& nodeA = nodes[constraint.nodeA];
    const auto& nodeB = nodes[constraint.nodeB];

    double delta[3] = {};
    double velocityDelta[3] = {};
    double lengthSquared = 0.0;
    for (std::size_t axis = 0; axis < 3u; ++axis) {
      delta[axis] = nodeB.position[axis] - nodeA.position[axis];
      velocityDelta[axis] = nodeB.velocity[axis] - nodeA.velocity[axis];
      lengthSquared += delta[axis] * delta[axis];
    }

    const double length = std::max(std::sqrt(lengthSquared), 1e-9);
    double normal[3] = {};
    double normalVelocity = 0.0;
    for (std::size_t axis = 0; axis < 3u; ++axis) {
      normal[axis] = delta[axis] / length;
      normalVelocity += normal[axis] * velocityDelta[axis];
    }

    const double residual = length - constraint.restLength;
    const double scalarGradient
        = constraint.stiffness * residual + constraint.damping * normalVelocity;

    double basisA[cuda::kNewtonAssemblySolveDofsPerBody] = {};
    double basisB[cuda::kNewtonAssemblySolveDofsPerBody] = {};
    makeSceneRuntimeDistanceEqualityBasis(nodeA, normal, -1.0, basisA);
    makeSceneRuntimeDistanceEqualityBasis(nodeB, normal, 1.0, basisB);

    const std::size_t rowOffset = 2u * constraintIndex;
    double diagonalSum = regularization;
    for (std::size_t dof = 0; dof < cuda::kNewtonAssemblySolveDofsPerBody;
         ++dof) {
      diagonalSum += result.rows[rowOffset].hessianDiagonal[dof]
                     + result.rows[rowOffset + 1u].hessianDiagonal[dof];
    }
    const double lambda = -scalarGradient / std::max(diagonalSum, 1e-14);

    const std::size_t stepAOffset = static_cast<std::size_t>(constraint.nodeA)
                                    * cuda::kNewtonAssemblySolveDofsPerBody;
    const std::size_t stepBOffset = static_cast<std::size_t>(constraint.nodeB)
                                    * cuda::kNewtonAssemblySolveDofsPerBody;
    for (std::size_t dof = 0; dof < cuda::kNewtonAssemblySolveDofsPerBody;
         ++dof) {
      result.step[stepAOffset + dof] += lambda * basisA[dof];
      result.step[stepBOffset + dof] += lambda * basisB[dof];
    }
  }

  for (std::size_t constraintIndex = 0; constraintIndex < constraints.size();
       ++constraintIndex) {
    const auto& constraint = constraints[constraintIndex];
    const auto& nodeA = nodes[constraint.nodeA];
    const auto& nodeB = nodes[constraint.nodeB];

    double delta[3] = {};
    double lengthSquared = 0.0;
    for (std::size_t axis = 0; axis < 3u; ++axis) {
      delta[axis] = nodeB.position[axis] - nodeA.position[axis];
      lengthSquared += delta[axis] * delta[axis];
    }

    const double length = std::max(std::sqrt(lengthSquared), 1e-9);
    double normal[3] = {};
    for (std::size_t axis = 0; axis < 3u; ++axis) {
      normal[axis] = delta[axis] / length;
    }

    double basisA[cuda::kNewtonAssemblySolveDofsPerBody] = {};
    double basisB[cuda::kNewtonAssemblySolveDofsPerBody] = {};
    makeSceneRuntimeDistanceEqualityBasis(nodeA, normal, -1.0, basisA);
    makeSceneRuntimeDistanceEqualityBasis(nodeB, normal, 1.0, basisB);

    const std::size_t stepAOffset = static_cast<std::size_t>(constraint.nodeA)
                                    * cuda::kNewtonAssemblySolveDofsPerBody;
    const std::size_t stepBOffset = static_cast<std::size_t>(constraint.nodeB)
                                    * cuda::kNewtonAssemblySolveDofsPerBody;
    double linearizedCorrection = 0.0;
    for (std::size_t dof = 0; dof < cuda::kNewtonAssemblySolveDofsPerBody;
         ++dof) {
      linearizedCorrection += basisA[dof] * result.step[stepAOffset + dof];
      linearizedCorrection += basisB[dof] * result.step[stepBOffset + dof];
    }
    result.postSolveLinearizedResiduals[constraintIndex]
        = result.residuals[constraintIndex] + linearizedCorrection;
    result.maxPostSolveLinearizedResidualAbs = std::max(
        result.maxPostSolveLinearizedResidualAbs,
        std::abs(result.postSolveLinearizedResiduals[constraintIndex]));
  }

  double stepNormSquared = 0.0;
  for (const auto value : result.step) {
    if (std::abs(value) > 0.0) {
      ++result.activeDofCount;
    }
    stepNormSquared += value * value;
  }
  result.stepNorm = std::sqrt(stepNormSquared);
}

double maxResidualAbs(const std::vector<double>& residuals)
{
  double value = 0.0;
  for (const auto residual : residuals) {
    value = std::max(value, std::abs(residual));
  }
  return value;
}

void evaluateCpuSceneNonlinearEqualityConvergence(
    const std::vector<cuda::NewtonSceneNodeInput>& nodes,
    const std::vector<cuda::NewtonSceneDistanceEqualityConstraintInput>&
        constraints,
    const double regularization,
    const std::size_t maxIterationCount,
    const double residualTolerance,
    CpuSceneNonlinearEqualityConvergenceResult& result)
{
  std::vector<cuda::NewtonSceneNodeInput> currentNodes = nodes;
  CpuSceneNonlinearEqualityAssemblyResult initialAssembly;
  evaluateCpuSceneNonlinearEqualityAssembly(
      currentNodes, constraints, initialAssembly);

  result = CpuSceneNonlinearEqualityConvergenceResult{};
  result.nodeCount = nodes.size();
  result.constraintCount = constraints.size();
  result.rowCount = 2u * constraints.size();
  result.bodyCount = nodes.size();
  result.dofCount = nodes.size() * cuda::kNewtonAssemblySolveDofsPerBody;
  result.blockCount = constraints.size();
  result.blockEntryCount
      = result.blockCount * cuda::kNewtonAssemblySolveBlockEntries;
  result.maxIterationCount = maxIterationCount;
  result.initialResiduals = std::move(initialAssembly.residuals);
  result.step.assign(result.dofCount, 0.0);
  double currentMaxResidual = maxResidualAbs(result.initialResiduals);
  result.initialMaxConstraintResidualAbs = currentMaxResidual;

  for (std::size_t iteration = 0;
       currentMaxResidual > residualTolerance && iteration < maxIterationCount;
       ++iteration) {
    CpuSceneNonlinearEqualitySolveResult iterationSolve;
    evaluateCpuSceneNonlinearEqualitySolve(
        currentNodes, constraints, regularization, iterationSolve);

    for (std::size_t node = 0; node < currentNodes.size(); ++node) {
      const std::size_t offset = node * cuda::kNewtonAssemblySolveDofsPerBody;
      for (std::size_t dof = 0; dof < cuda::kNewtonAssemblySolveDofsPerBody;
           ++dof) {
        const double correction = kSceneNonlinearEqualityConvergenceStepScale
                                  * iterationSolve.step[offset + dof];
        result.step[offset + dof] += correction;
        if (dof < 3u) {
          currentNodes[node].position[dof] += correction;
        }
      }
    }

    CpuSceneNonlinearEqualityAssemblyResult finalAssembly;
    evaluateCpuSceneNonlinearEqualityAssembly(
        currentNodes, constraints, finalAssembly);
    result.finalResiduals = finalAssembly.residuals;
    ++result.completedIterationCount;
    currentMaxResidual = maxResidualAbs(result.finalResiduals);
  }

  CpuSceneNonlinearEqualityAssemblyResult finalAssembly;
  evaluateCpuSceneNonlinearEqualityAssembly(
      currentNodes, constraints, finalAssembly);
  result.rows = std::move(finalAssembly.rows);
  result.blocks = std::move(finalAssembly.blocks);
  result.finalResiduals = std::move(finalAssembly.residuals);
  result.finalMaxConstraintResidualAbs = maxResidualAbs(result.finalResiduals);
  result.maxDiagonal = finalAssembly.maxDiagonal;
  result.maxGradientAbs = finalAssembly.maxGradientAbs;
  result.maxBlockAbs = finalAssembly.maxBlockAbs;

  double stepNormSquared = 0.0;
  for (const auto value : result.step) {
    if (std::abs(value) > 0.0) {
      ++result.activeDofCount;
    }
    stepNormSquared += value * value;
  }
  result.stepNorm = std::sqrt(stepNormSquared);
  result.converged = result.finalMaxConstraintResidualAbs <= residualTolerance;
}

template <typename Fixture>
void populateSceneRuntimeDiagonalRows(
    Fixture& fixture, const sx::World& world, const sx::DeformableBody& body)
{
  fixture.sceneBodyCount = world.getDeformableBodyCount();
  fixture.sceneNodeCount = body.getNodeCount();
  fixture.sceneTriangleCount = body.getSurfaceTriangleCount();
  fixture.bodyCount = fixture.sceneNodeCount;

  std::vector<std::size_t> incidentTriangleCounts(fixture.sceneNodeCount, 0u);
  for (std::size_t triangle = 0; triangle < body.getSurfaceTriangleCount();
       ++triangle) {
    const auto surfaceTriangle = body.getSurfaceTriangle(triangle);
    ++incidentTriangleCounts[surfaceTriangle.nodeA];
    ++incidentTriangleCounts[surfaceTriangle.nodeB];
    ++incidentTriangleCounts[surfaceTriangle.nodeC];
  }

  fixture.rows.reserve(fixture.sceneNodeCount);
  for (std::size_t node = 0; node < fixture.sceneNodeCount; ++node) {
    fixture.rows.push_back(
        makeSceneRuntimeAssemblyRow(body, node, incidentTriangleCounts[node]));
  }
}

template <typename Fixture>
void populateSceneRuntimeSparseGraphFixture(
    Fixture& fixture, const sx::World& world, const sx::DeformableBody& body)
{
  populateSceneRuntimeDiagonalRows(fixture, world, body);

  fixture.blocks.reserve(3u * fixture.sceneTriangleCount);
  const auto appendSparseBlock
      = [&fixture, &body](const std::size_t nodeA, const std::size_t nodeB) {
          fixture.blocks.push_back(
              makeSceneRuntimeSparseBlock(body, nodeA, nodeB));
          ++fixture.sceneEdgePairCount;
        };
  for (std::size_t triangle = 0; triangle < body.getSurfaceTriangleCount();
       ++triangle) {
    const auto surfaceTriangle = body.getSurfaceTriangle(triangle);
    appendSparseBlock(surfaceTriangle.nodeA, surfaceTriangle.nodeB);
    appendSparseBlock(surfaceTriangle.nodeB, surfaceTriangle.nodeC);
    appendSparseBlock(surfaceTriangle.nodeC, surfaceTriangle.nodeA);
  }
}

cuda::NewtonEqualityReductionEntry makeEqualityReductionEntry(
    const std::size_t fullDof, const std::size_t reducedDof)
{
  cuda::NewtonEqualityReductionEntry entry;
  entry.fullDofIndex = static_cast<std::uint32_t>(fullDof);
  entry.reducedDofIndex = static_cast<std::uint32_t>(reducedDof);
  entry.basisValue = (fullDof % 2 == 0) ? 1.0 : 0.5;
  return entry;
}

cuda::NewtonEqualityReductionEntry makeSceneRuntimeEqualityReductionEntry(
    const std::size_t fullDof,
    const std::size_t reducedDof,
    const std::size_t localDof)
{
  cuda::NewtonEqualityReductionEntry entry;
  entry.fullDofIndex = static_cast<std::uint32_t>(fullDof);
  entry.reducedDofIndex = static_cast<std::uint32_t>(reducedDof);
  entry.basisValue = (localDof < 3u) ? 1.0 : 0.25;
  return entry;
}

void evaluateCpu(
    const std::vector<cuda::NewtonAssemblySolveRowInput>& rows,
    const std::size_t bodyCount,
    CpuAssemblySolveResult& result)
{
  const std::size_t dofCount
      = bodyCount * cuda::kNewtonAssemblySolveDofsPerBody;
  result = CpuAssemblySolveResult{};
  result.bodyCount = bodyCount;
  result.rowCount = rows.size();
  result.assembledDiagonal.assign(dofCount, 0.0);
  result.assembledGradient.assign(dofCount, 0.0);
  result.step.assign(dofCount, 0.0);
  result.residual.assign(dofCount, 0.0);

  for (const auto& row : rows) {
    const std::size_t offset = static_cast<std::size_t>(row.bodyIndex)
                               * cuda::kNewtonAssemblySolveDofsPerBody;
    for (std::size_t dof = 0; dof < cuda::kNewtonAssemblySolveDofsPerBody;
         ++dof) {
      result.assembledDiagonal[offset + dof] += row.hessianDiagonal[dof];
      result.assembledGradient[offset + dof] += row.gradient[dof];
    }
  }

  double stepNormSquared = 0.0;
  double residualNormSquared = 0.0;
  for (std::size_t dof = 0; dof < dofCount; ++dof) {
    const double effectiveDiagonal
        = result.assembledDiagonal[dof] + kRegularization;
    if (effectiveDiagonal > 1e-14) {
      ++result.activeDofCount;
    }
    result.step[dof] = -result.assembledGradient[dof] / effectiveDiagonal;
    result.residual[dof]
        = effectiveDiagonal * result.step[dof] + result.assembledGradient[dof];
    result.maxDiagonal
        = std::max(result.maxDiagonal, result.assembledDiagonal[dof]);
    result.maxGradientAbs = std::max(
        result.maxGradientAbs, std::abs(result.assembledGradient[dof]));
    stepNormSquared += result.step[dof] * result.step[dof];
    residualNormSquared += result.residual[dof] * result.residual[dof];
  }
  result.stepNorm = std::sqrt(stepNormSquared);
  result.residualNorm = std::sqrt(residualNormSquared);
}

sx::DeformableBodyOptions makeSceneRuntimeAssemblyBodyOptions(
    const int rowCount)
{
  sx::DeformableBodyOptions options;
  const int groupCount
      = std::max(1, static_cast<int>(std::sqrt(static_cast<double>(rowCount))));
  const auto appendSceneNode
      = [&options](const Eigen::Vector3d& start, const Eigen::Vector3d& end) {
          const double mass
              = 1.0 + 0.01 * static_cast<double>(options.positions.size() % 7u);
          options.positions.push_back(start);
          options.velocities.push_back(
              (end - start) / kSceneRuntimeAssemblyTimeStep);
          options.masses.push_back(mass);
        };

  for (int group = 0; group < groupCount; ++group) {
    const double x = static_cast<double>(group % 64) * 4.0;
    const double y = static_cast<double>(group / 64) * 4.0;
    const std::size_t base = options.positions.size();

    appendSceneNode({x, y, 0.0}, {x, y, 0.0});
    appendSceneNode({x + 1.0, y, 0.0}, {x + 1.0, y, 0.0});
    appendSceneNode({x, y + 1.0, 0.0}, {x, y + 1.0, 0.0});
    appendSceneNode({x + 0.25, y + 0.25, 0.20}, {x + 0.25, y + 0.25, -0.20});
    options.surfaceTriangles.push_back({base, base + 1u, base + 2u});

    appendSceneNode({x + 2.0, y, 0.0}, {x + 2.0, y, 0.0});
    appendSceneNode({x + 3.0, y, 0.0}, {x + 3.0, y, 0.0});
    appendSceneNode({x + 2.0, y + 1.0, 0.0}, {x + 2.0, y + 1.0, 0.0});
    options.surfaceTriangles.push_back({base + 4u, base + 5u, base + 6u});

    appendSceneNode({x + 2.25, y + 0.20, -0.5}, {x + 2.25, y - 0.20, -0.5});
    appendSceneNode({x + 2.25, y + 0.20, 0.5}, {x + 2.25, y - 0.20, 0.5});
    appendSceneNode({x + 3.25, y + 1.25, 0.0}, {x + 3.25, y + 1.25, 0.0});
    options.surfaceTriangles.push_back({base + 7u, base + 8u, base + 9u});
  }

  return options;
}

cuda::NewtonAssemblySolveRowInput makeSceneRuntimeAssemblyRow(
    const sx::DeformableBody& body,
    const std::size_t node,
    const std::size_t incidentTriangleCount)
{
  cuda::NewtonAssemblySolveRowInput row;
  row.bodyIndex = static_cast<std::uint32_t>(node);

  const Eigen::Vector3d position = body.getPosition(node);
  const Eigen::Vector3d velocity = body.getVelocity(node);
  const double mass = body.getMass(node);
  const double incidentScale = static_cast<double>(incidentTriangleCount);
  const double radius = position.norm();

  for (std::size_t dof = 0; dof < cuda::kNewtonAssemblySolveDofsPerBody;
       ++dof) {
    const std::size_t axis = dof % 3u;
    const double axisPosition = position[axis];
    const double axisVelocity = velocity[axis];
    if (dof < 3u) {
      row.hessianDiagonal[dof] = mass + 0.20 * static_cast<double>(axis + 1u)
                                 + 0.05 * incidentScale
                                 + 0.01 * std::abs(axisPosition);
      row.gradient[dof] = 0.08 * axisVelocity + 0.03 * axisPosition
                          + 0.01 * static_cast<double>(axis + 1u);
    } else {
      row.hessianDiagonal[dof] = 0.50 + 0.10 * mass
                                 + 0.05 * static_cast<double>(axis + 1u)
                                 + 0.025 * incidentScale + 0.005 * radius;
      row.gradient[dof] = 0.015 * static_cast<double>(axis + 1u) * axisPosition
                          - 0.02 * axisVelocity;
    }
  }

  return row;
}

SceneRuntimeAssemblySolveFixture makeSceneRuntimeAssemblySolveFixture(
    const int rowCount)
{
  sx::World world;
  world.setTimeStep(kSceneRuntimeAssemblyTimeStep);
  const sx::DeformableBody body = world.addDeformableBody(
      "plan083_scene_runtime_assembly_solve",
      makeSceneRuntimeAssemblyBodyOptions(rowCount));

  SceneRuntimeAssemblySolveFixture fixture;
  fixture.sceneBodyCount = world.getDeformableBodyCount();
  fixture.sceneNodeCount = body.getNodeCount();
  fixture.sceneTriangleCount = body.getSurfaceTriangleCount();
  fixture.bodyCount = fixture.sceneNodeCount;

  std::vector<std::size_t> incidentTriangleCounts(fixture.sceneNodeCount, 0u);
  for (std::size_t triangle = 0; triangle < body.getSurfaceTriangleCount();
       ++triangle) {
    const auto surfaceTriangle = body.getSurfaceTriangle(triangle);
    ++incidentTriangleCounts[surfaceTriangle.nodeA];
    ++incidentTriangleCounts[surfaceTriangle.nodeB];
    ++incidentTriangleCounts[surfaceTriangle.nodeC];
  }

  fixture.rows.reserve(fixture.sceneNodeCount);
  for (std::size_t node = 0; node < fixture.sceneNodeCount; ++node) {
    fixture.rows.push_back(
        makeSceneRuntimeAssemblyRow(body, node, incidentTriangleCounts[node]));
  }
  evaluateCpu(fixture.rows, fixture.bodyCount, fixture.cpu);
  return fixture;
}

void evaluateCpuEqualityReducedSolve(
    const std::vector<cuda::NewtonAssemblySolveRowInput>& rows,
    const std::size_t bodyCount,
    const std::vector<cuda::NewtonEqualityReductionEntry>& entries,
    const std::size_t reducedDofCount,
    CpuEqualityReducedSolveResult& result)
{
  CpuAssemblySolveResult full;
  evaluateCpu(rows, bodyCount, full);

  result = CpuEqualityReducedSolveResult{};
  result.bodyCount = bodyCount;
  result.rowCount = rows.size();
  result.fullDofCount = bodyCount * cuda::kNewtonAssemblySolveDofsPerBody;
  result.reductionEntryCount = entries.size();
  result.reducedDofCount = reducedDofCount;
  result.assembledDiagonal = full.assembledDiagonal;
  result.assembledGradient = full.assembledGradient;
  result.reducedDiagonal.assign(reducedDofCount, 0.0);
  result.reducedGradient.assign(reducedDofCount, 0.0);
  result.reducedStep.assign(reducedDofCount, 0.0);
  result.reducedResidual.assign(reducedDofCount, 0.0);
  result.fullStep.assign(result.fullDofCount, 0.0);

  for (const auto& entry : entries) {
    result.reducedDiagonal[entry.reducedDofIndex]
        += entry.basisValue * entry.basisValue
           * result.assembledDiagonal[entry.fullDofIndex];
    result.reducedGradient[entry.reducedDofIndex]
        += entry.basisValue * result.assembledGradient[entry.fullDofIndex];
  }

  double stepNormSquared = 0.0;
  double residualNormSquared = 0.0;
  for (std::size_t dof = 0; dof < reducedDofCount; ++dof) {
    const double effectiveDiagonal
        = result.reducedDiagonal[dof] + kRegularization;
    if (effectiveDiagonal > 1e-14) {
      ++result.activeReducedDofCount;
    }
    result.reducedStep[dof] = -result.reducedGradient[dof] / effectiveDiagonal;
    result.reducedResidual[dof] = effectiveDiagonal * result.reducedStep[dof]
                                  + result.reducedGradient[dof];
    result.maxDiagonal
        = std::max(result.maxDiagonal, result.reducedDiagonal[dof]);
    result.maxGradientAbs = std::max(
        result.maxGradientAbs, std::abs(result.reducedGradient[dof]));
    stepNormSquared += result.reducedStep[dof] * result.reducedStep[dof];
    residualNormSquared
        += result.reducedResidual[dof] * result.reducedResidual[dof];
  }

  for (const auto& entry : entries) {
    result.fullStep[entry.fullDofIndex]
        += entry.basisValue * result.reducedStep[entry.reducedDofIndex];
  }
  result.stepNorm = std::sqrt(stepNormSquared);
  result.residualNorm = std::sqrt(residualNormSquared);
}

void evaluateCpuOffDiagonalAssembly(
    const std::vector<cuda::NewtonOffDiagonalAssemblyRowInput>& rows,
    const std::size_t pairCount,
    CpuOffDiagonalAssemblyResult& result)
{
  result = CpuOffDiagonalAssemblyResult{};
  result.pairCount = pairCount;
  result.rowCount = rows.size();
  result.assembledBlocks.assign(
      pairCount * cuda::kNewtonAssemblySolveBlockEntries, 0.0);

  for (const auto& row : rows) {
    const std::size_t offset = static_cast<std::size_t>(row.pairIndex)
                               * cuda::kNewtonAssemblySolveBlockEntries;
    for (std::size_t entry = 0; entry < cuda::kNewtonAssemblySolveBlockEntries;
         ++entry) {
      result.assembledBlocks[offset + entry] += row.hessianBlock[entry];
    }
  }

  for (std::size_t pair = 0; pair < pairCount; ++pair) {
    bool active = false;
    const std::size_t offset = pair * cuda::kNewtonAssemblySolveBlockEntries;
    for (std::size_t entry = 0; entry < cuda::kNewtonAssemblySolveBlockEntries;
         ++entry) {
      const double value = std::abs(result.assembledBlocks[offset + entry]);
      result.maxBlockAbs = std::max(result.maxBlockAbs, value);
      active = active || value > 0.0;
    }
    if (active) {
      ++result.activeBlockCount;
    }
  }
}

void evaluateCpuSparseResidual(
    const std::vector<cuda::NewtonAssemblySolveRowInput>& rows,
    const std::size_t bodyCount,
    const std::vector<cuda::NewtonSparseBlockEntry>& blocks,
    const std::vector<double>& step,
    CpuSparseResidualResult& result)
{
  CpuAssemblySolveResult diagonal;
  evaluateCpu(rows, bodyCount, diagonal);

  result = CpuSparseResidualResult{};
  result.bodyCount = bodyCount;
  result.rowCount = rows.size();
  result.dofCount = bodyCount * cuda::kNewtonAssemblySolveDofsPerBody;
  result.blockCount = blocks.size();
  result.assembledDiagonal = diagonal.assembledDiagonal;
  result.assembledGradient = diagonal.assembledGradient;
  result.residual = diagonal.assembledGradient;

  for (std::size_t dof = 0; dof < result.dofCount; ++dof) {
    result.residual[dof]
        += (result.assembledDiagonal[dof] + kRegularization) * step[dof];
  }

  for (const auto& block : blocks) {
    for (std::size_t entry = 0; entry < cuda::kNewtonAssemblySolveBlockEntries;
         ++entry) {
      const std::size_t localRow
          = entry / cuda::kNewtonAssemblySolveDofsPerBody;
      const std::size_t localColumn
          = entry - localRow * cuda::kNewtonAssemblySolveDofsPerBody;
      const std::size_t rowDof = static_cast<std::size_t>(block.rowBodyIndex)
                                     * cuda::kNewtonAssemblySolveDofsPerBody
                                 + localRow;
      const std::size_t columnDof
          = static_cast<std::size_t>(block.columnBodyIndex)
                * cuda::kNewtonAssemblySolveDofsPerBody
            + localColumn;
      result.residual[rowDof] += block.hessianBlock[entry] * step[columnDof];
      result.residual[columnDof] += block.hessianBlock[entry] * step[rowDof];
    }
  }

  double residualNormSquared = 0.0;
  for (std::size_t dof = 0; dof < result.dofCount; ++dof) {
    result.maxDiagonal
        = std::max(result.maxDiagonal, result.assembledDiagonal[dof]);
    result.maxGradientAbs = std::max(
        result.maxGradientAbs, std::abs(result.assembledGradient[dof]));
    const double residualAbs = std::abs(result.residual[dof]);
    result.maxResidualAbs = std::max(result.maxResidualAbs, residualAbs);
    if (residualAbs > 0.0) {
      ++result.activeDofCount;
    }
    residualNormSquared += result.residual[dof] * result.residual[dof];
  }
  result.residualNorm = std::sqrt(residualNormSquared);
}

std::vector<double> computeSparseResidualFromAssembled(
    const std::vector<double>& assembledDiagonal,
    const std::vector<double>& assembledGradient,
    const std::vector<cuda::NewtonSparseBlockEntry>& blocks,
    const std::vector<double>& step)
{
  std::vector<double> residual = assembledGradient;
  for (std::size_t dof = 0; dof < residual.size(); ++dof) {
    residual[dof] += (assembledDiagonal[dof] + kRegularization) * step[dof];
  }

  for (const auto& block : blocks) {
    for (std::size_t entry = 0; entry < cuda::kNewtonAssemblySolveBlockEntries;
         ++entry) {
      const std::size_t localRow
          = entry / cuda::kNewtonAssemblySolveDofsPerBody;
      const std::size_t localColumn
          = entry - localRow * cuda::kNewtonAssemblySolveDofsPerBody;
      const std::size_t rowDof = static_cast<std::size_t>(block.rowBodyIndex)
                                     * cuda::kNewtonAssemblySolveDofsPerBody
                                 + localRow;
      const std::size_t columnDof
          = static_cast<std::size_t>(block.columnBodyIndex)
                * cuda::kNewtonAssemblySolveDofsPerBody
            + localColumn;
      residual[rowDof] += block.hessianBlock[entry] * step[columnDof];
      residual[columnDof] += block.hessianBlock[entry] * step[rowDof];
    }
  }
  return residual;
}

std::vector<double> computeSparseMatrixVectorFromAssembled(
    const std::vector<double>& assembledDiagonal,
    const std::vector<cuda::NewtonSparseBlockEntry>& blocks,
    const std::vector<double>& step)
{
  std::vector<double> product(step.size(), 0.0);
  for (std::size_t dof = 0; dof < product.size(); ++dof) {
    product[dof] = (assembledDiagonal[dof] + kRegularization) * step[dof];
  }

  for (const auto& block : blocks) {
    for (std::size_t entry = 0; entry < cuda::kNewtonAssemblySolveBlockEntries;
         ++entry) {
      const std::size_t localRow
          = entry / cuda::kNewtonAssemblySolveDofsPerBody;
      const std::size_t localColumn
          = entry - localRow * cuda::kNewtonAssemblySolveDofsPerBody;
      const std::size_t rowDof = static_cast<std::size_t>(block.rowBodyIndex)
                                     * cuda::kNewtonAssemblySolveDofsPerBody
                                 + localRow;
      const std::size_t columnDof
          = static_cast<std::size_t>(block.columnBodyIndex)
                * cuda::kNewtonAssemblySolveDofsPerBody
            + localColumn;
      product[rowDof] += block.hessianBlock[entry] * step[columnDof];
      product[columnDof] += block.hessianBlock[entry] * step[rowDof];
    }
  }
  return product;
}

double dotVectors(
    const std::vector<double>& lhs, const std::vector<double>& rhs)
{
  double dot = 0.0;
  for (std::size_t dof = 0; dof < lhs.size(); ++dof) {
    dot += lhs[dof] * rhs[dof];
  }
  return dot;
}

void evaluateCpuSparseJacobiSolve(
    const std::vector<cuda::NewtonAssemblySolveRowInput>& rows,
    const std::size_t bodyCount,
    const std::vector<cuda::NewtonSparseBlockEntry>& blocks,
    CpuSparseJacobiSolveResult& result)
{
  CpuAssemblySolveResult diagonal;
  evaluateCpu(rows, bodyCount, diagonal);

  result = CpuSparseJacobiSolveResult{};
  result.bodyCount = bodyCount;
  result.rowCount = rows.size();
  result.dofCount = bodyCount * cuda::kNewtonAssemblySolveDofsPerBody;
  result.blockCount = blocks.size();
  result.iterationCount = kSparseJacobiIterations;
  result.assembledDiagonal = diagonal.assembledDiagonal;
  result.assembledGradient = diagonal.assembledGradient;
  result.step.assign(result.dofCount, 0.0);

  for (std::size_t iteration = 0; iteration < kSparseJacobiIterations;
       ++iteration) {
    const auto residual = computeSparseResidualFromAssembled(
        result.assembledDiagonal,
        result.assembledGradient,
        blocks,
        result.step);
    for (std::size_t dof = 0; dof < result.dofCount; ++dof) {
      result.step[dof]
          -= residual[dof] / (result.assembledDiagonal[dof] + kRegularization);
    }
  }
  result.residual = computeSparseResidualFromAssembled(
      result.assembledDiagonal, result.assembledGradient, blocks, result.step);

  double stepNormSquared = 0.0;
  double residualNormSquared = 0.0;
  for (std::size_t dof = 0; dof < result.dofCount; ++dof) {
    result.maxDiagonal
        = std::max(result.maxDiagonal, result.assembledDiagonal[dof]);
    result.maxGradientAbs = std::max(
        result.maxGradientAbs, std::abs(result.assembledGradient[dof]));
    const double residualAbs = std::abs(result.residual[dof]);
    result.maxResidualAbs = std::max(result.maxResidualAbs, residualAbs);
    if (result.assembledDiagonal[dof] + kRegularization > 1e-14) {
      ++result.activeDofCount;
    }
    stepNormSquared += result.step[dof] * result.step[dof];
    residualNormSquared += result.residual[dof] * result.residual[dof];
  }
  result.stepNorm = std::sqrt(stepNormSquared);
  result.residualNorm = std::sqrt(residualNormSquared);
}

void evaluateCpuSparseCgSolve(
    const std::vector<cuda::NewtonAssemblySolveRowInput>& rows,
    const std::size_t bodyCount,
    const std::vector<cuda::NewtonSparseBlockEntry>& blocks,
    CpuSparseCgSolveResult& result)
{
  CpuAssemblySolveResult diagonal;
  evaluateCpu(rows, bodyCount, diagonal);

  result = CpuSparseCgSolveResult{};
  result.bodyCount = bodyCount;
  result.rowCount = rows.size();
  result.dofCount = bodyCount * cuda::kNewtonAssemblySolveDofsPerBody;
  result.blockCount = blocks.size();
  result.maxIterationCount = kSparseCgMaxIterations;
  result.residualTolerance = kSparseCgResidualTolerance;
  result.assembledDiagonal = diagonal.assembledDiagonal;
  result.assembledGradient = diagonal.assembledGradient;
  result.step.assign(result.dofCount, 0.0);

  std::vector<double> residual(result.dofCount, 0.0);
  std::vector<double> direction(result.dofCount, 0.0);
  for (std::size_t dof = 0; dof < result.dofCount; ++dof) {
    residual[dof] = -result.assembledGradient[dof];
    direction[dof] = residual[dof];
  }

  double residualSquared = std::max(0.0, dotVectors(residual, residual));
  result.initialResidualNorm = std::sqrt(residualSquared);
  const double toleranceSquared
      = result.residualTolerance * result.residualTolerance;

  for (std::size_t iteration = 0; iteration < result.maxIterationCount;
       ++iteration) {
    if (residualSquared <= toleranceSquared) {
      break;
    }

    const auto matrixDirection = computeSparseMatrixVectorFromAssembled(
        result.assembledDiagonal, blocks, direction);
    const double directionMatrixDirection
        = dotVectors(direction, matrixDirection);
    if (!std::isfinite(directionMatrixDirection)
        || std::abs(directionMatrixDirection) <= 1e-30) {
      break;
    }

    const double alpha = residualSquared / directionMatrixDirection;
    for (std::size_t dof = 0; dof < result.dofCount; ++dof) {
      result.step[dof] += alpha * direction[dof];
      residual[dof] -= alpha * matrixDirection[dof];
    }

    double nextResidualSquared = std::max(0.0, dotVectors(residual, residual));
    ++result.completedIterationCount;
    if (nextResidualSquared <= toleranceSquared) {
      residualSquared = nextResidualSquared;
      break;
    }

    const double beta = nextResidualSquared / residualSquared;
    for (std::size_t dof = 0; dof < result.dofCount; ++dof) {
      direction[dof] = residual[dof] + beta * direction[dof];
    }
    residualSquared = nextResidualSquared;
  }

  result.residual = computeSparseResidualFromAssembled(
      result.assembledDiagonal, result.assembledGradient, blocks, result.step);

  double stepNormSquared = 0.0;
  double residualNormSquared = 0.0;
  for (std::size_t dof = 0; dof < result.dofCount; ++dof) {
    result.maxDiagonal
        = std::max(result.maxDiagonal, result.assembledDiagonal[dof]);
    result.maxGradientAbs = std::max(
        result.maxGradientAbs, std::abs(result.assembledGradient[dof]));
    const double residualAbs = std::abs(result.residual[dof]);
    result.maxResidualAbs = std::max(result.maxResidualAbs, residualAbs);
    if (result.assembledDiagonal[dof] + kRegularization > 1e-14) {
      ++result.activeDofCount;
    }
    stepNormSquared += result.step[dof] * result.step[dof];
    residualNormSquared += result.residual[dof] * result.residual[dof];
  }
  result.stepNorm = std::sqrt(stepNormSquared);
  result.residualNorm = std::sqrt(residualNormSquared);
  result.converged = result.residualNorm <= result.residualTolerance;
}

std::vector<double> assembleDenseSparseMatrix(
    const std::vector<double>& assembledDiagonal,
    const std::vector<cuda::NewtonSparseBlockEntry>& blocks)
{
  const std::size_t dofCount = assembledDiagonal.size();
  std::vector<double> matrix(dofCount * dofCount, 0.0);
  for (std::size_t dof = 0; dof < dofCount; ++dof) {
    matrix[dof * dofCount + dof] = assembledDiagonal[dof] + kRegularization;
  }

  for (const auto& block : blocks) {
    for (std::size_t entry = 0; entry < cuda::kNewtonAssemblySolveBlockEntries;
         ++entry) {
      const std::size_t localRow
          = entry / cuda::kNewtonAssemblySolveDofsPerBody;
      const std::size_t localColumn
          = entry - localRow * cuda::kNewtonAssemblySolveDofsPerBody;
      const std::size_t rowDof = static_cast<std::size_t>(block.rowBodyIndex)
                                     * cuda::kNewtonAssemblySolveDofsPerBody
                                 + localRow;
      const std::size_t columnDof
          = static_cast<std::size_t>(block.columnBodyIndex)
                * cuda::kNewtonAssemblySolveDofsPerBody
            + localColumn;
      matrix[rowDof * dofCount + columnDof] += block.hessianBlock[entry];
      matrix[columnDof * dofCount + rowDof] += block.hessianBlock[entry];
    }
  }

  return matrix;
}

bool solveDenseCholesky(
    std::vector<double>& matrix,
    const std::vector<double>& assembledGradient,
    std::vector<double>& step,
    double& minimumFactorPivot)
{
  constexpr double kPivotEpsilon = 1e-14;
  const std::size_t dofCount = assembledGradient.size();
  minimumFactorPivot = 1e300;

  for (std::size_t row = 0; row < dofCount; ++row) {
    for (std::size_t column = 0; column < row; ++column) {
      double sum = matrix[row * dofCount + column];
      for (std::size_t inner = 0; inner < column; ++inner) {
        sum -= matrix[row * dofCount + inner]
               * matrix[column * dofCount + inner];
      }
      const double pivot = matrix[column * dofCount + column];
      if (!std::isfinite(pivot) || std::abs(pivot) <= kPivotEpsilon) {
        minimumFactorPivot = 0.0;
        return false;
      }
      matrix[row * dofCount + column] = sum / pivot;
    }

    double diagonal = matrix[row * dofCount + row];
    for (std::size_t inner = 0; inner < row; ++inner) {
      const double value = matrix[row * dofCount + inner];
      diagonal -= value * value;
    }
    if (!std::isfinite(diagonal) || diagonal <= kPivotEpsilon) {
      minimumFactorPivot = 0.0;
      return false;
    }

    const double pivot = std::sqrt(diagonal);
    matrix[row * dofCount + row] = pivot;
    minimumFactorPivot = std::min(minimumFactorPivot, pivot);
  }

  for (std::size_t row = 0; row < dofCount; ++row) {
    for (std::size_t column = row + 1u; column < dofCount; ++column) {
      matrix[row * dofCount + column] = 0.0;
    }
  }

  step.assign(dofCount, 0.0);
  for (std::size_t row = 0; row < dofCount; ++row) {
    double value = -assembledGradient[row];
    for (std::size_t column = 0; column < row; ++column) {
      value -= matrix[row * dofCount + column] * step[column];
    }
    step[row] = value / matrix[row * dofCount + row];
  }

  for (std::size_t reverse = 0; reverse < dofCount; ++reverse) {
    const std::size_t row = dofCount - 1u - reverse;
    double value = step[row];
    for (std::size_t column = row + 1u; column < dofCount; ++column) {
      value -= matrix[column * dofCount + row] * step[column];
    }
    step[row] = value / matrix[row * dofCount + row];
  }

  return true;
}

void evaluateCpuDirectSparseSolve(
    const std::vector<cuda::NewtonAssemblySolveRowInput>& rows,
    const std::size_t bodyCount,
    const std::vector<cuda::NewtonSparseBlockEntry>& blocks,
    CpuDirectSparseSolveResult& result)
{
  CpuAssemblySolveResult diagonal;
  evaluateCpu(rows, bodyCount, diagonal);

  result = CpuDirectSparseSolveResult{};
  result.bodyCount = bodyCount;
  result.rowCount = rows.size();
  result.dofCount = bodyCount * cuda::kNewtonAssemblySolveDofsPerBody;
  result.blockCount = blocks.size();
  result.assembledDiagonal = diagonal.assembledDiagonal;
  result.assembledGradient = diagonal.assembledGradient;
  result.factorMatrixLower
      = assembleDenseSparseMatrix(result.assembledDiagonal, blocks);
  result.factorized = solveDenseCholesky(
      result.factorMatrixLower,
      result.assembledGradient,
      result.step,
      result.minimumFactorPivot);
  if (!result.factorized) {
    result.step.assign(result.dofCount, 0.0);
  }
  result.residual = computeSparseResidualFromAssembled(
      result.assembledDiagonal, result.assembledGradient, blocks, result.step);

  double stepNormSquared = 0.0;
  double residualNormSquared = 0.0;
  for (std::size_t dof = 0; dof < result.dofCount; ++dof) {
    result.maxDiagonal
        = std::max(result.maxDiagonal, result.assembledDiagonal[dof]);
    result.maxGradientAbs = std::max(
        result.maxGradientAbs, std::abs(result.assembledGradient[dof]));
    const double residualAbs = std::abs(result.residual[dof]);
    result.maxResidualAbs = std::max(result.maxResidualAbs, residualAbs);
    if (result.assembledDiagonal[dof] + kRegularization > 1e-14) {
      ++result.activeDofCount;
    }
    stepNormSquared += result.step[dof] * result.step[dof];
    residualNormSquared += result.residual[dof] * result.residual[dof];
  }
  result.stepNorm = std::sqrt(stepNormSquared);
  result.residualNorm = std::sqrt(residualNormSquared);
}

AssemblySolveFixture makeFixture(const int rowCount)
{
  AssemblySolveFixture fixture;
  fixture.bodyCount
      = std::max<std::size_t>(1, static_cast<std::size_t>(rowCount) / 8);
  fixture.rows.reserve(static_cast<std::size_t>(rowCount));
  for (int row = 0; row < rowCount; ++row) {
    fixture.rows.push_back(makeRow(row, fixture.bodyCount));
  }
  evaluateCpu(fixture.rows, fixture.bodyCount, fixture.cpu);
  return fixture;
}

OffDiagonalAssemblyFixture makeOffDiagonalFixture(const int rowCount)
{
  OffDiagonalAssemblyFixture fixture;
  fixture.pairCount
      = std::max<std::size_t>(1, static_cast<std::size_t>(rowCount) / 16);
  fixture.rows.reserve(static_cast<std::size_t>(rowCount));
  for (int row = 0; row < rowCount; ++row) {
    fixture.rows.push_back(makeOffDiagonalRow(row, fixture.pairCount));
  }
  evaluateCpuOffDiagonalAssembly(fixture.rows, fixture.pairCount, fixture.cpu);
  return fixture;
}

SceneRuntimeOffDiagonalAssemblyFixture
makeSceneRuntimeOffDiagonalAssemblyFixture(const int rowCount)
{
  sx::World world;
  world.setTimeStep(kSceneRuntimeAssemblyTimeStep);
  const sx::DeformableBody body = world.addDeformableBody(
      "plan083_scene_runtime_sparse_off_diagonal_assembly",
      makeSceneRuntimeAssemblyBodyOptions(rowCount));

  SceneRuntimeOffDiagonalAssemblyFixture fixture;
  fixture.sceneBodyCount = world.getDeformableBodyCount();
  fixture.sceneNodeCount = body.getNodeCount();
  fixture.sceneTriangleCount = body.getSurfaceTriangleCount();
  fixture.rows.reserve(3u * fixture.sceneTriangleCount);

  const auto appendEdgePair
      = [&fixture, &body](const std::size_t nodeA, const std::size_t nodeB) {
          fixture.rows.push_back(makeSceneRuntimeOffDiagonalRow(
              body, fixture.sceneEdgePairCount, nodeA, nodeB));
          ++fixture.sceneEdgePairCount;
        };

  for (std::size_t triangle = 0; triangle < body.getSurfaceTriangleCount();
       ++triangle) {
    const auto surfaceTriangle = body.getSurfaceTriangle(triangle);
    appendEdgePair(surfaceTriangle.nodeA, surfaceTriangle.nodeB);
    appendEdgePair(surfaceTriangle.nodeB, surfaceTriangle.nodeC);
    appendEdgePair(surfaceTriangle.nodeC, surfaceTriangle.nodeA);
  }

  fixture.pairCount = fixture.sceneEdgePairCount;
  evaluateCpuOffDiagonalAssembly(fixture.rows, fixture.pairCount, fixture.cpu);
  return fixture;
}

SceneRuntimeSparseGraphAssemblyFixture
makeSceneRuntimeSparseGraphAssemblyFixture(const int rowCount)
{
  sx::World world;
  world.setTimeStep(kSceneRuntimeAssemblyTimeStep);
  const sx::DeformableBody body = world.addDeformableBody(
      "plan083_scene_runtime_sparse_graph_assembly",
      makeSceneRuntimeAssemblyBodyOptions(rowCount));

  SceneRuntimeSparseGraphAssemblyFixture fixture;
  fixture.sceneBodyCount = world.getDeformableBodyCount();
  fixture.nodes.reserve(body.getNodeCount());
  for (std::size_t node = 0; node < body.getNodeCount(); ++node) {
    fixture.nodes.push_back(makeSceneRuntimeNodeInput(body, node));
  }

  fixture.triangles.reserve(body.getSurfaceTriangleCount());
  for (std::size_t triangle = 0; triangle < body.getSurfaceTriangleCount();
       ++triangle) {
    fixture.triangles.push_back(makeSceneRuntimeTriangleInput(body, triangle));
  }

  evaluateCpuSceneSparseGraphAssembly(
      fixture.nodes, fixture.triangles, fixture.cpu);
  return fixture;
}

SceneRuntimeSparseGraphUniqueAssemblyFixture
makeSceneRuntimeSparseGraphUniqueAssemblyFixture(const int rowCount)
{
  sx::World world;
  world.setTimeStep(kSceneRuntimeAssemblyTimeStep);
  const sx::DeformableBody body = world.addDeformableBody(
      "plan083_scene_runtime_sparse_graph_unique_assembly",
      makeSceneRuntimeAssemblyBodyOptions(rowCount));

  SceneRuntimeSparseGraphUniqueAssemblyFixture fixture;
  fixture.sceneBodyCount = world.getDeformableBodyCount();
  fixture.nodes.reserve(body.getNodeCount());
  for (std::size_t node = 0; node < body.getNodeCount(); ++node) {
    fixture.nodes.push_back(makeSceneRuntimeNodeInput(body, node));
  }

  fixture.triangles.reserve(body.getSurfaceTriangleCount());
  for (std::size_t triangle = 0; triangle < body.getSurfaceTriangleCount();
       ++triangle) {
    fixture.triangles.push_back(makeSceneRuntimeTriangleInput(body, triangle));
  }
  if (!fixture.triangles.empty()) {
    auto duplicateTriangle = fixture.triangles.front();
    std::swap(duplicateTriangle.nodeA, duplicateTriangle.nodeC);
    fixture.triangles.push_back(duplicateTriangle);
  }

  evaluateCpuSceneSparseGraphUniqueAssembly(
      fixture.nodes, fixture.triangles, fixture.cpu);
  return fixture;
}

SceneRuntimeNonlinearEqualityAssemblyFixture
makeSceneRuntimeNonlinearEqualityAssemblyFixture(const int rowCount)
{
  sx::World world;
  world.setTimeStep(kSceneRuntimeAssemblyTimeStep);
  const sx::DeformableBody body = world.addDeformableBody(
      "plan083_scene_runtime_nonlinear_equality_assembly",
      makeSceneRuntimeAssemblyBodyOptions(rowCount));

  SceneRuntimeNonlinearEqualityAssemblyFixture fixture;
  fixture.sceneBodyCount = world.getDeformableBodyCount();
  fixture.sceneTriangleCount = body.getSurfaceTriangleCount();
  fixture.nodes.reserve(body.getNodeCount());
  for (std::size_t node = 0; node < body.getNodeCount(); ++node) {
    fixture.nodes.push_back(makeSceneRuntimeNodeInput(body, node));
  }

  const auto appendConstraint
      = [&fixture](const std::size_t nodeA, const std::size_t nodeB) {
          fixture.constraints.push_back(
              makeSceneRuntimeDistanceEqualityConstraint(
                  fixture.nodes, nodeA, nodeB, fixture.constraints.size()));
          ++fixture.sceneEdgePairCount;
        };
  fixture.constraints.reserve(3u * body.getSurfaceTriangleCount());
  for (std::size_t triangle = 0; triangle < body.getSurfaceTriangleCount();
       ++triangle) {
    const auto surfaceTriangle = body.getSurfaceTriangle(triangle);
    appendConstraint(surfaceTriangle.nodeA, surfaceTriangle.nodeB);
    appendConstraint(surfaceTriangle.nodeB, surfaceTriangle.nodeC);
    appendConstraint(surfaceTriangle.nodeC, surfaceTriangle.nodeA);
  }

  evaluateCpuSceneNonlinearEqualityAssembly(
      fixture.nodes, fixture.constraints, fixture.cpu);
  return fixture;
}

SceneRuntimeNonlinearEqualitySolveFixture
makeSceneRuntimeNonlinearEqualitySolveFixture(const int rowCount)
{
  auto assemblyFixture
      = makeSceneRuntimeNonlinearEqualityAssemblyFixture(rowCount);

  SceneRuntimeNonlinearEqualitySolveFixture fixture;
  fixture.nodes = std::move(assemblyFixture.nodes);
  fixture.constraints = std::move(assemblyFixture.constraints);
  fixture.sceneBodyCount = assemblyFixture.sceneBodyCount;
  fixture.sceneTriangleCount = assemblyFixture.sceneTriangleCount;
  fixture.sceneEdgePairCount = assemblyFixture.sceneEdgePairCount;
  evaluateCpuSceneNonlinearEqualitySolve(
      fixture.nodes,
      fixture.constraints,
      kSceneNonlinearEqualitySolveRegularization,
      fixture.cpu);
  return fixture;
}

SceneRuntimeNonlinearEqualityConvergenceFixture
makeSceneRuntimeNonlinearEqualityConvergenceFixture(const int rowCount)
{
  auto assemblyFixture
      = makeSceneRuntimeNonlinearEqualityAssemblyFixture(rowCount);

  SceneRuntimeNonlinearEqualityConvergenceFixture fixture;
  fixture.nodes = std::move(assemblyFixture.nodes);
  fixture.constraints = std::move(assemblyFixture.constraints);
  for (auto& constraint : fixture.constraints) {
    constraint.damping = 0.0;
  }
  fixture.sceneBodyCount = assemblyFixture.sceneBodyCount;
  fixture.sceneTriangleCount = assemblyFixture.sceneTriangleCount;
  fixture.sceneEdgePairCount = assemblyFixture.sceneEdgePairCount;
  evaluateCpuSceneNonlinearEqualityConvergence(
      fixture.nodes,
      fixture.constraints,
      kSceneNonlinearEqualitySolveRegularization,
      kSceneNonlinearEqualityConvergenceMaxIterations,
      kSceneNonlinearEqualityConvergenceResidualTolerance,
      fixture.cpu);
  return fixture;
}

SparseResidualFixture makeSparseResidualFixture(const int rowCount)
{
  SparseResidualFixture fixture;
  fixture.bodyCount
      = std::max<std::size_t>(2, static_cast<std::size_t>(rowCount) / 8);
  const std::size_t dofCount
      = fixture.bodyCount * cuda::kNewtonAssemblySolveDofsPerBody;
  fixture.rows.reserve(static_cast<std::size_t>(rowCount));
  for (int row = 0; row < rowCount; ++row) {
    fixture.rows.push_back(makeRow(row, fixture.bodyCount));
  }

  const std::size_t blockCount
      = std::max<std::size_t>(1, static_cast<std::size_t>(rowCount) / 8);
  fixture.blocks.reserve(blockCount);
  for (std::size_t block = 0; block < blockCount; ++block) {
    fixture.blocks.push_back(
        makeSparseBlock(static_cast<int>(block), fixture.bodyCount));
  }
  fixture.step = makeSparseStep(dofCount);
  evaluateCpuSparseResidual(
      fixture.rows,
      fixture.bodyCount,
      fixture.blocks,
      fixture.step,
      fixture.cpu);
  return fixture;
}

SceneRuntimeSparseResidualFixture makeSceneRuntimeSparseResidualFixture(
    const int rowCount)
{
  sx::World world;
  world.setTimeStep(kSceneRuntimeAssemblyTimeStep);
  const sx::DeformableBody body = world.addDeformableBody(
      "plan083_scene_runtime_sparse_residual",
      makeSceneRuntimeAssemblyBodyOptions(rowCount));

  SceneRuntimeSparseResidualFixture fixture;
  populateSceneRuntimeSparseGraphFixture(fixture, world, body);
  fixture.step = makeSceneRuntimeSparseStep(body);
  evaluateCpuSparseResidual(
      fixture.rows,
      fixture.bodyCount,
      fixture.blocks,
      fixture.step,
      fixture.cpu);
  return fixture;
}

SceneRuntimeSparseJacobiSolveFixture makeSceneRuntimeSparseJacobiSolveFixture(
    const int rowCount)
{
  sx::World world;
  world.setTimeStep(kSceneRuntimeAssemblyTimeStep);
  const sx::DeformableBody body = world.addDeformableBody(
      "plan083_scene_runtime_sparse_jacobi",
      makeSceneRuntimeAssemblyBodyOptions(rowCount));

  SceneRuntimeSparseJacobiSolveFixture fixture;
  populateSceneRuntimeSparseGraphFixture(fixture, world, body);
  evaluateCpuSparseJacobiSolve(
      fixture.rows, fixture.bodyCount, fixture.blocks, fixture.cpu);
  return fixture;
}

SceneRuntimeSparseCgSolveFixture makeSceneRuntimeSparseCgSolveFixture(
    const int rowCount)
{
  sx::World world;
  world.setTimeStep(kSceneRuntimeAssemblyTimeStep);
  const sx::DeformableBody body = world.addDeformableBody(
      "plan083_scene_runtime_sparse_cg",
      makeSceneRuntimeAssemblyBodyOptions(rowCount));

  SceneRuntimeSparseCgSolveFixture fixture;
  populateSceneRuntimeSparseGraphFixture(fixture, world, body);
  evaluateCpuSparseCgSolve(
      fixture.rows, fixture.bodyCount, fixture.blocks, fixture.cpu);
  return fixture;
}

SparseJacobiSolveFixture makeSparseJacobiSolveFixture(const int rowCount)
{
  SparseJacobiSolveFixture fixture;
  fixture.bodyCount
      = std::max<std::size_t>(2, static_cast<std::size_t>(rowCount) / 8);
  fixture.rows.reserve(static_cast<std::size_t>(rowCount));
  for (int row = 0; row < rowCount; ++row) {
    fixture.rows.push_back(makeRow(row, fixture.bodyCount));
  }

  const std::size_t blockCount
      = std::max<std::size_t>(1, static_cast<std::size_t>(rowCount) / 8);
  fixture.blocks.reserve(blockCount);
  for (std::size_t block = 0; block < blockCount; ++block) {
    fixture.blocks.push_back(
        makeSparseBlock(static_cast<int>(block), fixture.bodyCount));
  }
  evaluateCpuSparseJacobiSolve(
      fixture.rows, fixture.bodyCount, fixture.blocks, fixture.cpu);
  return fixture;
}

SparseCgSolveFixture makeSparseCgSolveFixture(const int rowCount)
{
  SparseCgSolveFixture fixture;
  fixture.bodyCount
      = std::max<std::size_t>(2, static_cast<std::size_t>(rowCount) / 8);
  fixture.rows.reserve(static_cast<std::size_t>(rowCount));
  for (int row = 0; row < rowCount; ++row) {
    fixture.rows.push_back(makeRow(row, fixture.bodyCount));
  }

  const std::size_t blockCount
      = std::max<std::size_t>(1, static_cast<std::size_t>(rowCount) / 8);
  fixture.blocks.reserve(blockCount);
  for (std::size_t block = 0; block < blockCount; ++block) {
    fixture.blocks.push_back(
        makeSparseBlock(static_cast<int>(block), fixture.bodyCount));
  }
  evaluateCpuSparseCgSolve(
      fixture.rows, fixture.bodyCount, fixture.blocks, fixture.cpu);
  return fixture;
}

DirectSparseSolveFixture makeDirectSparseSolveFixture(const int rowCount)
{
  DirectSparseSolveFixture fixture;
  fixture.bodyCount = kDirectSparseBodyCount;
  fixture.rows.reserve(static_cast<std::size_t>(rowCount));
  for (int row = 0; row < rowCount; ++row) {
    fixture.rows.push_back(makeRow(row, fixture.bodyCount));
  }

  fixture.blocks.reserve(fixture.bodyCount);
  for (std::size_t block = 0; block < fixture.bodyCount; ++block) {
    fixture.blocks.push_back(
        makeSparseBlock(static_cast<int>(block), fixture.bodyCount));
  }
  evaluateCpuDirectSparseSolve(
      fixture.rows, fixture.bodyCount, fixture.blocks, fixture.cpu);
  return fixture;
}

SceneRuntimeDirectSparseSolveFixture makeSceneRuntimeDirectSparseSolveFixture(
    const int rowCount)
{
  sx::World world;
  world.setTimeStep(kSceneRuntimeAssemblyTimeStep);
  const sx::DeformableBody body = world.addDeformableBody(
      "plan083_scene_runtime_direct_sparse",
      makeSceneRuntimeAssemblyBodyOptions(rowCount));

  SceneRuntimeDirectSparseSolveFixture fixture;
  fixture.sceneBodyCount = world.getDeformableBodyCount();
  fixture.sceneNodeCount = body.getNodeCount();
  fixture.sceneTriangleCount = body.getSurfaceTriangleCount();
  fixture.selectedSceneNodeCount = std::min<std::size_t>(
      kSceneRuntimeDirectSparseSelectedNodeCount, fixture.sceneNodeCount);
  fixture.bodyCount = fixture.selectedSceneNodeCount;

  std::vector<std::size_t> incidentTriangleCounts(fixture.sceneNodeCount, 0u);
  for (std::size_t triangle = 0; triangle < body.getSurfaceTriangleCount();
       ++triangle) {
    const auto surfaceTriangle = body.getSurfaceTriangle(triangle);
    ++incidentTriangleCounts[surfaceTriangle.nodeA];
    ++incidentTriangleCounts[surfaceTriangle.nodeB];
    ++incidentTriangleCounts[surfaceTriangle.nodeC];
  }

  fixture.rows.reserve(fixture.selectedSceneNodeCount);
  for (std::size_t node = 0; node < fixture.selectedSceneNodeCount; ++node) {
    auto row
        = makeSceneRuntimeAssemblyRow(body, node, incidentTriangleCounts[node]);
    row.bodyIndex = static_cast<std::uint32_t>(node);
    fixture.rows.push_back(row);
  }

  fixture.blocks.reserve(3u);
  const auto appendSelectedSparseBlock
      = [&fixture, &body](const std::size_t nodeA, const std::size_t nodeB) {
          if (nodeA >= fixture.selectedSceneNodeCount
              || nodeB >= fixture.selectedSceneNodeCount) {
            return;
          }
          auto block = makeSceneRuntimeSparseBlock(body, nodeA, nodeB);
          block.rowBodyIndex = static_cast<std::uint32_t>(nodeA);
          block.columnBodyIndex = static_cast<std::uint32_t>(nodeB);
          fixture.blocks.push_back(block);
          ++fixture.selectedSceneEdgePairCount;
        };
  for (std::size_t triangle = 0; triangle < body.getSurfaceTriangleCount();
       ++triangle) {
    const auto surfaceTriangle = body.getSurfaceTriangle(triangle);
    appendSelectedSparseBlock(surfaceTriangle.nodeA, surfaceTriangle.nodeB);
    appendSelectedSparseBlock(surfaceTriangle.nodeB, surfaceTriangle.nodeC);
    appendSelectedSparseBlock(surfaceTriangle.nodeC, surfaceTriangle.nodeA);
  }

  evaluateCpuDirectSparseSolve(
      fixture.rows, fixture.bodyCount, fixture.blocks, fixture.cpu);
  return fixture;
}

EqualityReducedSolveFixture makeEqualityReducedFixture(const int rowCount)
{
  EqualityReducedSolveFixture fixture;
  fixture.bodyCount
      = std::max<std::size_t>(1, static_cast<std::size_t>(rowCount) / 8);
  const std::size_t fullDofCount
      = fixture.bodyCount * cuda::kNewtonAssemblySolveDofsPerBody;
  fixture.reducedDofCount = std::max<std::size_t>(1, fullDofCount / 2);
  fixture.rows.reserve(static_cast<std::size_t>(rowCount));
  for (int row = 0; row < rowCount; ++row) {
    fixture.rows.push_back(makeRow(row, fixture.bodyCount));
  }
  fixture.entries.reserve(fullDofCount);
  for (std::size_t dof = 0; dof < fullDofCount; ++dof) {
    fixture.entries.push_back(
        makeEqualityReductionEntry(dof, dof % fixture.reducedDofCount));
  }
  evaluateCpuEqualityReducedSolve(
      fixture.rows,
      fixture.bodyCount,
      fixture.entries,
      fixture.reducedDofCount,
      fixture.cpu);
  return fixture;
}

SceneRuntimeEqualityReducedSolveFixture makeSceneRuntimeEqualityReducedFixture(
    const int rowCount)
{
  sx::World world;
  world.setTimeStep(kSceneRuntimeAssemblyTimeStep);
  const sx::DeformableBody body = world.addDeformableBody(
      "plan083_scene_runtime_equality_reduced",
      makeSceneRuntimeAssemblyBodyOptions(rowCount));

  SceneRuntimeEqualityReducedSolveFixture fixture;
  populateSceneRuntimeDiagonalRows(fixture, world, body);
  fixture.reducedDofCount = 3u * fixture.sceneNodeCount;
  const std::size_t fullDofCount
      = fixture.bodyCount * cuda::kNewtonAssemblySolveDofsPerBody;
  fixture.entries.reserve(fullDofCount);
  for (std::size_t node = 0; node < fixture.sceneNodeCount; ++node) {
    const std::size_t fullOffset = node * cuda::kNewtonAssemblySolveDofsPerBody;
    const std::size_t reducedOffset = node * 3u;
    for (std::size_t dof = 0; dof < cuda::kNewtonAssemblySolveDofsPerBody;
         ++dof) {
      fixture.entries.push_back(makeSceneRuntimeEqualityReductionEntry(
          fullOffset + dof, reducedOffset + (dof % 3u), dof));
    }
  }

  evaluateCpuEqualityReducedSolve(
      fixture.rows,
      fixture.bodyCount,
      fixture.entries,
      fixture.reducedDofCount,
      fixture.cpu);
  return fixture;
}

template <typename Lhs, typename Rhs>
double maxAbsDifference(const Lhs& lhs, const Rhs& rhs)
{
  double maxError = 0.0;
  for (std::size_t i = 0; i < lhs.size(); ++i) {
    maxError = std::max(maxError, std::abs(lhs[i] - rhs[i]));
  }
  return maxError;
}

double maxOutputError(
    const CpuAssemblySolveResult& expected,
    const cuda::NewtonAssemblySolveResult& actual)
{
  double maxError = 0.0;
  maxError = std::max(
      maxError,
      maxAbsDifference(expected.assembledDiagonal, actual.assembledDiagonal));
  maxError = std::max(
      maxError,
      maxAbsDifference(expected.assembledGradient, actual.assembledGradient));
  maxError = std::max(maxError, maxAbsDifference(expected.step, actual.step));
  maxError = std::max(
      maxError, maxAbsDifference(expected.residual, actual.residual));
  return maxError;
}

double maxOffDiagonalOutputError(
    const CpuOffDiagonalAssemblyResult& expected,
    const cuda::NewtonOffDiagonalAssemblyResult& actual)
{
  return maxAbsDifference(expected.assembledBlocks, actual.assembledBlocks);
}

double maxSceneSparseGraphAssemblyOutputError(
    const CpuSceneSparseGraphAssemblyResult& expected,
    const cuda::NewtonSceneSparseGraphAssemblyResult& actual)
{
  double maxError = 0.0;
  for (std::size_t row = 0; row < expected.rows.size(); ++row) {
    maxError = std::max(
        maxError,
        std::abs(
            static_cast<double>(expected.rows[row].bodyIndex)
            - static_cast<double>(actual.rows[row].bodyIndex)));
    for (std::size_t dof = 0; dof < cuda::kNewtonAssemblySolveDofsPerBody;
         ++dof) {
      maxError = std::max(
          maxError,
          std::abs(
              expected.rows[row].hessianDiagonal[dof]
              - actual.rows[row].hessianDiagonal[dof]));
      maxError = std::max(
          maxError,
          std::abs(
              expected.rows[row].gradient[dof]
              - actual.rows[row].gradient[dof]));
    }
  }

  for (std::size_t block = 0; block < expected.blocks.size(); ++block) {
    maxError = std::max(
        maxError,
        std::abs(
            static_cast<double>(expected.blocks[block].rowBodyIndex)
            - static_cast<double>(actual.blocks[block].rowBodyIndex)));
    maxError = std::max(
        maxError,
        std::abs(
            static_cast<double>(expected.blocks[block].columnBodyIndex)
            - static_cast<double>(actual.blocks[block].columnBodyIndex)));
    for (std::size_t entry = 0; entry < cuda::kNewtonAssemblySolveBlockEntries;
         ++entry) {
      maxError = std::max(
          maxError,
          std::abs(
              expected.blocks[block].hessianBlock[entry]
              - actual.blocks[block].hessianBlock[entry]));
    }
  }
  return maxError;
}

double maxSceneSparseGraphUniqueAssemblyOutputError(
    const CpuSceneSparseGraphUniqueAssemblyResult& expected,
    const cuda::NewtonSceneSparseGraphUniqueAssemblyResult& actual)
{
  double maxError = 0.0;
  for (std::size_t row = 0; row < expected.rows.size(); ++row) {
    maxError = std::max(
        maxError,
        std::abs(
            static_cast<double>(expected.rows[row].bodyIndex)
            - static_cast<double>(actual.rows[row].bodyIndex)));
    for (std::size_t dof = 0; dof < cuda::kNewtonAssemblySolveDofsPerBody;
         ++dof) {
      maxError = std::max(
          maxError,
          std::abs(
              expected.rows[row].hessianDiagonal[dof]
              - actual.rows[row].hessianDiagonal[dof]));
      maxError = std::max(
          maxError,
          std::abs(
              expected.rows[row].gradient[dof]
              - actual.rows[row].gradient[dof]));
    }
  }

  for (std::size_t block = 0; block < expected.blocks.size(); ++block) {
    maxError = std::max(
        maxError,
        std::abs(
            static_cast<double>(expected.blocks[block].rowBodyIndex)
            - static_cast<double>(actual.blocks[block].rowBodyIndex)));
    maxError = std::max(
        maxError,
        std::abs(
            static_cast<double>(expected.blocks[block].columnBodyIndex)
            - static_cast<double>(actual.blocks[block].columnBodyIndex)));
    for (std::size_t entry = 0; entry < cuda::kNewtonAssemblySolveBlockEntries;
         ++entry) {
      maxError = std::max(
          maxError,
          std::abs(
              expected.blocks[block].hessianBlock[entry]
              - actual.blocks[block].hessianBlock[entry]));
    }
  }
  return maxError;
}

double maxSceneNonlinearEqualityAssemblyOutputError(
    const CpuSceneNonlinearEqualityAssemblyResult& expected,
    const cuda::NewtonSceneNonlinearEqualityAssemblyResult& actual)
{
  double maxError = 0.0;
  maxError = std::max(
      maxError, maxAbsDifference(expected.residuals, actual.residuals));

  for (std::size_t row = 0; row < expected.rows.size(); ++row) {
    maxError = std::max(
        maxError,
        std::abs(
            static_cast<double>(expected.rows[row].bodyIndex)
            - static_cast<double>(actual.rows[row].bodyIndex)));
    for (std::size_t dof = 0; dof < cuda::kNewtonAssemblySolveDofsPerBody;
         ++dof) {
      maxError = std::max(
          maxError,
          std::abs(
              expected.rows[row].hessianDiagonal[dof]
              - actual.rows[row].hessianDiagonal[dof]));
      maxError = std::max(
          maxError,
          std::abs(
              expected.rows[row].gradient[dof]
              - actual.rows[row].gradient[dof]));
    }
  }

  for (std::size_t block = 0; block < expected.blocks.size(); ++block) {
    maxError = std::max(
        maxError,
        std::abs(
            static_cast<double>(expected.blocks[block].rowBodyIndex)
            - static_cast<double>(actual.blocks[block].rowBodyIndex)));
    maxError = std::max(
        maxError,
        std::abs(
            static_cast<double>(expected.blocks[block].columnBodyIndex)
            - static_cast<double>(actual.blocks[block].columnBodyIndex)));
    for (std::size_t entry = 0; entry < cuda::kNewtonAssemblySolveBlockEntries;
         ++entry) {
      maxError = std::max(
          maxError,
          std::abs(
              expected.blocks[block].hessianBlock[entry]
              - actual.blocks[block].hessianBlock[entry]));
    }
  }
  return maxError;
}

double maxSceneNonlinearEqualitySolveOutputError(
    const CpuSceneNonlinearEqualitySolveResult& expected,
    const cuda::NewtonSceneNonlinearEqualitySolveResult& actual)
{
  double maxError = 0.0;
  maxError = std::max(
      maxError, maxAbsDifference(expected.residuals, actual.residuals));
  maxError = std::max(maxError, maxAbsDifference(expected.step, actual.step));
  maxError = std::max(
      maxError,
      maxAbsDifference(
          expected.postSolveLinearizedResiduals,
          actual.postSolveLinearizedResiduals));

  for (std::size_t row = 0; row < expected.rows.size(); ++row) {
    maxError = std::max(
        maxError,
        std::abs(
            static_cast<double>(expected.rows[row].bodyIndex)
            - static_cast<double>(actual.rows[row].bodyIndex)));
    for (std::size_t dof = 0; dof < cuda::kNewtonAssemblySolveDofsPerBody;
         ++dof) {
      maxError = std::max(
          maxError,
          std::abs(
              expected.rows[row].hessianDiagonal[dof]
              - actual.rows[row].hessianDiagonal[dof]));
      maxError = std::max(
          maxError,
          std::abs(
              expected.rows[row].gradient[dof]
              - actual.rows[row].gradient[dof]));
    }
  }

  for (std::size_t block = 0; block < expected.blocks.size(); ++block) {
    maxError = std::max(
        maxError,
        std::abs(
            static_cast<double>(expected.blocks[block].rowBodyIndex)
            - static_cast<double>(actual.blocks[block].rowBodyIndex)));
    maxError = std::max(
        maxError,
        std::abs(
            static_cast<double>(expected.blocks[block].columnBodyIndex)
            - static_cast<double>(actual.blocks[block].columnBodyIndex)));
    for (std::size_t entry = 0; entry < cuda::kNewtonAssemblySolveBlockEntries;
         ++entry) {
      maxError = std::max(
          maxError,
          std::abs(
              expected.blocks[block].hessianBlock[entry]
              - actual.blocks[block].hessianBlock[entry]));
    }
  }
  return maxError;
}

double maxSceneNonlinearEqualityConvergenceOutputError(
    const CpuSceneNonlinearEqualityConvergenceResult& expected,
    const cuda::NewtonSceneNonlinearEqualityConvergenceResult& actual)
{
  double maxError = 0.0;
  maxError = std::max(
      maxError,
      maxAbsDifference(expected.initialResiduals, actual.initialResiduals));
  maxError = std::max(
      maxError,
      maxAbsDifference(expected.finalResiduals, actual.finalResiduals));
  maxError = std::max(maxError, maxAbsDifference(expected.step, actual.step));

  for (std::size_t row = 0; row < expected.rows.size(); ++row) {
    maxError = std::max(
        maxError,
        std::abs(
            static_cast<double>(expected.rows[row].bodyIndex)
            - static_cast<double>(actual.rows[row].bodyIndex)));
    for (std::size_t dof = 0; dof < cuda::kNewtonAssemblySolveDofsPerBody;
         ++dof) {
      maxError = std::max(
          maxError,
          std::abs(
              expected.rows[row].hessianDiagonal[dof]
              - actual.rows[row].hessianDiagonal[dof]));
      maxError = std::max(
          maxError,
          std::abs(
              expected.rows[row].gradient[dof]
              - actual.rows[row].gradient[dof]));
    }
  }

  for (std::size_t block = 0; block < expected.blocks.size(); ++block) {
    maxError = std::max(
        maxError,
        std::abs(
            static_cast<double>(expected.blocks[block].rowBodyIndex)
            - static_cast<double>(actual.blocks[block].rowBodyIndex)));
    maxError = std::max(
        maxError,
        std::abs(
            static_cast<double>(expected.blocks[block].columnBodyIndex)
            - static_cast<double>(actual.blocks[block].columnBodyIndex)));
    for (std::size_t entry = 0; entry < cuda::kNewtonAssemblySolveBlockEntries;
         ++entry) {
      maxError = std::max(
          maxError,
          std::abs(
              expected.blocks[block].hessianBlock[entry]
              - actual.blocks[block].hessianBlock[entry]));
    }
  }
  return maxError;
}

double maxSparseResidualOutputError(
    const CpuSparseResidualResult& expected,
    const cuda::NewtonSparseResidualResult& actual)
{
  double maxError = 0.0;
  maxError = std::max(
      maxError,
      maxAbsDifference(expected.assembledDiagonal, actual.assembledDiagonal));
  maxError = std::max(
      maxError,
      maxAbsDifference(expected.assembledGradient, actual.assembledGradient));
  maxError = std::max(
      maxError, maxAbsDifference(expected.residual, actual.residual));
  return maxError;
}

double maxSparseJacobiSolveOutputError(
    const CpuSparseJacobiSolveResult& expected,
    const cuda::NewtonSparseJacobiSolveResult& actual)
{
  double maxError = 0.0;
  maxError = std::max(
      maxError,
      maxAbsDifference(expected.assembledDiagonal, actual.assembledDiagonal));
  maxError = std::max(
      maxError,
      maxAbsDifference(expected.assembledGradient, actual.assembledGradient));
  maxError = std::max(maxError, maxAbsDifference(expected.step, actual.step));
  maxError = std::max(
      maxError, maxAbsDifference(expected.residual, actual.residual));
  return maxError;
}

double maxSparseCgSolveOutputError(
    const CpuSparseCgSolveResult& expected,
    const cuda::NewtonSparseCgSolveResult& actual)
{
  double maxError = 0.0;
  maxError = std::max(
      maxError,
      maxAbsDifference(expected.assembledDiagonal, actual.assembledDiagonal));
  maxError = std::max(
      maxError,
      maxAbsDifference(expected.assembledGradient, actual.assembledGradient));
  maxError = std::max(maxError, maxAbsDifference(expected.step, actual.step));
  maxError = std::max(
      maxError, maxAbsDifference(expected.residual, actual.residual));
  return maxError;
}

double maxDirectSparseSolveOutputError(
    const CpuDirectSparseSolveResult& expected,
    const cuda::NewtonDirectSparseSolveResult& actual)
{
  double maxError = 0.0;
  maxError = std::max(
      maxError,
      maxAbsDifference(expected.assembledDiagonal, actual.assembledDiagonal));
  maxError = std::max(
      maxError,
      maxAbsDifference(expected.assembledGradient, actual.assembledGradient));
  maxError = std::max(
      maxError,
      maxAbsDifference(expected.factorMatrixLower, actual.factorMatrixLower));
  maxError = std::max(maxError, maxAbsDifference(expected.step, actual.step));
  maxError = std::max(
      maxError, maxAbsDifference(expected.residual, actual.residual));
  return maxError;
}

double maxEqualityReducedOutputError(
    const CpuEqualityReducedSolveResult& expected,
    const cuda::NewtonEqualityReducedSolveResult& actual)
{
  double maxError = 0.0;
  maxError = std::max(
      maxError,
      maxAbsDifference(expected.assembledDiagonal, actual.assembledDiagonal));
  maxError = std::max(
      maxError,
      maxAbsDifference(expected.assembledGradient, actual.assembledGradient));
  maxError = std::max(
      maxError,
      maxAbsDifference(expected.reducedDiagonal, actual.reducedDiagonal));
  maxError = std::max(
      maxError,
      maxAbsDifference(expected.reducedGradient, actual.reducedGradient));
  maxError = std::max(
      maxError, maxAbsDifference(expected.reducedStep, actual.reducedStep));
  maxError = std::max(
      maxError,
      maxAbsDifference(expected.reducedResidual, actual.reducedResidual));
  maxError = std::max(
      maxError, maxAbsDifference(expected.fullStep, actual.fullStep));
  return maxError;
}

void recordCounters(
    benchmark::State& state,
    const AssemblySolveFixture& fixture,
    const double maxError)
{
  state.counters["rows"] = static_cast<double>(fixture.rows.size());
  state.counters["bodies"] = static_cast<double>(fixture.bodyCount);
  state.counters["dofs"] = static_cast<double>(
      fixture.bodyCount * cuda::kNewtonAssemblySolveDofsPerBody);
  state.counters["active_dofs"]
      = static_cast<double>(fixture.cpu.activeDofCount);
  state.counters["max_diagonal"] = fixture.cpu.maxDiagonal;
  state.counters["max_gradient_abs"] = fixture.cpu.maxGradientAbs;
  state.counters["step_norm"] = fixture.cpu.stepNorm;
  state.counters["residual_norm"] = fixture.cpu.residualNorm;
  state.counters["max_result_abs_error"] = maxError;
  state.SetItemsProcessed(
      static_cast<std::int64_t>(state.iterations() * fixture.rows.size()));
}

void recordSceneRuntimeAssemblyCounters(
    benchmark::State& state,
    const SceneRuntimeAssemblySolveFixture& fixture,
    const double maxError)
{
  state.counters["rows"] = static_cast<double>(fixture.rows.size());
  state.counters["bodies"] = static_cast<double>(fixture.bodyCount);
  state.counters["dofs"] = static_cast<double>(
      fixture.bodyCount * cuda::kNewtonAssemblySolveDofsPerBody);
  state.counters["active_dofs"]
      = static_cast<double>(fixture.cpu.activeDofCount);
  state.counters["scene_bodies"] = static_cast<double>(fixture.sceneBodyCount);
  state.counters["scene_nodes"] = static_cast<double>(fixture.sceneNodeCount);
  state.counters["scene_triangles"]
      = static_cast<double>(fixture.sceneTriangleCount);
  state.counters["max_diagonal"] = fixture.cpu.maxDiagonal;
  state.counters["max_gradient_abs"] = fixture.cpu.maxGradientAbs;
  state.counters["step_norm"] = fixture.cpu.stepNorm;
  state.counters["residual_norm"] = fixture.cpu.residualNorm;
  state.counters["max_result_abs_error"] = maxError;
  state.SetItemsProcessed(
      static_cast<std::int64_t>(state.iterations() * fixture.rows.size()));
}

void recordOffDiagonalCounters(
    benchmark::State& state,
    const OffDiagonalAssemblyFixture& fixture,
    const double maxError)
{
  state.counters["rows"] = static_cast<double>(fixture.rows.size());
  state.counters["pairs"] = static_cast<double>(fixture.pairCount);
  state.counters["block_entries"] = static_cast<double>(
      fixture.pairCount * cuda::kNewtonAssemblySolveBlockEntries);
  state.counters["active_blocks"]
      = static_cast<double>(fixture.cpu.activeBlockCount);
  state.counters["max_block_abs"] = fixture.cpu.maxBlockAbs;
  state.counters["max_result_abs_error"] = maxError;
  state.SetItemsProcessed(
      static_cast<std::int64_t>(state.iterations() * fixture.rows.size()));
}

void recordSceneRuntimeOffDiagonalCounters(
    benchmark::State& state,
    const SceneRuntimeOffDiagonalAssemblyFixture& fixture,
    const double maxError)
{
  recordOffDiagonalCounters(state, fixture, maxError);
  state.counters["scene_bodies"] = static_cast<double>(fixture.sceneBodyCount);
  state.counters["scene_nodes"] = static_cast<double>(fixture.sceneNodeCount);
  state.counters["scene_triangles"]
      = static_cast<double>(fixture.sceneTriangleCount);
  state.counters["scene_edge_pairs"]
      = static_cast<double>(fixture.sceneEdgePairCount);
}

void recordSceneRuntimeSparseGraphAssemblyCounters(
    benchmark::State& state,
    const SceneRuntimeSparseGraphAssemblyFixture& fixture,
    const double maxError)
{
  state.counters["rows"] = static_cast<double>(fixture.cpu.rowCount);
  state.counters["bodies"] = static_cast<double>(fixture.cpu.bodyCount);
  state.counters["dofs"] = static_cast<double>(fixture.cpu.dofCount);
  state.counters["blocks"] = static_cast<double>(fixture.cpu.blockCount);
  state.counters["block_entries"]
      = static_cast<double>(fixture.cpu.blockEntryCount);
  state.counters["scene_bodies"] = static_cast<double>(fixture.sceneBodyCount);
  state.counters["scene_nodes"] = static_cast<double>(fixture.cpu.nodeCount);
  state.counters["scene_triangles"]
      = static_cast<double>(fixture.cpu.triangleCount);
  state.counters["scene_edge_pairs"]
      = static_cast<double>(fixture.cpu.blockCount);
  state.counters["max_diagonal"] = fixture.cpu.maxDiagonal;
  state.counters["max_gradient_abs"] = fixture.cpu.maxGradientAbs;
  state.counters["max_block_abs"] = fixture.cpu.maxBlockAbs;
  state.counters["max_result_abs_error"] = maxError;
  state.SetItemsProcessed(
      static_cast<std::int64_t>(
          state.iterations()
          * (fixture.cpu.rowCount + fixture.cpu.blockCount)));
}

void recordSceneRuntimeSparseGraphUniqueAssemblyCounters(
    benchmark::State& state,
    const SceneRuntimeSparseGraphUniqueAssemblyFixture& fixture,
    const double maxError)
{
  state.counters["rows"] = static_cast<double>(fixture.cpu.rowCount);
  state.counters["bodies"] = static_cast<double>(fixture.cpu.bodyCount);
  state.counters["dofs"] = static_cast<double>(fixture.cpu.dofCount);
  state.counters["blocks"] = static_cast<double>(fixture.cpu.blockCount);
  state.counters["block_entries"]
      = static_cast<double>(fixture.cpu.blockEntryCount);
  state.counters["edge_slots"] = static_cast<double>(fixture.cpu.edgeSlotCount);
  state.counters["unique_edges"]
      = static_cast<double>(fixture.cpu.uniqueEdgeCount);
  state.counters["duplicate_edge_slots"]
      = static_cast<double>(fixture.cpu.duplicateEdgeSlotCount);
  state.counters["scene_bodies"] = static_cast<double>(fixture.sceneBodyCount);
  state.counters["scene_nodes"] = static_cast<double>(fixture.cpu.nodeCount);
  state.counters["scene_triangles"]
      = static_cast<double>(fixture.cpu.triangleCount);
  state.counters["max_diagonal"] = fixture.cpu.maxDiagonal;
  state.counters["max_gradient_abs"] = fixture.cpu.maxGradientAbs;
  state.counters["max_block_abs"] = fixture.cpu.maxBlockAbs;
  state.counters["max_result_abs_error"] = maxError;
  state.SetItemsProcessed(
      static_cast<std::int64_t>(
          state.iterations()
          * (fixture.cpu.rowCount + fixture.cpu.edgeSlotCount)));
}

void recordSceneRuntimeNonlinearEqualityAssemblyCounters(
    benchmark::State& state,
    const SceneRuntimeNonlinearEqualityAssemblyFixture& fixture,
    const double maxError)
{
  state.counters["rows"] = static_cast<double>(fixture.cpu.rowCount);
  state.counters["bodies"] = static_cast<double>(fixture.cpu.bodyCount);
  state.counters["dofs"] = static_cast<double>(fixture.cpu.dofCount);
  state.counters["constraints"]
      = static_cast<double>(fixture.cpu.constraintCount);
  state.counters["blocks"] = static_cast<double>(fixture.cpu.blockCount);
  state.counters["block_entries"]
      = static_cast<double>(fixture.cpu.blockEntryCount);
  state.counters["scene_bodies"] = static_cast<double>(fixture.sceneBodyCount);
  state.counters["scene_nodes"] = static_cast<double>(fixture.cpu.nodeCount);
  state.counters["scene_triangles"]
      = static_cast<double>(fixture.sceneTriangleCount);
  state.counters["scene_edge_pairs"]
      = static_cast<double>(fixture.sceneEdgePairCount);
  state.counters["max_constraint_residual_abs"]
      = fixture.cpu.maxConstraintResidualAbs;
  state.counters["max_diagonal"] = fixture.cpu.maxDiagonal;
  state.counters["max_gradient_abs"] = fixture.cpu.maxGradientAbs;
  state.counters["max_block_abs"] = fixture.cpu.maxBlockAbs;
  state.counters["max_result_abs_error"] = maxError;
  state.SetItemsProcessed(
      static_cast<std::int64_t>(
          state.iterations()
          * (fixture.cpu.rowCount + fixture.cpu.blockCount)));
}

void recordSceneRuntimeNonlinearEqualitySolveCounters(
    benchmark::State& state,
    const SceneRuntimeNonlinearEqualitySolveFixture& fixture,
    const double maxError)
{
  state.counters["rows"] = static_cast<double>(fixture.cpu.rowCount);
  state.counters["bodies"] = static_cast<double>(fixture.cpu.bodyCount);
  state.counters["dofs"] = static_cast<double>(fixture.cpu.dofCount);
  state.counters["constraints"]
      = static_cast<double>(fixture.cpu.constraintCount);
  state.counters["blocks"] = static_cast<double>(fixture.cpu.blockCount);
  state.counters["block_entries"]
      = static_cast<double>(fixture.cpu.blockEntryCount);
  state.counters["active_dofs"]
      = static_cast<double>(fixture.cpu.activeDofCount);
  state.counters["scene_bodies"] = static_cast<double>(fixture.sceneBodyCount);
  state.counters["scene_nodes"] = static_cast<double>(fixture.cpu.nodeCount);
  state.counters["scene_triangles"]
      = static_cast<double>(fixture.sceneTriangleCount);
  state.counters["scene_edge_pairs"]
      = static_cast<double>(fixture.sceneEdgePairCount);
  state.counters["regularization"] = kSceneNonlinearEqualitySolveRegularization;
  state.counters["max_constraint_residual_abs"]
      = fixture.cpu.maxConstraintResidualAbs;
  state.counters["max_post_solve_linearized_residual_abs"]
      = fixture.cpu.maxPostSolveLinearizedResidualAbs;
  state.counters["max_diagonal"] = fixture.cpu.maxDiagonal;
  state.counters["max_gradient_abs"] = fixture.cpu.maxGradientAbs;
  state.counters["max_block_abs"] = fixture.cpu.maxBlockAbs;
  state.counters["step_norm"] = fixture.cpu.stepNorm;
  state.counters["max_result_abs_error"] = maxError;
  state.SetItemsProcessed(
      static_cast<std::int64_t>(
          state.iterations()
          * (fixture.cpu.rowCount + fixture.cpu.blockCount
             + fixture.cpu.dofCount)));
}

void recordSceneRuntimeNonlinearEqualityConvergenceCounters(
    benchmark::State& state,
    const SceneRuntimeNonlinearEqualityConvergenceFixture& fixture,
    const double maxError)
{
  state.counters["rows"] = static_cast<double>(fixture.cpu.rowCount);
  state.counters["bodies"] = static_cast<double>(fixture.cpu.bodyCount);
  state.counters["dofs"] = static_cast<double>(fixture.cpu.dofCount);
  state.counters["constraints"]
      = static_cast<double>(fixture.cpu.constraintCount);
  state.counters["blocks"] = static_cast<double>(fixture.cpu.blockCount);
  state.counters["block_entries"]
      = static_cast<double>(fixture.cpu.blockEntryCount);
  state.counters["active_dofs"]
      = static_cast<double>(fixture.cpu.activeDofCount);
  state.counters["max_iterations"]
      = static_cast<double>(fixture.cpu.maxIterationCount);
  state.counters["completed_iterations"]
      = static_cast<double>(fixture.cpu.completedIterationCount);
  state.counters["scene_bodies"] = static_cast<double>(fixture.sceneBodyCount);
  state.counters["scene_nodes"] = static_cast<double>(fixture.cpu.nodeCount);
  state.counters["scene_triangles"]
      = static_cast<double>(fixture.sceneTriangleCount);
  state.counters["scene_edge_pairs"]
      = static_cast<double>(fixture.sceneEdgePairCount);
  state.counters["regularization"] = kSceneNonlinearEqualitySolveRegularization;
  state.counters["residual_tolerance"]
      = kSceneNonlinearEqualityConvergenceResidualTolerance;
  state.counters["converged"] = fixture.cpu.converged ? 1.0 : 0.0;
  state.counters["initial_max_constraint_residual_abs"]
      = fixture.cpu.initialMaxConstraintResidualAbs;
  state.counters["final_max_constraint_residual_abs"]
      = fixture.cpu.finalMaxConstraintResidualAbs;
  state.counters["max_diagonal"] = fixture.cpu.maxDiagonal;
  state.counters["max_gradient_abs"] = fixture.cpu.maxGradientAbs;
  state.counters["max_block_abs"] = fixture.cpu.maxBlockAbs;
  state.counters["step_norm"] = fixture.cpu.stepNorm;
  state.counters["max_result_abs_error"] = maxError;
  state.SetItemsProcessed(
      static_cast<std::int64_t>(
          state.iterations()
          * (fixture.cpu.rowCount + fixture.cpu.blockCount
             + fixture.cpu.dofCount)));
}

void recordSparseResidualCounters(
    benchmark::State& state,
    const SparseResidualFixture& fixture,
    const double maxError)
{
  state.counters["rows"] = static_cast<double>(fixture.rows.size());
  state.counters["bodies"] = static_cast<double>(fixture.bodyCount);
  state.counters["dofs"] = static_cast<double>(fixture.cpu.dofCount);
  state.counters["blocks"] = static_cast<double>(fixture.cpu.blockCount);
  state.counters["block_entries"] = static_cast<double>(
      fixture.cpu.blockCount * cuda::kNewtonAssemblySolveBlockEntries);
  state.counters["active_dofs"]
      = static_cast<double>(fixture.cpu.activeDofCount);
  state.counters["max_diagonal"] = fixture.cpu.maxDiagonal;
  state.counters["max_gradient_abs"] = fixture.cpu.maxGradientAbs;
  state.counters["output_norm"] = fixture.cpu.residualNorm;
  state.counters["max_output_abs"] = fixture.cpu.maxResidualAbs;
  state.counters["max_result_abs_error"] = maxError;
  state.SetItemsProcessed(
      static_cast<std::int64_t>(state.iterations() * fixture.rows.size()));
}

void recordSceneRuntimeSparseResidualCounters(
    benchmark::State& state,
    const SceneRuntimeSparseResidualFixture& fixture,
    const double maxError)
{
  recordSparseResidualCounters(state, fixture, maxError);
  state.counters["scene_bodies"] = static_cast<double>(fixture.sceneBodyCount);
  state.counters["scene_nodes"] = static_cast<double>(fixture.sceneNodeCount);
  state.counters["scene_triangles"]
      = static_cast<double>(fixture.sceneTriangleCount);
  state.counters["scene_edge_pairs"]
      = static_cast<double>(fixture.sceneEdgePairCount);
}

void recordSparseJacobiSolveCounters(
    benchmark::State& state,
    const SparseJacobiSolveFixture& fixture,
    const double maxError)
{
  state.counters["rows"] = static_cast<double>(fixture.rows.size());
  state.counters["bodies"] = static_cast<double>(fixture.bodyCount);
  state.counters["dofs"] = static_cast<double>(fixture.cpu.dofCount);
  state.counters["blocks"] = static_cast<double>(fixture.cpu.blockCount);
  state.counters["block_entries"] = static_cast<double>(
      fixture.cpu.blockCount * cuda::kNewtonAssemblySolveBlockEntries);
  state.counters["iterations"]
      = static_cast<double>(fixture.cpu.iterationCount);
  state.counters["active_dofs"]
      = static_cast<double>(fixture.cpu.activeDofCount);
  state.counters["max_diagonal"] = fixture.cpu.maxDiagonal;
  state.counters["max_gradient_abs"] = fixture.cpu.maxGradientAbs;
  state.counters["step_norm"] = fixture.cpu.stepNorm;
  state.counters["residual_norm"] = fixture.cpu.residualNorm;
  state.counters["max_residual_abs"] = fixture.cpu.maxResidualAbs;
  state.counters["max_result_abs_error"] = maxError;
  state.SetItemsProcessed(
      static_cast<std::int64_t>(state.iterations() * fixture.rows.size()));
}

void recordSceneRuntimeSparseJacobiSolveCounters(
    benchmark::State& state,
    const SceneRuntimeSparseJacobiSolveFixture& fixture,
    const double maxError)
{
  recordSparseJacobiSolveCounters(state, fixture, maxError);
  state.counters["scene_bodies"] = static_cast<double>(fixture.sceneBodyCount);
  state.counters["scene_nodes"] = static_cast<double>(fixture.sceneNodeCount);
  state.counters["scene_triangles"]
      = static_cast<double>(fixture.sceneTriangleCount);
  state.counters["scene_edge_pairs"]
      = static_cast<double>(fixture.sceneEdgePairCount);
}

void recordSparseCgSolveCounters(
    benchmark::State& state,
    const SparseCgSolveFixture& fixture,
    const double maxError)
{
  state.counters["rows"] = static_cast<double>(fixture.rows.size());
  state.counters["bodies"] = static_cast<double>(fixture.bodyCount);
  state.counters["dofs"] = static_cast<double>(fixture.cpu.dofCount);
  state.counters["blocks"] = static_cast<double>(fixture.cpu.blockCount);
  state.counters["block_entries"] = static_cast<double>(
      fixture.cpu.blockCount * cuda::kNewtonAssemblySolveBlockEntries);
  state.counters["max_iterations"]
      = static_cast<double>(fixture.cpu.maxIterationCount);
  state.counters["completed_iterations"]
      = static_cast<double>(fixture.cpu.completedIterationCount);
  state.counters["active_dofs"]
      = static_cast<double>(fixture.cpu.activeDofCount);
  state.counters["converged"] = fixture.cpu.converged ? 1.0 : 0.0;
  state.counters["residual_tolerance"] = fixture.cpu.residualTolerance;
  state.counters["initial_residual_norm"] = fixture.cpu.initialResidualNorm;
  state.counters["max_diagonal"] = fixture.cpu.maxDiagonal;
  state.counters["max_gradient_abs"] = fixture.cpu.maxGradientAbs;
  state.counters["step_norm"] = fixture.cpu.stepNorm;
  state.counters["residual_norm"] = fixture.cpu.residualNorm;
  state.counters["max_residual_abs"] = fixture.cpu.maxResidualAbs;
  state.counters["max_result_abs_error"] = maxError;
  state.SetItemsProcessed(
      static_cast<std::int64_t>(state.iterations() * fixture.rows.size()));
}

void recordSceneRuntimeSparseCgSolveCounters(
    benchmark::State& state,
    const SceneRuntimeSparseCgSolveFixture& fixture,
    const double maxError)
{
  recordSparseCgSolveCounters(state, fixture, maxError);
  state.counters["scene_bodies"] = static_cast<double>(fixture.sceneBodyCount);
  state.counters["scene_nodes"] = static_cast<double>(fixture.sceneNodeCount);
  state.counters["scene_triangles"]
      = static_cast<double>(fixture.sceneTriangleCount);
  state.counters["scene_edge_pairs"]
      = static_cast<double>(fixture.sceneEdgePairCount);
}

void recordDirectSparseSolveCounters(
    benchmark::State& state,
    const DirectSparseSolveFixture& fixture,
    const double maxError)
{
  state.counters["rows"] = static_cast<double>(fixture.rows.size());
  state.counters["bodies"] = static_cast<double>(fixture.bodyCount);
  state.counters["dofs"] = static_cast<double>(fixture.cpu.dofCount);
  state.counters["blocks"] = static_cast<double>(fixture.cpu.blockCount);
  state.counters["block_entries"] = static_cast<double>(
      fixture.cpu.blockCount * cuda::kNewtonAssemblySolveBlockEntries);
  state.counters["active_dofs"]
      = static_cast<double>(fixture.cpu.activeDofCount);
  state.counters["regularization"] = kRegularization;
  state.counters["factorized"] = fixture.cpu.factorized ? 1.0 : 0.0;
  state.counters["min_factor_pivot"] = fixture.cpu.minimumFactorPivot;
  state.counters["max_diagonal"] = fixture.cpu.maxDiagonal;
  state.counters["max_gradient_abs"] = fixture.cpu.maxGradientAbs;
  state.counters["step_norm"] = fixture.cpu.stepNorm;
  state.counters["residual_norm"] = fixture.cpu.residualNorm;
  state.counters["max_residual_abs"] = fixture.cpu.maxResidualAbs;
  state.counters["max_result_abs_error"] = maxError;
  state.SetItemsProcessed(
      static_cast<std::int64_t>(
          state.iterations()
          * (fixture.rows.size() + fixture.cpu.blockCount
             + fixture.cpu.dofCount)));
}

void recordSceneRuntimeDirectSparseSolveCounters(
    benchmark::State& state,
    const SceneRuntimeDirectSparseSolveFixture& fixture,
    const double maxError)
{
  recordDirectSparseSolveCounters(state, fixture, maxError);
  state.counters["scene_bodies"] = static_cast<double>(fixture.sceneBodyCount);
  state.counters["scene_nodes"] = static_cast<double>(fixture.sceneNodeCount);
  state.counters["scene_triangles"]
      = static_cast<double>(fixture.sceneTriangleCount);
  state.counters["selected_scene_nodes"]
      = static_cast<double>(fixture.selectedSceneNodeCount);
  state.counters["selected_scene_edge_pairs"]
      = static_cast<double>(fixture.selectedSceneEdgePairCount);
}

void recordEqualityReducedCounters(
    benchmark::State& state,
    const EqualityReducedSolveFixture& fixture,
    const double maxError)
{
  state.counters["rows"] = static_cast<double>(fixture.rows.size());
  state.counters["bodies"] = static_cast<double>(fixture.bodyCount);
  state.counters["dofs"] = static_cast<double>(fixture.cpu.fullDofCount);
  state.counters["reduction_entries"]
      = static_cast<double>(fixture.cpu.reductionEntryCount);
  state.counters["reduced_dofs"]
      = static_cast<double>(fixture.cpu.reducedDofCount);
  state.counters["active_reduced_dofs"]
      = static_cast<double>(fixture.cpu.activeReducedDofCount);
  state.counters["max_diagonal"] = fixture.cpu.maxDiagonal;
  state.counters["max_gradient_abs"] = fixture.cpu.maxGradientAbs;
  state.counters["step_norm"] = fixture.cpu.stepNorm;
  state.counters["residual_norm"] = fixture.cpu.residualNorm;
  state.counters["max_result_abs_error"] = maxError;
  state.SetItemsProcessed(
      static_cast<std::int64_t>(state.iterations() * fixture.rows.size()));
}

void recordSceneRuntimeEqualityReducedCounters(
    benchmark::State& state,
    const SceneRuntimeEqualityReducedSolveFixture& fixture,
    const double maxError)
{
  recordEqualityReducedCounters(state, fixture, maxError);
  state.counters["scene_bodies"] = static_cast<double>(fixture.sceneBodyCount);
  state.counters["scene_nodes"] = static_cast<double>(fixture.sceneNodeCount);
  state.counters["scene_triangles"]
      = static_cast<double>(fixture.sceneTriangleCount);
}

} // namespace

//==============================================================================
static void BM_NewtonAssemblySolveCpu(benchmark::State& state)
{
  const auto fixture = makeFixture(static_cast<int>(state.range(0)));
  CpuAssemblySolveResult result;

  for (auto _ : state) {
    evaluateCpu(fixture.rows, fixture.bodyCount, result);
    benchmark::DoNotOptimize(result.step.data());
  }

  recordCounters(state, fixture, 0.0);
}
BENCHMARK(BM_NewtonAssemblySolveCpu)->Arg(4096)->Arg(65536)->UseRealTime();

//==============================================================================
static void BM_NewtonAssemblySolveCuda(benchmark::State& state)
{
  if (!cuda::isCudaRuntimeAvailable()) {
    state.SkipWithError("CUDA runtime has no available device");
    return;
  }

  const auto fixture = makeFixture(static_cast<int>(state.range(0)));
  cuda::NewtonAssemblySolveResult result;

  for (auto _ : state) {
    cuda::evaluateNewtonAssemblySolveCuda(
        fixture.rows, fixture.bodyCount, kRegularization, result);
    benchmark::DoNotOptimize(result.step.data());
  }

  recordCounters(state, fixture, maxOutputError(fixture.cpu, result));
  state.counters["gpu_bodies"] = static_cast<double>(result.bodyCount);
  state.counters["gpu_rows"] = static_cast<double>(result.rowCount);
  state.counters["gpu_active_dofs"]
      = static_cast<double>(result.activeDofCount);
  state.counters["gpu_step_norm"] = result.stepNorm;
  state.counters["gpu_residual_norm"] = result.residualNorm;
  state.counters["host_setup_ns"] = result.timing.setupNs;
  state.counters["host_to_device_ns"] = result.timing.hostToDeviceNs;
  state.counters["assembly_kernel_ns"] = result.timing.assemblyKernelNs;
  state.counters["solve_kernel_ns"] = result.timing.solveKernelNs;
  state.counters["device_to_host_ns"] = result.timing.deviceToHostNs;
}
BENCHMARK(BM_NewtonAssemblySolveCuda)->Arg(4096)->Arg(65536)->UseRealTime();

//==============================================================================
static void BM_NewtonSceneRuntimeAssemblySolveCpu(benchmark::State& state)
{
  const auto fixture
      = makeSceneRuntimeAssemblySolveFixture(static_cast<int>(state.range(0)));
  CpuAssemblySolveResult result;

  for (auto _ : state) {
    evaluateCpu(fixture.rows, fixture.bodyCount, result);
    benchmark::DoNotOptimize(result.step.data());
  }

  recordSceneRuntimeAssemblyCounters(state, fixture, 0.0);
}
BENCHMARK(BM_NewtonSceneRuntimeAssemblySolveCpu)
    ->Arg(4096)
    ->Arg(65536)
    ->UseRealTime();

//==============================================================================
static void BM_NewtonSceneRuntimeAssemblySolveCuda(benchmark::State& state)
{
  if (!cuda::isCudaRuntimeAvailable()) {
    state.SkipWithError("CUDA runtime has no available device");
    return;
  }

  const auto fixture
      = makeSceneRuntimeAssemblySolveFixture(static_cast<int>(state.range(0)));
  cuda::NewtonAssemblySolveResult result;

  for (auto _ : state) {
    cuda::evaluateNewtonAssemblySolveCuda(
        fixture.rows, fixture.bodyCount, kRegularization, result);
    benchmark::DoNotOptimize(result.step.data());
  }

  recordSceneRuntimeAssemblyCounters(
      state, fixture, maxOutputError(fixture.cpu, result));
  state.counters["gpu_rows"] = static_cast<double>(result.rowCount);
  state.counters["gpu_bodies"] = static_cast<double>(result.bodyCount);
  state.counters["gpu_dofs"] = static_cast<double>(
      result.bodyCount * cuda::kNewtonAssemblySolveDofsPerBody);
  state.counters["gpu_active_dofs"]
      = static_cast<double>(result.activeDofCount);
  state.counters["gpu_scene_bodies"]
      = static_cast<double>(fixture.sceneBodyCount);
  state.counters["gpu_scene_nodes"]
      = static_cast<double>(fixture.sceneNodeCount);
  state.counters["gpu_scene_triangles"]
      = static_cast<double>(fixture.sceneTriangleCount);
  state.counters["gpu_step_norm"] = result.stepNorm;
  state.counters["gpu_residual_norm"] = result.residualNorm;
  state.counters["host_setup_ns"] = result.timing.setupNs;
  state.counters["host_to_device_ns"] = result.timing.hostToDeviceNs;
  state.counters["assembly_kernel_ns"] = result.timing.assemblyKernelNs;
  state.counters["solve_kernel_ns"] = result.timing.solveKernelNs;
  state.counters["device_to_host_ns"] = result.timing.deviceToHostNs;
}
BENCHMARK(BM_NewtonSceneRuntimeAssemblySolveCuda)
    ->Arg(4096)
    ->Arg(65536)
    ->UseRealTime();

//==============================================================================
static void BM_NewtonOffDiagonalAssemblyCpu(benchmark::State& state)
{
  const auto fixture = makeOffDiagonalFixture(static_cast<int>(state.range(0)));
  CpuOffDiagonalAssemblyResult result;

  for (auto _ : state) {
    evaluateCpuOffDiagonalAssembly(fixture.rows, fixture.pairCount, result);
    benchmark::DoNotOptimize(result.assembledBlocks.data());
  }

  recordOffDiagonalCounters(state, fixture, 0.0);
}
BENCHMARK(BM_NewtonOffDiagonalAssemblyCpu)
    ->Arg(4096)
    ->Arg(65536)
    ->UseRealTime();

//==============================================================================
static void BM_NewtonOffDiagonalAssemblyCuda(benchmark::State& state)
{
  if (!cuda::isCudaRuntimeAvailable()) {
    state.SkipWithError("CUDA runtime has no available device");
    return;
  }

  const auto fixture = makeOffDiagonalFixture(static_cast<int>(state.range(0)));
  cuda::NewtonOffDiagonalAssemblyResult result;

  for (auto _ : state) {
    cuda::evaluateNewtonOffDiagonalAssemblyCuda(
        fixture.rows, fixture.pairCount, result);
    benchmark::DoNotOptimize(result.assembledBlocks.data());
  }

  recordOffDiagonalCounters(
      state, fixture, maxOffDiagonalOutputError(fixture.cpu, result));
  state.counters["gpu_rows"] = static_cast<double>(result.rowCount);
  state.counters["gpu_pairs"] = static_cast<double>(result.pairCount);
  state.counters["gpu_active_blocks"]
      = static_cast<double>(result.activeBlockCount);
  state.counters["gpu_max_block_abs"] = result.maxBlockAbs;
  state.counters["host_setup_ns"] = result.timing.setupNs;
  state.counters["host_to_device_ns"] = result.timing.hostToDeviceNs;
  state.counters["assembly_kernel_ns"] = result.timing.assemblyKernelNs;
  state.counters["solve_kernel_ns"] = result.timing.solveKernelNs;
  state.counters["device_to_host_ns"] = result.timing.deviceToHostNs;
}
BENCHMARK(BM_NewtonOffDiagonalAssemblyCuda)
    ->Arg(4096)
    ->Arg(65536)
    ->UseRealTime();

//==============================================================================
static void BM_NewtonSceneRuntimeOffDiagonalAssemblyCpu(benchmark::State& state)
{
  const auto fixture = makeSceneRuntimeOffDiagonalAssemblyFixture(
      static_cast<int>(state.range(0)));
  CpuOffDiagonalAssemblyResult result;

  for (auto _ : state) {
    evaluateCpuOffDiagonalAssembly(fixture.rows, fixture.pairCount, result);
    benchmark::DoNotOptimize(result.assembledBlocks.data());
  }

  recordSceneRuntimeOffDiagonalCounters(state, fixture, 0.0);
}
BENCHMARK(BM_NewtonSceneRuntimeOffDiagonalAssemblyCpu)
    ->Arg(4096)
    ->Arg(65536)
    ->UseRealTime();

//==============================================================================
static void BM_NewtonSceneRuntimeOffDiagonalAssemblyCuda(
    benchmark::State& state)
{
  if (!cuda::isCudaRuntimeAvailable()) {
    state.SkipWithError("CUDA runtime has no available device");
    return;
  }

  const auto fixture = makeSceneRuntimeOffDiagonalAssemblyFixture(
      static_cast<int>(state.range(0)));
  cuda::NewtonOffDiagonalAssemblyResult result;

  for (auto _ : state) {
    cuda::evaluateNewtonOffDiagonalAssemblyCuda(
        fixture.rows, fixture.pairCount, result);
    benchmark::DoNotOptimize(result.assembledBlocks.data());
  }

  recordSceneRuntimeOffDiagonalCounters(
      state, fixture, maxOffDiagonalOutputError(fixture.cpu, result));
  state.counters["gpu_rows"] = static_cast<double>(result.rowCount);
  state.counters["gpu_pairs"] = static_cast<double>(result.pairCount);
  state.counters["gpu_active_blocks"]
      = static_cast<double>(result.activeBlockCount);
  state.counters["gpu_max_block_abs"] = result.maxBlockAbs;
  state.counters["gpu_scene_bodies"]
      = static_cast<double>(fixture.sceneBodyCount);
  state.counters["gpu_scene_nodes"]
      = static_cast<double>(fixture.sceneNodeCount);
  state.counters["gpu_scene_triangles"]
      = static_cast<double>(fixture.sceneTriangleCount);
  state.counters["gpu_scene_edge_pairs"]
      = static_cast<double>(fixture.sceneEdgePairCount);
  state.counters["host_setup_ns"] = result.timing.setupNs;
  state.counters["host_to_device_ns"] = result.timing.hostToDeviceNs;
  state.counters["assembly_kernel_ns"] = result.timing.assemblyKernelNs;
  state.counters["solve_kernel_ns"] = result.timing.solveKernelNs;
  state.counters["device_to_host_ns"] = result.timing.deviceToHostNs;
}
BENCHMARK(BM_NewtonSceneRuntimeOffDiagonalAssemblyCuda)
    ->Arg(4096)
    ->Arg(65536)
    ->UseRealTime();

//==============================================================================
static void BM_NewtonSceneRuntimeSparseGraphAssemblyCpu(benchmark::State& state)
{
  const auto fixture = makeSceneRuntimeSparseGraphAssemblyFixture(
      static_cast<int>(state.range(0)));
  CpuSceneSparseGraphAssemblyResult result;

  for (auto _ : state) {
    evaluateCpuSceneSparseGraphAssembly(
        fixture.nodes, fixture.triangles, result);
    benchmark::DoNotOptimize(result.rows.data());
    benchmark::DoNotOptimize(result.blocks.data());
  }

  recordSceneRuntimeSparseGraphAssemblyCounters(state, fixture, 0.0);
}
BENCHMARK(BM_NewtonSceneRuntimeSparseGraphAssemblyCpu)
    ->Arg(4096)
    ->Arg(65536)
    ->UseRealTime();

//==============================================================================
static void BM_NewtonSceneRuntimeSparseGraphAssemblyCuda(
    benchmark::State& state)
{
  if (!cuda::isCudaRuntimeAvailable()) {
    state.SkipWithError("CUDA runtime has no available device");
    return;
  }

  const auto fixture = makeSceneRuntimeSparseGraphAssemblyFixture(
      static_cast<int>(state.range(0)));
  cuda::NewtonSceneSparseGraphAssemblyResult result;

  for (auto _ : state) {
    cuda::evaluateNewtonSceneSparseGraphAssemblyCuda(
        fixture.nodes, fixture.triangles, result);
    benchmark::DoNotOptimize(result.rows.data());
    benchmark::DoNotOptimize(result.blocks.data());
  }

  recordSceneRuntimeSparseGraphAssemblyCounters(
      state,
      fixture,
      maxSceneSparseGraphAssemblyOutputError(fixture.cpu, result));
  state.counters["gpu_rows"] = static_cast<double>(result.rowCount);
  state.counters["gpu_bodies"] = static_cast<double>(result.bodyCount);
  state.counters["gpu_dofs"] = static_cast<double>(result.dofCount);
  state.counters["gpu_blocks"] = static_cast<double>(result.blockCount);
  state.counters["gpu_block_entries"]
      = static_cast<double>(result.blockEntryCount);
  state.counters["gpu_scene_bodies"]
      = static_cast<double>(fixture.sceneBodyCount);
  state.counters["gpu_scene_nodes"] = static_cast<double>(result.nodeCount);
  state.counters["gpu_scene_triangles"]
      = static_cast<double>(result.triangleCount);
  state.counters["gpu_scene_edge_pairs"]
      = static_cast<double>(result.blockCount);
  state.counters["gpu_max_diagonal"] = result.maxDiagonal;
  state.counters["gpu_max_gradient_abs"] = result.maxGradientAbs;
  state.counters["gpu_max_block_abs"] = result.maxBlockAbs;
  state.counters["host_setup_ns"] = result.timing.setupNs;
  state.counters["host_to_device_ns"] = result.timing.hostToDeviceNs;
  state.counters["incidence_kernel_ns"] = result.timing.incidenceKernelNs;
  state.counters["diagonal_kernel_ns"] = result.timing.diagonalKernelNs;
  state.counters["sparse_block_kernel_ns"] = result.timing.sparseBlockKernelNs;
  state.counters["device_to_host_ns"] = result.timing.deviceToHostNs;
}
BENCHMARK(BM_NewtonSceneRuntimeSparseGraphAssemblyCuda)
    ->Arg(4096)
    ->Arg(65536)
    ->UseRealTime();

//==============================================================================
static void BM_NewtonSceneRuntimeSparseGraphUniqueAssemblyCpu(
    benchmark::State& state)
{
  const auto fixture = makeSceneRuntimeSparseGraphUniqueAssemblyFixture(
      static_cast<int>(state.range(0)));
  CpuSceneSparseGraphUniqueAssemblyResult result;

  for (auto _ : state) {
    evaluateCpuSceneSparseGraphUniqueAssembly(
        fixture.nodes, fixture.triangles, result);
    benchmark::DoNotOptimize(result.rows.data());
    benchmark::DoNotOptimize(result.blocks.data());
  }

  recordSceneRuntimeSparseGraphUniqueAssemblyCounters(state, fixture, 0.0);
}
BENCHMARK(BM_NewtonSceneRuntimeSparseGraphUniqueAssemblyCpu)
    ->Arg(4096)
    ->Arg(65536)
    ->UseRealTime();

//==============================================================================
static void BM_NewtonSceneRuntimeSparseGraphUniqueAssemblyCuda(
    benchmark::State& state)
{
  if (!cuda::isCudaRuntimeAvailable()) {
    state.SkipWithError("CUDA runtime has no available device");
    return;
  }

  const auto fixture = makeSceneRuntimeSparseGraphUniqueAssemblyFixture(
      static_cast<int>(state.range(0)));
  cuda::NewtonSceneSparseGraphUniqueAssemblyResult result;

  for (auto _ : state) {
    cuda::evaluateNewtonSceneSparseGraphUniqueAssemblyCuda(
        fixture.nodes, fixture.triangles, result);
    benchmark::DoNotOptimize(result.rows.data());
    benchmark::DoNotOptimize(result.blocks.data());
  }

  recordSceneRuntimeSparseGraphUniqueAssemblyCounters(
      state,
      fixture,
      maxSceneSparseGraphUniqueAssemblyOutputError(fixture.cpu, result));
  state.counters["gpu_rows"] = static_cast<double>(result.rowCount);
  state.counters["gpu_bodies"] = static_cast<double>(result.bodyCount);
  state.counters["gpu_dofs"] = static_cast<double>(result.dofCount);
  state.counters["gpu_blocks"] = static_cast<double>(result.blockCount);
  state.counters["gpu_block_entries"]
      = static_cast<double>(result.blockEntryCount);
  state.counters["gpu_edge_slots"] = static_cast<double>(result.edgeSlotCount);
  state.counters["gpu_unique_edges"]
      = static_cast<double>(result.uniqueEdgeCount);
  state.counters["gpu_duplicate_edge_slots"]
      = static_cast<double>(result.duplicateEdgeSlotCount);
  state.counters["gpu_scene_bodies"]
      = static_cast<double>(fixture.sceneBodyCount);
  state.counters["gpu_scene_nodes"] = static_cast<double>(result.nodeCount);
  state.counters["gpu_scene_triangles"]
      = static_cast<double>(result.triangleCount);
  state.counters["gpu_max_diagonal"] = result.maxDiagonal;
  state.counters["gpu_max_gradient_abs"] = result.maxGradientAbs;
  state.counters["gpu_max_block_abs"] = result.maxBlockAbs;
  state.counters["host_setup_ns"] = result.timing.setupNs;
  state.counters["host_to_device_ns"] = result.timing.hostToDeviceNs;
  state.counters["incidence_kernel_ns"] = result.timing.incidenceKernelNs;
  state.counters["diagonal_kernel_ns"] = result.timing.diagonalKernelNs;
  state.counters["unique_edge_mark_kernel_ns"]
      = result.timing.uniqueEdgeMarkKernelNs;
  state.counters["sparse_block_kernel_ns"] = result.timing.sparseBlockKernelNs;
  state.counters["device_to_host_ns"] = result.timing.deviceToHostNs;
}
BENCHMARK(BM_NewtonSceneRuntimeSparseGraphUniqueAssemblyCuda)
    ->Arg(4096)
    ->Arg(65536)
    ->UseRealTime();

//==============================================================================
static void BM_NewtonSceneRuntimeNonlinearEqualityAssemblyCpu(
    benchmark::State& state)
{
  const auto fixture = makeSceneRuntimeNonlinearEqualityAssemblyFixture(
      static_cast<int>(state.range(0)));
  CpuSceneNonlinearEqualityAssemblyResult result;

  for (auto _ : state) {
    evaluateCpuSceneNonlinearEqualityAssembly(
        fixture.nodes, fixture.constraints, result);
    benchmark::DoNotOptimize(result.rows.data());
    benchmark::DoNotOptimize(result.blocks.data());
  }

  recordSceneRuntimeNonlinearEqualityAssemblyCounters(state, fixture, 0.0);
}
BENCHMARK(BM_NewtonSceneRuntimeNonlinearEqualityAssemblyCpu)
    ->Arg(4096)
    ->Arg(65536)
    ->UseRealTime();

//==============================================================================
static void BM_NewtonSceneRuntimeNonlinearEqualityAssemblyCuda(
    benchmark::State& state)
{
  if (!cuda::isCudaRuntimeAvailable()) {
    state.SkipWithError("CUDA runtime has no available device");
    return;
  }

  const auto fixture = makeSceneRuntimeNonlinearEqualityAssemblyFixture(
      static_cast<int>(state.range(0)));
  cuda::NewtonSceneNonlinearEqualityAssemblyResult result;

  for (auto _ : state) {
    cuda::evaluateNewtonSceneNonlinearEqualityAssemblyCuda(
        fixture.nodes, fixture.constraints, result);
    benchmark::DoNotOptimize(result.rows.data());
    benchmark::DoNotOptimize(result.blocks.data());
  }

  recordSceneRuntimeNonlinearEqualityAssemblyCounters(
      state,
      fixture,
      maxSceneNonlinearEqualityAssemblyOutputError(fixture.cpu, result));
  state.counters["gpu_rows"] = static_cast<double>(result.rowCount);
  state.counters["gpu_bodies"] = static_cast<double>(result.bodyCount);
  state.counters["gpu_dofs"] = static_cast<double>(result.dofCount);
  state.counters["gpu_constraints"]
      = static_cast<double>(result.constraintCount);
  state.counters["gpu_blocks"] = static_cast<double>(result.blockCount);
  state.counters["gpu_block_entries"]
      = static_cast<double>(result.blockEntryCount);
  state.counters["gpu_scene_bodies"]
      = static_cast<double>(fixture.sceneBodyCount);
  state.counters["gpu_scene_nodes"] = static_cast<double>(result.nodeCount);
  state.counters["gpu_scene_triangles"]
      = static_cast<double>(fixture.sceneTriangleCount);
  state.counters["gpu_scene_edge_pairs"]
      = static_cast<double>(fixture.sceneEdgePairCount);
  state.counters["gpu_max_constraint_residual_abs"]
      = result.maxConstraintResidualAbs;
  state.counters["gpu_max_diagonal"] = result.maxDiagonal;
  state.counters["gpu_max_gradient_abs"] = result.maxGradientAbs;
  state.counters["gpu_max_block_abs"] = result.maxBlockAbs;
  state.counters["host_setup_ns"] = result.timing.setupNs;
  state.counters["host_to_device_ns"] = result.timing.hostToDeviceNs;
  state.counters["constraint_kernel_ns"] = result.timing.constraintKernelNs;
  state.counters["device_to_host_ns"] = result.timing.deviceToHostNs;
}
BENCHMARK(BM_NewtonSceneRuntimeNonlinearEqualityAssemblyCuda)
    ->Arg(4096)
    ->Arg(65536)
    ->UseRealTime();

//==============================================================================
static void BM_NewtonSceneRuntimeNonlinearEqualitySolveCpu(
    benchmark::State& state)
{
  const auto fixture = makeSceneRuntimeNonlinearEqualitySolveFixture(
      static_cast<int>(state.range(0)));
  CpuSceneNonlinearEqualitySolveResult result;

  for (auto _ : state) {
    evaluateCpuSceneNonlinearEqualitySolve(
        fixture.nodes,
        fixture.constraints,
        kSceneNonlinearEqualitySolveRegularization,
        result);
    benchmark::DoNotOptimize(result.step.data());
    benchmark::DoNotOptimize(result.postSolveLinearizedResiduals.data());
  }

  recordSceneRuntimeNonlinearEqualitySolveCounters(state, fixture, 0.0);
}
BENCHMARK(BM_NewtonSceneRuntimeNonlinearEqualitySolveCpu)
    ->Arg(4096)
    ->Arg(65536)
    ->UseRealTime();

//==============================================================================
static void BM_NewtonSceneRuntimeNonlinearEqualitySolveCuda(
    benchmark::State& state)
{
  if (!cuda::isCudaRuntimeAvailable()) {
    state.SkipWithError("CUDA runtime has no available device");
    return;
  }

  const auto fixture = makeSceneRuntimeNonlinearEqualitySolveFixture(
      static_cast<int>(state.range(0)));
  cuda::NewtonSceneNonlinearEqualitySolveResult result;

  for (auto _ : state) {
    cuda::evaluateNewtonSceneNonlinearEqualitySolveCuda(
        fixture.nodes,
        fixture.constraints,
        kSceneNonlinearEqualitySolveRegularization,
        result);
    benchmark::DoNotOptimize(result.step.data());
    benchmark::DoNotOptimize(result.postSolveLinearizedResiduals.data());
  }

  recordSceneRuntimeNonlinearEqualitySolveCounters(
      state,
      fixture,
      maxSceneNonlinearEqualitySolveOutputError(fixture.cpu, result));
  state.counters["gpu_rows"] = static_cast<double>(result.rowCount);
  state.counters["gpu_bodies"] = static_cast<double>(result.bodyCount);
  state.counters["gpu_dofs"] = static_cast<double>(result.dofCount);
  state.counters["gpu_constraints"]
      = static_cast<double>(result.constraintCount);
  state.counters["gpu_blocks"] = static_cast<double>(result.blockCount);
  state.counters["gpu_block_entries"]
      = static_cast<double>(result.blockEntryCount);
  state.counters["gpu_active_dofs"]
      = static_cast<double>(result.activeDofCount);
  state.counters["gpu_scene_bodies"]
      = static_cast<double>(fixture.sceneBodyCount);
  state.counters["gpu_scene_nodes"] = static_cast<double>(result.nodeCount);
  state.counters["gpu_scene_triangles"]
      = static_cast<double>(fixture.sceneTriangleCount);
  state.counters["gpu_scene_edge_pairs"]
      = static_cast<double>(fixture.sceneEdgePairCount);
  state.counters["gpu_regularization"] = result.regularization;
  state.counters["gpu_max_constraint_residual_abs"]
      = result.maxConstraintResidualAbs;
  state.counters["gpu_max_post_solve_linearized_residual_abs"]
      = result.maxPostSolveLinearizedResidualAbs;
  state.counters["gpu_max_diagonal"] = result.maxDiagonal;
  state.counters["gpu_max_gradient_abs"] = result.maxGradientAbs;
  state.counters["gpu_max_block_abs"] = result.maxBlockAbs;
  state.counters["gpu_step_norm"] = result.stepNorm;
  state.counters["host_setup_ns"] = result.timing.setupNs;
  state.counters["host_to_device_ns"] = result.timing.hostToDeviceNs;
  state.counters["solve_kernel_ns"] = result.timing.solveKernelNs;
  state.counters["post_residual_kernel_ns"]
      = result.timing.postResidualKernelNs;
  state.counters["device_to_host_ns"] = result.timing.deviceToHostNs;
}
BENCHMARK(BM_NewtonSceneRuntimeNonlinearEqualitySolveCuda)
    ->Arg(4096)
    ->Arg(65536)
    ->UseRealTime();

//==============================================================================
static void BM_NewtonSceneRuntimeNonlinearEqualityConvergenceCpu(
    benchmark::State& state)
{
  const auto fixture = makeSceneRuntimeNonlinearEqualityConvergenceFixture(
      static_cast<int>(state.range(0)));
  CpuSceneNonlinearEqualityConvergenceResult result;

  for (auto _ : state) {
    evaluateCpuSceneNonlinearEqualityConvergence(
        fixture.nodes,
        fixture.constraints,
        kSceneNonlinearEqualitySolveRegularization,
        kSceneNonlinearEqualityConvergenceMaxIterations,
        kSceneNonlinearEqualityConvergenceResidualTolerance,
        result);
    benchmark::DoNotOptimize(result.step.data());
    benchmark::DoNotOptimize(result.finalResiduals.data());
  }

  recordSceneRuntimeNonlinearEqualityConvergenceCounters(state, fixture, 0.0);
}
BENCHMARK(BM_NewtonSceneRuntimeNonlinearEqualityConvergenceCpu)
    ->Arg(4096)
    ->Arg(65536)
    ->UseRealTime();

//==============================================================================
static void BM_NewtonSceneRuntimeNonlinearEqualityConvergenceCuda(
    benchmark::State& state)
{
  if (!cuda::isCudaRuntimeAvailable()) {
    state.SkipWithError("CUDA runtime has no available device");
    return;
  }

  const auto fixture = makeSceneRuntimeNonlinearEqualityConvergenceFixture(
      static_cast<int>(state.range(0)));
  cuda::NewtonSceneNonlinearEqualityConvergenceResult result;

  for (auto _ : state) {
    cuda::evaluateNewtonSceneNonlinearEqualityConvergenceCuda(
        fixture.nodes,
        fixture.constraints,
        kSceneNonlinearEqualitySolveRegularization,
        kSceneNonlinearEqualityConvergenceMaxIterations,
        kSceneNonlinearEqualityConvergenceResidualTolerance,
        result);
    benchmark::DoNotOptimize(result.step.data());
    benchmark::DoNotOptimize(result.finalResiduals.data());
  }

  recordSceneRuntimeNonlinearEqualityConvergenceCounters(
      state,
      fixture,
      maxSceneNonlinearEqualityConvergenceOutputError(fixture.cpu, result));
  state.counters["gpu_rows"] = static_cast<double>(result.rowCount);
  state.counters["gpu_bodies"] = static_cast<double>(result.bodyCount);
  state.counters["gpu_dofs"] = static_cast<double>(result.dofCount);
  state.counters["gpu_constraints"]
      = static_cast<double>(result.constraintCount);
  state.counters["gpu_blocks"] = static_cast<double>(result.blockCount);
  state.counters["gpu_block_entries"]
      = static_cast<double>(result.blockEntryCount);
  state.counters["gpu_active_dofs"]
      = static_cast<double>(result.activeDofCount);
  state.counters["gpu_max_iterations"]
      = static_cast<double>(result.maxIterationCount);
  state.counters["gpu_completed_iterations"]
      = static_cast<double>(result.completedIterationCount);
  state.counters["gpu_scene_bodies"]
      = static_cast<double>(fixture.sceneBodyCount);
  state.counters["gpu_scene_nodes"] = static_cast<double>(result.nodeCount);
  state.counters["gpu_scene_triangles"]
      = static_cast<double>(fixture.sceneTriangleCount);
  state.counters["gpu_scene_edge_pairs"]
      = static_cast<double>(fixture.sceneEdgePairCount);
  state.counters["gpu_regularization"] = result.regularization;
  state.counters["gpu_residual_tolerance"] = result.residualTolerance;
  state.counters["gpu_converged"] = result.converged ? 1.0 : 0.0;
  state.counters["gpu_initial_max_constraint_residual_abs"]
      = result.initialMaxConstraintResidualAbs;
  state.counters["gpu_final_max_constraint_residual_abs"]
      = result.finalMaxConstraintResidualAbs;
  state.counters["gpu_max_diagonal"] = result.maxDiagonal;
  state.counters["gpu_max_gradient_abs"] = result.maxGradientAbs;
  state.counters["gpu_max_block_abs"] = result.maxBlockAbs;
  state.counters["gpu_step_norm"] = result.stepNorm;
  state.counters["host_setup_ns"] = result.timing.setupNs;
  state.counters["host_to_device_ns"] = result.timing.hostToDeviceNs;
  state.counters["iteration_kernel_ns"] = result.timing.iterationKernelNs;
  state.counters["residual_kernel_ns"] = result.timing.residualKernelNs;
  state.counters["convergence_readback_ns"]
      = result.timing.convergenceReadbackNs;
  state.counters["final_assembly_kernel_ns"]
      = result.timing.finalAssemblyKernelNs;
  state.counters["device_to_host_ns"] = result.timing.deviceToHostNs;
}
BENCHMARK(BM_NewtonSceneRuntimeNonlinearEqualityConvergenceCuda)
    ->Arg(4096)
    ->Arg(65536)
    ->UseRealTime();

//==============================================================================
static void BM_NewtonSparseResidualCpu(benchmark::State& state)
{
  const auto fixture
      = makeSparseResidualFixture(static_cast<int>(state.range(0)));
  CpuSparseResidualResult result;

  for (auto _ : state) {
    evaluateCpuSparseResidual(
        fixture.rows, fixture.bodyCount, fixture.blocks, fixture.step, result);
    benchmark::DoNotOptimize(result.residual.data());
  }

  recordSparseResidualCounters(state, fixture, 0.0);
}
BENCHMARK(BM_NewtonSparseResidualCpu)->Arg(4096)->Arg(65536)->UseRealTime();

//==============================================================================
static void BM_NewtonSparseResidualCuda(benchmark::State& state)
{
  if (!cuda::isCudaRuntimeAvailable()) {
    state.SkipWithError("CUDA runtime has no available device");
    return;
  }

  const auto fixture
      = makeSparseResidualFixture(static_cast<int>(state.range(0)));
  cuda::NewtonSparseResidualResult result;

  for (auto _ : state) {
    cuda::evaluateNewtonSparseResidualCuda(
        fixture.rows,
        fixture.bodyCount,
        fixture.blocks,
        fixture.step,
        kRegularization,
        result);
    benchmark::DoNotOptimize(result.residual.data());
  }

  recordSparseResidualCounters(
      state, fixture, maxSparseResidualOutputError(fixture.cpu, result));
  state.counters["gpu_rows"] = static_cast<double>(result.rowCount);
  state.counters["gpu_bodies"] = static_cast<double>(result.bodyCount);
  state.counters["gpu_dofs"] = static_cast<double>(result.dofCount);
  state.counters["gpu_blocks"] = static_cast<double>(result.blockCount);
  state.counters["gpu_active_dofs"]
      = static_cast<double>(result.activeDofCount);
  state.counters["gpu_output_norm"] = result.residualNorm;
  state.counters["gpu_max_output_abs"] = result.maxResidualAbs;
  state.counters["host_setup_ns"] = result.timing.setupNs;
  state.counters["host_to_device_ns"] = result.timing.hostToDeviceNs;
  state.counters["assembly_kernel_ns"] = result.timing.assemblyKernelNs;
  state.counters["gradient_seed_ns"] = result.timing.gradientSeedNs;
  state.counters["diagonal_kernel_ns"] = result.timing.diagonalKernelNs;
  state.counters["off_diagonal_kernel_ns"] = result.timing.offDiagonalKernelNs;
  state.counters["device_to_host_ns"] = result.timing.deviceToHostNs;
}
BENCHMARK(BM_NewtonSparseResidualCuda)->Arg(4096)->Arg(65536)->UseRealTime();

//==============================================================================
static void BM_NewtonSceneRuntimeSparseResidualCpu(benchmark::State& state)
{
  const auto fixture
      = makeSceneRuntimeSparseResidualFixture(static_cast<int>(state.range(0)));
  CpuSparseResidualResult result;

  for (auto _ : state) {
    evaluateCpuSparseResidual(
        fixture.rows, fixture.bodyCount, fixture.blocks, fixture.step, result);
    benchmark::DoNotOptimize(result.residual.data());
  }

  recordSceneRuntimeSparseResidualCounters(state, fixture, 0.0);
}
BENCHMARK(BM_NewtonSceneRuntimeSparseResidualCpu)
    ->Arg(4096)
    ->Arg(65536)
    ->UseRealTime();

//==============================================================================
static void BM_NewtonSceneRuntimeSparseResidualCuda(benchmark::State& state)
{
  if (!cuda::isCudaRuntimeAvailable()) {
    state.SkipWithError("CUDA runtime has no available device");
    return;
  }

  const auto fixture
      = makeSceneRuntimeSparseResidualFixture(static_cast<int>(state.range(0)));
  cuda::NewtonSparseResidualResult result;

  for (auto _ : state) {
    cuda::evaluateNewtonSparseResidualCuda(
        fixture.rows,
        fixture.bodyCount,
        fixture.blocks,
        fixture.step,
        kRegularization,
        result);
    benchmark::DoNotOptimize(result.residual.data());
  }

  recordSceneRuntimeSparseResidualCounters(
      state, fixture, maxSparseResidualOutputError(fixture.cpu, result));
  state.counters["gpu_rows"] = static_cast<double>(result.rowCount);
  state.counters["gpu_bodies"] = static_cast<double>(result.bodyCount);
  state.counters["gpu_dofs"] = static_cast<double>(result.dofCount);
  state.counters["gpu_blocks"] = static_cast<double>(result.blockCount);
  state.counters["gpu_active_dofs"]
      = static_cast<double>(result.activeDofCount);
  state.counters["gpu_output_norm"] = result.residualNorm;
  state.counters["gpu_max_output_abs"] = result.maxResidualAbs;
  state.counters["gpu_scene_bodies"]
      = static_cast<double>(fixture.sceneBodyCount);
  state.counters["gpu_scene_nodes"]
      = static_cast<double>(fixture.sceneNodeCount);
  state.counters["gpu_scene_triangles"]
      = static_cast<double>(fixture.sceneTriangleCount);
  state.counters["gpu_scene_edge_pairs"]
      = static_cast<double>(fixture.sceneEdgePairCount);
  state.counters["host_setup_ns"] = result.timing.setupNs;
  state.counters["host_to_device_ns"] = result.timing.hostToDeviceNs;
  state.counters["assembly_kernel_ns"] = result.timing.assemblyKernelNs;
  state.counters["gradient_seed_ns"] = result.timing.gradientSeedNs;
  state.counters["diagonal_kernel_ns"] = result.timing.diagonalKernelNs;
  state.counters["off_diagonal_kernel_ns"] = result.timing.offDiagonalKernelNs;
  state.counters["device_to_host_ns"] = result.timing.deviceToHostNs;
}
BENCHMARK(BM_NewtonSceneRuntimeSparseResidualCuda)
    ->Arg(4096)
    ->Arg(65536)
    ->UseRealTime();

//==============================================================================
static void BM_NewtonSparseJacobiSolveCpu(benchmark::State& state)
{
  const auto fixture
      = makeSparseJacobiSolveFixture(static_cast<int>(state.range(0)));
  CpuSparseJacobiSolveResult result;

  for (auto _ : state) {
    evaluateCpuSparseJacobiSolve(
        fixture.rows, fixture.bodyCount, fixture.blocks, result);
    benchmark::DoNotOptimize(result.step.data());
  }

  recordSparseJacobiSolveCounters(state, fixture, 0.0);
}
BENCHMARK(BM_NewtonSparseJacobiSolveCpu)->Arg(4096)->Arg(65536)->UseRealTime();

//==============================================================================
static void BM_NewtonSparseJacobiSolveCuda(benchmark::State& state)
{
  if (!cuda::isCudaRuntimeAvailable()) {
    state.SkipWithError("CUDA runtime has no available device");
    return;
  }

  const auto fixture
      = makeSparseJacobiSolveFixture(static_cast<int>(state.range(0)));
  cuda::NewtonSparseJacobiSolveResult result;

  for (auto _ : state) {
    cuda::evaluateNewtonSparseJacobiSolveCuda(
        fixture.rows,
        fixture.bodyCount,
        fixture.blocks,
        kSparseJacobiIterations,
        kRegularization,
        result);
    benchmark::DoNotOptimize(result.step.data());
  }

  recordSparseJacobiSolveCounters(
      state, fixture, maxSparseJacobiSolveOutputError(fixture.cpu, result));
  state.counters["gpu_rows"] = static_cast<double>(result.rowCount);
  state.counters["gpu_bodies"] = static_cast<double>(result.bodyCount);
  state.counters["gpu_dofs"] = static_cast<double>(result.dofCount);
  state.counters["gpu_blocks"] = static_cast<double>(result.blockCount);
  state.counters["gpu_iterations"] = static_cast<double>(result.iterationCount);
  state.counters["gpu_active_dofs"]
      = static_cast<double>(result.activeDofCount);
  state.counters["gpu_step_norm"] = result.stepNorm;
  state.counters["gpu_residual_norm"] = result.residualNorm;
  state.counters["gpu_max_residual_abs"] = result.maxResidualAbs;
  state.counters["host_setup_ns"] = result.timing.setupNs;
  state.counters["host_to_device_ns"] = result.timing.hostToDeviceNs;
  state.counters["assembly_kernel_ns"] = result.timing.assemblyKernelNs;
  state.counters["iteration_kernel_ns"] = result.timing.iterationKernelNs;
  state.counters["final_residual_kernel_ns"]
      = result.timing.finalResidualKernelNs;
  state.counters["device_to_host_ns"] = result.timing.deviceToHostNs;
}
BENCHMARK(BM_NewtonSparseJacobiSolveCuda)->Arg(4096)->Arg(65536)->UseRealTime();

//==============================================================================
static void BM_NewtonSceneRuntimeSparseJacobiSolveCpu(benchmark::State& state)
{
  const auto fixture = makeSceneRuntimeSparseJacobiSolveFixture(
      static_cast<int>(state.range(0)));
  CpuSparseJacobiSolveResult result;

  for (auto _ : state) {
    evaluateCpuSparseJacobiSolve(
        fixture.rows, fixture.bodyCount, fixture.blocks, result);
    benchmark::DoNotOptimize(result.step.data());
  }

  recordSceneRuntimeSparseJacobiSolveCounters(state, fixture, 0.0);
}
BENCHMARK(BM_NewtonSceneRuntimeSparseJacobiSolveCpu)
    ->Arg(4096)
    ->Arg(65536)
    ->UseRealTime();

//==============================================================================
static void BM_NewtonSceneRuntimeSparseJacobiSolveCuda(benchmark::State& state)
{
  if (!cuda::isCudaRuntimeAvailable()) {
    state.SkipWithError("CUDA runtime has no available device");
    return;
  }

  const auto fixture = makeSceneRuntimeSparseJacobiSolveFixture(
      static_cast<int>(state.range(0)));
  cuda::NewtonSparseJacobiSolveResult result;

  for (auto _ : state) {
    cuda::evaluateNewtonSparseJacobiSolveCuda(
        fixture.rows,
        fixture.bodyCount,
        fixture.blocks,
        kSparseJacobiIterations,
        kRegularization,
        result);
    benchmark::DoNotOptimize(result.step.data());
  }

  recordSceneRuntimeSparseJacobiSolveCounters(
      state, fixture, maxSparseJacobiSolveOutputError(fixture.cpu, result));
  state.counters["gpu_rows"] = static_cast<double>(result.rowCount);
  state.counters["gpu_bodies"] = static_cast<double>(result.bodyCount);
  state.counters["gpu_dofs"] = static_cast<double>(result.dofCount);
  state.counters["gpu_blocks"] = static_cast<double>(result.blockCount);
  state.counters["gpu_iterations"] = static_cast<double>(result.iterationCount);
  state.counters["gpu_active_dofs"]
      = static_cast<double>(result.activeDofCount);
  state.counters["gpu_step_norm"] = result.stepNorm;
  state.counters["gpu_residual_norm"] = result.residualNorm;
  state.counters["gpu_max_residual_abs"] = result.maxResidualAbs;
  state.counters["gpu_scene_bodies"]
      = static_cast<double>(fixture.sceneBodyCount);
  state.counters["gpu_scene_nodes"]
      = static_cast<double>(fixture.sceneNodeCount);
  state.counters["gpu_scene_triangles"]
      = static_cast<double>(fixture.sceneTriangleCount);
  state.counters["gpu_scene_edge_pairs"]
      = static_cast<double>(fixture.sceneEdgePairCount);
  state.counters["host_setup_ns"] = result.timing.setupNs;
  state.counters["host_to_device_ns"] = result.timing.hostToDeviceNs;
  state.counters["assembly_kernel_ns"] = result.timing.assemblyKernelNs;
  state.counters["iteration_kernel_ns"] = result.timing.iterationKernelNs;
  state.counters["final_residual_kernel_ns"]
      = result.timing.finalResidualKernelNs;
  state.counters["device_to_host_ns"] = result.timing.deviceToHostNs;
}
BENCHMARK(BM_NewtonSceneRuntimeSparseJacobiSolveCuda)
    ->Arg(4096)
    ->Arg(65536)
    ->UseRealTime();

//==============================================================================
static void BM_NewtonSparseCgSolveCpu(benchmark::State& state)
{
  const auto fixture
      = makeSparseCgSolveFixture(static_cast<int>(state.range(0)));
  CpuSparseCgSolveResult result;

  for (auto _ : state) {
    evaluateCpuSparseCgSolve(
        fixture.rows, fixture.bodyCount, fixture.blocks, result);
    benchmark::DoNotOptimize(result.step.data());
  }

  recordSparseCgSolveCounters(state, fixture, 0.0);
}
BENCHMARK(BM_NewtonSparseCgSolveCpu)->Arg(4096)->Arg(65536)->UseRealTime();

//==============================================================================
static void BM_NewtonSparseCgSolveCuda(benchmark::State& state)
{
  if (!cuda::isCudaRuntimeAvailable()) {
    state.SkipWithError("CUDA runtime has no available device");
    return;
  }

  const auto fixture
      = makeSparseCgSolveFixture(static_cast<int>(state.range(0)));
  cuda::NewtonSparseCgSolveResult result;

  for (auto _ : state) {
    cuda::evaluateNewtonSparseCgSolveCuda(
        fixture.rows,
        fixture.bodyCount,
        fixture.blocks,
        kSparseCgMaxIterations,
        kSparseCgResidualTolerance,
        kRegularization,
        result);
    benchmark::DoNotOptimize(result.step.data());
  }

  recordSparseCgSolveCounters(
      state, fixture, maxSparseCgSolveOutputError(fixture.cpu, result));
  state.counters["gpu_rows"] = static_cast<double>(result.rowCount);
  state.counters["gpu_bodies"] = static_cast<double>(result.bodyCount);
  state.counters["gpu_dofs"] = static_cast<double>(result.dofCount);
  state.counters["gpu_blocks"] = static_cast<double>(result.blockCount);
  state.counters["gpu_max_iterations"]
      = static_cast<double>(result.maxIterationCount);
  state.counters["gpu_completed_iterations"]
      = static_cast<double>(result.completedIterationCount);
  state.counters["gpu_active_dofs"]
      = static_cast<double>(result.activeDofCount);
  state.counters["gpu_converged"] = result.converged ? 1.0 : 0.0;
  state.counters["gpu_residual_tolerance"] = result.residualTolerance;
  state.counters["gpu_initial_residual_norm"] = result.initialResidualNorm;
  state.counters["gpu_step_norm"] = result.stepNorm;
  state.counters["gpu_residual_norm"] = result.residualNorm;
  state.counters["gpu_max_residual_abs"] = result.maxResidualAbs;
  state.counters["host_setup_ns"] = result.timing.setupNs;
  state.counters["host_to_device_ns"] = result.timing.hostToDeviceNs;
  state.counters["assembly_kernel_ns"] = result.timing.assemblyKernelNs;
  state.counters["iteration_kernel_ns"] = result.timing.iterationKernelNs;
  state.counters["final_residual_kernel_ns"]
      = result.timing.finalResidualKernelNs;
  state.counters["device_to_host_ns"] = result.timing.deviceToHostNs;
}
BENCHMARK(BM_NewtonSparseCgSolveCuda)->Arg(4096)->Arg(65536)->UseRealTime();

//==============================================================================
static void BM_NewtonSceneRuntimeSparseCgSolveCpu(benchmark::State& state)
{
  const auto fixture
      = makeSceneRuntimeSparseCgSolveFixture(static_cast<int>(state.range(0)));
  CpuSparseCgSolveResult result;

  for (auto _ : state) {
    evaluateCpuSparseCgSolve(
        fixture.rows, fixture.bodyCount, fixture.blocks, result);
    benchmark::DoNotOptimize(result.step.data());
  }

  recordSceneRuntimeSparseCgSolveCounters(state, fixture, 0.0);
}
BENCHMARK(BM_NewtonSceneRuntimeSparseCgSolveCpu)
    ->Arg(4096)
    ->Arg(65536)
    ->UseRealTime();

//==============================================================================
static void BM_NewtonSceneRuntimeSparseCgSolveCuda(benchmark::State& state)
{
  if (!cuda::isCudaRuntimeAvailable()) {
    state.SkipWithError("CUDA runtime has no available device");
    return;
  }

  const auto fixture
      = makeSceneRuntimeSparseCgSolveFixture(static_cast<int>(state.range(0)));
  cuda::NewtonSparseCgSolveResult result;

  for (auto _ : state) {
    cuda::evaluateNewtonSparseCgSolveCuda(
        fixture.rows,
        fixture.bodyCount,
        fixture.blocks,
        kSparseCgMaxIterations,
        kSparseCgResidualTolerance,
        kRegularization,
        result);
    benchmark::DoNotOptimize(result.step.data());
  }

  recordSceneRuntimeSparseCgSolveCounters(
      state, fixture, maxSparseCgSolveOutputError(fixture.cpu, result));
  state.counters["gpu_rows"] = static_cast<double>(result.rowCount);
  state.counters["gpu_bodies"] = static_cast<double>(result.bodyCount);
  state.counters["gpu_dofs"] = static_cast<double>(result.dofCount);
  state.counters["gpu_blocks"] = static_cast<double>(result.blockCount);
  state.counters["gpu_max_iterations"]
      = static_cast<double>(result.maxIterationCount);
  state.counters["gpu_completed_iterations"]
      = static_cast<double>(result.completedIterationCount);
  state.counters["gpu_active_dofs"]
      = static_cast<double>(result.activeDofCount);
  state.counters["gpu_converged"] = result.converged ? 1.0 : 0.0;
  state.counters["gpu_residual_tolerance"] = result.residualTolerance;
  state.counters["gpu_initial_residual_norm"] = result.initialResidualNorm;
  state.counters["gpu_step_norm"] = result.stepNorm;
  state.counters["gpu_residual_norm"] = result.residualNorm;
  state.counters["gpu_max_residual_abs"] = result.maxResidualAbs;
  state.counters["gpu_scene_bodies"]
      = static_cast<double>(fixture.sceneBodyCount);
  state.counters["gpu_scene_nodes"]
      = static_cast<double>(fixture.sceneNodeCount);
  state.counters["gpu_scene_triangles"]
      = static_cast<double>(fixture.sceneTriangleCount);
  state.counters["gpu_scene_edge_pairs"]
      = static_cast<double>(fixture.sceneEdgePairCount);
  state.counters["host_setup_ns"] = result.timing.setupNs;
  state.counters["host_to_device_ns"] = result.timing.hostToDeviceNs;
  state.counters["assembly_kernel_ns"] = result.timing.assemblyKernelNs;
  state.counters["iteration_kernel_ns"] = result.timing.iterationKernelNs;
  state.counters["final_residual_kernel_ns"]
      = result.timing.finalResidualKernelNs;
  state.counters["device_to_host_ns"] = result.timing.deviceToHostNs;
}
BENCHMARK(BM_NewtonSceneRuntimeSparseCgSolveCuda)
    ->Arg(4096)
    ->Arg(65536)
    ->UseRealTime();

//==============================================================================
static void BM_NewtonDirectSparseSolveCpu(benchmark::State& state)
{
  const auto fixture
      = makeDirectSparseSolveFixture(static_cast<int>(state.range(0)));
  CpuDirectSparseSolveResult result;

  for (auto _ : state) {
    evaluateCpuDirectSparseSolve(
        fixture.rows, fixture.bodyCount, fixture.blocks, result);
    benchmark::DoNotOptimize(result.step.data());
  }

  recordDirectSparseSolveCounters(state, fixture, 0.0);
}
BENCHMARK(BM_NewtonDirectSparseSolveCpu)->Arg(4096)->Arg(65536)->UseRealTime();

//==============================================================================
static void BM_NewtonDirectSparseSolveCuda(benchmark::State& state)
{
  if (!cuda::isCudaRuntimeAvailable()) {
    state.SkipWithError("CUDA runtime has no available device");
    return;
  }

  const auto fixture
      = makeDirectSparseSolveFixture(static_cast<int>(state.range(0)));
  cuda::NewtonDirectSparseSolveResult result;

  for (auto _ : state) {
    cuda::evaluateNewtonDirectSparseSolveCuda(
        fixture.rows,
        fixture.bodyCount,
        fixture.blocks,
        kRegularization,
        result);
    benchmark::DoNotOptimize(result.step.data());
  }

  recordDirectSparseSolveCounters(
      state, fixture, maxDirectSparseSolveOutputError(fixture.cpu, result));
  state.counters["gpu_rows"] = static_cast<double>(result.rowCount);
  state.counters["gpu_bodies"] = static_cast<double>(result.bodyCount);
  state.counters["gpu_dofs"] = static_cast<double>(result.dofCount);
  state.counters["gpu_blocks"] = static_cast<double>(result.blockCount);
  state.counters["gpu_active_dofs"]
      = static_cast<double>(result.activeDofCount);
  state.counters["gpu_regularization"] = result.regularization;
  state.counters["gpu_min_factor_pivot"] = result.minimumFactorPivot;
  state.counters["gpu_step_norm"] = result.stepNorm;
  state.counters["gpu_residual_norm"] = result.residualNorm;
  state.counters["gpu_max_residual_abs"] = result.maxResidualAbs;
  state.counters["host_setup_ns"] = result.timing.setupNs;
  state.counters["host_to_device_ns"] = result.timing.hostToDeviceNs;
  state.counters["assembly_kernel_ns"] = result.timing.assemblyKernelNs;
  state.counters["dense_matrix_kernel_ns"] = result.timing.denseMatrixKernelNs;
  state.counters["factor_solve_kernel_ns"] = result.timing.factorSolveKernelNs;
  state.counters["final_residual_kernel_ns"]
      = result.timing.finalResidualKernelNs;
  state.counters["device_to_host_ns"] = result.timing.deviceToHostNs;
}
BENCHMARK(BM_NewtonDirectSparseSolveCuda)->Arg(4096)->Arg(65536)->UseRealTime();

//==============================================================================
static void BM_NewtonSceneRuntimeDirectSparseSolveCpu(benchmark::State& state)
{
  const auto fixture = makeSceneRuntimeDirectSparseSolveFixture(
      static_cast<int>(state.range(0)));
  CpuDirectSparseSolveResult result;

  for (auto _ : state) {
    evaluateCpuDirectSparseSolve(
        fixture.rows, fixture.bodyCount, fixture.blocks, result);
    benchmark::DoNotOptimize(result.step.data());
  }

  recordSceneRuntimeDirectSparseSolveCounters(state, fixture, 0.0);
}
BENCHMARK(BM_NewtonSceneRuntimeDirectSparseSolveCpu)
    ->Arg(4096)
    ->Arg(65536)
    ->UseRealTime();

//==============================================================================
static void BM_NewtonSceneRuntimeDirectSparseSolveCuda(benchmark::State& state)
{
  if (!cuda::isCudaRuntimeAvailable()) {
    state.SkipWithError("CUDA runtime has no available device");
    return;
  }

  const auto fixture = makeSceneRuntimeDirectSparseSolveFixture(
      static_cast<int>(state.range(0)));
  cuda::NewtonDirectSparseSolveResult result;

  for (auto _ : state) {
    cuda::evaluateNewtonDirectSparseSolveCuda(
        fixture.rows,
        fixture.bodyCount,
        fixture.blocks,
        kRegularization,
        result);
    benchmark::DoNotOptimize(result.step.data());
  }

  recordSceneRuntimeDirectSparseSolveCounters(
      state, fixture, maxDirectSparseSolveOutputError(fixture.cpu, result));
  state.counters["gpu_rows"] = static_cast<double>(result.rowCount);
  state.counters["gpu_bodies"] = static_cast<double>(result.bodyCount);
  state.counters["gpu_dofs"] = static_cast<double>(result.dofCount);
  state.counters["gpu_blocks"] = static_cast<double>(result.blockCount);
  state.counters["gpu_active_dofs"]
      = static_cast<double>(result.activeDofCount);
  state.counters["gpu_scene_bodies"]
      = static_cast<double>(fixture.sceneBodyCount);
  state.counters["gpu_scene_nodes"]
      = static_cast<double>(fixture.sceneNodeCount);
  state.counters["gpu_scene_triangles"]
      = static_cast<double>(fixture.sceneTriangleCount);
  state.counters["gpu_selected_scene_nodes"]
      = static_cast<double>(fixture.selectedSceneNodeCount);
  state.counters["gpu_selected_scene_edge_pairs"]
      = static_cast<double>(fixture.selectedSceneEdgePairCount);
  state.counters["gpu_regularization"] = result.regularization;
  state.counters["gpu_min_factor_pivot"] = result.minimumFactorPivot;
  state.counters["gpu_step_norm"] = result.stepNorm;
  state.counters["gpu_residual_norm"] = result.residualNorm;
  state.counters["gpu_max_residual_abs"] = result.maxResidualAbs;
  state.counters["host_setup_ns"] = result.timing.setupNs;
  state.counters["host_to_device_ns"] = result.timing.hostToDeviceNs;
  state.counters["assembly_kernel_ns"] = result.timing.assemblyKernelNs;
  state.counters["dense_matrix_kernel_ns"] = result.timing.denseMatrixKernelNs;
  state.counters["factor_solve_kernel_ns"] = result.timing.factorSolveKernelNs;
  state.counters["final_residual_kernel_ns"]
      = result.timing.finalResidualKernelNs;
  state.counters["device_to_host_ns"] = result.timing.deviceToHostNs;
}
BENCHMARK(BM_NewtonSceneRuntimeDirectSparseSolveCuda)
    ->Arg(4096)
    ->Arg(65536)
    ->UseRealTime();

//==============================================================================
static void BM_NewtonEqualityReducedSolveCpu(benchmark::State& state)
{
  const auto fixture
      = makeEqualityReducedFixture(static_cast<int>(state.range(0)));
  CpuEqualityReducedSolveResult result;

  for (auto _ : state) {
    evaluateCpuEqualityReducedSolve(
        fixture.rows,
        fixture.bodyCount,
        fixture.entries,
        fixture.reducedDofCount,
        result);
    benchmark::DoNotOptimize(result.reducedStep.data());
  }

  recordEqualityReducedCounters(state, fixture, 0.0);
}
BENCHMARK(BM_NewtonEqualityReducedSolveCpu)
    ->Arg(4096)
    ->Arg(65536)
    ->UseRealTime();

//==============================================================================
static void BM_NewtonEqualityReducedSolveCuda(benchmark::State& state)
{
  if (!cuda::isCudaRuntimeAvailable()) {
    state.SkipWithError("CUDA runtime has no available device");
    return;
  }

  const auto fixture
      = makeEqualityReducedFixture(static_cast<int>(state.range(0)));
  cuda::NewtonEqualityReducedSolveResult result;

  for (auto _ : state) {
    cuda::evaluateNewtonEqualityReducedSolveCuda(
        fixture.rows,
        fixture.bodyCount,
        fixture.entries,
        fixture.reducedDofCount,
        kRegularization,
        result);
    benchmark::DoNotOptimize(result.reducedStep.data());
  }

  recordEqualityReducedCounters(
      state, fixture, maxEqualityReducedOutputError(fixture.cpu, result));
  state.counters["gpu_rows"] = static_cast<double>(result.rowCount);
  state.counters["gpu_bodies"] = static_cast<double>(result.bodyCount);
  state.counters["gpu_dofs"] = static_cast<double>(result.fullDofCount);
  state.counters["gpu_reduction_entries"]
      = static_cast<double>(result.reductionEntryCount);
  state.counters["gpu_reduced_dofs"]
      = static_cast<double>(result.reducedDofCount);
  state.counters["gpu_active_reduced_dofs"]
      = static_cast<double>(result.activeReducedDofCount);
  state.counters["gpu_step_norm"] = result.stepNorm;
  state.counters["gpu_residual_norm"] = result.residualNorm;
  state.counters["host_setup_ns"] = result.timing.setupNs;
  state.counters["host_to_device_ns"] = result.timing.hostToDeviceNs;
  state.counters["assembly_kernel_ns"] = result.timing.assemblyKernelNs;
  state.counters["reduction_kernel_ns"] = result.timing.reductionKernelNs;
  state.counters["solve_kernel_ns"] = result.timing.solveKernelNs;
  state.counters["expansion_kernel_ns"] = result.timing.expansionKernelNs;
  state.counters["device_to_host_ns"] = result.timing.deviceToHostNs;
}
BENCHMARK(BM_NewtonEqualityReducedSolveCuda)
    ->Arg(4096)
    ->Arg(65536)
    ->UseRealTime();

//==============================================================================
static void BM_NewtonSceneRuntimeEqualityReducedSolveCpu(
    benchmark::State& state)
{
  const auto fixture = makeSceneRuntimeEqualityReducedFixture(
      static_cast<int>(state.range(0)));
  CpuEqualityReducedSolveResult result;

  for (auto _ : state) {
    evaluateCpuEqualityReducedSolve(
        fixture.rows,
        fixture.bodyCount,
        fixture.entries,
        fixture.reducedDofCount,
        result);
    benchmark::DoNotOptimize(result.reducedStep.data());
  }

  recordSceneRuntimeEqualityReducedCounters(state, fixture, 0.0);
}
BENCHMARK(BM_NewtonSceneRuntimeEqualityReducedSolveCpu)
    ->Arg(65536)
    ->UseRealTime();

//==============================================================================
static void BM_NewtonSceneRuntimeEqualityReducedSolveCuda(
    benchmark::State& state)
{
  if (!cuda::isCudaRuntimeAvailable()) {
    state.SkipWithError("CUDA runtime has no available device");
    return;
  }

  const auto fixture = makeSceneRuntimeEqualityReducedFixture(
      static_cast<int>(state.range(0)));
  cuda::NewtonEqualityReducedSolveResult result;

  for (auto _ : state) {
    cuda::evaluateNewtonEqualityReducedSolveCuda(
        fixture.rows,
        fixture.bodyCount,
        fixture.entries,
        fixture.reducedDofCount,
        kRegularization,
        result);
    benchmark::DoNotOptimize(result.reducedStep.data());
  }

  recordSceneRuntimeEqualityReducedCounters(
      state, fixture, maxEqualityReducedOutputError(fixture.cpu, result));
  state.counters["gpu_rows"] = static_cast<double>(result.rowCount);
  state.counters["gpu_bodies"] = static_cast<double>(result.bodyCount);
  state.counters["gpu_dofs"] = static_cast<double>(result.fullDofCount);
  state.counters["gpu_reduction_entries"]
      = static_cast<double>(result.reductionEntryCount);
  state.counters["gpu_reduced_dofs"]
      = static_cast<double>(result.reducedDofCount);
  state.counters["gpu_active_reduced_dofs"]
      = static_cast<double>(result.activeReducedDofCount);
  state.counters["gpu_scene_bodies"]
      = static_cast<double>(fixture.sceneBodyCount);
  state.counters["gpu_scene_nodes"]
      = static_cast<double>(fixture.sceneNodeCount);
  state.counters["gpu_scene_triangles"]
      = static_cast<double>(fixture.sceneTriangleCount);
  state.counters["gpu_step_norm"] = result.stepNorm;
  state.counters["gpu_residual_norm"] = result.residualNorm;
  state.counters["host_setup_ns"] = result.timing.setupNs;
  state.counters["host_to_device_ns"] = result.timing.hostToDeviceNs;
  state.counters["assembly_kernel_ns"] = result.timing.assemblyKernelNs;
  state.counters["reduction_kernel_ns"] = result.timing.reductionKernelNs;
  state.counters["solve_kernel_ns"] = result.timing.solveKernelNs;
  state.counters["expansion_kernel_ns"] = result.timing.expansionKernelNs;
  state.counters["device_to_host_ns"] = result.timing.deviceToHostNs;
}
BENCHMARK(BM_NewtonSceneRuntimeEqualityReducedSolveCuda)
    ->Arg(65536)
    ->UseRealTime();

BENCHMARK_MAIN();
