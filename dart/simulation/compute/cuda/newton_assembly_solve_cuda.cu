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

#include <cuda_runtime.h>
#include <dart/simulation/compute/cuda/cuda_runtime.cuh>
#include <dart/simulation/compute/cuda/newton_assembly_solve_cuda.cuh>

#include <cmath>
#include <cstddef>

namespace dart::simulation::compute::cuda::detail {
namespace {

__global__ void newtonAssemblyRowsKernel(
    const NewtonAssemblySolveRowInput* rows,
    double* assembledDiagonal,
    double* assembledGradient,
    const std::size_t rowCount)
{
  const auto index
      = static_cast<std::size_t>(blockIdx.x * blockDim.x + threadIdx.x);
  const std::size_t totalDofs = rowCount * kNewtonAssemblySolveDofsPerBody;
  if (index >= totalDofs) {
    return;
  }

  const std::size_t row = index / kNewtonAssemblySolveDofsPerBody;
  const std::size_t dof = index - row * kNewtonAssemblySolveDofsPerBody;
  const NewtonAssemblySolveRowInput input = rows[row];
  const std::size_t outputIndex = static_cast<std::size_t>(input.bodyIndex)
                                      * kNewtonAssemblySolveDofsPerBody
                                  + dof;
  atomicAdd(&assembledDiagonal[outputIndex], input.hessianDiagonal[dof]);
  atomicAdd(&assembledGradient[outputIndex], input.gradient[dof]);
}

__global__ void newtonOffDiagonalAssemblyRowsKernel(
    const NewtonOffDiagonalAssemblyRowInput* rows,
    double* assembledBlocks,
    const std::size_t rowCount)
{
  const auto index
      = static_cast<std::size_t>(blockIdx.x * blockDim.x + threadIdx.x);
  const std::size_t totalEntries = rowCount * kNewtonAssemblySolveBlockEntries;
  if (index >= totalEntries) {
    return;
  }

  const std::size_t row = index / kNewtonAssemblySolveBlockEntries;
  const std::size_t entry = index - row * kNewtonAssemblySolveBlockEntries;
  const NewtonOffDiagonalAssemblyRowInput input = rows[row];
  const std::size_t outputIndex = static_cast<std::size_t>(input.pairIndex)
                                      * kNewtonAssemblySolveBlockEntries
                                  + entry;
  atomicAdd(&assembledBlocks[outputIndex], input.hessianBlock[entry]);
}

__global__ void newtonSceneTriangleIncidenceKernel(
    const NewtonSceneSurfaceTriangleInput* triangles,
    std::uint32_t* incidentTriangleCounts,
    const std::size_t triangleCount)
{
  const auto triangle
      = static_cast<std::size_t>(blockIdx.x * blockDim.x + threadIdx.x);
  if (triangle >= triangleCount) {
    return;
  }

  const NewtonSceneSurfaceTriangleInput input = triangles[triangle];
  atomicAdd(&incidentTriangleCounts[input.nodeA], 1u);
  atomicAdd(&incidentTriangleCounts[input.nodeB], 1u);
  atomicAdd(&incidentTriangleCounts[input.nodeC], 1u);
}

__global__ void newtonSceneDiagonalRowsKernel(
    const NewtonSceneNodeInput* nodes,
    const std::uint32_t* incidentTriangleCounts,
    NewtonAssemblySolveRowInput* rows,
    const std::size_t nodeCount)
{
  const auto node
      = static_cast<std::size_t>(blockIdx.x * blockDim.x + threadIdx.x);
  if (node >= nodeCount) {
    return;
  }

  const NewtonSceneNodeInput input = nodes[node];
  NewtonAssemblySolveRowInput row;
  row.bodyIndex = static_cast<std::uint32_t>(node);

  const double x = input.position[0];
  const double y = input.position[1];
  const double z = input.position[2];
  const double radius = sqrt(x * x + y * y + z * z);
  const double incidentScale
      = static_cast<double>(incidentTriangleCounts[node]);

  for (std::size_t dof = 0; dof < kNewtonAssemblySolveDofsPerBody; ++dof) {
    const std::size_t axis = dof % 3u;
    const double axisPosition = input.position[axis];
    const double axisVelocity = input.velocity[axis];
    if (dof < 3u) {
      row.hessianDiagonal[dof]
          = input.mass + 0.20 * static_cast<double>(axis + 1u)
            + 0.05 * incidentScale + 0.01 * fabs(axisPosition);
      row.gradient[dof] = 0.08 * axisVelocity + 0.03 * axisPosition
                          + 0.01 * static_cast<double>(axis + 1u);
    } else {
      row.hessianDiagonal[dof] = 0.50 + 0.10 * input.mass
                                 + 0.05 * static_cast<double>(axis + 1u)
                                 + 0.025 * incidentScale + 0.005 * radius;
      row.gradient[dof] = 0.015 * static_cast<double>(axis + 1u) * axisPosition
                          - 0.02 * axisVelocity;
    }
  }

  rows[node] = row;
}

__device__ NewtonSparseBlockEntry makeSceneSparseBlockEntry(
    const NewtonSceneNodeInput* nodes,
    const std::uint32_t nodeA,
    const std::uint32_t nodeB)
{
  NewtonSparseBlockEntry block;
  block.rowBodyIndex = nodeA;
  block.columnBodyIndex = nodeB;

  const NewtonSceneNodeInput a = nodes[nodeA];
  const NewtonSceneNodeInput b = nodes[nodeB];
  double delta[3] = {};
  double velocityDelta[3] = {};
  double lengthSquared = 0.0;
  for (std::size_t axis = 0; axis < 3u; ++axis) {
    delta[axis] = b.position[axis] - a.position[axis];
    velocityDelta[axis] = b.velocity[axis] - a.velocity[axis];
    lengthSquared += delta[axis] * delta[axis];
  }

  const double length = fmax(sqrt(lengthSquared), 1e-9);
  const double massScale = 0.5 * (a.mass + b.mass);
  for (std::size_t localRow = 0; localRow < kNewtonAssemblySolveDofsPerBody;
       ++localRow) {
    const std::size_t rowAxis = localRow % 3u;
    for (std::size_t localColumn = 0;
         localColumn < kNewtonAssemblySolveDofsPerBody;
         ++localColumn) {
      const std::size_t columnAxis = localColumn % 3u;
      const double axisCoupling
          = delta[rowAxis] * delta[columnAxis] / (length * length);
      const double velocityCoupling
          = velocityDelta[rowAxis] * velocityDelta[columnAxis];
      const double diagonalBias = rowAxis == columnAxis ? 0.001 : 0.0001;
      const std::size_t entry
          = localRow * kNewtonAssemblySolveDofsPerBody + localColumn;
      block.hessianBlock[entry]
          = diagonalBias * massScale + 0.00025 * axisCoupling
            + 0.00005 * velocityCoupling
            + 0.00001 * static_cast<double>(localRow + localColumn + 1u);
    }
  }

  return block;
}

__global__ void newtonSceneSparseBlocksKernel(
    const NewtonSceneNodeInput* nodes,
    const NewtonSceneSurfaceTriangleInput* triangles,
    NewtonSparseBlockEntry* blocks,
    const std::size_t triangleCount)
{
  const auto edge
      = static_cast<std::size_t>(blockIdx.x * blockDim.x + threadIdx.x);
  const std::size_t edgeCount = 3u * triangleCount;
  if (edge >= edgeCount) {
    return;
  }

  const std::size_t triangleIndex = edge / 3u;
  const std::size_t localEdge = edge - triangleIndex * 3u;
  const NewtonSceneSurfaceTriangleInput triangle = triangles[triangleIndex];

  std::uint32_t nodeA = triangle.nodeA;
  std::uint32_t nodeB = triangle.nodeB;
  if (localEdge == 1u) {
    nodeA = triangle.nodeB;
    nodeB = triangle.nodeC;
  } else if (localEdge == 2u) {
    nodeA = triangle.nodeC;
    nodeB = triangle.nodeA;
  }

  blocks[edge] = makeSceneSparseBlockEntry(nodes, nodeA, nodeB);
}

__device__ void makeSceneDistanceEqualityBasis(
    const NewtonSceneNodeInput& node,
    const double normal[3],
    const double translationalSign,
    double basis[kNewtonAssemblySolveDofsPerBody])
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

__global__ void newtonSceneNonlinearEqualityAssemblyKernel(
    const NewtonSceneNodeInput* nodes,
    const NewtonSceneDistanceEqualityConstraintInput* constraints,
    NewtonAssemblySolveRowInput* rows,
    NewtonSparseBlockEntry* blocks,
    double* residuals,
    const std::size_t constraintCount)
{
  const auto constraintIndex
      = static_cast<std::size_t>(blockIdx.x * blockDim.x + threadIdx.x);
  if (constraintIndex >= constraintCount) {
    return;
  }

  const NewtonSceneDistanceEqualityConstraintInput constraint
      = constraints[constraintIndex];
  const NewtonSceneNodeInput nodeA = nodes[constraint.nodeA];
  const NewtonSceneNodeInput nodeB = nodes[constraint.nodeB];

  double delta[3] = {};
  double velocityDelta[3] = {};
  double lengthSquared = 0.0;
  for (std::size_t axis = 0; axis < 3u; ++axis) {
    delta[axis] = nodeB.position[axis] - nodeA.position[axis];
    velocityDelta[axis] = nodeB.velocity[axis] - nodeA.velocity[axis];
    lengthSquared += delta[axis] * delta[axis];
  }

  const double length = fmax(sqrt(lengthSquared), 1e-9);
  double normal[3] = {};
  double normalVelocity = 0.0;
  for (std::size_t axis = 0; axis < 3u; ++axis) {
    normal[axis] = delta[axis] / length;
    normalVelocity += normal[axis] * velocityDelta[axis];
  }

  const double residual = length - constraint.restLength;
  const double scalarGradient
      = constraint.stiffness * residual + constraint.damping * normalVelocity;
  residuals[constraintIndex] = residual;

  double basisA[kNewtonAssemblySolveDofsPerBody] = {};
  double basisB[kNewtonAssemblySolveDofsPerBody] = {};
  makeSceneDistanceEqualityBasis(nodeA, normal, -1.0, basisA);
  makeSceneDistanceEqualityBasis(nodeB, normal, 1.0, basisB);

  NewtonAssemblySolveRowInput rowA;
  NewtonAssemblySolveRowInput rowB;
  rowA.bodyIndex = constraint.nodeA;
  rowB.bodyIndex = constraint.nodeB;
  NewtonSparseBlockEntry block;
  block.rowBodyIndex = constraint.nodeA;
  block.columnBodyIndex = constraint.nodeB;

  for (std::size_t dof = 0; dof < kNewtonAssemblySolveDofsPerBody; ++dof) {
    const double dampingBias
        = 0.001 * constraint.damping * static_cast<double>(dof + 1u);
    rowA.hessianDiagonal[dof]
        = constraint.stiffness * basisA[dof] * basisA[dof] + dampingBias;
    rowB.hessianDiagonal[dof]
        = constraint.stiffness * basisB[dof] * basisB[dof] + dampingBias;
    rowA.gradient[dof] = scalarGradient * basisA[dof];
    rowB.gradient[dof] = scalarGradient * basisB[dof];
  }

  for (std::size_t localRow = 0; localRow < kNewtonAssemblySolveDofsPerBody;
       ++localRow) {
    for (std::size_t localColumn = 0;
         localColumn < kNewtonAssemblySolveDofsPerBody;
         ++localColumn) {
      const std::size_t entry
          = localRow * kNewtonAssemblySolveDofsPerBody + localColumn;
      block.hessianBlock[entry]
          = constraint.stiffness * basisA[localRow] * basisB[localColumn];
    }
  }

  const std::size_t rowOffset = 2u * constraintIndex;
  rows[rowOffset] = rowA;
  rows[rowOffset + 1u] = rowB;
  blocks[constraintIndex] = block;
}

__global__ void newtonSceneNonlinearEqualitySolveKernel(
    const NewtonSceneNodeInput* nodes,
    const NewtonSceneDistanceEqualityConstraintInput* constraints,
    NewtonAssemblySolveRowInput* rows,
    NewtonSparseBlockEntry* blocks,
    double* residuals,
    double* step,
    const std::size_t constraintCount,
    const double regularization)
{
  const auto constraintIndex
      = static_cast<std::size_t>(blockIdx.x * blockDim.x + threadIdx.x);
  if (constraintIndex >= constraintCount) {
    return;
  }

  const NewtonSceneDistanceEqualityConstraintInput constraint
      = constraints[constraintIndex];
  const NewtonSceneNodeInput nodeA = nodes[constraint.nodeA];
  const NewtonSceneNodeInput nodeB = nodes[constraint.nodeB];

  double delta[3] = {};
  double velocityDelta[3] = {};
  double lengthSquared = 0.0;
  for (std::size_t axis = 0; axis < 3u; ++axis) {
    delta[axis] = nodeB.position[axis] - nodeA.position[axis];
    velocityDelta[axis] = nodeB.velocity[axis] - nodeA.velocity[axis];
    lengthSquared += delta[axis] * delta[axis];
  }

  const double length = fmax(sqrt(lengthSquared), 1e-9);
  double normal[3] = {};
  double normalVelocity = 0.0;
  for (std::size_t axis = 0; axis < 3u; ++axis) {
    normal[axis] = delta[axis] / length;
    normalVelocity += normal[axis] * velocityDelta[axis];
  }

  const double residual = length - constraint.restLength;
  const double scalarGradient
      = constraint.stiffness * residual + constraint.damping * normalVelocity;
  residuals[constraintIndex] = residual;

  double basisA[kNewtonAssemblySolveDofsPerBody] = {};
  double basisB[kNewtonAssemblySolveDofsPerBody] = {};
  makeSceneDistanceEqualityBasis(nodeA, normal, -1.0, basisA);
  makeSceneDistanceEqualityBasis(nodeB, normal, 1.0, basisB);

  NewtonAssemblySolveRowInput rowA;
  NewtonAssemblySolveRowInput rowB;
  rowA.bodyIndex = constraint.nodeA;
  rowB.bodyIndex = constraint.nodeB;
  NewtonSparseBlockEntry block;
  block.rowBodyIndex = constraint.nodeA;
  block.columnBodyIndex = constraint.nodeB;

  double diagonalSum = regularization;
  for (std::size_t dof = 0; dof < kNewtonAssemblySolveDofsPerBody; ++dof) {
    const double dampingBias
        = 0.001 * constraint.damping * static_cast<double>(dof + 1u);
    rowA.hessianDiagonal[dof]
        = constraint.stiffness * basisA[dof] * basisA[dof] + dampingBias;
    rowB.hessianDiagonal[dof]
        = constraint.stiffness * basisB[dof] * basisB[dof] + dampingBias;
    rowA.gradient[dof] = scalarGradient * basisA[dof];
    rowB.gradient[dof] = scalarGradient * basisB[dof];
    diagonalSum += rowA.hessianDiagonal[dof] + rowB.hessianDiagonal[dof];
  }

  for (std::size_t localRow = 0; localRow < kNewtonAssemblySolveDofsPerBody;
       ++localRow) {
    for (std::size_t localColumn = 0;
         localColumn < kNewtonAssemblySolveDofsPerBody;
         ++localColumn) {
      const std::size_t entry
          = localRow * kNewtonAssemblySolveDofsPerBody + localColumn;
      block.hessianBlock[entry]
          = constraint.stiffness * basisA[localRow] * basisB[localColumn];
    }
  }

  const double lambda = -scalarGradient / fmax(diagonalSum, 1e-14);
  const std::size_t stepAOffset = static_cast<std::size_t>(constraint.nodeA)
                                  * kNewtonAssemblySolveDofsPerBody;
  const std::size_t stepBOffset = static_cast<std::size_t>(constraint.nodeB)
                                  * kNewtonAssemblySolveDofsPerBody;
  for (std::size_t dof = 0; dof < kNewtonAssemblySolveDofsPerBody; ++dof) {
    atomicAdd(&step[stepAOffset + dof], lambda * basisA[dof]);
    atomicAdd(&step[stepBOffset + dof], lambda * basisB[dof]);
  }

  const std::size_t rowOffset = 2u * constraintIndex;
  rows[rowOffset] = rowA;
  rows[rowOffset + 1u] = rowB;
  blocks[constraintIndex] = block;
}

__global__ void newtonSceneNonlinearEqualityPostResidualKernel(
    const NewtonSceneNodeInput* nodes,
    const NewtonSceneDistanceEqualityConstraintInput* constraints,
    const double* residuals,
    const double* step,
    double* postSolveLinearizedResiduals,
    const std::size_t constraintCount)
{
  const auto constraintIndex
      = static_cast<std::size_t>(blockIdx.x * blockDim.x + threadIdx.x);
  if (constraintIndex >= constraintCount) {
    return;
  }

  const NewtonSceneDistanceEqualityConstraintInput constraint
      = constraints[constraintIndex];
  const NewtonSceneNodeInput nodeA = nodes[constraint.nodeA];
  const NewtonSceneNodeInput nodeB = nodes[constraint.nodeB];

  double delta[3] = {};
  double lengthSquared = 0.0;
  for (std::size_t axis = 0; axis < 3u; ++axis) {
    delta[axis] = nodeB.position[axis] - nodeA.position[axis];
    lengthSquared += delta[axis] * delta[axis];
  }

  const double length = fmax(sqrt(lengthSquared), 1e-9);
  double normal[3] = {};
  for (std::size_t axis = 0; axis < 3u; ++axis) {
    normal[axis] = delta[axis] / length;
  }

  double basisA[kNewtonAssemblySolveDofsPerBody] = {};
  double basisB[kNewtonAssemblySolveDofsPerBody] = {};
  makeSceneDistanceEqualityBasis(nodeA, normal, -1.0, basisA);
  makeSceneDistanceEqualityBasis(nodeB, normal, 1.0, basisB);

  const std::size_t stepAOffset = static_cast<std::size_t>(constraint.nodeA)
                                  * kNewtonAssemblySolveDofsPerBody;
  const std::size_t stepBOffset = static_cast<std::size_t>(constraint.nodeB)
                                  * kNewtonAssemblySolveDofsPerBody;
  double linearizedCorrection = 0.0;
  for (std::size_t dof = 0; dof < kNewtonAssemblySolveDofsPerBody; ++dof) {
    linearizedCorrection += basisA[dof] * step[stepAOffset + dof];
    linearizedCorrection += basisB[dof] * step[stepBOffset + dof];
  }

  postSolveLinearizedResiduals[constraintIndex]
      = residuals[constraintIndex] + linearizedCorrection;
}

__global__ void newtonEqualityReductionKernel(
    const NewtonEqualityReductionEntry* entries,
    const double* assembledDiagonal,
    const double* assembledGradient,
    double* reducedDiagonal,
    double* reducedGradient,
    const std::size_t entryCount)
{
  const auto index
      = static_cast<std::size_t>(blockIdx.x * blockDim.x + threadIdx.x);
  if (index >= entryCount) {
    return;
  }

  const NewtonEqualityReductionEntry entry = entries[index];
  const double basis = entry.basisValue;
  atomicAdd(
      &reducedDiagonal[entry.reducedDofIndex],
      basis * basis * assembledDiagonal[entry.fullDofIndex]);
  atomicAdd(
      &reducedGradient[entry.reducedDofIndex],
      basis * assembledGradient[entry.fullDofIndex]);
}

__global__ void newtonSparseDiagonalResidualKernel(
    const double* assembledDiagonal,
    const double* step,
    double* residual,
    const std::size_t dofCount,
    const double regularization)
{
  const auto index
      = static_cast<std::size_t>(blockIdx.x * blockDim.x + threadIdx.x);
  if (index >= dofCount) {
    return;
  }

  residual[index] += (assembledDiagonal[index] + regularization) * step[index];
}

__global__ void newtonSparseBlockResidualKernel(
    const NewtonSparseBlockEntry* blocks,
    const double* step,
    double* residual,
    const std::size_t blockCount)
{
  const auto index
      = static_cast<std::size_t>(blockIdx.x * blockDim.x + threadIdx.x);
  const std::size_t totalEntries
      = blockCount * kNewtonAssemblySolveBlockEntries;
  if (index >= totalEntries) {
    return;
  }

  const std::size_t block = index / kNewtonAssemblySolveBlockEntries;
  const std::size_t entry = index - block * kNewtonAssemblySolveBlockEntries;
  const std::size_t localRow = entry / kNewtonAssemblySolveDofsPerBody;
  const std::size_t localColumn
      = entry - localRow * kNewtonAssemblySolveDofsPerBody;
  const NewtonSparseBlockEntry input = blocks[block];
  const double value = input.hessianBlock[entry];
  const std::size_t rowDof = static_cast<std::size_t>(input.rowBodyIndex)
                                 * kNewtonAssemblySolveDofsPerBody
                             + localRow;
  const std::size_t columnDof = static_cast<std::size_t>(input.columnBodyIndex)
                                    * kNewtonAssemblySolveDofsPerBody
                                + localColumn;
  atomicAdd(&residual[rowDof], value * step[columnDof]);
  atomicAdd(&residual[columnDof], value * step[rowDof]);
}

__global__ void newtonSparseJacobiUpdateKernel(
    const double* assembledDiagonal,
    const double* residual,
    double* step,
    const std::size_t dofCount,
    const double regularization,
    const double epsilonForDivision)
{
  const auto index
      = static_cast<std::size_t>(blockIdx.x * blockDim.x + threadIdx.x);
  if (index >= dofCount) {
    return;
  }

  const double effectiveDiagonal = assembledDiagonal[index] + regularization;
  if (isfinite(effectiveDiagonal)
      && fabs(effectiveDiagonal) > epsilonForDivision) {
    step[index] -= residual[index] / effectiveDiagonal;
  }
}

__global__ void newtonSparseCgSeedKernel(
    const double* assembledGradient,
    double* step,
    double* residual,
    double* direction,
    const std::size_t dofCount)
{
  const auto index
      = static_cast<std::size_t>(blockIdx.x * blockDim.x + threadIdx.x);
  if (index >= dofCount) {
    return;
  }

  const double value = -assembledGradient[index];
  step[index] = 0.0;
  residual[index] = value;
  direction[index] = value;
}

__global__ void newtonVectorDotKernel(
    const double* lhs,
    const double* rhs,
    double* output,
    const std::size_t dofCount)
{
  const auto index
      = static_cast<std::size_t>(blockIdx.x * blockDim.x + threadIdx.x);
  if (index >= dofCount) {
    return;
  }

  atomicAdd(output, lhs[index] * rhs[index]);
}

__global__ void newtonSparseCgStepKernel(
    double* step,
    double* residual,
    const double* direction,
    const double* matrixDirection,
    const std::size_t dofCount,
    const double alpha)
{
  const auto index
      = static_cast<std::size_t>(blockIdx.x * blockDim.x + threadIdx.x);
  if (index >= dofCount) {
    return;
  }

  step[index] += alpha * direction[index];
  residual[index] -= alpha * matrixDirection[index];
}

__global__ void newtonSparseCgDirectionKernel(
    const double* residual,
    double* direction,
    const std::size_t dofCount,
    const double beta)
{
  const auto index
      = static_cast<std::size_t>(blockIdx.x * blockDim.x + threadIdx.x);
  if (index >= dofCount) {
    return;
  }

  direction[index] = residual[index] + beta * direction[index];
}

__global__ void newtonDiagonalSolveKernel(
    const double* assembledDiagonal,
    const double* assembledGradient,
    double* step,
    double* residual,
    const std::size_t dofCount,
    const double regularization,
    const double epsilonForDivision)
{
  const auto index
      = static_cast<std::size_t>(blockIdx.x * blockDim.x + threadIdx.x);
  if (index >= dofCount) {
    return;
  }

  const double effectiveDiagonal = assembledDiagonal[index] + regularization;
  double value = 0.0;
  if (isfinite(effectiveDiagonal)
      && fabs(effectiveDiagonal) > epsilonForDivision) {
    value = -assembledGradient[index] / effectiveDiagonal;
  }
  step[index] = value;
  residual[index] = effectiveDiagonal * value + assembledGradient[index];
}

__global__ void newtonExpandEqualityReducedStepKernel(
    const NewtonEqualityReductionEntry* entries,
    const double* reducedStep,
    double* fullStep,
    const std::size_t entryCount)
{
  const auto index
      = static_cast<std::size_t>(blockIdx.x * blockDim.x + threadIdx.x);
  if (index >= entryCount) {
    return;
  }

  const NewtonEqualityReductionEntry entry = entries[index];
  atomicAdd(
      &fullStep[entry.fullDofIndex],
      entry.basisValue * reducedStep[entry.reducedDofIndex]);
}

} // namespace

//==============================================================================
cudaError_t launchNewtonAssemblyRowsKernel(
    const NewtonAssemblySolveRowInput* rows,
    double* assembledDiagonal,
    double* assembledGradient,
    const std::size_t rowCount)
{
  if (rowCount == 0) {
    return cudaSuccess;
  }

  constexpr unsigned int blockSize = 256;
  const std::size_t totalDofs = rowCount * kNewtonAssemblySolveDofsPerBody;
  const unsigned int gridSize = launchGrid1D(totalDofs, blockSize);
  newtonAssemblyRowsKernel<<<gridSize, blockSize>>>(
      rows, assembledDiagonal, assembledGradient, rowCount);
  return cudaGetLastError();
}

//==============================================================================
cudaError_t launchNewtonOffDiagonalAssemblyRowsKernel(
    const NewtonOffDiagonalAssemblyRowInput* rows,
    double* assembledBlocks,
    const std::size_t rowCount)
{
  if (rowCount == 0) {
    return cudaSuccess;
  }

  constexpr unsigned int blockSize = 256;
  const std::size_t totalEntries = rowCount * kNewtonAssemblySolveBlockEntries;
  const unsigned int gridSize = launchGrid1D(totalEntries, blockSize);
  newtonOffDiagonalAssemblyRowsKernel<<<gridSize, blockSize>>>(
      rows, assembledBlocks, rowCount);
  return cudaGetLastError();
}

//==============================================================================
cudaError_t launchNewtonSceneTriangleIncidenceKernel(
    const NewtonSceneSurfaceTriangleInput* triangles,
    std::uint32_t* incidentTriangleCounts,
    const std::size_t triangleCount)
{
  if (triangleCount == 0) {
    return cudaSuccess;
  }

  constexpr unsigned int blockSize = 256;
  const unsigned int gridSize = launchGrid1D(triangleCount, blockSize);
  newtonSceneTriangleIncidenceKernel<<<gridSize, blockSize>>>(
      triangles, incidentTriangleCounts, triangleCount);
  return cudaGetLastError();
}

//==============================================================================
cudaError_t launchNewtonSceneDiagonalRowsKernel(
    const NewtonSceneNodeInput* nodes,
    const std::uint32_t* incidentTriangleCounts,
    NewtonAssemblySolveRowInput* rows,
    const std::size_t nodeCount)
{
  if (nodeCount == 0) {
    return cudaSuccess;
  }

  constexpr unsigned int blockSize = 256;
  const unsigned int gridSize = launchGrid1D(nodeCount, blockSize);
  newtonSceneDiagonalRowsKernel<<<gridSize, blockSize>>>(
      nodes, incidentTriangleCounts, rows, nodeCount);
  return cudaGetLastError();
}

//==============================================================================
cudaError_t launchNewtonSceneSparseBlocksKernel(
    const NewtonSceneNodeInput* nodes,
    const NewtonSceneSurfaceTriangleInput* triangles,
    NewtonSparseBlockEntry* blocks,
    const std::size_t triangleCount)
{
  if (triangleCount == 0) {
    return cudaSuccess;
  }

  constexpr unsigned int blockSize = 256;
  const unsigned int gridSize = launchGrid1D(3u * triangleCount, blockSize);
  newtonSceneSparseBlocksKernel<<<gridSize, blockSize>>>(
      nodes, triangles, blocks, triangleCount);
  return cudaGetLastError();
}

//==============================================================================
cudaError_t launchNewtonSceneNonlinearEqualityAssemblyKernel(
    const NewtonSceneNodeInput* nodes,
    const NewtonSceneDistanceEqualityConstraintInput* constraints,
    NewtonAssemblySolveRowInput* rows,
    NewtonSparseBlockEntry* blocks,
    double* residuals,
    const std::size_t constraintCount)
{
  if (constraintCount == 0) {
    return cudaSuccess;
  }

  constexpr unsigned int blockSize = 256;
  const unsigned int gridSize = launchGrid1D(constraintCount, blockSize);
  newtonSceneNonlinearEqualityAssemblyKernel<<<gridSize, blockSize>>>(
      nodes, constraints, rows, blocks, residuals, constraintCount);
  return cudaGetLastError();
}

//==============================================================================
cudaError_t launchNewtonSceneNonlinearEqualitySolveKernel(
    const NewtonSceneNodeInput* nodes,
    const NewtonSceneDistanceEqualityConstraintInput* constraints,
    NewtonAssemblySolveRowInput* rows,
    NewtonSparseBlockEntry* blocks,
    double* residuals,
    double* step,
    const std::size_t constraintCount,
    const double regularization)
{
  if (constraintCount == 0) {
    return cudaSuccess;
  }

  constexpr unsigned int blockSize = 256;
  const unsigned int gridSize = launchGrid1D(constraintCount, blockSize);
  newtonSceneNonlinearEqualitySolveKernel<<<gridSize, blockSize>>>(
      nodes,
      constraints,
      rows,
      blocks,
      residuals,
      step,
      constraintCount,
      regularization);
  return cudaGetLastError();
}

//==============================================================================
cudaError_t launchNewtonSceneNonlinearEqualityPostResidualKernel(
    const NewtonSceneNodeInput* nodes,
    const NewtonSceneDistanceEqualityConstraintInput* constraints,
    const double* residuals,
    const double* step,
    double* postSolveLinearizedResiduals,
    const std::size_t constraintCount)
{
  if (constraintCount == 0) {
    return cudaSuccess;
  }

  constexpr unsigned int blockSize = 256;
  const unsigned int gridSize = launchGrid1D(constraintCount, blockSize);
  newtonSceneNonlinearEqualityPostResidualKernel<<<gridSize, blockSize>>>(
      nodes,
      constraints,
      residuals,
      step,
      postSolveLinearizedResiduals,
      constraintCount);
  return cudaGetLastError();
}

//==============================================================================
cudaError_t launchNewtonEqualityReductionKernel(
    const NewtonEqualityReductionEntry* entries,
    const double* assembledDiagonal,
    const double* assembledGradient,
    double* reducedDiagonal,
    double* reducedGradient,
    const std::size_t entryCount)
{
  if (entryCount == 0) {
    return cudaSuccess;
  }

  constexpr unsigned int blockSize = 256;
  const unsigned int gridSize = launchGrid1D(entryCount, blockSize);
  newtonEqualityReductionKernel<<<gridSize, blockSize>>>(
      entries,
      assembledDiagonal,
      assembledGradient,
      reducedDiagonal,
      reducedGradient,
      entryCount);
  return cudaGetLastError();
}

//==============================================================================
cudaError_t launchNewtonSparseDiagonalResidualKernel(
    const double* assembledDiagonal,
    const double* step,
    double* residual,
    const std::size_t dofCount,
    const double regularization)
{
  if (dofCount == 0) {
    return cudaSuccess;
  }

  constexpr unsigned int blockSize = 256;
  const unsigned int gridSize = launchGrid1D(dofCount, blockSize);
  newtonSparseDiagonalResidualKernel<<<gridSize, blockSize>>>(
      assembledDiagonal, step, residual, dofCount, regularization);
  return cudaGetLastError();
}

//==============================================================================
cudaError_t launchNewtonSparseBlockResidualKernel(
    const NewtonSparseBlockEntry* blocks,
    const double* step,
    double* residual,
    const std::size_t blockCount)
{
  if (blockCount == 0) {
    return cudaSuccess;
  }

  constexpr unsigned int blockSize = 256;
  const std::size_t totalEntries
      = blockCount * kNewtonAssemblySolveBlockEntries;
  const unsigned int gridSize = launchGrid1D(totalEntries, blockSize);
  newtonSparseBlockResidualKernel<<<gridSize, blockSize>>>(
      blocks, step, residual, blockCount);
  return cudaGetLastError();
}

//==============================================================================
cudaError_t launchNewtonSparseJacobiUpdateKernel(
    const double* assembledDiagonal,
    const double* residual,
    double* step,
    const std::size_t dofCount,
    const double regularization,
    const double epsilonForDivision)
{
  if (dofCount == 0) {
    return cudaSuccess;
  }

  constexpr unsigned int blockSize = 256;
  const unsigned int gridSize = launchGrid1D(dofCount, blockSize);
  newtonSparseJacobiUpdateKernel<<<gridSize, blockSize>>>(
      assembledDiagonal,
      residual,
      step,
      dofCount,
      regularization,
      epsilonForDivision);
  return cudaGetLastError();
}

//==============================================================================
cudaError_t launchNewtonSparseCgSeedKernel(
    const double* assembledGradient,
    double* step,
    double* residual,
    double* direction,
    const std::size_t dofCount)
{
  if (dofCount == 0) {
    return cudaSuccess;
  }

  constexpr unsigned int blockSize = 256;
  const unsigned int gridSize = launchGrid1D(dofCount, blockSize);
  newtonSparseCgSeedKernel<<<gridSize, blockSize>>>(
      assembledGradient, step, residual, direction, dofCount);
  return cudaGetLastError();
}

//==============================================================================
cudaError_t launchNewtonVectorDotKernel(
    const double* lhs,
    const double* rhs,
    double* output,
    const std::size_t dofCount)
{
  if (dofCount == 0) {
    return cudaSuccess;
  }

  constexpr unsigned int blockSize = 256;
  const unsigned int gridSize = launchGrid1D(dofCount, blockSize);
  newtonVectorDotKernel<<<gridSize, blockSize>>>(lhs, rhs, output, dofCount);
  return cudaGetLastError();
}

//==============================================================================
cudaError_t launchNewtonSparseCgStepKernel(
    double* step,
    double* residual,
    const double* direction,
    const double* matrixDirection,
    const std::size_t dofCount,
    const double alpha)
{
  if (dofCount == 0) {
    return cudaSuccess;
  }

  constexpr unsigned int blockSize = 256;
  const unsigned int gridSize = launchGrid1D(dofCount, blockSize);
  newtonSparseCgStepKernel<<<gridSize, blockSize>>>(
      step, residual, direction, matrixDirection, dofCount, alpha);
  return cudaGetLastError();
}

//==============================================================================
cudaError_t launchNewtonSparseCgDirectionKernel(
    const double* residual,
    double* direction,
    const std::size_t dofCount,
    const double beta)
{
  if (dofCount == 0) {
    return cudaSuccess;
  }

  constexpr unsigned int blockSize = 256;
  const unsigned int gridSize = launchGrid1D(dofCount, blockSize);
  newtonSparseCgDirectionKernel<<<gridSize, blockSize>>>(
      residual, direction, dofCount, beta);
  return cudaGetLastError();
}

//==============================================================================
cudaError_t launchNewtonDiagonalSolveKernel(
    const double* assembledDiagonal,
    const double* assembledGradient,
    double* step,
    double* residual,
    const std::size_t dofCount,
    const double regularization,
    const double epsilonForDivision)
{
  if (dofCount == 0) {
    return cudaSuccess;
  }

  constexpr unsigned int blockSize = 256;
  const unsigned int gridSize = launchGrid1D(dofCount, blockSize);
  newtonDiagonalSolveKernel<<<gridSize, blockSize>>>(
      assembledDiagonal,
      assembledGradient,
      step,
      residual,
      dofCount,
      regularization,
      epsilonForDivision);
  return cudaGetLastError();
}

//==============================================================================
cudaError_t launchNewtonExpandEqualityReducedStepKernel(
    const NewtonEqualityReductionEntry* entries,
    const double* reducedStep,
    double* fullStep,
    const std::size_t entryCount)
{
  if (entryCount == 0) {
    return cudaSuccess;
  }

  constexpr unsigned int blockSize = 256;
  const unsigned int gridSize = launchGrid1D(entryCount, blockSize);
  newtonExpandEqualityReducedStepKernel<<<gridSize, blockSize>>>(
      entries, reducedStep, fullStep, entryCount);
  return cudaGetLastError();
}

} // namespace dart::simulation::compute::cuda::detail
