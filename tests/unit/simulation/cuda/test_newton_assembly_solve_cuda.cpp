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

#include <dart/simulation/common/exceptions.hpp>

#include <dart/simulation/compute/cuda/cuda_runtime.cuh>
#include <dart/simulation/compute/cuda/newton_assembly_solve_cuda.cuh>
#include <gtest/gtest.h>

#include <algorithm>
#include <limits>
#include <vector>

#include <cmath>

namespace cuda = dart::simulation::compute::cuda;

namespace {

constexpr double kExpectedSceneNonlinearEqualityConvergenceStepScale = 0.25;

struct ExpectedSparseCgSolve
{
  std::vector<double> assembledDiagonal;
  std::vector<double> assembledGradient;
  std::vector<double> step;
  std::vector<double> residual;
  std::size_t completedIterationCount = 0;
  double initialResidualNorm = 0.0;
  double residualNorm = 0.0;
  double stepNorm = 0.0;
  double maxResidualAbs = 0.0;
  bool converged = false;
};

struct ExpectedDirectSparseSolve
{
  std::vector<double> assembledDiagonal;
  std::vector<double> assembledGradient;
  std::vector<double> factorMatrixLower;
  std::vector<double> step;
  std::vector<double> residual;
  double minimumFactorPivot = 0.0;
  double residualNorm = 0.0;
  double stepNorm = 0.0;
  double maxResidualAbs = 0.0;
  bool factorized = false;
};

struct ExpectedSceneNonlinearEqualityConvergence
{
  std::vector<cuda::NewtonAssemblySolveRowInput> rows;
  std::vector<cuda::NewtonSparseBlockEntry> blocks;
  std::vector<double> initialResiduals;
  std::vector<double> finalResiduals;
  std::vector<double> step;
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

cuda::NewtonAssemblySolveRowInput makeRow(
    const std::uint32_t body,
    const double diagonalBase,
    const double gradientBase)
{
  cuda::NewtonAssemblySolveRowInput row;
  row.bodyIndex = body;
  for (std::size_t dof = 0; dof < cuda::kNewtonAssemblySolveDofsPerBody;
       ++dof) {
    row.hessianDiagonal[dof] = diagonalBase + static_cast<double>(dof) * 0.25;
    row.gradient[dof]
        = gradientBase + static_cast<double>(static_cast<int>(dof) - 2) * 0.1;
  }
  return row;
}

void evaluateExpected(
    const std::vector<cuda::NewtonAssemblySolveRowInput>& rows,
    const std::size_t bodyCount,
    const double regularization,
    std::vector<double>& diagonal,
    std::vector<double>& gradient,
    std::vector<double>& step)
{
  const std::size_t dofCount
      = bodyCount * cuda::kNewtonAssemblySolveDofsPerBody;
  diagonal.assign(dofCount, 0.0);
  gradient.assign(dofCount, 0.0);
  step.assign(dofCount, 0.0);

  for (const auto& row : rows) {
    const std::size_t offset = static_cast<std::size_t>(row.bodyIndex)
                               * cuda::kNewtonAssemblySolveDofsPerBody;
    for (std::size_t dof = 0; dof < cuda::kNewtonAssemblySolveDofsPerBody;
         ++dof) {
      diagonal[offset + dof] += row.hessianDiagonal[dof];
      gradient[offset + dof] += row.gradient[dof];
    }
  }

  for (std::size_t dof = 0; dof < dofCount; ++dof) {
    step[dof] = -gradient[dof] / (diagonal[dof] + regularization);
  }
}

void evaluateExpectedEqualityReduced(
    const std::vector<cuda::NewtonAssemblySolveRowInput>& rows,
    const std::size_t bodyCount,
    const std::vector<cuda::NewtonEqualityReductionEntry>& entries,
    const std::size_t reducedDofCount,
    const double regularization,
    std::vector<double>& fullDiagonal,
    std::vector<double>& fullGradient,
    std::vector<double>& reducedDiagonal,
    std::vector<double>& reducedGradient,
    std::vector<double>& reducedStep,
    std::vector<double>& fullStep)
{
  std::vector<double> ignoredStep;
  evaluateExpected(
      rows, bodyCount, regularization, fullDiagonal, fullGradient, ignoredStep);
  reducedDiagonal.assign(reducedDofCount, 0.0);
  reducedGradient.assign(reducedDofCount, 0.0);
  reducedStep.assign(reducedDofCount, 0.0);
  fullStep.assign(bodyCount * cuda::kNewtonAssemblySolveDofsPerBody, 0.0);

  for (const auto& entry : entries) {
    reducedDiagonal[entry.reducedDofIndex]
        += entry.basisValue * entry.basisValue
           * fullDiagonal[entry.fullDofIndex];
    reducedGradient[entry.reducedDofIndex]
        += entry.basisValue * fullGradient[entry.fullDofIndex];
  }

  for (std::size_t reduced = 0; reduced < reducedDofCount; ++reduced) {
    reducedStep[reduced] = -reducedGradient[reduced]
                           / (reducedDiagonal[reduced] + regularization);
  }

  for (const auto& entry : entries) {
    fullStep[entry.fullDofIndex]
        += entry.basisValue * reducedStep[entry.reducedDofIndex];
  }
}

cuda::NewtonOffDiagonalAssemblyRowInput makeOffDiagonalRow(
    const std::uint32_t pair, const double base)
{
  cuda::NewtonOffDiagonalAssemblyRowInput row;
  row.pairIndex = pair;
  for (std::size_t entry = 0; entry < cuda::kNewtonAssemblySolveBlockEntries;
       ++entry) {
    const auto signedOffset = static_cast<int>(entry % 11) - 5;
    row.hessianBlock[entry] = base + 0.01 * static_cast<double>(signedOffset);
  }
  return row;
}

std::vector<double> evaluateExpectedOffDiagonalBlocks(
    const std::vector<cuda::NewtonOffDiagonalAssemblyRowInput>& rows,
    const std::size_t pairCount)
{
  std::vector<double> blocks(
      pairCount * cuda::kNewtonAssemblySolveBlockEntries, 0.0);
  for (const auto& row : rows) {
    const std::size_t offset = static_cast<std::size_t>(row.pairIndex)
                               * cuda::kNewtonAssemblySolveBlockEntries;
    for (std::size_t entry = 0; entry < cuda::kNewtonAssemblySolveBlockEntries;
         ++entry) {
      blocks[offset + entry] += row.hessianBlock[entry];
    }
  }
  return blocks;
}

cuda::NewtonSparseBlockEntry makeSparseBlock(
    const std::uint32_t rowBody,
    const std::uint32_t columnBody,
    const double base)
{
  cuda::NewtonSparseBlockEntry block;
  block.rowBodyIndex = rowBody;
  block.columnBodyIndex = columnBody;
  for (std::size_t entry = 0; entry < cuda::kNewtonAssemblySolveBlockEntries;
       ++entry) {
    const auto signedOffset = static_cast<int>(entry % 13) - 6;
    block.hessianBlock[entry]
        = base + 0.015 * static_cast<double>(signedOffset);
  }
  return block;
}

cuda::NewtonSceneNodeInput makeSceneNode(
    const double px,
    const double py,
    const double pz,
    const double vx,
    const double vy,
    const double vz,
    const double mass)
{
  cuda::NewtonSceneNodeInput node;
  node.position[0] = px;
  node.position[1] = py;
  node.position[2] = pz;
  node.velocity[0] = vx;
  node.velocity[1] = vy;
  node.velocity[2] = vz;
  node.mass = mass;
  return node;
}

cuda::NewtonAssemblySolveRowInput makeExpectedSceneRow(
    const cuda::NewtonSceneNodeInput& node,
    const std::uint32_t nodeIndex,
    const std::uint32_t incidentTriangleCount)
{
  cuda::NewtonAssemblySolveRowInput row;
  row.bodyIndex = nodeIndex;
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

cuda::NewtonSparseBlockEntry makeExpectedSceneSparseBlock(
    const std::vector<cuda::NewtonSceneNodeInput>& nodes,
    const std::uint32_t nodeA,
    const std::uint32_t nodeB)
{
  cuda::NewtonSparseBlockEntry block;
  block.rowBodyIndex = nodeA;
  block.columnBodyIndex = nodeB;

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

void evaluateExpectedSceneSparseGraphAssembly(
    const std::vector<cuda::NewtonSceneNodeInput>& nodes,
    const std::vector<cuda::NewtonSceneSurfaceTriangleInput>& triangles,
    std::vector<cuda::NewtonAssemblySolveRowInput>& rows,
    std::vector<cuda::NewtonSparseBlockEntry>& blocks)
{
  std::vector<std::uint32_t> incidentTriangleCounts(nodes.size(), 0u);
  for (const auto& triangle : triangles) {
    ++incidentTriangleCounts[triangle.nodeA];
    ++incidentTriangleCounts[triangle.nodeB];
    ++incidentTriangleCounts[triangle.nodeC];
  }

  rows.clear();
  rows.reserve(nodes.size());
  for (std::size_t node = 0; node < nodes.size(); ++node) {
    rows.push_back(makeExpectedSceneRow(
        nodes[node],
        static_cast<std::uint32_t>(node),
        incidentTriangleCounts[node]));
  }

  blocks.clear();
  blocks.reserve(3u * triangles.size());
  for (const auto& triangle : triangles) {
    blocks.push_back(
        makeExpectedSceneSparseBlock(nodes, triangle.nodeA, triangle.nodeB));
    blocks.push_back(
        makeExpectedSceneSparseBlock(nodes, triangle.nodeB, triangle.nodeC));
    blocks.push_back(
        makeExpectedSceneSparseBlock(nodes, triangle.nodeC, triangle.nodeA));
  }
}

cuda::NewtonSceneDistanceEqualityConstraintInput makeDistanceEqualityConstraint(
    const std::uint32_t nodeA,
    const std::uint32_t nodeB,
    const double restLength,
    const double stiffness,
    const double damping)
{
  cuda::NewtonSceneDistanceEqualityConstraintInput constraint;
  constraint.nodeA = nodeA;
  constraint.nodeB = nodeB;
  constraint.restLength = restLength;
  constraint.stiffness = stiffness;
  constraint.damping = damping;
  return constraint;
}

void makeExpectedDistanceEqualityBasis(
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

void appendExpectedDistanceEqualityContribution(
    const std::vector<cuda::NewtonSceneNodeInput>& nodes,
    const cuda::NewtonSceneDistanceEqualityConstraintInput& constraint,
    std::vector<cuda::NewtonAssemblySolveRowInput>& rows,
    std::vector<cuda::NewtonSparseBlockEntry>& blocks,
    std::vector<double>& residuals)
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
  residuals.push_back(residual);

  double basisA[cuda::kNewtonAssemblySolveDofsPerBody] = {};
  double basisB[cuda::kNewtonAssemblySolveDofsPerBody] = {};
  makeExpectedDistanceEqualityBasis(nodeA, normal, -1.0, basisA);
  makeExpectedDistanceEqualityBasis(nodeB, normal, 1.0, basisB);

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

  rows.push_back(rowA);
  rows.push_back(rowB);
  blocks.push_back(block);
}

void evaluateExpectedSceneNonlinearEqualityAssembly(
    const std::vector<cuda::NewtonSceneNodeInput>& nodes,
    const std::vector<cuda::NewtonSceneDistanceEqualityConstraintInput>&
        constraints,
    std::vector<cuda::NewtonAssemblySolveRowInput>& rows,
    std::vector<cuda::NewtonSparseBlockEntry>& blocks,
    std::vector<double>& residuals)
{
  rows.clear();
  blocks.clear();
  residuals.clear();
  rows.reserve(2u * constraints.size());
  blocks.reserve(constraints.size());
  residuals.reserve(constraints.size());
  for (const auto& constraint : constraints) {
    appendExpectedDistanceEqualityContribution(
        nodes, constraint, rows, blocks, residuals);
  }
}

void evaluateExpectedSceneNonlinearEqualitySolve(
    const std::vector<cuda::NewtonSceneNodeInput>& nodes,
    const std::vector<cuda::NewtonSceneDistanceEqualityConstraintInput>&
        constraints,
    const double regularization,
    std::vector<cuda::NewtonAssemblySolveRowInput>& rows,
    std::vector<cuda::NewtonSparseBlockEntry>& blocks,
    std::vector<double>& residuals,
    std::vector<double>& step,
    std::vector<double>& postSolveLinearizedResiduals)
{
  evaluateExpectedSceneNonlinearEqualityAssembly(
      nodes, constraints, rows, blocks, residuals);
  step.assign(nodes.size() * cuda::kNewtonAssemblySolveDofsPerBody, 0.0);
  postSolveLinearizedResiduals.assign(constraints.size(), 0.0);

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
    makeExpectedDistanceEqualityBasis(nodeA, normal, -1.0, basisA);
    makeExpectedDistanceEqualityBasis(nodeB, normal, 1.0, basisB);

    const std::size_t rowOffset = 2u * constraintIndex;
    double diagonalSum = regularization;
    for (std::size_t dof = 0; dof < cuda::kNewtonAssemblySolveDofsPerBody;
         ++dof) {
      diagonalSum += rows[rowOffset].hessianDiagonal[dof]
                     + rows[rowOffset + 1u].hessianDiagonal[dof];
    }
    const double lambda = -scalarGradient / std::max(diagonalSum, 1e-14);

    const std::size_t stepAOffset = static_cast<std::size_t>(constraint.nodeA)
                                    * cuda::kNewtonAssemblySolveDofsPerBody;
    const std::size_t stepBOffset = static_cast<std::size_t>(constraint.nodeB)
                                    * cuda::kNewtonAssemblySolveDofsPerBody;
    for (std::size_t dof = 0; dof < cuda::kNewtonAssemblySolveDofsPerBody;
         ++dof) {
      step[stepAOffset + dof] += lambda * basisA[dof];
      step[stepBOffset + dof] += lambda * basisB[dof];
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
    makeExpectedDistanceEqualityBasis(nodeA, normal, -1.0, basisA);
    makeExpectedDistanceEqualityBasis(nodeB, normal, 1.0, basisB);

    const std::size_t stepAOffset = static_cast<std::size_t>(constraint.nodeA)
                                    * cuda::kNewtonAssemblySolveDofsPerBody;
    const std::size_t stepBOffset = static_cast<std::size_t>(constraint.nodeB)
                                    * cuda::kNewtonAssemblySolveDofsPerBody;
    double linearizedCorrection = 0.0;
    for (std::size_t dof = 0; dof < cuda::kNewtonAssemblySolveDofsPerBody;
         ++dof) {
      linearizedCorrection += basisA[dof] * step[stepAOffset + dof];
      linearizedCorrection += basisB[dof] * step[stepBOffset + dof];
    }
    postSolveLinearizedResiduals[constraintIndex]
        = residuals[constraintIndex] + linearizedCorrection;
  }
}

double maxAbs(const std::vector<double>& values)
{
  double maxValue = 0.0;
  for (const auto value : values) {
    maxValue = std::max(maxValue, std::abs(value));
  }
  return maxValue;
}

ExpectedSceneNonlinearEqualityConvergence
evaluateExpectedSceneNonlinearEqualityConvergence(
    const std::vector<cuda::NewtonSceneNodeInput>& nodes,
    const std::vector<cuda::NewtonSceneDistanceEqualityConstraintInput>&
        constraints,
    const double regularization,
    const std::size_t maxIterationCount,
    const double residualTolerance)
{
  ExpectedSceneNonlinearEqualityConvergence result;
  std::vector<cuda::NewtonSceneNodeInput> currentNodes = nodes;
  std::vector<cuda::NewtonAssemblySolveRowInput> ignoredRows;
  std::vector<cuda::NewtonSparseBlockEntry> ignoredBlocks;
  evaluateExpectedSceneNonlinearEqualityAssembly(
      currentNodes,
      constraints,
      ignoredRows,
      ignoredBlocks,
      result.initialResiduals);

  result.step.assign(nodes.size() * cuda::kNewtonAssemblySolveDofsPerBody, 0.0);
  double currentMaxResidual = maxAbs(result.initialResiduals);
  result.initialMaxConstraintResidualAbs = currentMaxResidual;

  for (std::size_t iteration = 0;
       currentMaxResidual > residualTolerance && iteration < maxIterationCount;
       ++iteration) {
    std::vector<cuda::NewtonAssemblySolveRowInput> iterationRows;
    std::vector<cuda::NewtonSparseBlockEntry> iterationBlocks;
    std::vector<double> iterationResiduals;
    std::vector<double> iterationStep;
    std::vector<double> ignoredPostSolveResiduals;
    evaluateExpectedSceneNonlinearEqualitySolve(
        currentNodes,
        constraints,
        regularization,
        iterationRows,
        iterationBlocks,
        iterationResiduals,
        iterationStep,
        ignoredPostSolveResiduals);

    for (std::size_t node = 0; node < currentNodes.size(); ++node) {
      const std::size_t offset = node * cuda::kNewtonAssemblySolveDofsPerBody;
      for (std::size_t dof = 0; dof < cuda::kNewtonAssemblySolveDofsPerBody;
           ++dof) {
        const double correction
            = kExpectedSceneNonlinearEqualityConvergenceStepScale
              * iterationStep[offset + dof];
        result.step[offset + dof] += correction;
        if (dof < 3u) {
          currentNodes[node].position[dof] += correction;
        }
      }
    }

    evaluateExpectedSceneNonlinearEqualityAssembly(
        currentNodes,
        constraints,
        result.rows,
        result.blocks,
        result.finalResiduals);
    ++result.completedIterationCount;
    currentMaxResidual = maxAbs(result.finalResiduals);
  }

  evaluateExpectedSceneNonlinearEqualityAssembly(
      currentNodes,
      constraints,
      result.rows,
      result.blocks,
      result.finalResiduals);
  result.finalMaxConstraintResidualAbs = maxAbs(result.finalResiduals);
  result.converged = result.finalMaxConstraintResidualAbs <= residualTolerance;

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

  double stepNormSquared = 0.0;
  for (const auto value : result.step) {
    if (std::abs(value) > 0.0) {
      ++result.activeDofCount;
    }
    stepNormSquared += value * value;
  }
  result.stepNorm = std::sqrt(stepNormSquared);
  return result;
}

void evaluateExpectedSparseResidual(
    const std::vector<cuda::NewtonAssemblySolveRowInput>& rows,
    const std::size_t bodyCount,
    const std::vector<cuda::NewtonSparseBlockEntry>& blocks,
    const std::vector<double>& step,
    const double regularization,
    std::vector<double>& diagonal,
    std::vector<double>& gradient,
    std::vector<double>& residual)
{
  std::vector<double> ignoredStep;
  evaluateExpected(
      rows, bodyCount, regularization, diagonal, gradient, ignoredStep);
  residual = gradient;
  for (std::size_t dof = 0; dof < residual.size(); ++dof) {
    residual[dof] += (diagonal[dof] + regularization) * step[dof];
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
}

std::vector<double> evaluateExpectedSparseMatrixVector(
    const std::vector<double>& diagonal,
    const std::vector<cuda::NewtonSparseBlockEntry>& blocks,
    const std::vector<double>& vector,
    const double regularization)
{
  std::vector<double> product(vector.size(), 0.0);
  for (std::size_t dof = 0; dof < product.size(); ++dof) {
    product[dof] = (diagonal[dof] + regularization) * vector[dof];
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
      product[rowDof] += block.hessianBlock[entry] * vector[columnDof];
      product[columnDof] += block.hessianBlock[entry] * vector[rowDof];
    }
  }
  return product;
}

std::vector<double> assembleExpectedDenseSparseMatrix(
    const std::vector<double>& diagonal,
    const std::vector<cuda::NewtonSparseBlockEntry>& blocks,
    const double regularization)
{
  const std::size_t dofCount = diagonal.size();
  std::vector<double> matrix(dofCount * dofCount, 0.0);
  for (std::size_t dof = 0; dof < dofCount; ++dof) {
    matrix[dof * dofCount + dof] = diagonal[dof] + regularization;
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

bool factorAndSolveExpectedDenseCholesky(
    std::vector<double>& matrix,
    const std::vector<double>& gradient,
    std::vector<double>& step,
    double& minimumFactorPivot)
{
  constexpr double kPivotEpsilon = 1e-14;
  const std::size_t dofCount = gradient.size();
  minimumFactorPivot = std::numeric_limits<double>::infinity();

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
    double value = -gradient[row];
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

double dotVectors(
    const std::vector<double>& lhs, const std::vector<double>& rhs)
{
  double dot = 0.0;
  for (std::size_t dof = 0; dof < lhs.size(); ++dof) {
    dot += lhs[dof] * rhs[dof];
  }
  return dot;
}

ExpectedSparseCgSolve evaluateExpectedSparseCgSolve(
    const std::vector<cuda::NewtonAssemblySolveRowInput>& rows,
    const std::size_t bodyCount,
    const std::vector<cuda::NewtonSparseBlockEntry>& blocks,
    const std::size_t maxIterationCount,
    const double residualTolerance,
    const double regularization)
{
  ExpectedSparseCgSolve result;
  std::vector<double> ignoredStep;
  evaluateExpected(
      rows,
      bodyCount,
      regularization,
      result.assembledDiagonal,
      result.assembledGradient,
      ignoredStep);

  const std::size_t dofCount
      = bodyCount * cuda::kNewtonAssemblySolveDofsPerBody;
  result.step.assign(dofCount, 0.0);
  std::vector<double> searchDirection(dofCount, 0.0);
  std::vector<double> cgResidual(dofCount, 0.0);
  for (std::size_t dof = 0; dof < dofCount; ++dof) {
    cgResidual[dof] = -result.assembledGradient[dof];
    searchDirection[dof] = cgResidual[dof];
  }

  double residualSquared = std::max(0.0, dotVectors(cgResidual, cgResidual));
  result.initialResidualNorm = std::sqrt(residualSquared);
  const double toleranceSquared = residualTolerance * residualTolerance;

  for (std::size_t iteration = 0; iteration < maxIterationCount; ++iteration) {
    if (residualSquared <= toleranceSquared) {
      break;
    }

    const auto matrixDirection = evaluateExpectedSparseMatrixVector(
        result.assembledDiagonal, blocks, searchDirection, regularization);
    const double directionMatrixDirection
        = dotVectors(searchDirection, matrixDirection);
    if (!std::isfinite(directionMatrixDirection)
        || std::abs(directionMatrixDirection) <= 1e-30) {
      break;
    }

    const double alpha = residualSquared / directionMatrixDirection;
    for (std::size_t dof = 0; dof < dofCount; ++dof) {
      result.step[dof] += alpha * searchDirection[dof];
      cgResidual[dof] -= alpha * matrixDirection[dof];
    }

    double nextResidualSquared
        = std::max(0.0, dotVectors(cgResidual, cgResidual));
    ++result.completedIterationCount;
    if (nextResidualSquared <= toleranceSquared) {
      residualSquared = nextResidualSquared;
      break;
    }

    const double beta = nextResidualSquared / residualSquared;
    for (std::size_t dof = 0; dof < dofCount; ++dof) {
      searchDirection[dof] = cgResidual[dof] + beta * searchDirection[dof];
    }
    residualSquared = nextResidualSquared;
  }

  evaluateExpectedSparseResidual(
      rows,
      bodyCount,
      blocks,
      result.step,
      regularization,
      result.assembledDiagonal,
      result.assembledGradient,
      result.residual);

  double stepNormSquared = 0.0;
  double residualNormSquared = 0.0;
  for (std::size_t dof = 0; dof < dofCount; ++dof) {
    stepNormSquared += result.step[dof] * result.step[dof];
    const double residualAbs = std::abs(result.residual[dof]);
    result.maxResidualAbs = std::max(result.maxResidualAbs, residualAbs);
    residualNormSquared += result.residual[dof] * result.residual[dof];
  }
  result.stepNorm = std::sqrt(stepNormSquared);
  result.residualNorm = std::sqrt(residualNormSquared);
  result.converged = result.residualNorm <= residualTolerance;
  return result;
}

ExpectedDirectSparseSolve evaluateExpectedDirectSparseSolve(
    const std::vector<cuda::NewtonAssemblySolveRowInput>& rows,
    const std::size_t bodyCount,
    const std::vector<cuda::NewtonSparseBlockEntry>& blocks,
    const double regularization)
{
  ExpectedDirectSparseSolve result;
  std::vector<double> ignoredStep;
  evaluateExpected(
      rows,
      bodyCount,
      regularization,
      result.assembledDiagonal,
      result.assembledGradient,
      ignoredStep);

  result.factorMatrixLower = assembleExpectedDenseSparseMatrix(
      result.assembledDiagonal, blocks, regularization);
  result.factorized = factorAndSolveExpectedDenseCholesky(
      result.factorMatrixLower,
      result.assembledGradient,
      result.step,
      result.minimumFactorPivot);
  if (!result.factorized) {
    return result;
  }

  evaluateExpectedSparseResidual(
      rows,
      bodyCount,
      blocks,
      result.step,
      regularization,
      result.assembledDiagonal,
      result.assembledGradient,
      result.residual);

  double stepNormSquared = 0.0;
  double residualNormSquared = 0.0;
  for (std::size_t dof = 0; dof < result.step.size(); ++dof) {
    stepNormSquared += result.step[dof] * result.step[dof];
    const double residualAbs = std::abs(result.residual[dof]);
    result.maxResidualAbs = std::max(result.maxResidualAbs, residualAbs);
    residualNormSquared += result.residual[dof] * result.residual[dof];
  }
  result.stepNorm = std::sqrt(stepNormSquared);
  result.residualNorm = std::sqrt(residualNormSquared);
  return result;
}

void evaluateExpectedSparseJacobiSolve(
    const std::vector<cuda::NewtonAssemblySolveRowInput>& rows,
    const std::size_t bodyCount,
    const std::vector<cuda::NewtonSparseBlockEntry>& blocks,
    const std::size_t iterationCount,
    const double regularization,
    std::vector<double>& diagonal,
    std::vector<double>& gradient,
    std::vector<double>& step,
    std::vector<double>& residual)
{
  std::vector<double> ignoredStep;
  evaluateExpected(
      rows, bodyCount, regularization, diagonal, gradient, ignoredStep);
  step.assign(bodyCount * cuda::kNewtonAssemblySolveDofsPerBody, 0.0);

  for (std::size_t iteration = 0; iteration < iterationCount; ++iteration) {
    std::vector<double> iterationResidual;
    evaluateExpectedSparseResidual(
        rows,
        bodyCount,
        blocks,
        step,
        regularization,
        diagonal,
        gradient,
        iterationResidual);
    for (std::size_t dof = 0; dof < step.size(); ++dof) {
      step[dof] -= iterationResidual[dof] / (diagonal[dof] + regularization);
    }
  }

  evaluateExpectedSparseResidual(
      rows,
      bodyCount,
      blocks,
      step,
      regularization,
      diagonal,
      gradient,
      residual);
}

} // namespace

//==============================================================================
TEST(NewtonAssemblySolveCuda, MatchesCpuDiagonalAssemblyAndSolve)
{
  if (!cuda::isCudaRuntimeAvailable()) {
    GTEST_SKIP() << "CUDA runtime has no available device";
  }

  const std::vector rows = {
      makeRow(0, 2.0, -0.5),
      makeRow(1, 3.0, 0.25),
      makeRow(0, 1.5, 0.75),
  };
  constexpr std::size_t bodyCount = 2;
  constexpr double regularization = 0.5;

  cuda::NewtonAssemblySolveResult result;
  cuda::evaluateNewtonAssemblySolveCuda(
      rows, bodyCount, regularization, result);

  std::vector<double> expectedDiagonal;
  std::vector<double> expectedGradient;
  std::vector<double> expectedStep;
  evaluateExpected(
      rows,
      bodyCount,
      regularization,
      expectedDiagonal,
      expectedGradient,
      expectedStep);

  ASSERT_EQ(result.assembledDiagonal.size(), expectedDiagonal.size());
  ASSERT_EQ(result.assembledGradient.size(), expectedGradient.size());
  ASSERT_EQ(result.step.size(), expectedStep.size());
  EXPECT_EQ(result.bodyCount, bodyCount);
  EXPECT_EQ(result.rowCount, rows.size());
  EXPECT_EQ(
      result.activeDofCount, bodyCount * cuda::kNewtonAssemblySolveDofsPerBody);
  EXPECT_LT(result.residualNorm, 1e-12);

  for (std::size_t dof = 0; dof < expectedStep.size(); ++dof) {
    EXPECT_NEAR(result.assembledDiagonal[dof], expectedDiagonal[dof], 1e-12)
        << dof;
    EXPECT_NEAR(result.assembledGradient[dof], expectedGradient[dof], 1e-12)
        << dof;
    EXPECT_NEAR(result.step[dof], expectedStep[dof], 1e-12) << dof;
    EXPECT_NEAR(result.residual[dof], 0.0, 1e-12) << dof;
  }
}

//==============================================================================
TEST(NewtonAssemblySolveCuda, RejectsInvalidRows)
{
  if (!cuda::isCudaRuntimeAvailable()) {
    GTEST_SKIP() << "CUDA runtime has no available device";
  }

  std::vector rows = {makeRow(2, 1.0, 0.0)};
  cuda::NewtonAssemblySolveResult result;
  EXPECT_THROW(
      cuda::evaluateNewtonAssemblySolveCuda(rows, 2, 0.1, result),
      dart::simulation::InvalidArgumentException);

  rows = {makeRow(0, 1.0, 0.0)};
  rows.front().gradient[0] = std::numeric_limits<double>::infinity();
  EXPECT_THROW(
      cuda::evaluateNewtonAssemblySolveCuda(rows, 1, 0.1, result),
      dart::simulation::InvalidArgumentException);
}

//==============================================================================
TEST(NewtonAssemblySolveCuda, MatchesCpuOffDiagonalSparseBlockAssembly)
{
  if (!cuda::isCudaRuntimeAvailable()) {
    GTEST_SKIP() << "CUDA runtime has no available device";
  }

  const std::vector rows = {
      makeOffDiagonalRow(0, 0.5),
      makeOffDiagonalRow(1, -0.25),
      makeOffDiagonalRow(0, 0.125),
      makeOffDiagonalRow(2, 0.75),
  };
  constexpr std::size_t pairCount = 3;

  cuda::NewtonOffDiagonalAssemblyResult result;
  cuda::evaluateNewtonOffDiagonalAssemblyCuda(rows, pairCount, result);

  const auto expected = evaluateExpectedOffDiagonalBlocks(rows, pairCount);
  ASSERT_EQ(result.assembledBlocks.size(), expected.size());
  EXPECT_EQ(result.pairCount, pairCount);
  EXPECT_EQ(result.rowCount, rows.size());
  EXPECT_EQ(result.activeBlockCount, pairCount);

  for (std::size_t entry = 0; entry < expected.size(); ++entry) {
    EXPECT_NEAR(result.assembledBlocks[entry], expected[entry], 1e-12) << entry;
  }
}

//==============================================================================
TEST(NewtonAssemblySolveCuda, RejectsInvalidOffDiagonalRows)
{
  if (!cuda::isCudaRuntimeAvailable()) {
    GTEST_SKIP() << "CUDA runtime has no available device";
  }

  std::vector rows = {makeOffDiagonalRow(2, 1.0)};
  cuda::NewtonOffDiagonalAssemblyResult result;
  EXPECT_THROW(
      cuda::evaluateNewtonOffDiagonalAssemblyCuda(rows, 2, result),
      dart::simulation::InvalidArgumentException);

  rows = {makeOffDiagonalRow(0, 1.0)};
  rows.front().hessianBlock[0] = std::numeric_limits<double>::infinity();
  EXPECT_THROW(
      cuda::evaluateNewtonOffDiagonalAssemblyCuda(rows, 1, result),
      dart::simulation::InvalidArgumentException);
}

//==============================================================================
TEST(NewtonAssemblySolveCuda, MatchesCpuSceneSparseGraphAssembly)
{
  if (!cuda::isCudaRuntimeAvailable()) {
    GTEST_SKIP() << "CUDA runtime has no available device";
  }

  const std::vector nodes = {
      makeSceneNode(0.0, 0.0, 0.0, 0.10, -0.20, 0.30, 1.0),
      makeSceneNode(1.0, 0.0, 0.2, -0.10, 0.40, -0.15, 1.5),
      makeSceneNode(0.0, 1.0, -0.1, 0.20, 0.05, -0.25, 2.0),
      makeSceneNode(1.0, 1.0, 0.3, -0.35, 0.15, 0.45, 2.5),
  };
  const std::vector<cuda::NewtonSceneSurfaceTriangleInput> triangles = {
      {0, 1, 2},
      {1, 3, 2},
  };

  cuda::NewtonSceneSparseGraphAssemblyResult result;
  cuda::evaluateNewtonSceneSparseGraphAssemblyCuda(nodes, triangles, result);

  std::vector<cuda::NewtonAssemblySolveRowInput> expectedRows;
  std::vector<cuda::NewtonSparseBlockEntry> expectedBlocks;
  evaluateExpectedSceneSparseGraphAssembly(
      nodes, triangles, expectedRows, expectedBlocks);

  EXPECT_EQ(result.nodeCount, nodes.size());
  EXPECT_EQ(result.triangleCount, triangles.size());
  EXPECT_EQ(result.rowCount, expectedRows.size());
  EXPECT_EQ(result.bodyCount, nodes.size());
  EXPECT_EQ(
      result.dofCount, nodes.size() * cuda::kNewtonAssemblySolveDofsPerBody);
  EXPECT_EQ(result.blockCount, expectedBlocks.size());
  EXPECT_EQ(
      result.blockEntryCount,
      expectedBlocks.size() * cuda::kNewtonAssemblySolveBlockEntries);
  ASSERT_EQ(result.rows.size(), expectedRows.size());
  ASSERT_EQ(result.blocks.size(), expectedBlocks.size());

  for (std::size_t row = 0; row < expectedRows.size(); ++row) {
    EXPECT_EQ(result.rows[row].bodyIndex, expectedRows[row].bodyIndex);
    for (std::size_t dof = 0; dof < cuda::kNewtonAssemblySolveDofsPerBody;
         ++dof) {
      EXPECT_NEAR(
          result.rows[row].hessianDiagonal[dof],
          expectedRows[row].hessianDiagonal[dof],
          1e-12)
          << row << "/" << dof;
      EXPECT_NEAR(
          result.rows[row].gradient[dof],
          expectedRows[row].gradient[dof],
          1e-12)
          << row << "/" << dof;
    }
  }

  for (std::size_t block = 0; block < expectedBlocks.size(); ++block) {
    EXPECT_EQ(
        result.blocks[block].rowBodyIndex, expectedBlocks[block].rowBodyIndex);
    EXPECT_EQ(
        result.blocks[block].columnBodyIndex,
        expectedBlocks[block].columnBodyIndex);
    for (std::size_t entry = 0; entry < cuda::kNewtonAssemblySolveBlockEntries;
         ++entry) {
      EXPECT_NEAR(
          result.blocks[block].hessianBlock[entry],
          expectedBlocks[block].hessianBlock[entry],
          1e-12)
          << block << "/" << entry;
    }
  }
}

//==============================================================================
TEST(NewtonAssemblySolveCuda, RejectsInvalidSceneSparseGraphInputs)
{
  if (!cuda::isCudaRuntimeAvailable()) {
    GTEST_SKIP() << "CUDA runtime has no available device";
  }

  std::vector nodes = {
      makeSceneNode(0.0, 0.0, 0.0, 0.10, -0.20, 0.30, 1.0),
      makeSceneNode(1.0, 0.0, 0.2, -0.10, 0.40, -0.15, 1.5),
      makeSceneNode(0.0, 1.0, -0.1, 0.20, 0.05, -0.25, 2.0),
  };
  cuda::NewtonSceneSparseGraphAssemblyResult result;

  EXPECT_THROW(
      cuda::evaluateNewtonSceneSparseGraphAssemblyCuda(
          nodes, {{0, 1, 3}}, result),
      dart::simulation::InvalidArgumentException);
  EXPECT_THROW(
      cuda::evaluateNewtonSceneSparseGraphAssemblyCuda(
          nodes, {{0, 1, 1}}, result),
      dart::simulation::InvalidArgumentException);

  nodes.front().mass = -1.0;
  EXPECT_THROW(
      cuda::evaluateNewtonSceneSparseGraphAssemblyCuda(
          nodes, {{0, 1, 2}}, result),
      dart::simulation::InvalidArgumentException);

  nodes.front().mass = 1.0;
  nodes.front().position[0] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(
      cuda::evaluateNewtonSceneSparseGraphAssemblyCuda(
          nodes, {{0, 1, 2}}, result),
      dart::simulation::InvalidArgumentException);
}

//==============================================================================
TEST(NewtonAssemblySolveCuda, MatchesCpuSceneNonlinearEqualityAssembly)
{
  if (!cuda::isCudaRuntimeAvailable()) {
    GTEST_SKIP() << "CUDA runtime has no available device";
  }

  const std::vector nodes = {
      makeSceneNode(0.0, 0.0, 0.0, 0.10, -0.20, 0.30, 1.0),
      makeSceneNode(1.0, 0.2, 0.1, -0.10, 0.40, -0.15, 1.5),
      makeSceneNode(0.1, 1.1, -0.2, 0.20, 0.05, -0.25, 2.0),
  };
  const std::vector constraints = {
      makeDistanceEqualityConstraint(0, 1, 0.95, 2.5, 0.2),
      makeDistanceEqualityConstraint(1, 2, 1.15, 1.75, 0.35),
  };

  cuda::NewtonSceneNonlinearEqualityAssemblyResult result;
  cuda::evaluateNewtonSceneNonlinearEqualityAssemblyCuda(
      nodes, constraints, result);

  std::vector<cuda::NewtonAssemblySolveRowInput> expectedRows;
  std::vector<cuda::NewtonSparseBlockEntry> expectedBlocks;
  std::vector<double> expectedResiduals;
  evaluateExpectedSceneNonlinearEqualityAssembly(
      nodes, constraints, expectedRows, expectedBlocks, expectedResiduals);

  EXPECT_EQ(result.nodeCount, nodes.size());
  EXPECT_EQ(result.constraintCount, constraints.size());
  EXPECT_EQ(result.rowCount, expectedRows.size());
  EXPECT_EQ(result.bodyCount, nodes.size());
  EXPECT_EQ(
      result.dofCount, nodes.size() * cuda::kNewtonAssemblySolveDofsPerBody);
  EXPECT_EQ(result.blockCount, expectedBlocks.size());
  EXPECT_EQ(
      result.blockEntryCount,
      expectedBlocks.size() * cuda::kNewtonAssemblySolveBlockEntries);
  ASSERT_EQ(result.rows.size(), expectedRows.size());
  ASSERT_EQ(result.blocks.size(), expectedBlocks.size());
  ASSERT_EQ(result.residuals.size(), expectedResiduals.size());

  for (std::size_t constraint = 0; constraint < expectedResiduals.size();
       ++constraint) {
    EXPECT_NEAR(
        result.residuals[constraint], expectedResiduals[constraint], 1e-12)
        << constraint;
  }

  for (std::size_t row = 0; row < expectedRows.size(); ++row) {
    EXPECT_EQ(result.rows[row].bodyIndex, expectedRows[row].bodyIndex);
    for (std::size_t dof = 0; dof < cuda::kNewtonAssemblySolveDofsPerBody;
         ++dof) {
      EXPECT_NEAR(
          result.rows[row].hessianDiagonal[dof],
          expectedRows[row].hessianDiagonal[dof],
          1e-12)
          << row << "/" << dof;
      EXPECT_NEAR(
          result.rows[row].gradient[dof],
          expectedRows[row].gradient[dof],
          1e-12)
          << row << "/" << dof;
    }
  }

  for (std::size_t block = 0; block < expectedBlocks.size(); ++block) {
    EXPECT_EQ(
        result.blocks[block].rowBodyIndex, expectedBlocks[block].rowBodyIndex);
    EXPECT_EQ(
        result.blocks[block].columnBodyIndex,
        expectedBlocks[block].columnBodyIndex);
    for (std::size_t entry = 0; entry < cuda::kNewtonAssemblySolveBlockEntries;
         ++entry) {
      EXPECT_NEAR(
          result.blocks[block].hessianBlock[entry],
          expectedBlocks[block].hessianBlock[entry],
          1e-12)
          << block << "/" << entry;
    }
  }
}

//==============================================================================
TEST(NewtonAssemblySolveCuda, MatchesCpuSceneNonlinearEqualitySolve)
{
  if (!cuda::isCudaRuntimeAvailable()) {
    GTEST_SKIP() << "CUDA runtime has no available device";
  }

  const std::vector nodes = {
      makeSceneNode(0.0, 0.0, 0.0, 0.10, -0.20, 0.30, 1.0),
      makeSceneNode(1.0, 0.2, 0.1, -0.10, 0.40, -0.15, 1.5),
      makeSceneNode(0.1, 1.1, -0.2, 0.20, 0.05, -0.25, 2.0),
  };
  const std::vector constraints = {
      makeDistanceEqualityConstraint(0, 1, 0.95, 2.5, 0.2),
      makeDistanceEqualityConstraint(1, 2, 1.15, 1.75, 0.35),
  };
  constexpr double regularization = 0.05;

  cuda::NewtonSceneNonlinearEqualitySolveResult result;
  cuda::evaluateNewtonSceneNonlinearEqualitySolveCuda(
      nodes, constraints, regularization, result);

  std::vector<cuda::NewtonAssemblySolveRowInput> expectedRows;
  std::vector<cuda::NewtonSparseBlockEntry> expectedBlocks;
  std::vector<double> expectedResiduals;
  std::vector<double> expectedStep;
  std::vector<double> expectedPostSolveResiduals;
  evaluateExpectedSceneNonlinearEqualitySolve(
      nodes,
      constraints,
      regularization,
      expectedRows,
      expectedBlocks,
      expectedResiduals,
      expectedStep,
      expectedPostSolveResiduals);

  EXPECT_EQ(result.nodeCount, nodes.size());
  EXPECT_EQ(result.constraintCount, constraints.size());
  EXPECT_EQ(result.rowCount, expectedRows.size());
  EXPECT_EQ(result.bodyCount, nodes.size());
  EXPECT_EQ(
      result.dofCount, nodes.size() * cuda::kNewtonAssemblySolveDofsPerBody);
  EXPECT_EQ(result.blockCount, expectedBlocks.size());
  EXPECT_EQ(
      result.blockEntryCount,
      expectedBlocks.size() * cuda::kNewtonAssemblySolveBlockEntries);
  EXPECT_EQ(result.regularization, regularization);
  EXPECT_GT(result.activeDofCount, 0u);
  EXPECT_GT(result.stepNorm, 0.0);
  ASSERT_EQ(result.rows.size(), expectedRows.size());
  ASSERT_EQ(result.blocks.size(), expectedBlocks.size());
  ASSERT_EQ(result.residuals.size(), expectedResiduals.size());
  ASSERT_EQ(result.step.size(), expectedStep.size());
  ASSERT_EQ(
      result.postSolveLinearizedResiduals.size(),
      expectedPostSolveResiduals.size());

  for (std::size_t constraint = 0; constraint < expectedResiduals.size();
       ++constraint) {
    EXPECT_NEAR(
        result.residuals[constraint], expectedResiduals[constraint], 1e-12)
        << constraint;
    EXPECT_NEAR(
        result.postSolveLinearizedResiduals[constraint],
        expectedPostSolveResiduals[constraint],
        1e-12)
        << constraint;
  }

  for (std::size_t dof = 0; dof < expectedStep.size(); ++dof) {
    EXPECT_NEAR(result.step[dof], expectedStep[dof], 1e-12) << dof;
  }

  for (std::size_t row = 0; row < expectedRows.size(); ++row) {
    EXPECT_EQ(result.rows[row].bodyIndex, expectedRows[row].bodyIndex);
    for (std::size_t dof = 0; dof < cuda::kNewtonAssemblySolveDofsPerBody;
         ++dof) {
      EXPECT_NEAR(
          result.rows[row].hessianDiagonal[dof],
          expectedRows[row].hessianDiagonal[dof],
          1e-12)
          << row << "/" << dof;
      EXPECT_NEAR(
          result.rows[row].gradient[dof],
          expectedRows[row].gradient[dof],
          1e-12)
          << row << "/" << dof;
    }
  }

  for (std::size_t block = 0; block < expectedBlocks.size(); ++block) {
    EXPECT_EQ(
        result.blocks[block].rowBodyIndex, expectedBlocks[block].rowBodyIndex);
    EXPECT_EQ(
        result.blocks[block].columnBodyIndex,
        expectedBlocks[block].columnBodyIndex);
    for (std::size_t entry = 0; entry < cuda::kNewtonAssemblySolveBlockEntries;
         ++entry) {
      EXPECT_NEAR(
          result.blocks[block].hessianBlock[entry],
          expectedBlocks[block].hessianBlock[entry],
          1e-12)
          << block << "/" << entry;
    }
  }
}

//==============================================================================
TEST(NewtonAssemblySolveCuda, MatchesCpuSceneNonlinearEqualityConvergence)
{
  if (!cuda::isCudaRuntimeAvailable()) {
    GTEST_SKIP() << "CUDA runtime has no available device";
  }

  const std::vector nodes = {
      makeSceneNode(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0),
      makeSceneNode(1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.5),
  };
  const std::vector constraints = {
      makeDistanceEqualityConstraint(0, 1, 0.8, 2.0, 0.0),
  };
  constexpr double regularization = 0.1;
  constexpr std::size_t maxIterationCount = 8;
  constexpr double residualTolerance = 1e-3;

  cuda::NewtonSceneNonlinearEqualityConvergenceResult result;
  cuda::evaluateNewtonSceneNonlinearEqualityConvergenceCuda(
      nodes,
      constraints,
      regularization,
      maxIterationCount,
      residualTolerance,
      result);

  const auto expected = evaluateExpectedSceneNonlinearEqualityConvergence(
      nodes, constraints, regularization, maxIterationCount, residualTolerance);

  EXPECT_EQ(result.nodeCount, nodes.size());
  EXPECT_EQ(result.constraintCount, constraints.size());
  EXPECT_EQ(result.rowCount, expected.rows.size());
  EXPECT_EQ(result.bodyCount, nodes.size());
  EXPECT_EQ(
      result.dofCount, nodes.size() * cuda::kNewtonAssemblySolveDofsPerBody);
  EXPECT_EQ(result.blockCount, expected.blocks.size());
  EXPECT_EQ(
      result.blockEntryCount,
      expected.blocks.size() * cuda::kNewtonAssemblySolveBlockEntries);
  EXPECT_EQ(result.maxIterationCount, maxIterationCount);
  EXPECT_EQ(result.completedIterationCount, expected.completedIterationCount);
  EXPECT_EQ(result.activeDofCount, expected.activeDofCount);
  EXPECT_EQ(result.regularization, regularization);
  EXPECT_EQ(result.residualTolerance, residualTolerance);
  EXPECT_EQ(result.converged, expected.converged);
  EXPECT_GT(result.completedIterationCount, 0u);
  EXPECT_LT(
      result.finalMaxConstraintResidualAbs,
      result.initialMaxConstraintResidualAbs);
  EXPECT_NEAR(
      result.initialMaxConstraintResidualAbs,
      expected.initialMaxConstraintResidualAbs,
      1e-12);
  EXPECT_NEAR(
      result.finalMaxConstraintResidualAbs,
      expected.finalMaxConstraintResidualAbs,
      1e-12);
  EXPECT_NEAR(result.maxDiagonal, expected.maxDiagonal, 1e-12);
  EXPECT_NEAR(result.maxGradientAbs, expected.maxGradientAbs, 1e-12);
  EXPECT_NEAR(result.maxBlockAbs, expected.maxBlockAbs, 1e-12);
  EXPECT_NEAR(result.stepNorm, expected.stepNorm, 1e-12);

  ASSERT_EQ(result.initialResiduals.size(), expected.initialResiduals.size());
  ASSERT_EQ(result.finalResiduals.size(), expected.finalResiduals.size());
  ASSERT_EQ(result.step.size(), expected.step.size());
  ASSERT_EQ(result.rows.size(), expected.rows.size());
  ASSERT_EQ(result.blocks.size(), expected.blocks.size());

  for (std::size_t constraint = 0;
       constraint < expected.initialResiduals.size();
       ++constraint) {
    EXPECT_NEAR(
        result.initialResiduals[constraint],
        expected.initialResiduals[constraint],
        1e-12)
        << constraint;
    EXPECT_NEAR(
        result.finalResiduals[constraint],
        expected.finalResiduals[constraint],
        1e-12)
        << constraint;
  }

  for (std::size_t dof = 0; dof < expected.step.size(); ++dof) {
    EXPECT_NEAR(result.step[dof], expected.step[dof], 1e-12) << dof;
  }

  for (std::size_t row = 0; row < expected.rows.size(); ++row) {
    EXPECT_EQ(result.rows[row].bodyIndex, expected.rows[row].bodyIndex);
    for (std::size_t dof = 0; dof < cuda::kNewtonAssemblySolveDofsPerBody;
         ++dof) {
      EXPECT_NEAR(
          result.rows[row].hessianDiagonal[dof],
          expected.rows[row].hessianDiagonal[dof],
          1e-12)
          << row << "/" << dof;
      EXPECT_NEAR(
          result.rows[row].gradient[dof],
          expected.rows[row].gradient[dof],
          1e-12)
          << row << "/" << dof;
    }
  }

  for (std::size_t block = 0; block < expected.blocks.size(); ++block) {
    EXPECT_EQ(
        result.blocks[block].rowBodyIndex, expected.blocks[block].rowBodyIndex);
    EXPECT_EQ(
        result.blocks[block].columnBodyIndex,
        expected.blocks[block].columnBodyIndex);
    for (std::size_t entry = 0; entry < cuda::kNewtonAssemblySolveBlockEntries;
         ++entry) {
      EXPECT_NEAR(
          result.blocks[block].hessianBlock[entry],
          expected.blocks[block].hessianBlock[entry],
          1e-12)
          << block << "/" << entry;
    }
  }
}

//==============================================================================
TEST(NewtonAssemblySolveCuda, RejectsInvalidSceneNonlinearEqualityInputs)
{
  if (!cuda::isCudaRuntimeAvailable()) {
    GTEST_SKIP() << "CUDA runtime has no available device";
  }

  std::vector nodes = {
      makeSceneNode(0.0, 0.0, 0.0, 0.10, -0.20, 0.30, 1.0),
      makeSceneNode(1.0, 0.2, 0.1, -0.10, 0.40, -0.15, 1.5),
  };
  cuda::NewtonSceneNonlinearEqualityAssemblyResult result;
  cuda::NewtonSceneNonlinearEqualitySolveResult solveResult;
  cuda::NewtonSceneNonlinearEqualityConvergenceResult convergenceResult;

  EXPECT_THROW(
      cuda::evaluateNewtonSceneNonlinearEqualityAssemblyCuda(
          nodes, {makeDistanceEqualityConstraint(0, 2, 1.0, 2.0, 0.1)}, result),
      dart::simulation::InvalidArgumentException);
  EXPECT_THROW(
      cuda::evaluateNewtonSceneNonlinearEqualityAssemblyCuda(
          nodes, {makeDistanceEqualityConstraint(0, 0, 1.0, 2.0, 0.1)}, result),
      dart::simulation::InvalidArgumentException);
  EXPECT_THROW(
      cuda::evaluateNewtonSceneNonlinearEqualityAssemblyCuda(
          nodes,
          {makeDistanceEqualityConstraint(0, 1, -1.0, 2.0, 0.1)},
          result),
      dart::simulation::InvalidArgumentException);
  EXPECT_THROW(
      cuda::evaluateNewtonSceneNonlinearEqualityAssemblyCuda(
          nodes, {makeDistanceEqualityConstraint(0, 1, 1.0, 0.0, 0.1)}, result),
      dart::simulation::InvalidArgumentException);

  EXPECT_THROW(
      cuda::evaluateNewtonSceneNonlinearEqualitySolveCuda(
          nodes,
          {makeDistanceEqualityConstraint(0, 1, 1.0, 2.0, 0.1)},
          -0.1,
          solveResult),
      dart::simulation::InvalidArgumentException);
  EXPECT_THROW(
      cuda::evaluateNewtonSceneNonlinearEqualityConvergenceCuda(
          nodes,
          {makeDistanceEqualityConstraint(0, 1, 1.0, 2.0, 0.1)},
          0.1,
          0,
          1e-6,
          convergenceResult),
      dart::simulation::InvalidArgumentException);
  EXPECT_THROW(
      cuda::evaluateNewtonSceneNonlinearEqualityConvergenceCuda(
          nodes,
          {makeDistanceEqualityConstraint(0, 1, 1.0, 2.0, 0.1)},
          0.1,
          4,
          -1e-6,
          convergenceResult),
      dart::simulation::InvalidArgumentException);

  nodes.front().velocity[0] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(
      cuda::evaluateNewtonSceneNonlinearEqualityAssemblyCuda(
          nodes, {makeDistanceEqualityConstraint(0, 1, 1.0, 2.0, 0.1)}, result),
      dart::simulation::InvalidArgumentException);
}

//==============================================================================
TEST(NewtonAssemblySolveCuda, MatchesCpuSparseResidual)
{
  if (!cuda::isCudaRuntimeAvailable()) {
    GTEST_SKIP() << "CUDA runtime has no available device";
  }

  const std::vector rows = {
      makeRow(0, 2.0, -0.5),
      makeRow(1, 3.0, 0.25),
      makeRow(0, 1.5, 0.75),
  };
  constexpr std::size_t bodyCount = 2;
  constexpr double regularization = 0.5;
  const std::vector blocks = {
      makeSparseBlock(0, 1, 0.125),
      makeSparseBlock(0, 1, -0.04),
  };
  const std::vector<double> step = {
      0.1,
      -0.2,
      0.3,
      -0.4,
      0.5,
      -0.6,
      0.7,
      -0.8,
      0.9,
      -1.0,
      1.1,
      -1.2,
  };

  cuda::NewtonSparseResidualResult result;
  cuda::evaluateNewtonSparseResidualCuda(
      rows, bodyCount, blocks, step, regularization, result);

  std::vector<double> expectedDiagonal;
  std::vector<double> expectedGradient;
  std::vector<double> expectedResidual;
  evaluateExpectedSparseResidual(
      rows,
      bodyCount,
      blocks,
      step,
      regularization,
      expectedDiagonal,
      expectedGradient,
      expectedResidual);

  ASSERT_EQ(result.assembledDiagonal.size(), expectedDiagonal.size());
  ASSERT_EQ(result.assembledGradient.size(), expectedGradient.size());
  ASSERT_EQ(result.residual.size(), expectedResidual.size());
  EXPECT_EQ(result.bodyCount, bodyCount);
  EXPECT_EQ(result.rowCount, rows.size());
  EXPECT_EQ(result.dofCount, expectedResidual.size());
  EXPECT_EQ(result.blockCount, blocks.size());
  EXPECT_EQ(result.activeDofCount, expectedResidual.size());

  for (std::size_t dof = 0; dof < expectedResidual.size(); ++dof) {
    EXPECT_NEAR(result.assembledDiagonal[dof], expectedDiagonal[dof], 1e-12)
        << dof;
    EXPECT_NEAR(result.assembledGradient[dof], expectedGradient[dof], 1e-12)
        << dof;
    EXPECT_NEAR(result.residual[dof], expectedResidual[dof], 1e-12) << dof;
  }
}

//==============================================================================
TEST(NewtonAssemblySolveCuda, RejectsInvalidSparseResidualInputs)
{
  if (!cuda::isCudaRuntimeAvailable()) {
    GTEST_SKIP() << "CUDA runtime has no available device";
  }

  const std::vector rows = {makeRow(0, 1.0, 0.0)};
  constexpr std::size_t bodyCount = 2;
  constexpr double regularization = 0.1;
  const std::vector<double> step(
      bodyCount * cuda::kNewtonAssemblySolveDofsPerBody, 0.25);
  cuda::NewtonSparseResidualResult result;

  std::vector blocks = {makeSparseBlock(2, 1, 0.125)};
  EXPECT_THROW(
      cuda::evaluateNewtonSparseResidualCuda(
          rows, bodyCount, blocks, step, regularization, result),
      dart::simulation::InvalidArgumentException);

  blocks = {makeSparseBlock(0, 2, 0.125)};
  EXPECT_THROW(
      cuda::evaluateNewtonSparseResidualCuda(
          rows, bodyCount, blocks, step, regularization, result),
      dart::simulation::InvalidArgumentException);

  blocks = {makeSparseBlock(0, 0, 0.125)};
  EXPECT_THROW(
      cuda::evaluateNewtonSparseResidualCuda(
          rows, bodyCount, blocks, step, regularization, result),
      dart::simulation::InvalidArgumentException);

  blocks = {makeSparseBlock(0, 1, 0.125)};
  blocks.front().hessianBlock[0] = std::numeric_limits<double>::infinity();
  EXPECT_THROW(
      cuda::evaluateNewtonSparseResidualCuda(
          rows, bodyCount, blocks, step, regularization, result),
      dart::simulation::InvalidArgumentException);

  blocks = {makeSparseBlock(0, 1, 0.125)};
  std::vector<double> shortStep = {0.0};
  EXPECT_THROW(
      cuda::evaluateNewtonSparseResidualCuda(
          rows, bodyCount, blocks, shortStep, regularization, result),
      dart::simulation::InvalidArgumentException);

  auto invalidStep = step;
  invalidStep.front() = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(
      cuda::evaluateNewtonSparseResidualCuda(
          rows, bodyCount, blocks, invalidStep, regularization, result),
      dart::simulation::InvalidArgumentException);
}

//==============================================================================
TEST(NewtonAssemblySolveCuda, MatchesCpuSparseJacobiSolve)
{
  if (!cuda::isCudaRuntimeAvailable()) {
    GTEST_SKIP() << "CUDA runtime has no available device";
  }

  const std::vector rows = {
      makeRow(0, 4.0, -0.5),
      makeRow(1, 5.0, 0.25),
      makeRow(0, 2.5, 0.75),
  };
  constexpr std::size_t bodyCount = 2;
  constexpr std::size_t iterationCount = 12;
  constexpr double regularization = 0.75;
  const std::vector blocks = {
      makeSparseBlock(0, 1, 0.025),
      makeSparseBlock(0, 1, -0.01),
  };

  cuda::NewtonSparseJacobiSolveResult result;
  cuda::evaluateNewtonSparseJacobiSolveCuda(
      rows, bodyCount, blocks, iterationCount, regularization, result);

  std::vector<double> expectedDiagonal;
  std::vector<double> expectedGradient;
  std::vector<double> expectedStep;
  std::vector<double> expectedResidual;
  evaluateExpectedSparseJacobiSolve(
      rows,
      bodyCount,
      blocks,
      iterationCount,
      regularization,
      expectedDiagonal,
      expectedGradient,
      expectedStep,
      expectedResidual);

  ASSERT_EQ(result.assembledDiagonal.size(), expectedDiagonal.size());
  ASSERT_EQ(result.assembledGradient.size(), expectedGradient.size());
  ASSERT_EQ(result.step.size(), expectedStep.size());
  ASSERT_EQ(result.residual.size(), expectedResidual.size());
  EXPECT_EQ(result.bodyCount, bodyCount);
  EXPECT_EQ(result.rowCount, rows.size());
  EXPECT_EQ(result.dofCount, expectedStep.size());
  EXPECT_EQ(result.blockCount, blocks.size());
  EXPECT_EQ(result.iterationCount, iterationCount);
  EXPECT_EQ(result.activeDofCount, expectedStep.size());
  EXPECT_LT(result.residualNorm, 1e-10);

  for (std::size_t dof = 0; dof < expectedStep.size(); ++dof) {
    EXPECT_NEAR(result.assembledDiagonal[dof], expectedDiagonal[dof], 1e-12)
        << dof;
    EXPECT_NEAR(result.assembledGradient[dof], expectedGradient[dof], 1e-12)
        << dof;
    EXPECT_NEAR(result.step[dof], expectedStep[dof], 1e-12) << dof;
    EXPECT_NEAR(result.residual[dof], expectedResidual[dof], 1e-12) << dof;
  }
}

//==============================================================================
TEST(NewtonAssemblySolveCuda, RejectsInvalidSparseJacobiSolveInputs)
{
  if (!cuda::isCudaRuntimeAvailable()) {
    GTEST_SKIP() << "CUDA runtime has no available device";
  }

  const std::vector rows = {makeRow(0, 1.0, 0.0)};
  constexpr std::size_t bodyCount = 2;
  constexpr double regularization = 0.1;
  const std::vector blocks = {makeSparseBlock(0, 1, 0.125)};
  cuda::NewtonSparseJacobiSolveResult result;

  EXPECT_THROW(
      cuda::evaluateNewtonSparseJacobiSolveCuda(
          rows, bodyCount, blocks, 0, regularization, result),
      dart::simulation::InvalidArgumentException);
}

//==============================================================================
TEST(NewtonAssemblySolveCuda, MatchesCpuSparseCgSolve)
{
  if (!cuda::isCudaRuntimeAvailable()) {
    GTEST_SKIP() << "CUDA runtime has no available device";
  }

  const std::vector rows = {
      makeRow(0, 4.0, -0.5),
      makeRow(1, 5.0, 0.25),
      makeRow(0, 2.5, 0.75),
  };
  constexpr std::size_t bodyCount = 2;
  constexpr std::size_t maxIterationCount = 16;
  constexpr double residualTolerance = 1e-12;
  constexpr double regularization = 0.75;
  const std::vector blocks = {
      makeSparseBlock(0, 1, 0.025),
      makeSparseBlock(0, 1, -0.01),
  };

  cuda::NewtonSparseCgSolveResult result;
  cuda::evaluateNewtonSparseCgSolveCuda(
      rows,
      bodyCount,
      blocks,
      maxIterationCount,
      residualTolerance,
      regularization,
      result);

  const ExpectedSparseCgSolve expected = evaluateExpectedSparseCgSolve(
      rows,
      bodyCount,
      blocks,
      maxIterationCount,
      residualTolerance,
      regularization);

  ASSERT_EQ(result.assembledDiagonal.size(), expected.assembledDiagonal.size());
  ASSERT_EQ(result.assembledGradient.size(), expected.assembledGradient.size());
  ASSERT_EQ(result.step.size(), expected.step.size());
  ASSERT_EQ(result.residual.size(), expected.residual.size());
  EXPECT_EQ(result.bodyCount, bodyCount);
  EXPECT_EQ(result.rowCount, rows.size());
  EXPECT_EQ(result.dofCount, expected.step.size());
  EXPECT_EQ(result.blockCount, blocks.size());
  EXPECT_EQ(result.maxIterationCount, maxIterationCount);
  EXPECT_EQ(result.completedIterationCount, expected.completedIterationCount);
  EXPECT_EQ(result.activeDofCount, expected.step.size());
  EXPECT_DOUBLE_EQ(result.residualTolerance, residualTolerance);
  EXPECT_EQ(result.converged, expected.converged);
  EXPECT_NEAR(result.initialResidualNorm, expected.initialResidualNorm, 1e-12);
  EXPECT_NEAR(result.stepNorm, expected.stepNorm, 1e-10);
  EXPECT_NEAR(result.residualNorm, expected.residualNorm, 1e-10);
  EXPECT_NEAR(result.maxResidualAbs, expected.maxResidualAbs, 1e-10);
  EXPECT_LT(result.residualNorm, 1e-10);

  for (std::size_t dof = 0; dof < expected.step.size(); ++dof) {
    EXPECT_NEAR(
        result.assembledDiagonal[dof], expected.assembledDiagonal[dof], 1e-12)
        << dof;
    EXPECT_NEAR(
        result.assembledGradient[dof], expected.assembledGradient[dof], 1e-12)
        << dof;
    EXPECT_NEAR(result.step[dof], expected.step[dof], 1e-10) << dof;
    EXPECT_NEAR(result.residual[dof], expected.residual[dof], 1e-10) << dof;
  }
}

//==============================================================================
TEST(NewtonAssemblySolveCuda, RejectsInvalidSparseCgSolveInputs)
{
  if (!cuda::isCudaRuntimeAvailable()) {
    GTEST_SKIP() << "CUDA runtime has no available device";
  }

  const std::vector rows = {makeRow(0, 1.0, 0.0)};
  constexpr std::size_t bodyCount = 2;
  constexpr double regularization = 0.1;
  constexpr double residualTolerance = 1e-12;
  const std::vector blocks = {makeSparseBlock(0, 1, 0.125)};
  cuda::NewtonSparseCgSolveResult result;

  EXPECT_THROW(
      cuda::evaluateNewtonSparseCgSolveCuda(
          rows,
          bodyCount,
          blocks,
          0,
          residualTolerance,
          regularization,
          result),
      dart::simulation::InvalidArgumentException);

  EXPECT_THROW(
      cuda::evaluateNewtonSparseCgSolveCuda(
          rows, bodyCount, blocks, 8, -1.0, regularization, result),
      dart::simulation::InvalidArgumentException);

  EXPECT_THROW(
      cuda::evaluateNewtonSparseCgSolveCuda(
          rows,
          bodyCount,
          blocks,
          8,
          std::numeric_limits<double>::quiet_NaN(),
          regularization,
          result),
      dart::simulation::InvalidArgumentException);
}

//==============================================================================
TEST(NewtonAssemblySolveCuda, MatchesCpuDirectSparseSolve)
{
  if (!cuda::isCudaRuntimeAvailable()) {
    GTEST_SKIP() << "CUDA runtime has no available device";
  }

  const std::vector rows = {
      makeRow(0, 6.0, -0.5),
      makeRow(1, 7.0, 0.25),
      makeRow(0, 2.5, 0.75),
  };
  constexpr std::size_t bodyCount = 2;
  constexpr double regularization = 1.0;
  const std::vector blocks = {
      makeSparseBlock(0, 1, 0.0125),
      makeSparseBlock(0, 1, -0.004),
  };

  cuda::NewtonDirectSparseSolveResult result;
  cuda::evaluateNewtonDirectSparseSolveCuda(
      rows, bodyCount, blocks, regularization, result);

  const ExpectedDirectSparseSolve expected = evaluateExpectedDirectSparseSolve(
      rows, bodyCount, blocks, regularization);
  ASSERT_TRUE(expected.factorized);

  ASSERT_EQ(result.assembledDiagonal.size(), expected.assembledDiagonal.size());
  ASSERT_EQ(result.assembledGradient.size(), expected.assembledGradient.size());
  ASSERT_EQ(result.factorMatrixLower.size(), expected.factorMatrixLower.size());
  ASSERT_EQ(result.step.size(), expected.step.size());
  ASSERT_EQ(result.residual.size(), expected.residual.size());
  EXPECT_EQ(result.bodyCount, bodyCount);
  EXPECT_EQ(result.rowCount, rows.size());
  EXPECT_EQ(result.dofCount, expected.step.size());
  EXPECT_EQ(result.blockCount, blocks.size());
  EXPECT_EQ(result.activeDofCount, expected.step.size());
  EXPECT_DOUBLE_EQ(result.regularization, regularization);
  EXPECT_NEAR(result.minimumFactorPivot, expected.minimumFactorPivot, 1e-12);
  EXPECT_NEAR(result.stepNorm, expected.stepNorm, 1e-10);
  EXPECT_NEAR(result.residualNorm, expected.residualNorm, 1e-10);
  EXPECT_NEAR(result.maxResidualAbs, expected.maxResidualAbs, 1e-10);
  EXPECT_LT(result.residualNorm, 1e-10);

  for (std::size_t dof = 0; dof < expected.step.size(); ++dof) {
    EXPECT_NEAR(
        result.assembledDiagonal[dof], expected.assembledDiagonal[dof], 1e-12)
        << dof;
    EXPECT_NEAR(
        result.assembledGradient[dof], expected.assembledGradient[dof], 1e-12)
        << dof;
    EXPECT_NEAR(result.step[dof], expected.step[dof], 1e-10) << dof;
    EXPECT_NEAR(result.residual[dof], expected.residual[dof], 1e-10) << dof;
  }

  for (std::size_t entry = 0; entry < expected.factorMatrixLower.size();
       ++entry) {
    EXPECT_NEAR(
        result.factorMatrixLower[entry],
        expected.factorMatrixLower[entry],
        1e-10)
        << entry;
  }
}

//==============================================================================
TEST(NewtonAssemblySolveCuda, RejectsInvalidDirectSparseSolveInputs)
{
  if (!cuda::isCudaRuntimeAvailable()) {
    GTEST_SKIP() << "CUDA runtime has no available device";
  }

  const std::vector rows = {makeRow(0, 1.0, 0.0)};
  const std::vector blocks = {makeSparseBlock(0, 1, 0.125)};
  cuda::NewtonDirectSparseSolveResult result;

  const std::size_t tooManyBodies = cuda::kNewtonDirectSparseSolveMaxDofs
                                        / cuda::kNewtonAssemblySolveDofsPerBody
                                    + 1u;
  EXPECT_THROW(
      cuda::evaluateNewtonDirectSparseSolveCuda(
          rows, tooManyBodies, {}, 0.1, result),
      dart::simulation::InvalidArgumentException);

  EXPECT_THROW(
      cuda::evaluateNewtonDirectSparseSolveCuda(rows, 2, blocks, -0.1, result),
      dart::simulation::InvalidArgumentException);

  auto singularRow = makeRow(0, 0.0, 0.0);
  singularRow.hessianDiagonal[0] = 0.0;
  for (std::size_t dof = 1; dof < cuda::kNewtonAssemblySolveDofsPerBody;
       ++dof) {
    singularRow.hessianDiagonal[dof] = 1.0;
  }
  EXPECT_THROW(
      cuda::evaluateNewtonDirectSparseSolveCuda(
          {singularRow}, 1, {}, 0.0, result),
      dart::simulation::InvalidArgumentException);
}

//==============================================================================
TEST(NewtonAssemblySolveCuda, MatchesCpuEqualityReducedSolve)
{
  if (!cuda::isCudaRuntimeAvailable()) {
    GTEST_SKIP() << "CUDA runtime has no available device";
  }

  const std::vector rows = {
      makeRow(0, 2.0, -0.5),
      makeRow(1, 3.0, 0.25),
      makeRow(0, 1.5, 0.75),
  };
  constexpr std::size_t bodyCount = 2;
  constexpr std::size_t reducedDofCount = 4;
  constexpr double regularization = 0.5;
  const std::vector<cuda::NewtonEqualityReductionEntry> entries = {
      {0, 0, 1.0},
      {1, 1, 1.0},
      {6, 1, 0.5},
      {8, 2, -1.25},
      {11, 3, 0.75},
  };

  cuda::NewtonEqualityReducedSolveResult result;
  cuda::evaluateNewtonEqualityReducedSolveCuda(
      rows, bodyCount, entries, reducedDofCount, regularization, result);

  std::vector<double> expectedFullDiagonal;
  std::vector<double> expectedFullGradient;
  std::vector<double> expectedReducedDiagonal;
  std::vector<double> expectedReducedGradient;
  std::vector<double> expectedReducedStep;
  std::vector<double> expectedFullStep;
  evaluateExpectedEqualityReduced(
      rows,
      bodyCount,
      entries,
      reducedDofCount,
      regularization,
      expectedFullDiagonal,
      expectedFullGradient,
      expectedReducedDiagonal,
      expectedReducedGradient,
      expectedReducedStep,
      expectedFullStep);

  ASSERT_EQ(result.assembledDiagonal.size(), expectedFullDiagonal.size());
  ASSERT_EQ(result.assembledGradient.size(), expectedFullGradient.size());
  ASSERT_EQ(result.reducedDiagonal.size(), expectedReducedDiagonal.size());
  ASSERT_EQ(result.reducedGradient.size(), expectedReducedGradient.size());
  ASSERT_EQ(result.reducedStep.size(), expectedReducedStep.size());
  ASSERT_EQ(result.fullStep.size(), expectedFullStep.size());
  EXPECT_EQ(result.bodyCount, bodyCount);
  EXPECT_EQ(result.rowCount, rows.size());
  EXPECT_EQ(result.fullDofCount, expectedFullDiagonal.size());
  EXPECT_EQ(result.reductionEntryCount, entries.size());
  EXPECT_EQ(result.reducedDofCount, reducedDofCount);
  EXPECT_EQ(result.activeReducedDofCount, reducedDofCount);
  EXPECT_LT(result.residualNorm, 1e-12);

  for (std::size_t index = 0; index < expectedFullDiagonal.size(); ++index) {
    EXPECT_NEAR(
        result.assembledDiagonal[index], expectedFullDiagonal[index], 1e-12)
        << index;
    EXPECT_NEAR(
        result.assembledGradient[index], expectedFullGradient[index], 1e-12)
        << index;
    EXPECT_NEAR(result.fullStep[index], expectedFullStep[index], 1e-12)
        << index;
  }
  for (std::size_t index = 0; index < reducedDofCount; ++index) {
    EXPECT_NEAR(
        result.reducedDiagonal[index], expectedReducedDiagonal[index], 1e-12)
        << index;
    EXPECT_NEAR(
        result.reducedGradient[index], expectedReducedGradient[index], 1e-12)
        << index;
    EXPECT_NEAR(result.reducedStep[index], expectedReducedStep[index], 1e-12)
        << index;
    EXPECT_NEAR(result.reducedResidual[index], 0.0, 1e-12) << index;
  }
}

//==============================================================================
TEST(NewtonAssemblySolveCuda, RejectsInvalidEqualityReductionEntries)
{
  if (!cuda::isCudaRuntimeAvailable()) {
    GTEST_SKIP() << "CUDA runtime has no available device";
  }

  const std::vector rows = {makeRow(0, 1.0, 0.0)};
  cuda::NewtonEqualityReducedSolveResult result;
  std::vector<cuda::NewtonEqualityReductionEntry> entries = {{6, 0, 1.0}};
  EXPECT_THROW(
      cuda::evaluateNewtonEqualityReducedSolveCuda(
          rows, 1, entries, 1, 0.1, result),
      dart::simulation::InvalidArgumentException);

  entries = {{0, 1, 1.0}};
  EXPECT_THROW(
      cuda::evaluateNewtonEqualityReducedSolveCuda(
          rows, 1, entries, 1, 0.1, result),
      dart::simulation::InvalidArgumentException);

  entries = {{0, 0, std::numeric_limits<double>::quiet_NaN()}};
  EXPECT_THROW(
      cuda::evaluateNewtonEqualityReducedSolveCuda(
          rows, 1, entries, 1, 0.1, result),
      dart::simulation::InvalidArgumentException);
}
