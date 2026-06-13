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
