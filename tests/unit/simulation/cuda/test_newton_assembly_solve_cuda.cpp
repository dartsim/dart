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

#include <limits>
#include <vector>

namespace cuda = dart::simulation::compute::cuda;

namespace {

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
