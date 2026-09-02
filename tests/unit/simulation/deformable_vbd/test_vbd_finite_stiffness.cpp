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
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS
 *   BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY,
 *   OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 *   PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 *   OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *   THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *   (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
 *   USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
 *   DAMAGE.
 */

#include <dart/simulation/compute/parallel_executor.hpp>
#include <dart/simulation/detail/deformable_vbd/avbd_row_inventory.hpp>
#include <dart/simulation/detail/deformable_vbd/block_descent.hpp>
#include <dart/simulation/detail/deformable_vbd/finite_stiffness_kernel.hpp>

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <limits>
#include <vector>

#include <cstdint>

namespace vbd = dart::simulation::detail::deformable_vbd;
namespace compute = dart::simulation::compute;

namespace {

using Vec3 = Eigen::Vector3d;

//==============================================================================
vbd::AvbdSpringFiniteStiffnessRow makeSpringRow(
    double stiffness, double materialStiffness)
{
  vbd::AvbdSpringFiniteStiffnessRow row;
  row.spring = 0;
  row.state.stiffness = stiffness;
  row.materialStiffness = materialStiffness;
  return row;
}

//==============================================================================
vbd::AvbdScalarRowDescriptor makeSpringDescriptor()
{
  vbd::AvbdScalarRowDescriptor descriptor;
  descriptor.key.role = vbd::AvbdScalarRowRole::DeformableSpring;
  descriptor.key.objectA = 17;
  descriptor.key.featureA = 3;
  descriptor.kind = vbd::AvbdScalarRowKind::FiniteStiffness;
  descriptor.startStiffness = 4.0;
  descriptor.materialStiffness = 100.0;
  return descriptor;
}

} // namespace

//==============================================================================
TEST(VbdFiniteStiffness, SpringConstraintValueIsStretch)
{
  EXPECT_DOUBLE_EQ(
      vbd::avbdSpringConstraintValue(Vec3::Zero(), Vec3(1.25, 0.0, 0.0), 1.0),
      0.25);
  EXPECT_DOUBLE_EQ(
      vbd::avbdSpringConstraintValue(Vec3::Zero(), Vec3(0.75, 0.0, 0.0), 1.0),
      -0.25);
}

//==============================================================================
TEST(VbdFiniteStiffness, CompressedSpringUsesQuasiNewtonColumnNorms)
{
  const Vec3 self = Vec3::Zero();
  const Vec3 other(0.3, 0.4, 0.0);
  const vbd::AvbdSpringFiniteStiffnessRow row
      = makeSpringRow(/*stiffness=*/8.0, /*materialStiffness=*/20.0);

  vbd::VertexBlock block;
  vbd::addAvbdSpringFiniteStiffness(
      block, self, other, /*restLength=*/1.0, row);

  const Vec3 direction(0.6, 0.8, 0.0);
  const Vec3 expectedForce = -4.0 * direction;
  Eigen::Matrix3d expectedHessian = 8.0 * direction * direction.transpose();
  expectedHessian.diagonal() += Vec3(6.4, 4.8, 8.0);

  EXPECT_NEAR((block.force - expectedForce).norm(), 0.0, 1e-12);
  EXPECT_NEAR((block.hessian - expectedHessian).norm(), 0.0, 1e-12);
  EXPECT_GT(block.hessian.determinant(), 0.0);
}

//==============================================================================
TEST(VbdFiniteStiffness, ProjectedQuasiNewtonDiagonalPropagatesNonfiniteInputs)
{
  const double nan = std::numeric_limits<double>::quiet_NaN();

  const Vec3 nonfiniteDirection = vbd::avbdQuasiNewtonProjectedDistanceDiagonal(
      Vec3(nan, 0.0, 1.0), /*scale=*/2.0);
  const Vec3 nonfiniteScale
      = vbd::avbdQuasiNewtonProjectedDistanceDiagonal(Vec3::UnitX(), nan);

  EXPECT_FALSE(nonfiniteDirection.allFinite());
  EXPECT_FALSE(nonfiniteScale.allFinite());
}

//==============================================================================
TEST(VbdFiniteStiffness, RowUpdateRampsAndCapsAtMaterialStiffness)
{
  vbd::AvbdSpringFiniteStiffnessRow row = makeSpringRow(5.0, 20.0);
  vbd::AvbdSpringFiniteStiffnessOptions options;
  options.beta = 12.0;

  vbd::AvbdScalarRowState updated
      = vbd::updateAvbdSpringFiniteStiffnessRow(row.state, -0.25, row, options);
  EXPECT_DOUBLE_EQ(updated.lambda, 0.0);
  EXPECT_DOUBLE_EQ(updated.stiffness, 8.0);

  row.state.stiffness = 19.0;
  updated
      = vbd::updateAvbdSpringFiniteStiffnessRow(row.state, 0.25, row, options);
  EXPECT_DOUBLE_EQ(updated.stiffness, 20.0);
}

//==============================================================================
TEST(VbdFiniteStiffness, InventoryWarmStartsFiniteRowsWithoutLambda)
{
  vbd::AvbdScalarRowDescriptor descriptor = makeSpringDescriptor();
  std::vector<vbd::AvbdScalarRowDescriptor> descriptors = {descriptor};
  vbd::AvbdScalarRowInventory inventory;
  inventory.syncActiveRows(descriptors, {});
  ASSERT_EQ(inventory.size(), 1u);
  inventory[0].state.lambda = 30.0;
  inventory[0].state.stiffness = 80.0;

  vbd::AvbdRowWarmStartOptions options;
  options.gamma = 0.5;
  inventory.syncActiveRows(descriptors, options);

  ASSERT_EQ(inventory.size(), 1u);
  EXPECT_DOUBLE_EQ(inventory[0].state.lambda, 0.0);
  EXPECT_DOUBLE_EQ(inventory[0].state.stiffness, 40.0);
}

//==============================================================================
TEST(VbdFiniteStiffness, BlockDescentRowsUpdateEffectiveSpringStiffness)
{
  std::vector<Vec3> positions = {Vec3(0.0, 0.0, 0.0), Vec3(1.4, 0.0, 0.0)};
  const std::vector<double> masses = {1.0, 1.0};
  const std::vector<std::uint8_t> fixed = {1u, 0u};
  const std::vector<Vec3> inertialTargets = positions;
  const std::vector<vbd::SpringElement> springs = {{0, 1, 1.0}};
  const auto coloring = vbd::colorSprings(positions.size(), springs);
  const auto adjacency = vbd::SpringAdjacency::build(positions.size(), springs);

  std::vector<vbd::AvbdSpringFiniteStiffnessRow> rows
      = {makeSpringRow(2.0, 100.0)};

  vbd::BlockDescentOptions options;
  options.iterations = 4;
  vbd::AvbdSpringFiniteStiffnessOptions avbdOptions;
  avbdOptions.beta = 50.0;

  const vbd::BlockDescentStats stats
      = vbd::blockDescentMassSpringAvbdFiniteStiffness(
          positions,
          masses,
          fixed,
          inertialTargets,
          springs,
          100.0,
          0.1,
          rows,
          coloring,
          adjacency,
          options,
          avbdOptions);

  EXPECT_EQ(stats.iterations, 4u);
  EXPECT_LT(positions[1].x(), 1.4);
  EXPECT_GT(rows[0].state.stiffness, 2.0);
  EXPECT_LE(rows[0].state.stiffness, rows[0].materialStiffness);
  EXPECT_DOUBLE_EQ(rows[0].state.lambda, 0.0);
}

//==============================================================================
TEST(VbdFiniteStiffness, SelfContactDualUpdatesAreDeterministicAcrossWorkers)
{
  constexpr std::size_t rowCount = 8193u;
  constexpr std::size_t frictionPairCount = 8193u;

  struct SolveResult
  {
    std::vector<Vec3> positions;
    std::vector<vbd::AvbdSelfContactNormalRow> selfContactRows;
    std::vector<vbd::AvbdSelfContactFrictionRow> frictionRows;
    vbd::BlockDescentStats stats;
  };

  const auto solve = [](std::size_t workerCount) {
    SolveResult result;
    result.positions
        = {Vec3(0.3, 0.3, 0.01),
           Vec3(0.0, 0.0, 0.0),
           Vec3(1.0, 0.0, 0.0),
           Vec3(0.0, 1.0, 1.0)};
    const std::vector<Vec3> restPositions
        = {Vec3(0.3, 0.3, 0.02),
           Vec3(0.0, 0.0, 0.0),
           Vec3(1.0, 0.0, 0.0),
           Vec3(0.0, 1.0, 1.0)};
    const std::array<Vec3, 4> stepStart{
        result.positions[0],
        result.positions[1],
        result.positions[2],
        result.positions[3]};
    const std::vector<double> masses(result.positions.size(), 1.0);
    const std::vector<std::uint8_t> fixed = {0u, 1u, 1u, 1u};
    std::vector<Vec3> inertialTargets = result.positions;
    inertialTargets[0] += Vec3(0.01, -0.005, 0.002);

    const std::array<std::uint32_t, 4> vertices{0u, 1u, 2u, 3u};
    const std::vector<vbd::TetMeshElement> tets{
        {vertices,
         vbd::makeTetRestShape(
             {restPositions[0],
              restPositions[1],
              restPositions[2],
              restPositions[3]})}};
    const auto coloring = vbd::colorTetMesh(result.positions.size(), tets);
    const auto adjacency
        = vbd::TetAdjacency::build(result.positions.size(), tets);

    result.selfContactRows.reserve(rowCount);
    for (std::size_t i = 0u; i < rowCount; ++i) {
      const double index = static_cast<double>(i);
      vbd::AvbdSelfContactNormalRow row;
      row.nodes = {0u, 1u, 2u, 3u};
      row.state.stiffness = 10.0 + 0.01 * index;
      row.state.lambda = 0.0005 * index;
      row.squaredActivationDistance = 4e-4;
      result.selfContactRows.push_back(row);
    }

    result.frictionRows.reserve(2u * frictionPairCount);
    for (std::size_t pair = 0u; pair < frictionPairCount; ++pair) {
      const double index = static_cast<double>(pair);
      vbd::AvbdSelfContactFrictionRow first;
      first.nodes = {0u, 1u, 2u, 3u};
      first.stepStartPositions = stepStart;
      first.stepStartPositions[0] -= Vec3(0.005, -0.003, 0.0);
      first.axis = 0u;
      first.state.stiffness = 6.0 + 0.01 * index;
      first.state.lambda = 0.001 * index;
      first.bounds = {-15.0, 15.0};
      vbd::AvbdSelfContactFrictionRow second = first;
      second.axis = 1u;
      second.state.lambda = -0.0005 * index;
      result.frictionRows.push_back(first);
      result.frictionRows.push_back(second);
    }
    result.selfContactRows.back().nodes = {99u, 100u, 101u, 102u};
    result.frictionRows[result.frictionRows.size() - 2u].nodes
        = {99u, 100u, 101u, 102u};
    result.frictionRows.back().nodes = {99u, 100u, 101u, 102u};

    vbd::BlockDescentOptions options;
    options.iterations = 2u;
    options.regularization = 1e-12;
    vbd::AvbdSelfContactNormalOptions selfContactOptions;
    selfContactOptions.beta = 3.0;
    selfContactOptions.maxStiffness = 1000.0;
    vbd::AvbdSelfContactFrictionOptions frictionOptions;
    frictionOptions.beta = 4.0;
    frictionOptions.maxStiffness = 1000.0;

    compute::ParallelExecutor executor(std::max<std::size_t>(1u, workerCount));
    result.stats = vbd::blockDescentTetMeshAvbdSelfContact(
        result.positions,
        masses,
        fixed,
        inertialTargets,
        tets,
        /*mu=*/500.0,
        /*lambda=*/800.0,
        /*timeStep=*/0.02,
        coloring,
        adjacency,
        options,
        nullptr,
        &result.selfContactRows,
        &selfContactOptions,
        &result.frictionRows,
        &frictionOptions,
        workerCount == 0u ? nullptr : &executor);
    return result;
  };

  const SolveResult serial = solve(0u);
  const double lastIndex = static_cast<double>(rowCount - 1u);
  EXPECT_EQ(serial.selfContactRows.back().state.lambda, 0.0005 * lastIndex);
  EXPECT_EQ(
      serial.frictionRows[serial.frictionRows.size() - 2u].state.lambda,
      0.001 * lastIndex);
  EXPECT_EQ(serial.frictionRows.back().state.lambda, -0.0005 * lastIndex);
  const auto expectSame = [&](const SolveResult& parallel) {
    ASSERT_EQ(parallel.positions.size(), serial.positions.size());
    for (std::size_t i = 0u; i < serial.positions.size(); ++i) {
      EXPECT_TRUE(
          (parallel.positions[i].array() == serial.positions[i].array()).all())
          << "position=" << i;
    }
    EXPECT_EQ(parallel.stats.iterations, serial.stats.iterations);
    EXPECT_EQ(parallel.stats.vertexUpdates, serial.stats.vertexUpdates);
    EXPECT_EQ(
        parallel.stats.finalResidualNormSquared,
        serial.stats.finalResidualNormSquared);

    const auto expectStates = [](const auto& actual, const auto& expected) {
      ASSERT_EQ(actual.size(), expected.size());
      for (std::size_t i = 0u; i < expected.size(); ++i) {
        EXPECT_EQ(actual[i].state.lambda, expected[i].state.lambda)
            << "row=" << i;
        EXPECT_EQ(actual[i].state.stiffness, expected[i].state.stiffness)
            << "row=" << i;
      }
    };
    expectStates(parallel.selfContactRows, serial.selfContactRows);
    expectStates(parallel.frictionRows, serial.frictionRows);
  };

  expectSame(solve(2u));
  expectSame(solve(4u));
}
