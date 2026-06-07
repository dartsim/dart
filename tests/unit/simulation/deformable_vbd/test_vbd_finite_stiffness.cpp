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
 *     copyright notice and this list of conditions in the documentation
 *     and/or other materials provided with the distribution.
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 *   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 */

#include <dart/simulation/detail/deformable_vbd/avbd_row_inventory.hpp>
#include <dart/simulation/detail/deformable_vbd/block_descent.hpp>
#include <dart/simulation/detail/deformable_vbd/finite_stiffness_kernel.hpp>

#include <gtest/gtest.h>

#include <array>
#include <vector>

namespace vbd = dart::simulation::detail::deformable_vbd;

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
vbd::AvbdTetMaterialFiniteStiffnessRow makeTetRow(
    double stiffness, double materialStiffness)
{
  vbd::AvbdTetMaterialFiniteStiffnessRow row;
  row.tet = 0;
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
TEST(VbdFiniteStiffness, TetMaterialConstraintValueIsStrainNorm)
{
  const std::array<Vec3, 4> rest
      = {Vec3(0.0, 0.0, 0.0),
         Vec3(1.0, 0.0, 0.0),
         Vec3(0.0, 1.0, 0.0),
         Vec3(0.0, 0.0, 1.0)};
  const vbd::TetRestShape restShape = vbd::makeTetRestShape(rest);
  EXPECT_DOUBLE_EQ(vbd::avbdTetMaterialConstraintValue(restShape, rest), 0.0);

  std::array<Vec3, 4> deformed = rest;
  deformed[3].z() = 1.25;
  EXPECT_GT(vbd::avbdTetMaterialConstraintValue(restShape, deformed), 0.0);
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
TEST(VbdFiniteStiffness, TetMaterialRowUpdateRampsAndCapsAtUnitScale)
{
  vbd::AvbdTetMaterialFiniteStiffnessRow row = makeTetRow(0.1, 1.0);
  vbd::AvbdTetMaterialFiniteStiffnessOptions options;
  options.beta = 2.0;

  vbd::AvbdScalarRowState updated
      = vbd::updateAvbdTetMaterialFiniteStiffnessRow(
          row.state, 0.25, row, options);
  EXPECT_DOUBLE_EQ(updated.lambda, 0.0);
  EXPECT_DOUBLE_EQ(updated.stiffness, 0.6);

  row.state.stiffness = 0.9;
  updated = vbd::updateAvbdTetMaterialFiniteStiffnessRow(
      row.state, 0.25, row, options);
  EXPECT_DOUBLE_EQ(updated.stiffness, 1.0);
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
TEST(VbdFiniteStiffness, BlockDescentRowsUpdateEffectiveTetMaterialStiffness)
{
  std::vector<Vec3> positions
      = {Vec3(0.0, 0.0, 0.0),
         Vec3(1.0, 0.0, 0.0),
         Vec3(0.0, 1.0, 0.0),
         Vec3(0.0, 0.0, 1.3)};
  const std::vector<Vec3> restPositions
      = {Vec3(0.0, 0.0, 0.0),
         Vec3(1.0, 0.0, 0.0),
         Vec3(0.0, 1.0, 0.0),
         Vec3(0.0, 0.0, 1.0)};
  const std::vector<double> masses(positions.size(), 1.0);
  const std::vector<std::uint8_t> fixed = {1u, 1u, 1u, 0u};
  const std::vector<Vec3> inertialTargets = positions;
  const std::array<std::uint32_t, 4> vertices = {0, 1, 2, 3};
  const std::vector<vbd::TetMeshElement> tets
      = {{vertices,
          vbd::makeTetRestShape(
              {restPositions[0],
               restPositions[1],
               restPositions[2],
               restPositions[3]})}};
  const auto coloring = vbd::colorTetMesh(positions.size(), tets);
  const auto adjacency = vbd::TetAdjacency::build(positions.size(), tets);

  std::vector<vbd::AvbdTetMaterialFiniteStiffnessRow> rows
      = {makeTetRow(0.05, 1.0)};

  vbd::BlockDescentOptions options;
  options.iterations = 4;
  vbd::AvbdTetMaterialFiniteStiffnessOptions avbdOptions;
  avbdOptions.beta = 1.5;

  const vbd::BlockDescentStats stats
      = vbd::blockDescentTetMeshAvbdFiniteStiffness(
          positions,
          masses,
          fixed,
          inertialTargets,
          tets,
          800.0,
          1200.0,
          0.1,
          rows,
          coloring,
          adjacency,
          options,
          avbdOptions);

  EXPECT_EQ(stats.iterations, 4u);
  EXPECT_LT(positions[3].z(), 1.3);
  EXPECT_GT(rows[0].state.stiffness, 0.05);
  EXPECT_LE(rows[0].state.stiffness, rows[0].materialStiffness);
  EXPECT_DOUBLE_EQ(rows[0].state.lambda, 0.0);
}
