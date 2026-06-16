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
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS AND
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 */

#include <dart/simulation/detail/deformable_vbd/avbd_constraint.hpp>
#include <dart/simulation/detail/deformable_vbd/avbd_row_inventory.hpp>

#include <gtest/gtest.h>

#include <vector>

namespace vbd = dart::simulation::detail::deformable_vbd;

namespace {

//==============================================================================
vbd::AvbdScalarRowDescriptor makeRow(
    vbd::AvbdScalarRowRole role, std::uint64_t feature, std::uint8_t axis = 0)
{
  vbd::AvbdScalarRowDescriptor descriptor;
  descriptor.key.role = role;
  descriptor.key.objectA = 42;
  descriptor.key.featureA = feature;
  descriptor.key.axis = axis;
  descriptor.startStiffness = 4.0;
  return descriptor;
}

} // namespace

//==============================================================================
TEST(AvbdConstraint, RegularizesPreExistingConstraintError)
{
  EXPECT_NEAR(
      vbd::regularizeAvbdConstraintValue(0.30, 0.20, 0.95), 0.11, 1e-15);
  EXPECT_DOUBLE_EQ(vbd::regularizeAvbdConstraintValue(0.30, 0.20, 0.0), 0.30);
  EXPECT_NEAR(vbd::regularizeAvbdConstraintValue(0.30, 0.20, 1.0), 0.10, 1e-15);
}

//==============================================================================
TEST(AvbdConstraint, WarmStartDecaysLambdaAndKeepsMinimumStiffness)
{
  vbd::AvbdScalarRowState previous;
  previous.stiffness = 40.0;
  previous.lambda = 10.0;

  const vbd::AvbdScalarRowState warm
      = vbd::warmStartAvbdHardConstraint(previous, 5.0, 0.95, 0.5);

  EXPECT_DOUBLE_EQ(warm.lambda, 4.75);
  EXPECT_DOUBLE_EQ(warm.stiffness, 20.0);

  previous.stiffness = 6.0;
  const vbd::AvbdScalarRowState clamped
      = vbd::warmStartAvbdHardConstraint(previous, 5.0, 0.95, 0.5);
  EXPECT_DOUBLE_EQ(clamped.stiffness, 5.0);
}

//==============================================================================
TEST(AvbdConstraint, HardConstraintDualUpdateGrowsStiffnessInsideBounds)
{
  vbd::AvbdScalarRowState row;
  row.stiffness = 2.0;
  row.lambda = 0.25;

  const vbd::AvbdScalarRowState updated
      = vbd::updateAvbdHardConstraintRow(row, 0.5, 10.0);

  EXPECT_DOUBLE_EQ(updated.lambda, 1.25);
  EXPECT_DOUBLE_EQ(updated.stiffness, 7.0);
}

//==============================================================================
TEST(AvbdConstraint, HardConstraintDualUpdateDoesNotGrowWhenClamped)
{
  vbd::AvbdScalarRowState row;
  row.stiffness = 2.0;
  row.lambda = 0.25;

  vbd::AvbdScalarRowBounds bounds;
  bounds.lower = 0.0;
  bounds.upper = 1.0;

  const vbd::AvbdScalarRowState upperClamped
      = vbd::updateAvbdHardConstraintRow(row, 0.5, 10.0, bounds);
  EXPECT_DOUBLE_EQ(upperClamped.lambda, 1.0);
  EXPECT_DOUBLE_EQ(upperClamped.stiffness, 2.0);

  const vbd::AvbdScalarRowState lowerClamped
      = vbd::updateAvbdHardConstraintRow(row, -1.0, 10.0, bounds);
  EXPECT_DOUBLE_EQ(lowerClamped.lambda, 0.0);
  EXPECT_DOUBLE_EQ(lowerClamped.stiffness, 2.0);
}

//==============================================================================
TEST(AvbdConstraint, FiniteStiffnessRampSaturatesAtMaterialStiffness)
{
  EXPECT_DOUBLE_EQ(vbd::updateAvbdFiniteStiffness(5.0, -0.25, 12.0, 20.0), 8.0);
  EXPECT_DOUBLE_EQ(
      vbd::updateAvbdFiniteStiffness(19.0, 0.25, 12.0, 20.0), 20.0);
}

//==============================================================================
TEST(AvbdConstraint, RowInventoryWarmStartsPersistentHardRows)
{
  vbd::AvbdScalarRowDescriptor contact
      = makeRow(vbd::AvbdScalarRowRole::ContactNormal, 7);
  vbd::AvbdScalarRowDescriptor attachment
      = makeRow(vbd::AvbdScalarRowRole::Attachment, 3);

  vbd::AvbdScalarRowInventory inventory;
  std::vector<vbd::AvbdScalarRowDescriptor> descriptors = {contact, attachment};
  inventory.syncActiveRows(descriptors, {});
  ASSERT_EQ(inventory.size(), 2u);
  inventory[0].state.stiffness = 40.0;
  inventory[0].state.lambda = 10.0;
  inventory[1].state.stiffness = 20.0;
  inventory[1].state.lambda = 5.0;

  vbd::AvbdRowWarmStartOptions options;
  options.alpha = 0.5;
  options.gamma = 0.25;

  descriptors = {attachment};
  inventory.syncActiveRows(descriptors, options);

  ASSERT_EQ(inventory.size(), 1u);
  EXPECT_EQ(inventory[0].descriptor.key, attachment.key);
  EXPECT_DOUBLE_EQ(inventory[0].state.lambda, 0.625);
  EXPECT_DOUBLE_EQ(inventory[0].state.stiffness, 5.0);
  EXPECT_EQ(inventory.find(contact.key), nullptr);
}

//==============================================================================
TEST(AvbdConstraint, RowInventoryWarmStartsStableOrderInPlace)
{
  vbd::AvbdScalarRowDescriptor contact
      = makeRow(vbd::AvbdScalarRowRole::ContactNormal, 7);
  vbd::AvbdScalarRowDescriptor attachment
      = makeRow(vbd::AvbdScalarRowRole::Attachment, 3);

  vbd::AvbdScalarRowInventory inventory;
  const std::vector<vbd::AvbdScalarRowDescriptor> descriptors
      = {contact, attachment};
  inventory.syncActiveRows(descriptors, {});
  ASSERT_EQ(inventory.size(), 2u);
  inventory[0].state.stiffness = 40.0;
  inventory[0].state.lambda = 10.0;
  inventory[0].direction = Eigen::Vector3d::UnitX();
  inventory[1].state.stiffness = 20.0;
  inventory[1].state.lambda = 5.0;
  inventory[1].direction = Eigen::Vector3d::UnitY();

  vbd::AvbdRowWarmStartOptions options;
  options.alpha = 0.5;
  options.gamma = 0.25;
  inventory.syncActiveRows(descriptors, options);

  ASSERT_EQ(inventory.size(), 2u);
  EXPECT_EQ(inventory[0].descriptor.key, contact.key);
  EXPECT_DOUBLE_EQ(inventory[0].state.lambda, 1.25);
  EXPECT_DOUBLE_EQ(inventory[0].state.stiffness, 10.0);
  EXPECT_TRUE(inventory[0].direction.isZero());
  EXPECT_EQ(inventory[1].descriptor.key, attachment.key);
  EXPECT_DOUBLE_EQ(inventory[1].state.lambda, 0.625);
  EXPECT_DOUBLE_EQ(inventory[1].state.stiffness, 5.0);
  EXPECT_TRUE(inventory[1].direction.isZero());
}

//==============================================================================
TEST(AvbdConstraint, RowInventoryWarmStartsGeneratedStableOrderInPlace)
{
  vbd::AvbdScalarRowDescriptor contact
      = makeRow(vbd::AvbdScalarRowRole::ContactNormal, 7);
  vbd::AvbdScalarRowDescriptor attachment
      = makeRow(vbd::AvbdScalarRowRole::Attachment, 3);
  const std::vector<vbd::AvbdScalarRowDescriptor> descriptors
      = {contact, attachment};

  vbd::AvbdScalarRowInventory inventory;
  std::size_t keyCalls = 0u;
  std::size_t descriptorCalls = 0u;
  const auto keyAt = [&](std::size_t index) {
    ++keyCalls;
    return descriptors[index].key;
  };
  const auto descriptorAt = [&](std::size_t index) {
    ++descriptorCalls;
    return descriptors[index];
  };

  inventory.syncActiveRowsByIndex(descriptors.size(), keyAt, descriptorAt, {});
  ASSERT_EQ(inventory.size(), 2u);
  EXPECT_EQ(keyCalls, 0u);
  EXPECT_EQ(descriptorCalls, descriptors.size());
  inventory[0].state.stiffness = 40.0;
  inventory[0].state.lambda = 10.0;
  inventory[0].direction = Eigen::Vector3d::UnitX();
  inventory[1].state.stiffness = 20.0;
  inventory[1].state.lambda = 5.0;
  inventory[1].direction = Eigen::Vector3d::UnitY();

  vbd::AvbdRowWarmStartOptions options;
  options.alpha = 0.5;
  options.gamma = 0.25;
  keyCalls = 0u;
  descriptorCalls = 0u;
  inventory.syncActiveRowsByIndex(
      descriptors.size(), keyAt, descriptorAt, options);

  ASSERT_EQ(inventory.size(), 2u);
  EXPECT_EQ(keyCalls, descriptors.size());
  EXPECT_EQ(descriptorCalls, descriptors.size());
  EXPECT_EQ(inventory[0].descriptor.key, contact.key);
  EXPECT_DOUBLE_EQ(inventory[0].state.lambda, 1.25);
  EXPECT_DOUBLE_EQ(inventory[0].state.stiffness, 10.0);
  EXPECT_TRUE(inventory[0].direction.isZero());
  EXPECT_EQ(inventory[1].descriptor.key, attachment.key);
  EXPECT_DOUBLE_EQ(inventory[1].state.lambda, 0.625);
  EXPECT_DOUBLE_EQ(inventory[1].state.stiffness, 5.0);
  EXPECT_TRUE(inventory[1].direction.isZero());
}

//==============================================================================
TEST(AvbdConstraint, RowInventoryKeepsDescriptorOrderAndSeparatesAxes)
{
  vbd::AvbdScalarRowDescriptor normal
      = makeRow(vbd::AvbdScalarRowRole::ContactNormal, 9, 0);
  vbd::AvbdScalarRowDescriptor selfContact
      = makeRow(vbd::AvbdScalarRowRole::SelfContactNormal, 9, 0);
  vbd::AvbdScalarRowDescriptor tangent
      = makeRow(vbd::AvbdScalarRowRole::FrictionTangent, 9, 0);
  vbd::AvbdScalarRowDescriptor secondTangent
      = makeRow(vbd::AvbdScalarRowRole::FrictionTangent, 9, 1);

  vbd::AvbdScalarRowInventory inventory;
  const std::vector<vbd::AvbdScalarRowDescriptor> descriptors
      = {secondTangent, normal, selfContact, tangent};
  inventory.syncActiveRows(descriptors, {});

  ASSERT_EQ(inventory.size(), descriptors.size());
  EXPECT_EQ(inventory[0].descriptor.key, secondTangent.key);
  EXPECT_EQ(inventory[1].descriptor.key, normal.key);
  EXPECT_EQ(inventory[2].descriptor.key, selfContact.key);
  EXPECT_EQ(inventory[3].descriptor.key, tangent.key);
  EXPECT_FALSE(normal.key == selfContact.key);
  EXPECT_FALSE(normal.key == tangent.key);
  EXPECT_FALSE(tangent.key == secondTangent.key);
}

//==============================================================================
TEST(AvbdConstraint, RowInventoryWarmStartsOutOfOrderPreviousRows)
{
  vbd::AvbdScalarRowDescriptor first
      = makeRow(vbd::AvbdScalarRowRole::ContactNormal, 1);
  vbd::AvbdScalarRowDescriptor second
      = makeRow(vbd::AvbdScalarRowRole::ContactNormal, 2);
  vbd::AvbdScalarRowDescriptor third
      = makeRow(vbd::AvbdScalarRowRole::ContactNormal, 3);

  vbd::AvbdScalarRowInventory inventory;
  std::vector<vbd::AvbdScalarRowDescriptor> descriptors
      = {third, first, second};
  inventory.syncActiveRows(descriptors, {});
  ASSERT_EQ(inventory.size(), descriptors.size());
  inventory[0].state.stiffness = 30.0;
  inventory[0].state.lambda = 3.0;
  inventory[1].state.stiffness = 10.0;
  inventory[1].state.lambda = 1.0;
  inventory[2].state.stiffness = 20.0;
  inventory[2].state.lambda = 2.0;

  vbd::AvbdRowWarmStartOptions options;
  options.alpha = 0.5;
  options.gamma = 0.5;

  descriptors = {second, third, first};
  inventory.syncActiveRows(descriptors, options);

  ASSERT_EQ(inventory.size(), descriptors.size());
  EXPECT_EQ(inventory[0].descriptor.key, second.key);
  EXPECT_DOUBLE_EQ(inventory[0].state.lambda, 0.5);
  EXPECT_DOUBLE_EQ(inventory[0].state.stiffness, 10.0);
  EXPECT_EQ(inventory[1].descriptor.key, third.key);
  EXPECT_DOUBLE_EQ(inventory[1].state.lambda, 0.75);
  EXPECT_DOUBLE_EQ(inventory[1].state.stiffness, 15.0);
  EXPECT_EQ(inventory[2].descriptor.key, first.key);
  EXPECT_DOUBLE_EQ(inventory[2].state.lambda, 0.25);
  EXPECT_DOUBLE_EQ(inventory[2].state.stiffness, 5.0);
}

//==============================================================================
TEST(AvbdConstraint, RowInventoryFiniteRowsDropLambdaAndClampToMaterial)
{
  vbd::AvbdScalarRowDescriptor descriptor
      = makeRow(vbd::AvbdScalarRowRole::JointLinear, 11);
  descriptor.kind = vbd::AvbdScalarRowKind::FiniteStiffness;
  descriptor.materialStiffness = 12.0;

  vbd::AvbdScalarRowInventory inventory;
  std::vector<vbd::AvbdScalarRowDescriptor> descriptors = {descriptor};
  inventory.syncActiveRows(descriptors, {});
  ASSERT_EQ(inventory.size(), 1u);
  inventory[0].state.stiffness = 30.0;
  inventory[0].state.lambda = 9.0;

  vbd::AvbdRowWarmStartOptions options;
  options.alpha = 0.25;
  options.gamma = 0.5;
  inventory.syncActiveRows(descriptors, options);

  ASSERT_EQ(inventory.size(), 1u);
  EXPECT_DOUBLE_EQ(inventory[0].state.lambda, 0.0);
  EXPECT_DOUBLE_EQ(inventory[0].state.stiffness, 12.0);
}

//==============================================================================
TEST(AvbdConstraint, RowInventoryColdStartsReplacedKeyAtConstantCount)
{
  // rowB sorts after rowC by key, so under the slow path the cold-started rowC
  // must not inherit rowB's stale persistent state when a single key is
  // replaced while the active-row count stays constant.
  vbd::AvbdScalarRowDescriptor rowA
      = makeRow(vbd::AvbdScalarRowRole::ContactNormal, 1);
  vbd::AvbdScalarRowDescriptor rowB
      = makeRow(vbd::AvbdScalarRowRole::ContactNormal, 5);
  vbd::AvbdScalarRowDescriptor rowC
      = makeRow(vbd::AvbdScalarRowRole::ContactNormal, 3);

  vbd::AvbdScalarRowInventory inventory;
  std::vector<vbd::AvbdScalarRowDescriptor> descriptors = {rowA, rowB};
  inventory.syncActiveRows(descriptors, {});
  ASSERT_EQ(inventory.size(), 2u);
  inventory[0].state.stiffness = 40.0;
  inventory[0].state.lambda = 10.0;
  inventory[1].state.stiffness = 80.0;
  inventory[1].state.lambda = 20.0;

  vbd::AvbdRowWarmStartOptions options;
  options.alpha = 0.5;
  options.gamma = 0.25;

  // Replace rowB with the brand-new rowC; count is unchanged at 2.
  descriptors = {rowA, rowC};
  inventory.syncActiveRows(descriptors, options);

  ASSERT_EQ(inventory.size(), 2u);

  // rowA persists -> warm starts from its seeded state.
  EXPECT_EQ(inventory[0].descriptor.key, rowA.key);
  EXPECT_DOUBLE_EQ(inventory[0].state.lambda, 1.25);
  EXPECT_DOUBLE_EQ(inventory[0].state.stiffness, 10.0);

  // rowC is a new key -> cold starts: lambda 0, stiffness = clamped start
  // stiffness, never rowB's stale 80.0/20.0.
  EXPECT_EQ(inventory[1].descriptor.key, rowC.key);
  EXPECT_DOUBLE_EQ(inventory[1].state.lambda, 0.0);
  EXPECT_DOUBLE_EQ(inventory[1].state.stiffness, 4.0);

  EXPECT_EQ(inventory.find(rowB.key), nullptr);
}
