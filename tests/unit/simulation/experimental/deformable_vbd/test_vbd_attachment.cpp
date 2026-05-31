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

#include <dart/simulation/experimental/detail/deformable_vbd/attachment_kernel.hpp>
#include <dart/simulation/experimental/detail/deformable_vbd/avbd_row_inventory.hpp>
#include <dart/simulation/experimental/detail/deformable_vbd/block_descent.hpp>

#include <gtest/gtest.h>

#include <vector>

namespace vbd = dart::simulation::experimental::detail::deformable_vbd;

namespace {

using Vec3 = Eigen::Vector3d;

//==============================================================================
vbd::AvbdPointAttachmentRow makeXAttachment(double targetX, double stiffness)
{
  vbd::AvbdPointAttachmentRow row;
  row.vertex = 0;
  row.target = Vec3(targetX, 0.0, 0.0);
  row.axis = Vec3::UnitX();
  row.state.stiffness = stiffness;
  row.state.lambda = 0.0;
  return row;
}

//==============================================================================
vbd::AvbdScalarRowDescriptor makeAttachmentDescriptor(std::uint8_t axis)
{
  vbd::AvbdScalarRowDescriptor descriptor;
  descriptor.key.role = vbd::AvbdScalarRowRole::Attachment;
  descriptor.key.objectA = 12;
  descriptor.key.featureA = 0;
  descriptor.key.axis = axis;
  descriptor.startStiffness = 5.0;
  return descriptor;
}

} // namespace

//==============================================================================
TEST(VbdAttachment, AvbdAttachmentForcePullsVertexTowardTarget)
{
  vbd::AvbdPointAttachmentRow row = makeXAttachment(1.25, 40.0);
  const Vec3 position(0.25, 2.0, -3.0);

  vbd::VertexBlock block;
  const double forceMagnitude
      = vbd::addAvbdPointAttachment(block, position, row, /*alpha=*/0.0);

  EXPECT_DOUBLE_EQ(forceMagnitude, 40.0);
  EXPECT_DOUBLE_EQ(block.force.x(), 40.0);
  EXPECT_DOUBLE_EQ(block.force.y(), 0.0);
  EXPECT_DOUBLE_EQ(block.force.z(), 0.0);
  EXPECT_NEAR(
      (block.hessian - 40.0 * Vec3::UnitX() * Vec3::UnitX().transpose()).norm(),
      0.0,
      1e-15);
}

//==============================================================================
TEST(VbdAttachment, AlphaRegularizesPreExistingAttachmentError)
{
  vbd::AvbdPointAttachmentRow row = makeXAttachment(1.0, 100.0);
  row.previousConstraintValue = 0.8;
  const Vec3 position(0.2, 0.0, 0.0);

  vbd::VertexBlock unregularized;
  const double fullForce = vbd::addAvbdPointAttachment(
      unregularized, position, row, /*alpha=*/0.0);

  vbd::VertexBlock regularized;
  const double regularizedForce
      = vbd::addAvbdPointAttachment(regularized, position, row, /*alpha=*/0.75);

  EXPECT_NEAR(fullForce, 80.0, 1e-15);
  EXPECT_NEAR(regularizedForce, 20.0, 1e-12);
  EXPECT_NEAR(regularized.force.x(), 20.0, 1e-12);
}

//==============================================================================
TEST(VbdAttachment, AttachmentInventoryWarmStartsEachAxisIndependently)
{
  std::vector<vbd::AvbdScalarRowDescriptor> descriptors
      = {makeAttachmentDescriptor(0),
         makeAttachmentDescriptor(1),
         makeAttachmentDescriptor(2)};
  vbd::AvbdScalarRowInventory inventory;
  inventory.syncActiveRows(descriptors, {});
  ASSERT_EQ(inventory.size(), 3u);
  inventory[0].state.lambda = 12.0;
  inventory[0].state.stiffness = 80.0;
  inventory[1].state.lambda = 6.0;
  inventory[1].state.stiffness = 40.0;
  inventory[2].state.lambda = 3.0;
  inventory[2].state.stiffness = 20.0;

  vbd::AvbdRowWarmStartOptions options;
  options.alpha = 0.5;
  options.gamma = 0.25;
  descriptors = {makeAttachmentDescriptor(2), makeAttachmentDescriptor(0)};
  inventory.syncActiveRows(descriptors, options);

  ASSERT_EQ(inventory.size(), 2u);
  EXPECT_EQ(inventory[0].descriptor.key.axis, 2);
  EXPECT_DOUBLE_EQ(inventory[0].state.lambda, 0.375);
  EXPECT_DOUBLE_EQ(inventory[0].state.stiffness, 5.0);
  EXPECT_EQ(inventory[1].descriptor.key.axis, 0);
  EXPECT_DOUBLE_EQ(inventory[1].state.lambda, 1.5);
  EXPECT_DOUBLE_EQ(inventory[1].state.stiffness, 20.0);
}

//==============================================================================
TEST(VbdAttachment, BlockDescentAttachmentUpdatesDualStateDuringSolve)
{
  std::vector<Vec3> positions = {Vec3(0.0, 0.0, 0.0)};
  const std::vector<double> masses = {1.0};
  const std::vector<std::uint8_t> fixed = {0u};
  const std::vector<Vec3> inertialTargets = positions;
  const std::vector<vbd::SpringElement> springs;
  const auto coloring = vbd::colorSprings(1, springs);
  const auto adjacency = vbd::SpringAdjacency::build(1, springs);

  vbd::AvbdPointAttachmentRow attachment = makeXAttachment(1.0, 20.0);
  std::vector<vbd::AvbdPointAttachmentRow> attachments = {attachment};

  vbd::BlockDescentOptions options;
  options.iterations = 5;
  vbd::AvbdPointAttachmentOptions avbdOptions;
  avbdOptions.alpha = 0.0;
  avbdOptions.beta = 100.0;

  const vbd::BlockDescentStats stats
      = vbd::blockDescentMassSpringAvbdAttachments(
          positions,
          masses,
          fixed,
          inertialTargets,
          springs,
          0.0,
          0.1,
          attachments,
          coloring,
          adjacency,
          options,
          avbdOptions);

  EXPECT_EQ(stats.iterations, 5u);
  EXPECT_GT(positions[0].x(), 0.0);
  EXPECT_LT(positions[0].x(), 1.0);
  EXPECT_NEAR(positions[0].y(), 0.0, 1e-15);
  EXPECT_NEAR(positions[0].z(), 0.0, 1e-15);
  EXPECT_GT(attachments[0].state.lambda, 0.0);
  EXPECT_GT(attachments[0].state.stiffness, 20.0);
}
