/*
 * Copyright (c) 2011-2025, The DART development contributors
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

#include <dart8/common/constants.hpp>
#include <dart8/common/exceptions.hpp>
#include <dart8/frame/fixed_frame.hpp>
#include <dart8/frame/free_frame.hpp>
#include <dart8/world.hpp>

#include <Eigen/Geometry>
#include <gtest/gtest.h>

using namespace dart8;

//==============================================================================
// FreeFrame Tests
//==============================================================================

TEST(FreeFrame, Construction)
{
  World world;

  // Create FreeFrame with auto-generated name
  auto frame1 = world.addFreeFrame();
  EXPECT_TRUE(frame1.isValid());
  EXPECT_FALSE(frame1.isWorld());

  // Create FreeFrame with custom name
  auto frame2 = world.addFreeFrame("my_frame");
  EXPECT_TRUE(frame2.isValid());

  // Different entities
  EXPECT_FALSE(frame1.isSameInstanceAs(frame2));
}

TEST(FreeFrame, ParentFrame)
{
  World world;

  // Create frame with default parent (world)
  auto frame1 = world.addFreeFrame("frame1");
  auto parent1 = frame1.getParentFrame();
  EXPECT_TRUE(parent1.isWorld());

  // Create frame with custom parent
  auto frame2 = world.addFreeFrame("frame2", frame1);
  auto parent2 = frame2.getParentFrame();
  EXPECT_TRUE(parent2.isSameInstanceAs(frame1));

  // Change parent frame
  frame2.setParentFrame(Frame::world());
  auto newParent = frame2.getParentFrame();
  EXPECT_TRUE(newParent.isWorld());
}

TEST(FreeFrame, LocalTransform)
{
  World world;
  auto frame = world.addFreeFrame("test_frame");

  // Default transform should be identity
  Eigen::Isometry3d T_initial = frame.getLocalTransform();
  EXPECT_TRUE(T_initial.isApprox(Eigen::Isometry3d::Identity()));

  // Set custom transform
  Eigen::Isometry3d T_desired = Eigen::Isometry3d::Identity();
  T_desired.translate(Eigen::Vector3d(1.0, 2.0, 3.0));
  T_desired.rotate(Eigen::AngleAxisd(dart8::pi / 4, Eigen::Vector3d::UnitZ()));

  frame.setLocalTransform(T_desired);
  Eigen::Isometry3d T_actual = frame.getLocalTransform();
  EXPECT_TRUE(T_actual.isApprox(T_desired));
}

TEST(FreeFrame, IndependentMotion)
{
  World world;

  // Create parent and child frames
  auto parent = world.addFreeFrame("parent");
  auto child = world.addFreeFrame("child", parent);

  // Set parent transform
  Eigen::Isometry3d T_parent = Eigen::Isometry3d::Identity();
  T_parent.translate(Eigen::Vector3d(1.0, 0.0, 0.0));
  parent.setLocalTransform(T_parent);

  // Set child transform
  Eigen::Isometry3d T_child = Eigen::Isometry3d::Identity();
  T_child.translate(Eigen::Vector3d(0.0, 1.0, 0.0));
  child.setLocalTransform(T_child);

  // Verify independent motion
  EXPECT_TRUE(parent.getLocalTransform().isApprox(T_parent));
  EXPECT_TRUE(child.getLocalTransform().isApprox(T_child));
}

//==============================================================================
// FixedFrame Tests
//==============================================================================

TEST(FixedFrame, ConstructionRequiresName)
{
  World world;
  auto parent = world.addFreeFrame("parent");

  // Empty name should throw
  EXPECT_THROW(
      world.addFixedFrame("", parent), dart8::InvalidArgumentException);

  // Valid name should succeed
  auto attached = world.addFixedFrame("attached", parent);
  EXPECT_TRUE(attached.isValid());
}

TEST(FixedFrame, CannotAttachToWorld)
{
  World world;

  // Attaching to world frame should throw
  EXPECT_THROW(
      world.addFixedFrame("attached", Frame::world()),
      dart8::InvalidArgumentException);
}

TEST(FixedFrame, ConstructionWithOffset)
{
  World world;
  auto parent = world.addFreeFrame("parent");

  // Create with default offset (identity)
  auto attached1 = world.addFixedFrame("attached1", parent);
  Eigen::Isometry3d T1 = attached1.getLocalTransform();
  EXPECT_TRUE(T1.isApprox(Eigen::Isometry3d::Identity()));

  // Create with custom offset
  Eigen::Isometry3d offset = Eigen::Isometry3d::Identity();
  offset.translate(Eigen::Vector3d(1.0, 2.0, 3.0));
  offset.rotate(Eigen::AngleAxisd(dart8::pi / 6, Eigen::Vector3d::UnitX()));

  auto attached2 = world.addFixedFrame("attached2", parent, offset);
  Eigen::Isometry3d T2 = attached2.getLocalTransform();
  EXPECT_TRUE(T2.isApprox(offset));
}

TEST(FixedFrame, SetLocalTransform)
{
  World world;
  auto parent = world.addFreeFrame("parent");
  auto attached = world.addFixedFrame("attached", parent);

  // Modify the offset
  Eigen::Isometry3d new_offset = Eigen::Isometry3d::Identity();
  new_offset.translate(Eigen::Vector3d(0.5, 1.5, 2.5));
  attached.setLocalTransform(new_offset);

  Eigen::Isometry3d T = attached.getLocalTransform();
  EXPECT_TRUE(T.isApprox(new_offset));
}

TEST(FixedFrame, ParentFrame)
{
  World world;
  auto parent1 = world.addFreeFrame("parent1");
  auto attached = world.addFixedFrame("attached", parent1);

  // Verify parent
  Frame parent = attached.getParentFrame();
  EXPECT_TRUE(parent.isSameInstanceAs(parent1));
}

TEST(FixedFrame, RigidAttachment)
{
  World world;
  auto parent = world.addFreeFrame("parent");

  // Create attached frame with offset
  Eigen::Isometry3d offset = Eigen::Isometry3d::Identity();
  offset.translate(Eigen::Vector3d(1.0, 0.0, 0.0));
  auto attached = world.addFixedFrame("attached", parent, offset);

  // Change parent transform - offset should remain constant
  Eigen::Isometry3d new_parent_transform = Eigen::Isometry3d::Identity();
  new_parent_transform.translate(Eigen::Vector3d(5.0, 5.0, 5.0));
  parent.setLocalTransform(new_parent_transform);

  // FixedFrame offset should not change
  Eigen::Isometry3d attached_offset = attached.getLocalTransform();
  EXPECT_TRUE(attached_offset.isApprox(offset));
}

//==============================================================================
// Mixed Frame Types Tests
//==============================================================================

TEST(Frames, MixedHierarchy)
{
  World world;

  // Create hierarchy: FreeFrame -> FixedFrame -> FreeFrame
  auto free1 = world.addFreeFrame("free1");
  auto attached = world.addFixedFrame("attached", free1);
  auto free2 = world.addFreeFrame("free2", attached);

  // Set transforms
  Eigen::Isometry3d T_free1 = Eigen::Isometry3d::Identity();
  T_free1.translate(Eigen::Vector3d(1.0, 0.0, 0.0));
  free1.setLocalTransform(T_free1);

  Eigen::Isometry3d T_attached = Eigen::Isometry3d::Identity();
  T_attached.translate(Eigen::Vector3d(0.0, 1.0, 0.0));
  attached.setLocalTransform(T_attached);

  Eigen::Isometry3d T_free2 = Eigen::Isometry3d::Identity();
  T_free2.translate(Eigen::Vector3d(0.0, 0.0, 1.0));
  free2.setLocalTransform(T_free2);

  // Verify each frame maintains its transform
  EXPECT_TRUE(free1.getLocalTransform().isApprox(T_free1));
  EXPECT_TRUE(attached.getLocalTransform().isApprox(T_attached));
  EXPECT_TRUE(free2.getLocalTransform().isApprox(T_free2));
}

TEST(Frames, MultipleFixedFrames)
{
  World world;
  auto parent = world.addFreeFrame("parent");

  // Create multiple attached frames with different offsets
  Eigen::Isometry3d offset1 = Eigen::Isometry3d::Identity();
  offset1.translate(Eigen::Vector3d(1.0, 0.0, 0.0));
  auto attached1 = world.addFixedFrame("attached1", parent, offset1);

  Eigen::Isometry3d offset2 = Eigen::Isometry3d::Identity();
  offset2.translate(Eigen::Vector3d(0.0, 1.0, 0.0));
  auto attached2 = world.addFixedFrame("attached2", parent, offset2);

  Eigen::Isometry3d offset3 = Eigen::Isometry3d::Identity();
  offset3.translate(Eigen::Vector3d(0.0, 0.0, 1.0));
  auto attached3 = world.addFixedFrame("attached3", parent, offset3);

  // Verify independent offsets
  EXPECT_TRUE(attached1.getLocalTransform().isApprox(offset1));
  EXPECT_TRUE(attached2.getLocalTransform().isApprox(offset2));
  EXPECT_TRUE(attached3.getLocalTransform().isApprox(offset3));

  // All should have same parent
  EXPECT_TRUE(attached1.getParentFrame().isSameInstanceAs(parent));
  EXPECT_TRUE(attached2.getParentFrame().isSameInstanceAs(parent));
  EXPECT_TRUE(attached3.getParentFrame().isSameInstanceAs(parent));
}
