/*
 * Copyright (c) 2011-2026, The DART development contributors
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

#include <dart/simulation/experimental/common/exceptions.hpp>
#include <dart/simulation/experimental/comps/frame_types.hpp>
#include <dart/simulation/experimental/frame/fixed_frame.hpp>
#include <dart/simulation/experimental/frame/frame.hpp>
#include <dart/simulation/experimental/frame/free_frame.hpp>
#include <dart/simulation/experimental/world.hpp>

#include <Eigen/Geometry>
#include <gtest/gtest.h>

using namespace dart::simulation::experimental;

//==============================================================================
// Frame Caching Tests (Lazy Evaluation)
//==============================================================================

// Test that world transform uses lazy evaluation (not computed until requested)
TEST(Frame, LazyEvaluation)
{
  World world;
  auto freeFrame = world.addFreeFrame("test");

  auto& registry = world.getRegistry();
  auto entity = freeFrame.getEntity();

  // Set transform but don't query world transform yet
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  T.translate(Eigen::Vector3d(1, 2, 3));
  freeFrame.setLocalTransform(T);

  // Cache should be marked dirty
  {
    const auto& cache = registry.get<comps::FrameCache>(entity);
    EXPECT_TRUE(cache.needTransformUpdate)
        << "setLocalTransform should mark cache as dirty";
  }

  // Query world transform - this should compute and cache it
  auto T_world = freeFrame.getTransform();

  // Cache should now be clean
  {
    const auto& cache = registry.get<comps::FrameCache>(entity);
    EXPECT_FALSE(cache.needTransformUpdate)
        << "getTransform should compute and clear dirty flag";
  }

  // Verify computed transform
  EXPECT_TRUE(T_world.isApprox(T));
}

// Test that setLocalTransform invalidates cache
TEST(Frame, CacheInvalidation)
{
  World world;
  auto freeFrame = world.addFreeFrame("test");

  auto& registry = world.getRegistry();
  auto entity = freeFrame.getEntity();

  // Trigger cache computation
  [[maybe_unused]] auto T1 = freeFrame.getTransform();
  {
    const auto& cache = registry.get<comps::FrameCache>(entity);
    EXPECT_FALSE(cache.needTransformUpdate) << "Cache should be clean";
  }

  // Modify transform
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  T.translate(Eigen::Vector3d(5, 6, 7));
  freeFrame.setLocalTransform(T);

  // Cache should be dirty again
  {
    const auto& cache = registry.get<comps::FrameCache>(entity);
    EXPECT_TRUE(cache.needTransformUpdate)
        << "setLocalTransform should invalidate cache";
  }
}

// Test FreeFrame hierarchical transforms (manual cache invalidation needed)
//
// NOTE: Automatic hierarchical cache invalidation (parent change invalidates
// children) is not yet implemented in Phase 1. When a parent's transform
// changes, you must manually invalidate child caches by calling
// child.setLocalTransform() with the same transform.
//
// TODO(Phase 2): Implement automatic hierarchical cache invalidation
// DISABLED: Waiting for Phase 2 implementation
TEST(Frame, DISABLED_HierarchicalTransforms)
{
  World world;

  // Create parent and child frames
  auto parent = world.addFreeFrame("parent");
  auto child = world.addFreeFrame("child", parent);

  // Set initial transforms
  Eigen::Isometry3d T_parent = Eigen::Isometry3d::Identity();
  T_parent.translate(Eigen::Vector3d(1, 0, 0));
  parent.setLocalTransform(T_parent);

  Eigen::Isometry3d T_child_rel = Eigen::Isometry3d::Identity();
  T_child_rel.translate(Eigen::Vector3d(0, 1, 0));
  child.setLocalTransform(T_child_rel);

  // Compute both transforms to cache them
  [[maybe_unused]] auto T_parent_world = parent.getTransform();
  auto T_child_world = child.getTransform();

  // Verify child world transform = parent * child_local
  Eigen::Isometry3d T_expected = T_parent * T_child_rel;
  EXPECT_TRUE(T_child_world.isApprox(T_expected));

  // Modify parent transform
  Eigen::Isometry3d T_parent_new = Eigen::Isometry3d::Identity();
  T_parent_new.translate(Eigen::Vector3d(10, 10, 10));
  parent.setLocalTransform(T_parent_new);

  // WORKAROUND: Manually invalidate child cache by re-setting its transform
  // This is necessary until automatic hierarchical cache invalidation is
  // implemented
  child.setLocalTransform(T_child_rel);

  // Recompute child world transform
  auto T_child_world_new = child.getTransform();

  // Verify new child world transform = new_parent * child_local
  Eigen::Isometry3d T_expected_new = T_parent_new * T_child_rel;
  EXPECT_TRUE(T_child_world_new.isApprox(T_expected_new));
}

//==============================================================================
// FixedFrame Caching Tests
//==============================================================================

// Reject parenting across worlds
TEST(Frame, RejectCrossWorldParent)
{
  World world1;
  World world2;

  auto frame1 = world1.addFreeFrame("frame1");
  auto frame2 = world2.addFreeFrame("frame2");

  EXPECT_THROW(
      frame1.setParentFrame(frame2),
      dart::simulation::experimental::InvalidArgumentException);
}

// Reject parenting a frame to itself
TEST(Frame, RejectSelfParent)
{
  World world;
  auto frame = world.addFreeFrame("self");

  EXPECT_THROW(
      frame.setParentFrame(frame),
      dart::simulation::experimental::InvalidArgumentException);
}

// Reject cycles in the frame hierarchy
TEST(Frame, RejectCyclicParent)
{
  World world;
  auto parent = world.addFreeFrame("parent");
  auto child = world.addFreeFrame("child", parent);

  EXPECT_THROW(
      parent.setParentFrame(child),
      dart::simulation::experimental::InvalidOperationException);
}

// Mark descendants dirty when reparenting
TEST(Frame, ReparentInvalidatesSubtreeCaches)
{
  World world;
  auto parent = world.addFreeFrame("parent");
  auto child = world.addFreeFrame("child", parent);
  auto grandchild = world.addFreeFrame("grand", child);

  Eigen::Isometry3d T_child = Eigen::Isometry3d::Identity();
  T_child.translate(Eigen::Vector3d(1, 0, 0));
  child.setLocalTransform(T_child);

  Eigen::Isometry3d T_grand = Eigen::Isometry3d::Identity();
  T_grand.translate(Eigen::Vector3d(0, 2, 0));
  grandchild.setLocalTransform(T_grand);

  auto& registry = world.getRegistry();
  auto childEntity = child.getEntity();
  auto grandEntity = grandchild.getEntity();

  // Prime caches
  [[maybe_unused]] auto grandWorld = grandchild.getTransform();
  EXPECT_FALSE(
      registry.get<comps::FrameCache>(childEntity).needTransformUpdate);
  EXPECT_FALSE(
      registry.get<comps::FrameCache>(grandEntity).needTransformUpdate);

  child.setParentFrame(Frame::world());

  EXPECT_TRUE(registry.get<comps::FrameCache>(childEntity).needTransformUpdate);
  EXPECT_TRUE(registry.get<comps::FrameCache>(grandEntity).needTransformUpdate);

  auto newGrandWorld = grandchild.getTransform();
  (void)newGrandWorld;
  EXPECT_FALSE(
      registry.get<comps::FrameCache>(childEntity).needTransformUpdate);
  EXPECT_FALSE(
      registry.get<comps::FrameCache>(grandEntity).needTransformUpdate);
}

// Test FixedFrame lazy evaluation
TEST(Frame, FixedFrameCaching)
{
  World world;
  auto parent = world.addFreeFrame("parent");

  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  T.translate(Eigen::Vector3d(1, 2, 3));
  auto fixedFrame = world.addFixedFrame("attached", parent, T);

  // Query world transform
  auto T1 = fixedFrame.getTransform();
  EXPECT_TRUE(T1.isApprox(T));

  // Check that cache is clean after query
  auto& registry = world.getRegistry();
  auto entity = fixedFrame.getEntity();
  {
    const auto& cache = registry.get<comps::FrameCache>(entity);
    EXPECT_FALSE(cache.needTransformUpdate)
        << "Cache should be clean after getTransform()";
  }

  // Query again - should use cache
  auto T2 = fixedFrame.getTransform();
  EXPECT_TRUE(T2.isApprox(T1));
}

// Test FixedFrame cache invalidation when offset is changed
TEST(Frame, FixedFrameCacheInvalidation)
{
  World world;
  auto parent = world.addFreeFrame("parent");

  Eigen::Isometry3d T1 = Eigen::Isometry3d::Identity();
  T1.translate(Eigen::Vector3d(1, 0, 0));
  auto fixedFrame = world.addFixedFrame("attached", parent, T1);

  [[maybe_unused]] auto T_cached = fixedFrame.getTransform();

  auto& registry = world.getRegistry();
  auto entity = fixedFrame.getEntity();
  {
    const auto& cache = registry.get<comps::FrameCache>(entity);
    EXPECT_FALSE(cache.needTransformUpdate) << "Cache should be clean";
  }

  // Change the offset
  Eigen::Isometry3d T2 = Eigen::Isometry3d::Identity();
  T2.translate(Eigen::Vector3d(5, 5, 5));
  fixedFrame.setLocalTransform(T2);

  // Cache should be invalidated
  {
    const auto& cache = registry.get<comps::FrameCache>(entity);
    EXPECT_TRUE(cache.needTransformUpdate)
        << "setLocalTransform should invalidate cache";
  }

  // Verify new transform
  auto T_new = fixedFrame.getTransform();
  EXPECT_TRUE(T_new.isApprox(T2));
}

//==============================================================================
// Frame Hierarchy Tests
//==============================================================================

// Test changing parent frame
TEST(Frame, ChangeParent)
{
  World world;

  auto parent1 = world.addFreeFrame("parent1");
  auto parent2 = world.addFreeFrame("parent2");
  auto child = world.addFreeFrame("child", parent1);

  // Verify initial parent
  EXPECT_TRUE(child.getParentFrame().isSameInstanceAs(parent1));

  // Change parent
  child.setParentFrame(parent2);
  EXPECT_TRUE(child.getParentFrame().isSameInstanceAs(parent2));

  // Change to world
  child.setParentFrame(Frame::world());
  EXPECT_TRUE(child.getParentFrame().isWorld());
}

// Test relative transform queries
TEST(Frame, RelativeTransform)
{
  World world;

  auto parent = world.addFreeFrame("parent");
  auto child = world.addFreeFrame("child", parent);

  // Set transforms
  Eigen::Isometry3d T_parent = Eigen::Isometry3d::Identity();
  T_parent.translate(Eigen::Vector3d(1, 0, 0));
  parent.setLocalTransform(T_parent);

  Eigen::Isometry3d T_child_local = Eigen::Isometry3d::Identity();
  T_child_local.translate(Eigen::Vector3d(0, 1, 0));
  child.setLocalTransform(T_child_local);

  // Compute relative transform from parent to child
  auto T_rel = child.getTransform(parent);

  // Verify: T_rel = parent^{-1} * child
  Eigen::Isometry3d T_actual
      = parent.getTransform().inverse() * child.getTransform();
  EXPECT_TRUE(T_rel.isApprox(T_actual));
}

// Test world frame is special
TEST(Frame, WorldFrame)
{
  auto world_frame = Frame::world();

  EXPECT_TRUE(world_frame.isWorld());
  EXPECT_TRUE(world_frame.isValid());

  // World frame parent is itself
  auto parent = world_frame.getParentFrame();
  EXPECT_TRUE(parent.isWorld());

  // World frame transform is identity
  auto T = world_frame.getTransform();
  EXPECT_TRUE(T.isApprox(Eigen::Isometry3d::Identity()));

  // World frame local transform is identity
  auto T_local = world_frame.getLocalTransform();
  EXPECT_TRUE(T_local.isApprox(Eigen::Isometry3d::Identity()));
}
