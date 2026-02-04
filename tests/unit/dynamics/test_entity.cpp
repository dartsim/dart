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

#include <dart/dynamics/box_shape.hpp>
#include <dart/dynamics/simple_frame.hpp>

#include <gtest/gtest.h>

using namespace dart;
using namespace dart::dynamics;

TEST(EntityTest, GetParentFrame)
{
  auto frame = SimpleFrame::createShared(Frame::World(), "entity_test");

  EXPECT_EQ(frame->getParentFrame(), Frame::World());

  auto child = SimpleFrame::createShared(frame.get(), "child_frame");
  EXPECT_EQ(child->getParentFrame(), frame.get());
}

TEST(EntityTest, DescendsFromWorld)
{
  auto frame = SimpleFrame::createShared(Frame::World(), "world_child");

  EXPECT_TRUE(frame->descendsFrom(Frame::World()));
}

TEST(EntityTest, DescendsFromParent)
{
  auto parent = SimpleFrame::createShared(Frame::World(), "parent");
  auto child = SimpleFrame::createShared(parent.get(), "child");
  auto grandchild = SimpleFrame::createShared(child.get(), "grandchild");

  EXPECT_TRUE(child->descendsFrom(parent.get()));
  EXPECT_TRUE(grandchild->descendsFrom(child.get()));
  EXPECT_TRUE(grandchild->descendsFrom(parent.get()));
  EXPECT_TRUE(grandchild->descendsFrom(Frame::World()));
}

TEST(EntityTest, DescendsFromSelf)
{
  auto frame = SimpleFrame::createShared(Frame::World(), "self_test");

  EXPECT_TRUE(frame->descendsFrom(frame.get()));
}

TEST(EntityTest, DescendsFromUnrelated)
{
  auto frame1 = SimpleFrame::createShared(Frame::World(), "frame1");
  auto frame2 = SimpleFrame::createShared(Frame::World(), "frame2");

  EXPECT_FALSE(frame1->descendsFrom(frame2.get()));
  EXPECT_FALSE(frame2->descendsFrom(frame1.get()));
}

TEST(EntityTest, DescendsFromNullptr)
{
  auto frame = SimpleFrame::createShared(Frame::World(), "null_test");

  EXPECT_TRUE(frame->descendsFrom(nullptr));
}

TEST(EntityTest, IsQuiet)
{
  auto frame = SimpleFrame::createShared(Frame::World(), "quiet_test");

  EXPECT_FALSE(frame->isQuiet());
}

TEST(EntityTest, IsFrame)
{
  auto frame = SimpleFrame::createShared(Frame::World(), "frame_test");

  EXPECT_TRUE(frame->isFrame());
}

TEST(EntityTest, DirtyTransform)
{
  auto frame = SimpleFrame::createShared(Frame::World(), "dirty_tf");

  frame->dirtyTransform();
  EXPECT_TRUE(frame->needsTransformUpdate());
}

TEST(EntityTest, DirtyVelocity)
{
  auto frame = SimpleFrame::createShared(Frame::World(), "dirty_vel");

  frame->dirtyVelocity();
  EXPECT_TRUE(frame->needsVelocityUpdate());
}

TEST(EntityTest, DirtyAcceleration)
{
  auto frame = SimpleFrame::createShared(Frame::World(), "dirty_acc");

  frame->dirtyAcceleration();
  EXPECT_TRUE(frame->needsAccelerationUpdate());
}

TEST(EntityTest, NeedsUpdateAfterParentChange)
{
  auto parent1 = SimpleFrame::createShared(Frame::World(), "parent1");
  auto parent2 = SimpleFrame::createShared(Frame::World(), "parent2");
  auto child = SimpleFrame::createShared(parent1.get(), "child");

  child->setParentFrame(parent2.get());

  EXPECT_EQ(child->getParentFrame(), parent2.get());
}

TEST(EntityTest, ChangeParentFrameViaSetParentFrame)
{
  auto parent1 = SimpleFrame::createShared(Frame::World(), "p1");
  auto parent2 = SimpleFrame::createShared(Frame::World(), "p2");
  auto child = SimpleFrame::createShared(parent1.get(), "c");

  EXPECT_EQ(child->getParentFrame(), parent1.get());

  child->setParentFrame(parent2.get());
  EXPECT_EQ(child->getParentFrame(), parent2.get());

  child->setParentFrame(Frame::World());
  EXPECT_EQ(child->getParentFrame(), Frame::World());
}

TEST(EntityTest, ChangeParentFrameNoOp)
{
  auto parent = SimpleFrame::createShared(Frame::World(), "noop_parent");
  auto child = SimpleFrame::createShared(parent.get(), "noop_child");

  child->setParentFrame(parent.get());
  EXPECT_EQ(child->getParentFrame(), parent.get());
}
