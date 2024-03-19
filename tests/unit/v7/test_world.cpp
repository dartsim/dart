/*
 * Copyright (c) The DART development contributors
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

#include <dart/v7/world.hpp>

#include <gtest/gtest.h>

using namespace dart;
using namespace v7;

TEST(WorldTest, SaveLoadWorld)
{
  auto w = World();
  w.saveToFile("test_world.urdf");
}

TEST(WorldTest, SingleStep)
{
  auto w = World();
  w.step();
}

TEST(WorldTest, RigidBodyAddRemove)
{
  auto w = World();
  auto body1 = w.addRigidBody({"body1", 0.0});

  EXPECT_EQ(body1.getName(), "body1");

  EXPECT_TRUE(w.hasRigidBody(body1));

  w.removeRigidBody(body1);

  EXPECT_FALSE(w.hasRigidBody(body1));

  auto body2 = w.addRigidBody();
  EXPECT_FALSE(w.hasRigidBody(body1));
  EXPECT_TRUE(w.hasRigidBody(body2));
}
