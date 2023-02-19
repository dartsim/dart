/*
 * Copyright (c) 2011-2023, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/master/LICENSE
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

#include "dart/physics/physics.hpp"

#include <gtest/gtest.h>

using namespace dart;
using namespace physics;

template <typename S>
struct PhysicsEmptyTest : public testing::Test
{
  using Scalar = S;
};

using Types = testing::Types<float, double>;
TYPED_TEST_SUITE(PhysicsEmptyTest, Types);

//==============================================================================
TYPED_TEST(PhysicsEmptyTest, Empty)
{
  using S = typename TestFixture::Scalar;

  auto world = World<S>();

  auto mb_1 = world.createMultiBody();
  EXPECT_TRUE(mb_1 != nullptr);
  EXPECT_EQ(mb_1->getNumDofs(), 0);

  auto mb_2 = world.createMultiBody();
  EXPECT_TRUE(mb_2 != nullptr);
  EXPECT_EQ(mb_2->getNumDofs(), 0);

  auto pos = mb_1->getPositions();
  EXPECT_EQ(pos.size(), 0);

  world.setSimulationMode();
}

//==============================================================================
TYPED_TEST(PhysicsEmptyTest, CreateLink)
{
  using S = typename TestFixture::Scalar;

  auto world = World<S>();

  auto mb_1 = world.createMultiBody();

  auto link_1 = mb_1->createLink();
  EXPECT_TRUE(link_1 != nullptr);
}
