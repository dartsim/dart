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

#include "helpers/GTestUtils.hpp"

#include "dart/dynamics/FreeJoint.hpp"
#include "dart/dynamics/RevoluteJoint.hpp"
#include "dart/dynamics/Skeleton.hpp"
#include "dart/simulation/World.hpp"
#include "dart/utils/usd/UsdParser.hpp"

#include <gtest/gtest.h>

using namespace dart;
using namespace dart::utils;
using namespace dart::simulation;

TEST(UsdParser, SimpleChainSkeleton)
{
  auto skeleton
      = UsdParser::readSkeleton("dart://sample/usd/simple_chain.usda");
  ASSERT_NE(nullptr, skeleton);
  EXPECT_EQ(skeleton->getNumBodyNodes(), 2u);
  EXPECT_EQ(skeleton->getNumJoints(), 2u);

  const auto* rootJoint = skeleton->getJoint(0);
  ASSERT_NE(nullptr, rootJoint);
  EXPECT_EQ(rootJoint->getType(), dynamics::FreeJoint::getStaticType());

  const auto* shoulder = skeleton->getJoint("shoulder");
  ASSERT_NE(nullptr, shoulder);
  EXPECT_EQ(shoulder->getType(), dynamics::RevoluteJoint::getStaticType());
}

TEST(UsdParser, HumanoidWorld)
{
  auto world
      = UsdParser::readWorld("dart://sample/usd/unitree_h1_minimal.usda");
  ASSERT_NE(nullptr, world);
  ASSERT_EQ(world->getNumSkeletons(), 1u);
  const auto skeleton = world->getSkeleton(0);
  ASSERT_NE(nullptr, skeleton);

  EXPECT_GE(skeleton->getNumBodyNodes(), 10u);
  EXPECT_GE(skeleton->getNumJoints(), 10u);

  // Ensure that left leg chain exists and is connected.
  auto leftThigh = skeleton->getBodyNode("World_UnitreeH1_left_thigh");
  auto leftShin = skeleton->getBodyNode("World_UnitreeH1_left_shin");
  auto leftFoot = skeleton->getBodyNode("World_UnitreeH1_left_foot");
  ASSERT_NE(nullptr, leftThigh);
  ASSERT_NE(nullptr, leftShin);
  ASSERT_NE(nullptr, leftFoot);
  EXPECT_EQ(leftShin->getParentBodyNode(), leftThigh);
  EXPECT_EQ(leftFoot->getParentBodyNode(), leftShin);
}
