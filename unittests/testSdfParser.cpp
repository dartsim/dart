/*
 * Copyright (c) 2013-2015, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Jeongseok Lee <jslee02@gmail.com>
 *
 * Georgia Tech Graphics Lab and Humanoid Robotics Lab
 *
 * Directed by Prof. C. Karen Liu and Prof. Mike Stilman
 * <karenliu@cc.gatech.edu> <mstilman@cc.gatech.edu>
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

#include <iostream>
#include <gtest/gtest.h>
#include "TestHelpers.h"

#include "dart/dynamics/SoftBodyNode.h"
#include "dart/dynamics/RevoluteJoint.h"
#include "dart/dynamics/PlanarJoint.h"
#include "dart/dynamics/Skeleton.h"
#include "dart/simulation/World.h"
#include "dart/utils/sdf/SdfParser.h"

using namespace dart;
using namespace math;
using namespace dynamics;
using namespace simulation;
using namespace utils;

//==============================================================================
TEST(SdfParser, SDFSingleBodyWithoutJoint)
{
  // Regression test for #444
  WorldPtr world
      = SdfParser::readWorld(
            DART_DATA_PATH"/sdf/test/single_bodynode_skeleton.world");
  EXPECT_TRUE(world != nullptr);

  SkeletonPtr skel = world->getSkeleton(0);
  EXPECT_TRUE(skel != nullptr);
  EXPECT_EQ(skel->getNumBodyNodes(), 1u);
  EXPECT_EQ(skel->getNumJoints(), 1u);

  BodyNodePtr bodyNode = skel->getBodyNode(0);
  EXPECT_TRUE(bodyNode != nullptr);
  EXPECT_EQ(bodyNode->getNumShapeNodesWith<VisualAddon>(), 1u);
  EXPECT_EQ(bodyNode->getNumShapeNodesWith<CollisionAddon>(), 1u);

  JointPtr joint = skel->getJoint(0);
  EXPECT_TRUE(joint != nullptr);
  EXPECT_EQ(joint->getType(), FreeJoint::getStaticType());
}

//==============================================================================
TEST(SdfParser, ParsingSDFFiles)
{
  const auto numSteps = 10u;

  // Create a list of sdf files to test with where the sdf files contains World
  std::vector<std::string> worldFiles;
  worldFiles.push_back(DART_DATA_PATH"sdf/benchmark.world");
  worldFiles.push_back(DART_DATA_PATH"sdf/double_pendulum.world");
  worldFiles.push_back(DART_DATA_PATH"sdf/double_pendulum_with_base.world");
  worldFiles.push_back(DART_DATA_PATH"sdf/empty.world");
  worldFiles.push_back(DART_DATA_PATH"sdf/ground.world");
  worldFiles.push_back(DART_DATA_PATH"sdf/test/single_bodynode_skeleton.world");

  std::vector<WorldPtr> worlds;
  for (const auto& worldFile : worldFiles)
    worlds.push_back(SdfParser::readWorld(worldFile));

  for (auto world : worlds)
  {
    EXPECT_TRUE(nullptr != world);

    for (auto i = 0u; i < numSteps; ++i)
      world->step();
  }

  // Create another list of sdf files to test with where the sdf files contains
  // Skeleton
  std::vector<std::string> skeletonFiles;
  skeletonFiles.push_back(DART_DATA_PATH"sdf/atlas/atlas_v3_no_head.sdf");
  skeletonFiles.push_back(DART_DATA_PATH"sdf/atlas/atlas_v3_no_head_soft_feet.sdf");

  auto world = std::make_shared<World>();
  std::vector<SkeletonPtr> skeletons;
  for (const auto& skeletonFile : skeletonFiles)
    skeletons.push_back(SdfParser::readSkeleton(skeletonFile));

  for (auto skeleton : skeletons)
  {
    EXPECT_TRUE(nullptr != skeleton);

    world->addSkeleton(skeleton);
    for (auto i = 0u; i < numSteps; ++i)
      world->step();

    world->removeAllSkeletons();
  }
}

//==============================================================================
int main(int argc, char* argv[])
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
