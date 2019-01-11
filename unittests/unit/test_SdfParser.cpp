/*
 * Copyright (c) 2011-2019, The DART development contributors
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

#include <iostream>
#include <gtest/gtest.h>
#include "TestHelpers.hpp"

#include "dart/dynamics/SoftBodyNode.hpp"
#include "dart/dynamics/RevoluteJoint.hpp"
#include "dart/dynamics/PlanarJoint.hpp"
#include "dart/dynamics/Skeleton.hpp"
#include "dart/simulation/World.hpp"
#include "dart/io/sdf/SdfParser.hpp"

using namespace dart;
using namespace math;
using namespace dynamics;
using namespace simulation;
using namespace io;

//==============================================================================
TEST(SdfParser, SDFSingleBodyWithoutJoint)
{
  // Regression test for #444
  WorldPtr world
      = SdfParser::readWorld(
            "dart://sample/sdf/test/single_bodynode_skeleton.world");
  EXPECT_TRUE(world != nullptr);

  SkeletonPtr skel = world->getSkeleton(0);
  EXPECT_TRUE(skel != nullptr);
  EXPECT_EQ(skel->getNumBodyNodes(), 1u);
  EXPECT_EQ(skel->getNumJoints(), 1u);

  BodyNodePtr bodyNode = skel->getBodyNode(0);
  EXPECT_TRUE(bodyNode != nullptr);
  EXPECT_EQ(bodyNode->getNumShapeNodesWith<VisualAspect>(), 1u);
  EXPECT_EQ(bodyNode->getNumShapeNodesWith<CollisionAspect>(), 1u);

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
  worldFiles.push_back("dart://sample/sdf/benchmark.world");
  worldFiles.push_back("dart://sample/sdf/double_pendulum.world");
  worldFiles.push_back("dart://sample/sdf/double_pendulum_with_base.world");
  worldFiles.push_back("dart://sample/sdf/empty.world");
  worldFiles.push_back("dart://sample/sdf/ground.world");
  worldFiles.push_back("dart://sample/sdf/test/single_bodynode_skeleton.world");

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
  std::vector<common::Uri> skeletonFiles;
  skeletonFiles.push_back("dart://sample/sdf/atlas/atlas_v3_no_head.sdf");
  skeletonFiles.push_back("dart://sample/sdf/atlas/atlas_v3_no_head_soft_feet.sdf");

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
TEST(SdfParser, ReadMaterial)
{
  std::string sdf_filename = "dart://sample/sdf/quad.sdf";
  SkeletonPtr skeleton = SdfParser::readSkeleton(sdf_filename);
  EXPECT_TRUE(nullptr != skeleton);  auto bodynode = skeleton->getBodyNode(0);

  for (auto shapenode : bodynode->getShapeNodes()) {
    if (shapenode->has<dart::dynamics::VisualAspect>()) {
      Eigen::Vector4d color = shapenode->getVisualAspect()->getRGBA();
      Eigen::Vector4d expected_color(0.5, 0.6, 0.8, 1.0);
      double diff = (color - expected_color).norm();
      EXPECT_LT(diff, 1e-4);
    }
  }

}
