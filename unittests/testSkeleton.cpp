/*
 * Copyright (c) 2015, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Michael X. Grey <mxgrey@gatech.edu>
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

#include "dart/math/Geometry.h"
#include "dart/utils/SkelParser.h"
#include "dart/dynamics/BodyNode.h"
#include "dart/dynamics/RevoluteJoint.h"
#include "dart/dynamics/Skeleton.h"
#include "dart/simulation/World.h"

using namespace dart;
using namespace math;
using namespace dynamics;
using namespace simulation;

TEST(Skeleton, Restructuring)
{
  std::vector<std::string> fileList;
  fileList.push_back(DART_DATA_PATH"skel/test/chainwhipa.skel");
  fileList.push_back(DART_DATA_PATH"skel/test/single_pendulum.skel");
  fileList.push_back(DART_DATA_PATH"skel/test/single_pendulum_euler_joint.skel");
  fileList.push_back(DART_DATA_PATH"skel/test/single_pendulum_ball_joint.skel");
  fileList.push_back(DART_DATA_PATH"skel/test/double_pendulum.skel");
  fileList.push_back(DART_DATA_PATH"skel/test/double_pendulum_euler_joint.skel");
  fileList.push_back(DART_DATA_PATH"skel/test/double_pendulum_ball_joint.skel");
  fileList.push_back(DART_DATA_PATH"skel/test/serial_chain_revolute_joint.skel");
  fileList.push_back(DART_DATA_PATH"skel/test/serial_chain_eulerxyz_joint.skel");
  fileList.push_back(DART_DATA_PATH"skel/test/serial_chain_ball_joint.skel");
  fileList.push_back(DART_DATA_PATH"skel/test/serial_chain_ball_joint_20.skel");
  fileList.push_back(DART_DATA_PATH"skel/test/serial_chain_ball_joint_40.skel");
  fileList.push_back(DART_DATA_PATH"skel/test/simple_tree_structure.skel");
  fileList.push_back(DART_DATA_PATH"skel/test/simple_tree_structure_euler_joint.skel");
  fileList.push_back(DART_DATA_PATH"skel/test/simple_tree_structure_ball_joint.skel");
  fileList.push_back(DART_DATA_PATH"skel/test/tree_structure.skel");
  fileList.push_back(DART_DATA_PATH"skel/test/tree_structure_euler_joint.skel");
  fileList.push_back(DART_DATA_PATH"skel/test/tree_structure_ball_joint.skel");
  fileList.push_back(DART_DATA_PATH"skel/fullbody1.skel");

  std::vector<WorldPtr> worlds;
  for(size_t i=0; i<fileList.size(); ++i)
    worlds.push_back(dart::utils::SkelParser::readWorld(fileList[i]));

  std::vector<SkeletonPtr> skeletons;
  for(size_t i=0; i<worlds.size(); ++i)
  {
    WorldPtr world = worlds[i];
    for(size_t j=0; j<world->getNumSkeletons(); ++j)
      skeletons.push_back(world->getSkeleton(j));
  }

#ifndef NDEBUG
  size_t numIterations = 10;
#else
  size_t numIterations = 2*skeletons.size();
#endif

  // Test moves within the current Skeleton
  for(size_t i=0; i<numIterations; ++i)
  {
    size_t index = floor(math::random(0, skeletons.size()));
    index = std::min(index, skeletons.size()-1);
    SkeletonPtr skeleton = skeletons[index];
    SkeletonPtr original = skeleton->clone();

    size_t maxNode = skeleton->getNumBodyNodes()-1;
    BodyNode* bn1 = skeleton->getBodyNode(floor(math::random(0, maxNode)));
    BodyNode* bn2 = skeleton->getBodyNode(ceil(math::random(0, maxNode)));

    if(bn1 == bn2)
    {
      --i;
      continue;
    }

    BodyNode* child = bn1->descendsFrom(bn2)? bn1 : bn2;
    BodyNode* parent = child == bn1? bn2 : bn1;

    child->moveTo(parent);

    EXPECT_TRUE(skeleton->getNumBodyNodes() == original->getNumBodyNodes());
    if(skeleton->getNumBodyNodes() == original->getNumBodyNodes())
    {
      for(size_t j=0; j<skeleton->getNumBodyNodes(); ++j)
      {
        // Make sure no BodyNodes have been lost or gained in translation
        std::string name = original->getBodyNode(j)->getName();
        BodyNode* bn = skeleton->getBodyNode(name);
        EXPECT_FALSE(bn == nullptr);
        if(bn)
          EXPECT_TRUE(bn->getName() == name);

        name = skeleton->getBodyNode(j)->getName();
        bn = original->getBodyNode(name);
        EXPECT_FALSE(bn == nullptr);
        if(bn)
          EXPECT_TRUE(bn->getName() == name);


        // Make sure no Joints have been lost or gained in translation
        name = original->getJoint(j)->getName();
        Joint* joint = skeleton->getJoint(name);
        EXPECT_FALSE(joint == nullptr);
        if(joint)
          EXPECT_TRUE(joint->getName() == name);

        name = skeleton->getJoint(j)->getName();
        joint = original->getJoint(name);
        EXPECT_FALSE(joint == nullptr);
        if(joint)
          EXPECT_TRUE(joint->getName() == name);
      }
    }

    EXPECT_TRUE(skeleton->getNumDofs() == original->getNumDofs());
    for(size_t j=0; j<skeleton->getNumDofs(); ++j)
    {
      std::string name = original->getDof(j)->getName();
      DegreeOfFreedom* dof = skeleton->getDof(name);
      EXPECT_FALSE(dof == nullptr);
      if(dof)
        EXPECT_TRUE(dof->getName() == name);

      name = skeleton->getDof(j)->getName();
      dof = original->getDof(name);
      EXPECT_FALSE(dof == nullptr);
      if(dof)
        EXPECT_TRUE(dof->getName() == name);
    }
  }
}

int main(int argc, char* argv[])
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
