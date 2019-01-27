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

#include "dart/math/Geometry.hpp"
#include "dart/dynamics/BodyNode.hpp"
#include "dart/dynamics/RevoluteJoint.hpp"
#include "dart/dynamics/Skeleton.hpp"
#include "dart/simulation/World.hpp"

using namespace dart;
using namespace math;
using namespace dynamics;
using namespace simulation;

/******************************************************************************/
TEST(BUILDING, BASIC)
{
  // Skeletons
  SkeletonPtr skel1 = Skeleton::create();

  std::pair<RevoluteJoint*, BodyNode*> pair;
  BodyNode* body1, * body2, * body3;
  RevoluteJoint* joint1, * joint2, * joint3;

  pair = skel1->createJointAndBodyNodePair<RevoluteJoint>(nullptr);
  joint1 = pair.first;
  body1 = pair.second;

  pair = body1->createChildJointAndBodyNodePair<RevoluteJoint>();
  joint2 = pair.first;
  body2 = pair.second;

  pair = body2->createChildJointAndBodyNodePair<RevoluteJoint>();
  joint3 = pair.first;
  body3 = pair.second;

	// Joints
  joint1->setTransformFromParentBodyNode(Eigen::Isometry3d::Identity());
  joint1->setTransformFromChildBodyNode(Eigen::Isometry3d::Identity());
  joint1->setAxis(Eigen::Vector3d(1.0, 0.0, 0.0));

  joint2->setTransformFromParentBodyNode(Eigen::Isometry3d::Identity());
  joint2->setTransformFromChildBodyNode(Eigen::Isometry3d::Identity());
  joint2->setAxis(Eigen::Vector3d(1.0, 0.0, 0.0));

  joint3->setTransformFromParentBodyNode(Eigen::Isometry3d::Identity());
  joint3->setTransformFromChildBodyNode(Eigen::Isometry3d::Identity());
  joint3->setAxis(Eigen::Vector3d(1.0, 0.0, 0.0));

  // World
  WorldPtr world = World::create();
  world->addSkeleton(skel1);

	//--------------------------------------------------------------------------
	//
	//--------------------------------------------------------------------------
  EXPECT_TRUE(body1->getParentBodyNode() == nullptr);
  EXPECT_TRUE(body1->getNumChildBodyNodes() == 1);
  EXPECT_TRUE(body1->getChildBodyNode(0) == body2);

  EXPECT_TRUE(body2->getParentBodyNode() == body1);
  EXPECT_TRUE(body2->getNumChildBodyNodes() == 1);
  EXPECT_TRUE(body2->getChildBodyNode(0) == body3);

  EXPECT_TRUE(body3->getParentBodyNode() == body2);
  EXPECT_TRUE(body3->getNumChildBodyNodes() == 0);
//  EXPECT_TRUE(body3->getChildBodyNode(0) == nullptr);

  EXPECT_TRUE(skel1->getNumBodyNodes() == 3);
  EXPECT_TRUE(skel1->getNumDofs() == 3);

  EXPECT_TRUE(world->getNumSkeletons() == 1);

  int nSteps = 20;
  for (int i = 0; i < nSteps; ++i)
      world->step();
}
