/*
 * Copyright (c) 2013, Georgia Tech Research Corporation
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

#include "dart/math/Geometry.h"
#include "dart/dynamics/BodyNode.h"
#include "dart/dynamics/RevoluteJoint.h"
#include "dart/dynamics/Skeleton.h"
#include "dart/simulation/World.h"

using namespace dart;
using namespace math;
using namespace dynamics;
using namespace simulation;

/******************************************************************************/
TEST(BUILDING, BASIC)
{
	//--------------------------------------------------------------------------
	//
	//--------------------------------------------------------------------------
	// Bodies
    BodyNode* body1 = new BodyNode;
    BodyNode* body2 = new BodyNode;
    BodyNode* body3 = new BodyNode;

	// Joints
    RevoluteJoint* joint1 = new RevoluteJoint;
    RevoluteJoint* joint2 = new RevoluteJoint;
    RevoluteJoint* joint3 = new RevoluteJoint;

	// Skeletons
    Skeleton* skel1 = new Skeleton;

	// World
    World* world = new World;

	//--------------------------------------------------------------------------
	//
	//--------------------------------------------------------------------------
	// Bodies
    body1->addChildBodyNode(body2);
    body2->addChildBodyNode(body3);

    body1->setParentJoint(joint1);
    body2->setParentJoint(joint2);
    body3->setParentJoint(joint3);

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

	// Skeleton
    skel1->addBodyNode(body1);
    skel1->addBodyNode(body2);
    skel1->addBodyNode(body3);

	// World
    world->addSkeleton(skel1);

	//--------------------------------------------------------------------------
	//
	//--------------------------------------------------------------------------
    EXPECT_TRUE(body1->getParentBodyNode() == NULL);
    EXPECT_TRUE(body1->getNumChildBodyNodes() == 1);
    EXPECT_TRUE(body1->getChildBodyNode(0) == body2);

    EXPECT_TRUE(body2->getParentBodyNode() == body1);
    EXPECT_TRUE(body2->getNumChildBodyNodes() == 1);
    EXPECT_TRUE(body2->getChildBodyNode(0) == body3);

    EXPECT_TRUE(body3->getParentBodyNode() == body2);
    EXPECT_TRUE(body3->getNumChildBodyNodes() == 0);
    //EXPECT_TRUE(body3.getChildBodyNode(0) == NULL);

    EXPECT_TRUE(skel1->getNumBodyNodes() == 3);
    EXPECT_TRUE(skel1->getNumGenCoords() == 3);

    EXPECT_TRUE(world->getNumSkeletons() == 1);

    int nSteps = 20;
    for (int i = 0; i < nSteps; ++i)
        world->step();

    delete world;
}

/******************************************************************************/
TEST(BUILDING, ADDING_AND_REMOVING_SKELETONS)
{
    // World
    World* world = new World;

    Skeleton* skeleton1 = createThreeLinkRobot(Eigen::Vector3d(1.0, 1.0, 1.0),
                                               DOF_X,
                                               Eigen::Vector3d(1.0, 1.0, 1.0),
                                               DOF_Y,
                                               Eigen::Vector3d(1.0, 1.0, 1.0),
                                               DOF_Z);

    Skeleton* skeleton2 = createThreeLinkRobot(Eigen::Vector3d(1.0, 1.0, 1.0),
                                               DOF_X,
                                               Eigen::Vector3d(1.0, 1.0, 1.0),
                                               DOF_Y,
                                               Eigen::Vector3d(1.0, 1.0, 1.0),
                                               DOF_Z);

    Skeleton* skeleton3 = createThreeLinkRobot(Eigen::Vector3d(1.0, 1.0, 1.0),
                                               DOF_X,
                                               Eigen::Vector3d(1.0, 1.0, 1.0),
                                               DOF_Y,
                                               Eigen::Vector3d(1.0, 1.0, 1.0),
                                               DOF_Z);

    Skeleton* skeleton4 = createThreeLinkRobot(Eigen::Vector3d(1.0, 1.0, 1.0),
                                               DOF_X,
                                               Eigen::Vector3d(1.0, 1.0, 1.0),
                                               DOF_Y,
                                               Eigen::Vector3d(1.0, 1.0, 1.0),
                                               DOF_Z);

    int nSteps = 20;

    // Empty world
    for (int i = 0; i < nSteps; ++i)
        world->step();

    // Add skeleton1, skeleton2
    world->addSkeleton(skeleton1);
    world->addSkeleton(skeleton2);
    EXPECT_TRUE(world->getNumSkeletons() == 2);
    for (int i = 0; i < nSteps; ++i)
        world->step();

    // Remove skeleton2
    world->removeSkeleton(skeleton2);
    EXPECT_TRUE(world->getNumSkeletons() == 1);
    for (int i = 0; i < nSteps; ++i)
        world->step();

    // Add skeleton3, skeleton4
    world->addSkeleton(skeleton3);
    world->addSkeleton(skeleton4);
    EXPECT_TRUE(world->getNumSkeletons() == 3);
    for (int i = 0; i < nSteps; ++i)
        world->step();

    // Remove skeleton1
    world->removeSkeleton(skeleton1);
    EXPECT_TRUE(world->getNumSkeletons() == 2);
    for (int i = 0; i < nSteps; ++i)
        world->step();

    delete world;
}

/******************************************************************************/
int main(int argc, char* argv[])
{
	::testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
