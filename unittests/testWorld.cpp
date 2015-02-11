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
TEST(WORLD, ADDING_AND_REMOVING_SKELETONS)
{
    // World
    World* world = new World;

    //-------------------- Test World::removeSkeleton() ------------------------
    Skeleton* skeleton1 = createThreeLinkRobot(Eigen::Vector3d(1.0, 1.0, 1.0),
                                               DOF_X,
                                               Eigen::Vector3d(1.0, 1.0, 1.0),
                                               DOF_Y,
                                               Eigen::Vector3d(1.0, 1.0, 1.0),
                                               DOF_Z,
                                               false, false);

    Skeleton* skeleton2 = createThreeLinkRobot(Eigen::Vector3d(1.0, 1.0, 1.0),
                                               DOF_X,
                                               Eigen::Vector3d(1.0, 1.0, 1.0),
                                               DOF_Y,
                                               Eigen::Vector3d(1.0, 1.0, 1.0),
                                               DOF_Z,
                                               false, false);

    Skeleton* skeleton3 = createThreeLinkRobot(Eigen::Vector3d(1.0, 1.0, 1.0),
                                               DOF_X,
                                               Eigen::Vector3d(1.0, 1.0, 1.0),
                                               DOF_Y,
                                               Eigen::Vector3d(1.0, 1.0, 1.0),
                                               DOF_Z,
                                               false, false);

    Skeleton* skeleton4 = createThreeLinkRobot(Eigen::Vector3d(1.0, 1.0, 1.0),
                                               DOF_X,
                                               Eigen::Vector3d(1.0, 1.0, 1.0),
                                               DOF_Y,
                                               Eigen::Vector3d(1.0, 1.0, 1.0),
                                               DOF_Z,
                                               false, false);

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

    std::string s1name = skeleton1->getName();
    std::string s2name = skeleton2->getName();
    EXPECT_TRUE(skeleton1 == world->getSkeleton(s1name));
    EXPECT_TRUE(skeleton2 == world->getSkeleton(s2name));

    // Remove skeleton2
    world->removeSkeleton(skeleton2);
    EXPECT_TRUE(world->getNumSkeletons() == 1);
    for (int i = 0; i < nSteps; ++i)
        world->step();

    EXPECT_TRUE(skeleton1 == world->getSkeleton(s1name));
    EXPECT_FALSE(skeleton2 == world->getSkeleton(s2name));
    EXPECT_TRUE(world->getSkeleton(s2name) == NULL);

    // Add skeleton3, skeleton4
    world->addSkeleton(skeleton3);
    world->addSkeleton(skeleton4);
    EXPECT_TRUE(world->getNumSkeletons() == 3);
    for (int i = 0; i < nSteps; ++i)
        world->step();

    std::string s3name = skeleton3->getName();
    std::string s4name = skeleton4->getName();

    EXPECT_TRUE(s3name == s2name);
    EXPECT_TRUE(skeleton3 == world->getSkeleton(s3name));
    EXPECT_TRUE(skeleton4 == world->getSkeleton(s4name));

    // Remove skeleton1
    world->removeSkeleton(skeleton1);
    EXPECT_TRUE(world->getNumSkeletons() == 2);
    for (int i = 0; i < nSteps; ++i)
        world->step();

    EXPECT_FALSE(skeleton1 == world->getSkeleton(s1name));
    EXPECT_TRUE(world->getSkeleton(s1name) == NULL);

    // Remove all the skeletons
    world->removeAllSkeletons();
    EXPECT_EQ(world->getNumSkeletons(), 0);
    for (int i = 0; i < nSteps; ++i)
        world->step();

    EXPECT_FALSE(skeleton3 == world->getSkeleton(s3name));
    EXPECT_TRUE(world->getSkeleton(s3name) == NULL);

    EXPECT_FALSE(skeleton4 == world->getSkeleton(s4name));
    EXPECT_TRUE(world->getSkeleton(s4name) == NULL);

    delete world;
}

/******************************************************************************/
int main(int argc, char* argv[])
{
	::testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
