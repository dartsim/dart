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

#include "dart/dynamics/SoftBodyNode.h"
#include "dart/dynamics/RevoluteJoint.h"
#include "dart/dynamics/Skeleton.h"
#include "dart/dynamics/SoftSkeleton.h"
#include "dart/utils/Paths.h"
#include "dart/simulation/World.h"
#include "dart/simulation/SoftWorld.h"
#include "dart/utils/SkelParser.h"
#include "dart/utils/SoftParser.h"

using namespace dart;
using namespace math;
using namespace dynamics;
using namespace simulation;
using namespace utils;

/******************************************************************************/
TEST(SKEL_PARSER, DATA_STRUCTUER)
{
    bool v1 = true;
    int v2 = -3;
    unsigned int v3 = 1;
    float v4 = -3.140f;
    double v5 = 1.4576640;
    char v6 = 'd';
    Eigen::Vector2d v7 = Eigen::Vector2d::Ones();
    Eigen::Vector3d v8 = Eigen::Vector3d::Ones();
    //Eigen::Vector3d v9 = Eigen::Vector3d::Ones();
    //math::SO3 v10;
    Eigen::Isometry3d v11 = Eigen::Isometry3d::Identity();

    std::string str1 = toString(v1);
    std::string str2 = toString(v2);
    std::string str3 = toString(v3);
    std::string str4 = toString(v4);
    std::string str5 = toString(v5);
    std::string str6 = toString(v6);
    std::string str7 = toString(v7);
    std::string str8 = toString(v8);
    //std::string str9 = toString(v9);
    //std::string str10 = toString(v10);
    std::string str11 = toString(v11);

    bool b = toBool(str1);
    int i = toInt(str2);
    unsigned int ui = toUInt(str3);
    float f = toFloat(str4);
    double d = toDouble(str5);
    char c = toChar(str6);
    Eigen::Vector2d vec2 = toVector2d(str7);
    Eigen::Vector3d vec3 = toVector3d(str8);
    //Eigen::Vector3d valso3 = toVector3d(str9);
    //math::SO3 valSO3 = toSO3(str10);
    Eigen::Isometry3d valSE3 = toIsometry3d(str11);

    EXPECT_EQ(b, v1);
    EXPECT_EQ(i, v2);
    EXPECT_EQ(ui, v3);
    EXPECT_EQ(f, v4);
    EXPECT_EQ(d, v5);
    EXPECT_EQ(c, v6);
    for (int i = 0; i < 2; i++) {
        EXPECT_EQ(vec2[i], v7[i]);
    }
    EXPECT_EQ(vec3, v8);
    //EXPECT_EQ(valso3, v9);
    //EXPECT_EQ(valSO3, v10);
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j)
            EXPECT_EQ(valSE3(i,j), v11(i,j));
}

TEST(SKEL_PARSER, EMPTY)
{
    World* world = SkelParser::readSkelFile(DART_DATA_PATH"skel/test/empty.skel");

    EXPECT_TRUE(world != NULL);
    EXPECT_EQ(world->getTimeStep(), 0.001);
    EXPECT_EQ(world->getGravity()(0), 0);
    EXPECT_EQ(world->getGravity()(1), 0);
    EXPECT_EQ(world->getGravity()(2), -9.81);
    EXPECT_EQ(world->getNumSkeletons(), 0);

    EXPECT_EQ(world->getTime(), 0);
    world->step();
    EXPECT_EQ(world->getTime(), world->getTimeStep());

    delete world;
}

TEST(SKEL_PARSER, PENDULUM)
{
    World* world = SkelParser::readSkelFile(DART_DATA_PATH"skel/test/single_pendulum.skel");

    EXPECT_TRUE(world != NULL);
    EXPECT_EQ(world->getTimeStep(), 0.001);
    EXPECT_EQ(world->getGravity()(0), 0);
    EXPECT_EQ(world->getGravity()(1), -9.81);
    EXPECT_EQ(world->getGravity()(2), 0);
    EXPECT_EQ(world->getNumSkeletons(), 1);

    Skeleton* skel1 = world->getSkeleton("single_pendulum");

    EXPECT_EQ(skel1->getNumBodyNodes(), 1);

    world->step();

    delete world;
}

TEST(SKEL_PARSER, SERIAL_CAHIN)
{
    World* world = SkelParser::readSkelFile(DART_DATA_PATH"skel/test/serial_chain_ball_joint.skel");

    EXPECT_TRUE(world != NULL);
    EXPECT_EQ(world->getTimeStep(), 0.001);
    EXPECT_EQ(world->getGravity()(0), 0);
    EXPECT_EQ(world->getGravity()(1), -9.81);
    EXPECT_EQ(world->getGravity()(2), 0);
    EXPECT_EQ(world->getNumSkeletons(), 1);

    Skeleton* skel1 = world->getSkeleton("skeleton 1");

    EXPECT_EQ(skel1->getNumBodyNodes(), 10);

    world->step();

    delete world;
}

TEST(SKEL_PARSER, RIGID_SOFT_BODIES)
{
  using namespace dart;
  using namespace math;
  using namespace dynamics;
  using namespace simulation;
  using namespace utils;

  SoftWorld* softWorld
      = SoftSkelParser::readSoftFile(
          DART_DATA_PATH"skel/test/test_articulated_bodies.skel");
  EXPECT_TRUE(softWorld != NULL);

  Skeleton* skel1 = softWorld->getSkeleton("skeleton 1");
  SoftSkeleton* softSkel1 = dynamic_cast<SoftSkeleton*>(skel1);
  EXPECT_TRUE(softSkel1 != NULL);
  EXPECT_EQ(softSkel1->getNumBodyNodes(), 2);
  EXPECT_EQ(softSkel1->getNumRigidBodyNodes(), 1);
  EXPECT_EQ(softSkel1->getNumSoftBodyNodes(), 1);

  SoftBodyNode* sbn = softSkel1->getSoftBodyNode(0);
  EXPECT_TRUE(sbn->getNumPointMasses() > 0);

  softWorld->step();

  delete softWorld;
}

/******************************************************************************/
int main(int argc, char* argv[])
{
	::testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
