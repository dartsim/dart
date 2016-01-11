/*
 * Copyright (c) 2015, Georgia Tech Research Corporation
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

#include "tinyxml2.h"

#include "dart/dynamics/SoftBodyNode.h"
#include "dart/dynamics/RevoluteJoint.h"
#include "dart/dynamics/PlanarJoint.h"
#include "dart/dynamics/Skeleton.h"
#include "dart/simulation/World.h"
#include "dart/simulation/World.h"
#include "dart/utils/VskParser.h"

using namespace kido;
using namespace math;
using namespace dynamics;
using namespace simulation;
using namespace utils;

//==============================================================================
TEST(VskParser, EmptySkeleton)
{
  WorldPtr world(new World());
  EXPECT_TRUE(world != nullptr);

  SkeletonPtr skeleton
      = VskParser::readSkeleton(KIDO_DATA_PATH"vsk/test/empty.vsk");
  EXPECT_TRUE(skeleton == nullptr);

  world->addSkeleton(skeleton);
  EXPECT_EQ(world->getNumSkeletons(), 0u);

  world->step();
}

//==============================================================================
TEST(VskParser, SingleStepSimulations)
{
  WorldPtr world(new World());
  EXPECT_NE(world , nullptr);

  SkeletonPtr nick
      = VskParser::readSkeleton(KIDO_DATA_PATH"vsk/Nick01.vsk");
  EXPECT_NE(nick  , nullptr);
  EXPECT_EQ(nick->getNumMarkers(), 53u);

  SkeletonPtr sehoon
      = VskParser::readSkeleton(KIDO_DATA_PATH"vsk/SehoonVSK3.vsk");
  EXPECT_NE(sehoon, nullptr);
  EXPECT_EQ(nick->getNumMarkers(), 53u);

  SkeletonPtr yuting
      = VskParser::readSkeleton(KIDO_DATA_PATH"vsk/Yuting.vsk");
  EXPECT_NE(yuting, nullptr);
  EXPECT_EQ(nick->getNumMarkers(), 53u);

  world->removeAllSkeletons();
  world->addSkeleton(nick);
  EXPECT_EQ(world->getNumSkeletons(), 1u);
  world->step();

  world->removeAllSkeletons();
  world->addSkeleton(sehoon);
  EXPECT_EQ(world->getNumSkeletons(), 1u);
  world->step();

  world->removeAllSkeletons();
  world->addSkeleton(yuting);
  EXPECT_EQ(world->getNumSkeletons(), 1u);
  world->step();
}

//==============================================================================
int main(int argc, char* argv[])
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
