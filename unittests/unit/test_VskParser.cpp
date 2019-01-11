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
#include "dart/simulation/World.hpp"
#include "dart/io/VskParser.hpp"

using namespace dart;
using namespace math;
using namespace dynamics;
using namespace simulation;
using namespace io;

//==============================================================================
TEST(VskParser, EmptySkeleton)
{
  WorldPtr world = World::create();
  EXPECT_TRUE(world != nullptr);

  SkeletonPtr skeleton
      = VskParser::readSkeleton("dart://sample/vsk/test/empty.vsk");
  EXPECT_TRUE(skeleton == nullptr);

  world->addSkeleton(skeleton);
  EXPECT_EQ(world->getNumSkeletons(), 0u);

  world->step();
}

//==============================================================================
TEST(VskParser, SingleStepSimulations)
{
  WorldPtr world = World::create();
  EXPECT_NE(world , nullptr);

  SkeletonPtr nick
      = VskParser::readSkeleton("dart://sample/vsk/Nick01.vsk");
  EXPECT_NE(nick  , nullptr);
  EXPECT_EQ(nick->getNumMarkers(), 53u);

  SkeletonPtr sehoon
      = VskParser::readSkeleton("dart://sample/vsk/SehoonVSK3.vsk");
  EXPECT_NE(sehoon, nullptr);
  EXPECT_EQ(nick->getNumMarkers(), 53u);

  SkeletonPtr yuting
      = VskParser::readSkeleton("dart://sample/vsk/Yuting.vsk");
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
