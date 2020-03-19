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

#include <gtest/gtest.h>
#include "dart/dart.hpp"

#include "TestHelpers.hpp"

using namespace dart::math;
using namespace dart::collision;
using namespace dart::dynamics;
using namespace dart::simulation;

//==============================================================================
TEST(Issue1193, AngularVelAdd)
{
  WorldPtr world = World::create();
  EXPECT_TRUE(world != nullptr);
  world->setGravity(Eigen::Vector3d(0.0, -10.0, 0.0));
  const double dt = 0.001;
  world->setTimeStep(dt);

  SkeletonPtr sphereSkel = createSphere(0.05, Vector3d(0.0, 1.0, 0.0));
  BodyNode* sphere = sphereSkel->getBodyNode(0);
  Joint* sphereJoint = sphere->getParentJoint();
  sphereJoint->setVelocity(0, 10.0);  // ang_x -> Affect lz and ly
  sphereJoint->setVelocity(1, 10.0);  // ang_y -> No effect
  sphereJoint->setVelocity(2, 10.0);  // ang_z -> Affect lx and ly
  world->addSkeleton(sphereSkel);

  Eigen::Vector3d linearVelBefore = sphere->getLinearVelocity();
  EXPECT_EQ(Vector3d::Zero(), linearVelBefore);

  int maxSteps = 500;
  for (int i = 0; i < maxSteps; i++)
  {
    world->step();
  }

  Vector3d linearVelAfter = sphere->getLinearVelocity();
  double lx = linearVelAfter[0];
  double ly = linearVelAfter[1];
  double lz = linearVelAfter[2];

  EXPECT_NEAR(0.0, lx, 1e-8);
  EXPECT_NEAR(maxSteps * world->getGravity().y() * dt, ly, 1e-8);
  EXPECT_NEAR(0.0, lz, 1e-8);
}
