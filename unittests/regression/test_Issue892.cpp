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

#include <TestHelpers.hpp>
#include <dart/dart.hpp>
#include <dart/utils/urdf/DartLoader.hpp>
#include <gtest/gtest.h>

using namespace dart::collision;
using namespace dart::dynamics;
using namespace dart::simulation;

//==============================================================================
SkeletonPtr createBox(double offset, double angle, std::string name)
{
  SkeletonPtr box = Skeleton::create(name);
  RevoluteJoint::Properties props;
  props.mAxis = Eigen::Vector3d::UnitY();
  props.mT_ParentBodyToJoint.translation() = Eigen::Vector3d(offset, 0.0, 0.0);
  props.mT_ChildBodyToJoint.translation() = Eigen::Vector3d(0.0, 0.0, -0.4);

  BodyNode* bn
      = box->createJointAndBodyNodePair<RevoluteJoint>(nullptr, props).second;

  auto shape = std::make_shared<BoxShape>(Eigen::Vector3d(0.2, 0.1, 1.0));
  bn->createShapeNodeWith<VisualAspect, CollisionAspect, DynamicsAspect>(shape);

  box->getDof(0)->setPosition(-angle);

  return box;
}

//==============================================================================
TEST(Issue892, LCPSolverShouldNotRetrunNanValues)
{
  // Set more than 1100 steps so that the two boxes are in contact.
  const auto numSteps = 2000u;

  auto box1 = createBox(+0.4, +0.1, "box0");
  auto box2 = createBox(-0.4, -0.1, "box1");

  WorldPtr world = World::create();
  world->addSkeleton(box1);
  world->addSkeleton(box2);

  for (auto i = 0u; i < numSteps; ++i)
  {
    world->step();
    EXPECT_FALSE(box1->getRootJoint()->getPositions().hasNaN());
    EXPECT_FALSE(box2->getRootJoint()->getPositions().hasNaN());

    if (i > 1100)
    {
      const CollisionResult result = world->getLastCollisionResult();
      EXPECT_TRUE(result.isCollision());

      auto angle1 = box1->getRootJoint()->getPosition(0);
      auto angle2 = box1->getRootJoint()->getPosition(0);

      EXPECT_NEAR(angle1, -0.345858, 0.1);
      EXPECT_NEAR(angle2, -0.345858, 0.1);
    }
  }
}
