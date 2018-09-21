/*
 * Copyright (c) 2011-2018, The DART development contributors
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

#include "dart/dynamics/FreeJoint.hpp"
#include "dart/dynamics/BoxShape.hpp"
#include "dart/dynamics/Skeleton.hpp"

#include "dart/simulation/World.hpp"
#include "dart/constraint/ConstraintSolver.hpp"


class CollisionGroupsTest
    : public testing::Test,
      public testing::WithParamInterface<const char*>
{

};

TEST_P(CollisionGroupsTest, WorldSubscription)
{
  if(!dart::collision::CollisionDetector::getFactory()->canCreate(GetParam()))
  {
    std::cout << "Skipping test for [" << GetParam() << "], because it is not "
              << "available" << std::endl;
    return;
  }
  else
  {
    std::cout << "Running CollisionGroups test for [" << GetParam() << "]"
              << std::endl;
  }

  dart::simulation::WorldPtr world = dart::simulation::World::create();
  world->getConstraintSolver()->setCollisionDetector(
        dart::collision::CollisionDetector::getFactory()->create(GetParam()));

  dart::dynamics::SkeletonPtr skel_A = dart::dynamics::Skeleton::create("A");
  dart::dynamics::SkeletonPtr skel_B = dart::dynamics::Skeleton::create("B");

  world->addSkeleton(skel_A);
  world->addSkeleton(skel_B);

  // There should be no collisions because there are no shapes in the world.
  EXPECT_FALSE(world->checkCollision());

  // We will now add some BodyNodes and collision shapes *after* the Skeletons
  // have been added to the world, to see that the collision geometries get
  // updated automatically.
  Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
  tf.translation() = (1.0 + 0.25)*Eigen::Vector3d::UnitX();

  auto boxShape = std::make_shared<dart::dynamics::BoxShape>(
        Eigen::Vector3d::Constant(1.0));
  auto pair = skel_A->createJointAndBodyNodePair<dart::dynamics::FreeJoint>();
  pair.first->setTransform(tf);
  auto sn1 = pair.second->createShapeNodeWith<
      dart::dynamics::CollisionAspect>(boxShape);

  tf.translation() = (1.0 -0.25)*Eigen::Vector3d::UnitX();

  pair = skel_B->createJointAndBodyNodePair<dart::dynamics::FreeJoint>();
  pair.first->setTransform(tf);
  auto sn2 = pair.second->createShapeNodeWith<
      dart::dynamics::CollisionAspect>(boxShape);

  EXPECT_TRUE(world->checkCollision());

  // Now we'll change the properties of one the box shape so that there should
  // no longer be any collisions.
  boxShape->setSize(Eigen::Vector3d::Constant(0.1));
  dart::collision::CollisionResult result2;
  EXPECT_FALSE(world->checkCollision());

  // Now we'll replace one of the boxes with a large one, so that a collision
  // will occur again.
  auto largeBox = std::make_shared<dart::dynamics::BoxShape>(
        Eigen::Vector3d::Constant(1.0));
  sn1->setShape(largeBox);
  EXPECT_TRUE(world->checkCollision());

  // TODO(MXG): Make it so that this test does not crash bullet
//  sn2->setShape(largeBox);
//  EXPECT_TRUE(world->checkCollision());
}

INSTANTIATE_TEST_CASE_P(CollisionEngine, CollisionGroupsTest,
                        testing::Values("dart", "fcl", "bullet", "ode"));
