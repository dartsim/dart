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

#include "dart/dynamics/FreeJoint.hpp"
#include "dart/dynamics/BoxShape.hpp"
#include "dart/dynamics/Skeleton.hpp"
#include <dart/dynamics/SphereShape.hpp>

#include "dart/simulation/World.hpp"
#include "dart/constraint/ConstraintSolver.hpp"


class CollisionGroupsTest
    : public testing::Test,
      public testing::WithParamInterface<const char*>
{

};

TEST_P(CollisionGroupsTest, SkeletonSubscription)
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

  // Note: When skeletons are added to a world, the constraint solver will
  // subscribe to them.
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
  boxShape->setSize(Eigen::Vector3d::Constant(0.2));
  dart::collision::CollisionResult result2;
  EXPECT_FALSE(world->checkCollision());

  // Now we'll replace one of the boxes with a large one, so that a collision
  // will occur again.
  auto largeBox = std::make_shared<dart::dynamics::BoxShape>(
        Eigen::Vector3d::Constant(0.95));
  sn1->setShape(largeBox);
  EXPECT_TRUE(world->checkCollision());

  // After this, both shapes will have been replaced with new shape instances.
  // If the shape map of the collision detector is empty upon destruction, then
  // the internal collision shapes are being managed correctly. Otherwise, if
  // they are not being managed correctly, then an assertion will fail when
  // testing in debug mode.
  auto sphereShape = std::make_shared<dart::dynamics::SphereShape>(0.01);
  sn2->setShape(sphereShape);
  EXPECT_FALSE(world->checkCollision());

  // Resize the sphere so that there is a collision again.
  sphereShape->setRadius(0.5);
  EXPECT_TRUE(world->checkCollision());

  // Remove the shape node so that there should no longer be a collision
  sn2->remove();
  EXPECT_FALSE(world->checkCollision());

  // Create a new shape node so that there should be a collision again
  pair.second->createShapeNodeWith<
      dart::dynamics::CollisionAspect>(sphereShape);
  EXPECT_TRUE(world->checkCollision());

  // Remove the BodyNode so that there should no longer be a collision
  pair.second->remove();
  EXPECT_FALSE(world->checkCollision());

  // Add a new BodyNode and Shape Node so that there is a collision again
  pair = skel_B->createJointAndBodyNodePair<dart::dynamics::FreeJoint>();
  pair.first->setTransform(tf);
  pair.second->createShapeNodeWith<
      dart::dynamics::CollisionAspect>(sphereShape);
  EXPECT_TRUE(world->checkCollision());

  // Remove a skeleton so that there are no longer collisions
  world->removeSkeleton(skel_B);
  EXPECT_FALSE(world->checkCollision());
}

TEST_P(CollisionGroupsTest, BodyNodeSubscription)
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

  auto cd = dart::collision::CollisionDetector::getFactory()
      ->create(GetParam());

  auto group = cd->createCollisionGroup();

  auto skel1 = dart::dynamics::Skeleton::create("skel1");
  auto skel2 = dart::dynamics::Skeleton::create("skel2");

  auto pair_1a = skel1->createJointAndBodyNodePair<dart::dynamics::FreeJoint>();
  auto pair_2a = skel2->createJointAndBodyNodePair<dart::dynamics::FreeJoint>();

  group->subscribeTo(pair_1a.second, pair_2a.second);
  // The BodyNodes currently have no collision geometries, so we expect there to
  // be no collisions.
  EXPECT_FALSE(group->collide());

  Eigen::Isometry3d tf{Eigen::Translation3d(0.5, 0.0, 0.0)};
  pair_1a.first->setTransform(tf);
  pair_1a.first->setName("1a");
  pair_1a.second->setName("1a");

  tf.translation()[0] = -0.5;
  pair_2a.first->setTransform(tf);
  pair_2a.first->setName("2a");
  pair_2a.second->setName("2a");

  auto sphere = std::make_shared<dart::dynamics::SphereShape>(0.75);

  auto sn_1a = pair_1a.second->createShapeNodeWith<
      dart::dynamics::CollisionAspect>(sphere);

  auto sn_2a = pair_2a.second->createShapeNodeWith<
      dart::dynamics::CollisionAspect>(sphere);

  // The BodyNodes have been given overlapping shapes, so now we expect the
  // collision information to automatically update and identify a collision.
  EXPECT_TRUE(group->collide());

  tf.translation()[0] =  1.0;
  pair_1a.first->setTransform(tf);
  tf.translation()[0] = -1.0;
  pair_2a.first->setTransform(tf);

  // The collision geometries have been moved far enough from each other that
  // there should no longer be any collisions
  EXPECT_FALSE(group->collide());

  sphere->setRadius(1.1);
  // The shapes of the collision geometries have been expanded enough that they
  // should collide again.
  EXPECT_TRUE(group->collide());

  auto box = std::make_shared<dart::dynamics::BoxShape>(
        Eigen::Vector3d::Constant(0.5));
  sn_2a->setShape(box);
  // Now a shape has been replaced with a smaller one, so there should no longer
  // be a collision.
  EXPECT_FALSE(group->collide());

  auto pair_1b = skel1->createJointAndBodyNodePair<dart::dynamics::FreeJoint>();
  auto pair_2b = skel2->createJointAndBodyNodePair<dart::dynamics::FreeJoint>();

  tf.translation() = 0.5*Eigen::Vector3d::UnitX();
  pair_1b.first->setTransform(tf);
  pair_1b.first->setName("1b");
  pair_1b.second->setName("1b");

  tf.translation()[0] = -0.5;
  pair_2b.first->setTransform(tf);
  pair_2b.first->setName("2b");
  pair_2b.second->setName("2b");

  pair_1b.second->createShapeNodeWith<dart::dynamics::CollisionAspect>(sphere);
  pair_2b.second->createShapeNodeWith<dart::dynamics::CollisionAspect>(sphere);

  // The collision group should not be tracking the new BodyNodes, so there
  // should still be no collisions
  EXPECT_FALSE(group->collide());

  // Change the size of the box so that there are overlapping collision
  // geometries. The collision information should update automatically, and a
  // collision should be detected.
  box->setSize(Eigen::Vector3d::Constant(2.0));
  EXPECT_TRUE(group->collide());

  sn_1a->remove();
  // A shape node has been removed, so there should no longer be a collision
  EXPECT_FALSE(group->collide());

  pair_1a.second->createShapeNodeWith<dart::dynamics::CollisionAspect>(sphere);
  // A new shape node has been added with the same geometry as the one that was
  // removed, so there should be a collision again
  EXPECT_TRUE(group->collide());

  // Remove one of the BodyNodes so that there should no longer be a collision.
  group->removeShapeFramesOf(pair_1a.second);
  EXPECT_FALSE(group->collide());
}

INSTANTIATE_TEST_CASE_P(CollisionEngine, CollisionGroupsTest,
                        testing::Values("dart", "fcl", "bullet", "ode"));
