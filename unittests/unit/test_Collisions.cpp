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

TEST(Collisions, WorldSubscription)
{
  dart::simulation::WorldPtr world = dart::simulation::World::create();

  dart::dynamics::SkeletonPtr skel_A = dart::dynamics::Skeleton::create("A");
  dart::dynamics::SkeletonPtr skel_B = dart::dynamics::Skeleton::create("B");

  world->addSkeleton(skel_A);
  world->addSkeleton(skel_B);

  Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
  tf.translation() = 0.25*Eigen::Vector3d::UnitX();

  auto boxShape = std::make_shared<dart::dynamics::BoxShape>(
        Eigen::Vector3d::Constant(1.0));

  // We will now add some BodyNodes and collision shapes *after* the Skeletons
  // have been added to the world, to see that the collision geometries get
  // updated automatically.
  auto pair = skel_A->createJointAndBodyNodePair<dart::dynamics::FreeJoint>();
  pair.first->setTransform(tf);
  pair.second->createShapeNodeWith<dart::dynamics::CollisionAspect>(boxShape);

  tf.translation() = -0.25*Eigen::Vector3d::UnitX();

  pair = skel_B->createJointAndBodyNodePair<dart::dynamics::FreeJoint>();
  pair.first->setTransform(tf);
  pair.second->createShapeNodeWith<dart::dynamics::CollisionAspect>(boxShape);

  EXPECT_TRUE(world->checkCollision());
}

