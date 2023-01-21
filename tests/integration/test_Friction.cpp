/*
 * Copyright (c) 2011-2023, The DART development contributors
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

#include "dart/dynamics/SimpleFrame.hpp"
#include "dart/math/Helpers.hpp"
#include "dart/math/Random.hpp"
#include "dart/simulation/simulation.hpp"

#include <dart/test/dynamics/TestHelpers.hpp>

#include <gtest/gtest.h>

using namespace dart;
using namespace dynamics;

//==============================================================================
dynamics::SkeletonPtr createFloor()
{
  auto floor = dynamics::Skeleton::create("floor");

  // Give the floor a body
  auto body
      = floor->createJointAndBodyNodePair<dynamics::WeldJoint>(nullptr).second;

  // Give the body a shape
  double floorWidth = 10.0;
  double floorHeight = 0.01;
  auto box = std::make_shared<dynamics::BoxShape>(
      math::Vector3d(floorWidth, floorWidth, floorHeight));
  auto* shapeNode = body->createShapeNodeWith<
      dynamics::VisualAspect,
      dynamics::CollisionAspect,
      dynamics::DynamicsAspect>(box);
  shapeNode->getVisualAspect()->setColor(dart::math::Colord::LightGray());

  // Put the body into position
  math::Isometry3d tf = math::Isometry3d::Identity();
  tf.translation() = math::Vector3d(0.0, 0.0, -0.5 - floorHeight / 2.0);
  body->getParentJoint()->setTransformFromParentBodyNode(tf);

  return floor;
}

//==============================================================================
TEST(Friction, FrictionPerShapeNode)
{
  auto skeleton1
      = createBox(math::Vector3d(0.3, 0.3, 0.3), math::Vector3d(-0.5, 0, 0));
  skeleton1->setName("Skeleton1");
  auto skeleton2
      = createBox(math::Vector3d(0.3, 0.3, 0.3), math::Vector3d(+0.5, 0, 0));
  skeleton2->setName("Skeleton2");
  auto skeleton3 = createBox(
      math::Vector3d(0.3, 0.3, 0.3),
      math::Vector3d(+1.5, 0, 0),
      math::Vector3d(0, 0, 0.7853981633974483));
  skeleton3->setName("Skeleton3");
  auto skeleton4 = createBox(
      math::Vector3d(0.3, 0.3, 0.3),
      math::Vector3d(-1.5, 0, 0),
      math::Vector3d(0, 0, 0.7853981633974483));
  skeleton4->setName("Skeleton4");

  auto body1 = skeleton1->getRootBodyNode();
  // default friction values
  EXPECT_DOUBLE_EQ(
      1.0, body1->getShapeNode(0)->getDynamicsAspect()->getFrictionCoeff());
  EXPECT_DOUBLE_EQ(
      1.0,
      body1->getShapeNode(0)->getDynamicsAspect()->getPrimaryFrictionCoeff());
  EXPECT_DOUBLE_EQ(
      1.0,
      body1->getShapeNode(0)->getDynamicsAspect()->getSecondaryFrictionCoeff());

  auto body2 = skeleton2->getRootBodyNode();
  // default friction values
  EXPECT_DOUBLE_EQ(
      1.0, body2->getShapeNode(0)->getDynamicsAspect()->getFrictionCoeff());
  EXPECT_DOUBLE_EQ(
      1.0,
      body2->getShapeNode(0)->getDynamicsAspect()->getPrimaryFrictionCoeff());
  EXPECT_DOUBLE_EQ(
      1.0,
      body2->getShapeNode(0)->getDynamicsAspect()->getSecondaryFrictionCoeff());
  // test setting primary friction
  body2->getShapeNode(0)->getDynamicsAspect()->setPrimaryFrictionCoeff(0.5);
  EXPECT_DOUBLE_EQ(
      0.75, body2->getShapeNode(0)->getDynamicsAspect()->getFrictionCoeff());
  EXPECT_DOUBLE_EQ(
      0.5,
      body2->getShapeNode(0)->getDynamicsAspect()->getPrimaryFrictionCoeff());
  EXPECT_DOUBLE_EQ(
      1.0,
      body2->getShapeNode(0)->getDynamicsAspect()->getSecondaryFrictionCoeff());
  // test setting secondary friction
  body2->getShapeNode(0)->getDynamicsAspect()->setSecondaryFrictionCoeff(0.25);
  EXPECT_DOUBLE_EQ(
      0.375, body2->getShapeNode(0)->getDynamicsAspect()->getFrictionCoeff());
  EXPECT_DOUBLE_EQ(
      0.5,
      body2->getShapeNode(0)->getDynamicsAspect()->getPrimaryFrictionCoeff());
  EXPECT_DOUBLE_EQ(
      0.25,
      body2->getShapeNode(0)->getDynamicsAspect()->getSecondaryFrictionCoeff());
  // set all friction coeffs to 0.0
  body2->getShapeNode(0)->getDynamicsAspect()->setFrictionCoeff(0.0);
  EXPECT_DOUBLE_EQ(
      0.0, body2->getShapeNode(0)->getDynamicsAspect()->getFrictionCoeff());
  EXPECT_DOUBLE_EQ(
      0.0,
      body2->getShapeNode(0)->getDynamicsAspect()->getPrimaryFrictionCoeff());
  EXPECT_DOUBLE_EQ(
      0.0,
      body2->getShapeNode(0)->getDynamicsAspect()->getSecondaryFrictionCoeff());

  auto body3 = skeleton3->getRootBodyNode();
  // default friction values
  EXPECT_DOUBLE_EQ(
      1.0, body3->getShapeNode(0)->getDynamicsAspect()->getFrictionCoeff());
  EXPECT_DOUBLE_EQ(
      1.0,
      body3->getShapeNode(0)->getDynamicsAspect()->getPrimaryFrictionCoeff());
  EXPECT_DOUBLE_EQ(
      1.0,
      body3->getShapeNode(0)->getDynamicsAspect()->getSecondaryFrictionCoeff());
  EXPECT_DOUBLE_EQ(
      0.0,
      body3->getShapeNode(0)
          ->getDynamicsAspect()
          ->getFirstFrictionDirection()
          .squaredNorm());
  EXPECT_EQ(
      nullptr,
      body3->getShapeNode(0)
          ->getDynamicsAspect()
          ->getFirstFrictionDirectionFrame());
  // this body is rotated by 45 degrees, so set friction direction in body frame
  // along Y axis so that gravity pushes it in x and y
  body3->getShapeNode(0)->getDynamicsAspect()->setPrimaryFrictionCoeff(0.0);
  body3->getShapeNode(0)->getDynamicsAspect()->setFirstFrictionDirection(
      math::Vector3d(0, 1, 0));
  // check friction values
  EXPECT_DOUBLE_EQ(
      0.5, body3->getShapeNode(0)->getDynamicsAspect()->getFrictionCoeff());
  EXPECT_DOUBLE_EQ(
      0.0,
      body3->getShapeNode(0)->getDynamicsAspect()->getPrimaryFrictionCoeff());
  EXPECT_DOUBLE_EQ(
      1.0,
      body3->getShapeNode(0)->getDynamicsAspect()->getSecondaryFrictionCoeff());
  EXPECT_DOUBLE_EQ(
      1.0,
      body3->getShapeNode(0)
          ->getDynamicsAspect()
          ->getFirstFrictionDirection()
          .squaredNorm());
  EXPECT_EQ(
      nullptr,
      body3->getShapeNode(0)
          ->getDynamicsAspect()
          ->getFirstFrictionDirectionFrame());

  auto body4 = skeleton4->getRootBodyNode();
  // default friction values
  EXPECT_DOUBLE_EQ(
      1.0, body4->getShapeNode(0)->getDynamicsAspect()->getFrictionCoeff());
  EXPECT_DOUBLE_EQ(
      1.0,
      body4->getShapeNode(0)->getDynamicsAspect()->getPrimaryFrictionCoeff());
  EXPECT_DOUBLE_EQ(
      1.0,
      body4->getShapeNode(0)->getDynamicsAspect()->getSecondaryFrictionCoeff());
  EXPECT_DOUBLE_EQ(
      0.0,
      body4->getShapeNode(0)
          ->getDynamicsAspect()
          ->getFirstFrictionDirection()
          .squaredNorm());
  EXPECT_EQ(
      nullptr,
      body4->getShapeNode(0)
          ->getDynamicsAspect()
          ->getFirstFrictionDirectionFrame());
  // this body is rotated by 45 degrees, but set friction direction according to
  // world frame so that the body orientation doesn't matter. thus a diagonal
  // axis is needed to push it in x and y
  body4->getShapeNode(0)->getDynamicsAspect()->setPrimaryFrictionCoeff(0.0);
  body4->getShapeNode(0)->getDynamicsAspect()->setFirstFrictionDirectionFrame(
      Frame::World());
  body4->getShapeNode(0)->getDynamicsAspect()->setFirstFrictionDirection(
      math::Vector3d(0.5 * std::sqrt(2), 0.5 * std::sqrt(2), 0));
  // check friction values
  EXPECT_DOUBLE_EQ(
      0.5, body4->getShapeNode(0)->getDynamicsAspect()->getFrictionCoeff());
  EXPECT_DOUBLE_EQ(
      0.0,
      body4->getShapeNode(0)->getDynamicsAspect()->getPrimaryFrictionCoeff());
  EXPECT_DOUBLE_EQ(
      1.0,
      body4->getShapeNode(0)->getDynamicsAspect()->getSecondaryFrictionCoeff());
  EXPECT_DOUBLE_EQ(
      1.0,
      body4->getShapeNode(0)
          ->getDynamicsAspect()
          ->getFirstFrictionDirection()
          .squaredNorm());
  EXPECT_EQ(
      Frame::World(),
      body4->getShapeNode(0)
          ->getDynamicsAspect()
          ->getFirstFrictionDirectionFrame());

  // Create a world and add the rigid body
  auto world = simulation::World::create();
  EXPECT_TRUE(test::equals(world->getGravity(), ::math::Vector3d(0, 0, -9.81)));
  world->setGravity(math::Vector3d(0.0, -5.0, -9.81));
  EXPECT_TRUE(
      test::equals(world->getGravity(), ::math::Vector3d(0.0, -5.0, -9.81)));

  world->addSkeleton(createFloor());
  world->addSkeleton(skeleton1);
  world->addSkeleton(skeleton2);
  world->addSkeleton(skeleton3);
  world->addSkeleton(skeleton4);

  const auto numSteps = 500;
  for (auto i = 0u; i < numSteps; ++i) {
    world->step();

    // Wait until the first box settle-in on the ground
    if (i > 300) {
      const auto x1 = body1->getTransform().translation()[0];
      const auto y1 = body1->getTransform().translation()[1];
      EXPECT_NEAR(x1, -0.5, 0.00001);
      EXPECT_NEAR(y1, -0.17889, 0.001);

      // The second box still moves even after landing on the ground because its
      // friction is zero.
      const auto x2 = body2->getTransform().translation()[0];
      const auto y2 = body2->getTransform().translation()[1];
      EXPECT_NEAR(x2, 0.5, 0.00001);
      EXPECT_LE(y2, -0.17889);

      // The third box still moves even after landing on the ground because its
      // friction is zero along the first friction direction.
      const auto x3 = body3->getTransform().translation()[0];
      const auto y3 = body3->getTransform().translation()[1];
      EXPECT_GE(x3, 1.5249);
      EXPECT_LE(y3, -0.20382);

      // The fourth box still moves even after landing on the ground because its
      // friction is zero along the first friction direction.
      const auto x4 = body4->getTransform().translation()[0];
      const auto y4 = body4->getTransform().translation()[1];
      EXPECT_LE(x4, -1.5249);
      EXPECT_LE(y4, -0.20382);
    }
  }
}
