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

#include "dart/collision/ode/OdeCollisionDetector.hpp"
#include "dart/constraint/ConstraintSolver.hpp"
#include "dart/dynamics/SimpleFrame.hpp"
#include "dart/math/Geometry.hpp"
#include "dart/math/Helpers.hpp"
#include "dart/math/Random.hpp"

#include "TestHelpers.hpp"

using namespace dart;
using namespace dynamics;

//==============================================================================
std::shared_ptr<World> createWorld()
{
  auto world = simulation::World::create();
  world->getConstraintSolver()->setCollisionDetector(
      collision::OdeCollisionDetector::create());
  return world;
}

//==============================================================================
dynamics::SkeletonPtr createFloor()
{
  auto floor = dynamics::Skeleton::create("floor");

  // Give the floor a body
  auto body
      = floor->createJointAndBodyNodePair<dynamics::WeldJoint>(nullptr).second;

  // Give the body a shape
  double floorWidth = 10000.0;
  double floorHeight = 0.01;
  auto box = std::make_shared<dynamics::BoxShape>(
      Eigen::Vector3d(floorWidth, floorWidth, floorHeight));
  auto* shapeNode = body->createShapeNodeWith<
      dynamics::VisualAspect,
      dynamics::CollisionAspect,
      dynamics::DynamicsAspect>(box);
  shapeNode->getVisualAspect()->setColor(dart::Color::LightGray());
  shapeNode->getDynamicsAspect()->setSlipCompliance(0);
  shapeNode->getDynamicsAspect()->setSecondarySlipCompliance(0);

  // Put the body into position
  Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
  tf.translation() = Eigen::Vector3d(0.0, 0.0, -floorHeight / 2.0);
  body->getParentJoint()->setTransformFromParentBodyNode(tf);

  return floor;
}

//==============================================================================
SkeletonPtr createCylinder(
    double _radius,
    double _height,
    const Eigen::Vector3d& _position = Eigen::Vector3d::Zero(),
    const Eigen::Vector3d& _orientation = Eigen::Vector3d::Zero())
{
  SkeletonPtr cylinder = createObject(_position, _orientation);

  BodyNode* bn = cylinder->getBodyNode(0);
  std::shared_ptr<Shape> cylinderShape(new CylinderShape(_radius, _height));
  bn->createShapeNodeWith<VisualAspect, CollisionAspect, DynamicsAspect>(
      cylinderShape);

  return cylinder;
}

//==============================================================================
TEST(ForceDependentSlip, BoxSlipVelocity)
{
  using Eigen::Vector3d;
  const double mass = 5.0;
  const double slip = 0.02;
  auto skeleton1 = createBox({0.3, 0.3, 0.3}, {0, -0.5, 0.15});
  skeleton1->setName("Skeleton1");
  auto skeleton2 = createBox({0.3, 0.3, 0.3}, {0, +0.5, 0.15});
  skeleton2->setName("Skeleton2");

  auto body1 = skeleton1->getRootBodyNode();
  body1->setMass(mass);
  auto body1Dynamics = body1->getShapeNode(0)->getDynamicsAspect();

  EXPECT_DOUBLE_EQ(body1Dynamics->getFrictionCoeff(), 1.0);

  body1Dynamics->setSlipCompliance(slip);
  body1Dynamics->setFirstFrictionDirection(Vector3d::UnitX());
  EXPECT_DOUBLE_EQ(body1Dynamics->getSlipCompliance(), slip);
  EXPECT_EQ(body1Dynamics->getFirstFrictionDirection(), Vector3d::UnitX());

  auto body2 = skeleton2->getRootBodyNode();
  body2->setMass(mass);
  auto body2Dynamics = body2->getShapeNode(0)->getDynamicsAspect();
  body2Dynamics->setFirstFrictionDirection(Vector3d::UnitX());
  EXPECT_EQ(body2Dynamics->getFirstFrictionDirection(), Vector3d::UnitX());
  EXPECT_DOUBLE_EQ(body2Dynamics->getFrictionCoeff(), 1.0);

  // Create a world and add the rigid bodies
  auto world = createWorld();
  world->addSkeleton(createFloor());
  world->addSkeleton(skeleton1);
  world->addSkeleton(skeleton2);

  const auto numSteps = 2000;
  const double extForce = 10.0;
  for (auto i = 0u; i < numSteps; ++i)
  {
    body1->addExtForce({extForce, 0, 0});
    body2->addExtForce({extForce, 0, 0});
    world->step();

    if (i > 1000)
    {
      // The velocity of body1 should stabilize at F_ext * slip = 0.2 m/s
      EXPECT_NEAR(extForce * slip, body1->getLinearVelocity().x(), 2e-5);
      EXPECT_NEAR(0.0, body1->getLinearVelocity().y(), 2e-5);

      // The second box should remain at rest because of friction
      EXPECT_NEAR(0.0, body2->getLinearVelocity().x(), 2e-5);
      EXPECT_NEAR(0.0, body2->getLinearVelocity().y(), 2e-5);
    }
  }

  const double slip2 = 0.03;
  // Test slip compliance in the secondary friction direction
  body1Dynamics->setSlipCompliance(0);
  body1Dynamics->setSecondarySlipCompliance(slip2);

  EXPECT_DOUBLE_EQ(body1Dynamics->getSlipCompliance(), 0.0);
  EXPECT_DOUBLE_EQ(body1Dynamics->getSecondarySlipCompliance(), slip2);

  // Step without external force so the body stop moving
  for (auto i = 0u; i < 500; ++i)
  {
    world->step();
  }
  EXPECT_NEAR(0.0, body1->getLinearVelocity().x(), 2e-5);
  EXPECT_NEAR(0.0, body1->getLinearVelocity().y(), 2e-5);

  // The second box should remain at rest because of friction
  EXPECT_NEAR(0.0, body2->getLinearVelocity().x(), 2e-5);
  EXPECT_NEAR(0.0, body2->getLinearVelocity().y(), 2e-5);

  // Apply force in the +y direction
  for (auto i = 0u; i < numSteps; ++i)
  {
    body1->addExtForce({0, extForce, 0});
    body2->addExtForce({0, extForce, 0});
    world->step();

    if (i > 1500)
    {
      // The velocity of body1 should stabilize at F_ext * slip2 = 0.3 m/s
      EXPECT_NEAR(0.0, body1->getLinearVelocity().x(), 2e-5);
      EXPECT_NEAR(extForce * slip2, body1->getLinearVelocity().y(), 2e-5);

      // The second box should remain at rest because of friction
      EXPECT_NEAR(0.0, body2->getLinearVelocity().x(), 2e-5);
      EXPECT_NEAR(0.0, body2->getLinearVelocity().y(), 2e-5);
    }
  }
}

//==============================================================================
// Test two cylinders, one with its z axis pointing in the z axis of the world
// so it's purely slipping, and the other with its z axis pointing in the y axis
// of the world so it's rolling and slipping.
TEST(ForceDependentSlip, CylinderSlipVelocity)
{
  using Eigen::Vector3d;
  const double mass = 2.0;
  const double radius = 0.5;
  const double slip = 0.02;
  auto skeleton1 = createCylinder(radius, 0.3, {0, -5, radius});
  skeleton1->setName("Skeleton1");

  auto skeleton2 = createCylinder(
      radius, 0.8, {0, 5, radius}, {math::constantsd::half_pi(), 0, 0});
  skeleton2->setName("Skeleton2");

  auto body1 = skeleton1->getRootBodyNode();
  body1->setMass(5.0);
  auto body1Dynamics = body1->getShapeNode(0)->getDynamicsAspect();

  EXPECT_DOUBLE_EQ(body1Dynamics->getFrictionCoeff(), 1.0);

  body1Dynamics->setSlipCompliance(slip);
  body1Dynamics->setFirstFrictionDirection(Vector3d::UnitX());
  EXPECT_DOUBLE_EQ(body1Dynamics->getSlipCompliance(), slip);
  EXPECT_EQ(body1Dynamics->getFirstFrictionDirection(), Vector3d::UnitX());

  auto body2 = skeleton2->getRootBodyNode();
  body2->setMass(mass);
  auto body2Dynamics = body2->getShapeNode(0)->getDynamicsAspect();
  EXPECT_DOUBLE_EQ(body2Dynamics->getFrictionCoeff(), 1.0);

  // Set the friction direction to +z in the shape frame because it will always
  // be orthogonal to the floor's normal.
  body2Dynamics->setFirstFrictionDirection(Vector3d::UnitZ());
  // Since we want to test rolling with slipping for body2, we want to set a
  // non-zero slip parameter in the direction orthogonal to the axis of rotation
  // of the cylinder. The axis of rotation is in the body's +z direction, so we
  // make that the first friction direction and set the non-zero slip parameter
  // in the secondary direction.
  body2Dynamics->setSecondarySlipCompliance(slip);
  EXPECT_DOUBLE_EQ(body2Dynamics->getSecondarySlipCompliance(), slip);
  EXPECT_EQ(body2Dynamics->getFirstFrictionDirection(), Vector3d::UnitZ());

  // Create a world and add the rigid bodies
  auto world = createWorld();

  world->addSkeleton(createFloor());
  world->addSkeleton(skeleton1);
  world->addSkeleton(skeleton2);

  const double dt = 0.001;
  const auto numSteps = 2000;
  const double extForceX = 1.0;
  const double extTorqueY = 2.0;

  auto lastVel = body2->getLinearVelocity();
  for (auto i = 0u; i < numSteps; ++i)
  {
    body1->addExtForce({extForceX, 0, 0});
    body2->addExtTorque({0, extTorqueY, 0.0}, false);
    world->step();

    if (i > 1000)
    {
      // The velocity of body1 should stabilize at F_ext * slip
      EXPECT_NEAR(extForceX * slip, body1->getLinearVelocity().x(), 1e-4);
      EXPECT_NEAR(0.0, body1->getLinearVelocity().y(), 1e-4);

      // body2 rolls with sliding. The difference between the linear velocity
      // and the expected non-sliding velocity (angular velocity * radius) is
      // equal to F_fr * slip, where F_fr is the friction force. We compute the
      // friction force from the linear acceleration since it's the only linear
      // force on the body.
      auto linVel = body2->getLinearVelocity().x();
      auto spinVel = body2->getAngularVelocity().y() * radius;
      // There appears to be a bug in DART in obtaining the linear acceleration
      // of the body using (BodyNode::getLinearAcceleration), so we compute it
      // here via finite difference.
      auto accel = (body2->getLinearVelocity() - lastVel) / dt;
      EXPECT_NEAR(mass * accel.x() * slip, spinVel - linVel, 2e-4);
      EXPECT_NEAR(0.0, body2->getLinearVelocity().y(), 1e-4);
    }

    lastVel = body2->getLinearVelocity();
  }
}
