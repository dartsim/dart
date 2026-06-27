/*
 * Copyright (c) 2011, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/main/LICENSE
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

#include "dart/constraint/ConstraintSolver.hpp"
#include "dart/dynamics/BoxShape.hpp"
#include "dart/dynamics/CylinderShape.hpp"
#include "dart/dynamics/FreeJoint.hpp"
#include "dart/dynamics/Inertia.hpp"
#include "dart/dynamics/RevoluteJoint.hpp"
#include "dart/dynamics/Skeleton.hpp"
#include "dart/dynamics/WeldJoint.hpp"
#include "dart/simulation/World.hpp"

#include <gtest/gtest.h>

#include <vector>

using namespace dart;
using namespace dart::dynamics;

namespace {

constexpr double kFloorHeight = 0.1;
constexpr double kFloorSize = 10.0;
constexpr double kBoxSize = 0.2;
constexpr double kPenetration = 0.01;
constexpr std::size_t kCorrectionSteps = 50;
constexpr double kHalfPi = 1.57079632679489661923;

SkeletonPtr createFloor()
{
  auto floor = Skeleton::create("floor");
  auto pair = floor->createJointAndBodyNodePair<WeldJoint>(nullptr);
  auto* body = pair.second;

  auto shape = std::make_shared<BoxShape>(
      Eigen::Vector3d(kFloorSize, kFloorSize, kFloorHeight));
  auto shapeNode = body->createShapeNodeWith<
      VisualAspect,
      CollisionAspect,
      DynamicsAspect>(shape);
  auto* dynamics = shapeNode->getDynamicsAspect();
  dynamics->setFrictionCoeff(0.0);
  dynamics->setRestitutionCoeff(0.0);

  Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
  tf.translation().z() = -kFloorHeight / 2.0;
  pair.first->setTransformFromParentBodyNode(tf);

  return floor;
}

SkeletonPtr createBox(double centerHeight)
{
  auto box = Skeleton::create("box");
  auto pair = box->createJointAndBodyNodePair<FreeJoint>(nullptr);
  auto* joint = pair.first;
  auto* body = pair.second;

  auto shape = std::make_shared<BoxShape>(Eigen::Vector3d::Constant(kBoxSize));
  auto shapeNode = body->createShapeNodeWith<
      VisualAspect,
      CollisionAspect,
      DynamicsAspect>(shape);
  auto* dynamics = shapeNode->getDynamicsAspect();
  dynamics->setFrictionCoeff(0.0);
  dynamics->setRestitutionCoeff(0.0);

  const double mass = 1.0;
  Inertia inertia;
  inertia.setMass(mass);
  inertia.setMoment(shape->computeInertia(mass));
  body->setInertia(inertia);

  Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
  tf.translation().z() = centerHeight;
  FreeJoint::setTransformOf(joint, tf);

  return box;
}

simulation::WorldPtr createPenetratingWorld(bool splitImpulse)
{
  auto world = simulation::World::create();
  world->setGravity(Eigen::Vector3d::Zero());
  world->setTimeStep(0.001);
  auto* solver = world->getConstraintSolver();
  EXPECT_NE(nullptr, solver);
  solver->setSplitImpulseEnabled(splitImpulse);

  auto floor = createFloor();
  auto box = createBox(kBoxSize / 2.0 - kPenetration);
  box->setVelocities(Eigen::VectorXd::Zero(box->getNumDofs()));

  world->addSkeleton(floor);
  world->addSkeleton(box);

  return world;
}

SkeletonPtr createSupportedPendulumBase(
    BodyNode** baseBodyOut,
    BodyNode** upperLinkOut,
    RevoluteJoint** upperJointOut)
{
  auto skeleton = Skeleton::create("supported_pendulum_base");
  skeleton->disableSelfCollisionCheck();

  auto rootPair = skeleton->createJointAndBodyNodePair<FreeJoint>(nullptr);
  auto* rootJoint = rootPair.first;
  auto* baseBody = rootPair.second;
  baseBody->setName("base");

  Inertia baseInertia;
  baseInertia.setMass(100.0);
  baseInertia.setMoment(Eigen::Matrix3d::Identity());
  baseBody->setInertia(baseInertia);

  auto plateShape = std::make_shared<CylinderShape>(0.8, 0.02);
  auto* plateNode
      = baseBody->createShapeNodeWith<CollisionAspect, DynamicsAspect>(
          plateShape);
  Eigen::Isometry3d plateTf = Eigen::Isometry3d::Identity();
  plateTf.translation().z() = 0.01;
  plateNode->setRelativeTransform(plateTf);

  auto poleShape = std::make_shared<BoxShape>(Eigen::Vector3d(0.2, 0.2, 2.2));
  auto* poleNode
      = baseBody->createShapeNodeWith<CollisionAspect, DynamicsAspect>(
          poleShape);
  Eigen::Isometry3d poleTf = Eigen::Isometry3d::Identity();
  poleTf.translation() = Eigen::Vector3d(-0.275, 0.0, 1.1);
  poleNode->setRelativeTransform(poleTf);

  Eigen::Isometry3d rootTf = Eigen::Isometry3d::Identity();
  rootTf.translation().x() = 1.0;
  FreeJoint::setTransformOf(rootJoint, rootTf);

  GenericJoint<math::R1Space>::Properties upperGeneric(
      Joint::Properties("upper_joint"));
  upperGeneric.mDampingCoefficients[0] = 3.0;
  RevoluteJoint::Properties upperProperties(
      upperGeneric, RevoluteJoint::UniqueProperties(Eigen::Vector3d::UnitX()));
  upperProperties.mT_ParentBodyToJoint.translation()
      = Eigen::Vector3d(0.0, 0.0, 2.1);

  BodyNode::Properties upperBodyProperties(
      BodyNode::AspectProperties("upper_link"));
  Inertia upperInertia;
  upperInertia.setMass(1.0);
  upperInertia.setLocalCOM(Eigen::Vector3d(0.0, 0.0, 0.5));
  upperInertia.setMoment(Eigen::Matrix3d::Identity());
  upperBodyProperties.mInertia = upperInertia;

  auto upperPair = skeleton->createJointAndBodyNodePair<RevoluteJoint>(
      baseBody, upperProperties, upperBodyProperties);
  auto* upperJoint = upperPair.first;
  auto* upperLink = upperPair.second;
  upperJoint->setPosition(0, -kHalfPi);

  auto upperShape = std::make_shared<CylinderShape>(0.1, 0.9);
  auto* upperNode
      = upperLink->createShapeNodeWith<CollisionAspect, DynamicsAspect>(
          upperShape);
  Eigen::Isometry3d upperTf = Eigen::Isometry3d::Identity();
  upperTf.translation().z() = 0.5;
  upperNode->setRelativeTransform(upperTf);

  *baseBodyOut = baseBody;
  *upperLinkOut = upperLink;
  *upperJointOut = upperJoint;
  return skeleton;
}

} // namespace

//==============================================================================
// With split impulse enabled, a resting penetrating contact is corrected by the
// position pass (the body is pushed up) without injecting a residual velocity.
TEST(Issue201, SplitImpulseKeepsRestingContactVelocityZero)
{
  auto world = createPenetratingWorld(/*splitImpulse=*/true);
  auto* solver = world->getConstraintSolver();
  ASSERT_TRUE(solver->isSplitImpulseEnabled());

  auto box = world->getSkeleton("box");
  ASSERT_NE(box, nullptr);
  const auto* body = box->getRootBodyNode();
  ASSERT_NE(body, nullptr);
  const double initialHeight = body->getTransform().translation().z();

  for (std::size_t i = 0; i < kCorrectionSteps; ++i)
    world->step();

  EXPECT_NEAR(body->getLinearVelocity().z(), 0.0, 1e-6);
  EXPECT_GT(body->getTransform().translation().z(), initialHeight + 1e-6);
}

//==============================================================================
// The constraint solver defaults to split impulse OFF.
TEST(Issue201, SplitImpulseDisabledByDefault)
{
  auto world = simulation::World::create();
  auto* solver = world->getConstraintSolver();
  ASSERT_NE(nullptr, solver);
  EXPECT_FALSE(solver->isSplitImpulseEnabled());
}

//==============================================================================
// Guard test: with split impulse OFF, the contact solve must reproduce the
// pre-existing Baumgarte (ERP/ERV) penetration-correction behavior exactly.
// Running the same penetrating scene twice (both flag-off) must produce
// bit-identical trajectories, and the default path must apply the Baumgarte
// penetration correction in the velocity solve (which manifests as a small
// upward separation velocity), NOT the split-impulse position pass (which would
// leave the velocity at zero). This pins the default path to the legacy
// behavior that gz-physics / gz-sim rely on.
TEST(Issue201, DefaultPathReproducesBaumgarteBehavior)
{
  auto worldA = createPenetratingWorld(/*splitImpulse=*/false);
  auto worldB = createPenetratingWorld(/*splitImpulse=*/false);

  ASSERT_FALSE(worldA->getConstraintSolver()->isSplitImpulseEnabled());
  ASSERT_FALSE(worldB->getConstraintSolver()->isSplitImpulseEnabled());

  auto boxA = worldA->getSkeleton("box");
  auto boxB = worldB->getSkeleton("box");
  ASSERT_NE(boxA, nullptr);
  ASSERT_NE(boxB, nullptr);

  // Two independent runs of the default path must be byte-for-byte identical.
  for (std::size_t i = 0; i < kCorrectionSteps; ++i) {
    worldA->step();
    worldB->step();
    EXPECT_EQ(
        boxA->getRootBodyNode()->getTransform().translation().z(),
        boxB->getRootBodyNode()->getTransform().translation().z());
    EXPECT_EQ(
        boxA->getRootBodyNode()->getLinearVelocity().z(),
        boxB->getRootBodyNode()->getLinearVelocity().z());
  }

  // The Baumgarte penetration correction acts through the velocity solve, so
  // after one step the resting penetrating box carries a small positive
  // separation velocity. This is exactly the legacy behavior; the split-impulse
  // path (flag-on) would instead leave this velocity at zero. Confirm the
  // default path is the velocity-correction path.
  auto worldStep = createPenetratingWorld(/*splitImpulse=*/false);
  auto box = worldStep->getSkeleton("box");
  worldStep->step();
  EXPECT_GT(box->getRootBodyNode()->getLinearVelocity().z(), 0.0);
}

//==============================================================================
// The default Baumgarte velocity correction should keep its observable upward
// separation velocity, but a shallow resting support contact must not leak that
// correction into tiny lateral or tilt velocities on the free root.
TEST(Issue201, ShallowSupportedFreeRootDoesNotDriftSideways)
{
  auto world = simulation::World::create();
  auto floor = createFloor();
  floor->setMobile(false);
  world->addSkeleton(floor);

  BodyNode* baseBody = nullptr;
  BodyNode* upperLink = nullptr;
  RevoluteJoint* upperJoint = nullptr;
  world->addSkeleton(
      createSupportedPendulumBase(&baseBody, &upperLink, &upperJoint));

  ASSERT_NE(baseBody, nullptr);
  ASSERT_NE(upperLink, nullptr);
  ASSERT_NE(upperJoint, nullptr);

  for (std::size_t i = 0; i < 10; ++i)
    world->step();

  EXPECT_LT(upperJoint->getVelocity(0), 0.0);
  EXPECT_GT(baseBody->getLinearVelocity().z(), 1e-5);
  EXPECT_NEAR(baseBody->getLinearVelocity().x(), 0.0, 1e-6);
  EXPECT_NEAR(baseBody->getLinearVelocity().y(), 0.0, 1e-6);
  EXPECT_NEAR(baseBody->getAngularVelocity().x(), 0.0, 1e-6);
  EXPECT_NEAR(baseBody->getAngularVelocity().y(), 0.0, 1e-6);
}
