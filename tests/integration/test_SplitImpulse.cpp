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

#include "dart/collision/CollisionResult.hpp"
#include "dart/collision/dart/DARTCollisionDetector.hpp"
#include "dart/collision/fcl/FCLCollisionDetector.hpp"
#include "dart/config.hpp"
#if HAVE_BULLET
  #include "dart/collision/bullet/BulletCollisionDetector.hpp"
#endif
#if HAVE_ODE
  #include "dart/collision/ode/OdeCollisionDetector.hpp"
#endif
#include "dart/constraint/BallJointConstraint.hpp"
#include "dart/constraint/BoxedLcpConstraintSolver.hpp"
#include "dart/constraint/ConstraintSolver.hpp"
#include "dart/constraint/PgsBoxedLcpSolver.hpp"
#include "dart/dynamics/BoxShape.hpp"
#include "dart/dynamics/CylinderShape.hpp"
#include "dart/dynamics/FreeJoint.hpp"
#include "dart/dynamics/Inertia.hpp"
#include "dart/dynamics/RevoluteJoint.hpp"
#include "dart/dynamics/Skeleton.hpp"
#include "dart/dynamics/SphereShape.hpp"
#include "dart/dynamics/WeldJoint.hpp"
#include "dart/simulation/World.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <string>
#include <vector>

#include <cmath>

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

// How the arm of createSupportedPendulumBase() is mounted.
enum class PendulumArm
{
  Revolute,
  Welded
};

// The base and upper arm of the double pendulum in gz-physics' JointDetach
// test: a 100 kg base whose plate rests on the floor, with a 1 kg arm held out
// level from the top of its pole, on a damped revolute joint or welded (then
// upperJointOut is null).
SkeletonPtr createSupportedPendulumBase(
    BodyNode** baseBodyOut,
    BodyNode** upperLinkOut,
    RevoluteJoint** upperJointOut,
    PendulumArm arm = PendulumArm::Revolute)
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

  RevoluteJoint* upperJoint = nullptr;
  BodyNode* upperLink = nullptr;
  if (arm == PendulumArm::Welded) {
    const WeldJoint::Properties weldProperties(Joint::Properties(
        "upper_joint",
        upperProperties.mT_ParentBodyToJoint
            * Eigen::AngleAxisd(-kHalfPi, Eigen::Vector3d::UnitX())));
    upperLink = skeleton
                    ->createJointAndBodyNodePair<WeldJoint>(
                        baseBody, weldProperties, upperBodyProperties)
                    .second;
  } else {
    auto upperPair = skeleton->createJointAndBodyNodePair<RevoluteJoint>(
        baseBody, upperProperties, upperBodyProperties);
    upperJoint = upperPair.first;
    upperLink = upperPair.second;
    upperJoint->setPosition(0, -kHalfPi);
  }

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

// Collision detectors built into this DART.
std::vector<std::string> getCollisionDetectorNames()
{
  std::vector<std::string> names{"fcl", "dart"};
#if HAVE_ODE
  names.emplace_back("ode");
#endif
#if HAVE_BULLET
  names.emplace_back("bullet");
#endif
  return names;
}

collision::CollisionDetectorPtr createCollisionDetector(const std::string& name)
{
  if (name == "dart")
    return collision::DARTCollisionDetector::create();
#if HAVE_ODE
  if (name == "ode")
    return collision::OdeCollisionDetector::create();
#endif
#if HAVE_BULLET
  if (name == "bullet")
    return collision::BulletCollisionDetector::create();
#endif
  return collision::FCLCollisionDetector::create();
}

simulation::WorldPtr createArticulatedTreeWorld(
    const std::string& collisionDetector, double friction, bool deactivation)
{
  auto world = simulation::World::create();
  world->setTimeStep(0.001);
  world->getConstraintSolver()->setCollisionDetector(
      createCollisionDetector(collisionDetector));
  auto options = world->getDeactivationOptions();
  options.mEnabled = deactivation;
  world->setDeactivationOptions(options);

  auto floor = createFloor();
  floor->setMobile(false);
  floor->getBodyNode(0)->getShapeNode(0)->getDynamicsAspect()->setFrictionCoeff(
      friction);
  world->addSkeleton(floor);
  return world;
}

// A 5 kg base box resting on the floor, with a 1 kg arm that hangs from a
// bracket 0.25 m to the side on a revolute joint about y. The arm has no
// collision shape, so only the base touches the floor.
SkeletonPtr createArmRobot(
    double friction, BodyNode** baseOut, RevoluteJoint** armJointOut)
{
  auto robot = Skeleton::create("arm_robot");
  auto basePair = robot->createJointAndBodyNodePair<FreeJoint>(nullptr);
  auto* base = basePair.second;
  const Eigen::Vector3d baseSize(0.4, 0.4, 0.1);
  base->createShapeNodeWith<CollisionAspect, DynamicsAspect>(
          std::make_shared<BoxShape>(baseSize))
      ->getDynamicsAspect()
      ->setFrictionCoeff(friction);
  Inertia baseInertia;
  baseInertia.setMass(5.0);
  baseInertia.setMoment(BoxShape::computeInertia(baseSize, 5.0));
  base->setInertia(baseInertia);
  Eigen::Isometry3d baseTf = Eigen::Isometry3d::Identity();
  baseTf.translation().z() = baseSize.z() / 2.0 - 1e-6;
  FreeJoint::setTransformOf(basePair.first, baseTf);

  RevoluteJoint::Properties armJointProperties;
  armJointProperties.mAxis = Eigen::Vector3d::UnitY();
  armJointProperties.mT_ParentBodyToJoint.translation()
      = Eigen::Vector3d(0.25, 0.0, 0.3);
  armJointProperties.mT_ChildBodyToJoint.translation()
      = Eigen::Vector3d(0.0, 0.0, 0.15);
  auto armPair = robot->createJointAndBodyNodePair<RevoluteJoint>(
      base, armJointProperties);
  Inertia armInertia;
  armInertia.setMass(1.0);
  armInertia.setMoment(
      BoxShape::computeInertia(Eigen::Vector3d(0.04, 0.04, 0.3), 1.0));
  armPair.second->setInertia(armInertia);

  *baseOut = base;
  *armJointOut = armPair.first;
  return robot;
}

// A 1 kg ball of radius 0.2 m resting on the floor, with a 2 kg bob that hangs
// 0.12 m below its center on a revolute joint about y, released at the given
// angle. The bob has no collision shape.
SkeletonPtr createPendulumBall(double friction, double angle)
{
  auto ball = Skeleton::create("pendulum_ball");
  auto ballPair = ball->createJointAndBodyNodePair<FreeJoint>(nullptr);
  constexpr double kRadius = 0.2;
  auto ballShape = std::make_shared<SphereShape>(kRadius);
  ballPair.second
      ->createShapeNodeWith<CollisionAspect, DynamicsAspect>(ballShape)
      ->getDynamicsAspect()
      ->setFrictionCoeff(friction);
  Inertia ballInertia;
  ballInertia.setMass(1.0);
  ballInertia.setMoment(ballShape->computeInertia(1.0));
  ballPair.second->setInertia(ballInertia);
  Eigen::Isometry3d ballTf = Eigen::Isometry3d::Identity();
  ballTf.translation().z() = kRadius - 1e-6;
  FreeJoint::setTransformOf(ballPair.first, ballTf);

  RevoluteJoint::Properties pendulumProperties;
  pendulumProperties.mAxis = Eigen::Vector3d::UnitY();
  pendulumProperties.mT_ChildBodyToJoint.translation()
      = Eigen::Vector3d(0.0, 0.0, 0.12);
  auto pendulumPair = ball->createJointAndBodyNodePair<RevoluteJoint>(
      ballPair.second, pendulumProperties);
  Inertia bobInertia;
  bobInertia.setMass(2.0);
  bobInertia.setMoment(SphereShape::computeInertia(0.03, 2.0));
  pendulumPair.second->setInertia(bobInertia);
  pendulumPair.first->setPosition(0, angle);
  return ball;
}

double computeMechanicalEnergy(const Skeleton& skeleton)
{
  return skeleton.computeKineticEnergy() + skeleton.computePotentialEnergy();
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
// The base of gz-physics' JointDetach model rests on its plate while its arm
// swings, and keeps the observable upward separation velocity of the default
// Baumgarte correction. The plate starts 1e-5 m into the floor, so that its
// first contacts do not hinge on detecting an exact tangency. With FCL mesh
// cylinders, DART 6.19.4's default, the base then moves as in 6.19.4: after 10
// steps at (-4.0e-9, 3.4e-8) m/s, the second being the arm's recoil, and
// tilting at (-6.2e-10, 2.1e-7) rad/s. The bounds are about five times the
// largest values that starting the base up to 1e-6 m higher or lower, or tilted
// by up to 1e-6 rad, gives here and in 6.19.4 (2.1e-7 m/s and 8.9e-6 rad/s).
// With 6.20's analytic FCL cylinder the plate rests on a few contact points and
// the base rocks, at 1.8e-4 rad/s after 10 steps.
TEST(Issue201, ShallowSupportedFreeRootDoesNotDriftSideways)
{
  auto world = simulation::World::create();
  auto detector = collision::FCLCollisionDetector::create();
  detector->setPrimitiveShapeType(collision::FCLCollisionDetector::MESH);
  world->getConstraintSolver()->setCollisionDetector(detector);
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

  Eigen::Isometry3d baseTf = baseBody->getTransform();
  baseTf.translation().z() -= 1e-5;
  FreeJoint::setTransformOf(baseBody, baseTf);

  for (std::size_t i = 0; i < 10; ++i)
    world->step();

  EXPECT_LT(upperJoint->getVelocity(0), 0.0);
  EXPECT_GT(baseBody->getLinearVelocity().z(), 1e-5);
  EXPECT_NEAR(baseBody->getLinearVelocity().x(), 0.0, 1e-6);
  EXPECT_NEAR(baseBody->getLinearVelocity().y(), 0.0, 1e-6);
  EXPECT_NEAR(baseBody->getAngularVelocity().x(), 0.0, 5e-5);
  EXPECT_NEAR(baseBody->getAngularVelocity().y(), 0.0, 5e-5);
}

//==============================================================================
// A box seeded to slide at 5e-6 m/s and to tilt at 5e-5 rad/s on a frictionless
// floor keeps sliding, while the floor stops the tilt into it, as in DART
// 6.19.4, which also stops the tilt.
TEST(Issue201, ShallowSupportKeepsIntentionalLowSpeedSlide)
{
  auto world = simulation::World::create();
  auto floor = createFloor();
  floor->setMobile(false);
  world->addSkeleton(floor);

  const double penetration = 5e-5;
  auto box = createBox(kBoxSize / 2.0 - penetration);
  world->addSkeleton(box);

  auto* rootBody = box->getRootBodyNode();
  ASSERT_NE(rootBody, nullptr);
  auto* rootJoint = dynamic_cast<FreeJoint*>(rootBody->getParentJoint());
  ASSERT_NE(rootJoint, nullptr);

  const double lateralSpeed = 5e-6;
  const double tiltSpeed = 5e-5;
  rootJoint->setLinearVelocity(
      Eigen::Vector3d(lateralSpeed, 0.0, 0.0), Frame::World(), Frame::World());
  rootJoint->setAngularVelocity(
      Eigen::Vector3d(0.0, tiltSpeed, 0.0), Frame::World(), Frame::World());

  for (std::size_t i = 0; i < 3; ++i)
    world->step();

  ASSERT_GT(world->getLastCollisionResult().getNumContacts(), 0u);
  EXPECT_NEAR(rootBody->getLinearVelocity().x(), lateralSpeed, 1e-8);
  EXPECT_LT(std::abs(rootBody->getAngularVelocity().y()), 0.1 * tiltSpeed);
}

//==============================================================================
// With the linear sleep threshold tuned below the default to 1e-5 m/s, and no
// dwell, a box sliding at 8e-6 m/s on a frictionless floor after World::reset()
// keeps sliding and does not become a sleep candidate: the final-quiet bound
// follows the tuned threshold instead of a fixed default.
TEST(Issue201, ShallowSupportSlideRespectsTunedSleepThresholds)
{
  auto world = simulation::World::create();
  auto options = world->getDeactivationOptions();
  options.mLinearSpeedThreshold = 1e-5;
  options.mTimeUntilSleep = 0.0;
  world->setDeactivationOptions(options);

  auto floor = createFloor();
  floor->setMobile(false);
  world->addSkeleton(floor);

  const double penetration = 5e-5;
  auto box = createBox(kBoxSize / 2.0 - penetration);
  world->addSkeleton(box);

  auto* rootBody = box->getRootBodyNode();
  ASSERT_NE(rootBody, nullptr);
  auto* rootJoint = dynamic_cast<FreeJoint*>(rootBody->getParentJoint());
  ASSERT_NE(rootJoint, nullptr);

  const double lateralSpeed = 8e-6;
  rootJoint->setLinearVelocity(
      Eigen::Vector3d(lateralSpeed, 0.0, 0.0), Frame::World(), Frame::World());
  world->reset();
  world->step();

  ASSERT_GT(world->getLastCollisionResult().getNumContacts(), 0u);
  EXPECT_NEAR(rootBody->getLinearVelocity().x(), lateralSpeed, 1e-8);
  EXPECT_FALSE(box->isSleepCandidate());
}

//==============================================================================
// World::reset() keeps the velocities that the caller set: a box sliding at
// 5e-5 m/s on a frictionless floor keeps that speed on the step after it.
TEST(Issue201, ShallowSupportKeepsLowSpeedSlideAfterReset)
{
  auto world = simulation::World::create();
  auto floor = createFloor();
  floor->setMobile(false);
  world->addSkeleton(floor);

  const double penetration = 5e-5;
  auto box = createBox(kBoxSize / 2.0 - penetration);
  world->addSkeleton(box);

  auto* rootBody = box->getRootBodyNode();
  ASSERT_NE(rootBody, nullptr);
  auto* rootJoint = dynamic_cast<FreeJoint*>(rootBody->getParentJoint());
  ASSERT_NE(rootJoint, nullptr);

  const double lateralSpeed = 5e-5;
  rootJoint->setLinearVelocity(
      Eigen::Vector3d(lateralSpeed, 0.0, 0.0), Frame::World(), Frame::World());
  world->reset();
  world->step();

  ASSERT_GT(world->getLastCollisionResult().getNumContacts(), 0u);
  EXPECT_NEAR(rootBody->getLinearVelocity().x(), lateralSpeed, 1e-8);
}

//==============================================================================
// A box that lands on a frictionless floor while sliding at 5e-6 m/s and
// tilting at 5e-5 rad/s keeps sliding, while the landing stops the tilt, as in
// DART 6.19.4.
TEST(Issue201, ShallowSupportKeepsLowSpeedSlideAfterLanding)
{
  auto world = simulation::World::create();
  world->setTimeStep(0.001);
  auto floor = createFloor();
  floor->setMobile(false);
  world->addSkeleton(floor);

  const double initialGap = 5e-5;
  auto box = createBox(kBoxSize / 2.0 + initialGap);
  world->addSkeleton(box);

  auto* rootBody = box->getRootBodyNode();
  ASSERT_NE(rootBody, nullptr);
  auto* rootJoint = dynamic_cast<FreeJoint*>(rootBody->getParentJoint());
  ASSERT_NE(rootJoint, nullptr);

  const double lateralSpeed = 5e-6;
  const double tiltSpeed = 5e-5;
  rootJoint->setLinearVelocity(
      Eigen::Vector3d(lateralSpeed, 0.0, -0.04),
      Frame::World(),
      Frame::World());
  rootJoint->setAngularVelocity(
      Eigen::Vector3d(0.0, tiltSpeed, 0.0), Frame::World(), Frame::World());

  bool sawAirborneStep = false;
  for (std::size_t i = 0; i < 10; ++i) {
    world->step();
    if (world->getLastCollisionResult().getNumContacts() == 0u) {
      sawAirborneStep = true;
      continue;
    }

    if (sawAirborneStep)
      break;
  }

  ASSERT_TRUE(sawAirborneStep);
  ASSERT_GT(world->getLastCollisionResult().getNumContacts(), 0u);
  EXPECT_NEAR(rootBody->getLinearVelocity().x(), lateralSpeed, 1e-8);
  EXPECT_LT(std::abs(rootBody->getAngularVelocity().y()), 0.1 * tiltSpeed);
}

//==============================================================================
// Setting the velocity of a sliding, tilting box to zero stops it: the
// frictionless floor gives it no lateral or tilt motion on the next step.
TEST(Issue201, ShallowSupportKeepsIntentionalStop)
{
  auto world = simulation::World::create();
  auto floor = createFloor();
  floor->setMobile(false);
  world->addSkeleton(floor);

  const double penetration = 5e-5;
  auto box = createBox(kBoxSize / 2.0 - penetration);
  world->addSkeleton(box);

  auto* rootBody = box->getRootBodyNode();
  ASSERT_NE(rootBody, nullptr);
  auto* rootJoint = dynamic_cast<FreeJoint*>(rootBody->getParentJoint());
  ASSERT_NE(rootJoint, nullptr);

  rootJoint->setLinearVelocity(
      Eigen::Vector3d(5e-6, 0.0, 0.0), Frame::World(), Frame::World());
  rootJoint->setAngularVelocity(
      Eigen::Vector3d(0.0, 5e-5, 0.0), Frame::World(), Frame::World());
  world->step();

  ASSERT_GT(world->getLastCollisionResult().getNumContacts(), 0u);
  EXPECT_GT(std::abs(rootBody->getLinearVelocity().x()), 0.0);
  EXPECT_GT(std::abs(rootBody->getAngularVelocity().y()), 0.0);

  rootJoint->setLinearVelocity(
      Eigen::Vector3d::Zero(), Frame::World(), Frame::World());
  rootJoint->setAngularVelocity(
      Eigen::Vector3d::Zero(), Frame::World(), Frame::World());
  world->step();

  ASSERT_GT(world->getLastCollisionResult().getNumContacts(), 0u);
  EXPECT_NEAR(rootBody->getLinearVelocity().x(), 0.0, 1e-8);
  EXPECT_NEAR(rootBody->getAngularVelocity().y(), 0.0, 1e-8);
}

//==============================================================================
// World::step(false) keeps the commands of a velocity-actuated free root that
// rests on a floor as the caller set them.
TEST(Issue201, ShallowSupportKeepsVelocityCommands)
{
  auto world = simulation::World::create();
  auto floor = createFloor();
  floor->setMobile(false);
  world->addSkeleton(floor);

  const double penetration = 5e-5;
  auto box = createBox(kBoxSize / 2.0 - penetration);
  world->addSkeleton(box);

  auto* rootBody = box->getRootBodyNode();
  ASSERT_NE(rootBody, nullptr);
  auto* rootJoint = dynamic_cast<FreeJoint*>(rootBody->getParentJoint());
  ASSERT_NE(rootJoint, nullptr);
  rootJoint->setActuatorType(Joint::VELOCITY);

  const Eigen::VectorXd expectedCommands = Eigen::VectorXd::Zero(
      static_cast<Eigen::Index>(rootJoint->getNumDofs()));
  rootJoint->setCommands(expectedCommands);

  world->step(false);

  ASSERT_GT(world->getLastCollisionResult().getNumContacts(), 0u);
  for (std::size_t i = 0; i < rootJoint->getNumDofs(); ++i) {
    EXPECT_EQ(
        rootJoint->getCommand(i),
        expectedCommands[static_cast<Eigen::Index>(i)]);
  }
}

//==============================================================================
// A resting support that received a solver impulse wakes on the next step, and
// a box sliding at 5e-6 m/s on its frictionless top keeps sliding.
TEST(Issue201, ShallowSupportWakesImpulseAppliedRestingSupport)
{
  auto world = simulation::World::create();

  const double penetration = 5e-5;
  auto support = createBox(kBoxSize / 2.0);
  auto box = createBox(1.5 * kBoxSize - penetration);
  world->addSkeleton(support);
  world->addSkeleton(box);

  auto* rootBody = box->getRootBodyNode();
  ASSERT_NE(rootBody, nullptr);
  auto* rootJoint = dynamic_cast<FreeJoint*>(rootBody->getParentJoint());
  ASSERT_NE(rootJoint, nullptr);

  const double lateralSpeed = 5e-6;
  rootJoint->setLinearVelocity(
      Eigen::Vector3d(lateralSpeed, 0.0, 0.0), Frame::World(), Frame::World());
  world->reset();

  support->setResting(true);
  support->setImpulseApplied(true);
  world->step();

  ASSERT_GT(world->getLastCollisionResult().getNumContacts(), 0u);
  EXPECT_FALSE(support->isResting());
  EXPECT_NEAR(rootBody->getLinearVelocity().x(), lateralSpeed, 1e-8);
}

//==============================================================================
// A frictionless contact with the underside of an immobile ceiling exerts no
// lateral impulse, so a free box touching it keeps sliding at 5e-6 m/s.
TEST(Issue201, CeilingContactKeepsLowSpeedSlide)
{
  auto world = simulation::World::create();
  world->setTimeStep(0.001);

  auto ceiling = Skeleton::create("ceiling");
  {
    auto pair = ceiling->createJointAndBodyNodePair<WeldJoint>(nullptr);
    auto shape = std::make_shared<BoxShape>(
        Eigen::Vector3d(kFloorSize, kFloorSize, kFloorHeight));
    auto* shapeNode
        = pair.second->createShapeNodeWith<CollisionAspect, DynamicsAspect>(
            shape);
    auto* dynamics = shapeNode->getDynamicsAspect();
    dynamics->setFrictionCoeff(0.0);
    dynamics->setRestitutionCoeff(0.0);
    Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
    // Ceiling underside at z = 1.0, with the ceiling body's origin below the
    // free box.
    tf.translation().z() = 1.0 + kFloorHeight / 2.0;
    shapeNode->setRelativeTransform(tf);
  }
  ceiling->setMobile(false);
  world->addSkeleton(ceiling);

  // Free box whose top face penetrates the ceiling underside by 5e-5 m.
  const double penetration = 5e-5;
  auto box = createBox(1.0 - kBoxSize / 2.0 + penetration);
  world->addSkeleton(box);

  auto* rootBody = box->getRootBodyNode();
  ASSERT_NE(rootBody, nullptr);
  auto* rootJoint = dynamic_cast<FreeJoint*>(rootBody->getParentJoint());
  ASSERT_NE(rootJoint, nullptr);

  const double lateralSpeed = 5e-6;
  rootJoint->setLinearVelocity(
      Eigen::Vector3d(lateralSpeed, 0.0, 0.0), Frame::World(), Frame::World());

  world->step();

  // Precondition: the box-ceiling contact must be present this step.
  ASSERT_GT(world->getLastCollisionResult().getNumContacts(), 0u);
  EXPECT_NEAR(rootBody->getLinearVelocity().x(), lateralSpeed, 1e-8);
}

#if HAVE_BULLET
//==============================================================================
// With allowNegativePenetrationDepthContacts enabled, Bullet keeps proximity
// hits with negative penetration depth in the collision result, but the
// constraint solver creates no contact constraints for them, so a free body
// hovering above a support keeps sliding at 5e-6 m/s.
TEST(Issue201, NegativeDepthProximityContactKeepsLowSpeedSlide)
{
  auto world = simulation::World::create();
  world->setTimeStep(0.001);
  auto* solver = world->getConstraintSolver();
  ASSERT_NE(nullptr, solver);
  solver->setCollisionDetector(collision::BulletCollisionDetector::create());
  solver->getCollisionOption().allowNegativePenetrationDepthContacts = true;

  constexpr double kCylinderRadius = 0.5;
  constexpr double kCylinderHeight = 1.0;
  constexpr double kProximityGap = 0.01;

  auto support = Skeleton::create("support");
  {
    auto pair = support->createJointAndBodyNodePair<WeldJoint>(nullptr);
    auto shape
        = std::make_shared<CylinderShape>(kCylinderRadius, kCylinderHeight);
    pair.second->createShapeNodeWith<CollisionAspect, DynamicsAspect>(shape);
  }
  support->setMobile(false);
  world->addSkeleton(support);

  auto hoverer = Skeleton::create("hoverer");
  {
    auto pair = hoverer->createJointAndBodyNodePair<FreeJoint>(nullptr);
    auto shape
        = std::make_shared<CylinderShape>(kCylinderRadius, kCylinderHeight);
    pair.second->createShapeNodeWith<CollisionAspect, DynamicsAspect>(shape);

    Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
    tf.translation().z() = kCylinderHeight + kProximityGap;
    FreeJoint::setTransformOf(pair.first, tf);
  }
  world->addSkeleton(hoverer);

  auto* rootBody = hoverer->getRootBodyNode();
  ASSERT_NE(rootBody, nullptr);
  auto* rootJoint = dynamic_cast<FreeJoint*>(rootBody->getParentJoint());
  ASSERT_NE(rootJoint, nullptr);

  const double lateralSpeed = 5e-6;
  rootJoint->setLinearVelocity(
      Eigen::Vector3d(lateralSpeed, 0.0, 0.0), Frame::World(), Frame::World());

  world->step();

  // Precondition: the proximity contact must be present with negative depth.
  const auto& result = world->getLastCollisionResult();
  bool hasNegativeDepthContact = false;
  for (std::size_t i = 0; i < result.getNumContacts(); ++i) {
    if (result.getContact(i).penetrationDepth < 0.0)
      hasNegativeDepthContact = true;
  }
  ASSERT_TRUE(hasNegativeDepthContact);
  EXPECT_NEAR(rootBody->getLinearVelocity().x(), lateralSpeed, 1e-8);
}
#endif

//==============================================================================
// With its arm welded, the pendulum base is rigid, but its mass is off-axis:
// the arm moves its center of mass away from the root origin and couples tilt
// with yaw in its inertia. On a frictionless floor it must neither spin up nor
// gain energy, with every collision detector, as in DART 6.19.4. A drift clamp
// that edited only the root's tilt velocity kept the yaw rate coupled to it and
// spun this body up to 0.3 rad/s in 2 s with FCL.
TEST(Issue201, ShallowSupportKeepsOffAxisRigidBodyMomentum)
{
  for (const auto& collisionDetector : getCollisionDetectorNames()) {
    SCOPED_TRACE(collisionDetector);
    auto world = createArticulatedTreeWorld(collisionDetector, 0.0, false);

    BodyNode* baseBody = nullptr;
    BodyNode* upperLink = nullptr;
    RevoluteJoint* upperJoint = nullptr;
    auto base = createSupportedPendulumBase(
        &baseBody, &upperLink, &upperJoint, PendulumArm::Welded);
    world->addSkeleton(base);

    for (std::size_t i = 0; i < 2000; ++i)
      world->step();

    ASSERT_GT(world->getLastCollisionResult().getNumContacts(), 0u);
    EXPECT_LT(std::abs(baseBody->getAngularVelocity().z()), 1e-2);
    EXPECT_LT(base->computeKineticEnergy(), 1e-4);
  }
}

//==============================================================================
// Seeded low-speed lateral motion on a frictionless level support has no
// physical cause to stop, in any direction and over many steps. The support
// stops a seeded tilt into it, and the contact impulses that do so must not
// leak into the lateral motion, as in DART 6.19.4, which also stops the tilt.
TEST(Issue201, ShallowSupportPreservesLowSpeedMotionInAnyDirection)
{
  const double lateralSpeed = 5e-6;
  const double tiltSpeed = 5e-5;
  for (int k = 0; k < 8; ++k) {
    const double angle = k * kHalfPi / 2.0;
    SCOPED_TRACE(angle);
    const Eigen::Vector3d direction(std::cos(angle), std::sin(angle), 0.0);
    const Eigen::Vector3d tiltAxis = Eigen::Vector3d::UnitZ().cross(direction);

    auto world = simulation::World::create();
    auto floor = createFloor();
    floor->setMobile(false);
    world->addSkeleton(floor);

    const double penetration = 5e-5;
    auto box = createBox(kBoxSize / 2.0 - penetration);
    world->addSkeleton(box);

    auto* rootBody = box->getRootBodyNode();
    ASSERT_NE(rootBody, nullptr);
    auto* rootJoint = dynamic_cast<FreeJoint*>(rootBody->getParentJoint());
    ASSERT_NE(rootJoint, nullptr);
    rootJoint->setLinearVelocity(
        lateralSpeed * direction, Frame::World(), Frame::World());
    rootJoint->setAngularVelocity(
        tiltSpeed * tiltAxis, Frame::World(), Frame::World());

    double maxLateralError = 0.0;
    for (std::size_t i = 0; i < 200; ++i) {
      world->step();
      const Eigen::Vector3d velocity = rootBody->getLinearVelocity();
      maxLateralError = std::max(
          maxLateralError,
          (velocity.head<2>() - lateralSpeed * direction.head<2>()).norm());
    }

    ASSERT_GT(world->getLastCollisionResult().getNumContacts(), 0u);
    EXPECT_LT(maxLateralError, 1e-8);
    EXPECT_NEAR(rootBody->getAngularVelocity().dot(tiltAxis), 0.0, 1e-8);
  }
}

//==============================================================================
// Gravity with a component along the support physically drives slow motion. A
// light sphere on level ground under gravity (0, 0.1, -0.1), as in gz-sim's
// imu_rotating_demo.sdf, gains only about 1e-4 m/s per step, and must roll as
// rolling without slipping predicts.
TEST(Issue201, ShallowSupportKeepsRollingUnderTangentialGravity)
{
  auto world = simulation::World::create();
  world->setTimeStep(0.001);
  world->setGravity(Eigen::Vector3d(0.0, 0.1, -0.1));
  auto options = world->getDeactivationOptions();
  options.mEnabled = false;
  world->setDeactivationOptions(options);

  auto floor = createFloor();
  floor->setMobile(false);
  floor->getBodyNode(0)->getShapeNode(0)->getDynamicsAspect()->setFrictionCoeff(
      1.0);
  world->addSkeleton(floor);

  constexpr double kRadius = 1.0;
  constexpr double kMass = 0.1;
  constexpr double kMoment = 1.66667e-4;
  auto sphere = Skeleton::create("sphere");
  auto pair = sphere->createJointAndBodyNodePair<FreeJoint>(nullptr);
  auto* body = pair.second;
  Inertia inertia;
  inertia.setMass(kMass);
  inertia.setMoment(kMoment * Eigen::Matrix3d::Identity());
  body->setInertia(inertia);
  body->createShapeNodeWith<CollisionAspect, DynamicsAspect>(
      std::make_shared<SphereShape>(kRadius));
  Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
  tf.translation().z() = kRadius;
  FreeJoint::setTransformOf(pair.first, tf);
  world->addSkeleton(sphere);

  constexpr std::size_t kSteps = 200;
  for (std::size_t i = 0; i < kSteps; ++i)
    world->step();

  ASSERT_GT(world->getLastCollisionResult().getNumContacts(), 0u);
  // Rolling without slipping: v(t) = g_t t / (1 + I / (m r^2)).
  const double expectedSpeed = 0.1 * static_cast<double>(kSteps)
                               * world->getTimeStep()
                               / (1.0 + kMoment / (kMass * kRadius * kRadius));
  EXPECT_NEAR(
      body->getLinearVelocity().y(), expectedSpeed, 0.01 * expectedSpeed);
  EXPECT_NEAR(
      body->getAngularVelocity().x(),
      -expectedSpeed / kRadius,
      0.01 * expectedSpeed / kRadius);
}

//==============================================================================
// Static friction cancels a small horizontal push on a box resting on a level
// floor, so the box must not creep.
TEST(Issue201, ShallowSupportLetsFrictionResistAppliedForce)
{
  auto world = simulation::World::create();
  world->setTimeStep(0.001);
  auto options = world->getDeactivationOptions();
  options.mEnabled = false;
  world->setDeactivationOptions(options);

  auto floor = createFloor();
  floor->setMobile(false);
  floor->getBodyNode(0)->getShapeNode(0)->getDynamicsAspect()->setFrictionCoeff(
      1.0);
  world->addSkeleton(floor);

  const double penetration = 5e-5;
  auto box = createBox(kBoxSize / 2.0 - penetration);
  box->getBodyNode(0)->getShapeNode(0)->getDynamicsAspect()->setFrictionCoeff(
      1.0);
  world->addSkeleton(box);

  auto* rootBody = box->getRootBodyNode();
  ASSERT_NE(rootBody, nullptr);
  const double startX = rootBody->getTransform().translation().x();

  // 0.1 N on a 1 kg box: 1e-4 m/s per step, far below static friction.
  for (std::size_t i = 0; i < 300; ++i) {
    rootBody->addExtForce(Eigen::Vector3d(0.1, 0.0, 0.0));
    world->step();
  }

  ASSERT_GT(world->getLastCollisionResult().getNumContacts(), 0u);
  EXPECT_NEAR(rootBody->getTransform().translation().x(), startX, 1e-9);
  EXPECT_NEAR(rootBody->getLinearVelocity().x(), 0.0, 1e-8);
}

//==============================================================================
// Kinetic friction that removes less than 2e-4 m/s per step is still Coulomb
// friction. A box sliding at 0.1 m/s on a mu 0.01 floor loses mu g dt, about
// 1e-4 m/s, per 1 ms step, and stops v^2 / (2 mu g) = 0.051 m further on. A
// sphere launched sliding at 0.01 m/s without spin turns that friction into
// spin until it rolls at 5/7 of its launch speed, losing 2/7 of its kinetic
// energy.
TEST(Issue201, ShallowSupportKeepsWeakKineticFriction)
{
  constexpr double kFriction = 0.01;
  const auto createWorld = [&]() {
    auto world = simulation::World::create();
    world->setTimeStep(0.001);
    auto options = world->getDeactivationOptions();
    options.mEnabled = false;
    world->setDeactivationOptions(options);
    auto floor = createFloor();
    floor->setMobile(false);
    floor->getBodyNode(0)
        ->getShapeNode(0)
        ->getDynamicsAspect()
        ->setFrictionCoeff(kFriction);
    world->addSkeleton(floor);
    return world;
  };

  {
    SCOPED_TRACE("sliding box");
    auto world = createWorld();
    const double penetration = 5e-5;
    auto box = createBox(kBoxSize / 2.0 - penetration);
    box->getBodyNode(0)->getShapeNode(0)->getDynamicsAspect()->setFrictionCoeff(
        kFriction);
    world->addSkeleton(box);

    auto* rootBody = box->getRootBodyNode();
    auto* rootJoint = dynamic_cast<FreeJoint*>(rootBody->getParentJoint());
    ASSERT_NE(rootJoint, nullptr);
    constexpr double kSpeed = 0.1;
    rootJoint->setLinearVelocity(
        Eigen::Vector3d(kSpeed, 0.0, 0.0), Frame::World(), Frame::World());
    const double startX = rootBody->getTransform().translation().x();

    // It stops after kSpeed / (mu g) = 1.02 s.
    for (std::size_t i = 0; i < 1500; ++i)
      world->step();

    ASSERT_GT(world->getLastCollisionResult().getNumContacts(), 0u);
    const double expected
        = kSpeed * kSpeed / (2.0 * kFriction * -world->getGravity().z());
    EXPECT_NEAR(
        rootBody->getTransform().translation().x() - startX,
        expected,
        0.02 * expected);
    EXPECT_NEAR(rootBody->getLinearVelocity().x(), 0.0, 1e-6);
  }

  {
    SCOPED_TRACE("sliding sphere");
    auto world = createWorld();
    constexpr double kRadius = 0.2;
    auto sphere = Skeleton::create("sphere");
    auto pair = sphere->createJointAndBodyNodePair<FreeJoint>(nullptr);
    auto shape = std::make_shared<SphereShape>(kRadius);
    pair.second->createShapeNodeWith<CollisionAspect, DynamicsAspect>(shape)
        ->getDynamicsAspect()
        ->setFrictionCoeff(kFriction);
    Inertia inertia;
    inertia.setMass(1.0);
    inertia.setMoment(shape->computeInertia(1.0));
    pair.second->setInertia(inertia);
    Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
    tf.translation().z() = kRadius - 1e-6;
    FreeJoint::setTransformOf(pair.first, tf);
    world->addSkeleton(sphere);

    constexpr double kSpeed = 0.01;
    pair.first->setLinearVelocity(
        Eigen::Vector3d(kSpeed, 0.0, 0.0), Frame::World(), Frame::World());
    const double startEnergy = sphere->computeKineticEnergy();

    // It rolls after 2 kSpeed / (7 mu g) = 0.029 s.
    for (std::size_t i = 0; i < 200; ++i)
      world->step();

    ASSERT_GT(world->getLastCollisionResult().getNumContacts(), 0u);
    const double rollingSpeed = 5.0 / 7.0 * kSpeed;
    EXPECT_NEAR(
        pair.second->getLinearVelocity().x(),
        rollingSpeed,
        0.01 * rollingSpeed);
    EXPECT_NEAR(
        pair.second->getAngularVelocity().y(),
        rollingSpeed / kRadius,
        0.01 * rollingSpeed / kRadius);
    EXPECT_NEAR(
        sphere->computeKineticEnergy(),
        5.0 / 7.0 * startEnergy,
        0.01 * startEnergy);
  }
}

//==============================================================================
// Once an applied force or torque that friction and the support cancelled
// stops, the box stays put instead of creeping or tipping.
TEST(Issue201, ShallowSupportDoesNotCreepAfterDriveStops)
{
  for (const bool torque : {false, true}) {
    SCOPED_TRACE(torque ? "torque" : "force");
    auto world = simulation::World::create();
    world->setTimeStep(0.001);
    auto options = world->getDeactivationOptions();
    options.mEnabled = false;
    world->setDeactivationOptions(options);

    auto floor = createFloor();
    floor->setMobile(false);
    floor->getBodyNode(0)
        ->getShapeNode(0)
        ->getDynamicsAspect()
        ->setFrictionCoeff(1.0);
    world->addSkeleton(floor);

    const double penetration = 5e-5;
    auto box = createBox(kBoxSize / 2.0 - penetration);
    box->getBodyNode(0)->getShapeNode(0)->getDynamicsAspect()->setFrictionCoeff(
        1.0);
    world->addSkeleton(box);

    auto* rootBody = box->getRootBodyNode();
    ASSERT_NE(rootBody, nullptr);

    // 0.1 N or 2e-3 N m on a 1 kg box: far below sliding and tipping.
    for (std::size_t i = 0; i < 100; ++i) {
      if (torque)
        rootBody->addExtTorque(Eigen::Vector3d(0.0, 2e-3, 0.0));
      else
        rootBody->addExtForce(Eigen::Vector3d(0.1, 0.0, 0.0));
      world->step();
    }

    const Eigen::Isometry3d released = rootBody->getTransform();
    for (std::size_t i = 0; i < 1000; ++i)
      world->step();

    ASSERT_GT(world->getLastCollisionResult().getNumContacts(), 0u);
    const Eigen::Isometry3d end = rootBody->getTransform();
    EXPECT_LT(
        (end.translation() - released.translation()).head<2>().norm(), 1e-9);
    EXPECT_LT(
        Eigen::AngleAxisd(end.linear() * released.linear().transpose()).angle(),
        1e-9);
  }
}

//==============================================================================
// Child links push the free root of an articulated tree: an arm swinging on a
// passive spring joint pushes its base back and forth. On a frictionless floor
// the base recoils while the tree's center of mass stays in place. On a mu 1
// floor, friction holds the base without damping the arm. Both hold with every
// collision detector and with the PGS LCP solver, whose friction leaves a small
// residual slip.
TEST(Issue201, ShallowSupportKeepsArticulatedTreeMomentum)
{
  std::vector<std::pair<std::string, bool>> configurations;
  for (const auto& collisionDetector : getCollisionDetectorNames())
    configurations.emplace_back(collisionDetector, false);
  configurations.emplace_back("fcl", true);

  for (const auto& [collisionDetector, pgs] : configurations) {
    for (const double friction : {0.0, 1.0}) {
      for (const bool deactivation : {true, false}) {
        SCOPED_TRACE(collisionDetector + (pgs ? " + PGS" : ""));
        SCOPED_TRACE(friction);
        SCOPED_TRACE(deactivation ? "deactivation on" : "deactivation off");
        auto world = createArticulatedTreeWorld(
            collisionDetector, friction, deactivation);
        if (pgs) {
          auto* solver = dynamic_cast<constraint::BoxedLcpConstraintSolver*>(
              world->getConstraintSolver());
          ASSERT_NE(solver, nullptr);
          solver->setBoxedLcpSolver(
              std::make_shared<constraint::PgsBoxedLcpSolver>());
        }

        // The arm's joint spring (5 N m/rad) pulls it toward 0.05 rad, so it
        // swings from the start.
        BodyNode* base = nullptr;
        RevoluteJoint* armJoint = nullptr;
        auto robot = createArmRobot(friction, &base, &armJoint);
        armJoint->setSpringStiffness(0, 5.0);
        armJoint->setRestPosition(0, 0.05);
        world->addSkeleton(robot);

        // The first contact may move the base before friction holds it, so
        // the base drift counts from step 10.
        const Eigen::Vector3d startCom = robot->getCOM();
        Eigen::Vector3d startBase = base->getTransform().translation();
        double maxBaseDrift = 0.0;
        double maxComDrift = 0.0;
        double maxArmAngle = 0.0;
        for (std::size_t i = 1; i <= 5000; ++i) {
          world->step();
          if (i == 10)
            startBase = base->getTransform().translation();
          if (i >= 10) {
            maxBaseDrift = std::max(
                maxBaseDrift,
                (base->getTransform().translation() - startBase)
                    .head<2>()
                    .norm());
          }
          maxComDrift = std::max(
              maxComDrift, (robot->getCOM() - startCom).head<2>().norm());
          maxArmAngle = std::max(maxArmAngle, armJoint->getPosition(0));
        }

        ASSERT_GT(world->getLastCollisionResult().getNumContacts(), 0u);
        EXPECT_FALSE(robot->isResting());
        // Gravity and the spring balance the arm at about 0.039 rad. Released
        // from 0, it keeps swinging to about twice that.
        EXPECT_GT(maxArmAngle, 0.07);
        if (friction == 0.0) {
          EXPECT_LT(maxComDrift, 1e-6);
          EXPECT_GT(maxBaseDrift, 1e-3);
        } else {
          EXPECT_LT(maxBaseDrift, 1e-6);
        }
      }
    }
  }
}

//==============================================================================
// An arm released from 1 rad swings far enough to rock its base on a
// frictionless floor, so the support contacts keep changing the base's
// rotation. The tree must move as in DART 6.19.4, with every collision
// detector: its center of mass shifts by the 1.84e-3 m that the rocking
// contacts give there, not more, and it gains no energy beyond the time
// stepping's. Two such robots share the floor, one swinging each way.
TEST(Issue201, ShallowSupportKeepsRockingArticulatedTreeMomentum)
{
  for (const auto& collisionDetector : getCollisionDetectorNames()) {
    SCOPED_TRACE(collisionDetector);
    auto world = createArticulatedTreeWorld(collisionDetector, 0.0, true);
    std::vector<SkeletonPtr> robots;
    for (const double armAngle : {1.0, -1.0}) {
      BodyNode* base = nullptr;
      RevoluteJoint* armJoint = nullptr;
      auto robot = createArmRobot(0.0, &base, &armJoint);
      armJoint->setPosition(0, armAngle);
      Eigen::Isometry3d baseTf = base->getTransform();
      baseTf.translation().x() += 1.5 * static_cast<double>(robots.size());
      FreeJoint::setTransformOf(base, baseTf);
      world->addSkeleton(robot);
      robots.push_back(robot);
    }

    std::vector<Eigen::Vector3d> startComs;
    std::vector<double> startEnergies;
    for (const auto& robot : robots) {
      startComs.push_back(robot->getCOM());
      startEnergies.push_back(computeMechanicalEnergy(*robot));
    }
    double maxComDrift = 0.0;
    double maxEnergyGain = 0.0;
    for (std::size_t i = 0; i < 5000; ++i) {
      world->step();
      for (std::size_t j = 0; j < robots.size(); ++j) {
        maxComDrift = std::max(
            maxComDrift, (robots[j]->getCOM() - startComs[j]).head<2>().norm());
        maxEnergyGain = std::max(
            maxEnergyGain,
            computeMechanicalEnergy(*robots[j]) - startEnergies[j]);
      }
    }

    ASSERT_GT(world->getLastCollisionResult().getNumContacts(), 0u);
    EXPECT_LT(maxComDrift, 3e-3);
    // The time stepping alone lets the swing's energy rise by about 2.2e-3 J.
    EXPECT_LT(maxEnergyGain, 1e-2);
  }
}

//==============================================================================
// A pendulum swinging inside a ball rolls it back and forth on a floor with
// little friction (mu 0.01). The swing must not turn into a one-sided push
// that rolls the ball away, with every collision detector.
TEST(Issue201, ShallowSupportKeepsPendulumBallInPlace)
{
  for (const auto& collisionDetector : getCollisionDetectorNames()) {
    SCOPED_TRACE(collisionDetector);
    auto world = createArticulatedTreeWorld(collisionDetector, 0.01, true);
    auto ball = createPendulumBall(0.01, 0.3);
    world->addSkeleton(ball);

    const Eigen::Vector3d startCom = ball->getCOM();
    double maxComDrift = 0.0;
    for (std::size_t i = 0; i < 10000; ++i) {
      world->step();
      maxComDrift
          = std::max(maxComDrift, (ball->getCOM() - startCom).head<2>().norm());
    }

    ASSERT_GT(world->getLastCollisionResult().getNumContacts(), 0u);
    EXPECT_LT(maxComDrift, 1.5e-2);
  }
}

//==============================================================================
// On a mu 1 floor, a pendulum swinging 0.005 rad inside the ball rocks it back
// and forth by about 0.7 mm in DART 6.19.4 (2.7 mm with Bullet). The ball's
// tilt rate changes by far less than 5e-4 rad/s on each step, and the ball must
// still rock instead of being held still, with every collision detector.
TEST(Issue201, ShallowSupportLetsPendulumRockBall)
{
  for (const auto& collisionDetector : getCollisionDetectorNames()) {
    SCOPED_TRACE(collisionDetector);
    auto world = createArticulatedTreeWorld(collisionDetector, 1.0, true);
    auto ball = createPendulumBall(1.0, 0.005);
    world->addSkeleton(ball);

    const auto* body = ball->getRootBodyNode();
    double minX = body->getTransform().translation().x();
    double maxX = minX;
    for (std::size_t i = 0; i < 10000; ++i) {
      world->step();
      minX = std::min(minX, body->getTransform().translation().x());
      maxX = std::max(maxX, body->getTransform().translation().x());
    }

    ASSERT_GT(world->getLastCollisionResult().getNumContacts(), 0u);
    EXPECT_GT(maxX - minX, 3.5e-4) << "the ball was held still";
  }
}

//==============================================================================
// A joint constraint couples skeletons as a joint does: a bob hung from a base
// by a BallJointConstraint pushes the base back and forth as it swings on a
// frictionless floor. The base recoils by about 1 cm while the pair's center of
// mass stays put, as in DART 6.19.4, with every collision detector.
TEST(Issue201, ShallowSupportKeepsJointConstraintRecoil)
{
  for (const auto& collisionDetector : getCollisionDetectorNames()) {
    SCOPED_TRACE(collisionDetector);
    auto world = createArticulatedTreeWorld(collisionDetector, 0.0, false);

    // A 5 kg base box rests on the floor. A 1 kg bob without a collision shape
    // hangs 0.3 m below a pivot 0.6 m above the base, released at 0.1 rad.
    const Eigen::Vector3d baseSize(0.4, 0.4, 0.1);
    auto base = Skeleton::create("base");
    auto* baseBody
        = base->createJointAndBodyNodePair<FreeJoint>(nullptr).second;
    baseBody
        ->createShapeNodeWith<CollisionAspect, DynamicsAspect>(
            std::make_shared<BoxShape>(baseSize))
        ->getDynamicsAspect()
        ->setFrictionCoeff(0.0);
    Inertia baseInertia;
    baseInertia.setMass(5.0);
    baseInertia.setMoment(BoxShape::computeInertia(baseSize, 5.0));
    baseBody->setInertia(baseInertia);
    Eigen::Isometry3d baseTf = Eigen::Isometry3d::Identity();
    baseTf.translation().z() = baseSize.z() / 2.0 - 1e-6;
    FreeJoint::setTransformOf(baseBody, baseTf);
    world->addSkeleton(base);

    const Eigen::Vector3d pivot(0.25, 0.0, 0.65);
    auto bob = Skeleton::create("bob");
    auto* bobBody = bob->createJointAndBodyNodePair<FreeJoint>(nullptr).second;
    Inertia bobInertia;
    bobInertia.setMass(1.0);
    bobInertia.setMoment(
        BoxShape::computeInertia(Eigen::Vector3d::Constant(0.05), 1.0));
    bobBody->setInertia(bobInertia);
    Eigen::Isometry3d bobTf = Eigen::Isometry3d::Identity();
    bobTf.translation()
        = pivot + 0.3 * Eigen::Vector3d(std::sin(0.1), 0.0, -std::cos(0.1));
    FreeJoint::setTransformOf(bobBody, bobTf);
    world->addSkeleton(bob);
    world->getConstraintSolver()->addConstraint(
        std::make_shared<constraint::BallJointConstraint>(
            baseBody, bobBody, pivot));

    const auto computeCom = [&]() -> Eigen::Vector3d {
      return (5.0 * baseBody->getCOM() + bobBody->getCOM()) / 6.0;
    };
    const Eigen::Vector3d startBase = baseBody->getTransform().translation();
    const Eigen::Vector3d startCom = computeCom();
    double maxBaseDrift = 0.0;
    double maxComDrift = 0.0;
    for (std::size_t i = 0; i < 2000; ++i) {
      world->step();
      maxBaseDrift = std::max(
          maxBaseDrift,
          (baseBody->getTransform().translation() - startBase)
              .head<2>()
              .norm());
      maxComDrift
          = std::max(maxComDrift, (computeCom() - startCom).head<2>().norm());
    }

    ASSERT_GT(world->getLastCollisionResult().getNumContacts(), 0u);
    EXPECT_GT(maxBaseDrift, 5e-3);
    EXPECT_LT(maxComDrift, 1e-9);
  }
}

//==============================================================================
// Friction holds the base of a robot whose arm hangs at rest from a bracket to
// the side, so the base must not creep: it settles within 5e-10 m over 20 s in
// DART 6.19.4.
TEST(Issue201, ShallowSupportDoesNotCreepUnderHeldArticulatedTree)
{
  auto world = createArticulatedTreeWorld("fcl", 1.0, false);
  BodyNode* base = nullptr;
  RevoluteJoint* armJoint = nullptr;
  auto robot = createArmRobot(1.0, &base, &armJoint);
  armJoint->setSpringStiffness(0, 5.0);
  world->addSkeleton(robot);

  for (std::size_t i = 0; i < 10; ++i)
    world->step();
  const Eigen::Vector3d startBase = base->getTransform().translation();
  for (std::size_t i = 0; i < 20000; ++i)
    world->step();

  ASSERT_GT(world->getLastCollisionResult().getNumContacts(), 0u);
  EXPECT_LT(
      (base->getTransform().translation() - startBase).head<2>().norm(), 1e-9);
  EXPECT_LT(base->getLinearVelocity().head<2>().norm(), 1e-10);
}

//==============================================================================
// The arm robot stands with friction on a rigid 10 kg pallet, and the pallet
// rests on a frictionless floor, so the swinging arm rocks the pallet back and
// forth. The center of mass of robot and pallet must stay put, with every
// collision detector (5.5e-5 m, as in DART 6.19.4). A drift clamp on the pallet
// that cancelled each small recoil and passed the larger ones ran the pair away
// by 6 cm in 10 s.
TEST(Issue201, ShallowSupportKeepsMomentumOfArticulatedTreeOnPallet)
{
  for (const auto& collisionDetector : getCollisionDetectorNames()) {
    SCOPED_TRACE(collisionDetector);
    auto world = createArticulatedTreeWorld(collisionDetector, 0.0, true);

    const Eigen::Vector3d palletSize(1.0, 1.0, 0.1);
    auto pallet = Skeleton::create("pallet");
    auto* palletBody
        = pallet->createJointAndBodyNodePair<FreeJoint>(nullptr).second;
    palletBody
        ->createShapeNodeWith<CollisionAspect, DynamicsAspect>(
            std::make_shared<BoxShape>(palletSize))
        ->getDynamicsAspect()
        ->setFrictionCoeff(1.0);
    Inertia palletInertia;
    palletInertia.setMass(10.0);
    palletInertia.setMoment(BoxShape::computeInertia(palletSize, 10.0));
    palletBody->setInertia(palletInertia);
    Eigen::Isometry3d palletTf = Eigen::Isometry3d::Identity();
    palletTf.translation().z() = palletSize.z() / 2.0 - 1e-6;
    FreeJoint::setTransformOf(palletBody, palletTf);
    world->addSkeleton(pallet);

    // The arm's joint spring (5 N m/rad) pulls it toward 0.3 rad, so it swings
    // from the start.
    BodyNode* base = nullptr;
    RevoluteJoint* armJoint = nullptr;
    auto robot = createArmRobot(1.0, &base, &armJoint);
    armJoint->setSpringStiffness(0, 5.0);
    armJoint->setRestPosition(0, 0.3);
    Eigen::Isometry3d baseTf = base->getTransform();
    baseTf.translation().z() += palletSize.z() - 1e-6;
    FreeJoint::setTransformOf(base, baseTf);
    world->addSkeleton(robot);

    const auto computeCom = [&]() -> Eigen::Vector3d {
      return (pallet->getMass() * pallet->getCOM()
              + robot->getMass() * robot->getCOM())
             / (pallet->getMass() + robot->getMass());
    };
    const Eigen::Vector3d startCom = computeCom();
    double maxComDrift = 0.0;
    for (std::size_t i = 0; i < 10000; ++i) {
      world->step();
      maxComDrift
          = std::max(maxComDrift, (computeCom() - startCom).head<2>().norm());
    }

    ASSERT_GT(world->getLastCollisionResult().getNumContacts(), 0u);
    EXPECT_LT(maxComDrift, 1e-3);
  }
}
