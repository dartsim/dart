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

#include "TestHelpers.hpp"
#include "dart/collision/CollisionObject.hpp"
#include "dart/collision/Contact.hpp"
#include "dart/common/Macros.hpp"
#include "dart/common/Platform.hpp"
#include "dart/constraint/BoxedLcpConstraintSolver.hpp"
#include "dart/constraint/BoxedLcpSolver.hpp"
#include "dart/constraint/ConstraintBase.hpp"
#include "dart/constraint/ContactConstraint.hpp"
#include "dart/constraint/DantzigBoxedLcpSolver.hpp"
#include "dart/constraint/PgsBoxedLcpSolver.hpp"
#include "dart/dynamics/BoxShape.hpp"
#include "dart/dynamics/FreeJoint.hpp"
#include "dart/dynamics/Joint.hpp"
#include "dart/dynamics/PrismaticJoint.hpp"
#include "dart/dynamics/RevoluteJoint.hpp"
#include "dart/dynamics/ShapeFrame.hpp"
#include "dart/dynamics/Skeleton.hpp"
#include "dart/dynamics/SphereShape.hpp"
#include "dart/dynamics/WeldJoint.hpp"
#include "dart/simulation/World.hpp"

#include <Eigen/Core>
#include <gtest/gtest.h>

#include <memory>

#include <cmath>

using namespace dart;

//==============================================================================
void testContactWithKinematicJoint(
    const constraint::BoxedLcpSolverPtr& lcpSolver, double tol)
{
  auto world = std::make_shared<simulation::World>();
  world->setConstraintSolver(
      std::make_unique<constraint::BoxedLcpConstraintSolver>(lcpSolver));

  auto skeleton1 = dynamics::Skeleton::create("skeleton1");
  auto pair1 = skeleton1->createJointAndBodyNodePair<dynamics::FreeJoint>();
  auto bodyNode1 = pair1.second;
  auto joint1 = pair1.first;
  joint1->setActuatorType(dynamics::Joint::VELOCITY);
  auto shape1
      = std::make_shared<dynamics::BoxShape>(Eigen::Vector3d(1.0, 1.0, 1.0));
  bodyNode1->createShapeNodeWith<VisualAspect, CollisionAspect, DynamicsAspect>(
      shape1);
  skeleton1->setPosition(5, 0.0);

  auto skeleton2 = dynamics::Skeleton::create("skeleton2");
  auto pair2 = skeleton2->createJointAndBodyNodePair<dynamics::FreeJoint>();
  auto bodyNode2 = pair2.second;
  auto joint2 = pair2.first;
  joint2->setActuatorType(dynamics::Joint::FORCE);
  auto shape2
      = std::make_shared<dynamics::BoxShape>(Eigen::Vector3d(0.5, 0.5, 0.5));
  bodyNode2->createShapeNodeWith<VisualAspect, CollisionAspect, DynamicsAspect>(
      shape2);
  skeleton2->setPosition(5, 0.75);

  world->addSkeleton(skeleton1);
  world->addSkeleton(skeleton2);

  for (auto i = 0u; i < 100; ++i) {
    skeleton1->setCommand(3, 0.1);

    world->step();

    EXPECT_EQ(bodyNode1->getLinearVelocity()[0], 0.1);

    // Need few steps to settle down
    if (i > 15) {
      EXPECT_NEAR(bodyNode2->getLinearVelocity()[0], 0.1, tol);
    }
  }
}

//==============================================================================
TEST(ContactConstraint, ContactWithKinematicJoint)
{
  testContactWithKinematicJoint(
      std::make_shared<constraint::DantzigBoxedLcpSolver>(), 1e-6);

#if DART_ARCH_32BITS
  testContactWithKinematicJoint(
      std::make_shared<constraint::PgsBoxedLcpSolver>(), 1e-3);
#else
  testContactWithKinematicJoint(
      std::make_shared<constraint::PgsBoxedLcpSolver>(), 1e-4);
#endif
}

//==============================================================================
dynamics::BodyNode* addBox(
    dynamics::BodyNode* body, const Eigen::Vector3d& size, double mass)
{
  body->createShapeNodeWith<VisualAspect, CollisionAspect, DynamicsAspect>(
      std::make_shared<dynamics::BoxShape>(size));
  dynamics::Inertia inertia;
  inertia.setMass(mass);
  inertia.setMoment(dynamics::BoxShape::computeInertia(size, mass));
  body->setInertia(inertia);
  return body;
}

//==============================================================================
dynamics::SkeletonPtr addGround(simulation::World& world)
{
  auto ground = dynamics::Skeleton::create("ground");
  auto pair = ground->createJointAndBodyNodePair<dynamics::WeldJoint>();
  addBox(pair.second, Eigen::Vector3d(4.0, 4.0, 0.1), 1.0);
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.translation().z() = -0.05;
  pair.first->setTransformFromParentBodyNode(pose);
  ground->setMobile(false);
  world.addSkeleton(ground);
  return ground;
}

//==============================================================================
// Adds two plates on revolute joints that share the palm's z axis. The plates
// overlap along that axis, so no joint motion can separate them along their
// self-contact normal and its LCP row is singular.
void addCoaxialPlates(
    dynamics::Skeleton& hand, dynamics::BodyNode* palm, double friction = 1.0)
{
  for (int i = 0; i < 2; ++i) {
    dynamics::RevoluteJoint::Properties properties;
    properties.mAxis = Eigen::Vector3d::UnitZ();
    properties.mT_ParentBodyToJoint.translation().z() = 0.1 + 0.015 * i;
    addBox(
        hand.createJointAndBodyNodePair<dynamics::RevoluteJoint>(
                palm, properties)
            .second,
        Eigen::Vector3d(0.2, 0.2, 0.02),
        0.2)
        ->getShapeNode(0)
        ->getDynamicsAspect()
        ->setFrictionCoeff(friction);
  }
  hand.setSelfCollisionCheck(true);
}

//==============================================================================
// A palm with the coaxial plates rests on the ground. Without friction rows,
// only the plates' own responses reveal that the row is singular.
void testUnseparableSelfContact(
    bool withFallback, bool splitImpulse, bool frictionless)
{
  auto world = simulation::World::create();
  auto* solver = dynamic_cast<constraint::BoxedLcpConstraintSolver*>(
      world->getConstraintSolver());
  ASSERT_NE(nullptr, solver);
  if (!withFallback)
    solver->setSecondaryBoxedLcpSolver(nullptr);
  solver->setSplitImpulseEnabled(splitImpulse);

  const auto ground = addGround(*world);

  auto hand = dynamics::Skeleton::create("hand");
  auto* palm = addBox(
      hand->createJointAndBodyNodePair<dynamics::FreeJoint>().second,
      Eigen::Vector3d(0.3, 0.3, 0.05),
      1.0);
  addCoaxialPlates(*hand, palm, frictionless ? 0.0 : 1.0);
  hand->setPosition(5, 0.025);
  world->addSkeleton(hand);

  for (int i = 0; i < 500; ++i) {
    world->step();
    ASSERT_TRUE(hand->getPositions().allFinite()) << "step " << i;
    ASSERT_LT(hand->getVelocities().cwiseAbs().maxCoeff(), 1.0) << "step " << i;
  }

  // The palm still rests on the ground, which carries the whole hand.
  EXPECT_NEAR(palm->getWorldTransform().translation().z(), 0.025, 0.005);
  const auto& result = world->getLastCollisionResult();
  double groundForce = 0.0;
  for (std::size_t i = 0; i < result.getNumContacts(); ++i) {
    const auto& contact = result.getContact(i);
    ASSERT_TRUE(contact.force.allFinite());
    if (contact.collisionObject1->getShapeFrame()->asShapeNode()->getSkeleton()
            == ground
        || contact.collisionObject2->getShapeFrame()
                   ->asShapeNode()
                   ->getSkeleton()
               == ground) {
      groundForce += std::abs(contact.force.z());
    }
  }
  EXPECT_NEAR(groundForce, 1.4 * 9.81, 0.1 * 1.4 * 9.81);
}

//==============================================================================
TEST(ContactConstraint, UnseparableSelfContactStaysAtRest)
{
  for (const bool withFallback : {true, false}) {
    for (const bool splitImpulse : {false, true}) {
      for (const bool frictionless : {false, true}) {
        SCOPED_TRACE(
            ::testing::Message() << "withFallback=" << withFallback
                                 << " splitImpulse=" << splitImpulse
                                 << " frictionless=" << frictionless);
        testUnseparableSelfContact(withFallback, splitImpulse, frictionless);
      }
    }
  }
}

//==============================================================================
// Adds a palm welded to the world in a tilted pose, so that no contact normal
// between links on its z axis is exactly axis aligned.
dynamics::BodyNode* addTiltedFixedPalm(dynamics::Skeleton& hand)
{
  auto pair = hand.createJointAndBodyNodePair<dynamics::WeldJoint>();
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.linear() = (Eigen::AngleAxisd(0.1, Eigen::Vector3d::UnitX())
                   * Eigen::AngleAxisd(0.2, Eigen::Vector3d::UnitY())
                   * Eigen::AngleAxisd(0.3, Eigen::Vector3d::UnitZ()))
                      .toRotationMatrix();
  pose.translation().z() = 0.5;
  pair.first->setTransformFromParentBodyNode(pose);
  return addBox(pair.second, Eigen::Vector3d(0.3, 0.3, 0.05), 1.0);
}

//==============================================================================
// The coaxial plates on a welded, tilted palm: neither plate can move along the
// normal even on its own, so the row is singular without any cancellation.
TEST(ContactConstraint, UnseparableSelfContactOnFixedBaseStaysAtRest)
{
  // Without friction rows, only the plates' own tangential responses reveal
  // the motion that the normal lacks.
  for (const double friction : {1.0, 0.0}) {
    SCOPED_TRACE(::testing::Message() << "friction=" << friction);
    auto world = simulation::World::create();
    world->setGravity(Eigen::Vector3d::Zero());

    auto hand = dynamics::Skeleton::create("hand");
    auto* palm = addTiltedFixedPalm(*hand);
    addCoaxialPlates(*hand, palm, friction);
    world->addSkeleton(hand);

    for (int i = 0; i < 500; ++i) {
      world->step();
      ASSERT_TRUE(hand->getPositions().allFinite()) << "step " << i;
      ASSERT_LT(hand->getVelocities().cwiseAbs().maxCoeff(), 1.0)
          << "step " << i;
    }
  }
}

//==============================================================================
// A frictionless plate on a revolute joint whose axis is the normal of the
// tilted static block it presses into. The two skeletons cannot cancel each
// other's responses, but the plate alone cannot move along the normal either.
TEST(ContactConstraint, UnseparableFrictionlessContactStaysAtRest)
{
  auto world = simulation::World::create();
  world->setGravity(Eigen::Vector3d::Zero());

  auto block = dynamics::Skeleton::create("block");
  auto* blockBody = addTiltedFixedPalm(*block);
  blockBody->getShapeNode(0)->getDynamicsAspect()->setFrictionCoeff(0.0);
  block->setMobile(false);
  world->addSkeleton(block);

  auto turntable = dynamics::Skeleton::create("turntable");
  auto* base = addTiltedFixedPalm(*turntable);
  base->removeAllShapeNodes();
  dynamics::RevoluteJoint::Properties properties;
  properties.mAxis = Eigen::Vector3d::UnitZ();
  properties.mT_ParentBodyToJoint.translation().z() = 0.03;
  auto* plate = addBox(
      turntable
          ->createJointAndBodyNodePair<dynamics::RevoluteJoint>(
              base, properties)
          .second,
      Eigen::Vector3d(0.2, 0.2, 0.02),
      0.2);
  plate->getShapeNode(0)->getDynamicsAspect()->setFrictionCoeff(0.0);
  world->addSkeleton(turntable);

  for (int i = 0; i < 500; ++i) {
    world->step();
    ASSERT_TRUE(turntable->getPositions().allFinite()) << "step " << i;
    ASSERT_LT(turntable->getVelocities().cwiseAbs().maxCoeff(), 1.0)
        << "step " << i;
  }
}

//==============================================================================
// Two spheres on sibling revolute joints along the tilted palm's z axis overlap
// along it. Their tangential response is so small that adding slip compliance
// would round it away, so the singularity check must not use the compliant
// friction-row diagonals.
TEST(ContactConstraint, UnseparableSelfContactWithSlipComplianceStaysAtRest)
{
  auto world = simulation::World::create();
  world->setGravity(Eigen::Vector3d::Zero());

  auto hand = dynamics::Skeleton::create("hand");
  auto* palm = addTiltedFixedPalm(*hand);
  for (int i = 0; i < 2; ++i) {
    dynamics::RevoluteJoint::Properties properties;
    properties.mAxis = Eigen::Vector3d::UnitZ();
    properties.mT_ParentBodyToJoint.translation().z() = 0.1 + 0.03 * i;
    properties.mT_ChildBodyToJoint.translation().x() = -1e-6;
    auto* sphere = hand->createJointAndBodyNodePair<dynamics::RevoluteJoint>(
                           palm, properties)
                       .second;
    auto* shapeNode = sphere->createShapeNodeWith<
        VisualAspect,
        CollisionAspect,
        DynamicsAspect>(std::make_shared<dynamics::SphereShape>(0.02));
    shapeNode->getDynamicsAspect()->setFrictionCoeff(0.5);
    shapeNode->getDynamicsAspect()->setPrimarySlipCompliance(10.0);
    shapeNode->getDynamicsAspect()->setSecondarySlipCompliance(10.0);
    dynamics::Inertia inertia;
    inertia.setMass(20.0);
    inertia.setMoment(10.0 * Eigen::Matrix3d::Identity());
    sphere->setInertia(inertia);
  }
  hand->setSelfCollisionCheck(true);
  world->addSkeleton(hand);

  for (int i = 0; i < 500; ++i) {
    world->step();
    ASSERT_TRUE(hand->getPositions().allFinite()) << "step " << i;
    ASSERT_LT(hand->getVelocities().cwiseAbs().maxCoeff(), 1.0) << "step " << i;
  }
}

//==============================================================================
// A plate welded to the palm through a second link overlaps the palm. The two
// cannot move relative to each other in any direction, so even the contact's
// friction rows cancel. With Dantzig alone, an unpinned singular row zeroes
// the whole group, including the palm's ground contacts.
TEST(ContactConstraint, UnseparableWeldedSelfContactKeepsGroundContacts)
{
  auto world = simulation::World::create();
  auto* solver = dynamic_cast<constraint::BoxedLcpConstraintSolver*>(
      world->getConstraintSolver());
  ASSERT_NE(nullptr, solver);
  solver->setSecondaryBoxedLcpSolver(nullptr);
  addGround(*world);

  auto hand = dynamics::Skeleton::create("hand");
  auto* palm = addBox(
      hand->createJointAndBodyNodePair<dynamics::FreeJoint>().second,
      Eigen::Vector3d(0.3, 0.3, 0.05),
      1.0);
  auto mount = hand->createJointAndBodyNodePair<dynamics::WeldJoint>(palm);
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.translation().z() = 0.3;
  mount.first->setTransformFromParentBodyNode(pose);
  addBox(mount.second, Eigen::Vector3d(0.05, 0.05, 0.05), 0.2);
  auto plate
      = hand->createJointAndBodyNodePair<dynamics::WeldJoint>(mount.second);
  pose.translation().z() = -0.29;
  plate.first->setTransformFromParentBodyNode(pose);
  addBox(plate.second, Eigen::Vector3d(0.2, 0.2, 0.02), 0.2);
  hand->setSelfCollisionCheck(true);
  hand->setPosition(5, 0.025);
  world->addSkeleton(hand);

  for (int i = 0; i < 500; ++i) {
    world->step();
    ASSERT_LT(hand->getVelocities().cwiseAbs().maxCoeff(), 0.02)
        << "step " << i;
  }
  EXPECT_NEAR(palm->getWorldTransform().translation().z(), 0.025, 0.001);
}

//==============================================================================
// A link whose center of mass sits on its revolute axis, with a tiny moment of
// inertia, carries a sphere 1 m out that presses into the ground. The axis is
// 1e-13 rad off the ground normal, so the link is far more mobile along the
// ground than along its normal, which the contact sees as less than 1e-12 of
// the link's motion: the row is singular although its diagonal is 1e-10.
TEST(ContactConstraint, LowInertiaLinkAgainstGroundStaysAtRest)
{
  auto world = simulation::World::create();
  world->setGravity(Eigen::Vector3d::Zero());
  addGround(*world);

  auto arm = dynamics::Skeleton::create("arm");
  dynamics::RevoluteJoint::Properties properties;
  properties.mAxis = Eigen::Vector3d(0.0, 1e-13, 1.0).normalized();
  properties.mT_ParentBodyToJoint.translation().z() = 0.02;
  auto* link = arm->createJointAndBodyNodePair<dynamics::RevoluteJoint>(
                      nullptr, properties)
                   .second;
  auto* shapeNode = link->createShapeNodeWith<
      VisualAspect,
      CollisionAspect,
      DynamicsAspect>(std::make_shared<dynamics::SphereShape>(0.025));
  shapeNode->setRelativeTranslation(Eigen::Vector3d::UnitX());
  dynamics::Inertia inertia;
  inertia.setMass(1.0);
  inertia.setMoment(1e-16 * Eigen::Matrix3d::Identity());
  link->setInertia(inertia);
  world->addSkeleton(arm);

  for (int i = 0; i < 500; ++i) {
    world->step();
    ASSERT_TRUE(arm->getPositions().allFinite()) << "step " << i;
    ASSERT_LT(arm->getVelocities().cwiseAbs().maxCoeff(), 1.0) << "step " << i;
  }
}

//==============================================================================
// A light link welded to a heavier free body moves with that body, so its
// ground contacts are ordinary and must keep supporting it, also with another
// light body in the same contact group.
TEST(ContactConstraint, LightWeldedLinkKeepsGroundContacts)
{
  auto world = simulation::World::create();
  addGround(*world);

  const Eigen::Vector3d size(0.3, 0.3, 0.05);
  auto skeleton = dynamics::Skeleton::create("body");
  auto* body
      = skeleton->createJointAndBodyNodePair<dynamics::FreeJoint>().second;
  dynamics::Inertia inertia;
  inertia.setMass(1.0);
  inertia.setMoment(dynamics::BoxShape::computeInertia(size, 1.0));
  body->setInertia(inertia);
  auto* foot = addBox(
      skeleton->createJointAndBodyNodePair<dynamics::WeldJoint>(body).second,
      size,
      1e-13);
  skeleton->setPosition(5, 0.025);
  world->addSkeleton(skeleton);

  auto lightBox = dynamics::Skeleton::create("light box");
  addBox(
      lightBox->createJointAndBodyNodePair<dynamics::FreeJoint>().second,
      Eigen::Vector3d(0.1, 0.1, 0.05),
      1e-13);
  lightBox->setPosition(5, 0.075);
  world->addSkeleton(lightBox);

  for (int i = 0; i < 500; ++i)
    world->step();

  EXPECT_NEAR(foot->getWorldTransform().translation().z(), 0.025, 0.005);
}

//==============================================================================
// A sphere on a prismatic joint whose axis is almost parallel to the ground
// can still move along the contact normal, so its tiny normal diagonal is a
// real response and the contact must still stop the sphere, whether its
// scale comes from friction rows with large slip compliance or, without
// friction, from the sphere's own tangential response.
TEST(ContactConstraint, ShallowJointMotionStillStopsAtContact)
{
  for (const bool frictionless : {false, true}) {
    SCOPED_TRACE(::testing::Message() << "frictionless=" << frictionless);
    auto world = simulation::World::create();
    const auto ground = addGround(*world);

    auto skeleton = dynamics::Skeleton::create("slider");
    dynamics::PrismaticJoint::Properties properties;
    properties.mAxis = Eigen::Vector3d(1.0, 0.0, 1e-10).normalized();
    properties.mT_ParentBodyToJoint.translation().z() = 0.025;
    auto* body = skeleton
                     ->createJointAndBodyNodePair<dynamics::PrismaticJoint>(
                         nullptr, properties)
                     .second;
    auto* shapeNode = body->createShapeNodeWith<
        VisualAspect,
        CollisionAspect,
        DynamicsAspect>(std::make_shared<dynamics::SphereShape>(0.025));
    if (frictionless) {
      shapeNode->getDynamicsAspect()->setFrictionCoeff(0.0);
      ground->getBodyNode(0)
          ->getShapeNode(0)
          ->getDynamicsAspect()
          ->setFrictionCoeff(0.0);
    } else {
      shapeNode->getDynamicsAspect()->setFrictionCoeff(0.5);
      shapeNode->getDynamicsAspect()->setPrimarySlipCompliance(10.0);
      shapeNode->getDynamicsAspect()->setSecondarySlipCompliance(10.0);
    }
    dynamics::Inertia inertia;
    inertia.setMass(1.0);
    inertia.setMoment(dynamics::SphereShape::computeInertia(0.025, 1.0));
    body->setInertia(inertia);
    skeleton->setVelocity(0, -1.0);
    world->addSkeleton(skeleton);

    world->step();
    world->step();

    EXPECT_GT(skeleton->getVelocity(0), 0.0);
  }
}

//==============================================================================
// A sphere on a prismatic joint 1e-10 rad off the contact plane touches a very
// light box that can only slide along that plane. The box's large sliding
// response must not hide the sphere's accurate stiff response along the
// normal, with or without friction, so the contact must still stop the sphere.
TEST(ContactConstraint, LightSlidingBodyKeepsStiffContact)
{
  for (const double friction : {0.0, 0.5}) {
    SCOPED_TRACE(::testing::Message() << "friction=" << friction);
    auto world = simulation::World::create();
    world->setGravity(Eigen::Vector3d::Zero());

    auto box = dynamics::Skeleton::create("box");
    dynamics::PrismaticJoint::Properties boxJoint;
    boxJoint.mAxis = Eigen::Vector3d::UnitX();
    addBox(
        box->createJointAndBodyNodePair<dynamics::PrismaticJoint>(
               nullptr, boxJoint)
            .second,
        Eigen::Vector3d(0.4, 0.4, 0.05),
        1e-6)
        ->getShapeNode(0)
        ->getDynamicsAspect()
        ->setFrictionCoeff(friction);
    world->addSkeleton(box);

    auto slider = dynamics::Skeleton::create("slider");
    dynamics::PrismaticJoint::Properties sliderJoint;
    sliderJoint.mAxis = Eigen::Vector3d(1.0, 0.0, 1e-10).normalized();
    sliderJoint.mT_ParentBodyToJoint.translation().z() = 0.05;
    auto* sphere = slider
                       ->createJointAndBodyNodePair<dynamics::PrismaticJoint>(
                           nullptr, sliderJoint)
                       .second;
    sphere
        ->createShapeNodeWith<VisualAspect, CollisionAspect, DynamicsAspect>(
            std::make_shared<dynamics::SphereShape>(0.025))
        ->getDynamicsAspect()
        ->setFrictionCoeff(friction);
    dynamics::Inertia inertia;
    inertia.setMass(1.0);
    inertia.setMoment(dynamics::SphereShape::computeInertia(0.025, 1.0));
    sphere->setInertia(inertia);
    slider->setVelocity(0, -1.0);
    world->addSkeleton(slider);

    world->step();
    world->step();

    EXPECT_GT(slider->getVelocity(0), -0.5);
  }
}

//==============================================================================
// A slider on a prismatic joint almost parallel to the plate it touches, both
// in one skeleton, can still move along the contact normal. Its self-contact
// diagonal is tiny but above round-off, so the contact must still stop it.
TEST(ContactConstraint, ShallowSelfContactMotionStillStops)
{
  auto world = simulation::World::create();
  world->setGravity(Eigen::Vector3d::Zero());

  auto skeleton = dynamics::Skeleton::create("rig");
  dynamics::PrismaticJoint::Properties plateJoint;
  plateJoint.mAxis = Eigen::Vector3d::UnitZ();
  auto* plate = addBox(
      skeleton
          ->createJointAndBodyNodePair<dynamics::PrismaticJoint>(
              nullptr, plateJoint)
          .second,
      Eigen::Vector3d(1.0, 1.0, 0.05),
      1.0);

  auto weldPair
      = skeleton->createJointAndBodyNodePair<dynamics::WeldJoint>(plate);
  Eigen::Isometry3d weldPose = Eigen::Isometry3d::Identity();
  weldPose.translation().z() = 0.5;
  weldPair.first->setTransformFromParentBodyNode(weldPose);
  dynamics::Inertia inertia;
  inertia.setMass(1.0);
  weldPair.second->setInertia(inertia);

  dynamics::PrismaticJoint::Properties sliderJoint;
  sliderJoint.mAxis = Eigen::Vector3d(1.0, 0.0, 1e-9).normalized();
  sliderJoint.mT_ParentBodyToJoint.translation().z() = -0.45;
  auto* slider = skeleton
                     ->createJointAndBodyNodePair<dynamics::PrismaticJoint>(
                         weldPair.second, sliderJoint)
                     .second;
  slider->createShapeNodeWith<VisualAspect, CollisionAspect, DynamicsAspect>(
      std::make_shared<dynamics::SphereShape>(0.025));
  inertia.setMoment(dynamics::SphereShape::computeInertia(0.025, 1.0));
  slider->setInertia(inertia);
  for (auto* body : {plate, slider})
    body->getShapeNode(0)->getDynamicsAspect()->setFrictionCoeff(0.0);

  skeleton->setSelfCollisionCheck(true);
  skeleton->setVelocity(1, -1.0);
  world->addSkeleton(skeleton);

  world->step();
  world->step();

  EXPECT_GT(skeleton->getVelocity(1), 0.0);
}

//==============================================================================
TEST(ContactConstraint, MalformedContactWithNullCollisionObjectsIsInactive)
{
  collision::Contact contact;
  contact.collisionObject1 = nullptr;
  contact.collisionObject2 = nullptr;
  contact.point = Eigen::Vector3d::Zero();
  contact.normal = Eigen::Vector3d::UnitZ();
  contact.force = Eigen::Vector3d::Zero();

  DART_SUPPRESS_DEPRECATED_BEGIN
  constraint::ContactConstraint constraint(contact, 0.001);
  DART_SUPPRESS_DEPRECATED_END

  constraint::ConstraintBase& base = constraint;
  EXPECT_EQ(0u, base.getDimension());
  EXPECT_FALSE(base.isActive());

  base.update();

  EXPECT_FALSE(base.isActive());
}
