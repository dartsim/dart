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

#include "dart/dart.hpp"

#include <dart/test/dynamics/TestHelpers.hpp>

#include <gtest/gtest.h>

using namespace dart::math;
using namespace dart::collision;
using namespace dart::dynamics;
using namespace dart::simulation;
using namespace dart::io;

//==============================================================================
TEST(Issue1193, AngularVelAdd)
{
  WorldPtr world = World::create();
  EXPECT_TRUE(world != nullptr);
  world->setGravity(math::Vector3d(0.0, -10.0, 0.0));
  const double dt = 0.001;
  world->setTimeStep(dt);

  SkeletonPtr sphereSkel = createSphere(0.05, Vector3d(0.0, 1.0, 0.0));
  BodyNode* sphere = sphereSkel->getBodyNode(0);
  Joint* sphereJoint = sphere->getParentJoint();
  sphereJoint->setVelocity(0, 10.0); // ang_x -> Affect lz and ly
  sphereJoint->setVelocity(1, 10.0); // ang_y -> No effect
  sphereJoint->setVelocity(2, 10.0); // ang_z -> Affect lx and ly
  world->addSkeleton(sphereSkel);

  math::Vector3d linearVelBefore = sphere->getLinearVelocity();
  EXPECT_EQ(Vector3d::Zero(), linearVelBefore);

  int maxSteps = 500;
  for (int i = 0; i < maxSteps; i++) {
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

const double tol = 1e-5;
#if defined(NDEBUG)
const int g_iters = 100000;
#else
const int g_iters = 1000;
#endif

math::Vector3d computeWorldAngularMomentum(const SkeletonPtr skel)
{
  math::Vector3d angMomentum = math::Vector3d::Zero();
  skel->eachBodyNode([&](const BodyNode* bn) {
    angMomentum += dart::math::dAdInvT(
                       bn->getWorldTransform(),
                       bn->getSpatialInertia() * bn->getSpatialVelocity())
                       .head<3>();
  });
  return angMomentum;
}

//==============================================================================
TEST(Issue1193, SingleBody)
{
  WorldPtr world = World::create();
  const double dt = 0.001;
  world->setTimeStep(dt);
  world->setGravity(Vector3d::Zero());
  SkeletonPtr skel = createBox({1, 1, 1});
  world->addSkeleton(skel);
  auto rootBn = skel->getRootBodyNode();

  math::Vector3d initPosition = rootBn->getWorldTransform().translation();
  Vector6d vels(0, 25, 0, 0, 0, -100);

  rootBn->getParentJoint()->setVelocities(vels);

  rootBn->addExtTorque({0, 50, 0});
  for (int i = 0; i < g_iters; ++i) {
    world->step();
  }
  math::Vector3d positionDiff
      = rootBn->getWorldTransform().translation() - initPosition;
  EXPECT_NEAR(0.0, positionDiff.x(), tol);
  EXPECT_NEAR(0.0, positionDiff.y(), tol);
  EXPECT_NEAR(g_iters * dt * vels[5], positionDiff.z(), tol);
}

//==============================================================================
TEST(Issue1193, SingleBodyWithOffDiagonalMoi)
{
  WorldPtr world = World::create();
  const double dt = 0.001;
  world->setTimeStep(dt);
  world->setGravity(Vector3d::Zero());
  SkeletonPtr skel = createBox({1, 4, 9});
  world->addSkeleton(skel);
  auto rootBn = skel->getRootBodyNode();
  rootBn->setMomentOfInertia(1.1, 1.1, 0.7, 0.1, 0, 0);
  math::Vector3d initPosition = rootBn->getWorldTransform().translation();

  // Setting this to (0, 25, 0, 0, 0, -100) causes the test to fail.
  const Vector6d vels(0, 2.5, 0, 0, 0, -10.0);

  rootBn->getParentJoint()->setVelocities(vels);

  rootBn->addExtTorque({0, 50, 0});
  //
  for (int i = 0; i < g_iters; ++i) {
    world->step();
  }
  math::Vector3d positionDiff
      = rootBn->getWorldTransform().translation() - initPosition;
  EXPECT_NEAR(0.0, positionDiff.x(), tol);
  EXPECT_NEAR(0.0, positionDiff.y(), tol);
  EXPECT_NEAR(g_iters * dt * vels[5], positionDiff.z(), tol);
}

//==============================================================================
TEST(Issue1193, SingleBodyWithJointOffset)
{
  WorldPtr world = World::create();
  const double dt = 0.001;
  world->setTimeStep(dt);
  world->setGravity(Vector3d::Zero());
  SkeletonPtr skel = createBox({1, 1, 1});
  world->addSkeleton(skel);
  auto rootBn = skel->getRootBodyNode();
  rootBn->setMomentOfInertia(1.1, 1.1, 0.7, 0.1, 0, 0);
  math::Vector3d initPosition = rootBn->getWorldTransform().translation();

  Vector6d vels(0, 2.5, 0, 0, 0, -10);

  auto* freeJoint = dynamic_cast<FreeJoint*>(rootBn->getParentJoint());
  freeJoint->setVelocities(vels);

  math::Isometry3d jointPoseInParent = math::Isometry3d::Identity();
  jointPoseInParent.translate(math::Vector3d(0.0, 4.0, 0));
  freeJoint->setTransformFromParentBodyNode(jointPoseInParent);

  rootBn->addExtTorque({0, 50, 0});
  for (int i = 0; i < g_iters; ++i) {
    world->step();
  }

  math::Vector3d positionDiff
      = rootBn->getWorldTransform().translation() - initPosition;
  EXPECT_NEAR(0.0, positionDiff.x(), tol);
  EXPECT_NEAR(4.0, positionDiff.y(), tol);
  EXPECT_NEAR(g_iters * dt * vels[5], positionDiff.z(), tol);
}

//==============================================================================
TEST(Issue1193, SingleBodyWithCOMOffset)
{
  WorldPtr world = World::create();
  const double dt = 0.001;
  world->setTimeStep(dt);
  world->setGravity(Vector3d::Zero());
  SkeletonPtr skel = createBox({1, 1, 1});
  world->addSkeleton(skel);
  auto rootBn = skel->getRootBodyNode();
  rootBn->setMomentOfInertia(1.0 / 6, 1.0 / 6.0, 1.0 / 6.0, 0, 0, 0);
  rootBn->setLocalCOM({1, 5, 8});
  const math::Vector3d extForce{10, 0, 0};
  rootBn->addExtForce(extForce);
  world->step();
  auto comLinearVel = rootBn->getCOMLinearVelocity();
  // TODO (azeey) Improve FreeJoint integration so that a higher tolerance is
  // not necessary here.
  EXPECT_NEAR(extForce.x() * dt, comLinearVel.x(), tol * 1e2);
}

//==============================================================================
TEST(Issue1193, WithFixedJoint)
{
  WorldPtr world = World::create();
  const double dt = 0.001;
  world->setTimeStep(dt);
  world->setGravity(Vector3d::Zero());
  SkeletonPtr skel = createBox({1, 1, 1}, {0, 0, 2});
  world->addSkeleton(skel);
  auto rootBn = skel->getRootBodyNode();

  math::Isometry3d comRelPose;
  comRelPose = math::Translation3d(0, 0, -2);
  auto comFrame = SimpleFrame::createShared(rootBn, "CombinedCOM", comRelPose);
  math::Vector3d initComPosition = comFrame->getWorldTransform().translation();

  GenericJoint<R1Space>::Properties joint2Prop(std::string("joint2"));
  BodyNode::Properties link2Prop(
      BodyNode::AspectProperties(std::string("link2")));
  link2Prop.mInertia.setMass(1.0);

  auto pair = rootBn->createChildJointAndBodyNodePair<WeldJoint>(
      WeldJoint::Properties(joint2Prop), link2Prop);
  auto* joint = pair.first;

  math::Isometry3d jointPoseInParent = math::Isometry3d::Identity();
  jointPoseInParent.translate(math::Vector3d(0.0, 0.0, -4));
  joint->setTransformFromParentBodyNode(jointPoseInParent);

  // TODO (azeey) Improve FreeJoint integration so we can test with larger
  // forces. Currently, increasing these forces much more causes the test to
  // fail.
  rootBn->setExtTorque({0, 2500, 0});
  rootBn->setExtForce({0, 0, -1000});

  for (int i = 0; i < g_iters; ++i) {
    world->step();
  }
  math::Vector3d positionDiff
      = comFrame->getWorldTransform().translation() - initComPosition;
  EXPECT_NEAR(0.0, positionDiff.x(), tol);
  EXPECT_NEAR(0.0, positionDiff.y(), tol);
}

//==============================================================================
TEST(Issue1193, WithRevoluteJoint)
{
  auto world = SdfParser::readWorld(
      "dart://sample/sdf/test/issue1193_revolute_test.sdf");
  ASSERT_TRUE(world != nullptr);
  const double dt = 0.001;
  world->setTimeStep(dt);

  SkeletonPtr skel = world->getSkeleton(0);
  auto rootBn = skel->getRootBodyNode();

  math::Isometry3d comRelPose;
  comRelPose = math::Translation3d(0, 0, -2);
  auto comFrame = SimpleFrame::createShared(rootBn, "CombinedCOM", comRelPose);
  math::Vector3d initComPosition = comFrame->getWorldTransform().translation();

  auto* joint = skel->getJoint("revJoint");
  ASSERT_NE(nullptr, joint);

  for (int i = 0; i < g_iters; ++i) {
    joint->setCommand(0, 0.1);
    world->step();
  }

  math::Vector3d positionDiff
      = comFrame->getWorldTransform().translation() - initComPosition;
  EXPECT_NEAR(0.0, positionDiff.x(), tol * 5e2);
  EXPECT_NEAR(0.0, positionDiff.y(), tol * 5e2);
}

//==============================================================================
TEST(Issue1193, ConservationOfMomentumWithRevoluteJointWithOffset)
{
  auto world = SdfParser::readWorld(
      "dart://sample/sdf/test/issue1193_revolute_with_offset_test.sdf");
  ASSERT_TRUE(world != nullptr);
  const double dt = 0.0001;
  world->setTimeStep(dt);
  world->setGravity(Vector3d::Zero());

  SkeletonPtr skel = world->getSkeleton(0);
  auto link1 = skel->getBodyNode("link1");

  auto* joint = skel->getJoint("revJoint");
  ASSERT_NE(nullptr, joint);

  link1->getParentJoint()->setVelocities(
      math::Vector6d(0, 0.25, 0, 0, 0, -0.1));
  world->step();
  math::Vector3d maxAngMomentumChange = math::Vector3d::Zero();
  math::Vector3d h0 = computeWorldAngularMomentum(skel);
  for (int i = 1; i < g_iters; ++i) {
    joint->setCommand(0, 0.1);
    world->step();

    math::Vector3d hNext = computeWorldAngularMomentum(skel);
    maxAngMomentumChange
        = (hNext - h0).cwiseAbs().cwiseMax(maxAngMomentumChange);
  }

  EXPECT_NEAR(0.0, maxAngMomentumChange.norm(), tol * 10);
}
