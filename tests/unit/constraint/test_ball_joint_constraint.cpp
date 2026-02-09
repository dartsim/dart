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

#include <dart/simulation/world.hpp>

#include <dart/constraint/ball_joint_constraint.hpp>
#include <dart/constraint/constraint_solver.hpp>

#include <dart/dynamics/body_node.hpp>
#include <dart/dynamics/free_joint.hpp>
#include <dart/dynamics/skeleton.hpp>

#include <gtest/gtest.h>

using namespace dart;
using namespace dart::dynamics;
using namespace dart::constraint;
using namespace dart::simulation;

namespace {

SkeletonPtr createFreeSkeleton(const std::string& name)
{
  auto skel = Skeleton::create(name);
  skel->createJointAndBodyNodePair<FreeJoint>();
  return skel;
}

} // namespace

TEST(BallJointConstraint, CreateWithTwoBodies)
{
  auto skel1 = createFreeSkeleton("skel1");
  auto skel2 = createFreeSkeleton("skel2");
  auto body1 = skel1->getBodyNode(0);
  auto body2 = skel2->getBodyNode(0);

  Eigen::Vector3d jointPos(1.0, 2.0, 3.0);

  auto constraint
      = std::make_shared<BallJointConstraint>(body1, body2, jointPos);

  EXPECT_EQ(constraint->getBodyNode1(), body1);
  EXPECT_EQ(constraint->getBodyNode2(), body2);
}

TEST(BallJointConstraint, CreateWithOneBody)
{
  auto skel = createFreeSkeleton("skel");
  auto body = skel->getBodyNode(0);

  Eigen::Vector3d jointPos(0.5, 0.5, 0.5);

  auto constraint = std::make_shared<BallJointConstraint>(body, jointPos);

  EXPECT_EQ(constraint->getBodyNode1(), body);
  EXPECT_EQ(constraint->getBodyNode2(), nullptr);
}

TEST(BallJointConstraint, DimensionIsThree)
{
  auto skel1 = createFreeSkeleton("skel1");
  auto skel2 = createFreeSkeleton("skel2");
  auto body1 = skel1->getBodyNode(0);
  auto body2 = skel2->getBodyNode(0);

  Eigen::Vector3d jointPos(0.0, 0.0, 0.0);

  auto constraint
      = std::make_shared<BallJointConstraint>(body1, body2, jointPos);

  EXPECT_EQ(constraint->getDimension(), 3u);
}

TEST(BallJointConstraint, GetType)
{
  auto skel = createFreeSkeleton("skel");
  auto body = skel->getBodyNode(0);

  Eigen::Vector3d jointPos(0.0, 0.0, 0.0);

  auto constraint = std::make_shared<BallJointConstraint>(body, jointPos);

  EXPECT_EQ(constraint->getType(), BallJointConstraint::getStaticType());
  EXPECT_FALSE(constraint->getType().empty());
}

TEST(BallJointConstraint, StaticSettings)
{
  double originalAllowance = BallJointConstraint::getErrorAllowance();
  double originalErp = BallJointConstraint::getErrorReductionParameter();
  double originalErv = BallJointConstraint::getMaxErrorReductionVelocity();
  double originalCfm = BallJointConstraint::getConstraintForceMixing();

  BallJointConstraint::setErrorAllowance(0.005);
  EXPECT_DOUBLE_EQ(BallJointConstraint::getErrorAllowance(), 0.005);

  BallJointConstraint::setErrorReductionParameter(0.5);
  EXPECT_DOUBLE_EQ(BallJointConstraint::getErrorReductionParameter(), 0.5);

  BallJointConstraint::setMaxErrorReductionVelocity(5.0);
  EXPECT_DOUBLE_EQ(BallJointConstraint::getMaxErrorReductionVelocity(), 5.0);

  BallJointConstraint::setConstraintForceMixing(1e-4);
  EXPECT_DOUBLE_EQ(BallJointConstraint::getConstraintForceMixing(), 1e-4);

  BallJointConstraint::setErrorAllowance(originalAllowance);
  BallJointConstraint::setErrorReductionParameter(originalErp);
  BallJointConstraint::setMaxErrorReductionVelocity(originalErv);
  BallJointConstraint::setConstraintForceMixing(originalCfm);
}

TEST(BallJointConstraint, SimulateWithTwoBodies)
{
  auto world = World::create();

  auto skel1 = createFreeSkeleton("skel1");
  auto skel2 = createFreeSkeleton("skel2");
  world->addSkeleton(skel1);
  world->addSkeleton(skel2);

  auto body1 = skel1->getBodyNode(0);
  auto body2 = skel2->getBodyNode(0);

  skel2->setPosition(3, 1.0);

  Eigen::Vector3d jointPos(0.5, 0.0, 0.0);
  auto constraint
      = std::make_shared<BallJointConstraint>(body1, body2, jointPos);

  world->getConstraintSolver()->addConstraint(constraint);

  for (int i = 0; i < 100; ++i) {
    world->step();
  }

  double distance = (skel1->getPositions().segment<3>(3)
                     - skel2->getPositions().segment<3>(3))
                        .norm();

  EXPECT_LT(distance, 2.0);
}

TEST(BallJointConstraint, SimulateWithOneBody)
{
  auto world = World::create();

  auto skel = createFreeSkeleton("skel");
  world->addSkeleton(skel);

  auto body = skel->getBodyNode(0);

  skel->setPosition(3, 1.0);
  skel->setPosition(4, 0.5);

  Eigen::Vector3d jointPos(0.0, 0.0, 0.0);
  auto constraint = std::make_shared<BallJointConstraint>(body, jointPos);

  world->getConstraintSolver()->addConstraint(constraint);

  for (int i = 0; i < 100; ++i) {
    world->step();
  }

  Eigen::Vector3d bodyPos = body->getTransform().translation();
  EXPECT_TRUE(bodyPos.norm() < 2.0);
}

TEST(BallJointConstraint, RemoveConstraint)
{
  auto world = World::create();

  auto skel1 = createFreeSkeleton("skel1");
  auto skel2 = createFreeSkeleton("skel2");
  world->addSkeleton(skel1);
  world->addSkeleton(skel2);

  auto body1 = skel1->getBodyNode(0);
  auto body2 = skel2->getBodyNode(0);

  Eigen::Vector3d jointPos(0.0, 0.0, 0.0);
  auto constraint
      = std::make_shared<BallJointConstraint>(body1, body2, jointPos);

  world->getConstraintSolver()->addConstraint(constraint);
  EXPECT_EQ(world->getConstraintSolver()->getNumConstraints(), 1u);

  world->getConstraintSolver()->removeConstraint(constraint);
  EXPECT_EQ(world->getConstraintSolver()->getNumConstraints(), 0u);
}

TEST(BallJointConstraint, SameSkeletonTwoBodies)
{
  auto world = World::create();

  auto skel = Skeleton::create("skel");
  auto [joint1, body1] = skel->createJointAndBodyNodePair<FreeJoint>();
  auto [joint2, body2] = skel->createJointAndBodyNodePair<FreeJoint>(body1);

  world->addSkeleton(skel);

  Eigen::Vector3d jointPos(0.0, 0.0, 0.0);
  auto constraint
      = std::make_shared<BallJointConstraint>(body1, body2, jointPos);

  world->getConstraintSolver()->addConstraint(constraint);

  for (int i = 0; i < 10; ++i) {
    world->step();
  }

  EXPECT_EQ(constraint->getBodyNode1(), body1);
  EXPECT_EQ(constraint->getBodyNode2(), body2);
}

TEST(BallJointConstraint, MultipleConstraints)
{
  auto world = World::create();

  auto skel1 = createFreeSkeleton("skel1");
  auto skel2 = createFreeSkeleton("skel2");
  auto skel3 = createFreeSkeleton("skel3");
  world->addSkeleton(skel1);
  world->addSkeleton(skel2);
  world->addSkeleton(skel3);

  auto body1 = skel1->getBodyNode(0);
  auto body2 = skel2->getBodyNode(0);
  auto body3 = skel3->getBodyNode(0);

  Eigen::Vector3d jointPos1(0.0, 0.0, 0.0);
  Eigen::Vector3d jointPos2(1.0, 0.0, 0.0);

  auto constraint1
      = std::make_shared<BallJointConstraint>(body1, body2, jointPos1);
  auto constraint2
      = std::make_shared<BallJointConstraint>(body2, body3, jointPos2);

  world->getConstraintSolver()->addConstraint(constraint1);
  world->getConstraintSolver()->addConstraint(constraint2);

  EXPECT_EQ(world->getConstraintSolver()->getNumConstraints(), 2u);

  for (int i = 0; i < 10; ++i) {
    world->step();
  }

  world->getConstraintSolver()->removeAllConstraints();
  EXPECT_EQ(world->getConstraintSolver()->getNumConstraints(), 0u);
}

TEST(BallJointConstraint, WorldStepReducesAnchorErrorWithDynamicSettings)
{
  auto world = World::create();
  world->setGravity(Eigen::Vector3d::Zero());

  auto skel1 = createFreeSkeleton("skel1");
  auto skel2 = createFreeSkeleton("skel2");
  world->addSkeleton(skel1);
  world->addSkeleton(skel2);

  auto body1 = skel1->getBodyNode(0);
  auto body2 = skel2->getBodyNode(0);

  skel2->setPosition(3, 1.5);
  skel2->setPosition(4, -0.5);

  Eigen::Vector3d jointPos = body1->getTransform().translation();
  auto constraint
      = std::make_shared<BallJointConstraint>(body1, body2, jointPos);

  const Eigen::Vector3d offset1 = body1->getTransform().inverse() * jointPos;
  const Eigen::Vector3d offset2 = body2->getTransform().inverse() * jointPos;

  const double prevAllowance = BallJointConstraint::getErrorAllowance();
  const double prevErp = BallJointConstraint::getErrorReductionParameter();
  const double prevErv = BallJointConstraint::getMaxErrorReductionVelocity();
  const double prevCfm = BallJointConstraint::getConstraintForceMixing();

  BallJointConstraint::setErrorAllowance(-0.01);
  BallJointConstraint::setErrorReductionParameter(-0.2);
  BallJointConstraint::setMaxErrorReductionVelocity(-1.0);
  BallJointConstraint::setConstraintForceMixing(1e-12);

  BallJointConstraint::setErrorAllowance(0.0);
  BallJointConstraint::setErrorReductionParameter(0.2);
  BallJointConstraint::setMaxErrorReductionVelocity(2.0);
  BallJointConstraint::setConstraintForceMixing(1e-6);

  world->getConstraintSolver()->addConstraint(constraint);

  skel2->setPosition(3, skel2->getPosition(3) + 0.4);

  const Eigen::Vector3d anchor1 = body1->getTransform() * offset1;
  const Eigen::Vector3d anchor2 = body2->getTransform() * offset2;
  const double initialError = (anchor1 - anchor2).norm();
  EXPECT_GT(initialError, 0.0);

  for (int i = 0; i < 200; ++i) {
    world->step();
  }

  const Eigen::Vector3d finalAnchor1 = body1->getTransform() * offset1;
  const Eigen::Vector3d finalAnchor2 = body2->getTransform() * offset2;
  const double finalError = (finalAnchor1 - finalAnchor2).norm();
  EXPECT_LT(finalError, initialError);

  BallJointConstraint::setErrorAllowance(prevAllowance);
  BallJointConstraint::setErrorReductionParameter(prevErp);
  BallJointConstraint::setMaxErrorReductionVelocity(prevErv);
  BallJointConstraint::setConstraintForceMixing(prevCfm);
}
