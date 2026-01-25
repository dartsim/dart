/*
 * Copyright (c) 2011-2025, The DART development contributors
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

#include <dart/dart.hpp>

#include <gtest/gtest.h>

#include <limits>

#include <cmath>

using namespace dart;
using namespace dart::dynamics;
using namespace dart::simulation;

namespace {

SkeletonPtr createTwoLinkSkeleton(double childMass, double childInertia)
{
  auto skeleton = Skeleton::create("invalid_dynamics_skeleton");

  auto rootPair = skeleton->createJointAndBodyNodePair<RevoluteJoint>();
  auto* rootBody = rootPair.second;
  rootBody->setMass(1.0);
  rootBody->setMomentOfInertia(1.0, 1.0, 1.0, 0.0, 0.0, 0.0);

  auto childPair
      = skeleton->createJointAndBodyNodePair<RevoluteJoint>(rootBody);
  auto* childBody = childPair.second;
  childBody->setMass(childMass);
  childBody->setMomentOfInertia(
      childInertia, childInertia, childInertia, 0.0, 0.0, 0.0);

  return skeleton;
}

SkeletonPtr createBoxSkeleton(
    const std::string& name,
    const Eigen::Vector3d& size,
    const Eigen::Vector3d& position,
    double mass,
    double inertia)
{
  auto skeleton = Skeleton::create(name);
  auto pair = skeleton->createJointAndBodyNodePair<FreeJoint>();
  auto* body = pair.second;

  body->setMass(mass);
  body->setMomentOfInertia(inertia, inertia, inertia, 0.0, 0.0, 0.0);

  auto shape = std::make_shared<BoxShape>(size);
  body->createShapeNodeWith<VisualAspect, CollisionAspect, DynamicsAspect>(
      shape);

  Eigen::Vector6d positions = Eigen::Vector6d::Zero();
  positions.tail<3>() = position;
  skeleton->setPositions(positions);

  return skeleton;
}

} // namespace

TEST(InvalidDynamicsInputs, ZeroMassLeafDoesNotCrash)
{
  auto world = World::create();
  auto skeleton = createTwoLinkSkeleton(0.0, 0.0);
  world->addSkeleton(skeleton);

  EXPECT_NO_THROW({
    for (int i = 0; i < 5; ++i) {
      world->step();
    }
  });

  EXPECT_TRUE(skeleton->getPositions().allFinite());
  EXPECT_TRUE(skeleton->getVelocities().allFinite());
}

TEST(InvalidDynamicsInputs, EpsilonMassLeafDoesNotCrash)
{
  constexpr double kEpsilonMass = 1e-300;
  auto world = World::create();
  auto skeleton = createTwoLinkSkeleton(kEpsilonMass, kEpsilonMass);
  world->addSkeleton(skeleton);

  EXPECT_NO_THROW({
    for (int i = 0; i < 5; ++i) {
      world->step();
    }
  });

  EXPECT_TRUE(skeleton->getPositions().allFinite());
  EXPECT_TRUE(skeleton->getVelocities().allFinite());
}

TEST(InvalidDynamicsInputs, ExtremeSpringDoesNotCrash)
{
  auto world = World::create();
  auto skeleton = Skeleton::create("spring_overflow_skeleton");

  auto pair = skeleton->createJointAndBodyNodePair<RevoluteJoint>();
  auto* joint = pair.first;
  auto* body = pair.second;
  body->setMass(1.0);
  body->setMomentOfInertia(1.0, 1.0, 1.0, 0.0, 0.0, 0.0);

  joint->setSpringStiffness(0, 1e300);
  joint->setRestPosition(0, 1e10);
  joint->setPosition(0, 0.0);

  world->addSkeleton(skeleton);

  EXPECT_NO_THROW({
    for (int i = 0; i < 5; ++i) {
      world->step();
    }
  });

  EXPECT_TRUE(skeleton->getPositions().allFinite());
  EXPECT_TRUE(skeleton->getVelocities().allFinite());
}

TEST(InvalidDynamicsInputs, ZeroMassContactDoesNotCrash)
{
  auto world = World::create();

  auto dynamicBox = createBoxSkeleton(
      "dynamic_box",
      Eigen::Vector3d(1.0, 1.0, 1.0),
      Eigen::Vector3d::Zero(),
      1.0,
      1.0);
  auto zeroMassBox = createBoxSkeleton(
      "zero_mass_box",
      Eigen::Vector3d(1.0, 1.0, 1.0),
      Eigen::Vector3d(0.0, 0.0, 0.4),
      0.0,
      0.0);

  world->addSkeleton(dynamicBox);
  world->addSkeleton(zeroMassBox);

  EXPECT_NO_THROW({
    for (int i = 0; i < 5; ++i) {
      world->step();
    }
  });

  EXPECT_TRUE(dynamicBox->getPositions().allFinite());
  EXPECT_TRUE(zeroMassBox->getPositions().allFinite());
}
