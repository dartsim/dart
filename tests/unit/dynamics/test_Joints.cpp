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

TEST(Joints, NonFiniteTransformFromParentBodyNodeRejected)
{
  auto skel = Skeleton::create("test");
  auto pair = skel->createJointAndBodyNodePair<RevoluteJoint>();
  auto* joint = pair.first;

  Eigen::Isometry3d goodTf = Eigen::Isometry3d::Identity();
  goodTf.translation() = Eigen::Vector3d(1, 2, 3);
  joint->setTransformFromParentBodyNode(goodTf);
  EXPECT_TRUE(goodTf.isApprox(joint->getTransformFromParentBodyNode(), 1e-10));

  Eigen::Isometry3d nanTf = Eigen::Isometry3d::Identity();
  nanTf.translation()[0] = std::numeric_limits<double>::quiet_NaN();
  joint->setTransformFromParentBodyNode(nanTf);
  EXPECT_TRUE(goodTf.isApprox(joint->getTransformFromParentBodyNode(), 1e-10));

  Eigen::Isometry3d infTf = Eigen::Isometry3d::Identity();
  infTf.translation()[0] = std::numeric_limits<double>::infinity();
  joint->setTransformFromParentBodyNode(infTf);
  EXPECT_TRUE(goodTf.isApprox(joint->getTransformFromParentBodyNode(), 1e-10));

  Eigen::Isometry3d badRotTf = Eigen::Isometry3d::Identity();
  badRotTf.linear() << 2, 0, 0, 0, 2, 0, 0, 0, 2;
  joint->setTransformFromParentBodyNode(badRotTf);
  EXPECT_TRUE(goodTf.isApprox(joint->getTransformFromParentBodyNode(), 1e-10));
}

TEST(Joints, NonFiniteTransformFromChildBodyNodeRejected)
{
  auto skel = Skeleton::create("test");
  auto pair = skel->createJointAndBodyNodePair<RevoluteJoint>();
  auto* joint = pair.first;

  Eigen::Isometry3d nanTf = Eigen::Isometry3d::Identity();
  nanTf.translation()[0] = std::numeric_limits<double>::quiet_NaN();
  joint->setTransformFromChildBodyNode(nanTf);
  EXPECT_TRUE(Eigen::Isometry3d::Identity().isApprox(
      joint->getTransformFromChildBodyNode(), 1e-10));
}

TEST(Joints, SimulationSurvivesOverflowTransforms)
{
  auto skel = Skeleton::create("overflow");
  auto pair = skel->createJointAndBodyNodePair<FreeJoint>();
  auto* body = pair.second;
  body->setMass(1.0);

  Eigen::Isometry3d extremeTf = Eigen::Isometry3d::Identity();
  extremeTf.translation() = Eigen::Vector3d(1e308, 1e308, 1e308);
  pair.first->setTransformFromParentBodyNode(extremeTf);

  auto world = World::create();
  world->addSkeleton(skel);

  for (int i = 0; i < 10; ++i) {
    EXPECT_NO_THROW(world->step());
  }
}
