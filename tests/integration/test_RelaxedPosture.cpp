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

#include "demo_models/RelaxedPosture.hpp"

#include <dart/dynamics/FreeJoint.hpp>
#include <dart/dynamics/HierarchicalIK.hpp>
#include <dart/dynamics/Skeleton.hpp>

#include <gtest/gtest.h>

#include <limits>

TEST(RelaxedPosture, BoundedAndIdealResidualsKeepOriginalWeights)
{
  const double infinity = std::numeric_limits<double>::infinity();
  Eigen::Vector3d ideal(0.0, 0.5, -0.25);
  dart_demos::RelaxedPosture posture(
      ideal,
      Eigen::Vector3d(-1.0, -1.0, -infinity),
      Eigen::Vector3d(1.0, 1.0, infinity),
      Eigen::Vector3d(2.0, 3.0, 4.0));
  Eigen::Vector3d x(-2.0, 2.0, 0.0);
  Eigen::VectorXd gradient(3);
  Eigen::Map<Eigen::VectorXd> map(gradient.data(), gradient.size());
  EXPECT_DOUBLE_EQ(posture.eval(x), 6.5);
  posture.evalGradient(x, map);
  EXPECT_EQ(gradient, Eigen::Vector3d(-2.0, 3.0, 0.0));
  EXPECT_DOUBLE_EQ(posture.eval(ideal), 0.0);
  posture.evalGradient(ideal, map);
  EXPECT_TRUE(gradient.isZero());

  posture.enforceIdealPosture = true;
  EXPECT_DOUBLE_EQ(posture.eval(x), 18.625);
  posture.evalGradient(x, map);
  EXPECT_EQ(gradient, Eigen::Vector3d(-4.0, 4.5, 1.0));
}

TEST(RelaxedPosture, TruncatesToInputAndGradientSizes)
{
  dart_demos::RelaxedPosture posture(
      Eigen::Vector3d::Zero(),
      Eigen::Vector3d::Zero(),
      Eigen::Vector3d::Zero(),
      Eigen::Vector3d::Ones());
  Eigen::VectorXd x(1);
  x << 2.0;
  Eigen::VectorXd gradient = Eigen::VectorXd::Constant(5, 9.0);
  posture.evalGradient(
      x, Eigen::Map<Eigen::VectorXd>(gradient.data(), gradient.size()));
  EXPECT_DOUBLE_EQ(posture.eval(x), 2.0);
  EXPECT_DOUBLE_EQ(gradient[0], 2.0);
  EXPECT_TRUE(gradient.tail(4).isZero());
}

TEST(RelaxedPosture, InstallsAsWholeBodyObjective)
{
  auto skeleton = dart::dynamics::Skeleton::create();
  skeleton->createJointAndBodyNodePair<dart::dynamics::FreeJoint>();
  const auto objective = std::make_shared<dart_demos::RelaxedPosture>(
      skeleton->getPositions(),
      Eigen::VectorXd::Constant(6, -1.0),
      Eigen::VectorXd::Constant(6, 1.0),
      Eigen::VectorXd::Ones(6));
  const auto ik = skeleton->getIK(true);
  ik->setObjective(objective);
  EXPECT_EQ(ik->getObjective(), objective);
  EXPECT_DOUBLE_EQ(ik->getObjective()->eval(skeleton->getPositions()), 0.0);
}
