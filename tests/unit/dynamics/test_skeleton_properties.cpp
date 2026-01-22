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

#include <dart/dynamics/body_node.hpp>
#include <dart/dynamics/free_joint.hpp>
#include <dart/dynamics/revolute_joint.hpp>
#include <dart/dynamics/skeleton.hpp>

#include <Eigen/Core>
#include <gtest/gtest.h>

#include <algorithm>
#include <vector>

using namespace dart::dynamics;

static SkeletonPtr createTestSkeleton()
{
  auto skeleton = Skeleton::create("skeleton_props");
  auto rootPair = skeleton->createJointAndBodyNodePair<FreeJoint>();
  auto childPair
      = rootPair.second->createChildJointAndBodyNodePair<RevoluteJoint>();
  childPair.first->setAxis(Eigen::Vector3d::UnitY());
  return skeleton;
}

TEST(SkeletonProperties, FlagsAndTiming)
{
  auto skeleton = Skeleton::create("TestSkeleton");
  EXPECT_EQ(skeleton->getName(), "TestSkeleton");

  EXPECT_TRUE(skeleton->isMobile());
  skeleton->setMobile(false);
  EXPECT_FALSE(skeleton->isMobile());
  skeleton->setMobile(true);
  EXPECT_TRUE(skeleton->isMobile());

  const double timeStep = 0.002;
  skeleton->setTimeStep(timeStep);
  EXPECT_DOUBLE_EQ(skeleton->getTimeStep(), timeStep);

  const Eigen::Vector3d gravity(0.0, -9.81, 0.0);
  skeleton->setGravity(gravity);
  EXPECT_TRUE(skeleton->getGravity().isApprox(gravity));

  EXPECT_FALSE(skeleton->getSelfCollisionCheck());
  skeleton->setSelfCollisionCheck(true);
  EXPECT_TRUE(skeleton->getSelfCollisionCheck());
  EXPECT_TRUE(skeleton->isEnabledSelfCollisionCheck());
  skeleton->setSelfCollisionCheck(false);
  EXPECT_FALSE(skeleton->getSelfCollisionCheck());

  EXPECT_FALSE(skeleton->getAdjacentBodyCheck());
  skeleton->setAdjacentBodyCheck(true);
  skeleton->setSelfCollisionCheck(true);
  EXPECT_TRUE(skeleton->getAdjacentBodyCheck());
  EXPECT_TRUE(skeleton->isEnabledAdjacentBodyCheck());
  skeleton->setAdjacentBodyCheck(false);
  EXPECT_FALSE(skeleton->getAdjacentBodyCheck());
}

TEST(SkeletonConfiguration, UpdatesSelectedDofs)
{
  auto skeleton = createTestSkeleton();
  const std::size_t dofs = skeleton->getNumDofs();

  skeleton->setPositions(Eigen::VectorXd::Zero(dofs));
  skeleton->setVelocities(Eigen::VectorXd::Zero(dofs));

  std::vector<std::size_t> indices = {0, 3, dofs - 1};
  Eigen::VectorXd positions(indices.size());
  positions << 1.2, -0.4, 0.7;

  Eigen::VectorXd velocities(indices.size());
  velocities << 0.2, -0.1, 0.3;

  Skeleton::Configuration config(indices, positions, velocities);
  skeleton->setConfiguration(config);

  const Eigen::VectorXd updatedPositions = skeleton->getPositions();
  const Eigen::VectorXd updatedVelocities = skeleton->getVelocities();

  for (std::size_t i = 0; i < indices.size(); ++i) {
    EXPECT_DOUBLE_EQ(updatedPositions[indices[i]], positions[i]);
    EXPECT_DOUBLE_EQ(updatedVelocities[indices[i]], velocities[i]);
  }

  for (std::size_t i = 0; i < dofs; ++i) {
    if (std::find(indices.begin(), indices.end(), i) == indices.end()) {
      EXPECT_DOUBLE_EQ(updatedPositions[i], 0.0);
      EXPECT_DOUBLE_EQ(updatedVelocities[i], 0.0);
    }
  }

  const auto subsetConfig
      = skeleton->getConfiguration(indices, Skeleton::CONFIG_VELOCITIES);
  EXPECT_EQ(subsetConfig.mPositions.size(), 0);
  EXPECT_EQ(subsetConfig.mVelocities.size(), positions.size());
  EXPECT_TRUE(subsetConfig.mVelocities.isApprox(velocities));
}
