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

#include "demo_models/SoftWormModel.hpp"
#include "demo_models/WorldHistory.hpp"

#include <gtest/gtest.h>

#include <limits>
#include <stdexcept>

namespace {

using dart_demos::WorldHistory;

//==============================================================================
TEST(WorldHistoryTest, RestoresSkeletonAndSoftPointState)
{
  dart::dynamics::SkeletonPtr worm;
  auto world = dart_demos::soft_worm_model::createWorld(worm);
  auto* point = worm->getBodyNode(0)->asSoftBodyNode()->getPointMass(0);
  const Eigen::VectorXd positions = worm->getPositions();
  const Eigen::VectorXd velocities = worm->getVelocities();
  const Eigen::Vector3d pointPositions = point->getPositions();
  const Eigen::Vector3d pointVelocities = point->getVelocities();
  WorldHistory history(world);
  EXPECT_EQ(history.getWorld(), world);
  EXPECT_EQ(history.getCurrentIndex(), 0u);
  EXPECT_EQ(history.getSize(), 1u);
  EXPECT_EQ(history.getFramesRecorded(), 1u);
  EXPECT_EQ(history.getMaxFrames(), 4000u);

  worm->setPositions(positions.array() + 0.1);
  worm->setVelocities(velocities.array() + 0.2);
  point->setPositions(pointPositions + Eigen::Vector3d(0.01, 0.02, 0.03));
  point->setVelocities(Eigen::Vector3d(0.1, 0.2, 0.3));
  const Eigen::VectorXd laterPositions = worm->getPositions();
  const Eigen::VectorXd laterVelocities = worm->getVelocities();
  const Eigen::Vector3d laterPointPositions = point->getPositions();
  const Eigen::Vector3d laterPointVelocities = point->getVelocities();
  history.record();

  history.moveTo(0);
  EXPECT_TRUE(worm->getPositions().isApprox(positions, 0.0));
  EXPECT_TRUE(worm->getVelocities().isApprox(velocities, 0.0));
  EXPECT_TRUE(point->getPositions().isApprox(pointPositions, 0.0));
  EXPECT_TRUE(point->getVelocities().isApprox(pointVelocities, 0.0));

  history.moveForward();
  EXPECT_TRUE(worm->getPositions().isApprox(laterPositions, 0.0));
  EXPECT_TRUE(worm->getVelocities().isApprox(laterVelocities, 0.0));
  EXPECT_TRUE(point->getPositions().isApprox(laterPointPositions, 0.0));
  EXPECT_TRUE(point->getVelocities().isApprox(laterPointVelocities, 0.0));
}

//==============================================================================
TEST(WorldHistoryTest, CapsFramesBranchesAndClampsPlayback)
{
  auto world = dart::simulation::World::create();
  auto skeleton = dart::dynamics::Skeleton::create("recorded");
  skeleton->createJointAndBodyNodePair<dart::dynamics::FreeJoint>(nullptr);
  world->addSkeleton(skeleton);
  WorldHistory history(world, 3);
  for (int frame = 1; frame <= 4; ++frame) {
    skeleton->setPosition(3, frame);
    history.record();
  }
  EXPECT_EQ(history.getSize(), 3u);
  EXPECT_EQ(history.getCurrentIndex(), 2u);
  EXPECT_EQ(history.getFramesRecorded(), 5u);
  history.moveBackward(std::numeric_limits<std::size_t>::max());
  EXPECT_EQ(history.getCurrentIndex(), 0u);
  EXPECT_EQ(skeleton->getPosition(3), 2.0);

  skeleton->setPosition(3, 9.0);
  history.record();
  EXPECT_EQ(history.getSize(), 2u);
  EXPECT_EQ(history.getCurrentIndex(), 1u);
  EXPECT_EQ(history.getFramesRecorded(), 6u);
  history.moveTo(0);
  EXPECT_EQ(skeleton->getPosition(3), 2.0);
  history.moveForward(std::numeric_limits<std::size_t>::max());
  EXPECT_EQ(history.getCurrentIndex(), 1u);
  EXPECT_EQ(skeleton->getPosition(3), 9.0);
  history.moveTo(std::numeric_limits<std::size_t>::max());
  EXPECT_EQ(history.getCurrentIndex(), 1u);
}

//==============================================================================
TEST(WorldHistoryTest, SupportsEmptyWorldAndOneFrameCap)
{
  auto world = dart::simulation::World::create();
  WorldHistory history(world, 1);
  history.record();
  history.moveBackward();
  history.moveForward();
  EXPECT_EQ(history.getSize(), 1u);
  EXPECT_EQ(history.getCurrentIndex(), 0u);
  EXPECT_EQ(history.getFramesRecorded(), 2u);
}

//==============================================================================
TEST(WorldHistoryTest, RejectsNullWorldAndZeroCapacity)
{
  EXPECT_THROW(WorldHistory(nullptr), std::invalid_argument);
  EXPECT_THROW(
      WorldHistory(dart::simulation::World::create(), 0),
      std::invalid_argument);
}

} // namespace
