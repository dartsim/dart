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

#include <dart/simulation/compute_graph/world_step_graph.hpp>

#include <dart/All.hpp>

#include <gtest/gtest.h>

#include <string>

using namespace dart;
using namespace dart::dynamics;
using namespace dart::simulation;

namespace {

SkeletonPtr createSimpleSkeleton(
    const std::string& name, double position, double velocity)
{
  auto skel = Skeleton::create(name);

  auto pair = skel->createJointAndBodyNodePair<RevoluteJoint>();
  auto* joint = pair.first;
  auto* body = pair.second;
  joint->setName(name + "_joint");
  body->setName(name + "_body");
  body->setMass(1.0);
  joint->setActuatorType(Joint::ACCELERATION);
  joint->setCommand(0, 1.0);

  const Eigen::VectorXd positions
      = Eigen::VectorXd::Constant(skel->getNumDofs(), position);
  const Eigen::VectorXd velocities
      = Eigen::VectorXd::Constant(skel->getNumDofs(), velocity);
  skel->setPositions(positions);
  skel->setVelocities(velocities);

  return skel;
}

} // namespace

TEST(WorldStepGraph, MatchesSequentialStep)
{
  auto skelA = createSimpleSkeleton("skel_a", 0.1, 0.2);
  auto skelB = createSimpleSkeleton("skel_b", -0.3, 0.4);

  auto worldSequential = World::create();
  worldSequential->setTimeStep(1e-3);
  worldSequential->addSkeleton(skelA->cloneSkeleton());
  worldSequential->addSkeleton(skelB->cloneSkeleton());

  auto worldGraph = World::create();
  worldGraph->setTimeStep(1e-3);
  worldGraph->addSkeleton(skelA->cloneSkeleton());
  worldGraph->addSkeleton(skelB->cloneSkeleton());

  GraphExecutionConfig graphConfig;
  graphConfig.enableComputeGraph = true;
  graphConfig.numWorkers = 2;
  graphConfig.batchSize = 1;
  worldGraph->setGraphExecutionConfig(graphConfig);

  worldSequential->step(true);
  worldGraph->step(true);

  EXPECT_NEAR(worldGraph->getTime(), worldSequential->getTime(), 1e-12);
  EXPECT_EQ(worldGraph->getSimFrames(), worldSequential->getSimFrames());

  ASSERT_EQ(worldSequential->getNumSkeletons(), worldGraph->getNumSkeletons());
  for (std::size_t i = 0; i < worldSequential->getNumSkeletons(); ++i) {
    auto seqSkel = worldSequential->getSkeleton(i);
    auto graphSkel = worldGraph->getSkeleton(i);
    ASSERT_TRUE(seqSkel);
    ASSERT_TRUE(graphSkel);
    EXPECT_TRUE(
        seqSkel->getPositions().isApprox(graphSkel->getPositions(), 1e-12));
    EXPECT_TRUE(
        seqSkel->getVelocities().isApprox(graphSkel->getVelocities(), 1e-12));
  }
}

TEST(WorldStepGraph, RespectsResetCommand)
{
  auto skel = createSimpleSkeleton("reset", 0.0, 0.1);
  auto* joint = skel->getJoint(0);

  auto world = World::create();
  world->setTimeStep(1e-3);
  world->addSkeleton(skel);

  GraphExecutionConfig graphConfig;
  graphConfig.enableComputeGraph = true;
  graphConfig.numWorkers = 1;
  world->setGraphExecutionConfig(graphConfig);

  joint->setCommand(0, 3.0);
  world->step(false);
  EXPECT_DOUBLE_EQ(joint->getCommand(0), 3.0);

  world->step(true);
  EXPECT_DOUBLE_EQ(joint->getCommand(0), 0.0);
}

TEST(WorldStepGraph, BatchingReducesNodeCount)
{
  auto world = World::create();
  world->setTimeStep(1e-3);

  const std::size_t numSkeletons = 4;
  for (std::size_t i = 0; i < numSkeletons; ++i) {
    world->addSkeleton(
        createSimpleSkeleton("batch_" + std::to_string(i), 0.0, 0.1));
  }

  ExecutorConfig execConfig;
  execConfig.batchSize = 2;
  WorldStepGraph stepGraph(*world, execConfig);

  const std::size_t numBatches
      = (numSkeletons + execConfig.batchSize - 1) / execConfig.batchSize;
  const std::size_t expectedNodes = numBatches * 2 + 2;
  EXPECT_EQ(stepGraph.getGraph().getNodeCount(), expectedNodes);
}

TEST(WorldStepGraph, StepsManySkeletons)
{
  auto world = World::create();
  world->setTimeStep(1e-3);

  const std::size_t numSkeletons = 16;
  for (std::size_t i = 0; i < numSkeletons; ++i) {
    world->addSkeleton(
        createSimpleSkeleton("stress_" + std::to_string(i), 0.0, 0.5));
  }

  GraphExecutionConfig graphConfig;
  graphConfig.enableComputeGraph = true;
  graphConfig.numWorkers = 4;
  graphConfig.batchSize = 4;
  world->setGraphExecutionConfig(graphConfig);

  auto firstSkel = world->getSkeleton(0);
  ASSERT_TRUE(firstSkel);
  const Eigen::VectorXd initialPositions = firstSkel->getPositions();

  world->step(true);

  EXPECT_EQ(world->getSimFrames(), 1);
  EXPECT_NEAR(world->getTime(), world->getTimeStep(), 1e-12);
  EXPECT_FALSE(firstSkel->getPositions().isApprox(initialPositions, 1e-12));
}
