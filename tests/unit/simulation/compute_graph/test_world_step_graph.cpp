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
#include <vector>

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

SkeletonPtr createMultiDofSkeleton(const std::string& name, double seed)
{
  auto skel = Skeleton::create(name);

  auto [freeJoint, body1] = skel->createJointAndBodyNodePair<FreeJoint>();
  freeJoint->setName(name + "_free");
  body1->setName(name + "_body_1");
  body1->setMass(1.0);

  auto [revJoint1, body2]
      = skel->createJointAndBodyNodePair<RevoluteJoint>(body1);
  revJoint1->setName(name + "_rev_1");
  body2->setName(name + "_body_2");
  body2->setMass(0.5);
  revJoint1->setAxis(Eigen::Vector3d::UnitY());

  auto [revJoint2, body3]
      = skel->createJointAndBodyNodePair<RevoluteJoint>(body2);
  revJoint2->setName(name + "_rev_2");
  body3->setName(name + "_body_3");
  body3->setMass(0.3);
  revJoint2->setAxis(Eigen::Vector3d::UnitZ());

  freeJoint->setActuatorType(Joint::ACCELERATION);
  revJoint1->setActuatorType(Joint::ACCELERATION);
  revJoint2->setActuatorType(Joint::ACCELERATION);

  const std::size_t dof = skel->getNumDofs();
  Eigen::VectorXd positions(dof);
  Eigen::VectorXd velocities(dof);
  Eigen::VectorXd commands(dof);
  for (std::size_t i = 0; i < dof; ++i) {
    positions[static_cast<Eigen::Index>(i)]
        = seed + 0.03 * static_cast<double>(i + 1);
    velocities[static_cast<Eigen::Index>(i)]
        = -0.2 * seed + 0.01 * static_cast<double>(i + 1);
    commands[static_cast<Eigen::Index>(i)]
        = 0.5 * seed - 0.015 * static_cast<double>(i + 1);
  }

  skel->setPositions(positions);
  skel->setVelocities(velocities);
  skel->setCommands(commands);

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

TEST(WorldStepGraph, MultiStepMatchesSequential)
{
  std::vector<SkeletonPtr> baseSkeletons;
  baseSkeletons.reserve(3);
  for (std::size_t i = 0; i < 3; ++i) {
    baseSkeletons.push_back(createMultiDofSkeleton(
        "multi_" + std::to_string(i), 0.4 * static_cast<double>(i + 1)));
  }

  auto worldSequential = World::create();
  worldSequential->setTimeStep(1e-3);
  auto worldGraph = World::create();
  worldGraph->setTimeStep(1e-3);

  for (const auto& skel : baseSkeletons) {
    worldSequential->addSkeleton(skel->cloneSkeleton());
    worldGraph->addSkeleton(skel->cloneSkeleton());
  }

  GraphExecutionConfig graphConfig;
  graphConfig.enableComputeGraph = true;
  graphConfig.numWorkers = 2;
  graphConfig.batchSize = 1;
  worldGraph->setGraphExecutionConfig(graphConfig);

  ASSERT_EQ(worldSequential->getNumSkeletons(), worldGraph->getNumSkeletons());

  for (std::size_t step = 0; step < 200; ++step) {
    worldSequential->step(false);
    worldGraph->step(false);

    EXPECT_DOUBLE_EQ(worldGraph->getTime(), worldSequential->getTime());
    EXPECT_EQ(worldGraph->getSimFrames(), worldSequential->getSimFrames());

    for (std::size_t i = 0; i < worldSequential->getNumSkeletons(); ++i) {
      auto seqSkel = worldSequential->getSkeleton(i);
      auto graphSkel = worldGraph->getSkeleton(i);
      ASSERT_TRUE(seqSkel);
      ASSERT_TRUE(graphSkel);
      EXPECT_TRUE(
          seqSkel->getPositions().isApprox(graphSkel->getPositions(), 1e-10));
      EXPECT_TRUE(
          seqSkel->getVelocities().isApprox(graphSkel->getVelocities(), 1e-10));
    }
  }
}

TEST(WorldStepGraph, EmptyWorld)
{
  auto world = World::create();
  world->setTimeStep(1e-3);

  GraphExecutionConfig graphConfig;
  graphConfig.enableComputeGraph = true;
  graphConfig.numWorkers = 2;
  graphConfig.batchSize = 1;
  world->setGraphExecutionConfig(graphConfig);

  EXPECT_EQ(world->getNumSkeletons(), 0u);

  for (int i = 0; i < 10; ++i)
    world->step(true);

  EXPECT_EQ(world->getSimFrames(), 10u);
  EXPECT_NEAR(world->getTime(), 10.0 * world->getTimeStep(), 1e-12);
}

TEST(WorldStepGraph, SingleSkeleton)
{
  auto skel = createMultiDofSkeleton("single", 0.7);
  const Eigen::VectorXd initialPositions = skel->getPositions();

  auto world = World::create();
  world->setTimeStep(1e-3);
  world->addSkeleton(skel);

  GraphExecutionConfig graphConfig;
  graphConfig.enableComputeGraph = true;
  graphConfig.numWorkers = 2;
  graphConfig.batchSize = 1;
  world->setGraphExecutionConfig(graphConfig);

  world->step(false);

  EXPECT_EQ(world->getNumSkeletons(), 1u);
  EXPECT_EQ(world->getSimFrames(), 1u);
  EXPECT_NEAR(world->getTime(), world->getTimeStep(), 1e-12);
  EXPECT_FALSE(skel->getPositions().isApprox(initialPositions, 1e-12));
}

TEST(WorldStepGraph, ImmobileSkeletonSkipped)
{
  auto mobileA = createMultiDofSkeleton("mobile_a", 0.3);
  auto mobileB = createMultiDofSkeleton("mobile_b", 0.8);
  auto immobile = createMultiDofSkeleton("immobile", -0.2);
  immobile->setMobile(false);

  const Eigen::VectorXd immobilePositions = immobile->getPositions();
  const Eigen::VectorXd immobileVelocities = immobile->getVelocities();
  const Eigen::VectorXd mobilePositions = mobileA->getPositions();

  auto world = World::create();
  world->setTimeStep(1e-3);
  world->addSkeleton(mobileA);
  world->addSkeleton(mobileB);
  world->addSkeleton(immobile);

  GraphExecutionConfig graphConfig;
  graphConfig.enableComputeGraph = true;
  graphConfig.numWorkers = 2;
  graphConfig.batchSize = 1;
  world->setGraphExecutionConfig(graphConfig);

  for (int i = 0; i < 20; ++i)
    world->step(false);

  EXPECT_TRUE(immobile->getPositions().isApprox(immobilePositions, 1e-12));
  EXPECT_TRUE(immobile->getVelocities().isApprox(immobileVelocities, 1e-12));
  EXPECT_FALSE(mobileA->getPositions().isApprox(mobilePositions, 1e-12));
}

TEST(WorldStepGraph, WorldConfigConstructor)
{
  WorldConfig config;
  config.graphExecution.enableComputeGraph = true;
  config.graphExecution.numWorkers = 2;
  config.graphExecution.batchSize = 1;

  auto world = World::create(config);
  world->setTimeStep(1e-3);
  world->addSkeleton(createMultiDofSkeleton("world_config", 0.9));

  world->step(false);

  EXPECT_EQ(world->getNumSkeletons(), 1u);
  EXPECT_EQ(world->getSimFrames(), 1u);
  EXPECT_NEAR(world->getTime(), world->getTimeStep(), 1e-12);
}
