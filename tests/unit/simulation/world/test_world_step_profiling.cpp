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

#include <dart/config.hpp>

#include <dart/simulation/body/collision_shape.hpp>
#include <dart/simulation/body/rigid_body.hpp>
#include <dart/simulation/body/rigid_body_options.hpp>
#include <dart/simulation/compute/parallel_executor.hpp>
#include <dart/simulation/compute/world_step_profile.hpp>
#include <dart/simulation/world.hpp>

#include <gtest/gtest.h>

#include <string>

namespace sx = dart::simulation;

TEST(WorldStepProfileIntegration, DisabledByDefaultLeavesProfileEmpty)
{
  sx::World world;
  EXPECT_FALSE(world.isStepProfilingEnabled());

  world.step();

  EXPECT_TRUE(world.getLastStepProfile().isEmpty());
}

#if DART_BUILD_PROFILE

TEST(WorldStepProfileIntegration, EnabledStepRecordsPipelineStages)
{
  sx::World world;
  world.setStepProfilingEnabled(true);
  EXPECT_TRUE(world.isStepProfilingEnabled());

  world.step();

  const auto& profile = world.getLastStepProfile();
  ASSERT_FALSE(profile.isEmpty());
  EXPECT_EQ(profile.stepCount, 1u);
  EXPECT_EQ(profile.stages.size(), 1u);

  // Empty worlds still refresh kinematics, but inactive solver domains should
  // not appear as placeholder stages.
  EXPECT_EQ(profile.getStage("rigid_body_velocity"), nullptr);
  EXPECT_EQ(profile.getStage("deformable_dynamics"), nullptr);
  ASSERT_NE(profile.getStage("kinematics"), nullptr);

  // Every recorded stage carries a name; wall time is non-negative.
  for (const auto& stage : profile.stages) {
    EXPECT_FALSE(stage.name.empty());
  }
  EXPECT_GE(profile.wallTime.count(), 0);
  EXPECT_GE(profile.wallTime, profile.totalStageTime());

  // The text summary is the AI-/human-facing surface; it names the stages.
  const std::string summary = profile.toSummaryText();
  EXPECT_NE(summary.find("kinematics"), std::string::npos);
  EXPECT_NE(summary.find("World Step Profile"), std::string::npos);
}

TEST(WorldStepProfileIntegration, StaticOnlyRigidWorldSkipsRigidDynamicsStages)
{
  sx::World world;

  sx::RigidBodyOptions options;
  options.isStatic = true;
  options.position = Eigen::Vector3d(0.0, 0.0, -0.5);
  auto ground = world.addRigidBody("ground", options);
  ground.setCollisionShape(
      sx::CollisionShape::makeBox(Eigen::Vector3d(5.0, 5.0, 0.5)));
  const Eigen::Vector3d initialTranslation = ground.getTranslation();

  world.setStepProfilingEnabled(true);
  world.step();

  EXPECT_EQ(world.getFrame(), 1u);
  EXPECT_DOUBLE_EQ(world.getTime(), world.getTimeStep());
  EXPECT_TRUE(ground.getTranslation().isApprox(initialTranslation));

  const auto& profile = world.getLastStepProfile();
  ASSERT_FALSE(profile.isEmpty());
  EXPECT_EQ(profile.getStage("rigid_body_velocity"), nullptr);
  EXPECT_EQ(profile.getStage("rigid_body_contact"), nullptr);
  EXPECT_EQ(profile.getStage("rigid_body_position"), nullptr);
  ASSERT_NE(profile.getStage("kinematics"), nullptr);
}

TEST(WorldStepProfileIntegration, EnabledDeformableStepRecordsDomainStage)
{
  sx::World world;
  sx::DeformableBodyOptions options;
  options.positions = {Eigen::Vector3d(0.0, 0.0, 1.0)};
  options.masses = {1.0};
  options.edgeStiffness = 0.0;
  world.addDeformableBody("particle", options);
  world.setStepProfilingEnabled(true);

  world.step();

  const auto& profile = world.getLastStepProfile();
  ASSERT_FALSE(profile.isEmpty());
  const auto* deformable = profile.getStage("deformable_dynamics");
  ASSERT_NE(deformable, nullptr);
  EXPECT_TRUE(
      sx::compute::hasAcceleration(
          deformable->acceleration,
          sx::compute::ComputeStageAcceleration::Gpu));
  ASSERT_NE(profile.getStage("kinematics"), nullptr);

  const std::string summary = profile.toSummaryText();
  EXPECT_NE(summary.find("deformable_dynamics"), std::string::npos);
  EXPECT_NE(summary.find("gpu"), std::string::npos);
}

TEST(WorldStepProfileIntegration, CapturesNestedParallelGraphProfiles)
{
  sx::World world;
  sx::compute::ParallelExecutor executor(2);
  world.setStepProfilingEnabled(true);

  world.step(executor);

  const auto& profile = world.getLastStepProfile();
  ASSERT_FALSE(profile.isEmpty());

  const auto* kinematics = profile.getStage("kinematics");
  ASSERT_NE(kinematics, nullptr);
  ASSERT_FALSE(kinematics->graphProfiles.empty());
  EXPECT_GE(kinematics->maxGraphWorkerCount(), 1u);
  EXPECT_LE(
      kinematics->maxGraphParallelism(), kinematics->maxGraphWorkerCount());
  EXPECT_NE(
      kinematics->acceleration
          & sx::compute::toMask(
              sx::compute::ComputeStageAcceleration::TaskParallel),
      0u);

  const auto summary = profile.toSummaryText();
  EXPECT_NE(summary.find("graph_profiles="), std::string::npos);
  EXPECT_NE(summary.find("max_workers="), std::string::npos);
}

TEST(WorldStepProfileIntegration, DisablingFreezesTheLastSnapshot)
{
  sx::World world;
  world.setStepProfilingEnabled(true);
  world.step();

  const std::size_t stageCount = world.getLastStepProfile().stages.size();
  ASSERT_GT(stageCount, 0u);

  // Once disabled, further steps must not touch the retained snapshot.
  world.setStepProfilingEnabled(false);
  for (int i = 0; i < 3; ++i) {
    world.step();
  }

  EXPECT_FALSE(world.isStepProfilingEnabled());
  EXPECT_EQ(world.getLastStepProfile().stages.size(), stageCount);
}

TEST(WorldStepProfileIntegration, ClearResetsProfilingStateAndSnapshot)
{
  sx::World world;
  world.setStepProfilingEnabled(true);
  world.step();

  ASSERT_FALSE(world.getLastStepProfile().isEmpty());

  world.clear();

  EXPECT_FALSE(world.isStepProfilingEnabled());
  EXPECT_TRUE(world.getLastStepProfile().isEmpty());
}

TEST(WorldStepProfileIntegration, WallTimeCoversReplayRecordingBookkeeping)
{
  sx::World world;
  world.setReplayRecordingEnabled(true);
  world.setStepProfilingEnabled(true);

  world.step();

  const auto& profile = world.getLastStepProfile();
  ASSERT_FALSE(profile.isEmpty());
  EXPECT_GE(world.getReplayFrameCount(), 2u);
  EXPECT_GE(profile.wallTime, profile.totalStageTime());

  const std::string summary = profile.toSummaryText();
  EXPECT_NE(summary.find("(unattributed overhead)"), std::string::npos);
}

TEST(WorldStepProfileIntegration, MultiStepReflectsLastStep)
{
  sx::World world;
  world.setStepProfilingEnabled(true);

  world.step(5);

  const auto& profile = world.getLastStepProfile();
  ASSERT_FALSE(profile.isEmpty());
  // Snapshot semantics match the deformable diagnostics: the profile reflects
  // the most recent step, not an accumulation across the five.
  EXPECT_EQ(profile.stepCount, 1u);
}

#else

TEST(WorldStepProfileIntegration, BuildProfileOffMakesRuntimeToggleANoop)
{
  sx::World world;
  world.setStepProfilingEnabled(true);

  EXPECT_FALSE(world.isStepProfilingEnabled());

  world.step();

  EXPECT_TRUE(world.getLastStepProfile().isEmpty());
}

#endif
