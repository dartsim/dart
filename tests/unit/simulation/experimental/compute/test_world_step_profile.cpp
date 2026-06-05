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

#include <dart/simulation/experimental/compute/compute_stage_metadata.hpp>
#include <dart/simulation/experimental/compute/world_step_profile.hpp>

#include <gtest/gtest.h>

#include <chrono>
#include <string>

using namespace std::chrono_literals;
using dart::simulation::experimental::compute::ComputeExecutionProfile;
using dart::simulation::experimental::compute::ComputeStageAcceleration;
using dart::simulation::experimental::compute::ComputeStageDomain;
using dart::simulation::experimental::compute::WorldStepProfile;
using dart::simulation::experimental::compute::WorldStepStageProfile;

namespace {

WorldStepProfile makeProfile()
{
  WorldStepProfile profile;
  profile.stepCount = 1;
  profile.wallTime = 10ms;
  auto& contact = profile.stages.emplace_back();
  contact.name = "rigid_body_contact";
  contact.domain = ComputeStageDomain::RigidBody;
  contact.duration = 6ms;
  auto& deformable = profile.stages.emplace_back();
  deformable.name = "deformable_dynamics";
  deformable.domain = ComputeStageDomain::DeformableBody;
  deformable.duration = 3ms;
  deformable.acceleration
      = ComputeStageAcceleration::TaskParallel | ComputeStageAcceleration::Gpu;
  deformable.acceleratedBackendEnabled = true;
  auto& kinematics = profile.stages.emplace_back();
  kinematics.name = "kinematics";
  kinematics.domain = ComputeStageDomain::Kinematics;
  kinematics.duration = 1ms;
  return profile;
}

} // namespace

TEST(WorldStepProfile, EmptyByDefault)
{
  const WorldStepProfile profile;
  EXPECT_TRUE(profile.isEmpty());
  EXPECT_EQ(profile.stepCount, 0u);
  EXPECT_EQ(profile.totalStageTime().count(), 0);
  EXPECT_EQ(profile.getStage("kinematics"), nullptr);
}

TEST(WorldStepProfile, TotalStageTimeSumsStages)
{
  const WorldStepProfile profile = makeProfile();
  EXPECT_FALSE(profile.isEmpty());
  // 6ms + 3ms + 1ms == 10ms, exactly the wall time here.
  EXPECT_EQ(profile.totalStageTime(), std::chrono::milliseconds(10));
}

TEST(WorldStepProfile, GetStageFindsByName)
{
  const WorldStepProfile profile = makeProfile();

  const auto* contact = profile.getStage("rigid_body_contact");
  ASSERT_NE(contact, nullptr);
  EXPECT_EQ(contact->domain, ComputeStageDomain::RigidBody);
  EXPECT_EQ(contact->duration, std::chrono::milliseconds(6));

  EXPECT_EQ(profile.getStage("does_not_exist"), nullptr);
}

TEST(WorldStepProfile, StageGraphHelpersSummarizeNestedGraphProfiles)
{
  WorldStepStageProfile stage;
  stage.name = "kinematics";
  stage.domain = ComputeStageDomain::Kinematics;
  stage.duration = 5ms;

  ComputeExecutionProfile first;
  first.workerCount = 2;
  first.wallTime = 2ms;
  first.maxParallelism = 1;
  stage.graphProfiles.push_back(first);

  ComputeExecutionProfile second;
  second.workerCount = 4;
  second.wallTime = 3ms;
  second.maxParallelism = 3;
  stage.graphProfiles.push_back(second);

  EXPECT_EQ(stage.totalGraphWallTime(), 5ms);
  EXPECT_EQ(stage.maxGraphWorkerCount(), 4u);
  EXPECT_EQ(stage.maxGraphParallelism(), 3u);
}

TEST(WorldStepProfile, SummaryTextListsStagesSlowestFirst)
{
  const WorldStepProfile profile = makeProfile();
  const std::string summary = profile.toSummaryText();

  // Header and every stage name appear.
  EXPECT_NE(summary.find("World Step Profile"), std::string::npos);
  EXPECT_NE(summary.find("rigid_body_contact"), std::string::npos);
  EXPECT_NE(summary.find("deformable_dynamics"), std::string::npos);
  EXPECT_NE(summary.find("kinematics"), std::string::npos);
  EXPECT_NE(summary.find("unattributed overhead"), std::string::npos);
  EXPECT_NE(summary.find("Execution details"), std::string::npos);
  EXPECT_NE(summary.find("acceleration=task_parallel|gpu"), std::string::npos);
  EXPECT_NE(summary.find("backend=accelerated"), std::string::npos);

  // Slowest-first ordering: the contact stage precedes the cheaper stages.
  const auto contactPos = summary.find("rigid_body_contact");
  const auto deformablePos = summary.find("deformable_dynamics");
  const auto kinematicsPos = summary.find("kinematics");
  EXPECT_LT(contactPos, deformablePos);
  EXPECT_LT(deformablePos, kinematicsPos);
}

TEST(WorldStepProfile, SummaryTextClampsInconsistentOverhead)
{
  WorldStepProfile profile = makeProfile();
  profile.wallTime = std::chrono::milliseconds(1);

  const std::string summary = profile.toSummaryText();
  const auto overheadPos = summary.find("(unattributed overhead)");
  ASSERT_NE(overheadPos, std::string::npos);
  const auto lineEnd = summary.find('\n', overheadPos);
  const std::string overheadLine = summary.substr(
      overheadPos,
      lineEnd == std::string::npos ? std::string::npos : lineEnd - overheadPos);

  EXPECT_NE(overheadLine.find("0.000"), std::string::npos);
}

TEST(WorldStepProfile, SummaryTextOnEmptyProfileIsInformative)
{
  const WorldStepProfile profile;
  const std::string summary = profile.toSummaryText();
  EXPECT_NE(summary.find("setStepProfilingEnabled"), std::string::npos);
}

TEST(WorldStepProfile, ResetClearsProfile)
{
  WorldStepProfile profile = makeProfile();
  ASSERT_FALSE(profile.isEmpty());

  profile.reset();
  EXPECT_TRUE(profile.isEmpty());
  EXPECT_EQ(profile.stepCount, 0u);
  EXPECT_EQ(profile.wallTime.count(), 0);
}
