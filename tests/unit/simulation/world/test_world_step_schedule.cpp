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

#include "dart/simulation/detail/world_step_schedule.hpp"

#include <dart/simulation/compute/world_step_stage.hpp>

#include <gtest/gtest.h>

#include <vector>

namespace sxdetail = dart::simulation::detail;
namespace sxcompute = dart::simulation::compute;
namespace sx = dart::simulation;

namespace {

using RigidFamily = sxdetail::BuiltInRigidBodySolverFamily;
using MultibodyFamily = sxdetail::BuiltInMultibodyIntegrationFamily;
using Slot = sxdetail::BuiltInWorldStepStageSlot;

std::vector<Slot> slots(
    const sxdetail::BuiltInWorldStepScheduleOptions& options)
{
  const auto schedule = sxdetail::makeBuiltInWorldStepSchedule(options);
  return std::vector<Slot>(schedule.begin(), schedule.end());
}

sxdetail::BuiltInWorldStepScheduleOptions defaultOptions()
{
  sxdetail::BuiltInWorldStepScheduleOptions options;
  options.rigidBodySolver = RigidFamily::SequentialImpulse;
  options.multibodyIntegration = MultibodyFamily::SemiImplicit;
  options.hasRigidBodies = true;
  options.hasMultibodyStructures = false;
  options.hasDeformableBodies = false;
  options.includeKinematics = true;
  return options;
}

void expectInlineCapacity(
    const sxdetail::BuiltInWorldStepScheduleOptions& options)
{
  const auto schedule = sxdetail::makeBuiltInWorldStepSchedule(options);
  EXPECT_LE(schedule.count, sxcompute::WorldStepPipeline::kInlineStageCount);
  EXPECT_LE(schedule.count, sxdetail::kMaxBuiltInWorldStepStageSlots);
}

} // namespace

TEST(BuiltInWorldStepSchedule, FreeRigidDefaultUsesSplitContactPipeline)
{
  auto options = defaultOptions();

  EXPECT_EQ(
      slots(options),
      (std::vector<Slot>{
          Slot::RigidBodyVelocity,
          Slot::RigidBodyContact,
          Slot::RigidBodyPosition,
          Slot::Kinematics}));
  expectInlineCapacity(options);
}

TEST(BuiltInWorldStepSchedule, EmptyWorldUsesOnlyWorldLevelKinematics)
{
  sxdetail::BuiltInWorldStepScheduleOptions options;
  options.includeKinematics = true;

  EXPECT_EQ(slots(options), (std::vector<Slot>{Slot::Kinematics}));
  expectInlineCapacity(options);
}

TEST(BuiltInWorldStepSchedule, DeformableOnlyWorldUsesDeformableStage)
{
  sxdetail::BuiltInWorldStepScheduleOptions options;
  options.hasDeformableBodies = true;
  options.includeKinematics = true;

  EXPECT_EQ(
      slots(options),
      (std::vector<Slot>{Slot::DeformableDynamics, Slot::Kinematics}));
  expectInlineCapacity(options);
}

TEST(BuiltInWorldStepSchedule, FreeRigidWithDeformableAddsActiveDomains)
{
  auto options = defaultOptions();
  options.hasDeformableBodies = true;

  EXPECT_EQ(
      slots(options),
      (std::vector<Slot>{
          Slot::RigidBodyVelocity,
          Slot::RigidBodyContact,
          Slot::DeformableDynamics,
          Slot::RigidBodyPosition,
          Slot::Kinematics}));
  expectInlineCapacity(options);
}

TEST(BuiltInWorldStepSchedule, ArticulatedDefaultUsesUnifiedConstraintSolve)
{
  auto options = defaultOptions();
  options.hasMultibodyStructures = true;

  const auto schedule = sxdetail::makeBuiltInWorldStepSchedule(options);
  EXPECT_EQ(
      std::vector<Slot>(schedule.begin(), schedule.end()),
      (std::vector<Slot>{
          Slot::RigidBodyVelocity,
          Slot::MultibodyVelocity,
          Slot::UnifiedConstraint,
          Slot::RigidBodyPosition,
          Slot::MultibodyPosition,
          Slot::Kinematics}));
  EXPECT_TRUE(schedule.contains(Slot::UnifiedConstraint));
  EXPECT_FALSE(schedule.contains(Slot::RigidBodyContact));
  EXPECT_FALSE(schedule.contains(Slot::MultibodyForwardDynamics));
  expectInlineCapacity(options);
}

TEST(BuiltInWorldStepSchedule, ArticulatedOnlyDefaultSkipsRigidSlots)
{
  auto options = defaultOptions();
  options.hasRigidBodies = false;
  options.hasMultibodyStructures = true;

  EXPECT_EQ(
      slots(options),
      (std::vector<Slot>{
          Slot::MultibodyVelocity,
          Slot::UnifiedConstraint,
          Slot::MultibodyPosition,
          Slot::Kinematics}));
  expectInlineCapacity(options);
}

TEST(BuiltInWorldStepSchedule, VariationalSelectionSubstitutesMultibodyStage)
{
  auto options = defaultOptions();
  options.multibodyIntegration = MultibodyFamily::Variational;
  options.hasMultibodyStructures = true;

  EXPECT_EQ(
      slots(options),
      (std::vector<Slot>{
          Slot::RigidBodyVelocity,
          Slot::RigidBodyContact,
          Slot::MultibodyVariationalIntegration,
          Slot::RigidBodyPosition,
          Slot::Kinematics}));
  expectInlineCapacity(options);
}

TEST(BuiltInWorldStepSchedule, VariationalWithDeformableKeepsAllDomains)
{
  auto options = defaultOptions();
  options.multibodyIntegration = MultibodyFamily::Variational;
  options.hasMultibodyStructures = true;
  options.hasDeformableBodies = true;

  EXPECT_EQ(
      slots(options),
      (std::vector<Slot>{
          Slot::RigidBodyVelocity,
          Slot::RigidBodyContact,
          Slot::MultibodyVariationalIntegration,
          Slot::DeformableDynamics,
          Slot::RigidBodyPosition,
          Slot::Kinematics}));
  expectInlineCapacity(options);
}

TEST(BuiltInWorldStepSchedule, VariationalSelectionRequiresMultibodyDomain)
{
  auto options = defaultOptions();
  options.multibodyIntegration = MultibodyFamily::Variational;
  options.hasMultibodyStructures = false;

  EXPECT_EQ(
      slots(options),
      (std::vector<Slot>{
          Slot::RigidBodyVelocity,
          Slot::RigidBodyContact,
          Slot::RigidBodyPosition,
          Slot::Kinematics}));
  expectInlineCapacity(options);
}

TEST(BuiltInWorldStepSchedule, ArticulatedDefaultWithDeformableKeepsAllDomains)
{
  auto options = defaultOptions();
  options.hasMultibodyStructures = true;
  options.hasDeformableBodies = true;

  EXPECT_EQ(
      slots(options),
      (std::vector<Slot>{
          Slot::RigidBodyVelocity,
          Slot::MultibodyVelocity,
          Slot::UnifiedConstraint,
          Slot::RigidBodyPosition,
          Slot::MultibodyPosition,
          Slot::DeformableDynamics,
          Slot::Kinematics}));
  expectInlineCapacity(options);
}

TEST(BuiltInWorldStepSchedule, IpcOwnsFreeRigidAdvance)
{
  auto options = defaultOptions();
  options.rigidBodySolver = RigidFamily::Ipc;

  EXPECT_EQ(
      slots(options),
      (std::vector<Slot>{Slot::RigidIpcContact, Slot::Kinematics}));
  expectInlineCapacity(options);
}

TEST(BuiltInWorldStepSchedule, IpcCanComposeWithMultibodyDomain)
{
  auto options = defaultOptions();
  options.rigidBodySolver = RigidFamily::Ipc;
  options.multibodyIntegration = MultibodyFamily::Variational;
  options.hasMultibodyStructures = true;

  EXPECT_EQ(
      slots(options),
      (std::vector<Slot>{
          Slot::RigidIpcContact,
          Slot::MultibodyVariationalIntegration,
          Slot::Kinematics}));
  expectInlineCapacity(options);
}

TEST(BuiltInWorldStepSchedule, IpcWithDeformableKeepsAllDomains)
{
  auto options = defaultOptions();
  options.rigidBodySolver = RigidFamily::Ipc;
  options.hasMultibodyStructures = true;
  options.hasDeformableBodies = true;

  EXPECT_EQ(
      slots(options),
      (std::vector<Slot>{
          Slot::RigidIpcContact,
          Slot::MultibodyForwardDynamics,
          Slot::DeformableDynamics,
          Slot::Kinematics}));
  expectInlineCapacity(options);
}

TEST(BuiltInWorldStepSchedule, IpcWithoutRigidBodiesSkipsRigidIpcStage)
{
  auto options = defaultOptions();
  options.rigidBodySolver = RigidFamily::Ipc;
  options.hasRigidBodies = false;
  options.hasMultibodyStructures = true;

  EXPECT_EQ(
      slots(options),
      (std::vector<Slot>{Slot::MultibodyForwardDynamics, Slot::Kinematics}));
  expectInlineCapacity(options);
}

TEST(BuiltInWorldStepSchedule, CustomFinalStageKeepsDynamicsScheduleShared)
{
  auto options = defaultOptions();
  options.hasMultibodyStructures = true;
  options.includeKinematics = false;

  EXPECT_EQ(
      slots(options),
      (std::vector<Slot>{
          Slot::RigidBodyVelocity,
          Slot::MultibodyVelocity,
          Slot::UnifiedConstraint,
          Slot::RigidBodyPosition,
          Slot::MultibodyPosition}));
  expectInlineCapacity(options);
}

// The former `builtInWorldStepScheduleNeedsPreparation` table was removed in
// PLAN-091 WP-091.10: which stages prepare is now expressed by overriding the
// virtual `compute::WorldStepStage::prepare()`, so there is no separate table
// to test. Stage preparation behavior is covered by the World step suites and
// the default-step golden trajectories.

TEST(BuiltInWorldStepSchedule, AddRejectsOverflowPastInlineCapacity)
{
  sxdetail::BuiltInWorldStepSchedule schedule;
  for (std::size_t i = 0; i < sxdetail::kMaxBuiltInWorldStepStageSlots; ++i) {
    schedule.add(Slot::DeformableDynamics);
  }

  EXPECT_EQ(schedule.count, sxdetail::kMaxBuiltInWorldStepStageSlots);
  EXPECT_THROW(schedule.add(Slot::Kinematics), sx::InvalidOperationException);
  EXPECT_EQ(schedule.count, sxdetail::kMaxBuiltInWorldStepStageSlots);
}
