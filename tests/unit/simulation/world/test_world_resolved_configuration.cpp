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
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 */

// PLAN-091 WP-091.11 slice 1: the World records the per-domain solver families
// it resolved at enterSimulationMode, exposed via getResolvedConfiguration()
// alongside the step profile. Today requested == resolved (no substitution);
// later slices classify the known silent substitutions.

#include <dart/simulation/body/rigid_body.hpp>
#include <dart/simulation/body/rigid_body_options.hpp>
#include <dart/simulation/comps/rigid_body.hpp>
#include <dart/simulation/compute/world_step_profile.hpp>
#include <dart/simulation/detail/entity_conversion.hpp>
#include <dart/simulation/detail/world_registry_access.hpp>
#include <dart/simulation/world.hpp>
#include <dart/simulation/world_options.hpp>

#include <Eigen/Core>
#include <gtest/gtest.h>

#include <string_view>

namespace sx = dart::simulation;

namespace {

const sx::compute::ResolvedConfigurationNote* findNote(
    const sx::compute::ResolvedSolverConfiguration& config,
    std::string_view domain)
{
  for (const auto& note : config.notes) {
    if (note.domain == domain) {
      return &note;
    }
  }
  return nullptr;
}

} // namespace

TEST(ResolvedConfiguration, EmptyBeforeSimulationMode)
{
  sx::World world;
  EXPECT_TRUE(world.getResolvedConfiguration().isEmpty());
  EXPECT_FALSE(world.getResolvedConfiguration().hasSubstitution());
}

TEST(ResolvedConfiguration, RecordsDefaultFamiliesAtFinalize)
{
  sx::World world;
  world.enterSimulationMode();

  const auto& config = world.getResolvedConfiguration();
  EXPECT_FALSE(config.isEmpty());
  EXPECT_FALSE(config.hasSubstitution());

  const auto* rigid = findNote(config, "rigid-body");
  ASSERT_NE(rigid, nullptr);
  EXPECT_EQ(rigid->resolved, "sequential-impulse");
  EXPECT_EQ(rigid->requested, rigid->resolved);
  EXPECT_FALSE(rigid->isSubstitution());

  const auto* contact = findNote(config, "rigid-contact");
  ASSERT_NE(contact, nullptr);
  EXPECT_EQ(contact->resolved, "sequential-impulse");

  const auto* multibody = findNote(config, "multibody");
  ASSERT_NE(multibody, nullptr);
  EXPECT_EQ(multibody->resolved, "semi-implicit");

  const auto* accelerator = findNote(config, "deformable-psd");
  ASSERT_NE(accelerator, nullptr);
  EXPECT_EQ(accelerator->requested, "cpu");
  EXPECT_EQ(accelerator->resolved, "cpu");
  EXPECT_FALSE(accelerator->isSubstitution());
}

TEST(ResolvedConfiguration, PreferAcceleratedFallsBackToCpuWhenUnavailable)
{
  sx::World world;
  world.enterSimulationMode();
  world.setComputeAcceleratorPolicy(
      sx::ComputeAcceleratorPolicy::PreferAccelerated);

  EXPECT_EQ(
      world.getComputeAcceleratorPolicy(),
      sx::ComputeAcceleratorPolicy::PreferAccelerated);
  const auto& config = world.getResolvedConfiguration();
  EXPECT_TRUE(config.hasSubstitution());

  const auto* accelerator = findNote(config, "deformable-psd");
  ASSERT_NE(accelerator, nullptr);
  EXPECT_TRUE(accelerator->isSubstitution());
  EXPECT_EQ(accelerator->requested, "accelerated");
  EXPECT_EQ(accelerator->resolved, "cpu");
  EXPECT_EQ(accelerator->reason, "no available accelerator registered");
}

TEST(ResolvedConfiguration, ReflectsRequestedContactMethod)
{
  sx::WorldOptions options;
  options.contactSolverMethod = sx::ContactSolverMethod::BoxedLcp;
  sx::World world(options);
  world.enterSimulationMode();

  const auto* contact
      = findNote(world.getResolvedConfiguration(), "rigid-contact");
  ASSERT_NE(contact, nullptr);
  EXPECT_EQ(contact->resolved, "boxed-lcp");
}

namespace {

// Emplace the internal AVBD rigid-contact opt-in on a body. The opt-in is not
// facade-selectable (PLAN-091 WP-091.1), so the resolved contact path then
// differs from the requested ContactSolverMethod.
sx::RigidBody addBodyWithAvbdContactConfig(sx::World& world)
{
  sx::RigidBodyOptions options;
  options.position = Eigen::Vector3d(0.0, 0.0, 0.5);
  auto body = world.addRigidBody("avbd_body", options);
  body.setCollisionShape(sx::CollisionShape::makeSphere(0.5));
  auto& registry = sx::detail::registryOf(world);
  registry.emplace_or_replace<sx::comps::RigidAvbdContactConfig>(
      sx::detail::toRegistryEntity(body.getEntity()));
  return body;
}

} // namespace

TEST(ResolvedConfiguration, RecordsAvbdContactSubstitution)
{
  sx::World world;
  addBodyWithAvbdContactConfig(world);
  world.enterSimulationMode();

  const auto& config = world.getResolvedConfiguration();
  EXPECT_TRUE(config.hasSubstitution());

  const auto* contact = findNote(config, "rigid-contact");
  ASSERT_NE(contact, nullptr);
  EXPECT_TRUE(contact->isSubstitution());
  EXPECT_EQ(contact->requested, "sequential-impulse");
  EXPECT_NE(contact->resolved.find("avbd"), std::string::npos);
}

TEST(ResolvedConfiguration, StrictResolutionRejectsSubstitution)
{
  sx::WorldOptions options;
  options.strictSolverResolution = true;
  sx::World world(options);
  addBodyWithAvbdContactConfig(world);

  EXPECT_THROW(world.enterSimulationMode(), std::exception);
}
