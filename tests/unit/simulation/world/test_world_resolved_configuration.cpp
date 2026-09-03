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

#include <dart/simulation/body/contact.hpp>
#include <dart/simulation/body/deformable_body.hpp>
#include <dart/simulation/body/deformable_body_options.hpp>
#include <dart/simulation/body/rigid_body.hpp>
#include <dart/simulation/body/rigid_body_options.hpp>
#include <dart/simulation/common/exceptions.hpp>
#include <dart/simulation/comps/deformable_body.hpp>
#include <dart/simulation/comps/joint.hpp>
#include <dart/simulation/comps/rigid_body.hpp>
#include <dart/simulation/comps/variational_contact_dual_state.hpp>
#include <dart/simulation/compute/variational_integration.hpp>
#include <dart/simulation/compute/world_step_profile.hpp>
#include <dart/simulation/detail/entity_conversion.hpp>
#include <dart/simulation/detail/rigid_avbd/rigid_world_contact.hpp>
#include <dart/simulation/detail/world_registry_access.hpp>
#include <dart/simulation/multibody/multibody.hpp>
#include <dart/simulation/world.hpp>
#include <dart/simulation/world_options.hpp>

#include <dart/common/stl_allocator.hpp>

#include <Eigen/Core>
#include <gtest/gtest.h>

#include <functional>
#include <new>
#include <sstream>
#include <string>
#include <string_view>

namespace sx = dart::simulation;

namespace {

class PredicateFailOnceMemoryAllocator final
  : public dart::common::MemoryAllocator
{
public:
  [[nodiscard]] std::string_view getType() const override
  {
    return "PredicateFailOnceMemoryAllocator";
  }

  [[nodiscard]] void* allocate(std::size_t bytes) noexcept override
  {
    return allocateImpl(bytes, __STDCPP_DEFAULT_NEW_ALIGNMENT__);
  }

  [[nodiscard]] void* allocate(
      std::size_t bytes, std::size_t alignment) noexcept override
  {
    return allocateImpl(bytes, alignment);
  }

  void deallocate(void* pointer, std::size_t /*bytes*/) override
  {
    ::operator delete(pointer);
  }

  void deallocate(
      void* pointer, std::size_t /*bytes*/, std::size_t alignment) override
  {
    if (alignment <= __STDCPP_DEFAULT_NEW_ALIGNMENT__) {
      ::operator delete(pointer);
      return;
    }
    ::operator delete(pointer, std::align_val_t(alignment));
  }

  void failOnceWhen(std::function<bool()> predicate)
  {
    m_predicate = std::move(predicate);
    m_armed = true;
  }

  [[nodiscard]] std::size_t failureCount() const noexcept
  {
    return m_failureCount;
  }

private:
  [[nodiscard]] void* allocateImpl(
      std::size_t bytes, std::size_t alignment) noexcept
  {
    if (bytes == 0u || alignment == 0u
        || (alignment & (alignment - 1u)) != 0u) {
      return nullptr;
    }
    if (m_armed && m_predicate && m_predicate()) {
      m_armed = false;
      ++m_failureCount;
      return nullptr;
    }
    if (alignment <= __STDCPP_DEFAULT_NEW_ALIGNMENT__) {
      return ::operator new(bytes, std::nothrow);
    }
    return ::operator new(bytes, std::align_val_t(alignment), std::nothrow);
  }

  std::function<bool()> m_predicate;
  bool m_armed = false;
  std::size_t m_failureCount = 0u;
};

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

TEST(ResolvedConfiguration, ClearRemovesFinalizedSolverIdentity)
{
  sx::World world;
  world.enterSimulationMode();
  ASSERT_FALSE(world.getResolvedConfiguration().isEmpty());

  world.clear();

  EXPECT_FALSE(world.isSimulationMode());
  EXPECT_TRUE(world.getResolvedConfiguration().isEmpty());
  EXPECT_FALSE(world.getResolvedConfiguration().hasSubstitution());
}

TEST(ResolvedConfiguration, IdempotentRigidModelSettersDoNotInvalidateBake)
{
  sx::World world;
  sx::RigidBodyOptions options;
  options.isStatic = true;
  auto body = world.addRigidBody("static", options);
  body.setCollisionShape(sx::CollisionShape::makeSphere(0.5));
  const sx::DeformableObstaclePolicy policy{
      .groundBarrier = true, .barrierOnly = true};
  body.setDeformableObstaclePolicy(policy);
  world.enterSimulationMode();
  const std::string resolvedBefore
      = world.getResolvedConfiguration().toSummaryText();
  ASSERT_FALSE(resolvedBefore.empty());

  body.setStatic(true);
  body.setKinematic(false);
  body.setDeformableObstaclePolicy(policy);
  body.setMass(body.getMass());
  body.setInertia(body.getInertia());

  EXPECT_EQ(world.getResolvedConfiguration().toSummaryText(), resolvedBefore);
}

TEST(ResolvedConfiguration, RestoringDesignModeRemovesLaterVbdIdentity)
{
  sx::World world;
  sx::DeformableBodyOptions bodyOptions;
  bodyOptions.positions
      = {Eigen::Vector3d(0.0, 0.0, 0.0), Eigen::Vector3d(0.0, 0.0, -0.5)};
  bodyOptions.masses = {1.0, 1.0};
  bodyOptions.fixedNodes = {0u};
  bodyOptions.edges = {sx::DeformableEdge{0u, 1u, -1.0}};
  world.addDeformableBody("strand", bodyOptions);
  world.configureDeformableSolver("strand", sx::DeformableSolverOptions{});

  world.setReplayRecordingEnabled(true);
  ASSERT_EQ(world.getReplayFrameCount(), 1u);
  world.step();
  ASSERT_TRUE(world.isSimulationMode());
  ASSERT_FALSE(world.getResolvedConfiguration().isEmpty());
  ASSERT_NE(
      world.getResolvedConfiguration().toSummaryText().find(
          "deformable-inner-solver: vbd -> vbd"),
      std::string::npos);

  world.restoreReplayFrame(0u);

  EXPECT_FALSE(world.isSimulationMode());
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

TEST(ResolvedConfiguration, RecordsRequiredPublicDeformableVbdSelection)
{
  sx::World world;
  sx::DeformableBodyOptions bodyOptions;
  bodyOptions.positions
      = {Eigen::Vector3d(0.0, 0.0, 0.0), Eigen::Vector3d(0.0, 0.0, -0.5)};
  bodyOptions.masses = {1.0, 1.0};
  bodyOptions.fixedNodes = {0u};
  bodyOptions.edges = {sx::DeformableEdge{0u, 1u, -1.0}};
  world.addDeformableBody("strand", bodyOptions);

  sx::DeformableSolverOptions solverOptions;
  solverOptions.iterations = 9u;
  world.configureDeformableSolver("strand", solverOptions);
  world.enterSimulationMode();

  const auto* note
      = findNote(world.getResolvedConfiguration(), "deformable-inner-solver");
  ASSERT_NE(note, nullptr);
  EXPECT_EQ(note->requested, "vbd");
  EXPECT_EQ(note->resolved, "vbd");
  EXPECT_FALSE(note->isSubstitution());
  EXPECT_NE(
      note->reason.find("required to execute or fail closed"),
      std::string::npos);
}

TEST(ResolvedConfiguration, InternalDeformableVbdFallbackIsNotReportedAsVbd)
{
  sx::World world;
  sx::RigidBodyOptions groundOptions;
  groundOptions.isStatic = true;
  groundOptions.position = Eigen::Vector3d(0.0, 0.0, -0.5);
  auto ground = world.addRigidBody("ground", groundOptions);
  ground.setCollisionShape(
      sx::CollisionShape::makeBox(Eigen::Vector3d(2.0, 2.0, 0.5)));
  auto obstaclePolicy = ground.getDeformableObstaclePolicy();
  obstaclePolicy.groundBarrier = true;
  ground.setDeformableObstaclePolicy(obstaclePolicy);

  sx::DeformableBodyOptions bodyOptions;
  bodyOptions.positions = {Eigen::Vector3d(0.0, 0.0, 0.1)};
  bodyOptions.masses = {1.0};
  world.addDeformableBody("point", bodyOptions);
  auto& registry = dart::simulation::detail::registryOf(world);
  const auto entity = *registry.view<sx::comps::DeformableBodyTag>().begin();
  registry.emplace<sx::comps::DeformableVbdConfig>(
      entity,
      sx::comps::DeformableVbdConfig{.enabled = true, .iterations = 9u});

  world.enterSimulationMode();
  const auto* note
      = findNote(world.getResolvedConfiguration(), "deformable-inner-solver");
  ASSERT_NE(note, nullptr);
  EXPECT_EQ(note->requested, "vbd (internal opt-in)");
  EXPECT_EQ(note->resolved, "runtime vbd-or-projected-newton");
  EXPECT_TRUE(note->isSubstitution());
  EXPECT_NE(
      note->reason.find("fall back to projected Newton"), std::string::npos);

  world.step();
  const auto& diagnostics = world.getLastDeformableSolverDiagnostics();
  EXPECT_EQ(diagnostics.vbdBodyCount, 0u);
  EXPECT_GT(diagnostics.solverIterations, 0u);
}

TEST(ResolvedConfiguration, DistinguishesMixedPublicAndInternalVbdSelections)
{
  sx::World world;
  sx::DeformableBodyOptions bodyOptions;
  bodyOptions.positions
      = {Eigen::Vector3d(0.0, 0.0, 0.0), Eigen::Vector3d(0.5, 0.0, 0.0)};
  bodyOptions.masses = {1.0, 1.0};
  bodyOptions.fixedNodes = {0u};
  bodyOptions.edges = {sx::DeformableEdge{0u, 1u, 0.5}};

  world.addDeformableBody("internal", bodyOptions);
  auto& registry = dart::simulation::detail::registryOf(world);
  const auto internalEntity
      = *registry.view<sx::comps::DeformableBodyTag>().begin();
  registry.emplace<sx::comps::DeformableVbdConfig>(
      internalEntity,
      sx::comps::DeformableVbdConfig{.enabled = true, .iterations = 7u});

  bodyOptions.positions[0].x() += 2.0;
  bodyOptions.positions[1].x() += 2.0;
  world.addDeformableBody("public", bodyOptions);
  world.configureDeformableSolver("public", sx::DeformableSolverOptions{});
  world.enterSimulationMode();

  const auto* note
      = findNote(world.getResolvedConfiguration(), "deformable-inner-solver");
  ASSERT_NE(note, nullptr);
  EXPECT_EQ(note->requested, "vbd (mixed public/internal selections)");
  EXPECT_EQ(
      note->resolved,
      "per-body required-vbd + runtime vbd-or-projected-newton");
  EXPECT_TRUE(note->isSubstitution());
  EXPECT_NE(
      note->reason.find("1 of 2 VBD selections require VBD execution"),
      std::string::npos);
  EXPECT_NE(
      note->reason.find("1 internal compatibility opt-ins"), std::string::npos);
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

TEST(ResolvedConfiguration, RecordsPublicAvbdFamilyAsRequested)
{
  sx::WorldOptions options;
  options.rigidBodySolver = sx::RigidBodySolver::Avbd;
  options.rigidConstraintOptions.iterations = 20;
  sx::World world(options);
  auto parent = world.addRigidBody("parent");
  sx::RigidBodyOptions childOptions;
  childOptions.position = Eigen::Vector3d::UnitX();
  auto child = world.addRigidBody("child", childOptions);
  world.addJoint(
      parent,
      child,
      sx::JointSpec{.name = "fixed", .type = sx::JointType::Fixed});
  world.enterSimulationMode();

  const auto& config = world.getResolvedConfiguration();
  EXPECT_FALSE(config.hasSubstitution());

  const auto* rigid = findNote(config, "rigid-body");
  ASSERT_NE(rigid, nullptr);
  EXPECT_EQ(rigid->requested, "avbd");
  EXPECT_EQ(rigid->resolved, "avbd");

  const auto* profile = findNote(config, "rigid-avbd-parameter-profile");
  ASSERT_NE(profile, nullptr);
  EXPECT_EQ(profile->requested, "paper-2025-table-2");
  EXPECT_EQ(profile->resolved, "paper-2025-table-2");
  EXPECT_NE(profile->reason.find("alpha=0.95"), std::string::npos);
  EXPECT_NE(profile->reason.find("beta=10"), std::string::npos);
  EXPECT_NE(profile->reason.find("gamma=0.99"), std::string::npos);

  const auto* contact = findNote(config, "rigid-contact");
  ASSERT_NE(contact, nullptr);
  EXPECT_EQ(contact->requested, "avbd");
  EXPECT_EQ(contact->resolved, "avbd");
  EXPECT_EQ(contact->reason, "as requested");

  const auto* pairConstraint = findNote(config, "rigid-pair-constraint");
  ASSERT_NE(pairConstraint, nullptr);
  EXPECT_EQ(pairConstraint->requested, "avbd");
  EXPECT_EQ(pairConstraint->resolved, "avbd");
  EXPECT_EQ(pairConstraint->reason, "as requested");

  const auto* iterations = findNote(config, "rigid-constraint-iterations");
  ASSERT_NE(iterations, nullptr);
  EXPECT_EQ(iterations->requested, "20");
  EXPECT_EQ(iterations->resolved, "20");
}

TEST(ResolvedConfiguration, RecordsSelectedAvbdSourceDemoProfiles)
{
  const auto checkProfile = [](sx::RigidAvbdParameterProfile profile,
                               const std::string& expectedName,
                               const std::vector<std::string>& reasonParts) {
    sx::WorldOptions options;
    options.rigidBodySolver = sx::RigidBodySolver::Avbd;
    options.rigidAvbdParameterProfile = profile;
    options.rigidConstraintOptions.iterations = 20;
    sx::World world(options);
    world.addRigidBody("body");
    world.enterSimulationMode();
    EXPECT_EQ(world.getRigidAvbdParameterProfile(), profile);
    const auto& config = world.getResolvedConfiguration();
    EXPECT_FALSE(config.hasSubstitution());
    const auto* note = findNote(config, "rigid-avbd-parameter-profile");
    ASSERT_NE(note, nullptr) << expectedName;
    EXPECT_EQ(note->requested, expectedName);
    EXPECT_EQ(note->resolved, expectedName);
    for (const auto& part : reasonParts) {
      EXPECT_NE(note->reason.find(part), std::string::npos)
          << expectedName << ": " << note->reason;
    }
  };
  checkProfile(
      sx::RigidAvbdParameterProfile::Paper2025Table2,
      "paper-2025-table-2",
      {"alpha=0.95", "beta=10,", "gamma=0.99"});
  checkProfile(
      sx::RigidAvbdParameterProfile::SourceDemo2d,
      "source-demo-2d",
      {"alpha=0.99", "beta=100000", "gamma=0.99", "post-stabilization"});
  checkProfile(
      sx::RigidAvbdParameterProfile::SourceDemo3d,
      "source-demo-3d",
      {"alpha=0.99", "beta=10000,", "betaAngular=100", "gamma=0.999"});
}

TEST(ResolvedConfiguration, RecordsPublicVbdFamilyAsRequested)
{
  sx::WorldOptions options;
  options.rigidBodySolver = sx::RigidBodySolver::Vbd;
  options.rigidConstraintOptions.iterations = 20;
  sx::World world(options);
  auto parent = world.addRigidBody("parent");
  sx::RigidBodyOptions childOptions;
  childOptions.position = Eigen::Vector3d::UnitX();
  auto child = world.addRigidBody("child", childOptions);
  auto joint = world.addJoint(
      parent,
      child,
      sx::JointSpec{.name = "fixed", .type = sx::JointType::Fixed});
  joint.setConstraintProjectionPolicy(
      {.startStiffness = 1.0e5,
       .linearStiffness = 1.0e5,
       .angularStiffness = 1.0e5});
  world.enterSimulationMode();

  const auto& config = world.getResolvedConfiguration();
  EXPECT_FALSE(config.hasSubstitution());

  const auto* rigid = findNote(config, "rigid-body");
  ASSERT_NE(rigid, nullptr);
  EXPECT_EQ(rigid->requested, "vbd");
  EXPECT_EQ(rigid->resolved, "vbd");
  EXPECT_EQ(findNote(config, "rigid-avbd-parameter-profile"), nullptr);

  const auto* contact = findNote(config, "rigid-contact");
  ASSERT_NE(contact, nullptr);
  EXPECT_EQ(contact->requested, "vbd");
  EXPECT_EQ(contact->resolved, "vbd");
  EXPECT_EQ(contact->reason, "as requested");

  const auto* pairConstraint = findNote(config, "rigid-pair-constraint");
  ASSERT_NE(pairConstraint, nullptr);
  EXPECT_EQ(pairConstraint->requested, "vbd");
  EXPECT_EQ(pairConstraint->resolved, "vbd");
  EXPECT_EQ(pairConstraint->reason, "as requested");

  const auto* iterations = findNote(config, "rigid-constraint-iterations");
  ASSERT_NE(iterations, nullptr);
  EXPECT_EQ(iterations->requested, "20");
  EXPECT_EQ(iterations->resolved, "20");
}

namespace {

// Emplace the internal AVBD rigid-contact opt-in on a body. The opt-in is not
// facade-selectable (PLAN-091 WP-091.1), so the resolved contact path then
// differs from the requested ContactSolverMethod.
sx::RigidBody addCollisionBody(
    sx::World& world, std::string_view name, double x)
{
  sx::RigidBodyOptions options;
  options.position = Eigen::Vector3d(x, 0.0, 0.5);
  auto body = world.addRigidBody(name, options);
  body.setCollisionShape(sx::CollisionShape::makeSphere(0.5));
  return body;
}

sx::RigidBody addBodyWithAvbdContactConfig(
    sx::World& world, bool enabled = true)
{
  auto body = addCollisionBody(world, "avbd_body", 0.0);
  auto& registry = sx::detail::registryOf(world);
  auto& config = registry.emplace_or_replace<sx::comps::RigidAvbdContactConfig>(
      sx::detail::toRegistryEntity(body.getEntity()));
  config.enabled = enabled;
  return body;
}

void addMixedRigidAndMultibodyContactScene(sx::World& world)
{
  world.setGravity(Eigen::Vector3d::Zero());

  sx::RigidBodyOptions rigidOptions;
  rigidOptions.position = Eigen::Vector3d(0.0, 0.0, 0.75);
  auto rigidBody = world.addRigidBody("mixed_rigid", rigidOptions);
  rigidBody.setCollisionShape(sx::CollisionShape::makeSphere(0.5));

  auto multibody = world.addMultibody("mixed_multibody");
  auto base = multibody.addLink("base");
  base.setMass(1.0);
  base.setCollisionShape(sx::CollisionShape::makeSphere(0.5));
}

struct TransactionalRigidScene
{
  sx::RigidBody configured;
  sx::RigidBody plain;
};

struct WarmAvbdTransactionalScene
{
  sx::RigidBody contact;
  sx::RigidBody jointChild;
  entt::entity deformableEntity = entt::null;
};

struct WarmAvbdHardConstraintScene
{
  sx::RigidBody contact;
  sx::RigidBody jointChild;
};

struct PersistentEntryScene
{
  entt::entity multibodyEntity = entt::null;
  entt::entity deformableEntity = entt::null;
  sx::Joint actuatorJoint;
};

TransactionalRigidScene addTransactionalRigidScene(
    sx::World& world, bool compatibilityAvbdEnabled)
{
  world.setGravity(Eigen::Vector3d::Zero());
  auto configured
      = addBodyWithAvbdContactConfig(world, compatibilityAvbdEnabled);
  auto plain = addCollisionBody(world, "plain_body", 0.75);
  configured.setLinearVelocity(Eigen::Vector3d(0.25, -0.1, 0.0));
  plain.setLinearVelocity(Eigen::Vector3d(-0.15, 0.05, 0.0));
  return {configured, plain};
}

WarmAvbdTransactionalScene addWarmAvbdTransactionalScene(sx::World& world)
{
  sx::RigidBodyOptions groundOptions;
  groundOptions.isStatic = true;
  groundOptions.position = Eigen::Vector3d(0.0, 0.0, -0.25);
  auto ground = world.addRigidBody("ground", groundOptions);
  ground.setCollisionShape(
      sx::CollisionShape::makeBox(Eigen::Vector3d(2.0, 2.0, 0.25)));

  sx::RigidBodyOptions contactOptions;
  contactOptions.mass = 1.0;
  contactOptions.position = Eigen::Vector3d(0.0, 0.0, 0.4);
  contactOptions.linearVelocity = Eigen::Vector3d(0.6, 0.2, -1.0);
  auto contact = world.addRigidBody("contact", contactOptions);
  contact.setCollisionShape(sx::CollisionShape::makeSphere(0.5));
  contact.setFriction(0.8);

  sx::RigidBodyOptions parentOptions;
  parentOptions.isStatic = true;
  parentOptions.position = Eigen::Vector3d(3.0, 0.0, 0.0);
  auto parent = world.addRigidBody("joint_parent", parentOptions);

  sx::RigidBodyOptions childOptions;
  childOptions.mass = 1.0;
  childOptions.position = Eigen::Vector3d(4.0, 0.0, 0.0);
  auto jointChild = world.addRigidBody("joint_child", childOptions);
  auto joint = world.addJoint(
      parent,
      jointChild,
      sx::JointSpec{.name = "fixed", .type = sx::JointType::Fixed});
  joint.setConstraintProjectionPolicy(
      {.startStiffness = 1.0,
       .linearStiffness = 200.0,
       .angularStiffness = 200.0});
  Eigen::Isometry3d displacedJointPose = Eigen::Isometry3d::Identity();
  displacedJointPose.translation() = Eigen::Vector3d(4.3, 0.2, 0.0);
  jointChild.setTransform(displacedJointPose);
  jointChild.setLinearVelocity(Eigen::Vector3d(0.4, -0.3, 0.0));
  jointChild.setAngularVelocity(Eigen::Vector3d(0.0, 0.0, 0.5));

  sx::DeformableBodyOptions deformableOptions;
  deformableOptions.positions = {Eigen::Vector3d(10.0, 0.0, 0.0)};
  deformableOptions.masses = {1.0};
  world.addDeformableBody("strict_trigger", deformableOptions);
  auto& registry = sx::detail::registryOf(world);
  const auto deformableView = registry.view<sx::comps::DeformableBodyTag>();
  const entt::entity deformableEntity = *deformableView.begin();
  auto& vbdConfig
      = registry.emplace<sx::comps::DeformableVbdConfig>(deformableEntity);
  vbdConfig.enabled = false;

  return {contact, jointChild, deformableEntity};
}

WarmAvbdHardConstraintScene addWarmAvbdHardConstraintScene(sx::World& world)
{
  sx::RigidBodyOptions groundOptions;
  groundOptions.isStatic = true;
  groundOptions.position = Eigen::Vector3d(0.0, 0.0, -0.25);
  auto ground = world.addRigidBody("ground", groundOptions);
  ground.setCollisionShape(
      sx::CollisionShape::makeBox(Eigen::Vector3d(2.0, 2.0, 0.25)));

  sx::RigidBodyOptions contactOptions;
  contactOptions.mass = 1.0;
  contactOptions.position = Eigen::Vector3d(0.0, 0.0, 0.4);
  contactOptions.linearVelocity = Eigen::Vector3d(0.6, 0.2, -1.0);
  auto contact = world.addRigidBody("contact", contactOptions);
  contact.setCollisionShape(sx::CollisionShape::makeSphere(0.5));
  contact.setFriction(0.8);

  sx::RigidBodyOptions parentOptions;
  parentOptions.isStatic = true;
  parentOptions.position = Eigen::Vector3d(3.0, 0.0, 0.0);
  auto parent = world.addRigidBody("joint_parent", parentOptions);

  sx::RigidBodyOptions childOptions;
  childOptions.mass = 1.0;
  childOptions.position = Eigen::Vector3d(4.0, 0.0, 0.0);
  auto jointChild = world.addRigidBody("joint_child", childOptions);
  world.addJoint(
      parent,
      jointChild,
      sx::JointSpec{.name = "fixed", .type = sx::JointType::Fixed});
  Eigen::Isometry3d displacedJointPose = Eigen::Isometry3d::Identity();
  displacedJointPose.translation() = Eigen::Vector3d(4.3, 0.2, 0.0);
  jointChild.setTransform(displacedJointPose);
  jointChild.setLinearVelocity(Eigen::Vector3d(0.4, -0.3, 0.0));
  jointChild.setAngularVelocity(Eigen::Vector3d(0.0, 0.0, 0.5));

  return {contact, jointChild};
}

sx::RigidBody addDistanceSpringScene(sx::World& world)
{
  sx::RigidBodyOptions baseOptions;
  baseOptions.isStatic = true;
  baseOptions.position = Eigen::Vector3d(-1.0, 0.0, 0.0);
  auto base = world.addRigidBody("spring_base", baseOptions);

  sx::RigidBodyOptions linkOptions;
  linkOptions.mass = 1.0;
  linkOptions.position = Eigen::Vector3d(1.0, 0.0, 0.0);
  auto link = world.addRigidBody("spring_link", linkOptions);
  world.addRigidBodyDistanceSpring(
      "spring", base, link, /*restLength=*/1.0, /*stiffness=*/200.0);
  return link;
}

PersistentEntryScene addPersistentEntryScene(
    sx::World& world, std::size_t deformableNodeCount = 4096u)
{
  auto robot = world.addMultibody("persistent_entry_robot");
  auto parent = robot.addLink("base");
  robot.setGroundContact(
      Eigen::Vector3d::UnitZ(),
      Eigen::Vector3d::Zero(),
      1.0e4,
      /*frictionCoefficient=*/0.0,
      /*frictionRegularization=*/1.0e-4,
      /*dampingCoefficient=*/20.0,
      /*dualUpdateCadence=*/1u);
  std::optional<sx::Joint> actuatorJoint;
  for (std::size_t index = 0u; index < 8u; ++index) {
    sx::JointSpec jointSpec;
    jointSpec.name = "rail_" + std::to_string(index);
    jointSpec.type = sx::JointType::Prismatic;
    jointSpec.axis = Eigen::Vector3d::UnitZ();
    auto link
        = robot.addLink("link_" + std::to_string(index), parent, jointSpec);
    link.setMass(1.0);
    link.setInertia(Eigen::Vector3d(0.01, 0.01, 0.01).asDiagonal());
    robot.addGroundContactPoint(link, Eigen::Vector3d::Zero());
    if (!actuatorJoint) {
      actuatorJoint = link.getParentJoint();
    }
    parent = link;
  }

  sx::DeformableBodyOptions deformableOptions;
  deformableOptions.positions.reserve(deformableNodeCount);
  deformableOptions.masses.assign(deformableNodeCount, 1.0);
  for (std::size_t node = 0u; node < deformableNodeCount; ++node) {
    deformableOptions.positions.emplace_back(
        10.0 + 0.001 * static_cast<double>(node), 0.0, 0.0);
  }
  world.addDeformableBody("persistent_entry_deformable", deformableOptions);

  auto& registry = sx::detail::registryOf(world);
  const entt::entity deformableEntity
      = *registry.view<sx::comps::DeformableBodyTag>().begin();
  registry.remove<sx::comps::DeformableContactConfig>(deformableEntity);
  return {
      sx::detail::toRegistryEntity(robot.getEntity()),
      deformableEntity,
      *actuatorJoint};
}

void setCompatibilityAvbdEnabled(
    sx::World& world, const sx::RigidBody& body, bool enabled)
{
  auto& registry = sx::detail::registryOf(world);
  registry
      .get<sx::comps::RigidAvbdContactConfig>(
          sx::detail::toRegistryEntity(body.getEntity()))
      .enabled = enabled;
}

void expectSameNextRigidStep(
    sx::World& actual,
    const TransactionalRigidScene& actualScene,
    sx::World& expected,
    const TransactionalRigidScene& expectedScene)
{
  actual.step();
  expected.step();

  EXPECT_EQ(actual.getFrame(), expected.getFrame());
  EXPECT_DOUBLE_EQ(actual.getTime(), expected.getTime());
  EXPECT_TRUE(actualScene.configured.getTransform().isApprox(
      expectedScene.configured.getTransform(), 1e-12));
  EXPECT_TRUE(actualScene.plain.getTransform().isApprox(
      expectedScene.plain.getTransform(), 1e-12));
  EXPECT_TRUE(actualScene.configured.getLinearVelocity().isApprox(
      expectedScene.configured.getLinearVelocity(), 1e-12));
  EXPECT_TRUE(actualScene.plain.getLinearVelocity().isApprox(
      expectedScene.plain.getLinearVelocity(), 1e-12));
  EXPECT_TRUE(actualScene.configured.getAngularVelocity().isApprox(
      expectedScene.configured.getAngularVelocity(), 1e-12));
  EXPECT_TRUE(actualScene.plain.getAngularVelocity().isApprox(
      expectedScene.plain.getAngularVelocity(), 1e-12));
}

void expectSameNextWarmAvbdStep(
    sx::World& actual,
    const WarmAvbdTransactionalScene& actualScene,
    sx::World& expected,
    const WarmAvbdTransactionalScene& expectedScene)
{
  actual.step();
  expected.step();

  EXPECT_TRUE(actualScene.contact.getTransform().isApprox(
      expectedScene.contact.getTransform(), 1e-12));
  EXPECT_TRUE(actualScene.contact.getLinearVelocity().isApprox(
      expectedScene.contact.getLinearVelocity(), 1e-12));
  EXPECT_TRUE(actualScene.contact.getAngularVelocity().isApprox(
      expectedScene.contact.getAngularVelocity(), 1e-12));
  EXPECT_TRUE(actualScene.jointChild.getTransform().isApprox(
      expectedScene.jointChild.getTransform(), 1e-12));
  EXPECT_TRUE(actualScene.jointChild.getLinearVelocity().isApprox(
      expectedScene.jointChild.getLinearVelocity(), 1e-12));
  EXPECT_TRUE(actualScene.jointChild.getAngularVelocity().isApprox(
      expectedScene.jointChild.getAngularVelocity(), 1e-12));
}

} // namespace

TEST(ResolvedConfiguration, RecordsAvbdContactSubstitution)
{
  sx::World world;
  addBodyWithAvbdContactConfig(world);
  addCollisionBody(world, "plain_body", 0.5);
  world.enterSimulationMode();
  world.step();

  const auto& config = world.getResolvedConfiguration();
  EXPECT_TRUE(config.hasSubstitution());

  const auto* contact = findNote(config, "rigid-contact");
  ASSERT_NE(contact, nullptr);
  EXPECT_TRUE(contact->isSubstitution());
  EXPECT_EQ(contact->requested, "sequential-impulse");
  EXPECT_NE(contact->resolved.find("avbd"), std::string::npos);
}

TEST(ResolvedConfiguration, DisabledAvbdContactConfigDoesNotClaimAvbd)
{
  sx::World world;
  addBodyWithAvbdContactConfig(world, false);
  addCollisionBody(world, "plain_body", 0.5);
  world.enterSimulationMode();
  world.step();

  const auto& config = world.getResolvedConfiguration();
  EXPECT_FALSE(config.hasSubstitution());
  const auto* contact = findNote(config, "rigid-contact");
  ASSERT_NE(contact, nullptr);
  EXPECT_EQ(contact->requested, "sequential-impulse");
  EXPECT_EQ(contact->resolved, "sequential-impulse");
  EXPECT_NE(contact->reason.find("disabled"), std::string::npos);
}

TEST(ResolvedConfiguration, UncoveredAvbdContactPairDoesNotClaimAvbd)
{
  sx::World world;
  addBodyWithAvbdContactConfig(world);
  addCollisionBody(world, "plain_body_a", 0.5);
  addCollisionBody(world, "plain_body_b", 1.0);
  world.enterSimulationMode();
  world.step();

  const auto& config = world.getResolvedConfiguration();
  EXPECT_FALSE(config.hasSubstitution());
  const auto* contact = findNote(config, "rigid-contact");
  ASSERT_NE(contact, nullptr);
  EXPECT_EQ(contact->requested, "sequential-impulse");
  EXPECT_EQ(contact->resolved, "sequential-impulse");
  EXPECT_NE(contact->reason.find("does not cover"), std::string::npos);
}

TEST(
    ResolvedConfiguration,
    RecordsSequentialImpulsePairConstraintsWithoutSubstitution)
{
  sx::World world;
  auto parent = world.addRigidBody("parent");
  sx::RigidBodyOptions childOptions;
  childOptions.position = Eigen::Vector3d::UnitX();
  auto child = world.addRigidBody("child", childOptions);
  world.addJoint(
      parent,
      child,
      sx::JointSpec{.name = "fixed", .type = sx::JointType::Fixed});
  world.enterSimulationMode();

  const auto& config = world.getResolvedConfiguration();
  EXPECT_FALSE(config.hasSubstitution());

  const auto* pairConstraint = findNote(config, "rigid-pair-constraint");
  ASSERT_NE(pairConstraint, nullptr);
  EXPECT_FALSE(pairConstraint->isSubstitution());
  EXPECT_EQ(pairConstraint->requested, "sequential-impulse");
  EXPECT_EQ(pairConstraint->resolved, "sequential-impulse");
  EXPECT_NE(
      pairConstraint->reason.find("solver-owned sequential-impulse rows"),
      std::string::npos);
}

TEST(ResolvedConfiguration, RecordsIpcPairConstraintsWithoutAvbdSubstitution)
{
  sx::WorldOptions options;
  options.rigidBodySolver = sx::RigidBodySolver::Ipc;
  sx::World world(options);
  auto parent = world.addRigidBody("parent");
  sx::RigidBodyOptions childOptions;
  childOptions.position = Eigen::Vector3d::UnitX();
  auto child = world.addRigidBody("child", childOptions);
  world.addJoint(
      parent,
      child,
      sx::JointSpec{.name = "fixed", .type = sx::JointType::Fixed});
  world.enterSimulationMode();

  const auto& config = world.getResolvedConfiguration();
  EXPECT_FALSE(config.hasSubstitution());

  const auto* pairConstraint = findNote(config, "rigid-pair-constraint");
  ASSERT_NE(pairConstraint, nullptr);
  EXPECT_EQ(pairConstraint->requested, "ipc");
  EXPECT_EQ(pairConstraint->resolved, "ipc");
  EXPECT_NE(pairConstraint->reason.find("IPC articulation"), std::string::npos);

  const auto* iterations = findNote(config, "rigid-constraint-iterations");
  ASSERT_NE(iterations, nullptr);
  EXPECT_EQ(iterations->resolved, "not-applicable");
}

TEST(ResolvedConfiguration, RuntimeIterationMutationRefreshesReport)
{
  sx::World world;
  world.enterSimulationMode();

  world.setRigidConstraintOptions({.iterations = 3u});

  const auto* iterations = findNote(
      world.getResolvedConfiguration(), "rigid-constraint-iterations");
  ASSERT_NE(iterations, nullptr);
  EXPECT_EQ(iterations->requested, "3");
  EXPECT_EQ(iterations->resolved, "3");
}

TEST(
    ResolvedConfiguration,
    MixedSemiImplicitWorldMarksRigidIterationsNotApplicable)
{
  sx::World world;
  addMixedRigidAndMultibodyContactScene(world);
  world.enterSimulationMode();

  const auto* iterations = findNote(
      world.getResolvedConfiguration(), "rigid-constraint-iterations");
  ASSERT_NE(iterations, nullptr);
  EXPECT_EQ(iterations->requested, "not-applicable");
  EXPECT_EQ(iterations->resolved, "not-applicable");
  EXPECT_NE(iterations->reason.find("unified constraint"), std::string::npos);

  ASSERT_FALSE(world.collide().empty());
  EXPECT_NO_THROW(world.step());
  EXPECT_EQ(world.computeStepMetrics().lastStepIterations, 0u);

  EXPECT_THROW(
      world.setRigidConstraintOptions({.iterations = 3u}),
      sx::InvalidOperationException);
  EXPECT_EQ(world.getRigidConstraintOptions().iterations, 8u);

  iterations = findNote(
      world.getResolvedConfiguration(), "rigid-constraint-iterations");
  ASSERT_NE(iterations, nullptr);
  EXPECT_EQ(iterations->resolved, "not-applicable");
}

TEST(
    ResolvedConfiguration,
    MixedSemiImplicitWorldRejectsNonDefaultIterationsAtEntry)
{
  sx::WorldOptions options;
  options.rigidConstraintOptions.iterations = 3u;
  sx::World world(options);
  addMixedRigidAndMultibodyContactScene(world);

  EXPECT_THROW(world.enterSimulationMode(), sx::InvalidOperationException);
  EXPECT_FALSE(world.isSimulationMode());
}

TEST(
    ResolvedConfiguration,
    MixedWorldRejectsSemiImplicitTransitionWithNonDefaultIterations)
{
  sx::WorldOptions options;
  options.rigidConstraintOptions.iterations = 3u;
  options.multibodyOptions.integrationFamily
      = sx::MultibodyIntegrationFamily::Variational;
  sx::World world(options);
  addMixedRigidAndMultibodyContactScene(world);
  world.enterSimulationMode();

  const auto* iterations = findNote(
      world.getResolvedConfiguration(), "rigid-constraint-iterations");
  ASSERT_NE(iterations, nullptr);
  EXPECT_EQ(iterations->requested, "3");
  EXPECT_EQ(iterations->resolved, "3");

  EXPECT_THROW(
      world.setMultibodyOptions(
          {.integrationFamily = sx::MultibodyIntegrationFamily::SemiImplicit}),
      sx::InvalidOperationException);
  EXPECT_EQ(
      world.getMultibodyOptions().integrationFamily,
      sx::MultibodyIntegrationFamily::Variational);

  iterations = findNote(
      world.getResolvedConfiguration(), "rigid-constraint-iterations");
  ASSERT_NE(iterations, nullptr);
  EXPECT_EQ(iterations->resolved, "3");
  EXPECT_NO_THROW(world.step());
}

TEST(
    ResolvedConfiguration,
    RigidSolverSetterFailureRestoresConfigurationReportAndNextStep)
{
  sx::WorldOptions options;
  options.strictSolverResolution = true;
  options.rigidBodySolver = sx::RigidBodySolver::Avbd;
  sx::World world(options);
  sx::World control(options);
  const auto scene = addTransactionalRigidScene(world, true);
  const auto controlScene = addTransactionalRigidScene(control, true);
  world.enterSimulationMode();
  control.enterSimulationMode();
  const std::string resolvedBefore
      = world.getResolvedConfiguration().toSummaryText();

  EXPECT_THROW(
      world.setRigidBodySolver(sx::RigidBodySolver::SequentialImpulse),
      sx::InvalidArgumentException);

  EXPECT_EQ(world.getRigidBodySolver(), sx::RigidBodySolver::Avbd);
  EXPECT_EQ(world.getResolvedConfiguration().toSummaryText(), resolvedBefore);
  expectSameNextRigidStep(world, scene, control, controlScene);
}

TEST(
    ResolvedConfiguration,
    RigidSolverSetterFailurePreservesAvbdWarmStartContinuation)
{
  sx::WorldOptions options;
  options.gravity = Eigen::Vector3d::Zero();
  options.timeStep = 0.05;
  options.rigidBodySolver = sx::RigidBodySolver::Avbd;
  options.rigidConstraintOptions.iterations = 1u;
  options.strictSolverResolution = true;
  sx::World world(options);
  sx::World control(options);
  const auto scene = addWarmAvbdTransactionalScene(world);
  const auto controlScene = addWarmAvbdTransactionalScene(control);

  ASSERT_FALSE(world.collide().empty());
  ASSERT_FALSE(control.collide().empty());
  world.step();
  control.step();
  const std::string resolvedBefore
      = world.getResolvedConfiguration().toSummaryText();
  std::ostringstream beforeFailureSnapshot;
  EXPECT_THROW(
      world.saveBinary(beforeFailureSnapshot), sx::InvalidOperationException);

  auto& registry = sx::detail::registryOf(world);
  registry.get<sx::comps::DeformableVbdConfig>(scene.deformableEntity).enabled
      = true;
  EXPECT_THROW(
      world.setRigidBodySolver(sx::RigidBodySolver::Vbd),
      sx::InvalidArgumentException);

  EXPECT_EQ(world.getRigidBodySolver(), sx::RigidBodySolver::Avbd);
  EXPECT_EQ(world.getResolvedConfiguration().toSummaryText(), resolvedBefore);
  std::ostringstream afterFailureSnapshot;
  EXPECT_THROW(
      world.saveBinary(afterFailureSnapshot), sx::InvalidOperationException);

  registry.get<sx::comps::DeformableVbdConfig>(scene.deformableEntity).enabled
      = false;
  expectSameNextWarmAvbdStep(world, scene, control, controlScene);
}

TEST(
    ResolvedConfiguration,
    CommittedSequentialImpulseTransitionClearsOwnedAvbdContinuation)
{
  sx::WorldOptions options;
  options.gravity = Eigen::Vector3d::Zero();
  options.timeStep = 0.05;
  options.rigidBodySolver = sx::RigidBodySolver::Avbd;
  options.rigidConstraintOptions.iterations = 1u;
  sx::World world(options);
  static_cast<void>(addWarmAvbdHardConstraintScene(world));

  ASSERT_FALSE(world.collide().empty());
  world.step();
  std::ostringstream warmSnapshot;
  EXPECT_THROW(world.saveBinary(warmSnapshot), sx::InvalidOperationException);

  ASSERT_NO_THROW(
      world.setRigidBodySolver(sx::RigidBodySolver::SequentialImpulse));
  std::ostringstream sequentialImpulseSnapshot;
  EXPECT_NO_THROW(world.saveBinary(sequentialImpulseSnapshot));

  ASSERT_NO_THROW(world.setRigidBodySolver(sx::RigidBodySolver::Avbd));
  std::ostringstream roundTripSnapshot;
  EXPECT_NO_THROW(world.saveBinary(roundTripSnapshot));
}

TEST(ResolvedConfiguration, CommittedIpcTransitionClearsAllAvbdContinuation)
{
  sx::WorldOptions options;
  options.gravity = Eigen::Vector3d::Zero();
  options.timeStep = 0.05;
  options.rigidBodySolver = sx::RigidBodySolver::Avbd;
  sx::World world(options);
  static_cast<void>(addWarmAvbdHardConstraintScene(world));

  ASSERT_FALSE(world.collide().empty());
  world.step();
  std::ostringstream warmSnapshot;
  EXPECT_THROW(world.saveBinary(warmSnapshot), sx::InvalidOperationException);

  ASSERT_NO_THROW(world.setRigidBodySolver(sx::RigidBodySolver::Ipc));
  std::ostringstream ipcSnapshot;
  EXPECT_NO_THROW(world.saveBinary(ipcSnapshot));

  ASSERT_NO_THROW(world.setRigidBodySolver(sx::RigidBodySolver::Avbd));
  std::ostringstream roundTripSnapshot;
  EXPECT_NO_THROW(world.saveBinary(roundTripSnapshot));
}

TEST(
    ResolvedConfiguration,
    AvbdSequentialImpulseTransitionsPreserveDistanceSpringContinuation)
{
  sx::WorldOptions options;
  options.gravity = Eigen::Vector3d::Zero();
  options.timeStep = 0.05;
  options.rigidConstraintOptions.iterations = 1u;
  sx::World transitioned(options);
  sx::World uninterrupted(options);
  auto transitionedLink = addDistanceSpringScene(transitioned);
  auto uninterruptedLink = addDistanceSpringScene(uninterrupted);

  transitioned.step();
  uninterrupted.step();
  std::ostringstream warmSnapshot;
  EXPECT_THROW(
      transitioned.saveBinary(warmSnapshot), sx::InvalidOperationException);

  ASSERT_NO_THROW(transitioned.setRigidBodySolver(sx::RigidBodySolver::Avbd));
  ASSERT_NO_THROW(
      transitioned.setRigidBodySolver(sx::RigidBodySolver::SequentialImpulse));
  std::ostringstream transitionedSnapshot;
  EXPECT_THROW(
      transitioned.saveBinary(transitionedSnapshot),
      sx::InvalidOperationException);

  transitioned.step();
  uninterrupted.step();
  EXPECT_TRUE(transitionedLink.getTransform().matrix().isApprox(
      uninterruptedLink.getTransform().matrix(), 1e-12));
  EXPECT_TRUE(transitionedLink.getLinearVelocity().isApprox(
      uninterruptedLink.getLinearVelocity(), 1e-12));
  EXPECT_TRUE(transitionedLink.getAngularVelocity().isApprox(
      uninterruptedLink.getAngularVelocity(), 1e-12));
}

TEST(
    ResolvedConfiguration,
    VbdTransitionClearsDistanceSpringContinuationBeforeReturningToAvbd)
{
  sx::WorldOptions options;
  options.gravity = Eigen::Vector3d::Zero();
  options.timeStep = 0.05;
  options.rigidBodySolver = sx::RigidBodySolver::Avbd;
  sx::World transitioned(options);
  auto transitionedLink = addDistanceSpringScene(transitioned);

  transitioned.step();
  std::ostringstream warmSnapshot;
  EXPECT_THROW(
      transitioned.saveBinary(warmSnapshot), sx::InvalidOperationException);

  ASSERT_NO_THROW(transitioned.setRigidBodySolver(sx::RigidBodySolver::Vbd));
  std::ostringstream clearedSnapshot;
  ASSERT_NO_THROW(transitioned.saveBinary(clearedSnapshot));

  sx::World cold;
  std::istringstream coldInput(clearedSnapshot.str());
  ASSERT_NO_THROW(cold.loadBinary(coldInput));
  auto coldLink = cold.getRigidBody("spring_link");
  ASSERT_TRUE(coldLink.has_value());

  ASSERT_NO_THROW(transitioned.setRigidBodySolver(sx::RigidBodySolver::Avbd));
  ASSERT_NO_THROW(cold.setRigidBodySolver(sx::RigidBodySolver::Avbd));
  transitioned.step();
  cold.step();
  EXPECT_TRUE(transitionedLink.getTransform().matrix().isApprox(
      coldLink->getTransform().matrix(), 1e-12));
  EXPECT_TRUE(transitionedLink.getLinearVelocity().isApprox(
      coldLink->getLinearVelocity(), 1e-12));
  EXPECT_TRUE(transitionedLink.getAngularVelocity().isApprox(
      coldLink->getAngularVelocity(), 1e-12));
}

TEST(
    ResolvedConfiguration,
    IpcTransitionClearsOrphanedDistanceSpringContinuationBeforeReturningToAvbd)
{
  namespace dvbd = sx::detail::deformable_vbd;

  sx::WorldOptions options;
  options.gravity = Eigen::Vector3d::Zero();
  options.timeStep = 0.05;
  options.rigidBodySolver = sx::RigidBodySolver::Avbd;
  sx::World transitioned(options);
  auto transitionedLink = addDistanceSpringScene(transitioned);

  transitioned.step();
  std::ostringstream warmSnapshot;
  EXPECT_THROW(
      transitioned.saveBinary(warmSnapshot), sx::InvalidOperationException);

  // IPC correctly rejects an actively configured distance spring. Remove its
  // registry entity after warming so that only the solver-owned continuation
  // remains, then verify that the committed IPC transition invalidates it.
  auto& transitionedRegistry = sx::detail::registryOf(transitioned);
  const auto springView
      = transitionedRegistry.view<dvbd::AvbdRigidWorldDistanceSpringConfig>();
  ASSERT_NE(springView.begin(), springView.end());
  const entt::entity springEntity = *springView.begin();
  const auto springTemplate
      = springView.get<dvbd::AvbdRigidWorldDistanceSpringConfig>(springEntity);
  transitionedRegistry.destroy(springEntity);
  std::ostringstream orphanedWarmSnapshot;
  EXPECT_THROW(
      transitioned.saveBinary(orphanedWarmSnapshot),
      sx::InvalidOperationException);

  ASSERT_NO_THROW(transitioned.setRigidBodySolver(sx::RigidBodySolver::Ipc));
  std::ostringstream clearedSnapshot;
  ASSERT_NO_THROW(transitioned.saveBinary(clearedSnapshot));

  sx::World cold;
  std::istringstream coldInput(clearedSnapshot.str());
  ASSERT_NO_THROW(cold.loadBinary(coldInput));
  auto coldLink = cold.getRigidBody("spring_link");
  ASSERT_TRUE(coldLink.has_value());

  ASSERT_NO_THROW(transitioned.setRigidBodySolver(sx::RigidBodySolver::Avbd));
  ASSERT_NO_THROW(cold.setRigidBodySolver(sx::RigidBodySolver::Avbd));
  const auto reinstallSpring = [&springTemplate](sx::World& target) {
    auto base = target.getRigidBody("spring_base");
    auto link = target.getRigidBody("spring_link");
    ASSERT_TRUE(base.has_value());
    ASSERT_TRUE(link.has_value());
    if (!base.has_value() || !link.has_value()) {
      return;
    }

    auto config = springTemplate;
    config.bodyA = sx::detail::toRegistryEntity(base->getEntity());
    config.bodyB = sx::detail::toRegistryEntity(link->getEntity());
    auto& registry = sx::detail::registryOf(target);
    registry.emplace<dvbd::AvbdRigidWorldDistanceSpringConfig>(
        registry.create(), config);
  };
  reinstallSpring(transitioned);
  reinstallSpring(cold);

  transitioned.step();
  cold.step();
  EXPECT_TRUE(transitionedLink.getTransform().matrix().isApprox(
      coldLink->getTransform().matrix(), 1e-12));
  EXPECT_TRUE(transitionedLink.getLinearVelocity().isApprox(
      coldLink->getLinearVelocity(), 1e-12));
  EXPECT_TRUE(transitionedLink.getAngularVelocity().isApprox(
      coldLink->getAngularVelocity(), 1e-12));
}

TEST(
    ResolvedConfiguration,
    RigidConstraintSetterFailureRestoresConfigurationReportAndNextStep)
{
  sx::WorldOptions options;
  options.strictSolverResolution = true;
  sx::World world(options);
  sx::World control(options);
  const auto scene = addTransactionalRigidScene(world, false);
  const auto controlScene = addTransactionalRigidScene(control, false);
  world.enterSimulationMode();
  control.enterSimulationMode();
  const sx::RigidConstraintOptions optionsBefore
      = world.getRigidConstraintOptions();
  const std::string resolvedBefore
      = world.getResolvedConfiguration().toSummaryText();

  setCompatibilityAvbdEnabled(world, scene.configured, true);
  EXPECT_THROW(
      world.setRigidConstraintOptions({.iterations = 3u}),
      sx::InvalidArgumentException);

  EXPECT_EQ(
      world.getRigidConstraintOptions().iterations, optionsBefore.iterations);
  EXPECT_EQ(world.getResolvedConfiguration().toSummaryText(), resolvedBefore);

  setCompatibilityAvbdEnabled(world, scene.configured, false);
  expectSameNextRigidStep(world, scene, control, controlScene);
  EXPECT_EQ(
      world.computeStepMetrics().lastStepIterations,
      control.computeStepMetrics().lastStepIterations);
}

TEST(
    ResolvedConfiguration,
    ContactMethodSetterFailureRestoresConfigurationReportAndNextStep)
{
  sx::WorldOptions options;
  options.strictSolverResolution = true;
  sx::World world(options);
  sx::World control(options);
  const auto scene = addTransactionalRigidScene(world, false);
  const auto controlScene = addTransactionalRigidScene(control, false);
  world.enterSimulationMode();
  control.enterSimulationMode();
  const std::string resolvedBefore
      = world.getResolvedConfiguration().toSummaryText();

  // A late private compatibility opt-in makes the setter's full preparation
  // fail strict resolution after its ordinary argument checks have passed.
  setCompatibilityAvbdEnabled(world, scene.configured, true);
  EXPECT_THROW(
      world.setContactSolverMethod(sx::ContactSolverMethod::BoxedLcp),
      sx::InvalidArgumentException);

  EXPECT_EQ(
      world.getContactSolverMethod(),
      sx::ContactSolverMethod::SequentialImpulse);
  EXPECT_EQ(world.getResolvedConfiguration().toSummaryText(), resolvedBefore);

  setCompatibilityAvbdEnabled(world, scene.configured, false);
  expectSameNextRigidStep(world, scene, control, controlScene);
}

TEST(
    ResolvedConfiguration,
    ComputePolicySetterFailureRestoresConfigurationReportAndNextStep)
{
  sx::WorldOptions options;
  options.strictSolverResolution = true;
  sx::World world(options);
  sx::World control(options);
  const auto scene = addTransactionalRigidScene(world, false);
  const auto controlScene = addTransactionalRigidScene(control, false);
  world.enterSimulationMode();
  control.enterSimulationMode();
  const std::string resolvedBefore
      = world.getResolvedConfiguration().toSummaryText();

  EXPECT_THROW(
      world.setComputeAcceleratorPolicy(
          sx::ComputeAcceleratorPolicy::PreferAccelerated),
      sx::InvalidArgumentException);

  EXPECT_EQ(
      world.getComputeAcceleratorPolicy(),
      sx::ComputeAcceleratorPolicy::CpuOnly);
  EXPECT_EQ(world.getResolvedConfiguration().toSummaryText(), resolvedBefore);
  expectSameNextRigidStep(world, scene, control, controlScene);
}

TEST(
    ResolvedConfiguration,
    MultibodyOptionsSetterFailureRestoresConfigurationReportAndNextStep)
{
  sx::WorldOptions options;
  options.strictSolverResolution = true;
  sx::World world(options);
  sx::World control(options);
  const auto scene = addTransactionalRigidScene(world, false);
  const auto controlScene = addTransactionalRigidScene(control, false);
  world.enterSimulationMode();
  control.enterSimulationMode();
  const sx::MultibodyOptions optionsBefore = world.getMultibodyOptions();
  const std::string resolvedBefore
      = world.getResolvedConfiguration().toSummaryText();

  // Exercise the preparation failure after all public option validation by
  // making an independent private substitution visible at this rebake.
  setCompatibilityAvbdEnabled(world, scene.configured, true);
  EXPECT_THROW(
      world.setMultibodyOptions(
          {.integrationFamily = sx::MultibodyIntegrationFamily::Variational,
           .variationalMaxIterations = 7u,
           .variationalTolerance = 1e-7}),
      sx::InvalidArgumentException);

  const sx::MultibodyOptions optionsAfter = world.getMultibodyOptions();
  EXPECT_EQ(optionsAfter.integrationFamily, optionsBefore.integrationFamily);
  EXPECT_EQ(
      optionsAfter.variationalMaxIterations,
      optionsBefore.variationalMaxIterations);
  EXPECT_DOUBLE_EQ(
      optionsAfter.variationalTolerance, optionsBefore.variationalTolerance);
  EXPECT_EQ(world.getResolvedConfiguration().toSummaryText(), resolvedBefore);

  setCompatibilityAvbdEnabled(world, scene.configured, false);
  expectSameNextRigidStep(world, scene, control, controlScene);
}

TEST(
    ResolvedConfiguration,
    MultibodySetterLateStrictFailureRestoresPersistentComponentsAndBinaryState)
{
  sx::WorldOptions options;
  options.gravity = Eigen::Vector3d::Zero();
  options.strictSolverResolution = true;
  sx::World world(options);
  sx::World control(options);
  const PersistentEntryScene scene = addPersistentEntryScene(world, 64u);
  const PersistentEntryScene controlScene
      = addPersistentEntryScene(control, 64u);
  const TransactionalRigidScene rigidScene
      = addTransactionalRigidScene(world, false);
  const TransactionalRigidScene controlRigidScene
      = addTransactionalRigidScene(control, false);
  world.enterSimulationMode();
  control.enterSimulationMode();

  auto installPersistentBoundary
      = [](sx::World& target, const PersistentEntryScene& targetScene) {
          auto& registry = sx::detail::registryOf(target);
          registry.remove<sx::compute::MultibodyVariationalState>(
              targetScene.multibodyEntity);
          registry.remove<sx::comps::DeformableContactConfig>(
              targetScene.deformableEntity);

          using DualState = sx::comps::VariationalContactDualState;
          auto& worldAllocator = target.getMemoryManager().getFreeAllocator();
          DualState dual{
              DualState::DualVector{
                  dart::common::StlAllocator<double>{worldAllocator}},
              29u};
          dual.duals = {7.0, 8.0};
          registry.emplace_or_replace<DualState>(
              targetScene.multibodyEntity, std::move(dual));
        };
  installPersistentBoundary(world, scene);
  installPersistentBoundary(control, controlScene);
  auto& registry = sx::detail::registryOf(world);
  ASSERT_FALSE(registry.all_of<sx::compute::MultibodyVariationalState>(
      scene.multibodyEntity));
  ASSERT_FALSE(registry.all_of<sx::comps::DeformableContactConfig>(
      scene.deformableEntity));
  setCompatibilityAvbdEnabled(world, rigidScene.configured, true);
  std::ostringstream beforeFailure;
  ASSERT_NO_THROW(world.saveBinary(beforeFailure));

  EXPECT_THROW(
      world.setMultibodyOptions(
          {.integrationFamily = sx::MultibodyIntegrationFamily::Variational}),
      sx::InvalidArgumentException);

  EXPECT_EQ(
      world.getMultibodyOptions().integrationFamily,
      sx::MultibodyIntegrationFamily::SemiImplicit);
  EXPECT_FALSE(registry.all_of<sx::compute::MultibodyVariationalState>(
      scene.multibodyEntity));
  EXPECT_FALSE(registry.all_of<sx::comps::DeformableContactConfig>(
      scene.deformableEntity));
  const auto& restoredDual
      = registry.get<sx::comps::VariationalContactDualState>(
          scene.multibodyEntity);
  EXPECT_EQ(restoredDual.stepsSinceDualUpdate, 29u);
  ASSERT_EQ(restoredDual.duals.size(), 2u);
  EXPECT_DOUBLE_EQ(restoredDual.duals[0], 7.0);
  EXPECT_DOUBLE_EQ(restoredDual.duals[1], 8.0);
  std::ostringstream afterFailure;
  ASSERT_NO_THROW(world.saveBinary(afterFailure));
  EXPECT_EQ(afterFailure.str(), beforeFailure.str());

  setCompatibilityAvbdEnabled(world, rigidScene.configured, false);
  ASSERT_NO_THROW(world.setMultibodyOptions(
      {.integrationFamily = sx::MultibodyIntegrationFamily::Variational}));
  ASSERT_NO_THROW(control.setMultibodyOptions(
      {.integrationFamily = sx::MultibodyIntegrationFamily::Variational}));
  std::ostringstream retriedConfiguration;
  std::ostringstream controlConfiguration;
  ASSERT_NO_THROW(world.saveBinary(retriedConfiguration));
  ASSERT_NO_THROW(control.saveBinary(controlConfiguration));
  EXPECT_EQ(retriedConfiguration.str(), controlConfiguration.str());

  ASSERT_NO_THROW(world.step());
  ASSERT_NO_THROW(control.step());
  std::ostringstream retriedStep;
  std::ostringstream controlStep;
  ASSERT_NO_THROW(world.saveBinary(retriedStep));
  ASSERT_NO_THROW(control.saveBinary(controlStep));
  EXPECT_EQ(retriedStep.str(), controlStep.str());
  EXPECT_TRUE(rigidScene.configured.getTransform().isApprox(
      controlRigidScene.configured.getTransform(), 0.0));
}

#ifdef DART_HAS_DIFF
TEST(
    ResolvedConfiguration,
    MultibodySetterFailureRestoresPersistentComponentsAndDerivatives)
{
  sx::WorldOptions options;
  options.differentiable = true;
  options.gravity = Eigen::Vector3d::Zero();
  options.strictSolverResolution = true;
  sx::World world(options);
  sx::World control(options);
  const PersistentEntryScene persistentScene
      = addPersistentEntryScene(world, 16u);
  const PersistentEntryScene controlPersistentScene
      = addPersistentEntryScene(control, 16u);
  const TransactionalRigidScene rigidScene
      = addTransactionalRigidScene(world, false);
  const TransactionalRigidScene controlRigidScene
      = addTransactionalRigidScene(control, false);
  world.enterSimulationMode();
  control.enterSimulationMode();
  world.step();
  control.step();

  const sx::StepDerivatives derivativesBefore = world.getStepDerivatives();
  auto installPersistentBoundary = [](sx::World& target,
                                      const PersistentEntryScene& scene) {
    auto& registry = sx::detail::registryOf(target);
    registry.remove<sx::compute::MultibodyVariationalState>(
        scene.multibodyEntity);
    registry.remove<sx::comps::DeformableContactConfig>(scene.deformableEntity);

    using DualState = sx::comps::VariationalContactDualState;
    auto& allocator = target.getMemoryManager().getFreeAllocator();
    DualState dual{
        DualState::DualVector{dart::common::StlAllocator<double>{allocator}},
        23u};
    dual.duals = {5.0, 6.0};
    registry.emplace_or_replace<DualState>(
        scene.multibodyEntity, std::move(dual));
  };
  installPersistentBoundary(world, persistentScene);
  installPersistentBoundary(control, controlPersistentScene);
  setCompatibilityAvbdEnabled(world, rigidScene.configured, true);

  std::ostringstream beforeFailure;
  ASSERT_NO_THROW(world.saveBinary(beforeFailure));
  EXPECT_THROW(
      world.setMultibodyOptions(
          {.integrationFamily = sx::MultibodyIntegrationFamily::Variational}),
      sx::InvalidArgumentException);

  EXPECT_EQ(
      world.getMultibodyOptions().integrationFamily,
      sx::MultibodyIntegrationFamily::SemiImplicit);
  auto& registry = sx::detail::registryOf(world);
  EXPECT_FALSE(registry.all_of<sx::compute::MultibodyVariationalState>(
      persistentScene.multibodyEntity));
  EXPECT_FALSE(registry.all_of<sx::comps::DeformableContactConfig>(
      persistentScene.deformableEntity));
  const auto& restoredDual
      = registry.get<sx::comps::VariationalContactDualState>(
          persistentScene.multibodyEntity);
  EXPECT_EQ(restoredDual.stepsSinceDualUpdate, 23u);
  ASSERT_EQ(restoredDual.duals.size(), 2u);
  EXPECT_DOUBLE_EQ(restoredDual.duals[0], 5.0);
  EXPECT_DOUBLE_EQ(restoredDual.duals[1], 6.0);

  const auto& derivativesAfter = world.getStepDerivatives();
  EXPECT_TRUE(derivativesAfter.stateJacobian.isApprox(
      derivativesBefore.stateJacobian, 0.0));
  EXPECT_TRUE(derivativesAfter.controlJacobian.isApprox(
      derivativesBefore.controlJacobian, 0.0));
  EXPECT_TRUE(derivativesAfter.parameterJacobian.isApprox(
      derivativesBefore.parameterJacobian, 0.0));
  std::ostringstream afterFailure;
  ASSERT_NO_THROW(world.saveBinary(afterFailure));
  EXPECT_EQ(afterFailure.str(), beforeFailure.str());

  setCompatibilityAvbdEnabled(world, rigidScene.configured, false);
  ASSERT_NO_THROW(world.setMultibodyOptions(
      {.integrationFamily = sx::MultibodyIntegrationFamily::Variational}));
  ASSERT_NO_THROW(control.setMultibodyOptions(
      {.integrationFamily = sx::MultibodyIntegrationFamily::Variational}));
  world.step();
  control.step();
  std::ostringstream retriedStep;
  std::ostringstream controlStep;
  ASSERT_NO_THROW(world.saveBinary(retriedStep));
  ASSERT_NO_THROW(control.saveBinary(controlStep));
  EXPECT_EQ(retriedStep.str(), controlStep.str());
  EXPECT_TRUE(rigidScene.configured.getTransform().isApprox(
      controlRigidScene.configured.getTransform(), 0.0));
}
#endif

TEST(ResolvedConfiguration, StrictResolutionRejectsSubstitution)
{
  sx::WorldOptions options;
  options.strictSolverResolution = true;
  sx::World world(options);
  addBodyWithAvbdContactConfig(world);
  addCollisionBody(world, "plain_body", 0.5);

  const double time = world.getTime();
  const std::size_t frame = world.getFrame();
  EXPECT_THROW(world.enterSimulationMode(), sx::InvalidArgumentException);
  EXPECT_FALSE(world.isSimulationMode());
  EXPECT_TRUE(world.getResolvedConfiguration().isEmpty());
  EXPECT_DOUBLE_EQ(world.getTime(), time);
  EXPECT_EQ(world.getFrame(), frame);

  EXPECT_THROW(world.step(), sx::InvalidArgumentException);
  EXPECT_FALSE(world.isSimulationMode());
  EXPECT_TRUE(world.getResolvedConfiguration().isEmpty());
  EXPECT_DOUBLE_EQ(world.getTime(), time);
  EXPECT_EQ(world.getFrame(), frame);

  EXPECT_THROW(world.enterSimulationMode(), sx::InvalidArgumentException);
  EXPECT_FALSE(world.isSimulationMode());
  EXPECT_TRUE(world.getResolvedConfiguration().isEmpty());
  EXPECT_DOUBLE_EQ(world.getTime(), time);
  EXPECT_EQ(world.getFrame(), frame);
}

TEST(
    ResolvedConfiguration,
    StrictEntryFailureRestoresCollisionCacheAndSceneConfiguration)
{
  sx::WorldOptions options;
  options.strictSolverResolution = true;
  sx::World world(options);
  auto configured = addBodyWithAvbdContactConfig(world);
  auto plain = addCollisionBody(world, "plain_body", 0.5);
  auto& registry = sx::detail::registryOf(world);
  const entt::entity configuredEntity
      = sx::detail::toRegistryEntity(configured.getEntity());
  const sx::comps::RigidAvbdContactConfig configBefore
      = registry.get<sx::comps::RigidAvbdContactConfig>(configuredEntity);
  const Eigen::Isometry3d configuredTransformBefore = configured.getTransform();
  const Eigen::Isometry3d plainTransformBefore = plain.getTransform();
  const std::string resolvedBefore
      = world.getResolvedConfiguration().toSummaryText();
  ASSERT_EQ(world.getRigidCollisionCandidatePairCapacity(), 0u);
  ASSERT_EQ(world.getRigidCollisionContactCapacity(), 0u);

  EXPECT_THROW(world.enterSimulationMode(), sx::InvalidArgumentException);

  EXPECT_FALSE(world.isSimulationMode());
  EXPECT_EQ(world.getRigidCollisionCandidatePairCapacity(), 0u);
  EXPECT_EQ(world.getRigidCollisionContactCapacity(), 0u);
  EXPECT_EQ(world.getResolvedConfiguration().toSummaryText(), resolvedBefore);
  EXPECT_TRUE(
      configured.getTransform().isApprox(configuredTransformBefore, 0.0));
  EXPECT_TRUE(plain.getTransform().isApprox(plainTransformBefore, 0.0));
  const auto& configAfter
      = registry.get<sx::comps::RigidAvbdContactConfig>(configuredEntity);
  EXPECT_EQ(configAfter.enabled, configBefore.enabled);
  EXPECT_DOUBLE_EQ(configAfter.startStiffness, configBefore.startStiffness);
  EXPECT_DOUBLE_EQ(configAfter.alpha, configBefore.alpha);
  EXPECT_DOUBLE_EQ(configAfter.beta, configBefore.beta);
  EXPECT_DOUBLE_EQ(configAfter.gamma, configBefore.gamma);
  EXPECT_DOUBLE_EQ(configAfter.maxStiffness, configBefore.maxStiffness);

  // A later explicit collision query is still usable and is the first action
  // that may resolve the automatic cache capacities after the rejected entry.
  EXPECT_FALSE(world.collide().empty());
  EXPECT_GT(world.getRigidCollisionCandidatePairCapacity(), 0u);
  EXPECT_GT(world.getRigidCollisionContactCapacity(), 0u);
}

TEST(
    ResolvedConfiguration,
    StrictEntryFailureDoesNotCaptureArticulatedPointJointReferencePose)
{
  namespace dvbd = sx::detail::deformable_vbd;

  sx::WorldOptions options;
  options.computeAcceleratorPolicy
      = sx::ComputeAcceleratorPolicy::PreferAccelerated;
  options.strictSolverResolution = true;
  options.multibodyOptions.integrationFamily
      = sx::MultibodyIntegrationFamily::Variational;
  sx::World world(options);

  auto robot = world.addMultibody("robot");
  auto base = robot.addLink("base");
  sx::JointSpec treeJointSpec;
  treeJointSpec.name = "slider";
  treeJointSpec.type = sx::JointType::Prismatic;
  treeJointSpec.axis = Eigen::Vector3d::UnitX();
  auto child = robot.addLink("child", base, treeJointSpec);
  auto treeJoint = child.getParentJoint();
  treeJoint.setPosition(Eigen::VectorXd::Constant(1, 0.25));

  auto pointJoint = world.addJoint(
      base,
      child,
      sx::JointSpec{.name = "closure", .type = sx::JointType::Fixed});
  auto& registry = sx::detail::registryOf(world);
  const entt::entity pointJointEntity
      = sx::detail::toRegistryEntity(pointJoint.getEntity());
  const sx::comps::JointModel jointModelBefore
      = registry.get<sx::comps::JointModel>(pointJointEntity);
  ASSERT_FALSE(jointModelBefore.hasRigidBodyPairConstraintGeometry);
  ASSERT_FALSE(
      registry.all_of<dvbd::AvbdRigidWorldPointJointConfig>(pointJointEntity));
  ASSERT_FALSE(
      registry.all_of<sx::comps::AvbdJointStiffness>(pointJointEntity));
  const std::string resolvedBefore
      = world.getResolvedConfiguration().toSummaryText();

  EXPECT_THROW(world.enterSimulationMode(), sx::InvalidArgumentException);

  EXPECT_FALSE(world.isSimulationMode());
  EXPECT_EQ(world.getResolvedConfiguration().toSummaryText(), resolvedBefore);
  ASSERT_FALSE(
      registry.all_of<dvbd::AvbdRigidWorldPointJointConfig>(pointJointEntity));
  ASSERT_FALSE(
      registry.all_of<sx::comps::AvbdJointStiffness>(pointJointEntity));
  const auto& jointModelAfterFailure
      = registry.get<sx::comps::JointModel>(pointJointEntity);
  EXPECT_EQ(
      jointModelAfterFailure.hasRigidBodyPairConstraintGeometry,
      jointModelBefore.hasRigidBodyPairConstraintGeometry);
  EXPECT_TRUE(
      jointModelAfterFailure.rigidBodyPairConstraintLocalAnchorParent.isApprox(
          jointModelBefore.rigidBodyPairConstraintLocalAnchorParent, 0.0));
  EXPECT_TRUE(
      jointModelAfterFailure.rigidBodyPairConstraintLocalAnchorChild.isApprox(
          jointModelBefore.rigidBodyPairConstraintLocalAnchorChild, 0.0));
  EXPECT_TRUE(
      jointModelAfterFailure.rigidBodyPairConstraintTargetRelativeOrientation
          .coeffs()
          .isApprox(
              jointModelBefore.rigidBodyPairConstraintTargetRelativeOrientation
                  .coeffs(),
              0.0));

  world.setComputeAcceleratorPolicy(sx::ComputeAcceleratorPolicy::CpuOnly);
  treeJoint.setPosition(Eigen::VectorXd::Constant(1, 1.25));
  ASSERT_NO_THROW(world.enterSimulationMode());

  ASSERT_TRUE(
      registry.all_of<dvbd::AvbdRigidWorldPointJointConfig>(pointJointEntity));
  ASSERT_TRUE(registry.all_of<sx::comps::AvbdJointStiffness>(pointJointEntity));
  const auto& config
      = registry.get<dvbd::AvbdRigidWorldPointJointConfig>(pointJointEntity);
  EXPECT_TRUE(
      config.localAnchorA.isApprox(1.25 * Eigen::Vector3d::UnitX(), 1e-12));
  EXPECT_TRUE(config.localAnchorB.isZero(1e-12));
  EXPECT_TRUE(
      (base.getWorldTransform() * config.localAnchorA)
          .isApprox(child.getWorldTransform() * config.localAnchorB, 1e-12));
}

#ifdef DART_HAS_DIFF
TEST(
    ResolvedConfiguration,
    LateEntryFailureRestoresExistingAndCreatedPersistentComponents)
{
  sx::WorldOptions options;
  options.differentiable = true;
  options.gravity = Eigen::Vector3d::Zero();
  options.multibodyOptions.integrationFamily
      = sx::MultibodyIntegrationFamily::Variational;
  sx::World world(options);
  const PersistentEntryScene scene = addPersistentEntryScene(world, 64u);
  scene.actuatorJoint.setActuatorType(sx::ActuatorType::Locked);
  auto& registry = sx::detail::registryOf(world);
  auto& worldAllocator = world.getMemoryManager().getFreeAllocator();

  using DualState = sx::comps::VariationalContactDualState;
  DualState priorDual{
      DualState::DualVector{dart::common::StlAllocator<double>{worldAllocator}},
      17u};
  priorDual.duals = {3.0, 4.0};
  registry.emplace<DualState>(scene.multibodyEntity, std::move(priorDual));
  ASSERT_FALSE(registry.all_of<sx::compute::MultibodyVariationalState>(
      scene.multibodyEntity));
  ASSERT_FALSE(registry.all_of<sx::comps::DeformableContactConfig>(
      scene.deformableEntity));
  std::ostringstream beforeEntry;
  ASSERT_NO_THROW(world.saveBinary(beforeEntry));

  EXPECT_THROW(world.enterSimulationMode(), sx::NotImplementedException);

  EXPECT_FALSE(world.isSimulationMode());
  EXPECT_FALSE(registry.all_of<sx::compute::MultibodyVariationalState>(
      scene.multibodyEntity));
  EXPECT_FALSE(registry.all_of<sx::comps::DeformableContactConfig>(
      scene.deformableEntity));
  const auto& restoredDual = registry.get<DualState>(scene.multibodyEntity);
  EXPECT_EQ(restoredDual.stepsSinceDualUpdate, 17u);
  ASSERT_EQ(restoredDual.duals.size(), 2u);
  EXPECT_DOUBLE_EQ(restoredDual.duals[0], 3.0);
  EXPECT_DOUBLE_EQ(restoredDual.duals[1], 4.0);
  std::ostringstream afterFailure;
  ASSERT_NO_THROW(world.saveBinary(afterFailure));
  EXPECT_EQ(afterFailure.str(), beforeEntry.str());

  scene.actuatorJoint.setActuatorType(sx::ActuatorType::Force);
  EXPECT_NO_THROW(world.enterSimulationMode());
}
#endif

TEST(
    ResolvedConfiguration,
    EntryAllocationFailureRestoresPersistentComponentsAndBinaryState)
{
  PredicateFailOnceMemoryAllocator allocator;
  sx::WorldOptions options;
  options.baseAllocator = &allocator;
  options.freeListInitialAllocation = 4096u;
  options.frameScratchInitialCapacity = 4096u;
  options.gravity = Eigen::Vector3d::Zero();
  options.multibodyOptions.integrationFamily
      = sx::MultibodyIntegrationFamily::Variational;
  sx::World world(options);
  const PersistentEntryScene scene = addPersistentEntryScene(world);
  auto& registry = sx::detail::registryOf(world);

  ASSERT_FALSE(registry.all_of<sx::compute::MultibodyVariationalState>(
      scene.multibodyEntity));
  ASSERT_FALSE(registry.all_of<sx::comps::VariationalContactDualState>(
      scene.multibodyEntity));
  ASSERT_FALSE(registry.all_of<sx::comps::DeformableContactConfig>(
      scene.deformableEntity));
  std::ostringstream beforeEntry;
  ASSERT_NO_THROW(world.saveBinary(beforeEntry));

  // Entry reaches its base allocator exactly once, while
  // `reserveRegistryStorageForSimulation()` grows the world free list for the
  // multibody dynamics scratch. Everything the bake needs after that is served
  // from the block the free list already owns, so a trigger keyed on a lazily
  // created persistent component never fires: `DeformableContactConfig` is
  // emplaced later, in the deformable stage's prepare, and no base-allocator
  // request follows it. Fail the first request instead.
  //
  // That request is the earliest point entry can fail, and entry is built so
  // that "all allocation happens while the live World is still untouched": the
  // persistent components below are materialized after it, so this case asserts
  // that a mid-entry allocation failure leaves nothing behind, not that a
  // populated rollback restores correctly. The rollback-with-work case belongs
  // to `LateEntryFailureRestoresExistingAndCreatedPersistentComponents`, which
  // is compiled only under `DART_HAS_DIFF`.
  allocator.failOnceWhen([] { return true; });
  EXPECT_THROW(world.enterSimulationMode(), std::bad_alloc);
  ASSERT_EQ(allocator.failureCount(), 1u);
  EXPECT_FALSE(world.isSimulationMode());
  EXPECT_FALSE(registry.all_of<sx::compute::MultibodyVariationalState>(
      scene.multibodyEntity));
  EXPECT_FALSE(registry.all_of<sx::comps::VariationalContactDualState>(
      scene.multibodyEntity));
  EXPECT_FALSE(registry.all_of<sx::comps::DeformableContactConfig>(
      scene.deformableEntity));

  std::ostringstream afterFailure;
  ASSERT_NO_THROW(world.saveBinary(afterFailure));
  EXPECT_EQ(afterFailure.str(), beforeEntry.str());

  sx::World control;
  control.setGravity(Eigen::Vector3d::Zero());
  control.setMultibodyOptions(
      {.integrationFamily = sx::MultibodyIntegrationFamily::Variational});
  static_cast<void>(addPersistentEntryScene(control));

  ASSERT_NO_THROW(world.enterSimulationMode());
  ASSERT_NO_THROW(control.enterSimulationMode());
  std::ostringstream retriedEntry;
  std::ostringstream controlEntry;
  ASSERT_NO_THROW(world.saveBinary(retriedEntry));
  ASSERT_NO_THROW(control.saveBinary(controlEntry));
  EXPECT_EQ(retriedEntry.str(), controlEntry.str());

  ASSERT_NO_THROW(world.step());
  ASSERT_NO_THROW(control.step());
  std::ostringstream retriedStep;
  std::ostringstream controlStep;
  ASSERT_NO_THROW(world.saveBinary(retriedStep));
  ASSERT_NO_THROW(control.saveBinary(controlStep));
  EXPECT_EQ(retriedStep.str(), controlStep.str());
}
