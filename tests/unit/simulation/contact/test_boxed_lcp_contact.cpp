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

// Boxed-LCP rigid-body contact parity test (PLAN-080 WS4 / friction
// continuation). The opt-in BoxedLcp contact path must reproduce the
// SequentialImpulse path's resting behavior for a frictionless (normal-only)
// rigid-body scene: a body dropped onto a static ground comes to rest at the
// same height with a near-zero normal velocity and no deep penetration. A
// direct equal-mass head-on collision additionally checks momentum conservation
// against the closed-form result. Coulomb friction is exercised by a sliding
// box that decelerates consistently with the SequentialImpulse path and a small
// tangential push that static friction holds in place.

#include "tests/common/lcpsolver/lcp_test_harness.hpp"

#include <dart/simulation/body/collision_shape.hpp>
#include <dart/simulation/body/rigid_body.hpp>
#include <dart/simulation/body/rigid_body_options.hpp>
#include <dart/simulation/common/exceptions.hpp>
#include <dart/simulation/comps/joint.hpp>
#include <dart/simulation/comps/multibody.hpp>
#include <dart/simulation/comps/name.hpp>
#include <dart/simulation/comps/rigid_body.hpp>
#include <dart/simulation/detail/boxed_lcp_contact.hpp>
#include <dart/simulation/detail/deformable_vbd/rigid_world_contact.hpp>
#include <dart/simulation/detail/entity_conversion.hpp>
#include <dart/simulation/detail/world_registry_access.hpp>
#include <dart/simulation/multibody/joint.hpp>
#include <dart/simulation/multibody/link.hpp>
#include <dart/simulation/multibody/multibody.hpp>
#include <dart/simulation/world.hpp>
#include <dart/simulation/world_options.hpp>

#include <dart/math/lcp/lcp_types.hpp>
#include <dart/math/lcp/projection/apgd_solver.hpp>
#include <dart/math/lcp/projection/bgs_solver.hpp>
#include <dart/math/lcp/projection/blocked_jacobi_solver.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <gtest/gtest.h>

#include <algorithm>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <cmath>

namespace sx = dart::simulation;
namespace dvbd = dart::simulation::detail::deformable_vbd;

namespace {

//==============================================================================
// Build a frictionless sphere-on-static-ground drop scene with the requested
// contact solver method. Ground top face is at z = 0; the sphere (radius 0.5)
// starts above it so it should settle with its center near z = 0.5.
std::unique_ptr<sx::World> buildDropScene(
    sx::ContactSolverMethod method, double sphereHeight = 1.0)
{
  sx::WorldOptions options;
  options.timeStep = 0.005;
  options.gravity = Eigen::Vector3d(0.0, 0.0, -9.81);
  options.contactSolverMethod = method;
  auto world = std::make_unique<sx::World>(options);

  sx::RigidBodyOptions groundOptions;
  groundOptions.isStatic = true;
  groundOptions.position = Eigen::Vector3d(0.0, 0.0, -0.5);
  auto ground = world->addRigidBody("ground", groundOptions);
  ground.setCollisionShape(
      sx::CollisionShape::makeBox(Eigen::Vector3d(5.0, 5.0, 0.5)));
  ground.setFriction(0.0);

  sx::RigidBodyOptions sphereOptions;
  sphereOptions.position = Eigen::Vector3d(0.0, 0.0, sphereHeight);
  auto sphere = world->addRigidBody("sphere", sphereOptions);
  sphere.setCollisionShape(sx::CollisionShape::makeSphere(0.5));
  sphere.setFriction(0.0);

  return world;
}

} // namespace

//==============================================================================
// The contact-solver selector is reflected by the World and defaults to
// SequentialImpulse, independent of the differentiable flag.
TEST(BoxedLcpContact, MethodSelectorReflectsConstruction)
{
  sx::World defaultWorld;
  EXPECT_EQ(
      defaultWorld.getContactSolverMethod(),
      sx::ContactSolverMethod::SequentialImpulse);

  sx::WorldOptions options;
  options.contactSolverMethod = sx::ContactSolverMethod::BoxedLcp;
  sx::World lcpWorld(options);
  EXPECT_EQ(
      lcpWorld.getContactSolverMethod(), sx::ContactSolverMethod::BoxedLcp);
  EXPECT_FALSE(lcpWorld.isDifferentiable());
}

//==============================================================================
// Compound-shape contacts must key AVBD warm-start rows from the contacted
// shape, not from the body's primary shape.
TEST(AvbdContact, CompoundShapeFeatureKeysUseContactedShapeIndex)
{
  sx::World world;

  auto compound = world.addRigidBody("compound");
  compound.addCollisionShape(
      sx::CollisionShape::makeBox(Eigen::Vector3d(0.1, 0.1, 0.1)));
  sx::CollisionShape secondary
      = sx::CollisionShape::makeBox(Eigen::Vector3d(0.25, 0.2, 0.2));
  secondary.localTransform.translation() = Eigen::Vector3d(1.0, 0.0, 0.0);
  compound.addCollisionShape(secondary);

  sx::RigidBodyOptions targetOptions;
  targetOptions.position = Eigen::Vector3d(1.35, 0.0, 0.0);
  auto target = world.addRigidBody("target", targetOptions);
  target.setCollisionShape(
      sx::CollisionShape::makeBox(Eigen::Vector3d(0.25, 0.2, 0.2)));

  const std::vector<sx::Contact> contacts = world.collide();
  ASSERT_FALSE(contacts.empty());
  const sx::Contact& contact = contacts.front();
  const bool compoundIsA
      = sx::detail::toRegistryEntity(contact.bodyA.getEntity())
        == sx::detail::toRegistryEntity(compound.getEntity());
  ASSERT_TRUE(
      compoundIsA
      || sx::detail::toRegistryEntity(contact.bodyB.getEntity())
             == sx::detail::toRegistryEntity(compound.getEntity()));
  const std::size_t compoundShapeIndex
      = compoundIsA ? contact.shapeIndexA : contact.shapeIndexB;
  const Eigen::Vector3d compoundLocalPoint
      = compoundIsA ? contact.localPointA : contact.localPointB;
  ASSERT_EQ(compoundShapeIndex, 1u);
  ASSERT_TRUE(compoundLocalPoint.allFinite());

  const dvbd::AvbdRigidWorldContactSnapshot snapshot
      = dvbd::buildAvbdRigidWorldContactSnapshot(
          dart::simulation::detail::registryOf(world),
          contacts,
          dvbd::AvbdRigidWorldContactOptions{});
  ASSERT_FALSE(snapshot.contacts.empty());
  const auto& manifold = snapshot.contacts.front();
  const dvbd::AvbdContactEndpointId compoundEndpoint
      = compoundIsA ? manifold.endpointA : manifold.endpointB;

  const std::uint64_t featureCode = dvbd::avbdBoxContactFeatureCode(
      compoundLocalPoint, secondary.halfExtents);
  // Each shape occupies a disjoint feature-id block (shapeIndex * stride) so
  // distinct shapes never alias, even across shape types.
  const std::uint64_t expectedFeature = dvbd::packAvbdContactFeatureId(
      dvbd::avbdBoxContactFeatureKind(featureCode),
      compoundShapeIndex * dvbd::kAvbdRigidWorldShapeFeatureStride
          + dvbd::packAvbdBoxContactFeatureId(0, featureCode));
  EXPECT_EQ(compoundEndpoint.feature, expectedFeature);
}

//==============================================================================
// The private AVBD contact path is an internal forward-solve opt-in. Its first
// World slice projects a supported penetrating rigid-body contact into the
// velocity that the existing position stage then applies.
TEST(AvbdContact, PenetratingRigidBodyProjectsVelocity)
{
  auto avbd = buildDropScene(sx::ContactSolverMethod::SequentialImpulse, 0.49);
  avbd->setGravity(Eigen::Vector3d::Zero());
  auto sphere = avbd->getRigidBody("sphere");
  ASSERT_TRUE(sphere.has_value());
  dart::simulation::detail::registryOf(*avbd)
      .emplace_or_replace<sx::comps::RigidAvbdContactConfig>(
          dart::simulation::detail::toRegistryEntity(sphere->getEntity()));
  avbd->enterSimulationMode();

  avbd->step();

  EXPECT_GT(sphere->getTranslation().z(), 0.498);
  EXPECT_LT(sphere->getTranslation().z(), 0.505);
  EXPECT_GT(sphere->getLinearVelocity().z(), 0.0);
}

//==============================================================================
// Fixed-joint rows are still private AVBD detail, but the contact-stage opt-in
// should append them to the same rigid World projection when they link rigid
// body entities.
TEST(AvbdContact, FixedJointRowsParticipateInProjection)
{
  auto avbd = buildDropScene(sx::ContactSolverMethod::SequentialImpulse, 0.49);
  avbd->setGravity(Eigen::Vector3d::Zero());

  auto ground = avbd->getRigidBody("ground");
  auto sphere = avbd->getRigidBody("sphere");
  ASSERT_TRUE(ground.has_value());
  ASSERT_TRUE(sphere.has_value());

  Eigen::Isometry3d spherePose = Eigen::Isometry3d::Identity();
  spherePose.translation() = Eigen::Vector3d(0.25, 0.0, 0.49);
  sphere->setTransform(spherePose);

  auto& registry = dart::simulation::detail::registryOf(*avbd);
  const auto& toReg = dart::simulation::detail::toRegistryEntity;
  registry.emplace_or_replace<sx::comps::RigidAvbdContactConfig>(
      toReg(sphere->getEntity()));

  const entt::entity jointEntity = registry.create();
  auto& joint = registry.emplace<sx::comps::Joint>(jointEntity);
  joint.type = sx::comps::JointType::Fixed;
  joint.parentLink = toReg(ground->getEntity());
  joint.childLink = toReg(sphere->getEntity());

  auto& config
      = registry.emplace<dvbd::AvbdRigidWorldPointJointConfig>(jointEntity);
  config.localAnchorA = Eigen::Vector3d(0.0, 0.0, 1.0);
  config.localAnchorB = Eigen::Vector3d::Zero();
  config.startStiffness = 1e5;
  config.maxStiffness = 1e6;

  avbd->enterSimulationMode();
  avbd->step();

  EXPECT_LT(std::abs(sphere->getTranslation().x()), 0.05);
  EXPECT_LT(sphere->getLinearVelocity().x(), 0.0);
  EXPECT_GT(sphere->getTranslation().z(), 0.498);
}

//==============================================================================
// Private fixed-joint rows should also project rigid bodies when no contact
// rows are present, so the World path does not depend on contact activation.
TEST(AvbdContact, FixedJointRowsProjectWithoutContacts)
{
  sx::WorldOptions options;
  options.timeStep = 0.005;
  options.gravity = Eigen::Vector3d::Zero();
  sx::World world(options);

  sx::RigidBodyOptions baseOptions;
  baseOptions.isStatic = true;
  auto base = world.addRigidBody("base", baseOptions);

  sx::RigidBodyOptions linkOptions;
  linkOptions.position = Eigen::Vector3d::UnitX();
  auto link = world.addRigidBody("link", linkOptions);

  auto& registry = dart::simulation::detail::registryOf(world);
  const entt::entity jointEntity = registry.create();
  auto& joint = registry.emplace<sx::comps::Joint>(jointEntity);
  joint.type = sx::comps::JointType::Fixed;
  joint.parentLink
      = dart::simulation::detail::toRegistryEntity(base.getEntity());
  joint.childLink
      = dart::simulation::detail::toRegistryEntity(link.getEntity());

  auto& config
      = registry.emplace<dvbd::AvbdRigidWorldPointJointConfig>(jointEntity);
  config.startStiffness = 1e5;
  config.maxStiffness = 1e6;

  world.enterSimulationMode();
  world.step();

  EXPECT_LT(std::abs(link.getTranslation().x()), 0.05);
  EXPECT_LT(link.getLinearVelocity().x(), 0.0);
  EXPECT_TRUE(base.getTranslation().isApprox(Eigen::Vector3d::Zero()));
}

//==============================================================================
// Missing private fixed-joint configs should be initialized from the
// design-time pose on opt-in rigid bodies when simulation mode starts, not
// re-baselined from later drift.
TEST(AvbdContact, FixedJointPoseBridgeCapturesSimulationEntryPose)
{
  sx::WorldOptions options;
  options.timeStep = 0.005;
  options.gravity = Eigen::Vector3d::Zero();
  sx::World world(options);

  sx::RigidBodyOptions baseOptions;
  baseOptions.isStatic = true;
  auto base = world.addRigidBody("base", baseOptions);

  sx::RigidBodyOptions linkOptions;
  linkOptions.position = Eigen::Vector3d::UnitX();
  auto link = world.addRigidBody("link", linkOptions);

  auto& registry = dart::simulation::detail::registryOf(world);
  registry.emplace_or_replace<sx::comps::RigidAvbdContactConfig>(
      sx::detail::toRegistryEntity(link.getEntity()));

  const entt::entity jointEntity = registry.create();
  auto& joint = registry.emplace<sx::comps::Joint>(jointEntity);
  joint.type = sx::comps::JointType::Fixed;
  joint.parentLink = sx::detail::toRegistryEntity(base.getEntity());
  joint.childLink = sx::detail::toRegistryEntity(link.getEntity());

  ASSERT_FALSE(
      registry.all_of<dvbd::AvbdRigidWorldPointJointConfig>(jointEntity));
  world.enterSimulationMode();

  const auto& config
      = registry.get<dvbd::AvbdRigidWorldPointJointConfig>(jointEntity);
  EXPECT_NEAR(
      (config.localAnchorA - Eigen::Vector3d::UnitX()).norm(), 0.0, 1e-12);
  EXPECT_NEAR(config.localAnchorB.norm(), 0.0, 1e-12);

  Eigen::Isometry3d driftedPose = Eigen::Isometry3d::Identity();
  driftedPose.translation() = Eigen::Vector3d(1.25, 0.0, 0.0);
  link.setTransform(driftedPose);

  world.step();

  EXPECT_LT(std::abs(link.getTranslation().x() - 1.0), 0.05);
  EXPECT_LT(link.getLinearVelocity().x(), 0.0);
  EXPECT_TRUE(base.getTranslation().isApprox(Eigen::Vector3d::Zero()));
}

//==============================================================================
// Private rigid-body revolute ECS joints should configure the same point-joint
// primitive as fixed joints, but leave the configured hinge axis free.
TEST(AvbdContact, RevoluteRigidBodyJointPoseBridgeCapturesAxisConfig)
{
  sx::WorldOptions options;
  options.timeStep = 0.005;
  options.gravity = Eigen::Vector3d::Zero();
  sx::World world(options);

  sx::RigidBodyOptions baseOptions;
  baseOptions.isStatic = true;
  auto base = world.addRigidBody("base", baseOptions);

  sx::RigidBodyOptions linkOptions;
  linkOptions.position = Eigen::Vector3d::UnitX();
  auto link = world.addRigidBody("link", linkOptions);

  auto& registry = dart::simulation::detail::registryOf(world);
  auto& avbdConfig
      = registry.emplace_or_replace<sx::comps::RigidAvbdContactConfig>(
          sx::detail::toRegistryEntity(link.getEntity()));
  avbdConfig.startStiffness = 123.0;
  avbdConfig.maxStiffness = 456.0;

  const Eigen::Vector3d hingeAxis = Eigen::Vector3d(1.0, 2.0, 3.0).normalized();
  const entt::entity jointEntity = registry.create();
  auto& joint = registry.emplace<sx::comps::Joint>(jointEntity);
  joint.type = sx::comps::JointType::Revolute;
  joint.parentLink = sx::detail::toRegistryEntity(base.getEntity());
  joint.childLink = sx::detail::toRegistryEntity(link.getEntity());
  joint.axis = hingeAxis;

  ASSERT_FALSE(
      registry.all_of<dvbd::AvbdRigidWorldPointJointConfig>(jointEntity));
  world.enterSimulationMode();

  ASSERT_TRUE(
      registry.all_of<dvbd::AvbdRigidWorldPointJointConfig>(jointEntity));
  const auto& config
      = registry.get<dvbd::AvbdRigidWorldPointJointConfig>(jointEntity);
  EXPECT_NEAR(
      (config.localAnchorA - Eigen::Vector3d::UnitX()).norm(), 0.0, 1e-12);
  EXPECT_NEAR(config.localAnchorB.norm(), 0.0, 1e-12);
  EXPECT_EQ(config.linearAxisMask, dvbd::kAvbdRigidJointAllAxesMask);
  EXPECT_EQ(config.angularAxisMask, dvbd::avbdRigidJointAllButAxisMask(2u));
  EXPECT_NEAR(config.angularAxes.col(0).dot(hingeAxis), 0.0, 1e-12);
  EXPECT_NEAR(config.angularAxes.col(1).dot(hingeAxis), 0.0, 1e-12);
  EXPECT_NEAR(std::abs(config.angularAxes.col(2).dot(hingeAxis)), 1.0, 1e-12);
  EXPECT_EQ(config.startStiffness, 123.0);
  EXPECT_EQ(config.maxStiffness, 456.0);

  const auto inputs = dvbd::extractAvbdRigidWorldPointJointInputs(registry);
  ASSERT_EQ(inputs.size(), 1u);
  EXPECT_EQ(inputs[0].linearAxisMask, config.linearAxisMask);
  EXPECT_EQ(inputs[0].angularAxisMask, config.angularAxisMask);
  EXPECT_NEAR((inputs[0].angularAxes - config.angularAxes).norm(), 0.0, 1e-12);
}

//==============================================================================
// Private rigid-body prismatic ECS joints should leave the configured
// translation axis free while constraining all angular axes.
TEST(AvbdContact, PrismaticRigidBodyJointPoseBridgeCapturesAxisConfig)
{
  sx::WorldOptions options;
  options.timeStep = 0.005;
  options.gravity = Eigen::Vector3d::Zero();
  sx::World world(options);

  sx::RigidBodyOptions baseOptions;
  baseOptions.isStatic = true;
  auto base = world.addRigidBody("base", baseOptions);

  sx::RigidBodyOptions linkOptions;
  linkOptions.position = Eigen::Vector3d::UnitX();
  auto link = world.addRigidBody("link", linkOptions);

  auto& registry = dart::simulation::detail::registryOf(world);
  const Eigen::Vector3d translationAxis
      = Eigen::Vector3d(-0.5, 0.25, 1.0).normalized();
  const entt::entity jointEntity = registry.create();
  auto& joint = registry.emplace<sx::comps::Joint>(jointEntity);
  joint.type = sx::comps::JointType::Prismatic;
  joint.parentLink = sx::detail::toRegistryEntity(base.getEntity());
  joint.childLink = sx::detail::toRegistryEntity(link.getEntity());
  joint.axis = translationAxis;

  ASSERT_FALSE(
      registry.all_of<dvbd::AvbdRigidWorldPointJointConfig>(jointEntity));
  world.enterSimulationMode();

  ASSERT_TRUE(
      registry.all_of<dvbd::AvbdRigidWorldPointJointConfig>(jointEntity));
  const auto& config
      = registry.get<dvbd::AvbdRigidWorldPointJointConfig>(jointEntity);
  EXPECT_EQ(config.linearAxisMask, dvbd::avbdRigidJointAllButAxisMask(2u));
  EXPECT_EQ(config.angularAxisMask, dvbd::kAvbdRigidJointAllAxesMask);
  EXPECT_NEAR(config.linearAxes.col(0).dot(translationAxis), 0.0, 1e-12);
  EXPECT_NEAR(config.linearAxes.col(1).dot(translationAxis), 0.0, 1e-12);
  EXPECT_NEAR(
      std::abs(config.linearAxes.col(2).dot(translationAxis)), 1.0, 1e-12);

  const auto inputs = dvbd::extractAvbdRigidWorldPointJointInputs(registry);
  ASSERT_EQ(inputs.size(), 1u);
  EXPECT_EQ(inputs[0].linearAxisMask, config.linearAxisMask);
  EXPECT_EQ(inputs[0].angularAxisMask, config.angularAxisMask);
  EXPECT_NEAR((inputs[0].linearAxes - config.linearAxes).norm(), 0.0, 1e-12);
}

//==============================================================================
// The public facade should create the fixed-joint row config without exposing
// ECS or AVBD detail components to users.
TEST(AvbdContact, PublicRigidBodyFixedJointProjectsFromCapturedPose)
{
  sx::WorldOptions options;
  options.timeStep = 0.005;
  options.gravity = Eigen::Vector3d::Zero();
  sx::World world(options);

  sx::RigidBodyOptions baseOptions;
  baseOptions.isStatic = true;
  auto base = world.addRigidBody("base", baseOptions);

  sx::RigidBodyOptions linkOptions;
  linkOptions.position = Eigen::Vector3d::UnitX();
  auto link = world.addRigidBody("link", linkOptions);

  auto joint = world.addRigidBodyFixedJoint("base_to_link", base, link);
  EXPECT_EQ(joint.getName(), "base_to_link");
  EXPECT_EQ(joint.getType(), sx::JointType::Fixed);
  EXPECT_EQ(joint.getDOFCount(), 0u);
  EXPECT_EQ(joint.getParentRigidBody().getName(), "base");
  EXPECT_EQ(joint.getChildRigidBody().getName(), "link");
  EXPECT_TRUE(world.hasRigidBodyFixedJoint("base_to_link"));
  EXPECT_FALSE(world.hasRigidBodyFixedJoint("missing"));
  EXPECT_EQ(world.getRigidBodyFixedJointCount(), 1u);
  auto foundJoint = world.getRigidBodyFixedJoint("base_to_link");
  ASSERT_TRUE(foundJoint.has_value());
  EXPECT_EQ(foundJoint->getParentRigidBody().getName(), "base");
  EXPECT_EQ(foundJoint->getChildRigidBody().getName(), "link");
  EXPECT_FALSE(world.getRigidBodyFixedJoint("missing").has_value());
  const auto fixedJoints = world.getRigidBodyFixedJoints();
  ASSERT_EQ(fixedJoints.size(), 1u);
  EXPECT_EQ(fixedJoints.front().getName(), "base_to_link");
  EXPECT_THROW(
      {
        auto parentLink = joint.getParentLink();
        (void)parentLink;
      },
      sx::InvalidArgumentException);
  EXPECT_THROW(
      {
        auto childLink = joint.getChildLink();
        (void)childLink;
      },
      sx::InvalidArgumentException);
  EXPECT_THROW(
      world.addRigidBodyFixedJoint("base_to_link", base, link),
      sx::InvalidArgumentException);

  Eigen::Isometry3d driftedPose = Eigen::Isometry3d::Identity();
  driftedPose.translation() = Eigen::Vector3d(1.25, 0.0, 0.0);
  link.setTransform(driftedPose);

  world.enterSimulationMode();
  world.step();

  EXPECT_LT(std::abs(link.getTranslation().x() - 1.0), 0.05);
  EXPECT_LT(link.getLinearVelocity().x(), 0.0);
  EXPECT_THROW(
      world.addRigidBodyFixedJoint("late_joint", base, link),
      sx::InvalidOperationException);
}

//==============================================================================
// The public one-DOF rigid-body facade should expose a revolute joint that
// projects the captured anchor while leaving the hinge axis free.
TEST(AvbdContact, PublicRigidBodyRevoluteJointProjectsAnchor)
{
  sx::WorldOptions options;
  options.timeStep = 0.005;
  options.gravity = Eigen::Vector3d::Zero();
  sx::World world(options);

  sx::RigidBodyOptions baseOptions;
  baseOptions.isStatic = true;
  auto base = world.addRigidBody("base", baseOptions);

  sx::RigidBodyOptions linkOptions;
  linkOptions.position = Eigen::Vector3d::UnitX();
  auto link = world.addRigidBody("link", linkOptions);

  auto joint = world.addRigidBodyRevoluteJoint(
      "base_to_link_hinge", base, link, Eigen::Vector3d::UnitZ());
  EXPECT_EQ(joint.getType(), sx::JointType::Revolute);
  EXPECT_EQ(joint.getDOFCount(), 1u);
  EXPECT_EQ(joint.getParentRigidBody().getName(), "base");
  EXPECT_EQ(joint.getChildRigidBody().getName(), "link");
  EXPECT_TRUE(world.hasRigidBodyJoint("base_to_link_hinge"));
  EXPECT_FALSE(world.hasRigidBodyFixedJoint("base_to_link_hinge"));
  EXPECT_EQ(world.getRigidBodyJointCount(), 1u);
  EXPECT_EQ(world.getRigidBodyFixedJointCount(), 0u);
  auto foundJoint = world.getRigidBodyJoint("base_to_link_hinge");
  ASSERT_TRUE(foundJoint.has_value());
  EXPECT_EQ(foundJoint->getType(), sx::JointType::Revolute);
  const auto rigidBodyJoints = world.getRigidBodyJoints();
  ASSERT_EQ(rigidBodyJoints.size(), 1u);
  EXPECT_EQ(rigidBodyJoints.front().getName(), "base_to_link_hinge");

  Eigen::Isometry3d driftedPose = Eigen::Isometry3d::Identity();
  driftedPose.translation() = Eigen::Vector3d(1.25, 0.25, 0.0);
  driftedPose.linear()
      = Eigen::AngleAxisd(0.5, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  link.setTransform(driftedPose);

  world.enterSimulationMode();
  world.step();

  EXPECT_LT(std::abs(link.getTranslation().x() - 1.0), 0.05);
  EXPECT_LT(std::abs(link.getTranslation().y()), 0.05);
  EXPECT_TRUE(base.getTranslation().isApprox(Eigen::Vector3d::Zero()));
}

//==============================================================================
// The public prismatic facade should project drift orthogonal to the slide axis
// while leaving translation along that axis unconstrained.
TEST(AvbdContact, PublicRigidBodyPrismaticJointProjectsOrthogonalDrift)
{
  sx::WorldOptions options;
  options.timeStep = 0.005;
  options.gravity = Eigen::Vector3d::Zero();
  sx::World world(options);

  sx::RigidBodyOptions baseOptions;
  baseOptions.isStatic = true;
  auto base = world.addRigidBody("base", baseOptions);

  sx::RigidBodyOptions linkOptions;
  linkOptions.position = Eigen::Vector3d::UnitZ();
  auto link = world.addRigidBody("link", linkOptions);

  auto joint = world.addRigidBodyPrismaticJoint(
      "base_to_link_slider", base, link, Eigen::Vector3d::UnitZ());
  EXPECT_EQ(joint.getType(), sx::JointType::Prismatic);
  EXPECT_EQ(joint.getDOFCount(), 1u);
  EXPECT_EQ(world.getRigidBodyJointCount(), 1u);
  EXPECT_FALSE(world.hasRigidBodyFixedJoint("base_to_link_slider"));

  Eigen::Isometry3d driftedPose = Eigen::Isometry3d::Identity();
  driftedPose.translation() = Eigen::Vector3d(0.25, 0.0, 1.5);
  link.setTransform(driftedPose);

  world.enterSimulationMode();
  world.step();

  EXPECT_LT(std::abs(link.getTranslation().x()), 0.05);
  EXPECT_NEAR(link.getTranslation().z(), 1.5, 0.05);
  EXPECT_TRUE(base.getTranslation().isApprox(Eigen::Vector3d::Zero()));
}

//==============================================================================
// Saving a public fixed joint before simulation should not lose the AVBD row
// config permanently; loading and entering simulation mode restores it from the
// saved rigid-body poses.
TEST(AvbdContact, PublicRigidBodyFixedJointSurvivesSaveLoad)
{
  sx::WorldOptions options;
  options.timeStep = 0.005;
  options.gravity = Eigen::Vector3d::Zero();
  sx::World world(options);

  sx::RigidBodyOptions baseOptions;
  baseOptions.isStatic = true;
  auto base = world.addRigidBody("base", baseOptions);

  sx::RigidBodyOptions linkOptions;
  linkOptions.position = Eigen::Vector3d::UnitX();
  auto link = world.addRigidBody("link", linkOptions);
  (void)world.addRigidBodyFixedJoint("base_to_link", base, link);

  Eigen::Isometry3d designPose = Eigen::Isometry3d::Identity();
  designPose.translation() = Eigen::Vector3d(1.25, 0.0, 0.0);
  link.setTransform(designPose);

  std::stringstream data;
  world.saveBinary(data);

  sx::World restored;
  restored.loadBinary(data);

  auto restoredBase = restored.getRigidBody("base");
  auto restoredLink = restored.getRigidBody("link");
  ASSERT_TRUE(restoredBase.has_value());
  ASSERT_TRUE(restoredLink.has_value());

  auto& registry = dart::simulation::detail::registryOf(restored);
  EXPECT_EQ(restored.getRigidBodyFixedJointCount(), 1u);
  EXPECT_TRUE(restored.hasRigidBodyFixedJoint("base_to_link"));
  auto restoredJoint = restored.getRigidBodyFixedJoint("base_to_link");
  ASSERT_TRUE(restoredJoint.has_value());
  const entt::entity jointEntity
      = sx::detail::toRegistryEntity(restoredJoint->getEntity());
  EXPECT_EQ(restoredJoint->getParentRigidBody().getName(), "base");
  EXPECT_EQ(restoredJoint->getChildRigidBody().getName(), "link");
  ASSERT_FALSE(
      registry.all_of<dvbd::AvbdRigidWorldPointJointConfig>(jointEntity));

  restored.enterSimulationMode();

  ASSERT_TRUE(
      registry.all_of<dvbd::AvbdRigidWorldPointJointConfig>(jointEntity));
  const auto& config
      = registry.get<dvbd::AvbdRigidWorldPointJointConfig>(jointEntity);
  EXPECT_NEAR(
      (config.localAnchorA - Eigen::Vector3d::UnitX()).norm(), 0.0, 1e-12);
  EXPECT_NEAR(config.localAnchorB.norm(), 0.0, 1e-12);

  restored.step();

  EXPECT_LT(std::abs(restoredLink->getTranslation().x() - 1.0), 0.05);
  EXPECT_LT(restoredLink->getLinearVelocity().x(), 0.0);
  EXPECT_TRUE(restoredBase->getTranslation().isApprox(Eigen::Vector3d::Zero()));
}

//==============================================================================
TEST(AvbdContact, PublicRigidBodyJointBreakStateSurvivesSaveLoad)
{
  sx::World world;

  sx::RigidBodyOptions baseOptions;
  baseOptions.isStatic = true;
  auto base = world.addRigidBody("base", baseOptions);

  sx::RigidBodyOptions linkOptions;
  linkOptions.position = Eigen::Vector3d::UnitX();
  auto link = world.addRigidBody("link", linkOptions);

  auto joint = world.addRigidBodyFixedJoint("base_to_link", base, link);
  joint.setBreakForce(7.5);

  auto& registry = dart::simulation::detail::registryOf(world);
  registry
      .get<sx::comps::Joint>(sx::detail::toRegistryEntity(joint.getEntity()))
      .broken = true;

  std::stringstream data;
  world.saveBinary(data);

  sx::World restored;
  restored.loadBinary(data);

  auto restoredJoint = restored.getRigidBodyFixedJoint("base_to_link");
  ASSERT_TRUE(restoredJoint.has_value());
  EXPECT_DOUBLE_EQ(restoredJoint->getBreakForce(), 7.5);
  EXPECT_TRUE(restoredJoint->isBroken());

  restoredJoint->resetBreakage();
  EXPECT_FALSE(restoredJoint->isBroken());
}

//==============================================================================
// Saving after simulation mode should also preserve the public fixed joint. A
// loaded simulation-mode world cannot re-enter simulation mode, so loadBinary()
// must rebuild the private AVBD row config directly.
TEST(AvbdContact, PublicRigidBodyFixedJointSurvivesSimulationModeSaveLoad)
{
  sx::WorldOptions options;
  options.timeStep = 0.005;
  options.gravity = Eigen::Vector3d::Zero();
  sx::World world(options);

  sx::RigidBodyOptions baseOptions;
  baseOptions.isStatic = true;
  auto base = world.addRigidBody("base", baseOptions);

  sx::RigidBodyOptions linkOptions;
  linkOptions.position = Eigen::Vector3d::UnitX();
  auto link = world.addRigidBody("link", linkOptions);
  (void)world.addRigidBodyFixedJoint("base_to_link", base, link);

  world.enterSimulationMode();

  std::stringstream data;
  world.saveBinary(data);

  sx::World restored;
  restored.loadBinary(data);
  ASSERT_TRUE(restored.isSimulationMode());

  auto restoredBase = restored.getRigidBody("base");
  auto restoredLink = restored.getRigidBody("link");
  ASSERT_TRUE(restoredBase.has_value());
  ASSERT_TRUE(restoredLink.has_value());

  auto& registry = dart::simulation::detail::registryOf(restored);
  EXPECT_EQ(restored.getRigidBodyFixedJointCount(), 1u);
  EXPECT_TRUE(restored.hasRigidBodyFixedJoint("base_to_link"));
  auto restoredJoint = restored.getRigidBodyFixedJoint("base_to_link");
  ASSERT_TRUE(restoredJoint.has_value());
  const entt::entity jointEntity
      = sx::detail::toRegistryEntity(restoredJoint->getEntity());
  EXPECT_EQ(restoredJoint->getParentRigidBody().getName(), "base");
  EXPECT_EQ(restoredJoint->getChildRigidBody().getName(), "link");
  ASSERT_TRUE(
      registry.all_of<dvbd::AvbdRigidWorldPointJointConfig>(jointEntity));
  const auto& config
      = registry.get<dvbd::AvbdRigidWorldPointJointConfig>(jointEntity);
  EXPECT_NEAR(
      (config.localAnchorA - Eigen::Vector3d::UnitX()).norm(), 0.0, 1e-12);
  EXPECT_NEAR(config.localAnchorB.norm(), 0.0, 1e-12);

  Eigen::Isometry3d driftedPose = Eigen::Isometry3d::Identity();
  driftedPose.translation() = Eigen::Vector3d(1.25, 0.0, 0.0);
  restoredLink->setTransform(driftedPose);

  restored.step();

  EXPECT_LT(std::abs(restoredLink->getTranslation().x() - 1.0), 0.05);
  EXPECT_LT(restoredLink->getLinearVelocity().x(), 0.0);
  EXPECT_TRUE(restoredBase->getTranslation().isApprox(Eigen::Vector3d::Zero()));
}

//==============================================================================
// Public fixed joints should remain active when ordinary, non-AVBD-opted
// contacts involve the fixed body. The contact still falls through to the
// selected default contact solver while the fixed-joint rows project
// independently.
TEST(AvbdContact, PublicFixedJointProjectsWithDefaultContactOnFixedBody)
{
  sx::WorldOptions options;
  options.timeStep = 0.005;
  options.gravity = Eigen::Vector3d::Zero();
  sx::World world(options);

  sx::RigidBodyOptions baseOptions;
  baseOptions.isStatic = true;
  auto base = world.addRigidBody("base", baseOptions);

  sx::RigidBodyOptions linkOptions;
  linkOptions.position = Eigen::Vector3d(1.0, 0.0, 0.49);
  auto link = world.addRigidBody("link", linkOptions);
  link.setCollisionShape(
      sx::CollisionShape::makeBox(Eigen::Vector3d(0.5, 0.5, 0.5)));

  sx::RigidBodyOptions groundOptions;
  groundOptions.isStatic = true;
  groundOptions.position = Eigen::Vector3d(1.0, 0.0, -0.5);
  auto ground = world.addRigidBody("ground", groundOptions);
  ground.setCollisionShape(
      sx::CollisionShape::makeBox(Eigen::Vector3d(2.0, 2.0, 0.5)));

  (void)world.addRigidBodyFixedJoint("base_to_link", base, link);
  world.enterSimulationMode();

  auto& registry = dart::simulation::detail::registryOf(world);
  EXPECT_FALSE(registry.all_of<sx::comps::RigidAvbdContactConfig>(
      sx::detail::toRegistryEntity(link.getEntity())));
  EXPECT_FALSE(registry.all_of<sx::comps::RigidAvbdContactConfig>(
      sx::detail::toRegistryEntity(ground.getEntity())));
  ASSERT_FALSE(world.collide().empty());

  Eigen::Isometry3d driftedPose = link.getTransform();
  driftedPose.translation().x() = 1.25;
  link.setTransform(driftedPose);
  const double driftBeforeStep = std::abs(link.getTranslation().x() - 1.0);
  ASSERT_FALSE(world.collide().empty());

  world.step();

  EXPECT_LT(std::abs(link.getTranslation().x() - 1.0), driftBeforeStep);
  EXPECT_LT(link.getLinearVelocity().x(), 0.0);
  EXPECT_TRUE(base.getTranslation().isApprox(Eigen::Vector3d::Zero()));
}

//==============================================================================
// The no-contact fixed-joint path should also route angular rows through the
// private AVBD projection instead of only correcting point-anchor drift.
TEST(AvbdContact, FixedJointAngularRowsProjectWithoutContacts)
{
  sx::WorldOptions options;
  options.timeStep = 0.005;
  options.gravity = Eigen::Vector3d::Zero();
  sx::World world(options);

  sx::RigidBodyOptions baseOptions;
  baseOptions.isStatic = true;
  auto base = world.addRigidBody("base", baseOptions);

  sx::RigidBodyOptions linkOptions;
  linkOptions.orientation = Eigen::AngleAxisd(0.5, Eigen::Vector3d::UnitZ());
  auto link = world.addRigidBody("link", linkOptions);

  auto& registry = dart::simulation::detail::registryOf(world);
  const entt::entity jointEntity = registry.create();
  auto& joint = registry.emplace<sx::comps::Joint>(jointEntity);
  joint.type = sx::comps::JointType::Fixed;
  joint.parentLink
      = dart::simulation::detail::toRegistryEntity(base.getEntity());
  joint.childLink
      = dart::simulation::detail::toRegistryEntity(link.getEntity());

  auto& config
      = registry.emplace<dvbd::AvbdRigidWorldPointJointConfig>(jointEntity);
  config.startStiffness = 1e5;
  config.maxStiffness = 1e6;

  world.enterSimulationMode();
  world.step();

  const Eigen::AngleAxisd residual(link.getTransform().linear());
  EXPECT_LT(std::abs(residual.angle()), 0.05);
  EXPECT_LT(link.getAngularVelocity().z(), 0.0);
  EXPECT_TRUE(
      base.getTransform().linear().isApprox(Eigen::Matrix3d::Identity()));
}

//==============================================================================
// Private fixed-joint rows should continue to project when an unrelated contact
// falls back to the ordinary rigid contact solver.
TEST(AvbdContact, FixedJointRowsProjectWithFallbackContacts)
{
  auto world = buildDropScene(sx::ContactSolverMethod::SequentialImpulse, 0.49);
  world->setGravity(Eigen::Vector3d::Zero());

  auto sphere = world->getRigidBody("sphere");
  ASSERT_TRUE(sphere.has_value());

  sx::RigidBodyOptions baseOptions;
  baseOptions.isStatic = true;
  baseOptions.position = Eigen::Vector3d(10.0, 0.0, 0.0);
  auto base = world->addRigidBody("joint_base", baseOptions);

  sx::RigidBodyOptions linkOptions;
  linkOptions.position = Eigen::Vector3d(11.0, 0.0, 0.0);
  auto link = world->addRigidBody("joint_link", linkOptions);

  auto& registry = dart::simulation::detail::registryOf(*world);
  const entt::entity jointEntity = registry.create();
  auto& joint = registry.emplace<sx::comps::Joint>(jointEntity);
  joint.type = sx::comps::JointType::Fixed;
  joint.parentLink
      = dart::simulation::detail::toRegistryEntity(base.getEntity());
  joint.childLink
      = dart::simulation::detail::toRegistryEntity(link.getEntity());

  auto& config
      = registry.emplace<dvbd::AvbdRigidWorldPointJointConfig>(jointEntity);
  config.startStiffness = 1e5;
  config.maxStiffness = 1e6;

  world->enterSimulationMode();
  world->step();

  EXPECT_LT(std::abs(link.getTranslation().x() - 10.0), 0.05);
  EXPECT_LT(link.getLinearVelocity().x(), 0.0);
  EXPECT_GT(sphere->getTranslation().z(), 0.49);
}

//==============================================================================
// A body dropped onto a static ground rests in the same place under both
// solver paths (non-penetration, normal velocity -> 0, same resting height).
TEST(BoxedLcpContact, RestingHeightMatchesSequentialImpulse)
{
  auto reference = buildDropScene(sx::ContactSolverMethod::SequentialImpulse);
  auto lcp = buildDropScene(sx::ContactSolverMethod::BoxedLcp);

  reference->enterSimulationMode();
  lcp->enterSimulationMode();
  reference->step(1000);
  lcp->step(1000);

  auto referenceSphere = reference->getRigidBody("sphere");
  auto lcpSphere = lcp->getRigidBody("sphere");
  ASSERT_TRUE(referenceSphere.has_value());
  ASSERT_TRUE(lcpSphere.has_value());

  const double referenceZ = referenceSphere->getTranslation().z();
  const double lcpZ = lcpSphere->getTranslation().z();

  // Both rest near z = 0.5 (sphere radius on a ground top at z = 0).
  EXPECT_NEAR(lcpZ, 0.5, 2e-2);
  // The LCP path matches the sequential-impulse resting height within a tight
  // tolerance.
  EXPECT_NEAR(lcpZ, referenceZ, 1e-6);

  // Non-penetration at rest: the sphere bottom stays at or above the ground.
  EXPECT_GT(lcpZ, 0.5 - 1e-3);

  // Normal velocity has essentially stopped.
  EXPECT_LT(std::abs(lcpSphere->getLinearVelocity().z()), 0.1);

  // The static ground did not move under the LCP path.
  EXPECT_TRUE(lcp->getRigidBody("ground")->getTranslation().isApprox(
      Eigen::Vector3d(0.0, 0.0, -0.5)));
}

//==============================================================================
// A single resting drop step: with the body already touching the ground at
// near-zero velocity, the LCP normal impulse cancels the incoming gravity
// velocity exactly as the sequential path does.
TEST(BoxedLcpContact, SingleStepNormalVelocityMatches)
{
  auto reference = buildDropScene(sx::ContactSolverMethod::SequentialImpulse);
  auto lcp = buildDropScene(sx::ContactSolverMethod::BoxedLcp);

  // Place both spheres just touching the ground (center at z = 0.5) so the
  // first step is a normal-only contact resolution.
  for (auto* world : {reference.get(), lcp.get()}) {
    auto sphere = world->getRigidBody("sphere");
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    pose.translation() = Eigen::Vector3d(0.0, 0.0, 0.5);
    sphere->setTransform(pose);
    world->enterSimulationMode();
  }

  reference->step();
  lcp->step();

  const double referenceVz
      = reference->getRigidBody("sphere")->getLinearVelocity().z();
  const double lcpVz = lcp->getRigidBody("sphere")->getLinearVelocity().z();

  // The contact removes the downward approach; residual normal velocity is
  // near zero and matches the sequential path closely.
  EXPECT_NEAR(lcpVz, referenceVz, 1e-9);
  EXPECT_LT(std::abs(lcpVz), 1e-6);
}

//==============================================================================
// Momentum/known-physics check: two equal-mass spheres in a head-on, fully
// inelastic (zero restitution), frictionless collision both come to rest, so
// total linear momentum (zero) is conserved -- identical under both paths.
TEST(BoxedLcpContact, EqualMassHeadOnConservesMomentum)
{
  const auto run = [](sx::ContactSolverMethod method) {
    sx::WorldOptions options;
    options.timeStep = 0.001;
    options.gravity = Eigen::Vector3d::Zero();
    options.contactSolverMethod = method;
    sx::World world(options);

    sx::RigidBodyOptions optionsA;
    optionsA.position = Eigen::Vector3d(-0.45, 0.0, 0.0);
    optionsA.linearVelocity = Eigen::Vector3d(1.0, 0.0, 0.0);
    auto bodyA = world.addRigidBody("a", optionsA);
    bodyA.setCollisionShape(sx::CollisionShape::makeSphere(0.5));
    bodyA.setFriction(0.0);

    sx::RigidBodyOptions optionsB;
    optionsB.position = Eigen::Vector3d(0.45, 0.0, 0.0);
    optionsB.linearVelocity = Eigen::Vector3d(-1.0, 0.0, 0.0);
    auto bodyB = world.addRigidBody("b", optionsB);
    bodyB.setCollisionShape(sx::CollisionShape::makeSphere(0.5));
    bodyB.setFriction(0.0);

    world.step();
    return std::make_pair(
        bodyA.getLinearVelocity().x(), bodyB.getLinearVelocity().x());
  };

  const auto reference = run(sx::ContactSolverMethod::SequentialImpulse);
  const auto lcp = run(sx::ContactSolverMethod::BoxedLcp);

  // Closed form: equal masses, head-on, fully inelastic -> both at rest.
  EXPECT_NEAR(lcp.first, 0.0, 1e-9);
  EXPECT_NEAR(lcp.second, 0.0, 1e-9);

  // Momentum conserved (zero before and after).
  EXPECT_NEAR(lcp.first + lcp.second, 0.0, 1e-12);

  // Parity with the sequential-impulse path.
  EXPECT_NEAR(lcp.first, reference.first, 1e-9);
  EXPECT_NEAR(lcp.second, reference.second, 1e-9);
}

namespace {

//==============================================================================
// Build a box-on-static-box-ground scene with the requested contact solver and
// Coulomb friction coefficient. Ground top face is at z = 0; the box (half
// extents 0.5) starts resting on it (center at z = 0.5). The box is given an
// initial horizontal velocity so the tangential rows are exercised.
std::unique_ptr<sx::World> buildFrictionScene(
    sx::ContactSolverMethod method,
    double friction,
    const Eigen::Vector3d& initialVelocity)
{
  sx::WorldOptions options;
  options.timeStep = 0.005;
  options.gravity = Eigen::Vector3d(0.0, 0.0, -9.81);
  options.contactSolverMethod = method;
  auto world = std::make_unique<sx::World>(options);

  sx::RigidBodyOptions groundOptions;
  groundOptions.isStatic = true;
  groundOptions.position = Eigen::Vector3d(0.0, 0.0, -0.5);
  auto ground = world->addRigidBody("ground", groundOptions);
  ground.setCollisionShape(
      sx::CollisionShape::makeBox(Eigen::Vector3d(20.0, 20.0, 0.5)));
  ground.setFriction(friction);

  sx::RigidBodyOptions boxOptions;
  boxOptions.position = Eigen::Vector3d(0.0, 0.0, 0.5);
  boxOptions.linearVelocity = initialVelocity;
  auto box = world->addRigidBody("box", boxOptions);
  box.setCollisionShape(
      sx::CollisionShape::makeBox(Eigen::Vector3d(0.5, 0.5, 0.5)));
  box.setFriction(friction);

  return world;
}

std::unique_ptr<sx::World> buildSeparatedSphereGroundScene(
    int sphereCount, double friction)
{
  sx::WorldOptions options;
  options.timeStep = 0.005;
  options.gravity = Eigen::Vector3d(0.0, 0.0, -9.81);
  options.contactSolverMethod = sx::ContactSolverMethod::BoxedLcp;
  auto world = std::make_unique<sx::World>(options);

  sx::RigidBodyOptions groundOptions;
  groundOptions.isStatic = true;
  groundOptions.position = Eigen::Vector3d(0.0, 0.0, -0.5);
  auto ground = world->addRigidBody("ground", groundOptions);
  ground.setCollisionShape(
      sx::CollisionShape::makeBox(Eigen::Vector3d(24.0, 24.0, 0.5)));
  ground.setFriction(friction);

  const int columns = static_cast<int>(
      std::ceil(std::sqrt(static_cast<double>(sphereCount))));
  constexpr double kSpacing = 2.0;
  for (int i = 0; i < sphereCount; ++i) {
    const int row = i / columns;
    const int col = i - row * columns;
    sx::RigidBodyOptions sphereOptions;
    sphereOptions.position = Eigen::Vector3d(
        kSpacing * static_cast<double>(col),
        kSpacing * static_cast<double>(row),
        0.5);
    sphereOptions.linearVelocity = Eigen::Vector3d(
        0.35 - 0.03 * static_cast<double>(i % 4),
        -0.22 + 0.025 * static_cast<double>(i % 5),
        -0.02);
    auto sphere
        = world->addRigidBody("sphere_" + std::to_string(i), sphereOptions);
    sphere.setCollisionShape(sx::CollisionShape::makeSphere(0.5));
    sphere.setFriction(friction);
  }

  return world;
}

std::unique_ptr<sx::World> buildSeparatedBoxGroundScene(
    int boxCount, double friction)
{
  sx::WorldOptions options;
  options.timeStep = boxCount >= 24 ? 0.001 : 0.005;
  options.gravity = Eigen::Vector3d(0.0, 0.0, -9.81);
  options.contactSolverMethod = sx::ContactSolverMethod::BoxedLcp;
  auto world = std::make_unique<sx::World>(options);

  sx::RigidBodyOptions groundOptions;
  groundOptions.isStatic = true;
  groundOptions.position = Eigen::Vector3d(0.0, 0.0, -0.5);
  auto ground = world->addRigidBody("ground", groundOptions);
  ground.setCollisionShape(
      sx::CollisionShape::makeBox(Eigen::Vector3d(24.0, 24.0, 0.5)));
  ground.setFriction(friction);

  const int columns
      = static_cast<int>(std::ceil(std::sqrt(static_cast<double>(boxCount))));
  constexpr double kSpacing = 2.0;
  for (int i = 0; i < boxCount; ++i) {
    const int row = i / columns;
    const int col = i - row * columns;
    sx::RigidBodyOptions boxOptions;
    boxOptions.position = Eigen::Vector3d(
        kSpacing * static_cast<double>(col),
        kSpacing * static_cast<double>(row),
        0.5);
    boxOptions.linearVelocity = Eigen::Vector3d(
        0.35 - 0.015 * static_cast<double>(i % 3),
        -0.2 + 0.01 * static_cast<double>(i % 2),
        -0.02);
    auto box = world->addRigidBody("box_" + std::to_string(i), boxOptions);
    box.setCollisionShape(
        sx::CollisionShape::makeBox(Eigen::Vector3d(0.5, 0.5, 0.5)));
    box.setFriction(friction);
  }

  return world;
}

std::unique_ptr<sx::World> buildArticulatedGroundScene(
    sx::ContactSolverMethod method, int linkCount = 1)
{
  if (linkCount <= 0) {
    return nullptr;
  }

  sx::WorldOptions options;
  options.timeStep = 0.002;
  options.gravity = Eigen::Vector3d(0.0, 0.0, -9.81);
  options.contactSolverMethod = method;
  auto world = std::make_unique<sx::World>(options);

  const int columns
      = static_cast<int>(std::ceil(std::sqrt(static_cast<double>(linkCount))));
  constexpr double kSpacing = 1.5;
  for (int i = 0; i < linkCount; ++i) {
    const int row = i / columns;
    const int col = i - row * columns;
    auto robot = world->addMultibody(
        linkCount == 1 ? "leg_robot" : "leg_robot_" + std::to_string(i));
    auto base = robot.addLink("base");
    sx::JointSpec spec;
    spec.name = "slider";
    spec.type = sx::JointType::Prismatic;
    spec.axis = Eigen::Vector3d::UnitZ();
    spec.transformFromParent = Eigen::Isometry3d::Identity();
    spec.transformFromParent.translation() = Eigen::Vector3d(
        kSpacing * static_cast<double>(col),
        kSpacing * static_cast<double>(row),
        0.0);
    auto leg = robot.addLink("leg", base, spec);
    leg.setMass(1.0);
    leg.setInertia(Eigen::Matrix3d::Identity());
    leg.setCollisionShape(sx::CollisionShape::makeSphere(0.2));
    leg.getParentJoint().setPosition(Eigen::VectorXd::Constant(1, -0.305));
    leg.getParentJoint().setVelocity(Eigen::VectorXd::Constant(1, -0.05));
  }

  sx::RigidBodyOptions groundOptions;
  groundOptions.isStatic = true;
  groundOptions.position = Eigen::Vector3d(0.0, 0.0, -1.0);
  auto ground = world->addRigidBody("ground", groundOptions);
  ground.setCollisionShape(
      sx::CollisionShape::makeBox(Eigen::Vector3d(24.0, 24.0, 0.5)));

  return world;
}

std::unique_ptr<sx::World> buildArticulatedRigidImpactScene(
    sx::ContactSolverMethod method, int pairCount = 1)
{
  EXPECT_GT(pairCount, 0);
  sx::WorldOptions options;
  options.timeStep = 0.001;
  options.gravity = Eigen::Vector3d::Zero();
  options.contactSolverMethod = method;
  auto world = std::make_unique<sx::World>(options);

  constexpr double kSpacing = 1.0;
  for (int i = 0; i < pairCount; ++i) {
    const double y = kSpacing * static_cast<double>(i);
    auto robot = world->addMultibody(
        pairCount == 1 ? "striker_robot"
                       : "striker_robot_" + std::to_string(i));
    auto base = robot.addLink("base");
    sx::JointSpec spec;
    spec.name = "rail";
    spec.type = sx::JointType::Prismatic;
    spec.axis = Eigen::Vector3d::UnitX();
    spec.transformFromParent = Eigen::Isometry3d::Identity();
    spec.transformFromParent.translation() = Eigen::Vector3d(0.0, y, 0.0);
    auto striker = robot.addLink("striker", base, spec);
    striker.setMass(2.0);
    striker.setInertia(Eigen::Matrix3d::Identity());
    striker.setCollisionShape(sx::CollisionShape::makeSphere(0.2));
    auto joint = striker.getParentJoint();
    joint.setVelocity(Eigen::VectorXd::Constant(1, 1.0));

    sx::RigidBodyOptions targetOptions;
    targetOptions.position = Eigen::Vector3d(0.399, y, 0.0);
    targetOptions.mass = 1.0;
    targetOptions.inertia = Eigen::Matrix3d::Identity();
    auto target = world->addRigidBody(
        pairCount == 1 ? "target" : "target_" + std::to_string(i),
        targetOptions);
    target.setCollisionShape(sx::CollisionShape::makeSphere(0.2));
  }

  return world;
}

std::unique_ptr<sx::World> buildArticulatedLinkImpactScene(
    sx::ContactSolverMethod method, int pairCount = 1)
{
  EXPECT_GT(pairCount, 0);
  sx::WorldOptions options;
  options.timeStep = 0.001;
  options.gravity = Eigen::Vector3d::Zero();
  options.contactSolverMethod = method;
  auto world = std::make_unique<sx::World>(options);

  constexpr double kSpacing = 1.0;
  for (int i = 0; i < pairCount; ++i) {
    const double y = kSpacing * static_cast<double>(i);

    auto strikerRobot = world->addMultibody(
        pairCount == 1 ? "striker_robot"
                       : "striker_robot_" + std::to_string(i));
    auto strikerBase = strikerRobot.addLink("base");
    sx::JointSpec strikerSpec;
    strikerSpec.name = "rail";
    strikerSpec.type = sx::JointType::Prismatic;
    strikerSpec.axis = Eigen::Vector3d::UnitX();
    strikerSpec.transformFromParent = Eigen::Isometry3d::Identity();
    strikerSpec.transformFromParent.translation()
        = Eigen::Vector3d(0.0, y, 0.0);
    auto striker = strikerRobot.addLink("striker", strikerBase, strikerSpec);
    striker.setMass(2.0);
    striker.setInertia(Eigen::Matrix3d::Identity());
    striker.setCollisionShape(sx::CollisionShape::makeSphere(0.2));
    striker.getParentJoint().setVelocity(Eigen::VectorXd::Constant(1, 1.0));

    auto targetRobot = world->addMultibody(
        pairCount == 1 ? "target_robot" : "target_robot_" + std::to_string(i));
    auto targetBase = targetRobot.addLink("base");
    sx::JointSpec targetSpec;
    targetSpec.name = "rail";
    targetSpec.type = sx::JointType::Prismatic;
    targetSpec.axis = Eigen::Vector3d::UnitX();
    targetSpec.transformFromParent = Eigen::Isometry3d::Identity();
    targetSpec.transformFromParent.translation()
        = Eigen::Vector3d(0.399, y, 0.0);
    auto target = targetRobot.addLink("target", targetBase, targetSpec);
    target.setMass(1.0);
    target.setInertia(Eigen::Matrix3d::Identity());
    target.setCollisionShape(sx::CollisionShape::makeSphere(0.2));
  }

  return world;
}

std::unique_ptr<sx::World> buildCartesianArticulatedGroundScene(
    sx::ContactSolverMethod method, int chainCount = 1)
{
  if (chainCount <= 0) {
    return nullptr;
  }

  sx::WorldOptions options;
  options.timeStep = 0.002;
  options.gravity = Eigen::Vector3d(0.0, 0.0, -9.81);
  options.contactSolverMethod = method;
  auto world = std::make_unique<sx::World>(options);

  sx::RigidBodyOptions groundOptions;
  groundOptions.isStatic = true;
  groundOptions.position = Eigen::Vector3d(0.0, 0.0, -1.0);
  auto ground = world->addRigidBody("ground", groundOptions);
  ground.setCollisionShape(
      sx::CollisionShape::makeBox(Eigen::Vector3d(24.0, 24.0, 0.5)));

  const int columns
      = static_cast<int>(std::ceil(std::sqrt(static_cast<double>(chainCount))));
  constexpr double kSpacing = 1.5;
  for (int i = 0; i < chainCount; ++i) {
    const int row = i / columns;
    const int col = i - row * columns;
    auto robot = world->addMultibody(
        chainCount == 1 ? "cartesian_robot"
                        : "cartesian_robot_" + std::to_string(i));
    auto base = robot.addLink("base");

    sx::JointSpec xSpec;
    xSpec.name = "x_slider";
    xSpec.type = sx::JointType::Prismatic;
    xSpec.axis = Eigen::Vector3d::UnitX();
    xSpec.transformFromParent = Eigen::Isometry3d::Identity();
    xSpec.transformFromParent.translation() = Eigen::Vector3d(
        kSpacing * static_cast<double>(col),
        kSpacing * static_cast<double>(row),
        0.0);
    auto xLink = robot.addLink("x_link", base, xSpec);
    xLink.setMass(0.25);
    xLink.setInertia(0.05 * Eigen::Matrix3d::Identity());
    xLink.getParentJoint().setPosition(Eigen::VectorXd::Constant(1, 0.10));
    xLink.getParentJoint().setVelocity(Eigen::VectorXd::Constant(1, 0.04));

    sx::JointSpec ySpec;
    ySpec.name = "y_slider";
    ySpec.type = sx::JointType::Prismatic;
    ySpec.axis = Eigen::Vector3d::UnitY();
    auto yLink = robot.addLink("y_link", xLink, ySpec);
    yLink.setMass(0.25);
    yLink.setInertia(0.05 * Eigen::Matrix3d::Identity());
    yLink.getParentJoint().setPosition(Eigen::VectorXd::Constant(1, -0.10));
    yLink.getParentJoint().setVelocity(Eigen::VectorXd::Constant(1, -0.03));

    sx::JointSpec zSpec;
    zSpec.name = "z_slider";
    zSpec.type = sx::JointType::Prismatic;
    zSpec.axis = Eigen::Vector3d::UnitZ();
    auto tip = robot.addLink("tip", yLink, zSpec);
    tip.setMass(1.0);
    tip.setInertia(Eigen::Matrix3d::Identity());
    tip.setCollisionShape(sx::CollisionShape::makeSphere(0.2));
    tip.getParentJoint().setPosition(Eigen::VectorXd::Constant(1, -0.305));
    tip.getParentJoint().setVelocity(Eigen::VectorXd::Constant(1, -0.05));
  }

  return world;
}

std::unique_ptr<sx::World> buildSphereStackScene(
    int sphereCount, double friction, bool snapshotVelocities)
{
  sx::WorldOptions options;
  options.timeStep = (!snapshotVelocities && sphereCount >= 4) ? 0.001 : 0.005;
  options.gravity = Eigen::Vector3d(0.0, 0.0, -9.81);
  options.contactSolverMethod = sx::ContactSolverMethod::BoxedLcp;
  auto world = std::make_unique<sx::World>(options);

  sx::RigidBodyOptions groundOptions;
  groundOptions.isStatic = true;
  groundOptions.position = Eigen::Vector3d(0.0, 0.0, -0.5);
  auto ground = world->addRigidBody("ground", groundOptions);
  ground.setCollisionShape(
      sx::CollisionShape::makeBox(Eigen::Vector3d(8.0, 8.0, 0.5)));
  ground.setFriction(friction);

  for (int i = 0; i < sphereCount; ++i) {
    sx::RigidBodyOptions sphereOptions;
    sphereOptions.position
        = Eigen::Vector3d(0.0, 0.0, 0.5 + static_cast<double>(i));
    if (snapshotVelocities) {
      sphereOptions.linearVelocity = Eigen::Vector3d(
          0.18 - 0.06 * static_cast<double>(i),
          -0.12 + 0.05 * static_cast<double>(i),
          -0.16 - 0.08 * static_cast<double>(i));
    } else {
      sphereOptions.linearVelocity
          = Eigen::Vector3d(0.0, 0.0, -0.02 - 0.02 * static_cast<double>(i));
    }
    auto sphere = world->addRigidBody(
        "stack_sphere_" + std::to_string(i), sphereOptions);
    sphere.setCollisionShape(sx::CollisionShape::makeSphere(0.5));
    sphere.setFriction(friction);
  }

  return world;
}

double maxNormalContactCoupling(
    const sx::detail::BoxedLcpContactSnapshot& snapshot)
{
  double maxNormalCoupling = 0.0;
  for (Eigen::Index r = 0; r < static_cast<Eigen::Index>(snapshot.contactCount);
       ++r) {
    for (Eigen::Index c = 0;
         c < static_cast<Eigen::Index>(snapshot.contactCount);
         ++c) {
      if (r != c) {
        maxNormalCoupling
            = std::max(maxNormalCoupling, std::abs(snapshot.A(r, c)));
      }
    }
  }
  return maxNormalCoupling;
}

void expectSphereStackStepInvariants(sx::World& world, int sphereCount)
{
  const auto ground = world.getRigidBody("ground");
  ASSERT_TRUE(ground.has_value());
  EXPECT_TRUE(
      ground->getTranslation().isApprox(Eigen::Vector3d(0.0, 0.0, -0.5)));

  for (int i = 0; i < sphereCount; ++i) {
    const auto sphere = world.getRigidBody("stack_sphere_" + std::to_string(i));
    ASSERT_TRUE(sphere.has_value());
    SCOPED_TRACE("sphere " + std::to_string(i));
    EXPECT_TRUE(sphere->getTranslation().allFinite());
    EXPECT_TRUE(sphere->getLinearVelocity().allFinite());
    EXPECT_GE(
        sphere->getTranslation().z(), 0.5 + static_cast<double>(i) - 2e-2);
    EXPECT_NEAR(
        sphere->getTranslation().z(), 0.5 + static_cast<double>(i), 6e-2);
    EXPECT_LT(std::abs(sphere->getLinearVelocity().z()), 0.15);
    EXPECT_LT(sphere->getTranslation().head<2>().norm(), 1e-3);
    EXPECT_LT(sphere->getLinearVelocity().head<2>().norm(), 1e-3);
  }

  for (int i = 1; i < sphereCount; ++i) {
    const auto upper = world.getRigidBody("stack_sphere_" + std::to_string(i));
    const auto lower
        = world.getRigidBody("stack_sphere_" + std::to_string(i - 1));
    ASSERT_TRUE(upper.has_value());
    ASSERT_TRUE(lower.has_value());
    const double spacing
        = upper->getTranslation().z() - lower->getTranslation().z();
    SCOPED_TRACE("spacing " + std::to_string(i - 1) + "-" + std::to_string(i));
    EXPECT_GE(spacing, 1.0 - 4e-2);
    EXPECT_NEAR(spacing, 1.0, 8e-2);
  }
}

struct ArticulatedGroundStepResult
{
  double linkZ{0.0};
  double jointVelocity{0.0};
  std::size_t contactCount{0};
  bool contactTouchesLink{false};
};

struct MultiArticulatedGroundStepResult
{
  double maxHeightError{0.0};
  double maxAbsJointVelocity{0.0};
  std::size_t contactCount{0};
  std::size_t linkContactCount{0};
  bool allFinite{true};
};

struct CartesianArticulatedGroundStepResult
{
  double maxHeightError{0.0};
  double maxAbsJointVelocity{0.0};
  double maxPlanarJointSpeed{0.0};
  std::size_t contactCount{0};
  std::size_t linkContactCount{0};
  std::size_t dofCount{0};
  bool allFinite{true};
};

struct ArticulatedRigidImpactResult
{
  double strikerVelocity{0.0};
  double targetVelocity{0.0};
  double momentum{0.0};
  std::size_t contactCount{0};
  bool contactTouchesLink{false};
  bool contactTouchesRigidBody{false};
};

struct MultiArticulatedRigidImpactResult
{
  double maxMomentumError{0.0};
  double minTargetVelocity{0.0};
  double maxStrikerVelocity{0.0};
  std::size_t contactCount{0};
  std::size_t linkContactCount{0};
  std::size_t rigidBodyContactCount{0};
  bool allFinite{true};
};

struct ArticulatedLinkImpactResult
{
  double strikerVelocity{0.0};
  double targetVelocity{0.0};
  double momentum{0.0};
  std::size_t contactCount{0};
  bool contactTouchesTwoLinks{false};
};

struct MultiArticulatedLinkImpactResult
{
  double maxMomentumError{0.0};
  double minTargetVelocity{0.0};
  double maxStrikerVelocity{0.0};
  double minRelativeVelocity{0.0};
  std::size_t contactCount{0};
  std::size_t twoLinkContactCount{0};
  bool allFinite{true};
};

ArticulatedGroundStepResult runArticulatedGroundStep(
    sx::ContactSolverMethod method)
{
  auto world = buildArticulatedGroundScene(method);
  world->enterSimulationMode();

  const auto contacts = world->collide();
  auto& registry = dart::simulation::detail::registryOf(*world);
  bool contactTouchesLink = false;
  for (const auto& contact : contacts) {
    const entt::entity entityA
        = sx::detail::toRegistryEntity(contact.bodyA.getEntity());
    const entt::entity entityB
        = sx::detail::toRegistryEntity(contact.bodyB.getEntity());
    contactTouchesLink = contactTouchesLink
                         || registry.all_of<sx::comps::Link>(entityA)
                         || registry.all_of<sx::comps::Link>(entityB);
  }

  world->step(400);

  const auto robot = world->getMultibody("leg_robot");
  EXPECT_TRUE(robot.has_value());
  if (!robot.has_value()) {
    return {};
  }
  const auto leg = robot->getLink("leg");
  EXPECT_TRUE(leg.has_value());
  if (!leg.has_value()) {
    return {};
  }

  const auto joint = leg->getParentJoint();
  return ArticulatedGroundStepResult{
      .linkZ = leg->getWorldTransform().translation().z(),
      .jointVelocity = joint.getVelocity()[0],
      .contactCount = contacts.size(),
      .contactTouchesLink = contactTouchesLink,
  };
}

MultiArticulatedGroundStepResult runMultiArticulatedGroundStep(
    sx::ContactSolverMethod method, int linkCount)
{
  auto world = buildArticulatedGroundScene(method, linkCount);
  EXPECT_TRUE(world != nullptr);
  if (world == nullptr) {
    return {};
  }
  world->enterSimulationMode();

  const auto contacts = world->collide();
  auto& registry = dart::simulation::detail::registryOf(*world);
  std::size_t linkContactCount = 0;
  for (const auto& contact : contacts) {
    const entt::entity entityA
        = sx::detail::toRegistryEntity(contact.bodyA.getEntity());
    const entt::entity entityB
        = sx::detail::toRegistryEntity(contact.bodyB.getEntity());
    if (registry.all_of<sx::comps::Link>(entityA)
        || registry.all_of<sx::comps::Link>(entityB)) {
      ++linkContactCount;
    }
  }

  world->step(200);

  MultiArticulatedGroundStepResult result;
  result.contactCount = contacts.size();
  result.linkContactCount = linkContactCount;
  for (int i = 0; i < linkCount; ++i) {
    auto robot = world->getMultibody("leg_robot_" + std::to_string(i));
    EXPECT_TRUE(robot.has_value());
    if (!robot.has_value()) {
      result.allFinite = false;
      continue;
    }
    auto leg = robot->getLink("leg");
    EXPECT_TRUE(leg.has_value());
    if (!leg.has_value()) {
      result.allFinite = false;
      continue;
    }

    const auto joint = leg->getParentJoint();
    const double linkZ = leg->getWorldTransform().translation().z();
    const double jointVelocity = joint.getVelocity()[0];
    result.allFinite = result.allFinite && std::isfinite(linkZ)
                       && std::isfinite(jointVelocity);
    result.maxHeightError
        = std::max(result.maxHeightError, std::abs(linkZ + 0.3));
    result.maxAbsJointVelocity
        = std::max(result.maxAbsJointVelocity, std::abs(jointVelocity));
  }

  return result;
}

CartesianArticulatedGroundStepResult runCartesianArticulatedGroundStep(
    sx::ContactSolverMethod method, int chainCount)
{
  auto world = buildCartesianArticulatedGroundScene(method, chainCount);
  EXPECT_TRUE(world != nullptr);
  if (world == nullptr) {
    return {};
  }
  world->enterSimulationMode();

  const auto contacts = world->collide();
  auto& registry = dart::simulation::detail::registryOf(*world);
  std::size_t linkContactCount = 0;
  for (const auto& contact : contacts) {
    const entt::entity entityA
        = sx::detail::toRegistryEntity(contact.bodyA.getEntity());
    const entt::entity entityB
        = sx::detail::toRegistryEntity(contact.bodyB.getEntity());
    if (registry.all_of<sx::comps::Link>(entityA)
        || registry.all_of<sx::comps::Link>(entityB)) {
      ++linkContactCount;
    }
  }

  world->step(200);

  CartesianArticulatedGroundStepResult result;
  result.contactCount = contacts.size();
  result.linkContactCount = linkContactCount;
  for (int i = 0; i < chainCount; ++i) {
    auto robot = world->getMultibody(
        chainCount == 1 ? "cartesian_robot"
                        : "cartesian_robot_" + std::to_string(i));
    EXPECT_TRUE(robot.has_value());
    if (!robot.has_value()) {
      result.allFinite = false;
      continue;
    }
    auto xLink = robot->getLink("x_link");
    auto yLink = robot->getLink("y_link");
    auto tip = robot->getLink("tip");
    EXPECT_TRUE(xLink.has_value());
    EXPECT_TRUE(yLink.has_value());
    EXPECT_TRUE(tip.has_value());
    if (!xLink.has_value() || !yLink.has_value() || !tip.has_value()) {
      result.allFinite = false;
      continue;
    }

    const double xVelocity = xLink->getParentJoint().getVelocity()[0];
    const double yVelocity = yLink->getParentJoint().getVelocity()[0];
    const double zVelocity = tip->getParentJoint().getVelocity()[0];
    const double tipZ = tip->getWorldTransform().translation().z();
    const double planarSpeed = std::hypot(xVelocity, yVelocity);
    result.dofCount += 3u;
    result.allFinite = result.allFinite && std::isfinite(xVelocity)
                       && std::isfinite(yVelocity) && std::isfinite(zVelocity)
                       && std::isfinite(tipZ);
    result.maxHeightError
        = std::max(result.maxHeightError, std::abs(tipZ + 0.3));
    result.maxAbsJointVelocity = std::max(
        result.maxAbsJointVelocity,
        std::max(
            {std::abs(xVelocity), std::abs(yVelocity), std::abs(zVelocity)}));
    result.maxPlanarJointSpeed
        = std::max(result.maxPlanarJointSpeed, planarSpeed);
  }

  return result;
}

void expectCartesianPrismaticChainsGroundStepMaintainsInvariants(int chainCount)
{
  const std::size_t dofCount = 3u * static_cast<std::size_t>(chainCount);

  const CartesianArticulatedGroundStepResult reference
      = runCartesianArticulatedGroundStep(
          sx::ContactSolverMethod::SequentialImpulse, chainCount);
  const CartesianArticulatedGroundStepResult lcp
      = runCartesianArticulatedGroundStep(
          sx::ContactSolverMethod::BoxedLcp, chainCount);

  ASSERT_EQ(lcp.contactCount, static_cast<std::size_t>(chainCount));
  EXPECT_EQ(lcp.linkContactCount, static_cast<std::size_t>(chainCount));
  EXPECT_EQ(lcp.dofCount, dofCount);
  EXPECT_TRUE(lcp.allFinite);
  EXPECT_LE(lcp.maxHeightError, 2e-2);
  EXPECT_LT(lcp.maxAbsJointVelocity, 0.12);
  EXPECT_LT(lcp.maxPlanarJointSpeed, 0.08);

  ASSERT_EQ(reference.contactCount, lcp.contactCount);
  EXPECT_EQ(reference.linkContactCount, lcp.linkContactCount);
  EXPECT_EQ(reference.dofCount, lcp.dofCount);
  EXPECT_TRUE(reference.allFinite);
  EXPECT_NEAR(reference.maxHeightError, lcp.maxHeightError, 2e-2);
  EXPECT_NEAR(reference.maxAbsJointVelocity, lcp.maxAbsJointVelocity, 0.12);
  EXPECT_NEAR(reference.maxPlanarJointSpeed, lcp.maxPlanarJointSpeed, 0.08);
}

ArticulatedRigidImpactResult runArticulatedRigidImpactStep(
    sx::ContactSolverMethod method)
{
  auto world = buildArticulatedRigidImpactScene(method);
  world->enterSimulationMode();

  const auto contacts = world->collide();
  auto& registry = dart::simulation::detail::registryOf(*world);
  bool contactTouchesLink = false;
  bool contactTouchesRigidBody = false;
  for (const auto& contact : contacts) {
    const entt::entity entityA
        = sx::detail::toRegistryEntity(contact.bodyA.getEntity());
    const entt::entity entityB
        = sx::detail::toRegistryEntity(contact.bodyB.getEntity());
    contactTouchesLink = contactTouchesLink
                         || registry.all_of<sx::comps::Link>(entityA)
                         || registry.all_of<sx::comps::Link>(entityB);
    contactTouchesRigidBody
        = contactTouchesRigidBody
          || registry.all_of<sx::comps::RigidBodyTag>(entityA)
          || registry.all_of<sx::comps::RigidBodyTag>(entityB);
  }

  world->step();

  const auto robot = world->getMultibody("striker_robot");
  EXPECT_TRUE(robot.has_value());
  if (!robot.has_value()) {
    return {};
  }
  const auto striker = robot->getLink("striker");
  EXPECT_TRUE(striker.has_value());
  const auto target = world->getRigidBody("target");
  EXPECT_TRUE(target.has_value());
  if (!striker.has_value() || !target.has_value()) {
    return {};
  }

  const double strikerVelocity = striker->getParentJoint().getVelocity()[0];
  const double targetVelocity = target->getLinearVelocity().x();
  return ArticulatedRigidImpactResult{
      .strikerVelocity = strikerVelocity,
      .targetVelocity = targetVelocity,
      .momentum = 2.0 * strikerVelocity + targetVelocity,
      .contactCount = contacts.size(),
      .contactTouchesLink = contactTouchesLink,
      .contactTouchesRigidBody = contactTouchesRigidBody,
  };
}

MultiArticulatedRigidImpactResult runMultiArticulatedRigidImpactStep(
    sx::ContactSolverMethod method, int pairCount, int stepCount = 1)
{
  EXPECT_GT(stepCount, 0);
  auto world = buildArticulatedRigidImpactScene(method, pairCount);
  EXPECT_TRUE(world != nullptr);
  if (world == nullptr) {
    return {};
  }
  world->enterSimulationMode();

  const auto contacts = world->collide();
  auto& registry = dart::simulation::detail::registryOf(*world);
  std::size_t linkContactCount = 0;
  std::size_t rigidBodyContactCount = 0;
  for (const auto& contact : contacts) {
    const entt::entity entityA
        = sx::detail::toRegistryEntity(contact.bodyA.getEntity());
    const entt::entity entityB
        = sx::detail::toRegistryEntity(contact.bodyB.getEntity());
    if (registry.all_of<sx::comps::Link>(entityA)
        || registry.all_of<sx::comps::Link>(entityB)) {
      ++linkContactCount;
    }
    if (registry.all_of<sx::comps::RigidBodyTag>(entityA)
        || registry.all_of<sx::comps::RigidBodyTag>(entityB)) {
      ++rigidBodyContactCount;
    }
  }

  world->step(stepCount);

  MultiArticulatedRigidImpactResult result;
  result.contactCount = contacts.size();
  result.linkContactCount = linkContactCount;
  result.rigidBodyContactCount = rigidBodyContactCount;
  for (int i = 0; i < pairCount; ++i) {
    auto robot = world->getMultibody("striker_robot_" + std::to_string(i));
    auto target = world->getRigidBody("target_" + std::to_string(i));
    EXPECT_TRUE(robot.has_value());
    EXPECT_TRUE(target.has_value());
    if (!robot.has_value() || !target.has_value()) {
      result.allFinite = false;
      continue;
    }
    auto striker = robot->getLink("striker");
    EXPECT_TRUE(striker.has_value());
    if (!striker.has_value()) {
      result.allFinite = false;
      continue;
    }

    const double strikerVelocity = striker->getParentJoint().getVelocity()[0];
    const double targetVelocity = target->getLinearVelocity().x();
    const double momentum = 2.0 * strikerVelocity + targetVelocity;
    result.allFinite = result.allFinite && std::isfinite(strikerVelocity)
                       && std::isfinite(targetVelocity);
    result.maxMomentumError
        = std::max(result.maxMomentumError, std::abs(momentum - 2.0));
    if (i == 0) {
      result.minTargetVelocity = targetVelocity;
      result.maxStrikerVelocity = strikerVelocity;
    } else {
      result.minTargetVelocity
          = std::min(result.minTargetVelocity, targetVelocity);
      result.maxStrikerVelocity
          = std::max(result.maxStrikerVelocity, strikerVelocity);
    }
  }

  return result;
}

void expectArticulatedRigidImpactPairsStepMaintainsInvariants(
    int pairCount, int stepCount = 1)
{
  const MultiArticulatedRigidImpactResult reference
      = runMultiArticulatedRigidImpactStep(
          sx::ContactSolverMethod::SequentialImpulse, pairCount, stepCount);
  const MultiArticulatedRigidImpactResult lcp
      = runMultiArticulatedRigidImpactStep(
          sx::ContactSolverMethod::BoxedLcp, pairCount, stepCount);

  ASSERT_EQ(lcp.contactCount, static_cast<std::size_t>(pairCount));
  EXPECT_EQ(lcp.linkContactCount, static_cast<std::size_t>(pairCount));
  EXPECT_EQ(lcp.rigidBodyContactCount, static_cast<std::size_t>(pairCount));
  EXPECT_TRUE(lcp.allFinite);
  EXPECT_LT(lcp.maxStrikerVelocity, 1.0);
  EXPECT_GT(lcp.minTargetVelocity, 0.0);
  EXPECT_NEAR(lcp.maxMomentumError, 0.0, 1e-9);

  ASSERT_EQ(reference.contactCount, lcp.contactCount);
  EXPECT_EQ(reference.linkContactCount, lcp.linkContactCount);
  EXPECT_EQ(reference.rigidBodyContactCount, lcp.rigidBodyContactCount);
  EXPECT_TRUE(reference.allFinite);
  EXPECT_NEAR(reference.maxStrikerVelocity, lcp.maxStrikerVelocity, 1e-9);
  EXPECT_NEAR(reference.minTargetVelocity, lcp.minTargetVelocity, 1e-9);
  EXPECT_NEAR(reference.maxMomentumError, lcp.maxMomentumError, 1e-9);
}

ArticulatedLinkImpactResult runArticulatedLinkImpactStep(
    sx::ContactSolverMethod method)
{
  auto world = buildArticulatedLinkImpactScene(method);
  world->enterSimulationMode();

  const auto contacts = world->collide();
  auto& registry = dart::simulation::detail::registryOf(*world);
  bool contactTouchesTwoLinks = false;
  for (const auto& contact : contacts) {
    const entt::entity entityA
        = sx::detail::toRegistryEntity(contact.bodyA.getEntity());
    const entt::entity entityB
        = sx::detail::toRegistryEntity(contact.bodyB.getEntity());
    contactTouchesTwoLinks = contactTouchesTwoLinks
                             || (registry.all_of<sx::comps::Link>(entityA)
                                 && registry.all_of<sx::comps::Link>(entityB));
  }

  world->step();

  const auto strikerRobot = world->getMultibody("striker_robot");
  const auto targetRobot = world->getMultibody("target_robot");
  EXPECT_TRUE(strikerRobot.has_value());
  EXPECT_TRUE(targetRobot.has_value());
  if (!strikerRobot.has_value() || !targetRobot.has_value()) {
    return {};
  }
  const auto striker = strikerRobot->getLink("striker");
  const auto target = targetRobot->getLink("target");
  EXPECT_TRUE(striker.has_value());
  EXPECT_TRUE(target.has_value());
  if (!striker.has_value() || !target.has_value()) {
    return {};
  }

  const double strikerVelocity = striker->getParentJoint().getVelocity()[0];
  const double targetVelocity = target->getParentJoint().getVelocity()[0];
  return ArticulatedLinkImpactResult{
      .strikerVelocity = strikerVelocity,
      .targetVelocity = targetVelocity,
      .momentum = 2.0 * strikerVelocity + targetVelocity,
      .contactCount = contacts.size(),
      .contactTouchesTwoLinks = contactTouchesTwoLinks,
  };
}

MultiArticulatedLinkImpactResult runMultiArticulatedLinkImpactStep(
    sx::ContactSolverMethod method, int pairCount, int stepCount = 1)
{
  EXPECT_GT(stepCount, 0);
  auto world = buildArticulatedLinkImpactScene(method, pairCount);
  EXPECT_TRUE(world != nullptr);
  if (world == nullptr) {
    return {};
  }
  world->enterSimulationMode();

  const auto contacts = world->collide();
  auto& registry = dart::simulation::detail::registryOf(*world);
  std::size_t twoLinkContactCount = 0;
  for (const auto& contact : contacts) {
    const entt::entity entityA
        = sx::detail::toRegistryEntity(contact.bodyA.getEntity());
    const entt::entity entityB
        = sx::detail::toRegistryEntity(contact.bodyB.getEntity());
    if (registry.all_of<sx::comps::Link>(entityA)
        && registry.all_of<sx::comps::Link>(entityB)) {
      ++twoLinkContactCount;
    }
  }

  world->step(stepCount);

  MultiArticulatedLinkImpactResult result;
  result.contactCount = contacts.size();
  result.twoLinkContactCount = twoLinkContactCount;
  for (int i = 0; i < pairCount; ++i) {
    auto strikerRobot
        = world->getMultibody("striker_robot_" + std::to_string(i));
    auto targetRobot = world->getMultibody("target_robot_" + std::to_string(i));
    EXPECT_TRUE(strikerRobot.has_value());
    EXPECT_TRUE(targetRobot.has_value());
    if (!strikerRobot.has_value() || !targetRobot.has_value()) {
      result.allFinite = false;
      continue;
    }
    auto striker = strikerRobot->getLink("striker");
    auto target = targetRobot->getLink("target");
    EXPECT_TRUE(striker.has_value());
    EXPECT_TRUE(target.has_value());
    if (!striker.has_value() || !target.has_value()) {
      result.allFinite = false;
      continue;
    }

    const double strikerVelocity = striker->getParentJoint().getVelocity()[0];
    const double targetVelocity = target->getParentJoint().getVelocity()[0];
    const double relativeVelocity = targetVelocity - strikerVelocity;
    const double momentum = 2.0 * strikerVelocity + targetVelocity;
    result.allFinite = result.allFinite && std::isfinite(strikerVelocity)
                       && std::isfinite(targetVelocity);
    result.maxMomentumError
        = std::max(result.maxMomentumError, std::abs(momentum - 2.0));
    if (i == 0) {
      result.minTargetVelocity = targetVelocity;
      result.maxStrikerVelocity = strikerVelocity;
      result.minRelativeVelocity = relativeVelocity;
    } else {
      result.minTargetVelocity
          = std::min(result.minTargetVelocity, targetVelocity);
      result.maxStrikerVelocity
          = std::max(result.maxStrikerVelocity, strikerVelocity);
      result.minRelativeVelocity
          = std::min(result.minRelativeVelocity, relativeVelocity);
    }
  }

  return result;
}

void expectArticulatedLinkImpactPairsStepMaintainsInvariants(
    int pairCount, int stepCount = 1)
{
  const MultiArticulatedLinkImpactResult reference
      = runMultiArticulatedLinkImpactStep(
          sx::ContactSolverMethod::SequentialImpulse, pairCount, stepCount);
  const MultiArticulatedLinkImpactResult lcp
      = runMultiArticulatedLinkImpactStep(
          sx::ContactSolverMethod::BoxedLcp, pairCount, stepCount);

  ASSERT_EQ(lcp.contactCount, static_cast<std::size_t>(pairCount));
  EXPECT_EQ(lcp.twoLinkContactCount, static_cast<std::size_t>(pairCount));
  EXPECT_TRUE(lcp.allFinite);
  EXPECT_LT(lcp.maxStrikerVelocity, 1.0);
  EXPECT_GT(lcp.minTargetVelocity, 0.0);
  EXPECT_GE(lcp.minRelativeVelocity, -1e-9);
  EXPECT_NEAR(lcp.maxMomentumError, 0.0, 1e-9);

  ASSERT_EQ(reference.contactCount, lcp.contactCount);
  EXPECT_EQ(reference.twoLinkContactCount, lcp.twoLinkContactCount);
  EXPECT_TRUE(reference.allFinite);
  EXPECT_NEAR(reference.maxStrikerVelocity, lcp.maxStrikerVelocity, 1e-9);
  EXPECT_NEAR(reference.minTargetVelocity, lcp.minTargetVelocity, 1e-9);
  EXPECT_NEAR(reference.minRelativeVelocity, lcp.minRelativeVelocity, 1e-9);
  EXPECT_NEAR(reference.maxMomentumError, lcp.maxMomentumError, 1e-9);
}

void expectSeparatedSphereStepInvariants(sx::World& world, int sphereCount)
{
  const auto ground = world.getRigidBody("ground");
  ASSERT_TRUE(ground.has_value());
  EXPECT_TRUE(
      ground->getTranslation().isApprox(Eigen::Vector3d(0.0, 0.0, -0.5)));

  for (int i = 0; i < sphereCount; ++i) {
    const auto sphere = world.getRigidBody("sphere_" + std::to_string(i));
    ASSERT_TRUE(sphere.has_value());
    SCOPED_TRACE("sphere " + std::to_string(i));

    const Eigen::Vector2d initialTangentialVelocity(
        0.35 - 0.03 * static_cast<double>(i % 4),
        -0.22 + 0.025 * static_cast<double>(i % 5));
    EXPECT_TRUE(sphere->getTranslation().allFinite());
    EXPECT_TRUE(sphere->getLinearVelocity().allFinite());
    EXPECT_GE(sphere->getTranslation().z(), 0.5 - 1e-3);
    EXPECT_NEAR(sphere->getTranslation().z(), 0.5, 2e-2);
    EXPECT_LT(std::abs(sphere->getLinearVelocity().z()), 0.1);
    EXPECT_LT(
        sphere->getLinearVelocity().head<2>().norm(),
        initialTangentialVelocity.norm());
  }
}

void expectSeparatedBoxStepInvariants(sx::World& world, int boxCount)
{
  const auto ground = world.getRigidBody("ground");
  ASSERT_TRUE(ground.has_value());
  EXPECT_TRUE(
      ground->getTranslation().isApprox(Eigen::Vector3d(0.0, 0.0, -0.5)));

  for (int i = 0; i < boxCount; ++i) {
    const auto box = world.getRigidBody("box_" + std::to_string(i));
    ASSERT_TRUE(box.has_value());
    SCOPED_TRACE("box " + std::to_string(i));

    const Eigen::Vector2d initialTangentialVelocity(
        0.35 - 0.015 * static_cast<double>(i % 3),
        -0.2 + 0.01 * static_cast<double>(i % 2));
    EXPECT_TRUE(box->getTranslation().allFinite());
    EXPECT_TRUE(box->getLinearVelocity().allFinite());
    EXPECT_TRUE(box->getAngularVelocity().allFinite());
    EXPECT_GE(box->getTranslation().z(), 0.5 - 1e-3);
    EXPECT_NEAR(box->getTranslation().z(), 0.5, 2e-2);
    EXPECT_LT(std::abs(box->getLinearVelocity().z()), 0.1);
    EXPECT_LT(
        box->getLinearVelocity().head<2>().norm(),
        initialTangentialVelocity.norm());
  }
}

void expectSeparatedBoxDenseStepSmokeInvariants(sx::World& world, int boxCount)
{
  const auto ground = world.getRigidBody("ground");
  ASSERT_TRUE(ground.has_value());
  EXPECT_TRUE(
      ground->getTranslation().isApprox(Eigen::Vector3d(0.0, 0.0, -0.5)));

  const std::vector<sx::Contact> contacts = world.collide();
  EXPECT_EQ(contacts.size(), static_cast<std::size_t>(4 * boxCount));

  for (int i = 0; i < boxCount; ++i) {
    const auto box = world.getRigidBody("box_" + std::to_string(i));
    ASSERT_TRUE(box.has_value());
    SCOPED_TRACE("box " + std::to_string(i));

    EXPECT_TRUE(box->getTranslation().allFinite());
    EXPECT_TRUE(box->getLinearVelocity().allFinite());
    EXPECT_TRUE(box->getAngularVelocity().allFinite());
    EXPECT_GE(box->getTranslation().z(), 0.5 - 1e-3);
    EXPECT_NEAR(box->getTranslation().z(), 0.5, 2e-2);
  }
}

void expectBoxedFrictionIndexShape(
    const sx::detail::BoxedLcpContactSnapshot& snapshot,
    double friction,
    std::size_t expectedContactCount,
    std::size_t expectedBodyCount)
{
  ASSERT_EQ(snapshot.contactCount, expectedContactCount);
  ASSERT_EQ(
      snapshot.size(), static_cast<Eigen::Index>(3 * snapshot.contactCount));
  ASSERT_EQ(snapshot.bodyCount, expectedBodyCount);
  ASSERT_EQ(snapshot.A.rows(), snapshot.size());
  ASSERT_EQ(snapshot.A.cols(), snapshot.size());
  ASSERT_EQ(snapshot.J.rows(), snapshot.size());
  ASSERT_EQ(
      snapshot.J.cols(), static_cast<Eigen::Index>(6 * snapshot.bodyCount));

  for (std::size_t contact = 0; contact < snapshot.contactCount; ++contact) {
    const Eigen::Index normalRow = static_cast<Eigen::Index>(contact);
    EXPECT_EQ(snapshot.findex[normalRow], -1);
    EXPECT_EQ(snapshot.lo[normalRow], 0.0);
    EXPECT_TRUE(std::isinf(snapshot.hi[normalRow]));

    for (Eigen::Index tangent = 0; tangent < 2; ++tangent) {
      const Eigen::Index row = static_cast<Eigen::Index>(
          snapshot.contactCount + 2 * contact + tangent);
      EXPECT_EQ(snapshot.findex[row], normalRow);
      EXPECT_NEAR(snapshot.lo[row], -friction, 1e-12);
      EXPECT_NEAR(snapshot.hi[row], friction, 1e-12);
    }
  }
}

void expectBoxedFrictionIndexSnapshot(
    const sx::detail::BoxedLcpContactSnapshot& snapshot,
    double friction,
    std::size_t expectedContactCount,
    std::size_t expectedBodyCount)
{
  expectBoxedFrictionIndexShape(
      snapshot, friction, expectedContactCount, expectedBodyCount);

  dart::math::LcpOptions checkOptions;
  checkOptions.absoluteTolerance = 1e-7;
  checkOptions.relativeTolerance = 1e-3;
  checkOptions.complementarityTolerance = 1e-3;
  const dart::math::LcpProblem problem(
      snapshot.A, snapshot.b, snapshot.lo, snapshot.hi, snapshot.findex);
  const dart::test::LcpCheckResult check
      = dart::test::CheckLcpSolution(problem, snapshot.f, checkOptions);
  EXPECT_TRUE(check.ok) << "message=" << check.message
                        << " residual=" << check.residual
                        << " complementarity=" << check.complementarity
                        << " boundViolation=" << check.boundViolation
                        << " tol=" << check.tol << " compTol=" << check.compTol;
}

void expectBlockSolverPassesWorldContactSnapshot(
    dart::math::LcpSolver& solver,
    const sx::detail::BoxedLcpContactSnapshot& snapshot)
{
  const dart::math::LcpProblem problem(
      snapshot.A, snapshot.b, snapshot.lo, snapshot.hi, snapshot.findex);

  dart::math::LcpOptions options;
  options.maxIterations = 200;
  options.absoluteTolerance = 1e-7;
  options.relativeTolerance = 1e-4;
  options.complementarityTolerance = 1e-6;
  options.validateSolution = false;
  options.warmStart = false;

  Eigen::VectorXd x = Eigen::VectorXd::Zero(snapshot.size());
  const dart::test::LcpSolveReport report
      = dart::test::SolveAndCheck(solver, problem, x, options);
  EXPECT_EQ(report.result.status, dart::math::LcpSolverStatus::Success)
      << solver.getName() << " " << dart::test::DescribeReport(report);
  EXPECT_TRUE(report.check.ok)
      << solver.getName() << " " << dart::test::DescribeReport(report);
}

template <typename Solver, typename Parameters>
void expectExplicitBlockSplitRejected(
    const sx::detail::BoxedLcpContactSnapshot& snapshot, Parameters& params)
{
  Solver solver;
  dart::math::LcpOptions options = solver.getDefaultOptions();
  options.customOptions = &params;
  options.validateSolution = false;

  const dart::math::LcpProblem problem(
      snapshot.A, snapshot.b, snapshot.lo, snapshot.hi, snapshot.findex);
  Eigen::VectorXd x = Eigen::VectorXd::Zero(snapshot.size());
  const dart::math::LcpResult result = solver.solve(problem, x, options);
  EXPECT_EQ(result.status, dart::math::LcpSolverStatus::InvalidProblem)
      << solver.getName()
      << " accepted an explicit block split: " << result.message;
  EXPECT_NE(
      result.message.find("Block partition must include friction index"),
      std::string::npos)
      << result.message;
}

} // namespace

//==============================================================================
// Real DART 7 contact assembly: contacts are produced by World::collide(), then
// the opt-in BoxedLcp path assembles the same boxed/friction-index LCP snapshot
// that World::step() consumes. Validate the assembled LCP contract directly and
// verify the applied impulse removes downward approach while reducing slip.
TEST(BoxedLcpContact, WorldContactSnapshotSatisfiesLcpContract)
{
  constexpr double kFriction = 0.7;
  const Eigen::Vector3d initialVelocity(0.4, -0.2, -0.2);
  sx::WorldOptions options;
  options.timeStep = 0.005;
  options.gravity = Eigen::Vector3d(0.0, 0.0, -9.81);
  options.contactSolverMethod = sx::ContactSolverMethod::BoxedLcp;
  sx::World lcp(options);

  sx::RigidBodyOptions groundOptions;
  groundOptions.isStatic = true;
  groundOptions.position = Eigen::Vector3d(0.0, 0.0, -0.5);
  auto ground = lcp.addRigidBody("ground", groundOptions);
  ground.setCollisionShape(
      sx::CollisionShape::makeBox(Eigen::Vector3d(20.0, 20.0, 0.5)));
  ground.setFriction(kFriction);

  sx::RigidBodyOptions sphereOptions;
  sphereOptions.position = Eigen::Vector3d(0.0, 0.0, 0.5);
  sphereOptions.linearVelocity = initialVelocity;
  auto sphere = lcp.addRigidBody("sphere", sphereOptions);
  sphere.setCollisionShape(sx::CollisionShape::makeSphere(0.5));
  sphere.setFriction(kFriction);

  const double initialTangentialSpeed
      = sphere.getLinearVelocity().head<2>().norm();

  const std::vector<sx::Contact> contacts = lcp.collide();
  ASSERT_FALSE(contacts.empty());

  const sx::detail::BoxedLcpContactSnapshot snapshot
      = sx::detail::solveBoxedLcpContacts(
          sx::detail::registryOf(lcp), contacts, lcp.getTimeStep());

  expectBoxedFrictionIndexSnapshot(snapshot, kFriction, 1u, 1u);

  const Eigen::Vector3d finalVelocity = sphere.getLinearVelocity();
  EXPECT_GE(finalVelocity.z(), -1e-6);
  EXPECT_LT(finalVelocity.head<2>().norm(), initialTangentialSpeed);
}

//==============================================================================
// The same real DART 7 boxed-LCP assembly path should validate when the world
// contributes multiple independent contacts to one coupled LCP snapshot.
TEST(BoxedLcpContact, TwoSphereWorldContactSnapshotSatisfiesLcpContract)
{
  constexpr double kFriction = 0.7;
  const Eigen::Vector3d initialVelocityA(0.4, -0.2, -0.2);
  const Eigen::Vector3d initialVelocityB(-0.3, 0.25, -0.2);

  sx::WorldOptions options;
  options.timeStep = 0.005;
  options.gravity = Eigen::Vector3d(0.0, 0.0, -9.81);
  options.contactSolverMethod = sx::ContactSolverMethod::BoxedLcp;
  sx::World lcp(options);

  sx::RigidBodyOptions groundOptions;
  groundOptions.isStatic = true;
  groundOptions.position = Eigen::Vector3d(0.0, 0.0, -0.5);
  auto ground = lcp.addRigidBody("ground", groundOptions);
  ground.setCollisionShape(
      sx::CollisionShape::makeBox(Eigen::Vector3d(20.0, 20.0, 0.5)));
  ground.setFriction(kFriction);

  sx::RigidBodyOptions sphereOptionsA;
  sphereOptionsA.position = Eigen::Vector3d(-1.0, 0.0, 0.5);
  sphereOptionsA.linearVelocity = initialVelocityA;
  auto sphereA = lcp.addRigidBody("sphere_a", sphereOptionsA);
  sphereA.setCollisionShape(sx::CollisionShape::makeSphere(0.5));
  sphereA.setFriction(kFriction);

  sx::RigidBodyOptions sphereOptionsB;
  sphereOptionsB.position = Eigen::Vector3d(1.0, 0.0, 0.5);
  sphereOptionsB.linearVelocity = initialVelocityB;
  auto sphereB = lcp.addRigidBody("sphere_b", sphereOptionsB);
  sphereB.setCollisionShape(sx::CollisionShape::makeSphere(0.5));
  sphereB.setFriction(kFriction);

  const double initialTangentialSpeedA
      = sphereA.getLinearVelocity().head<2>().norm();
  const double initialTangentialSpeedB
      = sphereB.getLinearVelocity().head<2>().norm();

  const std::vector<sx::Contact> contacts = lcp.collide();
  ASSERT_EQ(contacts.size(), 2u);

  const sx::detail::BoxedLcpContactSnapshot snapshot
      = sx::detail::solveBoxedLcpContacts(
          sx::detail::registryOf(lcp), contacts, lcp.getTimeStep());

  expectBoxedFrictionIndexSnapshot(snapshot, kFriction, 2u, 2u);

  const Eigen::Vector3d finalVelocityA = sphereA.getLinearVelocity();
  const Eigen::Vector3d finalVelocityB = sphereB.getLinearVelocity();
  EXPECT_GE(finalVelocityA.z(), -1e-6);
  EXPECT_GE(finalVelocityB.z(), -1e-6);
  EXPECT_LT(finalVelocityA.head<2>().norm(), initialTangentialSpeedA);
  EXPECT_LT(finalVelocityB.head<2>().norm(), initialTangentialSpeedB);
}

//==============================================================================
// DART 7 contact snapshots store normal rows first and tangent rows later. BGS
// and Blocked Jacobi must therefore derive contact blocks from `findex` instead
// of assuming contiguous triplets when solving world-contact LCPs.
TEST(BoxedLcpContact, BlockSolversUseFindexContactBlocksOnWorldSnapshot)
{
  constexpr double kFriction = 0.7;
  auto lcp = buildSeparatedSphereGroundScene(2, kFriction);

  const std::vector<sx::Contact> contacts = lcp->collide();
  ASSERT_EQ(contacts.size(), 2u);

  const sx::detail::BoxedLcpContactSnapshot snapshot
      = sx::detail::solveBoxedLcpContacts(
          sx::detail::registryOf(*lcp), contacts, lcp->getTimeStep());

  expectBoxedFrictionIndexShape(snapshot, kFriction, 2u, 2u);
  ASSERT_EQ(snapshot.contactCount, 2u);
  ASSERT_EQ(snapshot.size(), 6);
  EXPECT_EQ(snapshot.findex[2], 0);
  EXPECT_EQ(snapshot.findex[3], 0);
  EXPECT_EQ(snapshot.findex[4], 1);
  EXPECT_EQ(snapshot.findex[5], 1);

  dart::math::BgsSolver bgs;
  expectBlockSolverPassesWorldContactSnapshot(bgs, snapshot);

  dart::math::BlockedJacobiSolver blockedJacobi;
  expectBlockSolverPassesWorldContactSnapshot(blockedJacobi, snapshot);
}

//==============================================================================
// Explicit block sizes are valid only when each block contains every row needed
// by its `findex` dependencies. A contiguous split of the real DART 7 row
// layout would separate tangent rows from their normal owner and must fail
// loudly.
TEST(BoxedLcpContact, BlockSolversRejectExplicitWorldContactFindexSplit)
{
  constexpr double kFriction = 0.7;
  auto lcp = buildSeparatedSphereGroundScene(2, kFriction);

  const std::vector<sx::Contact> contacts = lcp->collide();
  ASSERT_EQ(contacts.size(), 2u);

  const sx::detail::BoxedLcpContactSnapshot snapshot
      = sx::detail::solveBoxedLcpContacts(
          sx::detail::registryOf(*lcp), contacts, lcp->getTimeStep());

  expectBoxedFrictionIndexShape(snapshot, kFriction, 2u, 2u);

  dart::math::BgsSolver::Parameters bgsParams;
  bgsParams.blockSizes = {3, 3};
  expectExplicitBlockSplitRejected<
      dart::math::BgsSolver,
      dart::math::BgsSolver::Parameters>(snapshot, bgsParams);

  dart::math::BlockedJacobiSolver::Parameters blockedJacobiParams;
  blockedJacobiParams.blockSizes = {3, 3};
  expectExplicitBlockSplitRejected<
      dart::math::BlockedJacobiSolver,
      dart::math::BlockedJacobiSolver::Parameters>(
      snapshot, blockedJacobiParams);
}

//==============================================================================
// A box face resting on ground produces a dense multi-point contact patch: all
// contact rows share the same dynamic body, so the Delassus system is
// rank-deficient unless the boxed-LCP friction regularization is effective.
TEST(BoxedLcpContact, DenseBoxWorldContactSnapshotSatisfiesLcpContract)
{
  constexpr double kFriction = 0.5;
  auto lcp = buildFrictionScene(
      sx::ContactSolverMethod::BoxedLcp,
      kFriction,
      Eigen::Vector3d(0.35, -0.2, -0.02));

  const std::vector<sx::Contact> contacts = lcp->collide();
  ASSERT_GE(contacts.size(), 4u);

  const sx::detail::BoxedLcpContactSnapshot snapshot
      = sx::detail::solveBoxedLcpContacts(
          sx::detail::registryOf(*lcp), contacts, lcp->getTimeStep());

  expectBoxedFrictionIndexShape(snapshot, kFriction, contacts.size(), 1u);

  const dart::math::LcpProblem problem(
      snapshot.A, snapshot.b, snapshot.lo, snapshot.hi, snapshot.findex);
  dart::math::ApgdSolver solver;
  Eigen::VectorXd x = Eigen::VectorXd::Zero(snapshot.size());
  dart::math::LcpOptions options;
  options.maxIterations = 200;
  options.absoluteTolerance = 1e-6;
  options.relativeTolerance = 1e-4;
  options.complementarityTolerance = 1e-6;
  options.validateSolution = false;
  solver.solve(problem, x, options);

  const dart::test::LcpCheckResult check
      = dart::test::CheckLcpSolution(problem, x, options);
  EXPECT_TRUE(check.ok) << "message=" << check.message
                        << " residual=" << check.residual
                        << " complementarity=" << check.complementarity
                        << " boundViolation=" << check.boundViolation
                        << " tol=" << check.tol << " compTol=" << check.compTol;
}

//==============================================================================
// A vertical stack creates coupled contacts: the bottom sphere touches the
// ground, while adjacent spheres touch each other and share dynamic bodies.
// Validate the boxed/findex LCP contract and prove the normal block is not just
// a collection of independent contact rows.
TEST(BoxedLcpContact, SphereStackWorldContactSnapshotSatisfiesLcpContract)
{
  constexpr double kFriction = 0.6;
  constexpr int kSphereCount = 3;

  auto lcp = buildSphereStackScene(kSphereCount, kFriction, true);

  const std::vector<sx::Contact> contacts = lcp->collide();
  ASSERT_EQ(contacts.size(), static_cast<std::size_t>(kSphereCount));

  const sx::detail::BoxedLcpContactSnapshot snapshot
      = sx::detail::solveBoxedLcpContacts(
          sx::detail::registryOf(*lcp), contacts, lcp->getTimeStep());

  expectBoxedFrictionIndexSnapshot(
      snapshot, kFriction, kSphereCount, kSphereCount);

  EXPECT_GT(maxNormalContactCoupling(snapshot), 1e-6);
}

//==============================================================================
// A larger four-sphere stack extends the real DART 7 boxed-LCP snapshot
// coverage to a 12-row, 4-contact coupled friction-index system.
TEST(BoxedLcpContact, LargerSphereStackWorldContactSnapshotSatisfiesLcpContract)
{
  constexpr double kFriction = 0.6;
  constexpr int kSphereCount = 4;

  auto lcp = buildSphereStackScene(kSphereCount, kFriction, true);

  const std::vector<sx::Contact> contacts = lcp->collide();
  ASSERT_EQ(contacts.size(), static_cast<std::size_t>(kSphereCount));

  const sx::detail::BoxedLcpContactSnapshot snapshot
      = sx::detail::solveBoxedLcpContacts(
          sx::detail::registryOf(*lcp), contacts, lcp->getTimeStep());

  expectBoxedFrictionIndexSnapshot(
      snapshot, kFriction, kSphereCount, kSphereCount);
  EXPECT_GT(maxNormalContactCoupling(snapshot), 1e-6);
}

//==============================================================================
// A five-sphere stack extends coupled multi-contact snapshot evidence to a
// 15-row, 5-contact friction-index LCP assembled from shared dynamic bodies.
TEST(BoxedLcpContact, StressSphereStackWorldContactSnapshotSatisfiesLcpContract)
{
  constexpr double kFriction = 0.6;
  constexpr int kSphereCount = 5;

  auto lcp = buildSphereStackScene(kSphereCount, kFriction, true);

  const std::vector<sx::Contact> contacts = lcp->collide();
  ASSERT_EQ(contacts.size(), static_cast<std::size_t>(kSphereCount));

  const sx::detail::BoxedLcpContactSnapshot snapshot
      = sx::detail::solveBoxedLcpContacts(
          sx::detail::registryOf(*lcp), contacts, lcp->getTimeStep());

  expectBoxedFrictionIndexSnapshot(
      snapshot, kFriction, kSphereCount, kSphereCount);
  EXPECT_GT(maxNormalContactCoupling(snapshot), 1e-6);
}

//==============================================================================
// A six-sphere stack extends coupled multi-contact snapshot evidence beyond the
// previous 5-contact boundary while preserving the same shared-dynamic-body
// contact topology.
TEST(
    BoxedLcpContact,
    LargerStressSphereStackWorldContactSnapshotSatisfiesLcpContract)
{
  constexpr double kFriction = 0.6;
  constexpr int kSphereCount = 6;

  auto lcp = buildSphereStackScene(kSphereCount, kFriction, true);

  const std::vector<sx::Contact> contacts = lcp->collide();
  ASSERT_EQ(contacts.size(), static_cast<std::size_t>(kSphereCount));

  const sx::detail::BoxedLcpContactSnapshot snapshot
      = sx::detail::solveBoxedLcpContacts(
          sx::detail::registryOf(*lcp), contacts, lcp->getTimeStep());

  expectBoxedFrictionIndexSnapshot(
      snapshot, kFriction, kSphereCount, kSphereCount);
  EXPECT_GT(maxNormalContactCoupling(snapshot), 1e-6);
}

//==============================================================================
// A seven-sphere stack extends coupled multi-contact snapshot evidence to a
// 21-row friction-index LCP assembled from one ground contact and six shared
// dynamic body contacts.
TEST(BoxedLcpContact, SevenSphereStackWorldContactSnapshotSatisfiesLcpContract)
{
  constexpr double kFriction = 0.6;
  constexpr int kSphereCount = 7;

  auto lcp = buildSphereStackScene(kSphereCount, kFriction, true);

  const std::vector<sx::Contact> contacts = lcp->collide();
  ASSERT_EQ(contacts.size(), static_cast<std::size_t>(kSphereCount));

  const sx::detail::BoxedLcpContactSnapshot snapshot
      = sx::detail::solveBoxedLcpContacts(
          sx::detail::registryOf(*lcp), contacts, lcp->getTimeStep());

  expectBoxedFrictionIndexSnapshot(
      snapshot, kFriction, kSphereCount, kSphereCount);
  EXPECT_GT(maxNormalContactCoupling(snapshot), 1e-6);
}

//==============================================================================
// An eight-sphere stack extends coupled multi-contact snapshot evidence beyond
// the previous 7-contact boundary without claiming public-step stability.
TEST(BoxedLcpContact, EightSphereStackWorldContactSnapshotSatisfiesLcpContract)
{
  constexpr double kFriction = 0.6;
  constexpr int kSphereCount = 8;

  auto lcp = buildSphereStackScene(kSphereCount, kFriction, true);

  const std::vector<sx::Contact> contacts = lcp->collide();
  ASSERT_EQ(contacts.size(), static_cast<std::size_t>(kSphereCount));

  const sx::detail::BoxedLcpContactSnapshot snapshot
      = sx::detail::solveBoxedLcpContacts(
          sx::detail::registryOf(*lcp), contacts, lcp->getTimeStep());

  expectBoxedFrictionIndexSnapshot(
      snapshot, kFriction, kSphereCount, kSphereCount);
  EXPECT_GT(maxNormalContactCoupling(snapshot), 1e-6);
}

//==============================================================================
// A nine-sphere stack extends the direct coupled-stack snapshot boundary to a
// 27-row friction-index LCP while still avoiding public-step stability claims.
TEST(BoxedLcpContact, NineSphereStackWorldContactSnapshotSatisfiesLcpContract)
{
  constexpr double kFriction = 0.6;
  constexpr int kSphereCount = 9;

  auto lcp = buildSphereStackScene(kSphereCount, kFriction, true);

  const std::vector<sx::Contact> contacts = lcp->collide();
  ASSERT_EQ(contacts.size(), static_cast<std::size_t>(kSphereCount));

  const sx::detail::BoxedLcpContactSnapshot snapshot
      = sx::detail::solveBoxedLcpContacts(
          sx::detail::registryOf(*lcp), contacts, lcp->getTimeStep());

  expectBoxedFrictionIndexSnapshot(
      snapshot, kFriction, kSphereCount, kSphereCount);
  EXPECT_GT(maxNormalContactCoupling(snapshot), 1e-6);
}

//==============================================================================
// A ten-sphere stack extends the direct coupled-stack snapshot boundary to a
// 30-row friction-index LCP while still avoiding public-step stability claims.
TEST(BoxedLcpContact, TenSphereStackWorldContactSnapshotSatisfiesLcpContract)
{
  constexpr double kFriction = 0.6;
  constexpr int kSphereCount = 10;

  auto lcp = buildSphereStackScene(kSphereCount, kFriction, true);

  const std::vector<sx::Contact> contacts = lcp->collide();
  ASSERT_EQ(contacts.size(), static_cast<std::size_t>(kSphereCount));

  const sx::detail::BoxedLcpContactSnapshot snapshot
      = sx::detail::solveBoxedLcpContacts(
          sx::detail::registryOf(*lcp), contacts, lcp->getTimeStep());

  expectBoxedFrictionIndexSnapshot(
      snapshot, kFriction, kSphereCount, kSphereCount);
  EXPECT_GT(maxNormalContactCoupling(snapshot), 1e-6);
}

//==============================================================================
// An eleven-sphere stack extends the direct coupled-stack snapshot boundary to
// a 33-row friction-index LCP while still avoiding public-step stability
// claims.
TEST(BoxedLcpContact, ElevenSphereStackWorldContactSnapshotSatisfiesLcpContract)
{
  constexpr double kFriction = 0.6;
  constexpr int kSphereCount = 11;

  auto lcp = buildSphereStackScene(kSphereCount, kFriction, true);

  const std::vector<sx::Contact> contacts = lcp->collide();
  ASSERT_EQ(contacts.size(), static_cast<std::size_t>(kSphereCount));

  const sx::detail::BoxedLcpContactSnapshot snapshot
      = sx::detail::solveBoxedLcpContacts(
          sx::detail::registryOf(*lcp), contacts, lcp->getTimeStep());

  expectBoxedFrictionIndexSnapshot(
      snapshot, kFriction, kSphereCount, kSphereCount);
  EXPECT_GT(maxNormalContactCoupling(snapshot), 1e-6);
}

//==============================================================================
// A twelve-sphere stack extends the direct coupled-stack snapshot boundary to a
// 36-row friction-index LCP while still avoiding public-step stability claims.
TEST(BoxedLcpContact, TwelveSphereStackWorldContactSnapshotSatisfiesLcpContract)
{
  constexpr double kFriction = 0.6;
  constexpr int kSphereCount = 12;

  auto lcp = buildSphereStackScene(kSphereCount, kFriction, true);

  const std::vector<sx::Contact> contacts = lcp->collide();
  ASSERT_EQ(contacts.size(), static_cast<std::size_t>(kSphereCount));

  const sx::detail::BoxedLcpContactSnapshot snapshot
      = sx::detail::solveBoxedLcpContacts(
          sx::detail::registryOf(*lcp), contacts, lcp->getTimeStep());

  expectBoxedFrictionIndexSnapshot(
      snapshot, kFriction, kSphereCount, kSphereCount);
  EXPECT_GT(maxNormalContactCoupling(snapshot), 1e-6);
}

//==============================================================================
// A thirteen-sphere stack extends the direct coupled-stack snapshot boundary to
// a 39-row friction-index LCP while still avoiding public-step stability
// claims.
TEST(
    BoxedLcpContact,
    ThirteenSphereStackWorldContactSnapshotSatisfiesLcpContract)
{
  constexpr double kFriction = 0.6;
  constexpr int kSphereCount = 13;

  auto lcp = buildSphereStackScene(kSphereCount, kFriction, true);

  const std::vector<sx::Contact> contacts = lcp->collide();
  ASSERT_EQ(contacts.size(), static_cast<std::size_t>(kSphereCount));

  const sx::detail::BoxedLcpContactSnapshot snapshot
      = sx::detail::solveBoxedLcpContacts(
          sx::detail::registryOf(*lcp), contacts, lcp->getTimeStep());

  expectBoxedFrictionIndexSnapshot(
      snapshot, kFriction, kSphereCount, kSphereCount);
  EXPECT_GT(maxNormalContactCoupling(snapshot), 1e-6);
}

//==============================================================================
// A fourteen-sphere stack extends the direct coupled-stack snapshot boundary to
// a 42-row friction-index LCP while still avoiding public-step stability
// claims.
TEST(
    BoxedLcpContact,
    FourteenSphereStackWorldContactSnapshotSatisfiesLcpContract)
{
  constexpr double kFriction = 0.6;
  constexpr int kSphereCount = 14;

  auto lcp = buildSphereStackScene(kSphereCount, kFriction, true);

  const std::vector<sx::Contact> contacts = lcp->collide();
  ASSERT_EQ(contacts.size(), static_cast<std::size_t>(kSphereCount));

  const sx::detail::BoxedLcpContactSnapshot snapshot
      = sx::detail::solveBoxedLcpContacts(
          sx::detail::registryOf(*lcp), contacts, lcp->getTimeStep());

  expectBoxedFrictionIndexSnapshot(
      snapshot, kFriction, kSphereCount, kSphereCount);
  EXPECT_GT(maxNormalContactCoupling(snapshot), 1e-6);
}

//==============================================================================
// A fifteen-sphere stack extends the direct coupled-stack snapshot boundary to
// a 45-row friction-index LCP while still avoiding public-step stability
// claims.
TEST(
    BoxedLcpContact, FifteenSphereStackWorldContactSnapshotSatisfiesLcpContract)
{
  constexpr double kFriction = 0.6;
  constexpr int kSphereCount = 15;

  auto lcp = buildSphereStackScene(kSphereCount, kFriction, true);

  const std::vector<sx::Contact> contacts = lcp->collide();
  ASSERT_EQ(contacts.size(), static_cast<std::size_t>(kSphereCount));

  const sx::detail::BoxedLcpContactSnapshot snapshot
      = sx::detail::solveBoxedLcpContacts(
          sx::detail::registryOf(*lcp), contacts, lcp->getTimeStep());

  expectBoxedFrictionIndexSnapshot(
      snapshot, kFriction, kSphereCount, kSphereCount);
  EXPECT_GT(maxNormalContactCoupling(snapshot), 1e-6);
}

//==============================================================================
// A sixteen-sphere stack extends the direct coupled-stack snapshot boundary to
// a 48-row friction-index LCP while still avoiding public-step stability
// claims.
TEST(
    BoxedLcpContact, SixteenSphereStackWorldContactSnapshotSatisfiesLcpContract)
{
  constexpr double kFriction = 0.6;
  constexpr int kSphereCount = 16;

  auto lcp = buildSphereStackScene(kSphereCount, kFriction, true);

  const std::vector<sx::Contact> contacts = lcp->collide();
  ASSERT_EQ(contacts.size(), static_cast<std::size_t>(kSphereCount));

  const sx::detail::BoxedLcpContactSnapshot snapshot
      = sx::detail::solveBoxedLcpContacts(
          sx::detail::registryOf(*lcp), contacts, lcp->getTimeStep());

  expectBoxedFrictionIndexSnapshot(
      snapshot, kFriction, kSphereCount, kSphereCount);
  EXPECT_GT(maxNormalContactCoupling(snapshot), 1e-6);
}

//==============================================================================
// A twenty-four-sphere stack extends the direct coupled-stack snapshot boundary
// to a 72-row friction-index LCP while still avoiding public-step stability
// claims.
TEST(
    BoxedLcpContact,
    TwentyFourSphereStackWorldContactSnapshotSatisfiesLcpContract)
{
  constexpr double kFriction = 0.6;
  constexpr int kSphereCount = 24;

  auto lcp = buildSphereStackScene(kSphereCount, kFriction, true);

  const std::vector<sx::Contact> contacts = lcp->collide();
  ASSERT_EQ(contacts.size(), static_cast<std::size_t>(kSphereCount));

  const sx::detail::BoxedLcpContactSnapshot snapshot
      = sx::detail::solveBoxedLcpContacts(
          sx::detail::registryOf(*lcp), contacts, lcp->getTimeStep());

  expectBoxedFrictionIndexSnapshot(
      snapshot, kFriction, kSphereCount, kSphereCount);
  EXPECT_GT(maxNormalContactCoupling(snapshot), 1e-6);
}

//==============================================================================
// A thirty-two-sphere stack extends the direct coupled-stack snapshot boundary
// to a 96-row friction-index LCP while still avoiding public-step stability
// claims.
TEST(
    BoxedLcpContact,
    ThirtyTwoSphereStackWorldContactSnapshotSatisfiesLcpContract)
{
  constexpr double kFriction = 0.6;
  constexpr int kSphereCount = 32;

  auto lcp = buildSphereStackScene(kSphereCount, kFriction, true);

  const std::vector<sx::Contact> contacts = lcp->collide();
  ASSERT_EQ(contacts.size(), static_cast<std::size_t>(kSphereCount));

  const sx::detail::BoxedLcpContactSnapshot snapshot
      = sx::detail::solveBoxedLcpContacts(
          sx::detail::registryOf(*lcp), contacts, lcp->getTimeStep());

  expectBoxedFrictionIndexSnapshot(
      snapshot, kFriction, kSphereCount, kSphereCount);
  EXPECT_GT(maxNormalContactCoupling(snapshot), 1e-6);
}

//==============================================================================
// End-to-end DART 7 World stepping for coupled contacts: a 3-sphere stack
// advances through the public BoxedLcp path for many time steps. This checks
// motion-level invariants for the same shared-body contact topology validated
// by the direct LCP snapshot test above.
TEST(BoxedLcpContact, SphereStackWorldStepMaintainsContactInvariants)
{
  constexpr double kFriction = 0.6;
  constexpr int kSphereCount = 3;

  auto lcp = buildSphereStackScene(kSphereCount, kFriction, false);

  lcp->enterSimulationMode();
  lcp->step(200);

  expectSphereStackStepInvariants(*lcp, kSphereCount);
}

//==============================================================================
// Longer-running DART 7 World stepping for coupled contacts: the same
// 3-sphere stack remains stable beyond the shorter 200-step smoke horizon.
TEST(BoxedLcpContact, LongRunningSphereStackWorldStepMaintainsContactInvariants)
{
  constexpr double kFriction = 0.6;
  constexpr int kSphereCount = 3;

  auto lcp = buildSphereStackScene(kSphereCount, kFriction, false);

  lcp->enterSimulationMode();
  lcp->step(500);

  expectSphereStackStepInvariants(*lcp, kSphereCount);
}

//==============================================================================
// The 4-sphere stack is a taller coupled contact chain than the 3-sphere
// end-to-end case above. Use the same smaller integration step as the benchmark
// fixture and advance a shorter horizon that still exercises repeated public
// World::step() contact solves without claiming long-horizon 4-stack stability.
TEST(BoxedLcpContact, LargerSphereStackWorldStepMaintainsContactInvariants)
{
  constexpr double kFriction = 0.6;
  constexpr int kSphereCount = 4;

  auto lcp = buildSphereStackScene(kSphereCount, kFriction, false);

  lcp->enterSimulationMode();
  lcp->step(200);

  expectSphereStackStepInvariants(*lcp, kSphereCount);
}

//==============================================================================
// The 5-sphere stack extends the existing 5-contact boxed/findex snapshot into
// the public World::step() path. It needs a longer horizon than the 4-sphere
// gate before satisfying the near-rest velocity invariant.
TEST(BoxedLcpContact, StressSphereStackWorldStepMaintainsContactInvariants)
{
  constexpr double kFriction = 0.6;
  constexpr int kSphereCount = 5;

  auto lcp = buildSphereStackScene(kSphereCount, kFriction, false);

  lcp->enterSimulationMode();
  lcp->step(500);

  expectSphereStackStepInvariants(*lcp, kSphereCount);
}

//==============================================================================
// The 6-sphere stack extends the coupled contact chain in the public
// World::step() path beyond the previous 5-contact boundary.
TEST(
    BoxedLcpContact, LargerStressSphereStackWorldStepMaintainsContactInvariants)
{
  constexpr double kFriction = 0.6;
  constexpr int kSphereCount = 6;

  auto lcp = buildSphereStackScene(kSphereCount, kFriction, false);

  lcp->enterSimulationMode();
  lcp->step(1000);

  expectSphereStackStepInvariants(*lcp, kSphereCount);
}

//==============================================================================
// End-to-end DART 7 World stepping: two independent sphere-ground contacts are
// advanced through the public BoxedLcp contact solver for many time steps. This
// complements the direct LCP snapshot tests above by checking the integrated
// motion invariants that users observe from World::step().
TEST(BoxedLcpContact, TwoSphereWorldStepMaintainsContactInvariants)
{
  constexpr double kFriction = 0.7;
  sx::WorldOptions options;
  options.timeStep = 0.005;
  options.gravity = Eigen::Vector3d(0.0, 0.0, -9.81);
  options.contactSolverMethod = sx::ContactSolverMethod::BoxedLcp;
  sx::World lcp(options);

  sx::RigidBodyOptions groundOptions;
  groundOptions.isStatic = true;
  groundOptions.position = Eigen::Vector3d(0.0, 0.0, -0.5);
  auto ground = lcp.addRigidBody("ground", groundOptions);
  ground.setCollisionShape(
      sx::CollisionShape::makeBox(Eigen::Vector3d(20.0, 20.0, 0.5)));
  ground.setFriction(kFriction);

  sx::RigidBodyOptions sphereOptionsA;
  sphereOptionsA.position = Eigen::Vector3d(-1.0, 0.0, 0.5);
  sphereOptionsA.linearVelocity = Eigen::Vector3d(0.4, -0.2, -0.02);
  auto sphereA = lcp.addRigidBody("sphere_a", sphereOptionsA);
  sphereA.setCollisionShape(sx::CollisionShape::makeSphere(0.5));
  sphereA.setFriction(kFriction);

  sx::RigidBodyOptions sphereOptionsB;
  sphereOptionsB.position = Eigen::Vector3d(1.0, 0.0, 0.5);
  sphereOptionsB.linearVelocity = Eigen::Vector3d(-0.3, 0.25, -0.02);
  auto sphereB = lcp.addRigidBody("sphere_b", sphereOptionsB);
  sphereB.setCollisionShape(sx::CollisionShape::makeSphere(0.5));
  sphereB.setFriction(kFriction);

  const double initialTangentialSpeedA
      = sphereA.getLinearVelocity().head<2>().norm();
  const double initialTangentialSpeedB
      = sphereB.getLinearVelocity().head<2>().norm();

  lcp.enterSimulationMode();
  lcp.step(200);

  const auto expectSettledSphere = [](const sx::RigidBody& sphere,
                                      double initialTangentialSpeed) {
    EXPECT_TRUE(sphere.getTranslation().allFinite());
    EXPECT_TRUE(sphere.getLinearVelocity().allFinite());
    EXPECT_GE(sphere.getTranslation().z(), 0.5 - 1e-3);
    EXPECT_NEAR(sphere.getTranslation().z(), 0.5, 2e-2);
    EXPECT_LT(std::abs(sphere.getLinearVelocity().z()), 0.1);
    EXPECT_LT(
        sphere.getLinearVelocity().head<2>().norm(), initialTangentialSpeed);
  };

  expectSettledSphere(sphereA, initialTangentialSpeedA);
  expectSettledSphere(sphereB, initialTangentialSpeedB);
  EXPECT_TRUE(
      ground.getTranslation().isApprox(Eigen::Vector3d(0.0, 0.0, -0.5)));
}

//==============================================================================
// Denser end-to-end DART 7 World stepping: four independent sphere-ground
// contacts are advanced together through the public BoxedLcp path. This extends
// the two-contact invariant test to a larger simultaneous contact set.
TEST(BoxedLcpContact, FourSphereWorldStepMaintainsContactInvariants)
{
  constexpr double kFriction = 0.7;
  constexpr int kSphereCount = 4;

  auto lcp = buildSeparatedSphereGroundScene(kSphereCount, kFriction);

  const std::vector<sx::Contact> contacts = lcp->collide();
  ASSERT_EQ(contacts.size(), static_cast<std::size_t>(kSphereCount));

  lcp->enterSimulationMode();
  lcp->step(200);

  expectSeparatedSphereStepInvariants(*lcp, kSphereCount);
}

//==============================================================================
// Denser separated-contact DART 7 World stepping: sixteen independent
// sphere-ground contacts exercise a larger public BoxedLcp contact solve while
// keeping the physical invariant easy to interpret.
TEST(BoxedLcpContact, SixteenSphereWorldStepMaintainsContactInvariants)
{
  constexpr double kFriction = 0.7;
  constexpr int kSphereCount = 16;

  auto lcp = buildSeparatedSphereGroundScene(kSphereCount, kFriction);

  const std::vector<sx::Contact> contacts = lcp->collide();
  ASSERT_EQ(contacts.size(), static_cast<std::size_t>(kSphereCount));

  lcp->enterSimulationMode();
  lcp->step(200);

  expectSeparatedSphereStepInvariants(*lcp, kSphereCount);
}

//==============================================================================
// Dense separated-contact DART 7 World stepping: each box contributes a
// four-point face-contact patch, so the public BoxedLcp path advances a larger
// dense contact set with multiple dynamic bodies.
TEST(BoxedLcpContact, FourBoxWorldStepMaintainsDenseContactInvariants)
{
  constexpr double kFriction = 0.5;
  constexpr int kBoxCount = 4;

  auto lcp = buildSeparatedBoxGroundScene(kBoxCount, kFriction);

  const std::vector<sx::Contact> contacts = lcp->collide();
  ASSERT_EQ(contacts.size(), static_cast<std::size_t>(4 * kBoxCount));

  lcp->enterSimulationMode();
  lcp->step(200);

  expectSeparatedBoxStepInvariants(*lcp, kBoxCount);
}

//==============================================================================
// Stress dense separated-contact DART 7 World stepping with twice the current
// box count, preserving the same public BoxedLcp motion invariants.
TEST(BoxedLcpContact, EightBoxWorldStepMaintainsDenseContactInvariants)
{
  constexpr double kFriction = 0.5;
  constexpr int kBoxCount = 8;

  auto lcp = buildSeparatedBoxGroundScene(kBoxCount, kFriction);

  const std::vector<sx::Contact> contacts = lcp->collide();
  ASSERT_EQ(contacts.size(), static_cast<std::size_t>(4 * kBoxCount));

  lcp->enterSimulationMode();
  lcp->step(200);

  expectSeparatedBoxStepInvariants(*lcp, kBoxCount);
}

//==============================================================================
TEST(BoxedLcpContact, SixteenBoxWorldStepMaintainsDenseContactInvariants)
{
  constexpr double kFriction = 0.5;
  constexpr int kBoxCount = 16;

  auto lcp = buildSeparatedBoxGroundScene(kBoxCount, kFriction);

  const std::vector<sx::Contact> contacts = lcp->collide();
  ASSERT_EQ(contacts.size(), static_cast<std::size_t>(4 * kBoxCount));

  lcp->enterSimulationMode();
  lcp->step(500);

  expectSeparatedBoxStepInvariants(*lcp, kBoxCount);
}

//==============================================================================
TEST(BoxedLcpContact, TwentyFourBoxWorldStepMaintainsDenseContactInvariants)
{
  constexpr double kFriction = 0.5;
  constexpr int kBoxCount = 24;

  auto lcp = buildSeparatedBoxGroundScene(kBoxCount, kFriction);

  const std::vector<sx::Contact> contacts = lcp->collide();
  ASSERT_EQ(contacts.size(), static_cast<std::size_t>(4 * kBoxCount));

  lcp->enterSimulationMode();
  lcp->step(2000);

  expectSeparatedBoxStepInvariants(*lcp, kBoxCount);
}

//==============================================================================
TEST(BoxedLcpContact, ThirtyTwoBoxWorldStepMaintainsDenseContactInvariants)
{
  constexpr double kFriction = 0.5;
  constexpr int kBoxCount = 32;

  auto lcp = buildSeparatedBoxGroundScene(kBoxCount, kFriction);

  const std::vector<sx::Contact> contacts = lcp->collide();
  ASSERT_EQ(contacts.size(), static_cast<std::size_t>(4 * kBoxCount));

  lcp->enterSimulationMode();
  lcp->step(4000);

  expectSeparatedBoxStepInvariants(*lcp, kBoxCount);
}

//==============================================================================
TEST(BoxedLcpContact, FortyEightBoxWorldStepMaintainsDenseContactInvariants)
{
  constexpr double kFriction = 0.5;
  constexpr int kBoxCount = 48;

  auto lcp = buildSeparatedBoxGroundScene(kBoxCount, kFriction);

  const std::vector<sx::Contact> contacts = lcp->collide();
  ASSERT_EQ(contacts.size(), static_cast<std::size_t>(4 * kBoxCount));

  lcp->enterSimulationMode();
  lcp->step(4000);

  expectSeparatedBoxStepInvariants(*lcp, kBoxCount);
}

//==============================================================================
TEST(BoxedLcpContact, SixtyFourBoxWorldStepPreservesDenseContactShape)
{
  constexpr double kFriction = 0.5;
  constexpr int kBoxCount = 64;

  auto lcp = buildSeparatedBoxGroundScene(kBoxCount, kFriction);

  const std::vector<sx::Contact> contacts = lcp->collide();
  ASSERT_EQ(contacts.size(), static_cast<std::size_t>(4 * kBoxCount));

  lcp->enterSimulationMode();
  lcp->step();

  expectSeparatedBoxDenseStepSmokeInvariants(*lcp, kBoxCount);
}

//==============================================================================
TEST(BoxedLcpContact, NinetySixBoxWorldStepPreservesDenseContactShape)
{
  constexpr double kFriction = 0.5;
  constexpr int kBoxCount = 96;

  auto lcp = buildSeparatedBoxGroundScene(kBoxCount, kFriction);

  const std::vector<sx::Contact> contacts = lcp->collide();
  ASSERT_EQ(contacts.size(), static_cast<std::size_t>(4 * kBoxCount));

  lcp->enterSimulationMode();
  lcp->step();

  expectSeparatedBoxDenseStepSmokeInvariants(*lcp, kBoxCount);
}

//==============================================================================
TEST(
    BoxedLcpContact,
    OneHundredTwentyEightBoxWorldStepPreservesDenseContactShape)
{
  constexpr double kFriction = 0.5;
  constexpr int kBoxCount = 128;

  auto lcp = buildSeparatedBoxGroundScene(kBoxCount, kFriction);

  const std::vector<sx::Contact> contacts = lcp->collide();
  ASSERT_EQ(contacts.size(), static_cast<std::size_t>(4 * kBoxCount));

  lcp->enterSimulationMode();
  lcp->step();

  expectSeparatedBoxDenseStepSmokeInvariants(*lcp, kBoxCount);
}

//==============================================================================
TEST(
    BoxedLcpContact,
    SixtyFourBoxWorldShortHorizonMaintainsDenseContactInvariants)
{
  constexpr double kFriction = 0.5;
  constexpr int kBoxCount = 64;

  auto lcp = buildSeparatedBoxGroundScene(kBoxCount, kFriction);

  const std::vector<sx::Contact> contacts = lcp->collide();
  ASSERT_EQ(contacts.size(), static_cast<std::size_t>(4 * kBoxCount));

  lcp->enterSimulationMode();
  lcp->step(75);

  expectSeparatedBoxStepInvariants(*lcp, kBoxCount);
}

//==============================================================================
TEST(
    BoxedLcpContact,
    NinetySixBoxWorldShortHorizonMaintainsDenseContactInvariants)
{
  constexpr double kFriction = 0.5;
  constexpr int kBoxCount = 96;

  auto lcp = buildSeparatedBoxGroundScene(kBoxCount, kFriction);

  const std::vector<sx::Contact> contacts = lcp->collide();
  ASSERT_EQ(contacts.size(), static_cast<std::size_t>(4 * kBoxCount));

  lcp->enterSimulationMode();
  lcp->step(75);

  expectSeparatedBoxStepInvariants(*lcp, kBoxCount);
}

//==============================================================================
// Articulated DART 7 World stepping: a fixed-base prismatic link starts in
// light contact with static ground. Under BoxedLcp this exercises the public
// unified constraint path, because the rigid-only boxed-LCP helper filters
// articulated-link contact out of its own assembly.
TEST(BoxedLcpContact, ArticulatedPrismaticLinkGroundStepMaintainsInvariants)
{
  const ArticulatedGroundStepResult reference
      = runArticulatedGroundStep(sx::ContactSolverMethod::SequentialImpulse);
  const ArticulatedGroundStepResult lcp
      = runArticulatedGroundStep(sx::ContactSolverMethod::BoxedLcp);

  ASSERT_EQ(lcp.contactCount, 1u);
  EXPECT_TRUE(lcp.contactTouchesLink);
  EXPECT_TRUE(std::isfinite(lcp.linkZ));
  EXPECT_TRUE(std::isfinite(lcp.jointVelocity));
  EXPECT_GT(lcp.linkZ, -0.315);
  EXPECT_NEAR(lcp.linkZ, -0.3, 2e-2);
  EXPECT_LT(std::abs(lcp.jointVelocity), 0.12);

  ASSERT_EQ(reference.contactCount, lcp.contactCount);
  EXPECT_TRUE(reference.contactTouchesLink);
  EXPECT_NEAR(reference.linkZ, lcp.linkZ, 2e-2);
  EXPECT_NEAR(reference.jointVelocity, lcp.jointVelocity, 0.12);
}

//==============================================================================
// Multi-articulated DART 7 World stepping: four independent fixed-base
// prismatic links contact the ground in one public BoxedLcp step sequence. This
// mirrors the benchmark shape and proves the unified articulated contact path
// handles multiple simultaneous link-ground contacts, not just a one-link
// smoke case.
TEST(
    BoxedLcpContact, FourArticulatedPrismaticLinksGroundStepMaintainsInvariants)
{
  constexpr int kLinkCount = 4;

  const MultiArticulatedGroundStepResult reference
      = runMultiArticulatedGroundStep(
          sx::ContactSolverMethod::SequentialImpulse, kLinkCount);
  const MultiArticulatedGroundStepResult lcp = runMultiArticulatedGroundStep(
      sx::ContactSolverMethod::BoxedLcp, kLinkCount);

  ASSERT_EQ(lcp.contactCount, static_cast<std::size_t>(kLinkCount));
  EXPECT_EQ(lcp.linkContactCount, static_cast<std::size_t>(kLinkCount));
  EXPECT_TRUE(lcp.allFinite);
  EXPECT_LE(lcp.maxHeightError, 2e-2);
  EXPECT_LT(lcp.maxAbsJointVelocity, 0.12);

  ASSERT_EQ(reference.contactCount, lcp.contactCount);
  EXPECT_EQ(reference.linkContactCount, lcp.linkContactCount);
  EXPECT_TRUE(reference.allFinite);
  EXPECT_NEAR(reference.maxHeightError, lcp.maxHeightError, 2e-2);
  EXPECT_NEAR(reference.maxAbsJointVelocity, lcp.maxAbsJointVelocity, 0.12);
}

//==============================================================================
// Connected multi-DOF articulated DART 7 World stepping: each robot is a
// serial three-axis prismatic chain, and its tip link contacts ground through
// the public BoxedLcp unified path. This extends the one-DOF link-ground cases
// to a real connected multibody tree while keeping the contact invariant
// directly interpretable.
TEST(BoxedLcpContact, CartesianPrismaticChainGroundStepMaintainsInvariants)
{
  constexpr int kChainCount = 2;
  expectCartesianPrismaticChainsGroundStepMaintainsInvariants(kChainCount);
}

//==============================================================================
// Four connected multi-DOF articulated DART 7 chains exercise the same public
// BoxedLcp unified path at the first benchmark-sized Cartesian-chain packet.
TEST(BoxedLcpContact, FourCartesianPrismaticChainsGroundStepMaintainsInvariants)
{
  constexpr int kChainCount = 4;
  expectCartesianPrismaticChainsGroundStepMaintainsInvariants(kChainCount);
}

//==============================================================================
// Eight connected multi-DOF articulated DART 7 chains match the next registered
// Cartesian-chain benchmark row and cover a 24-DOF unified contact packet.
TEST(
    BoxedLcpContact, EightCartesianPrismaticChainsGroundStepMaintainsInvariants)
{
  constexpr int kChainCount = 8;
  expectCartesianPrismaticChainsGroundStepMaintainsInvariants(kChainCount);
}

//==============================================================================
// Sixteen connected multi-DOF articulated DART 7 chains match the largest
// registered Cartesian-chain benchmark row and cover a 48-DOF unified contact
// packet.
TEST(
    BoxedLcpContact,
    SixteenCartesianPrismaticChainsGroundStepMaintainsInvariants)
{
  constexpr int kChainCount = 16;
  expectCartesianPrismaticChainsGroundStepMaintainsInvariants(kChainCount);
}

//==============================================================================
// Two-sided articulated contact: a prismatic link pushes a dynamic rigid body.
// The boxed-LCP unified path must apply equal-and-opposite impulses to the
// articulated generalized velocity and the rigid target velocity.
TEST(BoxedLcpContact, ArticulatedPrismaticLinkPushesDynamicRigidBody)
{
  const ArticulatedRigidImpactResult reference = runArticulatedRigidImpactStep(
      sx::ContactSolverMethod::SequentialImpulse);
  const ArticulatedRigidImpactResult lcp
      = runArticulatedRigidImpactStep(sx::ContactSolverMethod::BoxedLcp);

  ASSERT_EQ(lcp.contactCount, 1u);
  EXPECT_TRUE(lcp.contactTouchesLink);
  EXPECT_TRUE(lcp.contactTouchesRigidBody);
  EXPECT_TRUE(std::isfinite(lcp.strikerVelocity));
  EXPECT_TRUE(std::isfinite(lcp.targetVelocity));
  EXPECT_LT(lcp.strikerVelocity, 1.0);
  EXPECT_GT(lcp.targetVelocity, 0.0);
  EXPECT_NEAR(lcp.momentum, 2.0, 1e-9);

  ASSERT_EQ(reference.contactCount, lcp.contactCount);
  EXPECT_TRUE(reference.contactTouchesLink);
  EXPECT_TRUE(reference.contactTouchesRigidBody);
  EXPECT_NEAR(reference.strikerVelocity, lcp.strikerVelocity, 1e-9);
  EXPECT_NEAR(reference.targetVelocity, lcp.targetVelocity, 1e-9);
  EXPECT_NEAR(reference.momentum, lcp.momentum, 1e-9);
}

//==============================================================================
// Four simultaneous two-sided articulated contacts match the first benchmark-
// sized link-vs-rigid impact packet and keep all impulses independent.
TEST(BoxedLcpContact, FourArticulatedPrismaticLinksPushDynamicRigidBodies)
{
  constexpr int kPairCount = 4;
  expectArticulatedRigidImpactPairsStepMaintainsInvariants(kPairCount);
}

//==============================================================================
// Eight simultaneous link-vs-rigid contacts cover the intermediate registered
// articulated rigid-impact benchmark packet.
TEST(BoxedLcpContact, EightArticulatedPrismaticLinksPushDynamicRigidBodies)
{
  constexpr int kPairCount = 8;
  expectArticulatedRigidImpactPairsStepMaintainsInvariants(kPairCount);
}

//==============================================================================
// Sixteen simultaneous link-vs-rigid contacts match the largest registered
// articulated rigid-impact benchmark packet.
TEST(BoxedLcpContact, SixteenArticulatedPrismaticLinksPushDynamicRigidBodies)
{
  constexpr int kPairCount = 16;
  expectArticulatedRigidImpactPairsStepMaintainsInvariants(kPairCount);
}

//==============================================================================
// The same largest link-vs-rigid packet remains stable across a longer public
// step sequence after the initial impulse separates each pair.
TEST(
    BoxedLcpContact,
    SixteenArticulatedPrismaticLinksPushDynamicRigidBodiesForManySteps)
{
  constexpr int kPairCount = 16;
  constexpr int kStepCount = 200;
  expectArticulatedRigidImpactPairsStepMaintainsInvariants(
      kPairCount, kStepCount);
}

//==============================================================================
// Cross-multibody articulated contact: a prismatic link pushes a prismatic link
// owned by a separate multibody. This exercises the unified row's second
// articulated endpoint rather than a dynamic rigid-body endpoint.
TEST(BoxedLcpContact, ArticulatedPrismaticLinkPushesArticulatedPrismaticLink)
{
  const ArticulatedLinkImpactResult reference = runArticulatedLinkImpactStep(
      sx::ContactSolverMethod::SequentialImpulse);
  const ArticulatedLinkImpactResult lcp
      = runArticulatedLinkImpactStep(sx::ContactSolverMethod::BoxedLcp);

  ASSERT_EQ(lcp.contactCount, 1u);
  EXPECT_TRUE(lcp.contactTouchesTwoLinks);
  EXPECT_TRUE(std::isfinite(lcp.strikerVelocity));
  EXPECT_TRUE(std::isfinite(lcp.targetVelocity));
  EXPECT_LT(lcp.strikerVelocity, 1.0);
  EXPECT_GT(lcp.targetVelocity, 0.0);
  EXPECT_GE(lcp.targetVelocity - lcp.strikerVelocity, -1e-9);
  EXPECT_NEAR(lcp.momentum, 2.0, 1e-9);

  ASSERT_EQ(reference.contactCount, lcp.contactCount);
  EXPECT_TRUE(reference.contactTouchesTwoLinks);
  EXPECT_NEAR(reference.strikerVelocity, lcp.strikerVelocity, 1e-9);
  EXPECT_NEAR(reference.targetVelocity, lcp.targetVelocity, 1e-9);
  EXPECT_NEAR(reference.momentum, lcp.momentum, 1e-9);
}

//==============================================================================
// Four simultaneous cross-multibody articulated contacts match the first
// benchmark-sized link-vs-link impact packet and exercise two articulated
// endpoints per contact.
TEST(
    BoxedLcpContact, FourArticulatedPrismaticLinksPushArticulatedPrismaticLinks)
{
  constexpr int kPairCount = 4;
  expectArticulatedLinkImpactPairsStepMaintainsInvariants(kPairCount);
}

//==============================================================================
// Eight simultaneous cross-multibody contacts cover the intermediate registered
// articulated link-impact benchmark packet.
TEST(
    BoxedLcpContact,
    EightArticulatedPrismaticLinksPushArticulatedPrismaticLinks)
{
  constexpr int kPairCount = 8;
  expectArticulatedLinkImpactPairsStepMaintainsInvariants(kPairCount);
}

//==============================================================================
// Sixteen simultaneous cross-multibody contacts match the largest registered
// articulated link-impact benchmark packet.
TEST(
    BoxedLcpContact,
    SixteenArticulatedPrismaticLinksPushArticulatedPrismaticLinks)
{
  constexpr int kPairCount = 16;
  expectArticulatedLinkImpactPairsStepMaintainsInvariants(kPairCount);
}

//==============================================================================
// The largest cross-multibody articulated packet also stays stable across a
// longer public step sequence after the initial link-vs-link impulse.
TEST(
    BoxedLcpContact,
    SixteenArticulatedPrismaticLinksPushArticulatedPrismaticLinksForManySteps)
{
  constexpr int kPairCount = 16;
  constexpr int kStepCount = 200;
  expectArticulatedLinkImpactPairsStepMaintainsInvariants(
      kPairCount, kStepCount);
}

//==============================================================================
// Kinetic-friction parity: a box sliding horizontally on a frictional ground
// decelerates, and the BoxedLcp path's slowing behavior tracks the
// SequentialImpulse path within a documented tolerance. Friction mu = 0.5.
TEST(BoxedLcpContact, SlidingBoxDeceleratesLikeSequentialImpulse)
{
  constexpr double kFriction = 0.5;
  const Eigen::Vector3d push(1.0, 0.0, 0.0);
  auto reference = buildFrictionScene(
      sx::ContactSolverMethod::SequentialImpulse, kFriction, push);
  auto lcp
      = buildFrictionScene(sx::ContactSolverMethod::BoxedLcp, kFriction, push);

  const std::vector<sx::Contact> lcpContacts = lcp->collide();
  ASSERT_GE(lcpContacts.size(), 4u);

  reference->enterSimulationMode();
  lcp->enterSimulationMode();
  reference->step(200);
  lcp->step(200);

  const double referenceVx
      = reference->getRigidBody("box")->getLinearVelocity().x();
  const double lcpVx = lcp->getRigidBody("box")->getLinearVelocity().x();

  // Friction removed forward speed in both paths (the box slowed markedly).
  EXPECT_LT(lcpVx, 0.9);
  EXPECT_GE(lcpVx, -1e-6); // friction does not reverse the slide

  // The BoxedLcp slowing tracks the SequentialImpulse path. The two solvers use
  // different inner iteration schemes (pivoting boxed LCP vs Gauss-Seidel box
  // friction), so the documented parity tolerance is loose (absolute 0.15 m/s
  // on the residual forward speed).
  EXPECT_NEAR(lcpVx, referenceVx, 0.15);

  // The box stayed on the ground (no spurious vertical drift) under the LCP
  // path.
  EXPECT_NEAR(lcp->getRigidBody("box")->getTranslation().z(), 0.5, 1e-2);
}

//==============================================================================
// Static-friction hold: a small tangential push well inside the friction cone
// is fully resisted, so the box stays essentially in place under the BoxedLcp
// path. Friction mu = 0.8 with a tiny initial velocity.
TEST(BoxedLcpContact, StaticFrictionHoldsSmallPush)
{
  constexpr double kFriction = 0.8;
  // A small tangential velocity that one friction impulse can fully cancel.
  const Eigen::Vector3d smallPush(0.02, 0.0, 0.0);
  auto lcp = buildFrictionScene(
      sx::ContactSolverMethod::BoxedLcp, kFriction, smallPush);

  const std::vector<sx::Contact> contacts = lcp->collide();
  ASSERT_GE(contacts.size(), 4u);

  lcp->enterSimulationMode();

  const double startX = lcp->getRigidBody("box")->getTranslation().x();
  lcp->step(200);
  const double endX = lcp->getRigidBody("box")->getTranslation().x();
  const double endVx = lcp->getRigidBody("box")->getLinearVelocity().x();

  // Static friction held the box: tangential velocity is driven to ~zero and
  // the box barely moved over the whole run.
  EXPECT_LT(std::abs(endVx), 1e-3);
  EXPECT_LT(std::abs(endX - startX), 5e-3);
}
