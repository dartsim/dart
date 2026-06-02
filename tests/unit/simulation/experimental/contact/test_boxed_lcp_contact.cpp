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

#include <dart/simulation/experimental/body/collision_shape.hpp>
#include <dart/simulation/experimental/body/rigid_body.hpp>
#include <dart/simulation/experimental/body/rigid_body_options.hpp>
#include <dart/simulation/experimental/comps/joint.hpp>
#include <dart/simulation/experimental/comps/rigid_body.hpp>
#include <dart/simulation/experimental/detail/deformable_vbd/rigid_world_contact.hpp>
#include <dart/simulation/experimental/detail/entity_conversion.hpp>
#include <dart/simulation/experimental/world.hpp>
#include <dart/simulation/experimental/world_options.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <gtest/gtest.h>

#include <memory>
#include <vector>

#include <cmath>

namespace sx = dart::simulation::experimental;
namespace dvbd = dart::simulation::experimental::detail::deformable_vbd;

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
        == compound.getEntity();
  ASSERT_TRUE(
      compoundIsA
      || sx::detail::toRegistryEntity(contact.bodyB.getEntity())
             == compound.getEntity());
  const std::size_t compoundShapeIndex
      = compoundIsA ? contact.shapeIndexA : contact.shapeIndexB;
  const Eigen::Vector3d compoundLocalPoint
      = compoundIsA ? contact.localPointA : contact.localPointB;
  ASSERT_EQ(compoundShapeIndex, 1u);
  ASSERT_TRUE(compoundLocalPoint.allFinite());

  const dvbd::AvbdRigidWorldContactSnapshot snapshot
      = dvbd::buildAvbdRigidWorldContactSnapshot(
          world.getRegistry(), contacts, dvbd::AvbdRigidWorldContactOptions{});
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
  avbd->getRegistry().emplace_or_replace<sx::comps::RigidAvbdContactConfig>(
      sphere->getEntity());
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

  auto& registry = avbd->getRegistry();
  registry.emplace_or_replace<sx::comps::RigidAvbdContactConfig>(
      sphere->getEntity());

  const entt::entity jointEntity = registry.create();
  auto& joint = registry.emplace<sx::comps::Joint>(jointEntity);
  joint.type = sx::comps::JointType::Fixed;
  joint.parentLink = ground->getEntity();
  joint.childLink = sphere->getEntity();

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

  auto& registry = world.getRegistry();
  const entt::entity jointEntity = registry.create();
  auto& joint = registry.emplace<sx::comps::Joint>(jointEntity);
  joint.type = sx::comps::JointType::Fixed;
  joint.parentLink = base.getEntity();
  joint.childLink = link.getEntity();

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

  auto& registry = world.getRegistry();
  const entt::entity jointEntity = registry.create();
  auto& joint = registry.emplace<sx::comps::Joint>(jointEntity);
  joint.type = sx::comps::JointType::Fixed;
  joint.parentLink = base.getEntity();
  joint.childLink = link.getEntity();

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

  auto& registry = world->getRegistry();
  const entt::entity jointEntity = registry.create();
  auto& joint = registry.emplace<sx::comps::Joint>(jointEntity);
  joint.type = sx::comps::JointType::Fixed;
  joint.parentLink = base.getEntity();
  joint.childLink = link.getEntity();

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

} // namespace

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
