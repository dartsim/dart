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
 *     copyright notice and this list of conditions in the documentation
 *     and/or other materials provided with the distribution.
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

#include <dart/simulation/experimental/body/collision_shape.hpp>
#include <dart/simulation/experimental/body/deformable_body.hpp>
#include <dart/simulation/experimental/body/rigid_body.hpp>
#include <dart/simulation/experimental/common/exceptions.hpp>
#include <dart/simulation/experimental/compute/sequential_executor.hpp>
#include <dart/simulation/experimental/compute/world_step_stage.hpp>
#include <dart/simulation/experimental/world.hpp>

#include <gtest/gtest.h>

#include <limits>
#include <sstream>

namespace sx = dart::simulation::experimental;
namespace compute = dart::simulation::experimental::compute;

namespace {

//==============================================================================
void expectVectorNear(
    const Eigen::Vector3d& actual,
    const Eigen::Vector3d& expected,
    double tolerance = 1e-12)
{
  EXPECT_NEAR(actual.x(), expected.x(), tolerance);
  EXPECT_NEAR(actual.y(), expected.y(), tolerance);
  EXPECT_NEAR(actual.z(), expected.z(), tolerance);
}

//==============================================================================
sx::DeformableBodyOptions makeTwoNodeBody()
{
  sx::DeformableBodyOptions options;
  options.positions = {Eigen::Vector3d::Zero(), Eigen::Vector3d::UnitX()};
  options.edges = {sx::DeformableEdge{0, 1, -1.0}};
  options.fixedNodes = {0};
  return options;
}

} // namespace

//==============================================================================
TEST(DeformableBody, AddGetAndExposeNodeStateWithoutSolverDetails)
{
  sx::World world;

  auto options = makeTwoNodeBody();
  options.velocities = {Eigen::Vector3d::Zero(), Eigen::Vector3d::UnitY()};
  options.masses = {2.0, 3.0};

  auto body = world.addDeformableBody("strand", options);

  EXPECT_TRUE(body.isValid());
  EXPECT_EQ(body.getName(), "strand");
  EXPECT_TRUE(world.hasDeformableBody("strand"));
  EXPECT_EQ(world.getDeformableBodyCount(), 1u);

  auto restored = world.getDeformableBody("strand");
  ASSERT_TRUE(restored.has_value());
  EXPECT_TRUE(restored->isValid());
  EXPECT_EQ(restored->getNodeCount(), 2u);
  expectVectorNear(restored->getPosition(0), Eigen::Vector3d::Zero());
  expectVectorNear(restored->getVelocity(1), Eigen::Vector3d::UnitY());
  EXPECT_DOUBLE_EQ(restored->getMass(0), 2.0);
  EXPECT_TRUE(restored->isFixedNode(0));
  EXPECT_FALSE(restored->isFixedNode(1));

  ASSERT_EQ(restored->getEdgeCount(), 1u);
  const auto edge = restored->getEdge(0);
  EXPECT_EQ(edge.nodeA, 0u);
  EXPECT_EQ(edge.nodeB, 1u);
  EXPECT_DOUBLE_EQ(edge.restLength, 1.0);
}

//==============================================================================
TEST(DeformableBody, RejectsInvalidModels)
{
  sx::World world;

  sx::DeformableBodyOptions empty;
  EXPECT_THROW(
      world.addDeformableBody("empty", empty), sx::InvalidArgumentException);

  auto mismatchedVelocity = makeTwoNodeBody();
  mismatchedVelocity.velocities = {Eigen::Vector3d::Zero()};
  EXPECT_THROW(
      world.addDeformableBody("bad_velocity", mismatchedVelocity),
      sx::InvalidArgumentException);

  auto nonfinitePosition = makeTwoNodeBody();
  nonfinitePosition.positions[1].x() = std::numeric_limits<double>::infinity();
  EXPECT_THROW(
      world.addDeformableBody("bad_position", nonfinitePosition),
      sx::InvalidArgumentException);

  auto nonfiniteVelocity = makeTwoNodeBody();
  nonfiniteVelocity.velocities
      = {Eigen::Vector3d::Zero(),
         Eigen::Vector3d(std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0)};
  EXPECT_THROW(
      world.addDeformableBody("bad_finite_velocity", nonfiniteVelocity),
      sx::InvalidArgumentException);

  auto invalidMass = makeTwoNodeBody();
  invalidMass.masses = {1.0, -1.0};
  EXPECT_THROW(
      world.addDeformableBody("bad_mass", invalidMass),
      sx::InvalidArgumentException);

  auto zeroMass = makeTwoNodeBody();
  zeroMass.masses = {1.0, 0.0};
  EXPECT_THROW(
      world.addDeformableBody("zero_mass", zeroMass),
      sx::InvalidArgumentException);

  auto nonfiniteMass = makeTwoNodeBody();
  nonfiniteMass.masses = {1.0, std::numeric_limits<double>::quiet_NaN()};
  EXPECT_THROW(
      world.addDeformableBody("bad_finite_mass", nonfiniteMass),
      sx::InvalidArgumentException);

  auto duplicateFixed = makeTwoNodeBody();
  duplicateFixed.fixedNodes = {0, 0};
  EXPECT_THROW(
      world.addDeformableBody("duplicate_fixed", duplicateFixed),
      sx::InvalidArgumentException);

  auto invalidFixed = makeTwoNodeBody();
  invalidFixed.fixedNodes = {2};
  EXPECT_THROW(
      world.addDeformableBody("bad_fixed", invalidFixed),
      sx::InvalidArgumentException);

  auto invalidEdge = makeTwoNodeBody();
  invalidEdge.edges = {sx::DeformableEdge{0, 2, 1.0}};
  EXPECT_THROW(
      world.addDeformableBody("bad_edge", invalidEdge),
      sx::InvalidArgumentException);

  auto coincidentEdge = makeTwoNodeBody();
  coincidentEdge.positions[1] = Eigen::Vector3d::Zero();
  EXPECT_THROW(
      world.addDeformableBody("coincident", coincidentEdge),
      sx::InvalidArgumentException);

  auto negativeStiffness = makeTwoNodeBody();
  negativeStiffness.edgeStiffness = -1.0;
  EXPECT_THROW(
      world.addDeformableBody("bad_stiffness", negativeStiffness),
      sx::InvalidArgumentException);

  auto nonfiniteStiffness = makeTwoNodeBody();
  nonfiniteStiffness.edgeStiffness = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(
      world.addDeformableBody("bad_finite_stiffness", nonfiniteStiffness),
      sx::InvalidArgumentException);

  auto negativeDamping = makeTwoNodeBody();
  negativeDamping.damping = -1.0;
  EXPECT_THROW(
      world.addDeformableBody("bad_damping", negativeDamping),
      sx::InvalidArgumentException);

  auto nonfiniteDamping = makeTwoNodeBody();
  nonfiniteDamping.damping = std::numeric_limits<double>::infinity();
  EXPECT_THROW(
      world.addDeformableBody("bad_finite_damping", nonfiniteDamping),
      sx::InvalidArgumentException);
}

//==============================================================================
TEST(DeformableBody, RestLengthSpringIsStationaryWithoutLoads)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());
  world.setTimeStep(0.1);

  auto body = world.addDeformableBody("rest", makeTwoNodeBody());

  world.step(3);

  expectVectorNear(body.getPosition(0), Eigen::Vector3d::Zero());
  expectVectorNear(body.getPosition(1), Eigen::Vector3d::UnitX());
  expectVectorNear(body.getVelocity(0), Eigen::Vector3d::Zero());
  expectVectorNear(body.getVelocity(1), Eigen::Vector3d::Zero());
}

//==============================================================================
TEST(DeformableBody, FixedNodeStaysFixedWhileStretchedEdgeContracts)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d(0.0, 0.0, -9.81));
  world.setTimeStep(0.1);

  auto options = makeTwoNodeBody();
  options.positions[1] = Eigen::Vector3d(2.0, 0.0, 0.0);
  options.edges = {sx::DeformableEdge{0, 1, 1.0}};
  options.edgeStiffness = 100.0;
  auto body = world.addDeformableBody("stretch", options);

  world.step();

  expectVectorNear(body.getPosition(0), Eigen::Vector3d::Zero());
  expectVectorNear(body.getVelocity(0), Eigen::Vector3d::Zero());
  EXPECT_LT(body.getPosition(1).x(), 2.0);
  EXPECT_GT(body.getPosition(1).x(), 0.99);
  EXPECT_LT(body.getPosition(1).z(), 0.0);
}

//==============================================================================
TEST(DeformableBody, FixedSpringMatchesAnalyticImplicitEulerStep)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());
  world.setTimeStep(0.1);

  auto options = makeTwoNodeBody();
  options.positions[1] = Eigen::Vector3d(2.0, 0.0, 0.0);
  options.edges = {sx::DeformableEdge{0, 1, 1.0}};
  options.edgeStiffness = 100.0;
  options.masses = {1.0, 1.0};
  auto body = world.addDeformableBody("analytic_spring", options);

  world.step();

  constexpr double mass = 1.0;
  constexpr double timeStep = 0.1;
  constexpr double stiffness = 100.0;
  constexpr double initialPosition = 2.0;
  constexpr double restLength = 1.0;
  constexpr double inertialWeight = mass / (timeStep * timeStep);
  constexpr double expectedX
      = (inertialWeight * initialPosition + stiffness * restLength)
        / (inertialWeight + stiffness);

  expectVectorNear(body.getPosition(1), Eigen::Vector3d(expectedX, 0.0, 0.0));
  expectVectorNear(
      body.getVelocity(1),
      Eigen::Vector3d((expectedX - initialPosition) / timeStep, 0.0, 0.0));
}

//==============================================================================
TEST(DeformableBody, FreeParticleMatchesImplicitEulerTargetForDifferentMasses)
{
  const Eigen::Vector3d expectedPosition
      = Eigen::Vector3d(0.5, -0.25, 1.0) + 0.1 * Eigen::Vector3d(1.0, 2.0, 3.0)
        + 0.01 * Eigen::Vector3d(0.0, 0.0, -9.81);
  const Eigen::Vector3d expectedVelocity
      = (expectedPosition - Eigen::Vector3d(0.5, -0.25, 1.0)) / 0.1;

  sx::World lightWorld;
  lightWorld.setGravity(Eigen::Vector3d(0.0, 0.0, -9.81));
  lightWorld.setTimeStep(0.1);
  sx::DeformableBodyOptions lightOptions;
  lightOptions.positions = {Eigen::Vector3d(0.5, -0.25, 1.0)};
  lightOptions.velocities = {Eigen::Vector3d(1.0, 2.0, 3.0)};
  lightOptions.masses = {0.01};
  auto light = lightWorld.addDeformableBody("light", lightOptions);

  sx::World heavyWorld;
  heavyWorld.setGravity(Eigen::Vector3d(0.0, 0.0, -9.81));
  heavyWorld.setTimeStep(0.1);
  sx::DeformableBodyOptions heavyOptions = lightOptions;
  heavyOptions.masses = {10.0};
  auto heavy = heavyWorld.addDeformableBody("heavy", heavyOptions);

  lightWorld.step();
  heavyWorld.step();

  expectVectorNear(light.getPosition(0), expectedPosition, 1e-12);
  expectVectorNear(heavy.getPosition(0), expectedPosition, 1e-12);
  expectVectorNear(light.getVelocity(0), expectedVelocity, 1e-12);
  expectVectorNear(heavy.getVelocity(0), expectedVelocity, 1e-12);
}

//==============================================================================
TEST(DeformableBody, StepCountWithExecutorRunsDefaultDeformableStage)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());
  world.setTimeStep(0.1);

  auto options = makeTwoNodeBody();
  options.positions[1] = Eigen::Vector3d(2.0, 0.0, 0.0);
  options.edges = {sx::DeformableEdge{0, 1, 1.0}};
  auto body = world.addDeformableBody("executor_step", options);

  compute::SequentialExecutor executor;
  world.step(2, executor);

  expectVectorNear(body.getPosition(0), Eigen::Vector3d::Zero());
  EXPECT_LT(body.getPosition(1).x(), 2.0);
  EXPECT_EQ(world.getFrame(), 2u);
}

//==============================================================================
TEST(DeformableBody, CustomStageStepOverloadsRunDefaultDeformableStage)
{
  auto addStretchedBody = [](sx::World& world) {
    world.setGravity(Eigen::Vector3d::Zero());
    world.setTimeStep(0.1);

    auto options = makeTwoNodeBody();
    options.positions[1] = Eigen::Vector3d(2.0, 0.0, 0.0);
    options.edges = {sx::DeformableEdge{0, 1, 1.0}};
    options.edgeStiffness = 100.0;
    return world.addDeformableBody("custom_stage", options);
  };

  compute::SequentialExecutor executor;
  compute::KinematicsStage replacementKinematics;

  sx::World defaultWorld;
  sx::World customWorld;
  auto defaultBody = addStretchedBody(defaultWorld);
  auto customBody = addStretchedBody(customWorld);

  defaultWorld.step(executor);
  customWorld.step(executor, replacementKinematics);

  expectVectorNear(customBody.getPosition(1), defaultBody.getPosition(1));
  expectVectorNear(customBody.getVelocity(1), defaultBody.getVelocity(1));
  EXPECT_LT(customBody.getPosition(1).x(), 2.0);
  EXPECT_DOUBLE_EQ(customWorld.getTime(), defaultWorld.getTime());
  EXPECT_EQ(customWorld.getFrame(), defaultWorld.getFrame());

  sx::World countedDefaultWorld;
  sx::World countedCustomWorld;
  auto countedDefaultBody = addStretchedBody(countedDefaultWorld);
  auto countedCustomBody = addStretchedBody(countedCustomWorld);

  countedDefaultWorld.step(2, executor);
  countedCustomWorld.step(2, executor, replacementKinematics);

  expectVectorNear(
      countedCustomBody.getPosition(1), countedDefaultBody.getPosition(1));
  expectVectorNear(
      countedCustomBody.getVelocity(1), countedDefaultBody.getVelocity(1));
  EXPECT_LT(countedCustomBody.getPosition(1).x(), 2.0);
  EXPECT_DOUBLE_EQ(countedCustomWorld.getTime(), countedDefaultWorld.getTime());
  EXPECT_EQ(countedCustomWorld.getFrame(), countedDefaultWorld.getFrame());
}

//==============================================================================
TEST(DeformableBody, StaticGroundBarrierPreventsCrossing)
{
  sx::World world;
  world.setTimeStep(0.1);

  sx::RigidBodyOptions groundOptions;
  groundOptions.isStatic = true;
  groundOptions.position = Eigen::Vector3d(0.0, 0.0, -0.5);
  auto ground = world.addRigidBody("ground", groundOptions);
  ground.setCollisionShape(
      sx::CollisionShape::makeBox(Eigen::Vector3d(10.0, 10.0, 0.5)));
  ground.setDeformableGroundBarrier(true);
  EXPECT_TRUE(ground.isDeformableGroundBarrier());

  sx::DeformableBodyOptions options;
  options.positions = {Eigen::Vector3d(0.0, 0.0, 0.5)};
  options.velocities = {Eigen::Vector3d(0.0, 0.0, -10.0)};
  auto body = world.addDeformableBody("falling_node", options);

  world.step(5);

  EXPECT_LT(body.getPosition(0).z(), 0.5);
  EXPECT_GE(body.getPosition(0).z(), 1e-4 - 1e-12);
}

//==============================================================================
TEST(DeformableBody, ActiveStaticGroundContactAllowsTangentialMotion)
{
  sx::World world;
  world.setTimeStep(0.1);

  sx::RigidBodyOptions groundOptions;
  groundOptions.isStatic = true;
  groundOptions.position = Eigen::Vector3d(0.0, 0.0, -0.5);
  auto ground = world.addRigidBody("ground", groundOptions);
  ground.setCollisionShape(
      sx::CollisionShape::makeBox(Eigen::Vector3d(10.0, 10.0, 0.5)));
  ground.setDeformableGroundBarrier(true);

  sx::DeformableBodyOptions options;
  options.positions = {Eigen::Vector3d(0.0, 0.0, 1e-4)};
  options.velocities = {Eigen::Vector3d(1.0, 0.0, 0.0)};
  auto body = world.addDeformableBody("sliding_node", options);

  world.step();

  EXPECT_GT(body.getPosition(0).x(), 0.0);
  EXPECT_GE(body.getPosition(0).z(), 1e-4 - 1e-12);
  EXPECT_GT(body.getVelocity(0).x(), 0.0);
}

//==============================================================================
TEST(DeformableBody, StaticGroundBarrierUsesFiniteStaticFootprint)
{
  sx::World world;
  world.setTimeStep(0.1);

  sx::RigidBodyOptions obstacleOptions;
  obstacleOptions.isStatic = true;
  obstacleOptions.position = Eigen::Vector3d(100.0, 0.0, 9.5);
  auto obstacle = world.addRigidBody("distant_obstacle", obstacleOptions);
  obstacle.setCollisionShape(
      sx::CollisionShape::makeBox(Eigen::Vector3d(1.0, 1.0, 0.5)));
  obstacle.setDeformableGroundBarrier(true);

  sx::DeformableBodyOptions options;
  options.positions = {Eigen::Vector3d(0.0, 0.0, 0.5)};
  auto body = world.addDeformableBody("falling_node", options);

  world.step();

  EXPECT_LT(body.getPosition(0).z(), 0.5);
  EXPECT_LT(body.getPosition(0).z(), 1.0);
}

//==============================================================================
TEST(DeformableBody, StaticGroundBarrierUsesOrientedBoxFootprint)
{
  sx::World world;
  world.setTimeStep(0.1);

  sx::RigidBodyOptions obstacleOptions;
  obstacleOptions.isStatic = true;
  obstacleOptions.orientation = Eigen::AngleAxisd(
      0.25 * 3.14159265358979323846, Eigen::Vector3d::UnitZ());
  auto obstacle = world.addRigidBody("rotated_obstacle", obstacleOptions);
  obstacle.setCollisionShape(
      sx::CollisionShape::makeBox(Eigen::Vector3d(2.0, 2.0, 1.0)));
  obstacle.setDeformableGroundBarrier(true);

  sx::DeformableBodyOptions options;
  options.positions = {Eigen::Vector3d(2.5, 1.0, 0.25)};
  auto body = world.addDeformableBody("outside_rotated_footprint", options);

  world.step();

  EXPECT_LT(body.getPosition(0).z(), 0.25);
}

//==============================================================================
TEST(DeformableBody, StaticGroundBarrierUsesTiltedBoxSurfaceHeight)
{
  sx::World world;
  world.setTimeStep(0.1);

  sx::RigidBodyOptions obstacleOptions;
  obstacleOptions.isStatic = true;
  obstacleOptions.orientation = Eigen::AngleAxisd(
      0.25 * 3.14159265358979323846, Eigen::Vector3d::UnitY());
  auto obstacle = world.addRigidBody("tilted_obstacle", obstacleOptions);
  obstacle.setCollisionShape(
      sx::CollisionShape::makeBox(Eigen::Vector3d(2.0, 2.0, 0.2)));
  obstacle.setDeformableGroundBarrier(true);

  sx::DeformableBodyOptions options;
  options.positions = {Eigen::Vector3d(0.0, 0.0, 0.35)};
  auto body = world.addDeformableBody("above_tilted_surface", options);

  world.step();

  EXPECT_LT(body.getPosition(0).z(), 0.35);
  EXPECT_LT(body.getPosition(0).z(), 0.5);
}

//==============================================================================
TEST(DeformableBody, StaticCollisionRequiresGroundBarrierOptIn)
{
  sx::World world;
  world.setTimeStep(0.1);

  sx::RigidBodyOptions ceilingOptions;
  ceilingOptions.isStatic = true;
  ceilingOptions.position = Eigen::Vector3d(0.0, 0.0, 0.5);
  auto ceiling = world.addRigidBody("ordinary_static_box", ceilingOptions);
  ceiling.setCollisionShape(
      sx::CollisionShape::makeBox(Eigen::Vector3d(10.0, 10.0, 0.5)));
  ASSERT_FALSE(ceiling.isDeformableGroundBarrier());

  sx::RigidBodyOptions sphereOptions;
  sphereOptions.isStatic = true;
  sphereOptions.position = Eigen::Vector3d(0.0, 0.0, 0.0);
  auto sphere = world.addRigidBody("ordinary_static_sphere", sphereOptions);
  sphere.setCollisionShape(sx::CollisionShape::makeSphere(1.0));
  ASSERT_FALSE(sphere.isDeformableGroundBarrier());

  sx::DeformableBodyOptions options;
  options.positions = {Eigen::Vector3d(0.0, 0.0, 0.25)};
  options.velocities = {Eigen::Vector3d(0.0, 0.0, -1.0)};
  auto body = world.addDeformableBody("falling_node", options);

  world.step();

  EXPECT_LT(body.getPosition(0).z(), 0.25);
  EXPECT_LT(body.getPosition(0).z(), 1.0);
}

//==============================================================================
TEST(DeformableBody, StageMetadataUsesDeformableDomain)
{
  compute::DeformableDynamicsStage stage;

  EXPECT_EQ(stage.getName(), "deformable_dynamics");
  EXPECT_EQ(
      stage.getMetadata().domain, compute::ComputeStageDomain::DeformableBody);
}

//==============================================================================
TEST(DeformableBody, StepIsDeterministic)
{
  auto addBody = [](sx::World& world) {
    world.setGravity(Eigen::Vector3d(0.0, 0.0, -1.0));
    world.setTimeStep(0.05);

    sx::DeformableBodyOptions options;
    options.positions
        = {Eigen::Vector3d::Zero(),
           Eigen::Vector3d(1.5, 0.2, 0.0),
           Eigen::Vector3d(2.0, 0.5, 0.1)};
    options.velocities
        = {Eigen::Vector3d::Zero(),
           Eigen::Vector3d(0.1, 0.0, 0.0),
           Eigen::Vector3d(0.0, -0.1, 0.0)};
    options.edges
        = {sx::DeformableEdge{0, 1, 1.0}, sx::DeformableEdge{1, 2, 0.75}};
    options.fixedNodes = {0};
    options.edgeStiffness = 25.0;
    return world.addDeformableBody("chain", options);
  };

  sx::World worldA;
  sx::World worldB;
  auto bodyA = addBody(worldA);
  auto bodyB = addBody(worldB);

  worldA.step(10);
  worldB.step(10);

  for (std::size_t i = 0; i < bodyA.getNodeCount(); ++i) {
    expectVectorNear(bodyA.getPosition(i), bodyB.getPosition(i));
    expectVectorNear(bodyA.getVelocity(i), bodyB.getVelocity(i));
  }
}

//==============================================================================
TEST(DeformableBody, SerializationPreservesModelAndState)
{
  sx::World world1;
  auto body1 = world1.addDeformableBody("serialized", makeTwoNodeBody());
  body1.setVelocity(1, Eigen::Vector3d(0.25, 0.0, 0.0));

  std::stringstream stream;
  world1.saveBinary(stream);

  sx::World world2;
  world2.loadBinary(stream);

  ASSERT_EQ(world2.getDeformableBodyCount(), 1u);
  auto body2 = world2.getDeformableBody("serialized");
  ASSERT_TRUE(body2.has_value());
  ASSERT_EQ(body2->getNodeCount(), 2u);
  EXPECT_TRUE(body2->isFixedNode(0));
  expectVectorNear(body2->getPosition(1), Eigen::Vector3d::UnitX());
  expectVectorNear(body2->getVelocity(1), Eigen::Vector3d(0.25, 0.0, 0.0));
  ASSERT_EQ(body2->getEdgeCount(), 1u);
  EXPECT_DOUBLE_EQ(body2->getEdge(0).restLength, 1.0);
}
