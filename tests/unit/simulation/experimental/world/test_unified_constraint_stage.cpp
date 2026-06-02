/*
 * Copyright (c) 2011, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/main/LICENSE
 *
 * This file is provided under the "BSD-style" License
 */

#include <dart/simulation/experimental/body/collision_shape.hpp>
#include <dart/simulation/experimental/body/contact.hpp>
#include <dart/simulation/experimental/body/rigid_body.hpp>
#include <dart/simulation/experimental/comps/dynamics.hpp>
#include <dart/simulation/experimental/compute/multibody_dynamics.hpp>
#include <dart/simulation/experimental/compute/sequential_executor.hpp>
#include <dart/simulation/experimental/world.hpp>

#include <gtest/gtest.h>

namespace {

namespace sx = dart::simulation::experimental;

//==============================================================================
// The unified constraint stage, run directly, resolves a rigid-body contact: a
// sphere overlapping a static ground has its downward velocity arrested and is
// projected out of penetration. This exercises the stage's orchestration:
// collision query, rigid-rigid assembly, the joint solve, impulse application,
// and the rigid positional projection. Link-side orchestration is exercised by
// the full emergent world suite through the default pipeline.
TEST(UnifiedConstraintStage, ArrestsAndSeparatesOverlappingRigidContact)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());

  sx::RigidBodyOptions groundOptions;
  groundOptions.isStatic = true;
  groundOptions.position = Eigen::Vector3d(0.0, 0.0, -1.0); // top at z = -0.5
  auto ground = world.addRigidBody("ground", groundOptions);
  ground.setCollisionShape(
      sx::CollisionShape::makeBox(Eigen::Vector3d(5.0, 5.0, 0.5)));

  // Sphere (radius 0.2) centered at z = -0.31, so its bottom (-0.51) overlaps
  // the ground top (-0.5) by 0.01; it is descending.
  sx::RigidBodyOptions ballOptions;
  ballOptions.position = Eigen::Vector3d(0.0, 0.0, -0.31);
  ballOptions.linearVelocity = Eigen::Vector3d(0.0, 0.0, -1.0);
  auto ball = world.addRigidBody("ball", ballOptions);
  ball.setCollisionShape(sx::CollisionShape::makeSphere(0.2));

  ASSERT_FALSE(world.collide().empty());
  auto& registry = world.getRegistry();
  const double verticalBefore
      = registry.get<sx::comps::Transform>(ball.getEntity()).position.z();

  sx::compute::UnifiedConstraintStage stage;
  sx::compute::SequentialExecutor executor;
  stage.execute(world, executor);

  // No restitution: the approaching normal velocity is driven to zero.
  EXPECT_NEAR(
      registry.get<sx::comps::Velocity>(ball.getEntity()).linear.z(),
      0.0,
      1e-9);
  // The positional projection pushes the penetrating sphere upward.
  EXPECT_GT(
      registry.get<sx::comps::Transform>(ball.getEntity()).position.z(),
      verticalBefore);
}

//==============================================================================
// With no contacts the stage is a no-op: a single free body keeps its velocity.
TEST(UnifiedConstraintStage, LeavesContactFreeBodiesUntouched)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());

  sx::RigidBodyOptions options;
  options.position = Eigen::Vector3d(0.0, 0.0, 5.0);
  options.linearVelocity = Eigen::Vector3d(1.0, -2.0, 3.0);
  auto body = world.addRigidBody("free", options);
  body.setCollisionShape(sx::CollisionShape::makeSphere(0.2));

  ASSERT_TRUE(world.collide().empty());

  sx::compute::UnifiedConstraintStage stage;
  sx::compute::SequentialExecutor executor;
  stage.execute(world, executor);

  EXPECT_TRUE(world.getRegistry()
                  .get<sx::comps::Velocity>(body.getEntity())
                  .linear.isApprox(Eigen::Vector3d(1.0, -2.0, 3.0), 1e-12));
}

} // namespace
