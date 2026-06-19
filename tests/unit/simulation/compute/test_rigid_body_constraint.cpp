/*
 * Copyright (c) 2011, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/main/LICENSE
 *
 * This file is provided under the "BSD-style" License
 */

#include <dart/simulation/body/contact.hpp>
#include <dart/simulation/body/rigid_body.hpp>
#include <dart/simulation/compute/rigid_body_constraint.hpp>
#include <dart/simulation/detail/entity_conversion.hpp>
#include <dart/simulation/detail/world_registry_access.hpp>
#include <dart/simulation/world.hpp>

#include <gtest/gtest.h>

#include <span>
#include <vector>

#include <cmath>

namespace {

//==============================================================================
void expectMatrixExactlyEqual(
    const Eigen::MatrixXd& lhs, const Eigen::MatrixXd& rhs)
{
  ASSERT_EQ(lhs.rows(), rhs.rows());
  ASSERT_EQ(lhs.cols(), rhs.cols());
  for (Eigen::Index row = 0; row < lhs.rows(); ++row) {
    for (Eigen::Index col = 0; col < lhs.cols(); ++col) {
      EXPECT_EQ(lhs(row, col), rhs(row, col))
          << "at (" << row << ", " << col << ")";
    }
  }
}

//==============================================================================
void expectVectorExactlyEqual(
    const Eigen::VectorXd& lhs, const Eigen::VectorXd& rhs)
{
  ASSERT_EQ(lhs.size(), rhs.size());
  for (Eigen::Index i = 0; i < lhs.size(); ++i) {
    EXPECT_EQ(lhs[i], rhs[i]) << "at " << i;
  }
}

//==============================================================================
void expectVectorExactlyEqual(
    const Eigen::VectorXi& lhs, const Eigen::VectorXi& rhs)
{
  ASSERT_EQ(lhs.size(), rhs.size());
  for (Eigen::Index i = 0; i < lhs.size(); ++i) {
    EXPECT_EQ(lhs[i], rhs[i]) << "at " << i;
  }
}

} // namespace

//==============================================================================
TEST(RigidBodyConstraint, AssemblesRigidOnlyStackRowsDeterministically)
{
  namespace sx = dart::simulation;

  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());

  sx::RigidBodyOptions groundOptions;
  groundOptions.isStatic = true;
  groundOptions.position = Eigen::Vector3d(0.0, 0.0, -0.5);
  auto ground = world.addRigidBody("ground", groundOptions);
  ground.setFriction(0.25);

  sx::RigidBodyOptions lowerOptions;
  lowerOptions.position = Eigen::Vector3d(0.0, 0.0, 0.5);
  lowerOptions.linearVelocity = Eigen::Vector3d(0.0, 0.0, -1.0);
  auto lower = world.addRigidBody("lower", lowerOptions);
  lower.setFriction(1.0);

  sx::RigidBodyOptions upperOptions;
  upperOptions.position = Eigen::Vector3d(0.0, 0.0, 1.5);
  upperOptions.linearVelocity = Eigen::Vector3d(0.0, 0.0, -2.0);
  auto upper = world.addRigidBody("upper", upperOptions);
  upper.setFriction(0.25);

  std::vector<sx::Contact> contacts;
  sx::Contact groundLower;
  groundLower.bodyA = sx::CollisionBody(ground.getEntity(), &world);
  groundLower.bodyB = sx::CollisionBody(lower.getEntity(), &world);
  groundLower.point = Eigen::Vector3d(0.0, 0.0, 0.0);
  groundLower.normal = Eigen::Vector3d::UnitZ();
  groundLower.depth = 0.02;
  contacts.push_back(groundLower);

  sx::Contact lowerUpper;
  lowerUpper.bodyA = sx::CollisionBody(lower.getEntity(), &world);
  lowerUpper.bodyB = sx::CollisionBody(upper.getEntity(), &world);
  lowerUpper.point = Eigen::Vector3d(0.0, 0.0, 1.0);
  lowerUpper.normal = Eigen::Vector3d::UnitZ();
  lowerUpper.depth = 0.03;
  contacts.push_back(lowerUpper);

  const auto problem = sx::compute::assembleRigidBodyContactProblem(
      dart::simulation::detail::registryOf(world), contacts);
  const auto repeated = sx::compute::assembleRigidBodyContactProblem(
      dart::simulation::detail::registryOf(world), contacts);

  ASSERT_EQ(problem.constraints.size(), 2u);
  ASSERT_EQ(problem.delassus.rows(), 6);
  ASSERT_EQ(problem.delassus.cols(), 6);
  ASSERT_EQ(problem.rhs.size(), 6);
  ASSERT_EQ(problem.lo.size(), 6);
  ASSERT_EQ(problem.hi.size(), 6);
  ASSERT_EQ(problem.findex.size(), 6);

  expectMatrixExactlyEqual(problem.delassus, repeated.delassus);
  expectVectorExactlyEqual(problem.rhs, repeated.rhs);
  expectVectorExactlyEqual(problem.lo, repeated.lo);
  expectVectorExactlyEqual(problem.hi, repeated.hi);
  expectVectorExactlyEqual(problem.findex, repeated.findex);

  using sx::detail::toRegistryEntity;
  EXPECT_EQ(problem.constraints[0].bodyA, toRegistryEntity(ground.getEntity()));
  EXPECT_EQ(problem.constraints[0].bodyB, toRegistryEntity(lower.getEntity()));
  EXPECT_TRUE(problem.constraints[0].staticA);
  EXPECT_FALSE(problem.constraints[0].staticB);
  EXPECT_EQ(problem.constraints[1].bodyA, toRegistryEntity(lower.getEntity()));
  EXPECT_EQ(problem.constraints[1].bodyB, toRegistryEntity(upper.getEntity()));
  EXPECT_FALSE(problem.constraints[1].staticA);
  EXPECT_FALSE(problem.constraints[1].staticB);

  // Normal rows are rows 0 and 3. The shared lower body contributes an exact
  // negative off-diagonal, which is the rigid-only coupling later unified
  // assembly must preserve for multibody-free worlds.
  EXPECT_DOUBLE_EQ(problem.delassus(0, 0), 1.0);
  EXPECT_DOUBLE_EQ(problem.delassus(0, 3), -1.0);
  EXPECT_DOUBLE_EQ(problem.delassus(3, 0), -1.0);
  EXPECT_DOUBLE_EQ(problem.delassus(3, 3), 2.0);
  EXPECT_DOUBLE_EQ(problem.rhs[0], 1.0);
  EXPECT_DOUBLE_EQ(problem.rhs[3], 1.0);

  EXPECT_DOUBLE_EQ(problem.lo[0], 0.0);
  EXPECT_TRUE(std::isinf(problem.hi[0]));
  EXPECT_EQ(problem.findex[0], -1);
  EXPECT_DOUBLE_EQ(problem.lo[1], -0.5);
  EXPECT_DOUBLE_EQ(problem.hi[1], 0.5);
  EXPECT_EQ(problem.findex[1], 0);
  EXPECT_DOUBLE_EQ(problem.lo[2], -0.5);
  EXPECT_DOUBLE_EQ(problem.hi[2], 0.5);
  EXPECT_EQ(problem.findex[2], 0);

  EXPECT_DOUBLE_EQ(problem.lo[3], 0.0);
  EXPECT_TRUE(std::isinf(problem.hi[3]));
  EXPECT_EQ(problem.findex[3], -1);
  EXPECT_DOUBLE_EQ(problem.lo[4], -0.5);
  EXPECT_DOUBLE_EQ(problem.hi[4], 0.5);
  EXPECT_EQ(problem.findex[4], 3);
  EXPECT_DOUBLE_EQ(problem.lo[5], -0.5);
  EXPECT_DOUBLE_EQ(problem.hi[5], 0.5);
  EXPECT_EQ(problem.findex[5], 3);
}

//==============================================================================
TEST(RigidBodyConstraint, TreatsKinematicBodiesAsPrescribed)
{
  namespace sx = dart::simulation;

  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());

  sx::RigidBodyOptions kinematicOptions;
  kinematicOptions.mass = 2.0;
  kinematicOptions.inertia = 2.0 * Eigen::Matrix3d::Identity();
  kinematicOptions.position = Eigen::Vector3d::Zero();
  kinematicOptions.linearVelocity = Eigen::Vector3d(0.0, 0.0, 3.0);
  kinematicOptions.angularVelocity = Eigen::Vector3d(0.0, 4.0, 0.0);
  auto kinematic = world.addRigidBody("kinematic", kinematicOptions);
  kinematic.setKinematic(true);

  sx::RigidBodyOptions dynamicOptions;
  dynamicOptions.mass = 4.0;
  dynamicOptions.position = Eigen::Vector3d(0.0, 0.0, 1.0);
  dynamicOptions.linearVelocity = Eigen::Vector3d(0.0, 0.0, -2.0);
  auto dynamic = world.addRigidBody("dynamic", dynamicOptions);

  sx::Contact contact;
  contact.bodyA = sx::CollisionBody(kinematic.getEntity(), &world);
  contact.bodyB = sx::CollisionBody(dynamic.getEntity(), &world);
  contact.point = Eigen::Vector3d(0.0, 0.0, 0.5);
  contact.normal = Eigen::Vector3d::UnitZ();
  contact.depth = 0.01;

  const std::vector<sx::Contact> contacts{contact};
  const auto problem = sx::compute::assembleRigidBodyContactProblem(
      dart::simulation::detail::registryOf(world), contacts);

  ASSERT_EQ(problem.constraints.size(), 1u);
  const auto& constraint = problem.constraints[0];
  EXPECT_EQ(
      constraint.bodyA, sx::detail::toRegistryEntity(kinematic.getEntity()));
  EXPECT_EQ(
      constraint.bodyB, sx::detail::toRegistryEntity(dynamic.getEntity()));
  EXPECT_TRUE(constraint.staticA);
  EXPECT_FALSE(constraint.staticB);
  EXPECT_DOUBLE_EQ(constraint.invMassA, 0.0);
  EXPECT_TRUE(constraint.invInertiaA.isZero(0.0));
  EXPECT_DOUBLE_EQ(constraint.invMassB, 0.25);
  EXPECT_TRUE(
      constraint.invInertiaB.isApprox(Eigen::Matrix3d::Identity(), 1e-12));

  EXPECT_DOUBLE_EQ(problem.delassus(0, 0), 0.25);
  EXPECT_DOUBLE_EQ(problem.rhs[0], 2.0);

  const Eigen::Vector3d kinematicLinearBefore = kinematic.getLinearVelocity();
  const Eigen::Vector3d kinematicAngularBefore = kinematic.getAngularVelocity();
  const Eigen::Vector3d dynamicLinearBefore = dynamic.getLinearVelocity();

  sx::compute::applyRigidBodyContactImpulse(
      dart::simulation::detail::registryOf(world),
      constraint,
      Eigen::Vector3d::UnitZ());

  EXPECT_TRUE(
      kinematic.getLinearVelocity().isApprox(kinematicLinearBefore, 1e-12));
  EXPECT_TRUE(
      kinematic.getAngularVelocity().isApprox(kinematicAngularBefore, 1e-12));
  EXPECT_TRUE(dynamic.getLinearVelocity().isApprox(
      dynamicLinearBefore + Eigen::Vector3d(0.0, 0.0, 0.25), 1e-12));
}

//==============================================================================
TEST(RigidBodyConstraint, SupportsNormalFirstRowsAndJacobian)
{
  namespace sx = dart::simulation;

  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());

  sx::RigidBodyOptions groundOptions;
  groundOptions.isStatic = true;
  groundOptions.position = Eigen::Vector3d(0.0, 0.0, -0.5);
  auto ground = world.addRigidBody("ground", groundOptions);

  sx::RigidBodyOptions lowerOptions;
  lowerOptions.position = Eigen::Vector3d(0.0, 0.0, 0.5);
  auto lower = world.addRigidBody("lower", lowerOptions);

  sx::RigidBodyOptions upperOptions;
  upperOptions.position = Eigen::Vector3d(0.0, 0.0, 1.5);
  auto upper = world.addRigidBody("upper", upperOptions);

  std::vector<sx::Contact> contacts;
  sx::Contact groundLower;
  groundLower.bodyA = sx::CollisionBody(ground.getEntity(), &world);
  groundLower.bodyB = sx::CollisionBody(lower.getEntity(), &world);
  groundLower.point = Eigen::Vector3d(0.0, 0.0, 0.0);
  groundLower.normal = Eigen::Vector3d::UnitZ();
  groundLower.depth = 0.02;
  contacts.push_back(groundLower);

  sx::Contact lowerUpper;
  lowerUpper.bodyA = sx::CollisionBody(lower.getEntity(), &world);
  lowerUpper.bodyB = sx::CollisionBody(upper.getEntity(), &world);
  lowerUpper.point = Eigen::Vector3d(0.0, 0.0, 1.0);
  lowerUpper.normal = Eigen::Vector3d::UnitZ();
  lowerUpper.depth = 0.03;
  contacts.push_back(lowerUpper);

  sx::compute::RigidBodyContactAssemblyOptions options;
  options.rowLayout
      = sx::compute::RigidBodyContactRowLayout::NormalThenTangents;
  options.populateJacobian = true;
  options.regularizeFrictionRows = true;
  const auto problem = sx::compute::assembleRigidBodyContactProblem(
      dart::simulation::detail::registryOf(world), contacts, options);

  ASSERT_EQ(problem.constraints.size(), 2u);
  ASSERT_EQ(problem.dynamicBodies.size(), 2u);
  ASSERT_EQ(problem.delassus.rows(), 6);
  ASSERT_EQ(problem.jacobian.rows(), 6);
  ASSERT_EQ(problem.jacobian.cols(), 12);

  EXPECT_EQ(problem.findex[0], -1);
  EXPECT_EQ(problem.findex[1], -1);
  EXPECT_EQ(problem.findex[2], 0);
  EXPECT_EQ(problem.findex[3], 0);
  EXPECT_EQ(problem.findex[4], 1);
  EXPECT_EQ(problem.findex[5], 1);

  const Eigen::RowVector3d zRow(0.0, 0.0, 1.0);
  EXPECT_TRUE((problem.jacobian.block<1, 3>(0, 0).isApprox(zRow)));
  EXPECT_TRUE((problem.jacobian.block<1, 3>(1, 0).isApprox(-zRow)));
  EXPECT_TRUE((problem.jacobian.block<1, 3>(1, 6).isApprox(zRow)));

  sx::compute::RigidBodyContactAssemblyOptions unregularized = options;
  unregularized.regularizeFrictionRows = false;
  const auto reference = sx::compute::assembleRigidBodyContactProblem(
      dart::simulation::detail::registryOf(world), contacts, unregularized);
  EXPECT_GT(problem.delassus(2, 2), reference.delassus(2, 2));
  EXPECT_GT(problem.delassus(5, 5), reference.delassus(5, 5));
}

//==============================================================================
TEST(RigidBodyConstraint, AppliesBaumgarteVelocityBiasWhenRequested)
{
  namespace sx = dart::simulation;

  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());

  sx::RigidBodyOptions groundOptions;
  groundOptions.isStatic = true;
  groundOptions.position = Eigen::Vector3d(0.0, 0.0, -0.5);
  auto ground = world.addRigidBody("ground", groundOptions);

  sx::RigidBodyOptions bodyOptions;
  bodyOptions.position = Eigen::Vector3d(0.0, 0.0, 0.5);
  auto body = world.addRigidBody("body", bodyOptions);

  sx::Contact contact;
  contact.bodyA = sx::CollisionBody(ground.getEntity(), &world);
  contact.bodyB = sx::CollisionBody(body.getEntity(), &world);
  contact.point = Eigen::Vector3d::Zero();
  contact.normal = Eigen::Vector3d::UnitZ();
  contact.depth = 0.03;

  sx::compute::RigidBodyContactAssemblyOptions options;
  options.includeBaumgarteBias = true;
  options.timeStep = 0.01;
  const auto problem = sx::compute::assembleRigidBodyContactProblem(
      dart::simulation::detail::registryOf(world),
      std::span<const sx::Contact>(&contact, 1),
      options);

  ASSERT_EQ(problem.constraints.size(), 1u);
  constexpr double kExpectedBias = 0.2 * (0.03 - 1e-4) / 0.01;
  EXPECT_NEAR(problem.constraints[0].normalVelocityBias, kExpectedBias, 1e-12);
  EXPECT_NEAR(problem.rhs[0], kExpectedBias, 1e-12);
}
