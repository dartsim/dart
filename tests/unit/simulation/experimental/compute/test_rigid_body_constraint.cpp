/*
 * Copyright (c) 2011, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/main/LICENSE
 *
 * This file is provided under the "BSD-style" License
 */

#include <dart/simulation/experimental/body/contact.hpp>
#include <dart/simulation/experimental/body/rigid_body.hpp>
#include <dart/simulation/experimental/compute/rigid_body_constraint.hpp>
#include <dart/simulation/experimental/detail/entity_conversion.hpp>
#include <dart/simulation/experimental/world.hpp>

#include <gtest/gtest.h>

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
  namespace sx = dart::simulation::experimental;

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
  groundLower.bodyA = sx::CollisionBody(
      dart::simulation::experimental::detail::fromRegistryEntity(
          ground.getEntity()),
      &world);
  groundLower.bodyB = sx::CollisionBody(
      dart::simulation::experimental::detail::fromRegistryEntity(
          lower.getEntity()),
      &world);
  groundLower.point = Eigen::Vector3d(0.0, 0.0, 0.0);
  groundLower.normal = Eigen::Vector3d::UnitZ();
  groundLower.depth = 0.02;
  contacts.push_back(groundLower);

  sx::Contact lowerUpper;
  lowerUpper.bodyA = sx::CollisionBody(
      dart::simulation::experimental::detail::fromRegistryEntity(
          lower.getEntity()),
      &world);
  lowerUpper.bodyB = sx::CollisionBody(
      dart::simulation::experimental::detail::fromRegistryEntity(
          upper.getEntity()),
      &world);
  lowerUpper.point = Eigen::Vector3d(0.0, 0.0, 1.0);
  lowerUpper.normal = Eigen::Vector3d::UnitZ();
  lowerUpper.depth = 0.03;
  contacts.push_back(lowerUpper);

  const auto problem = sx::compute::assembleRigidBodyContactProblem(
      world.getRegistry(), contacts);
  const auto repeated = sx::compute::assembleRigidBodyContactProblem(
      world.getRegistry(), contacts);

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

  EXPECT_EQ(problem.constraints[0].bodyA, ground.getEntity());
  EXPECT_EQ(problem.constraints[0].bodyB, lower.getEntity());
  EXPECT_TRUE(problem.constraints[0].staticA);
  EXPECT_FALSE(problem.constraints[0].staticB);
  EXPECT_EQ(problem.constraints[1].bodyA, lower.getEntity());
  EXPECT_EQ(problem.constraints[1].bodyB, upper.getEntity());
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
