/*
 * Copyright (c) 2011, The DART development contributors
 * All rights reserved.
 *
 * This file is provided under the BSD-style License.
 */

#include <dart/simulation/world.hpp>

#include <dart/constraint/constraint_solver.hpp>

#include <dart/math/lcp/pivoting/dantzig_solver.hpp>
#include <dart/math/lcp/pivoting/lemke_solver.hpp>
#include <dart/math/lcp/projection/pgs_solver.hpp>

#include <gtest/gtest.h>

using namespace dart;
using namespace dart::simulation;

TEST(WorldConfig, DefaultUsesDantzigWithPgsFallback)
{
  auto world = World::create();
  ASSERT_NE(world, nullptr);

  auto* solver = world->getConstraintSolver();
  ASSERT_NE(solver, nullptr);

  auto primary = solver->getLcpSolver();
  auto secondary = solver->getSecondaryLcpSolver();

  ASSERT_NE(primary, nullptr);
  ASSERT_NE(secondary, nullptr);

  EXPECT_NE(dynamic_cast<math::DantzigSolver*>(primary.get()), nullptr);
  EXPECT_NE(dynamic_cast<math::PgsSolver*>(secondary.get()), nullptr);
}

TEST(WorldConfig, PrimaryOnlyWhenSecondaryIsNullopt)
{
  WorldConfig config;
  config.primaryLcpSolver = LcpSolverType::Pgs;
  config.secondaryLcpSolver = std::nullopt;

  auto world = World::create(config);
  ASSERT_NE(world, nullptr);

  auto* solver = world->getConstraintSolver();
  ASSERT_NE(solver, nullptr);

  auto primary = solver->getLcpSolver();
  auto secondary = solver->getSecondaryLcpSolver();

  ASSERT_NE(primary, nullptr);
  EXPECT_EQ(secondary, nullptr);

  EXPECT_NE(dynamic_cast<math::PgsSolver*>(primary.get()), nullptr);
}

TEST(WorldConfig, CustomPrimaryAndSecondary)
{
  WorldConfig config;
  config.primaryLcpSolver = LcpSolverType::Lemke;
  config.secondaryLcpSolver = LcpSolverType::Dantzig;

  auto world = World::create(config);
  ASSERT_NE(world, nullptr);

  auto* solver = world->getConstraintSolver();
  ASSERT_NE(solver, nullptr);

  auto primary = solver->getLcpSolver();
  auto secondary = solver->getSecondaryLcpSolver();

  ASSERT_NE(primary, nullptr);
  ASSERT_NE(secondary, nullptr);

  EXPECT_NE(dynamic_cast<math::LemkeSolver*>(primary.get()), nullptr);
  EXPECT_NE(dynamic_cast<math::DantzigSolver*>(secondary.get()), nullptr);
}

TEST(WorldConfig, AllSolverTypesCanBeUsedAsPrimary)
{
  for (auto type :
       {LcpSolverType::Dantzig, LcpSolverType::Pgs, LcpSolverType::Lemke}) {
    WorldConfig config;
    config.primaryLcpSolver = type;
    config.secondaryLcpSolver = std::nullopt;

    auto world = World::create(config);
    ASSERT_NE(world, nullptr);

    auto* solver = world->getConstraintSolver();
    ASSERT_NE(solver, nullptr);

    auto primary = solver->getLcpSolver();
    EXPECT_NE(primary, nullptr);
  }
}
