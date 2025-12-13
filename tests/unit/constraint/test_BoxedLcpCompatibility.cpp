/*
 * Copyright (c) 2011-2025, The DART development contributors
 * All rights reserved.
 *
 * This file is provided under the BSD-style License.
 */

#include "dart/common/Diagnostics.hpp"

DART_SUPPRESS_DEPRECATED_BEGIN
#include "dart/constraint/BoxedLcpConstraintSolver.hpp"
#include "dart/constraint/DantzigBoxedLcpSolver.hpp"
#include "dart/constraint/PgsBoxedLcpSolver.hpp"
DART_SUPPRESS_DEPRECATED_END

#include "dart/math/lcp/pivoting/DantzigSolver.hpp"
#include "dart/math/lcp/projection/PgsSolver.hpp"

#include <gtest/gtest.h>

#include <memory>

using namespace dart;

namespace {

//==============================================================================
TEST(BoxedLcpCompatibility, DantzigWrapperReportsTypeAndSolver)
{
  DART_SUPPRESS_DEPRECATED_BEGIN
  constraint::DantzigBoxedLcpSolver boxed;
  EXPECT_EQ(boxed.getType(), "DantzigBoxedLcpSolver");

  auto solver = boxed.getMathSolver();
  ASSERT_TRUE(solver);
  EXPECT_EQ(solver->getName(), "Dantzig");
  EXPECT_EQ(solver->getCategory(), "Pivoting");
  DART_SUPPRESS_DEPRECATED_END
}

//==============================================================================
TEST(BoxedLcpCompatibility, PgsOptionWiresToMathSolver)
{
  DART_SUPPRESS_DEPRECATED_BEGIN
  constraint::PgsBoxedLcpSolver boxed;
  constraint::PgsBoxedLcpSolver::Option option;
  option.mMaxIteration = 123;
  option.mTolerance = 1e-5;
  option.mDoRandomize = true;
  boxed.setOption(option);

  auto solver
      = std::dynamic_pointer_cast<math::PgsSolver>(boxed.getMathSolver());
  ASSERT_TRUE(solver);

  const auto params = solver->getParameters();
  EXPECT_TRUE(params.randomizeConstraintOrder);

  const auto opts = solver->getDefaultOptions();
  EXPECT_EQ(opts.maxIterations, option.mMaxIteration);
  EXPECT_DOUBLE_EQ(opts.absoluteTolerance, option.mTolerance);
  EXPECT_DOUBLE_EQ(opts.relativeTolerance, option.mTolerance * 10.0);
  DART_SUPPRESS_DEPRECATED_END
}

//==============================================================================
TEST(BoxedLcpCompatibility, ConstraintSolverUsesBoxedWrapper)
{
  DART_SUPPRESS_DEPRECATED_BEGIN
  constraint::BoxedLcpConstraintSolver solver;
  auto boxed = std::make_shared<constraint::PgsBoxedLcpSolver>();
  solver.setBoxedLcpSolver(boxed);

  EXPECT_EQ(solver.getBoxedLcpSolver(), boxed);
  EXPECT_EQ(solver.getLcpSolver(), boxed->getMathSolver());
  DART_SUPPRESS_DEPRECATED_END
}

} // namespace

