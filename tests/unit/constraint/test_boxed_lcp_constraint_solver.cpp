/*
 * Copyright (c) 2011, The DART development contributors
 * All rights reserved.
 *
 * This file is provided under the BSD-style License.
 */

#include "dart/common/diagnostics.hpp"
#include "dart/constraint/boxed_lcp_constraint_solver.hpp"
#include "dart/constraint/dantzig_boxed_lcp_solver.hpp"
#include "dart/constraint/pgs_boxed_lcp_solver.hpp"
#include "dart/dynamics/skeleton.hpp"
#include "dart/math/lcp/pivoting/dantzig_solver.hpp"
#include "dart/math/lcp/projection/pgs_solver.hpp"

#include <gtest/gtest.h>

#include <limits>
#include <memory>

DART_SUPPRESS_DEPRECATED_BEGIN

using namespace dart;

namespace {

class DummyConstraint : public constraint::ConstraintBase
{
public:
  explicit DummyConstraint(dynamics::SkeletonPtr skeleton)
    : mSkeleton(std::move(skeleton))
  {
    mDim = 1;
  }

  void update() override {}

  void getInformation(constraint::ConstraintInfo* info) override
  {
    info->lo[0] = 0.0;
    info->hi[0] = 1.0;
    info->b[0] = 0.0;
    info->w[0] = 0.0;
    info->findex[0] = -1;
  }

  void applyUnitImpulse(std::size_t) override {}

  void getVelocityChange(double* vel, bool) override
  {
    vel[0] = 1.0;
  }

  void excite() override {}
  void unexcite() override {}

  void applyImpulse(double* lambda) override
  {
    lastAppliedImpulse = lambda[0];
  }

  bool isActive() const override
  {
    return true;
  }

  dynamics::SkeletonPtr getRootSkeleton() const override
  {
    return mSkeleton;
  }

  double lastAppliedImpulse{std::numeric_limits<double>::quiet_NaN()};

private:
  dynamics::SkeletonPtr mSkeleton;
};

} // namespace

//==============================================================================
TEST(BoxedLcpConstraintSolver, DefaultWiresDantzigWithEarlyTermination)
{
  constraint::BoxedLcpConstraintSolver solver;

  auto dantzig
      = std::dynamic_pointer_cast<math::DantzigSolver>(solver.getLcpSolver());
  ASSERT_TRUE(dantzig);
  EXPECT_TRUE(dantzig->getDefaultOptions().earlyTermination);
  EXPECT_NE(solver.getSecondaryLcpSolver(), nullptr);
}

//==============================================================================
TEST(BoxedLcpConstraintSolver, PrimaryPgsOptionsSyncedFromBoxedSolver)
{
  constraint::BoxedLcpConstraintSolver solver;
  auto skeleton = dynamics::Skeleton::create("dummy");
  solver.addSkeleton(skeleton);
  solver.addConstraint(std::make_shared<DummyConstraint>(skeleton));

  auto boxedPgs = std::make_shared<constraint::PgsBoxedLcpSolver>();
  solver.setBoxedLcpSolver(boxedPgs);

  const constraint::PgsBoxedLcpSolver::Option option(
      2, 1e-10, 1e-7, 1e-11, true);
  boxedPgs->setOption(option);

  solver.solve();

  auto pgs = std::dynamic_pointer_cast<math::PgsSolver>(solver.getLcpSolver());
  ASSERT_TRUE(pgs);

  const auto& opts = pgs->getDefaultOptions();
  EXPECT_EQ(opts.maxIterations, option.mMaxIteration);
  EXPECT_DOUBLE_EQ(opts.absoluteTolerance, option.mDeltaXThreshold);
  EXPECT_DOUBLE_EQ(opts.relativeTolerance, option.mRelativeDeltaXTolerance);

  const auto& params = pgs->getParameters();
  EXPECT_DOUBLE_EQ(params.epsilonForDivision, option.mEpsilonForDivision);
  EXPECT_EQ(params.randomizeConstraintOrder, option.mRandomizeConstraintOrder);
}

//==============================================================================
TEST(BoxedLcpConstraintSolver, SecondaryPgsOptionsSyncedFromBoxedSolver)
{
  constraint::BoxedLcpConstraintSolver solver;
  auto skeleton = dynamics::Skeleton::create("dummy");
  solver.addSkeleton(skeleton);
  solver.addConstraint(std::make_shared<DummyConstraint>(skeleton));

  solver.setBoxedLcpSolver(
      std::make_shared<constraint::DantzigBoxedLcpSolver>());

  auto boxedPgs = std::make_shared<constraint::PgsBoxedLcpSolver>();
  solver.setSecondaryBoxedLcpSolver(boxedPgs);

  const constraint::PgsBoxedLcpSolver::Option option(
      5, 1e-12, 1e-9, 1e-8, true);
  boxedPgs->setOption(option);

  solver.solve();

  auto dantzig
      = std::dynamic_pointer_cast<math::DantzigSolver>(solver.getLcpSolver());
  ASSERT_TRUE(dantzig);
  EXPECT_TRUE(dantzig->getDefaultOptions().earlyTermination);

  auto pgs = std::dynamic_pointer_cast<math::PgsSolver>(
      solver.getSecondaryLcpSolver());
  ASSERT_TRUE(pgs);

  const auto& opts = pgs->getDefaultOptions();
  EXPECT_EQ(opts.maxIterations, option.mMaxIteration);
  EXPECT_DOUBLE_EQ(opts.absoluteTolerance, option.mDeltaXThreshold);
  EXPECT_DOUBLE_EQ(opts.relativeTolerance, option.mRelativeDeltaXTolerance);
}

//==============================================================================
TEST(BoxedLcpConstraintSolver, SecondaryCanBeDisabled)
{
  constraint::BoxedLcpConstraintSolver solver;
  solver.setSecondaryBoxedLcpSolver(nullptr);

  auto dantzig
      = std::dynamic_pointer_cast<math::DantzigSolver>(solver.getLcpSolver());
  ASSERT_TRUE(dantzig);
  EXPECT_FALSE(dantzig->getDefaultOptions().earlyTermination);
  EXPECT_EQ(solver.getSecondaryLcpSolver(), nullptr);
}

//==============================================================================
TEST(DantzigBoxedLcpSolver, AcceptsNullFindex)
{
  constraint::DantzigBoxedLcpSolver solver;

  constexpr int kSize = 1;
  double A[kSize] = {1.0};
  double x[kSize] = {0.0};
  const double target = 0.5;
  double b[kSize] = {A[0] * target};
  double lo[kSize] = {0.0};
  double hi[kSize] = {std::numeric_limits<double>::infinity()};

  const bool success = solver.solve(kSize, A, x, b, 0, lo, hi, nullptr, false);

  EXPECT_TRUE(success);
  EXPECT_NEAR(x[0], target, 1e-8);
}

//==============================================================================
TEST(PgsBoxedLcpSolver, AcceptsNullFindex)
{
  constraint::PgsBoxedLcpSolver solver;

  constexpr int kSize = 1;
  double A[kSize] = {1.0};
  double x[kSize] = {0.0};
  const double target = 0.5;
  double b[kSize] = {A[0] * target};
  double lo[kSize] = {0.0};
  double hi[kSize] = {std::numeric_limits<double>::infinity()};

  const bool success = solver.solve(kSize, A, x, b, 0, lo, hi, nullptr, false);

  EXPECT_TRUE(success);
  EXPECT_NEAR(x[0], target, 1e-6);
}

DART_SUPPRESS_DEPRECATED_END
