// Copyright (c) The DART development contributors
// SPDX-License-Identifier: BSD-2-Clause

#include "dart/constraint/BoxedLcpConstraintSolver.hpp"
#include "dart/constraint/ConstrainedGroup.hpp"
#include "dart/constraint/ConstraintBase.hpp"
#include "dart/constraint/DantzigBoxedLcpSolver.hpp"
#include "dart/constraint/NsgsFrictionSolver.hpp"
#include "dart/constraint/PgsBoxedLcpSolver.hpp"
#include "dart/constraint/detail/FrictionCone.hpp"
#include "dart/lcpsolver/dantzig/DantzigCommon.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <future>
#include <limits>
#include <vector>

#include <cmath>

using namespace dart::constraint;

namespace {

struct Problem
{
  explicit Problem(int dimension)
    : n(dimension),
      stride(dart::lcpsolver::dantzig::padding(dimension)),
      A(n * stride, 0.0),
      x(n, 0.0),
      b(n, 1.0),
      lo(n, -std::numeric_limits<double>::infinity()),
      hi(n, std::numeric_limits<double>::infinity()),
      findex(n, -1)
  {
    for (int i = 0; i < n; ++i)
      A[i * stride + i] = 1.0;
  }

  bool solve(BoxedLcpSolver& solver, bool early = true)
  {
    return solver.solve(
        n,
        A.data(),
        x.data(),
        b.data(),
        0,
        lo.data(),
        hi.data(),
        findex.data(),
        early);
  }

  int n;
  int stride;
  std::vector<double> A, x, b, lo, hi;
  std::vector<int> findex;
};

Problem slidingContact()
{
  Problem p(3);
  p.b = {1.0, 2.0, 2.0};
  p.lo = {0.0, -0.5, -0.5};
  p.hi = {std::numeric_limits<double>::infinity(), 0.5, 0.5};
  p.findex = {-1, 0, 0};
  return p;
}

class DivergentConstraint final : public ConstraintBase
{
public:
  DivergentConstraint()
  {
    mDim = 2;
  }
  void update() override {}
  void getInformation(ConstraintInfo* info) override
  {
    ++assemblies;
    for (int i = 0; i < 2; ++i) {
      info->x[i] = start[i];
      info->b[i] = 1.0;
      info->lo[i] = -10.0;
      info->hi[i] = 10.0;
      info->findex[i] = -1;
      info->w[i] = 0.0;
    }
  }
  void applyUnitImpulse(std::size_t index) override
  {
    active = index;
  }
  void getVelocityChange(double* velocity, bool) override
  {
    for (std::size_t i = 0; i < 2; ++i)
      velocity[i] = active == i ? 1.0 : 3.0;
  }
  void excite() override {}
  void unexcite() override {}
  void applyImpulse(double* x) override
  {
    applied = {x[0], x[1]};
  }
  bool isActive() const override
  {
    return true;
  }
  dart::dynamics::SkeletonPtr getRootSkeleton() const override
  {
    return nullptr;
  }

  int assemblies = 0;
  std::array<double, 2> start{};
  std::array<double, 2> applied{};
  std::size_t active = 0;
};

class CountingPgs final : public PgsBoxedLcpSolver
{
public:
  bool solve(
      int n,
      double* A,
      double* x,
      double* b,
      int nub,
      double* lo,
      double* hi,
      int* findex,
      bool early) override
  {
    ++calls;
    return PgsBoxedLcpSolver::solve(n, A, x, b, nub, lo, hi, findex, early);
  }
  int calls = 0;
};

class ExposedSolver final : public BoxedLcpConstraintSolver
{
public:
  using BoxedLcpConstraintSolver::BoxedLcpConstraintSolver;
  using BoxedLcpConstraintSolver::solveConstrainedGroup;
};

} // namespace

TEST(NsgsFrictionSolver, ThreeLawsAndReadOnlyTerms)
{
  for (const auto law :
       {NsgsFrictionSolver::Law::Coulomb,
        NsgsFrictionSolver::Law::Associated,
        NsgsFrictionSolver::Law::Box}) {
    SCOPED_TRACE(static_cast<int>(law));
    NsgsFrictionSolver::Options options;
    options.law = law;
    options.tolerance = 1e-10;
    NsgsFrictionSolver solver(options);
    auto p = slidingContact();
    const auto original = p;
    ASSERT_TRUE(p.solve(solver));
    EXPECT_EQ(original.A, p.A);
    EXPECT_EQ(original.b, p.b);
    EXPECT_EQ(original.lo, p.lo);
    EXPECT_EQ(original.hi, p.hi);
    EXPECT_EQ(original.findex, p.findex);
    if (law == NsgsFrictionSolver::Law::Associated) {
      const double normal = (1.0 + std::sqrt(2.0)) / 1.25;
      EXPECT_NEAR(normal, p.x[0], 1e-9);
      EXPECT_NEAR(normal * 0.5 / std::sqrt(2.0), p.x[1], 1e-9);
    } else {
      EXPECT_NEAR(1.0, p.x[0], 1e-9);
      EXPECT_NEAR(
          law == NsgsFrictionSolver::Law::Box ? 0.5 : 0.5 / std::sqrt(2.0),
          p.x[1],
          1e-9);
    }
    EXPECT_NEAR(p.x[1], p.x[2], 1e-9);
    const auto stats = solver.getStats();
    EXPECT_EQ(1u, stats.numSolves);
    EXPECT_EQ(1u, stats.numConverged);
    EXPECT_EQ(1u, stats.numContacts);
    EXPECT_EQ(
        law == NsgsFrictionSolver::Law::Box ? 1u : 0u, stats.numBoxContacts);
    EXPECT_LE(stats.maxViolation, options.tolerance);
    EXPECT_EQ(0u, stats.numFailed);
  }
}

TEST(NsgsFrictionSolver, CapReturnsBestCompletedIterateInBothTerminationModes)
{
  // SPD system whose first three GS residuals increase after sweep one.
  const double matrix[3][3]
      = {{2.746616368648108, -2.3817935197188653, 1.0222986699719865},
         {-2.3817935197188653, 2.431342759494226, -1.0309647875706964},
         {1.0222986699719865, -1.0309647875706964, 0.6712721439928121}};
  for (bool early : {true, false}) {
    Problem p(3);
    for (int i = 0; i < 3; ++i)
      std::copy(matrix[i], matrix[i] + 3, p.A.begin() + i * p.stride);
    p.b = {-1.5812411969951867, -0.2238416945870532, -0.3709961947182103};
    NsgsFrictionSolver::Options options;
    options.maxSweeps = 3;
    options.tolerance = 1e-12;
    NsgsFrictionSolver solver(options);
    ASSERT_TRUE(p.solve(solver, early));
    EXPECT_NEAR(-0.5757051530911388, p.x[0], 1e-14);
    EXPECT_NEAR(-0.6560376940929923, p.x[1], 1e-14);
    EXPECT_NEAR(-0.6834863452206419, p.x[2], 1e-14);
    const auto stats = solver.getStats();
    EXPECT_EQ(1u, stats.numAcceptedAtCap);
    EXPECT_EQ(0u, stats.numConverged);
    EXPECT_EQ(0u, stats.numFailed);
    EXPECT_EQ(3u, stats.numIterations);
    EXPECT_NEAR(0.8638191468189201, stats.maxViolation, 1e-14);
  }
}

TEST(NsgsFrictionSolver, DivergenceUsesExistingSecondaryAndReassembly)
{
  auto primary = std::make_shared<NsgsFrictionSolver>();
  auto options = primary->getOptions();
  options.maxSweeps = 1;
  primary->setOptions(options);
  auto secondary = std::make_shared<CountingPgs>();
  ExposedSolver solver(primary, secondary);
  auto constraint = std::make_shared<DivergentConstraint>();
  ConstrainedGroup group;
  group.addConstraint(constraint);
  solver.solveConstrainedGroup(group);
  EXPECT_EQ(1u, primary->getStats().numFailed);
  EXPECT_EQ(0u, primary->getStats().numAcceptedAtCap);
  EXPECT_EQ(1, secondary->calls);
  EXPECT_EQ(2, constraint->assemblies);
  EXPECT_TRUE(std::isfinite(constraint->applied[0]));
  EXPECT_TRUE(std::isfinite(constraint->applied[1]));
  EXPECT_NE(0.0, constraint->applied[0]);
}

TEST(NsgsFrictionSolver, DivergenceKeepsWarmStartWithoutSecondary)
{
  auto primary = std::make_shared<NsgsFrictionSolver>();
  auto options = primary->getOptions();
  options.maxSweeps = 1;
  primary->setOptions(options);
  ExposedSolver solver(primary, nullptr);
  auto constraint = std::make_shared<DivergentConstraint>();
  constraint->start = {0.1, 0.1};
  ConstrainedGroup group;
  group.addConstraint(constraint);
  solver.solveConstrainedGroup(group);
  EXPECT_EQ(constraint->start, constraint->applied);
  EXPECT_EQ(1, constraint->assemblies);
  const auto stats = primary->getStats();
  EXPECT_EQ(0u, stats.numFailed);
  EXPECT_EQ(1u, stats.numAcceptedAtCap);
  EXPECT_EQ(0u, stats.numConverged);
  EXPECT_EQ(1u, stats.numIterations);
  EXPECT_NEAR(0.6, stats.maxViolation, 1e-14);
}

TEST(NsgsFrictionSolver, DivergenceHonorsTerminationMode)
{
  for (bool early : {true, false}) {
    SCOPED_TRACE(early);
    NsgsFrictionSolver::Options options;
    options.maxSweeps = 1;
    NsgsFrictionSolver solver(options);
    Problem p(2);
    p.A[1] = p.A[p.stride] = 3.0;
    p.lo = {-10.0, -10.0};
    p.hi = {10.0, 10.0};
    p.x = {0.1, 0.1};
    const auto start = p.x;
    EXPECT_EQ(!early, p.solve(solver, early));
    EXPECT_EQ(start, p.x);
    const auto stats = solver.getStats();
    EXPECT_EQ(early ? 1u : 0u, stats.numFailed);
    EXPECT_EQ(early ? 0u : 1u, stats.numAcceptedAtCap);
    EXPECT_EQ(0u, stats.numConverged);
    EXPECT_EQ(1u, stats.numIterations);
    EXPECT_NEAR(early ? 0.0 : 0.6, stats.maxViolation, 1e-14);
  }
}

TEST(NsgsFrictionSolver, SelfIndexedRowsFailBeforeChangingWarmStart)
{
  Problem original(1);
  original.lo[0] = -2.0;
  original.hi[0] = 2.0;
  original.findex[0] = 0;
  DantzigBoxedLcpSolver dantzig;
  PgsBoxedLcpSolver pgs;
  for (BoxedLcpSolver* solver :
       {static_cast<BoxedLcpSolver*>(&dantzig),
        static_cast<BoxedLcpSolver*>(&pgs)}) {
    auto p = original;
    ASSERT_TRUE(p.solve(*solver));
    EXPECT_DOUBLE_EQ(0.0, p.x[0]);
  }
  for (bool early : {true, false}) {
    SCOPED_TRACE(early);
    for (double warmStart : {0.0, 0.25}) {
      SCOPED_TRACE(warmStart);
      NsgsFrictionSolver solver;
      auto p = original;
      p.x[0] = warmStart;
      EXPECT_FALSE(p.solve(solver, early));
      EXPECT_DOUBLE_EQ(warmStart, p.x[0]);
      const auto stats = solver.getStats();
      EXPECT_EQ(1u, stats.numFailed);
      EXPECT_EQ(0u, stats.numAcceptedAtCap);
      EXPECT_EQ(0u, stats.numIterations);
      EXPECT_DOUBLE_EQ(0.0, stats.maxViolation);
    }
  }
}

TEST(NsgsFrictionSolver, NonFiniteInputFailsRegardlessOfTerminationMode)
{
  for (bool early : {true, false}) {
    NsgsFrictionSolver solver;
    Problem p(1);
    p.b[0] = std::numeric_limits<double>::quiet_NaN();
    EXPECT_FALSE(p.solve(solver, early));
    EXPECT_EQ(1u, solver.getStats().numFailed);
    EXPECT_EQ(0u, solver.getStats().numAcceptedAtCap);
    EXPECT_EQ(0.0, solver.getStats().maxViolation);
  }
}

TEST(
    NsgsFrictionSolver, NonFiniteSweepRestoresBestIterateInBothTerminationModes)
{
  for (bool early : {true, false}) {
    SCOPED_TRACE(early);
    NsgsFrictionSolver solver;
    Problem p(2);
    p.A[0] = 1e-300;
    p.A[1] = p.A[p.stride] = 1e-3;
    p.b = {0.0, 1.0};
    EXPECT_EQ(!early, p.solve(solver, early));
    EXPECT_DOUBLE_EQ(0.0, p.x[0]);
    EXPECT_DOUBLE_EQ(1.0, p.x[1]);
    const auto stats = solver.getStats();
    EXPECT_EQ(early ? 1u : 0u, stats.numFailed);
    EXPECT_EQ(early ? 0u : 1u, stats.numAcceptedAtCap);
    EXPECT_EQ(0u, stats.numConverged);
    EXPECT_EQ(2u, stats.numIterations);
    EXPECT_DOUBLE_EQ(early ? 0.0 : 1e-3, stats.maxViolation);
  }
}

TEST(NsgsFrictionSolver, NonFiniteRowUpdateRestoresCompletedWarmStart)
{
  for (bool early : {true, false}) {
    SCOPED_TRACE(early);
    NsgsFrictionSolver solver;
    Problem p(2);
    p.A[1] = p.A[p.stride] = 1.0;
    p.A[p.stride + 1] = 1e-300;
    p.b = {1e308, 0.0};
    p.x = {0.1, 0.1};
    const auto start = p.x;
    EXPECT_EQ(!early, p.solve(solver, early));
    EXPECT_EQ(start, p.x);
    const auto stats = solver.getStats();
    EXPECT_EQ(early ? 1u : 0u, stats.numFailed);
    EXPECT_EQ(early ? 0u : 1u, stats.numAcceptedAtCap);
    EXPECT_EQ(0u, stats.numConverged);
    EXPECT_EQ(1u, stats.numIterations);
    EXPECT_DOUBLE_EQ(early ? 0.0 : 1e308, stats.maxViolation);
  }
}

TEST(NsgsFrictionSolver, UncertifiedLocalSolveHonorsTerminationMode)
{
  const Eigen::Matrix3d H
      = Eigen::Vector3d(std::numeric_limits<double>::denorm_min(), 2.0, 2.0)
            .asDiagonal();
  const Eigen::Vector3d c(-1.0, 0.0, 0.0);
  const detail::FrictionCone cone{
      Eigen::Vector2d::Constant(0.5), detail::FrictionConeLaw::Ellipse};
  ASSERT_FALSE(detail::solveExactContact(H, c, cone).certified);
  ASSERT_FALSE(detail::solveConeQp(H, c, cone).certified);
  for (auto law :
       {NsgsFrictionSolver::Law::Coulomb,
        NsgsFrictionSolver::Law::Associated}) {
    SCOPED_TRACE(static_cast<int>(law));
    for (bool early : {true, false}) {
      SCOPED_TRACE(early);
      NsgsFrictionSolver::Options options;
      options.law = law;
      NsgsFrictionSolver solver(options);
      auto p = slidingContact();
      for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
          p.A[i * p.stride + j] = H(i, j);
      p.b = {1.0, 0.0, 0.0};
      p.x = {0.25, 0.0, 0.0};
      const auto start = p.x;
      EXPECT_EQ(!early, p.solve(solver, early));
      EXPECT_EQ(start, p.x);
      const auto stats = solver.getStats();
      EXPECT_EQ(early ? 1u : 0u, stats.numFailed);
      EXPECT_EQ(early ? 0u : 1u, stats.numAcceptedAtCap);
      EXPECT_EQ(0u, stats.numConverged);
      EXPECT_EQ(1u, stats.numIterations);
      EXPECT_EQ(1u, stats.numContacts);
      EXPECT_EQ(0u, stats.numBoxContacts);
      EXPECT_DOUBLE_EQ(early ? 0.0 : 1.0, stats.maxViolation);
    }
  }
}

TEST(NsgsFrictionSolver, MaxViolationIncludesOnlyAcceptedSolvesSinceReset)
{
  NsgsFrictionSolver::Options options;
  options.maxSweeps = 0;
  NsgsFrictionSolver solver(options);
  Problem p(1);
  p.b[0] = 0.25;
  ASSERT_TRUE(p.solve(solver));
  const auto before = solver.getStats();
  EXPECT_DOUBLE_EQ(0.25, before.maxViolation);
  p.b[0] = 0.1;
  ASSERT_TRUE(p.solve(solver));
  const auto after = solver.getStats();
  EXPECT_EQ(1u, after.numSolves - before.numSolves);
  EXPECT_EQ(1u, after.numAcceptedAtCap - before.numAcceptedAtCap);
  EXPECT_DOUBLE_EQ(0.25, after.maxViolation);

  options.maxSweeps = 1;
  solver.setOptions(options);
  Problem divergent(2);
  divergent.A[1] = divergent.A[divergent.stride] = 3.0;
  divergent.lo = {-10.0, -10.0};
  divergent.hi = {10.0, 10.0};
  EXPECT_FALSE(divergent.solve(solver));
  p.b[0] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(p.solve(solver));
  EXPECT_EQ(2u, solver.getStats().numFailed);
  EXPECT_DOUBLE_EQ(0.25, solver.getStats().maxViolation);

  solver.resetStats();
  EXPECT_EQ(0u, solver.getStats().numSolves);
  EXPECT_EQ(0u, solver.getStats().numFailed);
  EXPECT_DOUBLE_EQ(0.0, solver.getStats().maxViolation);
  p = Problem(1);
  ASSERT_TRUE(p.solve(solver));
  EXPECT_EQ(1u, solver.getStats().numConverged);
  EXPECT_DOUBLE_EQ(0.0, solver.getStats().maxViolation);
}

TEST(NsgsFrictionSolver, StatsAccumulateAcrossThreadsAndReset)
{
  NsgsFrictionSolver solver;
  constexpr int workers = 4;
  constexpr int solves = 8;
  std::array<std::future<bool>, workers> results;
  for (auto& result : results)
    result = std::async(std::launch::async, [&] {
      solver.reserve(3);
      for (int i = 0; i < solves; ++i) {
        auto p = slidingContact();
        if (!p.solve(solver))
          return false;
      }
      return true;
    });
  for (auto& result : results)
    EXPECT_TRUE(result.get());
  EXPECT_EQ(workers * solves, solver.getStats().numSolves);
  EXPECT_EQ(workers * solves, solver.getStats().numConverged);
  EXPECT_EQ(workers * solves, solver.getStats().numContacts);
  EXPECT_EQ(0u, solver.getStats().numFailed);
  solver.resetStats();
  const auto stats = solver.getStats();
  EXPECT_EQ(0u, stats.numSolves);
  EXPECT_EQ(0u, stats.numConverged);
  EXPECT_EQ(0u, stats.numContacts);
  EXPECT_EQ(0u, stats.numIterations);
  EXPECT_EQ(0.0, stats.maxViolation);
}

TEST(NsgsFrictionSolver, AnisotropyDefaultsToBoxAndCanSelectEllipse)
{
  for (bool box : {true, false}) {
    NsgsFrictionSolver::Options options;
    options.boxForAnisotropic = box;
    options.tolerance = 1e-9;
    NsgsFrictionSolver solver(options);
    auto p = slidingContact();
    p.hi[2] = 1.0;
    p.lo[2] = -1.0;
    ASSERT_TRUE(p.solve(solver));
    EXPECT_NEAR(1.0, p.x[0], 1e-8);
    EXPECT_EQ(box ? 1u : 0u, solver.getStats().numBoxContacts);
    if (box) {
      EXPECT_NEAR(0.5, p.x[1], 1e-8);
      EXPECT_NEAR(1.0, p.x[2], 1e-8);
    } else {
      const detail::FrictionCone cone{
          Eigen::Vector2d(0.5, 1.0), detail::FrictionConeLaw::Ellipse};
      const auto exact = detail::solveExactContact(
          Eigen::Matrix3d::Identity(), Eigen::Vector3d(-1, -2, -2), cone);
      ASSERT_TRUE(exact.certified);
      for (int i = 0; i < 3; ++i)
        EXPECT_NEAR(exact.impulse[i], p.x[i], 1e-8);
    }
    EXPECT_EQ(1u, solver.getStats().numConverged);
    EXPECT_LE(solver.getStats().maxViolation, options.tolerance);
  }
}

TEST(NsgsFrictionSolver, ZeroBlockSeparatingContactConvergesImmediately)
{
  NsgsFrictionSolver solver;
  auto p = slidingContact();
  std::fill(p.A.begin(), p.A.end(), 0.0);
  p.b = {-1.0, 0.0, 0.0};
  EXPECT_TRUE(p.solve(solver));
  for (double impulse : p.x)
    EXPECT_EQ(0.0, impulse);
  const auto stats = solver.getStats();
  EXPECT_EQ(1u, stats.numContacts);
  EXPECT_EQ(1u, stats.numConverged);
  EXPECT_EQ(0u, stats.numIterations);
  EXPECT_EQ(0u, stats.numFailed);
  EXPECT_EQ(0.0, stats.maxViolation);
}

TEST(NsgsFrictionSolver, ZeroNormalCurvatureKeepsBoundedBestIterateAtCap)
{
  NsgsFrictionSolver::Options options;
  options.maxSweeps = 3;
  NsgsFrictionSolver solver(options);
  auto p = slidingContact();
  p.A[0] = 0.0;
  p.b = {1.0, 0.0, 0.0};
  EXPECT_TRUE(p.solve(solver));
  for (double impulse : p.x) {
    EXPECT_TRUE(std::isfinite(impulse));
    EXPECT_LT(std::abs(impulse), 1.0);
  }
  const auto stats = solver.getStats();
  EXPECT_EQ(1u, stats.numContacts);
  EXPECT_EQ(0u, stats.numConverged);
  EXPECT_EQ(1u, stats.numAcceptedAtCap);
  EXPECT_EQ(options.maxSweeps, stats.numIterations);
  EXPECT_EQ(0u, stats.numFailed);
  EXPECT_GT(stats.maxViolation, options.tolerance);
}

TEST(NsgsFrictionSolver, ZeroCapAndAlreadyConvergedWarmStart)
{
  NsgsFrictionSolver::Options options;
  options.maxSweeps = 0;
  NsgsFrictionSolver solver(options);
  Problem p(1);
  EXPECT_TRUE(p.solve(solver));
  EXPECT_EQ(0.0, p.x[0]);
  EXPECT_EQ(1u, solver.getStats().numAcceptedAtCap);
  EXPECT_EQ(0u, solver.getStats().numIterations);
  p.x[0] = 1.0;
  EXPECT_TRUE(p.solve(solver));
  EXPECT_EQ(1u, solver.getStats().numConverged);
  EXPECT_EQ(2u, solver.getStats().numSolves);
  EXPECT_EQ(1.0, solver.getStats().maxViolation);
}

TEST(NsgsFrictionSolver, CapProjectsLegacyBoxWarmStartIntoCircle)
{
  NsgsFrictionSolver::Options options;
  options.maxSweeps = 0;
  options.boxForAnisotropic = false;
  NsgsFrictionSolver solver(options);
  auto p = slidingContact();
  p.x = {1.0, 0.5, 0.5};
  ASSERT_TRUE(p.solve(solver));
  EXPECT_GE(p.x[0], 0.0);
  EXPECT_LE(std::hypot(p.x[1], p.x[2]), 0.5 * p.x[0] + 1e-14);
  EXPECT_EQ(1u, solver.getStats().numAcceptedAtCap);
}

TEST(NsgsFrictionSolver, InvalidOptionsAndIndicesFailClosed)
{
  for (bool early : {true, false}) {
    SCOPED_TRACE(early);
    NsgsFrictionSolver solver;
    EXPECT_EQ("NsgsFrictionSolver", solver.getType());
    EXPECT_EQ(solver.getType(), NsgsFrictionSolver::getStaticType());
    Problem p(1);
    auto options = solver.getOptions();
    options.maxSweeps = -1;
    solver.setOptions(options);
    EXPECT_FALSE(p.solve(solver, early));
    options.maxSweeps = 1;
    for (double tolerance :
         {-1.0,
          std::numeric_limits<double>::quiet_NaN(),
          std::numeric_limits<double>::infinity()}) {
      options.tolerance = tolerance;
      solver.setOptions(options);
      EXPECT_FALSE(p.solve(solver, early));
    }
    options.tolerance = 1e-9;
    options.law = static_cast<NsgsFrictionSolver::Law>(-1);
    solver.setOptions(options);
    EXPECT_FALSE(p.solve(solver, early));
    solver.setOptions(NsgsFrictionSolver::Options{});
    for (int index : {-2, 1}) {
      p.findex[0] = index;
      EXPECT_FALSE(p.solve(solver, early));
    }
    p.findex[0] = -1;
    p.lo[0] = 2.0;
    p.hi[0] = 1.0;
    EXPECT_FALSE(p.solve(solver, early));
    p.lo[0] = -1.0;
    p.A[0] = -1.0;
    EXPECT_FALSE(p.solve(solver, early));
    EXPECT_EQ(9u, solver.getStats().numFailed);
    EXPECT_EQ(0u, solver.getStats().numAcceptedAtCap);
    EXPECT_EQ(0.0, solver.getStats().maxViolation);
  }
}

TEST(NsgsFrictionSolver, OneDimensionalAndTinyAxisFriction)
{
  for (auto law :
       {NsgsFrictionSolver::Law::Coulomb,
        NsgsFrictionSolver::Law::Associated,
        NsgsFrictionSolver::Law::Box}) {
    SCOPED_TRACE(static_cast<int>(law));
    for (bool box : {true, false}) {
      SCOPED_TRACE(box);
      NsgsFrictionSolver::Options options;
      options.law = law;
      options.boxForAnisotropic = box;
      options.tolerance = 1e-10;
      const bool associated = law == NsgsFrictionSolver::Law::Associated;
      const unsigned boxContacts
          = box || law == NsgsFrictionSolver::Law::Box ? 1u : 0u;
      for (double minor : {0.0, 1e-8}) {
        SCOPED_TRACE(minor);
        NsgsFrictionSolver solver(options);
        auto p = slidingContact();
        p.lo[2] = -minor;
        p.hi[2] = minor;
        ASSERT_TRUE(p.solve(solver));
        EXPECT_NEAR(associated ? 1.6 : 1.0, p.x[0], 1e-9);
        EXPECT_NEAR(associated ? 0.8 : 0.5, p.x[1], 1e-9);
        EXPECT_EQ(0.0, p.x[2]);
        const auto stats = solver.getStats();
        EXPECT_EQ(1u, stats.numConverged);
        EXPECT_EQ(boxContacts, stats.numBoxContacts);
        EXPECT_LE(stats.maxViolation, options.tolerance);
        EXPECT_EQ(0u, stats.numFailed);
      }
      Problem p(2);
      p.b = {1.0, 2.0};
      p.lo = {0.0, -0.5};
      p.hi = {std::numeric_limits<double>::infinity(), 0.5};
      p.findex = {-1, 0};
      NsgsFrictionSolver solver(options);
      ASSERT_TRUE(p.solve(solver));
      EXPECT_NEAR(associated ? 1.6 : 1.0, p.x[0], 1e-9);
      EXPECT_NEAR(associated ? 0.8 : 0.5, p.x[1], 1e-9);
      const auto stats = solver.getStats();
      EXPECT_EQ(1u, stats.numContacts);
      EXPECT_EQ(boxContacts, stats.numBoxContacts);
      EXPECT_EQ(1u, stats.numConverged);
      EXPECT_LE(stats.maxViolation, options.tolerance);
      EXPECT_EQ(0u, stats.numFailed);
    }
  }
}

TEST(NsgsFrictionSolver, FiniteNormalCapKeepsOriginalPgsBounds)
{
  NsgsFrictionSolver solver;
  auto p = slidingContact();
  p.hi[0] = 0.25;
  ASSERT_TRUE(p.solve(solver));
  EXPECT_EQ(0.25, p.x[0]);
  EXPECT_EQ(0.125, p.x[1]);
  EXPECT_EQ(0.125, p.x[2]);
  const auto stats = solver.getStats();
  EXPECT_EQ(1u, stats.numContacts);
  EXPECT_EQ(1u, stats.numBoxContacts);
  EXPECT_EQ(1u, stats.numConverged);
}

TEST(NsgsFrictionSolver, MixedContactLimitAndUnboundedRows)
{
  const std::array<double, 5> expected{{1.0, -0.5, 0.0, 0.3, -0.2}};
  for (auto law :
       {NsgsFrictionSolver::Law::Coulomb,
        NsgsFrictionSolver::Law::Associated,
        NsgsFrictionSolver::Law::Box}) {
    SCOPED_TRACE(static_cast<int>(law));
    Problem p(5);
    for (int i = 0; i < 5; ++i) {
      for (int j = 0; j < 5; ++j)
        p.A[i * p.stride + j] = i == j ? 2.0 : 0.1;
      p.b[i] = 0.0;
      for (int j = 0; j < 5; ++j)
        p.b[i] += p.A[i * p.stride + j] * expected[j];
    }
    // Manufacture the independent law solution. An associated sliding row
    // has normal velocity mu*|u_t|; Coulomb and box have zero normal velocity.
    p.b[0] -= law == NsgsFrictionSolver::Law::Associated ? 1.0 : 0.0;
    p.b[1] -= 2.0;
    p.b[3] += 0.1;
    p.lo = {0.0, -0.5, -0.5, -0.3, -std::numeric_limits<double>::infinity()};
    p.hi
        = {std::numeric_limits<double>::infinity(),
           0.5,
           0.5,
           0.3,
           std::numeric_limits<double>::infinity()};
    p.findex = {-1, 0, 0, -1, -1};
    NsgsFrictionSolver::Options options;
    options.law = law;
    options.tolerance = 1e-9;
    NsgsFrictionSolver solver(options);
    ASSERT_TRUE(p.solve(solver));
    for (int i = 0; i < 5; ++i)
      EXPECT_NEAR(expected[i], p.x[i], 1e-8);
    const auto stats = solver.getStats();
    EXPECT_EQ(1u, stats.numConverged);
    EXPECT_EQ(1u, stats.numContacts);
    EXPECT_LE(stats.maxViolation, options.tolerance);
  }
}
