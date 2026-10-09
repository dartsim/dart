// Copyright (c) The DART development contributors
// SPDX-License-Identifier: BSD-2-Clause

#include "dart/constraint/BoxedLcpConstraintSolver.hpp"
#include "dart/constraint/ConstrainedGroup.hpp"
#include "dart/constraint/ConstraintBase.hpp"
#include "dart/constraint/DantzigBoxedLcpSolver.hpp"
#include "dart/constraint/FbfFrictionSolver.hpp"
#include "dart/constraint/NsgsFrictionSolver.hpp"
#include "dart/constraint/PgsBoxedLcpSolver.hpp"
#include "dart/constraint/detail/FrictionCone.hpp"
#include "dart/constraint/detail/FrictionRows.hpp"
#include "dart/lcpsolver/dantzig/DantzigCommon.hpp"

#include <Eigen/Geometry>
#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <future>
#include <limits>
#include <random>
#include <utility>
#include <vector>

#include <cmath>
#include <cstring>

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
    const auto* fbf = dynamic_cast<const FbfFrictionSolver*>(&solver);
    const auto shrinks = fbf ? fbf->getStats().numStepShrinks : 0;
    const bool result = solver.solve(
        n,
        A.data(),
        x.data(),
        b.data(),
        0,
        lo.data(),
        hi.data(),
        findex.data(),
        early);
    if (fbf && fbf->getOptions().stepScale == 0.5
        && fbf->getOptions().tolerance >= 1e-12) {
      EXPECT_EQ(shrinks, fbf->getStats().numStepShrinks);
    }
    return result;
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

Problem contactProblem(
    const Eigen::MatrixXd& matrix,
    const Eigen::VectorXd& rhs,
    const std::vector<std::pair<double, double>>& mus)
{
  Problem p(static_cast<int>(matrix.rows()));
  for (int i = 0; i < p.n; ++i) {
    p.b[i] = rhs[i];
    p.lo[i] = 0.0;
    for (int j = 0; j < p.n; ++j)
      p.A[i * p.stride + j] = matrix(i, j);
  }
  for (std::size_t c = 0; c < mus.size(); ++c) {
    for (int axis = 1; axis <= 2; ++axis) {
      const auto row = 3 * c + axis;
      const double mu = axis == 1 ? mus[c].first : mus[c].second;
      p.lo[row] = -mu;
      p.hi[row] = mu;
      p.findex[row] = 3 * c;
    }
  }
  return p;
}

Problem boxOnGround(double vx, double vy)
{
  Eigen::Matrix<double, 12, 6> J = Eigen::Matrix<double, 12, 6>::Zero();
  int row = 0;
  for (double sx : {-1.0, 1.0}) {
    for (double sy : {-1.0, 1.0}) {
      const Eigen::Vector3d r(0.1 * sx, 0.1 * sy, -0.1);
      for (const auto& d :
           {Eigen::Vector3d::UnitZ().eval(),
            Eigen::Vector3d::UnitX().eval(),
            Eigen::Vector3d::UnitY().eval()}) {
        J.block<1, 3>(row, 0) = d.transpose();
        J.block<1, 3>(row, 3) = r.cross(d).transpose();
        ++row;
      }
    }
  }
  Eigen::Matrix<double, 6, 1> invMass;
  invMass << 1.0, 1.0, 1.0, 150.0, 150.0, 150.0;
  Eigen::MatrixXd matrix = J * invMass.asDiagonal() * J.transpose();
  matrix.diagonal() *= 1.0 + 1e-5;
  Eigen::Matrix<double, 6, 1> velocity;
  velocity << vx, vy, -9.81e-3, 0.0, 0.0, 0.0;
  return contactProblem(
      matrix, -J * velocity, {{0.5, 0.5}, {0.5, 0.5}, {0.5, 0.5}, {0.5, 0.5}});
}

Problem mixedProblem()
{
  const std::array<double, 5> expected{{1.0, -0.5, 0.0, 0.3, -0.2}};
  Problem p(5);
  for (int i = 0; i < 5; ++i) {
    for (int j = 0; j < 5; ++j)
      p.A[i * p.stride + j] = i == j ? 2.0 : 0.1;
    p.b[i] = 0.0;
    for (int j = 0; j < 5; ++j)
      p.b[i] += p.A[i * p.stride + j] * expected[j];
  }
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
  return p;
}

void expectConverged(const FbfFrictionSolver& solver)
{
  const auto stats = solver.getStats();
  EXPECT_EQ(1u, stats.numSolves);
  EXPECT_EQ(1u, stats.numConverged);
  EXPECT_EQ(0u, stats.numAcceptedAtCap);
  EXPECT_EQ(0u, stats.numFailed);
  EXPECT_LE(stats.maxViolation, solver.getOptions().tolerance);
  if (solver.getOptions().tolerance >= 1e-12) {
    EXPECT_EQ(0u, stats.numStepShrinks);
  }
}

void expectStatsEqual(
    const FrictionSolveStats& expected,
    const FrictionSolveStats& actual,
    std::uint64_t multiplier = 1)
{
  EXPECT_EQ(multiplier * expected.numSolves, actual.numSolves);
  EXPECT_EQ(multiplier * expected.numConverged, actual.numConverged);
  EXPECT_EQ(multiplier * expected.numAcceptedAtCap, actual.numAcceptedAtCap);
  EXPECT_EQ(multiplier * expected.numFailed, actual.numFailed);
  EXPECT_EQ(multiplier * expected.numContacts, actual.numContacts);
  EXPECT_EQ(multiplier * expected.numBoxContacts, actual.numBoxContacts);
  EXPECT_EQ(multiplier * expected.numLocalFallbacks, actual.numLocalFallbacks);
  EXPECT_EQ(multiplier * expected.numIterations, actual.numIterations);
  EXPECT_EQ(
      multiplier * expected.numInnerIterations, actual.numInnerIterations);
  EXPECT_EQ(multiplier * expected.numStepShrinks, actual.numStepShrinks);
  EXPECT_EQ(multiplier * expected.numInnerCaps, actual.numInnerCaps);
  EXPECT_EQ(expected.maxViolation, actual.maxViolation);
}

class BankRng
{
public:
  double uniform()
  {
    return std::ldexp(static_cast<double>(mEngine() >> 11), -53);
  }

  double normal()
  {
    const double radius = std::sqrt(-2.0 * std::log(1.0 - uniform()));
    return radius * std::cos(2.0 * std::acos(-1.0) * uniform());
  }

private:
  std::mt19937_64 mEngine{3};
};

} // namespace

TEST(FbfFrictionSolver, WarmStartResidualChangeWithinToleranceIsAcceptedAtCap)
{
  // A complete SPD sweep raises the warm residual by 7.81e-6 m/s.
  for (double tolerance : {1e-5, 1e-6}) {
    SCOPED_TRACE(tolerance);
    Problem p(2);
    p.A[1] = p.A[p.stride] = 0.9;
    p.b = {1.9, 1.9};
    p.x = {0.99989, 1.00011};
    const auto startingImpulse = p.x;
    FbfFrictionSolver::Options options;
    options.maxOuterIterations = 1;
    options.tolerance = tolerance;
    FbfFrictionSolver solver(options);
    const bool accepted = tolerance == 1e-5;
    EXPECT_EQ(accepted, p.solve(solver));
    EXPECT_EQ(startingImpulse, p.x);
    const auto stats = solver.getStats();
    EXPECT_EQ(accepted ? 1u : 0u, stats.numAcceptedAtCap);
    EXPECT_EQ(accepted ? 0u : 1u, stats.numFailed);
    EXPECT_EQ(0u, stats.numConverged);
    EXPECT_EQ(1u, stats.numIterations);
    EXPECT_EQ(1u, stats.numInnerIterations);
    EXPECT_NEAR(accepted ? 1.1e-5 : 0.0, stats.maxViolation, 1e-14);
  }
}

TEST(FbfFrictionSolver, DivergenceUsesExistingSecondaryAndReassembly)
{
  auto primary = std::make_shared<FbfFrictionSolver>();
  auto options = primary->getOptions();
  options.maxOuterIterations = 1;
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
  EXPECT_EQ(1u, primary->getStats().numIterations);
  EXPECT_EQ(1u, primary->getStats().numInnerIterations);
  EXPECT_TRUE(std::isfinite(constraint->applied[0]));
  EXPECT_TRUE(std::isfinite(constraint->applied[1]));
  EXPECT_NE(0.0, constraint->applied[0]);
}

TEST(FbfFrictionSolver, DivergenceKeepsWarmStartWithoutSecondary)
{
  auto primary = std::make_shared<FbfFrictionSolver>();
  auto options = primary->getOptions();
  options.maxOuterIterations = 1;
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
  EXPECT_EQ(1u, stats.numInnerIterations);
  EXPECT_NEAR(0.6, stats.maxViolation, 1e-14);
}

TEST(FbfFrictionSolver, DivergenceHonorsTerminationMode)
{
  for (bool early : {true, false}) {
    SCOPED_TRACE(early);
    FbfFrictionSolver::Options options;
    options.maxOuterIterations = 1;
    FbfFrictionSolver solver(options);
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
    EXPECT_EQ(1u, stats.numInnerIterations);
    EXPECT_NEAR(early ? 0.0 : 0.6, stats.maxViolation, 1e-14);
  }
}

TEST(FbfFrictionSolver, SelfIndexedRowsFailBeforeChangingWarmStart)
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
      FbfFrictionSolver solver;
      auto p = original;
      p.x[0] = warmStart;
      EXPECT_FALSE(p.solve(solver, early));
      EXPECT_DOUBLE_EQ(warmStart, p.x[0]);
      const auto stats = solver.getStats();
      EXPECT_EQ(1u, stats.numFailed);
      EXPECT_EQ(0u, stats.numAcceptedAtCap);
      EXPECT_EQ(0u, stats.numIterations);
      EXPECT_EQ(0u, stats.numInnerIterations);
      EXPECT_DOUBLE_EQ(0.0, stats.maxViolation);
    }
  }
}

TEST(FbfFrictionSolver, NonFiniteInputFailsRegardlessOfTerminationMode)
{
  for (bool early : {true, false}) {
    FbfFrictionSolver solver;
    Problem p(1);
    p.b[0] = std::numeric_limits<double>::quiet_NaN();
    EXPECT_FALSE(p.solve(solver, early));
    EXPECT_EQ(1u, solver.getStats().numFailed);
    EXPECT_EQ(0u, solver.getStats().numAcceptedAtCap);
    EXPECT_EQ(0.0, solver.getStats().maxViolation);
    EXPECT_EQ(0u, solver.getStats().numInnerIterations);
  }
}

TEST(FbfFrictionSolver, NonFiniteSweepRestoresBestIterateInBothTerminationModes)
{
  for (bool early : {true, false}) {
    SCOPED_TRACE(early);
    FbfFrictionSolver solver;
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
    EXPECT_EQ(2u, stats.numInnerIterations);
    EXPECT_DOUBLE_EQ(early ? 0.0 : 1e-3, stats.maxViolation);
  }
}

TEST(FbfFrictionSolver, NonFiniteRowUpdateRestoresCompletedWarmStart)
{
  for (bool early : {true, false}) {
    SCOPED_TRACE(early);
    FbfFrictionSolver solver;
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
    EXPECT_EQ(1u, stats.numInnerIterations);
    EXPECT_DOUBLE_EQ(early ? 0.0 : 1e308, stats.maxViolation);
  }
}

TEST(FbfFrictionSolver, MaxViolationIncludesOnlyAcceptedSolvesSinceReset)
{
  FbfFrictionSolver::Options options;
  options.maxOuterIterations = 0;
  FbfFrictionSolver solver(options);
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

  options.maxOuterIterations = 1;
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

TEST(FbfFrictionSolver, ZeroBlockSeparatingContactConvergesImmediately)
{
  FbfFrictionSolver solver;
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
  EXPECT_EQ(0u, stats.numInnerIterations);
  EXPECT_EQ(0u, stats.numFailed);
  EXPECT_EQ(0.0, stats.maxViolation);
}

TEST(FbfFrictionSolver, ZeroCapAndAlreadyConvergedWarmStart)
{
  FbfFrictionSolver::Options options;
  options.maxOuterIterations = 0;
  FbfFrictionSolver solver(options);
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

TEST(FbfFrictionSolver, CapProjectsLegacyBoxWarmStartIntoCircle)
{
  FbfFrictionSolver::Options options;
  options.maxOuterIterations = 0;
  options.boxForAnisotropic = false;
  FbfFrictionSolver solver(options);
  auto p = slidingContact();
  p.x = {1.0, 0.5, 0.5};
  ASSERT_TRUE(p.solve(solver));
  EXPECT_GE(p.x[0], 0.0);
  EXPECT_LE(std::hypot(p.x[1], p.x[2]), 0.5 * p.x[0] + 1e-14);
  EXPECT_EQ(1u, solver.getStats().numAcceptedAtCap);
}

TEST(FbfFrictionSolver, FiniteNormalCapKeepsOriginalPgsBounds)
{
  FbfFrictionSolver solver;
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
  EXPECT_EQ(1u, stats.numIterations);
  EXPECT_EQ(1u, stats.numInnerIterations);
}

TEST(FbfFrictionSolver, TypeDefaultsAndOptions)
{
  FbfFrictionSolver solver;
  EXPECT_EQ("FbfFrictionSolver", solver.getType());
  EXPECT_EQ(FbfFrictionSolver::getStaticType(), solver.getType());
  auto options = solver.getOptions();
  EXPECT_TRUE(options.boxForAnisotropic);
  EXPECT_EQ(100, options.maxOuterIterations);
  EXPECT_EQ(1e-5, options.tolerance);
  EXPECT_EQ(0.5, options.stepScale);
  EXPECT_EQ(20, options.maxInnerSweeps);
  EXPECT_EQ(0.1, options.innerToleranceFactor);
  options.boxForAnisotropic = false;
  options.maxOuterIterations = 71;
  options.tolerance = 2e-7;
  options.stepScale = 0.7;
  options.maxInnerSweeps = 9;
  options.innerToleranceFactor = 0.05;
  solver.setOptions(options);
  FbfFrictionSolver constructed(options);
  for (const auto* backend : {&solver, &constructed}) {
    const auto& actual = backend->getOptions();
    EXPECT_EQ(options.boxForAnisotropic, actual.boxForAnisotropic);
    EXPECT_EQ(options.maxOuterIterations, actual.maxOuterIterations);
    EXPECT_EQ(options.tolerance, actual.tolerance);
    EXPECT_EQ(options.stepScale, actual.stepScale);
    EXPECT_EQ(options.maxInnerSweeps, actual.maxInnerSweeps);
    EXPECT_EQ(options.innerToleranceFactor, actual.innerToleranceFactor);
    EXPECT_EQ("FbfFrictionSolver", backend->getType());
  }
}

TEST(FbfFrictionSolver, LawsAndReadOnlyTerms)
{
  for (int problem = 0; problem < 4; ++problem) {
    SCOPED_TRACE(problem);
    FbfFrictionSolver::Options options;
    options.tolerance = 1e-10;
    options.maxOuterIterations = 500;
    options.boxForAnisotropic = problem != 2;
    FbfFrictionSolver solver(options);
    auto p = problem == 3 ? mixedProblem() : slidingContact();
    Eigen::Vector3d expected(1.0, 0.5 / std::sqrt(2.0), 0.5 / std::sqrt(2.0));
    if (problem == 1 || problem == 2) {
      p.lo[2] = -1.0;
      p.hi[2] = 1.0;
      if (problem == 1) {
        expected << 1.0, 0.5, 1.0;
      } else {
        const detail::FrictionCone cone{
            Eigen::Vector2d(0.5, 1.0), detail::FrictionConeLaw::Ellipse};
        const auto exact = detail::solveExactContact(
            Eigen::Matrix3d::Identity(),
            Eigen::Vector3d(-1.0, -2.0, -2.0),
            cone);
        ASSERT_TRUE(exact.certified);
        expected = exact.impulse;
      }
    }
    if (problem == 3)
      expected << 1.0, -0.5, 0.0;
    const auto original = p;
    ASSERT_TRUE(p.solve(solver));
    EXPECT_EQ(original.A, p.A);
    EXPECT_EQ(original.b, p.b);
    EXPECT_EQ(original.lo, p.lo);
    EXPECT_EQ(original.hi, p.hi);
    EXPECT_EQ(original.findex, p.findex);
    for (int i = 0; i < 3; ++i)
      EXPECT_NEAR(expected[i], p.x[i], 1e-8);
    if (problem == 3) {
      EXPECT_NEAR(0.3, p.x[3], 1e-8);
      EXPECT_NEAR(-0.2, p.x[4], 1e-8);
    }
    expectConverged(solver);
    EXPECT_EQ(1u, solver.getStats().numContacts);
    EXPECT_EQ(problem == 1 ? 1u : 0u, solver.getStats().numBoxContacts);
  }
}

TEST(FbfFrictionSolver, FirstIterateUsesCertifiedStep)
{
  struct Case
  {
    Eigen::Matrix3d matrix;
    Eigen::Vector3d rhs;
    Eigen::Vector2d mu;
    Eigen::Vector3d first;
    double bound;
    double violation;
    detail::FrictionConeLaw law;
  };
  Eigen::Matrix3d orthogonal;
  orthogonal << 50.5, -49.5, 0.0, -49.5, 50.5, 0.0, 0.0, 0.0, 1.0;
  const std::array<Case, 3> cases{
      {{Eigen::Matrix3d::Identity(),
        Eigen::Vector3d(1.0, 2.0, 2.0),
        Eigen::Vector2d::Constant(0.5),
        Eigen::Vector3d(0.5, 0.1414213562373095, 0.1414213562373095),
        1.0,
        0.449444101085,
        detail::FrictionConeLaw::Ellipse},
       {Eigen::Matrix3d::Identity(),
        Eigen::Vector3d(1.0, 2.0, 2.0),
        Eigen::Vector2d(0.5, 1.0),
        Eigen::Vector3d(
            0.21411677590276, 0.0686704431944328, 0.137340886388866),
        1.0,
        0.527038102536,
        detail::FrictionConeLaw::Box},
       {orthogonal,
        Eigen::Vector3d(1.0, 0.5, -0.5),
        Eigen::Vector2d::Constant(0.5),
        Eigen::Vector3d(
            0.00814388870246244, 0.00341153370184196, -0.0022230988733568),
        std::sqrt(100.0 * 50.5),
        0.677627718245,
        detail::FrictionConeLaw::Ellipse}}};
  for (std::size_t index = 0; index < cases.size(); ++index) {
    SCOPED_TRACE(index);
    const auto& c = cases[index];
    FbfFrictionSolver::Options options;
    options.maxOuterIterations = 1;
    options.tolerance = 1e-12;
    FbfFrictionSolver solver(options);
    auto p = contactProblem(c.matrix, c.rhs, {{c.mu[0], c.mu[1]}});
    ASSERT_TRUE(p.solve(solver));
    for (int i = 0; i < 3; ++i)
      EXPECT_NEAR(c.first[i], p.x[i], 1e-12);
    const auto stats = solver.getStats();
    EXPECT_EQ(1u, stats.numIterations);
    EXPECT_EQ(1u, stats.numInnerIterations);
    EXPECT_EQ(0u, stats.numStepShrinks);
    EXPECT_EQ(1u, stats.numAcceptedAtCap);
    EXPECT_NEAR(c.violation, stats.maxViolation, 1e-11);

    double largestRow = 0.0, largestColumn = 0.0;
    for (int row = 1; row < 3; ++row) {
      double sum = 0.0;
      for (int col = 0; col < 3; ++col)
        sum += std::abs(c.matrix(row, col));
      largestRow = std::max(largestRow, sum);
    }
    for (int col = 0; col < 3; ++col) {
      double sum = 0.0;
      for (int row = 1; row < 3; ++row)
        sum += std::abs(c.matrix(row, col));
      largestColumn = std::max(largestColumn, sum);
    }
    const double bound = std::sqrt(largestRow) * std::sqrt(largestColumn);
    EXPECT_NEAR(c.bound, bound, 1e-12);
    const double muEff = c.law == detail::FrictionConeLaw::Box
                             ? std::hypot(c.mu[0], c.mu[1])
                             : c.mu.maxCoeff();
    const double gamma = options.stepScale / (muEff * bound);
    const detail::FrictionCone cone{c.mu, c.law};
    const double shift
        = detail::deSaxce(Eigen::Vector3d(0.0, -c.rhs[1], -c.rhs[2]), cone)[0];
    Eigen::Matrix3d proximal = c.matrix;
    proximal.diagonal().array() += 1.0 / gamma;
    Eigen::Vector3d linear = -c.rhs;
    linear[0] += shift;
    const auto inner = detail::solveConeQp(proximal, linear, cone);
    ASSERT_TRUE(inner.certified);
    Eigen::Vector3d velocity;
    for (int row = 0; row < 3; ++row) {
      velocity[row] = -c.rhs[row];
      for (int col = 0; col < 3; ++col)
        velocity[row] += c.matrix(row, col) * inner.impulse[col];
    }
    const double trialShift = detail::deSaxce(
        Eigen::Vector3d(0.0, velocity[1], velocity[2]), cone)[0];
    Eigen::Vector3d corrected = inner.impulse;
    corrected[0] -= gamma * (trialShift - shift);
    const auto independentlyComputed = detail::projectCone(corrected, cone);
    EXPECT_EQ(
        0,
        std::memcmp(
            p.x.data(), independentlyComputed.data(), 3 * sizeof(double)));
  }
}

TEST(FbfFrictionSolver, StepSearchCorrectsWithTheSolvedStep)
{
  struct Case
  {
    double scale;
    Eigen::Vector3d expected;
    std::uint64_t shrinks;
  };
  const std::array<Case, 2> cases{
      {{2.5,
        Eigen::Vector3d(7.0 / 6.0, 0.219988776369148, 0.219988776369148),
        1},
       {3.0,
        Eigen::Vector3d(
            1.035715736040609, 0.211055221998827, 0.211055221998827),
        2}}};
  for (const auto& c : cases) {
    SCOPED_TRACE(c.scale);
    FbfFrictionSolver::Options options;
    options.stepScale = c.scale;
    options.maxOuterIterations = 1;
    options.tolerance = 1e-12;
    FbfFrictionSolver solver(options);
    auto p = slidingContact();
    ASSERT_TRUE(p.solve(solver));
    for (int i = 0; i < 3; ++i)
      EXPECT_NEAR(c.expected[i], p.x[i], 1e-12);
    const auto first = solver.getStats();
    EXPECT_EQ(c.shrinks, first.numStepShrinks);
    EXPECT_EQ(c.shrinks + 1, first.numInnerIterations);
    EXPECT_EQ(1u, first.numIterations);
    EXPECT_EQ(1u, first.numAcceptedAtCap);
    if (c.scale == 2.5) {
      EXPECT_NEAR(0.285492859525, first.maxViolation, 1e-11);
    }
    auto repeat = slidingContact();
    ASSERT_TRUE(repeat.solve(solver));
    EXPECT_EQ(0, std::memcmp(p.x.data(), repeat.x.data(), 3 * sizeof(double)));
    expectStatsEqual(first, solver.getStats(), 2);
  }
}

TEST(FbfFrictionSolver, ExhaustedStepSearchHonorsTerminationMode)
{
  for (bool early : {true, false}) {
    SCOPED_TRACE(early);
    FbfFrictionSolver::Options options;
    options.stepScale = 1e6;
    options.tolerance = 1e-12;
    FbfFrictionSolver solver(options);
    auto p = slidingContact();
    const auto start = p.x;
    EXPECT_EQ(!early, p.solve(solver, early));
    EXPECT_EQ(0, std::memcmp(start.data(), p.x.data(), 3 * sizeof(double)));
    const auto stats = solver.getStats();
    EXPECT_EQ(1u, stats.numIterations);
    EXPECT_EQ(20u, stats.numStepShrinks);
    EXPECT_EQ(21u, stats.numInnerIterations);
    EXPECT_EQ(early ? 1u : 0u, stats.numFailed);
    EXPECT_EQ(early ? 0u : 1u, stats.numAcceptedAtCap);
    EXPECT_EQ(0u, stats.numConverged);
    EXPECT_NEAR(early ? 0.0 : 2.0 / std::sqrt(5.0), stats.maxViolation, 1e-12);
  }
}

TEST(FbfFrictionSolver, RatioTestIsScaleInvariant)
{
  FbfFrictionSolver::Options options;
  options.stepScale = 2.5;
  options.maxOuterIterations = 1;
  options.tolerance = 1e-12;
  FbfFrictionSolver solver(options);
  auto p = slidingContact();
  for (int row = 0; row < 3; ++row)
    p.A[row * p.stride + row] = 1e-170;
  ASSERT_TRUE(p.solve(solver));
  const std::array<double, 3> expected{
      {7.0 / 6.0, 0.219988776369148, 0.219988776369148}};
  for (int i = 0; i < 3; ++i)
    EXPECT_NEAR(expected[i], p.x[i] * 1e-170, 1e-12);
  const auto stats = solver.getStats();
  EXPECT_EQ(1u, stats.numStepShrinks);
  EXPECT_EQ(2u, stats.numInnerIterations);
  EXPECT_EQ(1u, stats.numAcceptedAtCap);
}

TEST(FbfFrictionSolver, InnerSolveStopsOnKktResidual)
{
  for (const auto& velocity : {std::pair{0.0, 0.0}, std::pair{0.3, 0.3}}) {
    SCOPED_TRACE(velocity.first);
    FbfFrictionSolver::Options options;
    options.tolerance = 1e-6;
    options.maxOuterIterations = 100000;
    options.maxInnerSweeps = 1000;
    std::uint64_t previous = 0;
    for (double factor : {0.5, 0.1, 1e-3, 0.0}) {
      SCOPED_TRACE(factor);
      options.innerToleranceFactor = factor;
      FbfFrictionSolver solver(options);
      auto p = boxOnGround(velocity.first, velocity.second);
      ASSERT_TRUE(p.solve(solver));
      expectConverged(solver);
      const auto stats = solver.getStats();
      EXPECT_GT(stats.numInnerIterations, previous);
      EXPECT_GE(stats.numInnerIterations, stats.numIterations);
      EXPECT_EQ(0u, stats.numInnerCaps);
      previous = stats.numInnerIterations;
    }
    options.maxInnerSweeps = 1;
    options.innerToleranceFactor = 0.0;
    FbfFrictionSolver capped(options);
    auto p = boxOnGround(velocity.first, velocity.second);
    ASSERT_TRUE(p.solve(capped));
    expectConverged(capped);
    EXPECT_GT(capped.getStats().numIterations, 0u);
    EXPECT_EQ(capped.getStats().numIterations, capped.getStats().numInnerCaps);

    options = FbfFrictionSolver::Options{};
    options.tolerance = 1e-9;
    options.maxOuterIterations = 1000;
    FbfFrictionSolver tight(options);
    p = boxOnGround(velocity.first, velocity.second);
    ASSERT_TRUE(p.solve(tight));
    expectConverged(tight);
  }
}

TEST(FbfFrictionSolver, EveryInnerSolveSweepsAtLeastOnce)
{
  for (double scale : {1.0, 1e-12}) {
    SCOPED_TRACE(scale);
    FbfFrictionSolver::Options options;
    if (scale == 1.0)
      options.innerToleranceFactor = 2.0;
    else
      options.tolerance = 1e-17;
    FbfFrictionSolver solver(options);
    auto p = slidingContact();
    for (auto& value : p.b)
      value *= scale;
    ASSERT_TRUE(p.solve(solver));
    expectConverged(solver);
    const auto stats = solver.getStats();
    EXPECT_GT(stats.numIterations, 0u);
    EXPECT_GE(stats.numInnerIterations, stats.numIterations);
    EXPECT_NEAR(1.0, p.x[0] / scale, 1e-4);
    EXPECT_NEAR(0.5 / std::sqrt(2.0), p.x[1] / scale, 1e-4);
    EXPECT_NEAR(0.5 / std::sqrt(2.0), p.x[2] / scale, 1e-4);
  }
}

TEST(FbfFrictionSolver, PlainPathMatchesNsgsSweeps)
{
  const double matrix[3][3]
      = {{2.746616368648108, -2.3817935197188653, 1.0222986699719865},
         {-2.3817935197188653, 2.431342759494226, -1.0309647875706964},
         {1.0222986699719865, -1.0309647875706964, 0.6712721439928121}};
  const std::array<double, 3> expected{
      {-0.5757051530911388, -0.6560376940929923, -0.6834863452206419}};
  for (bool early : {true, false}) {
    SCOPED_TRACE(early);
    Problem p(3);
    for (int i = 0; i < 3; ++i)
      std::copy(matrix[i], matrix[i] + 3, p.A.begin() + i * p.stride);
    p.b = {-1.5812411969951867, -0.2238416945870532, -0.3709961947182103};
    auto reference = p;
    FbfFrictionSolver::Options options;
    options.maxOuterIterations = 3;
    options.tolerance = 1e-12;
    FbfFrictionSolver solver(options);
    NsgsFrictionSolver::Options nsgsOptions;
    nsgsOptions.maxSweeps = options.maxOuterIterations;
    nsgsOptions.tolerance = options.tolerance;
    NsgsFrictionSolver nsgs(nsgsOptions);
    ASSERT_TRUE(p.solve(solver, early));
    ASSERT_TRUE(reference.solve(nsgs, early));
    EXPECT_EQ(
        0, std::memcmp(p.x.data(), reference.x.data(), 3 * sizeof(double)));
    for (int i = 0; i < 3; ++i)
      EXPECT_NEAR(expected[i], p.x[i], 1e-14);
    const auto stats = solver.getStats();
    EXPECT_NEAR(0.8638191468189201, stats.maxViolation, 1e-14);
    EXPECT_EQ(nsgs.getStats().maxViolation, stats.maxViolation);
    EXPECT_EQ(3u, stats.numIterations);
    EXPECT_EQ(3u, stats.numInnerIterations);
    EXPECT_EQ(0u, stats.numInnerCaps);
    EXPECT_EQ(1u, stats.numAcceptedAtCap);
    EXPECT_EQ(0u, stats.numConverged);
    EXPECT_EQ(0u, stats.numFailed);
  }
}

TEST(FbfFrictionSolver, UncertifiedLocalSolveHonorsTerminationMode)
{
  const Eigen::Matrix3d matrix
      = Eigen::Vector3d(std::numeric_limits<double>::denorm_min(), 2.0, 2.0)
            .asDiagonal();
  const detail::FrictionCone zeroCone{
      Eigen::Vector2d::Zero(), detail::FrictionConeLaw::Ellipse};
  ASSERT_FALSE(
      detail::solveConeQp(matrix, Eigen::Vector3d(-1.0, 0.0, 0.0), zeroCone)
          .certified);
  for (bool proximal : {false, true}) {
    SCOPED_TRACE(proximal);
    for (bool early : {true, false}) {
      SCOPED_TRACE(early);
      FbfFrictionSolver solver;
      auto p = slidingContact();
      if (proximal) {
        p.A[1] = 0.5;
      } else {
        for (int i = 0; i < 3; ++i)
          for (int j = 0; j < 3; ++j)
            p.A[i * p.stride + j] = matrix(i, j);
        p.b = {1.0, 0.0, 0.0};
        p.lo[1] = p.lo[2] = p.hi[1] = p.hi[2] = 0.0;
      }
      p.x = {0.25, 0.0, 0.0};
      const auto start = p.x;
      EXPECT_EQ(!early, p.solve(solver, early));
      EXPECT_EQ(start, p.x);
      const auto stats = solver.getStats();
      EXPECT_EQ(early ? 1u : 0u, stats.numFailed);
      EXPECT_EQ(early ? 0u : 1u, stats.numAcceptedAtCap);
      EXPECT_EQ(0u, stats.numConverged);
      EXPECT_EQ(1u, stats.numIterations);
      EXPECT_EQ(1u, stats.numInnerIterations);
      EXPECT_EQ(1u, stats.numContacts);
      EXPECT_EQ(0u, stats.numBoxContacts);
      EXPECT_NEAR(
          early ? 0.0 : (proximal ? 0.680073525437 : 1.0),
          stats.maxViolation,
          1e-12);
    }
  }
}

TEST(FbfFrictionSolver, ZeroNormalCurvatureKeepsBoundedBestIterateAtCap)
{
  FbfFrictionSolver::Options options;
  options.maxOuterIterations = 3;
  FbfFrictionSolver solver(options);
  auto p = slidingContact();
  p.A[0] = 0.0;
  p.b = {1.0, 0.0, 0.0};
  ASSERT_TRUE(p.solve(solver));
  for (double impulse : p.x)
    EXPECT_EQ(0.0, impulse);
  const auto stats = solver.getStats();
  EXPECT_EQ(1u, stats.numContacts);
  EXPECT_EQ(0u, stats.numConverged);
  EXPECT_EQ(1u, stats.numAcceptedAtCap);
  EXPECT_EQ(3u, stats.numIterations);
  EXPECT_EQ(0u, stats.numFailed);
  EXPECT_EQ(1.0, stats.maxViolation);
}

TEST(FbfFrictionSolver, InvalidOptionsAndIndicesFailClosed)
{
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();
  for (bool early : {true, false}) {
    SCOPED_TRACE(early);
    FbfFrictionSolver solver;
    auto reject = [&](const FbfFrictionSolver::Options& options, Problem p) {
      solver.setOptions(options);
      p.x[0] = 0.25;
      const auto start = p.x;
      EXPECT_FALSE(p.solve(solver, early));
      EXPECT_EQ(start, p.x);
      EXPECT_EQ(0u, solver.getStats().numInnerIterations);
    };
    FbfFrictionSolver::Options options;
    options.maxOuterIterations = -1;
    reject(options, Problem(1));
    for (double value : {-1.0, nan, inf}) {
      options = FbfFrictionSolver::Options{};
      options.tolerance = value;
      reject(options, Problem(1));
    }
    for (double value : {0.0, -1.0, nan, inf}) {
      options = FbfFrictionSolver::Options{};
      options.stepScale = value;
      reject(options, Problem(1));
    }
    for (int value : {0, -1}) {
      options = FbfFrictionSolver::Options{};
      options.maxInnerSweeps = value;
      reject(options, Problem(1));
    }
    for (double value : {-1.0, nan, inf}) {
      options = FbfFrictionSolver::Options{};
      options.innerToleranceFactor = value;
      reject(options, Problem(1));
    }
    options = FbfFrictionSolver::Options{};
    for (int index : {-2, 1}) {
      Problem p(1);
      p.findex[0] = index;
      reject(options, p);
    }
    Problem reversed(1);
    reversed.lo[0] = 2.0;
    reversed.hi[0] = 1.0;
    reject(options, reversed);
    Problem negative(1);
    negative.A[0] = -1.0;
    reject(options, negative);
    auto overflow = slidingContact();
    overflow.lo[1] = overflow.lo[2] = -1e100;
    overflow.hi[1] = overflow.hi[2] = 1e100;
    overflow.A[overflow.stride + 1] = 1e300;
    reject(options, overflow);
    const auto stats = solver.getStats();
    EXPECT_EQ(18u, stats.numSolves);
    EXPECT_EQ(18u, stats.numFailed);
    EXPECT_EQ(0u, stats.numAcceptedAtCap);
    EXPECT_EQ(0u, stats.numConverged);
    EXPECT_EQ(0.0, stats.maxViolation);
  }
}

TEST(FbfFrictionSolver, StatsAccumulateAcrossThreadsAndReset)
{
  FbfFrictionSolver one;
  auto p = slidingContact();
  ASSERT_TRUE(p.solve(one));
  const auto reference = one.getStats();
  FbfFrictionSolver solver;
  constexpr int workers = 4;
  constexpr int solves = 8;
  std::array<std::future<bool>, workers> results;
  for (auto& result : results)
    result = std::async(std::launch::async, [&] {
      solver.reserve(3);
      for (int i = 0; i < solves; ++i) {
        auto problem = slidingContact();
        if (!problem.solve(solver))
          return false;
      }
      return true;
    });
  for (auto& result : results)
    EXPECT_TRUE(result.get());
  expectStatsEqual(reference, solver.getStats(), workers * solves);
  solver.resetStats();
  expectStatsEqual(FrictionSolveStats{}, solver.getStats());
}

TEST(FbfFrictionSolver, OneDimensionalAndTwoRowFriction)
{
  for (bool box : {true, false}) {
    SCOPED_TRACE(box);
    FbfFrictionSolver::Options options;
    options.boxForAnisotropic = box;
    options.tolerance = 1e-10;
    for (double minor : {0.0, 1e-8}) {
      SCOPED_TRACE(minor);
      FbfFrictionSolver solver(options);
      auto p = slidingContact();
      p.lo[2] = -minor;
      p.hi[2] = minor;
      ASSERT_TRUE(p.solve(solver));
      EXPECT_NEAR(1.0, p.x[0], 1e-9);
      EXPECT_NEAR(0.5, p.x[1], 1e-9);
      EXPECT_EQ(0.0, p.x[2]);
      expectConverged(solver);
      EXPECT_EQ(box ? 1u : 0u, solver.getStats().numBoxContacts);
    }
    Problem p(2);
    p.b = {1.0, 2.0};
    p.lo = {0.0, -0.5};
    p.hi = {std::numeric_limits<double>::infinity(), 0.5};
    p.findex = {-1, 0};
    FbfFrictionSolver solver(options);
    ASSERT_TRUE(p.solve(solver));
    EXPECT_NEAR(1.0, p.x[0], 1e-9);
    EXPECT_NEAR(0.5, p.x[1], 1e-9);
    EXPECT_EQ(1u, solver.getStats().numContacts);
    EXPECT_EQ(box ? 1u : 0u, solver.getStats().numBoxContacts);
    expectConverged(solver);
  }
}

TEST(FbfFrictionSolver, MixedContactLimitAndUnboundedRows)
{
  FbfFrictionSolver::Options options;
  options.tolerance = 1e-9;
  FbfFrictionSolver solver(options);
  auto p = mixedProblem();
  ASSERT_TRUE(p.solve(solver));
  const std::array<double, 5> expected{{1.0, -0.5, 0.0, 0.3, -0.2}};
  for (int i = 0; i < 5; ++i)
    EXPECT_NEAR(expected[i], p.x[i], 1e-8);
  expectConverged(solver);
  EXPECT_EQ(1u, solver.getStats().numContacts);
}

TEST(FbfFrictionSolver, CappedContactUsesPgsRowsInsideProximalSolve)
{
  Problem p(6);
  p.b = {1.0, 2.0, 2.0, 1.0, 0.3, -0.4};
  p.lo = {0.0, -0.5, -0.5, 0.0, -0.4, -0.4};
  p.hi = {std::numeric_limits<double>::infinity(), 0.5, 0.5, 0.25, 0.4, 0.4};
  p.findex = {-1, 0, 0, -1, 3, 3};
  for (const auto& pair : {std::pair{0, 3}, std::pair{1, 4}, std::pair{2, 5}})
    p.A[pair.first * p.stride + pair.second]
        = p.A[pair.second * p.stride + pair.first] = 0.2;
  p.A[1] = p.A[p.stride] = 0.1;
  FbfFrictionSolver::Options options;
  options.tolerance = 1e-9;
  options.maxOuterIterations = 1000;
  FbfFrictionSolver solver(options);
  ASSERT_TRUE(p.solve(solver));
  expectConverged(solver);
  const auto stats = solver.getStats();
  EXPECT_GT(stats.numIterations, 0u);
  EXPECT_EQ(2u, stats.numContacts);
  EXPECT_EQ(1u, stats.numBoxContacts);
  EXPECT_EQ(0.25, p.x[3]);
  EXPECT_EQ(0.1, p.x[4]);
  EXPECT_EQ(-0.1, p.x[5]);
  const std::array<double, 3> expected{{0.918635048, 0.313649506, 0.335554132}};
  for (int i = 0; i < 3; ++i)
    EXPECT_NEAR(expected[i], p.x[i], 1e-8);
  for (int row : {3, 4, 5}) {
    const double velocity = detail::rowVelocity(
        row, p.n, p.stride, p.A.data(), p.x.data(), p.b.data());
    EXPECT_LE(
        detail::scalarViolation(
            row,
            velocity,
            p.A[row * p.stride + row],
            p.x.data(),
            p.lo.data(),
            p.hi.data(),
            p.findex.data()),
        options.tolerance);
  }
}

TEST(FbfFrictionSolver, StiffScalarRowDoesNotShrinkTheStep)
{
  auto p = boxOnGround(0.0, 0.0);
  Problem extended(13);
  for (int i = 0; i < p.n; ++i) {
    for (int j = 0; j < p.n; ++j)
      extended.A[i * extended.stride + j] = p.A[i * p.stride + j];
    extended.b[i] = p.b[i];
    extended.lo[i] = p.lo[i];
    extended.hi[i] = p.hi[i];
    extended.findex[i] = p.findex[i];
  }
  extended.A[12 * extended.stride + 12] = 1000.0;
  extended.A[12] = extended.A[12 * extended.stride] = 0.1;
  extended.b[12] = -0.01;
  extended.lo[12] = 0.0;
  FbfFrictionSolver base, stiff;
  ASSERT_TRUE(p.solve(base));
  ASSERT_TRUE(extended.solve(stiff));
  expectConverged(base);
  expectConverged(stiff);
  EXPECT_EQ(base.getStats().numIterations, stiff.getStats().numIterations);
  for (int i = 0; i < p.n; ++i)
    EXPECT_EQ(p.x[i], extended.x[i]);
  EXPECT_EQ(0.0, extended.x[12]);
}

TEST(FbfFrictionSolver, CodexRegressionProblems)
{
  Eigen::Matrix3d orthogonal, tiny;
  orthogonal << 50.5, -49.5, 0.0, -49.5, 50.5, 0.0, 0.0, 0.0, 1.0;
  tiny << 2.0, 0.3, -0.2, 0.3, 1.5, 0.1, -0.2, 0.1, 1.2;
  struct Case
  {
    Eigen::Matrix3d matrix;
    Eigen::Vector3d rhs;
    double mu;
    Eigen::Vector3d expected;
    Eigen::Vector3d margin;
    int outer;
  };
  const std::array<Case, 4> cases{
      {{orthogonal,
        Eigen::Vector3d(1.0, 0.5, -0.5),
        0.5,
        Eigen::Vector3d(0.036927798803, 0.017471794738, -0.005970927041),
        Eigen::Vector3d::Constant(1e-9),
        300},
       {tiny,
        Eigen::Vector3d(1.0, 2.0, -1.0),
        1e-8,
        Eigen::Vector3d(0.4999999991068, 4.496175434696e-9, -2.187328592398e-9),
        Eigen::Vector3d(2e-9, 1e-14, 1e-14),
        300},
       {Eigen::Matrix3d::Identity(),
        Eigen::Vector3d(1.0, 0.3, 0.0),
        0.5,
        Eigen::Vector3d(0.5, 0.25, 0.0),
        Eigen::Vector3d::Zero(),
        300},
       {Eigen::Vector3d(1.0, 100.0, 1.0).asDiagonal(),
        Eigen::Vector3d(1.0, 30.0, 0.2),
        0.5,
        Eigen::Vector3d(1.0, 0.3, 0.2),
        Eigen::Vector3d::Constant(1e-8),
        5000}}};
  for (std::size_t index = 0; index < cases.size(); ++index) {
    SCOPED_TRACE(index);
    const auto& c = cases[index];
    auto p = contactProblem(c.matrix, c.rhs, {{c.mu, c.mu}});
    if (index == 2)
      p.hi[0] = 0.5;
    FbfFrictionSolver::Options options;
    options.tolerance = 1e-9;
    options.maxOuterIterations = c.outer;
    FbfFrictionSolver solver(options);
    ASSERT_TRUE(p.solve(solver));
    expectConverged(solver);
    for (int i = 0; i < 3; ++i) {
      if (c.margin[i] == 0.0) {
        EXPECT_EQ(c.expected[i], p.x[i]);
      } else {
        EXPECT_NEAR(c.expected[i], p.x[i], c.margin[i]);
      }
    }
    EXPECT_EQ(index == 2 ? 1u : 0u, solver.getStats().numBoxContacts);
  }
}

TEST(FbfFrictionSolver, BoxContactsDoNotGlide)
{
  FbfFrictionSolver::Options options;
  options.tolerance = 1e-9;
  options.maxOuterIterations = 500;
  FbfFrictionSolver solver(options);
  auto p = slidingContact();
  p.lo[2] = -1.0;
  p.hi[2] = 1.0;
  ASSERT_TRUE(p.solve(solver));
  expectConverged(solver);
  EXPECT_NEAR(1.0, p.x[0], 1e-8);
  EXPECT_NEAR(0.5, p.x[1], 1e-8);
  EXPECT_NEAR(1.0, p.x[2], 1e-8);
  EXPECT_LE(std::abs(p.x[0] - p.b[0]), 1e-8);

  options.maxOuterIterations = 5000;
  BankRng rng;
  int sliding = 0;
  for (int problem = 0; problem < 50; ++problem) {
    SCOPED_TRACE(problem);
    Eigen::Matrix3d R;
    for (int i = 0; i < 9; ++i)
      R(i / 3, i % 3) = rng.normal();
    const Eigen::Matrix3d matrix
        = R * R.transpose() + 0.5 * Eigen::Matrix3d::Identity();
    const double cn = -0.05 - 0.95 * rng.uniform();
    const double ct1 = 2.0 * rng.normal();
    const double ct2 = 2.0 * rng.normal();
    const Eigen::Vector3d linear(cn, ct1, ct2);
    auto q = contactProblem(matrix, -linear, {{0.5, 0.3}});
    FbfFrictionSolver backend(options);
    ASSERT_TRUE(q.solve(backend));
    expectConverged(backend);
    EXPECT_EQ(1u, backend.getStats().numBoxContacts);
    const Eigen::Vector3d impulse(q.x[0], q.x[1], q.x[2]);
    const Eigen::Vector3d velocity = matrix * impulse + linear;
    if (impulse[0] > 1e-9) {
      EXPECT_LE(std::abs(velocity[0]), 1e-8);
      if (std::min(std::abs(velocity[1]), std::abs(velocity[2])) > 1e-6)
        ++sliding;
    }
    const detail::FrictionCone cone{
        Eigen::Vector2d(0.5, 0.3), detail::FrictionConeLaw::Box};
    const auto exact = detail::solveExactContact(matrix, linear, cone);
    ASSERT_TRUE(exact.certified);
    for (int i = 0; i < 3; ++i)
      EXPECT_NEAR(exact.impulse[i], impulse[i], 1e-6);
  }
  EXPECT_GE(sliding, 25);
}
