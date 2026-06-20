#include "DantzigProblemCases.hpp"
#include "dart/constraint/ConstrainedGroup.hpp"
#include "dart/constraint/ConstraintBase.hpp"
#include "dart/constraint/DantzigBoxedLcpSolver.hpp"
#include "dart/constraint/DantzigLCPSolver.hpp"
#include "dart/constraint/PgsBoxedLcpSolver.hpp"
#include "dart/lcpsolver/ODELCPSolver.hpp"
#include "dart/lcpsolver/dantzig/DantzigCommon.hpp"
#include "dart/lcpsolver/dantzig/DantzigMisc.hpp"
#include "dart/lcpsolver/dantzig/DantzigPivotMatrix.hpp"
#include "dart/lcpsolver/dantzig/lcp.h"

#include <gtest/gtest.h>

#include <algorithm>
#include <memory>
#include <utility>
#include <vector>

#include <cmath>

namespace {

using dart::lcpsolver::dantzig::DantzigLcpScratch;
using dart::lcpsolver::dantzig::kInfinity;
using dart::lcpsolver::dantzig::multiply0;
using dart::lcpsolver::dantzig::multiply1;
using dart::lcpsolver::dantzig::multiply2;
using dart::lcpsolver::dantzig::padding;
using dart::test::makeDantzigCorrectnessCases;
using dart::test::solveDantzigBaseline;
using dart::test::solveDantzigNative;

class SingleDofConstraint final : public dart::constraint::ConstraintBase
{
public:
  SingleDofConstraint(double matrixValue, double rhs)
    : mMatrixValue(matrixValue), mRhs(rhs)
  {
    mDim = 1;
  }

  void update() override {}

  void getInformation(dart::constraint::ConstraintInfo* info) override
  {
    info->x[0] = 0.0;
    info->lo[0] = -kInfinity;
    info->hi[0] = kInfinity;
    info->b[0] = mRhs;
    info->w[0] = 0.0;
    info->findex[0] = -1;
  }

  void applyUnitImpulse(std::size_t index) override
  {
    mLastUnitImpulse = index;
  }

  void getVelocityChange(double* vel, bool /*withCfm*/) override
  {
    ASSERT_EQ(0u, mLastUnitImpulse);
    vel[0] = mMatrixValue;
  }

  void excite() override
  {
    mExcited = true;
  }

  void unexcite() override
  {
    mExcited = false;
  }

  void applyImpulse(double* lambda) override
  {
    mAppliedImpulse = lambda[0];
  }

  bool isActive() const override
  {
    return true;
  }

  dart::dynamics::SkeletonPtr getRootSkeleton() const override
  {
    return nullptr;
  }

  double getAppliedImpulse() const
  {
    return mAppliedImpulse;
  }

  bool isExcited() const
  {
    return mExcited;
  }

private:
  double mMatrixValue;
  double mRhs;
  std::size_t mLastUnitImpulse = 0;
  double mAppliedImpulse = 0.0;
  bool mExcited = false;
};

void expectNear(
    const std::vector<double>& actual,
    const std::vector<double>& expected,
    double tolerance)
{
  ASSERT_EQ(actual.size(), expected.size());
  for (std::size_t i = 0; i < actual.size(); ++i) {
    EXPECT_NEAR(actual[i], expected[i], tolerance) << "index " << i;
  }
}

void expectMatrixMultiplyUsesPaddedStrides(int p, int q, int r)
{
  const int pskip = padding(p);
  const int qskip = padding(q);
  const int rskip = padding(r);

  std::vector<double> b0(static_cast<std::size_t>(p * qskip), -1.0);
  std::vector<double> b1(static_cast<std::size_t>(q * pskip), -1.0);
  for (int i = 0; i < p; ++i) {
    for (int k = 0; k < q; ++k) {
      const double value = static_cast<double>(i * q + k + 1);
      b0[static_cast<std::size_t>(i * qskip + k)] = value;
      b1[static_cast<std::size_t>(k * pskip + i)] = value;
    }
  }

  std::vector<double> c0(static_cast<std::size_t>(q * rskip), -1.0);
  std::vector<double> c2(static_cast<std::size_t>(r * qskip), -1.0);
  for (int k = 0; k < q; ++k) {
    for (int j = 0; j < r; ++j) {
      const double value = static_cast<double>((k + 1) * (j + 2));
      c0[static_cast<std::size_t>(k * rskip + j)] = value;
      c2[static_cast<std::size_t>(j * qskip + k)] = value;
    }
  }

  std::vector<double> expected(static_cast<std::size_t>(p * r), 0.0);
  for (int i = 0; i < p; ++i) {
    for (int j = 0; j < r; ++j) {
      for (int k = 0; k < q; ++k) {
        expected[static_cast<std::size_t>(i * r + j)]
            += b0[static_cast<std::size_t>(i * qskip + k)]
               * c0[static_cast<std::size_t>(k * rskip + j)];
      }
    }
  }

  const auto expectProduct
      = [p, r, rskip, &expected](const std::vector<double>& actual) {
          for (int i = 0; i < p; ++i) {
            for (int j = 0; j < r; ++j) {
              EXPECT_DOUBLE_EQ(
                  expected[static_cast<std::size_t>(i * r + j)],
                  actual[static_cast<std::size_t>(i * rskip + j)]);
            }
            for (int j = r; j < rskip; ++j) {
              EXPECT_EQ(-1.0, actual[static_cast<std::size_t>(i * rskip + j)]);
            }
          }
        };

  std::vector<double> a(static_cast<std::size_t>(p * rskip), -1.0);
  multiply0(a.data(), b0.data(), c0.data(), p, q, r);
  expectProduct(a);

  std::fill(a.begin(), a.end(), -1.0);
  multiply1(a.data(), b1.data(), c0.data(), p, q, r);
  expectProduct(a);

  std::fill(a.begin(), a.end(), -1.0);
  multiply2(a.data(), b0.data(), c2.data(), p, q, r);
  expectProduct(a);
}

} // namespace

TEST(DantzigNative, MatchesLegacyBaselineAcrossRepresentativeProblems)
{
  for (const auto& problem : makeDantzigCorrectnessCases()) {
    std::vector<double> native;
    std::vector<double> baseline;

    ASSERT_TRUE(solveDantzigNative(problem, &native)) << problem.name;
    ASSERT_TRUE(solveDantzigBaseline(problem, &baseline)) << problem.name;
    expectNear(native, baseline, 1e-10);
  }
}

TEST(DantzigNative, CompatibilityHeaderForwardsToNativeKernel)
{
  auto problem = makeDantzigCorrectnessCases().front();
  std::vector<double> native;
  std::vector<double> compat(static_cast<std::size_t>(problem.n), 0.0);
  std::vector<double> w(static_cast<std::size_t>(problem.n), 0.0);

  ASSERT_TRUE(solveDantzigNative(problem, &native));
  ASSERT_TRUE(dart::lcpsolver::dantzig::dSolveLCP(
      problem.n,
      problem.A.data(),
      compat.data(),
      problem.b.data(),
      w.data(),
      problem.nub,
      problem.lo.data(),
      problem.hi.data(),
      problem.findex.data()));

  expectNear(compat, native, 1e-12);
}

TEST(DantzigNative, RandomIntStaysInRangeAtMaximumRand)
{
  constexpr unsigned long seedBeforeMaxRand = 0x0dbdbb1eUL;
  constexpr int range = 7;

  dart::lcpsolver::dantzig::setRandomSeed(seedBeforeMaxRand);
  EXPECT_EQ(0xffffffffUL, dart::lcpsolver::dantzig::random());

  dart::lcpsolver::dantzig::setRandomSeed(seedBeforeMaxRand);
  const int actual = dart::lcpsolver::dantzig::randomInt(range);

  EXPECT_GE(actual, 0);
  EXPECT_LT(actual, range);
}

TEST(DantzigNative, PivotMatrixExternalViewUsesActiveRowPointers)
{
  constexpr int rows = 2;
  constexpr int cols = 2;
  const int nskip = padding(cols);
  std::vector<double> data(static_cast<std::size_t>(rows * nskip), 0.0);
  data[0 * nskip + 0] = 1.0;
  data[0 * nskip + 1] = 2.0;
  data[1 * nskip + 0] = 3.0;
  data[1 * nskip + 1] = 4.0;
  std::vector<double*> rowPointers(static_cast<std::size_t>(rows), nullptr);

  dart::lcpsolver::dantzig::PivotMatrix<double> matrix(
      rows, cols, data.data(), nskip, rowPointers.data());

  EXPECT_EQ(1.0, matrix(0, 0));
  EXPECT_EQ(4.0, matrix(1, 1));

  matrix(0, 1) = 20.0;
  EXPECT_EQ(20.0, data[0 * nskip + 1]);

  matrix.swapRows(0, 1);
  EXPECT_EQ(3.0, matrix(0, 0));
  EXPECT_EQ(20.0, matrix(1, 1));

  const auto& constMatrix = matrix;
  EXPECT_EQ(4.0, constMatrix(0, 1));
}

TEST(DantzigNative, MatrixMultiplyUsesPaddedStrides)
{
  expectMatrixMultiplyUsesPaddedStrides(2, 3, 2);
  expectMatrixMultiplyUsesPaddedStrides(2, 10, 10);
}

TEST(DantzigNative, PublicHeaderDoesNotLeakPrivateSolverMacros)
{
#if defined(ROWPTRS) || defined(AROW) || defined(NUB_OPTIMIZATIONS)
  FAIL() << "DantzigLcp.hpp leaked private implementation macros";
#else
  SUCCEED();
#endif
}

TEST(DantzigNative, ScratchMoveAssignmentKeepsStateAllocatorValid)
{
  DantzigLcpScratch<double> source;
  source.reserveState(3);
  source.state[0] = true;
  source.state[1] = false;
  source.state[2] = true;

  DantzigLcpScratch<double> target;
  target = std::move(source);

  ASSERT_NE(nullptr, target.state);
  EXPECT_EQ(3u, target.stateCapacity);
  EXPECT_TRUE(target.state[0]);
  EXPECT_FALSE(target.state[1]);
  EXPECT_TRUE(target.state[2]);
  EXPECT_EQ(nullptr, source.state);
  EXPECT_EQ(0u, source.stateCapacity);

  target.reserveState(4);
  EXPECT_EQ(4u, target.stateCapacity);
}

TEST(DantzigNative, PgsBoxedLcpSolverCoversNativeLdltAndRandomPaths)
{
  {
    auto problem = dart::test::makeUnboundedDantzigCase(3);
    std::vector<double> x(static_cast<std::size_t>(problem.n), 0.0);
    dart::constraint::PgsBoxedLcpSolver solver;

    EXPECT_TRUE(solver.solve(
        problem.n,
        problem.A.data(),
        x.data(),
        problem.b.data(),
        problem.nub,
        problem.lo.data(),
        problem.hi.data(),
        problem.findex.data(),
        false));
    for (double value : x) {
      EXPECT_TRUE(std::isfinite(value));
    }
  }

  {
    auto problem = dart::test::makeBoxedCoupledDantzigCase(5);
    std::vector<double> x(static_cast<std::size_t>(problem.n), 0.0);
    dart::constraint::PgsBoxedLcpSolver solver;
    solver.setOption(
        dart::constraint::PgsBoxedLcpSolver::Option(9, 0.0, -1.0, 1e-9, true));

    solver.solve(
        problem.n,
        problem.A.data(),
        x.data(),
        problem.b.data(),
        problem.nub,
        problem.lo.data(),
        problem.hi.data(),
        problem.findex.data(),
        false);
    for (double value : x) {
      EXPECT_TRUE(std::isfinite(value));
    }
  }
}

TEST(DantzigNative, DeprecatedOdeLcpSolverUsesNativeDantzigPath)
{
  const Eigen::MatrixXd A = (Eigen::MatrixXd(1, 1) << 2.0).finished();
  const Eigen::VectorXd b = (Eigen::VectorXd(1) << -4.0).finished();
  Eigen::VectorXd x;
  dart::lcpsolver::ODELCPSolver solver;

  ASSERT_TRUE(solver.Solve(A, b, &x, 0, 0.0, 4, true));
  ASSERT_EQ(1, x.size());
  EXPECT_NEAR(2.0, x[0], 1e-12);
}

TEST(DantzigNative, DantzigBoxedLcpSolverKeepsPublicLayoutStable)
{
  EXPECT_EQ(
      sizeof(dart::constraint::BoxedLcpSolver),
      sizeof(dart::constraint::DantzigBoxedLcpSolver));

  auto problem = dart::test::makeBoxedDiagonalDantzigCase(4);
  std::vector<double> boxed(static_cast<std::size_t>(problem.n), 0.0);
  std::vector<double> native;
  dart::constraint::DantzigBoxedLcpSolver solver;

  ASSERT_TRUE(solveDantzigNative(problem, &native));
  ASSERT_TRUE(solver.solve(
      problem.n,
      problem.A.data(),
      boxed.data(),
      problem.b.data(),
      problem.nub,
      problem.lo.data(),
      problem.hi.data(),
      problem.findex.data(),
      false));

  expectNear(boxed, native, 1e-12);
}

TEST(DantzigNative, DeprecatedDantzigLcpSolverSolvesConstrainedGroup)
{
  auto constraint = std::make_shared<SingleDofConstraint>(2.0, 4.0);
  dart::constraint::ConstrainedGroup group;
  group.addConstraint(constraint);

  dart::constraint::DantzigLCPSolver solver(0.001);
  solver.solve(&group);

  EXPECT_TRUE(constraint->isExcited());
  EXPECT_NEAR(2.0, constraint->getAppliedImpulse(), 1e-12);
}
