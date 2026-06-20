#include "dart/lcpsolver/dantzig/DantzigCommon.hpp"
#include "dart/lcpsolver/dantzig/DantzigLcp.hpp"
#include "dart/lcpsolver/dantzig/DantzigMisc.hpp"
#include "dart/lcpsolver/dantzig/DantzigPivotMatrix.hpp"
#include "dart/lcpsolver/dantzig/lcp.h"
#include "tests/baseline/odelcpsolver/lcp.h"

#include <gtest/gtest.h>

#include <vector>

namespace {

using dart::lcpsolver::dantzig::Infinity;
using dart::lcpsolver::dantzig::Multiply0;
using dart::lcpsolver::dantzig::Multiply1;
using dart::lcpsolver::dantzig::Multiply2;
using dart::lcpsolver::dantzig::Padding;

struct DantzigProblem
{
  int n = 0;
  int nub = 0;
  std::vector<double> A;
  std::vector<double> b;
  std::vector<double> lo;
  std::vector<double> hi;
  std::vector<int> findex;
};

DantzigProblem makeUnboundedProblem()
{
  DantzigProblem problem;
  problem.n = 1;
  problem.nub = 1;
  problem.A = {2.0};
  problem.b = {4.0};
  problem.lo = {-Infinity};
  problem.hi = {Infinity};
  problem.findex = {-1};
  return problem;
}

DantzigProblem makeBoxedProblem()
{
  DantzigProblem problem;
  problem.n = 2;
  problem.nub = 0;
  const int nskip = Padding(problem.n);
  problem.A.assign(static_cast<std::size_t>(problem.n * nskip), 0.0);
  problem.A[0 * nskip + 0] = 2.0;
  problem.A[1 * nskip + 1] = 3.0;
  problem.b = {1.0, 3.0};
  problem.lo = {0.0, 0.0};
  problem.hi = {10.0, 10.0};
  problem.findex = {-1, -1};
  return problem;
}

std::vector<double> solveNative(DantzigProblem problem)
{
  std::vector<double> x(static_cast<std::size_t>(problem.n), 0.0);
  std::vector<double> w(static_cast<std::size_t>(problem.n), 0.0);

  EXPECT_TRUE(dart::lcpsolver::dantzig::SolveLCP<double>(
      problem.n,
      problem.A.data(),
      x.data(),
      problem.b.data(),
      w.data(),
      problem.nub,
      problem.lo.data(),
      problem.hi.data(),
      problem.findex.data()));

  return x;
}

std::vector<double> solveNativeCompat(DantzigProblem problem)
{
  std::vector<double> x(static_cast<std::size_t>(problem.n), 0.0);
  std::vector<double> w(static_cast<std::size_t>(problem.n), 0.0);

  EXPECT_TRUE(dart::lcpsolver::dantzig::dSolveLCP(
      problem.n,
      problem.A.data(),
      x.data(),
      problem.b.data(),
      w.data(),
      problem.nub,
      problem.lo.data(),
      problem.hi.data(),
      problem.findex.data()));

  return x;
}

std::vector<double> solveBaseline(DantzigProblem problem)
{
  std::vector<double> x(static_cast<std::size_t>(problem.n), 0.0);
  std::vector<double> w(static_cast<std::size_t>(problem.n), 0.0);

  EXPECT_TRUE(dart::baseline::ode::dSolveLCP(
      problem.n,
      problem.A.data(),
      x.data(),
      problem.b.data(),
      w.data(),
      problem.nub,
      problem.lo.data(),
      problem.hi.data(),
      problem.findex.data()));

  return x;
}

void expectNear(
    const std::vector<double>& actual, const std::vector<double>& expected)
{
  ASSERT_EQ(actual.size(), expected.size());
  for (std::size_t i = 0; i < actual.size(); ++i) {
    EXPECT_NEAR(actual[i], expected[i], 1e-12);
  }
}

void expectMatrixMultiplyUsesPaddedStrides(int p, int q, int r)
{
  const int pskip = Padding(p);
  const int qskip = Padding(q);
  const int rskip = Padding(r);

  std::vector<double> b0(static_cast<std::size_t>(p * qskip), -1.0);
  std::vector<double> b1(static_cast<std::size_t>(q * pskip), -1.0);
  for (int i = 0; i < p; ++i) {
    for (int k = 0; k < q; ++k) {
      const double value = static_cast<double>(i * q + k + 1);
      b0[i * qskip + k] = value;
      b1[k * pskip + i] = value;
    }
  }

  std::vector<double> c0(static_cast<std::size_t>(q * rskip), -1.0);
  std::vector<double> c2(static_cast<std::size_t>(r * qskip), -1.0);
  for (int k = 0; k < q; ++k) {
    for (int j = 0; j < r; ++j) {
      const double value = static_cast<double>((k + 1) * (j + 2));
      c0[k * rskip + j] = value;
      c2[j * qskip + k] = value;
    }
  }

  std::vector<double> expected(static_cast<std::size_t>(p * r), 0.0);
  for (int i = 0; i < p; ++i) {
    for (int j = 0; j < r; ++j) {
      for (int k = 0; k < q; ++k) {
        expected[i * r + j] += b0[i * qskip + k] * c0[k * rskip + j];
      }
    }
  }

  const auto expectProduct
      = [p, r, rskip, &expected](const std::vector<double>& actual) {
          for (int i = 0; i < p; ++i) {
            for (int j = 0; j < r; ++j) {
              EXPECT_DOUBLE_EQ(expected[i * r + j], actual[i * rskip + j]);
            }
            for (int j = r; j < rskip; ++j) {
              EXPECT_EQ(-1.0, actual[i * rskip + j]);
            }
          }
        };

  std::vector<double> a(static_cast<std::size_t>(p * rskip), -1.0);
  Multiply0(a.data(), b0.data(), c0.data(), p, q, r);
  expectProduct(a);

  std::fill(a.begin(), a.end(), -1.0);
  Multiply1(a.data(), b1.data(), c0.data(), p, q, r);
  expectProduct(a);

  std::fill(a.begin(), a.end(), -1.0);
  Multiply2(a.data(), b0.data(), c2.data(), p, q, r);
  expectProduct(a);
}

} // namespace

TEST(DantzigNative, SolvesUnboundedProblem)
{
  expectNear(solveNative(makeUnboundedProblem()), {2.0});
}

TEST(DantzigNative, CompatibilityHeaderForwardsToNativeKernel)
{
  expectNear(solveNativeCompat(makeUnboundedProblem()), {2.0});
}

TEST(DantzigNative, MatchesOdeBaselineOnBoxedProblem)
{
  const DantzigProblem problem = makeBoxedProblem();

  expectNear(solveNative(problem), solveBaseline(problem));
}

TEST(DantzigNative, RandomIntStaysInRangeAtMaximumRand)
{
  constexpr unsigned long seedBeforeMaxRand = 0x0dbdbb1eUL;
  constexpr int range = 7;

  dart::lcpsolver::dantzig::dRandSetSeed(seedBeforeMaxRand);
  EXPECT_EQ(0xffffffffUL, dart::lcpsolver::dantzig::dRand());

  dart::lcpsolver::dantzig::dRandSetSeed(seedBeforeMaxRand);
  const int actual = dart::lcpsolver::dantzig::dRandInt(range);

  EXPECT_GE(actual, 0);
  EXPECT_LT(actual, range);
}

TEST(DantzigNative, PivotMatrixExternalViewUsesActiveRowPointers)
{
  constexpr int rows = 2;
  constexpr int cols = 2;
  const int nskip = Padding(cols);
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
