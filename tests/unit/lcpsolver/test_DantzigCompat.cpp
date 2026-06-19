#include "dart/external/odelcpsolver/lcp.h"
#include "dart/external/odelcpsolver/matrix.h"
#include "dart/lcpsolver/dantzig/lcp.h"

#include <gtest/gtest.h>

#undef dSetZero

namespace dart {
namespace external {
namespace ode {

using dReal = ::dart::lcpsolver::dantzig::dReal;

bool dSolveLCP(
    int n,
    dReal* A,
    dReal* x,
    dReal* b,
    dReal* w,
    int nub,
    dReal* lo,
    dReal* hi,
    int* findex,
    bool earlyTermination = false);

void dSetZero(dReal* a, int n);

} // namespace ode
} // namespace external
} // namespace dart

namespace {

template <typename Solve>
void expectUnboundedSolve(Solve solve)
{
  constexpr int n = 1;
  double A[n] = {2.0};
  double x[n] = {0.0};
  double b[n] = {4.0};
  double w[n] = {0.0};
  double lo[n] = {-dInfinity};
  double hi[n] = {dInfinity};
  int findex[n] = {-1};

  EXPECT_TRUE(solve(n, A, x, b, w, n, lo, hi, findex, false));
  EXPECT_DOUBLE_EQ(2.0, x[0]);
}

} // namespace

TEST(DantzigCompat, SolvesThroughDartOwnedNamespace)
{
  expectUnboundedSolve(::dart::lcpsolver::dantzig::dSolveLCP);
}

TEST(DantzigCompat, SolvesThroughLegacyExternalNamespace)
{
  expectUnboundedSolve(::dart::external::ode::dSolveLCP);
}

TEST(DantzigCompat, LinksLegacyMatrixSymbol)
{
  double values[2] = {1.0, -2.0};

  ::dart::external::ode::dSetZero(values, 2);

  EXPECT_DOUBLE_EQ(0.0, values[0]);
  EXPECT_DOUBLE_EQ(0.0, values[1]);
}
