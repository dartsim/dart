#include "dart/external/odelcpsolver/lcp.h"
#include "dart/lcpsolver/dantzig/lcp.h"

#include <gtest/gtest.h>

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
