#define dInfinity HUGE_VAL
#define dPAD(a) (((a) > 1) ? ((((a)-1) | 3) + 1) : (a))

#include "dart/lcpsolver/dantzig/DantzigCommon.hpp"

#include <gtest/gtest.h>

#include <cmath>

TEST(DantzigHeaderCompatibility, CoexistsWithOdeStyleMacros)
{
  EXPECT_EQ(4, ::dart::lcpsolver::dantzig::padding(3));
  EXPECT_TRUE(std::isinf(::dart::lcpsolver::dantzig::kInfinity));
  EXPECT_EQ(4, dPAD(3));
  EXPECT_TRUE(std::isinf(dInfinity));
}
