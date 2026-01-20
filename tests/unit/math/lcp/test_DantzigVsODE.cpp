/*
 * Copyright (c) 2011, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/main/LICENSE
 *
 * This file is provided under the following "BSD-style" License:
 *   Redistribution and use in source and binary forms, with or
 *   without modification, are permitted provided that the following
 *   conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 *   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 */

#include "dart/math/lcp/pivoting/dantzig/common.hpp"
#include "dart/math/lcp/pivoting/dantzig/lcp.hpp"

// Undefine assertion macros from dantzig to avoid conflicts with baseline
#undef dIASSERT
#undef dUASSERT
#undef dDEBUGMSG
#undef dICHECK
#undef dAASSERT
#undef dIVERIFY
#undef D_ALL_PARAM_NAMES_X

#include "tests/baseline/odelcpsolver/lcp.h"
#include "tests/common/lcpsolver/LCPTestProblems.hpp"

#include <gtest/gtest.h>

#include <vector>

#include <cmath>

// Types from both implementations
using dReal = double; // Compatibility alias for tests
                      // dart/math/lcp/pivoting/dantzig/)

// Forward declare the baseline function
namespace dart {
namespace baseline {
namespace ode {
extern bool dSolveLCP(
    int n,
    dReal* A,
    dReal* x,
    dReal* b,
    dReal* w,
    int nub,
    dReal* lo,
    dReal* hi,
    int* findex,
    bool earlyTermination);
}
} // namespace baseline
} // namespace dart

namespace {

// Helper function to verify complementarity conditions
// For LCP: A*x = b+w, where x >= lo, w >= 0 (for unbounded case), and x'*w = 0
bool verifyComplementarity(
    const std::vector<dReal>& A,
    const std::vector<dReal>& x,
    const std::vector<dReal>& b,
    const std::vector<dReal>& w,
    int n,
    double tolerance = 1e-6)
{
  // Check A*x = b+w
  for (int i = 0; i < n; ++i) {
    double lhs = 0.0;
    for (int j = 0; j < n; ++j) {
      lhs += A[i * n + j] * x[j];
    }
    double rhs = b[i] + w[i];
    if (std::abs(lhs - rhs) > tolerance) {
      return false;
    }
  }

  // Check complementarity: x[i]*w[i] ≈ 0 for all i
  for (int i = 0; i < n; ++i) {
    if (std::abs(x[i] * w[i]) > tolerance) {
      return false;
    }
  }

  return true;
}

// Helper function to compare two solutions
bool solutionsMatch(
    const std::vector<dReal>& x1,
    const std::vector<dReal>& x2,
    double tolerance = 1e-5)
{
  if (x1.size() != x2.size())
    return false;

  for (size_t i = 0; i < x1.size(); ++i) {
    if (std::abs(x1[i] - x2[i]) > tolerance) {
      return false;
    }
  }
  return true;
}

// Test helper that runs both solvers and compares results
void testDantzigVsODE(dart::test::LCPProblem problem)
{
  const int n = problem.dimension;
  const int stride = dart::math::padding(n);

  // Prepare data for ODE baseline
  std::vector<dReal> A_ode(stride * n, 0.0);
  std::vector<dReal> b_ode(n);
  std::vector<dReal> x_ode(n, 0.0);
  std::vector<dReal> w_ode(n, 0.0);
  std::vector<dReal> lo_ode(n, 0.0); // Standard LCP: x >= 0
  std::vector<dReal> hi_ode(n, 1e10);

  // Prepare data for Dantzig solver
  std::vector<dReal> A_dantzig(stride * n, 0.0);
  std::vector<dReal> b_dantzig(n);
  std::vector<dReal> x_dantzig(n, 0.0);
  std::vector<dReal> w_dantzig(n, 0.0);
  std::vector<dReal> lo_dantzig(n, 0.0); // Standard LCP: x >= 0
  std::vector<dReal> hi_dantzig(n, 1e10);
  std::vector<dReal> A_dense(n * n, 0.0);

  // Copy data
  for (int i = 0; i < n; ++i) {
    for (int j = 0; j < n; ++j) {
      const dReal value = problem.A(i, j);
      A_dense[i * n + j] = value;
      A_ode[i * stride + j] = value;
      A_dantzig[i * stride + j] = value;
    }
    b_ode[i] = problem.b(i);
    b_dantzig[i] = problem.b(i);
  }

  // Solve with ODE baseline (from tests/baseline/odelcpsolver/)
  bool success_ode = dart::baseline::ode::dSolveLCP(
      n,
      A_ode.data(),
      x_ode.data(),
      b_ode.data(),
      w_ode.data(),
      0,
      lo_ode.data(),
      hi_ode.data(),
      nullptr,
      false);

  // Solve with Dantzig solver (from dart/math/lcp/pivoting/dantzig/)
  bool success_dantzig = dart::math::SolveLCP<double>(
      n,
      A_dantzig.data(),
      x_dantzig.data(),
      b_dantzig.data(),
      w_dantzig.data(),
      0,
      lo_dantzig.data(),
      hi_dantzig.data(),
      nullptr,
      false);

  // Both should succeed
  EXPECT_TRUE(success_ode) << "ODE baseline failed for " << problem.name;
  EXPECT_TRUE(success_dantzig) << "Dantzig solver failed for " << problem.name;

  if (success_ode && success_dantzig) {
    // Verify both solutions satisfy complementarity conditions
    EXPECT_TRUE(verifyComplementarity(A_dense, x_ode, b_ode, w_ode, n))
        << "ODE solution violates complementarity for " << problem.name;
    EXPECT_TRUE(
        verifyComplementarity(A_dense, x_dantzig, b_dantzig, w_dantzig, n))
        << "Dantzig solution violates complementarity for " << problem.name;

    // Solutions should match (both should find the same solution)
    EXPECT_TRUE(solutionsMatch(x_ode, x_dantzig))
        << "Solutions don't match for " << problem.name;
  }
}

} // anonymous namespace

//==============================================================================
TEST(DantzigVsODE, Problem1D)
{
  testDantzigVsODE(dart::test::LCPTestProblems::getProblem1D());
}

//==============================================================================
TEST(DantzigVsODE, Problem2D)
{
  testDantzigVsODE(dart::test::LCPTestProblems::getProblem2D());
}

//==============================================================================
TEST(DantzigVsODE, Problem4D)
{
  testDantzigVsODE(dart::test::LCPTestProblems::getProblem4D());
}

//==============================================================================
TEST(DantzigVsODE, Problem6D)
{
  testDantzigVsODE(dart::test::LCPTestProblems::getProblem6D());
}

//==============================================================================
TEST(DantzigVsODE, Problem12D)
{
  testDantzigVsODE(dart::test::LCPTestProblems::getProblem12D());
}

//==============================================================================
TEST(DantzigVsODE, Problem24D)
{
  testDantzigVsODE(dart::test::LCPTestProblems::getProblem24D());
}

//==============================================================================
TEST(DantzigVsODE, Problem48D)
{
  testDantzigVsODE(dart::test::LCPTestProblems::getProblem48D());
}
