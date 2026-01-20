/*
 * Copyright (c) 2011, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/main/LICENSE
 *
 * Benchmark comparing Dantzig LCP solver vs ODE baseline
 */

#include "dart/math/lcp/pivoting/dantzig/lcp.hpp"
#include "tests/baseline/odelcpsolver/lcp.h"
#include "tests/common/lcpsolver/LCPTestProblems.hpp"

#include <benchmark/benchmark.h>

#include <vector>

using dReal = double;

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

// Helper function to prepare test data
struct LCPBenchData
{
  std::vector<dReal> A;
  std::vector<dReal> b;
  std::vector<dReal> x;
  std::vector<dReal> w;
  std::vector<dReal> lo;
  std::vector<dReal> hi;
  int n;

  LCPBenchData(const dart::test::LCPProblem& problem)
    : A(problem.dimension * problem.dimension),
      b(problem.dimension),
      x(problem.dimension, 0.0),
      w(problem.dimension, 0.0),
      lo(problem.dimension, -1e10),
      hi(problem.dimension, 1e10),
      n(problem.dimension)
  {
    // Convert Eigen matrix to row-major array
    for (int i = 0; i < n; ++i) {
      for (int j = 0; j < n; ++j) {
        A[i * n + j] = problem.A(i, j);
      }
      b[i] = problem.b(i);
    }
  }

  void reset(const dart::test::LCPProblem& problem)
  {
    for (int i = 0; i < n; ++i) {
      for (int j = 0; j < n; ++j) {
        A[i * n + j] = problem.A(i, j);
      }
      b[i] = problem.b(i);
      x[i] = 0.0;
      w[i] = 0.0;
    }
  }
};

// Benchmark Dantzig solver - double precision (F64)
static void BM_Dantzig_F64_Solver(
    benchmark::State& state, dart::test::LCPProblem problem)
{
  LCPBenchData data(problem);

  for (auto _ : state) {
    data.reset(problem);

    // Solve LCP using Dantzig solver (from dart/math/lcp/pivoting/dantzig/)
    bool success = dart::math::SolveLCP<double>(
        data.n,
        data.A.data(),
        data.x.data(),
        data.b.data(),
        data.w.data(),
        0,
        data.lo.data(),
        data.hi.data(),
        nullptr,
        false);

    benchmark::DoNotOptimize(success);
    benchmark::DoNotOptimize(data.x.data());
  }

  state.SetLabel("Dantzig-F64/" + problem.name);
}

// Benchmark Dantzig solver - float precision (F32)
static void BM_Dantzig_F32_Solver(
    benchmark::State& state, dart::test::LCPProblem problem)
{
  // Convert problem data to float
  std::vector<float> A_f(problem.dimension * problem.dimension);
  std::vector<float> b_f(problem.dimension);
  std::vector<float> x_f(problem.dimension, 0.0f);
  std::vector<float> w_f(problem.dimension, 0.0f);
  std::vector<float> lo_f(problem.dimension, -1e10f);
  std::vector<float> hi_f(problem.dimension, 1e10f);

  for (int i = 0; i < problem.dimension; ++i) {
    for (int j = 0; j < problem.dimension; ++j) {
      A_f[i * problem.dimension + j] = static_cast<float>(problem.A(i, j));
    }
    b_f[i] = static_cast<float>(problem.b(i));
  }

  for (auto _ : state) {
    // Reset solution vectors
    std::fill(x_f.begin(), x_f.end(), 0.0f);
    std::fill(w_f.begin(), w_f.end(), 0.0f);

    // Solve LCP using template version with float
    bool success = dart::math::SolveLCP<float>(
        problem.dimension,
        A_f.data(),
        x_f.data(),
        b_f.data(),
        w_f.data(),
        0,
        lo_f.data(),
        hi_f.data(),
        nullptr,
        false);

    benchmark::DoNotOptimize(success);
    benchmark::DoNotOptimize(x_f.data());
  }

  state.SetLabel("Dantzig-F32/" + problem.name);
}

// Benchmark ODE baseline solver - double precision (F64)
static void BM_ODEBase_F64_Solver(
    benchmark::State& state, dart::test::LCPProblem problem)
{
  LCPBenchData data(problem);

  for (auto _ : state) {
    data.reset(problem);

    // Solve LCP using ODE baseline (from tests/baseline/odelcpsolver/)
    bool success = dart::baseline::ode::dSolveLCP(
        data.n,
        data.A.data(),
        data.x.data(),
        data.b.data(),
        data.w.data(),
        0,
        data.lo.data(),
        data.hi.data(),
        nullptr,
        false);

    benchmark::DoNotOptimize(success);
    benchmark::DoNotOptimize(data.x.data());
  }

  state.SetLabel("ODEBase-F64/" + problem.name);
}

// Register benchmarks comparing Dantzig (F64/F32) vs ODE baseline (F64)
#define REGISTER_BENCHMARK_TRIPLE(NAME, PROBLEM)                               \
  BENCHMARK_CAPTURE(BM_Dantzig_F64_Solver, NAME, PROBLEM);                     \
  BENCHMARK_CAPTURE(BM_Dantzig_F32_Solver, NAME, PROBLEM);                     \
  BENCHMARK_CAPTURE(BM_ODEBase_F64_Solver, NAME, PROBLEM)

// Standard test problems - comparing double, float, and ODE baseline
REGISTER_BENCHMARK_TRIPLE(1D, dart::test::LCPTestProblems::getProblem1D());
REGISTER_BENCHMARK_TRIPLE(2D, dart::test::LCPTestProblems::getProblem2D());
REGISTER_BENCHMARK_TRIPLE(4D, dart::test::LCPTestProblems::getProblem4D());
REGISTER_BENCHMARK_TRIPLE(6D, dart::test::LCPTestProblems::getProblem6D());
REGISTER_BENCHMARK_TRIPLE(12D, dart::test::LCPTestProblems::getProblem12D());
REGISTER_BENCHMARK_TRIPLE(24D, dart::test::LCPTestProblems::getProblem24D());
REGISTER_BENCHMARK_TRIPLE(48D, dart::test::LCPTestProblems::getProblem48D());
