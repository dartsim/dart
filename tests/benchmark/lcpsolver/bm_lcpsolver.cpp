/*
 * Copyright (c) 2011-2025, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/main/LICENSE
 *
 * Benchmark comparing Dantzig LCP solver vs ODE baseline
 */

#include "LCPTestProblems.hpp"

#include "dart/lcpsolver/dantzig/lcp.h"
#include "tests/baseline/odelcpsolver/lcp.h"

#include <benchmark/benchmark.h>

#include <vector>

using dart::lcpsolver::dReal;

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

// Benchmark Dantzig solver
static void BM_Dantzig(benchmark::State& state, dart::test::LCPProblem problem)
{
  LCPBenchData data(problem);

  for (auto _ : state) {
    data.reset(problem);

    // Solve LCP using Dantzig solver (from dart/lcpsolver/dantzig/)
    bool success = dart::lcpsolver::dSolveLCP(
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

  state.SetLabel("Dantzig/" + problem.name);
}

// Benchmark ODE baseline solver
static void BM_ODE_Baseline(benchmark::State& state, dart::test::LCPProblem problem)
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

  state.SetLabel("ODE/" + problem.name);
}

// Register benchmarks for all problem sizes with both solvers
#define REGISTER_BENCHMARK_PAIR(NAME, PROBLEM) \
  BENCHMARK_CAPTURE(BM_Dantzig, NAME, PROBLEM); \
  BENCHMARK_CAPTURE(BM_ODE_Baseline, NAME, PROBLEM)

REGISTER_BENCHMARK_PAIR(1D, dart::test::LCPTestProblems::getProblem1D());
REGISTER_BENCHMARK_PAIR(2D, dart::test::LCPTestProblems::getProblem2D());
REGISTER_BENCHMARK_PAIR(4D, dart::test::LCPTestProblems::getProblem4D());
REGISTER_BENCHMARK_PAIR(6D, dart::test::LCPTestProblems::getProblem6D());
REGISTER_BENCHMARK_PAIR(12D, dart::test::LCPTestProblems::getProblem12D());
REGISTER_BENCHMARK_PAIR(24D, dart::test::LCPTestProblems::getProblem24D());
REGISTER_BENCHMARK_PAIR(48D, dart::test::LCPTestProblems::getProblem48D());
