#include "unit/lcpsolver/DantzigProblemCases.hpp"

#include <benchmark/benchmark.h>

namespace {

void solveNative(benchmark::State& state, int caseIndex)
{
  const auto cases = dart::test::makeDantzigPerformanceCases();
  const auto& problem = cases[static_cast<std::size_t>(caseIndex)];
  dart::lcpsolver::dantzig::DantzigLcpScratch<double> scratch;
  dart::test::DantzigProblemWorkspace workspace(problem);
  double checksum = 0.0;

  for (auto _ : state) {
    const bool success = dart::test::solveDantzigNativeWorkspace(
        problem, &workspace, &scratch);
    if (!success)
      state.SkipWithError("native Dantzig solve failed");
    checksum += dart::test::solutionChecksum(workspace.x);
  }

  benchmark::DoNotOptimize(checksum);
}

void solveLegacy(benchmark::State& state, int caseIndex)
{
  const auto cases = dart::test::makeDantzigPerformanceCases();
  const auto& problem = cases[static_cast<std::size_t>(caseIndex)];
  dart::test::DantzigProblemWorkspace workspace(problem);
  double checksum = 0.0;

  for (auto _ : state) {
    const bool success
        = dart::test::solveDantzigBaselineWorkspace(problem, &workspace);
    if (!success)
      state.SkipWithError("legacy Dantzig solve failed");
    checksum += dart::test::solutionChecksum(workspace.x);
  }

  benchmark::DoNotOptimize(checksum);
}

} // namespace

BENCHMARK_CAPTURE(solveNative, unbounded_diagonal_96, 0);
BENCHMARK_CAPTURE(solveLegacy, unbounded_diagonal_96, 0);

BENCHMARK_CAPTURE(solveNative, boxed_diagonal_96, 1);
BENCHMARK_CAPTURE(solveLegacy, boxed_diagonal_96, 1);

BENCHMARK_CAPTURE(solveNative, boxed_coupled_96, 2);
BENCHMARK_CAPTURE(solveLegacy, boxed_coupled_96, 2);

BENCHMARK_CAPTURE(solveNative, friction_32, 3);
BENCHMARK_CAPTURE(solveLegacy, friction_32, 3);
