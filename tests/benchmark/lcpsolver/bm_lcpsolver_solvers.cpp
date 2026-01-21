/*
 * Copyright (c) 2011, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/main/LICENSE
 *
 * Benchmarks for the math::LcpSolver implementations (Dantzig, PGS).
 */

#include <dart/math/lcp/pivoting/dantzig_solver.hpp>
#include <dart/math/lcp/pivoting/lemke_solver.hpp>
#include <dart/math/lcp/projection/pgs_solver.hpp>

#include <Eigen/Dense>
#include <benchmark/benchmark.h>

#include <limits>
#include <random>

namespace {

using dart::math::LcpOptions;
using dart::math::LcpProblem;

LcpProblem makeStandardSpdProblem(int n, unsigned seed)
{
  std::mt19937 rng(seed);
  std::uniform_real_distribution<double> dist(-1.0, 1.0);

  Eigen::MatrixXd M(n, n);
  for (int r = 0; r < n; ++r) {
    for (int c = 0; c < n; ++c) {
      M(r, c) = dist(rng);
    }
  }

  Eigen::MatrixXd A
      = M.transpose() * M
        + static_cast<double>(n) * Eigen::MatrixXd::Identity(n, n);

  Eigen::VectorXd xStar(n);
  for (int i = 0; i < n; ++i) {
    xStar[i] = std::abs(dist(rng)) + 0.1;
  }

  Eigen::VectorXd b = A * xStar;
  Eigen::VectorXd lo = Eigen::VectorXd::Zero(n);
  Eigen::VectorXd hi
      = Eigen::VectorXd::Constant(n, std::numeric_limits<double>::infinity());
  Eigen::VectorXi findex = Eigen::VectorXi::Constant(n, -1);
  return LcpProblem(
      std::move(A),
      std::move(b),
      std::move(lo),
      std::move(hi),
      std::move(findex));
}

LcpProblem makeBoxedActiveBoundsSpdProblem(int n, unsigned seed)
{
  std::mt19937 rng(seed);
  std::uniform_real_distribution<double> dist(-1.0, 1.0);
  std::uniform_real_distribution<double> slackDist(0.1, 1.0);

  Eigen::MatrixXd M(n, n);
  for (int r = 0; r < n; ++r) {
    for (int c = 0; c < n; ++c) {
      M(r, c) = dist(rng);
    }
  }

  Eigen::MatrixXd A
      = M.transpose() * M
        + static_cast<double>(n) * Eigen::MatrixXd::Identity(n, n);

  Eigen::VectorXd lo = Eigen::VectorXd::Constant(n, -1.0);
  Eigen::VectorXd hi = Eigen::VectorXd::Constant(n, 1.0);
  Eigen::VectorXi findex = Eigen::VectorXi::Constant(n, -1);

  Eigen::VectorXd xStar(n);
  Eigen::VectorXd w = Eigen::VectorXd::Zero(n);
  for (int i = 0; i < n; ++i) {
    const int mode = i % 3;
    if (mode == 0) {
      xStar[i] = lo[i];
      w[i] = slackDist(rng);
    } else if (mode == 1) {
      xStar[i] = hi[i];
      w[i] = -slackDist(rng);
    } else {
      xStar[i] = 0.5 * dist(rng);
      w[i] = 0.0;
    }
  }

  Eigen::VectorXd b = A * xStar - w;

  return LcpProblem(
      std::move(A),
      std::move(b),
      std::move(lo),
      std::move(hi),
      std::move(findex));
}

LcpProblem makeFrictionIndexSpdProblem(int numContacts, unsigned seed)
{
  const int n = 3 * numContacts;
  std::mt19937 rng(seed);
  std::uniform_real_distribution<double> dist(-1.0, 1.0);
  std::uniform_real_distribution<double> muDist(0.2, 1.0);

  Eigen::MatrixXd M(n, n);
  for (int r = 0; r < n; ++r) {
    for (int c = 0; c < n; ++c) {
      M(r, c) = dist(rng);
    }
  }

  Eigen::MatrixXd A
      = M.transpose() * M
        + static_cast<double>(n) * Eigen::MatrixXd::Identity(n, n);

  Eigen::VectorXd xStar = Eigen::VectorXd::Zero(n);
  Eigen::VectorXd lo = Eigen::VectorXd::Zero(n);
  Eigen::VectorXd hi = Eigen::VectorXd::Zero(n);
  Eigen::VectorXi findex = Eigen::VectorXi::Constant(n, -1);

  for (int c = 0; c < numContacts; ++c) {
    const int base = 3 * c;
    const double normal = std::abs(dist(rng)) + 0.5;
    const double mu = muDist(rng);

    xStar[base + 0] = normal;
    xStar[base + 1] = 0.5 * mu * normal;
    xStar[base + 2] = -0.25 * mu * normal;

    lo[base + 0] = 0.0;
    hi[base + 0] = std::numeric_limits<double>::infinity();

    lo[base + 1] = -mu;
    hi[base + 1] = mu;
    findex[base + 1] = base + 0;

    lo[base + 2] = -mu;
    hi[base + 2] = mu;
    findex[base + 2] = base + 0;
  }

  Eigen::VectorXd b = A * xStar;
  return LcpProblem(
      std::move(A),
      std::move(b),
      std::move(lo),
      std::move(hi),
      std::move(findex));
}

static void BM_DantzigSolver_Standard(benchmark::State& state)
{
  const int n = static_cast<int>(state.range(0));
  auto problem
      = makeStandardSpdProblem(n, /*seed=*/42u + static_cast<unsigned>(n));

  dart::math::DantzigSolver solver;
  LcpOptions options = solver.getDefaultOptions();
  options.warmStart = false;
  options.validateSolution = false;

  Eigen::VectorXd x = Eigen::VectorXd::Zero(n);

  for (auto _ : state) {
    x.setZero();
    auto result = solver.solve(problem, x, options);
    benchmark::DoNotOptimize(result.status);
    benchmark::DoNotOptimize(x.data());
  }
}

static void BM_PgsSolver_Standard(benchmark::State& state)
{
  const int n = static_cast<int>(state.range(0));
  auto problem
      = makeStandardSpdProblem(n, /*seed=*/84u + static_cast<unsigned>(n));

  dart::math::PgsSolver solver;
  LcpOptions options = solver.getDefaultOptions();
  options.warmStart = false;
  options.validateSolution = false;
  options.maxIterations = static_cast<int>(state.range(1));

  Eigen::VectorXd x = Eigen::VectorXd::Zero(n);

  for (auto _ : state) {
    x.setZero();
    auto result = solver.solve(problem, x, options);
    benchmark::DoNotOptimize(result.status);
    benchmark::DoNotOptimize(x.data());
  }
}

static void BM_PgsSolver_Standard_Relax15(benchmark::State& state)
{
  const int n = static_cast<int>(state.range(0));
  auto problem
      = makeStandardSpdProblem(n, /*seed=*/148u + static_cast<unsigned>(n));

  dart::math::PgsSolver solver;
  LcpOptions options = solver.getDefaultOptions();
  options.warmStart = false;
  options.validateSolution = false;
  options.maxIterations = static_cast<int>(state.range(1));
  options.relaxation = 1.5;

  Eigen::VectorXd x = Eigen::VectorXd::Zero(n);

  for (auto _ : state) {
    x.setZero();
    auto result = solver.solve(problem, x, options);
    benchmark::DoNotOptimize(result.status);
    benchmark::DoNotOptimize(x.data());
  }
}

static void BM_LemkeSolver_Standard(benchmark::State& state)
{
  const int n = static_cast<int>(state.range(0));
  auto problem
      = makeStandardSpdProblem(n, /*seed=*/314u + static_cast<unsigned>(n));

  dart::math::LemkeSolver solver;
  LcpOptions options = solver.getDefaultOptions();
  options.warmStart = false;
  options.validateSolution = false;

  Eigen::VectorXd x = Eigen::VectorXd::Zero(n);

  for (auto _ : state) {
    x.setZero();
    auto result = solver.solve(problem, x, options);
    benchmark::DoNotOptimize(result.status);
    benchmark::DoNotOptimize(x.data());
  }
}

static void BM_DantzigSolver_BoxedActiveBounds(benchmark::State& state)
{
  const int n = static_cast<int>(state.range(0));
  auto problem = makeBoxedActiveBoundsSpdProblem(
      n, /*seed=*/900u + static_cast<unsigned>(n));

  dart::math::DantzigSolver solver;
  LcpOptions options = solver.getDefaultOptions();
  options.warmStart = false;
  options.validateSolution = false;

  Eigen::VectorXd x = Eigen::VectorXd::Zero(n);

  for (auto _ : state) {
    x.setZero();
    auto result = solver.solve(problem, x, options);
    benchmark::DoNotOptimize(result.status);
    benchmark::DoNotOptimize(x.data());
  }
}

static void BM_PgsSolver_BoxedActiveBounds(benchmark::State& state)
{
  const int n = static_cast<int>(state.range(0));
  auto problem = makeBoxedActiveBoundsSpdProblem(
      n, /*seed=*/901u + static_cast<unsigned>(n));

  dart::math::PgsSolver solver;
  LcpOptions options = solver.getDefaultOptions();
  options.warmStart = false;
  options.validateSolution = false;
  options.maxIterations = static_cast<int>(state.range(1));

  Eigen::VectorXd x = Eigen::VectorXd::Zero(n);

  for (auto _ : state) {
    x.setZero();
    auto result = solver.solve(problem, x, options);
    benchmark::DoNotOptimize(result.status);
    benchmark::DoNotOptimize(x.data());
  }
}

static void BM_DantzigSolver_FrictionIndex(benchmark::State& state)
{
  const int numContacts = static_cast<int>(state.range(0));
  const int n = 3 * numContacts;
  auto problem = makeFrictionIndexSpdProblem(
      numContacts, /*seed=*/1337u + static_cast<unsigned>(n));

  dart::math::DantzigSolver solver;
  LcpOptions options = solver.getDefaultOptions();
  options.warmStart = false;
  options.validateSolution = false;

  Eigen::VectorXd x = Eigen::VectorXd::Zero(n);

  for (auto _ : state) {
    x.setZero();
    auto result = solver.solve(problem, x, options);
    benchmark::DoNotOptimize(result.status);
    benchmark::DoNotOptimize(x.data());
  }
}

static void BM_PgsSolver_FrictionIndex(benchmark::State& state)
{
  const int numContacts = static_cast<int>(state.range(0));
  const int n = 3 * numContacts;
  auto problem = makeFrictionIndexSpdProblem(
      numContacts, /*seed=*/4242u + static_cast<unsigned>(n));

  dart::math::PgsSolver solver;
  LcpOptions options = solver.getDefaultOptions();
  options.warmStart = false;
  options.validateSolution = false;
  options.maxIterations = static_cast<int>(state.range(1));

  Eigen::VectorXd x = Eigen::VectorXd::Zero(n);

  for (auto _ : state) {
    x.setZero();
    auto result = solver.solve(problem, x, options);
    benchmark::DoNotOptimize(result.status);
    benchmark::DoNotOptimize(x.data());
  }
}

} // namespace

BENCHMARK(BM_DantzigSolver_Standard)->Arg(12)->Arg(24)->Arg(48)->Arg(96);
BENCHMARK(BM_PgsSolver_Standard)
    ->Args({12, 30})
    ->Args({24, 30})
    ->Args({48, 30})
    ->Args({96, 30});
BENCHMARK(BM_PgsSolver_Standard)
    ->Args({12, 100})
    ->Args({24, 100})
    ->Args({48, 100})
    ->Args({96, 100});
BENCHMARK(BM_PgsSolver_Standard_Relax15)
    ->Args({12, 30})
    ->Args({24, 30})
    ->Args({48, 30})
    ->Args({96, 30});

BENCHMARK(BM_LemkeSolver_Standard)->Arg(12)->Arg(24)->Arg(48)->Arg(96);

BENCHMARK(BM_DantzigSolver_BoxedActiveBounds)
    ->Arg(12)
    ->Arg(24)
    ->Arg(48)
    ->Arg(96);
BENCHMARK(BM_PgsSolver_BoxedActiveBounds)
    ->Args({12, 30})
    ->Args({24, 30})
    ->Args({48, 30})
    ->Args({96, 30});
BENCHMARK(BM_PgsSolver_BoxedActiveBounds)
    ->Args({12, 100})
    ->Args({24, 100})
    ->Args({48, 100})
    ->Args({96, 100});

BENCHMARK(BM_DantzigSolver_FrictionIndex)->Arg(4)->Arg(16)->Arg(64);
BENCHMARK(BM_PgsSolver_FrictionIndex)
    ->Args({4, 30})
    ->Args({16, 30})
    ->Args({64, 30});
BENCHMARK(BM_PgsSolver_FrictionIndex)
    ->Args({4, 100})
    ->Args({16, 100})
    ->Args({64, 100});
