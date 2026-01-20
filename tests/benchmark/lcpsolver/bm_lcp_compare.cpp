/*
 * Copyright (c) 2011, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/main/LICENSE
 *
 * Solver-agnostic benchmark harness for LCP comparisons.
 */

#include "tests/common/lcpsolver/lcp_test_harness.hpp"

#include <dart/math/lcp/newton/fischer_burmeister_newton_solver.hpp>
#include <dart/math/lcp/newton/minimum_map_newton_solver.hpp>
#include <dart/math/lcp/newton/penalized_fischer_burmeister_newton_solver.hpp>
#include <dart/math/lcp/other/interior_point_solver.hpp>
#include <dart/math/lcp/other/mprgp_solver.hpp>
#include <dart/math/lcp/other/shock_propagation_solver.hpp>
#include <dart/math/lcp/other/staggering_solver.hpp>
#include <dart/math/lcp/pivoting/baraff_solver.hpp>
#include <dart/math/lcp/pivoting/dantzig_solver.hpp>
#include <dart/math/lcp/pivoting/direct_solver.hpp>
#include <dart/math/lcp/pivoting/lemke_solver.hpp>
#include <dart/math/lcp/projection/bgs_solver.hpp>
#include <dart/math/lcp/projection/blocked_jacobi_solver.hpp>
#include <dart/math/lcp/projection/jacobi_solver.hpp>
#include <dart/math/lcp/projection/nncg_solver.hpp>
#include <dart/math/lcp/projection/pgs_solver.hpp>
#include <dart/math/lcp/projection/red_black_gauss_seidel_solver.hpp>
#include <dart/math/lcp/projection/subspace_minimization_solver.hpp>
#include <dart/math/lcp/projection/symmetric_psor_solver.hpp>

#include <Eigen/Dense>
#include <benchmark/benchmark.h>

#include <limits>
#include <random>
#include <sstream>
#include <string>

namespace {

using dart::math::LcpOptions;
using dart::math::LcpProblem;

LcpOptions MakeBenchmarkOptions(int maxIterations)
{
  LcpOptions options;
  options.maxIterations = maxIterations;
  options.absoluteTolerance = 1e-6;
  options.relativeTolerance = 1e-4;
  options.complementarityTolerance = 1e-6;
  options.relaxation = 1.0;
  options.warmStart = false;
  options.validateSolution = false;
  options.earlyTermination = false;
  return options;
}

LcpProblem MakeStandardSpdProblem(int n, unsigned seed)
{
  std::mt19937 rng(seed);
  std::uniform_real_distribution<double> dist(-1.0, 1.0);

  Eigen::MatrixXd M(n, n);
  for (int r = 0; r < n; ++r) {
    for (int c = 0; c < n; ++c)
      M(r, c) = dist(rng);
  }

  Eigen::MatrixXd A
      = M.transpose() * M
        + static_cast<double>(n) * Eigen::MatrixXd::Identity(n, n);

  Eigen::VectorXd xStar(n);
  for (int i = 0; i < n; ++i)
    xStar[i] = std::abs(dist(rng)) + 0.1;

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

LcpProblem MakeBoxedActiveBoundsProblem(int n, unsigned seed)
{
  std::mt19937 rng(seed);
  std::uniform_real_distribution<double> dist(-1.0, 1.0);
  std::uniform_real_distribution<double> slackDist(0.1, 1.0);

  Eigen::MatrixXd M(n, n);
  for (int r = 0; r < n; ++r) {
    for (int c = 0; c < n; ++c)
      M(r, c) = dist(rng);
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

LcpProblem MakeFrictionIndexProblem(int numContacts, unsigned seed)
{
  const int n = 3 * numContacts;
  std::mt19937 rng(seed);
  std::uniform_real_distribution<double> dist(-1.0, 1.0);
  std::uniform_real_distribution<double> muDist(0.2, 1.0);

  Eigen::MatrixXd M(n, n);
  for (int r = 0; r < n; ++r) {
    for (int c = 0; c < n; ++c)
      M(r, c) = dist(rng);
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

LcpProblem MakeScaledProblem(int n, double scale, unsigned seed)
{
  auto problem = MakeStandardSpdProblem(n, seed);
  problem.A *= scale;
  problem.b *= scale;
  return problem;
}

std::string MakeLabel(const std::string& solver, const std::string& category)
{
  std::ostringstream out;
  out << solver << "/" << category;
  return out.str();
}

void AddShockPropagationCounters(
    benchmark::State& state,
    const dart::math::ShockPropagationSolver::Parameters& params)
{
  const int blockCount = static_cast<int>(params.blockSizes.size());
  int layerCount = static_cast<int>(params.layers.size());
  if (layerCount == 0 && blockCount > 0)
    layerCount = 1;

  int maxBlockSize = 0;
  for (const int size : params.blockSizes)
    maxBlockSize = std::max(maxBlockSize, size);

  int maxBlocksPerLayer = 0;
  if (!params.layers.empty()) {
    for (const auto& layer : params.layers)
      maxBlocksPerLayer
          = std::max(maxBlocksPerLayer, static_cast<int>(layer.size()));
  } else {
    maxBlocksPerLayer = blockCount;
  }

  state.counters["layer_count"] = layerCount;
  state.counters["block_count"] = blockCount;
  state.counters["max_block_size"] = maxBlockSize;
  state.counters["max_blocks_per_layer"] = maxBlocksPerLayer;
}

template <typename Solver>
void RunBenchmark(
    benchmark::State& state,
    const LcpProblem& problem,
    const LcpOptions& options,
    const std::string& label)
{
  Solver solver;
  Eigen::VectorXd x = Eigen::VectorXd::Zero(problem.b.size());
  dart::math::LcpResult lastResult;

  for (auto _ : state) {
    x.setZero();
    lastResult = solver.solve(problem, x, options);
    benchmark::DoNotOptimize(lastResult.status);
    benchmark::DoNotOptimize(x.data());
  }

  const auto check = dart::test::CheckLcpSolution(problem, x, options);
  state.counters["iterations"] = lastResult.iterations;
  state.counters["residual"] = check.residual;
  state.counters["complementarity"] = check.complementarity;
  state.counters["bound_violation"] = check.boundViolation;
  state.counters["contract_ok"] = check.ok ? 1.0 : 0.0;
  state.SetLabel(label);
}

static void BM_LcpCompare_Dantzig_Standard(benchmark::State& state)
{
  const int n = static_cast<int>(state.range(0));
  const auto problem
      = MakeStandardSpdProblem(n, 42u + static_cast<unsigned>(n));
  const auto options = MakeBenchmarkOptions(100);
  RunBenchmark<dart::math::DantzigSolver>(
      state, problem, options, MakeLabel("Dantzig", "Standard"));
}

static void BM_LcpCompare_Baraff_Standard(benchmark::State& state)
{
  const int n = static_cast<int>(state.range(0));
  const auto problem
      = MakeStandardSpdProblem(n, 144u + static_cast<unsigned>(n));
  const auto options = MakeBenchmarkOptions(100);
  RunBenchmark<dart::math::BaraffSolver>(
      state, problem, options, MakeLabel("Baraff", "Standard"));
}

static void BM_LcpCompare_Direct_Standard(benchmark::State& state)
{
  const int n = static_cast<int>(state.range(0));
  const auto problem
      = MakeStandardSpdProblem(n, 154u + static_cast<unsigned>(n));
  const auto options = MakeBenchmarkOptions(1);
  RunBenchmark<dart::math::DirectSolver>(
      state, problem, options, MakeLabel("Direct", "Standard"));
}

static void BM_LcpCompare_InteriorPoint_Standard(benchmark::State& state)
{
  const int n = static_cast<int>(state.range(0));
  const auto problem
      = MakeStandardSpdProblem(n, 164u + static_cast<unsigned>(n));
  const auto options = MakeBenchmarkOptions(50);
  RunBenchmark<dart::math::InteriorPointSolver>(
      state, problem, options, MakeLabel("InteriorPoint", "Standard"));
}

static void BM_LcpCompare_Mprgp_Standard(benchmark::State& state)
{
  const int n = static_cast<int>(state.range(0));
  const auto problem
      = MakeStandardSpdProblem(n, 174u + static_cast<unsigned>(n));
  const auto options = MakeBenchmarkOptions(200);
  RunBenchmark<dart::math::MprgpSolver>(
      state, problem, options, MakeLabel("MPRGP", "Standard"));
}

static void BM_LcpCompare_ShockPropagation_Standard(benchmark::State& state)
{
  const int n = static_cast<int>(state.range(0));
  const auto problem
      = MakeStandardSpdProblem(n, 194u + static_cast<unsigned>(n));

  dart::math::ShockPropagationSolver::Parameters params;
  params.blockSizes.clear();
  const int blockSize = 3;
  int remaining = n;
  while (remaining > 0) {
    const int size = std::min(blockSize, remaining);
    params.blockSizes.push_back(size);
    remaining -= size;
  }

  params.layers.clear();
  params.layers.reserve(params.blockSizes.size());
  for (int i = 0; i < static_cast<int>(params.blockSizes.size()); ++i)
    params.layers.push_back({i});

  auto options = MakeBenchmarkOptions(100);
  options.customOptions = &params;
  RunBenchmark<dart::math::ShockPropagationSolver>(
      state, problem, options, MakeLabel("ShockPropagation", "Standard"));
  AddShockPropagationCounters(state, params);
}

static void BM_LcpCompare_BlockedJacobi_Standard(benchmark::State& state)
{
  const int n = static_cast<int>(state.range(0));
  const auto problem
      = MakeStandardSpdProblem(n, 184u + static_cast<unsigned>(n));
  const auto options = MakeBenchmarkOptions(200);
  RunBenchmark<dart::math::BlockedJacobiSolver>(
      state, problem, options, MakeLabel("BlockedJacobi", "Standard"));
}

static void BM_LcpCompare_Pgs_Standard(benchmark::State& state)
{
  const int n = static_cast<int>(state.range(0));
  const auto problem
      = MakeStandardSpdProblem(n, 84u + static_cast<unsigned>(n));
  const auto options = MakeBenchmarkOptions(100);
  RunBenchmark<dart::math::PgsSolver>(
      state, problem, options, MakeLabel("Pgs", "Standard"));
}

static void BM_LcpCompare_Jacobi_Standard(benchmark::State& state)
{
  const int n = static_cast<int>(state.range(0));
  const auto problem
      = MakeStandardSpdProblem(n, 94u + static_cast<unsigned>(n));
  const auto options = MakeBenchmarkOptions(200);
  RunBenchmark<dart::math::JacobiSolver>(
      state, problem, options, MakeLabel("Jacobi", "Standard"));
}

static void BM_LcpCompare_SymmetricPsor_Standard(benchmark::State& state)
{
  const int n = static_cast<int>(state.range(0));
  const auto problem
      = MakeStandardSpdProblem(n, 104u + static_cast<unsigned>(n));
  const auto options = MakeBenchmarkOptions(100);
  RunBenchmark<dart::math::SymmetricPsorSolver>(
      state, problem, options, MakeLabel("SymmetricPsor", "Standard"));
}

static void BM_LcpCompare_RedBlackGaussSeidel_Standard(benchmark::State& state)
{
  const int n = static_cast<int>(state.range(0));
  const auto problem
      = MakeStandardSpdProblem(n, 114u + static_cast<unsigned>(n));
  const auto options = MakeBenchmarkOptions(100);
  RunBenchmark<dart::math::RedBlackGaussSeidelSolver>(
      state, problem, options, MakeLabel("RedBlackGS", "Standard"));
}

static void BM_LcpCompare_Staggering_Standard(benchmark::State& state)
{
  const int n = static_cast<int>(state.range(0));
  const auto problem
      = MakeStandardSpdProblem(n, 118u + static_cast<unsigned>(n));
  const auto options = MakeBenchmarkOptions(100);
  RunBenchmark<dart::math::StaggeringSolver>(
      state, problem, options, MakeLabel("Staggering", "Standard"));
}

static void BM_LcpCompare_Bgs_Standard(benchmark::State& state)
{
  const int n = static_cast<int>(state.range(0));
  const auto problem
      = MakeStandardSpdProblem(n, 124u + static_cast<unsigned>(n));
  const auto options = MakeBenchmarkOptions(200);
  RunBenchmark<dart::math::BgsSolver>(
      state, problem, options, MakeLabel("Bgs", "Standard"));
}

static void BM_LcpCompare_Nncg_Standard(benchmark::State& state)
{
  const int n = static_cast<int>(state.range(0));
  const auto problem
      = MakeStandardSpdProblem(n, 134u + static_cast<unsigned>(n));
  auto options = MakeBenchmarkOptions(200);
  dart::math::NncgSolver::Parameters params;
  params.pgsIterations = 1;
  params.restartInterval = 10;
  params.restartThreshold = 1.0;
  options.customOptions = &params;
  RunBenchmark<dart::math::NncgSolver>(
      state, problem, options, MakeLabel("Nncg", "Standard"));
}

static void BM_LcpCompare_SubspaceMinimization_Standard(benchmark::State& state)
{
  const int n = static_cast<int>(state.range(0));
  const auto problem
      = MakeStandardSpdProblem(n, 204u + static_cast<unsigned>(n));
  auto options = MakeBenchmarkOptions(200);
  dart::math::SubspaceMinimizationSolver::Parameters params;
  params.pgsIterations = 5;
  options.customOptions = &params;
  RunBenchmark<dart::math::SubspaceMinimizationSolver>(
      state, problem, options, MakeLabel("SubspaceMinimization", "Standard"));
}

static void BM_LcpCompare_Lemke_Standard(benchmark::State& state)
{
  const int n = static_cast<int>(state.range(0));
  const auto problem
      = MakeStandardSpdProblem(n, 314u + static_cast<unsigned>(n));
  const auto options = MakeBenchmarkOptions(100);
  RunBenchmark<dart::math::LemkeSolver>(
      state, problem, options, MakeLabel("Lemke", "Standard"));
}

static void BM_LcpCompare_MinimumMapNewton_Standard(benchmark::State& state)
{
  const int n = static_cast<int>(state.range(0));
  const auto problem
      = MakeStandardSpdProblem(n, 515u + static_cast<unsigned>(n));
  const auto options = MakeBenchmarkOptions(50);
  RunBenchmark<dart::math::MinimumMapNewtonSolver>(
      state, problem, options, MakeLabel("MinimumMapNewton", "Standard"));
}

static void BM_LcpCompare_FischerBurmeisterNewton_Standard(
    benchmark::State& state)
{
  const int n = static_cast<int>(state.range(0));
  const auto problem
      = MakeStandardSpdProblem(n, 616u + static_cast<unsigned>(n));
  const auto options = MakeBenchmarkOptions(50);
  RunBenchmark<dart::math::FischerBurmeisterNewtonSolver>(
      state,
      problem,
      options,
      MakeLabel("FischerBurmeisterNewton", "Standard"));
}

static void BM_LcpCompare_PenalizedFischerBurmeisterNewton_Standard(
    benchmark::State& state)
{
  const int n = static_cast<int>(state.range(0));
  const auto problem
      = MakeStandardSpdProblem(n, 717u + static_cast<unsigned>(n));
  auto options = MakeBenchmarkOptions(50);
  dart::math::PenalizedFischerBurmeisterNewtonSolver::Parameters params;
  params.lambda = 1.0;
  options.customOptions = &params;
  RunBenchmark<dart::math::PenalizedFischerBurmeisterNewtonSolver>(
      state,
      problem,
      options,
      MakeLabel("PenalizedFischerBurmeisterNewton", "Standard"));
}

static void BM_LcpCompare_Dantzig_Boxed(benchmark::State& state)
{
  const int n = static_cast<int>(state.range(0));
  const auto problem
      = MakeBoxedActiveBoundsProblem(n, 900u + static_cast<unsigned>(n));
  const auto options = MakeBenchmarkOptions(100);
  RunBenchmark<dart::math::DantzigSolver>(
      state, problem, options, MakeLabel("Dantzig", "Boxed"));
}

static void BM_LcpCompare_Pgs_Boxed(benchmark::State& state)
{
  const int n = static_cast<int>(state.range(0));
  const auto problem
      = MakeBoxedActiveBoundsProblem(n, 901u + static_cast<unsigned>(n));
  const auto options = MakeBenchmarkOptions(100);
  RunBenchmark<dart::math::PgsSolver>(
      state, problem, options, MakeLabel("Pgs", "Boxed"));
}

static void BM_LcpCompare_Jacobi_Boxed(benchmark::State& state)
{
  const int n = static_cast<int>(state.range(0));
  const auto problem
      = MakeBoxedActiveBoundsProblem(n, 905u + static_cast<unsigned>(n));
  const auto options = MakeBenchmarkOptions(200);
  RunBenchmark<dart::math::JacobiSolver>(
      state, problem, options, MakeLabel("Jacobi", "Boxed"));
}

static void BM_LcpCompare_SymmetricPsor_Boxed(benchmark::State& state)
{
  const int n = static_cast<int>(state.range(0));
  const auto problem
      = MakeBoxedActiveBoundsProblem(n, 906u + static_cast<unsigned>(n));
  const auto options = MakeBenchmarkOptions(100);
  RunBenchmark<dart::math::SymmetricPsorSolver>(
      state, problem, options, MakeLabel("SymmetricPsor", "Boxed"));
}

static void BM_LcpCompare_RedBlackGaussSeidel_Boxed(benchmark::State& state)
{
  const int n = static_cast<int>(state.range(0));
  const auto problem
      = MakeBoxedActiveBoundsProblem(n, 907u + static_cast<unsigned>(n));
  const auto options = MakeBenchmarkOptions(100);
  RunBenchmark<dart::math::RedBlackGaussSeidelSolver>(
      state, problem, options, MakeLabel("RedBlackGS", "Boxed"));
}

static void BM_LcpCompare_Staggering_Boxed(benchmark::State& state)
{
  const int n = static_cast<int>(state.range(0));
  const auto problem
      = MakeBoxedActiveBoundsProblem(n, 908u + static_cast<unsigned>(n));
  const auto options = MakeBenchmarkOptions(100);
  RunBenchmark<dart::math::StaggeringSolver>(
      state, problem, options, MakeLabel("Staggering", "Boxed"));
}

static void BM_LcpCompare_Bgs_Boxed(benchmark::State& state)
{
  const int n = static_cast<int>(state.range(0));
  const auto problem
      = MakeBoxedActiveBoundsProblem(n, 903u + static_cast<unsigned>(n));
  const auto options = MakeBenchmarkOptions(200);
  RunBenchmark<dart::math::BgsSolver>(
      state, problem, options, MakeLabel("Bgs", "Boxed"));
}

static void BM_LcpCompare_BlockedJacobi_Boxed(benchmark::State& state)
{
  const int n = static_cast<int>(state.range(0));
  const auto problem
      = MakeBoxedActiveBoundsProblem(n, 909u + static_cast<unsigned>(n));
  const auto options = MakeBenchmarkOptions(200);
  RunBenchmark<dart::math::BlockedJacobiSolver>(
      state, problem, options, MakeLabel("BlockedJacobi", "Boxed"));
}

static void BM_LcpCompare_Nncg_Boxed(benchmark::State& state)
{
  const int n = static_cast<int>(state.range(0));
  const auto problem
      = MakeBoxedActiveBoundsProblem(n, 902u + static_cast<unsigned>(n));
  auto options = MakeBenchmarkOptions(200);
  dart::math::NncgSolver::Parameters params;
  params.pgsIterations = 1;
  params.restartInterval = 10;
  params.restartThreshold = 1.0;
  options.customOptions = &params;
  RunBenchmark<dart::math::NncgSolver>(
      state, problem, options, MakeLabel("Nncg", "Boxed"));
}

static void BM_LcpCompare_SubspaceMinimization_Boxed(benchmark::State& state)
{
  const int n = static_cast<int>(state.range(0));
  const auto problem
      = MakeBoxedActiveBoundsProblem(n, 904u + static_cast<unsigned>(n));
  auto options = MakeBenchmarkOptions(200);
  dart::math::SubspaceMinimizationSolver::Parameters params;
  params.pgsIterations = 5;
  options.customOptions = &params;
  RunBenchmark<dart::math::SubspaceMinimizationSolver>(
      state, problem, options, MakeLabel("SubspaceMinimization", "Boxed"));
}

static void BM_LcpCompare_Dantzig_FrictionIndex(benchmark::State& state)
{
  const int numContacts = static_cast<int>(state.range(0));
  const auto problem = MakeFrictionIndexProblem(
      numContacts, 1337u + static_cast<unsigned>(numContacts));
  const auto options = MakeBenchmarkOptions(100);
  RunBenchmark<dart::math::DantzigSolver>(
      state, problem, options, MakeLabel("Dantzig", "FrictionIndex"));
}

static void BM_LcpCompare_Pgs_FrictionIndex(benchmark::State& state)
{
  const int numContacts = static_cast<int>(state.range(0));
  const auto problem = MakeFrictionIndexProblem(
      numContacts, 4242u + static_cast<unsigned>(numContacts));
  const auto options = MakeBenchmarkOptions(100);
  RunBenchmark<dart::math::PgsSolver>(
      state, problem, options, MakeLabel("Pgs", "FrictionIndex"));
}

static void BM_LcpCompare_Jacobi_FrictionIndex(benchmark::State& state)
{
  const int numContacts = static_cast<int>(state.range(0));
  const auto problem = MakeFrictionIndexProblem(
      numContacts, 4444u + static_cast<unsigned>(numContacts));
  const auto options = MakeBenchmarkOptions(200);
  RunBenchmark<dart::math::JacobiSolver>(
      state, problem, options, MakeLabel("Jacobi", "FrictionIndex"));
}

static void BM_LcpCompare_SymmetricPsor_FrictionIndex(benchmark::State& state)
{
  const int numContacts = static_cast<int>(state.range(0));
  const auto problem = MakeFrictionIndexProblem(
      numContacts, 4545u + static_cast<unsigned>(numContacts));
  const auto options = MakeBenchmarkOptions(100);
  RunBenchmark<dart::math::SymmetricPsorSolver>(
      state, problem, options, MakeLabel("SymmetricPsor", "FrictionIndex"));
}

static void BM_LcpCompare_RedBlackGaussSeidel_FrictionIndex(
    benchmark::State& state)
{
  const int numContacts = static_cast<int>(state.range(0));
  const auto problem = MakeFrictionIndexProblem(
      numContacts, 4646u + static_cast<unsigned>(numContacts));
  const auto options = MakeBenchmarkOptions(100);
  RunBenchmark<dart::math::RedBlackGaussSeidelSolver>(
      state, problem, options, MakeLabel("RedBlackGS", "FrictionIndex"));
}

static void BM_LcpCompare_Staggering_FrictionIndex(benchmark::State& state)
{
  const int numContacts = static_cast<int>(state.range(0));
  const auto problem = MakeFrictionIndexProblem(
      numContacts, 4747u + static_cast<unsigned>(numContacts));
  const auto options = MakeBenchmarkOptions(100);
  RunBenchmark<dart::math::StaggeringSolver>(
      state, problem, options, MakeLabel("Staggering", "FrictionIndex"));
}

static void BM_LcpCompare_Bgs_FrictionIndex(benchmark::State& state)
{
  const int numContacts = static_cast<int>(state.range(0));
  const auto problem = MakeFrictionIndexProblem(
      numContacts, 4343u + static_cast<unsigned>(numContacts));
  const auto options = MakeBenchmarkOptions(200);
  RunBenchmark<dart::math::BgsSolver>(
      state, problem, options, MakeLabel("Bgs", "FrictionIndex"));
}

static void BM_LcpCompare_BlockedJacobi_FrictionIndex(benchmark::State& state)
{
  const int numContacts = static_cast<int>(state.range(0));
  const auto problem = MakeFrictionIndexProblem(
      numContacts, 4848u + static_cast<unsigned>(numContacts));
  const auto options = MakeBenchmarkOptions(200);
  RunBenchmark<dart::math::BlockedJacobiSolver>(
      state, problem, options, MakeLabel("BlockedJacobi", "FrictionIndex"));
}

static void BM_LcpCompare_Nncg_FrictionIndex(benchmark::State& state)
{
  const int numContacts = static_cast<int>(state.range(0));
  const auto problem = MakeFrictionIndexProblem(
      numContacts, 4949u + static_cast<unsigned>(numContacts));
  auto options = MakeBenchmarkOptions(200);
  dart::math::NncgSolver::Parameters params;
  params.pgsIterations = 1;
  params.restartInterval = 10;
  params.restartThreshold = 1.0;
  options.customOptions = &params;
  RunBenchmark<dart::math::NncgSolver>(
      state, problem, options, MakeLabel("Nncg", "FrictionIndex"));
}

static void BM_LcpCompare_SubspaceMinimization_FrictionIndex(
    benchmark::State& state)
{
  const int numContacts = static_cast<int>(state.range(0));
  const auto problem = MakeFrictionIndexProblem(
      numContacts, 4950u + static_cast<unsigned>(numContacts));
  auto options = MakeBenchmarkOptions(200);
  dart::math::SubspaceMinimizationSolver::Parameters params;
  params.pgsIterations = 5;
  options.customOptions = &params;
  RunBenchmark<dart::math::SubspaceMinimizationSolver>(
      state,
      problem,
      options,
      MakeLabel("SubspaceMinimization", "FrictionIndex"));
}

static void BM_LcpCompare_ShockPropagation_FrictionIndex(
    benchmark::State& state)
{
  const int numContacts = static_cast<int>(state.range(0));
  const auto problem = MakeFrictionIndexProblem(
      numContacts, 4948u + static_cast<unsigned>(numContacts));

  dart::math::ShockPropagationSolver::Parameters params;
  params.blockSizes.assign(numContacts, 3);
  params.layers.clear();
  params.layers.reserve(numContacts);
  for (int i = 0; i < numContacts; ++i)
    params.layers.push_back({i});

  auto options = MakeBenchmarkOptions(100);
  options.customOptions = &params;
  RunBenchmark<dart::math::ShockPropagationSolver>(
      state, problem, options, MakeLabel("ShockPropagation", "FrictionIndex"));
  AddShockPropagationCounters(state, params);
}

static void BM_LcpCompare_Dantzig_Scaled(benchmark::State& state)
{
  const int n = static_cast<int>(state.range(0));
  const double scale = (state.range(1) == 0) ? 1e-6 : 1e6;
  const auto problem
      = MakeScaledProblem(n, scale, 777u + static_cast<unsigned>(n));
  const auto options = MakeBenchmarkOptions(100);
  RunBenchmark<dart::math::DantzigSolver>(
      state, problem, options, MakeLabel("Dantzig", "Scaled"));
}

static void BM_LcpCompare_Pgs_Scaled(benchmark::State& state)
{
  const int n = static_cast<int>(state.range(0));
  const double scale = (state.range(1) == 0) ? 1e-6 : 1e6;
  const auto problem
      = MakeScaledProblem(n, scale, 888u + static_cast<unsigned>(n));
  const auto options = MakeBenchmarkOptions(100);
  RunBenchmark<dart::math::PgsSolver>(
      state, problem, options, MakeLabel("Pgs", "Scaled"));
}

static void BM_LCP_COMPARE_SMOKE(benchmark::State& state)
{
  const auto problem = MakeStandardSpdProblem(12, 1u);
  const auto options = MakeBenchmarkOptions(30);
  RunBenchmark<dart::math::DantzigSolver>(
      state, problem, options, MakeLabel("Dantzig", "Smoke"));
}

} // namespace

BENCHMARK(BM_LcpCompare_Dantzig_Standard)->Arg(12)->Arg(24)->Arg(48)->Arg(96);
BENCHMARK(BM_LcpCompare_Baraff_Standard)->Arg(12)->Arg(24)->Arg(48)->Arg(96);
BENCHMARK(BM_LcpCompare_Direct_Standard)->Arg(2)->Arg(3);
BENCHMARK(BM_LcpCompare_InteriorPoint_Standard)
    ->Arg(12)
    ->Arg(24)
    ->Arg(48)
    ->Arg(96);
BENCHMARK(BM_LcpCompare_Mprgp_Standard)->Arg(12)->Arg(24)->Arg(48)->Arg(96);
BENCHMARK(BM_LcpCompare_ShockPropagation_Standard)
    ->Arg(12)
    ->Arg(24)
    ->Arg(48)
    ->Arg(96);
BENCHMARK(BM_LcpCompare_BlockedJacobi_Standard)
    ->Arg(12)
    ->Arg(24)
    ->Arg(48)
    ->Arg(96);
BENCHMARK(BM_LcpCompare_Pgs_Standard)->Arg(12)->Arg(24)->Arg(48)->Arg(96);
BENCHMARK(BM_LcpCompare_Jacobi_Standard)->Arg(12)->Arg(24)->Arg(48)->Arg(96);
BENCHMARK(BM_LcpCompare_SymmetricPsor_Standard)
    ->Arg(12)
    ->Arg(24)
    ->Arg(48)
    ->Arg(96);
BENCHMARK(BM_LcpCompare_RedBlackGaussSeidel_Standard)
    ->Arg(12)
    ->Arg(24)
    ->Arg(48)
    ->Arg(96);
BENCHMARK(BM_LcpCompare_Staggering_Standard)
    ->Arg(12)
    ->Arg(24)
    ->Arg(48)
    ->Arg(96);
BENCHMARK(BM_LcpCompare_Bgs_Standard)->Arg(12)->Arg(24)->Arg(48)->Arg(96);
BENCHMARK(BM_LcpCompare_Nncg_Standard)->Arg(12)->Arg(24)->Arg(48)->Arg(96);
BENCHMARK(BM_LcpCompare_SubspaceMinimization_Standard)
    ->Arg(12)
    ->Arg(24)
    ->Arg(48)
    ->Arg(96);
BENCHMARK(BM_LcpCompare_Lemke_Standard)->Arg(12)->Arg(24)->Arg(48)->Arg(96);
BENCHMARK(BM_LcpCompare_MinimumMapNewton_Standard)
    ->Arg(12)
    ->Arg(24)
    ->Arg(48)
    ->Arg(96);
BENCHMARK(BM_LcpCompare_FischerBurmeisterNewton_Standard)
    ->Arg(12)
    ->Arg(24)
    ->Arg(48)
    ->Arg(96);
BENCHMARK(BM_LcpCompare_PenalizedFischerBurmeisterNewton_Standard)
    ->Arg(12)
    ->Arg(24)
    ->Arg(48)
    ->Arg(96);

BENCHMARK(BM_LcpCompare_Dantzig_Boxed)->Arg(12)->Arg(24)->Arg(48);
BENCHMARK(BM_LcpCompare_Pgs_Boxed)->Arg(12)->Arg(24)->Arg(48);
BENCHMARK(BM_LcpCompare_Jacobi_Boxed)->Arg(12)->Arg(24)->Arg(48);
BENCHMARK(BM_LcpCompare_SymmetricPsor_Boxed)->Arg(12)->Arg(24)->Arg(48);
BENCHMARK(BM_LcpCompare_RedBlackGaussSeidel_Boxed)->Arg(12)->Arg(24)->Arg(48);
BENCHMARK(BM_LcpCompare_Staggering_Boxed)->Arg(12)->Arg(24)->Arg(48);
BENCHMARK(BM_LcpCompare_Bgs_Boxed)->Arg(12)->Arg(24)->Arg(48);
BENCHMARK(BM_LcpCompare_BlockedJacobi_Boxed)->Arg(12)->Arg(24)->Arg(48);
BENCHMARK(BM_LcpCompare_Nncg_Boxed)->Arg(12)->Arg(24)->Arg(48);
BENCHMARK(BM_LcpCompare_SubspaceMinimization_Boxed)->Arg(12)->Arg(24)->Arg(48);

BENCHMARK(BM_LcpCompare_Dantzig_FrictionIndex)->Arg(4)->Arg(16)->Arg(64);
BENCHMARK(BM_LcpCompare_Pgs_FrictionIndex)->Arg(4)->Arg(16)->Arg(64);
BENCHMARK(BM_LcpCompare_Jacobi_FrictionIndex)->Arg(4)->Arg(16)->Arg(64);
BENCHMARK(BM_LcpCompare_SymmetricPsor_FrictionIndex)->Arg(4)->Arg(16)->Arg(64);
BENCHMARK(BM_LcpCompare_RedBlackGaussSeidel_FrictionIndex)
    ->Arg(4)
    ->Arg(16)
    ->Arg(64);
BENCHMARK(BM_LcpCompare_Staggering_FrictionIndex)->Arg(4)->Arg(16)->Arg(64);
BENCHMARK(BM_LcpCompare_Bgs_FrictionIndex)->Arg(4)->Arg(16)->Arg(64);
BENCHMARK(BM_LcpCompare_BlockedJacobi_FrictionIndex)->Arg(4)->Arg(16)->Arg(64);
BENCHMARK(BM_LcpCompare_Nncg_FrictionIndex)->Arg(4)->Arg(16)->Arg(64);
BENCHMARK(BM_LcpCompare_SubspaceMinimization_FrictionIndex)
    ->Arg(4)
    ->Arg(16)
    ->Arg(64);
BENCHMARK(BM_LcpCompare_ShockPropagation_FrictionIndex)
    ->Arg(4)
    ->Arg(16)
    ->Arg(64);

BENCHMARK(BM_LcpCompare_Dantzig_Scaled)->Args({12, 0})->Args({12, 1});
BENCHMARK(BM_LcpCompare_Pgs_Scaled)->Args({12, 0})->Args({12, 1});

BENCHMARK(BM_LCP_COMPARE_SMOKE);
