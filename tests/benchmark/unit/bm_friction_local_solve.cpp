#include <dart/constraint/detail/FrictionCone.hpp>

#include <benchmark/benchmark.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <vector>

#include <cmath>
#include <cstdint>

namespace {

using dart::constraint::detail::FrictionCone;
using dart::constraint::detail::FrictionConeLaw;
using dart::constraint::detail::LocalSolveResult;
using dart::constraint::detail::solveConeQp;
using dart::constraint::detail::solveExactContact;

struct Problem
{
  Eigen::Matrix3d h;
  Eigen::Vector3d c;
  FrictionCone cone;
};

Problem makeProblem(int caseIndex, FrictionConeLaw law)
{
  Problem problem;
  problem.h << 2.0, 0.2, -0.1, 0.2, 1.5, 0.1, -0.1, 0.1, 1.0;
  problem.cone.mu = Eigen::Vector2d(0.7, 1.2);
  problem.cone.law = law;
  const auto& mu = problem.cone.mu;

  if (caseIndex == 0) {
    const Eigen::Vector3d impulse(1.0, 0.1 * mu[0], -0.1 * mu[1]);
    problem.c = -problem.h * impulse;
  } else if (caseIndex == 1) {
    problem.c = Eigen::Vector3d(1.0, 0.2 / mu[0], 0.2 / mu[1]);
  } else {
    const bool ellipse = law == FrictionConeLaw::Ellipse;
    const Eigen::Vector3d impulse(
        1.0, -mu[0] * (ellipse ? 0.6 : 1.0), -mu[1] * (ellipse ? 0.8 : 1.0));
    // These opposing tangent velocities give a known sliding solution.
    Eigen::Vector3d velocity(0.0, 0.6 / mu[0], 0.8 / mu[1]);
    if (caseIndex == 2)
      velocity[0] = ellipse ? 1.0 : 1.4;
    problem.c = velocity - problem.h * impulse;
  }

  return problem;
}

template <typename Values>
double sortedQuantile(const Values& sorted, double fraction)
{
  const double position = fraction * static_cast<double>(sorted.size() - 1);
  const auto lower = static_cast<std::size_t>(std::floor(position));
  const auto upper = static_cast<std::size_t>(std::ceil(position));
  return sorted[lower]
         + (position - static_cast<double>(lower))
               * (sorted[upper] - sorted[lower]);
}

// Google Benchmark passes repetition means; this is the batched ns/solve p99.
double p99(const std::vector<double>& observations)
{
  auto sorted = observations;
  std::sort(sorted.begin(), sorted.end());
  return sortedQuantile(sorted, 0.99);
}

struct CallLatency
{
  double medianNs = 0.0;
  double p99Ns = 0.0;
  double clockPairMedianNs = 0.0;
  bool certified = true;
  bool measured = false;
};

CallLatency measureCallLatency(const Problem& problem, int caseIndex)
{
  using Clock = std::chrono::steady_clock;
  using Nanoseconds = std::chrono::duration<double, std::nano>;
  std::array<double, 10000> samples;
  std::array<double, 10000> clockSamples;
  CallLatency latency;

  for (std::size_t i = 0; i < samples.size(); ++i) {
    const auto start = Clock::now();
    LocalSolveResult result
        = caseIndex == 3 ? solveExactContact(problem.h, problem.c, problem.cone)
                         : solveConeQp(problem.h, problem.c, problem.cone);
    const auto end = Clock::now();
    benchmark::DoNotOptimize(result.impulse);
    benchmark::DoNotOptimize(result.certified);
    latency.certified = latency.certified && result.certified;
    samples[i] = Nanoseconds(end - start).count();

    const auto clockStart = Clock::now();
    const auto clockEnd = Clock::now();
    clockSamples[i] = Nanoseconds(clockEnd - clockStart).count();
  }

  std::sort(samples.begin(), samples.end());
  std::sort(clockSamples.begin(), clockSamples.end());
  latency.medianNs = sortedQuantile(samples, 0.5);
  latency.p99Ns = sortedQuantile(samples, 0.99);
  latency.clockPairMedianNs = sortedQuantile(clockSamples, 0.5);
  latency.measured = true;
  return latency;
}

void localSolve(benchmark::State& state, int caseIndex, FrictionConeLaw law)
{
  const auto problem = makeProblem(caseIndex, law);
  // Collect once per case before Google Benchmark starts the timed loop.
  static std::array<CallLatency, 8> latencies;
  auto& latency = latencies[caseIndex + (law == FrictionConeLaw::Box ? 4 : 0)];
  if (!latency.measured)
    latency = measureCallLatency(problem, caseIndex);
  if (!latency.certified) {
    state.SkipWithError("individual local friction samples were not certified");
    return;
  }

  std::uint64_t fallbackCount = 0;
  std::uint64_t qpCount = 0;

  for (auto _ : state) {
    LocalSolveResult result
        = caseIndex == 3 ? solveExactContact(problem.h, problem.c, problem.cone)
                         : solveConeQp(problem.h, problem.c, problem.cone);
    benchmark::DoNotOptimize(result.impulse);
    benchmark::DoNotOptimize(result.certified);
    if (!result.certified) {
      state.SkipWithError("local friction solve was not certified");
      break;
    }
    fallbackCount += result.numLocalFallbacks;
    qpCount += result.numQpSolves;
  }

  state.counters["fallbacks/solve"] = benchmark::Counter(
      static_cast<double>(fallbackCount), benchmark::Counter::kAvgIterations);
  state.counters["QPs/solve"] = benchmark::Counter(
      static_cast<double>(qpCount), benchmark::Counter::kAvgIterations);
  state.counters["median_call_ns"] = latency.medianNs;
  state.counters["p99_call_ns"] = latency.p99Ns;
  state.counters["clock_pair_median_ns"] = latency.clockPairMedianNs;
}

} // namespace

#define FRICTION_BENCHMARK(name, caseIndex, law)                               \
  BENCHMARK_CAPTURE(localSolve, name, caseIndex, FrictionConeLaw::law)         \
      ->Unit(benchmark::kNanosecond)                                           \
      ->MinTime(0.02)                                                          \
      ->Repetitions(51)                                                        \
      ->ComputeStatistics("p99", p99)                                          \
      ->ReportAggregatesOnly()

FRICTION_BENCHMARK(ellipse_interior, 0, Ellipse);
FRICTION_BENCHMARK(ellipse_apex, 1, Ellipse);
FRICTION_BENCHMARK(ellipse_boundary, 2, Ellipse);
FRICTION_BENCHMARK(ellipse_exact, 3, Ellipse);
FRICTION_BENCHMARK(box_interior, 0, Box);
FRICTION_BENCHMARK(box_apex, 1, Box);
FRICTION_BENCHMARK(box_boundary, 2, Box);
FRICTION_BENCHMARK(box_exact, 3, Box);

#undef FRICTION_BENCHMARK
