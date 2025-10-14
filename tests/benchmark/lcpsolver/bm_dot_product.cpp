(/*
 * Benchmark: Dot Product Optimization
 *
 * Compares manual loop unrolling vs Eigen SIMD for dot products.
 * This is a critical operation used extensively in the LCP solver.
 */

#include <Eigen/Core>
#include <benchmark/benchmark.h>

#include <random>
#include <vector>

using dReal = double;

// Original ODE-style implementation (2-at-a-time unrolling)
dReal dotOriginal(const dReal* a, const dReal* b, int n)
{
  dReal p0, q0, m0, p1, q1, m1, sum;
  sum = 0;
  n -= 2;
  while (n >= 0) {
    p0 = a[0];
    q0 = b[0];
    m0 = p0 * q0;
    p1 = a[1];
    q1 = b[1];
    m1 = p1 * q1;
    sum += m0;
    sum += m1;
    a += 2;
    b += 2;
    n -= 2;
  }
  n += 2;
  while (n > 0) {
    sum += (*a) * (*b);
    a++;
    b++;
    n--;
  }
  return sum;
}

// Eigen-optimized implementation
dReal dotEigen(const dReal* a, const dReal* b, int n)
{
  Eigen::Map<const Eigen::VectorXd> va(a, n);
  Eigen::Map<const Eigen::VectorXd> vb(b, n);
  return va.dot(vb);
}

// Benchmark fixture
template <int N>
class DotProductFixture : public benchmark::Fixture {
public:
  void SetUp(const ::benchmark::State&) override
  {
    std::random_device rd;
    std::mt19937 gen(42); // Fixed seed for reproducibility
    std::uniform_real_distribution<> dis(-1.0, 1.0);

    a.resize(N);
    b.resize(N);
    for (int i = 0; i < N; ++i) {
      a[i] = dis(gen);
      b[i] = dis(gen);
    }
  }

  std::vector<dReal> a, b;
};

// Instantiate fixtures for different sizes
using DotProduct10 = DotProductFixture<10>;
using DotProduct50 = DotProductFixture<50>;
using DotProduct100 = DotProductFixture<100>;
using DotProduct200 = DotProductFixture<200>;

// Benchmarks for N=10
BENCHMARK_F(DotProduct10, Original)(benchmark::State& st) {
  volatile dReal result;
  for (auto _ : st) {
    result = dotOriginal(a.data(), b.data(), 10);
    benchmark::DoNotOptimize(result);
  }
  st.SetLabel("Original-10");
}

BENCHMARK_F(DotProduct10, Eigen)(benchmark::State& st) {
  volatile dReal result;
  for (auto _ : st) {
    result = dotEigen(a.data(), b.data(), 10);
    benchmark::DoNotOptimize(result);
  }
  st.SetLabel("Eigen-10");
}

// Benchmarks for N=50
BENCHMARK_F(DotProduct50, Original)(benchmark::State& st) {
  volatile dReal result;
  for (auto _ : st) {
    result = dotOriginal(a.data(), b.data(), 50);
    benchmark::DoNotOptimize(result);
  }
  st.SetLabel("Original-50");
}

BENCHMARK_F(DotProduct50, Eigen)(benchmark::State& st) {
  volatile dReal result;
  for (auto _ : st) {
    result = dotEigen(a.data(), b.data(), 50);
    benchmark::DoNotOptimize(result);
  }
  st.SetLabel("Eigen-50");
}

// Benchmarks for N=100
BENCHMARK_F(DotProduct100, Original)(benchmark::State& st) {
  volatile dReal result;
  for (auto _ : st) {
    result = dotOriginal(a.data(), b.data(), 100);
    benchmark::DoNotOptimize(result);
  }
  st.SetLabel("Original-100");
}

BENCHMARK_F(DotProduct100, Eigen)(benchmark::State& st) {
  volatile dReal result;
  for (auto _ : st) {
    result = dotEigen(a.data(), b.data(), 100);
    benchmark::DoNotOptimize(result);
  }
  st.SetLabel("Eigen-100");
}

// Benchmarks for N=200
BENCHMARK_F(DotProduct200, Original)(benchmark::State& st) {
  volatile dReal result;
  for (auto _ : st) {
    result = dotOriginal(a.data(), b.data(), 200);
    benchmark::DoNotOptimize(result);
  }
  st.SetLabel("Original-200");
}

BENCHMARK_F(DotProduct200, Eigen)(benchmark::State& st) {
  volatile dReal result;
  for (auto _ : st) {
    result = dotEigen(a.data(), b.data(), 200);
    benchmark::DoNotOptimize(result);
  }
  st.SetLabel("Eigen-200");
}

BENCHMARK_MAIN();
