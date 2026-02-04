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

#include <dart/math/lcp/pivoting/dantzig/matrix.hpp>

#include <Eigen/Dense>
#include <benchmark/benchmark.h>

#include <random>
#include <vector>

#include <cstring>

using namespace dart::math;

class MatrixMultiplyFixture : public benchmark::Fixture
{
public:
  void SetUp(const ::benchmark::State& state) override
  {
    size = state.range(0);
    std::mt19937 rng(42);
    std::uniform_real_distribution<double> dist(-1.0, 1.0);

    A.resize(size * size);
    B.resize(size * size);
    C.resize(size * size);

    for (auto& val : A) {
      val = dist(rng);
    }
    for (auto& val : B) {
      val = dist(rng);
    }
  }

  void TearDown(const ::benchmark::State&) override
  {
    A.clear();
    B.clear();
    C.clear();
  }

protected:
  int size;
  std::vector<double> A;
  std::vector<double> B;
  std::vector<double> C;
};

// Baseline: Raw pointer triple-loop implementation
BENCHMARK_DEFINE_F(MatrixMultiplyFixture, RawPointer_Multiply0)
(benchmark::State& state)
{
  for (auto _ : state) {
    // A = B * C (all row-major)
    std::memset(A.data(), 0, size * size * sizeof(double));
    for (int i = 0; i < size; ++i) {
      for (int j = 0; j < size; ++j) {
        double sum = 0.0;
        for (int k = 0; k < size; ++k) {
          sum += B[i * size + k] * C[k * size + j];
        }
        A[i * size + j] = sum;
      }
    }
    benchmark::DoNotOptimize(A.data());
    benchmark::ClobberMemory();
  }
  state.SetItemsProcessed(state.iterations() * size * size * size);
}

// Eigen SIMD version
BENCHMARK_DEFINE_F(MatrixMultiplyFixture, Eigen_Multiply0)
(benchmark::State& state)
{
  using Matrix
      = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;
  Eigen::Map<Matrix> mapA(A.data(), size, size);
  Eigen::Map<const Matrix> mapB(B.data(), size, size);
  Eigen::Map<const Matrix> mapC(C.data(), size, size);

  for (auto _ : state) {
    mapA.noalias() = mapB * mapC;
    benchmark::DoNotOptimize(A.data());
    benchmark::ClobberMemory();
  }
  state.SetItemsProcessed(state.iterations() * size * size * size);
}

// Test B' * C (B transposed)
BENCHMARK_DEFINE_F(MatrixMultiplyFixture, RawPointer_Multiply1)
(benchmark::State& state)
{
  for (auto _ : state) {
    // A = B' * C (B transposed, row-major)
    std::memset(A.data(), 0, size * size * sizeof(double));
    for (int i = 0; i < size; ++i) {
      for (int j = 0; j < size; ++j) {
        double sum = 0.0;
        for (int k = 0; k < size; ++k) {
          sum += B[k * size + i] * C[k * size + j]; // B transposed access
        }
        A[i * size + j] = sum;
      }
    }
    benchmark::DoNotOptimize(A.data());
    benchmark::ClobberMemory();
  }
  state.SetItemsProcessed(state.iterations() * size * size * size);
}

BENCHMARK_DEFINE_F(MatrixMultiplyFixture, Eigen_Multiply1)
(benchmark::State& state)
{
  using Matrix
      = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;
  Eigen::Map<Matrix> mapA(A.data(), size, size);
  Eigen::Map<const Matrix> mapB(B.data(), size, size);
  Eigen::Map<const Matrix> mapC(C.data(), size, size);

  for (auto _ : state) {
    mapA.noalias() = mapB.transpose() * mapC;
    benchmark::DoNotOptimize(A.data());
    benchmark::ClobberMemory();
  }
  state.SetItemsProcessed(state.iterations() * size * size * size);
}

// Test B * C' (C transposed)
BENCHMARK_DEFINE_F(MatrixMultiplyFixture, RawPointer_Multiply2)
(benchmark::State& state)
{
  for (auto _ : state) {
    // A = B * C' (C transposed, row-major)
    std::memset(A.data(), 0, size * size * sizeof(double));
    for (int i = 0; i < size; ++i) {
      for (int j = 0; j < size; ++j) {
        double sum = 0.0;
        for (int k = 0; k < size; ++k) {
          sum += B[i * size + k] * C[j * size + k]; // C transposed access
        }
        A[i * size + j] = sum;
      }
    }
    benchmark::DoNotOptimize(A.data());
    benchmark::ClobberMemory();
  }
  state.SetItemsProcessed(state.iterations() * size * size * size);
}

BENCHMARK_DEFINE_F(MatrixMultiplyFixture, Eigen_Multiply2)
(benchmark::State& state)
{
  using Matrix
      = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;
  Eigen::Map<Matrix> mapA(A.data(), size, size);
  Eigen::Map<const Matrix> mapB(B.data(), size, size);
  Eigen::Map<const Matrix> mapC(C.data(), size, size);

  for (auto _ : state) {
    mapA.noalias() = mapB * mapC.transpose();
    benchmark::DoNotOptimize(A.data());
    benchmark::ClobberMemory();
  }
  state.SetItemsProcessed(state.iterations() * size * size * size);
}

// Register benchmarks for various sizes relevant to LCP solver
// Small matrices (common in LCP solver due to contact problem sizes)
BENCHMARK_REGISTER_F(MatrixMultiplyFixture, RawPointer_Multiply0)
    ->Args({4})
    ->Args({6})
    ->Args({8})
    ->Args({12})
    ->Args({16})
    ->Args({24})
    ->Args({32})
    ->Args({48})
    ->Unit(benchmark::kNanosecond);

BENCHMARK_REGISTER_F(MatrixMultiplyFixture, Eigen_Multiply0)
    ->Args({4})
    ->Args({6})
    ->Args({8})
    ->Args({12})
    ->Args({16})
    ->Args({24})
    ->Args({32})
    ->Args({48})
    ->Unit(benchmark::kNanosecond);

BENCHMARK_REGISTER_F(MatrixMultiplyFixture, RawPointer_Multiply1)
    ->Args({4})
    ->Args({6})
    ->Args({8})
    ->Args({12})
    ->Args({16})
    ->Args({24})
    ->Args({32})
    ->Args({48})
    ->Unit(benchmark::kNanosecond);

BENCHMARK_REGISTER_F(MatrixMultiplyFixture, Eigen_Multiply1)
    ->Args({4})
    ->Args({6})
    ->Args({8})
    ->Args({12})
    ->Args({16})
    ->Args({24})
    ->Args({32})
    ->Args({48})
    ->Unit(benchmark::kNanosecond);

BENCHMARK_REGISTER_F(MatrixMultiplyFixture, RawPointer_Multiply2)
    ->Args({4})
    ->Args({6})
    ->Args({8})
    ->Args({12})
    ->Args({16})
    ->Args({24})
    ->Args({32})
    ->Args({48})
    ->Unit(benchmark::kNanosecond);

BENCHMARK_REGISTER_F(MatrixMultiplyFixture, Eigen_Multiply2)
    ->Args({4})
    ->Args({6})
    ->Args({8})
    ->Args({12})
    ->Args({16})
    ->Args({24})
    ->Args({32})
    ->Args({48})
    ->Unit(benchmark::kNanosecond);

BENCHMARK_MAIN();
