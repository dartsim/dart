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
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 */

/// @file bm_lie_group_batch.cpp
/// @brief Benchmarks for the DART 7 Lie-group batch API.
///
/// Run:
///   pixi run bm-lie-group-batch

#include <dart/math/lie_groups.hpp>

#include <Eigen/Core>
#include <benchmark/benchmark.h>

#include <vector>

#include <cstddef>
#include <cstdint>

namespace {

constexpr int kSmallBatchSize = 128;
constexpr int kMediumBatchSize = 1024;
constexpr int kLargeBatchSize = 8192;

template <typename S>
S deterministicTangentValue(std::size_t index, int component)
{
  const auto raw = static_cast<int>((index * 17 + component * 31 + 7) % 101);
  return static_cast<S>(0.01) * static_cast<S>(raw - 50);
}

template <typename T>
struct LieGroupBatchData
{
  using Scalar = typename T::Scalar;

  std::vector<Scalar> tangents;
  std::vector<Scalar> groups;
  std::vector<Scalar> outputTangents;
  std::vector<Scalar> outputGroups;
};

template <typename T>
LieGroupBatchData<T> makeLieGroupBatchData(std::size_t count)
{
  using S = typename T::Scalar;
  constexpr int P = T::ParamSize;
  constexpr int D = T::DoF;

  LieGroupBatchData<T> data;
  data.tangents.resize(count * D);
  data.groups.resize(count * P);
  data.outputTangents.resize(count * D);
  data.outputGroups.resize(count * P);

  for (std::size_t i = 0; i < count; ++i) {
    for (int j = 0; j < D; ++j) {
      data.tangents[i * D + j] = deterministicTangentValue<S>(i, j);
    }
  }

  dart::math::expBatch<T>(data.tangents.data(), data.groups.data(), count);

  return data;
}

void setBatchItemsProcessed(benchmark::State& state, std::size_t count)
{
  state.SetItemsProcessed(
      static_cast<int64_t>(state.iterations()) * static_cast<int64_t>(count));
}

template <typename T>
void BM_LieGroupBatchExp(benchmark::State& state)
{
  const auto count = static_cast<std::size_t>(state.range(0));
  auto data = makeLieGroupBatchData<T>(count);

  for (auto _ : state) {
    dart::math::expBatch<T>(
        data.tangents.data(), data.outputGroups.data(), count);
    benchmark::DoNotOptimize(data.outputGroups.data());
    benchmark::ClobberMemory();
  }

  setBatchItemsProcessed(state, count);
}

template <typename T>
void BM_LieGroupScalarExpLoop(benchmark::State& state)
{
  using S = typename T::Scalar;
  constexpr int P = T::ParamSize;
  constexpr int D = T::DoF;

  const auto count = static_cast<std::size_t>(state.range(0));
  auto data = makeLieGroupBatchData<T>(count);

  for (auto _ : state) {
    for (std::size_t i = 0; i < count; ++i) {
      const typename T::Tangent tangent(
          Eigen::Map<const Eigen::Matrix<S, D, 1>>(
              data.tangents.data() + i * D));
      Eigen::Map<T>(data.outputGroups.data() + i * P)
          = dart::math::Exp(tangent);
    }
    benchmark::DoNotOptimize(data.outputGroups.data());
    benchmark::ClobberMemory();
  }

  setBatchItemsProcessed(state, count);
}

template <typename T>
void BM_LieGroupBatchLog(benchmark::State& state)
{
  const auto count = static_cast<std::size_t>(state.range(0));
  auto data = makeLieGroupBatchData<T>(count);

  for (auto _ : state) {
    dart::math::logBatch<T>(
        data.groups.data(), data.outputTangents.data(), count);
    benchmark::DoNotOptimize(data.outputTangents.data());
    benchmark::ClobberMemory();
  }

  setBatchItemsProcessed(state, count);
}

template <typename T>
void BM_LieGroupScalarLogLoop(benchmark::State& state)
{
  using S = typename T::Scalar;
  constexpr int P = T::ParamSize;
  constexpr int D = T::DoF;

  const auto count = static_cast<std::size_t>(state.range(0));
  auto data = makeLieGroupBatchData<T>(count);

  for (auto _ : state) {
    for (std::size_t i = 0; i < count; ++i) {
      const Eigen::Map<const T> group(data.groups.data() + i * P);
      Eigen::Map<Eigen::Matrix<S, D, 1>>(data.outputTangents.data() + i * D)
          = dart::math::Log(group).params();
    }
    benchmark::DoNotOptimize(data.outputTangents.data());
    benchmark::ClobberMemory();
  }

  setBatchItemsProcessed(state, count);
}

template <typename T>
void BM_LieGroupBatchAdjoint(benchmark::State& state)
{
  const auto count = static_cast<std::size_t>(state.range(0));
  auto data = makeLieGroupBatchData<T>(count);

  for (auto _ : state) {
    dart::math::adjointBatch<T>(
        data.groups.data(),
        data.tangents.data(),
        data.outputTangents.data(),
        count);
    benchmark::DoNotOptimize(data.outputTangents.data());
    benchmark::ClobberMemory();
  }

  setBatchItemsProcessed(state, count);
}

template <typename T>
void BM_LieGroupScalarAdjointLoop(benchmark::State& state)
{
  using S = typename T::Scalar;
  constexpr int P = T::ParamSize;
  constexpr int D = T::DoF;

  const auto count = static_cast<std::size_t>(state.range(0));
  auto data = makeLieGroupBatchData<T>(count);

  for (auto _ : state) {
    for (std::size_t i = 0; i < count; ++i) {
      const Eigen::Map<const T> group(data.groups.data() + i * P);
      const typename T::Tangent tangent(
          Eigen::Map<const Eigen::Matrix<S, D, 1>>(
              data.tangents.data() + i * D));
      Eigen::Map<Eigen::Matrix<S, D, 1>>(data.outputTangents.data() + i * D)
          = dart::math::Ad(group, tangent).params();
    }
    benchmark::DoNotOptimize(data.outputTangents.data());
    benchmark::ClobberMemory();
  }

  setBatchItemsProcessed(state, count);
}

using SO3d = dart::math::SO3<double>;
using SE3d = dart::math::SE3<double>;

void BM_LieGroupBatchExpSO3(benchmark::State& state)
{
  BM_LieGroupBatchExp<SO3d>(state);
}
BENCHMARK(BM_LieGroupBatchExpSO3)
    ->Arg(kSmallBatchSize)
    ->Arg(kMediumBatchSize)
    ->Arg(kLargeBatchSize)
    ->Unit(benchmark::kNanosecond);

void BM_LieGroupScalarExpLoopSO3(benchmark::State& state)
{
  BM_LieGroupScalarExpLoop<SO3d>(state);
}
BENCHMARK(BM_LieGroupScalarExpLoopSO3)
    ->Arg(kSmallBatchSize)
    ->Arg(kMediumBatchSize)
    ->Arg(kLargeBatchSize)
    ->Unit(benchmark::kNanosecond);

void BM_LieGroupBatchLogSO3(benchmark::State& state)
{
  BM_LieGroupBatchLog<SO3d>(state);
}
BENCHMARK(BM_LieGroupBatchLogSO3)
    ->Arg(kSmallBatchSize)
    ->Arg(kMediumBatchSize)
    ->Arg(kLargeBatchSize)
    ->Unit(benchmark::kNanosecond);

void BM_LieGroupScalarLogLoopSO3(benchmark::State& state)
{
  BM_LieGroupScalarLogLoop<SO3d>(state);
}
BENCHMARK(BM_LieGroupScalarLogLoopSO3)
    ->Arg(kSmallBatchSize)
    ->Arg(kMediumBatchSize)
    ->Arg(kLargeBatchSize)
    ->Unit(benchmark::kNanosecond);

void BM_LieGroupBatchAdjointSO3(benchmark::State& state)
{
  BM_LieGroupBatchAdjoint<SO3d>(state);
}
BENCHMARK(BM_LieGroupBatchAdjointSO3)
    ->Arg(kSmallBatchSize)
    ->Arg(kMediumBatchSize)
    ->Arg(kLargeBatchSize)
    ->Unit(benchmark::kNanosecond);

void BM_LieGroupScalarAdjointLoopSO3(benchmark::State& state)
{
  BM_LieGroupScalarAdjointLoop<SO3d>(state);
}
BENCHMARK(BM_LieGroupScalarAdjointLoopSO3)
    ->Arg(kSmallBatchSize)
    ->Arg(kMediumBatchSize)
    ->Arg(kLargeBatchSize)
    ->Unit(benchmark::kNanosecond);

void BM_LieGroupBatchExpSE3(benchmark::State& state)
{
  BM_LieGroupBatchExp<SE3d>(state);
}
BENCHMARK(BM_LieGroupBatchExpSE3)
    ->Arg(kSmallBatchSize)
    ->Arg(kMediumBatchSize)
    ->Arg(kLargeBatchSize)
    ->Unit(benchmark::kNanosecond);

void BM_LieGroupScalarExpLoopSE3(benchmark::State& state)
{
  BM_LieGroupScalarExpLoop<SE3d>(state);
}
BENCHMARK(BM_LieGroupScalarExpLoopSE3)
    ->Arg(kSmallBatchSize)
    ->Arg(kMediumBatchSize)
    ->Arg(kLargeBatchSize)
    ->Unit(benchmark::kNanosecond);

void BM_LieGroupBatchLogSE3(benchmark::State& state)
{
  BM_LieGroupBatchLog<SE3d>(state);
}
BENCHMARK(BM_LieGroupBatchLogSE3)
    ->Arg(kSmallBatchSize)
    ->Arg(kMediumBatchSize)
    ->Arg(kLargeBatchSize)
    ->Unit(benchmark::kNanosecond);

void BM_LieGroupScalarLogLoopSE3(benchmark::State& state)
{
  BM_LieGroupScalarLogLoop<SE3d>(state);
}
BENCHMARK(BM_LieGroupScalarLogLoopSE3)
    ->Arg(kSmallBatchSize)
    ->Arg(kMediumBatchSize)
    ->Arg(kLargeBatchSize)
    ->Unit(benchmark::kNanosecond);

void BM_LieGroupBatchAdjointSE3(benchmark::State& state)
{
  BM_LieGroupBatchAdjoint<SE3d>(state);
}
BENCHMARK(BM_LieGroupBatchAdjointSE3)
    ->Arg(kSmallBatchSize)
    ->Arg(kMediumBatchSize)
    ->Arg(kLargeBatchSize)
    ->Unit(benchmark::kNanosecond);

void BM_LieGroupScalarAdjointLoopSE3(benchmark::State& state)
{
  BM_LieGroupScalarAdjointLoop<SE3d>(state);
}
BENCHMARK(BM_LieGroupScalarAdjointLoopSE3)
    ->Arg(kSmallBatchSize)
    ->Arg(kMediumBatchSize)
    ->Arg(kLargeBatchSize)
    ->Unit(benchmark::kNanosecond);

} // namespace
