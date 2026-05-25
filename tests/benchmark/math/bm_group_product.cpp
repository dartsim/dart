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

/// @file bm_group_product.cpp
/// @brief Benchmarks for robot-shaped Lie-group direct products.
///
/// The humanoid case models a floating base, two spherical joints, and
/// twenty-one scalar joints as SE(3) x SO(3) x SO(3) x R x ... x R.
///
/// Run:
///   ./bm_group_product --benchmark_counters_tabular=true

#include <dart/math/lie_groups.hpp>

#include <Eigen/StdVector>
#include <benchmark/benchmark.h>

#include <vector>

namespace {

template <typename S>
using HumanoidConfiguration = dart::math::GroupProduct<
    S,
    dart::math::SE3,
    dart::math::SO3,
    dart::math::SO3,
    dart::math::R1,
    dart::math::R1,
    dart::math::R1,
    dart::math::R1,
    dart::math::R1,
    dart::math::R1,
    dart::math::R1,
    dart::math::R1,
    dart::math::R1,
    dart::math::R1,
    dart::math::R1,
    dart::math::R1,
    dart::math::R1,
    dart::math::R1,
    dart::math::R1,
    dart::math::R1,
    dart::math::R1,
    dart::math::R1,
    dart::math::R1,
    dart::math::R1,
    dart::math::R1>;

using Humanoid = HumanoidConfiguration<double>;

template <typename T>
using AlignedVector = std::vector<T, Eigen::aligned_allocator<T>>;

constexpr std::size_t kHumanoidScalarStart = 3;

Humanoid makeHumanoidConfiguration(int seed)
{
  Humanoid q = Humanoid::Identity();
  q.template get<0>() = dart::math::SE3<double>::Random();
  q.template get<1>() = dart::math::SO3<double>::Random();
  q.template get<2>() = dart::math::SO3<double>::Random();

  for (std::size_t i = kHumanoidScalarStart; i < Humanoid::ProductSize; ++i) {
    const auto jointIndex = static_cast<double>(i - kHumanoidScalarStart + 1);
    q.params(i)[0] = 0.01 * static_cast<double>(seed + 1) * jointIndex;
  }

  return q;
}

struct HumanoidData
{
  AlignedVector<Humanoid> lhs;
  AlignedVector<Humanoid> rhs;
  AlignedVector<Humanoid> out;
  AlignedVector<Humanoid::Tangent> tangent;
};

HumanoidData makeHumanoidData(int n)
{
  HumanoidData data;
  data.lhs.reserve(n);
  data.rhs.reserve(n);
  data.out.reserve(n);
  data.tangent.reserve(n);

  for (int i = 0; i < n; ++i) {
    data.lhs.push_back(makeHumanoidConfiguration(i));
    data.rhs.push_back(makeHumanoidConfiguration(i + n));
    data.out.push_back(Humanoid::Identity());
    data.tangent.push_back(Humanoid::Tangent::Zero());
  }

  return data;
}

// =============================================================================
// Humanoid direct-product operations
// =============================================================================

static void BM_GroupProduct_HumanoidCompose(benchmark::State& state)
{
  const int n = static_cast<int>(state.range(0));
  auto data = makeHumanoidData(n);

  for (auto _ : state) {
    for (int i = 0; i < n; ++i) {
      data.out[i] = data.lhs[i] * data.rhs[i];
    }
    benchmark::DoNotOptimize(data.out.data());
  }

  state.SetItemsProcessed(
      static_cast<int64_t>(state.iterations()) * static_cast<int64_t>(n));
}
BENCHMARK(BM_GroupProduct_HumanoidCompose)
    ->Arg(1)
    ->Arg(8)
    ->Arg(32)
    ->Arg(128)
    ->Unit(benchmark::kNanosecond);

static void BM_GroupProduct_HumanoidInverse(benchmark::State& state)
{
  const int n = static_cast<int>(state.range(0));
  auto data = makeHumanoidData(n);

  for (auto _ : state) {
    for (int i = 0; i < n; ++i) {
      data.out[i] = data.lhs[i].inverse();
    }
    benchmark::DoNotOptimize(data.out.data());
  }

  state.SetItemsProcessed(
      static_cast<int64_t>(state.iterations()) * static_cast<int64_t>(n));
}
BENCHMARK(BM_GroupProduct_HumanoidInverse)
    ->Arg(1)
    ->Arg(8)
    ->Arg(32)
    ->Arg(128)
    ->Unit(benchmark::kNanosecond);

static void BM_GroupProduct_HumanoidLog(benchmark::State& state)
{
  const int n = static_cast<int>(state.range(0));
  auto data = makeHumanoidData(n);

  for (auto _ : state) {
    for (int i = 0; i < n; ++i) {
      data.tangent[i] = data.lhs[i].log();
    }
    benchmark::DoNotOptimize(data.tangent.data());
  }

  state.SetItemsProcessed(
      static_cast<int64_t>(state.iterations()) * static_cast<int64_t>(n));
}
BENCHMARK(BM_GroupProduct_HumanoidLog)
    ->Arg(1)
    ->Arg(8)
    ->Arg(32)
    ->Arg(128)
    ->Unit(benchmark::kNanosecond);

} // namespace
