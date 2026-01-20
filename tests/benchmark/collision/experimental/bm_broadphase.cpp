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

#include <dart/collision/experimental/broad_phase/aabb_tree.hpp>
#include <dart/collision/experimental/broad_phase/brute_force.hpp>
#include <dart/collision/experimental/broad_phase/spatial_hash.hpp>
#include <dart/collision/experimental/broad_phase/sweep_and_prune.hpp>

#include <benchmark/benchmark.h>

#include <memory>
#include <random>
#include <vector>

using namespace dart::collision::experimental;

namespace {

struct TestObject
{
  std::size_t id;
  Aabb aabb;
};

std::vector<TestObject> generateObjects(
    int numObjects, double worldSize, double objectSize, unsigned int seed)
{
  std::mt19937 rng(seed);
  std::uniform_real_distribution<double> posDist(-worldSize / 2, worldSize / 2);

  std::vector<TestObject> objects;
  objects.reserve(numObjects);

  for (int i = 0; i < numObjects; ++i) {
    double x = posDist(rng);
    double y = posDist(rng);
    double z = posDist(rng);

    objects.push_back(
        {static_cast<std::size_t>(i),
         Aabb(
             Eigen::Vector3d(x, y, z),
             Eigen::Vector3d(x + objectSize, y + objectSize, z + objectSize))});
  }

  return objects;
}

template <typename BroadPhaseT>
void populateBroadPhase(BroadPhaseT& bp, const std::vector<TestObject>& objects)
{
  for (const auto& obj : objects) {
    bp.add(obj.id, obj.aabb);
  }
}

} // namespace

static void BM_BruteForce_QueryPairs(benchmark::State& state)
{
  const int numObjects = state.range(0);
  auto objects = generateObjects(numObjects, 100.0, 2.0, 42);

  BruteForceBroadPhase bp;
  populateBroadPhase(bp, objects);

  for (auto _ : state) {
    auto pairs = bp.queryPairs();
    benchmark::DoNotOptimize(pairs);
  }

  state.SetItemsProcessed(state.iterations() * numObjects);
}

static void BM_AabbTree_QueryPairs(benchmark::State& state)
{
  const int numObjects = state.range(0);
  auto objects = generateObjects(numObjects, 100.0, 2.0, 42);

  AabbTreeBroadPhase bp;
  populateBroadPhase(bp, objects);

  for (auto _ : state) {
    auto pairs = bp.queryPairs();
    benchmark::DoNotOptimize(pairs);
  }

  state.SetItemsProcessed(state.iterations() * numObjects);
}

static void BM_SweepAndPrune_QueryPairs(benchmark::State& state)
{
  const int numObjects = state.range(0);
  auto objects = generateObjects(numObjects, 100.0, 2.0, 42);

  SweepAndPruneBroadPhase bp;
  populateBroadPhase(bp, objects);

  for (auto _ : state) {
    auto pairs = bp.queryPairs();
    benchmark::DoNotOptimize(pairs);
  }

  state.SetItemsProcessed(state.iterations() * numObjects);
}

static void BM_SpatialHash_QueryPairs(benchmark::State& state)
{
  const int numObjects = state.range(0);
  auto objects = generateObjects(numObjects, 100.0, 2.0, 42);

  SpatialHashBroadPhase bp(2.0);
  populateBroadPhase(bp, objects);

  for (auto _ : state) {
    auto pairs = bp.queryPairs();
    benchmark::DoNotOptimize(pairs);
  }

  state.SetItemsProcessed(state.iterations() * numObjects);
}

static void BM_BruteForce_Add(benchmark::State& state)
{
  const int numObjects = state.range(0);
  auto objects = generateObjects(numObjects, 100.0, 2.0, 42);

  for (auto _ : state) {
    BruteForceBroadPhase bp;
    populateBroadPhase(bp, objects);
    benchmark::DoNotOptimize(bp);
  }

  state.SetItemsProcessed(state.iterations() * numObjects);
}

static void BM_AabbTree_Add(benchmark::State& state)
{
  const int numObjects = state.range(0);
  auto objects = generateObjects(numObjects, 100.0, 2.0, 42);

  for (auto _ : state) {
    AabbTreeBroadPhase bp;
    populateBroadPhase(bp, objects);
    benchmark::DoNotOptimize(bp);
  }

  state.SetItemsProcessed(state.iterations() * numObjects);
}

static void BM_SweepAndPrune_Add(benchmark::State& state)
{
  const int numObjects = state.range(0);
  auto objects = generateObjects(numObjects, 100.0, 2.0, 42);

  for (auto _ : state) {
    SweepAndPruneBroadPhase bp;
    populateBroadPhase(bp, objects);
    benchmark::DoNotOptimize(bp);
  }

  state.SetItemsProcessed(state.iterations() * numObjects);
}

static void BM_SpatialHash_Add(benchmark::State& state)
{
  const int numObjects = state.range(0);
  auto objects = generateObjects(numObjects, 100.0, 2.0, 42);

  for (auto _ : state) {
    SpatialHashBroadPhase bp(2.0);
    populateBroadPhase(bp, objects);
    benchmark::DoNotOptimize(bp);
  }

  state.SetItemsProcessed(state.iterations() * numObjects);
}

static void BM_BruteForce_Update(benchmark::State& state)
{
  const int numObjects = state.range(0);
  auto objects = generateObjects(numObjects, 100.0, 2.0, 42);

  BruteForceBroadPhase bp;
  populateBroadPhase(bp, objects);

  std::mt19937 rng(123);
  std::uniform_real_distribution<double> offsetDist(-0.5, 0.5);

  for (auto _ : state) {
    for (const auto& obj : objects) {
      Eigen::Vector3d offset(offsetDist(rng), offsetDist(rng), offsetDist(rng));
      Aabb updated(obj.aabb.min + offset, obj.aabb.max + offset);
      bp.update(obj.id, updated);
    }
  }

  state.SetItemsProcessed(state.iterations() * numObjects);
}

static void BM_AabbTree_Update(benchmark::State& state)
{
  const int numObjects = state.range(0);
  auto objects = generateObjects(numObjects, 100.0, 2.0, 42);

  AabbTreeBroadPhase bp;
  populateBroadPhase(bp, objects);

  std::mt19937 rng(123);
  std::uniform_real_distribution<double> offsetDist(-0.5, 0.5);

  for (auto _ : state) {
    for (const auto& obj : objects) {
      Eigen::Vector3d offset(offsetDist(rng), offsetDist(rng), offsetDist(rng));
      Aabb updated(obj.aabb.min + offset, obj.aabb.max + offset);
      bp.update(obj.id, updated);
    }
  }

  state.SetItemsProcessed(state.iterations() * numObjects);
}

static void BM_SweepAndPrune_Update(benchmark::State& state)
{
  const int numObjects = state.range(0);
  auto objects = generateObjects(numObjects, 100.0, 2.0, 42);

  SweepAndPruneBroadPhase bp;
  populateBroadPhase(bp, objects);

  std::mt19937 rng(123);
  std::uniform_real_distribution<double> offsetDist(-0.5, 0.5);

  for (auto _ : state) {
    for (const auto& obj : objects) {
      Eigen::Vector3d offset(offsetDist(rng), offsetDist(rng), offsetDist(rng));
      Aabb updated(obj.aabb.min + offset, obj.aabb.max + offset);
      bp.update(obj.id, updated);
    }
  }

  state.SetItemsProcessed(state.iterations() * numObjects);
}

static void BM_SpatialHash_Update(benchmark::State& state)
{
  const int numObjects = state.range(0);
  auto objects = generateObjects(numObjects, 100.0, 2.0, 42);

  SpatialHashBroadPhase bp(2.0);
  populateBroadPhase(bp, objects);

  std::mt19937 rng(123);
  std::uniform_real_distribution<double> offsetDist(-0.5, 0.5);

  for (auto _ : state) {
    for (const auto& obj : objects) {
      Eigen::Vector3d offset(offsetDist(rng), offsetDist(rng), offsetDist(rng));
      Aabb updated(obj.aabb.min + offset, obj.aabb.max + offset);
      bp.update(obj.id, updated);
    }
  }

  state.SetItemsProcessed(state.iterations() * numObjects);
}

static void BM_BruteForce_QueryOverlapping(benchmark::State& state)
{
  const int numObjects = state.range(0);
  auto objects = generateObjects(numObjects, 100.0, 2.0, 42);

  BruteForceBroadPhase bp;
  populateBroadPhase(bp, objects);

  Aabb query(Eigen::Vector3d(-10, -10, -10), Eigen::Vector3d(10, 10, 10));

  for (auto _ : state) {
    auto results = bp.queryOverlapping(query);
    benchmark::DoNotOptimize(results);
  }

  state.SetItemsProcessed(state.iterations());
}

static void BM_AabbTree_QueryOverlapping(benchmark::State& state)
{
  const int numObjects = state.range(0);
  auto objects = generateObjects(numObjects, 100.0, 2.0, 42);

  AabbTreeBroadPhase bp;
  populateBroadPhase(bp, objects);

  Aabb query(Eigen::Vector3d(-10, -10, -10), Eigen::Vector3d(10, 10, 10));

  for (auto _ : state) {
    auto results = bp.queryOverlapping(query);
    benchmark::DoNotOptimize(results);
  }

  state.SetItemsProcessed(state.iterations());
}

static void BM_SweepAndPrune_QueryOverlapping(benchmark::State& state)
{
  const int numObjects = state.range(0);
  auto objects = generateObjects(numObjects, 100.0, 2.0, 42);

  SweepAndPruneBroadPhase bp;
  populateBroadPhase(bp, objects);

  Aabb query(Eigen::Vector3d(-10, -10, -10), Eigen::Vector3d(10, 10, 10));

  for (auto _ : state) {
    auto results = bp.queryOverlapping(query);
    benchmark::DoNotOptimize(results);
  }

  state.SetItemsProcessed(state.iterations());
}

static void BM_SpatialHash_QueryOverlapping(benchmark::State& state)
{
  const int numObjects = state.range(0);
  auto objects = generateObjects(numObjects, 100.0, 2.0, 42);

  SpatialHashBroadPhase bp(2.0);
  populateBroadPhase(bp, objects);

  Aabb query(Eigen::Vector3d(-10, -10, -10), Eigen::Vector3d(10, 10, 10));

  for (auto _ : state) {
    auto results = bp.queryOverlapping(query);
    benchmark::DoNotOptimize(results);
  }

  state.SetItemsProcessed(state.iterations());
}

BENCHMARK(BM_BruteForce_QueryPairs)
    ->Arg(10)
    ->Arg(50)
    ->Arg(100)
    ->Arg(500)
    ->Arg(1000);
BENCHMARK(BM_AabbTree_QueryPairs)
    ->Arg(10)
    ->Arg(50)
    ->Arg(100)
    ->Arg(500)
    ->Arg(1000);
BENCHMARK(BM_SweepAndPrune_QueryPairs)
    ->Arg(10)
    ->Arg(50)
    ->Arg(100)
    ->Arg(500)
    ->Arg(1000);
BENCHMARK(BM_SpatialHash_QueryPairs)
    ->Arg(10)
    ->Arg(50)
    ->Arg(100)
    ->Arg(500)
    ->Arg(1000);

BENCHMARK(BM_BruteForce_Add)->Arg(10)->Arg(100)->Arg(1000);
BENCHMARK(BM_AabbTree_Add)->Arg(10)->Arg(100)->Arg(1000);
BENCHMARK(BM_SweepAndPrune_Add)->Arg(10)->Arg(100)->Arg(1000);
BENCHMARK(BM_SpatialHash_Add)->Arg(10)->Arg(100)->Arg(1000);

BENCHMARK(BM_BruteForce_Update)->Arg(10)->Arg(100)->Arg(1000);
BENCHMARK(BM_AabbTree_Update)->Arg(10)->Arg(100)->Arg(1000);
BENCHMARK(BM_SweepAndPrune_Update)->Arg(10)->Arg(100)->Arg(1000);
BENCHMARK(BM_SpatialHash_Update)->Arg(10)->Arg(100)->Arg(1000);

BENCHMARK(BM_BruteForce_QueryOverlapping)->Arg(100)->Arg(1000)->Arg(10000);
BENCHMARK(BM_AabbTree_QueryOverlapping)->Arg(100)->Arg(1000)->Arg(10000);
BENCHMARK(BM_SweepAndPrune_QueryOverlapping)->Arg(100)->Arg(1000)->Arg(10000);
BENCHMARK(BM_SpatialHash_QueryOverlapping)->Arg(100)->Arg(1000)->Arg(10000);

static void BM_AabbTree_QueryPairs_ReusableBuffer(benchmark::State& state)
{
  const int numObjects = state.range(0);
  auto objects = generateObjects(numObjects, 100.0, 2.0, 42);

  AabbTreeBroadPhase bp;
  populateBroadPhase(bp, objects);

  std::vector<BroadPhasePair> pairs;
  pairs.reserve(numObjects * 10);

  for (auto _ : state) {
    bp.queryPairs(pairs);
    benchmark::DoNotOptimize(pairs.data());
  }

  state.SetItemsProcessed(state.iterations() * numObjects);
}

static void BM_AabbTree_Build_Bulk(benchmark::State& state)
{
  const int numObjects = state.range(0);
  auto objects = generateObjects(numObjects, 100.0, 2.0, 42);

  std::vector<std::size_t> ids;
  std::vector<Aabb> aabbs;
  ids.reserve(numObjects);
  aabbs.reserve(numObjects);
  for (const auto& obj : objects) {
    ids.push_back(obj.id);
    aabbs.push_back(obj.aabb);
  }

  for (auto _ : state) {
    AabbTreeBroadPhase bp;
    bp.build(ids, aabbs);
    benchmark::DoNotOptimize(bp);
  }

  state.SetItemsProcessed(state.iterations() * numObjects);
}

static void BM_AabbTree_UpdateRange_Bulk(benchmark::State& state)
{
  const int numObjects = state.range(0);
  auto objects = generateObjects(numObjects, 100.0, 2.0, 42);

  AabbTreeBroadPhase bp;
  populateBroadPhase(bp, objects);

  std::vector<std::size_t> ids;
  std::vector<Aabb> aabbs;
  ids.reserve(numObjects);
  aabbs.reserve(numObjects);

  std::mt19937 rng(123);
  std::uniform_real_distribution<double> offsetDist(-0.5, 0.5);

  for (auto _ : state) {
    ids.clear();
    aabbs.clear();
    for (const auto& obj : objects) {
      Eigen::Vector3d offset(offsetDist(rng), offsetDist(rng), offsetDist(rng));
      ids.push_back(obj.id);
      aabbs.emplace_back(obj.aabb.min + offset, obj.aabb.max + offset);
    }
    bp.updateRange(ids, aabbs);
  }

  state.SetItemsProcessed(state.iterations() * numObjects);
}

BENCHMARK(BM_AabbTree_QueryPairs_ReusableBuffer)
    ->Arg(10)
    ->Arg(50)
    ->Arg(100)
    ->Arg(500)
    ->Arg(1000);
BENCHMARK(BM_AabbTree_Build_Bulk)->Arg(10)->Arg(100)->Arg(1000);
BENCHMARK(BM_AabbTree_UpdateRange_Bulk)->Arg(10)->Arg(100)->Arg(1000);

BENCHMARK_MAIN();
