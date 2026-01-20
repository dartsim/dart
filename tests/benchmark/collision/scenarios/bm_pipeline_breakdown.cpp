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

#include "tests/benchmark/collision/fixtures/scene_builders.hpp"

#include <dart/collision/experimental/collision_world.hpp>
#include <dart/collision/experimental/shapes/shape.hpp>

#include <benchmark/benchmark.h>

#include <algorithm>
#include <chrono>
#include <memory>
#include <vector>

using namespace dart::collision::experimental;

namespace {

enum class ShapeKind
{
  Sphere,
  Box,
  Capsule,
};

struct ShapeSpec
{
  ShapeKind kind;
  Eigen::Isometry3d transform;
};

constexpr double kSphereRadius = 0.5;
constexpr double kCapsuleRadius = 0.4;
constexpr double kCapsuleHeight = 1.2;
constexpr double kDenseRange = 2.0;
constexpr double kSparseRange = 10.0;
constexpr double kRp3dDenseRange = 10.0;
constexpr double kRp3dSparseRange = 50.0;
const Eigen::Vector3d kBoxHalfExtents(0.5, 0.5, 0.5);

std::size_t MaxContactsForCount(std::size_t count)
{
  return std::max<std::size_t>(1000, count * 10);
}

std::vector<ShapeSpec> MakeMixedScene(
    std::size_t count, double range, unsigned int seed)
{
  auto rng = dart::benchmark::collision::MakeDeterministicRng(seed);
  std::vector<ShapeSpec> specs;
  specs.reserve(count);

  for (std::size_t i = 0; i < count; ++i) {
    ShapeSpec spec;
    switch (i % 3) {
      case 0:
        spec.kind = ShapeKind::Sphere;
        break;
      case 1:
        spec.kind = ShapeKind::Box;
        break;
      default:
        spec.kind = ShapeKind::Capsule;
        break;
    }
    spec.transform
        = dart::benchmark::collision::RandomTransformWithRotation(rng, range);
    specs.push_back(spec);
  }

  return specs;
}

std::vector<ShapeSpec> MakeSphereScene(
    std::size_t count, double range, unsigned int seed)
{
  auto rng = dart::benchmark::collision::MakeDeterministicRng(seed);
  std::vector<ShapeSpec> specs;
  specs.reserve(count);

  for (std::size_t i = 0; i < count; ++i) {
    ShapeSpec spec;
    spec.kind = ShapeKind::Sphere;
    spec.transform = dart::benchmark::collision::RandomTransform(rng, range);
    specs.push_back(spec);
  }

  return specs;
}

std::unique_ptr<Shape> MakeExperimentalShape(const ShapeSpec& spec)
{
  switch (spec.kind) {
    case ShapeKind::Sphere:
      return std::make_unique<SphereShape>(kSphereRadius);
    case ShapeKind::Box:
      return std::make_unique<BoxShape>(kBoxHalfExtents);
    case ShapeKind::Capsule:
      return std::make_unique<CapsuleShape>(kCapsuleRadius, kCapsuleHeight);
  }

  return std::make_unique<SphereShape>(kSphereRadius);
}

void BuildExperimentalWorld(
    const std::vector<ShapeSpec>& specs,
    CollisionWorld& world,
    std::vector<CollisionObject>& objects,
    std::vector<Eigen::Isometry3d>& baseTransforms)
{
  objects.reserve(specs.size());
  baseTransforms.reserve(specs.size());

  for (const auto& spec : specs) {
    baseTransforms.push_back(spec.transform);
    objects.emplace_back(
        world.createObject(MakeExperimentalShape(spec), spec.transform));
  }
}

using SceneBuilder = std::vector<ShapeSpec> (*)(
    std::size_t count, double range, unsigned int seed);

void RunPipelineBenchmark(
    benchmark::State& state, double range, SceneBuilder buildScene)
{
  using Clock = std::chrono::steady_clock;

  const std::size_t count = static_cast<std::size_t>(state.range(0));
  auto specs = buildScene(count, range, 42);

  CollisionWorld world;
  std::vector<CollisionObject> objects;
  std::vector<Eigen::Isometry3d> baseTransforms;
  BuildExperimentalWorld(specs, world, objects, baseTransforms);

  CollisionOption option
      = CollisionOption::fullContacts(MaxContactsForCount(count));
  CollisionResult result;
  BroadPhaseSnapshot snapshot;
  BatchStats stats;

  std::uint64_t updateNs = 0;
  std::uint64_t broadPhaseNs = 0;
  std::uint64_t narrowPhaseNs = 0;
  std::uint64_t mergeNs = 0;
  std::size_t totalPairs = 0;
  std::size_t totalPairsTested = 0;
  std::size_t totalContacts = 0;
  std::size_t totalPairBytes = 0;
  std::size_t totalContactBytes = 0;
  std::size_t totalTempBytes = 0;
  std::size_t totalUpdated = 0;

  std::size_t iteration = 0;
  for (auto _ : state) {
    const double shift = 0.001 * static_cast<double>(iteration + 1);

    auto startUpdate = Clock::now();
    for (std::size_t i = 0; i < objects.size(); ++i) {
      Eigen::Isometry3d tf = baseTransforms[i];
      tf.translation().x() += shift;
      objects[i].setTransform(tf);
    }
    totalUpdated += world.updateAll();
    auto endUpdate = Clock::now();
    updateNs += std::chrono::duration_cast<std::chrono::nanoseconds>(
                    endUpdate - startUpdate)
                    .count();

    auto startBroadPhase = Clock::now();
    snapshot = world.buildBroadPhaseSnapshot();
    auto endBroadPhase = Clock::now();
    broadPhaseNs += std::chrono::duration_cast<std::chrono::nanoseconds>(
                        endBroadPhase - startBroadPhase)
                        .count();

    auto startNarrowPhase = Clock::now();
    stats.clear();
    world.collideAll(snapshot, option, result, &stats);
    auto endNarrowPhase = Clock::now();
    narrowPhaseNs += std::chrono::duration_cast<std::chrono::nanoseconds>(
                         endNarrowPhase - startNarrowPhase)
                         .count();

    auto startMerge = Clock::now();
    benchmark::DoNotOptimize(result.numContacts());
    auto endMerge = Clock::now();
    mergeNs += std::chrono::duration_cast<std::chrono::nanoseconds>(
                   endMerge - startMerge)
                   .count();

    totalPairs += snapshot.pairs.size();
    totalPairsTested += stats.numPairsTested;
    totalContacts += result.numContacts();
    totalPairBytes += stats.pairBytes;
    totalContactBytes += stats.contactBytes;
    totalTempBytes += stats.tempBytes;

    ++iteration;
  }

  state.SetComplexityN(count);
  state.counters["objects"] = static_cast<double>(count);
  state.counters["aabb_update_ns"]
      = benchmark::Counter(updateNs, benchmark::Counter::kAvgIterations);
  state.counters["broadphase_ns"]
      = benchmark::Counter(broadPhaseNs, benchmark::Counter::kAvgIterations);
  state.counters["narrowphase_ns"]
      = benchmark::Counter(narrowPhaseNs, benchmark::Counter::kAvgIterations);
  state.counters["merge_ns"]
      = benchmark::Counter(mergeNs, benchmark::Counter::kAvgIterations);
  state.counters["pairs"]
      = benchmark::Counter(totalPairs, benchmark::Counter::kAvgIterations);
  state.counters["aabb_updates"]
      = benchmark::Counter(totalUpdated, benchmark::Counter::kAvgIterations);
  state.counters["pairs_tested"] = benchmark::Counter(
      totalPairsTested, benchmark::Counter::kAvgIterations);
  state.counters["contacts"]
      = benchmark::Counter(totalContacts, benchmark::Counter::kAvgIterations);
  state.counters["pair_bytes"]
      = benchmark::Counter(totalPairBytes, benchmark::Counter::kAvgIterations);
  state.counters["contact_bytes"] = benchmark::Counter(
      totalContactBytes, benchmark::Counter::kAvgIterations);
  state.counters["temp_bytes"]
      = benchmark::Counter(totalTempBytes, benchmark::Counter::kAvgIterations);
}

} // namespace

static void BM_Scenario_PipelineBreakdown_Dense_Experimental(
    benchmark::State& state)
{
  RunPipelineBenchmark(state, kDenseRange, MakeMixedScene);
}
BENCHMARK(BM_Scenario_PipelineBreakdown_Dense_Experimental)
    ->Arg(1000)
    ->Arg(10000)
    ->Complexity();

static void BM_Scenario_PipelineBreakdown_Sparse_Experimental(
    benchmark::State& state)
{
  RunPipelineBenchmark(state, kSparseRange, MakeMixedScene);
}
BENCHMARK(BM_Scenario_PipelineBreakdown_Sparse_Experimental)
    ->Arg(1000)
    ->Arg(10000)
    ->Complexity();

static void BM_Scenario_PipelineBreakdown_RP3D_Dense_Spheres_Experimental(
    benchmark::State& state)
{
  RunPipelineBenchmark(state, kRp3dDenseRange, MakeSphereScene);
}
BENCHMARK(BM_Scenario_PipelineBreakdown_RP3D_Dense_Spheres_Experimental)
    ->Arg(1000)
    ->Arg(10000)
    ->Complexity();

static void BM_Scenario_PipelineBreakdown_RP3D_Sparse_Spheres_Experimental(
    benchmark::State& state)
{
  RunPipelineBenchmark(state, kRp3dSparseRange, MakeSphereScene);
}
BENCHMARK(BM_Scenario_PipelineBreakdown_RP3D_Sparse_Spheres_Experimental)
    ->Arg(1000)
    ->Arg(10000)
    ->Complexity();

BENCHMARK_MAIN();
