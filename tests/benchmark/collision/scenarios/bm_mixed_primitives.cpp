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

#include <dart/config.hpp>

#include <dart/collision/CollisionOption.hpp>
#include <dart/collision/CollisionResult.hpp>
#include <dart/collision/experimental/collision_world.hpp>
#include <dart/collision/experimental/shapes/shape.hpp>
#include <dart/collision/fcl/FCLCollisionDetector.hpp>

#include <dart/dynamics/BoxShape.hpp>
#include <dart/dynamics/CapsuleShape.hpp>
#include <dart/dynamics/FreeJoint.hpp>
#include <dart/dynamics/ShapeNode.hpp>
#include <dart/dynamics/Skeleton.hpp>
#include <dart/dynamics/SphereShape.hpp>

#if DART_HAVE_BULLET
  #include <dart/collision/bullet/BulletCollisionDetector.hpp>
#endif

#if DART_HAVE_ODE
  #include <dart/collision/ode/OdeCollisionDetector.hpp>
#endif

#include "tests/benchmark/collision/fixtures/scene_builders.hpp"
#include "tests/benchmark/collision/fixtures/shape_factories.hpp"

#include <benchmark/benchmark.h>

#include <algorithm>
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

struct ChildShapeSpec
{
  ShapeKind kind;
  Eigen::Isometry3d relativeTransform;
};

constexpr double kSphereRadius = 0.5;
constexpr double kCapsuleRadius = 0.4;
constexpr double kCapsuleHeight = 1.2;
constexpr double kDenseRange = 2.0;
constexpr double kSparseRange = 10.0;
constexpr double kAggregateOffset = 1.6;
const Eigen::Vector3d kBoxHalfExtents(0.5, 0.5, 0.5);
const Eigen::Vector3d kBoxSize(1.0, 1.0, 1.0);

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

std::shared_ptr<dart::dynamics::Shape> MakeDynamicsShape(const ShapeSpec& spec)
{
  switch (spec.kind) {
    case ShapeKind::Sphere:
      return std::make_shared<dart::dynamics::SphereShape>(kSphereRadius);
    case ShapeKind::Box:
      return std::make_shared<dart::dynamics::BoxShape>(kBoxSize);
    case ShapeKind::Capsule:
      return std::make_shared<dart::dynamics::CapsuleShape>(
          kCapsuleRadius, kCapsuleHeight);
  }

  return std::make_shared<dart::dynamics::SphereShape>(kSphereRadius);
}

void BuildExperimentalWorld(
    const std::vector<ShapeSpec>& specs,
    CollisionWorld& world,
    std::vector<CollisionObject>& objects)
{
  objects.reserve(specs.size());

  for (const auto& spec : specs) {
    objects.emplace_back(
        world.createObject(MakeExperimentalShape(spec), spec.transform));
  }
}

template <typename DetectorPtr>
void BuildCollisionGroup(
    const DetectorPtr& detector,
    const std::vector<ShapeSpec>& specs,
    std::vector<dart::dynamics::SkeletonPtr>& skeletons,
    dart::collision::CollisionGroupPtr& group)
{
  skeletons.reserve(specs.size());
  group = detector->createCollisionGroup();

  for (const auto& spec : specs) {
    auto shape = MakeDynamicsShape(spec);
    auto skel = dart::benchmark::collision::CreateSingleShapeSkeleton(
        shape, spec.transform);
    skeletons.push_back(skel);
    dart::benchmark::collision::AddSkeletonToGroup(group.get(), skel);
  }
}

void RunExperimentalScenarioBenchmark(benchmark::State& state, double range)
{
  const std::size_t count = static_cast<std::size_t>(state.range(0));
  auto specs = MakeMixedScene(count, range, 42);

  CollisionWorld world;
  std::vector<CollisionObject> objects;
  BuildExperimentalWorld(specs, world, objects);

  CollisionOption option
      = CollisionOption::fullContacts(MaxContactsForCount(count));
  CollisionResult result;

  for (auto _ : state) {
    result.clear();
    benchmark::DoNotOptimize(world.collide(option, result));
    benchmark::DoNotOptimize(result.numContacts());
  }

  state.SetComplexityN(count);
}

template <typename DetectorPtr>
void RunDetectorScenarioBenchmark(
    benchmark::State& state, const DetectorPtr& detector, double range)
{
  const std::size_t count = static_cast<std::size_t>(state.range(0));
  auto specs = MakeMixedScene(count, range, 42);

  std::vector<dart::dynamics::SkeletonPtr> skeletons;
  dart::collision::CollisionGroupPtr group;
  BuildCollisionGroup(detector, specs, skeletons, group);

  dart::collision::CollisionOption option;
  option.maxNumContacts = MaxContactsForCount(count);
  dart::collision::CollisionResult result;

  for (auto _ : state) {
    result.clear();
    benchmark::DoNotOptimize(detector->collide(group.get(), option, &result));
    benchmark::DoNotOptimize(result.getNumContacts());
  }

  state.SetComplexityN(count);
}

} // namespace

static void BM_Scenario_MixedPrimitives_Dense_Experimental(
    benchmark::State& state)
{
  RunExperimentalScenarioBenchmark(state, kDenseRange);
}
BENCHMARK(BM_Scenario_MixedPrimitives_Dense_Experimental)
    ->Arg(100)
    ->Arg(1000)
    ->Arg(10000)
    ->Complexity();

static void BM_Scenario_MixedPrimitives_Sparse_Experimental(
    benchmark::State& state)
{
  RunExperimentalScenarioBenchmark(state, kSparseRange);
}
BENCHMARK(BM_Scenario_MixedPrimitives_Sparse_Experimental)
    ->Arg(100)
    ->Arg(1000)
    ->Arg(10000)
    ->Complexity();

static void BM_Scenario_MixedPrimitives_Dense_FCL(benchmark::State& state)
{
  auto detector = dart::collision::FCLCollisionDetector::create();
  RunDetectorScenarioBenchmark(state, detector, kDenseRange);
}
BENCHMARK(BM_Scenario_MixedPrimitives_Dense_FCL)
    ->Arg(100)
    ->Arg(1000)
    ->Arg(10000)
    ->Complexity();

static void BM_Scenario_MixedPrimitives_Sparse_FCL(benchmark::State& state)
{
  auto detector = dart::collision::FCLCollisionDetector::create();
  RunDetectorScenarioBenchmark(state, detector, kSparseRange);
}
BENCHMARK(BM_Scenario_MixedPrimitives_Sparse_FCL)
    ->Arg(100)
    ->Arg(1000)
    ->Arg(10000)
    ->Complexity();

#if DART_HAVE_BULLET
static void BM_Scenario_MixedPrimitives_Dense_Bullet(benchmark::State& state)
{
  auto detector = dart::collision::BulletCollisionDetector::create();
  RunDetectorScenarioBenchmark(state, detector, kDenseRange);
}
BENCHMARK(BM_Scenario_MixedPrimitives_Dense_Bullet)
    ->Arg(100)
    ->Arg(1000)
    ->Arg(10000)
    ->Complexity();

static void BM_Scenario_MixedPrimitives_Sparse_Bullet(benchmark::State& state)
{
  auto detector = dart::collision::BulletCollisionDetector::create();
  RunDetectorScenarioBenchmark(state, detector, kSparseRange);
}
BENCHMARK(BM_Scenario_MixedPrimitives_Sparse_Bullet)
    ->Arg(100)
    ->Arg(1000)
    ->Arg(10000)
    ->Complexity();
#endif

#if DART_HAVE_ODE
static void BM_Scenario_MixedPrimitives_Dense_ODE(benchmark::State& state)
{
  auto detector = dart::collision::OdeCollisionDetector::create();
  RunDetectorScenarioBenchmark(state, detector, kDenseRange);
}
BENCHMARK(BM_Scenario_MixedPrimitives_Dense_ODE)
    ->Arg(100)
    ->Arg(1000)
    ->Arg(10000)
    ->Complexity();

static void BM_Scenario_MixedPrimitives_Sparse_ODE(benchmark::State& state)
{
  auto detector = dart::collision::OdeCollisionDetector::create();
  RunDetectorScenarioBenchmark(state, detector, kSparseRange);
}
BENCHMARK(BM_Scenario_MixedPrimitives_Sparse_ODE)
    ->Arg(100)
    ->Arg(1000)
    ->Arg(10000)
    ->Complexity();
#endif

BENCHMARK_MAIN();
