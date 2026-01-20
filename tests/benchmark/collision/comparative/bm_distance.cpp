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

#include <dart/collision/DistanceOption.hpp>
#include <dart/collision/DistanceResult.hpp>
#include <dart/collision/experimental/narrow_phase/distance.hpp>
#include <dart/collision/experimental/shapes/shape.hpp>
#include <dart/collision/fcl/FCLCollisionDetector.hpp>

#include <dart/dynamics/BoxShape.hpp>
#include <dart/dynamics/CapsuleShape.hpp>
#include <dart/dynamics/FreeJoint.hpp>
#include <dart/dynamics/Skeleton.hpp>
#include <dart/dynamics/SphereShape.hpp>

#if DART_HAVE_BULLET
  #include <dart/collision/bullet/BulletCollisionDetector.hpp>
#endif

#if DART_HAVE_ODE
  #include <dart/collision/ode/OdeCollisionDetector.hpp>
#endif

#include "tests/benchmark/collision/fixtures/edge_cases.hpp"
#include "tests/benchmark/collision/fixtures/shape_factories.hpp"

#include <benchmark/benchmark.h>

#include <iostream>
#include <memory>

#include <cmath>

using namespace dart::collision::experimental;

namespace {

struct PairContext
{
  dart::dynamics::SkeletonPtr skel1;
  dart::dynamics::SkeletonPtr skel2;
  dart::collision::CollisionGroupPtr group;
};

template <typename DetectorPtr>
PairContext MakePairContext(
    const DetectorPtr& detector,
    const std::shared_ptr<dart::dynamics::Shape>& shape1,
    const Eigen::Isometry3d& tf1,
    const std::shared_ptr<dart::dynamics::Shape>& shape2,
    const Eigen::Isometry3d& tf2)
{
  PairContext ctx;
  ctx.skel1
      = dart::benchmark::collision::CreateSingleShapeSkeleton(shape1, tf1);
  ctx.skel2
      = dart::benchmark::collision::CreateSingleShapeSkeleton(shape2, tf2);
  ctx.group = detector->createCollisionGroup();
  dart::benchmark::collision::AddSkeletonToGroup(ctx.group.get(), ctx.skel1);
  dart::benchmark::collision::AddSkeletonToGroup(ctx.group.get(), ctx.skel2);
  return ctx;
}

template <typename DetectorPtr>
void RunDistanceDetectorBenchmark(
    benchmark::State& state,
    const DetectorPtr& detector,
    const std::shared_ptr<dart::dynamics::Shape>& shape1,
    const Eigen::Isometry3d& tf1,
    const std::shared_ptr<dart::dynamics::Shape>& shape2,
    const Eigen::Isometry3d& tf2,
    const dart::collision::DistanceOption& option)
{
  auto ctx = MakePairContext(detector, shape1, tf1, shape2, tf2);
  dart::collision::DistanceResult result;

  for (auto _ : state) {
    result.clear();
    double dist = detector->distance(ctx.group.get(), option, &result);
    benchmark::DoNotOptimize(dist);
  }
}

Eigen::Isometry3d OffsetTransform(double x, double y, double z)
{
  return dart::benchmark::collision::MakeTransform(Eigen::Vector3d(x, y, z));
}

DistanceOption MakeExperimentalDistanceOption()
{
  DistanceOption option = DistanceOption::unlimited();
  option.enableNearestPoints = true;
  return option;
}

} // namespace

static void BM_Distance_SphereSphere_Experimental(benchmark::State& state)
{
  SphereShape s1(1.0);
  SphereShape s2(1.0);

  const Eigen::Isometry3d tf1 = Eigen::Isometry3d::Identity();
  const Eigen::Isometry3d tf2 = OffsetTransform(5.0, 0.0, 0.0);

  DistanceResult result;
  DistanceOption option = MakeExperimentalDistanceOption();

  for (auto _ : state) {
    result.clear();
    double dist = distanceSphereSphere(s1, tf1, s2, tf2, result, option);
    benchmark::DoNotOptimize(dist);
  }
}
BENCHMARK(BM_Distance_SphereSphere_Experimental);

static void BM_Distance_SphereSphere_FCL(benchmark::State& state)
{
  auto detector = dart::collision::FCLCollisionDetector::create();

  auto shape1 = std::make_shared<dart::dynamics::SphereShape>(1.0);
  auto shape2 = std::make_shared<dart::dynamics::SphereShape>(1.0);

  const Eigen::Isometry3d tf1 = Eigen::Isometry3d::Identity();
  const Eigen::Isometry3d tf2 = OffsetTransform(5.0, 0.0, 0.0);

  const auto option = dart::benchmark::collision::MakeDistanceOption();
  RunDistanceDetectorBenchmark(
      state, detector, shape1, tf1, shape2, tf2, option);
}
BENCHMARK(BM_Distance_SphereSphere_FCL);

#if DART_HAVE_BULLET
static void BM_Distance_SphereSphere_Bullet(benchmark::State& state)
{
  auto detector = dart::collision::BulletCollisionDetector::create();

  auto shape1 = std::make_shared<dart::dynamics::SphereShape>(1.0);
  auto shape2 = std::make_shared<dart::dynamics::SphereShape>(1.0);

  const Eigen::Isometry3d tf1 = Eigen::Isometry3d::Identity();
  const Eigen::Isometry3d tf2 = OffsetTransform(5.0, 0.0, 0.0);

  const auto option = dart::benchmark::collision::MakeDistanceOption();
  RunDistanceDetectorBenchmark(
      state, detector, shape1, tf1, shape2, tf2, option);
}
BENCHMARK(BM_Distance_SphereSphere_Bullet);
#endif

#if DART_HAVE_ODE
static void BM_Distance_SphereSphere_ODE(benchmark::State& state)
{
  auto detector = dart::collision::OdeCollisionDetector::create();

  auto shape1 = std::make_shared<dart::dynamics::SphereShape>(1.0);
  auto shape2 = std::make_shared<dart::dynamics::SphereShape>(1.0);

  const Eigen::Isometry3d tf1 = Eigen::Isometry3d::Identity();
  const Eigen::Isometry3d tf2 = OffsetTransform(5.0, 0.0, 0.0);

  const auto option = dart::benchmark::collision::MakeDistanceOption();
  RunDistanceDetectorBenchmark(
      state, detector, shape1, tf1, shape2, tf2, option);
}
BENCHMARK(BM_Distance_SphereSphere_ODE);
#endif

static void BM_Distance_BoxBox_Experimental(benchmark::State& state)
{
  BoxShape b1(Eigen::Vector3d(0.5, 0.5, 0.5));
  BoxShape b2(Eigen::Vector3d(0.5, 0.5, 0.5));

  const Eigen::Isometry3d tf1 = Eigen::Isometry3d::Identity();
  const Eigen::Isometry3d tf2 = OffsetTransform(3.0, 0.0, 0.0);

  DistanceResult result;
  DistanceOption option = MakeExperimentalDistanceOption();

  for (auto _ : state) {
    result.clear();
    double dist = distanceBoxBox(b1, tf1, b2, tf2, result, option);
    benchmark::DoNotOptimize(dist);
  }
}
BENCHMARK(BM_Distance_BoxBox_Experimental);

static void BM_Distance_BoxBox_FCL(benchmark::State& state)
{
  auto detector = dart::collision::FCLCollisionDetector::create();

  auto shape1 = std::make_shared<dart::dynamics::BoxShape>(
      Eigen::Vector3d(1.0, 1.0, 1.0));
  auto shape2 = std::make_shared<dart::dynamics::BoxShape>(
      Eigen::Vector3d(1.0, 1.0, 1.0));

  const Eigen::Isometry3d tf1 = Eigen::Isometry3d::Identity();
  const Eigen::Isometry3d tf2 = OffsetTransform(3.0, 0.0, 0.0);

  const auto option = dart::benchmark::collision::MakeDistanceOption();
  RunDistanceDetectorBenchmark(
      state, detector, shape1, tf1, shape2, tf2, option);
}
BENCHMARK(BM_Distance_BoxBox_FCL);

#if DART_HAVE_BULLET
static void BM_Distance_BoxBox_Bullet(benchmark::State& state)
{
  auto detector = dart::collision::BulletCollisionDetector::create();

  auto shape1 = std::make_shared<dart::dynamics::BoxShape>(
      Eigen::Vector3d(1.0, 1.0, 1.0));
  auto shape2 = std::make_shared<dart::dynamics::BoxShape>(
      Eigen::Vector3d(1.0, 1.0, 1.0));

  const Eigen::Isometry3d tf1 = Eigen::Isometry3d::Identity();
  const Eigen::Isometry3d tf2 = OffsetTransform(3.0, 0.0, 0.0);

  const auto option = dart::benchmark::collision::MakeDistanceOption();
  RunDistanceDetectorBenchmark(
      state, detector, shape1, tf1, shape2, tf2, option);
}
BENCHMARK(BM_Distance_BoxBox_Bullet);
#endif

#if DART_HAVE_ODE
static void BM_Distance_BoxBox_ODE(benchmark::State& state)
{
  auto detector = dart::collision::OdeCollisionDetector::create();

  auto shape1 = std::make_shared<dart::dynamics::BoxShape>(
      Eigen::Vector3d(1.0, 1.0, 1.0));
  auto shape2 = std::make_shared<dart::dynamics::BoxShape>(
      Eigen::Vector3d(1.0, 1.0, 1.0));

  const Eigen::Isometry3d tf1 = Eigen::Isometry3d::Identity();
  const Eigen::Isometry3d tf2 = OffsetTransform(3.0, 0.0, 0.0);

  const auto option = dart::benchmark::collision::MakeDistanceOption();
  RunDistanceDetectorBenchmark(
      state, detector, shape1, tf1, shape2, tf2, option);
}
BENCHMARK(BM_Distance_BoxBox_ODE);
#endif

static void BM_Distance_CapsuleCapsule_Experimental(benchmark::State& state)
{
  CapsuleShape c1(0.5, 2.0);
  CapsuleShape c2(0.5, 2.0);

  const Eigen::Isometry3d tf1 = Eigen::Isometry3d::Identity();
  const Eigen::Isometry3d tf2 = OffsetTransform(3.0, 0.0, 0.0);

  DistanceResult result;
  DistanceOption option = MakeExperimentalDistanceOption();

  for (auto _ : state) {
    result.clear();
    double dist = distanceCapsuleCapsule(c1, tf1, c2, tf2, result, option);
    benchmark::DoNotOptimize(dist);
  }
}
BENCHMARK(BM_Distance_CapsuleCapsule_Experimental);

static void BM_Distance_CapsuleCapsule_FCL(benchmark::State& state)
{
  auto detector = dart::collision::FCLCollisionDetector::create();

  auto shape1 = std::make_shared<dart::dynamics::CapsuleShape>(0.5, 2.0);
  auto shape2 = std::make_shared<dart::dynamics::CapsuleShape>(0.5, 2.0);

  const Eigen::Isometry3d tf1 = Eigen::Isometry3d::Identity();
  const Eigen::Isometry3d tf2 = OffsetTransform(3.0, 0.0, 0.0);

  const auto option = dart::benchmark::collision::MakeDistanceOption();
  RunDistanceDetectorBenchmark(
      state, detector, shape1, tf1, shape2, tf2, option);
}
BENCHMARK(BM_Distance_CapsuleCapsule_FCL);

#if DART_HAVE_BULLET
static void BM_Distance_CapsuleCapsule_Bullet(benchmark::State& state)
{
  auto detector = dart::collision::BulletCollisionDetector::create();

  auto shape1 = std::make_shared<dart::dynamics::CapsuleShape>(0.5, 2.0);
  auto shape2 = std::make_shared<dart::dynamics::CapsuleShape>(0.5, 2.0);

  const Eigen::Isometry3d tf1 = Eigen::Isometry3d::Identity();
  const Eigen::Isometry3d tf2 = OffsetTransform(3.0, 0.0, 0.0);

  const auto option = dart::benchmark::collision::MakeDistanceOption();
  RunDistanceDetectorBenchmark(
      state, detector, shape1, tf1, shape2, tf2, option);
}
BENCHMARK(BM_Distance_CapsuleCapsule_Bullet);
#endif

#if DART_HAVE_ODE
static void BM_Distance_CapsuleCapsule_ODE(benchmark::State& state)
{
  auto detector = dart::collision::OdeCollisionDetector::create();

  auto shape1 = std::make_shared<dart::dynamics::CapsuleShape>(0.5, 2.0);
  auto shape2 = std::make_shared<dart::dynamics::CapsuleShape>(0.5, 2.0);

  const Eigen::Isometry3d tf1 = Eigen::Isometry3d::Identity();
  const Eigen::Isometry3d tf2 = OffsetTransform(3.0, 0.0, 0.0);

  const auto option = dart::benchmark::collision::MakeDistanceOption();
  RunDistanceDetectorBenchmark(
      state, detector, shape1, tf1, shape2, tf2, option);
}
BENCHMARK(BM_Distance_CapsuleCapsule_ODE);
#endif

namespace edge_case_bench {

using dart::benchmark::collision::EdgeCase;
using dart::benchmark::collision::MakeBoxBoxTransforms;
using dart::benchmark::collision::MakeBoxSpec;
using dart::benchmark::collision::MakeCapsuleBoxTransforms;
using dart::benchmark::collision::MakeCapsuleCapsuleTransforms;
using dart::benchmark::collision::MakeCapsuleSpec;
using dart::benchmark::collision::MakeCapsuleSphereTransforms;
using dart::benchmark::collision::MakeSphereBoxTransforms;
using dart::benchmark::collision::MakeSphereSpec;
using dart::benchmark::collision::MakeSphereSphereTransforms;
using dart::benchmark::collision::PairKind;
using dart::benchmark::collision::ScaleFromIndex;

void AddScaleArgs(benchmark::internal::Benchmark* bench)
{
  bench->Arg(0)->Arg(1)->Arg(2);
}

void RunDistanceCaseExperimental(
    benchmark::State& state, PairKind pair, EdgeCase edge)
{
  const double scale = ScaleFromIndex(static_cast<int>(state.range(0)));
  const auto sphereSpec = MakeSphereSpec(scale);
  const auto capsuleSpec = MakeCapsuleSpec(scale);
  const auto boxSpec = MakeBoxSpec(scale, edge == EdgeCase::kThinFeature);

  DistanceResult result;
  const auto option = MakeExperimentalDistanceOption();

  switch (pair) {
    case PairKind::kSphereSphere: {
      SphereShape s1(sphereSpec.radius);
      SphereShape s2(sphereSpec.radius);
      const auto tfs = MakeSphereSphereTransforms(
          sphereSpec.radius, sphereSpec.radius, edge);

      for (auto _ : state) {
        result.clear();
        const double dist
            = distanceSphereSphere(s1, tfs.tf1, s2, tfs.tf2, result, option);
        benchmark::DoNotOptimize(dist);
      }
      return;
    }
    case PairKind::kBoxBox: {
      BoxShape b1(boxSpec.halfExtents);
      BoxShape b2(boxSpec.halfExtents);
      const auto tfs = MakeBoxBoxTransforms(
          boxSpec.halfExtents, boxSpec.halfExtents, edge);

      for (auto _ : state) {
        result.clear();
        const double dist
            = distanceBoxBox(b1, tfs.tf1, b2, tfs.tf2, result, option);
        benchmark::DoNotOptimize(dist);
      }
      return;
    }
    case PairKind::kCapsuleCapsule: {
      CapsuleShape c1(capsuleSpec.radius, capsuleSpec.height);
      CapsuleShape c2(capsuleSpec.radius, capsuleSpec.height);
      const auto tfs = MakeCapsuleCapsuleTransforms(
          capsuleSpec.radius, capsuleSpec.radius, edge);

      for (auto _ : state) {
        result.clear();
        const double dist
            = distanceCapsuleCapsule(c1, tfs.tf1, c2, tfs.tf2, result, option);
        benchmark::DoNotOptimize(dist);
      }
      return;
    }
    case PairKind::kSphereBox: {
      SphereShape sphere(sphereSpec.radius);
      BoxShape box(boxSpec.halfExtents);
      const auto tfs = MakeSphereBoxTransforms(
          sphereSpec.radius, boxSpec.halfExtents, edge);

      for (auto _ : state) {
        result.clear();
        const double dist
            = distanceSphereBox(sphere, tfs.tf1, box, tfs.tf2, result, option);
        benchmark::DoNotOptimize(dist);
      }
      return;
    }
    case PairKind::kCapsuleSphere: {
      CapsuleShape capsule(capsuleSpec.radius, capsuleSpec.height);
      SphereShape sphere(sphereSpec.radius);
      const auto tfs = MakeCapsuleSphereTransforms(
          capsuleSpec.radius, sphereSpec.radius, edge);

      for (auto _ : state) {
        result.clear();
        const double dist = distanceCapsuleSphere(
            capsule, tfs.tf1, sphere, tfs.tf2, result, option);
        benchmark::DoNotOptimize(dist);
      }
      return;
    }
    case PairKind::kCapsuleBox: {
      CapsuleShape capsule(capsuleSpec.radius, capsuleSpec.height);
      BoxShape box(boxSpec.halfExtents);
      const auto tfs = MakeCapsuleBoxTransforms(
          capsuleSpec.radius, boxSpec.halfExtents, edge);

      for (auto _ : state) {
        result.clear();
        const double dist = distanceCapsuleBox(
            capsule, tfs.tf1, box, tfs.tf2, result, option);
        benchmark::DoNotOptimize(dist);
      }
      return;
    }
  }
}

template <typename DetectorPtr>
void RunDistanceCaseDetector(
    benchmark::State& state,
    const DetectorPtr& detector,
    PairKind pair,
    EdgeCase edge)
{
  const double scale = ScaleFromIndex(static_cast<int>(state.range(0)));
  const auto sphereSpec = MakeSphereSpec(scale);
  const auto capsuleSpec = MakeCapsuleSpec(scale);
  const auto boxSpec = MakeBoxSpec(scale, edge == EdgeCase::kThinFeature);
  const auto option = dart::benchmark::collision::MakeDistanceOption();

  switch (pair) {
    case PairKind::kSphereSphere: {
      auto shape1
          = std::make_shared<dart::dynamics::SphereShape>(sphereSpec.radius);
      auto shape2
          = std::make_shared<dart::dynamics::SphereShape>(sphereSpec.radius);
      const auto tfs = MakeSphereSphereTransforms(
          sphereSpec.radius, sphereSpec.radius, edge);
      RunDistanceDetectorBenchmark(
          state, detector, shape1, tfs.tf1, shape2, tfs.tf2, option);
      return;
    }
    case PairKind::kBoxBox: {
      auto shape1 = std::make_shared<dart::dynamics::BoxShape>(boxSpec.size);
      auto shape2 = std::make_shared<dart::dynamics::BoxShape>(boxSpec.size);
      const auto tfs = MakeBoxBoxTransforms(
          boxSpec.halfExtents, boxSpec.halfExtents, edge);
      RunDistanceDetectorBenchmark(
          state, detector, shape1, tfs.tf1, shape2, tfs.tf2, option);
      return;
    }
    case PairKind::kCapsuleCapsule: {
      auto shape1 = std::make_shared<dart::dynamics::CapsuleShape>(
          capsuleSpec.radius, capsuleSpec.height);
      auto shape2 = std::make_shared<dart::dynamics::CapsuleShape>(
          capsuleSpec.radius, capsuleSpec.height);
      const auto tfs = MakeCapsuleCapsuleTransforms(
          capsuleSpec.radius, capsuleSpec.radius, edge);
      RunDistanceDetectorBenchmark(
          state, detector, shape1, tfs.tf1, shape2, tfs.tf2, option);
      return;
    }
    case PairKind::kSphereBox: {
      auto shape1
          = std::make_shared<dart::dynamics::SphereShape>(sphereSpec.radius);
      auto shape2 = std::make_shared<dart::dynamics::BoxShape>(boxSpec.size);
      const auto tfs = MakeSphereBoxTransforms(
          sphereSpec.radius, boxSpec.halfExtents, edge);
      RunDistanceDetectorBenchmark(
          state, detector, shape1, tfs.tf1, shape2, tfs.tf2, option);
      return;
    }
    case PairKind::kCapsuleSphere: {
      auto shape1 = std::make_shared<dart::dynamics::CapsuleShape>(
          capsuleSpec.radius, capsuleSpec.height);
      auto shape2
          = std::make_shared<dart::dynamics::SphereShape>(sphereSpec.radius);
      const auto tfs = MakeCapsuleSphereTransforms(
          capsuleSpec.radius, sphereSpec.radius, edge);
      RunDistanceDetectorBenchmark(
          state, detector, shape1, tfs.tf1, shape2, tfs.tf2, option);
      return;
    }
    case PairKind::kCapsuleBox: {
      auto shape1 = std::make_shared<dart::dynamics::CapsuleShape>(
          capsuleSpec.radius, capsuleSpec.height);
      auto shape2 = std::make_shared<dart::dynamics::BoxShape>(boxSpec.size);
      const auto tfs = MakeCapsuleBoxTransforms(
          capsuleSpec.radius, boxSpec.halfExtents, edge);
      RunDistanceDetectorBenchmark(
          state, detector, shape1, tfs.tf1, shape2, tfs.tf2, option);
      return;
    }
  }
}

static void BM_Distance_EdgeCases_Experimental(
    benchmark::State& state, PairKind pair, EdgeCase edge)
{
  RunDistanceCaseExperimental(state, pair, edge);
}

static void BM_Distance_EdgeCases_FCL(
    benchmark::State& state, PairKind pair, EdgeCase edge)
{
  auto detector = dart::collision::FCLCollisionDetector::create();
  RunDistanceCaseDetector(state, detector, pair, edge);
}

#if DART_HAVE_BULLET
static void BM_Distance_EdgeCases_Bullet(
    benchmark::State& state, PairKind pair, EdgeCase edge)
{
  auto detector = dart::collision::BulletCollisionDetector::create();
  RunDistanceCaseDetector(state, detector, pair, edge);
}
#endif

#if DART_HAVE_ODE
static void BM_Distance_EdgeCases_ODE(
    benchmark::State& state, PairKind pair, EdgeCase edge)
{
  auto detector = dart::collision::OdeCollisionDetector::create();
  RunDistanceCaseDetector(state, detector, pair, edge);
}
#endif

} // namespace edge_case_bench

using dart::benchmark::collision::EdgeCase;
using dart::benchmark::collision::PairKind;

static void RegisterDistanceEdgeCases()
{
  using edge_case_bench::AddScaleArgs;

  AddScaleArgs(BENCHMARK_CAPTURE(
      BM_Distance_EdgeCases_Experimental,
      SphereSphere_Touching,
      PairKind::kSphereSphere,
      EdgeCase::kTouching));
  AddScaleArgs(BENCHMARK_CAPTURE(
      BM_Distance_EdgeCases_Experimental,
      SphereSphere_DeepPenetration,
      PairKind::kSphereSphere,
      EdgeCase::kDeepPenetration));
  AddScaleArgs(BENCHMARK_CAPTURE(
      BM_Distance_EdgeCases_Experimental,
      SphereSphere_Grazing,
      PairKind::kSphereSphere,
      EdgeCase::kGrazing));

  AddScaleArgs(BENCHMARK_CAPTURE(
      BM_Distance_EdgeCases_Experimental,
      BoxBox_Touching,
      PairKind::kBoxBox,
      EdgeCase::kTouching));
  AddScaleArgs(BENCHMARK_CAPTURE(
      BM_Distance_EdgeCases_Experimental,
      BoxBox_DeepPenetration,
      PairKind::kBoxBox,
      EdgeCase::kDeepPenetration));
  AddScaleArgs(BENCHMARK_CAPTURE(
      BM_Distance_EdgeCases_Experimental,
      BoxBox_Grazing,
      PairKind::kBoxBox,
      EdgeCase::kGrazing));
  AddScaleArgs(BENCHMARK_CAPTURE(
      BM_Distance_EdgeCases_Experimental,
      BoxBox_ThinFeature,
      PairKind::kBoxBox,
      EdgeCase::kThinFeature));

  AddScaleArgs(BENCHMARK_CAPTURE(
      BM_Distance_EdgeCases_Experimental,
      CapsuleCapsule_Touching,
      PairKind::kCapsuleCapsule,
      EdgeCase::kTouching));
  AddScaleArgs(BENCHMARK_CAPTURE(
      BM_Distance_EdgeCases_Experimental,
      CapsuleCapsule_DeepPenetration,
      PairKind::kCapsuleCapsule,
      EdgeCase::kDeepPenetration));
  AddScaleArgs(BENCHMARK_CAPTURE(
      BM_Distance_EdgeCases_Experimental,
      CapsuleCapsule_Grazing,
      PairKind::kCapsuleCapsule,
      EdgeCase::kGrazing));

  AddScaleArgs(BENCHMARK_CAPTURE(
      BM_Distance_EdgeCases_Experimental,
      SphereBox_Touching,
      PairKind::kSphereBox,
      EdgeCase::kTouching));
  AddScaleArgs(BENCHMARK_CAPTURE(
      BM_Distance_EdgeCases_Experimental,
      SphereBox_DeepPenetration,
      PairKind::kSphereBox,
      EdgeCase::kDeepPenetration));
  AddScaleArgs(BENCHMARK_CAPTURE(
      BM_Distance_EdgeCases_Experimental,
      SphereBox_Grazing,
      PairKind::kSphereBox,
      EdgeCase::kGrazing));
  AddScaleArgs(BENCHMARK_CAPTURE(
      BM_Distance_EdgeCases_Experimental,
      SphereBox_ThinFeature,
      PairKind::kSphereBox,
      EdgeCase::kThinFeature));

  AddScaleArgs(BENCHMARK_CAPTURE(
      BM_Distance_EdgeCases_Experimental,
      CapsuleSphere_Touching,
      PairKind::kCapsuleSphere,
      EdgeCase::kTouching));
  AddScaleArgs(BENCHMARK_CAPTURE(
      BM_Distance_EdgeCases_Experimental,
      CapsuleSphere_DeepPenetration,
      PairKind::kCapsuleSphere,
      EdgeCase::kDeepPenetration));
  AddScaleArgs(BENCHMARK_CAPTURE(
      BM_Distance_EdgeCases_Experimental,
      CapsuleSphere_Grazing,
      PairKind::kCapsuleSphere,
      EdgeCase::kGrazing));

  AddScaleArgs(BENCHMARK_CAPTURE(
      BM_Distance_EdgeCases_Experimental,
      CapsuleBox_Touching,
      PairKind::kCapsuleBox,
      EdgeCase::kTouching));
  AddScaleArgs(BENCHMARK_CAPTURE(
      BM_Distance_EdgeCases_Experimental,
      CapsuleBox_DeepPenetration,
      PairKind::kCapsuleBox,
      EdgeCase::kDeepPenetration));
  AddScaleArgs(BENCHMARK_CAPTURE(
      BM_Distance_EdgeCases_Experimental,
      CapsuleBox_Grazing,
      PairKind::kCapsuleBox,
      EdgeCase::kGrazing));
  AddScaleArgs(BENCHMARK_CAPTURE(
      BM_Distance_EdgeCases_Experimental,
      CapsuleBox_ThinFeature,
      PairKind::kCapsuleBox,
      EdgeCase::kThinFeature));

  AddScaleArgs(BENCHMARK_CAPTURE(
      BM_Distance_EdgeCases_FCL,
      SphereSphere_Touching,
      PairKind::kSphereSphere,
      EdgeCase::kTouching));
  AddScaleArgs(BENCHMARK_CAPTURE(
      BM_Distance_EdgeCases_FCL,
      SphereSphere_DeepPenetration,
      PairKind::kSphereSphere,
      EdgeCase::kDeepPenetration));
  AddScaleArgs(BENCHMARK_CAPTURE(
      BM_Distance_EdgeCases_FCL,
      SphereSphere_Grazing,
      PairKind::kSphereSphere,
      EdgeCase::kGrazing));

  AddScaleArgs(BENCHMARK_CAPTURE(
      BM_Distance_EdgeCases_FCL,
      BoxBox_Touching,
      PairKind::kBoxBox,
      EdgeCase::kTouching));
  AddScaleArgs(BENCHMARK_CAPTURE(
      BM_Distance_EdgeCases_FCL,
      BoxBox_DeepPenetration,
      PairKind::kBoxBox,
      EdgeCase::kDeepPenetration));
  AddScaleArgs(BENCHMARK_CAPTURE(
      BM_Distance_EdgeCases_FCL,
      BoxBox_Grazing,
      PairKind::kBoxBox,
      EdgeCase::kGrazing));
  AddScaleArgs(BENCHMARK_CAPTURE(
      BM_Distance_EdgeCases_FCL,
      BoxBox_ThinFeature,
      PairKind::kBoxBox,
      EdgeCase::kThinFeature));

  AddScaleArgs(BENCHMARK_CAPTURE(
      BM_Distance_EdgeCases_FCL,
      CapsuleCapsule_Touching,
      PairKind::kCapsuleCapsule,
      EdgeCase::kTouching));
  AddScaleArgs(BENCHMARK_CAPTURE(
      BM_Distance_EdgeCases_FCL,
      CapsuleCapsule_DeepPenetration,
      PairKind::kCapsuleCapsule,
      EdgeCase::kDeepPenetration));
  AddScaleArgs(BENCHMARK_CAPTURE(
      BM_Distance_EdgeCases_FCL,
      CapsuleCapsule_Grazing,
      PairKind::kCapsuleCapsule,
      EdgeCase::kGrazing));

  AddScaleArgs(BENCHMARK_CAPTURE(
      BM_Distance_EdgeCases_FCL,
      SphereBox_Touching,
      PairKind::kSphereBox,
      EdgeCase::kTouching));
  AddScaleArgs(BENCHMARK_CAPTURE(
      BM_Distance_EdgeCases_FCL,
      SphereBox_DeepPenetration,
      PairKind::kSphereBox,
      EdgeCase::kDeepPenetration));
  AddScaleArgs(BENCHMARK_CAPTURE(
      BM_Distance_EdgeCases_FCL,
      SphereBox_Grazing,
      PairKind::kSphereBox,
      EdgeCase::kGrazing));
  AddScaleArgs(BENCHMARK_CAPTURE(
      BM_Distance_EdgeCases_FCL,
      SphereBox_ThinFeature,
      PairKind::kSphereBox,
      EdgeCase::kThinFeature));

  AddScaleArgs(BENCHMARK_CAPTURE(
      BM_Distance_EdgeCases_FCL,
      CapsuleSphere_Touching,
      PairKind::kCapsuleSphere,
      EdgeCase::kTouching));
  AddScaleArgs(BENCHMARK_CAPTURE(
      BM_Distance_EdgeCases_FCL,
      CapsuleSphere_DeepPenetration,
      PairKind::kCapsuleSphere,
      EdgeCase::kDeepPenetration));
  AddScaleArgs(BENCHMARK_CAPTURE(
      BM_Distance_EdgeCases_FCL,
      CapsuleSphere_Grazing,
      PairKind::kCapsuleSphere,
      EdgeCase::kGrazing));

  AddScaleArgs(BENCHMARK_CAPTURE(
      BM_Distance_EdgeCases_FCL,
      CapsuleBox_Touching,
      PairKind::kCapsuleBox,
      EdgeCase::kTouching));
  AddScaleArgs(BENCHMARK_CAPTURE(
      BM_Distance_EdgeCases_FCL,
      CapsuleBox_DeepPenetration,
      PairKind::kCapsuleBox,
      EdgeCase::kDeepPenetration));
  AddScaleArgs(BENCHMARK_CAPTURE(
      BM_Distance_EdgeCases_FCL,
      CapsuleBox_Grazing,
      PairKind::kCapsuleBox,
      EdgeCase::kGrazing));
  AddScaleArgs(BENCHMARK_CAPTURE(
      BM_Distance_EdgeCases_FCL,
      CapsuleBox_ThinFeature,
      PairKind::kCapsuleBox,
      EdgeCase::kThinFeature));

#if DART_HAVE_BULLET
  AddScaleArgs(BENCHMARK_CAPTURE(
      BM_Distance_EdgeCases_Bullet,
      SphereSphere_Touching,
      PairKind::kSphereSphere,
      EdgeCase::kTouching));
  AddScaleArgs(BENCHMARK_CAPTURE(
      BM_Distance_EdgeCases_Bullet,
      SphereSphere_DeepPenetration,
      PairKind::kSphereSphere,
      EdgeCase::kDeepPenetration));
  AddScaleArgs(BENCHMARK_CAPTURE(
      BM_Distance_EdgeCases_Bullet,
      SphereSphere_Grazing,
      PairKind::kSphereSphere,
      EdgeCase::kGrazing));

  AddScaleArgs(BENCHMARK_CAPTURE(
      BM_Distance_EdgeCases_Bullet,
      BoxBox_Touching,
      PairKind::kBoxBox,
      EdgeCase::kTouching));
  AddScaleArgs(BENCHMARK_CAPTURE(
      BM_Distance_EdgeCases_Bullet,
      BoxBox_DeepPenetration,
      PairKind::kBoxBox,
      EdgeCase::kDeepPenetration));
  AddScaleArgs(BENCHMARK_CAPTURE(
      BM_Distance_EdgeCases_Bullet,
      BoxBox_Grazing,
      PairKind::kBoxBox,
      EdgeCase::kGrazing));
  AddScaleArgs(BENCHMARK_CAPTURE(
      BM_Distance_EdgeCases_Bullet,
      BoxBox_ThinFeature,
      PairKind::kBoxBox,
      EdgeCase::kThinFeature));

  AddScaleArgs(BENCHMARK_CAPTURE(
      BM_Distance_EdgeCases_Bullet,
      CapsuleCapsule_Touching,
      PairKind::kCapsuleCapsule,
      EdgeCase::kTouching));
  AddScaleArgs(BENCHMARK_CAPTURE(
      BM_Distance_EdgeCases_Bullet,
      CapsuleCapsule_DeepPenetration,
      PairKind::kCapsuleCapsule,
      EdgeCase::kDeepPenetration));
  AddScaleArgs(BENCHMARK_CAPTURE(
      BM_Distance_EdgeCases_Bullet,
      CapsuleCapsule_Grazing,
      PairKind::kCapsuleCapsule,
      EdgeCase::kGrazing));

  AddScaleArgs(BENCHMARK_CAPTURE(
      BM_Distance_EdgeCases_Bullet,
      SphereBox_Touching,
      PairKind::kSphereBox,
      EdgeCase::kTouching));
  AddScaleArgs(BENCHMARK_CAPTURE(
      BM_Distance_EdgeCases_Bullet,
      SphereBox_DeepPenetration,
      PairKind::kSphereBox,
      EdgeCase::kDeepPenetration));
  AddScaleArgs(BENCHMARK_CAPTURE(
      BM_Distance_EdgeCases_Bullet,
      SphereBox_Grazing,
      PairKind::kSphereBox,
      EdgeCase::kGrazing));
  AddScaleArgs(BENCHMARK_CAPTURE(
      BM_Distance_EdgeCases_Bullet,
      SphereBox_ThinFeature,
      PairKind::kSphereBox,
      EdgeCase::kThinFeature));

  AddScaleArgs(BENCHMARK_CAPTURE(
      BM_Distance_EdgeCases_Bullet,
      CapsuleSphere_Touching,
      PairKind::kCapsuleSphere,
      EdgeCase::kTouching));
  AddScaleArgs(BENCHMARK_CAPTURE(
      BM_Distance_EdgeCases_Bullet,
      CapsuleSphere_DeepPenetration,
      PairKind::kCapsuleSphere,
      EdgeCase::kDeepPenetration));
  AddScaleArgs(BENCHMARK_CAPTURE(
      BM_Distance_EdgeCases_Bullet,
      CapsuleSphere_Grazing,
      PairKind::kCapsuleSphere,
      EdgeCase::kGrazing));

  AddScaleArgs(BENCHMARK_CAPTURE(
      BM_Distance_EdgeCases_Bullet,
      CapsuleBox_Touching,
      PairKind::kCapsuleBox,
      EdgeCase::kTouching));
  AddScaleArgs(BENCHMARK_CAPTURE(
      BM_Distance_EdgeCases_Bullet,
      CapsuleBox_DeepPenetration,
      PairKind::kCapsuleBox,
      EdgeCase::kDeepPenetration));
  AddScaleArgs(BENCHMARK_CAPTURE(
      BM_Distance_EdgeCases_Bullet,
      CapsuleBox_Grazing,
      PairKind::kCapsuleBox,
      EdgeCase::kGrazing));
  AddScaleArgs(BENCHMARK_CAPTURE(
      BM_Distance_EdgeCases_Bullet,
      CapsuleBox_ThinFeature,
      PairKind::kCapsuleBox,
      EdgeCase::kThinFeature));
#endif

#if DART_HAVE_ODE
  AddScaleArgs(BENCHMARK_CAPTURE(
      BM_Distance_EdgeCases_ODE,
      SphereSphere_Touching,
      PairKind::kSphereSphere,
      EdgeCase::kTouching));
  AddScaleArgs(BENCHMARK_CAPTURE(
      BM_Distance_EdgeCases_ODE,
      SphereSphere_DeepPenetration,
      PairKind::kSphereSphere,
      EdgeCase::kDeepPenetration));
  AddScaleArgs(BENCHMARK_CAPTURE(
      BM_Distance_EdgeCases_ODE,
      SphereSphere_Grazing,
      PairKind::kSphereSphere,
      EdgeCase::kGrazing));

  AddScaleArgs(BENCHMARK_CAPTURE(
      BM_Distance_EdgeCases_ODE,
      BoxBox_Touching,
      PairKind::kBoxBox,
      EdgeCase::kTouching));
  AddScaleArgs(BENCHMARK_CAPTURE(
      BM_Distance_EdgeCases_ODE,
      BoxBox_DeepPenetration,
      PairKind::kBoxBox,
      EdgeCase::kDeepPenetration));
  AddScaleArgs(BENCHMARK_CAPTURE(
      BM_Distance_EdgeCases_ODE,
      BoxBox_Grazing,
      PairKind::kBoxBox,
      EdgeCase::kGrazing));
  AddScaleArgs(BENCHMARK_CAPTURE(
      BM_Distance_EdgeCases_ODE,
      BoxBox_ThinFeature,
      PairKind::kBoxBox,
      EdgeCase::kThinFeature));

  AddScaleArgs(BENCHMARK_CAPTURE(
      BM_Distance_EdgeCases_ODE,
      CapsuleCapsule_Touching,
      PairKind::kCapsuleCapsule,
      EdgeCase::kTouching));
  AddScaleArgs(BENCHMARK_CAPTURE(
      BM_Distance_EdgeCases_ODE,
      CapsuleCapsule_DeepPenetration,
      PairKind::kCapsuleCapsule,
      EdgeCase::kDeepPenetration));
  AddScaleArgs(BENCHMARK_CAPTURE(
      BM_Distance_EdgeCases_ODE,
      CapsuleCapsule_Grazing,
      PairKind::kCapsuleCapsule,
      EdgeCase::kGrazing));

  AddScaleArgs(BENCHMARK_CAPTURE(
      BM_Distance_EdgeCases_ODE,
      SphereBox_Touching,
      PairKind::kSphereBox,
      EdgeCase::kTouching));
  AddScaleArgs(BENCHMARK_CAPTURE(
      BM_Distance_EdgeCases_ODE,
      SphereBox_DeepPenetration,
      PairKind::kSphereBox,
      EdgeCase::kDeepPenetration));
  AddScaleArgs(BENCHMARK_CAPTURE(
      BM_Distance_EdgeCases_ODE,
      SphereBox_Grazing,
      PairKind::kSphereBox,
      EdgeCase::kGrazing));
  AddScaleArgs(BENCHMARK_CAPTURE(
      BM_Distance_EdgeCases_ODE,
      SphereBox_ThinFeature,
      PairKind::kSphereBox,
      EdgeCase::kThinFeature));

  AddScaleArgs(BENCHMARK_CAPTURE(
      BM_Distance_EdgeCases_ODE,
      CapsuleSphere_Touching,
      PairKind::kCapsuleSphere,
      EdgeCase::kTouching));
  AddScaleArgs(BENCHMARK_CAPTURE(
      BM_Distance_EdgeCases_ODE,
      CapsuleSphere_DeepPenetration,
      PairKind::kCapsuleSphere,
      EdgeCase::kDeepPenetration));
  AddScaleArgs(BENCHMARK_CAPTURE(
      BM_Distance_EdgeCases_ODE,
      CapsuleSphere_Grazing,
      PairKind::kCapsuleSphere,
      EdgeCase::kGrazing));

  AddScaleArgs(BENCHMARK_CAPTURE(
      BM_Distance_EdgeCases_ODE,
      CapsuleBox_Touching,
      PairKind::kCapsuleBox,
      EdgeCase::kTouching));
  AddScaleArgs(BENCHMARK_CAPTURE(
      BM_Distance_EdgeCases_ODE,
      CapsuleBox_DeepPenetration,
      PairKind::kCapsuleBox,
      EdgeCase::kDeepPenetration));
  AddScaleArgs(BENCHMARK_CAPTURE(
      BM_Distance_EdgeCases_ODE,
      CapsuleBox_Grazing,
      PairKind::kCapsuleBox,
      EdgeCase::kGrazing));
  AddScaleArgs(BENCHMARK_CAPTURE(
      BM_Distance_EdgeCases_ODE,
      CapsuleBox_ThinFeature,
      PairKind::kCapsuleBox,
      EdgeCase::kThinFeature));
#endif
}

struct DistanceEdgeCaseRegistrar
{
  DistanceEdgeCaseRegistrar()
  {
    RegisterDistanceEdgeCases();
  }
};

static DistanceEdgeCaseRegistrar g_distance_edge_case_registrar;

namespace accuracy {

constexpr double kTolerance = 1e-6;

bool verifySphereSphereDistance()
{
  SphereShape exp1(1.0);
  SphereShape exp2(1.0);

  const Eigen::Isometry3d tf1 = Eigen::Isometry3d::Identity();
  const Eigen::Isometry3d tf2 = OffsetTransform(5.0, 0.0, 0.0);

  DistanceResult expResult;
  DistanceOption expOption = MakeExperimentalDistanceOption();
  double expDist
      = distanceSphereSphere(exp1, tf1, exp2, tf2, expResult, expOption);

  auto detector = dart::collision::FCLCollisionDetector::create();
  auto shape1 = std::make_shared<dart::dynamics::SphereShape>(1.0);
  auto shape2 = std::make_shared<dart::dynamics::SphereShape>(1.0);
  auto ctx = MakePairContext(detector, shape1, tf1, shape2, tf2);

  auto fclOption = dart::benchmark::collision::MakeDistanceOption();
  dart::collision::DistanceResult fclResult;
  double fclDist = detector->distance(ctx.group.get(), fclOption, &fclResult);

  const double expected = 3.0;

  if (std::abs(expDist - expected) > kTolerance) {
    std::cerr << "ACCURACY FAIL: Sphere-sphere distance expected " << expected
              << ", got " << expDist << "\n";
    return false;
  }

  if (std::abs(fclDist - expected) > 1e-2) {
    std::cerr << "ACCURACY INFO: FCL distance " << fclDist << " (expected "
              << expected << ")\n";
  }

  return true;
}

void runAccuracyChecks()
{
  std::cout << "\n=== DISTANCE ACCURACY VERIFICATION ===\n";

  bool allPassed = true;

  std::cout << "Sphere-Sphere distance: ";
  if (verifySphereSphereDistance()) {
    std::cout << "PASS\n";
  } else {
    std::cout << "FAIL\n";
    allPassed = false;
  }

  std::cout << "=== DISTANCE ACCURACY VERIFICATION "
            << (allPassed ? "PASSED" : "FAILED") << " ===\n\n";
}

} // namespace accuracy

int main(int argc, char** argv)
{
  accuracy::runAccuracyChecks();

  ::benchmark::Initialize(&argc, argv);
  ::benchmark::RunSpecifiedBenchmarks();
  ::benchmark::Shutdown();
  return 0;
}
