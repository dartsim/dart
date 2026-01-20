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
    const Eigen::Isometry3d& tf2)
{
  auto ctx = MakePairContext(detector, shape1, tf1, shape2, tf2);
  dart::collision::DistanceOption option;
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

} // namespace

static void BM_Distance_SphereSphere_Experimental(benchmark::State& state)
{
  SphereShape s1(1.0);
  SphereShape s2(1.0);

  const Eigen::Isometry3d tf1 = Eigen::Isometry3d::Identity();
  const Eigen::Isometry3d tf2 = OffsetTransform(5.0, 0.0, 0.0);

  DistanceResult result;
  DistanceOption option;

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

  RunDistanceDetectorBenchmark(state, detector, shape1, tf1, shape2, tf2);
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

  RunDistanceDetectorBenchmark(state, detector, shape1, tf1, shape2, tf2);
}
BENCHMARK(BM_Distance_SphereSphere_Bullet);
#endif

static void BM_Distance_BoxBox_Experimental(benchmark::State& state)
{
  BoxShape b1(Eigen::Vector3d(0.5, 0.5, 0.5));
  BoxShape b2(Eigen::Vector3d(0.5, 0.5, 0.5));

  const Eigen::Isometry3d tf1 = Eigen::Isometry3d::Identity();
  const Eigen::Isometry3d tf2 = OffsetTransform(3.0, 0.0, 0.0);

  DistanceResult result;
  DistanceOption option;

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

  RunDistanceDetectorBenchmark(state, detector, shape1, tf1, shape2, tf2);
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

  RunDistanceDetectorBenchmark(state, detector, shape1, tf1, shape2, tf2);
}
BENCHMARK(BM_Distance_BoxBox_Bullet);
#endif

static void BM_Distance_CapsuleCapsule_Experimental(benchmark::State& state)
{
  CapsuleShape c1(0.5, 2.0);
  CapsuleShape c2(0.5, 2.0);

  const Eigen::Isometry3d tf1 = Eigen::Isometry3d::Identity();
  const Eigen::Isometry3d tf2 = OffsetTransform(3.0, 0.0, 0.0);

  DistanceResult result;
  DistanceOption option;

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

  RunDistanceDetectorBenchmark(state, detector, shape1, tf1, shape2, tf2);
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

  RunDistanceDetectorBenchmark(state, detector, shape1, tf1, shape2, tf2);
}
BENCHMARK(BM_Distance_CapsuleCapsule_Bullet);
#endif

namespace accuracy {

constexpr double kTolerance = 1e-6;

bool verifySphereSphereDistance()
{
  SphereShape exp1(1.0);
  SphereShape exp2(1.0);

  const Eigen::Isometry3d tf1 = Eigen::Isometry3d::Identity();
  const Eigen::Isometry3d tf2 = OffsetTransform(5.0, 0.0, 0.0);

  DistanceResult expResult;
  DistanceOption expOption;
  double expDist
      = distanceSphereSphere(exp1, tf1, exp2, tf2, expResult, expOption);

  auto detector = dart::collision::FCLCollisionDetector::create();
  auto shape1 = std::make_shared<dart::dynamics::SphereShape>(1.0);
  auto shape2 = std::make_shared<dart::dynamics::SphereShape>(1.0);
  auto ctx = MakePairContext(detector, shape1, tf1, shape2, tf2);

  dart::collision::DistanceOption fclOption;
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
