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

#include <dart/collision/experimental/collision_world.hpp>
#include <dart/collision/experimental/narrow_phase/box_box.hpp>
#include <dart/collision/experimental/narrow_phase/capsule_box.hpp>
#include <dart/collision/experimental/narrow_phase/capsule_capsule.hpp>
#include <dart/collision/experimental/narrow_phase/capsule_sphere.hpp>
#include <dart/collision/experimental/narrow_phase/cylinder_collision.hpp>
#include <dart/collision/experimental/narrow_phase/distance.hpp>
#include <dart/collision/experimental/narrow_phase/plane_sphere.hpp>
#include <dart/collision/experimental/narrow_phase/sphere_box.hpp>
#include <dart/collision/experimental/narrow_phase/sphere_sphere.hpp>
#include <dart/collision/experimental/shapes/shape.hpp>

#include <benchmark/benchmark.h>

#include <random>

using namespace dart::collision::experimental;

namespace {

std::mt19937 rng(42);

Eigen::Vector3d randomPosition(double range)
{
  std::uniform_real_distribution<double> dist(-range, range);
  return Eigen::Vector3d(dist(rng), dist(rng), dist(rng));
}

Eigen::Isometry3d randomTransform(double posRange)
{
  Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
  tf.translation() = randomPosition(posRange);
  return tf;
}

Eigen::Isometry3d randomTransformWithRotation(double posRange)
{
  std::uniform_real_distribution<double> angleDist(0, 2 * M_PI);
  Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
  tf.translation() = randomPosition(posRange);
  tf.linear() = Eigen::AngleAxisd(angleDist(rng), Eigen::Vector3d::UnitZ())
                    .toRotationMatrix();
  return tf;
}

} // namespace

static void BM_SphereSphereCollision(benchmark::State& state)
{
  SphereShape s1(1.0);
  SphereShape s2(1.0);

  Eigen::Isometry3d tf1 = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d tf2 = Eigen::Isometry3d::Identity();
  tf2.translation() = Eigen::Vector3d(1.5, 0, 0);

  CollisionResult result;
  CollisionOption option;

  for (auto _ : state) {
    result.clear();
    benchmark::DoNotOptimize(collideSpheres(s1, tf1, s2, tf2, result, option));
  }
}
BENCHMARK(BM_SphereSphereCollision);

static void BM_BoxBoxCollision(benchmark::State& state)
{
  BoxShape b1(Eigen::Vector3d(1, 1, 1));
  BoxShape b2(Eigen::Vector3d(1, 1, 1));

  Eigen::Isometry3d tf1 = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d tf2 = Eigen::Isometry3d::Identity();
  tf2.translation() = Eigen::Vector3d(1.5, 0, 0);

  CollisionResult result;
  CollisionOption option;

  for (auto _ : state) {
    result.clear();
    benchmark::DoNotOptimize(collideBoxes(b1, tf1, b2, tf2, result, option));
  }
}
BENCHMARK(BM_BoxBoxCollision);

static void BM_CapsuleCapsuleCollision(benchmark::State& state)
{
  CapsuleShape c1(0.5, 2.0);
  CapsuleShape c2(0.5, 2.0);

  Eigen::Isometry3d tf1 = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d tf2 = Eigen::Isometry3d::Identity();
  tf2.translation() = Eigen::Vector3d(0.8, 0, 0);

  CollisionResult result;
  CollisionOption option;

  for (auto _ : state) {
    result.clear();
    benchmark::DoNotOptimize(collideCapsules(c1, tf1, c2, tf2, result, option));
  }
}
BENCHMARK(BM_CapsuleCapsuleCollision);

static void BM_CylinderCylinderCollision(benchmark::State& state)
{
  CylinderShape c1(0.5, 2.0);
  CylinderShape c2(0.5, 2.0);

  Eigen::Isometry3d tf1 = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d tf2 = Eigen::Isometry3d::Identity();
  tf2.translation() = Eigen::Vector3d(0.8, 0, 0);

  CollisionResult result;
  CollisionOption option;

  for (auto _ : state) {
    result.clear();
    benchmark::DoNotOptimize(
        collideCylinders(c1, tf1, c2, tf2, result, option));
  }
}
BENCHMARK(BM_CylinderCylinderCollision);

static void BM_SphereBoxCollision(benchmark::State& state)
{
  SphereShape sphere(1.0);
  BoxShape box(Eigen::Vector3d(1, 1, 1));

  Eigen::Isometry3d tf1 = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d tf2 = Eigen::Isometry3d::Identity();
  tf2.translation() = Eigen::Vector3d(1.2, 0, 0);

  CollisionResult result;
  CollisionOption option;

  for (auto _ : state) {
    result.clear();
    benchmark::DoNotOptimize(
        collideSphereBox(sphere, tf1, box, tf2, result, option));
  }
}
BENCHMARK(BM_SphereBoxCollision);

static void BM_CapsuleSphereCollision(benchmark::State& state)
{
  CapsuleShape capsule(0.5, 2.0);
  SphereShape sphere(0.5);

  Eigen::Isometry3d tf1 = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d tf2 = Eigen::Isometry3d::Identity();
  tf2.translation() = Eigen::Vector3d(0.8, 0, 0);

  CollisionResult result;
  CollisionOption option;

  for (auto _ : state) {
    result.clear();
    benchmark::DoNotOptimize(
        collideCapsuleSphere(capsule, tf1, sphere, tf2, result, option));
  }
}
BENCHMARK(BM_CapsuleSphereCollision);

static void BM_PlaneSphereCollision(benchmark::State& state)
{
  PlaneShape plane(Eigen::Vector3d::UnitZ(), 0.0);
  SphereShape sphere(0.5);

  Eigen::Isometry3d tf1 = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d tf2 = Eigen::Isometry3d::Identity();
  tf2.translation() = Eigen::Vector3d(0, 0, 0.3);

  CollisionResult result;
  CollisionOption option;

  for (auto _ : state) {
    result.clear();
    benchmark::DoNotOptimize(
        collidePlaneSphere(plane, tf1, sphere, tf2, result, option));
  }
}
BENCHMARK(BM_PlaneSphereCollision);

static void BM_SphereSphereDistance(benchmark::State& state)
{
  SphereShape s1(1.0);
  SphereShape s2(1.0);

  Eigen::Isometry3d tf1 = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d tf2 = Eigen::Isometry3d::Identity();
  tf2.translation() = Eigen::Vector3d(5.0, 0, 0);

  DistanceResult result;
  DistanceOption option;

  for (auto _ : state) {
    result.clear();
    benchmark::DoNotOptimize(
        distanceSphereSphere(s1, tf1, s2, tf2, result, option));
  }
}
BENCHMARK(BM_SphereSphereDistance);

static void BM_BoxBoxDistance(benchmark::State& state)
{
  BoxShape b1(Eigen::Vector3d(1, 1, 1));
  BoxShape b2(Eigen::Vector3d(1, 1, 1));

  Eigen::Isometry3d tf1 = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d tf2 = Eigen::Isometry3d::Identity();
  tf2.translation() = Eigen::Vector3d(3.0, 0, 0);

  DistanceResult result;
  DistanceOption option;

  for (auto _ : state) {
    result.clear();
    benchmark::DoNotOptimize(distanceBoxBox(b1, tf1, b2, tf2, result, option));
  }
}
BENCHMARK(BM_BoxBoxDistance);

static void BM_CapsuleCapsuleDistance(benchmark::State& state)
{
  CapsuleShape c1(0.5, 2.0);
  CapsuleShape c2(0.5, 2.0);

  Eigen::Isometry3d tf1 = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d tf2 = Eigen::Isometry3d::Identity();
  tf2.translation() = Eigen::Vector3d(3.0, 0, 0);

  DistanceResult result;
  DistanceOption option;

  for (auto _ : state) {
    result.clear();
    benchmark::DoNotOptimize(
        distanceCapsuleCapsule(c1, tf1, c2, tf2, result, option));
  }
}
BENCHMARK(BM_CapsuleCapsuleDistance);

static void BM_BroadPhaseNObjects(benchmark::State& state)
{
  const int n = state.range(0);
  rng.seed(42);

  CollisionWorld world;

  std::vector<std::shared_ptr<CollisionObject>> objects;

  for (int i = 0; i < n; ++i) {
    auto shape = std::make_shared<SphereShape>(0.5);
    auto obj = std::make_shared<CollisionObject>(shape, randomTransform(10.0));
    objects.push_back(obj);
    world.addObject(obj);
  }

  for (auto _ : state) {
    CollisionResult result;
    CollisionOption option;
    benchmark::DoNotOptimize(world.collide(option, result));
  }

  state.SetComplexityN(n);
}
BENCHMARK(BM_BroadPhaseNObjects)
    ->Arg(10)
    ->Arg(50)
    ->Arg(100)
    ->Arg(200)
    ->Complexity();

static void BM_CollisionWorldMixedShapes(benchmark::State& state)
{
  const int n = state.range(0);
  rng.seed(42);

  CollisionWorld world;

  std::vector<std::shared_ptr<CollisionObject>> objects;

  for (int i = 0; i < n; ++i) {
    std::shared_ptr<Shape> shape;
    switch (i % 3) {
      case 0:
        shape = std::make_shared<SphereShape>(0.5);
        break;
      case 1:
        shape = std::make_shared<BoxShape>(Eigen::Vector3d(0.4, 0.4, 0.4));
        break;
      case 2:
        shape = std::make_shared<CapsuleShape>(0.3, 1.0);
        break;
    }
    auto obj = std::make_shared<CollisionObject>(shape, randomTransform(10.0));
    objects.push_back(obj);
    world.addObject(obj);
  }

  for (auto _ : state) {
    CollisionResult result;
    CollisionOption option;
    benchmark::DoNotOptimize(world.collide(option, result));
  }

  state.SetComplexityN(n);
}
BENCHMARK(BM_CollisionWorldMixedShapes)
    ->Arg(10)
    ->Arg(50)
    ->Arg(100)
    ->Arg(200)
    ->Complexity();

static void BM_CollisionWorldAllShapes(benchmark::State& state)
{
  const int n = state.range(0);
  rng.seed(42);

  CollisionWorld world;

  std::vector<std::shared_ptr<CollisionObject>> objects;

  for (int i = 0; i < n; ++i) {
    std::shared_ptr<Shape> shape;
    switch (i % 5) {
      case 0:
        shape = std::make_shared<SphereShape>(0.5);
        break;
      case 1:
        shape = std::make_shared<BoxShape>(Eigen::Vector3d(0.4, 0.4, 0.4));
        break;
      case 2:
        shape = std::make_shared<CapsuleShape>(0.3, 1.0);
        break;
      case 3:
        shape = std::make_shared<CylinderShape>(0.3, 1.0);
        break;
      case 4:
        shape = std::make_shared<PlaneShape>(Eigen::Vector3d::UnitZ(), -5.0);
        break;
    }
    auto obj = std::make_shared<CollisionObject>(
        shape, randomTransformWithRotation(10.0));
    objects.push_back(obj);
    world.addObject(obj);
  }

  for (auto _ : state) {
    CollisionResult result;
    CollisionOption option;
    benchmark::DoNotOptimize(world.collide(option, result));
  }

  state.SetComplexityN(n);
}
BENCHMARK(BM_CollisionWorldAllShapes)
    ->Arg(10)
    ->Arg(50)
    ->Arg(100)
    ->Arg(200)
    ->Complexity();

static void BM_BatchDistanceQueries(benchmark::State& state)
{
  const int n = state.range(0);
  rng.seed(42);

  std::vector<SphereShape> spheres;
  std::vector<Eigen::Isometry3d> transforms;

  for (int i = 0; i < n; ++i) {
    spheres.emplace_back(0.5);
    transforms.push_back(randomTransform(20.0));
  }

  for (auto _ : state) {
    double totalDist = 0.0;
    for (int i = 0; i < n; ++i) {
      for (int j = i + 1; j < n; ++j) {
        DistanceResult result;
        DistanceOption option;
        totalDist += distanceSphereSphere(
            spheres[i],
            transforms[i],
            spheres[j],
            transforms[j],
            result,
            option);
      }
    }
    benchmark::DoNotOptimize(totalDist);
  }

  state.SetComplexityN(n);
}
BENCHMARK(BM_BatchDistanceQueries)
    ->Arg(10)
    ->Arg(20)
    ->Arg(50)
    ->Arg(100)
    ->Complexity();

static void BM_VaryingSizes(benchmark::State& state)
{
  const double sizeMultiplier = state.range(0) / 10.0;
  rng.seed(42);

  SphereShape s1(1.0 * sizeMultiplier);
  SphereShape s2(1.0 * sizeMultiplier);
  BoxShape b1(Eigen::Vector3d::Ones() * sizeMultiplier);
  BoxShape b2(Eigen::Vector3d::Ones() * sizeMultiplier);

  Eigen::Isometry3d tf1 = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d tf2 = Eigen::Isometry3d::Identity();
  tf2.translation() = Eigen::Vector3d(1.5 * sizeMultiplier, 0, 0);

  for (auto _ : state) {
    CollisionResult result;
    CollisionOption option;

    result.clear();
    benchmark::DoNotOptimize(collideSpheres(s1, tf1, s2, tf2, result, option));

    result.clear();
    benchmark::DoNotOptimize(collideBoxes(b1, tf1, b2, tf2, result, option));
  }
}
BENCHMARK(BM_VaryingSizes)->Arg(1)->Arg(10)->Arg(100)->Arg(1000);

BENCHMARK_MAIN();
