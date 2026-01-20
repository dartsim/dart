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

#include <dart/collision/RaycastOption.hpp>
#include <dart/collision/RaycastResult.hpp>
#include <dart/collision/experimental/collision_world.hpp>
#include <dart/collision/experimental/shapes/shape.hpp>
#include <dart/collision/experimental/types.hpp>

#include <dart/dynamics/BoxShape.hpp>
#include <dart/dynamics/CapsuleShape.hpp>
#include <dart/dynamics/CylinderShape.hpp>
#include <dart/dynamics/FreeJoint.hpp>
#include <dart/dynamics/PlaneShape.hpp>
#include <dart/dynamics/Skeleton.hpp>
#include <dart/dynamics/SphereShape.hpp>

#if DART_HAVE_BULLET
  #include <dart/collision/bullet/BulletCollisionDetector.hpp>
#endif

#include "tests/benchmark/collision/fixtures/shape_factories.hpp"

#include <benchmark/benchmark.h>

#include <memory>

using namespace dart::collision::experimental;

namespace {

struct RaycastContext
{
  dart::dynamics::SkeletonPtr skeleton;
  dart::collision::CollisionGroupPtr group;
};

template <typename DetectorPtr>
RaycastContext MakeRaycastContext(
    const DetectorPtr& detector,
    const std::shared_ptr<dart::dynamics::Shape>& shape,
    const Eigen::Isometry3d& transform)
{
  RaycastContext ctx;
  ctx.skeleton
      = dart::benchmark::collision::CreateSingleShapeSkeleton(shape, transform);
  ctx.group = detector->createCollisionGroup();
  dart::benchmark::collision::AddSkeletonToGroup(ctx.group.get(), ctx.skeleton);
  return ctx;
}

template <typename DetectorPtr>
void RunRaycastDetectorBenchmark(
    benchmark::State& state,
    const DetectorPtr& detector,
    const std::shared_ptr<dart::dynamics::Shape>& shape,
    const Eigen::Isometry3d& transform,
    const Eigen::Vector3d& from,
    const Eigen::Vector3d& to)
{
  auto ctx = MakeRaycastContext(detector, shape, transform);
  dart::collision::RaycastOption option(false, true);
  dart::collision::RaycastResult result;

  for (auto _ : state) {
    result.clear();
    bool hit = ctx.group->raycast(from, to, option, &result);
    benchmark::DoNotOptimize(hit);
  }
}

} // namespace

static void BM_Raycast_Sphere_Experimental(benchmark::State& state)
{
  CollisionWorld world;
  auto shape = std::make_unique<SphereShape>(0.5);
  world.createObject(std::move(shape), Eigen::Isometry3d::Identity());

  const Eigen::Vector3d origin(-2.0, 0.0, 0.0);
  const Eigen::Vector3d direction(4.0, 0.0, 0.0);
  const Ray ray(origin, direction, direction.norm());

  RaycastOption option;
  option.maxDistance = direction.norm();
  option.backfaceCulling = true;

  RaycastResult result;

  for (auto _ : state) {
    result.clear();
    benchmark::DoNotOptimize(world.raycast(ray, option, result));
  }
}
BENCHMARK(BM_Raycast_Sphere_Experimental);

#if DART_HAVE_BULLET
static void BM_Raycast_Sphere_Bullet(benchmark::State& state)
{
  auto detector = dart::collision::BulletCollisionDetector::create();
  auto shape = std::make_shared<dart::dynamics::SphereShape>(0.5);

  const Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  const Eigen::Vector3d from(-2.0, 0.0, 0.0);
  const Eigen::Vector3d to(2.0, 0.0, 0.0);

  RunRaycastDetectorBenchmark(state, detector, shape, transform, from, to);
}
BENCHMARK(BM_Raycast_Sphere_Bullet);
#endif

static void BM_Raycast_Box_Experimental(benchmark::State& state)
{
  CollisionWorld world;
  auto shape = std::make_unique<BoxShape>(Eigen::Vector3d(0.5, 0.5, 0.5));
  world.createObject(std::move(shape), Eigen::Isometry3d::Identity());

  const Eigen::Vector3d origin(-2.0, 0.0, 0.0);
  const Eigen::Vector3d direction(4.0, 0.0, 0.0);
  const Ray ray(origin, direction, direction.norm());

  RaycastOption option;
  option.maxDistance = direction.norm();
  option.backfaceCulling = true;

  RaycastResult result;

  for (auto _ : state) {
    result.clear();
    benchmark::DoNotOptimize(world.raycast(ray, option, result));
  }
}
BENCHMARK(BM_Raycast_Box_Experimental);

#if DART_HAVE_BULLET
static void BM_Raycast_Box_Bullet(benchmark::State& state)
{
  auto detector = dart::collision::BulletCollisionDetector::create();
  auto shape = std::make_shared<dart::dynamics::BoxShape>(
      Eigen::Vector3d(1.0, 1.0, 1.0));

  const Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  const Eigen::Vector3d from(-2.0, 0.0, 0.0);
  const Eigen::Vector3d to(2.0, 0.0, 0.0);

  RunRaycastDetectorBenchmark(state, detector, shape, transform, from, to);
}
BENCHMARK(BM_Raycast_Box_Bullet);
#endif

static void BM_Raycast_Capsule_Experimental(benchmark::State& state)
{
  CollisionWorld world;
  auto shape = std::make_unique<CapsuleShape>(0.5, 2.0);
  world.createObject(std::move(shape), Eigen::Isometry3d::Identity());

  const Eigen::Vector3d origin(-2.0, 0.0, 0.0);
  const Eigen::Vector3d direction(4.0, 0.0, 0.0);
  const Ray ray(origin, direction, direction.norm());

  RaycastOption option;
  option.maxDistance = direction.norm();
  option.backfaceCulling = true;

  RaycastResult result;

  for (auto _ : state) {
    result.clear();
    benchmark::DoNotOptimize(world.raycast(ray, option, result));
  }
}
BENCHMARK(BM_Raycast_Capsule_Experimental);

#if DART_HAVE_BULLET
static void BM_Raycast_Capsule_Bullet(benchmark::State& state)
{
  auto detector = dart::collision::BulletCollisionDetector::create();
  auto shape = std::make_shared<dart::dynamics::CapsuleShape>(0.5, 2.0);

  const Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  const Eigen::Vector3d from(-2.0, 0.0, 0.0);
  const Eigen::Vector3d to(2.0, 0.0, 0.0);

  RunRaycastDetectorBenchmark(state, detector, shape, transform, from, to);
}
BENCHMARK(BM_Raycast_Capsule_Bullet);
#endif

static void BM_Raycast_Cylinder_Experimental(benchmark::State& state)
{
  CollisionWorld world;
  auto shape = std::make_unique<CylinderShape>(0.5, 2.0);
  world.createObject(std::move(shape), Eigen::Isometry3d::Identity());

  const Eigen::Vector3d origin(-2.0, 0.0, 0.0);
  const Eigen::Vector3d direction(4.0, 0.0, 0.0);
  const Ray ray(origin, direction, direction.norm());

  RaycastOption option;
  option.maxDistance = direction.norm();
  option.backfaceCulling = true;

  RaycastResult result;

  for (auto _ : state) {
    result.clear();
    benchmark::DoNotOptimize(world.raycast(ray, option, result));
  }
}
BENCHMARK(BM_Raycast_Cylinder_Experimental);

#if DART_HAVE_BULLET
static void BM_Raycast_Cylinder_Bullet(benchmark::State& state)
{
  auto detector = dart::collision::BulletCollisionDetector::create();
  auto shape = std::make_shared<dart::dynamics::CylinderShape>(0.5, 2.0);

  const Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  const Eigen::Vector3d from(-2.0, 0.0, 0.0);
  const Eigen::Vector3d to(2.0, 0.0, 0.0);

  RunRaycastDetectorBenchmark(state, detector, shape, transform, from, to);
}
BENCHMARK(BM_Raycast_Cylinder_Bullet);
#endif

static void BM_Raycast_Plane_Experimental(benchmark::State& state)
{
  CollisionWorld world;
  auto shape = std::make_unique<PlaneShape>(Eigen::Vector3d::UnitZ(), 0.0);
  world.createObject(std::move(shape), Eigen::Isometry3d::Identity());

  const Eigen::Vector3d origin(0.0, 0.0, 1.0);
  const Eigen::Vector3d direction(0.0, 0.0, -2.0);
  const Ray ray(origin, direction, direction.norm());

  RaycastOption option;
  option.maxDistance = direction.norm();
  option.backfaceCulling = true;

  RaycastResult result;

  for (auto _ : state) {
    result.clear();
    benchmark::DoNotOptimize(world.raycast(ray, option, result));
  }
}
BENCHMARK(BM_Raycast_Plane_Experimental);

#if DART_HAVE_BULLET
static void BM_Raycast_Plane_Bullet(benchmark::State& state)
{
  auto detector = dart::collision::BulletCollisionDetector::create();
  auto shape = std::make_shared<dart::dynamics::PlaneShape>(
      Eigen::Vector3d::UnitZ(), 0.0);

  const Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  const Eigen::Vector3d from(0.0, 0.0, 1.0);
  const Eigen::Vector3d to(0.0, 0.0, -1.0);

  RunRaycastDetectorBenchmark(state, detector, shape, transform, from, to);
}
BENCHMARK(BM_Raycast_Plane_Bullet);
#endif

BENCHMARK_MAIN();
