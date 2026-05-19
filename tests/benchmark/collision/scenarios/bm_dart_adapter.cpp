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

#include <dart/collision/collision_group.hpp>
#include <dart/collision/collision_option.hpp>
#include <dart/collision/collision_result.hpp>
#include <dart/collision/dart/dart_collision_detector.hpp>
#include <dart/collision/distance_option.hpp>
#include <dart/collision/distance_result.hpp>
#include <dart/collision/raycast_option.hpp>
#include <dart/collision/raycast_result.hpp>

#include <dart/dynamics/box_shape.hpp>
#include <dart/dynamics/capsule_shape.hpp>
#include <dart/dynamics/free_joint.hpp>
#include <dart/dynamics/shape_node.hpp>
#include <dart/dynamics/skeleton.hpp>
#include <dart/dynamics/sphere_shape.hpp>

#include <benchmark/benchmark.h>

#include <algorithm>
#include <memory>
#include <vector>

#include <cmath>
#include <cstdint>

namespace {

struct AdapterScene
{
  std::shared_ptr<dart::collision::DartCollisionDetector> detector;
  std::unique_ptr<dart::collision::CollisionGroup> group;
  std::vector<dart::dynamics::SkeletonPtr> skeletons;
  std::vector<dart::dynamics::FreeJoint*> joints;
  int gridSide = 0;
  double spacing = 1.0;
};

std::shared_ptr<dart::dynamics::Shape> makeShape(std::size_t index)
{
  switch (index % 3u) {
    case 0u:
      return std::make_shared<dart::dynamics::SphereShape>(0.35);
    case 1u:
      return std::make_shared<dart::dynamics::BoxShape>(
          Eigen::Vector3d::Constant(0.65));
    default:
      return std::make_shared<dart::dynamics::CapsuleShape>(0.25, 0.75);
  }
}

Eigen::Isometry3d makeGridTransform(
    std::size_t index, int side, double spacing, double zOffset = 0.0)
{
  Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
  const auto x = static_cast<int>(index % static_cast<std::size_t>(side));
  const auto y = static_cast<int>(index / static_cast<std::size_t>(side));
  tf.translation() = Eigen::Vector3d(
      static_cast<double>(x) * spacing,
      static_cast<double>(y) * spacing,
      zOffset);
  return tf;
}

AdapterScene makeAdapterScene(std::size_t objectCount, double spacing)
{
  AdapterScene scene;
  scene.detector = dart::collision::DartCollisionDetector::create();
  scene.group = scene.detector->createCollisionGroup();
  scene.skeletons.reserve(objectCount);
  scene.joints.reserve(objectCount);
  scene.gridSide = static_cast<int>(
      std::ceil(std::sqrt(static_cast<double>(objectCount))));
  scene.spacing = spacing;

  for (std::size_t i = 0; i < objectCount; ++i) {
    auto skeleton = dart::dynamics::Skeleton::create();
    auto pair
        = skeleton->createJointAndBodyNodePair<dart::dynamics::FreeJoint>();
    auto* joint = pair.first;
    auto* body = pair.second;
    body->createShapeNodeWith<dart::dynamics::CollisionAspect>(makeShape(i));
    joint->setTransform(makeGridTransform(i, scene.gridSide, spacing));

    scene.group->addShapeFramesOf(skeleton.get());
    scene.joints.push_back(joint);
    scene.skeletons.push_back(std::move(skeleton));
  }

  return scene;
}

void moveDirtySubset(AdapterScene& scene, std::size_t iteration)
{
  const auto count = scene.joints.size();
  const auto dirtyCount = std::max<std::size_t>(1u, count / 32u);
  const auto zOffset = (iteration % 2u == 0u) ? 0.04 : -0.04;

  for (std::size_t i = 0; i < dirtyCount; ++i) {
    const auto index = (iteration * dirtyCount + i * 17u) % count;
    scene.joints[index]->setTransform(
        makeGridTransform(index, scene.gridSide, scene.spacing, zOffset));
  }
}

void setObjectCounter(benchmark::State& state, std::size_t objectCount)
{
  state.counters["objects"] = static_cast<double>(objectCount);
  state.SetItemsProcessed(
      state.iterations() * static_cast<std::int64_t>(objectCount));
}

static void BM_DartAdapter_CollidePersistentScene(benchmark::State& state)
{
  const auto objectCount = static_cast<std::size_t>(state.range(0));
  auto scene = makeAdapterScene(objectCount, 0.8);

  dart::collision::CollisionOption option;
  option.enableContact = true;
  option.maxNumContacts = 10000u;

  dart::collision::CollisionResult result;
  for (auto _ : state) {
    result.clear();
    benchmark::DoNotOptimize(scene.group->collide(option, &result));
    benchmark::DoNotOptimize(result.getNumContacts());
  }

  setObjectCounter(state, objectCount);
}

BENCHMARK(BM_DartAdapter_CollidePersistentScene)
    ->Arg(64)
    ->Arg(256)
    ->Unit(benchmark::kMillisecond);

static void BM_DartAdapter_CollideDirtySubset(benchmark::State& state)
{
  const auto objectCount = static_cast<std::size_t>(state.range(0));
  auto scene = makeAdapterScene(objectCount, 2.0);

  dart::collision::CollisionOption option;
  option.enableContact = false;
  option.maxNumContacts = 1u;

  dart::collision::CollisionResult result;
  std::size_t iteration = 0u;
  for (auto _ : state) {
    moveDirtySubset(scene, iteration++);
    result.clear();
    benchmark::DoNotOptimize(scene.group->collide(option, &result));
    benchmark::DoNotOptimize(result.getNumContacts());
  }

  setObjectCounter(state, objectCount);
}

BENCHMARK(BM_DartAdapter_CollideDirtySubset)
    ->Arg(256)
    ->Arg(1024)
    ->Unit(benchmark::kMillisecond);

static void BM_DartAdapter_DistancePersistentScene(benchmark::State& state)
{
  const auto objectCount = static_cast<std::size_t>(state.range(0));
  auto scene = makeAdapterScene(objectCount, 1.5);

  dart::collision::DistanceOption option;
  option.enableNearestPoints = true;

  dart::collision::DistanceResult result;
  for (auto _ : state) {
    result.clear();
    benchmark::DoNotOptimize(scene.group->distance(option, &result));
    benchmark::DoNotOptimize(result.minDistance);
  }

  setObjectCounter(state, objectCount);
}

BENCHMARK(BM_DartAdapter_DistancePersistentScene)
    ->Arg(64)
    ->Arg(256)
    ->Unit(benchmark::kMillisecond);

static void BM_DartAdapter_RaycastPersistentScene(benchmark::State& state)
{
  const auto objectCount = static_cast<std::size_t>(state.range(0));
  auto scene = makeAdapterScene(objectCount, 1.0);
  const Eigen::Vector3d from(-1.0, 0.0, 0.0);
  const Eigen::Vector3d to(
      static_cast<double>(scene.gridSide) * scene.spacing + 1.0, 0.0, 0.0);

  dart::collision::RaycastOption option;
  dart::collision::RaycastResult result;
  for (auto _ : state) {
    result.clear();
    benchmark::DoNotOptimize(scene.group->raycast(from, to, option, &result));
    benchmark::DoNotOptimize(result.mRayHits.size());
  }

  setObjectCounter(state, objectCount);
}

BENCHMARK(BM_DartAdapter_RaycastPersistentScene)
    ->Arg(256)
    ->Arg(1024)
    ->Unit(benchmark::kMillisecond);

} // namespace

BENCHMARK_MAIN();
