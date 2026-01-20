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
#include <dart/dynamics/MeshShape.hpp>
#include <dart/dynamics/Skeleton.hpp>
#include <dart/dynamics/SphereShape.hpp>

#include <dart/math/TriMesh.hpp>

#if DART_HAVE_BULLET
  #include <dart/collision/bullet/BulletCollisionDetector.hpp>
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

struct MeshData
{
  std::vector<Eigen::Vector3d> vertices;
  std::vector<Eigen::Vector3i> triangles;
  std::shared_ptr<dart::math::TriMeshd> triMesh;
};

constexpr double kSphereRadius = 0.4;
constexpr double kCapsuleRadius = 0.3;
constexpr double kCapsuleHeight = 1.0;
constexpr double kSceneRange = 10.0;
constexpr int kMeshSubdivisions = 40;
constexpr double kMeshHalfExtent = 1.0;
constexpr std::size_t kMeshCount = 50;
const Eigen::Vector3d kBoxHalfExtents(0.4, 0.4, 0.4);
const Eigen::Vector3d kBoxSize(0.8, 0.8, 0.8);

std::size_t MaxContactsForCount(std::size_t count)
{
  return std::max<std::size_t>(1000, count * 10);
}

MeshData BuildGridMesh(int subdivisions, double halfExtent)
{
  MeshData data;
  const int vertsPerSide = subdivisions + 1;
  const double step = (2.0 * halfExtent) / subdivisions;

  data.vertices.reserve(static_cast<std::size_t>(vertsPerSide * vertsPerSide));
  for (int i = 0; i < vertsPerSide; ++i) {
    const double x = -halfExtent + step * i;
    for (int j = 0; j < vertsPerSide; ++j) {
      const double y = -halfExtent + step * j;
      data.vertices.emplace_back(x, y, 0.0);
    }
  }

  data.triangles.reserve(
      static_cast<std::size_t>(subdivisions * subdivisions * 2));
  for (int i = 0; i < subdivisions; ++i) {
    for (int j = 0; j < subdivisions; ++j) {
      const int idx0 = i * vertsPerSide + j;
      const int idx1 = idx0 + 1;
      const int idx2 = idx0 + vertsPerSide;
      const int idx3 = idx2 + 1;
      data.triangles.emplace_back(idx0, idx2, idx1);
      data.triangles.emplace_back(idx1, idx2, idx3);
    }
  }

  data.triMesh = std::make_shared<dart::math::TriMeshd>();
  data.triMesh->reserveVertices(data.vertices.size());
  for (const auto& vertex : data.vertices) {
    data.triMesh->addVertex(vertex);
  }
  data.triMesh->reserveTriangles(data.triangles.size());
  for (const auto& triangle : data.triangles) {
    data.triMesh->addTriangle(triangle[0], triangle[1], triangle[2]);
  }

  return data;
}

std::unique_ptr<Shape> MakeExperimentalPrimitive(ShapeKind kind)
{
  switch (kind) {
    case ShapeKind::Sphere:
      return std::make_unique<SphereShape>(kSphereRadius);
    case ShapeKind::Box:
      return std::make_unique<BoxShape>(kBoxHalfExtents);
    case ShapeKind::Capsule:
      return std::make_unique<CapsuleShape>(kCapsuleRadius, kCapsuleHeight);
  }

  return std::make_unique<SphereShape>(kSphereRadius);
}

std::shared_ptr<dart::dynamics::Shape> MakeDynamicsPrimitive(ShapeKind kind)
{
  switch (kind) {
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

void BuildExperimentalScene(
    std::size_t primitiveCount,
    const MeshData& mesh,
    CollisionWorld& world,
    std::vector<CollisionObject>& objects)
{
  auto rng = dart::benchmark::collision::MakeDeterministicRng(101);
  objects.reserve(kMeshCount + primitiveCount);

  for (std::size_t i = 0; i < kMeshCount; ++i) {
    auto transform = dart::benchmark::collision::RandomTransformWithRotation(
        rng, kSceneRange);
    auto shape = std::make_unique<MeshShape>(mesh.vertices, mesh.triangles);
    objects.emplace_back(world.createObject(std::move(shape), transform));
  }

  for (std::size_t i = 0; i < primitiveCount; ++i) {
    ShapeKind kind = ShapeKind::Sphere;
    switch (i % 3) {
      case 0:
        kind = ShapeKind::Sphere;
        break;
      case 1:
        kind = ShapeKind::Box;
        break;
      default:
        kind = ShapeKind::Capsule;
        break;
    }
    auto transform = dart::benchmark::collision::RandomTransformWithRotation(
        rng, kSceneRange);
    objects.emplace_back(
        world.createObject(MakeExperimentalPrimitive(kind), transform));
  }
}

template <typename DetectorPtr>
void BuildDetectorScene(
    const DetectorPtr& detector,
    std::size_t primitiveCount,
    const MeshData& mesh,
    std::vector<dart::dynamics::SkeletonPtr>& skeletons,
    dart::collision::CollisionGroupPtr& group)
{
  auto rng = dart::benchmark::collision::MakeDeterministicRng(101);
  skeletons.reserve(kMeshCount + primitiveCount);
  group = detector->createCollisionGroup();

  for (std::size_t i = 0; i < kMeshCount; ++i) {
    auto transform = dart::benchmark::collision::RandomTransformWithRotation(
        rng, kSceneRange);
    auto shape = std::make_shared<dart::dynamics::MeshShape>(
        Eigen::Vector3d::Ones(), mesh.triMesh);
    auto skel = dart::benchmark::collision::CreateSingleShapeSkeleton(
        shape, transform);
    skeletons.push_back(skel);
    dart::benchmark::collision::AddSkeletonToGroup(group.get(), skel);
  }

  for (std::size_t i = 0; i < primitiveCount; ++i) {
    ShapeKind kind = ShapeKind::Sphere;
    switch (i % 3) {
      case 0:
        kind = ShapeKind::Sphere;
        break;
      case 1:
        kind = ShapeKind::Box;
        break;
      default:
        kind = ShapeKind::Capsule;
        break;
    }
    auto transform = dart::benchmark::collision::RandomTransformWithRotation(
        rng, kSceneRange);
    auto shape = MakeDynamicsPrimitive(kind);
    auto skel = dart::benchmark::collision::CreateSingleShapeSkeleton(
        shape, transform);
    skeletons.push_back(skel);
    dart::benchmark::collision::AddSkeletonToGroup(group.get(), skel);
  }
}

void RunExperimentalMeshScenario(benchmark::State& state, const MeshData& mesh)
{
  const std::size_t primitiveCount = static_cast<std::size_t>(state.range(0));

  CollisionWorld world;
  std::vector<CollisionObject> objects;
  BuildExperimentalScene(primitiveCount, mesh, world, objects);

  CollisionOption option = CollisionOption::fullContacts(
      MaxContactsForCount(kMeshCount + primitiveCount));
  CollisionResult result;

  for (auto _ : state) {
    result.clear();
    benchmark::DoNotOptimize(world.collide(option, result));
    benchmark::DoNotOptimize(result.numContacts());
  }

  state.SetComplexityN(kMeshCount + primitiveCount);
}

template <typename DetectorPtr>
void RunDetectorMeshScenario(
    benchmark::State& state, const DetectorPtr& detector, const MeshData& mesh)
{
  const std::size_t primitiveCount = static_cast<std::size_t>(state.range(0));

  std::vector<dart::dynamics::SkeletonPtr> skeletons;
  dart::collision::CollisionGroupPtr group;
  BuildDetectorScene(detector, primitiveCount, mesh, skeletons, group);

  dart::collision::CollisionOption option;
  option.maxNumContacts = MaxContactsForCount(kMeshCount + primitiveCount);
  dart::collision::CollisionResult result;

  for (auto _ : state) {
    result.clear();
    benchmark::DoNotOptimize(detector->collide(group.get(), option, &result));
    benchmark::DoNotOptimize(result.getNumContacts());
  }

  state.SetComplexityN(kMeshCount + primitiveCount);
}

} // namespace

static void BM_Scenario_MeshHeavy_Experimental(benchmark::State& state)
{
  static const MeshData mesh
      = BuildGridMesh(kMeshSubdivisions, kMeshHalfExtent);
  RunExperimentalMeshScenario(state, mesh);
}
BENCHMARK(BM_Scenario_MeshHeavy_Experimental)
    ->Arg(1000)
    ->Arg(5000)
    ->Complexity();

static void BM_Scenario_MeshHeavy_FCL(benchmark::State& state)
{
  static const MeshData mesh
      = BuildGridMesh(kMeshSubdivisions, kMeshHalfExtent);
  auto detector = dart::collision::FCLCollisionDetector::create();
  RunDetectorMeshScenario(state, detector, mesh);
}
BENCHMARK(BM_Scenario_MeshHeavy_FCL)->Arg(1000)->Arg(5000)->Complexity();

#if DART_HAVE_BULLET
static void BM_Scenario_MeshHeavy_Bullet(benchmark::State& state)
{
  static const MeshData mesh
      = BuildGridMesh(kMeshSubdivisions, kMeshHalfExtent);
  auto detector = dart::collision::BulletCollisionDetector::create();
  RunDetectorMeshScenario(state, detector, mesh);
}
BENCHMARK(BM_Scenario_MeshHeavy_Bullet)->Arg(1000)->Arg(5000)->Complexity();
#endif

BENCHMARK_MAIN();
