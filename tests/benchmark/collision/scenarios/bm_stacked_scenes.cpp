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

#include <dart/collision/native/collision_world.hpp>
#include <dart/collision/native/shapes/shape.hpp>
#include <dart/collision/native/types.hpp>
#include <dart/collision/raycast_option.hpp>
#include <dart/collision/raycast_result.hpp>

#include <benchmark/benchmark.h>

#include <algorithm>
#include <memory>
#include <vector>

#include <cmath>

using namespace dart::collision::native;

namespace {

enum class ShapeKind
{
  Box,
  Sphere,
  Capsule,
  Convex,
  Mesh,
};

struct ObjectSpec
{
  ShapeKind kind = ShapeKind::Box;
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  Eigen::Vector3d halfExtents = Eigen::Vector3d::Ones();
  double radius = 0.5;
  double height = 1.0;
  std::vector<Eigen::Vector3d> vertices;
  std::vector<Eigen::Vector3i> triangles;
  bool movable = true;
};

struct RaySegment
{
  Eigen::Vector3d from;
  Eigen::Vector3d to;
};

struct MeshData
{
  std::vector<Eigen::Vector3d> vertices;
  std::vector<Eigen::Vector3i> triangles;
};

constexpr double kPi = 3.14159265358979323846;
constexpr double kContactOverlap = 0.02;
constexpr double kRaycastRange = 80.0;
constexpr std::size_t kMovingRayCount = 500;

Eigen::Isometry3d MakeTransform(
    const Eigen::Vector3d& translation,
    const Eigen::Vector3d& axis = Eigen::Vector3d::UnitZ(),
    double angle = 0.0)
{
  Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
  tf.translation() = translation;
  if (std::abs(angle) > 0.0) {
    tf.linear()
        = Eigen::AngleAxisd(angle, axis.normalized()).toRotationMatrix();
  }
  return tf;
}

ObjectSpec MakeBox(
    const Eigen::Vector3d& translation,
    const Eigen::Vector3d& halfExtents,
    bool movable = true,
    double yaw = 0.0)
{
  ObjectSpec spec;
  spec.kind = ShapeKind::Box;
  spec.transform = MakeTransform(translation, Eigen::Vector3d::UnitZ(), yaw);
  spec.halfExtents = halfExtents;
  spec.movable = movable;
  return spec;
}

ObjectSpec MakeSphere(
    const Eigen::Vector3d& translation, double radius, bool movable = true)
{
  ObjectSpec spec;
  spec.kind = ShapeKind::Sphere;
  spec.transform = MakeTransform(translation);
  spec.radius = radius;
  spec.movable = movable;
  return spec;
}

ObjectSpec MakeCapsule(
    const Eigen::Vector3d& translation,
    double radius,
    double height,
    bool movable = true,
    double yaw = 0.0)
{
  ObjectSpec spec;
  spec.kind = ShapeKind::Capsule;
  spec.transform = MakeTransform(translation, Eigen::Vector3d::UnitZ(), yaw);
  spec.radius = radius;
  spec.height = height;
  spec.movable = movable;
  return spec;
}

ObjectSpec MakeConvex(
    const Eigen::Vector3d& translation,
    std::vector<Eigen::Vector3d> vertices,
    bool movable = true,
    const Eigen::Vector3d& axis = Eigen::Vector3d::UnitZ(),
    double angle = 0.0)
{
  ObjectSpec spec;
  spec.kind = ShapeKind::Convex;
  spec.transform = MakeTransform(translation, axis, angle);
  spec.vertices = std::move(vertices);
  spec.movable = movable;
  return spec;
}

ObjectSpec MakeMesh(
    const Eigen::Vector3d& translation,
    MeshData mesh,
    bool movable = false,
    double yaw = 0.0)
{
  ObjectSpec spec;
  spec.kind = ShapeKind::Mesh;
  spec.transform = MakeTransform(translation, Eigen::Vector3d::UnitZ(), yaw);
  spec.vertices = std::move(mesh.vertices);
  spec.triangles = std::move(mesh.triangles);
  spec.movable = movable;
  return spec;
}

std::unique_ptr<Shape> MakeShape(const ObjectSpec& spec)
{
  switch (spec.kind) {
    case ShapeKind::Box:
      return std::make_unique<BoxShape>(spec.halfExtents);
    case ShapeKind::Sphere:
      return std::make_unique<SphereShape>(spec.radius);
    case ShapeKind::Capsule:
      return std::make_unique<CapsuleShape>(spec.radius, spec.height);
    case ShapeKind::Convex:
      return std::make_unique<ConvexShape>(spec.vertices);
    case ShapeKind::Mesh:
      return std::make_unique<MeshShape>(spec.vertices, spec.triangles);
  }

  return std::make_unique<BoxShape>(spec.halfExtents);
}

std::size_t MaxContactsForCount(std::size_t objectCount)
{
  return std::max<std::size_t>(2000, objectCount * 24);
}

std::vector<Eigen::Vector3d> MakeElongatedConvexVertices(double scale = 1.0)
{
  std::vector<Eigen::Vector3d> vertices;
  vertices.reserve(22);

  for (int ring = -1; ring <= 1; ++ring) {
    const double z = static_cast<double>(ring) * 1.15 * scale;
    const double rx = (ring == 0 ? 1.10 : 0.82) * scale;
    const double ry = (ring == 0 ? 0.72 : 0.55) * scale;
    for (int i = 0; i < 6; ++i) {
      const double theta = (2.0 * kPi * static_cast<double>(i)) / 6.0;
      vertices.emplace_back(rx * std::cos(theta), ry * std::sin(theta), z);
    }
  }

  vertices.emplace_back(0.0, 0.0, 1.85 * scale);
  vertices.emplace_back(0.0, 0.0, -1.85 * scale);
  vertices.emplace_back(1.32 * scale, 0.0, 0.0);
  vertices.emplace_back(-1.32 * scale, 0.0, 0.0);
  return vertices;
}

std::vector<Eigen::Vector3d> MakeHaltonCellVertices(int cellIndex)
{
  const int vertexCount = 12 + (cellIndex % 10);
  std::vector<Eigen::Vector3d> vertices;
  vertices.reserve(static_cast<std::size_t>(vertexCount));

  for (int i = 0; i < vertexCount; ++i) {
    const double theta
        = 2.0 * kPi * std::fmod(0.61803398875 * (cellIndex + i + 1), 1.0);
    const double z = -0.52 + 1.04 * std::fmod(0.754877666 * (i + 3), 1.0);
    const double rx
        = 0.32 + 0.18 * std::fmod(0.56984029 * (cellIndex + 7), 1.0);
    const double ry = 0.28 + 0.20 * std::fmod(0.43857991 * (i + 11), 1.0);
    vertices.emplace_back(rx * std::cos(theta), ry * std::sin(theta), z);
  }

  vertices.emplace_back(0.0, 0.0, 0.62);
  vertices.emplace_back(0.0, 0.0, -0.62);
  return vertices;
}

MeshData MakeRippleTerrain(
    int subdivisions, double halfExtent, double amplitude)
{
  MeshData mesh;
  const int verticesPerSide = subdivisions + 1;
  const double step = (2.0 * halfExtent) / static_cast<double>(subdivisions);

  mesh.vertices.reserve(
      static_cast<std::size_t>(verticesPerSide * verticesPerSide));
  for (int x = 0; x < verticesPerSide; ++x) {
    const double px = -halfExtent + step * static_cast<double>(x);
    for (int y = 0; y < verticesPerSide; ++y) {
      const double py = -halfExtent + step * static_cast<double>(y);
      const double pz = amplitude * std::sin(0.35 * px) * std::cos(0.31 * py);
      mesh.vertices.emplace_back(px, py, pz);
    }
  }

  mesh.triangles.reserve(
      static_cast<std::size_t>(subdivisions * subdivisions * 2));
  for (int x = 0; x < subdivisions; ++x) {
    for (int y = 0; y < subdivisions; ++y) {
      const int i0 = x * verticesPerSide + y;
      const int i1 = i0 + 1;
      const int i2 = i0 + verticesPerSide;
      const int i3 = i2 + 1;
      mesh.triangles.emplace_back(i0, i2, i1);
      mesh.triangles.emplace_back(i1, i2, i3);
    }
  }

  return mesh;
}

void AddGroundBox(std::vector<ObjectSpec>& specs, double halfExtent)
{
  specs.push_back(MakeBox(
      Eigen::Vector3d(0.0, 0.0, -0.52),
      Eigen::Vector3d(halfExtent, halfExtent, 0.5),
      false));
}

std::vector<ObjectSpec> MakeBoxGridStack(int layers)
{
  std::vector<ObjectSpec> specs;
  AddGroundBox(specs, 160.0);

  constexpr int size = 8;
  const Eigen::Vector3d halfExtents(1.0, 1.0, 1.0);
  const double step = 2.0 * halfExtents.x() + 0.04;
  const double verticalStep = 2.0 * halfExtents.z() - kContactOverlap;
  const double offset = -0.5 * static_cast<double>(size - 1) * step;

  specs.reserve(1 + static_cast<std::size_t>(layers * size * size));
  for (int layer = 0; layer < layers; ++layer) {
    for (int y = 0; y < size; ++y) {
      for (int x = 0; x < size; ++x) {
        specs.push_back(MakeBox(
            Eigen::Vector3d(
                offset + static_cast<double>(x) * step,
                offset + static_cast<double>(y) * step,
                halfExtents.z() - kContactOverlap
                    + static_cast<double>(layer) * verticalStep),
            halfExtents,
            true,
            0.02 * static_cast<double>((x + y + layer) % 5)));
      }
    }
  }

  return specs;
}

void AddWall(
    std::vector<ObjectSpec>& specs,
    const Eigen::Vector3d& offset,
    int stackSize,
    const Eigen::Vector3d& halfExtents)
{
  const double stepY = 2.0 * halfExtents.y() + 0.02;
  const double stepZ = 2.0 * halfExtents.z() - kContactOverlap;
  double rowOffset = -static_cast<double>(stackSize) * stepY * 0.5;
  double z = halfExtents.z() - kContactOverlap;

  while (stackSize > 0) {
    for (int i = 0; i < stackSize; ++i) {
      specs.push_back(MakeBox(
          offset
              + Eigen::Vector3d(
                  0.0, rowOffset + static_cast<double>(i) * stepY, z),
          halfExtents));
    }
    rowOffset += halfExtents.y();
    z += stepZ;
    --stackSize;
  }
}

void AddPyramid(
    std::vector<ObjectSpec>& specs,
    const Eigen::Vector3d& offset,
    int stackSize,
    const Eigen::Vector3d& halfExtents)
{
  const double stepX = 2.04 * halfExtents.x();
  const double stepY = 2.04 * halfExtents.y();
  const double stepZ = 2.0 * halfExtents.z() - kContactOverlap;
  double offsetX = -static_cast<double>(stackSize) * stepX * 0.5;
  double offsetY = -static_cast<double>(stackSize) * stepY * 0.5;
  double z = halfExtents.z() - kContactOverlap;

  while (stackSize > 0) {
    for (int y = 0; y < stackSize; ++y) {
      for (int x = 0; x < stackSize; ++x) {
        specs.push_back(MakeBox(
            offset
                + Eigen::Vector3d(
                    offsetX + static_cast<double>(x) * stepX,
                    offsetY + static_cast<double>(y) * stepY,
                    z),
            halfExtents));
      }
    }
    offsetX += halfExtents.x();
    offsetY += halfExtents.y();
    z += stepZ;
    --stackSize;
  }
}

void AddCircularTower(
    std::vector<ObjectSpec>& specs,
    const Eigen::Vector3d& offset,
    int layers,
    int piecesPerLayer,
    const Eigen::Vector3d& halfExtents)
{
  const double radius
      = 1.3 * static_cast<double>(piecesPerLayer) * halfExtents.x() / kPi;
  const double verticalStep = 2.0 * halfExtents.z() - kContactOverlap;

  for (int layer = 0; layer < layers; ++layer) {
    const double z = halfExtents.z() - kContactOverlap
                     + static_cast<double>(layer) * verticalStep;
    const double layerYaw = kPi * static_cast<double>(layer)
                            / static_cast<double>(piecesPerLayer);
    for (int i = 0; i < piecesPerLayer; ++i) {
      const double yaw = layerYaw
                         + 2.0 * kPi * static_cast<double>(i)
                               / static_cast<double>(piecesPerLayer);
      specs.push_back(MakeBox(
          offset
              + Eigen::Vector3d(
                  radius * std::cos(yaw), radius * std::sin(yaw), z),
          halfExtents,
          true,
          yaw));
    }
  }
}

std::vector<ObjectSpec> MakeCompoundBoxStructures()
{
  std::vector<ObjectSpec> specs;
  AddGroundBox(specs, 100.0);

  const Eigen::Vector3d halfExtents(1.0, 1.0, 1.0);
  AddPyramid(specs, Eigen::Vector3d(-24.0, 0.0, 0.0), 12, halfExtents);
  AddWall(specs, Eigen::Vector3d(-3.0, -10.0, 0.0), 12, halfExtents);
  AddWall(specs, Eigen::Vector3d(4.0, 0.0, 0.0), 12, halfExtents);
  AddWall(specs, Eigen::Vector3d(11.0, 10.0, 0.0), 12, halfExtents);
  AddCircularTower(specs, Eigen::Vector3d(28.0, 0.0, 0.0), 8, 24, halfExtents);
  return specs;
}

std::vector<ObjectSpec> MakeScaledPrimitiveStack(int baseSize)
{
  std::vector<ObjectSpec> specs;
  AddGroundBox(specs, 180.0);

  const int layers = std::max(6, baseSize);
  const double step = 5.6;
  const double offset = -0.5 * static_cast<double>(baseSize - 1) * step;
  const double verticalStep = 2.35;

  specs.reserve(1 + static_cast<std::size_t>(layers * baseSize * baseSize));
  for (int layer = 0; layer < layers; ++layer) {
    for (int y = 0; y < baseSize; ++y) {
      for (int x = 0; x < baseSize; ++x) {
        const int shapeIndex = (x + 3 * y + 5 * layer) % 9;
        const double scale = 0.55 + 0.22 * static_cast<double>(shapeIndex % 3);
        const Eigen::Vector3d pos(
            offset + static_cast<double>(x) * step,
            offset + static_cast<double>(y) * step,
            1.0 + static_cast<double>(layer) * verticalStep);

        if (shapeIndex < 3) {
          specs.push_back(MakeBox(
              pos,
              Eigen::Vector3d(1.45, 1.45, 1.45) * scale,
              true,
              0.08 * static_cast<double>(shapeIndex + layer)));
        } else if (shapeIndex < 6) {
          specs.push_back(MakeSphere(pos, 1.45 * scale));
        } else {
          specs.push_back(MakeCapsule(
              pos,
              0.95 * scale,
              2.0 * scale,
              true,
              0.11 * static_cast<double>(shapeIndex + x)));
        }
      }
    }
  }

  return specs;
}

std::vector<ObjectSpec> MakeElongatedConvexStack(int gridSize)
{
  std::vector<ObjectSpec> specs;
  AddGroundBox(specs, 160.0);

  const int layers = 10;
  const double spacing = 3.15;
  const double offset = -0.5 * static_cast<double>(gridSize - 1) * spacing;
  const double verticalStep = 3.25;

  specs.reserve(1 + static_cast<std::size_t>(layers * gridSize * gridSize));
  for (int layer = 0; layer < layers; ++layer) {
    const double scale = 0.88 + 0.02 * static_cast<double>(layer % 4);
    auto vertices = MakeElongatedConvexVertices(scale);
    for (int y = 0; y < gridSize; ++y) {
      for (int x = 0; x < gridSize; ++x) {
        specs.push_back(MakeConvex(
            Eigen::Vector3d(
                offset + static_cast<double>(x) * spacing,
                offset + static_cast<double>(y) * spacing,
                1.55 + static_cast<double>(layer) * verticalStep),
            vertices,
            true,
            Eigen::Vector3d(0.2, 0.3, 1.0),
            0.12 * static_cast<double>(x + y + layer)));
      }
    }
  }

  return specs;
}

std::vector<ObjectSpec> MakeMixedPrimitiveStack(int gridSize)
{
  std::vector<ObjectSpec> specs = MakeScaledPrimitiveStack(gridSize);
  specs.push_back(MakeMesh(
      Eigen::Vector3d(0.0, 0.0, -0.03), MakeRippleTerrain(16, 55.0, 0.22)));
  return specs;
}

std::vector<ObjectSpec> MakeConvexTerrainSnapshot(std::size_t objectCount)
{
  std::vector<ObjectSpec> specs;
  specs.reserve(1 + objectCount);
  specs.push_back(MakeMesh(
      Eigen::Vector3d(0.0, 0.0, -0.08), MakeRippleTerrain(24, 65.0, 0.35)));

  const auto baseVertices = MakeElongatedConvexVertices(0.55);
  const int rowSize = static_cast<int>(
      std::ceil(std::sqrt(static_cast<double>(objectCount))));
  const double spacing = 2.75;
  const double offset = -0.5 * static_cast<double>(rowSize - 1) * spacing;

  for (std::size_t i = 0; i < objectCount; ++i) {
    const int row = static_cast<int>(i) / rowSize;
    const int col = static_cast<int>(i) % rowSize;
    const double x = offset + static_cast<double>(col) * spacing;
    const double y = offset + static_cast<double>(row) * spacing;
    const double z = 0.45 + 0.08 * static_cast<double>((row + col) % 5);
    specs.push_back(MakeConvex(
        Eigen::Vector3d(x, y, z),
        baseVertices,
        true,
        Eigen::Vector3d(0.3, 0.7, 1.0),
        0.17 * static_cast<double>(i % 31)));
  }

  return specs;
}

double Halton(int index, int base)
{
  double result = 0.0;
  double fraction = 1.0 / static_cast<double>(base);
  while (index > 0) {
    result += fraction * static_cast<double>(index % base);
    index /= base;
    fraction /= static_cast<double>(base);
  }
  return result;
}

std::vector<ObjectSpec> MakeHaltonConvexCells(int cellCount)
{
  std::vector<ObjectSpec> specs;
  AddGroundBox(specs, 12.0);
  specs.reserve(1 + static_cast<std::size_t>(cellCount));

  for (int i = 0; i < cellCount; ++i) {
    const Eigen::Vector3d pos(
        -5.0 + 10.0 * Halton(i + 1, 2),
        -5.0 + 10.0 * Halton(i + 1, 3),
        0.48 + 0.35 * Halton(i + 1, 5));
    specs.push_back(MakeConvex(
        pos,
        MakeHaltonCellVertices(i),
        true,
        Eigen::Vector3d(0.1 + Halton(i + 1, 7), 0.3, 1.0),
        2.0 * kPi * Halton(i + 1, 11)));
  }

  return specs;
}

void BuildWorld(
    const std::vector<ObjectSpec>& specs,
    CollisionWorld& world,
    std::vector<CollisionObject>& objects,
    std::vector<Eigen::Isometry3d>& baseTransforms,
    std::vector<bool>& movable)
{
  objects.reserve(specs.size());
  baseTransforms.reserve(specs.size());
  movable.reserve(specs.size());

  for (const auto& spec : specs) {
    baseTransforms.push_back(spec.transform);
    movable.push_back(spec.movable);
    objects.emplace_back(world.createObject(MakeShape(spec), spec.transform));
  }
}

void ApplyDeterministicMotion(
    std::vector<CollisionObject>& objects,
    const std::vector<Eigen::Isometry3d>& baseTransforms,
    const std::vector<bool>& movable,
    std::size_t iteration)
{
  const double phase = (static_cast<double>(iteration % 17) - 8.0) * 0.0025;
  for (std::size_t i = 0; i < objects.size(); ++i) {
    if (!movable[i]) {
      continue;
    }

    Eigen::Isometry3d transform = baseTransforms[i];
    transform.translation().x()
        += phase * static_cast<double>(static_cast<int>(i % 5) - 2);
    transform.translation().y()
        += phase * static_cast<double>(static_cast<int>((i / 5) % 5) - 2);
    objects[i].setTransform(transform);
  }
}

void ConsumeCollisionResult(
    const CollisionResult& result, const BatchStats& stats)
{
  auto contacts = result.numContacts();
  auto pairsTested = stats.numPairsTested;
  benchmark::DoNotOptimize(contacts);
  benchmark::DoNotOptimize(pairsTested);
  benchmark::ClobberMemory();
}

void RunCollisionScenario(
    benchmark::State& state, const std::vector<ObjectSpec>& specs)
{
  CollisionWorld world;
  std::vector<CollisionObject> objects;
  std::vector<Eigen::Isometry3d> baseTransforms;
  std::vector<bool> movable;
  BuildWorld(specs, world, objects, baseTransforms, movable);

  CollisionOption option
      = CollisionOption::fullContacts(MaxContactsForCount(specs.size()));
  CollisionResult result;
  BatchStats stats;

  std::size_t totalUpdated = world.updateAll();
  BroadPhaseSnapshot snapshot = world.buildBroadPhaseSnapshot();
  world.collideAll(snapshot, option, result, &stats);
  if (result.numContacts() == 0u) {
    state.SkipWithError("Scenario setup produced no native contacts.");
    return;
  }

  std::size_t totalContacts = 0;
  std::size_t totalPairs = 0;
  std::size_t totalPairsTested = 0;
  std::size_t iteration = 0;
  for (auto _ : state) {
    ApplyDeterministicMotion(objects, baseTransforms, movable, iteration);
    totalUpdated += world.updateAll();
    snapshot = world.buildBroadPhaseSnapshot();
    result.clear();
    stats.clear();
    world.collideAll(snapshot, option, result, &stats);
    totalContacts += result.numContacts();
    totalPairs += snapshot.pairs.size();
    totalPairsTested += stats.numPairsTested;
    ConsumeCollisionResult(result, stats);
    ++iteration;
  }

  state.SetComplexityN(static_cast<int64_t>(specs.size()));
  state.counters["objects"] = static_cast<double>(specs.size());
  state.counters["contacts"]
      = benchmark::Counter(totalContacts, benchmark::Counter::kAvgIterations);
  state.counters["pairs"]
      = benchmark::Counter(totalPairs, benchmark::Counter::kAvgIterations);
  state.counters["pairs_tested"] = benchmark::Counter(
      totalPairsTested, benchmark::Counter::kAvgIterations);
  state.counters["aabb_updates"]
      = benchmark::Counter(totalUpdated, benchmark::Counter::kAvgIterations);
}

std::vector<RaySegment> MakeMovingRaySegments(std::size_t count)
{
  std::vector<RaySegment> segments;
  segments.reserve(count);

  for (std::size_t i = 0; i < count; ++i) {
    const double theta
        = 2.0 * kPi
          * std::fmod(0.38196601125 * static_cast<double>(i + 1), 1.0);
    const double z = 3.5 + 0.01 * static_cast<double>(i % 19);
    const Eigen::Vector3d dir(std::cos(theta), std::sin(theta), 0.0);
    RaySegment segment;
    segment.from = kRaycastRange * dir + Eigen::Vector3d(0.0, 0.0, z);
    segment.to = -kRaycastRange * dir + Eigen::Vector3d(0.0, 0.0, z - 3.0);
    segments.push_back(segment);
  }

  return segments;
}

void RunCollisionAndRaycastScenario(
    benchmark::State& state, const std::vector<ObjectSpec>& specs)
{
  CollisionWorld world(BroadPhaseType::SweepAndPrune);
  std::vector<CollisionObject> objects;
  std::vector<Eigen::Isometry3d> baseTransforms;
  std::vector<bool> movable;
  BuildWorld(specs, world, objects, baseTransforms, movable);

  const auto raySegments = MakeMovingRaySegments(kMovingRayCount);
  CollisionOption collisionOption
      = CollisionOption::fullContacts(MaxContactsForCount(specs.size()));
  RaycastOption rayOption = RaycastOption::unlimited();
  CollisionResult collisionResult;
  RaycastResult rayResult;
  BatchStats stats;

  std::size_t totalUpdated = world.updateAll();
  BroadPhaseSnapshot snapshot = world.buildBroadPhaseSnapshot();
  world.collideAll(snapshot, collisionOption, collisionResult, &stats);
  if (collisionResult.numContacts() == 0u) {
    state.SkipWithError("Scenario setup produced no native contacts.");
    return;
  }

  std::size_t totalContacts = 0;
  std::size_t totalHits = 0;
  std::size_t totalPairs = 0;
  std::size_t iteration = 0;
  for (auto _ : state) {
    ApplyDeterministicMotion(objects, baseTransforms, movable, iteration);
    totalUpdated += world.updateAll();

    snapshot = world.buildBroadPhaseSnapshot();
    collisionResult.clear();
    stats.clear();
    world.collideAll(snapshot, collisionOption, collisionResult, &stats);

    std::size_t hits = 0;
    const double rayShift = 0.01 * static_cast<double>(iteration % 13);
    for (const auto& segment : raySegments) {
      rayResult.clear();
      const Eigen::Vector3d from
          = segment.from + Eigen::Vector3d(rayShift, -rayShift, 0.0);
      const Eigen::Vector3d to
          = segment.to + Eigen::Vector3d(rayShift, -rayShift, 0.0);
      const Eigen::Vector3d direction = to - from;
      const Ray ray(from, direction, direction.norm());
      if (world.raycast(ray, rayOption, rayResult)) {
        ++hits;
      }
    }

    totalContacts += collisionResult.numContacts();
    totalHits += hits;
    totalPairs += snapshot.pairs.size();
    benchmark::DoNotOptimize(hits);
    ConsumeCollisionResult(collisionResult, stats);
    ++iteration;
  }

  state.SetItemsProcessed(
      static_cast<int64_t>(state.iterations() * raySegments.size()));
  state.SetComplexityN(static_cast<int64_t>(specs.size()));
  state.counters["objects"] = static_cast<double>(specs.size());
  state.counters["rays"] = static_cast<double>(raySegments.size());
  state.counters["ray_hits"]
      = benchmark::Counter(totalHits, benchmark::Counter::kAvgIterations);
  state.counters["contacts"]
      = benchmark::Counter(totalContacts, benchmark::Counter::kAvgIterations);
  state.counters["pairs"]
      = benchmark::Counter(totalPairs, benchmark::Counter::kAvgIterations);
  state.counters["aabb_updates"]
      = benchmark::Counter(totalUpdated, benchmark::Counter::kAvgIterations);
}

} // namespace

static void BM_BoxGridStack_ContactPipeline(benchmark::State& state)
{
  RunCollisionScenario(
      state, MakeBoxGridStack(static_cast<int>(state.range(0))));
}
BENCHMARK(BM_BoxGridStack_ContactPipeline)
    ->Arg(16)
    ->Arg(32)
    ->Arg(47)
    ->Complexity();

static void BM_CompoundBoxStructures_ContactPipeline(benchmark::State& state)
{
  RunCollisionScenario(state, MakeCompoundBoxStructures());
}
BENCHMARK(BM_CompoundBoxStructures_ContactPipeline)->Complexity();

static void BM_ScaledPrimitiveStack_ContactPipeline(benchmark::State& state)
{
  RunCollisionScenario(
      state, MakeScaledPrimitiveStack(static_cast<int>(state.range(0))));
}
BENCHMARK(BM_ScaledPrimitiveStack_ContactPipeline)
    ->Arg(8)
    ->Arg(10)
    ->Complexity();

static void BM_ElongatedConvexHullStack_ContactPipeline(benchmark::State& state)
{
  RunCollisionScenario(
      state, MakeElongatedConvexStack(static_cast<int>(state.range(0))));
}
BENCHMARK(BM_ElongatedConvexHullStack_ContactPipeline)
    ->Arg(6)
    ->Arg(8)
    ->Complexity();

static void BM_MixedPrimitiveStackWithTerrain_ContactPipeline(
    benchmark::State& state)
{
  RunCollisionScenario(
      state, MakeMixedPrimitiveStack(static_cast<int>(state.range(0))));
}
BENCHMARK(BM_MixedPrimitiveStackWithTerrain_ContactPipeline)
    ->Arg(8)
    ->Arg(10)
    ->Complexity();

static void BM_ConvexHullTerrainSnapshot_ContactPipeline(
    benchmark::State& state)
{
  RunCollisionScenario(
      state,
      MakeConvexTerrainSnapshot(static_cast<std::size_t>(state.range(0))));
}
BENCHMARK(BM_ConvexHullTerrainSnapshot_ContactPipeline)
    ->Arg(400)
    ->Arg(1000)
    ->Complexity();

static void BM_ConvexHullTerrainSnapshot_MovingRaycasts(benchmark::State& state)
{
  RunCollisionAndRaycastScenario(
      state,
      MakeConvexTerrainSnapshot(static_cast<std::size_t>(state.range(0))));
}
BENCHMARK(BM_ConvexHullTerrainSnapshot_MovingRaycasts)
    ->Arg(400)
    ->Arg(1000)
    ->Complexity();

static void BM_HaltonConvexCellPack_ContactPipeline(benchmark::State& state)
{
  RunCollisionScenario(
      state, MakeHaltonConvexCells(static_cast<int>(state.range(0))));
}
BENCHMARK(BM_HaltonConvexCellPack_ContactPipeline)
    ->Arg(45)
    ->Arg(90)
    ->Complexity();

BENCHMARK_MAIN();
