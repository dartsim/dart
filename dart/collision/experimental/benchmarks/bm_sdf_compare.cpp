/*
 * Copyright (c) 2011, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/main/LICENSE
 *
 * This file is provided under the following "BSD-style" License
 */

#include <dart/collision/experimental/narrow_phase/distance.hpp>
#include <dart/collision/experimental/sdf/dense_esdf_field.hpp>
#include <dart/collision/experimental/sdf/dense_sdf_field.hpp>
#include <dart/collision/experimental/sdf/dense_tsdf_field.hpp>
#include <dart/collision/experimental/shapes/shape.hpp>

#include <benchmark/benchmark.h>

#ifdef DART_EXPERIMENTAL_HAVE_VOXBLOX
  #include <voxblox/core/common.h>
  #include <voxblox/core/esdf_map.h>
  #include <voxblox/core/layer.h>
#endif

#include <algorithm>
#include <memory>
#include <random>
#include <vector>

#include <cstdint>

namespace {

constexpr double kVoxelSize = 0.1;
constexpr int kDim = 32;
constexpr double kRadius = 0.8;
constexpr double kTruncation = 0.3;
constexpr double kEsdfMaxDistance = 2.0;

Eigen::Vector3d gridOrigin()
{
  return Eigen::Vector3d::Zero();
}

Eigen::Vector3d gridCenter()
{
  const double extent = kDim * kVoxelSize;
  return Eigen::Vector3d(extent * 0.5, extent * 0.5, extent * 0.5);
}

double sphereDistance(const Eigen::Vector3d& point)
{
  return (point - gridCenter()).norm() - kRadius;
}

std::shared_ptr<dart::collision::experimental::DenseSdfField> buildDenseField()
{
  using dart::collision::experimental::DenseSdfField;
  const Eigen::Vector3i dims(kDim, kDim, kDim);
  auto field = std::make_shared<DenseSdfField>(gridOrigin(), dims, kVoxelSize);

  for (int z = 0; z < kDim; ++z) {
    for (int y = 0; y < kDim; ++y) {
      for (int x = 0; x < kDim; ++x) {
        const Eigen::Vector3d pos
            = gridOrigin()
              + (Eigen::Vector3d(x + 0.5, y + 0.5, z + 0.5) * kVoxelSize);
        field->setDistance(Eigen::Vector3i(x, y, z), sphereDistance(pos));
        field->setObserved(Eigen::Vector3i(x, y, z), true);
      }
    }
  }

  return field;
}

std::shared_ptr<dart::collision::experimental::DenseTsdfField> buildDenseTsdf()
{
  using dart::collision::experimental::DenseTsdfField;
  const Eigen::Vector3i dims(kDim, kDim, kDim);
  auto field = std::make_shared<DenseTsdfField>(
      gridOrigin(), dims, kVoxelSize, kTruncation, kEsdfMaxDistance);

  for (int z = 0; z < kDim; ++z) {
    for (int y = 0; y < kDim; ++y) {
      for (int x = 0; x < kDim; ++x) {
        const Eigen::Vector3d pos
            = gridOrigin()
              + (Eigen::Vector3d(x + 0.5, y + 0.5, z + 0.5) * kVoxelSize);
        const double dist = sphereDistance(pos);
        const double clamped = std::clamp(dist, -kTruncation, kTruncation);
        field->setDistance(Eigen::Vector3i(x, y, z), clamped);
        field->setWeight(Eigen::Vector3i(x, y, z), 1.0);
      }
    }
  }

  return field;
}

std::shared_ptr<dart::collision::experimental::DenseEsdfField> buildDenseEsdf()
{
  using dart::collision::experimental::DenseEsdfField;
  using dart::collision::experimental::EsdfBuildOptions;

  auto tsdf = buildDenseTsdf();
  auto field = std::make_shared<DenseEsdfField>(
      tsdf->origin(), tsdf->dims(), kVoxelSize, kEsdfMaxDistance);

  EsdfBuildOptions options;
  options.surfaceDistance = 0.5 * kVoxelSize;
  options.maxDistance = kEsdfMaxDistance;
  options.minWeight = 1e-6;
  options.useDiagonalNeighbors = true;
  if (!field->buildFromTsdf(*tsdf, options)) {
    return nullptr;
  }
  return field;
}

std::vector<Eigen::Vector3d> buildQueryPoints(std::size_t count)
{
  std::mt19937 rng(42);
  const double extent = kDim * kVoxelSize;
  std::uniform_real_distribution<double> dist(0.2, extent - 0.2);

  std::vector<Eigen::Vector3d> points;
  points.reserve(count);
  for (std::size_t i = 0; i < count; ++i) {
    points.emplace_back(dist(rng), dist(rng), dist(rng));
  }
  return points;
}

#ifdef DART_EXPERIMENTAL_HAVE_VOXBLOX
std::shared_ptr<voxblox::EsdfMap> buildVoxbloxMap()
{
  constexpr int kVoxelsPerSide = 16;
  auto layer = std::make_shared<voxblox::Layer<voxblox::EsdfVoxel>>(
      kVoxelSize, kVoxelsPerSide);

  for (int z = 0; z < kDim; ++z) {
    for (int y = 0; y < kDim; ++y) {
      for (int x = 0; x < kDim; ++x) {
        const Eigen::Vector3d pos
            = gridOrigin()
              + (Eigen::Vector3d(x + 0.5, y + 0.5, z + 0.5) * kVoxelSize);
        const double dist = sphereDistance(pos);

        const voxblox::GlobalIndex global(x, y, z);
        voxblox::BlockIndex block;
        voxblox::VoxelIndex voxel;
        voxblox::getBlockAndVoxelIndexFromGlobalVoxelIndex(
            global, kVoxelsPerSide, &block, &voxel);

        auto block_ptr = layer->allocateBlockPtrByIndex(block);
        voxblox::EsdfVoxel& voxel_ref = block_ptr->getVoxelByVoxelIndex(voxel);
        voxel_ref.distance = static_cast<float>(dist);
        voxel_ref.observed = true;
      }
    }
  }

  return std::make_shared<voxblox::EsdfMap>(layer);
}
#endif

} // namespace

static void BM_DenseSdfDistance(benchmark::State& state)
{
  static auto field = buildDenseField();
  auto points = buildQueryPoints(static_cast<std::size_t>(state.range(0)));

  dart::collision::experimental::SdfQueryOptions options;
  options.interpolate = true;
  options.requireObserved = true;

  for (auto _ : state) {
    double sum = 0.0;
    for (const auto& point : points) {
      double dist = 0.0;
      field->distance(point, &dist, options);
      sum += dist;
    }
    benchmark::DoNotOptimize(sum);
  }

  state.SetItemsProcessed(
      state.iterations() * static_cast<std::uint64_t>(points.size()));
}

BENCHMARK(BM_DenseSdfDistance)->Arg(1024)->Arg(4096)->Arg(16384);

static void BM_DenseSphereVsSdfDistance(benchmark::State& state)
{
  using namespace dart::collision::experimental;

  static auto field = buildDenseField();
  static auto base_ptr
      = std::static_pointer_cast<const SignedDistanceField>(field);
  static SdfShape sdf(base_ptr);
  static SphereShape sphere(0.2);
  static Eigen::Isometry3d sdf_tf = Eigen::Isometry3d::Identity();

  auto points = buildQueryPoints(static_cast<std::size_t>(state.range(0)));

  DistanceOption option;
  for (auto _ : state) {
    double sum = 0.0;
    for (const auto& point : points) {
      Eigen::Isometry3d sphere_tf = Eigen::Isometry3d::Identity();
      sphere_tf.translation() = point;
      DistanceResult result;
      sum += distanceSphereSdf(sphere, sphere_tf, sdf, sdf_tf, result, option);
    }
    benchmark::DoNotOptimize(sum);
  }

  state.SetItemsProcessed(
      state.iterations() * static_cast<std::uint64_t>(points.size()));
}

BENCHMARK(BM_DenseSphereVsSdfDistance)->Arg(1024)->Arg(4096)->Arg(16384);

static void BM_DenseEsdfDistance(benchmark::State& state)
{
  static auto field = buildDenseEsdf();
  if (!field) {
    state.SkipWithError("Dense ESDF build failed.");
    return;
  }
  auto points = buildQueryPoints(static_cast<std::size_t>(state.range(0)));

  dart::collision::experimental::SdfQueryOptions options;
  options.interpolate = true;
  options.requireObserved = true;

  for (auto _ : state) {
    double sum = 0.0;
    for (const auto& point : points) {
      double dist = 0.0;
      field->distance(point, &dist, options);
      sum += dist;
    }
    benchmark::DoNotOptimize(sum);
  }

  state.SetItemsProcessed(
      state.iterations() * static_cast<std::uint64_t>(points.size()));
}

BENCHMARK(BM_DenseEsdfDistance)->Arg(1024)->Arg(4096)->Arg(16384);

static void BM_DenseEsdfBuild(benchmark::State& state)
{
  using dart::collision::experimental::DenseEsdfField;
  using dart::collision::experimental::EsdfBuildOptions;

  static auto tsdf = buildDenseTsdf();
  DenseEsdfField esdf(
      tsdf->origin(), tsdf->dims(), kVoxelSize, kEsdfMaxDistance);

  EsdfBuildOptions options;
  options.surfaceDistance = 0.5 * kVoxelSize;
  options.maxDistance = kEsdfMaxDistance;
  options.minWeight = 1e-6;
  options.useDiagonalNeighbors = true;

  for (auto _ : state) {
    bool ok = esdf.buildFromTsdf(*tsdf, options);
    benchmark::DoNotOptimize(ok);
  }

  state.SetItemsProcessed(
      state.iterations() * static_cast<std::uint64_t>(kDim) * kDim * kDim);
}

BENCHMARK(BM_DenseEsdfBuild);

#ifdef DART_EXPERIMENTAL_HAVE_VOXBLOX
static void BM_VoxbloxDistance(benchmark::State& state)
{
  static auto map = buildVoxbloxMap();
  auto points = buildQueryPoints(static_cast<std::size_t>(state.range(0)));

  for (auto _ : state) {
    double sum = 0.0;
    for (const auto& point : points) {
      double dist = 0.0;
      map->getDistanceAtPosition(point, true, &dist);
      sum += dist;
    }
    benchmark::DoNotOptimize(sum);
  }

  state.SetItemsProcessed(
      state.iterations() * static_cast<std::uint64_t>(points.size()));
}

BENCHMARK(BM_VoxbloxDistance)->Arg(1024)->Arg(4096)->Arg(16384);
#endif
