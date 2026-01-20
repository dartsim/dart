/*
 * Copyright (c) 2011, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/main/LICENSE
 *
 * This file is provided under the following "BSD-style" License
 */

#include <gtest/gtest.h>

#include <dart/collision/experimental/narrow_phase/distance.hpp>
#include <dart/collision/experimental/sdf/dense_sdf_field.hpp>
#include <dart/collision/experimental/shapes/shape.hpp>

#if DART_EXPERIMENTAL_HAVE_ESDF_MAP
#include <dart/collision/experimental/sdf/esdf_map_field.hpp>
#include <voxblox/core/common.h>
#include <voxblox/core/esdf_map.h>
#include <voxblox/core/layer.h>
#endif

#include <memory>
#include <random>
#include <vector>

namespace {

constexpr double kVoxelSize = 0.1;
constexpr int kDim = 32;
constexpr double kRadius = 0.8;
constexpr double kDistTol = 2.0 * kVoxelSize;

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
        const Eigen::Vector3d pos =
            gridOrigin()
            + (Eigen::Vector3d(x + 0.5, y + 0.5, z + 0.5) * kVoxelSize);
        field->setDistance(Eigen::Vector3i(x, y, z), sphereDistance(pos));
        field->setObserved(Eigen::Vector3i(x, y, z), true);
      }
    }
  }

  return field;
}

std::vector<Eigen::Vector3d> makeQueryPoints(std::size_t count)
{
  std::mt19937 rng(7);
  const double extent = kDim * kVoxelSize;
  std::uniform_real_distribution<double> dist(0.2, extent - 0.2);

  std::vector<Eigen::Vector3d> points;
  points.reserve(count);
  for (std::size_t i = 0; i < count; ++i) {
    points.emplace_back(dist(rng), dist(rng), dist(rng));
  }
  return points;
}

#if DART_EXPERIMENTAL_HAVE_ESDF_MAP
std::shared_ptr<voxblox::EsdfMap> buildEsdfMap()
{
  constexpr int kVoxelsPerSide = 16;
  auto layer = std::make_shared<voxblox::Layer<voxblox::EsdfVoxel>>(
      kVoxelSize, kVoxelsPerSide);

  for (int z = 0; z < kDim; ++z) {
    for (int y = 0; y < kDim; ++y) {
      for (int x = 0; x < kDim; ++x) {
        const Eigen::Vector3d pos =
            gridOrigin()
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

}  // namespace

TEST(SdfDenseField, MatchesAnalyticDistance)
{
  auto field = buildDenseField();
  dart::collision::experimental::SdfQueryOptions options;
  options.interpolate = true;
  options.requireObserved = true;

  const auto points = makeQueryPoints(64);
  for (const auto& point : points) {
    double dist = 0.0;
    Eigen::Vector3d grad = Eigen::Vector3d::Zero();
    ASSERT_TRUE(field->distanceAndGradient(point, &dist, &grad, options));

    const double expected = sphereDistance(point);
    EXPECT_NEAR(dist, expected, kDistTol);

    if (grad.squaredNorm() > 1e-12) {
      const Eigen::Vector3d analytic_grad =
          (point - gridCenter()).normalized();
      const double dot = grad.normalized().dot(analytic_grad);
      EXPECT_GT(dot, 0.9);
    }
  }
}

TEST(SdfDistance, SphereVsSdf)
{
  using namespace dart::collision::experimental;

  auto field = buildDenseField();
  SdfShape sdf(field);
  SphereShape sphere(0.2);

  Eigen::Isometry3d sphere_tf = Eigen::Isometry3d::Identity();
  sphere_tf.translation() = gridCenter() + Eigen::Vector3d(0.7, 0.0, 0.0);

  Eigen::Isometry3d sdf_tf = Eigen::Isometry3d::Identity();

  DistanceResult result;
  DistanceOption option;
  const double dist =
      distanceSphereSdf(sphere, sphere_tf, sdf, sdf_tf, result, option);

  const double expected =
      sphereDistance(sphere_tf.translation()) - sphere.getRadius();
  EXPECT_NEAR(dist, expected, kDistTol);
}

#if DART_EXPERIMENTAL_HAVE_ESDF_MAP
TEST(SdfDenseField, MatchesEsdfMap)
{
  using dart::collision::experimental::SdfQueryOptions;
  using dart::collision::experimental::EsdfMapField;

  auto dense = buildDenseField();
  auto map = buildEsdfMap();
  EsdfMapField map_field(map);

  SdfQueryOptions options;
  options.interpolate = true;
  options.requireObserved = true;

  const auto points = makeQueryPoints(64);
  for (const auto& point : points) {
    double dense_dist = 0.0;
    double map_dist = 0.0;
    ASSERT_TRUE(dense->distance(point, &dense_dist, options));
    ASSERT_TRUE(map_field.distance(point, &map_dist, options));
    EXPECT_NEAR(dense_dist, map_dist, 5e-3);
  }
}
#endif
