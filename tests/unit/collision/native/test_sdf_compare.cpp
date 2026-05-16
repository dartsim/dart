/*
 * Copyright (c) 2011, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/main/LICENSE
 *
 * This file is provided under the following "BSD-style" License
 */

#include <dart/collision/native/collision_world.hpp>
#include <dart/collision/native/narrow_phase/distance.hpp>
#include <dart/collision/native/narrow_phase/narrow_phase.hpp>
#include <dart/collision/native/sdf/dense_esdf_field.hpp>
#include <dart/collision/native/sdf/dense_sdf_field.hpp>
#include <dart/collision/native/sdf/dense_tsdf_field.hpp>
#include <dart/collision/native/shapes/shape.hpp>

#include <gtest/gtest.h>

#ifdef DART_NATIVE_HAVE_VOXBLOX
  #include <voxblox/core/common.h>
  #include <voxblox/core/esdf_map.h>
  #include <voxblox/core/layer.h>
#endif

#include <algorithm>
#include <memory>
#include <random>
#include <vector>

namespace {

constexpr double kVoxelSize = 0.1;
constexpr int kDim = 32;
constexpr double kRadius = 0.8;
constexpr double kDistTol = 2.0 * kVoxelSize;
constexpr double kEsdfTol = 2.5 * kVoxelSize;
constexpr double kTruncation = 0.3;

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

std::shared_ptr<dart::collision::native::DenseSdfField> buildDenseField()
{
  using dart::collision::native::DenseSdfField;
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

std::vector<Eigen::Vector3d> makeOctahedronVertices(double scale)
{
  return {
      {scale, 0.0, 0.0},
      {-scale, 0.0, 0.0},
      {0.0, scale, 0.0},
      {0.0, -scale, 0.0},
      {0.0, 0.0, scale},
      {0.0, 0.0, -scale}};
}

std::vector<dart::collision::native::MeshShape::Triangle>
makeOctahedronTriangles()
{
  return {
      {4, 0, 2},
      {4, 2, 1},
      {4, 1, 3},
      {4, 3, 0},
      {5, 2, 0},
      {5, 1, 2},
      {5, 3, 1},
      {5, 0, 3}};
}

std::shared_ptr<dart::collision::native::DenseTsdfField> buildDenseTsdf()
{
  using dart::collision::native::DenseTsdfField;
  const Eigen::Vector3i dims(kDim, kDim, kDim);
  auto field = std::make_shared<DenseTsdfField>(
      gridOrigin(), dims, kVoxelSize, kTruncation, 2.0);

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

#ifdef DART_NATIVE_HAVE_VOXBLOX
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

TEST(SdfDenseField, MatchesAnalyticDistance)
{
  auto field = buildDenseField();
  dart::collision::native::SdfQueryOptions options;
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
      const Eigen::Vector3d analytic_grad = (point - gridCenter()).normalized();
      const double dot = grad.normalized().dot(analytic_grad);
      EXPECT_GT(dot, 0.9);
    }
  }
}

TEST(SdfDistance, SphereVsSdf)
{
  using namespace dart::collision::native;

  auto field = buildDenseField();
  SdfShape sdf(field);
  SphereShape sphere(0.2);

  Eigen::Isometry3d sphere_tf = Eigen::Isometry3d::Identity();
  sphere_tf.translation() = gridCenter() + Eigen::Vector3d(0.7, 0.0, 0.0);

  Eigen::Isometry3d sdf_tf = Eigen::Isometry3d::Identity();

  DistanceResult result;
  DistanceOption option;
  const double dist
      = distanceSphereSdf(sphere, sphere_tf, sdf, sdf_tf, result, option);

  const double expected
      = sphereDistance(sphere_tf.translation()) - sphere.getRadius();
  EXPECT_NEAR(dist, expected, kDistTol);
}

TEST(SdfDistance, CylinderVsSdf)
{
  using namespace dart::collision::native;

  auto field = buildDenseField();
  SdfShape sdf(field);
  CylinderShape cylinder(0.2, 0.6);

  Eigen::Isometry3d cylinder_tf = Eigen::Isometry3d::Identity();
  cylinder_tf.translation() = gridCenter() + Eigen::Vector3d(1.2, 0.0, 0.0);

  Eigen::Isometry3d sdf_tf = Eigen::Isometry3d::Identity();

  DistanceResult result;
  DistanceOption option;
  double dist
      = distanceCylinderSdf(cylinder, cylinder_tf, sdf, sdf_tf, result, option);

  EXPECT_NEAR(dist, 0.2, kDistTol);
  EXPECT_TRUE(result.pointOnObject1.allFinite());
  EXPECT_TRUE(result.pointOnObject2.allFinite());
  EXPECT_TRUE(result.normal.allFinite());

  cylinder_tf.translation() = gridCenter() + Eigen::Vector3d(0.9, 0.0, 0.0);
  result.clear();
  dist
      = distanceCylinderSdf(cylinder, cylinder_tf, sdf, sdf_tf, result, option);

  EXPECT_LT(dist, 0.0);
  EXPECT_NEAR(dist, -0.1, kDistTol);
}

TEST(SdfDistance, CylinderSdfPairOrder)
{
  using namespace dart::collision::native;

  auto field = buildDenseField();
  CollisionWorld world;
  auto cylinder = world.createObject(std::make_unique<CylinderShape>(0.2, 0.6));
  auto sdf = world.createObject(std::make_unique<SdfShape>(field));

  Eigen::Isometry3d cylinder_tf = Eigen::Isometry3d::Identity();
  cylinder_tf.translation() = gridCenter() + Eigen::Vector3d(1.2, 0.0, 0.0);
  cylinder.setTransform(cylinder_tf);

  DistanceResult cylinder_sdf;
  DistanceResult sdf_cylinder;
  const DistanceOption option = DistanceOption::unlimited();

  const double dist1
      = NarrowPhase::distance(cylinder, sdf, option, cylinder_sdf);
  const double dist2
      = NarrowPhase::distance(sdf, cylinder, option, sdf_cylinder);

  EXPECT_NEAR(dist1, 0.2, kDistTol);
  EXPECT_NEAR(dist1, dist2, kDistTol);
  EXPECT_TRUE(cylinder_sdf.pointOnObject1.allFinite());
  EXPECT_TRUE(cylinder_sdf.pointOnObject2.allFinite());
  EXPECT_TRUE(sdf_cylinder.pointOnObject1.allFinite());
  EXPECT_TRUE(sdf_cylinder.pointOnObject2.allFinite());
  EXPECT_LT(cylinder_sdf.normal.dot(sdf_cylinder.normal), -0.9);
}

TEST(SdfDistance, ConvexVsSdf)
{
  using namespace dart::collision::native;

  auto field = buildDenseField();
  SdfShape sdf(field);
  ConvexShape convex(makeOctahedronVertices(0.2));

  Eigen::Isometry3d convex_tf = Eigen::Isometry3d::Identity();
  convex_tf.translation() = gridCenter() + Eigen::Vector3d(1.2, 0.0, 0.0);

  Eigen::Isometry3d sdf_tf = Eigen::Isometry3d::Identity();

  DistanceResult result;
  DistanceOption option;
  double dist
      = distanceConvexSdf(convex, convex_tf, sdf, sdf_tf, result, option);

  EXPECT_NEAR(dist, 0.2, kDistTol);
  EXPECT_TRUE(result.pointOnObject1.allFinite());
  EXPECT_TRUE(result.pointOnObject2.allFinite());
  EXPECT_TRUE(result.normal.allFinite());

  convex_tf.translation() = gridCenter() + Eigen::Vector3d(0.9, 0.0, 0.0);
  result.clear();
  dist = distanceConvexSdf(convex, convex_tf, sdf, sdf_tf, result, option);

  EXPECT_LT(dist, 0.0);
  EXPECT_NEAR(dist, -0.1, kDistTol);
}

TEST(SdfDistance, ConvexSdfPairOrder)
{
  using namespace dart::collision::native;

  auto field = buildDenseField();
  CollisionWorld world;
  auto convex = world.createObject(
      std::make_unique<ConvexShape>(makeOctahedronVertices(0.2)));
  auto sdf = world.createObject(std::make_unique<SdfShape>(field));

  Eigen::Isometry3d convex_tf = Eigen::Isometry3d::Identity();
  convex_tf.translation() = gridCenter() + Eigen::Vector3d(1.2, 0.0, 0.0);
  convex.setTransform(convex_tf);

  EXPECT_FALSE(NarrowPhase::isSupported(ShapeType::Convex, ShapeType::Sdf));
  EXPECT_FALSE(NarrowPhase::isSupported(ShapeType::Sdf, ShapeType::Convex));
  EXPECT_TRUE(
      NarrowPhase::isDistanceSupported(ShapeType::Convex, ShapeType::Sdf));
  EXPECT_TRUE(
      NarrowPhase::isDistanceSupported(ShapeType::Sdf, ShapeType::Convex));

  DistanceResult convex_sdf;
  DistanceResult sdf_convex;
  const DistanceOption option = DistanceOption::unlimited();

  const double dist1 = NarrowPhase::distance(convex, sdf, option, convex_sdf);
  const double dist2 = NarrowPhase::distance(sdf, convex, option, sdf_convex);

  EXPECT_NEAR(dist1, 0.2, kDistTol);
  EXPECT_NEAR(dist1, dist2, kDistTol);
  EXPECT_TRUE(convex_sdf.pointOnObject1.allFinite());
  EXPECT_TRUE(convex_sdf.pointOnObject2.allFinite());
  EXPECT_TRUE(sdf_convex.pointOnObject1.allFinite());
  EXPECT_TRUE(sdf_convex.pointOnObject2.allFinite());
  EXPECT_LT(convex_sdf.normal.dot(sdf_convex.normal), -0.9);
}

TEST(SdfDistance, MeshVsSdf)
{
  using namespace dart::collision::native;

  auto field = buildDenseField();
  SdfShape sdf(field);
  MeshShape mesh(makeOctahedronVertices(0.2), makeOctahedronTriangles());

  Eigen::Isometry3d mesh_tf = Eigen::Isometry3d::Identity();
  mesh_tf.translation() = gridCenter() + Eigen::Vector3d(1.2, 0.0, 0.0);

  Eigen::Isometry3d sdf_tf = Eigen::Isometry3d::Identity();

  DistanceResult result;
  DistanceOption option;
  double dist = distanceMeshSdf(mesh, mesh_tf, sdf, sdf_tf, result, option);

  EXPECT_NEAR(dist, 0.2, kDistTol);
  EXPECT_TRUE(result.pointOnObject1.allFinite());
  EXPECT_TRUE(result.pointOnObject2.allFinite());
  EXPECT_TRUE(result.normal.allFinite());

  mesh_tf.translation() = gridCenter() + Eigen::Vector3d(0.9, 0.0, 0.0);
  result.clear();
  dist = distanceMeshSdf(mesh, mesh_tf, sdf, sdf_tf, result, option);

  EXPECT_LT(dist, 0.0);
  EXPECT_NEAR(dist, -0.1, kDistTol);
}

TEST(SdfDistance, MeshSdfPairOrder)
{
  using namespace dart::collision::native;

  auto field = buildDenseField();
  CollisionWorld world;
  auto mesh = world.createObject(
      std::make_unique<MeshShape>(
          makeOctahedronVertices(0.2), makeOctahedronTriangles()));
  auto sdf = world.createObject(std::make_unique<SdfShape>(field));

  Eigen::Isometry3d mesh_tf = Eigen::Isometry3d::Identity();
  mesh_tf.translation() = gridCenter() + Eigen::Vector3d(1.2, 0.0, 0.0);
  mesh.setTransform(mesh_tf);

  EXPECT_FALSE(NarrowPhase::isSupported(ShapeType::Mesh, ShapeType::Sdf));
  EXPECT_FALSE(NarrowPhase::isSupported(ShapeType::Sdf, ShapeType::Mesh));
  EXPECT_TRUE(
      NarrowPhase::isDistanceSupported(ShapeType::Mesh, ShapeType::Sdf));
  EXPECT_TRUE(
      NarrowPhase::isDistanceSupported(ShapeType::Sdf, ShapeType::Mesh));

  DistanceResult mesh_sdf;
  DistanceResult sdf_mesh;
  const DistanceOption option = DistanceOption::unlimited();

  const double dist1 = NarrowPhase::distance(mesh, sdf, option, mesh_sdf);
  const double dist2 = NarrowPhase::distance(sdf, mesh, option, sdf_mesh);

  EXPECT_NEAR(dist1, 0.2, kDistTol);
  EXPECT_NEAR(dist1, dist2, kDistTol);
  EXPECT_TRUE(mesh_sdf.pointOnObject1.allFinite());
  EXPECT_TRUE(mesh_sdf.pointOnObject2.allFinite());
  EXPECT_TRUE(sdf_mesh.pointOnObject1.allFinite());
  EXPECT_TRUE(sdf_mesh.pointOnObject2.allFinite());
  EXPECT_LT(mesh_sdf.normal.dot(sdf_mesh.normal), -0.9);
}

TEST(SdfDistance, CompoundSdfPairOrder)
{
  using namespace dart::collision::native;

  auto field = buildDenseField();
  CollisionWorld world;

  auto compoundShape = std::make_unique<CompoundShape>();
  compoundShape->addChild(std::make_unique<SphereShape>(0.2));
  auto compound = world.createObject(std::move(compoundShape));
  auto sdf = world.createObject(std::make_unique<SdfShape>(field));

  Eigen::Isometry3d compound_tf = Eigen::Isometry3d::Identity();
  compound_tf.translation() = gridCenter() + Eigen::Vector3d(1.2, 0.0, 0.0);
  compound.setTransform(compound_tf);

  EXPECT_TRUE(
      NarrowPhase::isDistanceSupported(ShapeType::Compound, ShapeType::Sdf));
  EXPECT_TRUE(
      NarrowPhase::isDistanceSupported(ShapeType::Sdf, ShapeType::Compound));

  DistanceResult compound_sdf;
  DistanceResult sdf_compound;
  const DistanceOption option = DistanceOption::unlimited();

  const double dist1
      = NarrowPhase::distance(compound, sdf, option, compound_sdf);
  const double dist2
      = NarrowPhase::distance(sdf, compound, option, sdf_compound);

  EXPECT_NEAR(dist1, 0.2, kDistTol);
  EXPECT_NEAR(dist1, dist2, kDistTol);
  EXPECT_TRUE(compound_sdf.pointOnObject1.allFinite());
  EXPECT_TRUE(compound_sdf.pointOnObject2.allFinite());
  EXPECT_TRUE(sdf_compound.pointOnObject1.allFinite());
  EXPECT_TRUE(sdf_compound.pointOnObject2.allFinite());
  EXPECT_LT(compound_sdf.normal.dot(sdf_compound.normal), -0.9);
}

TEST(EsdfDenseField, BuildsFromTsdf)
{
  using dart::collision::native::DenseEsdfField;
  using dart::collision::native::EsdfBuildOptions;

  auto tsdf = buildDenseTsdf();
  DenseEsdfField esdf(tsdf->origin(), tsdf->dims(), kVoxelSize, 2.0);

  EsdfBuildOptions options;
  options.surfaceDistance = 0.5 * kVoxelSize;
  options.maxDistance = 2.0;
  options.minWeight = 1e-6;
  options.useDiagonalNeighbors = true;
  ASSERT_TRUE(esdf.buildFromTsdf(*tsdf, options));

  dart::collision::native::SdfQueryOptions query;
  query.interpolate = true;
  query.requireObserved = true;

  const auto points = makeQueryPoints(64);
  for (const auto& point : points) {
    double dist = 0.0;
    ASSERT_TRUE(esdf.distance(point, &dist, query));
    EXPECT_NEAR(dist, sphereDistance(point), kEsdfTol);
  }
}

#ifdef DART_NATIVE_HAVE_VOXBLOX
TEST(SdfDenseField, MatchesVoxbloxEsdf)
{
  auto dense = buildDenseField();
  auto map = buildVoxbloxMap();

  dart::collision::native::SdfQueryOptions options;
  options.interpolate = true;
  options.requireObserved = true;

  const auto points = makeQueryPoints(64);
  for (const auto& point : points) {
    double dense_dist = 0.0;
    double vox_dist = 0.0;
    ASSERT_TRUE(dense->distance(point, &dense_dist, options));
    ASSERT_TRUE(map->getDistanceAtPosition(point, true, &vox_dist));
    EXPECT_NEAR(dense_dist, vox_dist, 5e-3);
  }
}
#endif
