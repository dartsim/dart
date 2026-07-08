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

#include <dart/collision/native/narrow_phase/distance.hpp>
#include <dart/collision/native/shapes/shape.hpp>

#include <gtest/gtest.h>

#include <limits>
#include <memory>
#include <vector>

#include <cmath>
#include <cstdint>

using namespace dart::collision::native;

namespace {

Eigen::Isometry3d translated(double x, double y, double z)
{
  Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
  tf.translation() = Eigen::Vector3d(x, y, z);
  return tf;
}

MeshShape makePlaneMesh(double z)
{
  std::vector<Eigen::Vector3d> vertices
      = {{-1.0, -1.0, z}, {1.0, -1.0, z}, {1.0, 1.0, z}, {-1.0, 1.0, z}};
  std::vector<MeshShape::Triangle> triangles = {{0, 1, 2}, {0, 2, 3}};
  return MeshShape(vertices, triangles);
}

class PlaneField : public SignedDistanceField
{
public:
  bool distance(
      const Eigen::Vector3d& point_F,
      double* distance,
      const SdfQueryOptions& options) const override
  {
    return query(point_F, distance, nullptr, options);
  }

  bool distanceAndGradient(
      const Eigen::Vector3d& point_F,
      double* distance,
      Eigen::Vector3d* gradient,
      const SdfQueryOptions& options) const override
  {
    return query(point_F, distance, gradient, options);
  }

  void batchDistanceAndGradient(
      span<const Eigen::Vector3d> points_F,
      span<double> distances,
      span<Eigen::Vector3d> gradients,
      span<std::uint8_t> observed,
      const SdfQueryOptions& options) const override
  {
    for (std::size_t i = 0; i < points_F.size(); ++i) {
      double distance = 0.0;
      Eigen::Vector3d gradient = Eigen::Vector3d::Zero();
      const bool ok = query(points_F[i], &distance, &gradient, options);
      distances[i] = distance;
      gradients[i] = gradient;
      observed[i] = ok ? 1u : 0u;
    }
  }

  Aabb localAabb() const override
  {
    return Aabb(
        Eigen::Vector3d(-1.0, -1.0, -1.0), Eigen::Vector3d(1.0, 1.0, 1.0));
  }

  double voxelSize() const override
  {
    return 0.5;
  }

  double maxDistance() const override
  {
    return 10.0;
  }

private:
  bool query(
      const Eigen::Vector3d& point,
      double* distance,
      Eigen::Vector3d* gradient,
      const SdfQueryOptions& options) const
  {
    const double value = point.z();
    if (std::abs(value) > options.maxDistance) {
      return false;
    }

    if (distance) {
      *distance = value;
    }
    if (gradient) {
      *gradient = Eigen::Vector3d::UnitZ();
    }
    return true;
  }
};

class RejectingField final : public SignedDistanceField
{
public:
  bool distance(
      const Eigen::Vector3d&, double*, const SdfQueryOptions&) const override
  {
    return false;
  }

  bool distanceAndGradient(
      const Eigen::Vector3d&,
      double*,
      Eigen::Vector3d*,
      const SdfQueryOptions&) const override
  {
    return false;
  }

  void batchDistanceAndGradient(
      span<const Eigen::Vector3d> points_F,
      span<double> distances,
      span<Eigen::Vector3d> gradients,
      span<std::uint8_t> observed,
      const SdfQueryOptions&) const override
  {
    for (std::size_t i = 0; i < points_F.size(); ++i) {
      distances[i] = std::numeric_limits<double>::max();
      gradients[i] = Eigen::Vector3d::Zero();
      observed[i] = 0u;
    }
  }

  Aabb localAabb() const override
  {
    return Aabb(
        Eigen::Vector3d(-1.0, -1.0, -1.0), Eigen::Vector3d(1.0, 1.0, 1.0));
  }

  double voxelSize() const override
  {
    return 0.5;
  }

  double maxDistance() const override
  {
    return 10.0;
  }
};

class InvalidAabbField final : public PlaneField
{
public:
  Aabb localAabb() const override
  {
    return Aabb(Eigen::Vector3d::Ones(), -Eigen::Vector3d::Ones());
  }
};

} // namespace

TEST(NativeDistance, ComputesPrimitiveDistances)
{
  const Eigen::Isometry3d identity = Eigen::Isometry3d::Identity();

  SphereShape sphereA(0.5);
  SphereShape sphereB(0.25);
  DistanceResult sphereResult;
  EXPECT_NEAR(
      1.25,
      distanceSphereSphere(
          sphereA, identity, sphereB, translated(2.0, 0.0, 0.0), sphereResult),
      1e-12);
  EXPECT_TRUE(
      sphereResult.pointOnObject1.isApprox(Eigen::Vector3d(0.5, 0.0, 0.0)));
  EXPECT_TRUE(
      sphereResult.pointOnObject2.isApprox(Eigen::Vector3d(1.75, 0.0, 0.0)));
  EXPECT_TRUE(sphereResult.normal.isApprox(Eigen::Vector3d::UnitX()));

  DistanceResult coincidentSphereResult;
  EXPECT_NEAR(
      -0.75,
      distanceSphereSphere(
          sphereA, identity, sphereB, identity, coincidentSphereResult),
      1e-12);
  EXPECT_TRUE(coincidentSphereResult.normal.isApprox(Eigen::Vector3d::UnitX()));

  BoxShape boxA(Eigen::Vector3d(0.5, 0.5, 0.5));
  DistanceResult sphereBoxResult;
  EXPECT_NEAR(
      1.25,
      distanceSphereBox(
          sphereB, translated(-2.0, 0.0, 0.0), boxA, identity, sphereBoxResult),
      1e-12);

  BoxShape largeBox(Eigen::Vector3d(3.0, 2.0, 1.0));
  DistanceResult containedSphereBoxResult;
  EXPECT_NEAR(
      -1.25,
      distanceSphereBox(
          sphereB, identity, largeBox, identity, containedSphereBoxResult),
      1e-12);
  EXPECT_TRUE(
      containedSphereBoxResult.normal.isApprox(Eigen::Vector3d::UnitZ()));

  BoxShape boxB(Eigen::Vector3d(0.25, 0.25, 0.25));
  DistanceResult boxResult;
  EXPECT_NEAR(
      1.25,
      distanceBoxBox(
          boxA, identity, boxB, translated(2.0, 0.0, 0.0), boxResult),
      1e-12);

  CapsuleShape capsule(0.25, 1.0);
  DistanceResult capsuleSphereResult;
  EXPECT_NEAR(
      0.5,
      distanceCapsuleSphere(
          capsule,
          identity,
          sphereB,
          translated(1.0, 0.0, 0.0),
          capsuleSphereResult),
      1e-12);

  DistanceResult coincidentCapsuleSphereResult;
  EXPECT_NEAR(
      -0.5,
      distanceCapsuleSphere(
          capsule, identity, sphereB, identity, coincidentCapsuleSphereResult),
      1e-12);
  EXPECT_TRUE(
      coincidentCapsuleSphereResult.normal.isApprox(Eigen::Vector3d::UnitX()));

  DistanceResult capsuleCapsuleResult;
  EXPECT_NEAR(
      1.0,
      distanceCapsuleCapsule(
          capsule,
          identity,
          capsule,
          translated(1.5, 0.0, 0.0),
          capsuleCapsuleResult),
      1e-12);

  DistanceResult coincidentCapsuleCapsuleResult;
  EXPECT_NEAR(
      -0.5,
      distanceCapsuleCapsule(
          capsule, identity, capsule, identity, coincidentCapsuleCapsuleResult),
      1e-12);
  EXPECT_TRUE(
      coincidentCapsuleCapsuleResult.normal.isApprox(Eigen::Vector3d::UnitX()));

  DistanceResult capsuleBoxResult;
  EXPECT_NEAR(
      0.5,
      distanceCapsuleBox(
          capsule,
          translated(-1.25, 0.0, 0.0),
          boxA,
          identity,
          capsuleBoxResult),
      1e-12);
}

TEST(NativeDistance, HandlesRotatedBoxOverlap)
{
  BoxShape bar(Eigen::Vector3d(5.0, 0.25, 0.25));

  Eigen::Isometry3d rotated = Eigen::Isometry3d::Identity();
  rotated.linear()
      = Eigen::AngleAxisd(0.5 * std::acos(-1.0), Eigen::Vector3d::UnitZ())
            .toRotationMatrix();

  DistanceResult result;
  EXPECT_LE(
      distanceBoxBox(
          bar,
          Eigen::Isometry3d::Identity(),
          bar,
          rotated,
          result,
          DistanceOption::withUpperBound(0.0)),
      0.0);
  EXPECT_LE(result.distance, 0.0);
  EXPECT_TRUE(result.normal.allFinite());
}

TEST(NativeDistance, ComputesExactCapsuleBoxAxisDistance)
{
  CapsuleShape capsule(0.25, 20.0);
  BoxShape box(Eigen::Vector3d(1.0, 1.0, 1.0));

  Eigen::Isometry3d capsuleTransform = translated(2.0, 2.0, 0.0);
  capsuleTransform.linear()
      = Eigen::AngleAxisd(0.5 * std::acos(-1.0), Eigen::Vector3d::UnitY())
            .toRotationMatrix();

  DistanceResult result;
  EXPECT_NEAR(
      0.75,
      distanceCapsuleBox(
          capsule,
          capsuleTransform,
          box,
          Eigen::Isometry3d::Identity(),
          result,
          DistanceOption::withUpperBound(0.8)),
      1e-12);
  EXPECT_NEAR(1.75, result.pointOnObject1.y(), 1e-12);
  EXPECT_NEAR(1.0, result.pointOnObject2.y(), 1e-12);
}

TEST(NativeDistance, AccountsForCapsuleBoxContainmentDepth)
{
  CapsuleShape capsule(0.5, 2.0);
  BoxShape box(Eigen::Vector3d(10.0, 10.0, 10.0));

  DistanceResult result;
  EXPECT_NEAR(
      -10.5,
      distanceCapsuleBox(
          capsule,
          Eigen::Isometry3d::Identity(),
          box,
          Eigen::Isometry3d::Identity(),
          result,
          DistanceOption::withUpperBound(0.0)),
      1e-12);
  EXPECT_NEAR(-0.5, result.pointOnObject1.x(), 1e-12);
  EXPECT_NEAR(10.0, result.pointOnObject2.x(), 1e-12);
  EXPECT_TRUE(result.normal.isApprox(Eigen::Vector3d::UnitX(), 1e-12));
}

TEST(NativeDistance, ComputesPlaneShapeDistances)
{
  PlaneShape plane(Eigen::Vector3d::UnitZ(), 0.0);
  const Eigen::Isometry3d identity = Eigen::Isometry3d::Identity();

  SphereShape sphere(0.25);
  DistanceResult sphereResult;
  EXPECT_NEAR(
      0.75,
      distancePlaneShape(
          plane, identity, sphere, translated(0.0, 0.0, 1.0), sphereResult),
      1e-12);
  EXPECT_TRUE(
      sphereResult.pointOnObject1.isApprox(Eigen::Vector3d(0.0, 0.0, 0.0)));
  EXPECT_TRUE(
      sphereResult.pointOnObject2.isApprox(Eigen::Vector3d(0.0, 0.0, 0.75)));
  EXPECT_TRUE(sphereResult.normal.isApprox(Eigen::Vector3d::UnitZ()));

  BoxShape box(Eigen::Vector3d(0.5, 0.5, 0.25));
  DistanceResult boxResult;
  EXPECT_NEAR(
      0.75,
      distancePlaneShape(
          plane, identity, box, translated(0.0, 0.0, 1.0), boxResult),
      1e-12);

  CapsuleShape capsule(0.25, 1.0);
  DistanceResult capsuleResult;
  EXPECT_NEAR(
      0.25,
      distancePlaneShape(
          plane, identity, capsule, translated(0.0, 0.0, 1.0), capsuleResult),
      1e-12);

  CylinderShape cylinder(0.25, 1.0);
  DistanceResult cylinderResult;
  EXPECT_NEAR(
      0.5,
      distancePlaneShape(
          plane, identity, cylinder, translated(0.0, 0.0, 1.0), cylinderResult),
      1e-12);

  MeshShape mesh = makePlaneMesh(0.5);
  DistanceResult meshResult;
  EXPECT_NEAR(
      0.5,
      distancePlaneShape(plane, identity, mesh, identity, meshResult),
      1e-12);

  DistanceResult tangentSphereResult;
  EXPECT_NEAR(
      0.0,
      distancePlaneShape(
          plane,
          identity,
          sphere,
          translated(0.0, 0.0, 0.25),
          tangentSphereResult),
      1e-12);
  EXPECT_TRUE(tangentSphereResult.normal.isApprox(Eigen::Vector3d::UnitZ()));

  SdfShape unsupported(nullptr);
  DistanceResult unsupportedResult;
  EXPECT_EQ(
      std::numeric_limits<double>::max(),
      distancePlaneShape(
          plane, identity, unsupported, identity, unsupportedResult));
}

TEST(NativeDistance, HonorsUpperBoundAndNearestPointOptions)
{
  SphereShape sphereA(0.5);
  SphereShape sphereB(0.25);

  DistanceOption bounded = DistanceOption::withUpperBound(0.5);
  DistanceResult boundedResult;
  EXPECT_NEAR(
      1.25,
      distanceSphereSphere(
          sphereA,
          Eigen::Isometry3d::Identity(),
          sphereB,
          translated(2.0, 0.0, 0.0),
          boundedResult,
          bounded),
      1e-12);
  EXPECT_NEAR(1.25, boundedResult.distance, 1e-12);
  EXPECT_TRUE(boundedResult.pointOnObject1.isApprox(Eigen::Vector3d::Zero()));

  DistanceOption distanceOnly;
  distanceOnly.enableNearestPoints = false;
  DistanceResult distanceOnlyResult;
  EXPECT_NEAR(
      1.25,
      distanceSphereSphere(
          sphereA,
          Eigen::Isometry3d::Identity(),
          sphereB,
          translated(2.0, 0.0, 0.0),
          distanceOnlyResult,
          distanceOnly),
      1e-12);
  EXPECT_TRUE(
      distanceOnlyResult.pointOnObject1.isApprox(Eigen::Vector3d::Zero()));

  BoxShape box(Eigen::Vector3d(0.5, 0.5, 0.5));
  DistanceResult sphereBoxBoundedResult;
  EXPECT_NEAR(
      1.25,
      distanceSphereBox(
          sphereB,
          translated(-2.0, 0.0, 0.0),
          box,
          Eigen::Isometry3d::Identity(),
          sphereBoxBoundedResult,
          bounded),
      1e-12);

  CapsuleShape capsule(0.25, 1.0);
  DistanceResult capsuleSphereBoundedResult;
  EXPECT_NEAR(
      0.5,
      distanceCapsuleSphere(
          capsule,
          Eigen::Isometry3d::Identity(),
          sphereB,
          translated(1.0, 0.0, 0.0),
          capsuleSphereBoundedResult,
          DistanceOption::withUpperBound(0.1)),
      1e-12);

  DistanceResult capsuleBoxBoundedResult;
  EXPECT_NEAR(
      0.5,
      distanceCapsuleBox(
          capsule,
          translated(-1.25, 0.0, 0.0),
          box,
          Eigen::Isometry3d::Identity(),
          capsuleBoxBoundedResult,
          DistanceOption::withUpperBound(0.1)),
      1e-12);
}

TEST(NativeDistance, ComputesSdfDistances)
{
  auto field = std::make_shared<PlaneField>();
  SdfShape sdf(field);
  const Eigen::Isometry3d identity = Eigen::Isometry3d::Identity();

  SphereShape sphere(0.25);
  DistanceResult sphereResult;
  EXPECT_NEAR(
      0.75,
      distanceSphereSdf(
          sphere, translated(0.0, 0.0, 1.0), sdf, identity, sphereResult),
      1e-12);
  EXPECT_TRUE(
      sphereResult.pointOnObject1.isApprox(Eigen::Vector3d(0.0, 0.0, 0.75)));
  EXPECT_TRUE(sphereResult.pointOnObject2.isApprox(Eigen::Vector3d::Zero()));

  CapsuleShape capsule(0.25, 1.0);
  DistanceResult capsuleResult;
  EXPECT_NEAR(
      0.25,
      distanceCapsuleSdf(
          capsule, translated(0.0, 0.0, 1.0), sdf, identity, capsuleResult),
      1e-12);
  EXPECT_TRUE(
      capsuleResult.pointOnObject1.isApprox(Eigen::Vector3d(0.0, 0.0, 0.25)));
  EXPECT_TRUE(capsuleResult.pointOnObject2.isApprox(Eigen::Vector3d::Zero()));

  CylinderShape cylinder(0.25, 1.0);
  DistanceResult cylinderResult;
  EXPECT_NEAR(
      0.5,
      distanceCylinderSdf(
          cylinder, translated(0.0, 0.0, 1.0), sdf, identity, cylinderResult),
      1e-12);

  ConvexShape convex(
      {Eigen::Vector3d(-0.5, 0.0, 0.25),
       Eigen::Vector3d(0.5, 0.0, 0.25),
       Eigen::Vector3d(0.0, 0.5, 0.5),
       Eigen::Vector3d(0.0, 0.0, 1.0)});
  DistanceResult convexResult;
  EXPECT_NEAR(
      0.25,
      distanceConvexSdf(convex, identity, sdf, identity, convexResult),
      1e-12);

  MeshShape mesh = makePlaneMesh(0.4);
  DistanceResult meshResult;
  EXPECT_NEAR(
      0.4, distanceMeshSdf(mesh, identity, sdf, identity, meshResult), 1e-12);

  DistanceResult sdfResult;
  EXPECT_NEAR(
      -0.5,
      distanceSdfSdf(sdf, identity, sdf, translated(0.0, 0.0, 0.5), sdfResult),
      1e-12);
}

TEST(NativeDistance, PlacesRoundedSdfWitnessesOnFacingSide)
{
  auto field = std::make_shared<PlaneField>();
  SdfShape sdf(field);
  const Eigen::Isometry3d identity = Eigen::Isometry3d::Identity();

  SphereShape sphere(0.25);
  DistanceResult separatedSphere;
  EXPECT_NEAR(
      0.75,
      distanceSphereSdf(
          sphere, translated(0.0, 0.0, 1.0), sdf, identity, separatedSphere),
      1e-12);
  EXPECT_NEAR(0.75, separatedSphere.pointOnObject1.z(), 1e-12);
  EXPECT_NEAR(0.0, separatedSphere.pointOnObject2.z(), 1e-12);

  DistanceResult penetratingSphere;
  EXPECT_NEAR(
      -0.35,
      distanceSphereSdf(
          sphere, translated(0.0, 0.0, -0.1), sdf, identity, penetratingSphere),
      1e-12);
  EXPECT_NEAR(0.15, penetratingSphere.pointOnObject1.z(), 1e-12);
  EXPECT_NEAR(0.0, penetratingSphere.pointOnObject2.z(), 1e-12);

  CapsuleShape capsule(0.25, 1.0);
  DistanceResult separatedCapsule;
  EXPECT_NEAR(
      0.25,
      distanceCapsuleSdf(
          capsule, translated(0.0, 0.0, 1.0), sdf, identity, separatedCapsule),
      1e-12);
  EXPECT_NEAR(0.25, separatedCapsule.pointOnObject1.z(), 1e-12);
  EXPECT_NEAR(0.0, separatedCapsule.pointOnObject2.z(), 1e-12);

  DistanceResult penetratingCapsule;
  EXPECT_NEAR(
      -0.35,
      distanceCapsuleSdf(
          capsule,
          translated(0.0, 0.0, 0.4),
          sdf,
          identity,
          penetratingCapsule),
      1e-12);
  EXPECT_NEAR(0.15, penetratingCapsule.pointOnObject1.z(), 1e-12);
  EXPECT_NEAR(0.0, penetratingCapsule.pointOnObject2.z(), 1e-12);
}

TEST(NativeDistance, HandlesSdfMissesAndNullFields)
{
  const double maxDistance = std::numeric_limits<double>::max();
  const Eigen::Isometry3d identity = Eigen::Isometry3d::Identity();
  SdfShape nullSdf(nullptr);
  SdfShape rejectingSdf(std::make_shared<RejectingField>());
  SdfShape invalidAabbSdf(std::make_shared<InvalidAabbField>());
  SdfShape planeSdf(std::make_shared<PlaneField>());

  SphereShape sphere(0.25);
  CapsuleShape capsule(0.25, 1.0);
  CylinderShape cylinder(0.25, 1.0);
  ConvexShape convex(
      {Eigen::Vector3d(-0.5, 0.0, 0.25),
       Eigen::Vector3d(0.5, 0.0, 0.25),
       Eigen::Vector3d(0.0, 0.5, 0.5),
       Eigen::Vector3d(0.0, 0.0, 1.0)});
  MeshShape mesh = makePlaneMesh(0.4);

  DistanceResult result;
  EXPECT_EQ(
      maxDistance,
      distanceSphereSdf(sphere, identity, nullSdf, identity, result));

  result.clear();
  EXPECT_EQ(
      maxDistance,
      distanceCapsuleSdf(capsule, identity, nullSdf, identity, result));

  result.clear();
  EXPECT_EQ(
      maxDistance,
      distanceCylinderSdf(cylinder, identity, nullSdf, identity, result));

  result.clear();
  EXPECT_EQ(
      maxDistance,
      distanceCylinderSdf(cylinder, identity, rejectingSdf, identity, result));

  result.clear();
  EXPECT_EQ(
      maxDistance,
      distanceConvexSdf(convex, identity, nullSdf, identity, result));

  result.clear();
  EXPECT_EQ(
      maxDistance,
      distanceMeshSdf(mesh, identity, rejectingSdf, identity, result));

  result.clear();
  EXPECT_EQ(
      maxDistance,
      distanceSdfSdf(nullSdf, identity, planeSdf, identity, result));

  result.clear();
  EXPECT_EQ(
      maxDistance,
      distanceSdfSdf(
          invalidAabbSdf, identity, invalidAabbSdf, identity, result));
}
