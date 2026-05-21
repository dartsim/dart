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

#include <dart/collision/native/collision_object.hpp>
#include <dart/collision/native/collision_world.hpp>
#include <dart/collision/native/narrow_phase/convex_convex.hpp>
#include <dart/collision/native/narrow_phase/distance.hpp>
#include <dart/collision/native/narrow_phase/narrow_phase.hpp>
#include <dart/collision/native/shapes/shape.hpp>

#include <gtest/gtest.h>

#include <array>
#include <numbers>
#include <utility>
#include <vector>

#include <cmath>

using namespace dart::collision::native;

namespace {

Eigen::Isometry3d makeDistanceBoxTransform(const Eigen::Vector3d& translation)
{
  Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
  tf.translation() = translation;
  return tf;
}

Eigen::Isometry3d makeDistanceBoxTransform(
    const Eigen::Vector3d& translation,
    double angle,
    const Eigen::Vector3d& axis)
{
  Eigen::Isometry3d tf = makeDistanceBoxTransform(translation);
  tf.linear() = Eigen::AngleAxisd(angle, axis.normalized()).toRotationMatrix();
  return tf;
}

Eigen::Isometry3d makeDistanceTransform(const std::array<double, 16>& values)
{
  Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
  for (int r = 0; r < 4; ++r) {
    for (int c = 0; c < 4; ++c) {
      tf.matrix()(r, c) = values[static_cast<std::size_t>(4 * r + c)];
    }
  }
  return tf;
}

Eigen::Isometry3d makeDistanceSphereTransformInBoxFrame(
    const Eigen::Vector3d& translation,
    double angle,
    const Eigen::Vector3d& axis)
{
  Eigen::Isometry3d tf = makeDistanceBoxTransform(translation);
  tf.linear() = Eigen::AngleAxisd(angle, axis.normalized()).toRotationMatrix();
  return tf;
}

std::vector<Eigen::Isometry3d> makeSphereBoxDistanceBoxTransforms()
{
  return {
      Eigen::Isometry3d::Identity(),
      makeDistanceBoxTransform(Eigen::Vector3d(1.3, 2.7, 6.5)),
      makeDistanceBoxTransform(
          Eigen::Vector3d(1.3, 2.7, 6.5),
          std::numbers::pi_v<double> / 2.0,
          Eigen::Vector3d::UnitX()),
      makeDistanceBoxTransform(
          Eigen::Vector3d(1.3, 2.7, 6.5),
          std::numbers::pi_v<double> / 2.0,
          Eigen::Vector3d::UnitY()),
      makeDistanceBoxTransform(
          Eigen::Vector3d(1.3, 2.7, 6.5),
          std::numbers::pi_v<double> / 2.0,
          Eigen::Vector3d::UnitZ()),
      makeDistanceBoxTransform(
          Eigen::Vector3d(1.3, 2.7, 6.5),
          std::numbers::pi_v<double> / 3.0,
          Eigen::Vector3d(1.0, 2.0, 3.0)),
      makeDistanceBoxTransform(
          Eigen::Vector3d(1.3, 2.7, 6.5), 1e-12, Eigen::Vector3d::UnitX()),
  };
}

struct SphereBoxSphereOrientation
{
  double angle;
  Eigen::Vector3d axis;
};

std::array<SphereBoxSphereOrientation, 6>
makeSphereBoxDistanceSphereOrientations()
{
  return {{
      {0.0, Eigen::Vector3d::UnitX()},
      {std::numbers::pi_v<double> / 2.0, Eigen::Vector3d::UnitX()},
      {std::numbers::pi_v<double> / 2.0, Eigen::Vector3d::UnitY()},
      {std::numbers::pi_v<double> / 2.0, Eigen::Vector3d::UnitZ()},
      {std::numbers::pi_v<double> / 3.0, Eigen::Vector3d(1.0, 2.0, 3.0)},
      {1e-12, Eigen::Vector3d::UnitX()},
  }};
}

void expectVectorNear(
    const Eigen::Vector3d& actual,
    const Eigen::Vector3d& expected,
    double tolerance)
{
  EXPECT_NEAR(actual.x(), expected.x(), tolerance);
  EXPECT_NEAR(actual.y(), expected.y(), tolerance);
  EXPECT_NEAR(actual.z(), expected.z(), tolerance);
}

Eigen::Isometry3d makeDistanceTransformFromMatrix(
    double r00,
    double r01,
    double r02,
    double tx,
    double r10,
    double r11,
    double r12,
    double ty,
    double r20,
    double r21,
    double r22,
    double tz)
{
  Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
  tf.linear() << r00, r01, r02, r10, r11, r12, r20, r21, r22;
  tf.translation() = Eigen::Vector3d(tx, ty, tz);
  return tf;
}

void expectPointInsideCylinder(
    const Eigen::Vector3d& worldPoint,
    const Eigen::Isometry3d& cylinderTransform,
    double radius,
    double height,
    double tolerance)
{
  const Eigen::Vector3d local = cylinderTransform.inverse() * worldPoint;
  EXPECT_LE(std::abs(local.z()), 0.5 * height + tolerance)
      << "local=" << local.transpose();
  EXPECT_LE(local.head<2>().norm(), radius + tolerance)
      << "local=" << local.transpose();
}

void expectPointInsideBox(
    const Eigen::Vector3d& worldPoint,
    const Eigen::Isometry3d& boxTransform,
    const Eigen::Vector3d& halfExtents,
    double tolerance)
{
  const Eigen::Vector3d local = boxTransform.inverse() * worldPoint;
  for (int i = 0; i < 3; ++i) {
    EXPECT_LE(std::abs(local[i]), halfExtents[i] + tolerance)
        << "local=" << local.transpose();
  }
}

struct SphereBoxDistanceCase
{
  const char* name;
  Eigen::Vector3d halfSize;
  double sphereRadius;
  Eigen::Vector3d sphereCenterInBox;
  double expectedDistance;
  Eigen::Vector3d expectedPointOnSphereInBox;
  Eigen::Vector3d expectedPointOnBoxInBox;
};

void expectSphereBoxDistanceCase(
    const SphereBoxDistanceCase& testCase,
    const Eigen::Isometry3d& boxTransform,
    const SphereBoxSphereOrientation& sphereOrientation)
{
  SCOPED_TRACE(testCase.name);
  const SphereShape sphere(testCase.sphereRadius);
  const BoxShape box(testCase.halfSize);
  const Eigen::Isometry3d sphereTransform
      = boxTransform
        * makeDistanceSphereTransformInBoxFrame(
            testCase.sphereCenterInBox,
            sphereOrientation.angle,
            sphereOrientation.axis);

  DistanceResult result;
  const double distance
      = distanceSphereBox(sphere, sphereTransform, box, boxTransform, result);

  EXPECT_NEAR(distance, testCase.expectedDistance, 1e-9);
  EXPECT_NEAR(result.distance, testCase.expectedDistance, 1e-9);
  expectVectorNear(
      result.pointOnObject1,
      boxTransform * testCase.expectedPointOnSphereInBox,
      1e-9);
  expectVectorNear(
      result.pointOnObject2,
      boxTransform * testCase.expectedPointOnBoxInBox,
      1e-9);
}

void expectSphereBoxSignedDistanceCase(
    const SphereBoxDistanceCase& testCase,
    const Eigen::Isometry3d& boxTransform,
    const SphereBoxSphereOrientation& sphereOrientation)
{
  SCOPED_TRACE(testCase.name);
  const SphereShape sphere(testCase.sphereRadius);
  const BoxShape box(testCase.halfSize);
  const Eigen::Isometry3d sphereTransform
      = boxTransform
        * makeDistanceSphereTransformInBoxFrame(
            testCase.sphereCenterInBox,
            sphereOrientation.angle,
            sphereOrientation.axis);

  DistanceResult result;
  const double distance
      = distanceSphereBox(sphere, sphereTransform, box, boxTransform, result);

  EXPECT_NEAR(distance, testCase.expectedDistance, 1e-9);
  EXPECT_NEAR(result.distance, testCase.expectedDistance, 1e-9);
}

void expectSphereSphereSignedDistanceCase(
    const char* name,
    double radius1,
    const Eigen::Vector3d& center1,
    double radius2,
    const Eigen::Vector3d& center2)
{
  SCOPED_TRACE(name);
  const SphereShape sphere1(radius1);
  const SphereShape sphere2(radius2);
  Eigen::Isometry3d transform1 = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d transform2 = Eigen::Isometry3d::Identity();
  transform1.translation() = center1;
  transform2.translation() = center2;

  DistanceResult result;
  const double distance
      = distanceSphereSphere(sphere1, transform1, sphere2, transform2, result);

  const double expectedDistance
      = (center2 - center1).norm() - radius1 - radius2;
  EXPECT_NEAR(distance, expectedDistance, 1e-9);
  EXPECT_NEAR(result.distance, expectedDistance, 1e-9);
  EXPECT_NEAR((result.pointOnObject1 - center1).norm(), radius1, 1e-9);
  EXPECT_NEAR((result.pointOnObject2 - center2).norm(), radius2, 1e-9);
  EXPECT_NEAR(
      (result.pointOnObject1 - result.pointOnObject2).norm(),
      std::abs(expectedDistance),
      1e-9);
  EXPECT_NEAR(
      (center1 + result.pointOnObject2 - result.pointOnObject1 - center2)
          .norm(),
      radius1 + radius2,
      1e-9);

  if (expectedDistance > 0.0) {
    EXPECT_GT(result.distance, 0.0);
  } else {
    EXPECT_LT(result.distance, 0.0);
  }
}

void expectSpherePointDepth(
    double sphereRadius, const Eigen::Vector3d& point, double expectedDepth)
{
  SCOPED_TRACE(point.transpose());
  const SphereShape sphere(sphereRadius);
  const SphereShape pointShape(0.0);

  Eigen::Isometry3d sphereTransform = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d pointTransform = Eigen::Isometry3d::Identity();
  pointTransform.translation() = point;

  DistanceResult result;
  const double distance = distanceSphereSphere(
      sphere, sphereTransform, pointShape, pointTransform, result);

  EXPECT_NEAR(distance, -expectedDepth, 1e-12);
  EXPECT_NEAR(result.distance, -expectedDepth, 1e-12);
  EXPECT_TRUE(result.pointOnObject1.allFinite());
  EXPECT_TRUE(result.pointOnObject2.allFinite());
  EXPECT_TRUE(result.normal.allFinite());
}

void expectBoxPointDepth(
    const Eigen::Vector3d& halfExtents,
    const Eigen::Vector3d& point,
    double expectedDepth)
{
  SCOPED_TRACE(point.transpose());
  const BoxShape box(halfExtents);
  const SphereShape pointShape(0.0);

  Eigen::Isometry3d boxTransform = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d pointTransform = Eigen::Isometry3d::Identity();
  pointTransform.translation() = point;

  DistanceResult result;
  const double distance = distanceSphereBox(
      pointShape, pointTransform, box, boxTransform, result);

  EXPECT_NEAR(distance, -expectedDepth, 1e-12);
  EXPECT_NEAR(result.distance, -expectedDepth, 1e-12);
  EXPECT_TRUE(result.pointOnObject1.allFinite());
  EXPECT_TRUE(result.pointOnObject2.allFinite());
  EXPECT_TRUE(result.normal.allFinite());
}

void expectBoxBoxSignedDistanceCase(
    const char* name,
    const Eigen::Vector3d& halfExtents1,
    const Eigen::Isometry3d& transform1,
    const Eigen::Vector3d& halfExtents2,
    const Eigen::Isometry3d& transform2,
    double expectedDistance)
{
  SCOPED_TRACE(name);
  const BoxShape box1(halfExtents1);
  const BoxShape box2(halfExtents2);

  DistanceResult result;
  const double distance
      = distanceBoxBox(box1, transform1, box2, transform2, result);

  EXPECT_NEAR(distance, expectedDistance, 1e-9);
  EXPECT_NEAR(result.distance, expectedDistance, 1e-9);
  EXPECT_NEAR(
      (result.pointOnObject2 - result.pointOnObject1).norm(),
      std::abs(expectedDistance),
      1e-9);
  EXPECT_TRUE(result.pointOnObject1.allFinite());
  EXPECT_TRUE(result.pointOnObject2.allFinite());
  EXPECT_TRUE(result.normal.allFinite());

  if (expectedDistance > 0.0) {
    EXPECT_GT(result.distance, 0.0);
  } else {
    EXPECT_LT(result.distance, 0.0);
  }
}

void expectBoxBoxRegressionWitnesses(
    const char* name,
    const Eigen::Vector3d& boxSize1,
    const Eigen::Isometry3d& transform1,
    const Eigen::Vector3d& boxSize2,
    const Eigen::Isometry3d& transform2,
    const double* expectedDistance = nullptr)
{
  SCOPED_TRACE(name);
  const BoxShape box1(0.5 * boxSize1);
  const BoxShape box2(0.5 * boxSize2);

  DistanceResult result;
  const double distance
      = distanceBoxBox(box1, transform1, box2, transform2, result);

  EXPECT_TRUE(std::isfinite(distance));
  EXPECT_TRUE(result.pointOnObject1.allFinite());
  EXPECT_TRUE(result.pointOnObject2.allFinite());
  EXPECT_TRUE(result.normal.allFinite());
  EXPECT_NEAR(result.distance, distance, 1e-12);
  EXPECT_NEAR(
      (result.pointOnObject2 - result.pointOnObject1).norm(),
      std::abs(distance),
      1e-6);

  const Eigen::Vector3d p1 = transform1.inverse() * result.pointOnObject1;
  const Eigen::Vector3d p2 = transform2.inverse() * result.pointOnObject2;
  EXPECT_TRUE((p1.array().abs() <= (0.5 * boxSize1).array() + 1e-9).all());
  EXPECT_TRUE((p2.array().abs() <= (0.5 * boxSize2).array() + 1e-9).all());

  if (expectedDistance != nullptr) {
    EXPECT_NEAR(distance, *expectedDistance, 1e-9);
  }
}

void expectConvexConvexRegressionWitnesses(
    const char* name,
    std::vector<Eigen::Vector3d> vertices1,
    const Eigen::Isometry3d& transform1,
    std::vector<Eigen::Vector3d> vertices2,
    const Eigen::Isometry3d& transform2)
{
  SCOPED_TRACE(name);
  const ConvexShape convex1(std::move(vertices1));
  const ConvexShape convex2(std::move(vertices2));

  DistanceOption option;
  DistanceResult result;
  const double distance = distanceConvexConvex(
      convex1, transform1, convex2, transform2, result, option);

  EXPECT_TRUE(std::isfinite(distance));
  EXPECT_NEAR(result.distance, distance, 1e-12);
  EXPECT_TRUE(result.pointOnObject1.allFinite());
  EXPECT_TRUE(result.pointOnObject2.allFinite());
  EXPECT_TRUE(result.normal.allFinite());
  EXPECT_NEAR(
      (result.pointOnObject2 - result.pointOnObject1).norm(),
      std::abs(distance),
      1e-6);
}

Eigen::Isometry3d makeSignedDistanceFrame()
{
  Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
  tf.translation() = Eigen::Vector3d(0.1, 0.2, 0.3);
  tf.linear() = Eigen::AngleAxisd(
                    std::numbers::pi_v<double> / 5.0,
                    Eigen::Vector3d(1.0, 2.0, 3.0).normalized())
                    .toRotationMatrix();
  return tf;
}

Eigen::Isometry3d makeCapsuleTransformFromCenterLine(
    const Eigen::Vector3d& bottom, const Eigen::Vector3d& top)
{
  Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
  tf.translation() = 0.5 * (bottom + top);

  const Eigen::Vector3d axis = top - bottom;
  if (axis.squaredNorm() > 1e-24) {
    tf.linear() = Eigen::Quaterniond::FromTwoVectors(
                      Eigen::Vector3d::UnitZ(), axis.normalized())
                      .toRotationMatrix();
  }

  return tf;
}

Eigen::Vector3d projectPointOntoSegment(
    const Eigen::Vector3d& start,
    const Eigen::Vector3d& end,
    const Eigen::Vector3d& point)
{
  const Eigen::Vector3d direction = end - start;
  const double fraction
      = direction.dot(point - start) / direction.squaredNorm();
  return start + fraction * direction;
}

void expectCapsuleCapsuleCenterLineCase(
    const char* name,
    const Eigen::Vector3d& bottom1,
    const Eigen::Vector3d& top1,
    double radius1,
    const Eigen::Vector3d& bottom2,
    const Eigen::Vector3d& top2,
    double radius2,
    double expectedDistance,
    const Eigen::Vector3d& expectedCenterLinePoint1,
    const Eigen::Vector3d& expectedCenterLinePoint2,
    bool checkUniqueWitnesses = true)
{
  SCOPED_TRACE(name);
  const CapsuleShape capsule1(radius1, (top1 - bottom1).norm());
  const CapsuleShape capsule2(radius2, (top2 - bottom2).norm());
  const Eigen::Isometry3d transform1
      = makeCapsuleTransformFromCenterLine(bottom1, top1);
  const Eigen::Isometry3d transform2
      = makeCapsuleTransformFromCenterLine(bottom2, top2);

  DistanceResult result;
  const double distance = distanceCapsuleCapsule(
      capsule1, transform1, capsule2, transform2, result);

  EXPECT_NEAR(distance, expectedDistance, 1e-9);
  EXPECT_NEAR(result.distance, expectedDistance, 1e-9);
  EXPECT_TRUE(std::isfinite(result.pointOnObject1.x()));
  EXPECT_TRUE(std::isfinite(result.pointOnObject1.y()));
  EXPECT_TRUE(std::isfinite(result.pointOnObject1.z()));
  EXPECT_TRUE(std::isfinite(result.pointOnObject2.x()));
  EXPECT_TRUE(std::isfinite(result.pointOnObject2.y()));
  EXPECT_TRUE(std::isfinite(result.pointOnObject2.z()));

  if (checkUniqueWitnesses) {
    const Eigen::Vector3d centerLinePoint1
        = result.pointOnObject1 - result.normal * radius1;
    const Eigen::Vector3d centerLinePoint2
        = result.pointOnObject2 + result.normal * radius2;
    expectVectorNear(centerLinePoint1, expectedCenterLinePoint1, 1e-9);
    expectVectorNear(centerLinePoint2, expectedCenterLinePoint2, 1e-9);
  }
}

Eigen::Isometry3d makeCapsuleDistanceWorldFromCaseFrame()
{
  Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
  tf.translation() = Eigen::Vector3d(10.5, 12.75, -2.5);
  tf.linear() = Eigen::AngleAxisd(
                    std::numbers::pi_v<double> / 7.0,
                    Eigen::Vector3d(1.0, 2.0, 3.0).normalized())
                    .toRotationMatrix();
  return tf;
}

void expectCapsuleCapsulePoseDistance(
    const char* name,
    const CapsuleShape& capsule1,
    const Eigen::Isometry3d& transform1,
    const CapsuleShape& capsule2,
    const Eigen::Isometry3d& transform2,
    double expectedDistance,
    bool checkUniqueWitnesses,
    const Eigen::Vector3d& expectedPointOnCapsule1 = Eigen::Vector3d::Zero(),
    const Eigen::Vector3d& expectedPointOnCapsule2 = Eigen::Vector3d::Zero())
{
  SCOPED_TRACE(name);
  DistanceResult result;
  const double distance = distanceCapsuleCapsule(
      capsule1, transform1, capsule2, transform2, result);

  EXPECT_NEAR(distance, expectedDistance, 1e-9);
  EXPECT_NEAR(result.distance, expectedDistance, 1e-9);
  EXPECT_TRUE(std::isfinite(result.pointOnObject1.x()));
  EXPECT_TRUE(std::isfinite(result.pointOnObject1.y()));
  EXPECT_TRUE(std::isfinite(result.pointOnObject1.z()));
  EXPECT_TRUE(std::isfinite(result.pointOnObject2.x()));
  EXPECT_TRUE(std::isfinite(result.pointOnObject2.y()));
  EXPECT_TRUE(std::isfinite(result.pointOnObject2.z()));

  if (checkUniqueWitnesses) {
    expectVectorNear(result.pointOnObject1, expectedPointOnCapsule1, 1e-9);
    expectVectorNear(result.pointOnObject2, expectedPointOnCapsule2, 1e-9);
  }
}

} // namespace

TEST(DistanceSphereSphere, Separated)
{
  SphereShape s1(1.0);
  SphereShape s2(1.0);

  Eigen::Isometry3d tf1 = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d tf2 = Eigen::Isometry3d::Identity();
  tf2.translation() = Eigen::Vector3d(5.0, 0, 0);

  DistanceResult result;
  double dist = distanceSphereSphere(s1, tf1, s2, tf2, result);

  EXPECT_NEAR(dist, 3.0, 1e-6);
  EXPECT_NEAR(result.pointOnObject1.x(), 1.0, 1e-6);
  EXPECT_NEAR(result.pointOnObject2.x(), 4.0, 1e-6);
}

TEST(DistanceSphereSphere, Touching)
{
  SphereShape s1(1.0);
  SphereShape s2(1.0);

  Eigen::Isometry3d tf1 = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d tf2 = Eigen::Isometry3d::Identity();
  tf2.translation() = Eigen::Vector3d(2.0, 0, 0);

  DistanceResult result;
  double dist = distanceSphereSphere(s1, tf1, s2, tf2, result);

  EXPECT_NEAR(dist, 0.0, 1e-6);
}

TEST(DistanceSphereSphere, Penetrating)
{
  SphereShape s1(1.0);
  SphereShape s2(1.0);

  Eigen::Isometry3d tf1 = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d tf2 = Eigen::Isometry3d::Identity();
  tf2.translation() = Eigen::Vector3d(1.0, 0, 0);

  DistanceResult result;
  double dist = distanceSphereSphere(s1, tf1, s2, tf2, result);

  EXPECT_NEAR(dist, -1.0, 1e-6);
}

TEST(DistanceSphereSphere, ZeroRadiusCoincident)
{
  SphereShape s1(0.0);
  SphereShape s2(0.0);

  Eigen::Isometry3d tf1 = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d tf2 = Eigen::Isometry3d::Identity();

  DistanceResult result;
  double dist = distanceSphereSphere(s1, tf1, s2, tf2, result);

  EXPECT_NEAR(dist, 0.0, 1e-12);
  EXPECT_NEAR(result.distance, 0.0, 1e-12);
  EXPECT_EQ(result.pointOnObject1, Eigen::Vector3d::Zero());
  EXPECT_EQ(result.pointOnObject2, Eigen::Vector3d::Zero());
  EXPECT_EQ(result.normal, Eigen::Vector3d::UnitX());
}

TEST(DistanceSphereSphere, ScaleRegimes)
{
  const double scales[] = {1e-3, 1.0, 1e3};

  for (double scale : scales) {
    SphereShape s1(scale);
    SphereShape s2(scale);

    Eigen::Isometry3d tf1 = Eigen::Isometry3d::Identity();
    Eigen::Isometry3d tf2 = Eigen::Isometry3d::Identity();
    tf2.translation() = Eigen::Vector3d(3.0 * scale, 0, 0);

    DistanceResult result;
    double dist = distanceSphereSphere(s1, tf1, s2, tf2, result);

    double tol = 1e-6;
    if (scale > 1.0) {
      tol = 1e-6 * scale;
    }

    EXPECT_NEAR(dist, scale, tol);
    EXPECT_NEAR(result.pointOnObject1.x(), scale, tol);
    EXPECT_NEAR(result.pointOnObject2.x(), 2.0 * scale, tol);
  }
}

TEST(DistanceSphereSphere, SignedDistanceWitnessesAcrossSphereSets)
{
  struct SphereSpec
  {
    double radius;
    Eigen::Vector3d center;
  };

  const std::array<SphereSpec, 4> separatedSpheres{{
      {0.5, Eigen::Vector3d(0.0, 0.0, -1.0)},
      {0.6, Eigen::Vector3d(1.2, 0.0, 0.0)},
      {0.4, Eigen::Vector3d(-0.3, 0.0, 0.0)},
      {0.3, Eigen::Vector3d(0.0, 0.0, 1.0)},
  }};

  for (std::size_t i = 0; i < separatedSpheres.size(); ++i) {
    for (std::size_t j = i + 1; j < separatedSpheres.size(); ++j) {
      const auto& sphere1 = separatedSpheres[i];
      const auto& sphere2 = separatedSpheres[j];
      ASSERT_GT(
          (sphere2.center - sphere1.center).norm(),
          sphere1.radius + sphere2.radius);
      expectSphereSphereSignedDistanceCase(
          "separated small-sphere set",
          sphere1.radius,
          sphere1.center,
          sphere2.radius,
          sphere2.center);
    }
  }

  const std::array<SphereSpec, 4> penetratingSpheres{{
      {0.5, Eigen::Vector3d(0.0, 0.0, 0.0)},
      {0.5, Eigen::Vector3d(0.75, 0.0, 0.0)},
      {0.3, Eigen::Vector3d(0.2, 0.0, 0.0)},
      {0.4, Eigen::Vector3d(0.2, 0.0, 0.4)},
  }};

  for (std::size_t i = 0; i < penetratingSpheres.size(); ++i) {
    for (std::size_t j = i + 1; j < penetratingSpheres.size(); ++j) {
      const auto& sphere1 = penetratingSpheres[i];
      const auto& sphere2 = penetratingSpheres[j];
      ASSERT_LT(
          (sphere2.center - sphere1.center).norm(),
          sphere1.radius + sphere2.radius);
      expectSphereSphereSignedDistanceCase(
          "penetrating small-sphere set",
          sphere1.radius,
          sphere1.center,
          sphere2.radius,
          sphere2.center);
    }
  }

  const Eigen::Isometry3d frame = makeSignedDistanceFrame();
  for (const double offset : {40.0, 30.1, 29.9, 25.0}) {
    const Eigen::Vector3d center1 = frame * Eigen::Vector3d::Zero();
    const Eigen::Vector3d center2 = frame * Eigen::Vector3d(offset, 0.0, 0.0);

    expectSphereSphereSignedDistanceCase(
        "large spheres transformed together", 20.0, center1, 10.0, center2);
    expectSphereSphereSignedDistanceCase(
        "large spheres transformed together with argument order swapped",
        20.0,
        center2,
        10.0,
        center1);
  }

  expectSphereSphereSignedDistanceCase(
      "large spheres diagonal penetration",
      20.0,
      frame * Eigen::Vector3d::Zero(),
      10.0,
      frame * Eigen::Vector3d(20.0, 0.0, 20.0));
}

TEST(DistanceSphereSphere, PointDepthSamplesAcrossInsideSurfaceOutside)
{
  constexpr double radius = 1.0;

  expectSpherePointDepth(radius, Eigen::Vector3d::Zero(), 1.0);

  for (const int axis : {0, 1, 2}) {
    for (const double sign : {-1.0, 1.0}) {
      Eigen::Vector3d halfRadius = Eigen::Vector3d::Zero();
      halfRadius[axis] = sign * 0.5;
      expectSpherePointDepth(radius, halfRadius, 0.5);

      Eigen::Vector3d nearSurface = Eigen::Vector3d::Zero();
      nearSurface[axis] = sign * 0.9;
      expectSpherePointDepth(radius, nearSurface, 0.1);

      Eigen::Vector3d surface = Eigen::Vector3d::Zero();
      surface[axis] = sign;
      expectSpherePointDepth(radius, surface, 0.0);

      Eigen::Vector3d outside = Eigen::Vector3d::Zero();
      outside[axis] = sign * 1.1;
      expectSpherePointDepth(radius, outside, -0.1);
    }
  }

  for (const double sx : {-1.0, 1.0}) {
    for (const double sy : {-1.0, 1.0}) {
      expectSpherePointDepth(
          radius, Eigen::Vector3d(0.3 * sx, 0.4 * sy, 0.0), 0.5);
      expectSpherePointDepth(
          radius, Eigen::Vector3d(0.6 * sx, 0.8 * sy, 0.0), 0.0);
      expectSpherePointDepth(
          radius, Eigen::Vector3d(0.9 * sx, 1.2 * sy, 0.0), -0.5);
    }
  }
}

TEST(DistanceSphereBox, Separated)
{
  SphereShape sphere(1.0);
  BoxShape box(Eigen::Vector3d(0.5, 0.5, 0.5));

  Eigen::Isometry3d tfSphere = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d tfBox = Eigen::Isometry3d::Identity();
  tfBox.translation() = Eigen::Vector3d(3.0, 0, 0);

  DistanceResult result;
  double dist = distanceSphereBox(sphere, tfSphere, box, tfBox, result);

  EXPECT_NEAR(dist, 1.5, 1e-6);
}

TEST(DistanceSphereBox, Touching)
{
  SphereShape sphere(1.0);
  BoxShape box(Eigen::Vector3d(0.5, 0.5, 0.5));

  Eigen::Isometry3d tfSphere = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d tfBox = Eigen::Isometry3d::Identity();
  tfBox.translation() = Eigen::Vector3d(1.5, 0, 0);

  DistanceResult result;
  double dist = distanceSphereBox(sphere, tfSphere, box, tfBox, result);

  EXPECT_NEAR(dist, 0.0, 1e-6);
}

TEST(DistanceSphereBox, GrazingGap)
{
  SphereShape sphere(1.0);
  BoxShape box(Eigen::Vector3d(1.0, 1.0, 1.0));

  const double gap = 1e-4;

  Eigen::Isometry3d tfSphere = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d tfBox = Eigen::Isometry3d::Identity();
  tfSphere.translation() = Eigen::Vector3d(2.0 + gap, 0, 0);

  DistanceResult result;
  double dist = distanceSphereBox(sphere, tfSphere, box, tfBox, result);

  EXPECT_NEAR(dist, gap, 1e-7);
}

TEST(DistanceSphereBox, Penetrating)
{
  SphereShape sphere(1.0);
  BoxShape box(Eigen::Vector3d(0.5, 0.5, 0.5));

  Eigen::Isometry3d tfSphere = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d tfBox = Eigen::Isometry3d::Identity();
  tfBox.translation() = Eigen::Vector3d(1.0, 0, 0);

  DistanceResult result;
  double dist = distanceSphereBox(sphere, tfSphere, box, tfBox, result);

  EXPECT_LT(dist, 0.0);
}

TEST(DistanceSphereBox, PointDepthSamplesAcrossInsideSurfaceOutside)
{
  const Eigen::Vector3d halfExtents(1.0, 1.0, 1.0);

  expectBoxPointDepth(halfExtents, Eigen::Vector3d::Zero(), 1.0);

  for (const int axis : {0, 1, 2}) {
    for (const double sign : {-1.0, 1.0}) {
      Eigen::Vector3d halfDepth = Eigen::Vector3d::Zero();
      halfDepth[axis] = sign * 0.5;
      expectBoxPointDepth(halfExtents, halfDepth, 0.5);

      Eigen::Vector3d nearSurface = Eigen::Vector3d::Zero();
      nearSurface[axis] = sign * 0.9;
      expectBoxPointDepth(halfExtents, nearSurface, 0.1);

      Eigen::Vector3d surface = Eigen::Vector3d::Zero();
      surface[axis] = sign;
      expectBoxPointDepth(halfExtents, surface, 0.0);

      Eigen::Vector3d outside = Eigen::Vector3d::Zero();
      outside[axis] = sign * 1.1;
      expectBoxPointDepth(halfExtents, outside, -0.1);
    }
  }

  for (const double sx : {-1.0, 1.0}) {
    for (const double sy : {-1.0, 1.0}) {
      expectBoxPointDepth(
          halfExtents, Eigen::Vector3d(0.3 * sx, 0.4 * sy, 0.0), 0.6);
      expectBoxPointDepth(
          halfExtents, Eigen::Vector3d(1.0 * sx, 0.3 * sy, 0.0), 0.0);
      expectBoxPointDepth(
          halfExtents, Eigen::Vector3d(1.3 * sx, 1.4 * sy, 0.0), -0.5);
    }
  }

  for (const double sx : {-1.0, 1.0}) {
    expectBoxPointDepth(halfExtents, Eigen::Vector3d(1.2 * sx, 1.4, 1.4), -0.6);
    expectBoxPointDepth(halfExtents, Eigen::Vector3d(1.4, 1.2 * sx, 1.4), -0.6);
    expectBoxPointDepth(halfExtents, Eigen::Vector3d(1.4, 1.4, 1.2 * sx), -0.6);
  }
}

TEST(DistanceSphereBox, SeparatedAcrossBoxFramesAndSphereOrientations)
{
  constexpr double sphereRadius = 0.7;
  const Eigen::Vector3d halfSize(0.3, 0.6, 1.8);
  const Eigen::Vector3d faceCenter(
      halfSize.x() * 0.5,
      halfSize.y() * 0.5,
      halfSize.z() + sphereRadius * 1.1);
  const Eigen::Vector3d cornerCenter
      = halfSize + Eigen::Vector3d::Ones() * sphereRadius * 1.25;
  const Eigen::Vector3d cornerToSphere = cornerCenter - halfSize;
  const double cornerDistanceToBox = cornerToSphere.norm();

  const std::array<SphereBoxDistanceCase, 2> cases{{
      {"separated from +z face",
       halfSize,
       sphereRadius,
       faceCenter,
       sphereRadius * 0.1,
       faceCenter - Eigen::Vector3d::UnitZ() * sphereRadius,
       Eigen::Vector3d(faceCenter.x(), faceCenter.y(), halfSize.z())},
      {"separated from +x,+y,+z corner",
       halfSize,
       sphereRadius,
       cornerCenter,
       cornerDistanceToBox - sphereRadius,
       cornerCenter - cornerToSphere.normalized() * sphereRadius,
       halfSize},
  }};

  const auto boxTransforms = makeSphereBoxDistanceBoxTransforms();
  const auto sphereOrientations = makeSphereBoxDistanceSphereOrientations();
  for (const auto& testCase : cases) {
    for (const auto& boxTransform : boxTransforms) {
      for (const auto& sphereOrientation : sphereOrientations) {
        expectSphereBoxDistanceCase(testCase, boxTransform, sphereOrientation);
      }
    }
  }
}

TEST(DistanceSphereBox, ContactsSignedDistanceAcrossFramesAndOrientations)
{
  constexpr double sphereRadius = 0.7;
  const Eigen::Vector3d halfSize(0.3, 0.6, 1.8);
  const double externalDepth = sphereRadius * 0.5;
  const Eigen::Vector3d vertexNormalToBox
      = Eigen::Vector3d(-1.0, -2.0, -3.0).normalized();
  const double centerInset = halfSize.minCoeff() * 0.5;

  std::vector<SphereBoxDistanceCase> cases;
  cases.push_back(
      {"external +z face",
       halfSize,
       sphereRadius,
       halfSize + Eigen::Vector3d(0.0, 0.0, sphereRadius - externalDepth),
       -externalDepth,
       Eigen::Vector3d::Zero(),
       Eigen::Vector3d::Zero()});
  cases.push_back(
      {"external +x,+y,+z vertex",
       halfSize,
       sphereRadius,
       halfSize - vertexNormalToBox * (sphereRadius - externalDepth),
       -externalDepth,
       Eigen::Vector3d::Zero(),
       Eigen::Vector3d::Zero()});

  for (int axis = 0; axis < 3; ++axis) {
    for (double sign : {-1.0, 1.0}) {
      const Eigen::Vector3d direction = sign * Eigen::Vector3d::Unit(axis);
      cases.push_back(
          {"internal nearest face",
           halfSize,
           sphereRadius,
           direction * (centerInset - halfSize[axis]),
           -(sphereRadius + centerInset),
           Eigen::Vector3d::Zero(),
           Eigen::Vector3d::Zero()});
    }
  }

  cases.push_back(
      {"center on +z face",
       halfSize,
       sphereRadius,
       Eigen::Vector3d(0.0, 0.0, halfSize.z()),
       -sphereRadius,
       Eigen::Vector3d::Zero(),
       Eigen::Vector3d::Zero()});
  cases.push_back(
      {"center on +x,+y,+z corner",
       halfSize,
       sphereRadius,
       halfSize,
       -sphereRadius,
       Eigen::Vector3d::Zero(),
       Eigen::Vector3d::Zero()});

  const std::array<Eigen::Vector3d, 6> coincidentHalfSizes{{
      Eigen::Vector3d(10.0, 10.0, 10.0),
      Eigen::Vector3d(10.0, 15.0, 10.0),
      Eigen::Vector3d(10.0, 12.0, 14.0),
      Eigen::Vector3d(15.0, 10.0, 10.0),
      Eigen::Vector3d(15.0, 10.0, 14.0),
      Eigen::Vector3d(15.0, 12.0, 10.0),
  }};
  for (const auto& coincidentHalfSize : coincidentHalfSizes) {
    cases.push_back(
        {"coincident centers",
         coincidentHalfSize,
         5.0,
         Eigen::Vector3d::Zero(),
         -15.0,
         Eigen::Vector3d::Zero(),
         Eigen::Vector3d::Zero()});
  }

  const auto boxTransforms = makeSphereBoxDistanceBoxTransforms();
  const auto sphereOrientations = makeSphereBoxDistanceSphereOrientations();
  for (const auto& testCase : cases) {
    for (const auto& boxTransform : boxTransforms) {
      for (const auto& sphereOrientation : sphereOrientations) {
        expectSphereBoxSignedDistanceCase(
            testCase, boxTransform, sphereOrientation);
      }
    }
  }
}

TEST(DistanceSphereBox, IncompatibleScaleSeparatedNearestPoints)
{
  const Eigen::Vector3d largeSphereNormalToBox
      = Eigen::Vector3d(-1.0, -2.0, -3.0).normalized();
  const Eigen::Vector3d tinyBoxHalfSize(0.1, 0.15, 0.2);
  const Eigen::Vector3d largeSphereCenter
      = tinyBoxHalfSize - largeSphereNormalToBox * (10.0 + 0.01);

  const std::array<SphereBoxDistanceCase, 2> cases{{
      {"long skinny box separated from small sphere",
       Eigen::Vector3d(15.0, 1.0, 1.0),
       0.01,
       Eigen::Vector3d(15.0 * 0.95, 0.0, 1.0 + 0.01 + 0.001),
       0.001,
       Eigen::Vector3d(15.0 * 0.95, 0.0, 1.0 + 0.001),
       Eigen::Vector3d(15.0 * 0.95, 0.0, 1.0)},
      {"large sphere separated from tiny box",
       tinyBoxHalfSize,
       10.0,
       largeSphereCenter,
       0.01,
       largeSphereCenter + largeSphereNormalToBox * 10.0,
       tinyBoxHalfSize},
  }};

  const auto boxTransforms = makeSphereBoxDistanceBoxTransforms();
  for (const auto& testCase : cases) {
    for (const auto& boxTransform : boxTransforms) {
      expectSphereBoxDistanceCase(
          testCase,
          boxTransform,
          SphereBoxSphereOrientation{0.0, Eigen::Vector3d::UnitX()});
    }
  }
}

TEST(DistanceSphereBox, IncompatibleScaleContactsSignedDistance)
{
  const Eigen::Vector3d largeSphereNormalToBox
      = Eigen::Vector3d(-1.0, -2.0, -3.0).normalized();
  const Eigen::Vector3d tinyBoxHalfSize(0.1, 0.15, 0.2);
  const std::array<SphereBoxDistanceCase, 2> cases{{
      {"long skinny box collides with small sphere",
       Eigen::Vector3d(15.0, 1.0, 1.0),
       0.01,
       Eigen::Vector3d(15.0 * 0.95, 0.0, 1.0 + 0.01 * 0.5),
       -0.005,
       Eigen::Vector3d::Zero(),
       Eigen::Vector3d::Zero()},
      {"large sphere collides with tiny box",
       tinyBoxHalfSize,
       10.0,
       tinyBoxHalfSize - largeSphereNormalToBox * (10.0 - 0.05),
       -0.05,
       Eigen::Vector3d::Zero(),
       Eigen::Vector3d::Zero()},
  }};

  const auto boxTransforms = makeSphereBoxDistanceBoxTransforms();
  for (const auto& testCase : cases) {
    for (const auto& boxTransform : boxTransforms) {
      expectSphereBoxSignedDistanceCase(
          testCase,
          boxTransform,
          SphereBoxSphereOrientation{0.0, Eigen::Vector3d::UnitX()});
    }
  }
}

TEST(DistanceSphereBox, LargeSphereSignedDistanceWithCommonFrames)
{
  const SphereShape sphere(20.0);
  const BoxShape box(Eigen::Vector3d(2.5, 2.5, 2.5));
  const std::array<Eigen::Isometry3d, 2> frames{{
      Eigen::Isometry3d::Identity(),
      makeSignedDistanceFrame(),
  }};

  for (const auto& frame : frames) {
    DistanceResult overlapResult;
    const double overlapDistance
        = distanceSphereBox(sphere, frame, box, frame, overlapResult);
    EXPECT_LT(overlapDistance, 0.0);
    EXPECT_LT(overlapResult.distance, 0.0);

    for (const auto& offsetAndDistance :
         {std::pair{22.6, 0.1}, std::pair{40.0, 17.5}}) {
      Eigen::Isometry3d boxTransform = Eigen::Isometry3d::Identity();
      boxTransform.translation()
          = Eigen::Vector3d(offsetAndDistance.first, 0.0, 0.0);
      boxTransform = frame * boxTransform;

      DistanceResult result;
      const double distance
          = distanceSphereBox(sphere, frame, box, boxTransform, result);

      EXPECT_NEAR(distance, offsetAndDistance.second, 1e-9);
      EXPECT_NEAR(result.distance, offsetAndDistance.second, 1e-9);
      EXPECT_NEAR(
          (result.pointOnObject2 - result.pointOnObject1).norm(),
          offsetAndDistance.second,
          1e-9);
      EXPECT_TRUE(result.pointOnObject1.allFinite());
      EXPECT_TRUE(result.pointOnObject2.allFinite());
    }
  }
}

TEST(DistanceBoxBox, Separated)
{
  BoxShape b1(Eigen::Vector3d(0.5, 0.5, 0.5));
  BoxShape b2(Eigen::Vector3d(0.5, 0.5, 0.5));

  Eigen::Isometry3d tf1 = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d tf2 = Eigen::Isometry3d::Identity();
  tf2.translation() = Eigen::Vector3d(3.0, 0, 0);

  DistanceResult result;
  double dist = distanceBoxBox(b1, tf1, b2, tf2, result);

  EXPECT_NEAR(dist, 2.0, 1e-6);
}

TEST(DistanceBoxBox, OverlappingSameOrientation)
{
  BoxShape b1(Eigen::Vector3d::Ones());
  BoxShape b2(Eigen::Vector3d::Ones());

  Eigen::Isometry3d tf1 = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d tf2 = Eigen::Isometry3d::Identity();
  tf2.translation() = Eigen::Vector3d(1.5, 0, 0);

  DistanceResult result;
  double dist = distanceBoxBox(b1, tf1, b2, tf2, result);

  EXPECT_NEAR(dist, -0.5, 1e-6);
  EXPECT_NEAR(result.distance, -0.5, 1e-6);
  EXPECT_NEAR(result.normal.x(), 1.0, 1e-6);
  EXPECT_NEAR(result.pointOnObject1.x(), 1.0, 1e-6);
  EXPECT_NEAR(result.pointOnObject2.x(), 0.5, 1e-6);
}

TEST(DistanceBoxBox, ThinFeatureSeparated)
{
  BoxShape b1(Eigen::Vector3d(1.0, 1.0, 1e-3));
  BoxShape b2(Eigen::Vector3d(1.0, 1.0, 1e-3));

  const double gap = 5e-3;

  Eigen::Isometry3d tf1 = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d tf2 = Eigen::Isometry3d::Identity();
  tf2.translation() = Eigen::Vector3d(0, 0, 2e-3 + gap);

  DistanceResult result;
  double dist = distanceBoxBox(b1, tf1, b2, tf2, result);

  EXPECT_NEAR(dist, gap, 1e-6);
}

TEST(DistanceBoxBox, DegenerateSimplexWitnessRegression)
{
  BoxShape box1(Eigen::Vector3d(1.375, 3.0, 0.025));
  BoxShape box2(Eigen::Vector3d(0.212, 0.075, 0.0843));

  Eigen::Isometry3d tf1 = Eigen::Isometry3d::Identity();
  tf1.translation() = Eigen::Vector3d(1.625, 0.0, 0.5);

  Eigen::Isometry3d tf2 = Eigen::Isometry3d::Identity();
  tf2.linear() = Eigen::Quaterniond(0.672811, 0.340674, 0.155066, 0.638138)
                     .normalized()
                     .toRotationMatrix();
  tf2.translation() = Eigen::Vector3d(0.192074, -0.277870, 0.273546);

  DistanceResult result;
  const double dist = distanceBoxBox(box1, tf1, box2, tf2, result);

  EXPECT_NEAR(dist, 0.053516162824549, 1e-9);
  EXPECT_NEAR(result.distance, dist, 1e-12);
  EXPECT_TRUE(result.pointOnObject1.allFinite());
  EXPECT_TRUE(result.pointOnObject2.allFinite());
  EXPECT_TRUE(result.normal.allFinite());
  EXPECT_NEAR(
      (result.pointOnObject2 - result.pointOnObject1).norm(), dist, 1e-9);
}

TEST(DistanceBoxBox, RealWorldWitnessRegressions)
{
  expectBoxBoxRegressionWitnesses(
      "edge nearest-feature regression",
      Eigen::Vector3d(0.03, 0.12, 0.1),
      makeDistanceTransform({
          -3.0627937852578681533e-08,
          -0.99999999999999888978,
          -2.8893865161583314238e-08,
          0.63499979627350811029,
          0.9999999999999980016,
          -3.0627939739957803544e-08,
          6.4729926918527511769e-08,
          -0.48500002215636439651,
          -6.4729927722963847085e-08,
          -2.8893863029448751323e-08,
          0.99999999999999711342,
          1.0778146458339641356,
          0.0,
          0.0,
          0.0,
          1.0,
      }),
      Eigen::Vector3d(0.025, 0.35, 1.845),
      makeDistanceTransform({
          0.0,
          -1.0,
          0.0,
          0.8,
          1.0,
          0.0,
          0.0,
          -0.4575,
          0.0,
          0.0,
          1.0,
          1.0225,
          0.0,
          0.0,
          0.0,
          1.0,
      }));

  expectBoxBoxRegressionWitnesses(
      "thin tilted triangle-size regression",
      Eigen::Vector3d(0.46, 0.48, 0.01),
      makeDistanceTransform({
          1.0,
          0.0,
          0.0,
          -0.72099999999999997424,
          0.0,
          1.0,
          0.0,
          -0.77200000000000001954,
          0.0,
          0.0,
          1.0,
          0.81000000000000005329,
          0.0,
          0.0,
          0.0,
          1.0,
      }),
      Eigen::Vector3d(0.049521, 0.146, 0.0725),
      makeDistanceTransform({
          0.10758262492983036718,
          -0.6624881850015212903,
          -0.74130653817877356637,
          -0.42677133002999478872,
          0.22682184885125472595,
          -0.709614040775253474,
          0.6670830248314786326,
          -0.76596851247746788882,
          -0.96797615037608542021,
          -0.23991106241273435495,
          0.07392465377049164954,
          0.80746731400091054098,
          0.0,
          0.0,
          0.0,
          1.0,
      }));

  expectBoxBoxRegressionWitnesses(
      "colinear edge query regression",
      Eigen::Vector3d(0.49, 0.05, 0.21),
      makeDistanceTransform({
          4.8966386501092529215e-12,
          -1.0,
          0.0,
          -0.43999999999999994671,
          1.0,
          4.8966386501092529215e-12,
          0.0,
          -0.61499999999858001587,
          0.0,
          0.0,
          1.0,
          0.35499999999999998224,
          0.0,
          0.0,
          0.0,
          1.0,
      }),
      Eigen::Vector3d(0.035, 0.12, 0.03),
      makeDistanceTransform({
          0.83512153565236335595,
          -0.55006546945762568868,
          -9.4542360608233572896e-16,
          -0.40653441507331000704,
          0.55006546945762568868,
          0.83512153565236313391,
          1.1787444236552387666e-15,
          -0.69166166923735727945,
          1.2902271444330665572e-16,
          -1.4878153530113264589e-15,
          1.0,
          0.43057093858718892276,
          0.0,
          0.0,
          0.0,
          1.0,
      }));

  expectBoxBoxRegressionWitnesses(
      "thin plate overlap regression",
      Eigen::Vector3d(0.614, 3.0, 0.37),
      makeDistanceBoxTransform(Eigen::Vector3d(-0.675, 0.0, 0.9115)),
      Eigen::Vector3d(0.494, 0.552, 0.01),
      makeDistanceBoxTransform(Eigen::Vector3d(-0.692, 0.0, 0.935)));

  expectBoxBoxRegressionWitnesses(
      "offset asymmetric box regression",
      Eigen::Vector3d(0.2, 0.33, 0.1),
      makeDistanceBoxTransform(
          Eigen::Vector3d(
              -0.071000000000000035305,
              -0.77200000000000001954,
              0.79999999999999993339)),
      Eigen::Vector3d(0.452, 0.27, 0.6),
      makeDistanceBoxTransform(
          Eigen::Vector3d(
              0.12099999999999999645,
              -0.78769605692727695523,
              0.53422044196125151316)));

  const double expectedTouchingDistance = 0.0;
  expectBoxBoxRegressionWitnesses(
      "rotated expected-touching regression",
      Eigen::Vector3d(
          0.31650000000000000355,
          0.22759999999999999676,
          0.1768000000000000127),
      makeDistanceTransform({
          0.44540578475530234748,
          0.89532881496493399442,
          -8.8937407685638678971e-09,
          1.2652949075960071568,
          -0.89532881496493377238,
          0.44540578475530190339,
          -2.8948680226084145336e-08,
          1.4551012423210101243,
          -2.1957263975186326105e-08,
          2.0856732016652919226e-08,
          0.99999999999999955591,
          0.49480006232932938204,
          0.0,
          0.0,
          0.0,
          1.0,
      }),
      Eigen::Vector3d(
          0.49430000000000001714,
          0.35460000000000002629,
          0.075200000000000002953),
      makeDistanceTransform({
          0.44171122913485860728,
          0.8971572827861190591,
          -1.622764514865468214e-09,
          1.1304016226141906376,
          -0.8971572827861190591,
          0.44171122913485860728,
          -5.1621053952306079594e-09,
          1.8410802645284281009,
          -3.9144271413829990148e-09,
          3.7360349218094348098e-09,
          1.0,
          0.44400006232932492933,
          0.0,
          0.0,
          0.0,
          1.0,
      }),
      &expectedTouchingDistance);

  expectBoxBoxRegressionWitnesses(
      "near-perpendicular Drake regression",
      Eigen::Vector3d(
          0.050000000000000002776,
          0.55000000000000004441,
          0.2999999999999999889),
      makeDistanceTransform({
          0.00079632671073326442932,
          -0.99999968293183538748,
          0.0,
          0.75000000000000055511,
          0.99999968293183538748,
          0.00079632671073326442932,
          0.0,
          -2.4083553715553102684e-15,
          0.0,
          0.0,
          1.0,
          0.14999999999999999445,
          0.0,
          0.0,
          0.0,
          1.0,
      }),
      Eigen::Vector3d(0.25, 0.2000000000000000111, 0.14999999999999999445),
      makeDistanceTransform({
          6.1232339957367660359e-17,
          0.0,
          1.0,
          0.75,
          -1.0,
          6.1232339957367660359e-17,
          6.1232339957367660359e-17,
          0.0,
          -6.1232339957367660359e-17,
          -1.0,
          3.7493994566546440196e-33,
          0.14999999999999999445,
          0.0,
          0.0,
          0.0,
          1.0,
      }));
}

TEST(DistanceBoxBox, TiltedKissingContactAcrossScales)
{
  Eigen::Matrix3d rotation;
  rotation << 0.94096063217417758029, 0.29296840037289501035,
      0.16959541586174811667, -0.23569836841299879326, 0.92661523595848427348,
      -0.29296840037289506586, -0.2429801799032638987, 0.23569836841299884878,
      0.94096063217417758029;

  for (const double dim : {0.01, 0.25, 0.5, 10.0, 1000.0}) {
    const Eigen::Vector3d boxSize(dim, dim, dim);
    const Eigen::Vector3d origin(0.0, 0.0, 5.0 * dim);

    Eigen::Isometry3d tf1 = Eigen::Isometry3d::Identity();
    tf1.linear() = rotation;
    tf1.translation() = origin;

    for (const double relativeDistance :
         {-1e-15, -2.5e-16, -1e-16, 0.0, 1e-16, 2.5e-16, 1e-15}) {
      const double scaledDistance = relativeDistance * std::max(1.0, dim);
      Eigen::Isometry3d tf2 = Eigen::Isometry3d::Identity();
      tf2.linear() = rotation;
      tf2.translation()
          = origin + rotation * Eigen::Vector3d(0.0, dim + scaledDistance, 0.0);

      expectBoxBoxRegressionWitnesses(
          "tilted same-orientation near-touching stack",
          boxSize,
          tf1,
          boxSize,
          tf2);
    }
  }
}

TEST(DistanceSphereBox, NearBoundaryRegression)
{
  const SphereShape sphere(0.06);
  const BoxShape box(Eigen::Vector3d(0.05, 0.05, 0.05));

  const Eigen::Isometry3d sphereTf = makeDistanceTransform({
      -0.99999999999999955591,
      -4.4637642593504144998e-09,
      0.0,
      1.7855056639081962376e-10,
      4.4637642593504144998e-09,
      -0.99999999999999955591,
      0.0,
      0.039999999999999993894,
      0.0,
      0.0,
      1.0000000000000008882,
      0.33000000000000012657,
      0.0,
      0.0,
      0.0,
      1.0,
  });
  const Eigen::Isometry3d boxTf
      = makeDistanceBoxTransform(Eigen::Vector3d(0.05, 0.15, 0.35));

  DistanceResult result;
  const double distance
      = distanceSphereBox(sphere, sphereTf, box, boxTf, result);

  EXPECT_NEAR(distance, 0.0, 1e-9);
  EXPECT_NEAR(result.distance, distance, 1e-12);
  EXPECT_TRUE(result.pointOnObject1.allFinite());
  EXPECT_TRUE(result.pointOnObject2.allFinite());
  EXPECT_TRUE(result.normal.allFinite());
}

TEST(NarrowPhaseDistance, ConvexConvexPrismWitnessRegressions)
{
  expectConvexConvexRegressionWitnesses(
      "near-touching swept prism against triangular prism",
      {
          {0.5, 0.70710700000000004106, -0.70710700000000004106},
          {0.5, 0, -1},
          {0.5, 1, 0},
          {0.5, -1, 0},
          {0.5, -0.70710700000000004106, -0.70710700000000004106},
          {-0.5, 0, 1},
          {-0.5, -0.70710700000000004106, 0.70710700000000004106},
          {-0.5, 0.70710700000000004106, 0.70710700000000004106},
          {0.5, 0, 1},
          {-0.5, -1, 0},
          {-0.5, 0.70710700000000004106, -0.70710700000000004106},
          {-0.5, 0, -1},
          {-0.5, -0.70710700000000004106, -0.70710700000000004106},
          {-0.5, 1, 0},
          {0.5, 0.70710700000000004106, 0.70710700000000004106},
          {0.5, -0.70710700000000004106, 0.70710700000000004106},
      },
      makeDistanceTransform({
          -2.0399676677885372849e-09,
          0.77129744817973977522,
          -0.63647485922966484662,
          11.477445682202462862,
          2.4720879917217940548e-09,
          0.63647485922966484662,
          0.77129744817973977522,
          9.7785056920756936449,
          1,
          0,
          -3.2051032938795742666e-09,
          0,
          0,
          0,
          0,
          1,
      }),
      {
          {10.294759000000000881, 8.9321570000000001244, -0.5},
          {11.618370999999999782, 9.4565629999999991639, -0.5},
          {11.452372999999999692, 8.2276720000000000965, -0.5},
          {10.294759000000000881, 8.9321570000000001244, 0.5},
          {11.452372999999999692, 8.2276720000000000965, 0.5},
          {11.618370999999999782, 9.4565629999999991639, 0.5},
      },
      Eigen::Isometry3d::Identity());

  expectConvexConvexRegressionWitnesses(
      "skew prism against triangular prism",
      {
          {0.5, 0.5, -0.86602500000000004476},
          {0.5, -0.5, -0.86602500000000004476},
          {0.5, 1, 0},
          {-0.5, -0.5, 0.86602500000000004476},
          {-0.5, 0.5, 0.86602500000000004476},
          {-0.5, -1, 0},
          {0.5, -0.5, 0.86602500000000004476},
          {-0.5, 0.5, -0.86602500000000004476},
          {-0.5, -0.5, -0.86602500000000004476},
          {-0.5, 1, 0},
          {0.5, -1, 0},
          {0.5, 0.5, 0.86602500000000004476},
      },
      makeDistanceTransform({
          -3.0140487242790098936e-09,
          0.34009658459984087875,
          -0.94039051098122172778,
          4.2864836941313040342,
          1.090044683538143324e-09,
          0.94039051098122172778,
          0.34009658459984087875,
          7.3869576872253679412,
          1,
          0,
          -3.2051032938795742666e-09,
          0,
          0,
          0,
          0,
          1,
      }),
      {
          {4.1494020000000002568, 8.4937090000000008416, -0.5},
          {5.1884509999999997021, 7.476263000000000325, -0.5},
          {3.9316469999999998919, 7.3745029999999998083, -0.5},
          {4.1494020000000002568, 8.4937090000000008416, 0.5},
          {3.9316469999999998919, 7.3745029999999998083, 0.5},
          {5.1884509999999997021, 7.476263000000000325, 0.5},
      },
      Eigen::Isometry3d::Identity());

  expectConvexConvexRegressionWitnesses(
      "long rotated prism against triangular prism",
      {
          {0.5, 1, 0},
          {0.5, 0.62348999999999998867, -0.7818310000000000537},
          {0.5, -0.22252099999999999658, -0.97492800000000001681},
          {0.5, 0.62348999999999998867, 0.7818310000000000537},
          {-0.5, -0.90096900000000001985, 0.4338839999999999919},
          {-0.5, -0.22252099999999999658, 0.97492800000000001681},
          {-0.5, -0.90096900000000001985, -0.4338839999999999919},
          {0.5, -0.90096900000000001985, 0.4338839999999999919},
          {-0.5, 0.62348999999999998867, -0.7818310000000000537},
          {-0.5, 0.62348999999999998867, 0.7818310000000000537},
          {-0.5, -0.22252099999999999658, -0.97492800000000001681},
          {-0.5, 1, 0},
          {0.5, -0.90096900000000001985, -0.4338839999999999919},
          {0.5, -0.22252099999999999658, 0.97492800000000001681},
      },
      makeDistanceTransform({
          -2.0479437360480804916e-09,
          -0.76923712747984118732,
          -0.63896341186844385351,
          13.670417904867957049,
          -2.4654844510601008403e-09,
          0.63896341186844385351,
          -0.76923712747984118732,
          13.169816779836704512,
          1,
          0,
          -3.2051032938795742666e-09,
          0,
          0,
          0,
          0,
          1,
      }),
      {
          {14.835100999999999871, 13.060409999999999187, -0.5},
          {13.785037000000000873, 12.890523999999999205, -0.5},
          {14.767995000000000871, 13.150247999999999493, -0.5},
          {14.835100999999999871, 13.060409999999999187, 0.5},
          {14.767995000000000871, 13.150247999999999493, 0.5},
          {13.785037000000000873, 12.890523999999999205, 0.5},
      },
      Eigen::Isometry3d::Identity());

  expectConvexConvexRegressionWitnesses(
      "flat octagonal prism against triangular prism",
      {
          {0.5, -0.92388000000000003453, 0.38268299999999999539},
          {-0.5, 0.92388000000000003453, -0.38268299999999999539},
          {0.5, 0.92388000000000003453, -0.38268299999999999539},
          {-0.5, 0.92388000000000003453, 0.38268299999999999539},
          {-0.5, -0.92388000000000003453, -0.38268299999999999539},
          {0.5, 0.92388000000000003453, 0.38268299999999999539},
          {-0.5, -0.92388000000000003453, 0.38268299999999999539},
          {0.5, -0.92388000000000003453, -0.38268299999999999539},
      },
      makeDistanceTransform({
          -2.6487935924268804787e-09,
          -0.56303944378188575115,
          -0.82643002410717436579,
          15.712812998638135298,
          -1.8045995758494454423e-09,
          0.82643002410717436579,
          -0.56303944378188575115,
          10.219809745800642276,
          1,
          0,
          -3.2051032938795742666e-09,
          0,
          0,
          0,
          0,
          1,
      }),
      {
          {15.417082000000000619, 9.6986209999999992704, -0.5},
          {16.236605000000000842, 9.2332309999999999661, -0.5},
          {16.224319999999998743, 8.9158489999999996911, -0.5},
          {15.417082000000000619, 9.6986209999999992704, 0.5},
          {16.224319999999998743, 8.9158489999999996911, 0.5},
          {16.236605000000000842, 9.2332309999999999661, 0.5},
      },
      Eigen::Isometry3d::Identity());

  expectConvexConvexRegressionWitnesses(
      "hexagonal prism against triangular prism",
      {
          {0.5, 0.5, -0.86602500000000004476},
          {0.5, -0.5, -0.86602500000000004476},
          {0.5, 1, 0},
          {-0.5, -0.5, 0.86602500000000004476},
          {-0.5, 0.5, 0.86602500000000004476},
          {-0.5, -1, 0},
          {0.5, -0.5, 0.86602500000000004476},
          {-0.5, 0.5, -0.86602500000000004476},
          {-0.5, -0.5, -0.86602500000000004476},
          {-0.5, 1, 0},
          {0.5, -1, 0},
          {0.5, 0.5, 0.86602500000000004476},
      },
      makeDistanceTransform({
          -2.2411796174427631456e-09,
          0.7148738189791493669,
          -0.69925347545662319693,
          6.3335747278141161232,
          2.2912444319183419853e-09,
          0.69925347545662319693,
          0.7148738189791493669,
          8.4631576303276716544,
          1,
          0,
          -3.2051032938795742666e-09,
          0,
          0,
          0,
          0,
          1,
      }),
      {
          {5.2423630000000001061, 9.1228110000000004476, -0.5},
          {6.3596640000000004278, 8.2852599999999991809, -0.5},
          {5.6703770000000002227, 7.5985389999999997102, -0.5},
          {5.2423630000000001061, 9.1228110000000004476, 0.5},
          {5.6703770000000002227, 7.5985389999999997102, 0.5},
          {6.3596640000000004278, 8.2852599999999991809, 0.5},
      },
      Eigen::Isometry3d::Identity());

  expectConvexConvexRegressionWitnesses(
      "pentagonal prism against triangular prism",
      {
          {0.5, 0.30901699999999998614, -0.95105700000000004124},
          {0.5, 1, 0},
          {-0.5, -0.80901699999999998614, 0.58778500000000000192},
          {-0.5, 0.30901699999999998614, 0.95105700000000004124},
          {-0.5, -0.80901699999999998614, -0.58778500000000000192},
          {0.5, -0.80901699999999998614, 0.58778500000000000192},
          {-0.5, 0.30901699999999998614, -0.95105700000000004124},
          {-0.5, 1, 0},
          {0.5, -0.80901699999999998614, -0.58778500000000000192},
          {0.5, 0.30901699999999998614, 0.95105700000000004124},
      },
      makeDistanceTransform({
          -3.1934130613594990691e-09,
          -0.085331461972016672823,
          -0.99635261910516315087,
          11.261494468063983021,
          -2.7349614983807029573e-10,
          0.99635261910516315087,
          -0.085331461972016672823,
          5.7102314848013024928,
          1,
          0,
          -3.2051032938795742666e-09,
          0,
          0,
          0,
          0,
          1,
      }),
      {
          {10.924768999999999508, 5.6701019999999999754, -0.5},
          {10.83925699999999992, 4.8329899999999996751, -0.5},
          {11.452507000000000659, 5.5209809999999999164, -0.5},
          {10.924768999999999508, 5.6701019999999999754, 0.5},
          {11.452507000000000659, 5.5209809999999999164, 0.5},
          {10.83925699999999992, 4.8329899999999996751, 0.5},
      },
      Eigen::Isometry3d::Identity());

  expectConvexConvexRegressionWitnesses(
      "triangular prism against triangular prism",
      {
          {-0.5, -0.5, -0.86602500000000004476},
          {-0.5, -0.5, 0.86602500000000004476},
          {-0.5, 1, 0},
          {0.5, -0.5, -0.86602500000000004476},
          {0.5, 1, 0},
          {0.5, -0.5, 0.86602500000000004476},
      },
      makeDistanceTransform({
          -3.155367699156126597e-09,
          -0.17548349096519452739,
          -0.98448237383849002136,
          9.4277544959429171456,
          -5.6244271491403150214e-10,
          0.98448237383849002136,
          -0.17548349096519452739,
          4.3549730982604870633,
          1,
          0,
          -3.2051032938795742666e-09,
          0,
          0,
          0,
          0,
          1,
      }),
      {
          {7.502455000000000318, 3.9926669999999999661, -0.5},
          {7.9127499999999999503, 2.7334600000000000009, -0.5},
          {8.6743819999999995929, 3.7108469999999997846, -0.5},
          {7.502455000000000318, 3.9926669999999999661, 0.5},
          {8.6743819999999995929, 3.7108469999999997846, 0.5},
          {7.9127499999999999503, 2.7334600000000000009, 0.5},
      },
      Eigen::Isometry3d::Identity());
}

TEST(NarrowPhaseDistance, ConvexConvexDensePatchWitnessRegression)
{
  expectConvexConvexRegressionWitnesses(
      "dense patch convexes with nearly coincident features",
      {
          {7.7047904100000002003, 5.3783911499999996764, 2.5844529299999998706},
          {7.7023649100000000089, 5.3783911499999996764, 2.5830768499999998689},
          {7.6985051599999998473, 5.3783911499999996764, 2.5801444299999998222},
          {7.698255080000000028, 5.3783911499999996764, 2.5798758400000001423},
          {7.6993028399999996481, 5.4185012500000002689, 2.5809686100000002185},
          {7.6990137799999995849, 5.4185012500000002689, 2.5754017700000000346},
          {7.698255080000000028, 5.4185012500000002689, 2.5739778699999997791},
          {7.6995464199999998911, 5.4185012500000002689, 2.601399929999999916},
          {7.7026201900000001999, 5.3979848199999995728, 2.601399929999999916},
          {7.6989761000000003222, 5.4005056900000001363, 2.601399929999999916},
          {7.698255080000000028, 5.4023551100000002378, 2.601399929999999916},
          {7.7003324099999996832, 5.3979848199999995728, 2.601399929999999916},
          {7.698255080000000028, 5.4185012500000002689, 2.601399929999999916},
          {7.698255080000000028, 5.4173271999999998982, 2.5731573899999999888},
          {7.698255080000000028, 5.4144450800000001323, 2.5712845999999998092},
          {7.6996880299999999053, 5.4185012500000002689, 2.5920949000000002016},
          {7.7018140800000001178, 5.4122618999999998479, 2.5865317499999997963},
          {7.7038407400000004088, 5.402217689999999628, 2.601399929999999916},
          {7.7001765100000003628, 5.4017700499999996566, 2.5638155600000001044},
          {7.7043876799999999605, 5.4016286899999998994, 2.5638155600000001044},
          {7.6998023599999996236, 5.4016444000000003456, 2.5638155600000001044},
          {7.7019673800000001407, 5.4066704400000000774, 2.5661997699999998801},
          {7.7001109000000003135, 5.4066704400000000774, 2.5661997699999998801},
          {7.6996730900000001085, 5.4015580200000004041, 2.5638155600000001044},
          {7.6995555199999996532, 5.4014755599999997315, 2.5638155600000001044},
          {7.698255080000000028, 5.3981772299999999376, 2.5638155600000001044},
          {7.698255080000000028, 5.3783911499999996764, 2.5638155600000001044},
          {7.7032158500000003087, 5.4077816600000003788, 2.5865317499999997963},
          {7.7017549599999997056, 5.4128980100000001485, 2.5920949000000002016},
          {7.7018049800000003557, 5.4126899000000001649, 2.601399929999999916},
          {7.704890439999999785, 5.4005017699999999792, 2.5810054000000000052},
          {7.704890439999999785, 5.3997714200000004325, 2.5922163199999999073},
          {7.704890439999999785, 5.4013774000000003284, 2.5638155600000001044},
          {7.7043461000000004191, 5.402217689999999628, 2.5976617200000000629},
          {7.7031359500000000651, 5.4077816600000003788, 2.601399929999999916},
          {7.704890439999999785, 5.3964455999999998426, 2.594044949999999794},
          {7.704890439999999785, 5.3783911499999996764, 2.5638155600000001044},
          {7.704890439999999785, 5.3783911499999996764, 2.5845081200000001864},
      },
      Eigen::Isometry3d::Identity(),
      {
          {7.6898293000000004227, 5.3656467599999997375, 2.4919342000000002102},
          {7.6898144400000001397, 5.3656728200000003426, 2.4974946799999999669},
          {7.698255080000000028, 5.3389623500000000789, 2.5587030199999998281},
          {7.698255080000000028, 5.3440264800000001344, 2.5477826900000000165},
          {7.698255080000000028, 5.3833321400000002654, 2.5456903400000001625},
          {7.698255080000000028, 5.3412512000000003098, 2.5531353800000000653},
          {7.698255080000000028, 5.345264280000000312, 2.5456903400000001625},
          {7.6895153099999999924, 5.3833321400000002654, 2.5637977499999999864},
          {7.6953892000000001516, 5.3487214500000002104, 2.5531353800000000653},
          {7.6926626599999998746, 5.3577031100000001018, 2.54200722999999984},
          {7.6899927999999997397, 5.3659638200000001618, 2.5284642999999999979},
          {7.6896286399999995709, 5.3687477799999996364, 2.4909955099999998573},
          {7.691282209999999786, 5.3628758200000001821, 2.4909955099999998573},
          {7.6971644599999997638, 5.3554403199999995877, 2.4909955099999998573},
          {7.6917494900000003, 5.3620940600000004395, 2.4909955099999998573},
          {7.6940635600000000238, 5.3587932499999997304, 2.4909955099999998573},
          {7.6982541500000003509, 5.3543849300000001534, 2.4909955099999998573},
          {7.6982541500000003509, 5.3833321400000002654, 2.4909955099999998573},
          {7.6892570500000001488, 5.3688259599999996752, 2.5086299900000001983},
          {7.6887647000000001185, 5.3833321400000002654, 2.5587030199999998281},
          {7.6899649300000003649, 5.365885640000000123, 2.5253186099999997971},
          {7.6898999000000003434, 5.3787544399999998035, 2.5641990200000002176},
          {7.698255080000000028, 5.3410079799999996553, 2.5615835900000001324},
          {7.698255080000000028, 5.3466280399999996931, 2.563353479999999962},
          {7.690880899999999798, 5.375579580000000135, 2.5641990200000002176},
          {7.6896927399999999153, 5.3796838700000000344, 2.5641990200000002176},
          {7.6954997499999997501, 5.3490341499999995989, 2.5561449199999999315},
          {7.6895970499999997116, 5.3819944399999997131, 2.5641990200000002176},
          {7.698255080000000028, 5.3547627899999996615, 2.5641990200000002176},
          {7.689746620000000199, 5.3833321400000002654, 2.5641990200000002176},
          {7.698255080000000028, 5.3833321400000002654, 2.5641990200000002176},
          {7.6894298399999998495, 5.3688259599999996752, 2.4974946799999999669},
          {7.6896332899999997323, 5.3683004399999996181, 2.4909955099999998573},
          {7.6898385899999999182, 5.365655450000000215, 2.4909955099999998573},
          {7.6893267200000003925, 5.3688259599999996752, 2.503055170000000107},
          {7.6895821900000003168, 5.3688259599999996752, 2.4919342000000002102},
          {7.6901683700000003086, 5.3833321400000002654, 2.4909955099999998573},
          {7.688862239999999737, 5.3833321400000002654, 2.5531353800000000653},
          {7.6898571699999997975, 5.3657206000000003954, 2.5086299900000001983},
          {7.6976187300000002978, 5.3410079799999996553, 2.5587030199999998281},
          {7.689820010000000039, 5.365685850000000201, 2.503055170000000107},
          {7.689893399999999879, 5.3657640300000002398, 2.5141904799999998943},
          {7.6891920200000001273, 5.3688259599999996752, 2.5141904799999998943},
      },
      Eigen::Isometry3d::Identity());
}

TEST(DistanceBoxBox, SignedDistanceWithCommonFrames)
{
  const Eigen::Vector3d halfExtents1(10.0, 20.0, 25.0);
  const Eigen::Vector3d halfExtents2(5.0, 5.0, 5.0);

  const std::array<Eigen::Isometry3d, 2> frames{{
      Eigen::Isometry3d::Identity(),
      makeSignedDistanceFrame(),
  }};

  for (const auto& frame : frames) {
    expectBoxBoxSignedDistanceCase(
        "coincident boxes", halfExtents1, frame, halfExtents2, frame, -15.0);

    for (const auto& offsetAndDistance :
         {std::pair{15.1, 0.1}, std::pair{20.0, 5.0}}) {
      Eigen::Isometry3d translated = Eigen::Isometry3d::Identity();
      translated.translation()
          = Eigen::Vector3d(offsetAndDistance.first, 0.0, 0.0);

      expectBoxBoxSignedDistanceCase(
          "positive gap along common local x axis",
          halfExtents1,
          frame,
          halfExtents2,
          frame * translated,
          offsetAndDistance.second);
      expectBoxBoxSignedDistanceCase(
          "positive gap along common local x axis with argument order swapped",
          halfExtents2,
          frame * translated,
          halfExtents1,
          frame,
          offsetAndDistance.second);
    }
  }
}

TEST(DistanceCapsuleCapsule, Separated)
{
  CapsuleShape c1(0.5, 2.0);
  CapsuleShape c2(0.5, 2.0);

  Eigen::Isometry3d tf1 = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d tf2 = Eigen::Isometry3d::Identity();
  tf2.translation() = Eigen::Vector3d(3.0, 0, 0);

  DistanceResult result;
  double dist = distanceCapsuleCapsule(c1, tf1, c2, tf2, result);

  EXPECT_NEAR(dist, 2.0, 1e-6);
}

TEST(DistanceCapsuleCapsule, Parallel)
{
  CapsuleShape c1(0.5, 2.0);
  CapsuleShape c2(0.5, 2.0);

  Eigen::Isometry3d tf1 = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d tf2 = Eigen::Isometry3d::Identity();
  tf2.translation() = Eigen::Vector3d(2.0, 0, 0);

  DistanceResult result;
  double dist = distanceCapsuleCapsule(c1, tf1, c2, tf2, result);

  EXPECT_NEAR(dist, 1.0, 1e-6);
}

TEST(DistanceCapsuleCapsule, ZeroHeight)
{
  CapsuleShape c1(0.5, 0.0);
  CapsuleShape c2(0.75, 0.0);

  Eigen::Isometry3d tf1 = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d tf2 = Eigen::Isometry3d::Identity();
  tf2.translation() = Eigen::Vector3d(3.0, 0, 0);

  DistanceResult result;
  double dist = distanceCapsuleCapsule(c1, tf1, c2, tf2, result);

  EXPECT_NEAR(dist, 1.75, 1e-6);
}

TEST(DistanceCapsuleCapsule, DegenerateCenterLineEndpointsAndInterior)
{
  constexpr double radius1 = 0.05;
  constexpr double radius2 = 0.10;

  const Eigen::Vector3d pointA(0.25, 0.5, 1.0);
  const Eigen::Vector3d pointB(0.6, 1.0, -0.3);
  expectCapsuleCapsuleCenterLineCase(
      "both center lines have zero length",
      pointA,
      pointA,
      radius1,
      pointB,
      pointB,
      radius2,
      (pointB - pointA).norm() - radius1 - radius2,
      pointA,
      pointB);

  const Eigen::Vector3d tinyOffset(1e-6, 0.0, 0.0);
  expectCapsuleCapsuleCenterLineCase(
      "near-zero center lines use endpoint witnesses",
      pointA,
      pointA + tinyOffset,
      radius1,
      pointB,
      pointB + tinyOffset,
      radius2,
      (pointB - pointA).norm() - radius1 - radius2,
      pointA,
      pointB);

  const Eigen::Vector3d segmentStart(-1.0, -2.0, -3.0);
  const Eigen::Vector3d segmentEnd(1.0, 2.0, 3.0);
  const Eigen::Vector3d endpointNearStart = segmentStart + segmentStart;
  expectCapsuleCapsuleCenterLineCase(
      "point center line nearest first segment endpoint",
      endpointNearStart,
      endpointNearStart,
      radius1,
      segmentStart,
      segmentEnd,
      radius2,
      (endpointNearStart - segmentStart).norm() - radius1 - radius2,
      endpointNearStart,
      segmentStart);

  const Eigen::Vector3d endpointNearEnd = segmentEnd + segmentEnd;
  expectCapsuleCapsuleCenterLineCase(
      "point center line nearest second segment endpoint",
      endpointNearEnd,
      endpointNearEnd,
      radius1,
      segmentStart,
      segmentEnd,
      radius2,
      (endpointNearEnd - segmentEnd).norm() - radius1 - radius2,
      endpointNearEnd,
      segmentEnd);

  const Eigen::Vector3d segmentDirection = segmentEnd - segmentStart;
  const Eigen::Vector3d interiorPoint(
      segmentDirection.y(), -segmentDirection.x(), 0.0);
  expectCapsuleCapsuleCenterLineCase(
      "point center line nearest segment interior",
      interiorPoint,
      interiorPoint,
      radius1,
      segmentStart,
      segmentEnd,
      radius2,
      interiorPoint.norm() - radius1 - radius2,
      interiorPoint,
      Eigen::Vector3d::Zero());

  expectCapsuleCapsuleCenterLineCase(
      "segment interior nearest point center line",
      segmentStart,
      segmentEnd,
      radius1,
      interiorPoint,
      interiorPoint,
      radius2,
      interiorPoint.norm() - radius1 - radius2,
      Eigen::Vector3d::Zero(),
      interiorPoint);
}

TEST(DistanceCapsuleCapsule, ParallelCenterLineWitnessRegions)
{
  constexpr double radius1 = 0.05;
  constexpr double radius2 = 0.05;
  const Eigen::Vector3d direction = Eigen::Vector3d(1.0, 2.0, 3.0).normalized();
  const Eigen::Vector3d perpendicular
      = Eigen::Vector3d(direction.y(), -direction.x(), 0.0).normalized();

  expectCapsuleCapsuleCenterLineCase(
      "co-linear overlapping center lines",
      -direction,
      direction,
      radius1,
      Eigen::Vector3d::Zero(),
      2.0 * direction,
      radius2,
      -radius1 - radius2,
      Eigen::Vector3d::Zero(),
      Eigen::Vector3d::Zero(),
      false);

  const Eigen::Vector3d parallelOffset = perpendicular * 0.5;
  expectCapsuleCapsuleCenterLineCase(
      "parallel lines with overlapping projections",
      -direction,
      direction,
      radius1,
      parallelOffset,
      2.0 * direction + parallelOffset,
      radius2,
      parallelOffset.norm() - radius1 - radius2,
      Eigen::Vector3d::Zero(),
      parallelOffset);

  expectCapsuleCapsuleCenterLineCase(
      "parallel center lines share one endpoint",
      -direction,
      direction,
      radius1,
      direction,
      2.0 * direction,
      radius2,
      -radius1 - radius2,
      direction,
      direction,
      false);

  expectCapsuleCapsuleCenterLineCase(
      "co-linear non-overlapping center lines",
      -direction,
      direction,
      radius1,
      2.0 * direction,
      3.0 * direction,
      radius2,
      direction.norm() - radius1 - radius2,
      direction,
      2.0 * direction);

  const Eigen::Vector3d separatedStart = 2.0 * direction + perpendicular;
  expectCapsuleCapsuleCenterLineCase(
      "parallel non-overlapping center lines with lateral offset",
      -direction,
      direction,
      radius1,
      separatedStart,
      3.0 * direction + perpendicular,
      radius2,
      (separatedStart - direction).norm() - radius1 - radius2,
      direction,
      separatedStart);
}

TEST(DistanceCapsuleCapsule, SkewCenterLineWitnessRegions)
{
  constexpr double radius1 = 0.05;
  constexpr double radius2 = 0.07;
  const Eigen::Vector3d primaryDirection
      = Eigen::Vector3d(1.0, 2.0, 3.0).normalized();
  const Eigen::Vector3d secondaryDirection
      = Eigen::Vector3d(-2.0, 1.0, -0.5).normalized();
  const Eigen::Vector3d perpendicular
      = primaryDirection.cross(secondaryDirection).normalized();

  const Eigen::Vector3d intersection(-0.5, 1.25, 0.75);
  const Eigen::Vector3d startA = intersection + primaryDirection;
  const Eigen::Vector3d endA = intersection - 2.0 * primaryDirection;
  const Eigen::Vector3d startB = intersection + secondaryDirection;
  const Eigen::Vector3d endB = intersection - 2.5 * secondaryDirection;

  expectCapsuleCapsuleCenterLineCase(
      "skew center lines intersect",
      startA,
      endA,
      radius1,
      startB,
      endB,
      radius2,
      -radius1 - radius2,
      intersection,
      intersection,
      false);

  const Eigen::Vector3d interiorOffset = 1.25 * perpendicular;
  expectCapsuleCapsuleCenterLineCase(
      "nearest points are on both segment interiors",
      startA,
      endA,
      radius1,
      startB + interiorOffset,
      endB + interiorOffset,
      radius2,
      interiorOffset.norm() - radius1 - radius2,
      intersection,
      intersection + interiorOffset);

  const Eigen::Vector3d startOffset = 1.1 * (startA - intersection);
  const Eigen::Vector3d shiftedStartB = startB + startOffset;
  const Eigen::Vector3d shiftedEndB = endB + startOffset;
  const Eigen::Vector3d projectedStart
      = projectPointOntoSegment(shiftedStartB, shiftedEndB, startA);
  expectCapsuleCapsuleCenterLineCase(
      "first segment endpoint nearest second segment interior",
      startA,
      endA,
      radius1,
      shiftedStartB,
      shiftedEndB,
      radius2,
      (startA - projectedStart).norm() - radius1 - radius2,
      startA,
      projectedStart);

  const Eigen::Vector3d endOffset = 1.1 * (endA - intersection);
  const Eigen::Vector3d shiftedStartBFromEnd = startB + endOffset;
  const Eigen::Vector3d shiftedEndBFromEnd = endB + endOffset;
  const Eigen::Vector3d projectedEnd
      = projectPointOntoSegment(shiftedStartBFromEnd, shiftedEndBFromEnd, endA);
  expectCapsuleCapsuleCenterLineCase(
      "opposite segment endpoint nearest second segment interior",
      startA,
      endA,
      radius1,
      shiftedStartBFromEnd,
      shiftedEndBFromEnd,
      radius2,
      (endA - projectedEnd).norm() - radius1 - radius2,
      endA,
      projectedEnd);

  const Eigen::Vector3d endpointOffset
      = (startA - intersection).cwiseProduct(Eigen::Vector3d(1.75, 1.5, 1.25));
  const Eigen::Vector3d endpointStartB = startA + endpointOffset;
  const Eigen::Vector3d endpointEndB
      = endpointStartB
        + (endpointOffset.dot(secondaryDirection) > 0.0 ? secondaryDirection
                                                        : -secondaryDirection);
  expectCapsuleCapsuleCenterLineCase(
      "nearest points are both segment endpoints",
      startA,
      endA,
      radius1,
      endpointStartB,
      endpointEndB,
      radius2,
      endpointOffset.norm() - radius1 - radius2,
      startA,
      endpointStartB);
}

TEST(DistanceCapsuleCapsule, TransformedSignedDistanceConfigurations)
{
  const CapsuleShape capsule1(1.5, 2.5);
  const CapsuleShape capsule2(2.0, 3.0);
  const double halfHeight1 = capsule1.getHeight() * 0.5;
  const double halfHeight2 = capsule2.getHeight() * 0.5;
  const double radiusSum = capsule1.getRadius() + capsule2.getRadius();

  const Eigen::Isometry3d worldFromCase
      = makeCapsuleDistanceWorldFromCaseFrame();
  const Eigen::Isometry3d transform1 = worldFromCase;

  const auto makeCrossingTransform = [&](double signedDistance) {
    Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
    tf.linear()
        = Eigen::AngleAxisd(
              std::numbers::pi_v<double> / 2.0, Eigen::Vector3d::UnitY())
              .toRotationMatrix();
    tf.translation()
        = Eigen::Vector3d(0.0, 0.0, halfHeight1 + radiusSum + signedDistance);
    return worldFromCase * tf;
  };

  const Eigen::Vector3d pointOnCapsule1
      = worldFromCase
        * Eigen::Vector3d(0.0, 0.0, halfHeight1 + capsule1.getRadius());

  expectCapsuleCapsulePoseDistance(
      "transformed separated capsules",
      capsule1,
      transform1,
      capsule2,
      makeCrossingTransform(0.25),
      0.25,
      true,
      pointOnCapsule1,
      worldFromCase
          * Eigen::Vector3d(
              0.0, 0.0, halfHeight1 + capsule1.getRadius() + 0.25));

  expectCapsuleCapsulePoseDistance(
      "transformed penetrating capsules",
      capsule1,
      transform1,
      capsule2,
      makeCrossingTransform(-0.25),
      -0.25,
      true,
      pointOnCapsule1,
      worldFromCase
          * Eigen::Vector3d(
              0.0, 0.0, halfHeight1 + capsule1.getRadius() - 0.25));

  Eigen::Isometry3d singleIntersection = Eigen::Isometry3d::Identity();
  singleIntersection.linear()
      = Eigen::AngleAxisd(
            std::numbers::pi_v<double> / 2.0, Eigen::Vector3d::UnitY())
            .toRotationMatrix();
  singleIntersection.translation() = Eigen::Vector3d(0.0, 0.0, halfHeight1 / 2);
  expectCapsuleCapsulePoseDistance(
      "center lines intersect at one point",
      capsule1,
      transform1,
      capsule2,
      worldFromCase * singleIntersection,
      -radiusSum,
      false);

  Eigen::Isometry3d overlappingCenterLines = Eigen::Isometry3d::Identity();
  overlappingCenterLines.translation() = Eigen::Vector3d(0.0, 0.0, halfHeight2);
  expectCapsuleCapsulePoseDistance(
      "center lines overlap over an interval",
      capsule1,
      transform1,
      capsule2,
      worldFromCase * overlappingCenterLines,
      -radiusSum,
      false);
}

TEST(DistanceCapsuleSphere, Separated)
{
  CapsuleShape capsule(0.5, 2.0);
  SphereShape sphere(0.5);

  Eigen::Isometry3d tfCapsule = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d tfSphere = Eigen::Isometry3d::Identity();
  tfSphere.translation() = Eigen::Vector3d(3.0, 0, 0);

  DistanceResult result;
  double dist
      = distanceCapsuleSphere(capsule, tfCapsule, sphere, tfSphere, result);

  EXPECT_NEAR(dist, 2.0, 1e-6);
}

TEST(DistanceCapsuleSphere, LargeSphereSignedDistanceAlongCapsuleSide)
{
  CapsuleShape capsule(10.0, 20.0);
  SphereShape sphere(20.0);

  Eigen::Isometry3d tfCapsule = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d tfSphere = Eigen::Isometry3d::Identity();

  struct Case
  {
    const char* name;
    double capsuleOffset;
    double expectedDistance;
  };

  const std::array<Case, 2> cases{{
      {"side-separated", 40.0, 10.0},
      {"side-penetrating", 25.0, -5.0},
  }};

  for (const auto& testCase : cases) {
    SCOPED_TRACE(testCase.name);
    tfCapsule.translation() = Eigen::Vector3d(testCase.capsuleOffset, 0.0, 0.0);

    DistanceResult result;
    const double dist
        = distanceCapsuleSphere(capsule, tfCapsule, sphere, tfSphere, result);

    EXPECT_NEAR(dist, testCase.expectedDistance, 1e-9);
    EXPECT_NEAR(result.distance, testCase.expectedDistance, 1e-9);
    EXPECT_NEAR(
        (result.pointOnObject2 - result.pointOnObject1).norm(),
        std::abs(testCase.expectedDistance),
        1e-9);
    EXPECT_TRUE(result.pointOnObject1.allFinite());
    EXPECT_TRUE(result.pointOnObject2.allFinite());
  }
}

TEST(DistanceCapsuleSphere, LargeSphereSignedDistanceAlongCapsuleAxis)
{
  CapsuleShape capsule(50.0, 200.0);
  SphereShape sphere(50.0);

  struct Case
  {
    const char* name;
    Eigen::Vector3d sphereCenterInCapsuleFrame;
    double expectedDistance;
  };

  const std::array<Case, 4> cases{{
      {"bottom-endcap-separated", Eigen::Vector3d(0.0, 0.0, -225.0), 25.0},
      {"bottom-endcap-penetrating", Eigen::Vector3d(0.0, 0.0, -150.0), -50.0},
      {"top-endcap-separated", Eigen::Vector3d(0.0, 0.0, 225.0), 25.0},
      {"top-endcap-penetrating", Eigen::Vector3d(0.0, 0.0, 150.0), -50.0},
  }};

  const std::array<Eigen::Isometry3d, 2> capsuleTransforms{{
      Eigen::Isometry3d::Identity(),
      makeDistanceBoxTransform(
          Eigen::Vector3d(1.3, 2.7, 6.5),
          std::numbers::pi_v<double> / 2.0,
          Eigen::Vector3d::UnitX()),
  }};

  for (const auto& testCase : cases) {
    for (const auto& tfCapsule : capsuleTransforms) {
      SCOPED_TRACE(testCase.name);
      const Eigen::Isometry3d tfSphere
          = tfCapsule
            * makeDistanceBoxTransform(testCase.sphereCenterInCapsuleFrame);

      DistanceResult result;
      const double dist
          = distanceCapsuleSphere(capsule, tfCapsule, sphere, tfSphere, result);

      EXPECT_NEAR(dist, testCase.expectedDistance, 1e-9);
      EXPECT_NEAR(result.distance, testCase.expectedDistance, 1e-9);
      EXPECT_NEAR(
          (result.pointOnObject2 - result.pointOnObject1).norm(),
          std::abs(testCase.expectedDistance),
          1e-9);
      EXPECT_TRUE(result.pointOnObject1.allFinite());
      EXPECT_TRUE(result.pointOnObject2.allFinite());
      EXPECT_TRUE(result.normal.allFinite());
    }
  }
}

TEST(DistanceCapsuleSphere, ZeroHeight)
{
  CapsuleShape capsule(0.5, 0.0);
  SphereShape sphere(0.25);

  Eigen::Isometry3d tfCapsule = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d tfSphere = Eigen::Isometry3d::Identity();
  tfSphere.translation() = Eigen::Vector3d(2.0, 0, 0);

  DistanceResult result;
  double dist
      = distanceCapsuleSphere(capsule, tfCapsule, sphere, tfSphere, result);

  EXPECT_NEAR(dist, 1.25, 1e-6);
}

TEST(DistanceCapsuleBox, Separated)
{
  CapsuleShape capsule(0.5, 2.0);
  BoxShape box(Eigen::Vector3d(0.5, 0.5, 0.5));

  Eigen::Isometry3d tfCapsule = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d tfBox = Eigen::Isometry3d::Identity();
  tfBox.translation() = Eigen::Vector3d(3.0, 0, 0);

  DistanceResult result;
  double dist = distanceCapsuleBox(capsule, tfCapsule, box, tfBox, result);

  EXPECT_NEAR(dist, 2.0, 1e-6);
}

TEST(DistanceCapsuleBox, ThinSlabSeparated)
{
  CapsuleShape capsule(0.5, 2.0);
  BoxShape box(Eigen::Vector3d(1.0, 1.0, 1e-3));

  Eigen::Isometry3d tfCapsule = Eigen::Isometry3d::Identity();
  tfCapsule.translation() = Eigen::Vector3d(0, 0, 2.0);
  Eigen::Isometry3d tfBox = Eigen::Isometry3d::Identity();

  DistanceResult result;
  double dist = distanceCapsuleBox(capsule, tfCapsule, box, tfBox, result);

  EXPECT_NEAR(dist, 0.499, 1e-6);
}

TEST(DistanceCapsuleBox, AxisAndEndpointWitnessCases)
{
  CapsuleShape capsule(2.0, 4.0);
  BoxShape box(Eigen::Vector3d(0.5, 1.0, 2.0));

  struct Case
  {
    const char* name;
    Eigen::Isometry3d capsuleTransform;
    double expectedDistance;
    Eigen::Vector3d expectedPointOnCapsule;
    Eigen::Vector3d expectedPointOnBox;
    bool uniqueWitness;
  };

  Eigen::Isometry3d sideTransform = Eigen::Isometry3d::Identity();
  sideTransform.translation() = Eigen::Vector3d(3.0, 0.0, 0.0);

  Eigen::Isometry3d aboveTransform = Eigen::Isometry3d::Identity();
  aboveTransform.translation() = Eigen::Vector3d(0.0, 0.0, 8.0);

  Eigen::Isometry3d horizontalBehindTransform = Eigen::Isometry3d::Identity();
  horizontalBehindTransform.translation() = Eigen::Vector3d(-10.0, 0.0, 0.0);
  horizontalBehindTransform.linear()
      = Eigen::AngleAxisd(
            std::numbers::pi_v<double> / 2.0, Eigen::Vector3d::UnitY())
            .toRotationMatrix();

  Eigen::Isometry3d offsetHorizontalBehindTransform = horizontalBehindTransform;
  offsetHorizontalBehindTransform.translation()
      = Eigen::Vector3d(-10.0, 0.8, 1.5);

  const std::array<Case, 4> cases{{
      {"side separation",
       sideTransform,
       0.5,
       Eigen::Vector3d(1.0, 0.0, 0.0),
       Eigen::Vector3d(0.5, 0.0, 0.0),
       false},
      {"above top face",
       aboveTransform,
       2.0,
       Eigen::Vector3d(0.0, 0.0, 4.0),
       Eigen::Vector3d(0.0, 0.0, 2.0),
       true},
      {"horizontal endpoint behind box",
       horizontalBehindTransform,
       5.5,
       Eigen::Vector3d(-6.0, 0.0, 0.0),
       Eigen::Vector3d(-0.5, 0.0, 0.0),
       true},
      {"offset horizontal endpoint behind box",
       offsetHorizontalBehindTransform,
       5.5,
       Eigen::Vector3d(-6.0, 0.8, 1.5),
       Eigen::Vector3d(-0.5, 0.8, 1.5),
       true},
  }};

  const Eigen::Isometry3d boxTransform = Eigen::Isometry3d::Identity();
  for (const auto& testCase : cases) {
    SCOPED_TRACE(testCase.name);
    DistanceResult result;
    const double distance = distanceCapsuleBox(
        capsule, testCase.capsuleTransform, box, boxTransform, result);

    EXPECT_NEAR(distance, testCase.expectedDistance, 1e-9);
    EXPECT_NEAR(result.distance, testCase.expectedDistance, 1e-9);
    if (testCase.uniqueWitness) {
      expectVectorNear(
          result.pointOnObject1, testCase.expectedPointOnCapsule, 1e-9);
      expectVectorNear(
          result.pointOnObject2, testCase.expectedPointOnBox, 1e-9);
    } else {
      EXPECT_NEAR(
          result.pointOnObject1.x(), testCase.expectedPointOnCapsule.x(), 1e-9);
      EXPECT_NEAR(
          result.pointOnObject1.y(), testCase.expectedPointOnCapsule.y(), 1e-9);
      EXPECT_NEAR(
          result.pointOnObject2.x(), testCase.expectedPointOnBox.x(), 1e-9);
      EXPECT_NEAR(
          result.pointOnObject2.y(), testCase.expectedPointOnBox.y(), 1e-9);
      EXPECT_NEAR(result.pointOnObject1.z(), result.pointOnObject2.z(), 1e-9);
      EXPECT_LE(std::abs(result.pointOnObject1.z()), 2.0 + 1e-9);
    }
  }
}

TEST(DistancePlaneShape, SphereAbovePlane)
{
  PlaneShape plane(Eigen::Vector3d::UnitZ(), 0.0);
  SphereShape sphere(1.0);

  Eigen::Isometry3d tfPlane = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d tfSphere = Eigen::Isometry3d::Identity();
  tfSphere.translation() = Eigen::Vector3d(0, 0, 3.0);

  DistanceResult result;
  double dist = distancePlaneShape(plane, tfPlane, sphere, tfSphere, result);

  EXPECT_NEAR(dist, 2.0, 1e-6);
  EXPECT_NEAR(result.pointOnObject1.z(), 0.0, 1e-6);
  EXPECT_NEAR(result.pointOnObject2.z(), 2.0, 1e-6);
  EXPECT_NEAR(result.normal.z(), 1.0, 1e-6);
}

TEST(DistancePlaneShape, SpherePenetratingPlane)
{
  PlaneShape plane(Eigen::Vector3d::UnitZ(), 0.0);
  SphereShape sphere(1.0);

  Eigen::Isometry3d tfPlane = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d tfSphere = Eigen::Isometry3d::Identity();
  tfSphere.translation() = Eigen::Vector3d(0, 0, 0.5);

  DistanceResult result;
  double dist = distancePlaneShape(plane, tfPlane, sphere, tfSphere, result);

  EXPECT_NEAR(dist, -0.5, 1e-6);
  EXPECT_NEAR(result.pointOnObject1.z(), 0.0, 1e-6);
  EXPECT_NEAR(result.pointOnObject2.z(), -0.5, 1e-6);
  EXPECT_NEAR(result.normal.z(), -1.0, 1e-6);
}

TEST(DistancePlaneShape, BoxAbovePlane)
{
  PlaneShape plane(Eigen::Vector3d::UnitZ(), 0.0);
  BoxShape box(Eigen::Vector3d(1.0, 1.0, 1.0));

  Eigen::Isometry3d tfPlane = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d tfBox = Eigen::Isometry3d::Identity();
  tfBox.translation() = Eigen::Vector3d(0, 0, 1.5);

  DistanceResult result;
  double dist = distancePlaneShape(plane, tfPlane, box, tfBox, result);

  EXPECT_NEAR(dist, 0.5, 1e-6);
  EXPECT_NEAR(result.pointOnObject1.z(), 0.0, 1e-6);
  EXPECT_NEAR(result.pointOnObject2.z(), 0.5, 1e-6);
  EXPECT_NEAR(result.normal.z(), 1.0, 1e-6);
}

TEST(DistancePlaneShape, CapsuleAbovePlane)
{
  PlaneShape plane(Eigen::Vector3d::UnitZ(), 0.0);
  CapsuleShape capsule(0.5, 2.0);

  Eigen::Isometry3d tfPlane = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d tfCapsule = Eigen::Isometry3d::Identity();
  tfCapsule.translation() = Eigen::Vector3d(0, 0, 2.5);

  DistanceResult result;
  double dist = distancePlaneShape(plane, tfPlane, capsule, tfCapsule, result);

  EXPECT_NEAR(dist, 1.0, 1e-6);
  EXPECT_NEAR(result.pointOnObject1.z(), 0.0, 1e-6);
  EXPECT_NEAR(result.pointOnObject2.z(), 1.0, 1e-6);
  EXPECT_NEAR(result.normal.z(), 1.0, 1e-6);
}

TEST(DistancePlaneShape, CylinderAbovePlane)
{
  PlaneShape plane(Eigen::Vector3d::UnitZ(), 0.0);
  CylinderShape cylinder(0.5, 2.0);

  Eigen::Isometry3d tfPlane = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d tfCylinder = Eigen::Isometry3d::Identity();
  tfCylinder.translation() = Eigen::Vector3d(0, 0, 2.0);

  DistanceResult result;
  double dist
      = distancePlaneShape(plane, tfPlane, cylinder, tfCylinder, result);

  EXPECT_NEAR(dist, 1.0, 1e-6);
  EXPECT_NEAR(result.pointOnObject1.z(), 0.0, 1e-6);
  EXPECT_NEAR(result.pointOnObject2.z(), 1.0, 1e-6);
  EXPECT_NEAR(result.normal.z(), 1.0, 1e-6);
}

TEST(NarrowPhaseDistance, IsDistanceSupported)
{
  EXPECT_TRUE(
      NarrowPhase::isDistanceSupported(ShapeType::Sphere, ShapeType::Sphere));
  EXPECT_TRUE(NarrowPhase::isDistanceSupported(ShapeType::Box, ShapeType::Box));
  EXPECT_TRUE(
      NarrowPhase::isDistanceSupported(ShapeType::Sphere, ShapeType::Box));
  EXPECT_TRUE(
      NarrowPhase::isDistanceSupported(ShapeType::Box, ShapeType::Sphere));
  EXPECT_TRUE(
      NarrowPhase::isDistanceSupported(ShapeType::Capsule, ShapeType::Capsule));
  EXPECT_TRUE(
      NarrowPhase::isDistanceSupported(ShapeType::Capsule, ShapeType::Sphere));
  EXPECT_TRUE(
      NarrowPhase::isDistanceSupported(ShapeType::Sphere, ShapeType::Capsule));
  EXPECT_TRUE(
      NarrowPhase::isDistanceSupported(ShapeType::Capsule, ShapeType::Box));
  EXPECT_TRUE(
      NarrowPhase::isDistanceSupported(ShapeType::Box, ShapeType::Capsule));
  EXPECT_TRUE(
      NarrowPhase::isDistanceSupported(ShapeType::Plane, ShapeType::Sphere));
  EXPECT_TRUE(
      NarrowPhase::isDistanceSupported(ShapeType::Cylinder, ShapeType::Box));
}

TEST(NarrowPhaseDistance, SphereSphere)
{
  CollisionWorld world;
  auto obj1 = world.createObject(std::make_unique<SphereShape>(1.0));

  Eigen::Isometry3d tf2 = Eigen::Isometry3d::Identity();
  tf2.translation() = Eigen::Vector3d(5.0, 0, 0);
  auto obj2 = world.createObject(std::make_unique<SphereShape>(1.0), tf2);

  DistanceOption option;
  DistanceResult result;

  double dist = NarrowPhase::distance(obj1, obj2, option, result);

  EXPECT_NEAR(dist, 3.0, 1e-6);
}

TEST(NarrowPhaseDistance, BoxSphere)
{
  CollisionWorld world;
  auto obj1 = world.createObject(
      std::make_unique<BoxShape>(Eigen::Vector3d(0.5, 0.5, 0.5)));

  Eigen::Isometry3d tf2 = Eigen::Isometry3d::Identity();
  tf2.translation() = Eigen::Vector3d(3.0, 0, 0);
  auto obj2 = world.createObject(std::make_unique<SphereShape>(1.0), tf2);

  DistanceOption option;
  DistanceResult result;

  double dist = NarrowPhase::distance(obj1, obj2, option, result);

  EXPECT_NEAR(dist, 1.5, 1e-6);
}

TEST(NarrowPhaseDistance, CylinderBoxThinHeight)
{
  CollisionWorld world;

  Eigen::Isometry3d tfCylinder = Eigen::Isometry3d::Identity();
  tfCylinder.translation() = Eigen::Vector3d(2.0, 0, 0);
  auto obj1 = world.createObject(
      std::make_unique<CylinderShape>(0.5, 1e-3), tfCylinder);

  auto obj2 = world.createObject(
      std::make_unique<BoxShape>(Eigen::Vector3d(1.0, 1.0, 1.0)));

  DistanceOption option;
  DistanceResult result;

  double dist = NarrowPhase::distance(obj1, obj2, option, result);

  EXPECT_NEAR(dist, 0.5, 1e-5);
}

TEST(NarrowPhaseDistance, CylinderSphereSeparated)
{
  CollisionWorld world;
  auto obj1 = world.createObject(std::make_unique<CylinderShape>(1.0, 2.0));

  Eigen::Isometry3d tfSphere = Eigen::Isometry3d::Identity();
  tfSphere.translation() = Eigen::Vector3d(3.0, 0, 0);
  auto obj2 = world.createObject(std::make_unique<SphereShape>(0.5), tfSphere);

  DistanceOption option;
  DistanceResult result;

  double dist = NarrowPhase::distance(obj1, obj2, option, result);

  EXPECT_NEAR(dist, 1.5, 1e-6);
}

TEST(NarrowPhaseDistance, CylinderSphereSeparatedAcrossFramesAndOrientations)
{
  constexpr double cylinderRadius = 1.2;
  constexpr double cylinderHeight = 0.6;
  constexpr double cylinderHalfHeight = cylinderHeight * 0.5;
  constexpr double sphereRadius = 0.7;
  constexpr double targetDistance = 0.1;

  const Eigen::Vector3d barrelNormalToCylinder
      = Eigen::Vector3d(-1.0, -1.0, 0.0).normalized();
  const Eigen::Vector3d edgeNormalToCylinder
      = Eigen::Vector3d(-1.0, -1.0, -1.0).normalized();
  const Eigen::Vector3d edgePoint
      = Eigen::Vector3d(0.0, 0.0, cylinderHalfHeight)
        + Eigen::Vector3d(
              -edgeNormalToCylinder.x(), -edgeNormalToCylinder.y(), 0.0)
                  .normalized()
              * cylinderRadius;

  struct Case
  {
    const char* name;
    Eigen::Vector3d sphereCenter;
    double expectedDistance;
    Eigen::Vector3d expectedPointOnCylinder;
    Eigen::Vector3d expectedPointOnSphere;
  };

  const Eigen::Vector3d faceCenter(
      cylinderRadius * 0.25,
      cylinderRadius * 0.25,
      cylinderHalfHeight + sphereRadius + targetDistance);
  const Eigen::Vector3d barrelCenter
      = Eigen::Vector3d(0.0, 0.0, cylinderHalfHeight * 0.5)
        - barrelNormalToCylinder
              * (sphereRadius + cylinderRadius + targetDistance);
  const Eigen::Vector3d edgeCenter
      = edgePoint - edgeNormalToCylinder * (sphereRadius + targetDistance);

  const std::array<Case, 3> cases{{
      {"separated from cap face",
       faceCenter,
       targetDistance,
       Eigen::Vector3d(faceCenter.x(), faceCenter.y(), cylinderHalfHeight),
       faceCenter - Eigen::Vector3d::UnitZ() * sphereRadius},
      {"separated from barrel",
       barrelCenter,
       targetDistance,
       Eigen::Vector3d(
           -barrelNormalToCylinder.x() * cylinderRadius,
           -barrelNormalToCylinder.y() * cylinderRadius,
           barrelCenter.z()),
       barrelCenter + barrelNormalToCylinder * sphereRadius},
      {"separated from barrel edge",
       edgeCenter,
       targetDistance,
       edgePoint,
       edgeCenter + edgeNormalToCylinder * sphereRadius},
  }};

  const std::array<Eigen::Isometry3d, 4> cylinderTransforms{{
      Eigen::Isometry3d::Identity(),
      makeDistanceBoxTransform(Eigen::Vector3d(1.3, 2.7, 6.5)),
      makeDistanceBoxTransform(
          Eigen::Vector3d(1.3, 2.7, 6.5),
          std::numbers::pi_v<double> / 2.0,
          Eigen::Vector3d::UnitX()),
      makeDistanceBoxTransform(
          Eigen::Vector3d(1.3, 2.7, 6.5),
          std::numbers::pi_v<double> / 3.0,
          Eigen::Vector3d(1.0, 2.0, 3.0)),
  }};
  const std::array<SphereBoxSphereOrientation, 3> sphereOrientations{{
      {0.0, Eigen::Vector3d::UnitX()},
      {std::numbers::pi_v<double> / 2.0, Eigen::Vector3d::UnitY()},
      {std::numbers::pi_v<double> / 3.0, Eigen::Vector3d(1.0, 2.0, 3.0)},
  }};

  for (const auto& testCase : cases) {
    for (const auto& cylinderTransform : cylinderTransforms) {
      for (const auto& sphereOrientation : sphereOrientations) {
        SCOPED_TRACE(testCase.name);
        CollisionWorld world;
        auto cylinder = world.createObject(
            std::make_unique<CylinderShape>(cylinderRadius, cylinderHeight),
            cylinderTransform);
        Eigen::Isometry3d sphereTransform = Eigen::Isometry3d::Identity();
        sphereTransform.translation() = testCase.sphereCenter;
        sphereTransform.linear()
            = Eigen::AngleAxisd(
                  sphereOrientation.angle, sphereOrientation.axis.normalized())
                  .toRotationMatrix();
        auto sphere = world.createObject(
            std::make_unique<SphereShape>(sphereRadius),
            cylinderTransform * sphereTransform);

        DistanceResult result;
        const double distance = NarrowPhase::distance(
            cylinder, sphere, DistanceOption::unlimited(), result);

        EXPECT_NEAR(distance, testCase.expectedDistance, 1e-5);
        EXPECT_NEAR(result.distance, testCase.expectedDistance, 1e-5);
        expectVectorNear(
            result.pointOnObject1,
            cylinderTransform * testCase.expectedPointOnCylinder,
            5e-3);
        expectVectorNear(
            result.pointOnObject2,
            cylinderTransform * testCase.expectedPointOnSphere,
            5e-3);
      }
    }
  }
}

TEST(NarrowPhaseDistance, CylinderSphereIncompatibleScaleSeparated)
{
  const Eigen::Vector3d obliqueRadial
      = Eigen::Vector3d(1.0, 2.0, 0.0).normalized();

  struct Case
  {
    const char* name;
    double cylinderRadius;
    double cylinderHeight;
    double sphereRadius;
    Eigen::Vector3d sphereCenter;
    double expectedDistance;
    Eigen::Vector3d expectedPointOnCylinder;
    Eigen::Vector3d expectedPointOnSphere;
  };

  const double largeDiskRadius = 9.0;
  const double largeDiskHeight = 0.1;
  const double tinySphereRadius = 0.025;
  const double largeDiskTargetDistance = tinySphereRadius * 0.5;
  const Eigen::Vector3d largeDiskFacePoint
      = obliqueRadial * (largeDiskRadius - tinySphereRadius)
        + Eigen::Vector3d::UnitZ() * (largeDiskHeight * 0.5);
  const Eigen::Vector3d largeDiskBarrelPoint
      = obliqueRadial * largeDiskRadius
        + Eigen::Vector3d::UnitZ() * (tinySphereRadius * 0.1);

  const double tinyDiskRadius = 0.025;
  const double tinyDiskHeight = 0.1;
  const double largeSphereRadius = 9.0;
  const double tinyDiskTargetDistance = tinyDiskRadius * 0.5;
  const Eigen::Vector3d tinyDiskFacePoint
      = obliqueRadial * (tinyDiskRadius * 0.5)
        + Eigen::Vector3d::UnitZ() * (tinyDiskHeight * 0.5);
  const Eigen::Vector3d tinyDiskBarrelPoint
      = obliqueRadial * tinyDiskRadius
        + Eigen::Vector3d::UnitZ() * (tinyDiskHeight * 0.1);

  const std::array<Case, 4> cases{{
      {"large disk and tiny sphere separated from face",
       largeDiskRadius,
       largeDiskHeight,
       tinySphereRadius,
       largeDiskFacePoint
           + Eigen::Vector3d::UnitZ()
                 * (tinySphereRadius + largeDiskTargetDistance),
       largeDiskTargetDistance,
       largeDiskFacePoint,
       largeDiskFacePoint + Eigen::Vector3d::UnitZ() * largeDiskTargetDistance},
      {"large disk and tiny sphere separated from barrel",
       largeDiskRadius,
       largeDiskHeight,
       tinySphereRadius,
       largeDiskBarrelPoint
           + obliqueRadial * (tinySphereRadius + largeDiskTargetDistance),
       largeDiskTargetDistance,
       largeDiskBarrelPoint,
       largeDiskBarrelPoint + obliqueRadial * largeDiskTargetDistance},
      {"large sphere and tiny disk separated from face",
       tinyDiskRadius,
       tinyDiskHeight,
       largeSphereRadius,
       tinyDiskFacePoint
           + Eigen::Vector3d::UnitZ()
                 * (largeSphereRadius + tinyDiskTargetDistance),
       tinyDiskTargetDistance,
       tinyDiskFacePoint,
       tinyDiskFacePoint + Eigen::Vector3d::UnitZ() * tinyDiskTargetDistance},
      {"large sphere and tiny disk separated from barrel",
       tinyDiskRadius,
       tinyDiskHeight,
       largeSphereRadius,
       tinyDiskBarrelPoint
           + obliqueRadial * (largeSphereRadius + tinyDiskTargetDistance),
       tinyDiskTargetDistance,
       tinyDiskBarrelPoint,
       tinyDiskBarrelPoint + obliqueRadial * tinyDiskTargetDistance},
  }};

  const std::array<Eigen::Isometry3d, 2> cylinderTransforms{{
      Eigen::Isometry3d::Identity(),
      makeDistanceBoxTransform(
          Eigen::Vector3d(1.3, 2.7, 6.5),
          std::numbers::pi_v<double> / 3.0,
          Eigen::Vector3d(1.0, 2.0, 3.0)),
  }};

  for (const auto& testCase : cases) {
    for (const auto& cylinderTransform : cylinderTransforms) {
      SCOPED_TRACE(testCase.name);
      CollisionWorld world;
      auto cylinder = world.createObject(
          std::make_unique<CylinderShape>(
              testCase.cylinderRadius, testCase.cylinderHeight),
          cylinderTransform);
      Eigen::Isometry3d sphereTransform = Eigen::Isometry3d::Identity();
      sphereTransform.translation() = testCase.sphereCenter;
      auto sphere = world.createObject(
          std::make_unique<SphereShape>(testCase.sphereRadius),
          cylinderTransform * sphereTransform);

      DistanceResult result;
      const double distance = NarrowPhase::distance(
          cylinder, sphere, DistanceOption::unlimited(), result);

      EXPECT_NEAR(distance, testCase.expectedDistance, 5e-4);
      EXPECT_NEAR(result.distance, testCase.expectedDistance, 5e-4);
      expectVectorNear(
          result.pointOnObject1,
          cylinderTransform * testCase.expectedPointOnCylinder,
          5e-3);
      expectVectorNear(
          result.pointOnObject2,
          cylinderTransform * testCase.expectedPointOnSphere,
          5e-3);
    }
  }
}

TEST(NarrowPhaseDistance, CylinderSphereEdgeWitnessRegression)
{
  constexpr double cylinderRadius = 0.03;
  constexpr double cylinderHeight = 0.65;
  constexpr double sphereRadius = 0.055;

  Eigen::Isometry3d cylinderTransform = Eigen::Isometry3d::Identity();
  cylinderTransform.translation() = Eigen::Vector3d(0.6, 0.0, 0.325);

  const Eigen::Isometry3d sphereTransform = makeDistanceTransformFromMatrix(
      -0.9954758066,
      -0.0295866301,
      0.0902914702,
      0.5419794018,
      -0.0851034786,
      -0.1449450565,
      -0.9857729599,
      -0.0621175025,
      0.0422530022,
      -0.9889972506,
      0.1417713719,
      0.6016236576);

  CollisionWorld world;
  auto cylinder = world.createObject(
      std::make_unique<CylinderShape>(cylinderRadius, cylinderHeight),
      cylinderTransform);
  auto sphere = world.createObject(
      std::make_unique<SphereShape>(sphereRadius), sphereTransform);

  DistanceResult result;
  const double distance = NarrowPhase::distance(
      cylinder, sphere, DistanceOption::unlimited(), result);

  EXPECT_NEAR(distance, 0.0, 1e-4);
  EXPECT_NEAR(result.distance, distance, 1e-9);
  EXPECT_NEAR(
      (result.pointOnObject1 - result.pointOnObject2).norm(),
      std::abs(distance),
      1e-5);
  expectPointInsideCylinder(
      result.pointOnObject1,
      cylinderTransform,
      cylinderRadius,
      cylinderHeight,
      1e-5);
  EXPECT_LE(
      (sphereTransform.inverse() * result.pointOnObject2).norm(),
      sphereRadius + 1e-5);
}

TEST(NarrowPhaseDistance, CylinderBoxRegressionWitnessesInsideGeometry)
{
  constexpr double cylinderRadius = 0.05;
  constexpr double cylinderHeight = 0.06;
  const Eigen::Vector3d boxHalfExtents(0.0125, 0.175, 0.9225);

  const std::array<Eigen::Isometry3d, 2> cylinderTransforms{{
      makeDistanceTransformFromMatrix(
          -0.99999999997999022838,
          6.2572835802045040178e-10,
          6.3260669852976095481e-06,
          0.57500009756757608503,
          6.3260669851683709551e-06,
          -6.3943303429958554955e-10,
          0.99999999997999056145,
          -0.42711963046787942977,
          6.2573180158128459924e-10,
          1.0,
          6.3942912945996747041e-10,
          1.1867093358746836351),
      makeDistanceTransformFromMatrix(
          -0.97313010759279283679,
          -0.12202804064972551379,
          0.19526123781136842106,
          0.87472781461138560122,
          0.20950801135757171623,
          -0.11745920593569325607,
          0.97072639199619581429,
          -0.4038687881347159947,
          -0.095520609678929752073,
          0.98555187191549953329,
          0.13986894183635001365,
          1.5871328698116491385),
  }};
  const Eigen::Isometry3d boxTransform = makeDistanceTransformFromMatrix(
      0.0, -1.0, 0.0, 0.8, 1.0, 0.0, 0.0, -0.4575, 0.0, 0.0, 1.0, 1.0225);

  for (const auto& cylinderTransform : cylinderTransforms) {
    CollisionWorld world;
    auto cylinder = world.createObject(
        std::make_unique<CylinderShape>(cylinderRadius, cylinderHeight),
        cylinderTransform);
    auto box = world.createObject(
        std::make_unique<BoxShape>(boxHalfExtents), boxTransform);

    DistanceResult result;
    const double distance = NarrowPhase::distance(
        cylinder, box, DistanceOption::unlimited(), result);

    EXPECT_TRUE(std::isfinite(distance));
    EXPECT_NEAR(
        (result.pointOnObject1 - result.pointOnObject2).norm(),
        std::abs(distance),
        5e-3);
    expectPointInsideCylinder(
        result.pointOnObject1,
        cylinderTransform,
        cylinderRadius,
        cylinderHeight,
        5e-3);
    expectPointInsideBox(
        result.pointOnObject2, boxTransform, boxHalfExtents, 5e-3);
  }
}

TEST(NarrowPhaseDistance, CylinderCylinderThinHeight)
{
  CollisionWorld world;
  auto obj1 = world.createObject(std::make_unique<CylinderShape>(0.5, 1e-3));

  Eigen::Isometry3d tf2 = Eigen::Isometry3d::Identity();
  tf2.translation() = Eigen::Vector3d(2.0, 0, 0);
  auto obj2
      = world.createObject(std::make_unique<CylinderShape>(0.75, 1e-3), tf2);

  DistanceOption option;
  DistanceResult result;

  double dist = NarrowPhase::distance(obj1, obj2, option, result);

  EXPECT_NEAR(dist, 0.75, 1e-5);
}

std::vector<Eigen::Vector3d> makeOctahedronVertices(double scale = 1.0)
{
  return {
      {scale, 0, 0},
      {-scale, 0, 0},
      {0, scale, 0},
      {0, -scale, 0},
      {0, 0, scale},
      {0, 0, -scale}};
}

TEST(NarrowPhaseDistance, ConvexConvexSeparated)
{
  CollisionWorld world;

  auto obj1 = world.createObject(
      std::make_unique<ConvexShape>(makeOctahedronVertices(1.0)));

  Eigen::Isometry3d tf2 = Eigen::Isometry3d::Identity();
  tf2.translation() = Eigen::Vector3d(5.0, 0, 0);
  auto obj2 = world.createObject(
      std::make_unique<ConvexShape>(makeOctahedronVertices(1.0)), tf2);

  DistanceOption option;
  DistanceResult result;

  double dist = NarrowPhase::distance(obj1, obj2, option, result);

  EXPECT_GT(dist, 2.9);
  EXPECT_LT(dist, 3.1);
}

TEST(NarrowPhaseDistance, ConvexSphereSeparated)
{
  CollisionWorld world;

  auto obj1 = world.createObject(
      std::make_unique<ConvexShape>(makeOctahedronVertices(1.0)));

  Eigen::Isometry3d tf2 = Eigen::Isometry3d::Identity();
  tf2.translation() = Eigen::Vector3d(5.0, 0, 0);
  auto obj2 = world.createObject(std::make_unique<SphereShape>(1.0), tf2);

  DistanceOption option;
  DistanceResult result;

  double dist = NarrowPhase::distance(obj1, obj2, option, result);

  EXPECT_GT(dist, 2.9);
  EXPECT_LT(dist, 3.1);
}

TEST(NarrowPhaseDistance, ConvexIntersecting)
{
  CollisionWorld world;

  auto obj1 = world.createObject(
      std::make_unique<ConvexShape>(makeOctahedronVertices(1.0)));

  // Place octahedron 2 at (1,0,0). Its leftmost vertex will be at origin,
  // which is inside octahedron 1. The shapes clearly intersect.
  Eigen::Isometry3d tf2 = Eigen::Isometry3d::Identity();
  tf2.translation() = Eigen::Vector3d(1.0, 0, 0);
  auto obj2 = world.createObject(
      std::make_unique<ConvexShape>(makeOctahedronVertices(1.0)), tf2);

  DistanceOption option;
  DistanceResult result;

  double dist = NarrowPhase::distance(obj1, obj2, option, result);

  // For intersecting shapes, distance is negative (penetration depth).
  // The vertex (0,0,0) of octahedron 2 is inside octahedron 1.
  // Octahedron 1's faces have equation |x|+|y|+|z|=1, so the penetration
  // depth is approximately 1.0.
  EXPECT_LT(dist, 0.0);  // Negative distance = penetration
  EXPECT_GT(dist, -2.0); // Reasonable bound on penetration depth
}

TEST(NarrowPhaseDistance, ConvexTouching)
{
  CollisionWorld world;

  auto obj1 = world.createObject(
      std::make_unique<ConvexShape>(makeOctahedronVertices(1.0)));

  // Place octahedron 2 at (2,0,0) so its leftmost vertex at (1,0,0)
  // exactly touches octahedron 1's rightmost vertex at (1,0,0).
  Eigen::Isometry3d tf2 = Eigen::Isometry3d::Identity();
  tf2.translation() = Eigen::Vector3d(2.0, 0, 0);
  auto obj2 = world.createObject(
      std::make_unique<ConvexShape>(makeOctahedronVertices(1.0)), tf2);

  DistanceOption option;
  DistanceResult result;

  double dist = NarrowPhase::distance(obj1, obj2, option, result);

  // Shapes just touch at a single point - distance should be ~0
  EXPECT_NEAR(dist, 0.0, 1e-6);
}

TEST(NarrowPhaseDistance, IsDistanceSupportedConvex)
{
  EXPECT_TRUE(
      NarrowPhase::isDistanceSupported(ShapeType::Convex, ShapeType::Convex));
  EXPECT_TRUE(
      NarrowPhase::isDistanceSupported(ShapeType::Convex, ShapeType::Sphere));
  EXPECT_TRUE(
      NarrowPhase::isDistanceSupported(ShapeType::Mesh, ShapeType::Mesh));
  EXPECT_TRUE(
      NarrowPhase::isDistanceSupported(ShapeType::Mesh, ShapeType::Convex));
}
