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
#include <dart/collision/native/narrow_phase/distance.hpp>
#include <dart/collision/native/narrow_phase/narrow_phase.hpp>
#include <dart/collision/native/shapes/shape.hpp>

#include <gtest/gtest.h>

#include <array>
#include <numbers>
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
