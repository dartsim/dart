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

#include <dart/collision/native/Types.hpp>
#include <dart/collision/native/narrow_phase/SphereBox.hpp>
#include <dart/collision/native/shapes/Shape.hpp>

#include <gtest/gtest.h>

#include <array>
#include <vector>

using namespace dart::collision::native;

namespace {

constexpr double kPi = 3.141592653589793238462643383279502884;

Eigen::Isometry3d makeBoxTransform(const Eigen::Vector3d& translation)
{
  Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
  tf.translation() = translation;
  return tf;
}

Eigen::Isometry3d makeBoxTransform(
    const Eigen::Vector3d& translation,
    double angle,
    const Eigen::Vector3d& axis)
{
  Eigen::Isometry3d tf = makeBoxTransform(translation);
  tf.linear() = Eigen::AngleAxisd(angle, axis.normalized()).toRotationMatrix();
  return tf;
}

Eigen::Isometry3d makeSphereTransformInBoxFrame(
    const Eigen::Vector3d& translation, double angle)
{
  Eigen::Isometry3d tf = makeBoxTransform(translation);
  tf.linear()
      = Eigen::AngleAxisd(angle, Eigen::Vector3d(1.0, 2.0, 3.0).normalized())
            .toRotationMatrix();
  return tf;
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

} // namespace

TEST(SphereBox, Separated_AlongX)
{
  CollisionResult result;

  Eigen::Isometry3d boxTransform = Eigen::Isometry3d::Identity();

  bool collided = collideSphereBox(
      Eigen::Vector3d(5, 0, 0),
      1.0,
      Eigen::Vector3d(1, 1, 1),
      boxTransform,
      result);

  EXPECT_FALSE(collided);
  EXPECT_EQ(result.numContacts(), 0);
}

TEST(SphereBox, Separated_AlongY)
{
  CollisionResult result;

  Eigen::Isometry3d boxTransform = Eigen::Isometry3d::Identity();

  bool collided = collideSphereBox(
      Eigen::Vector3d(0, 5, 0),
      1.0,
      Eigen::Vector3d(1, 1, 1),
      boxTransform,
      result);

  EXPECT_FALSE(collided);
  EXPECT_EQ(result.numContacts(), 0);
}

TEST(SphereBox, Separated_AlongZ)
{
  CollisionResult result;

  Eigen::Isometry3d boxTransform = Eigen::Isometry3d::Identity();

  bool collided = collideSphereBox(
      Eigen::Vector3d(0, 0, 5),
      1.0,
      Eigen::Vector3d(1, 1, 1),
      boxTransform,
      result);

  EXPECT_FALSE(collided);
  EXPECT_EQ(result.numContacts(), 0);
}

TEST(SphereBox, Touching_FaceX)
{
  CollisionResult result;

  Eigen::Isometry3d boxTransform = Eigen::Isometry3d::Identity();

  bool collided = collideSphereBox(
      Eigen::Vector3d(2, 0, 0),
      1.0,
      Eigen::Vector3d(1, 1, 1),
      boxTransform,
      result);

  EXPECT_TRUE(collided);
  ASSERT_EQ(result.numContacts(), 1);
  EXPECT_NEAR(result.getContact(0).depth, 0.0, 1e-10);
}

TEST(SphereBox, Overlapping_FaceX)
{
  CollisionResult result;

  Eigen::Isometry3d boxTransform = Eigen::Isometry3d::Identity();

  bool collided = collideSphereBox(
      Eigen::Vector3d(1.5, 0, 0),
      1.0,
      Eigen::Vector3d(1, 1, 1),
      boxTransform,
      result);

  EXPECT_TRUE(collided);
  ASSERT_EQ(result.numContacts(), 1);
  EXPECT_NEAR(result.getContact(0).depth, 0.5, 1e-10);
}

TEST(SphereBox, Overlapping_FaceY)
{
  CollisionResult result;

  Eigen::Isometry3d boxTransform = Eigen::Isometry3d::Identity();

  bool collided = collideSphereBox(
      Eigen::Vector3d(0, 1.5, 0),
      1.0,
      Eigen::Vector3d(1, 1, 1),
      boxTransform,
      result);

  EXPECT_TRUE(collided);
  ASSERT_EQ(result.numContacts(), 1);
  EXPECT_NEAR(result.getContact(0).depth, 0.5, 1e-10);
}

TEST(SphereBox, Overlapping_FaceZ)
{
  CollisionResult result;

  Eigen::Isometry3d boxTransform = Eigen::Isometry3d::Identity();

  bool collided = collideSphereBox(
      Eigen::Vector3d(0, 0, 1.5),
      1.0,
      Eigen::Vector3d(1, 1, 1),
      boxTransform,
      result);

  EXPECT_TRUE(collided);
  ASSERT_EQ(result.numContacts(), 1);
  EXPECT_NEAR(result.getContact(0).depth, 0.5, 1e-10);
}

TEST(SphereBox, SphereCenterInsideBox)
{
  CollisionResult result;

  Eigen::Isometry3d boxTransform = Eigen::Isometry3d::Identity();

  bool collided = collideSphereBox(
      Eigen::Vector3d(0, 0, 0),
      0.5,
      Eigen::Vector3d(1, 1, 1),
      boxTransform,
      result);

  EXPECT_TRUE(collided);
  ASSERT_EQ(result.numContacts(), 1);
  EXPECT_NEAR(result.getContact(0).depth, 1.5, 1e-10);
}

TEST(SphereBox, SphereCenterAtBoxCenter)
{
  CollisionResult result;

  Eigen::Isometry3d boxTransform = Eigen::Isometry3d::Identity();

  bool collided = collideSphereBox(
      Eigen::Vector3d(0, 0, 0),
      1.0,
      Eigen::Vector3d(1, 1, 1),
      boxTransform,
      result);

  EXPECT_TRUE(collided);
  ASSERT_EQ(result.numContacts(), 1);
  EXPECT_NEAR(result.getContact(0).depth, 2.0, 1e-10);
}

TEST(SphereBox, NearCenterTranslatedBoxChoosesShortestAxis)
{
  const Eigen::Isometry3d boxTransform
      = makeBoxTransform(Eigen::Vector3d(3.0, -2.0, 5.0));
  const SphereShape sphere(0.25);

  const std::array<std::pair<Eigen::Vector3d, Eigen::Vector3d>, 3> cases{{
      {Eigen::Vector3d(1.0, 2.0, 3.0), -Eigen::Vector3d::UnitX()},
      {Eigen::Vector3d(2.0, 1.0, 3.0), -Eigen::Vector3d::UnitY()},
      {Eigen::Vector3d(2.0, 3.0, 1.0), -Eigen::Vector3d::UnitZ()},
  }};

  for (const auto& [halfExtents, expectedNormal] : cases) {
    SCOPED_TRACE(halfExtents.transpose());
    const BoxShape box(halfExtents);
    Eigen::Isometry3d sphereTransform = boxTransform;
    sphereTransform.translation() += Eigen::Vector3d(1e-14, -1e-14, 1e-14);

    CollisionResult result;
    const bool collided
        = collideSphereBox(sphere, sphereTransform, box, boxTransform, result);

    ASSERT_TRUE(collided);
    ASSERT_EQ(result.numContacts(), 1u);
    expectVectorNear(result.getContact(0).normal, expectedNormal, 1e-12);
    EXPECT_NEAR(
        result.getContact(0).depth,
        sphere.getRadius() + halfExtents.minCoeff(),
        1e-12);
  }
}

TEST(SphereBox, TranslatedNearOriginInsideChoosesShortestAxis)
{
  const Eigen::Isometry3d boxTransform
      = makeBoxTransform(Eigen::Vector3d(3.0, -2.0, 5.0));
  constexpr double sphereRadius = 0.25;
  const Eigen::Vector3d nearOrigin(1e-14, -1e-14, 1e-14);

  const std::array<std::pair<Eigen::Vector3d, Eigen::Vector3d>, 3> cases{{
      {Eigen::Vector3d(1.0, 2.0, 3.0), -Eigen::Vector3d::UnitX()},
      {Eigen::Vector3d(2.0, 1.0, 3.0), -Eigen::Vector3d::UnitY()},
      {Eigen::Vector3d(2.0, 3.0, 1.0), -Eigen::Vector3d::UnitZ()},
  }};

  for (const auto& [halfExtents, expectedNormal] : cases) {
    SCOPED_TRACE(halfExtents.transpose());
    CollisionResult result;
    const bool collided = collideSphereBox(
        boxTransform.translation() + nearOrigin,
        sphereRadius,
        halfExtents,
        boxTransform,
        result);

    ASSERT_TRUE(collided);
    ASSERT_EQ(result.numContacts(), 1u);
    expectVectorNear(result.getContact(0).normal, expectedNormal, 1e-12);
    EXPECT_NEAR(
        result.getContact(0).depth,
        sphereRadius + halfExtents.minCoeff(),
        1e-12);
  }
}

TEST(SphereBox, Corner)
{
  CollisionResult result;

  Eigen::Isometry3d boxTransform = Eigen::Isometry3d::Identity();

  double sphereRadius = 1.0;
  double offset = 1.0 + sphereRadius * 0.5;

  bool collided = collideSphereBox(
      Eigen::Vector3d(offset, offset, offset),
      sphereRadius,
      Eigen::Vector3d(1, 1, 1),
      boxTransform,
      result);

  double expectedDist = std::sqrt(3.0 * (offset - 1.0) * (offset - 1.0));
  bool shouldCollide = expectedDist < sphereRadius;

  EXPECT_EQ(collided, shouldCollide);
}

TEST(SphereBox, Edge)
{
  CollisionResult result;

  Eigen::Isometry3d boxTransform = Eigen::Isometry3d::Identity();

  double offset = 1.2;
  double sphereRadius = 1.0;

  bool collided = collideSphereBox(
      Eigen::Vector3d(offset, offset, 0),
      sphereRadius,
      Eigen::Vector3d(1, 1, 1),
      boxTransform,
      result);

  double edgeDist = std::sqrt(2.0 * (offset - 1.0) * (offset - 1.0));
  bool shouldCollide = edgeDist < sphereRadius;

  EXPECT_EQ(collided, shouldCollide);
}

TEST(SphereBox, RotatedBox)
{
  CollisionResult result;

  Eigen::Isometry3d boxTransform = Eigen::Isometry3d::Identity();
  boxTransform.rotate(Eigen::AngleAxisd(kPi / 4, Eigen::Vector3d::UnitZ()));

  double rotatedHalfWidth = std::sqrt(2.0);
  double sphereCenter = rotatedHalfWidth + 0.5;

  bool collided = collideSphereBox(
      Eigen::Vector3d(sphereCenter, 0, 0),
      1.0,
      Eigen::Vector3d(1, 1, 1),
      boxTransform,
      result);

  EXPECT_TRUE(collided);
  ASSERT_EQ(result.numContacts(), 1);
  EXPECT_NEAR(result.getContact(0).depth, 0.5, 1e-6);
}

TEST(SphereBox, TranslatedBox)
{
  CollisionResult result;

  Eigen::Isometry3d boxTransform = Eigen::Isometry3d::Identity();
  boxTransform.translation() = Eigen::Vector3d(5, 0, 0);

  bool collided = collideSphereBox(
      Eigen::Vector3d(6.5, 0, 0),
      1.0,
      Eigen::Vector3d(1, 1, 1),
      boxTransform,
      result);

  EXPECT_TRUE(collided);
  ASSERT_EQ(result.numContacts(), 1);
  EXPECT_NEAR(result.getContact(0).depth, 0.5, 1e-10);
}

TEST(SphereBox, DifferentSizes)
{
  CollisionResult result;

  Eigen::Isometry3d boxTransform = Eigen::Isometry3d::Identity();

  bool collided = collideSphereBox(
      Eigen::Vector3d(3, 0, 0),
      2.0,
      Eigen::Vector3d(2, 1, 1),
      boxTransform,
      result);

  EXPECT_TRUE(collided);
  ASSERT_EQ(result.numContacts(), 1);
  EXPECT_NEAR(result.getContact(0).depth, 1.0, 1e-10);
}

TEST(SphereBox, SmallShapes)
{
  CollisionResult result;

  Eigen::Isometry3d boxTransform = Eigen::Isometry3d::Identity();

  bool collided = collideSphereBox(
      Eigen::Vector3d(0.0015, 0, 0),
      0.001,
      Eigen::Vector3d(0.001, 0.001, 0.001),
      boxTransform,
      result);

  EXPECT_TRUE(collided);
  ASSERT_EQ(result.numContacts(), 1);
  EXPECT_NEAR(result.getContact(0).depth, 0.0005, 1e-10);
}

TEST(SphereBox, LargeShapes)
{
  CollisionResult result;

  Eigen::Isometry3d boxTransform = Eigen::Isometry3d::Identity();

  bool collided = collideSphereBox(
      Eigen::Vector3d(1500, 0, 0),
      1000.0,
      Eigen::Vector3d(1000, 1000, 1000),
      boxTransform,
      result);

  EXPECT_TRUE(collided);
  ASSERT_EQ(result.numContacts(), 1);
  EXPECT_NEAR(result.getContact(0).depth, 500.0, 1e-6);
}

TEST(SphereBox, MaxContactsRespected)
{
  CollisionResult result;
  CollisionOption option;
  option.maxNumContacts = 0;

  Eigen::Isometry3d boxTransform = Eigen::Isometry3d::Identity();

  bool collided = collideSphereBox(
      Eigen::Vector3d(1.5, 0, 0),
      1.0,
      Eigen::Vector3d(1, 1, 1),
      boxTransform,
      result,
      option);

  EXPECT_FALSE(collided);
  EXPECT_EQ(result.numContacts(), 0);
}

TEST(SphereBox, BinaryCheckDoesNotAddContacts)
{
  CollisionOption option = CollisionOption::binaryCheck();

  CollisionResult translatedResult;
  bool translatedHit = collideSphereBox(
      Eigen::Vector3d(1.5, 0, 0),
      1.0,
      Eigen::Vector3d(1, 1, 1),
      Eigen::Isometry3d::Identity(),
      translatedResult,
      option);

  EXPECT_TRUE(translatedHit);
  EXPECT_EQ(translatedResult.numContacts(), 0u);

  CollisionResult rotatedResult;
  const Eigen::Isometry3d rotatedBox = makeBoxTransform(
      Eigen::Vector3d(1.3, 2.7, 6.5),
      kPi / 3.0,
      Eigen::Vector3d(1.0, 2.0, 3.0));
  bool rotatedHit = collideSphereBox(
      rotatedBox * Eigen::Vector3d(1.5, 0, 0),
      1.0,
      Eigen::Vector3d(1, 1, 1),
      rotatedBox,
      rotatedResult,
      option);

  EXPECT_TRUE(rotatedHit);
  EXPECT_EQ(rotatedResult.numContacts(), 0u);

  CollisionResult separatedResult;
  bool separatedHit = collideSphereBox(
      Eigen::Vector3d(5, 0, 0),
      1.0,
      Eigen::Vector3d(1, 1, 1),
      Eigen::Isometry3d::Identity(),
      separatedResult,
      option);

  EXPECT_FALSE(separatedHit);
  EXPECT_EQ(separatedResult.numContacts(), 0u);
}

TEST(SphereBox, SeparatedAcrossBoxFramesAndSphereOrientations)
{
  constexpr double sphereRadius = 0.7;
  const Eigen::Vector3d halfSize(0.3, 0.6, 1.8);
  const BoxShape box(halfSize);
  const SphereShape sphere(sphereRadius);

  struct Case
  {
    const char* name;
    Eigen::Vector3d sphereCenterInBox;
  };

  const std::array<Case, 2> cases{{
      {"separated from +z face",
       Eigen::Vector3d(
           halfSize.x() * 0.5,
           halfSize.y() * 0.5,
           halfSize.z() + sphereRadius * 1.1)},
      {"separated from +x,+y,+z corner",
       halfSize + Eigen::Vector3d::Ones() * sphereRadius * 1.25},
  }};

  const std::array<Eigen::Isometry3d, 4> boxTransforms{{
      Eigen::Isometry3d::Identity(),
      makeBoxTransform(Eigen::Vector3d(1.3, 2.7, 6.5)),
      makeBoxTransform(
          Eigen::Vector3d(1.3, 2.7, 6.5), kPi / 2.0, Eigen::Vector3d::UnitY()),
      makeBoxTransform(
          Eigen::Vector3d(1.3, 2.7, 6.5),
          kPi / 3.0,
          Eigen::Vector3d(1.0, 2.0, 3.0)),
  }};
  constexpr std::array<double, 2> sphereRotationAngles{0.0, kPi / 3.0};

  for (const auto& testCase : cases) {
    for (const auto& tfBox : boxTransforms) {
      for (const double sphereRotationAngle : sphereRotationAngles) {
        SCOPED_TRACE(testCase.name);
        CollisionResult result;
        const bool collided = collideSphereBox(
            sphere,
            tfBox
                * makeSphereTransformInBoxFrame(
                    testCase.sphereCenterInBox, sphereRotationAngle),
            box,
            tfBox,
            result);

        EXPECT_FALSE(collided);
        EXPECT_EQ(result.numContacts(), 0u);
      }
    }
  }
}

TEST(SphereBox, ContactsAcrossBoxFramesAndSphereOrientations)
{
  constexpr double sphereRadius = 0.7;
  const Eigen::Vector3d halfSize(0.3, 0.6, 1.8);
  const BoxShape box(halfSize);
  const SphereShape sphere(sphereRadius);

  struct Case
  {
    const char* name;
    Eigen::Vector3d sphereCenterInBox;
    Eigen::Vector3d expectedNormalInBox;
    double expectedDepth;
  };

  std::vector<Case> cases;

  const double externalDepth = sphereRadius * 0.5;
  cases.push_back(
      {"external +z face",
       halfSize + Eigen::Vector3d(0.0, 0.0, sphereRadius - externalDepth),
       -Eigen::Vector3d::UnitZ(),
       externalDepth});

  const Eigen::Vector3d vertexNormalToBox
      = Eigen::Vector3d(-1.0, -2.0, -3.0).normalized();
  cases.push_back(
      {"external +x,+y,+z vertex",
       halfSize - vertexNormalToBox * (sphereRadius - externalDepth),
       vertexNormalToBox,
       externalDepth});

  const double centerInset = halfSize.minCoeff() * 0.5;
  for (int axis = 0; axis < 3; ++axis) {
    for (double sign : {-1.0, 1.0}) {
      const Eigen::Vector3d direction = sign * Eigen::Vector3d::Unit(axis);
      cases.push_back(
          {"internal nearest face",
           direction * (centerInset - halfSize[axis]),
           direction,
           sphereRadius + centerInset});
    }
  }

  cases.push_back(
      {"center on +z face",
       Eigen::Vector3d(0.0, 0.0, halfSize.z()),
       -Eigen::Vector3d::UnitZ(),
       sphereRadius});
  cases.push_back(
      {"center on +x,+y,+z corner",
       halfSize,
       -Eigen::Vector3d::UnitX(),
       sphereRadius});

  const std::array<Case, 6> coincidentCases{{
      {"coincident cube",
       Eigen::Vector3d::Zero(),
       -Eigen::Vector3d::UnitX(),
       15.0},
      {"coincident x and z minimum",
       Eigen::Vector3d::Zero(),
       -Eigen::Vector3d::UnitX(),
       15.0},
      {"coincident x minimum",
       Eigen::Vector3d::Zero(),
       -Eigen::Vector3d::UnitX(),
       15.0},
      {"coincident y and z minimum",
       Eigen::Vector3d::Zero(),
       -Eigen::Vector3d::UnitY(),
       15.0},
      {"coincident y minimum",
       Eigen::Vector3d::Zero(),
       -Eigen::Vector3d::UnitY(),
       15.0},
      {"coincident z minimum",
       Eigen::Vector3d::Zero(),
       -Eigen::Vector3d::UnitZ(),
       15.0},
  }};
  const std::array<Eigen::Vector3d, 6> coincidentHalfSizes{{
      Eigen::Vector3d(10.0, 10.0, 10.0),
      Eigen::Vector3d(10.0, 15.0, 10.0),
      Eigen::Vector3d(10.0, 12.0, 14.0),
      Eigen::Vector3d(15.0, 10.0, 10.0),
      Eigen::Vector3d(15.0, 10.0, 14.0),
      Eigen::Vector3d(15.0, 12.0, 10.0),
  }};

  const std::array<Eigen::Isometry3d, 4> boxTransforms{{
      Eigen::Isometry3d::Identity(),
      makeBoxTransform(Eigen::Vector3d(1.3, 2.7, 6.5)),
      makeBoxTransform(
          Eigen::Vector3d(1.3, 2.7, 6.5), kPi / 2.0, Eigen::Vector3d::UnitX()),
      makeBoxTransform(
          Eigen::Vector3d(1.3, 2.7, 6.5),
          kPi / 3.0,
          Eigen::Vector3d(1.0, 2.0, 3.0)),
  }};
  constexpr std::array<double, 2> sphereRotationAngles{0.0, kPi / 3.0};

  for (const auto& testCase : cases) {
    for (const auto& tfBox : boxTransforms) {
      for (const double sphereRotationAngle : sphereRotationAngles) {
        SCOPED_TRACE(testCase.name);
        CollisionResult result;
        const bool collided = collideSphereBox(
            sphere,
            tfBox
                * makeSphereTransformInBoxFrame(
                    testCase.sphereCenterInBox, sphereRotationAngle),
            box,
            tfBox,
            result);
        ASSERT_TRUE(collided);
        ASSERT_EQ(result.numContacts(), 1u);
        EXPECT_NEAR(result.getContact(0).depth, testCase.expectedDepth, 1e-9);
        expectVectorNear(
            tfBox.rotation().transpose() * result.getContact(0).normal,
            testCase.expectedNormalInBox,
            1e-9);
      }
    }
  }

  for (std::size_t i = 0; i < coincidentCases.size(); ++i) {
    const auto& testCase = coincidentCases[i];
    const BoxShape coincidentBox(coincidentHalfSizes[i]);
    const SphereShape coincidentSphere(5.0);

    for (const auto& tfBox : boxTransforms) {
      SCOPED_TRACE(testCase.name);
      CollisionResult result;
      const bool collided = collideSphereBox(
          coincidentSphere,
          tfBox * makeSphereTransformInBoxFrame(testCase.sphereCenterInBox, 0),
          coincidentBox,
          tfBox,
          result);

      ASSERT_TRUE(collided);
      ASSERT_EQ(result.numContacts(), 1u);
      EXPECT_NEAR(result.getContact(0).depth, testCase.expectedDepth, 1e-9);
      expectVectorNear(
          tfBox.rotation().transpose() * result.getContact(0).normal,
          testCase.expectedNormalInBox,
          1e-9);
    }
  }
}

TEST(SphereBox, ContactsWithIncompatibleScaleRatios)
{
  struct Case
  {
    const char* name;
    Eigen::Vector3d halfSize;
    double sphereRadius;
    Eigen::Vector3d sphereCenterInBox;
    Eigen::Vector3d expectedNormalInBox;
    double expectedDepth;
    bool expectedCollision;
  };

  const Eigen::Vector3d largeSphereNormalToBox
      = Eigen::Vector3d(-1.0, -2.0, -3.0).normalized();
  const std::array<Case, 4> cases{{
      {"long skinny box collides with small sphere",
       Eigen::Vector3d(15.0, 1.0, 1.0),
       0.01,
       Eigen::Vector3d(15.0 * 0.95, 0.0, 1.0 + 0.01 * 0.5),
       -Eigen::Vector3d::UnitZ(),
       0.005,
       true},
      {"long skinny box separated from small sphere",
       Eigen::Vector3d(15.0, 1.0, 1.0),
       0.01,
       Eigen::Vector3d(15.0 * 0.95, 0.0, 1.0 + 0.01 + 0.001),
       -Eigen::Vector3d::UnitZ(),
       0.0,
       false},
      {"large sphere collides with tiny box",
       Eigen::Vector3d(0.1, 0.15, 0.2),
       10.0,
       Eigen::Vector3d(0.1, 0.15, 0.2) - largeSphereNormalToBox * (10.0 - 0.05),
       largeSphereNormalToBox,
       0.05,
       true},
      {"large sphere separated from tiny box",
       Eigen::Vector3d(0.1, 0.15, 0.2),
       10.0,
       Eigen::Vector3d(0.1, 0.15, 0.2) - largeSphereNormalToBox * (10.0 + 0.01),
       largeSphereNormalToBox,
       0.0,
       false},
  }};

  const std::array<Eigen::Isometry3d, 2> boxTransforms{{
      Eigen::Isometry3d::Identity(),
      makeBoxTransform(
          Eigen::Vector3d(1.3, 2.7, 6.5),
          kPi / 3.0,
          Eigen::Vector3d(1.0, 2.0, 3.0)),
  }};

  for (const auto& testCase : cases) {
    for (const auto& tfBox : boxTransforms) {
      SCOPED_TRACE(testCase.name);
      const BoxShape box(testCase.halfSize);
      const SphereShape sphere(testCase.sphereRadius);
      CollisionResult result;
      const bool collided = collideSphereBox(
          sphere,
          tfBox * makeSphereTransformInBoxFrame(testCase.sphereCenterInBox, 0),
          box,
          tfBox,
          result);

      EXPECT_EQ(collided, testCase.expectedCollision);
      if (testCase.expectedCollision) {
        ASSERT_EQ(result.numContacts(), 1u);
        EXPECT_NEAR(result.getContact(0).depth, testCase.expectedDepth, 1e-9);
        expectVectorNear(
            tfBox.rotation().transpose() * result.getContact(0).normal,
            testCase.expectedNormalInBox,
            1e-9);
      } else {
        EXPECT_EQ(result.numContacts(), 0u);
      }
    }
  }
}

TEST(SphereBox, UsingShapeObjects)
{
  SphereShape sphere(1.0);
  BoxShape box(Eigen::Vector3d(1, 1, 1));

  Eigen::Isometry3d sphereTransform = Eigen::Isometry3d::Identity();
  sphereTransform.translation() = Eigen::Vector3d(1.5, 0, 0);

  Eigen::Isometry3d boxTransform = Eigen::Isometry3d::Identity();

  CollisionResult result;

  bool collided
      = collideSphereBox(sphere, sphereTransform, box, boxTransform, result);

  EXPECT_TRUE(collided);
  ASSERT_EQ(result.numContacts(), 1);
  EXPECT_NEAR(result.getContact(0).depth, 0.5, 1e-10);
}

TEST(SphereBox, Determinism)
{
  std::vector<ContactPoint> contacts;

  for (int i = 0; i < 100; ++i) {
    CollisionResult result;

    Eigen::Isometry3d boxTransform = Eigen::Isometry3d::Identity();
    boxTransform.rotate(
        Eigen::AngleAxisd(0.123, Eigen::Vector3d(1, 2, 3).normalized()));
    boxTransform.translation() = Eigen::Vector3d(0.5, 0.3, 0.2);

    bool collided = collideSphereBox(
        Eigen::Vector3d(1.1, 0.9, 0.8),
        1.5,
        Eigen::Vector3d(1.2, 0.8, 1.0),
        boxTransform,
        result);

    EXPECT_TRUE(collided);
    ASSERT_EQ(result.numContacts(), 1);
    contacts.push_back(result.getContact(0));
  }

  for (std::size_t i = 1; i < contacts.size(); ++i) {
    EXPECT_EQ(contacts[i].position.x(), contacts[0].position.x());
    EXPECT_EQ(contacts[i].position.y(), contacts[0].position.y());
    EXPECT_EQ(contacts[i].position.z(), contacts[0].position.z());
    EXPECT_EQ(contacts[i].normal.x(), contacts[0].normal.x());
    EXPECT_EQ(contacts[i].normal.y(), contacts[0].normal.y());
    EXPECT_EQ(contacts[i].normal.z(), contacts[0].normal.z());
    EXPECT_EQ(contacts[i].depth, contacts[0].depth);
  }
}
