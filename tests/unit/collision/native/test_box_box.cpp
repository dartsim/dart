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

#include <dart/collision/native/narrow_phase/box_box.hpp>
#include <dart/collision/native/narrow_phase/box_box/contact_reduction.hpp>
#include <dart/collision/native/narrow_phase/box_box/sat.hpp>
#include <dart/collision/native/shapes/shape.hpp>
#include <dart/collision/native/types.hpp>

#include <gtest/gtest.h>

#include <array>
#include <limits>
#include <random>
#include <vector>

#include <cmath>

using namespace dart::collision::native;

namespace {

constexpr double kPi = 3.141592653589793238462643383279502884;

} // namespace

namespace {

struct BoxSpec
{
  Eigen::Vector3d halfExtents;
  Eigen::Isometry3d transform;
};

Eigen::Isometry3d makeBoxBatchTransform(
    double x, double y, double z, double angle)
{
  Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
  tf.linear() = (Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitX())
                 * Eigen::AngleAxisd(0.5 * angle, Eigen::Vector3d::UnitY())
                 * Eigen::AngleAxisd(-0.25 * angle, Eigen::Vector3d::UnitZ()))
                    .toRotationMatrix();
  tf.translation() = Eigen::Vector3d(x, y, z);
  return tf;
}

Eigen::Isometry3d makeRotationTransform(
    double angle, const Eigen::Vector3d& axis)
{
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.linear() = Eigen::AngleAxisd(angle, axis).toRotationMatrix();
  return transform;
}

std::array<Eigen::Isometry3d, 6> makeCubeEquivalentOrientations()
{
  constexpr double pi = kPi;
  return {
      Eigen::Isometry3d::Identity(),
      makeRotationTransform(pi, Eigen::Vector3d::UnitX()),
      makeRotationTransform(0.5 * pi, Eigen::Vector3d::UnitX()),
      makeRotationTransform(1.5 * pi, Eigen::Vector3d::UnitX()),
      makeRotationTransform(0.5 * pi, Eigen::Vector3d::UnitY()),
      makeRotationTransform(1.5 * pi, Eigen::Vector3d::UnitY())};
}

template <typename ResultExpectation>
void expectBoxCubeSymmetryCases(
    const BoxSpec& fixedBox,
    const BoxSpec& cubeBox,
    double expectedDepth,
    const CollisionOption& option,
    const ResultExpectation& expectResult)
{
  for (const auto& cubeOrientation : makeCubeEquivalentOrientations()) {
    BoxSpec posedCube = cubeBox;
    posedCube.transform = cubeBox.transform * cubeOrientation;

    CollisionResult result;
    ASSERT_TRUE(collideBoxes(
        fixedBox.halfExtents,
        fixedBox.transform,
        posedCube.halfExtents,
        posedCube.transform,
        result,
        option));
    expectResult(
        result,
        fixedBox,
        posedCube,
        Eigen::Vector3d::UnitZ(),
        expectedDepth,
        "fixed-cube");

    CollisionResult swappedResult;
    ASSERT_TRUE(collideBoxes(
        posedCube.halfExtents,
        posedCube.transform,
        fixedBox.halfExtents,
        fixedBox.transform,
        swappedResult,
        option));
    expectResult(
        swappedResult,
        posedCube,
        fixedBox,
        -Eigen::Vector3d::UnitZ(),
        expectedDepth,
        "cube-fixed");
  }
}

void expectPointInsideBox(
    const Eigen::Vector3d& point,
    const BoxSpec& box,
    double tolerance,
    const char* label)
{
  const Eigen::Vector3d local = box.transform.inverse() * point;
  for (int i = 0; i < 3; ++i) {
    EXPECT_LE(std::abs(local[i]), box.halfExtents[i] + tolerance)
        << label << " local=" << local.transpose();
  }
}

void expectVectorExactlyEqual(
    const Eigen::Vector3d& expected, const Eigen::Vector3d& actual)
{
  EXPECT_EQ(expected.x(), actual.x());
  EXPECT_EQ(expected.y(), actual.y());
  EXPECT_EQ(expected.z(), actual.z());
}

void expectContactExactlyEqual(
    const ContactPoint& expected, const ContactPoint& actual)
{
  expectVectorExactlyEqual(expected.position, actual.position);
  expectVectorExactlyEqual(expected.normal, actual.normal);
  EXPECT_EQ(expected.depth, actual.depth);
  EXPECT_EQ(expected.object1, actual.object1);
  EXPECT_EQ(expected.object2, actual.object2);
  EXPECT_EQ(expected.featureIndex1, actual.featureIndex1);
  EXPECT_EQ(expected.featureIndex2, actual.featureIndex2);
}

void expectCollisionResultExactlyEqual(
    const CollisionResult& expected, const CollisionResult& actual)
{
  ASSERT_EQ(expected.numManifolds(), actual.numManifolds());
  ASSERT_EQ(expected.numContacts(), actual.numContacts());

  for (std::size_t i = 0; i < expected.numManifolds(); ++i) {
    const auto& expectedManifold = expected.getManifold(i);
    const auto& actualManifold = actual.getManifold(i);
    EXPECT_EQ(expectedManifold.getType(), actualManifold.getType());
    EXPECT_EQ(expectedManifold.getObject1(), actualManifold.getObject1());
    EXPECT_EQ(expectedManifold.getObject2(), actualManifold.getObject2());
    ASSERT_EQ(expectedManifold.numContacts(), actualManifold.numContacts());
    for (std::size_t j = 0; j < expectedManifold.numContacts(); ++j) {
      expectContactExactlyEqual(
          expectedManifold.getContact(j), actualManifold.getContact(j));
    }
  }
}

Eigen::Isometry3d makeBoxPose(
    const Eigen::Vector3d& translation,
    const Eigen::Vector3d& axis = Eigen::Vector3d::UnitZ(),
    double angle = 0.0)
{
  Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
  tf.linear() = Eigen::AngleAxisd(angle, axis.normalized()).toRotationMatrix();
  tf.translation() = translation;
  return tf;
}

void expectBoxBoxFiniteHit(
    const char* name,
    const Eigen::Vector3d& halfExtents1,
    const Eigen::Isometry3d& tf1,
    const Eigen::Vector3d& halfExtents2,
    const Eigen::Isometry3d& tf2)
{
  SCOPED_TRACE(name);
  CollisionResult result;
  const bool hit = collideBoxes(halfExtents1, tf1, halfExtents2, tf2, result);
  ASSERT_TRUE(hit);
  ASSERT_GE(result.numContacts(), 1u);

  for (std::size_t i = 0; i < result.numContacts(); ++i) {
    const auto& contact = result.getContact(i);
    EXPECT_GE(contact.depth, 0.0);
    EXPECT_TRUE(contact.position.allFinite());
    EXPECT_TRUE(contact.normal.allFinite());
    EXPECT_NEAR(contact.normal.norm(), 1.0, 1e-8);
  }
}

} // namespace

TEST(BoxBox, Separated_AlongX)
{
  CollisionResult result;

  Eigen::Isometry3d t1 = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d t2 = Eigen::Isometry3d::Identity();
  t2.translation() = Eigen::Vector3d(5, 0, 0);

  bool collided = collideBoxes(
      Eigen::Vector3d(1, 1, 1), t1, Eigen::Vector3d(1, 1, 1), t2, result);

  EXPECT_FALSE(collided);
  EXPECT_EQ(result.numContacts(), 0);
}

TEST(BoxBox, Separated_AlongY)
{
  CollisionResult result;

  Eigen::Isometry3d t1 = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d t2 = Eigen::Isometry3d::Identity();
  t2.translation() = Eigen::Vector3d(0, 5, 0);

  bool collided = collideBoxes(
      Eigen::Vector3d(1, 1, 1), t1, Eigen::Vector3d(1, 1, 1), t2, result);

  EXPECT_FALSE(collided);
  EXPECT_EQ(result.numContacts(), 0);
}

TEST(BoxBox, Separated_AlongZ)
{
  CollisionResult result;

  Eigen::Isometry3d t1 = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d t2 = Eigen::Isometry3d::Identity();
  t2.translation() = Eigen::Vector3d(0, 0, 5);

  bool collided = collideBoxes(
      Eigen::Vector3d(1, 1, 1), t1, Eigen::Vector3d(1, 1, 1), t2, result);

  EXPECT_FALSE(collided);
  EXPECT_EQ(result.numContacts(), 0);
}

TEST(BoxBox, Touching_AlongX)
{
  CollisionResult result;

  Eigen::Isometry3d t1 = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d t2 = Eigen::Isometry3d::Identity();
  t2.translation() = Eigen::Vector3d(2, 0, 0);

  bool collided = collideBoxes(
      Eigen::Vector3d(1, 1, 1), t1, Eigen::Vector3d(1, 1, 1), t2, result);

  EXPECT_TRUE(collided);
  ASSERT_GE(result.numContacts(), 1u);
  EXPECT_NEAR(result.getContact(0).depth, 0.0, 1e-10);
}

TEST(BoxBox, Overlapping_AlongX)
{
  CollisionResult result;

  Eigen::Isometry3d t1 = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d t2 = Eigen::Isometry3d::Identity();
  t2.translation() = Eigen::Vector3d(1.5, 0, 0);

  bool collided = collideBoxes(
      Eigen::Vector3d(1, 1, 1), t1, Eigen::Vector3d(1, 1, 1), t2, result);

  EXPECT_TRUE(collided);
  ASSERT_GE(result.numContacts(), 1u);
  EXPECT_NEAR(result.getContact(0).depth, 0.5, 1e-10);
}

TEST(BoxBox, Overlapping_AlongY)
{
  CollisionResult result;

  Eigen::Isometry3d t1 = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d t2 = Eigen::Isometry3d::Identity();
  t2.translation() = Eigen::Vector3d(0, 1.5, 0);

  bool collided = collideBoxes(
      Eigen::Vector3d(1, 1, 1), t1, Eigen::Vector3d(1, 1, 1), t2, result);

  EXPECT_TRUE(collided);
  ASSERT_GE(result.numContacts(), 1u);
  EXPECT_NEAR(result.getContact(0).depth, 0.5, 1e-10);
}

TEST(BoxBox, OverlappingFacePatch_AlongY)
{
  CollisionResult result;
  CollisionOption option;
  option.maxNumContacts = 8;

  Eigen::Isometry3d t1 = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d t2 = Eigen::Isometry3d::Identity();
  t2.translation() = Eigen::Vector3d(0, 1.5, 0);

  bool collided = collideBoxes(
      Eigen::Vector3d(1, 1, 1),
      t1,
      Eigen::Vector3d(1, 1, 1),
      t2,
      result,
      option);

  EXPECT_TRUE(collided);
  ASSERT_GE(result.numContacts(), 2u);
  ASSERT_LE(result.numContacts(), 4u);
  for (std::size_t i = 0; i < result.numContacts(); ++i) {
    EXPECT_NEAR(result.getContact(i).depth, 0.5, 1e-10);
    EXPECT_NEAR(std::abs(result.getContact(i).normal.y()), 1.0, 1e-10);
  }
}

TEST(BoxBox, Overlapping_AlongZ)
{
  CollisionResult result;

  Eigen::Isometry3d t1 = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d t2 = Eigen::Isometry3d::Identity();
  t2.translation() = Eigen::Vector3d(0, 0, 1.5);

  bool collided = collideBoxes(
      Eigen::Vector3d(1, 1, 1), t1, Eigen::Vector3d(1, 1, 1), t2, result);

  EXPECT_TRUE(collided);
  ASSERT_GE(result.numContacts(), 1u);
  EXPECT_NEAR(result.getContact(0).depth, 0.5, 1e-10);
}

TEST(BoxBox, Coincident)
{
  CollisionResult result;

  Eigen::Isometry3d t1 = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d t2 = Eigen::Isometry3d::Identity();

  bool collided = collideBoxes(
      Eigen::Vector3d(1, 1, 1), t1, Eigen::Vector3d(1, 1, 1), t2, result);

  EXPECT_TRUE(collided);
  ASSERT_GE(result.numContacts(), 1u);
  EXPECT_NEAR(result.getContact(0).depth, 2.0, 1e-10);
}

TEST(BoxBox, DifferentSizes)
{
  CollisionResult result;

  Eigen::Isometry3d t1 = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d t2 = Eigen::Isometry3d::Identity();
  t2.translation() = Eigen::Vector3d(2, 0, 0);

  bool collided = collideBoxes(
      Eigen::Vector3d(1, 1, 1), t1, Eigen::Vector3d(2, 2, 2), t2, result);

  EXPECT_TRUE(collided);
  ASSERT_GE(result.numContacts(), 1u);
  EXPECT_NEAR(result.getContact(0).depth, 1.0, 1e-10);
}

TEST(BoxBox, AxisSweepTransitionsAcrossSizesAndOffsets)
{
  struct SweepSpec
  {
    Eigen::Vector3d halfExtents1;
    Eigen::Vector3d halfExtents2;
    Eigen::Vector3d lateralOffset;
    double start;
    double step;
  };

  const std::array<SweepSpec, 3> specs{{
      {Eigen::Vector3d(0.5, 1.0, 0.5),
       Eigen::Vector3d(1.0, 0.5, 1.0),
       Eigen::Vector3d::Zero(),
       -5.0,
       0.1},
      {Eigen::Vector3d(0.05, 0.1, 0.05),
       Eigen::Vector3d(0.1, 0.05, 0.1),
       Eigen::Vector3d::Zero(),
       -0.5,
       0.01},
      {Eigen::Vector3d(0.5, 1.0, 0.5),
       Eigen::Vector3d(1.0, 0.5, 1.0),
       Eigen::Vector3d(0.0, -0.1, 0.0),
       -5.0,
       0.1},
  }};

  for (const auto& spec : specs) {
    for (int axisIndex = 0; axisIndex < 3; ++axisIndex) {
      if (axisIndex != 0 && spec.lateralOffset.norm() > 0.0) {
        continue;
      }

      Eigen::Vector3d axis = Eigen::Vector3d::Zero();
      axis[axisIndex] = 1.0;
      const double threshold
          = spec.halfExtents1[axisIndex] + spec.halfExtents2[axisIndex];
      constexpr double transitionTolerance = 1e-12;

      for (int i = 0; i < 100; ++i) {
        const double offset = spec.start + spec.step * i;
        SCOPED_TRACE(offset);

        CollisionResult result;
        const bool hit = collideBoxes(
            spec.halfExtents1,
            makeBoxPose(offset * axis + spec.lateralOffset),
            spec.halfExtents2,
            Eigen::Isometry3d::Identity(),
            result);
        const bool expectedHit
            = std::abs(offset) <= threshold + transitionTolerance;
        EXPECT_EQ(hit, expectedHit);

        if (!expectedHit) {
          EXPECT_EQ(result.numContacts(), 0u);
          continue;
        }

        ASSERT_GE(result.numContacts(), 1u);
        const auto& contact = result.getContact(0);
        EXPECT_GE(contact.depth, 0.0);
        EXPECT_TRUE(contact.position.allFinite());
        EXPECT_TRUE(contact.normal.allFinite());
      }
    }
  }
}

TEST(BoxBox, RotationSweepTransitionsAtNearTouchingOffset)
{
  const Eigen::Vector3d halfExtents(0.5, 0.5, 0.5);
  const Eigen::Isometry3d tf2 = Eigen::Isometry3d::Identity();

  struct RotationCase
  {
    double angle;
    bool expectedHit;
  };

  const std::array<RotationCase, 5> cases{{
      {0.0, false},
      {kPi / 20.0, true},
      {kPi / 4.0, true},
      {kPi / 2.0, false},
      {kPi, false},
  }};

  for (const auto& testCase : cases) {
    SCOPED_TRACE(testCase.angle);
    CollisionResult result;
    const bool hit = collideBoxes(
        halfExtents,
        makeBoxPose(
            Eigen::Vector3d(-1.01, 0.0, 0.0),
            Eigen::Vector3d::UnitY(),
            testCase.angle),
        halfExtents,
        tf2,
        result);
    EXPECT_EQ(hit, testCase.expectedHit);
    if (testCase.expectedHit) {
      ASSERT_GE(result.numContacts(), 1u);
      EXPECT_TRUE(result.getContact(0).position.allFinite());
      EXPECT_TRUE(result.getContact(0).normal.allFinite());
    } else {
      EXPECT_EQ(result.numContacts(), 0u);
    }
  }
}

TEST(BoxBox, TranslatedAndRotatedPenetrationsReportFiniteContacts)
{
  const Eigen::Vector3d unitBox(0.5, 0.5, 0.5);
  const Eigen::Vector3d rectangularBox(0.25, 0.5, 0.75);

  expectBoxBoxFiniteHit(
      "offset rectangular box",
      unitBox,
      Eigen::Isometry3d::Identity(),
      rectangularBox,
      makeBoxPose(Eigen::Vector3d(0.1, 0.0, 0.0)));

  expectBoxBoxFiniteHit(
      "translated unit box against rectangular box",
      unitBox,
      makeBoxPose(Eigen::Vector3d(-0.3, 0.5, 1.0)),
      rectangularBox,
      makeBoxPose(Eigen::Vector3d(0.1, 0.0, 0.0)));

  expectBoxBoxFiniteHit(
      "rotated shallow cube overlap",
      unitBox,
      makeBoxPose(
          Eigen::Vector3d(0.1, 0.0, 0.1), Eigen::Vector3d::UnitZ(), kPi / 4.0),
      unitBox,
      Eigen::Isometry3d::Identity());

  expectBoxBoxFiniteHit(
      "rotated edge-biased overlap",
      unitBox,
      makeBoxPose(
          Eigen::Vector3d(-0.5, 0.5, 0.0), Eigen::Vector3d::UnitZ(), kPi / 4.0),
      unitBox,
      Eigen::Isometry3d::Identity());

  expectBoxBoxFiniteHit(
      "oblique rotated overlap",
      unitBox,
      makeBoxPose(
          Eigen::Vector3d(-0.5, 0.1, 0.4),
          Eigen::Vector3d(0.0, 1.0, 1.0),
          kPi / 4.0),
      unitBox,
      makeBoxPose(Eigen::Vector3d(0.1, 0.0, 0.0)));

  expectBoxBoxFiniteHit(
      "small shallow face overlap",
      unitBox,
      Eigen::Isometry3d::Identity(),
      Eigen::Vector3d(0.25, 0.25, 0.25),
      makeBoxPose(Eigen::Vector3d(0.0, 0.73, 0.0)));

  expectBoxBoxFiniteHit(
      "small shallow corner overlap",
      unitBox,
      Eigen::Isometry3d::Identity(),
      Eigen::Vector3d(0.25, 0.25, 0.25),
      makeBoxPose(Eigen::Vector3d(0.3, 0.738, 0.0)));
}

TEST(BoxBox, Rotated90_AlongX)
{
  CollisionResult result;

  Eigen::Isometry3d t1 = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d t2 = Eigen::Isometry3d::Identity();
  t2.rotate(Eigen::AngleAxisd(kPi / 2, Eigen::Vector3d::UnitX()));
  t2.translation() = Eigen::Vector3d(0, 1.5, 0);

  bool collided = collideBoxes(
      Eigen::Vector3d(1, 1, 1), t1, Eigen::Vector3d(1, 1, 1), t2, result);

  EXPECT_TRUE(collided);
  ASSERT_GE(result.numContacts(), 1u);
  EXPECT_NEAR(result.getContact(0).depth, 0.5, 1e-10);
}

TEST(BoxBox, Rotated45_Diagonal)
{
  CollisionResult result;

  Eigen::Isometry3d t1 = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d t2 = Eigen::Isometry3d::Identity();
  t2.rotate(Eigen::AngleAxisd(kPi / 4, Eigen::Vector3d::UnitZ()));
  t2.translation() = Eigen::Vector3d(2.2, 0, 0);

  bool collided = collideBoxes(
      Eigen::Vector3d(1, 1, 1), t1, Eigen::Vector3d(1, 1, 1), t2, result);

  double rotatedHalfWidth = std::sqrt(2.0);
  double expectedPenetration = 1.0 + rotatedHalfWidth - 2.2;

  if (expectedPenetration > 0) {
    EXPECT_TRUE(collided);
    ASSERT_GE(result.numContacts(), 1u);
    EXPECT_NEAR(result.getContact(0).depth, expectedPenetration, 1e-6);
  } else {
    EXPECT_FALSE(collided);
  }
}

TEST(BoxBox, BothRotated)
{
  CollisionResult result;

  Eigen::Isometry3d t1 = Eigen::Isometry3d::Identity();
  t1.rotate(Eigen::AngleAxisd(kPi / 4, Eigen::Vector3d::UnitZ()));

  Eigen::Isometry3d t2 = Eigen::Isometry3d::Identity();
  t2.rotate(Eigen::AngleAxisd(-kPi / 4, Eigen::Vector3d::UnitZ()));
  t2.translation() = Eigen::Vector3d(2, 0, 0);

  bool collided = collideBoxes(
      Eigen::Vector3d(1, 1, 1), t1, Eigen::Vector3d(1, 1, 1), t2, result);

  EXPECT_TRUE(collided);
  ASSERT_GE(result.numContacts(), 1u);
}

TEST(BoxBox, SmallBoxes)
{
  CollisionResult result;

  Eigen::Isometry3d t1 = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d t2 = Eigen::Isometry3d::Identity();
  t2.translation() = Eigen::Vector3d(0.0015, 0, 0);

  bool collided = collideBoxes(
      Eigen::Vector3d(0.001, 0.001, 0.001),
      t1,
      Eigen::Vector3d(0.001, 0.001, 0.001),
      t2,
      result);

  EXPECT_TRUE(collided);
  ASSERT_GE(result.numContacts(), 1u);
  EXPECT_NEAR(result.getContact(0).depth, 0.0005, 1e-10);
}

TEST(BoxBox, LargeBoxes)
{
  CollisionResult result;

  Eigen::Isometry3d t1 = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d t2 = Eigen::Isometry3d::Identity();
  t2.translation() = Eigen::Vector3d(1500, 0, 0);

  bool collided = collideBoxes(
      Eigen::Vector3d(1000, 1000, 1000),
      t1,
      Eigen::Vector3d(1000, 1000, 1000),
      t2,
      result);

  EXPECT_TRUE(collided);
  ASSERT_GE(result.numContacts(), 1u);
  EXPECT_NEAR(result.getContact(0).depth, 500.0, 1e-6);
}

TEST(BoxBox, MaxContactsRespected)
{
  CollisionResult result;
  CollisionOption option;
  option.maxNumContacts = 0;

  Eigen::Isometry3d t1 = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d t2 = Eigen::Isometry3d::Identity();
  t2.translation() = Eigen::Vector3d(1.5, 0, 0);

  bool collided = collideBoxes(
      Eigen::Vector3d(1, 1, 1),
      t1,
      Eigen::Vector3d(1, 1, 1),
      t2,
      result,
      option);

  EXPECT_FALSE(collided);
  EXPECT_EQ(result.numContacts(), 0);
}

TEST(BoxBox, NormalDirection)
{
  CollisionResult result;

  Eigen::Isometry3d t1 = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d t2 = Eigen::Isometry3d::Identity();
  t2.translation() = Eigen::Vector3d(1.5, 0, 0);

  bool collided = collideBoxes(
      Eigen::Vector3d(1, 1, 1), t1, Eigen::Vector3d(1, 1, 1), t2, result);

  EXPECT_TRUE(collided);
  ASSERT_GE(result.numContacts(), 1u);

  const auto& normal = result.getContact(0).normal;
  EXPECT_NEAR(std::abs(normal.x()), 1.0, 1e-10);
  EXPECT_NEAR(normal.y(), 0.0, 1e-10);
  EXPECT_NEAR(normal.z(), 0.0, 1e-10);
}

TEST(BoxBox, UsingShapeObjects)
{
  BoxShape box1(Eigen::Vector3d(1, 1, 1));
  BoxShape box2(Eigen::Vector3d(1, 1, 1));

  Eigen::Isometry3d t1 = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d t2 = Eigen::Isometry3d::Identity();
  t2.translation() = Eigen::Vector3d(1.5, 0, 0);

  CollisionResult result;

  bool collided = collideBoxes(box1, t1, box2, t2, result);

  EXPECT_TRUE(collided);
  ASSERT_GE(result.numContacts(), 1u);
  EXPECT_NEAR(result.getContact(0).depth, 0.5, 1e-10);
}

TEST(BoxBox, CubeFacePatchSingleContactAcrossEquivalentOrientations)
{
  constexpr double pi = kPi;
  const double smallCubeHalfExtent = 0.5;
  const BoxSpec rotatedCube{
      Eigen::Vector3d::Constant(smallCubeHalfExtent),
      makeRotationTransform(0.25 * pi, Eigen::Vector3d::UnitY())};

  BoxSpec largeCube{
      Eigen::Vector3d::Constant(1.5), Eigen::Isometry3d::Identity()};
  largeCube.transform.translation() = Eigen::Vector3d(0.0, 0.0, -1.5);

  CollisionOption option;
  option.maxNumContacts = 1;

  const double expectedDepth = std::sqrt(2.0) * smallCubeHalfExtent;
  const double expectedContactZ = -0.5 * std::sqrt(2.0) * smallCubeHalfExtent;

  expectBoxCubeSymmetryCases(
      rotatedCube,
      largeCube,
      expectedDepth,
      option,
      [&](const CollisionResult& result,
          const BoxSpec& boxA,
          const BoxSpec& boxB,
          const Eigen::Vector3d& expectedNormal,
          double depth,
          const char* label) {
        ASSERT_EQ(result.numContacts(), 1u) << label;
        const auto& contact = result.getContact(0);
        EXPECT_TRUE(contact.normal.isApprox(expectedNormal, 1e-12))
            << label << " normal=" << contact.normal.transpose();
        EXPECT_NEAR(contact.depth, depth, 1e-12) << label;
        expectPointInsideBox(contact.position, boxA, 1e-12, label);
        expectPointInsideBox(contact.position, boxB, 1e-12, label);
        EXPECT_GE(contact.position.z(), expectedContactZ - 1e-12) << label;
      });
}

TEST(BoxBox, RotatedFacePatchReductionRespectsContactLimit)
{
  constexpr double pi = kPi;
  const double halfExtent = 0.5;
  const double tilt = 0.125 * pi;

  const BoxSpec tiltedCube{
      Eigen::Vector3d::Constant(halfExtent),
      makeRotationTransform(tilt, Eigen::Vector3d::UnitX())};

  BoxSpec diagonalCube{
      Eigen::Vector3d::Constant(halfExtent),
      makeRotationTransform(0.25 * pi, Eigen::Vector3d::UnitZ())};
  diagonalCube.transform.translation() = Eigen::Vector3d(0.0, 0.0, -0.75);

  CollisionOption option;
  option.maxNumContacts = 4;

  const double expectedDepth
      = std::sqrt(2.0) * halfExtent * std::cos(0.25 * pi - tilt) - 0.25;
  const double expectedContactZ = -0.25 - 0.5 * expectedDepth;

  expectBoxCubeSymmetryCases(
      tiltedCube,
      diagonalCube,
      expectedDepth,
      option,
      [&](const CollisionResult& result,
          const BoxSpec& boxA,
          const BoxSpec& boxB,
          const Eigen::Vector3d& expectedNormal,
          double depth,
          const char* label) {
        ASSERT_GE(result.numContacts(), 1u) << label;
        ASSERT_LE(result.numContacts(), option.maxNumContacts) << label;
        for (std::size_t i = 0; i < result.numContacts(); ++i) {
          const auto& contact = result.getContact(i);
          EXPECT_TRUE(contact.normal.isApprox(expectedNormal, 1e-12))
              << label << " normal=" << contact.normal.transpose();
          EXPECT_NEAR(contact.depth, depth, 1e-12) << label;
          expectPointInsideBox(contact.position, boxA, 1e-2, label);
          expectPointInsideBox(contact.position, boxB, 1e-2, label);
          EXPECT_GE(contact.position.z(), expectedContactZ - 1e-12) << label;
          EXPECT_LE(contact.position.x(), halfExtent + 1e-12) << label;
          EXPECT_GE(contact.position.x(), -halfExtent - 1e-12) << label;
        }
      });
}

TEST(BoxBox, EdgeEdgeContactAcrossEquivalentCubeOrientations)
{
  constexpr double pi = kPi;
  const double halfExtent = 0.5;
  const double expectedDepth = 0.1;
  const Eigen::Vector3d expectedNormal
      = Eigen::Vector3d(std::sqrt(0.5), 0.0, std::sqrt(0.5));

  BoxSpec tiltedCube{
      Eigen::Vector3d::Constant(halfExtent),
      makeRotationTransform(-0.25 * pi, Eigen::Vector3d::UnitY())
          * makeRotationTransform(0.25 * pi, Eigen::Vector3d::UnitZ())};
  tiltedCube.transform.translation()
      = expectedNormal * (2.0 * halfExtent * std::sqrt(2.0) - expectedDepth);

  const BoxSpec axisAlignedCube{
      Eigen::Vector3d::Constant(halfExtent), Eigen::Isometry3d::Identity()};

  CollisionOption option;
  option.maxNumContacts = 4;

  const double expectedContactDistance
      = halfExtent * std::sqrt(2.0) - 0.5 * expectedDepth;
  const Eigen::Vector3d expectedPosition
      = expectedNormal * expectedContactDistance;

  for (const auto& cubeOrientation : makeCubeEquivalentOrientations()) {
    BoxSpec posedAxisAlignedCube = axisAlignedCube;
    posedAxisAlignedCube.transform
        = axisAlignedCube.transform * cubeOrientation;

    CollisionResult result;
    ASSERT_TRUE(collideBoxes(
        tiltedCube.halfExtents,
        tiltedCube.transform,
        posedAxisAlignedCube.halfExtents,
        posedAxisAlignedCube.transform,
        result,
        option));
    ASSERT_EQ(result.numManifolds(), 1u);
    EXPECT_EQ(result.getManifold(0).getType(), ContactType::Edge);
    ASSERT_EQ(result.numContacts(), 1u);
    EXPECT_TRUE(result.getContact(0).normal.isApprox(expectedNormal, 1e-12))
        << "normal=" << result.getContact(0).normal.transpose();
    EXPECT_NEAR(result.getContact(0).depth, expectedDepth, 1e-12);
    EXPECT_TRUE(result.getContact(0).position.isApprox(expectedPosition, 1e-12))
        << "position=" << result.getContact(0).position.transpose();

    CollisionResult swappedResult;
    ASSERT_TRUE(collideBoxes(
        posedAxisAlignedCube.halfExtents,
        posedAxisAlignedCube.transform,
        tiltedCube.halfExtents,
        tiltedCube.transform,
        swappedResult,
        option));
    ASSERT_EQ(swappedResult.numManifolds(), 1u);
    EXPECT_EQ(swappedResult.getManifold(0).getType(), ContactType::Edge);
    ASSERT_EQ(swappedResult.numContacts(), 1u);
    EXPECT_TRUE(
        swappedResult.getContact(0).normal.isApprox(-expectedNormal, 1e-12))
        << "normal=" << swappedResult.getContact(0).normal.transpose();
    EXPECT_NEAR(swappedResult.getContact(0).depth, expectedDepth, 1e-12);
    EXPECT_TRUE(
        swappedResult.getContact(0).position.isApprox(expectedPosition, 1e-12))
        << "position=" << swappedResult.getContact(0).position.transpose();
  }
}

TEST(BoxBox, RotatedSmallBoxOnLargeGroundHasLocalContactPoint)
{
  const Eigen::Vector3d boxHalfExtents(0.15, 0.15, 0.15);
  const Eigen::Vector3d groundHalfExtents(5.0, 5.0, 0.05);

  Eigen::Isometry3d boxTf = Eigen::Isometry3d::Identity();
  boxTf.linear() = (Eigen::AngleAxisd(0.4, Eigen::Vector3d::UnitX())
                    * Eigen::AngleAxisd(-0.7, Eigen::Vector3d::UnitY())
                    * Eigen::AngleAxisd(0.2, Eigen::Vector3d::UnitZ()))
                       .toRotationMatrix();

  const Eigen::Vector3d zAxis = Eigen::Vector3d::UnitZ();
  const double boxRadiusZ
      = boxHalfExtents.x() * std::abs(zAxis.dot(boxTf.linear().col(0)))
        + boxHalfExtents.y() * std::abs(zAxis.dot(boxTf.linear().col(1)))
        + boxHalfExtents.z() * std::abs(zAxis.dot(boxTf.linear().col(2)));
  const double groundTop = groundHalfExtents.z();
  boxTf.translation()
      = Eigen::Vector3d(0.0, 0.0, groundTop + boxRadiusZ - 0.01);

  Eigen::Isometry3d groundTf = Eigen::Isometry3d::Identity();

  auto closestLocalContact
      = [](const CollisionResult& result, double normalSign) {
          const ContactPoint* closest = nullptr;
          double closestDistance = std::numeric_limits<double>::infinity();
          std::size_t countedContacts = 0;
          for (const auto& manifold : result.getManifolds()) {
            for (const auto& contact : manifold.getContacts()) {
              ++countedContacts;
              EXPECT_TRUE(contact.position.allFinite());
              EXPECT_TRUE(contact.normal.allFinite());
              EXPECT_NEAR(contact.depth, 0.01, 1e-9);
              EXPECT_GT(contact.normal.z() * normalSign, 0.25)
                  << "position=" << contact.position.transpose()
                  << " normal=" << contact.normal.transpose()
                  << " depth=" << contact.depth;

              const double distance = contact.position.head<2>().norm();
              if (distance < closestDistance) {
                closestDistance = distance;
                closest = &contact;
              }
            }
          }

          EXPECT_EQ(countedContacts, result.numContacts());
          EXPECT_GE(countedContacts, 1u);
          if (closest != nullptr) {
            EXPECT_LT(closestDistance, 0.3)
                << "closest=" << closest->position.transpose()
                << " normal=" << closest->normal.transpose();
          } else {
            EXPECT_LT(closestDistance, 0.3);
          }
          return (closest != nullptr) ? *closest : ContactPoint{};
        };

  CollisionResult result;
  result.clear();
  EXPECT_TRUE(
      collideBoxes(boxHalfExtents, boxTf, groundHalfExtents, groundTf, result));
  const auto contact = closestLocalContact(result, 1.0);

  CollisionResult swappedResult;
  swappedResult.clear();
  EXPECT_TRUE(collideBoxes(
      groundHalfExtents, groundTf, boxHalfExtents, boxTf, swappedResult));
  const auto swappedContact = closestLocalContact(swappedResult, -1.0);

  ASSERT_GE(result.numContacts(), 1u);
  ASSERT_GE(swappedResult.numContacts(), 1u);
  EXPECT_NEAR(contact.depth, swappedContact.depth, 1e-9);
  EXPECT_LT(contact.normal.dot(swappedContact.normal), -0.99);
}

TEST(BoxBox, RotatedBoxOnFlatGroundEmitsFacePatch)
{
  const Eigen::Vector3d boxHalfExtents(0.15, 0.15, 0.15);
  const Eigen::Vector3d groundHalfExtents(5.0, 5.0, 0.05);

  Eigen::Isometry3d boxTf = Eigen::Isometry3d::Identity();
  boxTf.linear() = (Eigen::AngleAxisd(0.005, Eigen::Vector3d::UnitX())
                    * Eigen::AngleAxisd(-0.004, Eigen::Vector3d::UnitY())
                    * Eigen::AngleAxisd(0.5, Eigen::Vector3d::UnitZ()))
                       .toRotationMatrix();

  const Eigen::Vector3d zAxis = Eigen::Vector3d::UnitZ();
  const double boxRadiusZ
      = boxHalfExtents.x() * std::abs(zAxis.dot(boxTf.linear().col(0)))
        + boxHalfExtents.y() * std::abs(zAxis.dot(boxTf.linear().col(1)))
        + boxHalfExtents.z() * std::abs(zAxis.dot(boxTf.linear().col(2)));
  const double groundTop = groundHalfExtents.z();
  boxTf.translation()
      = Eigen::Vector3d(0.0, 0.0, groundTop + boxRadiusZ - 0.01);

  Eigen::Isometry3d groundTf = Eigen::Isometry3d::Identity();

  CollisionOption option;
  option.maxNumContacts = 8;

  CollisionResult result;
  ASSERT_TRUE(collideBoxes(
      boxHalfExtents, boxTf, groundHalfExtents, groundTf, result, option));
  ASSERT_GE(result.numContacts(), 3u);

  Eigen::Vector2d minPoint(
      std::numeric_limits<double>::infinity(),
      std::numeric_limits<double>::infinity());
  Eigen::Vector2d maxPoint(
      -std::numeric_limits<double>::infinity(),
      -std::numeric_limits<double>::infinity());

  for (const auto& manifold : result.getManifolds()) {
    for (const auto& contact : manifold.getContacts()) {
      EXPECT_GT(contact.normal.z(), 0.9)
          << "position=" << contact.position.transpose()
          << " normal=" << contact.normal.transpose()
          << " depth=" << contact.depth;
      EXPECT_GE(contact.depth, 0.0);
      minPoint = minPoint.cwiseMin(contact.position.head<2>());
      maxPoint = maxPoint.cwiseMax(contact.position.head<2>());
    }
  }

  const double patchArea
      = (maxPoint.x() - minPoint.x()) * (maxPoint.y() - minPoint.y());
  EXPECT_GT(patchArea, 0.01)
      << "min=" << minPoint.transpose() << " max=" << maxPoint.transpose();
}

TEST(BoxBox, SatAxisStableForNearFaceBoxGroundPerturbations)
{
  const Eigen::Vector3d boxHalfExtents(0.15, 0.15, 0.15);
  const Eigen::Vector3d groundHalfExtents(5.0, 5.0, 0.05);
  const Eigen::Isometry3d groundTf = Eigen::Isometry3d::Identity();
  const dart::collision::native::box_box::BoxData ground{
      groundTf.translation(), groundHalfExtents, groundTf.rotation()};

  const double groundTop = groundHalfExtents.z();
  const double penetration = 0.01;
  const double yaw = 0.5;

  for (int i = -10; i <= 10; ++i) {
    const double perturbation = static_cast<double>(i) * 1e-4;
    Eigen::Isometry3d boxTf = Eigen::Isometry3d::Identity();
    boxTf.linear()
        = (Eigen::AngleAxisd(0.004 + perturbation, Eigen::Vector3d::UnitX())
           * Eigen::AngleAxisd(
               -0.003 + 0.5 * perturbation, Eigen::Vector3d::UnitY())
           * Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()))
              .toRotationMatrix();

    const Eigen::Vector3d zAxis = Eigen::Vector3d::UnitZ();
    const double boxRadiusZ
        = boxHalfExtents.x() * std::abs(zAxis.dot(boxTf.linear().col(0)))
          + boxHalfExtents.y() * std::abs(zAxis.dot(boxTf.linear().col(1)))
          + boxHalfExtents.z() * std::abs(zAxis.dot(boxTf.linear().col(2)));
    boxTf.translation()
        = Eigen::Vector3d(0.0, 0.0, groundTop + boxRadiusZ - penetration);

    const dart::collision::native::box_box::BoxData box{
        boxTf.translation(), boxHalfExtents, boxTf.rotation()};
    dart::collision::native::box_box::SatResult sat;
    ASSERT_TRUE(
        dart::collision::native::box_box::computeBoxBoxSat(box, ground, sat));
    EXPECT_EQ(sat.axisType, dart::collision::native::box_box::SatAxisType::Face)
        << "perturbation=" << perturbation << " axisIndex=" << sat.axisIndex
        << " normal=" << sat.normal.transpose();
  }
}

TEST(BoxBox, EdgeEdgeTiltedContactEmitsSingleSupportPoint)
{
  const Eigen::Vector3d halfExtents(2.0, 0.08, 0.08);
  const Eigen::Isometry3d box1Tf = Eigen::Isometry3d::Identity();
  const dart::collision::native::box_box::BoxData box1{
      box1Tf.translation(), halfExtents, box1Tf.rotation()};

  Eigen::Isometry3d box2Tf = Eigen::Isometry3d::Identity();
  box2Tf.linear() << 0.5195097239462093, -0.22755680602979128,
      -0.8236064270966159, -0.06965685052231302, 0.9493991403131945,
      -0.30625021722095036, 0.8516205551186782, 0.21646979558840784,
      0.47737119487592405;
  box2Tf.translation() = Eigen::Vector3d(
      -0.038403205753238676, -0.1346570769876879, -0.14933261157368102);
  const dart::collision::native::box_box::BoxData box2{
      box2Tf.translation(), halfExtents, box2Tf.rotation()};

  dart::collision::native::box_box::SatResult sat;
  ASSERT_TRUE(
      dart::collision::native::box_box::computeBoxBoxSat(box1, box2, sat));
  EXPECT_EQ(sat.axisType, dart::collision::native::box_box::SatAxisType::Edge);
  EXPECT_EQ(sat.axisIndex, 6);
  EXPECT_TRUE(sat.normal.allFinite());
  EXPECT_NEAR(sat.normal.norm(), 1.0, 1e-12);

  CollisionResult result;
  CollisionOption option;
  option.maxNumContacts = 8;
  ASSERT_TRUE(
      collideBoxes(halfExtents, box1Tf, halfExtents, box2Tf, result, option));
  ASSERT_EQ(result.numManifolds(), 1u);
  EXPECT_EQ(result.getManifold(0).getType(), ContactType::Edge);
  ASSERT_EQ(result.numContacts(), 1u);
  const auto& contact = result.getContact(0);
  EXPECT_TRUE(contact.position.allFinite());
  EXPECT_TRUE(contact.normal.allFinite());
  EXPECT_GT(contact.depth, 0.0);
  EXPECT_NEAR(contact.normal.norm(), 1.0, 1e-12);
}

TEST(BoxBox, FaceVertexContactAgainstGroundFace)
{
  const Eigen::Vector3d boxHalfExtents(0.20, 0.30, 0.40);
  const Eigen::Vector3d groundHalfExtents(5.0, 5.0, 0.05);

  Eigen::Isometry3d boxTf = Eigen::Isometry3d::Identity();
  boxTf.linear() = (Eigen::AngleAxisd(0.63, Eigen::Vector3d::UnitX())
                    * Eigen::AngleAxisd(-0.52, Eigen::Vector3d::UnitY())
                    * Eigen::AngleAxisd(0.41, Eigen::Vector3d::UnitZ()))
                       .toRotationMatrix();

  Eigen::Vector3d lowestLocalVertex = Eigen::Vector3d::Zero();
  double lowestVertexZ = std::numeric_limits<double>::infinity();
  for (const double sx : {-1.0, 1.0}) {
    for (const double sy : {-1.0, 1.0}) {
      for (const double sz : {-1.0, 1.0}) {
        const Eigen::Vector3d localVertex(
            sx * boxHalfExtents.x(),
            sy * boxHalfExtents.y(),
            sz * boxHalfExtents.z());
        const Eigen::Vector3d rotatedVertex = boxTf.linear() * localVertex;
        if (rotatedVertex.z() < lowestVertexZ) {
          lowestVertexZ = rotatedVertex.z();
          lowestLocalVertex = localVertex;
        }
      }
    }
  }

  const double penetration = 0.01;
  const double groundTop = groundHalfExtents.z();
  boxTf.translation()
      = Eigen::Vector3d(0.15, -0.10, groundTop - lowestVertexZ - penetration);
  const Eigen::Vector3d expectedVertex = boxTf * lowestLocalVertex;

  Eigen::Isometry3d groundTf = Eigen::Isometry3d::Identity();

  CollisionResult result;
  CollisionOption option;
  option.maxNumContacts = 8;
  ASSERT_TRUE(collideBoxes(
      boxHalfExtents, boxTf, groundHalfExtents, groundTf, result, option));
  ASSERT_GE(result.numContacts(), 1u);
  ASSERT_LE(result.numContacts(), 4u);

  double closestDistance = std::numeric_limits<double>::infinity();
  for (const auto& manifold : result.getManifolds()) {
    for (const auto& contact : manifold.getContacts()) {
      EXPECT_TRUE(contact.position.allFinite());
      EXPECT_TRUE(contact.normal.allFinite());
      EXPECT_GT(contact.normal.z(), 0.6)
          << "position=" << contact.position.transpose()
          << " normal=" << contact.normal.transpose();
      EXPECT_GE(contact.depth, 0.0);
      closestDistance = std::min(
          closestDistance, (contact.position - expectedVertex).norm());
    }
  }

  EXPECT_LT(closestDistance, 0.05)
      << "expected vertex=" << expectedVertex.transpose();
}

TEST(BoxBox, SatAxisFaceBiasTiePrefersFace)
{
  const Eigen::Vector3d halfExtents(1.0, 1.0, 1.0);
  const Eigen::Isometry3d box1Tf = Eigen::Isometry3d::Identity();
  const dart::collision::native::box_box::BoxData box1{
      box1Tf.translation(), halfExtents, box1Tf.rotation()};

  Eigen::Isometry3d box2Tf = Eigen::Isometry3d::Identity();
  box2Tf.linear() << 0.9931958804661412, 0.11587467619999768,
      -0.011619055066231654, -0.11301993020561968, 0.9831403341293836,
      0.14374136072919544, 0.02807914531112528, -0.14145014253619373,
      0.9895470775941321;
  box2Tf.translation() = Eigen::Vector3d(
      1.9109651304887858, -0.009774674473954012, -0.06826065396649084);
  const dart::collision::native::box_box::BoxData box2{
      box2Tf.translation(), halfExtents, box2Tf.rotation()};

  dart::collision::native::box_box::SatResult sat;
  ASSERT_TRUE(
      dart::collision::native::box_box::computeBoxBoxSat(box1, box2, sat));
  EXPECT_EQ(sat.axisType, dart::collision::native::box_box::SatAxisType::Face);
  EXPECT_EQ(sat.axisIndex, 0);
  EXPECT_TRUE(sat.normal.allFinite());
  EXPECT_NEAR(sat.normal.norm(), 1.0, 1e-12);
}

TEST(BoxBox, SatSkipsDegenerateCrossAxes)
{
  const Eigen::Vector3d halfExtents(1.0, 1.0, 1.0);
  const Eigen::Isometry3d box1Tf = Eigen::Isometry3d::Identity();
  const dart::collision::native::box_box::BoxData box1{
      box1Tf.translation(), halfExtents, box1Tf.rotation()};

  Eigen::Isometry3d box2Tf = Eigen::Isometry3d::Identity();
  box2Tf.rotate(Eigen::AngleAxisd(1e-8, Eigen::Vector3d::UnitZ()));
  box2Tf.translation() = Eigen::Vector3d(1.9, 0.0, 0.0);
  const dart::collision::native::box_box::BoxData box2{
      box2Tf.translation(), halfExtents, box2Tf.rotation()};

  dart::collision::native::box_box::SatResult sat;
  ASSERT_TRUE(
      dart::collision::native::box_box::computeBoxBoxSat(box1, box2, sat));
  EXPECT_EQ(sat.axisType, dart::collision::native::box_box::SatAxisType::Face);
  EXPECT_TRUE(sat.normal.allFinite());
  EXPECT_NEAR(sat.normal.norm(), 1.0, 1e-12);
}

TEST(BoxBox, Determinism)
{
  std::vector<ContactPoint> contacts;

  for (int i = 0; i < 100; ++i) {
    CollisionResult result;

    Eigen::Isometry3d t1 = Eigen::Isometry3d::Identity();
    t1.rotate(Eigen::AngleAxisd(0.123, Eigen::Vector3d(1, 2, 3).normalized()));
    t1.translation() = Eigen::Vector3d(0.1, 0.2, 0.3);

    Eigen::Isometry3d t2 = Eigen::Isometry3d::Identity();
    t2.rotate(Eigen::AngleAxisd(0.456, Eigen::Vector3d(3, 2, 1).normalized()));
    t2.translation() = Eigen::Vector3d(1.5, 0.5, 0.25);

    bool collided = collideBoxes(
        Eigen::Vector3d(1.1, 0.9, 1.2),
        t1,
        Eigen::Vector3d(0.8, 1.3, 1.0),
        t2,
        result);

    EXPECT_TRUE(collided);
    ASSERT_GE(result.numContacts(), 1u);
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

TEST(BoxBoxBatch, boxbox_batch_determinism_vs_single)
{
  BoxShape boxA(Eigen::Vector3d(1.1, 0.9, 0.7));
  BoxShape boxB(Eigen::Vector3d(0.8, 1.0, 0.6));

  std::mt19937 rng(1701u);
  std::uniform_real_distribution<double> offset(-0.15, 0.15);
  std::uniform_real_distribution<double> angle(-0.35, 0.35);

  std::vector<BoxPair> pairs;
  pairs.reserve(100);
  for (int i = 0; i < 100; ++i) {
    const Eigen::Isometry3d tfA = makeBoxBatchTransform(
        0.05 * offset(rng), offset(rng), offset(rng), angle(rng));
    const Eigen::Isometry3d tfB = makeBoxBatchTransform(
        0.9 + offset(rng), offset(rng), offset(rng), angle(rng));
    pairs.push_back(BoxPair{&boxA, &boxB, tfA, tfB});
  }

  CollisionOption option;
  option.maxNumContacts = 8;
  std::vector<CollisionResult> batchResults(pairs.size());
  collideBoxesBatch(pairs, batchResults, option);

  for (std::size_t i = 0; i < pairs.size(); ++i) {
    CollisionResult singleResult;
    ASSERT_TRUE(collideBoxes(
        pairs[i].shapeA->getHalfExtents(),
        pairs[i].tfA,
        pairs[i].shapeB->getHalfExtents(),
        pairs[i].tfB,
        singleResult,
        option));
    expectCollisionResultExactlyEqual(singleResult, batchResults[i]);
  }
}

// DART 6 supplementary coverage tests (not ported from DART 7)
TEST(BoxBox, BinaryCheckShapeOverloadLeavesResultEmpty)
{
  BoxShape boxA(Eigen::Vector3d::Ones());
  BoxShape boxB(Eigen::Vector3d::Ones());

  Eigen::Isometry3d tfA = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d tfB = Eigen::Isometry3d::Identity();
  tfB.translation() = Eigen::Vector3d(1.5, 0.25, 0.0);

  CollisionResult result;
  const CollisionOption option = CollisionOption::binaryCheck();
  EXPECT_TRUE(collideBoxes(boxA, tfA, boxB, tfB, result, option));
  EXPECT_EQ(result.numContacts(), 0u);
  EXPECT_EQ(result.numManifolds(), 0u);

  tfB.translation() = Eigen::Vector3d(3.0, 0.25, 0.0);
  EXPECT_FALSE(collideBoxes(boxA, tfA, boxB, tfB, result, option));
  EXPECT_EQ(result.numContacts(), 0u);
  EXPECT_EQ(result.numManifolds(), 0u);
}

TEST(BoxBox, ContactReductionDeduplicatesAndBreaksDistanceTiesByDepth)
{
  std::vector<box_box::ContactCandidate> candidates{
      {Eigen::Vector3d::Zero(), 0.1},
      {Eigen::Vector3d(1e-8, 0.0, 0.0), 0.5},
      {Eigen::Vector3d(2.0, 0.0, 0.0), 0.2},
      {Eigen::Vector3d(-2.0, 0.0, 0.0), 0.3},
  };

  const auto none = box_box::reduceContactCandidates(candidates, 0u);
  EXPECT_TRUE(none.empty());

  const auto reduced = box_box::reduceContactCandidates(candidates, 2u);
  ASSERT_EQ(reduced.size(), 2u);

  bool keptMergedDeepest = false;
  bool keptTieBreaker = false;
  for (const auto& candidate : reduced) {
    if (candidate.position.isApprox(Eigen::Vector3d::Zero(), 1e-7)) {
      EXPECT_DOUBLE_EQ(candidate.depth, 0.5);
      keptMergedDeepest = true;
    }
    if (candidate.position.isApprox(Eigen::Vector3d(-2.0, 0.0, 0.0), 1e-12)) {
      EXPECT_DOUBLE_EQ(candidate.depth, 0.3);
      keptTieBreaker = true;
    }
  }
  EXPECT_TRUE(keptMergedDeepest);
  EXPECT_TRUE(keptTieBreaker);
}
