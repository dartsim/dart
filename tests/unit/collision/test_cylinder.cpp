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

#include <dart/collision/native/narrow_phase/cylinder_collision.hpp>
#include <dart/collision/native/shapes/shape.hpp>

#include <gtest/gtest.h>

#include <array>
#include <numbers>
#include <random>
#include <stdexcept>
#include <tuple>
#include <vector>

#include <cmath>

using namespace dart::collision::native;

namespace {

Eigen::Isometry3d makeCylinderBatchTransform(
    double x, double y, double z, double angle)
{
  Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
  tf.linear() = (Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitX())
                 * Eigen::AngleAxisd(0.5 * angle, Eigen::Vector3d::UnitY()))
                    .toRotationMatrix();
  tf.translation() = Eigen::Vector3d(x, y, z);
  return tf;
}

Eigen::Isometry3d makeTranslation(const Eigen::Vector3d& translation)
{
  Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
  tf.translation() = translation;
  return tf;
}

Eigen::Isometry3d makeCylinderCylinderCommonFrame()
{
  return makeCylinderBatchTransform(8.40188, 3.94383, 7.83099, 0.37);
}

Eigen::Isometry3d makeSphereTransformInCylinderFrame(
    const Eigen::Vector3d& translation, double angle)
{
  Eigen::Isometry3d tf = makeTranslation(translation);
  tf.linear()
      = Eigen::AngleAxisd(angle, Eigen::Vector3d(1.0, -2.0, 3.0).normalized())
            .toRotationMatrix();
  return tf;
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

void expectEqualCylinderBoundaryCase(
    const char* name,
    const Eigen::Isometry3d& worldFromCase,
    const Eigen::Vector3d& secondCylinderCenterInCase,
    bool expectedHit,
    double expectedDepth)
{
  SCOPED_TRACE(name);
  CylinderShape cylinder1(5.0, 10.0);
  CylinderShape cylinder2(5.0, 10.0);

  const Eigen::Isometry3d tf1 = worldFromCase;
  const Eigen::Isometry3d tf2
      = worldFromCase * makeTranslation(secondCylinderCenterInCase);

  CollisionResult result;
  const bool hit = collideCylinders(cylinder1, tf1, cylinder2, tf2, result);
  EXPECT_EQ(hit, expectedHit);

  if (!expectedHit) {
    EXPECT_EQ(result.numContacts(), 0u);
    return;
  }

  ASSERT_GE(result.numContacts(), 1u);
  const auto& contact = result.getContact(0);
  EXPECT_NEAR(contact.depth, expectedDepth, 1e-7);
  EXPECT_TRUE(contact.position.allFinite());
  EXPECT_TRUE(contact.normal.allFinite());
}

Eigen::Isometry3d makeRotatedCylinderPose(
    const Eigen::Vector3d& translation,
    const Eigen::Vector3d& axis,
    double angle)
{
  Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
  tf.linear() = Eigen::AngleAxisd(angle, axis.normalized()).toRotationMatrix();
  tf.translation() = translation;
  return tf;
}

void expectUnequalCylinderHit(
    const char* name,
    const Eigen::Isometry3d& tf1,
    const Eigen::Isometry3d& tf2)
{
  SCOPED_TRACE(name);
  CylinderShape cylinder1(0.35, 0.5);
  CylinderShape cylinder2(0.5, 1.0);

  CollisionResult result;
  const bool hit = collideCylinders(cylinder1, tf1, cylinder2, tf2, result);
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

void expectCylinderBoxFiniteHit(
    const char* name,
    const Eigen::Isometry3d& tfCylinder,
    const Eigen::Isometry3d& tfBox)
{
  SCOPED_TRACE(name);
  CylinderShape cylinder(0.4, 0.7);
  BoxShape box(Eigen::Vector3d(0.25, 0.5, 0.75));

  CollisionResult result;
  const bool hit = collideCylinderBox(cylinder, tfCylinder, box, tfBox, result);
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

TEST(CylinderCylinder, NoCollision)
{
  CylinderShape cyl1(0.5, 2.0);
  CylinderShape cyl2(0.5, 2.0);

  Eigen::Isometry3d tf1 = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d tf2 = Eigen::Isometry3d::Identity();
  tf2.translation() = Eigen::Vector3d(3.0, 0, 0);

  CollisionResult result;
  bool collided = collideCylinders(cyl1, tf1, cyl2, tf2, result);

  EXPECT_FALSE(collided);
  EXPECT_EQ(result.numContacts(), 0u);
}

TEST(CylinderCylinder, ParallelOverlap)
{
  CylinderShape cyl1(0.5, 2.0);
  CylinderShape cyl2(0.5, 2.0);

  Eigen::Isometry3d tf1 = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d tf2 = Eigen::Isometry3d::Identity();
  tf2.translation() = Eigen::Vector3d(0.8, 0, 0);

  CollisionResult result;
  bool collided = collideCylinders(cyl1, tf1, cyl2, tf2, result);

  EXPECT_TRUE(collided);
  EXPECT_GE(result.numContacts(), 1u);
  EXPECT_LT(result.getContact(0).normal.x(), -0.9);
  EXPECT_NEAR(result.getContact(0).depth, 0.2, 1e-9);
}

TEST(CylinderCylinder, StackedOnTop)
{
  CylinderShape cyl1(0.5, 2.0);
  CylinderShape cyl2(0.5, 2.0);

  Eigen::Isometry3d tf1 = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d tf2 = Eigen::Isometry3d::Identity();
  tf2.translation() = Eigen::Vector3d(0, 0, 1.8);

  CollisionResult result;
  bool collided = collideCylinders(cyl1, tf1, cyl2, tf2, result);

  EXPECT_TRUE(collided);
  EXPECT_GE(result.numContacts(), 1u);
  EXPECT_LT(result.getContact(0).normal.z(), -0.9);
  EXPECT_NEAR(result.getContact(0).depth, 0.2, 1e-9);
  EXPECT_NEAR(result.getContact(0).position.z(), 0.9, 1e-9);
}

TEST(CylinderCylinder, Perpendicular)
{
  CylinderShape cyl1(0.5, 2.0);
  CylinderShape cyl2(0.5, 2.0);

  Eigen::Isometry3d tf1 = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d tf2 = Eigen::Isometry3d::Identity();
  tf2.linear() = Eigen::AngleAxisd(
                     std::numbers::pi_v<double> / 2, Eigen::Vector3d::UnitY())
                     .toRotationMatrix();
  tf2.translation() = Eigen::Vector3d(0.8, 0, 0);

  CollisionResult result;
  bool collided = collideCylinders(cyl1, tf1, cyl2, tf2, result);

  EXPECT_TRUE(collided);
  EXPECT_GE(result.numContacts(), 1u);
}

TEST(CylinderCylinder, ContactLimitAndObliqueSeparation)
{
  CylinderShape cylinder(0.5, 2.0);

  CollisionOption noContacts;
  noContacts.maxNumContacts = 0;
  CollisionResult limited;
  EXPECT_FALSE(collideCylinders(
      cylinder,
      Eigen::Isometry3d::Identity(),
      cylinder,
      Eigen::Isometry3d::Identity(),
      limited,
      noContacts));

  Eigen::Isometry3d oblique = Eigen::Isometry3d::Identity();
  oblique.linear()
      = Eigen::AngleAxisd(
            std::numbers::pi_v<double> / 2.0, Eigen::Vector3d::UnitY())
            .toRotationMatrix();
  oblique.translation() = Eigen::Vector3d(3.0, 0.0, 0.0);

  CollisionResult separated;
  EXPECT_FALSE(collideCylinders(
      cylinder, Eigen::Isometry3d::Identity(), cylinder, oblique, separated));
}

TEST(CylinderCylinder, DegenerateAxisClosestPointBranchesRemainFinite)
{
  const CylinderShape pointCylinder(0.3, 0.0);
  const CylinderShape segmentCylinder(0.3, 1.0);
  const Eigen::Isometry3d identity = Eigen::Isometry3d::Identity();
  const Eigen::Isometry3d tilted = makeRotatedCylinderPose(
      Eigen::Vector3d(0.2, 0.0, 0.0),
      Eigen::Vector3d::UnitY(),
      std::numbers::pi_v<double> / 2.0);

  for (const auto& [shapeA, tfA, shapeB, tfB] :
       {std::tuple{&pointCylinder, identity, &pointCylinder, tilted},
        std::tuple{&pointCylinder, identity, &segmentCylinder, tilted},
        std::tuple{&segmentCylinder, identity, &pointCylinder, tilted}}) {
    CollisionResult result;
    const bool collided = collideCylinders(*shapeA, tfA, *shapeB, tfB, result);

    ASSERT_TRUE(collided);
    ASSERT_GE(result.numContacts(), 1u);
    EXPECT_TRUE(result.getContact(0).position.allFinite());
    EXPECT_TRUE(result.getContact(0).normal.allFinite());
    EXPECT_TRUE(std::isfinite(result.getContact(0).depth));
  }
}

TEST(CylinderCylinder, EqualRadiusBoundaryAndSeparationAcrossFrames)
{
  const std::array<Eigen::Isometry3d, 2> worldFromCases{
      Eigen::Isometry3d::Identity(), makeCylinderCylinderCommonFrame()};

  for (const auto& worldFromCase : worldFromCases) {
    expectEqualCylinderBoundaryCase(
        "coincident-cylinders",
        worldFromCase,
        Eigen::Vector3d::Zero(),
        true,
        10.0);
    expectEqualCylinderBoundaryCase(
        "shallow-side-penetration",
        worldFromCase,
        Eigen::Vector3d(9.9, 0.0, 0.0),
        true,
        0.1);
    expectEqualCylinderBoundaryCase(
        "side-touching",
        worldFromCase,
        Eigen::Vector3d(10.0, 0.0, 0.0),
        true,
        0.0);
    expectEqualCylinderBoundaryCase(
        "side-separated",
        worldFromCase,
        Eigen::Vector3d(10.01, 0.0, 0.0),
        false,
        0.0);
  }
}

TEST(CylinderCylinder, AxisSweepTransitionsAcrossCoordinates)
{
  CylinderShape cylinder1(0.35, 0.5);
  CylinderShape cylinder2(0.5, 1.0);
  const std::array<Eigen::Vector3d, 3> axes{
      Eigen::Vector3d::UnitX(),
      Eigen::Vector3d::UnitY(),
      Eigen::Vector3d::UnitZ()};

  for (const auto& axis : axes) {
    const double contactThreshold
        = axis.z() == 0.0
              ? cylinder1.getRadius() + cylinder2.getRadius()
              : 0.5 * (cylinder1.getHeight() + cylinder2.getHeight());

    for (int i = 0; i < 100; ++i) {
      const double offset = -5.0 + 0.1 * i;
      SCOPED_TRACE(offset);

      CollisionResult result;
      const bool hit = collideCylinders(
          cylinder1,
          Eigen::Isometry3d::Identity(),
          cylinder2,
          makeTranslation(offset * axis),
          result);
      const bool expectedHit = std::abs(offset) <= contactThreshold;
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
      EXPECT_NEAR(contact.normal.norm(), 1.0, 1e-8);
    }
  }
}

TEST(CylinderCylinder, UnequalRadiusPenetrationReportsFiniteContacts)
{
  Eigen::Isometry3d tf1 = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d tf2 = makeTranslation(Eigen::Vector3d(0.0, 0.0, 0.3));
  expectUnequalCylinderHit("axial offset", tf1, tf2);

  tf1 = makeTranslation(Eigen::Vector3d(0.3, 0.1, 0.1));
  tf2 = Eigen::Isometry3d::Identity();
  expectUnequalCylinderHit("translated first cylinder", tf1, tf2);

  tf2 = makeRotatedCylinderPose(
      Eigen::Vector3d::Zero(),
      Eigen::Vector3d(0.0, 1.0, 1.0),
      std::numbers::pi_v<double> / 4.0);
  expectUnequalCylinderHit("tilted second cylinder", tf1, tf2);

  tf2 = makeRotatedCylinderPose(
      Eigen::Vector3d(-0.2, 0.7, 0.2),
      Eigen::Vector3d(0.0, 1.0, 1.0),
      std::numbers::pi_v<double> / 4.0);
  expectUnequalCylinderHit("translated tilted second cylinder", tf1, tf2);

  tf2 = makeRotatedCylinderPose(
      Eigen::Vector3d(0.6, -0.7, 0.2),
      Eigen::Vector3d(0.567, 1.2, 1.0),
      std::numbers::pi_v<double> / 4.0);
  expectUnequalCylinderHit("oblique second cylinder", tf1, tf2);

  tf2 = makeRotatedCylinderPose(
      Eigen::Vector3d(0.6, -0.7, 0.2),
      Eigen::Vector3d(-4.567, 1.2, 0.0),
      std::numbers::pi_v<double> / 3.0);
  expectUnequalCylinderHit("steep oblique second cylinder", tf1, tf2);
}

TEST(CylinderCylinderBatch, cylinder_cylinder_batch_determinism_vs_single)
{
  CylinderShape cylinderA(0.45, 1.2);
  CylinderShape cylinderB(0.40, 1.0);

  std::mt19937 rng(161803u);
  std::uniform_real_distribution<double> offset(-0.05, 0.05);
  std::uniform_real_distribution<double> angle(-0.15, 0.15);

  std::vector<CylinderPair> pairs;
  pairs.reserve(100);
  for (int i = 0; i < 100; ++i) {
    const Eigen::Isometry3d tfA = makeCylinderBatchTransform(
        offset(rng), offset(rng), offset(rng), angle(rng));
    const Eigen::Isometry3d tfB = makeCylinderBatchTransform(
        0.45 + offset(rng), offset(rng), offset(rng), angle(rng));

    pairs.push_back(CylinderPair{&cylinderA, &cylinderB, tfA, tfB});
  }

  CollisionOption option;
  std::vector<CollisionResult> batchResults(pairs.size());
  collideCylindersBatch(pairs, batchResults, option);

  for (std::size_t i = 0; i < pairs.size(); ++i) {
    CollisionResult singleResult;
    ASSERT_TRUE(collideCylinders(
        *pairs[i].shapeA,
        pairs[i].tfA,
        *pairs[i].shapeB,
        pairs[i].tfB,
        singleResult,
        option));
    expectCollisionResultExactlyEqual(singleResult, batchResults[i]);
  }
}

TEST(CylinderCylinderBatch, RejectsMalformedInputs)
{
  CylinderShape cylinderA(0.45, 1.2);
  CylinderShape cylinderB(0.40, 1.0);

  const Eigen::Isometry3d tfA = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d tfB = Eigen::Isometry3d::Identity();
  tfB.translation() = Eigen::Vector3d(0.45, 0.0, 0.0);

  const std::vector<CylinderPair> validPairs{
      {&cylinderA, &cylinderB, tfA, tfB}};
  std::vector<CollisionResult> emptyResults;
  EXPECT_THROW(
      collideCylindersBatch(validPairs, emptyResults), std::invalid_argument);

  const std::vector<CylinderPair> nullShapePairs{
      {nullptr, &cylinderB, tfA, tfB}};
  std::vector<CollisionResult> results(1);
  EXPECT_THROW(
      collideCylindersBatch(nullShapePairs, results), std::invalid_argument);
}

TEST(CylinderSphere, NoCollision)
{
  CylinderShape cylinder(0.5, 2.0);
  SphereShape sphere(0.5);

  Eigen::Isometry3d tfCylinder = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d tfSphere = Eigen::Isometry3d::Identity();
  tfSphere.translation() = Eigen::Vector3d(3.0, 0, 0);

  CollisionResult result;
  bool collided
      = collideCylinderSphere(cylinder, tfCylinder, sphere, tfSphere, result);

  EXPECT_FALSE(collided);
  EXPECT_EQ(result.numContacts(), 0u);
}

TEST(CylinderSphere, SphereAtSide)
{
  CylinderShape cylinder(0.5, 2.0);
  SphereShape sphere(0.5);

  Eigen::Isometry3d tfCylinder = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d tfSphere = Eigen::Isometry3d::Identity();
  tfSphere.translation() = Eigen::Vector3d(0.8, 0, 0);

  CollisionResult result;
  bool collided
      = collideCylinderSphere(cylinder, tfCylinder, sphere, tfSphere, result);

  EXPECT_TRUE(collided);
  EXPECT_EQ(result.numContacts(), 1u);
  EXPECT_NEAR(result.getContact(0).depth, 0.2, 1e-6);
}

TEST(CylinderSphere, SphereAtTop)
{
  CylinderShape cylinder(0.5, 2.0);
  SphereShape sphere(0.5);

  Eigen::Isometry3d tfCylinder = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d tfSphere = Eigen::Isometry3d::Identity();
  tfSphere.translation() = Eigen::Vector3d(0, 0, 1.3);

  CollisionResult result;
  bool collided
      = collideCylinderSphere(cylinder, tfCylinder, sphere, tfSphere, result);

  EXPECT_TRUE(collided);
  EXPECT_EQ(result.numContacts(), 1u);
}

TEST(CylinderSphere, SphereAtTopEdge)
{
  CylinderShape cylinder(0.5, 2.0);
  SphereShape sphere(0.5);

  Eigen::Isometry3d tfCylinder = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d tfSphere = Eigen::Isometry3d::Identity();
  tfSphere.translation() = Eigen::Vector3d(0.5, 0, 1.2);

  CollisionResult result;
  bool collided
      = collideCylinderSphere(cylinder, tfCylinder, sphere, tfSphere, result);

  EXPECT_TRUE(collided);
  EXPECT_EQ(result.numContacts(), 1u);
}

TEST(CylinderSphere, SphereSeparatedAcrossFramesAndOrientations)
{
  constexpr double cylinderRadius = 1.2;
  constexpr double cylinderHeight = 0.6;
  constexpr double cylinderHalfHeight = cylinderHeight * 0.5;
  constexpr double sphereRadius = 0.7;

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
  };

  const std::array<Case, 3> cases{{
      {"separated from cap face",
       Eigen::Vector3d(
           cylinderRadius * 0.25,
           cylinderRadius * 0.25,
           cylinderHalfHeight + sphereRadius * 1.1)},
      {"separated from barrel",
       Eigen::Vector3d(0.0, 0.0, cylinderHalfHeight * 0.5)
           - barrelNormalToCylinder * (sphereRadius + cylinderRadius + 0.1)},
      {"separated from barrel edge",
       edgePoint - edgeNormalToCylinder * (sphereRadius + 0.1)},
  }};

  const CylinderShape cylinder(cylinderRadius, cylinderHeight);
  const SphereShape sphere(sphereRadius);
  const std::array<Eigen::Isometry3d, 2> cylinderTransforms{{
      Eigen::Isometry3d::Identity(),
      makeCylinderBatchTransform(1.0, -2.0, 0.5, 0.37),
  }};
  constexpr std::array<double, 2> sphereRotationAngles{
      0.0, std::numbers::pi_v<double> / 5.0};

  for (const auto& testCase : cases) {
    for (const auto& tfCylinder : cylinderTransforms) {
      for (const double sphereRotationAngle : sphereRotationAngles) {
        SCOPED_TRACE(testCase.name);
        CollisionResult result;
        const bool collided = collideCylinderSphere(
            cylinder,
            tfCylinder,
            sphere,
            tfCylinder
                * makeSphereTransformInCylinderFrame(
                    testCase.sphereCenter, sphereRotationAngle),
            result);

        EXPECT_FALSE(collided);
        EXPECT_EQ(result.numContacts(), 0u);
      }
    }
  }
}

TEST(CylinderSphere, SphereContactsAcrossFramesAndOrientations)
{
  constexpr double cylinderRadius = 1.2;
  constexpr double cylinderHeight = 0.6;
  constexpr double cylinderHalfHeight = cylinderHeight * 0.5;
  constexpr double sphereRadius = 0.7;
  constexpr double targetDepth = sphereRadius * 0.25;

  const Eigen::Vector3d capPoint(
      cylinderRadius * 0.25, cylinderRadius * 0.25, cylinderHalfHeight);
  const Eigen::Vector3d capNormalToCylinder = -Eigen::Vector3d::UnitZ();
  const Eigen::Vector3d barrelPoint(
      cylinderRadius, 0.0, cylinderHalfHeight * 0.1);
  const Eigen::Vector3d edgePoint(cylinderRadius, 0.0, cylinderHalfHeight);
  const Eigen::Vector3d edgeNormalFromCylinder = edgePoint.normalized();

  struct Case
  {
    const char* name;
    double cylinderRadius;
    double cylinderHeight;
    Eigen::Vector3d sphereCenter;
    double expectedDepth;
  };

  const std::array<Case, 7> cases{{
      {"external cap face",
       cylinderRadius,
       cylinderHeight,
       capPoint - capNormalToCylinder * (sphereRadius - targetDepth),
       targetDepth},
      {"external barrel",
       cylinderRadius,
       cylinderHeight,
       barrelPoint + Eigen::Vector3d::UnitX() * (sphereRadius - targetDepth),
       targetDepth},
      {"external barrel edge",
       cylinderRadius,
       cylinderHeight,
       edgePoint + edgeNormalFromCylinder * (sphereRadius - targetDepth),
       targetDepth},
      {"internal cap face",
       cylinderRadius,
       cylinderHeight,
       capPoint + capNormalToCylinder * 0.1,
       sphereRadius + 0.1},
      {"internal barrel",
       cylinderRadius,
       cylinderHeight,
       barrelPoint - Eigen::Vector3d::UnitX() * 0.1,
       sphereRadius + 0.1},
      {"origin with cap nearest",
       cylinderHeight * 2.0,
       cylinderHeight,
       Eigen::Vector3d::Zero(),
       sphereRadius + cylinderHalfHeight},
      {"origin with barrel nearest",
       cylinderRadius,
       cylinderRadius * 4.0,
       Eigen::Vector3d::Zero(),
       sphereRadius + cylinderRadius},
  }};

  const SphereShape sphere(sphereRadius);
  const std::array<Eigen::Isometry3d, 2> cylinderTransforms{{
      Eigen::Isometry3d::Identity(),
      makeCylinderBatchTransform(1.0, -2.0, 0.5, 0.37),
  }};
  constexpr std::array<double, 2> sphereRotationAngles{
      0.0, std::numbers::pi_v<double> / 5.0};

  for (const auto& testCase : cases) {
    for (const auto& tfCylinder : cylinderTransforms) {
      for (const double sphereRotationAngle : sphereRotationAngles) {
        SCOPED_TRACE(testCase.name);
        const CylinderShape cylinder(
            testCase.cylinderRadius, testCase.cylinderHeight);
        CollisionResult result;
        const bool collided = collideCylinderSphere(
            cylinder,
            tfCylinder,
            sphere,
            tfCylinder
                * makeSphereTransformInCylinderFrame(
                    testCase.sphereCenter, sphereRotationAngle),
            result);

        ASSERT_TRUE(collided);
        ASSERT_EQ(result.numContacts(), 1u);
        EXPECT_NEAR(result.getContact(0).depth, testCase.expectedDepth, 1e-9);
      }
    }
  }
}

TEST(CylinderSphere, InternalBottomSurfaceAndContactLimit)
{
  CylinderShape cylinder(1.0, 2.0);
  SphereShape sphere(0.25);
  const Eigen::Isometry3d tfCylinder = Eigen::Isometry3d::Identity();

  {
    CollisionResult result;
    const bool collided = collideCylinderSphere(
        cylinder,
        tfCylinder,
        sphere,
        makeTranslation(Eigen::Vector3d(0.2, 0.1, -0.95)),
        result);

    ASSERT_TRUE(collided);
    ASSERT_EQ(result.numContacts(), 1u);
    EXPECT_GT(result.getContact(0).depth, sphere.getRadius());
    EXPECT_TRUE(result.getContact(0).position.allFinite());
    EXPECT_TRUE(result.getContact(0).normal.allFinite());
  }

  {
    CollisionOption option;
    option.maxNumContacts = 0;
    CollisionResult result;
    EXPECT_FALSE(collideCylinderSphere(
        cylinder,
        tfCylinder,
        sphere,
        Eigen::Isometry3d::Identity(),
        result,
        option));
  }
}

TEST(CylinderSphere, InternalTopBarrelAndExternalSideBranches)
{
  CylinderShape cylinder(1.0, 2.0);
  SphereShape sphere(0.25);
  const Eigen::Isometry3d tfCylinder = Eigen::Isometry3d::Identity();

  const std::array<Eigen::Vector3d, 3> centers{{
      Eigen::Vector3d(0.2, 0.1, 0.95),
      Eigen::Vector3d(0.9, 0.1, 0.0),
      Eigen::Vector3d(1.2, 0.0, 0.0),
  }};

  for (const auto& center : centers) {
    SCOPED_TRACE(center.transpose());
    CollisionResult result;
    const bool collided = collideCylinderSphere(
        cylinder, tfCylinder, sphere, makeTranslation(center), result);

    ASSERT_TRUE(collided);
    ASSERT_EQ(result.numContacts(), 1u);
    EXPECT_TRUE(result.getContact(0).position.allFinite());
    EXPECT_TRUE(result.getContact(0).normal.allFinite());
    EXPECT_TRUE(std::isfinite(result.getContact(0).depth));
  }
}

TEST(CylinderSphere, ContactsWithIncompatibleScaleRatios)
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
    double expectedDepth;
    bool expectedCollision;
  };

  const double largeDiskRadius = 9.0;
  const double largeDiskHeight = 0.1;
  const double tinySphereRadius = 0.025;
  const double largeDiskTargetDepth = tinySphereRadius * 0.5;
  const Eigen::Vector3d largeDiskFacePoint
      = obliqueRadial * (largeDiskRadius - tinySphereRadius)
        + Eigen::Vector3d::UnitZ() * (largeDiskHeight * 0.5);
  const Eigen::Vector3d largeDiskBarrelPoint
      = obliqueRadial * largeDiskRadius
        + Eigen::Vector3d::UnitZ() * (tinySphereRadius * 0.1);

  const double tinyDiskRadius = 0.025;
  const double tinyDiskHeight = 0.1;
  const double largeSphereRadius = 9.0;
  const double tinyDiskTargetDepth = tinyDiskRadius * 0.5;
  const Eigen::Vector3d tinyDiskFacePoint
      = obliqueRadial * (tinyDiskRadius * 0.5)
        + Eigen::Vector3d::UnitZ() * (tinyDiskHeight * 0.5);
  const Eigen::Vector3d tinyDiskBarrelPoint
      = obliqueRadial * tinyDiskRadius
        + Eigen::Vector3d::UnitZ() * (tinyDiskHeight * 0.1);

  const std::array<Case, 8> cases{{
      {"large disk and tiny sphere touching face",
       largeDiskRadius,
       largeDiskHeight,
       tinySphereRadius,
       largeDiskFacePoint
           + Eigen::Vector3d::UnitZ()
                 * (tinySphereRadius - largeDiskTargetDepth),
       largeDiskTargetDepth,
       true},
      {"large disk and tiny sphere separated from face",
       largeDiskRadius,
       largeDiskHeight,
       tinySphereRadius,
       largeDiskFacePoint
           + Eigen::Vector3d::UnitZ()
                 * (tinySphereRadius + largeDiskTargetDepth),
       0.0,
       false},
      {"large disk and tiny sphere touching barrel",
       largeDiskRadius,
       largeDiskHeight,
       tinySphereRadius,
       largeDiskBarrelPoint
           + obliqueRadial * (tinySphereRadius - largeDiskTargetDepth),
       largeDiskTargetDepth,
       true},
      {"large disk and tiny sphere separated from barrel",
       largeDiskRadius,
       largeDiskHeight,
       tinySphereRadius,
       largeDiskBarrelPoint
           + obliqueRadial * (tinySphereRadius + largeDiskTargetDepth),
       0.0,
       false},
      {"large sphere and tiny disk touching face",
       tinyDiskRadius,
       tinyDiskHeight,
       largeSphereRadius,
       tinyDiskFacePoint
           + Eigen::Vector3d::UnitZ()
                 * (largeSphereRadius - tinyDiskTargetDepth),
       tinyDiskTargetDepth,
       true},
      {"large sphere and tiny disk separated from face",
       tinyDiskRadius,
       tinyDiskHeight,
       largeSphereRadius,
       tinyDiskFacePoint
           + Eigen::Vector3d::UnitZ()
                 * (largeSphereRadius + tinyDiskTargetDepth),
       0.0,
       false},
      {"large sphere and tiny disk touching barrel",
       tinyDiskRadius,
       tinyDiskHeight,
       largeSphereRadius,
       tinyDiskBarrelPoint
           + obliqueRadial * (largeSphereRadius - tinyDiskTargetDepth),
       tinyDiskTargetDepth,
       true},
      {"large sphere and tiny disk separated from barrel",
       tinyDiskRadius,
       tinyDiskHeight,
       largeSphereRadius,
       tinyDiskBarrelPoint
           + obliqueRadial * (largeSphereRadius + tinyDiskTargetDepth),
       0.0,
       false},
  }};

  const std::array<Eigen::Isometry3d, 2> cylinderTransforms{{
      Eigen::Isometry3d::Identity(),
      makeCylinderBatchTransform(1.0, -2.0, 0.5, 0.37),
  }};
  constexpr std::array<double, 2> sphereRotationAngles{
      0.0, std::numbers::pi_v<double> / 5.0};

  for (const auto& testCase : cases) {
    for (const auto& tfCylinder : cylinderTransforms) {
      for (const double sphereRotationAngle : sphereRotationAngles) {
        SCOPED_TRACE(testCase.name);
        const CylinderShape cylinder(
            testCase.cylinderRadius, testCase.cylinderHeight);
        const SphereShape sphere(testCase.sphereRadius);

        CollisionResult result;
        const bool collided = collideCylinderSphere(
            cylinder,
            tfCylinder,
            sphere,
            tfCylinder
                * makeSphereTransformInCylinderFrame(
                    testCase.sphereCenter, sphereRotationAngle),
            result);

        EXPECT_EQ(collided, testCase.expectedCollision);
        if (testCase.expectedCollision) {
          ASSERT_EQ(result.numContacts(), 1u);
          EXPECT_NEAR(result.getContact(0).depth, testCase.expectedDepth, 1e-9);
        } else {
          EXPECT_EQ(result.numContacts(), 0u);
        }
      }
    }
  }
}

TEST(CylinderBox, NoCollision)
{
  CylinderShape cylinder(0.5, 2.0);
  BoxShape box(Eigen::Vector3d(0.5, 0.5, 0.5));

  Eigen::Isometry3d tfCylinder = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d tfBox = Eigen::Isometry3d::Identity();
  tfBox.translation() = Eigen::Vector3d(3.0, 0, 0);

  CollisionResult result;
  bool collided = collideCylinderBox(cylinder, tfCylinder, box, tfBox, result);

  EXPECT_FALSE(collided);
  EXPECT_EQ(result.numContacts(), 0u);
}

TEST(CylinderBox, CylinderSideTouchesBox)
{
  CylinderShape cylinder(0.5, 2.0);
  BoxShape box(Eigen::Vector3d(0.2, 0.2, 0.2));

  Eigen::Isometry3d tfCylinder = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d tfBox = Eigen::Isometry3d::Identity();
  tfBox.translation() = Eigen::Vector3d(0.4, 0, 0);

  CollisionResult result;
  bool collided = collideCylinderBox(cylinder, tfCylinder, box, tfBox, result);

  EXPECT_TRUE(collided);
  EXPECT_GE(result.numContacts(), 1u);
}

TEST(CylinderBox, BoxFaceOverlapWithoutCornerInside)
{
  CylinderShape cylinder(0.5, 2.0);
  BoxShape box(Eigen::Vector3d(0.5, 0.5, 0.5));

  Eigen::Isometry3d tfCylinder = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d tfBox = Eigen::Isometry3d::Identity();
  tfBox.translation() = Eigen::Vector3d(0.8, 0, 0);

  CollisionResult result;
  bool collided = collideCylinderBox(cylinder, tfCylinder, box, tfBox, result);

  EXPECT_TRUE(collided);
  EXPECT_GE(result.numContacts(), 1u);
}

TEST(CylinderBox, CylinderOnTopOfBox)
{
  CylinderShape cylinder(0.5, 2.0);
  BoxShape box(Eigen::Vector3d(0.2, 0.2, 0.2));

  Eigen::Isometry3d tfCylinder = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d tfBox = Eigen::Isometry3d::Identity();
  tfBox.translation() = Eigen::Vector3d(0, 0, 0.9);

  CollisionResult result;
  bool collided = collideCylinderBox(cylinder, tfCylinder, box, tfBox, result);

  EXPECT_TRUE(collided);
  EXPECT_GE(result.numContacts(), 1u);
}

TEST(CylinderBox, AxialCapAgainstLargeBoxCreatesSupportPatch)
{
  CylinderShape cylinder(0.8, 0.02);
  BoxShape box(Eigen::Vector3d(1050.0, 1050.0, 1050.0));

  Eigen::Isometry3d tfCylinder = Eigen::Isometry3d::Identity();
  tfCylinder.translation() = Eigen::Vector3d(0.0, 0.0, 0.009);
  Eigen::Isometry3d tfBox = Eigen::Isometry3d::Identity();
  tfBox.translation() = Eigen::Vector3d(0.0, 0.0, -1050.0);

  CollisionResult result;
  bool collided = collideCylinderBox(cylinder, tfCylinder, box, tfBox, result);

  EXPECT_TRUE(collided);
  ASSERT_EQ(result.numContacts(), 4u);
  for (std::size_t i = 0; i < result.numContacts(); ++i) {
    const auto& contact = result.getContact(i);
    EXPECT_NEAR(contact.depth, 0.001, 1e-12);
    EXPECT_NEAR(contact.normal.x(), 0.0, 1e-12);
    EXPECT_NEAR(contact.normal.y(), 0.0, 1e-12);
    EXPECT_NEAR(contact.normal.z(), 1.0, 1e-12);
    EXPECT_NEAR(contact.position.z(), 0.0005, 1e-12);
  }
}

TEST(CylinderBox, AxialCapPatchRespectsMaxContacts)
{
  CylinderShape cylinder(0.8, 0.02);
  BoxShape box(Eigen::Vector3d(1050.0, 1050.0, 1050.0));

  Eigen::Isometry3d tfCylinder = Eigen::Isometry3d::Identity();
  tfCylinder.translation() = Eigen::Vector3d(0.0, 0.0, 0.009);
  Eigen::Isometry3d tfBox = Eigen::Isometry3d::Identity();
  tfBox.translation() = Eigen::Vector3d(0.0, 0.0, -1050.0);

  CollisionOption option;
  option.maxNumContacts = 2;

  CollisionResult result;
  bool collided
      = collideCylinderBox(cylinder, tfCylinder, box, tfBox, result, option);

  EXPECT_TRUE(collided);
  EXPECT_EQ(result.numContacts(), 2u);
}

TEST(CylinderBox, PlaneLikeAndRotatedCornerFallbackBranches)
{
  {
    CylinderShape cylinder(0.8, 0.02);
    BoxShape box(Eigen::Vector3d(1050.0, 1050.0, 1050.0));

    Eigen::Isometry3d tfCylinder = Eigen::Isometry3d::Identity();
    tfCylinder.translation() = Eigen::Vector3d(0.0, 0.0, 0.009);
    Eigen::Isometry3d tfBox = Eigen::Isometry3d::Identity();
    tfBox.linear()
        = Eigen::AngleAxisd(
              std::numbers::pi_v<double> / 4.0, Eigen::Vector3d::UnitZ())
              .toRotationMatrix();
    tfBox.translation() = Eigen::Vector3d(0.0, 0.0, -1050.0);

    CollisionResult result;
    ASSERT_TRUE(collideCylinderBox(cylinder, tfCylinder, box, tfBox, result));
    ASSERT_EQ(result.numContacts(), 1u);
    EXPECT_TRUE(result.getContact(0).position.allFinite());
    EXPECT_TRUE(result.getContact(0).normal.allFinite());
  }

  {
    CylinderShape cylinder(0.5, 2.0);
    BoxShape box(Eigen::Vector3d(0.4, 0.4, 0.2));

    Eigen::Isometry3d tfBox = Eigen::Isometry3d::Identity();
    tfBox.linear()
        = Eigen::AngleAxisd(0.2, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    tfBox.translation() = Eigen::Vector3d(0.9, 0.0, 0.0);

    CollisionResult result;
    ASSERT_TRUE(collideCylinderBox(
        cylinder, Eigen::Isometry3d::Identity(), box, tfBox, result));
    ASSERT_EQ(result.numContacts(), 1u);
    EXPECT_TRUE(result.getContact(0).position.allFinite());
    EXPECT_TRUE(result.getContact(0).normal.allFinite());
    EXPECT_NEAR(result.getContact(0).normal.norm(), 1.0, 1e-8);
  }
}

TEST(CylinderBox, TranslatedAndRotatedIntersections)
{
  const CylinderShape cylinder(0.4, 0.7);
  const BoxShape box(Eigen::Vector3d(0.25, 0.5, 0.75));

  struct PoseCase
  {
    const char* name;
    Eigen::Isometry3d tfCylinder;
    Eigen::Isometry3d tfBox;
  };

  const std::array<PoseCase, 8> cases{{
      {"center offset",
       makeTranslation(Eigen::Vector3d(0.1, 0.0, 0.0)),
       Eigen::Isometry3d::Identity()},
      {"side overlap",
       makeTranslation(Eigen::Vector3d(0.6, 0.0, 0.0)),
       Eigen::Isometry3d::Identity()},
      {"edge overlap",
       makeTranslation(Eigen::Vector3d(0.6, 0.6, 0.0)),
       Eigen::Isometry3d::Identity()},
      {"corner overlap",
       makeTranslation(Eigen::Vector3d(0.6, 0.6, 0.5)),
       Eigen::Isometry3d::Identity()},
      {"cylinder tilted around y axis",
       makeRotatedCylinderPose(
           Eigen::Vector3d(0.6, 0.6, 0.5),
           Eigen::Vector3d::UnitY(),
           std::numbers::pi_v<double> / 3.0),
       Eigen::Isometry3d::Identity()},
      {"cylinder tilted around oblique axis",
       makeRotatedCylinderPose(
           Eigen::Vector3d(0.6, 0.0, 0.5),
           Eigen::Vector3d(0.67, 1.1, 0.12),
           std::numbers::pi_v<double> / 4.0),
       Eigen::Isometry3d::Identity()},
      {"coincident oblique rotations",
       makeRotatedCylinderPose(
           Eigen::Vector3d(0.6, 0.0, 0.5),
           Eigen::Vector3d(-0.1, 2.2, -1.0),
           std::numbers::pi_v<double> / 5.0),
       makeRotatedCylinderPose(
           Eigen::Vector3d(0.6, 0.0, 0.5),
           Eigen::Vector3d(1.0, 1.0, 0.0),
           -std::numbers::pi_v<double> / 4.0)},
      {"offset oblique rotations",
       makeRotatedCylinderPose(
           Eigen::Vector3d(0.6, 0.0, 0.5),
           Eigen::Vector3d(-0.1, 2.2, -1.0),
           std::numbers::pi_v<double> / 5.0),
       makeRotatedCylinderPose(
           Eigen::Vector3d(0.9, 0.8, 0.5),
           Eigen::Vector3d(1.0, 1.0, 0.0),
           -std::numbers::pi_v<double> / 4.0)},
  }};

  for (const auto& testCase : cases) {
    SCOPED_TRACE(testCase.name);
    CollisionResult result;
    const bool hit = collideCylinderBox(
        cylinder, testCase.tfCylinder, box, testCase.tfBox, result);
    EXPECT_TRUE(hit);
  }
}

TEST(CylinderBox, TranslatedAndRotatedPenetrationsReportFiniteContacts)
{
  expectCylinderBoxFiniteHit(
      "center offset",
      makeTranslation(Eigen::Vector3d(0.1, 0.0, 0.0)),
      Eigen::Isometry3d::Identity());
  expectCylinderBoxFiniteHit(
      "side overlap",
      makeTranslation(Eigen::Vector3d(0.6, 0.0, 0.0)),
      Eigen::Isometry3d::Identity());
  expectCylinderBoxFiniteHit(
      "edge overlap",
      makeTranslation(Eigen::Vector3d(0.6, 0.6, 0.0)),
      Eigen::Isometry3d::Identity());
  expectCylinderBoxFiniteHit(
      "corner overlap",
      makeTranslation(Eigen::Vector3d(0.6, 0.6, 0.5)),
      Eigen::Isometry3d::Identity());
  expectCylinderBoxFiniteHit(
      "cylinder tilted around y axis",
      makeRotatedCylinderPose(
          Eigen::Vector3d(0.6, 0.6, 0.5),
          Eigen::Vector3d::UnitY(),
          std::numbers::pi_v<double> / 3.0),
      Eigen::Isometry3d::Identity());
  expectCylinderBoxFiniteHit(
      "cylinder tilted around oblique axis",
      makeRotatedCylinderPose(
          Eigen::Vector3d(0.6, 0.0, 0.5),
          Eigen::Vector3d(0.67, 1.1, 0.12),
          std::numbers::pi_v<double> / 4.0),
      Eigen::Isometry3d::Identity());
  expectCylinderBoxFiniteHit(
      "coincident oblique rotations",
      makeRotatedCylinderPose(
          Eigen::Vector3d(0.6, 0.0, 0.5),
          Eigen::Vector3d(-0.1, 2.2, -1.0),
          std::numbers::pi_v<double> / 5.0),
      makeRotatedCylinderPose(
          Eigen::Vector3d(0.6, 0.0, 0.5),
          Eigen::Vector3d(1.0, 1.0, 0.0),
          -std::numbers::pi_v<double> / 4.0));
  expectCylinderBoxFiniteHit(
      "offset oblique rotations",
      makeRotatedCylinderPose(
          Eigen::Vector3d(0.6, 0.0, 0.5),
          Eigen::Vector3d(-0.1, 2.2, -1.0),
          std::numbers::pi_v<double> / 5.0),
      makeRotatedCylinderPose(
          Eigen::Vector3d(0.9, 0.8, 0.5),
          Eigen::Vector3d(1.0, 1.0, 0.0),
          -std::numbers::pi_v<double> / 4.0));
}

TEST(CylinderBox, CenteredBoxChoosesNearestSideFace)
{
  CylinderShape cylinder(2.0, 4.0);
  BoxShape box(Eigen::Vector3d(0.2, 0.2, 3.0));
  const Eigen::Isometry3d tfCylinder = Eigen::Isometry3d::Identity();

  struct Case
  {
    Eigen::Vector3d boxCenter;
    Eigen::Vector3d expectedNormal;
  };

  const std::array<Case, 4> cases{{
      {Eigen::Vector3d(0.1, 0.0, 0.0), Eigen::Vector3d::UnitX()},
      {Eigen::Vector3d(-0.1, 0.0, 0.0), -Eigen::Vector3d::UnitX()},
      {Eigen::Vector3d(0.0, 0.1, 0.0), Eigen::Vector3d::UnitY()},
      {Eigen::Vector3d(0.0, -0.1, 0.0), -Eigen::Vector3d::UnitY()},
  }};

  for (const auto& testCase : cases) {
    CollisionResult result;
    const bool collided = collideCylinderBox(
        cylinder, tfCylinder, box, makeTranslation(testCase.boxCenter), result);

    ASSERT_TRUE(collided);
    ASSERT_EQ(result.numContacts(), 1u);
    expectVectorExactlyEqual(
        testCase.expectedNormal, result.getContact(0).normal);
    EXPECT_TRUE(result.getContact(0).position.allFinite());
  }

  CollisionOption option;
  option.maxNumContacts = 0;
  CollisionResult limited;
  EXPECT_FALSE(collideCylinderBox(
      cylinder,
      tfCylinder,
      box,
      Eigen::Isometry3d::Identity(),
      limited,
      option));
}

TEST(CylinderCapsule, NoCollision)
{
  CylinderShape cylinder(0.5, 2.0);
  CapsuleShape capsule(0.5, 2.0);

  Eigen::Isometry3d tfCylinder = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d tfCapsule = Eigen::Isometry3d::Identity();
  tfCapsule.translation() = Eigen::Vector3d(3.0, 0, 0);

  CollisionResult result;
  bool collided = collideCylinderCapsule(
      cylinder, tfCylinder, capsule, tfCapsule, result);

  EXPECT_FALSE(collided);
  EXPECT_EQ(result.numContacts(), 0u);
}

TEST(CylinderCapsule, ParallelOverlap)
{
  CylinderShape cylinder(0.5, 2.0);
  CapsuleShape capsule(0.5, 2.0);

  Eigen::Isometry3d tfCylinder = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d tfCapsule = Eigen::Isometry3d::Identity();
  tfCapsule.translation() = Eigen::Vector3d(0.8, 0, 0);

  CollisionResult result;
  bool collided = collideCylinderCapsule(
      cylinder, tfCylinder, capsule, tfCapsule, result);

  EXPECT_TRUE(collided);
  EXPECT_GE(result.numContacts(), 1u);
}

TEST(CylinderCapsule, CapsuleOnTop)
{
  CylinderShape cylinder(0.5, 2.0);
  CapsuleShape capsule(0.3, 1.0);

  Eigen::Isometry3d tfCylinder = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d tfCapsule = Eigen::Isometry3d::Identity();
  tfCapsule.translation() = Eigen::Vector3d(0, 0, 1.5);

  CollisionResult result;
  bool collided = collideCylinderCapsule(
      cylinder, tfCylinder, capsule, tfCapsule, result);

  EXPECT_TRUE(collided);
  EXPECT_GE(result.numContacts(), 1u);
}

TEST(CylinderCapsule, EndpointRegionsAndCoincidentNormal)
{
  CylinderShape cylinder(1.0, 2.0);
  CapsuleShape capsule(0.2, 1.0);
  const Eigen::Isometry3d tfCylinder = Eigen::Isometry3d::Identity();

  const std::array<Eigen::Isometry3d, 4> capsuleTransforms{{
      makeTranslation(Eigen::Vector3d(0.9, 0.0, 0.0)),
      makeTranslation(Eigen::Vector3d(1.1, 0.0, 0.0)),
      makeTranslation(Eigen::Vector3d(0.0, 0.0, 0.5)),
      makeTranslation(Eigen::Vector3d(0.0, 0.0, 3.0)),
  }};
  const std::array<bool, 4> expectedHits{{true, true, true, false}};

  for (std::size_t i = 0; i < capsuleTransforms.size(); ++i) {
    SCOPED_TRACE(i);
    CollisionResult result;
    const bool collided = collideCylinderCapsule(
        cylinder, tfCylinder, capsule, capsuleTransforms[i], result);
    EXPECT_EQ(collided, expectedHits[i]);
    if (expectedHits[i]) {
      ASSERT_EQ(result.numContacts(), 1u);
      EXPECT_TRUE(result.getContact(0).position.allFinite());
      EXPECT_TRUE(result.getContact(0).normal.allFinite());
      EXPECT_TRUE(std::isfinite(result.getContact(0).depth));
    } else {
      EXPECT_EQ(result.numContacts(), 0u);
    }
  }

  CollisionOption option;
  option.maxNumContacts = 0;
  CollisionResult limited;
  EXPECT_FALSE(collideCylinderCapsule(
      cylinder,
      tfCylinder,
      capsule,
      Eigen::Isometry3d::Identity(),
      limited,
      option));
}

TEST(CylinderCapsule, EndpointRegionBranchesRemainFinite)
{
  CylinderShape cylinder(1.0, 2.0);
  CapsuleShape capsule(0.2, 1.0);
  const Eigen::Isometry3d tfCylinder = Eigen::Isometry3d::Identity();

  const std::array<Eigen::Isometry3d, 3> capsuleTransforms{{
      makeTranslation(Eigen::Vector3d(0.9, 0.0, 0.0)),
      makeTranslation(Eigen::Vector3d(1.1, 0.0, 0.0)),
      makeTranslation(Eigen::Vector3d(1.1, 0.0, 1.2)),
  }};

  for (const auto& tfCapsule : capsuleTransforms) {
    CollisionResult result;
    ASSERT_TRUE(collideCylinderCapsule(
        cylinder, tfCylinder, capsule, tfCapsule, result));
    ASSERT_EQ(result.numContacts(), 1u);
    EXPECT_TRUE(result.getContact(0).position.allFinite());
    EXPECT_TRUE(result.getContact(0).normal.allFinite());
    EXPECT_TRUE(std::isfinite(result.getContact(0).depth));
  }
}

TEST(CylinderPlane, NoCollision)
{
  CylinderShape cylinder(0.5, 2.0);
  PlaneShape plane(Eigen::Vector3d::UnitZ(), 0.0);

  Eigen::Isometry3d tfCylinder = Eigen::Isometry3d::Identity();
  tfCylinder.translation() = Eigen::Vector3d(0, 0, 2.0);
  Eigen::Isometry3d tfPlane = Eigen::Isometry3d::Identity();

  CollisionResult result;
  bool collided
      = collideCylinderPlane(cylinder, tfCylinder, plane, tfPlane, result);

  EXPECT_FALSE(collided);
  EXPECT_EQ(result.numContacts(), 0u);
}

TEST(CylinderPlane, CylinderStandingOnPlane)
{
  CylinderShape cylinder(0.5, 2.0);
  PlaneShape plane(Eigen::Vector3d::UnitZ(), 0.0);

  Eigen::Isometry3d tfCylinder = Eigen::Isometry3d::Identity();
  tfCylinder.translation() = Eigen::Vector3d(0, 0, 0.9);
  Eigen::Isometry3d tfPlane = Eigen::Isometry3d::Identity();

  CollisionResult result;
  bool collided
      = collideCylinderPlane(cylinder, tfCylinder, plane, tfPlane, result);

  EXPECT_TRUE(collided);
  EXPECT_GE(result.numContacts(), 1u);
}

TEST(CylinderPlane, HalfspaceDepthAcrossAxisDirections)
{
  CylinderShape cylinder(0.05, 0.2);
  PlaneShape plane(Eigen::Vector3d::UnitZ(), 0.0);
  const Eigen::Isometry3d tfPlane = Eigen::Isometry3d::Identity();

  for (const double angle : {0.0, std::numbers::pi_v<double>}) {
    Eigen::Isometry3d tfCylinder = Eigen::Isometry3d::Identity();
    tfCylinder.linear()
        = Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitX()).toRotationMatrix();
    tfCylinder.translation() = Eigen::Vector3d(0.0, 0.0, 0.049);

    CollisionResult result;
    const bool collided
        = collideCylinderPlane(cylinder, tfCylinder, plane, tfPlane, result);

    ASSERT_TRUE(collided);
    ASSERT_EQ(result.numContacts(), 1u);
    EXPECT_NEAR(result.getContact(0).depth, 0.051, 1e-12);
    EXPECT_TRUE(result.getContact(0).position.allFinite());
    EXPECT_TRUE(result.getContact(0).normal.allFinite());
  }
}

TEST(CylinderPlane, CylinderLyingOnPlane)
{
  CylinderShape cylinder(0.5, 2.0);
  PlaneShape plane(Eigen::Vector3d::UnitZ(), 0.0);

  Eigen::Isometry3d tfCylinder = Eigen::Isometry3d::Identity();
  tfCylinder.linear()
      = Eigen::AngleAxisd(
            std::numbers::pi_v<double> / 2, Eigen::Vector3d::UnitY())
            .toRotationMatrix();
  tfCylinder.translation() = Eigen::Vector3d(0, 0, 0.4);
  Eigen::Isometry3d tfPlane = Eigen::Isometry3d::Identity();

  CollisionResult result;
  bool collided
      = collideCylinderPlane(cylinder, tfCylinder, plane, tfPlane, result);

  EXPECT_TRUE(collided);
  EXPECT_GE(result.numContacts(), 1u);
}

TEST(CylinderPlane, CylinderTiltedOnPlane)
{
  CylinderShape cylinder(0.5, 2.0);
  PlaneShape plane(Eigen::Vector3d::UnitZ(), 0.0);

  Eigen::Isometry3d tfCylinder = Eigen::Isometry3d::Identity();
  tfCylinder.linear()
      = Eigen::AngleAxisd(
            std::numbers::pi_v<double> / 4, Eigen::Vector3d::UnitY())
            .toRotationMatrix();
  tfCylinder.translation() = Eigen::Vector3d(0, 0, 0.5);
  Eigen::Isometry3d tfPlane = Eigen::Isometry3d::Identity();

  CollisionResult result;
  bool collided
      = collideCylinderPlane(cylinder, tfCylinder, plane, tfPlane, result);

  EXPECT_TRUE(collided);
  EXPECT_GE(result.numContacts(), 1u);
}

TEST(CylinderPlane, ContactLimitDisablesPlaneHit)
{
  CylinderShape cylinder(0.5, 2.0);
  PlaneShape plane(Eigen::Vector3d::UnitZ(), 0.0);

  CollisionOption option;
  option.maxNumContacts = 0;
  CollisionResult result;
  EXPECT_FALSE(collideCylinderPlane(
      cylinder,
      Eigen::Isometry3d::Identity(),
      plane,
      Eigen::Isometry3d::Identity(),
      result,
      option));
}
