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
#include <dart/collision/native/narrow_phase/SphereSphere.hpp>
#include <dart/collision/native/shapes/Shape.hpp>

#include <gtest/gtest.h>

#include <array>
#include <random>
#include <stdexcept>
#include <vector>

#include <cmath>

using namespace dart::collision::native;

namespace {

constexpr double kPi = 3.141592653589793238462643383279502884;

} // namespace

namespace {

Eigen::Isometry3d makeSphereSphereCommonFrame()
{
  Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
  tf.linear() = (Eigen::AngleAxisd(0.25, Eigen::Vector3d::UnitX())
                 * Eigen::AngleAxisd(-0.7, Eigen::Vector3d::UnitY())
                 * Eigen::AngleAxisd(0.4, Eigen::Vector3d::UnitZ()))
                    .toRotationMatrix();
  tf.translation() = Eigen::Vector3d(8.40188, 3.94383, 7.83099);
  return tf;
}

Eigen::Isometry3d makeSphereSphereLocalTranslation(
    const Eigen::Vector3d& translation)
{
  Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
  tf.translation() = translation;
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

void expectLargeSphereSphereBoundaryCase(
    const char* name,
    const Eigen::Isometry3d& worldFromCase,
    const Eigen::Vector3d& secondCenterInCase,
    bool expectedHit,
    double expectedDepth)
{
  SCOPED_TRACE(name);
  SphereShape sphere1(20.0);
  SphereShape sphere2(10.0);

  const Eigen::Isometry3d tf1 = worldFromCase;
  const Eigen::Isometry3d tf2
      = worldFromCase * makeSphereSphereLocalTranslation(secondCenterInCase);

  CollisionResult result;
  const bool hit = collideSpheres(sphere1, tf1, sphere2, tf2, result);
  EXPECT_EQ(hit, expectedHit);

  if (!expectedHit) {
    EXPECT_EQ(result.numContacts(), 0u);
    return;
  }

  ASSERT_EQ(result.numContacts(), 1u);
  const auto& contact = result.getContact(0);
  EXPECT_NEAR(contact.depth, expectedDepth, 1e-9);
  EXPECT_TRUE(contact.position.allFinite());
  EXPECT_TRUE(contact.normal.allFinite());

  if (secondCenterInCase.norm() > 0.0) {
    const Eigen::Vector3d expectedNormal
        = worldFromCase.linear() * (-secondCenterInCase.normalized());
    EXPECT_NEAR((contact.normal - expectedNormal).norm(), 0.0, 1e-9);
  } else {
    EXPECT_NEAR(contact.normal.norm(), 1.0, 1e-9);
  }
}

} // namespace

TEST(SphereSphere, Separated)
{
  CollisionResult result;

  bool collided = collideSpheres(
      Eigen::Vector3d(0, 0, 0), 1.0, Eigen::Vector3d(3, 0, 0), 1.0, result);

  EXPECT_FALSE(collided);
  EXPECT_EQ(result.numContacts(), 0);
}

TEST(SphereSphere, Touching)
{
  CollisionResult result;

  bool collided = collideSpheres(
      Eigen::Vector3d(0, 0, 0), 1.0, Eigen::Vector3d(2, 0, 0), 1.0, result);

  EXPECT_TRUE(collided);
  ASSERT_EQ(result.numContacts(), 1);

  const auto& contact = result.getContact(0);
  EXPECT_NEAR(contact.depth, 0.0, 1e-10);
  EXPECT_NEAR(contact.normal.norm(), 1.0, 1e-10);
}

TEST(SphereSphere, Overlapping)
{
  CollisionResult result;

  bool collided = collideSpheres(
      Eigen::Vector3d(0, 0, 0), 1.0, Eigen::Vector3d(1.5, 0, 0), 1.0, result);

  EXPECT_TRUE(collided);
  ASSERT_EQ(result.numContacts(), 1);

  const auto& contact = result.getContact(0);
  EXPECT_NEAR(contact.depth, 0.5, 1e-10);
  EXPECT_NEAR(contact.normal.x(), -1.0, 1e-10);
  EXPECT_NEAR(contact.normal.y(), 0.0, 1e-10);
  EXPECT_NEAR(contact.normal.z(), 0.0, 1e-10);
}

TEST(SphereSphere, DeeplyPenetrating)
{
  CollisionResult result;

  bool collided = collideSpheres(
      Eigen::Vector3d(0, 0, 0), 1.0, Eigen::Vector3d(0.5, 0, 0), 1.0, result);

  EXPECT_TRUE(collided);
  ASSERT_EQ(result.numContacts(), 1);

  const auto& contact = result.getContact(0);
  EXPECT_NEAR(contact.depth, 1.5, 1e-10);
}

TEST(SphereSphere, Concentric)
{
  CollisionResult result;

  bool collided = collideSpheres(
      Eigen::Vector3d(0, 0, 0), 1.0, Eigen::Vector3d(0, 0, 0), 1.0, result);

  EXPECT_TRUE(collided);
  ASSERT_EQ(result.numContacts(), 1);

  const auto& contact = result.getContact(0);
  EXPECT_NEAR(contact.depth, 2.0, 1e-10);
  EXPECT_NEAR(contact.normal.norm(), 1.0, 1e-10);
}

TEST(SphereSphere, AxisSweepTransitionsAcrossCoordinates)
{
  const double radius1 = 0.35;
  const double radius2 = 0.5;
  const double radiusSum = radius1 + radius2;
  const std::array<Eigen::Vector3d, 3> axes{
      Eigen::Vector3d::UnitX(),
      Eigen::Vector3d::UnitY(),
      Eigen::Vector3d::UnitZ()};

  for (const auto& axis : axes) {
    for (int i = 0; i < 100; ++i) {
      const double offset = -5.0 + 0.1 * i;
      SCOPED_TRACE(offset);

      CollisionResult result;
      const bool hit = collideSpheres(
          Eigen::Vector3d::Zero(), radius2, offset * axis, radius1, result);
      const bool expectedHit = std::abs(offset) <= radiusSum;
      EXPECT_EQ(hit, expectedHit);

      if (!expectedHit) {
        EXPECT_EQ(result.numContacts(), 0u);
        continue;
      }

      ASSERT_EQ(result.numContacts(), 1u);
      const auto& contact = result.getContact(0);
      EXPECT_NEAR(contact.depth, radiusSum - std::abs(offset), 1e-12);
      EXPECT_TRUE(contact.position.allFinite());
      EXPECT_TRUE(contact.normal.allFinite());
      EXPECT_NEAR(contact.normal.norm(), 1.0, 1e-12);
    }
  }
}

TEST(SphereSphere, ZeroRadius)
{
  {
    CollisionResult result;
    const bool collided = collideSpheres(
        Eigen::Vector3d(0, 0, 0), 0.0, Eigen::Vector3d(2, 0, 0), 1.0, result);

    EXPECT_FALSE(collided);
    EXPECT_EQ(result.numContacts(), 0);
  }

  {
    CollisionResult result;
    const bool collided = collideSpheres(
        Eigen::Vector3d(0, 0, 0), 0.0, Eigen::Vector3d(1, 0, 0), 1.0, result);

    EXPECT_TRUE(collided);
    ASSERT_EQ(result.numContacts(), 1);

    const auto& contact = result.getContact(0);
    EXPECT_NEAR(contact.depth, 0.0, 1e-10);
    EXPECT_TRUE(contact.position.allFinite());
    EXPECT_TRUE(contact.normal.allFinite());
    EXPECT_NEAR(contact.normal.x(), -1.0, 1e-10);
    EXPECT_NEAR(contact.normal.y(), 0.0, 1e-10);
    EXPECT_NEAR(contact.normal.z(), 0.0, 1e-10);
  }

  {
    CollisionResult ab;
    const bool collidedAb = collideSpheres(
        Eigen::Vector3d(0, 0, 0), 0.0, Eigen::Vector3d(0.5, 0, 0), 1.0, ab);

    CollisionResult ba;
    const bool collidedBa = collideSpheres(
        Eigen::Vector3d(0.5, 0, 0), 1.0, Eigen::Vector3d(0, 0, 0), 0.0, ba);

    ASSERT_TRUE(collidedAb);
    ASSERT_TRUE(collidedBa);
    ASSERT_EQ(ab.numContacts(), 1);
    ASSERT_EQ(ba.numContacts(), 1);

    const auto& contactAb = ab.getContact(0);
    const auto& contactBa = ba.getContact(0);
    EXPECT_NEAR(contactAb.depth, 0.5, 1e-10);
    EXPECT_NEAR(contactBa.depth, 0.5, 1e-10);
    EXPECT_TRUE(contactAb.position.allFinite());
    EXPECT_TRUE(contactBa.position.allFinite());
    EXPECT_TRUE(contactAb.normal.allFinite());
    EXPECT_TRUE(contactBa.normal.allFinite());
    EXPECT_NEAR((contactAb.position - contactBa.position).norm(), 0.0, 1e-10);
    EXPECT_NEAR((contactAb.normal + contactBa.normal).norm(), 0.0, 1e-10);
  }

  {
    CollisionResult result;
    const bool collided = collideSpheres(
        Eigen::Vector3d(0, 0, 0), 0.0, Eigen::Vector3d(0, 0, 0), 0.0, result);

    EXPECT_TRUE(collided);
    ASSERT_EQ(result.numContacts(), 1);

    const auto& contact = result.getContact(0);
    EXPECT_NEAR(contact.depth, 0.0, 1e-10);
    EXPECT_TRUE(contact.position.allFinite());
    EXPECT_TRUE(contact.normal.allFinite());
    EXPECT_NEAR(contact.normal.norm(), 1.0, 1e-10);
  }
}

TEST(SphereSphere, DifferentRadii)
{
  CollisionResult result;

  bool collided = collideSpheres(
      Eigen::Vector3d(0, 0, 0), 2.0, Eigen::Vector3d(2.5, 0, 0), 1.0, result);

  EXPECT_TRUE(collided);
  ASSERT_EQ(result.numContacts(), 1);

  const auto& contact = result.getContact(0);
  EXPECT_NEAR(contact.depth, 0.5, 1e-10);
}

TEST(SphereSphere, SmallSpheres)
{
  CollisionResult result;

  bool collided = collideSpheres(
      Eigen::Vector3d(0, 0, 0),
      0.001,
      Eigen::Vector3d(0.0015, 0, 0),
      0.001,
      result);

  EXPECT_TRUE(collided);
  ASSERT_EQ(result.numContacts(), 1);

  const auto& contact = result.getContact(0);
  EXPECT_NEAR(contact.depth, 0.0005, 1e-10);
}

TEST(SphereSphere, LargeSpheres)
{
  CollisionResult result;

  bool collided = collideSpheres(
      Eigen::Vector3d(0, 0, 0),
      1000.0,
      Eigen::Vector3d(1500, 0, 0),
      1000.0,
      result);

  EXPECT_TRUE(collided);
  ASSERT_EQ(result.numContacts(), 1);

  const auto& contact = result.getContact(0);
  EXPECT_NEAR(contact.depth, 500.0, 1e-6);
}

TEST(SphereSphere, AlongYAxis)
{
  CollisionResult result;

  bool collided = collideSpheres(
      Eigen::Vector3d(0, 0, 0), 1.0, Eigen::Vector3d(0, 1.5, 0), 1.0, result);

  EXPECT_TRUE(collided);
  ASSERT_EQ(result.numContacts(), 1);

  const auto& contact = result.getContact(0);
  EXPECT_NEAR(contact.normal.x(), 0.0, 1e-10);
  EXPECT_NEAR(contact.normal.y(), -1.0, 1e-10);
  EXPECT_NEAR(contact.normal.z(), 0.0, 1e-10);
}

TEST(SphereSphere, AlongZAxis)
{
  CollisionResult result;

  bool collided = collideSpheres(
      Eigen::Vector3d(0, 0, 0), 1.0, Eigen::Vector3d(0, 0, 1.5), 1.0, result);

  EXPECT_TRUE(collided);
  ASSERT_EQ(result.numContacts(), 1);

  const auto& contact = result.getContact(0);
  EXPECT_NEAR(contact.normal.x(), 0.0, 1e-10);
  EXPECT_NEAR(contact.normal.y(), 0.0, 1e-10);
  EXPECT_NEAR(contact.normal.z(), -1.0, 1e-10);
}

TEST(SphereSphere, DiagonalDirection)
{
  CollisionResult result;

  const double dist = 1.5;
  const double component = dist / std::sqrt(3.0);

  bool collided = collideSpheres(
      Eigen::Vector3d(0, 0, 0),
      1.0,
      Eigen::Vector3d(component, component, component),
      1.0,
      result);

  EXPECT_TRUE(collided);
  ASSERT_EQ(result.numContacts(), 1);

  const auto& contact = result.getContact(0);
  EXPECT_NEAR(contact.depth, 0.5, 1e-10);

  const double expectedNormalComponent = -1.0 / std::sqrt(3.0);
  EXPECT_NEAR(contact.normal.x(), expectedNormalComponent, 1e-10);
  EXPECT_NEAR(contact.normal.y(), expectedNormalComponent, 1e-10);
  EXPECT_NEAR(contact.normal.z(), expectedNormalComponent, 1e-10);
}

TEST(SphereSphere, MaxContactsRespected)
{
  CollisionResult result;
  CollisionOption option;
  option.maxNumContacts = 0;

  bool collided = collideSpheres(
      Eigen::Vector3d(0, 0, 0),
      1.0,
      Eigen::Vector3d(1.5, 0, 0),
      1.0,
      result,
      option);

  EXPECT_FALSE(collided);
  EXPECT_EQ(result.numContacts(), 0);
}

TEST(SphereSphere, NormalDirectionDartConvention)
{
  CollisionResult result;

  bool collided = collideSpheres(
      Eigen::Vector3d(0, 0, 0), 1.0, Eigen::Vector3d(1.5, 0, 0), 1.0, result);

  EXPECT_TRUE(collided);
  ASSERT_EQ(result.numContacts(), 1);

  const auto& contact = result.getContact(0);
  EXPECT_NEAR(contact.normal.x(), -1.0, 1e-10);
}

TEST(SphereSphere, UsingShapeObjects)
{
  SphereShape sphere1(1.0);
  SphereShape sphere2(1.0);

  Eigen::Isometry3d transform1 = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d transform2 = Eigen::Isometry3d::Identity();
  transform2.translation() = Eigen::Vector3d(1.5, 0, 0);

  CollisionResult result;

  bool collided
      = collideSpheres(sphere1, transform1, sphere2, transform2, result);

  EXPECT_TRUE(collided);
  ASSERT_EQ(result.numContacts(), 1);
  EXPECT_NEAR(result.getContact(0).depth, 0.5, 1e-10);
}

TEST(SphereSphere, ShapeObjectsWithRotation)
{
  SphereShape sphere1(1.0);
  SphereShape sphere2(1.0);

  Eigen::Isometry3d transform1 = Eigen::Isometry3d::Identity();
  transform1.rotate(Eigen::AngleAxisd(kPi / 4, Eigen::Vector3d::UnitZ()));

  Eigen::Isometry3d transform2 = Eigen::Isometry3d::Identity();
  transform2.translation() = Eigen::Vector3d(1.5, 0, 0);
  transform2.rotate(Eigen::AngleAxisd(kPi / 2, Eigen::Vector3d::UnitX()));

  CollisionResult result;

  bool collided
      = collideSpheres(sphere1, transform1, sphere2, transform2, result);

  EXPECT_TRUE(collided);
  ASSERT_EQ(result.numContacts(), 1);
  EXPECT_NEAR(result.getContact(0).depth, 0.5, 1e-10);
}

TEST(SphereSphere, LargeRadiiBoundaryAndPenetrationAcrossFrames)
{
  const std::array<Eigen::Isometry3d, 2> worldFromCases{
      Eigen::Isometry3d::Identity(), makeSphereSphereCommonFrame()};

  for (const auto& worldFromCase : worldFromCases) {
    expectLargeSphereSphereBoundaryCase(
        "clearly-separated",
        worldFromCase,
        Eigen::Vector3d(40.0, 0.0, 0.0),
        false,
        0.0);
    expectLargeSphereSphereBoundaryCase(
        "positive-axis-touching",
        worldFromCase,
        Eigen::Vector3d(30.0, 0.0, 0.0),
        true,
        0.0);
    expectLargeSphereSphereBoundaryCase(
        "positive-axis-epsilon-separated",
        worldFromCase,
        Eigen::Vector3d(30.01, 0.0, 0.0),
        false,
        0.0);
    expectLargeSphereSphereBoundaryCase(
        "positive-axis-shallow-penetration",
        worldFromCase,
        Eigen::Vector3d(29.9, 0.0, 0.0),
        true,
        0.1);
    expectLargeSphereSphereBoundaryCase(
        "coincident-centers",
        worldFromCase,
        Eigen::Vector3d::Zero(),
        true,
        30.0);
    expectLargeSphereSphereBoundaryCase(
        "negative-axis-shallow-penetration",
        worldFromCase,
        Eigen::Vector3d(-29.9, 0.0, 0.0),
        true,
        0.1);
    expectLargeSphereSphereBoundaryCase(
        "negative-axis-touching",
        worldFromCase,
        Eigen::Vector3d(-30.0, 0.0, 0.0),
        true,
        0.0);
    expectLargeSphereSphereBoundaryCase(
        "negative-axis-epsilon-separated",
        worldFromCase,
        Eigen::Vector3d(-30.01, 0.0, 0.0),
        false,
        0.0);
  }
}

TEST(SphereSphere, Determinism)
{
  std::vector<ContactPoint> contacts;

  for (int i = 0; i < 100; ++i) {
    CollisionResult result;

    bool collided = collideSpheres(
        Eigen::Vector3d(0.123456789, 0.987654321, 0.111222333),
        1.5,
        Eigen::Vector3d(1.5, 0.5, 0.25),
        1.0,
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

TEST(SphereSphere, NegativeDirection)
{
  CollisionResult result;

  bool collided = collideSpheres(
      Eigen::Vector3d(0, 0, 0), 1.0, Eigen::Vector3d(-1.5, 0, 0), 1.0, result);

  EXPECT_TRUE(collided);
  ASSERT_EQ(result.numContacts(), 1);

  const auto& contact = result.getContact(0);
  EXPECT_NEAR(contact.normal.x(), 1.0, 1e-10);
  EXPECT_NEAR(contact.normal.y(), 0.0, 1e-10);
  EXPECT_NEAR(contact.normal.z(), 0.0, 1e-10);
}

TEST(SphereSphereBatch, sphere_sphere_batch_determinism_vs_single)
{
  SphereShape sphereA(0.75);
  SphereShape sphereB(0.60);

  std::mt19937 rng(314159u);
  std::uniform_real_distribution<double> offset(-0.10, 0.10);

  std::vector<SpherePair> pairs;
  pairs.reserve(100);
  for (int i = 0; i < 100; ++i) {
    Eigen::Isometry3d tfA = Eigen::Isometry3d::Identity();
    tfA.translation() = Eigen::Vector3d(offset(rng), offset(rng), offset(rng));

    Eigen::Isometry3d tfB = Eigen::Isometry3d::Identity();
    tfB.translation()
        = Eigen::Vector3d(0.85 + offset(rng), offset(rng), offset(rng));

    pairs.push_back(SpherePair{&sphereA, &sphereB, tfA, tfB});
  }

  CollisionOption option;
  std::vector<CollisionResult> batchResults(pairs.size());
  collideSpheresBatch(pairs, batchResults, option);

  for (std::size_t i = 0; i < pairs.size(); ++i) {
    CollisionResult singleResult;
    ASSERT_TRUE(collideSpheres(
        *pairs[i].shapeA,
        pairs[i].tfA,
        *pairs[i].shapeB,
        pairs[i].tfB,
        singleResult,
        option));
    expectCollisionResultExactlyEqual(singleResult, batchResults[i]);
  }
}

TEST(SphereSphereBatch, RejectsMalformedInputs)
{
  SphereShape sphereA(0.75);
  SphereShape sphereB(0.60);

  const Eigen::Isometry3d tfA = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d tfB = Eigen::Isometry3d::Identity();
  tfB.translation() = Eigen::Vector3d(0.85, 0.0, 0.0);

  const std::vector<SpherePair> validPairs{{&sphereA, &sphereB, tfA, tfB}};
  std::vector<CollisionResult> emptyResults;
  EXPECT_THROW(
      collideSpheresBatch(validPairs, emptyResults), std::invalid_argument);

  const std::vector<SpherePair> nullShapePairs{{nullptr, &sphereB, tfA, tfB}};
  std::vector<CollisionResult> results(1);
  EXPECT_THROW(
      collideSpheresBatch(nullShapePairs, results), std::invalid_argument);
}

TEST(SphereSphere, BinaryCheckReportsHitWithoutContacts)
{
  const CollisionOption option = CollisionOption::binaryCheck();

  CollisionResult result;
  EXPECT_TRUE(collideSpheres(
      Eigen::Vector3d(0, 0, 0),
      1.0,
      Eigen::Vector3d(1.5, 0, 0),
      1.0,
      result,
      option));
  EXPECT_EQ(result.numContacts(), 0);

  EXPECT_FALSE(collideSpheres(
      Eigen::Vector3d(0, 0, 0),
      1.0,
      Eigen::Vector3d(3, 0, 0),
      1.0,
      result,
      option));
  EXPECT_EQ(result.numContacts(), 0);

  SphereShape sphere1(1.0);
  SphereShape sphere2(1.0);
  Eigen::Isometry3d tf1 = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d tf2 = Eigen::Isometry3d::Identity();
  tf2.translation() = Eigen::Vector3d(0.0, 1.5, 0.0);
  EXPECT_TRUE(collideSpheres(sphere1, tf1, sphere2, tf2, result, option));
  EXPECT_EQ(result.numContacts(), 0);

  tf2.translation() = Eigen::Vector3d(0.0, 3.0, 0.0);
  EXPECT_FALSE(collideSpheres(sphere1, tf1, sphere2, tf2, result, option));
  EXPECT_EQ(result.numContacts(), 0);
}

TEST(SphereSphere, ZeroContactLimitShortCircuitsEvenWithoutContact)
{
  // maxNumContacts == 0 disables detection and must return false regardless of
  // enableContact, even for an overlapping pair. This takes precedence over the
  // enableContact binary-check path.
  CollisionOption option;
  option.enableContact = false;
  option.maxNumContacts = 0;

  CollisionResult result;
  EXPECT_FALSE(collideSpheres(
      Eigen::Vector3d(0, 0, 0),
      1.0,
      Eigen::Vector3d(1.5, 0, 0),
      1.0,
      result,
      option));
  EXPECT_EQ(result.numContacts(), 0);

  SphereShape sphere1(1.0);
  SphereShape sphere2(1.0);
  Eigen::Isometry3d tf1 = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d tf2 = Eigen::Isometry3d::Identity();
  tf2.translation() = Eigen::Vector3d(0.0, 1.5, 0.0);
  EXPECT_FALSE(collideSpheres(sphere1, tf1, sphere2, tf2, result, option));
  EXPECT_EQ(result.numContacts(), 0);
}

// DART 6 supplementary coverage tests (not ported from DART 7)
TEST(SphereSphere, ShapeOverloadZAxisAndContactLimit)
{
  SphereShape sphere1(1.0);
  SphereShape sphere2(1.0);
  Eigen::Isometry3d tf1 = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d tf2 = Eigen::Isometry3d::Identity();
  tf2.translation() = Eigen::Vector3d(0.0, 0.0, 1.5);

  CollisionResult zAxisResult;
  EXPECT_TRUE(collideSpheres(sphere1, tf1, sphere2, tf2, zAxisResult));
  ASSERT_EQ(zAxisResult.numContacts(), 1u);
  EXPECT_NEAR(zAxisResult.getContact(0).normal.z(), -1.0, 1e-10);
  EXPECT_NEAR(zAxisResult.getContact(0).depth, 0.5, 1e-10);

  CollisionOption capped;
  capped.maxNumContacts = 0u;
  CollisionResult cappedResult;
  EXPECT_FALSE(
      collideSpheres(sphere1, tf1, sphere2, tf2, cappedResult, capped));
  EXPECT_EQ(cappedResult.numContacts(), 0u);
}
