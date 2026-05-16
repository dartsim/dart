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

#include <dart/collision/native/narrow_phase/capsule_box.hpp>
#include <dart/collision/native/narrow_phase/capsule_capsule.hpp>
#include <dart/collision/native/narrow_phase/capsule_sphere.hpp>
#include <dart/collision/native/shapes/shape.hpp>

#include <gtest/gtest.h>

#include <random>
#include <stdexcept>
#include <vector>

using namespace dart::collision::native;

namespace {

Eigen::Isometry3d makeCapsuleBatchTransform(
    double x, double y, double z, double angle)
{
  Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
  tf.linear() = (Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitX())
                 * Eigen::AngleAxisd(0.5 * angle, Eigen::Vector3d::UnitY()))
                    .toRotationMatrix();
  tf.translation() = Eigen::Vector3d(x, y, z);
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

} // namespace

TEST(CapsuleCapsule, NoCollision)
{
  CapsuleShape capsule1(0.5, 2.0);
  CapsuleShape capsule2(0.5, 2.0);

  Eigen::Isometry3d tf1 = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d tf2 = Eigen::Isometry3d::Identity();
  tf2.translation() = Eigen::Vector3d(3.0, 0, 0);

  CollisionResult result;
  bool collided = collideCapsules(capsule1, tf1, capsule2, tf2, result);

  EXPECT_FALSE(collided);
  EXPECT_EQ(result.numContacts(), 0u);
}

TEST(CapsuleCapsule, ParallelOverlap)
{
  CapsuleShape capsule1(0.5, 2.0);
  CapsuleShape capsule2(0.5, 2.0);

  Eigen::Isometry3d tf1 = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d tf2 = Eigen::Isometry3d::Identity();
  tf2.translation() = Eigen::Vector3d(0.8, 0, 0);

  CollisionResult result;
  bool collided = collideCapsules(capsule1, tf1, capsule2, tf2, result);

  EXPECT_TRUE(collided);
  EXPECT_EQ(result.numContacts(), 1u);
  EXPECT_GT(result.getContact(0).depth, 0.0);
}

TEST(CapsuleCapsule, EndToEnd)
{
  CapsuleShape capsule1(0.5, 2.0);
  CapsuleShape capsule2(0.5, 2.0);

  Eigen::Isometry3d tf1 = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d tf2 = Eigen::Isometry3d::Identity();
  tf2.translation() = Eigen::Vector3d(0, 0, 2.5);

  CollisionResult result;
  bool collided = collideCapsules(capsule1, tf1, capsule2, tf2, result);

  EXPECT_TRUE(collided);
  EXPECT_EQ(result.numContacts(), 1u);
  EXPECT_NEAR(result.getContact(0).depth, 0.5, 1e-6);
}

TEST(CapsuleCapsule, Perpendicular)
{
  CapsuleShape capsule1(0.5, 2.0);
  CapsuleShape capsule2(0.5, 2.0);

  Eigen::Isometry3d tf1 = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d tf2 = Eigen::Isometry3d::Identity();
  tf2.linear() = Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitY())
                     .toRotationMatrix();
  tf2.translation() = Eigen::Vector3d(0.8, 0, 0);

  CollisionResult result;
  bool collided = collideCapsules(capsule1, tf1, capsule2, tf2, result);

  EXPECT_TRUE(collided);
  EXPECT_EQ(result.numContacts(), 1u);
}

TEST(CapsuleCapsule, Concentric)
{
  CapsuleShape capsule1(0.5, 2.0);
  CapsuleShape capsule2(0.3, 1.0);

  Eigen::Isometry3d tf1 = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d tf2 = Eigen::Isometry3d::Identity();

  CollisionResult result;
  bool collided = collideCapsules(capsule1, tf1, capsule2, tf2, result);

  EXPECT_TRUE(collided);
  EXPECT_EQ(result.numContacts(), 1u);
  EXPECT_NEAR(result.getContact(0).depth, 0.8, 1e-6);
}

TEST(CapsuleCapsuleBatch, capsule_capsule_batch_determinism_vs_single)
{
  CapsuleShape capsuleA(0.35, 1.2);
  CapsuleShape capsuleB(0.30, 1.0);

  std::mt19937 rng(271828u);
  std::uniform_real_distribution<double> offset(-0.05, 0.05);
  std::uniform_real_distribution<double> angle(-0.20, 0.20);

  std::vector<CapsulePair> pairs;
  pairs.reserve(100);
  for (int i = 0; i < 100; ++i) {
    const Eigen::Isometry3d tfA = makeCapsuleBatchTransform(
        offset(rng), offset(rng), offset(rng), angle(rng));
    const Eigen::Isometry3d tfB = makeCapsuleBatchTransform(
        0.45 + offset(rng), offset(rng), offset(rng), angle(rng));

    pairs.push_back(CapsulePair{&capsuleA, &capsuleB, tfA, tfB});
  }

  CollisionOption option;
  std::vector<CollisionResult> batchResults(pairs.size());
  collideCapsulesBatch(pairs, batchResults, option);

  for (std::size_t i = 0; i < pairs.size(); ++i) {
    CollisionResult singleResult;
    ASSERT_TRUE(collideCapsules(
        *pairs[i].shapeA,
        pairs[i].tfA,
        *pairs[i].shapeB,
        pairs[i].tfB,
        singleResult,
        option));
    expectCollisionResultExactlyEqual(singleResult, batchResults[i]);
  }
}

TEST(CapsuleCapsuleBatch, RejectsMalformedInputs)
{
  CapsuleShape capsuleA(0.35, 1.2);
  CapsuleShape capsuleB(0.30, 1.0);

  const Eigen::Isometry3d tfA = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d tfB = Eigen::Isometry3d::Identity();
  tfB.translation() = Eigen::Vector3d(0.45, 0.0, 0.0);

  const std::vector<CapsulePair> validPairs{{&capsuleA, &capsuleB, tfA, tfB}};
  std::vector<CollisionResult> emptyResults;
  EXPECT_THROW(
      collideCapsulesBatch(validPairs, emptyResults), std::invalid_argument);

  const std::vector<CapsulePair> nullShapePairs{{nullptr, &capsuleB, tfA, tfB}};
  std::vector<CollisionResult> results(1);
  EXPECT_THROW(
      collideCapsulesBatch(nullShapePairs, results), std::invalid_argument);
}

TEST(CapsuleSphere, NoCollision)
{
  CapsuleShape capsule(0.5, 2.0);
  SphereShape sphere(0.5);

  Eigen::Isometry3d tfCapsule = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d tfSphere = Eigen::Isometry3d::Identity();
  tfSphere.translation() = Eigen::Vector3d(3.0, 0, 0);

  CollisionResult result;
  bool collided
      = collideCapsuleSphere(capsule, tfCapsule, sphere, tfSphere, result);

  EXPECT_FALSE(collided);
  EXPECT_EQ(result.numContacts(), 0u);
}

TEST(CapsuleSphere, SphereAtMiddle)
{
  CapsuleShape capsule(0.5, 2.0);
  SphereShape sphere(0.5);

  Eigen::Isometry3d tfCapsule = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d tfSphere = Eigen::Isometry3d::Identity();
  tfSphere.translation() = Eigen::Vector3d(0.8, 0, 0);

  CollisionResult result;
  bool collided
      = collideCapsuleSphere(capsule, tfCapsule, sphere, tfSphere, result);

  EXPECT_TRUE(collided);
  EXPECT_EQ(result.numContacts(), 1u);
  EXPECT_NEAR(result.getContact(0).depth, 0.2, 1e-6);
}

TEST(CapsuleSphere, SphereAtTop)
{
  CapsuleShape capsule(0.5, 2.0);
  SphereShape sphere(0.5);

  Eigen::Isometry3d tfCapsule = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d tfSphere = Eigen::Isometry3d::Identity();
  tfSphere.translation() = Eigen::Vector3d(0, 0, 1.8);

  CollisionResult result;
  bool collided
      = collideCapsuleSphere(capsule, tfCapsule, sphere, tfSphere, result);

  EXPECT_TRUE(collided);
  EXPECT_EQ(result.numContacts(), 1u);
}

TEST(CapsuleSphere, SphereAtBottom)
{
  CapsuleShape capsule(0.5, 2.0);
  SphereShape sphere(0.5);

  Eigen::Isometry3d tfCapsule = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d tfSphere = Eigen::Isometry3d::Identity();
  tfSphere.translation() = Eigen::Vector3d(0, 0, -1.5);

  CollisionResult result;
  bool collided
      = collideCapsuleSphere(capsule, tfCapsule, sphere, tfSphere, result);

  EXPECT_TRUE(collided);
  EXPECT_EQ(result.numContacts(), 1u);
}

TEST(CapsuleBox, NoCollision)
{
  CapsuleShape capsule(0.5, 2.0);
  BoxShape box(Eigen::Vector3d(0.5, 0.5, 0.5));

  Eigen::Isometry3d tfCapsule = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d tfBox = Eigen::Isometry3d::Identity();
  tfBox.translation() = Eigen::Vector3d(3.0, 0, 0);

  CollisionResult result;
  bool collided = collideCapsuleBox(capsule, tfCapsule, box, tfBox, result);

  EXPECT_FALSE(collided);
  EXPECT_EQ(result.numContacts(), 0u);
}

TEST(CapsuleBox, CapsuleThroughBoxFace)
{
  CapsuleShape capsule(0.3, 2.0);
  BoxShape box(Eigen::Vector3d(1.0, 1.0, 1.0));

  Eigen::Isometry3d tfCapsule = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d tfBox = Eigen::Isometry3d::Identity();
  tfBox.translation() = Eigen::Vector3d(1.1, 0, 0);

  CollisionResult result;
  bool collided = collideCapsuleBox(capsule, tfCapsule, box, tfBox, result);

  EXPECT_TRUE(collided);
  EXPECT_EQ(result.numContacts(), 1u);
}

TEST(CapsuleBox, CapsuleAtBoxCorner)
{
  CapsuleShape capsule(0.3, 0.5);
  BoxShape box(Eigen::Vector3d(0.5, 0.5, 0.5));

  Eigen::Isometry3d tfCapsule = Eigen::Isometry3d::Identity();
  tfCapsule.translation() = Eigen::Vector3d(0.7, 0.7, 0.7);
  Eigen::Isometry3d tfBox = Eigen::Isometry3d::Identity();

  CollisionResult result;
  bool collided = collideCapsuleBox(capsule, tfCapsule, box, tfBox, result);

  EXPECT_TRUE(collided);
  EXPECT_EQ(result.numContacts(), 1u);
}

TEST(CapsuleBox, CapsulePenetratingBox)
{
  CapsuleShape capsule(0.2, 1.0);
  BoxShape box(Eigen::Vector3d(1.0, 1.0, 1.0));

  Eigen::Isometry3d tfCapsule = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d tfBox = Eigen::Isometry3d::Identity();

  CollisionResult result;
  bool collided = collideCapsuleBox(capsule, tfCapsule, box, tfBox, result);

  EXPECT_TRUE(collided);
  EXPECT_EQ(result.numContacts(), 1u);
}

TEST(CapsuleBox, RotatedCapsule)
{
  CapsuleShape capsule(0.3, 2.0);
  BoxShape box(Eigen::Vector3d(0.5, 0.5, 0.5));

  Eigen::Isometry3d tfCapsule = Eigen::Isometry3d::Identity();
  tfCapsule.linear() = Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitY())
                           .toRotationMatrix();
  tfCapsule.translation() = Eigen::Vector3d(0.6, 0, 0);
  Eigen::Isometry3d tfBox = Eigen::Isometry3d::Identity();

  CollisionResult result;
  bool collided = collideCapsuleBox(capsule, tfCapsule, box, tfBox, result);

  EXPECT_TRUE(collided);
  EXPECT_EQ(result.numContacts(), 1u);
}
