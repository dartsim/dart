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

#include <random>
#include <stdexcept>
#include <vector>

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
  tf2.linear() = Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitY())
                     .toRotationMatrix();
  tf2.translation() = Eigen::Vector3d(0.8, 0, 0);

  CollisionResult result;
  bool collided = collideCylinders(cyl1, tf1, cyl2, tf2, result);

  EXPECT_TRUE(collided);
  EXPECT_GE(result.numContacts(), 1u);
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

TEST(CylinderPlane, CylinderLyingOnPlane)
{
  CylinderShape cylinder(0.5, 2.0);
  PlaneShape plane(Eigen::Vector3d::UnitZ(), 0.0);

  Eigen::Isometry3d tfCylinder = Eigen::Isometry3d::Identity();
  tfCylinder.linear() = Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitY())
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
  tfCylinder.linear() = Eigen::AngleAxisd(M_PI / 4, Eigen::Vector3d::UnitY())
                            .toRotationMatrix();
  tfCylinder.translation() = Eigen::Vector3d(0, 0, 0.5);
  Eigen::Isometry3d tfPlane = Eigen::Isometry3d::Identity();

  CollisionResult result;
  bool collided
      = collideCylinderPlane(cylinder, tfCylinder, plane, tfPlane, result);

  EXPECT_TRUE(collided);
  EXPECT_GE(result.numContacts(), 1u);
}
