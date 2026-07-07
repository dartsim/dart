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
#include <dart/collision/native/types.hpp>

#include <gtest/gtest.h>

#include <array>
#include <stdexcept>

using namespace dart::collision::native;

namespace {

Eigen::Isometry3d translated(double x, double y, double z)
{
  Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
  tf.translation() = Eigen::Vector3d(x, y, z);
  return tf;
}

Eigen::Isometry3d rotatedAroundY(double radians)
{
  Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
  tf.linear()
      = Eigen::AngleAxisd(radians, Eigen::Vector3d::UnitY()).toRotationMatrix();
  return tf;
}

CollisionResult resultWithStoredContact()
{
  CollisionResult result;
  result.addContact(Eigen::Vector3d::Zero(), Eigen::Vector3d::UnitZ(), 0.0);
  return result;
}

} // namespace

TEST(CylinderCollision, CollidesParallelCylinders)
{
  CylinderShape cylinder1(0.5, 2.0);
  CylinderShape cylinder2(0.5, 2.0);

  CollisionResult result;
  const bool hit = collideCylinders(
      cylinder1,
      Eigen::Isometry3d::Identity(),
      cylinder2,
      translated(0.75, 0.0, 0.0),
      result);

  ASSERT_TRUE(hit);
  ASSERT_EQ(1u, result.numContacts());
  EXPECT_TRUE(
      result.getContact(0).normal.isApprox(-Eigen::Vector3d::UnitX(), 1e-12));
}

TEST(CylinderCollision, CoaxialCylindersUseLateralSeparation)
{
  CylinderShape cylinder1(0.5, 4.0);
  CylinderShape cylinder2(0.5, 4.0);

  CollisionResult result;
  const bool hit = collideCylinders(
      cylinder1,
      Eigen::Isometry3d::Identity(),
      cylinder2,
      Eigen::Isometry3d::Identity(),
      result);

  ASSERT_TRUE(hit);
  ASSERT_EQ(1u, result.numContacts());
  EXPECT_TRUE(
      result.getContact(0).normal.isApprox(-Eigen::Vector3d::UnitX(), 1e-12));
  EXPECT_NEAR(1.0, result.getContact(0).depth, 1e-12);
}

TEST(CylinderCollision, RejectsSeparatedCylinders)
{
  CylinderShape cylinder1(0.5, 2.0);
  CylinderShape cylinder2(0.5, 2.0);

  CollisionResult result;
  const bool hit = collideCylinders(
      cylinder1,
      Eigen::Isometry3d::Identity(),
      cylinder2,
      translated(1.5, 0.0, 0.0),
      result);

  EXPECT_FALSE(hit);
  EXPECT_EQ(0u, result.numContacts());
}

TEST(CylinderCollision, CollidesSkewCylinders)
{
  CylinderShape cylinder1(0.5, 2.0);
  CylinderShape cylinder2(0.5, 2.0);
  Eigen::Isometry3d tf2 = rotatedAroundY(0.5);
  tf2.translation() = Eigen::Vector3d(0.5, 0.0, 0.0);

  CollisionResult result;
  const bool hit = collideCylinders(
      cylinder1, Eigen::Isometry3d::Identity(), cylinder2, tf2, result);

  EXPECT_TRUE(hit);
  EXPECT_EQ(1u, result.numContacts());
}

TEST(CylinderCollision, RejectsSeparatedSkewCylinders)
{
  CylinderShape cylinder1(0.5, 1.0);
  CylinderShape cylinder2(0.5, 1.0);
  Eigen::Isometry3d tf2 = rotatedAroundY(1.5707963267948966);
  tf2.translation() = Eigen::Vector3d(0.0, 0.0, 1.1);

  CollisionResult result;
  const bool hit = collideCylinders(
      cylinder1, Eigen::Isometry3d::Identity(), cylinder2, tf2, result);

  EXPECT_FALSE(hit);
  EXPECT_EQ(0u, result.numContacts());

  CollisionResult binaryResult;
  EXPECT_FALSE(collideCylinders(
      cylinder1,
      Eigen::Isometry3d::Identity(),
      cylinder2,
      tf2,
      binaryResult,
      CollisionOption::binaryCheck()));
  EXPECT_EQ(0u, binaryResult.numContacts());
}

TEST(CylinderCollision, BatchCollidesCylinderPairs)
{
  CylinderShape cylinder1(0.5, 2.0);
  CylinderShape cylinder2(0.5, 2.0);
  const std::array<CylinderPair, 2> pairs{{
      {&cylinder1,
       &cylinder2,
       Eigen::Isometry3d::Identity(),
       translated(0.75, 0.0, 0.0)},
      {&cylinder1,
       &cylinder2,
       Eigen::Isometry3d::Identity(),
       translated(1.5, 0.0, 0.0)},
  }};
  std::array<CollisionResult, 2> results;

  collideCylindersBatch(pairs, results);

  EXPECT_EQ(1u, results[0].numContacts());
  EXPECT_EQ(0u, results[1].numContacts());
}

TEST(CylinderCollision, BatchRejectsInvalidInputs)
{
  CylinderShape cylinder(0.5, 2.0);
  const std::array<CylinderPair, 1> pairs{{
      {&cylinder,
       &cylinder,
       Eigen::Isometry3d::Identity(),
       Eigen::Isometry3d::Identity()},
  }};

  EXPECT_THROW(
      collideCylindersBatch(pairs, span<CollisionResult>()),
      std::invalid_argument);

  std::array<CollisionResult, 1> results;
  const std::array<CylinderPair, 1> nullPairs{{
      {nullptr,
       &cylinder,
       Eigen::Isometry3d::Identity(),
       Eigen::Isometry3d::Identity()},
  }};
  EXPECT_THROW(
      collideCylindersBatch(nullPairs, results), std::invalid_argument);
}

TEST(CylinderCollision, CollidesCylinderSphere)
{
  CylinderShape cylinder(0.5, 2.0);
  SphereShape sphere(0.5);

  CollisionResult result;
  const bool hit = collideCylinderSphere(
      cylinder,
      Eigen::Isometry3d::Identity(),
      sphere,
      translated(0.75, 0.0, 0.0),
      result);

  ASSERT_TRUE(hit);
  ASSERT_EQ(1u, result.numContacts());
  EXPECT_TRUE(
      result.getContact(0).normal.isApprox(-Eigen::Vector3d::UnitX(), 1e-12));
}

TEST(CylinderCollision, ContainedCylinderSphereUsesInwardNormal)
{
  CylinderShape cylinder(1.0, 4.0);
  SphereShape sphere(0.5);

  CollisionResult result;
  const bool hit = collideCylinderSphere(
      cylinder,
      Eigen::Isometry3d::Identity(),
      sphere,
      translated(0.75, 0.0, 0.0),
      result);

  ASSERT_TRUE(hit);
  ASSERT_EQ(1u, result.numContacts());
  EXPECT_TRUE(
      result.getContact(0).normal.isApprox(-Eigen::Vector3d::UnitX(), 1e-12));
  EXPECT_NEAR(0.75, result.getContact(0).depth, 1e-12);
}

TEST(CylinderCollision, CollidesCylinderBox)
{
  CylinderShape cylinder(0.5, 2.0);
  BoxShape box(Eigen::Vector3d(0.5, 0.5, 0.5));

  CollisionResult result;
  const bool hit = collideCylinderBox(
      cylinder,
      Eigen::Isometry3d::Identity(),
      box,
      translated(0.75, 0.0, 0.0),
      result);

  ASSERT_TRUE(hit);
  ASSERT_EQ(1u, result.numContacts());
  EXPECT_TRUE(
      result.getContact(0).normal.isApprox(-Eigen::Vector3d::UnitX(), 1e-12));
  EXPECT_NEAR(0.375, result.getContact(0).position.x(), 1e-12);
}

TEST(CylinderCollision, DeepFloorBoxUsesAxialNormal)
{
  CylinderShape cylinder(0.5, 2.0);
  BoxShape floorBox(Eigen::Vector3d(10.0, 10.0, 1.0));

  CollisionResult result;
  const bool hit = collideCylinderBox(
      cylinder,
      Eigen::Isometry3d::Identity(),
      floorBox,
      translated(0.0, 0.0, -1.25),
      result);

  ASSERT_TRUE(hit);
  ASSERT_GE(result.numContacts(), 1u);
  EXPECT_TRUE(
      result.getContact(0).normal.isApprox(Eigen::Vector3d::UnitZ(), 1e-12));
  EXPECT_NEAR(0.75, result.getContact(0).depth, 1e-12);
}

TEST(CylinderCollision, TouchingCylinderBoxCapReportsContact)
{
  CylinderShape cylinder(0.5, 2.0);
  BoxShape box(Eigen::Vector3d(1.0, 1.0, 0.5));

  CollisionResult result;
  const bool hit = collideCylinderBox(
      cylinder,
      Eigen::Isometry3d::Identity(),
      box,
      translated(0.0, 0.0, 1.5),
      result);

  ASSERT_TRUE(hit);
  ASSERT_GE(result.numContacts(), 1u);
  EXPECT_TRUE(
      result.getContact(0).normal.isApprox(-Eigen::Vector3d::UnitZ(), 1e-12));
  EXPECT_NEAR(0.0, result.getContact(0).depth, 1e-12);
}

TEST(CylinderCollision, CylinderBoxCapContactPositionLiesInOverlap)
{
  CylinderShape cylinder(0.5, 2.0);
  BoxShape box(Eigen::Vector3d(1.0, 1.0, 0.5));

  CollisionResult result;
  const bool hit = collideCylinderBox(
      cylinder,
      Eigen::Isometry3d::Identity(),
      box,
      translated(0.0, 0.0, 1.3),
      result);

  ASSERT_TRUE(hit);
  ASSERT_GE(result.numContacts(), 1u);
  for (std::size_t i = 0; i < result.numContacts(); ++i) {
    EXPECT_TRUE(
        result.getContact(i).normal.isApprox(-Eigen::Vector3d::UnitZ(), 1e-12));
    EXPECT_NEAR(0.9, result.getContact(i).position.z(), 1e-12);
    EXPECT_NEAR(0.2, result.getContact(i).depth, 1e-12);
  }
}

TEST(CylinderCollision, EmbeddedBoxUsesLateralSeparation)
{
  CylinderShape cylinder(0.5, 4.0);
  BoxShape box(Eigen::Vector3d(0.1, 0.2, 0.1));

  CollisionResult result;
  const bool hit = collideCylinderBox(
      cylinder,
      Eigen::Isometry3d::Identity(),
      box,
      translated(0, 0, 0),
      result);

  ASSERT_TRUE(hit);
  ASSERT_EQ(1u, result.numContacts());
  EXPECT_TRUE(
      result.getContact(0).normal.isApprox(Eigen::Vector3d::UnitX(), 1e-12));
  EXPECT_NEAR(0.6, result.getContact(0).depth, 1e-12);
}

TEST(CylinderCollision, CollidesCylinderCapsule)
{
  CylinderShape cylinder(0.5, 2.0);
  CapsuleShape capsule(0.5, 2.0);

  CollisionResult result;
  const bool hit = collideCylinderCapsule(
      cylinder,
      Eigen::Isometry3d::Identity(),
      capsule,
      translated(0.75, 0.0, 0.0),
      result);

  ASSERT_TRUE(hit);
  ASSERT_EQ(1u, result.numContacts());
  EXPECT_TRUE(
      result.getContact(0).normal.isApprox(-Eigen::Vector3d::UnitX(), 1e-12));
}

TEST(CylinderCollision, ContainedCapsuleEndpointUsesContainmentDepth)
{
  CylinderShape cylinder(1.0, 4.0);
  CapsuleShape capsule(0.2, 1.0);

  CollisionResult result;
  const bool hit = collideCylinderCapsule(
      cylinder,
      Eigen::Isometry3d::Identity(),
      capsule,
      translated(0.9, 0.0, 0.0),
      result);

  ASSERT_TRUE(hit);
  ASSERT_EQ(1u, result.numContacts());
  EXPECT_TRUE(
      result.getContact(0).normal.isApprox(-Eigen::Vector3d::UnitX(), 1e-12));
  EXPECT_NEAR(0.3, result.getContact(0).depth, 1e-12);
}

TEST(CylinderCollision, CollidesCylinderCapsuleBody)
{
  CylinderShape cylinder(0.5, 2.0);
  CapsuleShape capsule(0.1, 2.0);
  Eigen::Isometry3d capsuleTf = rotatedAroundY(1.5707963267948966);

  CollisionResult result;
  const bool hit = collideCylinderCapsule(
      cylinder, Eigen::Isometry3d::Identity(), capsule, capsuleTf, result);

  EXPECT_TRUE(hit);
  EXPECT_EQ(1u, result.numContacts());
}

TEST(CylinderCollision, CollidesCylinderPlane)
{
  CylinderShape cylinder(0.5, 2.0);
  PlaneShape plane(Eigen::Vector3d::UnitZ(), 0.0);

  CollisionResult result;
  const bool hit = collideCylinderPlane(
      cylinder,
      translated(0.0, 0.0, 0.75),
      plane,
      Eigen::Isometry3d::Identity(),
      result);

  ASSERT_TRUE(hit);
  ASSERT_EQ(1u, result.numContacts());
  EXPECT_TRUE(
      result.getContact(0).normal.isApprox(Eigen::Vector3d::UnitZ(), 1e-12));
}

TEST(CylinderCollision, BinaryChecksDoNotAddContacts)
{
  CylinderShape cylinder(0.5, 2.0);
  SphereShape sphere(0.5);
  BoxShape box(Eigen::Vector3d(0.5, 0.5, 0.5));
  CapsuleShape capsule(0.5, 2.0);
  PlaneShape plane(Eigen::Vector3d::UnitZ(), 0.0);
  CollisionOption option = CollisionOption::binaryCheck();

  CollisionResult cylinderResult;
  EXPECT_TRUE(collideCylinders(
      cylinder,
      Eigen::Isometry3d::Identity(),
      cylinder,
      translated(0.75, 0.0, 0.0),
      cylinderResult,
      option));
  EXPECT_EQ(0u, cylinderResult.numContacts());

  CollisionResult sphereResult;
  EXPECT_TRUE(collideCylinderSphere(
      cylinder,
      Eigen::Isometry3d::Identity(),
      sphere,
      translated(0.75, 0.0, 0.0),
      sphereResult,
      option));
  EXPECT_EQ(0u, sphereResult.numContacts());

  CollisionResult boxResult;
  EXPECT_TRUE(collideCylinderBox(
      cylinder,
      Eigen::Isometry3d::Identity(),
      box,
      translated(0.75, 0.0, 0.0),
      boxResult,
      option));
  EXPECT_EQ(0u, boxResult.numContacts());

  CollisionResult capsuleResult;
  EXPECT_TRUE(collideCylinderCapsule(
      cylinder,
      Eigen::Isometry3d::Identity(),
      capsule,
      translated(0.75, 0.0, 0.0),
      capsuleResult,
      option));
  EXPECT_EQ(0u, capsuleResult.numContacts());

  CollisionResult planeResult;
  EXPECT_TRUE(collideCylinderPlane(
      cylinder,
      translated(0.0, 0.0, 0.75),
      plane,
      Eigen::Isometry3d::Identity(),
      planeResult,
      option));
  EXPECT_EQ(0u, planeResult.numContacts());
}

TEST(CylinderCollision, BinaryChecksIgnoreExistingContacts)
{
  CylinderShape cylinder(0.5, 2.0);
  SphereShape sphere(0.5);
  BoxShape capBox(Eigen::Vector3d(1.0, 1.0, 0.5));
  CapsuleShape capsule(0.5, 2.0);
  PlaneShape plane(Eigen::Vector3d::UnitZ(), 0.0);
  CollisionOption option = CollisionOption::binaryCheck();

  CollisionResult cylinderResult = resultWithStoredContact();
  EXPECT_TRUE(collideCylinders(
      cylinder,
      Eigen::Isometry3d::Identity(),
      cylinder,
      translated(0.75, 0.0, 0.0),
      cylinderResult,
      option));
  EXPECT_EQ(1u, cylinderResult.numContacts());

  CollisionResult sphereResult = resultWithStoredContact();
  EXPECT_TRUE(collideCylinderSphere(
      cylinder,
      Eigen::Isometry3d::Identity(),
      sphere,
      translated(0.75, 0.0, 0.0),
      sphereResult,
      option));
  EXPECT_EQ(1u, sphereResult.numContacts());

  CollisionResult boxResult = resultWithStoredContact();
  EXPECT_TRUE(collideCylinderBox(
      cylinder,
      Eigen::Isometry3d::Identity(),
      capBox,
      translated(0.0, 0.0, 1.5),
      boxResult,
      option));
  EXPECT_EQ(1u, boxResult.numContacts());

  CollisionResult capsuleResult = resultWithStoredContact();
  EXPECT_TRUE(collideCylinderCapsule(
      cylinder,
      Eigen::Isometry3d::Identity(),
      capsule,
      translated(0.75, 0.0, 0.0),
      capsuleResult,
      option));
  EXPECT_EQ(1u, capsuleResult.numContacts());

  CollisionResult planeResult = resultWithStoredContact();
  EXPECT_TRUE(collideCylinderPlane(
      cylinder,
      translated(0.0, 0.0, 0.75),
      plane,
      Eigen::Isometry3d::Identity(),
      planeResult,
      option));
  EXPECT_EQ(1u, planeResult.numContacts());
}

TEST(CylinderCollision, RespectsExhaustedContactBudget)
{
  CylinderShape cylinder1(0.5, 2.0);
  CylinderShape cylinder2(0.5, 2.0);
  CollisionOption option;
  option.maxNumContacts = 0;

  CollisionResult result;
  EXPECT_FALSE(collideCylinders(
      cylinder1,
      Eigen::Isometry3d::Identity(),
      cylinder2,
      translated(0.75, 0.0, 0.0),
      result,
      option));
  EXPECT_EQ(0u, result.numContacts());

  CollisionOption binaryOption = CollisionOption::binaryCheck();
  binaryOption.maxNumContacts = 0;

  CollisionResult binaryResult;
  EXPECT_FALSE(collideCylinders(
      cylinder1,
      Eigen::Isometry3d::Identity(),
      cylinder2,
      translated(0.75, 0.0, 0.0),
      binaryResult,
      binaryOption));
  EXPECT_EQ(0u, binaryResult.numContacts());
}
