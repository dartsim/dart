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

#include <dart/collision/experimental/narrow_phase/sphere_box.hpp>
#include <dart/collision/experimental/shapes/shape.hpp>
#include <dart/collision/experimental/types.hpp>

#include <gtest/gtest.h>

using namespace dart::collision::experimental;

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
  boxTransform.rotate(Eigen::AngleAxisd(M_PI / 4, Eigen::Vector3d::UnitZ()));

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
