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

#include <dart/collision/experimental/narrow_phase/box_box.hpp>
#include <dart/collision/experimental/shapes/shape.hpp>
#include <dart/collision/experimental/types.hpp>

#include <gtest/gtest.h>

using namespace dart::collision::experimental;

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
  ASSERT_EQ(result.numContacts(), 1);
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
  ASSERT_EQ(result.numContacts(), 1);
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
  ASSERT_EQ(result.numContacts(), 1);
  EXPECT_NEAR(result.getContact(0).depth, 0.5, 1e-10);
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
  ASSERT_EQ(result.numContacts(), 1);
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
  ASSERT_EQ(result.numContacts(), 1);
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
  ASSERT_EQ(result.numContacts(), 1);
  EXPECT_NEAR(result.getContact(0).depth, 1.0, 1e-10);
}

TEST(BoxBox, Rotated90_AlongX)
{
  CollisionResult result;

  Eigen::Isometry3d t1 = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d t2 = Eigen::Isometry3d::Identity();
  t2.rotate(Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitX()));
  t2.translation() = Eigen::Vector3d(0, 1.5, 0);

  bool collided = collideBoxes(
      Eigen::Vector3d(1, 1, 1), t1, Eigen::Vector3d(1, 1, 1), t2, result);

  EXPECT_TRUE(collided);
  ASSERT_EQ(result.numContacts(), 1);
  EXPECT_NEAR(result.getContact(0).depth, 0.5, 1e-10);
}

TEST(BoxBox, Rotated45_Diagonal)
{
  CollisionResult result;

  Eigen::Isometry3d t1 = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d t2 = Eigen::Isometry3d::Identity();
  t2.rotate(Eigen::AngleAxisd(M_PI / 4, Eigen::Vector3d::UnitZ()));
  t2.translation() = Eigen::Vector3d(2.2, 0, 0);

  bool collided = collideBoxes(
      Eigen::Vector3d(1, 1, 1), t1, Eigen::Vector3d(1, 1, 1), t2, result);

  double rotatedHalfWidth = std::sqrt(2.0);
  double expectedPenetration = 1.0 + rotatedHalfWidth - 2.2;

  if (expectedPenetration > 0) {
    EXPECT_TRUE(collided);
    ASSERT_EQ(result.numContacts(), 1);
    EXPECT_NEAR(result.getContact(0).depth, expectedPenetration, 1e-6);
  } else {
    EXPECT_FALSE(collided);
  }
}

TEST(BoxBox, BothRotated)
{
  CollisionResult result;

  Eigen::Isometry3d t1 = Eigen::Isometry3d::Identity();
  t1.rotate(Eigen::AngleAxisd(M_PI / 4, Eigen::Vector3d::UnitZ()));

  Eigen::Isometry3d t2 = Eigen::Isometry3d::Identity();
  t2.rotate(Eigen::AngleAxisd(-M_PI / 4, Eigen::Vector3d::UnitZ()));
  t2.translation() = Eigen::Vector3d(2, 0, 0);

  bool collided = collideBoxes(
      Eigen::Vector3d(1, 1, 1), t1, Eigen::Vector3d(1, 1, 1), t2, result);

  EXPECT_TRUE(collided);
  ASSERT_EQ(result.numContacts(), 1);
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
  ASSERT_EQ(result.numContacts(), 1);
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
  ASSERT_EQ(result.numContacts(), 1);
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
  ASSERT_EQ(result.numContacts(), 1);

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
  ASSERT_EQ(result.numContacts(), 1);
  EXPECT_NEAR(result.getContact(0).depth, 0.5, 1e-10);
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
