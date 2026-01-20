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

#include <dart/collision/experimental/narrow_phase/sphere_sphere.hpp>
#include <dart/collision/experimental/shapes/shape.hpp>
#include <dart/collision/experimental/types.hpp>

#include <gtest/gtest.h>

using namespace dart::collision::experimental;

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
  transform1.rotate(Eigen::AngleAxisd(M_PI / 4, Eigen::Vector3d::UnitZ()));

  Eigen::Isometry3d transform2 = Eigen::Isometry3d::Identity();
  transform2.translation() = Eigen::Vector3d(1.5, 0, 0);
  transform2.rotate(Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitX()));

  CollisionResult result;

  bool collided
      = collideSpheres(sphere1, transform1, sphere2, transform2, result);

  EXPECT_TRUE(collided);
  ASSERT_EQ(result.numContacts(), 1);
  EXPECT_NEAR(result.getContact(0).depth, 0.5, 1e-10);
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
