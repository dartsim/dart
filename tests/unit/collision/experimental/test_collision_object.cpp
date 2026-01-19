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

#include <dart/collision/experimental/collision_object.hpp>
#include <dart/collision/experimental/shapes/shape.hpp>

#include <gtest/gtest.h>

using namespace dart::collision::experimental;

TEST(CollisionObject, Construction)
{
  auto sphere = std::make_shared<SphereShape>(1.0);
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.translation() = Eigen::Vector3d(1, 2, 3);

  CollisionObject obj(sphere, transform);

  EXPECT_EQ(obj.getShape(), sphere.get());
  EXPECT_EQ(obj.getShapePtr(), sphere);
  EXPECT_TRUE(obj.getTransform().isApprox(transform));
}

TEST(CollisionObject, UniqueIds)
{
  auto sphere1 = std::make_shared<SphereShape>(1.0);
  auto sphere2 = std::make_shared<SphereShape>(2.0);

  CollisionObject obj1(sphere1, Eigen::Isometry3d::Identity());
  CollisionObject obj2(sphere2, Eigen::Isometry3d::Identity());

  EXPECT_NE(obj1.getId(), obj2.getId());
}

TEST(CollisionObject, SetTransform)
{
  auto sphere = std::make_shared<SphereShape>(1.0);
  CollisionObject obj(sphere, Eigen::Isometry3d::Identity());

  Eigen::Isometry3d newTransform = Eigen::Isometry3d::Identity();
  newTransform.translation() = Eigen::Vector3d(5, 6, 7);

  obj.setTransform(newTransform);

  EXPECT_TRUE(obj.getTransform().isApprox(newTransform));
}

TEST(CollisionObject, ComputeAabb_Sphere)
{
  auto sphere = std::make_shared<SphereShape>(1.0);
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.translation() = Eigen::Vector3d(10, 0, 0);

  CollisionObject obj(sphere, transform);

  Aabb aabb = obj.computeAabb();

  EXPECT_NEAR(aabb.min.x(), 9.0, 1e-10);
  EXPECT_NEAR(aabb.max.x(), 11.0, 1e-10);
  EXPECT_NEAR(aabb.min.y(), -1.0, 1e-10);
  EXPECT_NEAR(aabb.max.y(), 1.0, 1e-10);
  EXPECT_NEAR(aabb.min.z(), -1.0, 1e-10);
  EXPECT_NEAR(aabb.max.z(), 1.0, 1e-10);
}

TEST(CollisionObject, ComputeAabb_Box)
{
  auto box = std::make_shared<BoxShape>(Eigen::Vector3d(1, 2, 3));
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();

  CollisionObject obj(box, transform);

  Aabb aabb = obj.computeAabb();

  EXPECT_NEAR(aabb.min.x(), -1.0, 1e-10);
  EXPECT_NEAR(aabb.max.x(), 1.0, 1e-10);
  EXPECT_NEAR(aabb.min.y(), -2.0, 1e-10);
  EXPECT_NEAR(aabb.max.y(), 2.0, 1e-10);
  EXPECT_NEAR(aabb.min.z(), -3.0, 1e-10);
  EXPECT_NEAR(aabb.max.z(), 3.0, 1e-10);
}

TEST(CollisionObject, ComputeAabb_RotatedBox)
{
  auto box = std::make_shared<BoxShape>(Eigen::Vector3d(1, 0.5, 0.5));
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.rotate(Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ()));

  CollisionObject obj(box, transform);

  Aabb aabb = obj.computeAabb();

  EXPECT_NEAR(aabb.min.x(), -0.5, 1e-10);
  EXPECT_NEAR(aabb.max.x(), 0.5, 1e-10);
  EXPECT_NEAR(aabb.min.y(), -1.0, 1e-10);
  EXPECT_NEAR(aabb.max.y(), 1.0, 1e-10);
}

TEST(CollisionObject, UserData)
{
  auto sphere = std::make_shared<SphereShape>(1.0);
  CollisionObject obj(sphere, Eigen::Isometry3d::Identity());

  EXPECT_EQ(obj.getUserData(), nullptr);

  int testData = 42;
  obj.setUserData(&testData);

  EXPECT_EQ(obj.getUserData(), &testData);
  EXPECT_EQ(*static_cast<int*>(obj.getUserData()), 42);
}

TEST(CollisionObject, NullShape)
{
  CollisionObject obj(nullptr, Eigen::Isometry3d::Identity());

  EXPECT_EQ(obj.getShape(), nullptr);
  EXPECT_EQ(obj.getShapePtr(), nullptr);

  Aabb aabb = obj.computeAabb();
  EXPECT_TRUE(aabb.min.isApprox(Eigen::Vector3d::Zero()));
  EXPECT_TRUE(aabb.max.isApprox(Eigen::Vector3d::Zero()));
}

TEST(CollisionObject, AabbUpdatesWithTransform)
{
  auto sphere = std::make_shared<SphereShape>(1.0);
  CollisionObject obj(sphere, Eigen::Isometry3d::Identity());

  Aabb aabb1 = obj.computeAabb();
  EXPECT_NEAR(aabb1.center().x(), 0.0, 1e-10);

  Eigen::Isometry3d newTransform = Eigen::Isometry3d::Identity();
  newTransform.translation() = Eigen::Vector3d(100, 0, 0);
  obj.setTransform(newTransform);

  Aabb aabb2 = obj.computeAabb();
  EXPECT_NEAR(aabb2.center().x(), 100.0, 1e-10);
}

TEST(CollisionObject, SharedShapeOwnership)
{
  auto sphere = std::make_shared<SphereShape>(1.0);
  EXPECT_EQ(sphere.use_count(), 1);

  {
    CollisionObject obj(sphere, Eigen::Isometry3d::Identity());
    EXPECT_EQ(sphere.use_count(), 2);
  }

  EXPECT_EQ(sphere.use_count(), 1);
}
