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
#include <dart/collision/experimental/narrow_phase/narrow_phase.hpp>
#include <dart/collision/experimental/shapes/shape.hpp>

#include <gtest/gtest.h>

using namespace dart::collision::experimental;

TEST(NarrowPhase, IsSupported)
{
  EXPECT_TRUE(NarrowPhase::isSupported(ShapeType::Sphere, ShapeType::Sphere));
  EXPECT_TRUE(NarrowPhase::isSupported(ShapeType::Box, ShapeType::Box));
  EXPECT_TRUE(NarrowPhase::isSupported(ShapeType::Sphere, ShapeType::Box));
  EXPECT_TRUE(NarrowPhase::isSupported(ShapeType::Box, ShapeType::Sphere));
  EXPECT_FALSE(NarrowPhase::isSupported(ShapeType::Mesh, ShapeType::Sphere));
  EXPECT_FALSE(NarrowPhase::isSupported(ShapeType::Capsule, ShapeType::Box));
}

TEST(NarrowPhase, SphereSphere_Colliding)
{
  auto s1 = std::make_shared<SphereShape>(1.0);
  auto s2 = std::make_shared<SphereShape>(1.0);

  Eigen::Isometry3d tf1 = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d tf2 = Eigen::Isometry3d::Identity();
  tf2.translation() = Eigen::Vector3d(1.5, 0, 0);

  CollisionObject obj1(s1, tf1);
  CollisionObject obj2(s2, tf2);

  CollisionOption option;
  CollisionResult result;

  bool hit = NarrowPhase::collide(obj1, obj2, option, result);

  EXPECT_TRUE(hit);
  EXPECT_GE(result.numContacts(), 1u);
}

TEST(NarrowPhase, SphereSphere_Separated)
{
  auto s1 = std::make_shared<SphereShape>(1.0);
  auto s2 = std::make_shared<SphereShape>(1.0);

  Eigen::Isometry3d tf1 = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d tf2 = Eigen::Isometry3d::Identity();
  tf2.translation() = Eigen::Vector3d(3.0, 0, 0);

  CollisionObject obj1(s1, tf1);
  CollisionObject obj2(s2, tf2);

  CollisionOption option;
  CollisionResult result;

  bool hit = NarrowPhase::collide(obj1, obj2, option, result);

  EXPECT_FALSE(hit);
  EXPECT_EQ(result.numContacts(), 0u);
}

TEST(NarrowPhase, BoxBox_Colliding)
{
  auto b1 = std::make_shared<BoxShape>(Eigen::Vector3d(1, 1, 1));
  auto b2 = std::make_shared<BoxShape>(Eigen::Vector3d(1, 1, 1));

  Eigen::Isometry3d tf1 = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d tf2 = Eigen::Isometry3d::Identity();
  tf2.translation() = Eigen::Vector3d(1.5, 0, 0);

  CollisionObject obj1(b1, tf1);
  CollisionObject obj2(b2, tf2);

  CollisionOption option;
  CollisionResult result;

  bool hit = NarrowPhase::collide(obj1, obj2, option, result);

  EXPECT_TRUE(hit);
  EXPECT_GE(result.numContacts(), 1u);
}

TEST(NarrowPhase, SphereBox_Colliding)
{
  auto s = std::make_shared<SphereShape>(1.0);
  auto b = std::make_shared<BoxShape>(Eigen::Vector3d(1, 1, 1));

  Eigen::Isometry3d tf1 = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d tf2 = Eigen::Isometry3d::Identity();
  tf2.translation() = Eigen::Vector3d(1.5, 0, 0);

  CollisionObject obj1(s, tf1);
  CollisionObject obj2(b, tf2);

  CollisionOption option;
  CollisionResult result;

  bool hit = NarrowPhase::collide(obj1, obj2, option, result);

  EXPECT_TRUE(hit);
  EXPECT_GE(result.numContacts(), 1u);
}

TEST(NarrowPhase, BoxSphere_Colliding)
{
  auto b = std::make_shared<BoxShape>(Eigen::Vector3d(1, 1, 1));
  auto s = std::make_shared<SphereShape>(1.0);

  Eigen::Isometry3d tf1 = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d tf2 = Eigen::Isometry3d::Identity();
  tf2.translation() = Eigen::Vector3d(1.5, 0, 0);

  CollisionObject obj1(b, tf1);
  CollisionObject obj2(s, tf2);

  CollisionOption option;
  CollisionResult result;

  bool hit = NarrowPhase::collide(obj1, obj2, option, result);

  EXPECT_TRUE(hit);
  EXPECT_GE(result.numContacts(), 1u);

  EXPECT_NEAR(result.getContact(0).normal.x(), -1.0, 0.01);
}

TEST(NarrowPhase, NullShape_NoCollision)
{
  CollisionObject obj1(nullptr, Eigen::Isometry3d::Identity());
  auto s = std::make_shared<SphereShape>(1.0);
  CollisionObject obj2(s, Eigen::Isometry3d::Identity());

  CollisionOption option;
  CollisionResult result;

  bool hit = NarrowPhase::collide(obj1, obj2, option, result);

  EXPECT_FALSE(hit);
}
