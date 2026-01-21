/*
 * Copyright (c) 2011, The DART development contributors
 * All rights reserved.
 *
 * This file is provided under the following "BSD-style" License:
 *   Redistribution and use in source and binary forms, with or without
 *   modification, are permitted provided that the following conditions
 *   are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <dart/gui/vsg/collision_scene_builder.hpp>
#include <dart/gui/vsg/materials.hpp>

#include <dart/collision/experimental/aabb.hpp>
#include <dart/collision/experimental/collision_object.hpp>
#include <dart/collision/experimental/collision_world.hpp>
#include <dart/collision/experimental/shapes/shape.hpp>
#include <dart/collision/experimental/types.hpp>

#include <gtest/gtest.h>

namespace vsg = dart::gui::vsg;
namespace collision = dart::collision::experimental;

TEST(VsgCollisionSceneBuilder, DefaultConstruction)
{
  vsg::CollisionSceneBuilder builder;
  auto scene = builder.build();
  ASSERT_NE(scene, nullptr);
}

TEST(VsgCollisionSceneBuilder, AddObject)
{
  collision::CollisionWorld world;
  auto obj = world.createObject(std::make_unique<collision::SphereShape>(1.0));

  vsg::CollisionSceneBuilder builder;
  builder.addObject(obj, vsg::colors::Red);

  auto scene = builder.build();
  ASSERT_NE(scene, nullptr);
}

TEST(VsgCollisionSceneBuilder, AddMultipleObjects)
{
  collision::CollisionWorld world;
  auto sphere
      = world.createObject(std::make_unique<collision::SphereShape>(1.0));
  auto box = world.createObject(
      std::make_unique<collision::BoxShape>(Eigen::Vector3d(1, 1, 1)));

  vsg::CollisionSceneBuilder builder;
  builder.addObject(sphere, vsg::colors::Red);
  builder.addObject(box, vsg::colors::Blue);

  auto scene = builder.build();
  ASSERT_NE(scene, nullptr);
}

TEST(VsgCollisionSceneBuilder, AddContacts)
{
  collision::CollisionWorld world;
  auto sphere1
      = world.createObject(std::make_unique<collision::SphereShape>(1.0));
  auto sphere2
      = world.createObject(std::make_unique<collision::SphereShape>(1.0));
  sphere2.setTransform(
      Eigen::Translation3d(1.5, 0, 0) * Eigen::Isometry3d::Identity());

  collision::CollisionOption option;
  option.enableContact = true;
  collision::CollisionResult result;
  world.collide(option, result);

  vsg::CollisionSceneBuilder builder;
  builder.addObject(sphere1);
  builder.addObject(sphere2);
  builder.addContacts(result, 0.2, 0.03);

  auto scene = builder.build();
  ASSERT_NE(scene, nullptr);
}

TEST(VsgCollisionSceneBuilder, AddAabb)
{
  collision::Aabb aabb;
  aabb.min = Eigen::Vector3d(-1, -1, -1);
  aabb.max = Eigen::Vector3d(1, 1, 1);

  vsg::CollisionSceneBuilder builder;
  builder.addAabb(aabb, vsg::colors::Yellow);

  auto scene = builder.build();
  ASSERT_NE(scene, nullptr);
}

TEST(VsgCollisionSceneBuilder, AddDistanceResult)
{
  collision::DistanceResult result;
  result.distance = 1.0;
  result.pointOnObject1 = Eigen::Vector3d(0, 0, 0);
  result.pointOnObject2 = Eigen::Vector3d(1, 0, 0);
  result.normal = Eigen::Vector3d(1, 0, 0);

  vsg::CollisionSceneBuilder builder;
  builder.addDistanceResult(result);

  auto scene = builder.build();
  ASSERT_NE(scene, nullptr);
}

TEST(VsgCollisionSceneBuilder, AddRaycast)
{
  collision::Ray ray(
      Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(1, 0, 0).normalized(), 10.0);

  collision::RaycastResult hit;
  hit.hit = true;
  hit.distance = 5.0;
  hit.point = Eigen::Vector3d(5, 0, 0);
  hit.normal = Eigen::Vector3d(-1, 0, 0);

  vsg::CollisionSceneBuilder builder;
  builder.addRaycast(ray, &hit);

  auto scene = builder.build();
  ASSERT_NE(scene, nullptr);
}

TEST(VsgCollisionSceneBuilder, AddRaycastMiss)
{
  collision::Ray ray(
      Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(1, 0, 0).normalized(), 10.0);

  vsg::CollisionSceneBuilder builder;
  builder.addRaycast(ray, nullptr);

  auto scene = builder.build();
  ASSERT_NE(scene, nullptr);
}

TEST(VsgCollisionSceneBuilder, AddSphereCast)
{
  Eigen::Vector3d start(0, 0, 0);
  Eigen::Vector3d end(10, 0, 0);
  double radius = 0.5;

  vsg::CollisionSceneBuilder builder;
  builder.addSphereCast(start, end, radius, nullptr);

  auto scene = builder.build();
  ASSERT_NE(scene, nullptr);
}

TEST(VsgCollisionSceneBuilder, Clear)
{
  collision::CollisionWorld world;
  auto obj = world.createObject(std::make_unique<collision::SphereShape>(1.0));

  vsg::CollisionSceneBuilder builder;
  builder.addObject(obj);
  builder.clear();

  auto scene = builder.build();
  ASSERT_NE(scene, nullptr);
}
