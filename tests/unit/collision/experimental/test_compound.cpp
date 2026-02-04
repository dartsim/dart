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

#include <dart/collision/experimental/collision_world.hpp>
#include <dart/collision/experimental/narrow_phase/narrow_phase.hpp>
#include <dart/collision/experimental/shapes/shape.hpp>

#include <gtest/gtest.h>

#include <memory>

#include <cmath>

using namespace dart::collision::experimental;

namespace {

Eigen::Isometry3d makeTransform(const Eigen::Vector3d& translation)
{
  Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
  tf.translation() = translation;
  return tf;
}

void addSphereChild(
    CompoundShape& compound, double radius, const Eigen::Vector3d& translation)
{
  compound.addChild(
      std::make_unique<SphereShape>(radius), makeTransform(translation));
}

void addBoxChild(
    CompoundShape& compound,
    const Eigen::Vector3d& halfExtents,
    const Eigen::Isometry3d& transform)
{
  compound.addChild(std::make_unique<BoxShape>(halfExtents), transform);
}

} // namespace

TEST(CompoundShape, ConstructionEmpty)
{
  CompoundShape compound;

  EXPECT_EQ(compound.getType(), ShapeType::Compound);
  EXPECT_EQ(compound.numChildren(), 0u);

  Aabb aabb = compound.computeLocalAabb();
  EXPECT_EQ(aabb.min, Eigen::Vector3d::Zero());
  EXPECT_EQ(aabb.max, Eigen::Vector3d::Zero());
}

TEST(CompoundShape, ConstructionWithChildren)
{
  std::vector<CompoundShape::ChildShape> children;
  CompoundShape::ChildShape child1;
  child1.shape = std::make_unique<SphereShape>(1.0);
  child1.localTransform = Eigen::Isometry3d::Identity();
  children.push_back(std::move(child1));

  CompoundShape::ChildShape child2;
  child2.shape = std::make_unique<BoxShape>(Eigen::Vector3d(1.0, 2.0, 3.0));
  child2.localTransform = makeTransform(Eigen::Vector3d(1.0, 0.0, 0.0));
  children.push_back(std::move(child2));

  CompoundShape compound(std::move(children));

  EXPECT_EQ(compound.numChildren(), 2u);
  EXPECT_EQ(compound.childShape(0).getType(), ShapeType::Sphere);
  EXPECT_EQ(compound.childShape(1).getType(), ShapeType::Box);
}

TEST(CompoundShape, AddChild)
{
  CompoundShape compound;
  compound.addChild(std::make_unique<SphereShape>(0.5));

  EXPECT_EQ(compound.numChildren(), 1u);
  EXPECT_EQ(compound.childShape(0).getType(), ShapeType::Sphere);
}

TEST(CompoundShape, RemoveChild)
{
  CompoundShape compound;
  compound.addChild(std::make_unique<SphereShape>(0.5));
  compound.addChild(std::make_unique<BoxShape>(Eigen::Vector3d(1, 1, 1)));

  compound.removeChild(0);

  EXPECT_EQ(compound.numChildren(), 1u);
  EXPECT_EQ(compound.childShape(0).getType(), ShapeType::Box);
}

TEST(CompoundShape, RemoveChildOutOfRange)
{
  CompoundShape compound;
  compound.addChild(std::make_unique<SphereShape>(0.5));

  compound.removeChild(5);

  EXPECT_EQ(compound.numChildren(), 1u);
}

TEST(CompoundShape, ChildAccess)
{
  CompoundShape compound;
  Eigen::Isometry3d tf = makeTransform(Eigen::Vector3d(1.0, 2.0, 3.0));
  compound.addChild(std::make_unique<BoxShape>(Eigen::Vector3d(1, 1, 1)), tf);

  EXPECT_EQ(compound.childShape(0).getType(), ShapeType::Box);
  EXPECT_EQ(compound.childTransform(0).translation(), tf.translation());
}

TEST(CompoundShape, ComputeLocalAabbSingleChild)
{
  CompoundShape compound;
  addSphereChild(compound, 1.0, Eigen::Vector3d::Zero());

  Aabb aabb = compound.computeLocalAabb();
  EXPECT_EQ(aabb.min, Eigen::Vector3d(-1, -1, -1));
  EXPECT_EQ(aabb.max, Eigen::Vector3d(1, 1, 1));
}

TEST(CompoundShape, ComputeLocalAabbTranslatedChild)
{
  CompoundShape compound;
  addSphereChild(compound, 1.0, Eigen::Vector3d(2.0, 0.0, 0.0));

  Aabb aabb = compound.computeLocalAabb();
  EXPECT_EQ(aabb.min, Eigen::Vector3d(1, -1, -1));
  EXPECT_EQ(aabb.max, Eigen::Vector3d(3, 1, 1));
}

TEST(CompoundShape, ComputeLocalAabbMultipleChildren)
{
  CompoundShape compound;
  addSphereChild(compound, 1.0, Eigen::Vector3d(2.0, 0.0, 0.0));
  addBoxChild(
      compound,
      Eigen::Vector3d(0.5, 0.5, 0.5),
      makeTransform(Eigen::Vector3d(-2.0, 0.0, 0.0)));

  Aabb aabb = compound.computeLocalAabb();
  EXPECT_EQ(aabb.min, Eigen::Vector3d(-2.5, -1, -1));
  EXPECT_EQ(aabb.max, Eigen::Vector3d(3, 1, 1));
}

TEST(CompoundShape, ComputeLocalAabbRotatedChild)
{
  CompoundShape compound;
  Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
  tf.rotate(Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitZ()));
  addBoxChild(compound, Eigen::Vector3d(1, 2, 3), tf);

  Aabb aabb = compound.computeLocalAabb();
  EXPECT_NEAR(aabb.min.x(), -2.0, 1e-12);
  EXPECT_NEAR(aabb.min.y(), -1.0, 1e-12);
  EXPECT_NEAR(aabb.min.z(), -3.0, 1e-12);
  EXPECT_NEAR(aabb.max.x(), 2.0, 1e-12);
  EXPECT_NEAR(aabb.max.y(), 1.0, 1e-12);
  EXPECT_NEAR(aabb.max.z(), 3.0, 1e-12);
}

TEST(CompoundCollision, SphereChild)
{
  auto compound = std::make_unique<CompoundShape>();
  addSphereChild(*compound, 1.0, Eigen::Vector3d::Zero());

  CollisionWorld world;
  auto compoundObj = world.createObject(std::move(compound));

  Eigen::Isometry3d tfSphere = Eigen::Isometry3d::Identity();
  tfSphere.translation() = Eigen::Vector3d(1.5, 0, 0);
  auto sphereObj
      = world.createObject(std::make_unique<SphereShape>(1.0), tfSphere);

  CollisionOption option;
  CollisionResult result;

  EXPECT_TRUE(NarrowPhase::collide(compoundObj, sphereObj, option, result));
}

TEST(CompoundCollision, BoxChild)
{
  auto compound = std::make_unique<CompoundShape>();
  addBoxChild(
      *compound, Eigen::Vector3d(1, 1, 1), Eigen::Isometry3d::Identity());

  CollisionWorld world;
  auto compoundObj = world.createObject(std::move(compound));

  Eigen::Isometry3d tfBox = Eigen::Isometry3d::Identity();
  tfBox.translation() = Eigen::Vector3d(1.5, 0, 0);
  auto boxObj = world.createObject(
      std::make_unique<BoxShape>(Eigen::Vector3d(1, 1, 1)), tfBox);

  CollisionOption option;
  CollisionResult result;

  EXPECT_TRUE(NarrowPhase::collide(compoundObj, boxObj, option, result));
}

TEST(CompoundCollision, TransformedChild)
{
  auto compound = std::make_unique<CompoundShape>();
  addBoxChild(
      *compound,
      Eigen::Vector3d(1, 1, 1),
      makeTransform(Eigen::Vector3d(2.0, 0.0, 0.0)));

  CollisionWorld world;
  auto compoundObj = world.createObject(std::move(compound));

  Eigen::Isometry3d tfBox = Eigen::Isometry3d::Identity();
  tfBox.translation() = Eigen::Vector3d(3.5, 0.0, 0.0);
  auto boxObj = world.createObject(
      std::make_unique<BoxShape>(Eigen::Vector3d(1, 1, 1)), tfBox);

  CollisionOption option;
  CollisionResult result;

  EXPECT_TRUE(NarrowPhase::collide(compoundObj, boxObj, option, result));
}

TEST(CompoundCollision, NoCollisionSeparatedChildren)
{
  auto compound = std::make_unique<CompoundShape>();
  addSphereChild(*compound, 0.5, Eigen::Vector3d(5.0, 0.0, 0.0));
  addSphereChild(*compound, 0.5, Eigen::Vector3d(-5.0, 0.0, 0.0));

  CollisionWorld world;
  auto compoundObj = world.createObject(std::move(compound));
  auto sphereObj = world.createObject(std::make_unique<SphereShape>(0.5));

  CollisionOption option;
  CollisionResult result;

  EXPECT_FALSE(NarrowPhase::collide(compoundObj, sphereObj, option, result));
}

TEST(CompoundCollision, CompoundCompound)
{
  auto compound1 = std::make_unique<CompoundShape>();
  addSphereChild(*compound1, 1.0, Eigen::Vector3d::Zero());

  auto compound2 = std::make_unique<CompoundShape>();
  addSphereChild(*compound2, 1.0, Eigen::Vector3d(1.5, 0.0, 0.0));

  CollisionWorld world;
  auto obj1 = world.createObject(std::move(compound1));
  auto obj2 = world.createObject(std::move(compound2));

  CollisionOption option;
  CollisionResult result;

  EXPECT_TRUE(NarrowPhase::collide(obj1, obj2, option, result));
}

TEST(CompoundCollision, CompoundCompoundSeparated)
{
  auto compound1 = std::make_unique<CompoundShape>();
  addSphereChild(*compound1, 1.0, Eigen::Vector3d::Zero());

  auto compound2 = std::make_unique<CompoundShape>();
  addSphereChild(*compound2, 1.0, Eigen::Vector3d(4.0, 0.0, 0.0));

  CollisionWorld world;
  auto obj1 = world.createObject(std::move(compound1));
  auto obj2 = world.createObject(std::move(compound2));

  CollisionOption option;
  CollisionResult result;

  EXPECT_FALSE(NarrowPhase::collide(obj1, obj2, option, result));
}

TEST(CompoundDistance, Sphere)
{
  auto compound = std::make_unique<CompoundShape>();
  addSphereChild(*compound, 1.0, Eigen::Vector3d(4.0, 0.0, 0.0));

  CollisionWorld world;
  auto compoundObj = world.createObject(std::move(compound));
  auto sphereObj = world.createObject(std::make_unique<SphereShape>(1.0));

  DistanceOption option;
  DistanceResult result;

  double dist = NarrowPhase::distance(compoundObj, sphereObj, option, result);

  EXPECT_NEAR(dist, 2.0, 1e-6);
}

TEST(CompoundDistance, Box)
{
  auto compound = std::make_unique<CompoundShape>();
  addBoxChild(
      *compound,
      Eigen::Vector3d(0.5, 0.5, 0.5),
      makeTransform(Eigen::Vector3d(2.0, 0.0, 0.0)));

  CollisionWorld world;
  auto compoundObj = world.createObject(std::move(compound));
  auto boxObj = world.createObject(
      std::make_unique<BoxShape>(Eigen::Vector3d(0.5, 0.5, 0.5)));

  DistanceOption option;
  DistanceResult result;

  double dist = NarrowPhase::distance(compoundObj, boxObj, option, result);

  EXPECT_NEAR(dist, 1.0, 1e-6);
}

TEST(CompoundDistance, CompoundCompound)
{
  auto compound1 = std::make_unique<CompoundShape>();
  addSphereChild(*compound1, 1.0, Eigen::Vector3d::Zero());

  auto compound2 = std::make_unique<CompoundShape>();
  addSphereChild(*compound2, 1.0, Eigen::Vector3d(5.0, 0.0, 0.0));

  CollisionWorld world;
  auto obj1 = world.createObject(std::move(compound1));
  auto obj2 = world.createObject(std::move(compound2));

  DistanceOption option;
  DistanceResult result;

  double dist = NarrowPhase::distance(obj1, obj2, option, result);

  EXPECT_NEAR(dist, 3.0, 1e-6);
}

TEST(CompoundDistance, NearestChild)
{
  auto compound = std::make_unique<CompoundShape>();
  addSphereChild(*compound, 1.0, Eigen::Vector3d::Zero());
  addSphereChild(*compound, 1.0, Eigen::Vector3d(5.0, 0.0, 0.0));

  CollisionWorld world;
  auto compoundObj = world.createObject(std::move(compound));

  Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
  tf.translation() = Eigen::Vector3d(3.0, 0.0, 0.0);
  auto sphereObj = world.createObject(std::make_unique<SphereShape>(1.0), tf);

  DistanceOption option;
  DistanceResult result;

  double dist = NarrowPhase::distance(compoundObj, sphereObj, option, result);

  EXPECT_NEAR(dist, 0.0, 1e-6);
}

TEST(CompoundRaycast, HitChild)
{
  auto compound = std::make_unique<CompoundShape>();
  addSphereChild(*compound, 1.0, Eigen::Vector3d(0.0, 0.0, 5.0));

  CollisionWorld world;
  auto compoundObj = world.createObject(std::move(compound));

  Ray ray(Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(0, 0, 1));
  RaycastOption option;
  RaycastResult result;

  EXPECT_TRUE(NarrowPhase::raycast(ray, compoundObj, option, result));
  EXPECT_NEAR(result.distance, 4.0, 1e-10);
}

TEST(CompoundRaycast, MissAllChildren)
{
  auto compound = std::make_unique<CompoundShape>();
  addSphereChild(*compound, 1.0, Eigen::Vector3d(0.0, 0.0, 5.0));

  CollisionWorld world;
  auto compoundObj = world.createObject(std::move(compound));

  Ray ray(Eigen::Vector3d(5, 0, 0), Eigen::Vector3d(0, 0, 1));
  RaycastOption option;
  RaycastResult result;

  EXPECT_FALSE(NarrowPhase::raycast(ray, compoundObj, option, result));
}

TEST(CompoundRaycast, ClosestChild)
{
  auto compound = std::make_unique<CompoundShape>();
  addSphereChild(*compound, 1.0, Eigen::Vector3d(0.0, 0.0, 5.0));
  addSphereChild(*compound, 1.0, Eigen::Vector3d(0.0, 0.0, 8.0));

  CollisionWorld world;
  auto compoundObj = world.createObject(std::move(compound));

  Ray ray(Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(0, 0, 1));
  RaycastOption option;
  RaycastResult result;

  EXPECT_TRUE(NarrowPhase::raycast(ray, compoundObj, option, result));
  EXPECT_NEAR(result.point.z(), 4.0, 1e-10);
}

TEST(CompoundRaycast, RotatedChild)
{
  auto compound = std::make_unique<CompoundShape>();
  Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
  tf.translation() = Eigen::Vector3d(0.0, 0.0, 5.0);
  tf.rotate(Eigen::AngleAxisd(M_PI / 4.0, Eigen::Vector3d::UnitY()));
  addBoxChild(*compound, Eigen::Vector3d(1, 1, 1), tf);

  CollisionWorld world;
  auto compoundObj = world.createObject(std::move(compound));

  Ray ray(Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(0, 0, 1));
  RaycastOption option;
  RaycastResult result;

  EXPECT_TRUE(NarrowPhase::raycast(ray, compoundObj, option, result));
  EXPECT_LT(result.distance, 5.0);
}

TEST(CompoundSphereCast, HitChild)
{
  auto compound = std::make_unique<CompoundShape>();
  addSphereChild(*compound, 1.0, Eigen::Vector3d(0.0, 0.0, 5.0));

  CollisionWorld world;
  auto compoundObj = world.createObject(std::move(compound));

  Eigen::Vector3d start(0, 0, 0);
  Eigen::Vector3d end(0, 0, 10);
  double radius = 0.5;

  CcdOption option;
  CcdResult result;

  EXPECT_TRUE(
      NarrowPhase::sphereCast(start, end, radius, compoundObj, option, result));
  EXPECT_GT(result.timeOfImpact, 0.0);
  EXPECT_LT(result.timeOfImpact, 1.0);
}

TEST(CompoundSphereCast, ClosestChild)
{
  auto compound = std::make_unique<CompoundShape>();
  addSphereChild(*compound, 1.0, Eigen::Vector3d(0.0, 0.0, 5.0));
  addSphereChild(*compound, 1.0, Eigen::Vector3d(0.0, 0.0, 8.0));

  CollisionWorld world;
  auto compoundObj = world.createObject(std::move(compound));

  Eigen::Vector3d start(0, 0, 0);
  Eigen::Vector3d end(0, 0, 10);
  double radius = 0.5;

  CcdOption option;
  CcdResult result;

  EXPECT_TRUE(
      NarrowPhase::sphereCast(start, end, radius, compoundObj, option, result));
  EXPECT_NEAR(result.point.z(), 4.0, 1e-6);
}

TEST(CompoundCapsuleCast, HitChild)
{
  auto compound = std::make_unique<CompoundShape>();
  addBoxChild(
      *compound,
      Eigen::Vector3d(1, 1, 1),
      makeTransform(Eigen::Vector3d(0.0, 0.0, 5.0)));

  CollisionWorld world;
  auto compoundObj = world.createObject(std::move(compound));

  CapsuleShape capsule(0.5, 2.0);
  Eigen::Isometry3d capsuleStart = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d capsuleEnd = Eigen::Isometry3d::Identity();
  capsuleEnd.translation() = Eigen::Vector3d(0, 0, 10);

  CcdOption option;
  CcdResult result;

  EXPECT_TRUE(
      NarrowPhase::capsuleCast(
          capsuleStart, capsuleEnd, capsule, compoundObj, option, result));
  EXPECT_GT(result.timeOfImpact, 0.0);
  EXPECT_LT(result.timeOfImpact, 1.0);
}

TEST(CompoundCapsuleCast, ClosestChild)
{
  auto compound = std::make_unique<CompoundShape>();
  addSphereChild(*compound, 1.0, Eigen::Vector3d(0.0, 0.0, 4.0));
  addSphereChild(*compound, 1.0, Eigen::Vector3d(0.0, 0.0, 8.0));

  CollisionWorld world;
  auto compoundObj = world.createObject(std::move(compound));

  CapsuleShape capsule(0.5, 2.0);
  Eigen::Isometry3d capsuleStart = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d capsuleEnd = Eigen::Isometry3d::Identity();
  capsuleEnd.translation() = Eigen::Vector3d(0, 0, 10);

  CcdOption option;
  CcdResult result;

  EXPECT_TRUE(
      NarrowPhase::capsuleCast(
          capsuleStart, capsuleEnd, capsule, compoundObj, option, result));
  EXPECT_LT(result.point.z(), 6.0);
}

TEST(NarrowPhaseCompound, IsSupported)
{
  EXPECT_TRUE(NarrowPhase::isSupported(ShapeType::Compound, ShapeType::Sphere));
  EXPECT_TRUE(NarrowPhase::isSupported(ShapeType::Sphere, ShapeType::Compound));
  EXPECT_TRUE(
      NarrowPhase::isSupported(ShapeType::Compound, ShapeType::Compound));
}

TEST(NarrowPhaseCompound, IsDistanceSupported)
{
  EXPECT_TRUE(
      NarrowPhase::isDistanceSupported(ShapeType::Compound, ShapeType::Sphere));
  EXPECT_TRUE(
      NarrowPhase::isDistanceSupported(ShapeType::Sphere, ShapeType::Compound));
}

TEST(NarrowPhaseCompound, IsRaycastSupported)
{
  EXPECT_TRUE(NarrowPhase::isRaycastSupported(ShapeType::Compound));
}

TEST(NarrowPhaseCompound, IsSphereCastSupported)
{
  EXPECT_TRUE(NarrowPhase::isSphereCastSupported(ShapeType::Compound));
}

TEST(NarrowPhaseCompound, IsCapsuleCastSupported)
{
  EXPECT_TRUE(NarrowPhase::isCapsuleCastSupported(ShapeType::Compound));
}

TEST(CollisionWorldCompound, PrimitiveCollision)
{
  CollisionWorld world;

  auto compound = std::make_unique<CompoundShape>();
  addBoxChild(
      *compound, Eigen::Vector3d(1, 1, 1), Eigen::Isometry3d::Identity());
  auto compoundObj = world.createObject(std::move(compound));

  Eigen::Isometry3d tfSphere = Eigen::Isometry3d::Identity();
  tfSphere.translation() = Eigen::Vector3d(1.5, 0, 0);
  world.createObject(std::make_unique<SphereShape>(1.0), tfSphere);

  CollisionOption option;
  CollisionResult result;

  EXPECT_TRUE(world.collide(option, result));
  EXPECT_GE(result.numContacts(), 1u);
  EXPECT_TRUE(compoundObj.isValid());
}
