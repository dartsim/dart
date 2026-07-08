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

#include <dart/collision/native/narrow_phase/plane_sphere.hpp>
#include <dart/collision/native/shapes/shape.hpp>
#include <dart/collision/native/types.hpp>

#include <gtest/gtest.h>

using namespace dart::collision::native;

namespace {

Eigen::Isometry3d translated(double x, double y, double z)
{
  Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
  tf.translation() = Eigen::Vector3d(x, y, z);
  return tf;
}

} // namespace

TEST(PlaneCollision, SphereContacts)
{
  PlaneShape plane(Eigen::Vector3d::UnitZ(), 0.0);
  SphereShape sphere(1.0);

  CollisionResult separated;
  EXPECT_FALSE(collidePlaneSphere(
      plane,
      Eigen::Isometry3d::Identity(),
      sphere,
      translated(0.0, 0.0, 2.0),
      separated));
  EXPECT_EQ(0u, separated.numContacts());

  CollisionResult touching;
  EXPECT_TRUE(collidePlaneSphere(
      plane,
      Eigen::Isometry3d::Identity(),
      sphere,
      translated(0.0, 0.0, 1.0),
      touching));
  ASSERT_EQ(1u, touching.numContacts());
  EXPECT_NEAR(0.0, touching.getContact(0).depth, 1e-12);

  CollisionResult penetrating;
  EXPECT_TRUE(collidePlaneSphere(
      plane,
      Eigen::Isometry3d::Identity(),
      sphere,
      translated(0.0, 0.0, 0.5),
      penetrating));
  ASSERT_EQ(1u, penetrating.numContacts());
  EXPECT_NEAR(0.5, penetrating.getContact(0).depth, 1e-12);
  EXPECT_TRUE(
      penetrating.getContact(0).normal.isApprox(Eigen::Vector3d::UnitZ()));
}

TEST(PlaneCollision, BoxCapsuleAndConvexContacts)
{
  PlaneShape plane(Eigen::Vector3d::UnitZ(), 0.0);

  BoxShape box(Eigen::Vector3d(1.0, 1.0, 1.0));
  CollisionResult boxResult;
  EXPECT_TRUE(collidePlaneBox(
      plane,
      Eigen::Isometry3d::Identity(),
      box,
      translated(0.0, 0.0, 0.5),
      boxResult));
  ASSERT_EQ(1u, boxResult.numContacts());
  EXPECT_NEAR(0.5, boxResult.getContact(0).depth, 1e-12);

  CapsuleShape capsule(0.5, 2.0);
  CollisionResult capsuleResult;
  EXPECT_TRUE(collidePlaneCapsule(
      plane,
      Eigen::Isometry3d::Identity(),
      capsule,
      translated(0.0, 0.0, 1.3),
      capsuleResult));
  ASSERT_EQ(1u, capsuleResult.numContacts());
  EXPECT_NEAR(0.2, capsuleResult.getContact(0).depth, 1e-12);

  CollisionResult deepCapsuleResult;
  EXPECT_TRUE(collidePlaneCapsule(
      plane,
      Eigen::Isometry3d::Identity(),
      capsule,
      translated(0.0, 0.0, 0.3),
      deepCapsuleResult));
  ASSERT_EQ(1u, deepCapsuleResult.numContacts());
  EXPECT_NEAR(1.2, deepCapsuleResult.getContact(0).depth, 1e-12);
  EXPECT_NEAR(-0.6, deepCapsuleResult.getContact(0).position.z(), 1e-12);

  ConvexShape convex(
      {Eigen::Vector3d(-0.5, -0.5, -0.25),
       Eigen::Vector3d(0.5, -0.5, -0.25),
       Eigen::Vector3d(0.0, 0.5, -0.25),
       Eigen::Vector3d(0.0, 0.0, 0.75)});
  CollisionResult convexResult;
  EXPECT_TRUE(collidePlaneConvex(
      plane,
      Eigen::Isometry3d::Identity(),
      convex,
      Eigen::Isometry3d::Identity(),
      convexResult));
  ASSERT_EQ(1u, convexResult.numContacts());
  EXPECT_NEAR(0.25, convexResult.getContact(0).depth, 1e-12);
}

TEST(PlaneCollision, BinaryChecksDoNotAddContacts)
{
  PlaneShape plane(Eigen::Vector3d::UnitZ(), 0.0);
  SphereShape sphere(1.0);
  BoxShape box(Eigen::Vector3d(1.0, 1.0, 1.0));
  CapsuleShape capsule(0.5, 2.0);
  ConvexShape convex(
      {Eigen::Vector3d(-0.5, -0.5, -0.25),
       Eigen::Vector3d(0.5, -0.5, -0.25),
       Eigen::Vector3d(0.0, 0.5, -0.25),
       Eigen::Vector3d(0.0, 0.0, 0.75)});

  CollisionResult sphereResult;
  EXPECT_TRUE(collidePlaneSphere(
      plane,
      Eigen::Isometry3d::Identity(),
      sphere,
      translated(0.0, 0.0, 0.5),
      sphereResult,
      CollisionOption::binaryCheck()));
  EXPECT_EQ(0u, sphereResult.numContacts());

  CollisionResult boxResult;
  EXPECT_TRUE(collidePlaneBox(
      plane,
      Eigen::Isometry3d::Identity(),
      box,
      translated(0.0, 0.0, 0.5),
      boxResult,
      CollisionOption::binaryCheck()));
  EXPECT_EQ(0u, boxResult.numContacts());

  CollisionResult capsuleResult;
  EXPECT_TRUE(collidePlaneCapsule(
      plane,
      Eigen::Isometry3d::Identity(),
      capsule,
      translated(0.0, 0.0, 1.3),
      capsuleResult,
      CollisionOption::binaryCheck()));
  EXPECT_EQ(0u, capsuleResult.numContacts());

  CollisionResult convexResult;
  EXPECT_TRUE(collidePlaneConvex(
      plane,
      Eigen::Isometry3d::Identity(),
      convex,
      Eigen::Isometry3d::Identity(),
      convexResult,
      CollisionOption::binaryCheck()));
  EXPECT_EQ(0u, convexResult.numContacts());
}
