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

#include <dart/collision/experimental/narrow_phase/plane_sphere.hpp>
#include <dart/collision/experimental/shapes/shape.hpp>

#include <gtest/gtest.h>

using namespace dart::collision::experimental;

TEST(PlaneShape, Construction)
{
  PlaneShape plane(Eigen::Vector3d::UnitZ(), 0.0);

  EXPECT_EQ(plane.getType(), ShapeType::Plane);
  EXPECT_EQ(plane.getNormal(), Eigen::Vector3d::UnitZ());
  EXPECT_DOUBLE_EQ(plane.getOffset(), 0.0);
}

TEST(PlaneShape, NormalNormalization)
{
  PlaneShape plane(Eigen::Vector3d(0, 0, 2), 1.0);

  EXPECT_TRUE(plane.getNormal().isApprox(Eigen::Vector3d::UnitZ()));
}

TEST(PlaneSphere, NoCollision)
{
  PlaneShape plane(Eigen::Vector3d::UnitZ(), 0.0);
  SphereShape sphere(1.0);

  Eigen::Isometry3d tfPlane = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d tfSphere = Eigen::Isometry3d::Identity();
  tfSphere.translation() = Eigen::Vector3d(0, 0, 2.0);

  CollisionResult result;
  bool collided = collidePlaneSphere(plane, tfPlane, sphere, tfSphere, result);

  EXPECT_FALSE(collided);
  EXPECT_EQ(result.numContacts(), 0u);
}

TEST(PlaneSphere, Touching)
{
  PlaneShape plane(Eigen::Vector3d::UnitZ(), 0.0);
  SphereShape sphere(1.0);

  Eigen::Isometry3d tfPlane = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d tfSphere = Eigen::Isometry3d::Identity();
  tfSphere.translation() = Eigen::Vector3d(0, 0, 1.0);

  CollisionResult result;
  bool collided = collidePlaneSphere(plane, tfPlane, sphere, tfSphere, result);

  EXPECT_TRUE(collided);
  EXPECT_EQ(result.numContacts(), 1u);
  EXPECT_NEAR(result.getContact(0).depth, 0.0, 1e-10);
}

TEST(PlaneSphere, Penetrating)
{
  PlaneShape plane(Eigen::Vector3d::UnitZ(), 0.0);
  SphereShape sphere(1.0);

  Eigen::Isometry3d tfPlane = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d tfSphere = Eigen::Isometry3d::Identity();
  tfSphere.translation() = Eigen::Vector3d(0, 0, 0.5);

  CollisionResult result;
  bool collided = collidePlaneSphere(plane, tfPlane, sphere, tfSphere, result);

  EXPECT_TRUE(collided);
  EXPECT_EQ(result.numContacts(), 1u);
  EXPECT_NEAR(result.getContact(0).depth, 0.5, 1e-10);
  EXPECT_TRUE(result.getContact(0).normal.isApprox(Eigen::Vector3d::UnitZ()));
}

TEST(PlaneBox, NoCollision)
{
  PlaneShape plane(Eigen::Vector3d::UnitZ(), 0.0);
  BoxShape box(Eigen::Vector3d(1, 1, 1));

  Eigen::Isometry3d tfPlane = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d tfBox = Eigen::Isometry3d::Identity();
  tfBox.translation() = Eigen::Vector3d(0, 0, 2.0);

  CollisionResult result;
  bool collided = collidePlaneBox(plane, tfPlane, box, tfBox, result);

  EXPECT_FALSE(collided);
  EXPECT_EQ(result.numContacts(), 0u);
}

TEST(PlaneBox, Penetrating)
{
  PlaneShape plane(Eigen::Vector3d::UnitZ(), 0.0);
  BoxShape box(Eigen::Vector3d(1, 1, 1));

  Eigen::Isometry3d tfPlane = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d tfBox = Eigen::Isometry3d::Identity();
  tfBox.translation() = Eigen::Vector3d(0, 0, 0.5);

  CollisionResult result;
  bool collided = collidePlaneBox(plane, tfPlane, box, tfBox, result);

  EXPECT_TRUE(collided);
  EXPECT_EQ(result.numContacts(), 1u);
  EXPECT_NEAR(result.getContact(0).depth, 0.5, 1e-6);
}

TEST(PlaneBox, RotatedBox)
{
  PlaneShape plane(Eigen::Vector3d::UnitZ(), 0.0);
  BoxShape box(Eigen::Vector3d(1, 1, 1));

  Eigen::Isometry3d tfPlane = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d tfBox = Eigen::Isometry3d::Identity();
  tfBox.linear() = Eigen::AngleAxisd(M_PI / 4, Eigen::Vector3d::UnitY())
                       .toRotationMatrix();
  tfBox.translation() = Eigen::Vector3d(0, 0, 1.0);

  CollisionResult result;
  bool collided = collidePlaneBox(plane, tfPlane, box, tfBox, result);

  EXPECT_TRUE(collided);
  EXPECT_EQ(result.numContacts(), 1u);
}

TEST(PlaneCapsule, NoCollision)
{
  PlaneShape plane(Eigen::Vector3d::UnitZ(), 0.0);
  CapsuleShape capsule(0.5, 2.0);

  Eigen::Isometry3d tfPlane = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d tfCapsule = Eigen::Isometry3d::Identity();
  tfCapsule.translation() = Eigen::Vector3d(0, 0, 2.0);

  CollisionResult result;
  bool collided
      = collidePlaneCapsule(plane, tfPlane, capsule, tfCapsule, result);

  EXPECT_FALSE(collided);
  EXPECT_EQ(result.numContacts(), 0u);
}

TEST(PlaneCapsule, Standing)
{
  PlaneShape plane(Eigen::Vector3d::UnitZ(), 0.0);
  CapsuleShape capsule(0.5, 2.0);

  Eigen::Isometry3d tfPlane = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d tfCapsule = Eigen::Isometry3d::Identity();
  tfCapsule.translation() = Eigen::Vector3d(0, 0, 1.3);

  CollisionResult result;
  bool collided
      = collidePlaneCapsule(plane, tfPlane, capsule, tfCapsule, result);

  EXPECT_TRUE(collided);
  EXPECT_EQ(result.numContacts(), 1u);
  EXPECT_NEAR(result.getContact(0).depth, 0.2, 1e-6);
}

TEST(PlaneCapsule, Lying)
{
  PlaneShape plane(Eigen::Vector3d::UnitZ(), 0.0);
  CapsuleShape capsule(0.5, 2.0);

  Eigen::Isometry3d tfPlane = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d tfCapsule = Eigen::Isometry3d::Identity();
  tfCapsule.linear() = Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitY())
                           .toRotationMatrix();
  tfCapsule.translation() = Eigen::Vector3d(0, 0, 0.3);

  CollisionResult result;
  bool collided
      = collidePlaneCapsule(plane, tfPlane, capsule, tfCapsule, result);

  EXPECT_TRUE(collided);
  EXPECT_EQ(result.numContacts(), 1u);
  EXPECT_NEAR(result.getContact(0).depth, 0.2, 1e-6);
}
