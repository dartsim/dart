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

#include <dart/collision/experimental/narrow_phase/cylinder_collision.hpp>
#include <dart/collision/experimental/shapes/shape.hpp>

#include <gtest/gtest.h>

using namespace dart::collision::experimental;

TEST(CylinderCylinder, NoCollision)
{
  CylinderShape cyl1(0.5, 2.0);
  CylinderShape cyl2(0.5, 2.0);

  Eigen::Isometry3d tf1 = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d tf2 = Eigen::Isometry3d::Identity();
  tf2.translation() = Eigen::Vector3d(3.0, 0, 0);

  CollisionResult result;
  bool collided = collideCylinders(cyl1, tf1, cyl2, tf2, result);

  EXPECT_FALSE(collided);
  EXPECT_EQ(result.numContacts(), 0u);
}

TEST(CylinderCylinder, ParallelOverlap)
{
  CylinderShape cyl1(0.5, 2.0);
  CylinderShape cyl2(0.5, 2.0);

  Eigen::Isometry3d tf1 = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d tf2 = Eigen::Isometry3d::Identity();
  tf2.translation() = Eigen::Vector3d(0.8, 0, 0);

  CollisionResult result;
  bool collided = collideCylinders(cyl1, tf1, cyl2, tf2, result);

  EXPECT_TRUE(collided);
  EXPECT_GE(result.numContacts(), 1u);
}

TEST(CylinderCylinder, StackedOnTop)
{
  CylinderShape cyl1(0.5, 2.0);
  CylinderShape cyl2(0.5, 2.0);

  Eigen::Isometry3d tf1 = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d tf2 = Eigen::Isometry3d::Identity();
  tf2.translation() = Eigen::Vector3d(0, 0, 1.8);

  CollisionResult result;
  bool collided = collideCylinders(cyl1, tf1, cyl2, tf2, result);

  EXPECT_TRUE(collided);
  EXPECT_GE(result.numContacts(), 1u);
}

TEST(CylinderCylinder, Perpendicular)
{
  CylinderShape cyl1(0.5, 2.0);
  CylinderShape cyl2(0.5, 2.0);

  Eigen::Isometry3d tf1 = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d tf2 = Eigen::Isometry3d::Identity();
  tf2.linear() = Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitY())
                     .toRotationMatrix();
  tf2.translation() = Eigen::Vector3d(0.8, 0, 0);

  CollisionResult result;
  bool collided = collideCylinders(cyl1, tf1, cyl2, tf2, result);

  EXPECT_TRUE(collided);
  EXPECT_GE(result.numContacts(), 1u);
}

TEST(CylinderSphere, NoCollision)
{
  CylinderShape cylinder(0.5, 2.0);
  SphereShape sphere(0.5);

  Eigen::Isometry3d tfCylinder = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d tfSphere = Eigen::Isometry3d::Identity();
  tfSphere.translation() = Eigen::Vector3d(3.0, 0, 0);

  CollisionResult result;
  bool collided
      = collideCylinderSphere(cylinder, tfCylinder, sphere, tfSphere, result);

  EXPECT_FALSE(collided);
  EXPECT_EQ(result.numContacts(), 0u);
}

TEST(CylinderSphere, SphereAtSide)
{
  CylinderShape cylinder(0.5, 2.0);
  SphereShape sphere(0.5);

  Eigen::Isometry3d tfCylinder = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d tfSphere = Eigen::Isometry3d::Identity();
  tfSphere.translation() = Eigen::Vector3d(0.8, 0, 0);

  CollisionResult result;
  bool collided
      = collideCylinderSphere(cylinder, tfCylinder, sphere, tfSphere, result);

  EXPECT_TRUE(collided);
  EXPECT_EQ(result.numContacts(), 1u);
  EXPECT_NEAR(result.getContact(0).depth, 0.2, 1e-6);
}

TEST(CylinderSphere, SphereAtTop)
{
  CylinderShape cylinder(0.5, 2.0);
  SphereShape sphere(0.5);

  Eigen::Isometry3d tfCylinder = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d tfSphere = Eigen::Isometry3d::Identity();
  tfSphere.translation() = Eigen::Vector3d(0, 0, 1.3);

  CollisionResult result;
  bool collided
      = collideCylinderSphere(cylinder, tfCylinder, sphere, tfSphere, result);

  EXPECT_TRUE(collided);
  EXPECT_EQ(result.numContacts(), 1u);
}

TEST(CylinderSphere, SphereAtTopEdge)
{
  CylinderShape cylinder(0.5, 2.0);
  SphereShape sphere(0.5);

  Eigen::Isometry3d tfCylinder = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d tfSphere = Eigen::Isometry3d::Identity();
  tfSphere.translation() = Eigen::Vector3d(0.5, 0, 1.2);

  CollisionResult result;
  bool collided
      = collideCylinderSphere(cylinder, tfCylinder, sphere, tfSphere, result);

  EXPECT_TRUE(collided);
  EXPECT_EQ(result.numContacts(), 1u);
}

TEST(CylinderBox, NoCollision)
{
  CylinderShape cylinder(0.5, 2.0);
  BoxShape box(Eigen::Vector3d(0.5, 0.5, 0.5));

  Eigen::Isometry3d tfCylinder = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d tfBox = Eigen::Isometry3d::Identity();
  tfBox.translation() = Eigen::Vector3d(3.0, 0, 0);

  CollisionResult result;
  bool collided = collideCylinderBox(cylinder, tfCylinder, box, tfBox, result);

  EXPECT_FALSE(collided);
  EXPECT_EQ(result.numContacts(), 0u);
}

TEST(CylinderBox, CylinderSideTouchesBox)
{
  CylinderShape cylinder(0.5, 2.0);
  BoxShape box(Eigen::Vector3d(0.2, 0.2, 0.2));

  Eigen::Isometry3d tfCylinder = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d tfBox = Eigen::Isometry3d::Identity();
  tfBox.translation() = Eigen::Vector3d(0.4, 0, 0);

  CollisionResult result;
  bool collided = collideCylinderBox(cylinder, tfCylinder, box, tfBox, result);

  EXPECT_TRUE(collided);
  EXPECT_GE(result.numContacts(), 1u);
}

TEST(CylinderBox, BoxFaceOverlapWithoutCornerInside)
{
  CylinderShape cylinder(0.5, 2.0);
  BoxShape box(Eigen::Vector3d(0.5, 0.5, 0.5));

  Eigen::Isometry3d tfCylinder = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d tfBox = Eigen::Isometry3d::Identity();
  tfBox.translation() = Eigen::Vector3d(0.8, 0, 0);

  CollisionResult result;
  bool collided = collideCylinderBox(cylinder, tfCylinder, box, tfBox, result);

  EXPECT_TRUE(collided);
  EXPECT_GE(result.numContacts(), 1u);
}

TEST(CylinderBox, CylinderOnTopOfBox)
{
  CylinderShape cylinder(0.5, 2.0);
  BoxShape box(Eigen::Vector3d(0.2, 0.2, 0.2));

  Eigen::Isometry3d tfCylinder = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d tfBox = Eigen::Isometry3d::Identity();
  tfBox.translation() = Eigen::Vector3d(0, 0, 0.9);

  CollisionResult result;
  bool collided = collideCylinderBox(cylinder, tfCylinder, box, tfBox, result);

  EXPECT_TRUE(collided);
  EXPECT_GE(result.numContacts(), 1u);
}

TEST(CylinderCapsule, NoCollision)
{
  CylinderShape cylinder(0.5, 2.0);
  CapsuleShape capsule(0.5, 2.0);

  Eigen::Isometry3d tfCylinder = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d tfCapsule = Eigen::Isometry3d::Identity();
  tfCapsule.translation() = Eigen::Vector3d(3.0, 0, 0);

  CollisionResult result;
  bool collided = collideCylinderCapsule(
      cylinder, tfCylinder, capsule, tfCapsule, result);

  EXPECT_FALSE(collided);
  EXPECT_EQ(result.numContacts(), 0u);
}

TEST(CylinderCapsule, ParallelOverlap)
{
  CylinderShape cylinder(0.5, 2.0);
  CapsuleShape capsule(0.5, 2.0);

  Eigen::Isometry3d tfCylinder = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d tfCapsule = Eigen::Isometry3d::Identity();
  tfCapsule.translation() = Eigen::Vector3d(0.8, 0, 0);

  CollisionResult result;
  bool collided = collideCylinderCapsule(
      cylinder, tfCylinder, capsule, tfCapsule, result);

  EXPECT_TRUE(collided);
  EXPECT_GE(result.numContacts(), 1u);
}

TEST(CylinderCapsule, CapsuleOnTop)
{
  CylinderShape cylinder(0.5, 2.0);
  CapsuleShape capsule(0.3, 1.0);

  Eigen::Isometry3d tfCylinder = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d tfCapsule = Eigen::Isometry3d::Identity();
  tfCapsule.translation() = Eigen::Vector3d(0, 0, 1.5);

  CollisionResult result;
  bool collided = collideCylinderCapsule(
      cylinder, tfCylinder, capsule, tfCapsule, result);

  EXPECT_TRUE(collided);
  EXPECT_GE(result.numContacts(), 1u);
}

TEST(CylinderPlane, NoCollision)
{
  CylinderShape cylinder(0.5, 2.0);
  PlaneShape plane(Eigen::Vector3d::UnitZ(), 0.0);

  Eigen::Isometry3d tfCylinder = Eigen::Isometry3d::Identity();
  tfCylinder.translation() = Eigen::Vector3d(0, 0, 2.0);
  Eigen::Isometry3d tfPlane = Eigen::Isometry3d::Identity();

  CollisionResult result;
  bool collided
      = collideCylinderPlane(cylinder, tfCylinder, plane, tfPlane, result);

  EXPECT_FALSE(collided);
  EXPECT_EQ(result.numContacts(), 0u);
}

TEST(CylinderPlane, CylinderStandingOnPlane)
{
  CylinderShape cylinder(0.5, 2.0);
  PlaneShape plane(Eigen::Vector3d::UnitZ(), 0.0);

  Eigen::Isometry3d tfCylinder = Eigen::Isometry3d::Identity();
  tfCylinder.translation() = Eigen::Vector3d(0, 0, 0.9);
  Eigen::Isometry3d tfPlane = Eigen::Isometry3d::Identity();

  CollisionResult result;
  bool collided
      = collideCylinderPlane(cylinder, tfCylinder, plane, tfPlane, result);

  EXPECT_TRUE(collided);
  EXPECT_GE(result.numContacts(), 1u);
}

TEST(CylinderPlane, CylinderLyingOnPlane)
{
  CylinderShape cylinder(0.5, 2.0);
  PlaneShape plane(Eigen::Vector3d::UnitZ(), 0.0);

  Eigen::Isometry3d tfCylinder = Eigen::Isometry3d::Identity();
  tfCylinder.linear() = Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitY())
                            .toRotationMatrix();
  tfCylinder.translation() = Eigen::Vector3d(0, 0, 0.4);
  Eigen::Isometry3d tfPlane = Eigen::Isometry3d::Identity();

  CollisionResult result;
  bool collided
      = collideCylinderPlane(cylinder, tfCylinder, plane, tfPlane, result);

  EXPECT_TRUE(collided);
  EXPECT_GE(result.numContacts(), 1u);
}

TEST(CylinderPlane, CylinderTiltedOnPlane)
{
  CylinderShape cylinder(0.5, 2.0);
  PlaneShape plane(Eigen::Vector3d::UnitZ(), 0.0);

  Eigen::Isometry3d tfCylinder = Eigen::Isometry3d::Identity();
  tfCylinder.linear() = Eigen::AngleAxisd(M_PI / 4, Eigen::Vector3d::UnitY())
                            .toRotationMatrix();
  tfCylinder.translation() = Eigen::Vector3d(0, 0, 0.5);
  Eigen::Isometry3d tfPlane = Eigen::Isometry3d::Identity();

  CollisionResult result;
  bool collided
      = collideCylinderPlane(cylinder, tfCylinder, plane, tfPlane, result);

  EXPECT_TRUE(collided);
  EXPECT_GE(result.numContacts(), 1u);
}
