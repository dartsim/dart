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

#include <dart/collision/native/narrow_phase/mesh_mesh.hpp>
#include <dart/collision/native/narrow_phase/narrow_phase.hpp>
#include <dart/collision/native/narrow_phase/plane_sphere.hpp>
#include <dart/collision/native/shapes/shape.hpp>

#include <gtest/gtest.h>

#include <array>
#include <numbers>
#include <vector>

#include <cmath>

using namespace dart::collision::native;

namespace {

Eigen::Isometry3d makePlaneShapeTestFrame()
{
  Eigen::Isometry3d frame = Eigen::Isometry3d::Identity();
  frame.linear()
      = (Eigen::AngleAxisd(
             std::numbers::pi_v<double> / 7.0,
             Eigen::Vector3d(1.0, 2.0, -1.0).normalized())
         * Eigen::AngleAxisd(
             std::numbers::pi_v<double> / 9.0, Eigen::Vector3d::UnitZ()))
            .toRotationMatrix();
  frame.translation() = Eigen::Vector3d(0.25, -0.5, 0.75);
  return frame;
}

double maxContactDepth(const CollisionResult& result)
{
  double maxDepth = 0.0;
  for (std::size_t i = 0; i < result.numContacts(); ++i) {
    maxDepth = std::max(maxDepth, result.getContact(i).depth);
  }
  return maxDepth;
}

void expectFiniteContacts(const CollisionResult& result)
{
  ASSERT_GT(result.numContacts(), 0u);
  for (std::size_t i = 0; i < result.numContacts(); ++i) {
    const auto& contact = result.getContact(i);
    EXPECT_TRUE(contact.position.allFinite());
    EXPECT_TRUE(contact.normal.allFinite());
    EXPECT_GT(contact.normal.norm(), 0.9);
    EXPECT_GE(contact.depth, 0.0);
  }
}

void expectHalfspaceCollisionAndSeparation(
    const Shape& shape,
    const Eigen::Isometry3d& collidingShapeInPlaneFrame,
    const Eigen::Isometry3d& separatedShapeInPlaneFrame,
    double expectedMaxDepth)
{
  const PlaneShape plane(Eigen::Vector3d::UnitX(), 0.0);
  CollisionOption option;
  option.maxNumContacts = 8u;

  const std::array<Eigen::Isometry3d, 2> commonFrames{
      Eigen::Isometry3d::Identity(), makePlaneShapeTestFrame()};

  for (const auto& commonFrame : commonFrames) {
    const Eigen::Isometry3d planeTransform = commonFrame;
    const Eigen::Isometry3d collidingTransform
        = commonFrame * collidingShapeInPlaneFrame;
    const Eigen::Isometry3d separatedTransform
        = commonFrame * separatedShapeInPlaneFrame;

    CollisionResult shapePlane;
    ASSERT_TRUE(
        NarrowPhase::collide(
            &shape,
            collidingTransform,
            &plane,
            planeTransform,
            option,
            shapePlane));
    expectFiniteContacts(shapePlane);
    EXPECT_NEAR(maxContactDepth(shapePlane), expectedMaxDepth, 1e-9);

    CollisionResult planeShape;
    ASSERT_TRUE(
        NarrowPhase::collide(
            &plane,
            planeTransform,
            &shape,
            collidingTransform,
            option,
            planeShape));
    expectFiniteContacts(planeShape);
    EXPECT_NEAR(maxContactDepth(planeShape), expectedMaxDepth, 1e-9);

    CollisionResult separatedShapePlane;
    EXPECT_FALSE(
        NarrowPhase::collide(
            &shape,
            separatedTransform,
            &plane,
            planeTransform,
            option,
            separatedShapePlane));
    EXPECT_EQ(separatedShapePlane.numContacts(), 0u);

    CollisionResult separatedPlaneShape;
    EXPECT_FALSE(
        NarrowPhase::collide(
            &plane,
            planeTransform,
            &shape,
            separatedTransform,
            option,
            separatedPlaneShape));
    EXPECT_EQ(separatedPlaneShape.numContacts(), 0u);
  }
}

std::vector<Eigen::Vector3d> makeAxisSupportHull(
    double xExtent, double yExtent, double zExtent)
{
  return {
      {-xExtent, 0.0, 0.0},
      {xExtent, 0.0, 0.0},
      {0.0, -yExtent, 0.0},
      {0.0, yExtent, 0.0},
      {0.0, 0.0, -zExtent},
      {0.0, 0.0, zExtent},
  };
}

} // namespace

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

TEST(PlaneShape, LocalAabbIsUnbounded)
{
  PlaneShape plane(Eigen::Vector3d::UnitY(), 4.5);

  const auto aabb = plane.computeLocalAabb();

  for (int i = 0; i < 3; ++i) {
    EXPECT_TRUE(std::isinf(aabb.min[i]));
    EXPECT_TRUE(std::isinf(aabb.max[i]));
    EXPECT_LT(aabb.min[i], 0.0);
    EXPECT_GT(aabb.max[i], 0.0);
  }
}

TEST(PlaneShape, HalfspacePrimitiveContactsAcrossTransforms)
{
  Eigen::Isometry3d colliding = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d separated = Eigen::Isometry3d::Identity();

  SphereShape sphere(10.0);
  separated.translation() = Eigen::Vector3d(10.1, 0.0, 0.0);
  expectHalfspaceCollisionAndSeparation(sphere, colliding, separated, 10.0);

  BoxShape box(Eigen::Vector3d(2.5, 5.0, 10.0));
  separated.translation() = Eigen::Vector3d(2.51, 0.0, 0.0);
  expectHalfspaceCollisionAndSeparation(box, colliding, separated, 2.5);

  CapsuleShape capsule(5.0, 10.0);
  separated.translation() = Eigen::Vector3d(5.1, 0.0, 0.0);
  expectHalfspaceCollisionAndSeparation(capsule, colliding, separated, 5.0);

  CylinderShape cylinder(5.0, 10.0);
  separated.translation() = Eigen::Vector3d(5.1, 0.0, 0.0);
  expectHalfspaceCollisionAndSeparation(cylinder, colliding, separated, 5.0);

  MeshShape crossingTriangle(
      {Eigen::Vector3d(20.0, 0.0, 0.0),
       Eigen::Vector3d(-20.0, 0.0, 0.0),
       Eigen::Vector3d(0.0, 20.0, 0.0)},
      {{0, 1, 2}});
  separated.translation() = Eigen::Vector3d(20.1, 0.0, 0.0);
  expectHalfspaceCollisionAndSeparation(
      crossingTriangle, colliding, separated, 20.0);

  ConvexShape anisotropicSupportHull(makeAxisSupportHull(5.0, 10.0, 20.0));
  separated.translation() = Eigen::Vector3d(5.1, 0.0, 0.0);
  expectHalfspaceCollisionAndSeparation(
      anisotropicSupportHull, colliding, separated, 5.0);
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
  tfBox.linear() = Eigen::AngleAxisd(
                       std::numbers::pi_v<double> / 4, Eigen::Vector3d::UnitY())
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
  tfCapsule.linear()
      = Eigen::AngleAxisd(
            std::numbers::pi_v<double> / 2, Eigen::Vector3d::UnitY())
            .toRotationMatrix();
  tfCapsule.translation() = Eigen::Vector3d(0, 0, 0.3);

  CollisionResult result;
  bool collided
      = collidePlaneCapsule(plane, tfPlane, capsule, tfCapsule, result);

  EXPECT_TRUE(collided);
  EXPECT_EQ(result.numContacts(), 1u);
  EXPECT_NEAR(result.getContact(0).depth, 0.2, 1e-6);
}

TEST(PlaneMesh, NoCollision)
{
  PlaneShape plane(Eigen::Vector3d::UnitZ(), 0.0);
  MeshShape mesh(
      {Eigen::Vector3d(-0.5, -0.5, 0.0),
       Eigen::Vector3d(0.5, -0.5, 0.0),
       Eigen::Vector3d(0.0, 0.5, 0.0)},
      {{0, 1, 2}});

  Eigen::Isometry3d tfPlane = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d tfMesh = Eigen::Isometry3d::Identity();
  tfMesh.translation() = Eigen::Vector3d(0, 0, 1.0);

  CollisionResult result;
  const bool collided = collidePlaneMesh(plane, tfPlane, mesh, tfMesh, result);

  EXPECT_FALSE(collided);
  EXPECT_EQ(result.numContacts(), 0u);
}

TEST(PlaneMesh, Penetrating)
{
  PlaneShape plane(Eigen::Vector3d::UnitZ(), 0.0);
  MeshShape mesh(
      {Eigen::Vector3d(-0.5, -0.5, 0.0),
       Eigen::Vector3d(0.5, -0.5, 0.0),
       Eigen::Vector3d(0.0, 0.5, 0.0)},
      {{0, 1, 2}});

  Eigen::Isometry3d tfPlane = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d tfMesh = Eigen::Isometry3d::Identity();
  tfMesh.translation() = Eigen::Vector3d(0, 0, -0.25);

  CollisionResult result;
  const bool collided = collidePlaneMesh(plane, tfPlane, mesh, tfMesh, result);

  EXPECT_TRUE(collided);
  EXPECT_EQ(result.numContacts(), 3u);
  EXPECT_NEAR(result.getContact(0).depth, 0.25, 1e-10);
  EXPECT_TRUE(result.getContact(0).normal.isApprox(Eigen::Vector3d::UnitZ()));
}
