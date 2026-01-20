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
#include <dart/collision/experimental/narrow_phase/distance.hpp>
#include <dart/collision/experimental/narrow_phase/narrow_phase.hpp>
#include <dart/collision/experimental/shapes/shape.hpp>

#include <gtest/gtest.h>

using namespace dart::collision::experimental;

TEST(DistancePlaneShape, SphereSeparated)
{
  PlaneShape plane(Eigen::Vector3d::UnitZ(), 0.0);
  SphereShape sphere(0.5);

  Eigen::Isometry3d tfPlane = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d tfSphere = Eigen::Isometry3d::Identity();
  tfSphere.translation() = Eigen::Vector3d(0, 0, 2.0);

  DistanceResult result;
  double dist = distancePlaneShape(plane, tfPlane, sphere, tfSphere, result);

  EXPECT_NEAR(dist, 1.5, 1e-6);
  EXPECT_NEAR(result.pointOnObject1.z(), 0.0, 1e-6);
  EXPECT_NEAR(result.pointOnObject2.z(), 1.5, 1e-6);
  EXPECT_GT(result.normal.dot(Eigen::Vector3d::UnitZ()), 0.99);
}

TEST(DistancePlaneShape, SpherePenetrating)
{
  PlaneShape plane(Eigen::Vector3d::UnitZ(), 0.0);
  SphereShape sphere(0.5);

  Eigen::Isometry3d tfPlane = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d tfSphere = Eigen::Isometry3d::Identity();
  tfSphere.translation() = Eigen::Vector3d(0, 0, 0.2);

  DistanceResult result;
  double dist = distancePlaneShape(plane, tfPlane, sphere, tfSphere, result);

  EXPECT_NEAR(dist, -0.3, 1e-6);
  EXPECT_NEAR(result.pointOnObject1.z(), 0.0, 1e-6);
  EXPECT_NEAR(result.pointOnObject2.z(), -0.3, 1e-6);
  EXPECT_GT(result.normal.dot(-Eigen::Vector3d::UnitZ()), 0.99);
}

TEST(DistancePlaneShape, CylinderSeparated)
{
  PlaneShape plane(Eigen::Vector3d::UnitZ(), 0.0);
  CylinderShape cylinder(0.5, 2.0);

  Eigen::Isometry3d tfPlane = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d tfCylinder = Eigen::Isometry3d::Identity();
  tfCylinder.translation() = Eigen::Vector3d(0, 0, 2.0);

  DistanceResult result;
  double dist
      = distancePlaneShape(plane, tfPlane, cylinder, tfCylinder, result);

  EXPECT_NEAR(dist, 1.0, 1e-6);
  EXPECT_NEAR(result.pointOnObject1.z(), 0.0, 1e-6);
  EXPECT_NEAR(result.pointOnObject2.z(), 1.0, 1e-6);
}

TEST(DistanceNarrowPhase, CylinderSphereDistance)
{
  CollisionWorld world;
  auto cyl = world.createObject(std::make_unique<CylinderShape>(0.5, 2.0));
  auto sphere = world.createObject(std::make_unique<SphereShape>(0.5));

  Eigen::Isometry3d tfCyl = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d tfSphere = Eigen::Isometry3d::Identity();
  tfSphere.translation() = Eigen::Vector3d(2.0, 0, 0);
  cyl.setTransform(tfCyl);
  sphere.setTransform(tfSphere);

  DistanceResult result;
  double dist
      = NarrowPhase::distance(cyl, sphere, DistanceOption::unlimited(), result);

  EXPECT_NEAR(dist, 1.0, 1e-4);

  tfSphere.translation() = Eigen::Vector3d(0.4, 0, 0);
  sphere.setTransform(tfSphere);
  result.clear();

  dist
      = NarrowPhase::distance(cyl, sphere, DistanceOption::unlimited(), result);

  EXPECT_NEAR(dist, -0.6, 1e-3);
}

TEST(DistanceSupport, PlaneAndCylinderPairs)
{
  EXPECT_TRUE(
      NarrowPhase::isDistanceSupported(ShapeType::Plane, ShapeType::Sphere));
  EXPECT_TRUE(
      NarrowPhase::isDistanceSupported(ShapeType::Plane, ShapeType::Cylinder));
  EXPECT_TRUE(
      NarrowPhase::isDistanceSupported(ShapeType::Cylinder, ShapeType::Box));
  EXPECT_FALSE(
      NarrowPhase::isDistanceSupported(ShapeType::Cylinder, ShapeType::Sdf));
}
