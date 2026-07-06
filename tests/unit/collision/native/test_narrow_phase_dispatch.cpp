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

#include <dart/collision/native/narrow_phase/narrow_phase.hpp>
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

TEST(NarrowPhaseDispatch, RoutesOverlappingSphereSphere)
{
  SphereShape sphere1(1.0);
  SphereShape sphere2(1.0);

  CollisionResult result;
  const bool hit = NarrowPhase::collide(
      &sphere1,
      Eigen::Isometry3d::Identity(),
      &sphere2,
      translated(1.5, 0.0, 0.0),
      CollisionOption(),
      result);

  EXPECT_TRUE(hit);
  EXPECT_GE(result.numContacts(), 1u);
}

TEST(NarrowPhaseDispatch, ReturnsFalseForSeparatedSphereSphere)
{
  SphereShape sphere1(1.0);
  SphereShape sphere2(1.0);

  CollisionResult result;
  const bool hit = NarrowPhase::collide(
      &sphere1,
      Eigen::Isometry3d::Identity(),
      &sphere2,
      translated(3.0, 0.0, 0.0),
      CollisionOption(),
      result);

  EXPECT_FALSE(hit);
  EXPECT_EQ(result.numContacts(), 0u);
}

TEST(NarrowPhaseDispatch, RoutesOverlappingBoxBox)
{
  BoxShape box1(Eigen::Vector3d(1.0, 1.0, 1.0));
  BoxShape box2(Eigen::Vector3d(1.0, 1.0, 1.0));

  CollisionResult result;
  const bool hit = NarrowPhase::collide(
      &box1,
      Eigen::Isometry3d::Identity(),
      &box2,
      translated(0.5, 0.0, 0.0),
      CollisionOption(),
      result);

  EXPECT_TRUE(hit);
  EXPECT_GE(result.numContacts(), 1u);
}

TEST(NarrowPhaseDispatch, ReturnsFalseForUnroutedPairs)
{
  SphereShape sphere(1.0);
  CapsuleShape capsule(0.5, 2.0);

  CollisionResult result;
  const bool hit = NarrowPhase::collide(
      &sphere,
      Eigen::Isometry3d::Identity(),
      &capsule,
      Eigen::Isometry3d::Identity(),
      CollisionOption(),
      result);

  EXPECT_FALSE(hit);
  EXPECT_EQ(result.numContacts(), 0u);
}

TEST(NarrowPhaseDispatch, ReportsSupportOnlyForSphereSphereAndBoxBox)
{
  EXPECT_TRUE(NarrowPhase::isSupported(ShapeType::Sphere, ShapeType::Sphere));
  EXPECT_TRUE(NarrowPhase::isSupported(ShapeType::Box, ShapeType::Box));

  EXPECT_FALSE(NarrowPhase::isSupported(ShapeType::Sphere, ShapeType::Box));
  EXPECT_FALSE(NarrowPhase::isSupported(ShapeType::Box, ShapeType::Sphere));
  EXPECT_FALSE(NarrowPhase::isSupported(ShapeType::Sphere, ShapeType::Capsule));
  EXPECT_FALSE(
      NarrowPhase::isSupported(ShapeType::Capsule, ShapeType::Capsule));
}

TEST(NarrowPhaseDispatch, RespectsExhaustedContactBudget)
{
  SphereShape sphere1(1.0);
  SphereShape sphere2(1.0);

  CollisionOption option;
  option.enableContact = true;
  option.maxNumContacts = 0;

  CollisionResult result;
  const bool hit = NarrowPhase::collide(
      &sphere1,
      Eigen::Isometry3d::Identity(),
      &sphere2,
      translated(1.5, 0.0, 0.0),
      option,
      result);

  EXPECT_FALSE(hit);
  EXPECT_EQ(result.numContacts(), 0u);
}
